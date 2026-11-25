"""无 ROS 的三圆环工位识别、稳定判定和视觉对位计算。

本模块只处理一张已经完成畸变矫正的 BGR 图像。它不打开摄像头、不访问
串口，也不控制底盘或机械臂，因此既可由 ``main.py`` 调用，也可单独用于
离线测试。配置中的参考圆心和纠偏矩阵移植自原 ROS 工程。
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace
from itertools import combinations
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np


class RingStationDetectionError(ValueError):
    """圆环配置或输入画面无效。"""


@dataclass(frozen=True)
class RingStationConfig:
    """三圆环几何、图像筛选门槛和固定观察基准。"""

    center_spacing_mm: float
    outer_diameter_mm: float
    ring1_side: str
    canny_low: int
    canny_high: int
    min_contour_points: int
    min_minor_axis_px: float
    max_major_axis_px: float
    min_ellipse_aspect: float
    center_cluster_radius_px: float
    diameter_band_gap_px: float
    min_cluster_support: int
    min_diameter_bands: int
    min_station_span_ratio: float
    max_collinearity_error_ratio: float
    roi_margin_px: int
    roi_fallback_after_failures: int
    reference_yaw_deg: float
    reference_centers_px: tuple[tuple[float, float], ...]
    image_to_station_affine: tuple[tuple[float, float], ...]
    minimum_confidence: float
    stable_frame_count: int
    maximum_center_jitter_px: float
    maximum_affine_residual: float
    perspective_ratio_range: tuple[float, float]
    maximum_position_error_mm: float
    maximum_angle_error_deg: float
    chassis_image_to_motion: tuple[tuple[float, float, float], ...] | None


@dataclass(frozen=True)
class RingStationDetection:
    """一帧圆环工位结果；失败时不携带上一帧坐标。"""

    detected: bool = False
    stable: bool = False
    confidence: float = 0.0
    centers_px: tuple[tuple[float, float], ...] = ()
    ellipses: tuple[tuple, ...] = ()
    station_yaw_deg: float = 0.0
    angle_error_deg: float = 0.0
    error_x_mm: float = 0.0
    error_y_mm: float = 0.0
    position_error_mm: float = 0.0
    correction_body_x_mm: float | None = None
    correction_body_y_mm: float | None = None
    correction_yaw_deg: float | None = None
    affine_residual: float = math.inf
    perspective_ratio: float = math.inf
    within_tolerance: bool = False
    processing_ms: float = 0.0
    detection_scope: str = "full"
    detail: str = "尚未识别"
    # 缺环降级支持：夹爪伸出去时常被挡住/带出画面，只能看到 1~2 个环。
    visible_ring_count: int = 0
    ring_indices: tuple[int, ...] = ()      # 这几次观测对应参考环心里的哪几个
    yaw_valid: bool = False                 # 只有 >=2 个环才谈得上航向
    # 诊断用：椭圆观测数 / 聚类后的候选数 —— 现场判断"为什么只报 1 个环"就看这两个
    ellipse_observation_count: int = 0
    ring_candidate_count: int = 0
    # 完整三环布局预测（长度 3，缺环的位置是推算出来的）。物料挡住中间/目标环时，
    # 用已看到的环 + 已知的 150mm 间距把被挡的环也推出来，稳定窗口才不会因为
    # "这一帧少看一个环"被整个清空（那正是左右圆心抖动的根源）。
    predicted_centers_px: tuple[tuple[float, float] | None, ...] = ()
    layout_scale: float = 0.0       # 观测像素 / 参考像素 的尺度比
    layout_source: str = ""         # observed / two_ring / prior / one_ring


@dataclass
class _EllipseObservation:
    center: np.ndarray
    axes: tuple[float, float]
    angle_deg: float
    major_axis: float
    minor_axis: float


@dataclass
class _RingCandidate:
    center: np.ndarray
    support: int
    diameter_bands: int
    ellipse: _EllipseObservation
    effective_diameter_px: float


def _finite_matrix(raw, shape, name):
    matrix = np.asarray(raw, dtype=float)
    if matrix.shape != shape or not np.all(np.isfinite(matrix)):
        raise RingStationDetectionError(
            f"{name} 必须是有限数值的 {shape[0]}x{shape[1]} 矩阵"
        )
    return tuple(tuple(float(value) for value in row) for row in matrix)


def load_ring_station_config(path: Path | str) -> RingStationConfig:
    """从 JSON 读取并严格校验三圆环工位配置。"""

    resolved = Path(path).expanduser().resolve()
    try:
        raw = json.loads(resolved.read_text(encoding="utf-8"))
        if int(raw["format_version"]) != 1:
            raise RingStationDetectionError("圆环配置版本不受支持")
        geometry = raw["geometry"]
        detection = raw["detection"]
        reference = raw["reference"]
        acceptance = raw["acceptance"]
        centers = tuple(
            (float(center[0]), float(center[1]))
            for center in reference["ring_centers_px"]
        )
        affine = _finite_matrix(
            reference["image_to_station_affine"], (2, 2),
            "image_to_station_affine",
        )
        correction_raw = raw.get("correction_calibration", {}).get(
            "chassis_image_to_motion"
        )
        # 3x3底盘标定矩阵把(x毫米, y毫米, 角度)转换为底盘纠偏量。
        correction = None
        if correction_raw is not None:
            correction = _finite_matrix(
                correction_raw, (3, 3), "chassis_image_to_motion"
            )
        config = RingStationConfig(
            center_spacing_mm=float(geometry["center_spacing_mm"]),
            outer_diameter_mm=float(geometry["outer_diameter_mm"]),
            ring1_side=str(geometry["ring1_side"]),
            canny_low=int(detection["canny_low"]),
            canny_high=int(detection["canny_high"]),
            min_contour_points=int(detection["min_contour_points"]),
            min_minor_axis_px=float(detection["min_minor_axis_px"]),
            max_major_axis_px=float(detection["max_major_axis_px"]),
            min_ellipse_aspect=float(detection["min_ellipse_aspect"]),
            center_cluster_radius_px=float(
                detection["center_cluster_radius_px"]
            ),
            diameter_band_gap_px=float(detection["diameter_band_gap_px"]),
            min_cluster_support=int(detection["min_cluster_support"]),
            min_diameter_bands=int(detection["min_diameter_bands"]),
            min_station_span_ratio=float(
                detection["min_station_span_ratio"]
            ),
            max_collinearity_error_ratio=float(
                detection["max_collinearity_error_ratio"]
            ),
            roi_margin_px=int(detection["roi_margin_px"]),
            roi_fallback_after_failures=int(
                detection["roi_fallback_after_failures"]
            ),
            reference_yaw_deg=float(reference["station_yaw_deg"]),
            reference_centers_px=centers,
            image_to_station_affine=affine,
            minimum_confidence=float(reference["minimum_confidence"]),
            stable_frame_count=int(reference["stable_frame_count"]),
            maximum_center_jitter_px=float(
                reference["maximum_center_jitter_px"]
            ),
            maximum_affine_residual=float(
                reference["maximum_affine_residual"]
            ),
            perspective_ratio_range=tuple(
                float(value) for value in reference["perspective_ratio_range"]
            ),
            maximum_position_error_mm=float(
                acceptance["maximum_position_error_mm"]
            ),
            maximum_angle_error_deg=float(
                acceptance["maximum_angle_error_deg"]
            ),
            chassis_image_to_motion=correction,
        )
    except RingStationDetectionError:
        raise
    except (OSError, json.JSONDecodeError, KeyError, TypeError, ValueError,
            IndexError) as error:
        raise RingStationDetectionError(
            f"无法读取圆环配置 {resolved}：{error}"
        ) from error

    if config.center_spacing_mm <= 0 or config.outer_diameter_mm <= 0:
        raise RingStationDetectionError("圆环实际尺寸必须大于0")
    if config.ring1_side not in {"left", "right"}:
        raise RingStationDetectionError("ring1_side 只能是 left 或 right")
    if len(config.reference_centers_px) != 3:
        raise RingStationDetectionError("参考基准必须包含三个圆心")
    if not 0 < config.canny_low < config.canny_high <= 255:
        raise RingStationDetectionError("Canny 阈值无效")
    if not 0.0 < config.min_ellipse_aspect <= 1.0:
        raise RingStationDetectionError("椭圆长短轴比例无效")
    if config.min_cluster_support < 2 or config.min_diameter_bands < 2:
        raise RingStationDetectionError("同心轮廓支持数门槛过低")
    if not 0.0 < config.minimum_confidence <= 1.0:
        raise RingStationDetectionError("置信度门槛无效")
    if config.stable_frame_count < 2:
        raise RingStationDetectionError("稳定窗口至少需要2帧")
    if config.maximum_center_jitter_px <= 0:
        raise RingStationDetectionError("圆心抖动门槛必须大于0")
    if len(config.perspective_ratio_range) != 2:
        raise RingStationDetectionError("透视比例范围必须包含上下界")
    if not 0 < config.perspective_ratio_range[0] < config.perspective_ratio_range[1]:
        raise RingStationDetectionError("透视比例范围无效")
    if config.roi_margin_px < 0 or config.roi_fallback_after_failures < 1:
        raise RingStationDetectionError("ROI 跟踪配置无效")
    return config


def _arc_coverage_deg(contour, center):
    """轮廓点相对圆心的极角覆盖跨度（度）。整圆 360，半圆 180，四分之一圆 90。"""

    points = contour.reshape(-1, 2).astype(np.float64)
    if len(points) < 4:
        return 0.0
    angles = np.degrees(np.arctan2(
        points[:, 1] - center[1], points[:, 0] - center[0]
    ))
    angles.sort()
    gaps = np.diff(angles)
    wrap_gap = 360.0 - float(angles[-1] - angles[0])
    max_gap = max(float(gaps.max()) if len(gaps) else 0.0, wrap_gap)
    return 360.0 - max_gap


def _refine_center_fixed_radius(contour, center, radius):
    """已知半径时的圆心最小二乘：对短弧远比 fitEllipse 稳。

    fitEllipse 同时估 5 个参数（圆心 2 + 长短轴 2 + 倾角 1），拿到一小段弧时
    这些参数互相补偿、严重欠定，圆心就沿法向乱跑——这是"圆弧只露四分之一、
    圆心很飘"的数学原因。

    半径一旦固定（环的外径是已知的，尺度可用三环间距估出来），就只剩 2 个
    未知数，一段弧也够解。把 (p-c)² = r² 展开：
        2·cx·px + 2·cy·py + (cx²+cy²-r²) = px²+py²
    对 cx、cy 是线性的，一次最小二乘即可。
    """

    points = contour.reshape(-1, 2).astype(np.float64)
    if len(points) < 6 or radius <= 1e-6:
        return center
    design = np.hstack([2.0 * points, np.ones((len(points), 1))])
    target = (points ** 2).sum(axis=1) - radius ** 2
    try:
        solution, *_ = np.linalg.lstsq(design, target, rcond=None)
    except np.linalg.LinAlgError:
        return center
    new_x, new_y = float(solution[0]), float(solution[1])
    if not (math.isfinite(new_x) and math.isfinite(new_y)):
        return center
    # 偏得离谱说明这组点根本不贴同一个圆（杂边），保留 fitEllipse 的结果。
    if math.hypot(new_x - center[0], new_y - center[1]) > radius * 0.8:
        return center
    return (new_x, new_y)


def _find(parent, index):
    if parent[index] != index:
        parent[index] = _find(parent, parent[index])
    return parent[index]


def _extract_observations(image, config):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # 模糊核要小：工位图案里的细实线圈只有 1~2px 宽，5x5/σ=1.2 的高斯会把它们
    # 糊成一片灰，Canny 直接看不见。3x3/σ=0.8 只压高频噪点，细线能保住。
    blurred = cv2.GaussianBlur(gray, (3, 3), 0.8)
    edges = cv2.Canny(blurred, config.canny_low, config.canny_high)
    # 不做形态学闭运算：11x11 核在 1280x720 上每像素 121 次运算，树莓派 ARM
    # 上要 150~250ms，直接把帧率打到 3FPS。工位图案里有**实线圆**，轮廓法能
    # 直接拟合，不需要靠闭运算去连虚线断口。
    # CHAIN_APPROX_SIMPLE 只保留拐点，点数比 NONE 少 50%~90%：
    # 后面的 fitEllipse 是逐轮廓 SVD，点数直接决定耗时，而拟合精度几乎不受影响
    # （椭圆拟合用的是点的分布，不是点的密度）。完整一圈简化后仍有几十个点，
    # 够 min_contour_points。
    contours, _ = cv2.findContours(
        edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
    )
    observations = []
    for contour in contours:
        if len(contour) < config.min_contour_points:
            continue
        center, axes, angle = cv2.fitEllipse(contour)
        major, minor = max(axes), min(axes)
        if (
            minor < config.min_minor_axis_px
            or major > config.max_major_axis_px
            or minor / major < config.min_ellipse_aspect
        ):
            continue
        # 短弧的圆心要重解一次：只露 1/4 圈的环，fitEllipse 的圆心会沿法向
        # 飘十几到几十像素（5 个参数在短弧上互相补偿）。固定半径、只解圆心
        # 两个未知数，同样的弧就能定住。整圈（≥240°）不受影响。
        coverage = _arc_coverage_deg(contour, center)
        if coverage < 240.0:
            equivalent_radius = (major + minor) * 0.25
            refined = _refine_center_fixed_radius(
                contour, center, equivalent_radius
            )
            if refined != center:
                center = refined
        observations.append(_EllipseObservation(
            center=np.asarray(center, dtype=np.float64),
            axes=(float(axes[0]), float(axes[1])),
            angle_deg=float(angle),
            major_axis=float(major),
            minor_axis=float(minor),
        ))
    return observations


def _cluster_observations(observations, config):
    parent = list(range(len(observations)))
    for first in range(len(observations)):
        for second in range(first):
            if np.linalg.norm(
                observations[first].center - observations[second].center
            ) <= config.center_cluster_radius_px:
                first_root = _find(parent, first)
                second_root = _find(parent, second)
                if first_root != second_root:
                    parent[second_root] = first_root

    groups = {}
    for index, observation in enumerate(observations):
        groups.setdefault(_find(parent, index), []).append(observation)

    candidates = []
    for group in groups.values():
        # 先剔掉"巨大椭圆"：被物料切断的弧、或者把两圈连在一起的杂边，
        # fitEllipse 会给出直径是正常圈 2~3 倍的椭圆。它会把 bands 计数、
        # effective_diameter 带偏，还会被 outer = max(...) 选中画出来——
        # 现场看到的"两边的圆突然变成一个很大的椭圆"就是这个。
        base_diameters = sorted(item.major_axis for item in group)
        median_diameter = float(np.median(base_diameters))
        if median_diameter > 1e-6:
            pruned = [
                item for item in group
                if item.major_axis <= median_diameter * 2.5
            ]
            if len(pruned) >= 2:
                group = pruned
        diameters = sorted(item.major_axis for item in group)
        bands = []
        for diameter in diameters:
            if not bands or diameter - bands[-1] > config.diameter_band_gap_px:
                bands.append(diameter)
        if (
            len(group) < config.min_cluster_support
            or len(bands) < config.min_diameter_bands
        ):
            continue
        # 同心一致性：真环的各圈共享同一个圆心，Hough 对同一组同心圆的圆心估计
        # 通常落在 10px 以内。而机构边角、物料轮廓凑出来的"组"是散的，一动就
        # 几十像素——这一道是压误判最有效的手段，且不牺牲检出（真环不受影响）。
        group_centers = np.asarray([item.center for item in group])
        group_median = np.median(group_centers, axis=0)
        spread = float(np.max(np.linalg.norm(
            group_centers - group_median, axis=1
        )))
        # 同心一致性阈值放宽到 1.5：物料挡住半个环时，各圈只剩一段弧，
        # 拟合圆心会整体散开几十像素，按 0.8（36px）卡会把整个环丢掉——
        # 这正是"两边有物料的环漏检"的主因。假候选（机构边角、物料轮廓）
        # 的圆心离散度通常上百像素，1.5 仍能挡住它们，真正的兜底是下面的
        # 剔除离群 + 稳定窗口。
        if spread > config.center_cluster_radius_px * 1.5:
            continue
        # 圆心剔除离群 + 按直径平方加权（≈面积占比）。
        # 为什么必须先剔除：被物料挡住的圈只剩一段短弧，fitEllipse 的圆心会偏出
        # 十几到几十像素；而每帧遮挡比例还在变，偏移量跟着变——这就是"两边有
        # 物料遮挡时圆心不稳"的直接原因。先用中值距离把偏心的圈剔掉，剩下的
        # 圈圆心基本一致，加权平均才稳。
        # 剔除之后仍按直径平方加权：外圈弧最长、估计最准，内圈半径小、点稀。
        weights = np.asarray(
            [item.major_axis ** 2 for item in group], dtype=np.float64
        )
        distances = np.linalg.norm(group_centers - group_median, axis=1)
        cutoff = max(10.0, float(np.median(distances)) * 3.0)
        keep = distances <= cutoff
        if int(np.count_nonzero(keep)) < 2:
            # 剔得太狠（比如只有两圈、其中一圈偏了）就全留，交给稳定窗口去滤。
            keep = np.ones(len(group), dtype=bool)
        kept_weights = weights[keep]
        if float(kept_weights.sum()) > 1e-6:
            center = np.average(
                group_centers[keep], axis=0, weights=kept_weights
            )
        else:
            center = group_median
        effective_diameter = float(np.median(diameters[-min(4, len(diameters)):]))
        center_tolerance = max(6.0, effective_diameter * 0.08)
        centered = [
            item for item in group
            if np.linalg.norm(item.center - center) <= center_tolerance
        ] or group
        outer = max(centered, key=lambda item: item.major_axis)
        candidates.append(_RingCandidate(
            center=center,
            support=len(group),
            diameter_bands=len(bands),
            ellipse=_EllipseObservation(
                center=center.copy(), axes=outer.axes,
                angle_deg=outer.angle_deg, major_axis=outer.major_axis,
                minor_axis=outer.minor_axis,
            ),
            effective_diameter_px=effective_diameter,
        ))
    # 按支持度降序，只留最强的前 3 组：工位最多三个环，多余的组一定是杂圆。
    # 这一步同时减少了下游 _select_group 的搜索空间（组合数从 C(n,3) 降下来），
    # 也避免弱组跟强组混在同一帧里导致圆心跳变。
    candidates.sort(key=lambda item: item.support, reverse=True)
    return candidates[:3]


def _reference_spacings(config):
    """三个参考环心两两之间的距离（像素），键为 (i, j) 且 i < j。"""
    reference = config.reference_centers_px
    spacings = {}
    for i in range(3):
        for j in range(i + 1, 3):
            spacings[(i, j)] = math.dist(reference[i], reference[j])
    return spacings


def _assign_to_reference(centers, config, ellipses=None):
    """把观测到的若干环心按顺序分配给参考环心，返回 (indices, 匹配总距离)。

    只看到 1~2 个环时，无法靠几何唯一确定它们到底是哪几个（相邻间距只差 1%），
    所以用**离参考位置最近**来判定。这依赖"车已经大致对准"——对准过程中成立，
    车离得很远时可能匹配错，此时 match_error_px 会明显变大，调用方可以据此拒用。

    观测点和参考点都按 x 升序，所以索引顺序天然一致，只需枚举"选哪几个参考点"。
    """
    ordered = sorted(centers, key=lambda point: point[0])
    count = len(ordered)
    if count == 0 or count > 3:
        return None, math.inf
    if count == 3:
        return (0, 1, 2), 0.0

    # 2 个环时**不能**拿参考像素坐标直接比：参考是标定距离下的像素，车一近
    # 一远，同一对环的像素间距就变了（现场参考间距 253px，实际拍到 400px，
    # 差 1.6 倍——直接用最近邻匹配会把 R2 判成 R1）。
    # 改用"间距归一化"：用观测到的环投影直径估出像素/毫米，再和
    # 150mm / 300mm 的物理间距比，选出最像的那一对。这个判据对距离不敏感。
    if count == 2 and ellipses:
        try:
            diameter_px = float(np.median(
                [float(max(ellipse[1])) for ellipse in ellipses]
            ))
        except (IndexError, KeyError, TypeError, ValueError):
            diameter_px = 0.0
        if diameter_px > 1e-6 and config.outer_diameter_mm > 0:
            px_per_mm = diameter_px / config.outer_diameter_mm
            span = float(math.dist(ordered[0], ordered[1]))
            best = (None, math.inf)
            for indices in combinations(range(3), 2):
                spacing_mm = (indices[1] - indices[0]) * config.center_spacing_mm
                expected_px = spacing_mm * px_per_mm
                if expected_px <= 1e-6:
                    continue
                score = abs(math.log(span / expected_px))
                if score < best[1]:
                    best = (tuple(indices), float(score))
            if best[0] is not None:
                return best

    # 单个环没有间距可交叉验证，只能退回"离参考位置最近"——这是信息量的
    # 硬限制。调用方有上一帧布局时会优先走 _assign_with_prior，不走这里。
    best = (None, math.inf)
    for indices in combinations(range(3), count):
        total = sum(
            math.dist(ordered[position], config.reference_centers_px[index])
            for position, index in enumerate(indices)
        )
        if total < best[1]:
            best = (tuple(indices), float(total))
    return best


def _select_pair(candidates, image_width, config):
    """从候选里挑一对共线圆环（任意两点都共线，所以靠"间距像不像参考"来筛）。"""

    if len(candidates) < 2:
        return None
    reference = _reference_spacings(config)
    best = None
    for pair in combinations(candidates, 2):
        points = np.asarray([item.center for item in pair])
        span = float(np.linalg.norm(points[1] - points[0]))
        if span <= 1.0:
            # 只挡"两个候选几乎重合"的退化情况。不再按画面比例设下限：
            # 只看到两个环时本来就该照画，间距小是车远，不是异常。
            continue
        # 用观测到的环直径估像素/毫米，再和物理间距（150 / 300mm）比。
        # 不能直接和参考像素间距比：参考是标定距离下的尺度，车一近一远比值全偏。
        diameter_px = float(np.mean(
            [item.effective_diameter_px for item in pair]
        ))
        if diameter_px > 1e-6 and config.outer_diameter_mm > 0:
            px_per_mm = diameter_px / config.outer_diameter_mm
            expected = [
                config.center_spacing_mm * px_per_mm,
                2.0 * config.center_spacing_mm * px_per_mm,
            ]
            scale_error = min(
                abs(math.log(span / value)) for value in expected
                if value > 1e-6
            )
        else:
            scale_error = min(
                abs(math.log(span / spacing)) for spacing in reference.values()
            )
        support = sum(item.support for item in pair)
        score = 4.0 * scale_error - 0.015 * support
        if best is None or score < best[0]:
            best = (score, tuple(sorted(pair, key=lambda item: item.center[0])))
    return None if best is None else (best[1], 0.0)


def _select_group(candidates, image_width, config):
    """按 3 -> 2 -> 1 的顺序退让，返回 (items, count, collinearity, 失败原因)。

    3 个：沿用原有严格共线约束（中间点必须落在两端之间）。
    2 个：任意两点共线，只要求间距接近某对参考间距。
    1 个：取支持度最高的那个。
    同一个候选子集里凑不出能用的一组时，失败原因会说明卡在哪一条约束上，
    方便现场调阈值。
    """

    triple = _select_triple(candidates, image_width, config)
    if triple is not None:
        _, ordered, collinearity, _ = triple
        return tuple(ordered), 3, collinearity, ""

    span_floor = image_width * config.min_station_span_ratio
    if not candidates:
        return (), 0, 0.0, "画面里没有找到同心圆环（检查光照/Canny 阈值）"
    if len(candidates) == 1:
        return (candidates[0],), 1, 0.0, ""
    if len(candidates) == 2:
        pair = _select_pair(candidates, image_width, config)
        if pair is not None:
            ordered, collinearity = pair
            return tuple(ordered), 2, collinearity, ""
        span = float(math.dist(candidates[0].center, candidates[1].center))
        return (), 0, 0.0, (f"只有 2 个候选，间距 {span:.0f}px 低于下限 "
                            f"{span_floor * 0.5:.0f}px")

    spans = [
        float(math.dist(a.center, b.center))
        for a, b in combinations(candidates, 2)
    ]
    widest = max(spans) if spans else 0.0
    if widest < span_floor:
        return (), 0, 0.0, (f"有 {len(candidates)} 个候选，但最大间距 {widest:.0f}px "
                            f"低于三环跨度下限 {span_floor:.0f}px（车太远/环太小）")
    return (), 0, 0.0, (f"有 {len(candidates)} 个候选，但没有一组满足共线约束"
                        f"（共线误差需 <= {config.max_collinearity_error_ratio}）")


def _select_triple(candidates, image_width, config):
    best = None
    for triple in combinations(candidates, 3):
        points = np.asarray([item.center for item in triple])
        endpoints = max(
            (float(np.linalg.norm(points[a] - points[b])), a, b)
            for a in range(3) for b in range(a)
        )
        span, first_index, third_index = endpoints
        if span < image_width * config.min_station_span_ratio:
            continue
        middle_index = 3 - first_index - third_index
        line = points[third_index] - points[first_index]
        offset = points[middle_index] - points[first_index]
        fraction = float(np.dot(offset, line) / (span * span))
        # 二维叉积的 z 分量。不能用 np.cross —— NumPy 2.x 对二维向量会直接抛
        # ValueError("Both input arrays must be 3-dimensional vectors")。
        cross_z = float(line[0] * offset[1] - line[1] * offset[0])
        perpendicular = abs(cross_z) / span
        collinearity = perpendicular / span
        if not 0.2 < fraction < 0.8:
            continue
        if collinearity > config.max_collinearity_error_ratio:
            continue

        endpoint_candidates = [triple[first_index], triple[third_index]]
        ring1, ring3 = sorted(
            endpoint_candidates,
            key=lambda item: item.center[0],
            reverse=config.ring1_side == "right",
        )
        ordered = (ring1, triple[middle_index], ring3)
        spacing_12 = float(np.linalg.norm(ordered[1].center - ordered[0].center))
        spacing_23 = float(np.linalg.norm(ordered[2].center - ordered[1].center))
        balance = abs(math.log(spacing_12 / spacing_23))
        support = sum(item.support for item in ordered)
        diameter_penalty = 0.0
        for item, spacing in zip(
            ordered, (spacing_12, (spacing_12 + spacing_23) / 2, spacing_23)
        ):
            ratio = item.effective_diameter_px / spacing
            if ratio < 0.25:
                diameter_penalty += 0.25 - ratio
            elif ratio > 1.10:
                diameter_penalty += ratio - 1.10
        score = 8.0 * collinearity + 0.35 * balance + 2.0 * diameter_penalty - 0.015 * support
        if best is None or score < best[0]:
            best = (score, ordered, collinearity, support)
    return best


def _affine_residual(ordered, config):
    """三环的局部仿射质量指标：用中环椭圆与三环中心距互校。"""

    image_x_per_mm = (
        ordered[2].center - ordered[0].center
    ) / (2.0 * config.center_spacing_mm)
    outer = ordered[1].ellipse
    theta = math.radians(outer.angle_deg)
    rotation = np.asarray([
        [math.cos(theta), -math.sin(theta)],
        [math.sin(theta), math.cos(theta)],
    ])
    ellipse_shape = rotation @ np.diag([
        (outer.axes[0] * 0.5) ** 2,
        (outer.axes[1] * 0.5) ** 2,
    ]) @ rotation.T
    radius_mm = config.outer_diameter_mm * 0.5
    remaining = ellipse_shape / (radius_mm * radius_mm) - np.outer(
        image_x_per_mm, image_x_per_mm
    )
    eigenvalues = np.linalg.eigvalsh((remaining + remaining.T) * 0.5)
    largest = max(float(eigenvalues[-1]), 0.0)
    return abs(float(eigenvalues[0])) / largest if largest > 1e-9 else math.inf


def _raw_detection(image, config):
    # 树莓派上 1280x720 全图处理太慢：Canny 约 15ms、几百个轮廓的 fitEllipse
    # 再几十毫秒，一帧 100ms 起。
    # 统一在 0.5 缩图上检测，返回前把坐标乘回去。相机仍出 1280x720，
    # 所以标定参数、参考环心、ROI 坐标全都不用动。
    # 1 缩图像素 = 2 原始像素 ≈ 0.75mm，对 2mm 的对位验收绰绰有余。
    scale = 0.5 if image.shape[1] >= 1000 else 1.0
    if scale < 1.0:
        work = cv2.resize(
            image, None, fx=scale, fy=scale,
            interpolation=cv2.INTER_AREA,
        )
        # 所有按绝对像素定的门槛都要跟着缩，否则缩图上全都不成立。
        work_config = replace(
            config,
            min_minor_axis_px=config.min_minor_axis_px * scale,
            max_major_axis_px=config.max_major_axis_px * scale,
            center_cluster_radius_px=config.center_cluster_radius_px * scale,
            roi_margin_px=max(1, int(round(config.roi_margin_px * scale))),
            min_contour_points=max(
                6, int(round(config.min_contour_points * scale))
            ),
        )
    else:
        work = image
        work_config = config

    observations = _extract_observations(work, work_config)
    candidates = _cluster_observations(observations, work_config)
    ordered, count, collinearity, reject = _select_group(
        candidates, work.shape[1], work_config
    )
    diagnostics = {
        "observations": len(observations),
        "candidates": len(candidates),
    }
    if count == 0:
        diagnostics["count"] = 0
        diagnostics["reject"] = reject
        return diagnostics

    centers = tuple(tuple(float(v) for v in item.center) for item in ordered)
    # 三环等间距共线：中间环必须落在两端连线的中点上。这条几何约束比
    # "被物料挡掉大半的 R2 自己拟合"可靠得多——物料是圆柱，恰好把 R2 的中心
    # 区域盖住，可见的只剩边缘零碎弧，拟合出来的圆心会明显偏。
    # 只在中点偏差过大（> 环直径的 8%）时才替换，避免把真实的透视差异当误差。
    if count == 3:
        first_point = np.asarray(centers[0], dtype=float)
        middle_point = np.asarray(centers[1], dtype=float)
        last_point = np.asarray(centers[2], dtype=float)
        midpoint = 0.5 * (first_point + last_point)
        ring_diameter = max(item.effective_diameter_px for item in ordered)
        if float(np.linalg.norm(middle_point - midpoint)) > max(
            12.0, ring_diameter * 0.08
        ):
            centers = (
                centers[0], (float(midpoint[0]), float(midpoint[1])), centers[2]
            )
    # support 按"圈数"归一：一个同心组里每圈贡献一个观测（Hough 每圈投一个圆），
    # 6 圈即满分。原先除以 10 是按"同心轮廓数"定的——那是轮廓法的口径，一图
    # 多圈会产生十几条轮廓；而 Hough 口径下 5 圈就是 5 个观测，除 10 只能拿到
    # 0.5，把整体置信度系统性压低，导致三环永远过不了 minimum_confidence。
    support = min(item.support for item in ordered) / 6.0
    bands = min(item.diameter_bands for item in ordered) / 6.0

    # 3 个环 -> 能算方向、透视比、仿射残差；
    # 2 个环 -> 只能算方向（两点确定一条线）；
    # 1 个环 -> 连方向都没有，只给位置。
    yaw = None
    if count >= 2:
        first, last = ordered[0].center, ordered[-1].center
        yaw = math.degrees(math.atan2(
            float(last[1] - first[1]), float(last[0] - first[0])))

    perspective = None
    residual = None
    if count == 3:
        spacing_12 = float(np.linalg.norm(ordered[1].center - ordered[0].center))
        spacing_23 = float(np.linalg.norm(ordered[2].center - ordered[1].center))
        perspective = spacing_12 / spacing_23
        geometry = math.exp(
            -10.0 * collinearity - 0.5 * abs(math.log(perspective)))
        residual = _affine_residual(ordered, config)
        confidence = min(
            1.0,
            0.55 * geometry + 0.25 * min(support, 1.0) + 0.20 * min(bands, 1.0),
        )
    else:
        # 缺环时共线性和透视比都无从评估，置信度只反映轮廓证据强度，
        # 再乘一个缺环惩罚，让调用方一眼看出这是部分观测。
        base = 0.45 * min(support, 1.0) + 0.35 * min(bands, 1.0)
        confidence = min(1.0, base * (0.85 if count == 2 else 0.65))

    result = {
        **diagnostics,
        "centers": centers,
        "ellipses": tuple((
            tuple(float(v) for v in item.ellipse.center),
            tuple(float(v) for v in item.ellipse.axes),
            float(item.ellipse.angle_deg),
        ) for item in ordered),
        "confidence": float(confidence),
        "yaw": yaw,
        "perspective": perspective,
        "residual": residual,
        "count": count,
    }
    if scale < 1.0:
        # 坐标还原到原始分辨率：上层（_assign_to_reference / _locate_ring2 /
        # 参考环心 / _make_roi）全部按原始像素工作，不感知这里的降采样。
        inverse = 1.0 / scale
        result["centers"] = tuple(
            (x * inverse, y * inverse) for x, y in result["centers"]
        )
        result["ellipses"] = tuple(
            ((center[0] * inverse, center[1] * inverse),
             (axes[0] * inverse, axes[1] * inverse), angle)
            for center, axes, angle in result["ellipses"]
        )
    return result


def _normalize_angle(angle):
    return (float(angle) + 180.0) % 360.0 - 180.0


def _locate_ring2(centers, indices, count, config):
    """定位环 2 的像素位置（后续所有对位误差都以它为基准）。

    - 3 环：中间那个就是环 2。
    - 2 环：按匹配到的参考索引取对应那个；若看到的是 1、3 号，取两点中点。
    - 1 环：只看到 1 个时无法确定朝向，按"无旋转误差"的假设用参考偏移推算。
      这种结果 yaw_valid=False，调用方据此判断能不能用来收尾。
    """

    if not centers or not indices:
        return None
    if count >= 3:
        return centers[1]
    if count == 2:
        if tuple(indices) == (0, 1):
            return centers[1]
        if tuple(indices) == (1, 2):
            return centers[0]
        return (0.5 * (centers[0][0] + centers[1][0]),
                0.5 * (centers[0][1] + centers[1][1]))
    index = indices[0]
    reference = config.reference_centers_px
    return (centers[0][0] + reference[1][0] - reference[index][0],
            centers[0][1] + reference[1][1] - reference[index][1])


def _assign_with_prior(centers, prior):
    """用上一帧的完整布局给这一帧的 1~2 个环定身份（比参考像素更稳）。

    缺环时拿参考像素坐标硬比是不可靠的：参考是标定距离下的尺度，车一近一远
    像素间距就变了。上一帧的布局和这一帧只差几十毫秒，尺度基本没变，所以
    "离上一帧预测位置最近"这个判据稳得多。
    """

    if prior is None:
        return None, math.inf
    layout = prior[0]
    if not layout or len(layout) != 3:
        return None, math.inf
    ordered = sorted(centers, key=lambda point: point[0])
    count = len(ordered)
    if count == 0 or count > 3:
        return None, math.inf
    best = (None, math.inf)
    for indices in combinations(range(3), count):
        if any(layout[index] is None for index in indices):
            continue
        total = sum(
            math.dist(ordered[position], layout[index])
            for position, index in enumerate(indices)
        )
        if total < best[1]:
            best = (tuple(indices), float(total))
    return best


def _predict_layout(centers, indices, ellipses, config, prior):
    """用看到的 1~3 个环推出完整三环的像素位置。

    返回 (predicted, scale, source)：
      predicted 长度固定为 3，按参考环号 0/1/2 排列，推不出的位置是 None。
      scale 是"观测像素 / 参考像素"的尺度比。
      source = observed / two_ring / two_ring_partial / prior / one_ring。

    这是"遮挡时不漏检"的核心：物料挡住一个环时，用已知的 150mm 间距和已看到
    的环，把被挡的环也算出来，稳定窗口就不会因为这一帧少看一个环被清空。
    """

    ref = config.reference_centers_px
    count = len(centers)
    if count <= 0 or indices is None or len(indices) != count:
        return (None, None, None), 0.0, ""

    if count >= 3:
        layout = [None, None, None]
        for center, index in zip(centers, indices):
            layout[int(index)] = (float(center[0]), float(center[1]))
        i0, i2 = int(indices[0]), int(indices[-1])
        span_ref = math.dist(ref[i0], ref[i2])
        span_obs = math.dist(centers[0], centers[-1])
        scale = (span_obs / span_ref) if span_ref > 1e-6 else 0.0
        return tuple(layout), float(scale), "observed"

    if count == 2:
        i, j = int(indices[0]), int(indices[1])
        ci = np.asarray(centers[0], dtype=float)
        cj = np.asarray(centers[1], dtype=float)
        ri = np.asarray(ref[i], dtype=float)
        rj = np.asarray(ref[j], dtype=float)
        v_ref, v_obs = rj - ri, cj - ci
        n_ref = float(np.linalg.norm(v_ref))
        n_obs = float(np.linalg.norm(v_obs))
        if n_ref < 1e-6 or n_obs < 1e-6:
            return (None, None, None), 0.0, ""
        # 先用环的投影直径校验这对环的间距，别让两个假候选生成错误布局，
        # 再污染后续所有帧的先验。
        try:
            diameter_px = float(np.mean([max(ellipses[k][1]) for k in (0, 1)]))
        except (IndexError, KeyError, TypeError, ValueError):
            diameter_px = 0.0
        if (diameter_px > 1e-6 and config.outer_diameter_mm > 0
                and config.center_spacing_mm > 0):
            expected_span = (
                abs(j - i) * config.center_spacing_mm
                * diameter_px / config.outer_diameter_mm
            )
            if expected_span > 1e-6:
                ratio = n_obs / expected_span
                if not 0.75 <= ratio <= 1.35:
                    # 间距和环大小对不上：只回这两个观测点，不预测第三个。
                    layout = [None, None, None]
                    layout[i] = (float(ci[0]), float(ci[1]))
                    layout[j] = (float(cj[0]), float(cj[1]))
                    return tuple(layout), 0.0, "two_ring_partial"
        scale = n_obs / n_ref
        angle = math.atan2(v_obs[1], v_obs[0]) - math.atan2(v_ref[1], v_ref[0])
        cos_a, sin_a = math.cos(angle), math.sin(angle)
        layout = []
        for k in range(3):
            d = np.asarray(ref[k], dtype=float) - ri
            p = ci + scale * np.array([
                cos_a * d[0] - sin_a * d[1],
                sin_a * d[0] + cos_a * d[1],
            ])
            layout.append((float(p[0]), float(p[1])))
        return tuple(layout), float(scale), "two_ring"

    # 只剩 1 个环：优先沿用上一帧的完整布局（只平移），但先验证这个观测和
    # 先验是同一个东西——位置不能跳太远、环的投影直径也不能和先验尺度矛盾。
    i = int(indices[0])
    ci = np.asarray(centers[0], dtype=float)
    if prior is not None:
        prior_layout, prior_scale, _ = prior
        if (prior_scale > 0 and i < len(prior_layout)
                and prior_layout[i] is not None):
            spacing_px = prior_scale * math.dist(ref[0], ref[1])
            jump = float(math.dist(ci, prior_layout[i]))
            try:
                diameter_px = float(max(ellipses[0][1]))
            except (IndexError, KeyError, TypeError, ValueError):
                diameter_px = 0.0
            expected_diameter = (spacing_px * config.outer_diameter_mm
                                 / config.center_spacing_mm
                                 if config.center_spacing_mm > 0 else 0.0)
            diameter_ok = (
                diameter_px <= 1e-6 or expected_diameter <= 1e-6
                or 0.5 <= diameter_px / expected_diameter <= 2.0
            )
            if jump <= max(40.0, 0.6 * spacing_px) and diameter_ok:
                dx = ci[0] - prior_layout[i][0]
                dy = ci[1] - prior_layout[i][1]
                layout = [
                    None if p is None else (float(p[0] + dx), float(p[1] + dy))
                    for p in prior_layout
                ]
                return tuple(layout), float(prior_scale), "prior"

    # 没有可用先验：用这个环的投影直径估间距，方向沿用参考布局的朝向。
    try:
        diameter_px = float(max(ellipses[0][1]))
    except (IndexError, KeyError, TypeError, ValueError):
        return (None, None, None), 0.0, ""
    if (diameter_px <= 1e-6 or config.outer_diameter_mm <= 0
            or config.center_spacing_mm <= 0):
        return (None, None, None), 0.0, ""
    spacing_px = diameter_px * (config.center_spacing_mm / config.outer_diameter_mm)
    u = np.asarray(ref[2], dtype=float) - np.asarray(ref[0], dtype=float)
    n = float(np.linalg.norm(u))
    if n < 1e-6:
        return (None, None, None), 0.0, ""
    u /= n
    layout = []
    for k in range(3):
        p = ci + u * spacing_px * (k - i)
        layout.append((float(p[0]), float(p[1])))
    return tuple(layout), 0.0, "one_ring"


class RingStationDetector:
    """全图搜索后转入 ROI，并只由连续直接观测产生稳定结果。"""

    def __init__(self, config: RingStationConfig):
        self.config = config
        self._samples = deque(maxlen=config.stable_frame_count)
        self._tracking_roi = None
        self._roi_failures = 0
        # 上一帧推出来的完整三环布局：只剩 1~2 个环时沿用它，稳定窗口才不会
        # 因为"这一帧少看一个环"被清空。
        self._last_layout = None

    def reset(self):
        """清除 ROI 和稳定窗口，防止跨模式使用旧坐标。"""

        self._samples.clear()
        self._tracking_roi = None
        self._roi_failures = 0
        self._last_layout = None

    def _make_roi(self, raw, shape):
        points = np.asarray(raw["centers"])
        # 补位占位的环没有实测椭圆（None），算包围盒时要跳过它们，
        # 否则 max(ellipse[1]) 会在 None 上炸。
        valid_ellipses = [
            ellipse for ellipse in raw["ellipses"] if ellipse is not None
        ]
        half_diameter = (
            max(max(ellipse[1]) for ellipse in valid_ellipses) / 2.0
            if valid_ellipses else 0.0
        )
        margin = self.config.roi_margin_px + half_diameter
        height, width = shape[:2]
        return (
            max(0, int(math.floor(points[:, 0].min() - margin))),
            max(0, int(math.floor(points[:, 1].min() - margin))),
            min(width, int(math.ceil(points[:, 0].max() + margin))),
            min(height, int(math.ceil(points[:, 1].max() + margin))),
        )

    @staticmethod
    def _offset_raw(raw, x0, y0):
        # 未检出时 _raw_detection 也会返回一个字典（只带观测数/候选数/失败原因），
        # 里面没有 centers/ellipses —— 这里必须先判断，否则 ROI 路径会 KeyError。
        if raw is None or "centers" not in raw:
            return raw
        raw["centers"] = tuple(
            (x + x0, y + y0) for x, y in raw["centers"]
        )
        raw["ellipses"] = tuple(
            ((center[0] + x0, center[1] + y0), axes, angle)
            for center, axes, angle in raw["ellipses"]
        )
        return raw

    def _find(self, frame):
        if self._tracking_roi is None:
            return _raw_detection(frame, self.config), "full"
        x0, y0, x1, y1 = self._tracking_roi
        raw = _raw_detection(frame[y0:y1, x0:x1], self.config)
        raw = self._offset_raw(raw, x0, y0)
        if raw is not None and int(raw.get("count", 0)) > 0:
            self._roi_failures = 0
            return raw, "roi"
        self._roi_failures += 1
        if self._roi_failures < self.config.roi_fallback_after_failures:
            # 必须原样返回 raw：_raw_detection 现在总是返回字典（带观测数/候选数/
            # 失败原因），调用方要靠它把"为什么没检到"打到画面上。
            # 以前这里返回 None，新 detect 直接 raw.get() 会 AttributeError。
            return raw, "roi"
        self._roi_failures = 0
        return _raw_detection(frame, self.config), "full_fallback"

    def detect(self, frame) -> RingStationDetection:
        """检测一帧，返回圆环对位误差、纠偏量、质量和耗时。"""

        started = time.perf_counter()
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            raise RingStationDetectionError("圆环检测要求 BGR 彩色图像")
        # 1. 提取椭圆、合并同心轮廓、筛共线圆环；有ROI时优先局部搜索。
        #    能看到 3 个最好，只看到 2 个或 1 个也照样输出（降级观测）。
        raw, scope = self._find(frame)
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        if raw is None:
            # 兜底：任何路径返回 None 都不能让检测器抛异常，否则一帧失败就带崩
            # 整个主循环（rclpy/串口那边的定时器同理）。
            raw = {"count": 0, "observations": 0, "candidates": 0,
                   "reject": "未返回检测结果"}
        if int(raw.get("count", 0)) == 0:
            self._samples.clear()
            return RingStationDetection(
                processing_ms=elapsed_ms,
                detection_scope=scope,
                detail=(f"未检出（椭圆 {raw.get('observations', 0)} 个 -> "
                        f"候选 {raw.get('candidates', 0)} 个）："
                        f"{raw.get('reject', '没有找到圆环')}"),
                ellipse_observation_count=int(raw.get("observations", 0)),
                ring_candidate_count=int(raw.get("candidates", 0)),
            )

        config = self.config
        measured_count = int(raw["count"])   # 这一帧**实际看到**几个环（报告给调用方）
        count = measured_count
        # 环的归属必须在**偏移过 ROI 之后**的最终坐标上判断：
        # 参考环心是全图坐标，拿裁剪后的局部坐标去匹配会全错。
        # （3 个环时归属恒为 (0,1,2)，所以这个坑只在缺环时才暴露。）
        indices = None
        if count < 3 and self._last_layout is not None:
            # 有上一帧的完整布局时，先按"离预测位置最近"定身份——缺环时这比
            # 拿参考像素硬比稳得多（车近车远，参考像素的尺度都对不上）。
            indices, match_error = _assign_with_prior(
                raw["centers"], self._last_layout)
        if indices is None:
            indices, match_error = _assign_to_reference(
                raw["centers"], config, raw["ellipses"])
        # 用看到的环推完整三环布局：缺环时靠它补位，让稳定窗口看到恒定的 3 环。
        predicted, layout_scale, layout_source = _predict_layout(
            raw["centers"], indices, raw["ellipses"], config,
            self._last_layout)
        filled = False
        original_indices = indices
        if (count < 3 and layout_source in ("two_ring", "prior")
                and predicted and len(predicted) == 3
                and all(point is not None for point in predicted)):
            # 只用可信的预测补位：two_ring（两个环 + 间距校验通过的仿射推算）和
            # prior（沿用上一帧布局的平移）都带尺度校验；two_ring_partial 和
            # one_ring 的尺度是硬估的，补进去会让窗口一直漂。
            # ellipses 必须和 centers 一一对齐：补位后 centers 按 0/1/2 排，
            # 而实测的 ellipses 还是原顺序。不对齐的话 draw 里 zip 会把 R2 的
            # 椭圆配到 R1 的圆心上去——现场看到的就是"R1 标在 R2 上"。
            # 缺的位置留 None，draw 会跳过椭圆、只画圆心十字。
            measured_ellipses = list(raw["ellipses"])
            aligned_ellipses = [None, None, None]
            for order, index in enumerate(original_indices or ()):
                if 0 <= int(index) < 3 and order < len(measured_ellipses):
                    aligned_ellipses[int(index)] = measured_ellipses[order]
            raw["centers"] = tuple(
                (float(point[0]), float(point[1])) for point in predicted
            )
            raw["ellipses"] = tuple(aligned_ellipses)
            indices = (0, 1, 2)
            count = 3
            raw["count"] = 3
            # 补位后必须把航向重算一遍：走 prior 分支说明实测只有 1 个环，
            # 而 _raw_detection 在 count<2 时根本不设 yaw（留 None）。count 被
            # 改成 3 之后 yaw_valid 变 True，后面拿 None 做减法会直接 TypeError。
            # 用补位后的两端环心定方向，才和这一帧的几何一致。
            first_center, last_center = raw["centers"][0], raw["centers"][-1]
            raw["yaw"] = math.degrees(math.atan2(
                float(last_center[1] - first_center[1]),
                float(last_center[0] - first_center[0]),
            ))
            filled = True
        raw["indices"] = indices        # 存进 samples，稳定窗口靠它判断"看到的还是同样几个环"
        # 记住可信的布局，供下一帧缺环时沿用。
        if layout_source in ("observed", "two_ring") and layout_scale > 0:
            self._last_layout = (predicted, layout_scale, layout_source)
        elif filled and layout_source == "prior":
            self._last_layout = (predicted, layout_scale, layout_source)
        if count == 3:
            self._tracking_roi = self._make_roi(raw, frame.shape)
        else:
            # 缺环时**不能**锁 ROI：_make_roi 是按"当前检到的环"算包围盒的，
            # 只看到 1~2 个时 ROI 会收缩到它们周围，其余环永远落在窗口外，
            # 下一帧更检不到 —— 自锁，再也恢复不了三环。缺环就退回全图搜索。
            self._tracking_roi = None
        # 2. 只有 3 个环才做完整的质量门槛（置信度 / 仿射残差 / 透视比）。
        #    缺环时这三项都无从评估，改用聚类层面的证据（同心轮廓支持数、
        #    直径波段数）把住，并由 visible_ring_count 告诉调用方这是部分观测。
        quality_detail = None
        if count == 3 and not filled:
            # 补位帧不做质量门槛：residual / perspective 在实测 2 环时是 None，
            # 拿去比大小会直接 TypeError；而且这一帧的置信度本来就只能由
            # 实测的圈数给出，拿三环标准卡它没有意义。
            if raw["confidence"] < config.minimum_confidence:
                quality_detail = "三圆环识别置信度不足"
            elif raw["residual"] > config.maximum_affine_residual:
                quality_detail = "局部仿射模型残差过大"
            elif not (
                config.perspective_ratio_range[0]
                <= raw["perspective"]
                <= config.perspective_ratio_range[1]
            ):
                quality_detail = "圆环透视比超出标定适用范围"
        elif count == 2:
            # 用环的投影直径估像素/毫米，再和物理间距比。不能拿参考像素间距
            # 直接比：参考是标定距离下的尺度（253px），现场拍到 400px，
            # 比值 1.58 会被误判成"间距不符"——这会让本来正确的两环配对被否掉。
            try:
                diameter_px = float(np.median(
                    [max(ellipse[1]) for ellipse in raw["ellipses"]]
                ))
            except (IndexError, KeyError, TypeError, ValueError):
                diameter_px = 0.0
            observed_span = float(math.dist(raw["centers"][0], raw["centers"][1]))
            if (indices is not None and len(indices) == 2
                    and diameter_px > 1e-6 and config.outer_diameter_mm > 0):
                px_per_mm = diameter_px / config.outer_diameter_mm
                spacing_mm = (indices[1] - indices[0]) * config.center_spacing_mm
                expected_px = spacing_mm * px_per_mm
                if expected_px > 1e-6:
                    ratio = observed_span / expected_px
                    if not 0.75 <= ratio <= 1.35:
                        quality_detail = "仅见 2 环，间距与所匹配的参考对不符"
        # count == 1 不加任何闸。单环既没有间距可交叉验证、也没有第二个点定方向，
        # "是哪一号环"只能靠离参考位置最近来猜；而这个猜测在车偏移超过约半个环距时
        # 必然失效（那时这个环的位置恰好就像隔壁那号）。这是信息量的硬限制，
        # 用匹配残差之类的判据拦不住（认错号时残差反而更小），加闸只会误杀
        # 正常的远距离观测。调用方应当把 visible_ring_count==1 视为"位置可用、
        # 但环号存疑、航向未知"，只在已经基本对准的前提下参考它。
        # count == 1 不做置信度门槛：缺环时共线性/透视都无从评估，置信度被
        # 系统性压低，按 3 环的标准卡会永远过不去。单环证据由聚类层面保证
        # （min_cluster_support 个同心轮廓 + min_diameter_bands 个直径波段）。

        if quality_detail is not None:
            self._samples.clear()
            return RingStationDetection(
                detected=True,
                confidence=raw["confidence"],
                centers_px=raw["centers"],
                ellipses=raw["ellipses"],
                station_yaw_deg=(raw["yaw"] if raw["yaw"] is not None
                                 else config.reference_yaw_deg),
                affine_residual=(raw["residual"] if raw["residual"] is not None
                                 else math.inf),
                perspective_ratio=(raw["perspective"]
                                   if raw["perspective"] is not None
                                   else math.inf),
                processing_ms=elapsed_ms,
                detection_scope=scope,
                detail=quality_detail,
                visible_ring_count=measured_count,
                predicted_centers_px=predicted,
                layout_scale=layout_scale,
                layout_source=layout_source,
                ring_indices=indices,
                yaw_valid=(count >= 2),
                ellipse_observation_count=int(raw.get("observations", 0)),
                ring_candidate_count=int(raw.get("candidates", 0)),
            )

        # 可见环的数量或"是哪几个"一变，稳定窗口必须重来（两者的中值不能混算）。
        if self._samples and (
            int(self._samples[-1]["count"]) != count
            or tuple(self._samples[-1]["indices"] or ()) != tuple(indices)
        ):
            self._samples.clear()
        self._samples.append(raw)
        # 连续帧中，每个圆心的X/Y分别取中值；坐标单位为原图像素。
        centers = []
        for ring_index in range(count):
            xs = [sample["centers"][ring_index][0] for sample in self._samples]
            ys = [sample["centers"][ring_index][1] for sample in self._samples]
            centers.append((float(np.median(xs)), float(np.median(ys))))
        centers = tuple(centers)

        maximum_jitter = 0.0
        for sample in self._samples:
            for ring_index in range(count):
                dx = sample["centers"][ring_index][0] - centers[ring_index][0]
                dy = sample["centers"][ring_index][1] - centers[ring_index][1]
                maximum_jitter = max(maximum_jitter, math.hypot(dx, dy))
        stable = bool(
            len(self._samples) == config.stable_frame_count
            and maximum_jitter <= config.maximum_center_jitter_px
        )

        # 只看到 1 个环时没有方向可言：航向误差记 0 并标记 yaw_valid=False，
        # 由调用方决定能不能拿它收尾（within_tolerance 会因此恒为 False）。
        # 防御：样本里只要有一个 yaw 是 None（比如补位前那一帧），整体就退回
        # "航向未知"，而不是拿 None 去做减法崩掉整个主循环。
        valid_yaws = [
            sample["yaw"] for sample in self._samples
            if sample.get("yaw") is not None
        ]
        yaw_valid = bool(count >= 2 and valid_yaws)
        if yaw_valid:
            angle_error = float(np.median([
                _normalize_angle(value - config.reference_yaw_deg)
                for value in valid_yaws
            ]))
            yaw = _normalize_angle(config.reference_yaw_deg + angle_error)
        else:
            angle_error = 0.0
            yaw = config.reference_yaw_deg

        affine = np.asarray(config.image_to_station_affine)
        # 图像像素偏差乘2x2标定矩阵，得到工位平面上的毫米偏差。
        # 只对**看得见**的环算误差；看不见的没法检查，不能假装它是准的。
        indices = tuple(indices)
        metric_errors = []
        for center, index in zip(centers, indices):
            pixel_error = np.asarray(center) - np.asarray(
                config.reference_centers_px[index]
            )
            metric_errors.append(affine @ pixel_error)
        metric_errors = tuple(metric_errors)
        # error_x/error_y 始终是"环 2 相对参考位"的偏差 —— 对准就是让环 2 到位。
        ring2_center = _locate_ring2(centers, indices, count, config)
        ring2_error = affine @ (
            np.asarray(ring2_center)
            - np.asarray(config.reference_centers_px[1])
        )
        error_x, error_y = float(ring2_error[0]), float(ring2_error[1])
        position_error = (max(float(np.linalg.norm(error)) for error in metric_errors)
                          if metric_errors else math.inf)
        within = bool(
            stable
            and yaw_valid
            and position_error <= config.maximum_position_error_mm
            and abs(angle_error) <= config.maximum_angle_error_deg
        )
        # 3x3底盘标定矩阵把(x毫米, y毫米, 角度)转换为底盘纠偏量。
        correction = None
        if config.chassis_image_to_motion is not None:
            correction = np.asarray(config.chassis_image_to_motion) @ np.asarray(
                (error_x, error_y, angle_error)
            )
        scope_text = f"{count}环{indices if indices else ''}"
        if not stable:
            detail = (f"{scope_text} 等待稳定观测"
                      f"（{len(self._samples)}/{config.stable_frame_count}）")
        elif not yaw_valid:
            detail = f"{scope_text} 只见到 1 个环，航向未知，不能收尾"
        elif within:
            detail = f"{scope_text} 工位视觉残差已达到验收精度"
        else:
            detail = f"{scope_text} 工位偏差超出验收精度"
        return RingStationDetection(
            detected=True,
            stable=stable,
            confidence=float(np.median([s["confidence"] for s in self._samples])),
            centers_px=centers,
            ellipses=raw["ellipses"],
            station_yaw_deg=yaw,
            angle_error_deg=angle_error,
            error_x_mm=error_x,
            error_y_mm=error_y,
            position_error_mm=position_error,
            correction_body_x_mm=None if correction is None else float(correction[0]),
            correction_body_y_mm=None if correction is None else float(correction[1]),
            correction_yaw_deg=None if correction is None else float(correction[2]),
            affine_residual=float(np.median([
                s["residual"] for s in self._samples
                if s["residual"] is not None] or [math.inf])),
            perspective_ratio=float(np.median([
                s["perspective"] for s in self._samples
                if s["perspective"] is not None] or [math.inf])),
            within_tolerance=within,
            processing_ms=elapsed_ms,
            detection_scope=scope,
            detail=detail,
            visible_ring_count=measured_count,
            predicted_centers_px=predicted,
            layout_scale=layout_scale,
            layout_source=layout_source,
            ring_indices=indices,
            yaw_valid=yaw_valid,
            ellipse_observation_count=int(raw.get("observations", 0)),
            ring_candidate_count=int(raw.get("candidates", 0)),
        )


def draw_ring_station_detection(frame, result: RingStationDetection):
    """在原画面上绘制当前帧的三环、误差和检测质量。"""

    if not result.detected:
        cv2.putText(
            frame, "Ring station: SEARCHING", (20, 95),
            cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0, 0, 255), 2,
            cv2.LINE_AA,
        )
        # 把"卡在哪一步"直接打出来：obs=提取到的椭圆数，cand=聚类后的环候选数。
        # obs 为 0 是光照/Canny 问题；obs 有但 cand 为 0 是同心轮廓/直径波段不够。
        cv2.putText(
            frame,
            f"obs={result.ellipse_observation_count}  "
            f"cand={result.ring_candidate_count}",
            (20, 132), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 255), 2,
            cv2.LINE_AA,
        )
        return frame
    colors = ((0, 180, 255), (0, 255, 0), (255, 160, 0))
    # 编号必须用 ring_indices，不能用 enumerate 的下标：
    # R1 漏检时剩下的是 R2、R3，用下标编号会把它们叫成 R1、R2——这正是
    # "R1 标在 R2 上"的观感来源。
    labels = result.ring_indices or ()
    points = []
    for position, (center, ellipse, color) in enumerate(
        zip(result.centers_px, result.ellipses, colors)
    ):
        ring_number = (
            int(labels[position]) + 1
            if position < len(labels) else position + 1
        )
        # 补位占位的环没有实测椭圆（ellipse is None）：只画十字，而且用灰色，
        # 和"这一帧真的看到了"的彩色环区分开。
        predicted_only = ellipse is None
        draw_color = (150, 150, 150) if predicted_only else color
        if not predicted_only:
            cv2.ellipse(frame, ellipse, color, 2, cv2.LINE_AA)
        point = tuple(int(round(value)) for value in center)
        points.append(point)
        cv2.drawMarker(frame, point, draw_color, cv2.MARKER_CROSS, 24, 2)
        cv2.putText(
            frame, f"R{ring_number}", (point[0] + 10, point[1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.65, draw_color, 2, cv2.LINE_AA,
        )
    # 只有一个点时 cv2.polylines 会触发断言（要求至少两点），单独判断。
    if len(points) >= 2:
        cv2.polylines(frame, [np.asarray(points, np.int32)], False, (255, 0, 255), 2)
    cv2.rectangle(frame, (10, 65), (820, 145), (0, 0, 0), -1)
    # 环数和"是哪几个"直接打在状态行前：夹爪伸出去挡住画面时一眼能看出
    # 现在是靠几个环在算，少环时也知道是哪两个。
    cv2.putText(
        frame,
        f"Rings {result.visible_ring_count}/3 {tuple(result.ring_indices)}  "
        f"Ring: {'STABLE' if result.stable else 'WAITING'}  "
        f"dX={result.error_x_mm:+.2f}mm dY={result.error_y_mm:+.2f}mm "
        f"dYaw={result.angle_error_deg:+.2f}deg"
        f"{'' if result.yaw_valid else '  (yaw unknown)'}",
        (20, 98), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
        (80, 255, 80) if result.within_tolerance else (255, 255, 255),
        2, cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        f"obs={result.ellipse_observation_count} "
        f"cand={result.ring_candidate_count}  "
        f"Confidence={result.confidence:.3f} "
        f"residual={'n/a' if not math.isfinite(result.affine_residual) else f'{result.affine_residual:.3f}'} "
        f"perspective={'n/a' if not math.isfinite(result.perspective_ratio) else f'{result.perspective_ratio:.3f}'} "
        f"{result.processing_ms:.1f}ms",
        (20, 132), cv2.FONT_HERSHEY_SIMPLEX, 0.60,
        (255, 255, 255), 2, cv2.LINE_AA,
    )
    return frame
