#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""行车视觉：车道保持 + 黑色障碍检测。

和工位视觉（转盘/圆环/物料）的区别
----------------------------------
工位视觉是**垂直俯视**一个平面，"像素 → 毫米"是一个常数比例。
行车视觉是摄像头**抬起斜视前方**，画面有透视：同一个像素宽度，近处代表
的毫米数比远处小。所以本模块的量测都尽量用"已知物理尺寸反推"的方式：

    车道宽 400 mm 是命题给定的 → 量出它在画面里多少像素 → 得到该行的 mm/px

这样不需要提前标定镜头，换相机、换抬升角度都能自适应（只要车道宽度不变）。

输出
----
- 车道：左右边界点、中心横向偏差、车道方向角
- 障碍：黑色圆柱的质心、底边、粗估距离、所占宽度
- 决策：GO / BLOCKED_LEFT / BLOCKED_RIGHT / BLOCKED / STOP
  （"BLOCKED_xxx" 表示前方被挡，建议的状态机动作）

调参
----
所有阈值都在 `build_default_document()` 里，可存成 JSON 现场调。
**这些默认值是按常见光照给的起点，必须拿真实画面调。**
用 `--show` 看实时调试图，`--dump` 存一张调试图。

直接运行（现场调参）：
    python3 lane_obstacle_detector.py --device /dev/video2 --show
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np


# ==================== 配置 ====================

def build_default_document() -> dict:
    """返回默认参数字典（可直接存 JSON）。"""

    return {
        "format_version": 1,
        "color_space": "OpenCV_HSV",

        # ---- 车道（灰色）分割 ----
        # 读数是 R180 G184 B187 那种灰。白底 V 更高，淡黄区 S 更高，
        # 所以用「低饱和 + 中亮度」把灰色挑出来。
        "lane": {
            "max_saturation": 45,       # S 上限：排除淡黄(黄 S 高)
            "min_value": 120,           # V 下限：排除阴影/暗角
            "max_value": 222,           # V 上限：排除白底(V 更高)
            "roi_top_ratio": 0.30,      # 只在画面下方 70% 找车道（上方是远景，噪声大）
            "open_kernel_px": 5,        # 开运算：去小噪点
            "close_kernel_px": 15,      # 闭运算：填车道上的小洞（文字/接缝）
            "min_lane_width_px": 60,    # 某一行车道窄于此值视为无效
            "max_lane_width_px": 1100,  # 某一行车道宽于此值视为已进入交叉口/误检
            "scan_rows": 24,            # 纵向采样多少行
        },

        # ---- 横向偏差与方向 ----
        "geometry": {
            "lane_width_mm": 400.0,     # 命题给定，用来反推 mm/px
            "near_row_ratio": 0.88,     # 用画面这个高度算横向偏差（近处准）
            "far_row_ratio": 0.52,      # 用这个高度算方向角
            "center_offset_warn_mm": 40.0,   # 横向偏差超过它开始提示修正
        },

        # ---- 黑色障碍 ----
        "obstacle": {
            "max_value": 75,            # V 上限：黑色
            "max_saturation": 130,      # S 上限：排除深色但饱和的东西
            "min_area_px": 120,         # 太小的黑点当噪声
            "max_area_px": 40000,       # 太大的黑块当阴影
            "min_solidity": 0.55,       # 轮廓实心度
            "max_aspect_ratio": 2.6,    # 长宽比：圆柱斜看是椭圆/矩形，太细长的是阴影缝
            "must_be_on_lane": True,    # 只认落在车道内的黑块（地面阴影在车道外）
            "on_lane_margin_px": 20,
            "diameter_mm": 50.0,        # 命题：φ50×100
            "height_mm": 100.0,
        },

        # ---- 可通行性 ----
        "clearance": {
            "vehicle_width_mm": 300.0,  # 命题：铅垂投影 ≤300mm 正方形
            "extra_margin_mm": 60.0,    # 安全余量；400-50=350，余量只有50，所以会被判堵死
            "danger_zone_row_ratio": 0.72,  # 障碍底边低于这一行才算"进入危险带"
        },

        # ---- 距离粗估 ----
        # 透视下"物体底边在画面越靠下 = 越近"。用一条标定过的参考：
        # distance_mm ≈ k_mm_px / (bottom_y_px - horizon_y_px)
        # 现场把障碍放在已知距离，看它底边落在哪一行，可反解 horizon_y_px。
        "range": {
            "horizon_y_ratio": 0.34,    # 地平线所在行（画面高度比例），需现场标
            "k_mm_px": 26000.0,         # 比例系数，需现场标（见 calibrate_range）
            "min_distance_mm": 60.0,
            "max_distance_mm": 2500.0,
        },
    }


def load_config(path) -> dict:
    """读 JSON 配置，缺的字段用默认值补齐（便于以后加字段不炸旧配置）。"""
    document = json.loads(Path(path).expanduser().read_text(encoding="utf-8"))
    return _merge(build_default_document(), document)


def _merge(base: dict, override: dict) -> dict:
    out = dict(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _merge(out[key], value)
        else:
            out[key] = value
    return out


# ==================== 结果 ====================

@dataclass(frozen=True)
class Obstacle:
    center_x_px: float
    center_y_px: float
    bottom_y_px: float
    width_px: float
    height_px: float
    width_mm: float
    distance_mm: float
    area_px: float
    contour: object = None


@dataclass(frozen=True)
class LaneX:            # 某一行上的车道左右边界
    y: int
    left_x: int
    right_x: int


@dataclass(frozen=True)
class LaneObstacleDetection:
    detected: bool = False
    lane_found: bool = False

    # 车道
    lane_edges: tuple = ()          # tuple[LaneX, ...]
    offset_mm: float = 0.0          # 车相对车道中心的横向偏差（正=车偏右，需左移）
    heading_deg: float = 0.0        # 车头相对车道方向的偏角（正=车道朝左偏）
    width_px_near: float = 0.0      # 近处那行的车道宽（像素）
    width_px_far: float = 0.0       # 远处那行的车道宽（像素）
    mm_per_px_near: float = 0.0     # 近处那行的 mm/px（由已知 400mm 反推）
    far_near_ratio: float = 0.0     # 远/近宽度比。透视下应 <1；>1 说明分割或方向反了

    # 障碍
    obstacles: tuple = ()
    nearest: Obstacle | None = None
    free_left_px: float = 0.0       # 最近障碍左侧剩余宽度（像素）
    free_right_px: float = 0.0      # 右侧
    free_left_mm: float = 0.0
    free_right_mm: float = 0.0

    # 决策
    blocked: bool = False
    action: str = "STOP"            # GO / BLOCKED_LEFT / BLOCKED_RIGHT / BLOCKED / STOP
    processing_ms: float = 0.0
    detail: str = ""


# ==================== 检测器 ====================

class LaneObstacleDetector:

    def __init__(self, document: dict | None = None):
        self.config = _merge(build_default_document(), document or {})
        self._last_lane_edges = ()

    # ---------------- 车道 ----------------

    def _lane_mask(self, frame) -> np.ndarray:
        cfg = self.config["lane"]
        height = frame.shape[0]
        top = int(height * float(cfg["roi_top_ratio"]))

        # 只在下方 ROI 上做颜色转换和形态学。整幅做的话，15×15 闭运算
        # 在 1280×720 上要十几毫秒，而上方本来就是远景、用不到。
        roi = frame[top:, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower = np.array([0, 0, int(cfg["min_value"])], dtype=np.uint8)
        upper = np.array([179, int(cfg["max_saturation"]),
                          int(cfg["max_value"])], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

        k_open = int(cfg["open_kernel_px"])
        if k_open > 1:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k_open, k_open))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        k_close = int(cfg["close_kernel_px"])
        if k_close > 1:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k_close, k_close))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 放回整幅尺寸，后面的逻辑（行扫描、障碍归属）都用整幅坐标
        full = np.zeros((height, frame.shape[1]), dtype=np.uint8)
        full[top:, :] = mask

        # 填内部空洞：车道上的黑色障碍会在灰度 mask 里打出洞。
        # 如果不填，从中线向外扫描时会停在**障碍边缘**而不是车道边界，
        # 把车道量窄，进而误判可通行宽度。
        return self._fill_holes(full)

    @staticmethod
    def _fill_holes(mask: np.ndarray) -> np.ndarray:
        """把连通区域内部的洞填上（车道上的障碍、地面接缝等）。"""
        h, w = mask.shape
        flooded = mask.copy()
        ff_mask = np.zeros((h + 2, w + 2), dtype=np.uint8)
        # 从四角往内泛洪，能填到的都是"背景"；剩下没填到的非零区里的空就是洞
        for seed in ((0, 0), (w - 1, 0), (0, h - 1), (w - 1, h - 1)):
            if flooded[seed[1], seed[0]] == 0:
                cv2.floodFill(flooded, ff_mask, seed, 255)
        holes = cv2.bitwise_not(flooded)
        return cv2.bitwise_or(mask, holes)

    def _scan_lane_edges(self, mask: np.ndarray) -> tuple:
        """从画面中线向两侧扩展找车道边界。

        这样做的原因：十字路口处画面里会有**别的车道段**（横臂）。
        如果简单地取"整行最左/最右"，会把横臂也算进来，得到错误的宽度。
        从中线向两侧扩展只取"包含中线的那一段"，天然正确。
        """
        cfg = self.config["lane"]
        height, width = mask.shape[:2]
        center_x = width // 2
        rows = np.linspace(int(height * float(cfg["roi_top_ratio"])),
                           height - 1, int(cfg["scan_rows"])).astype(int)

        edges = []
        for y in rows:
            row = mask[y]
            if not row[center_x]:
                continue
            left = center_x
            while left > 0 and row[left - 1]:
                left -= 1
            right = center_x
            while right < width - 1 and row[right + 1]:
                right += 1
            span = right - left
            if not (float(cfg["min_lane_width_px"]) <= span
                    <= float(cfg["max_lane_width_px"])):
                continue
            edges.append(LaneX(int(y), int(left), int(right)))
        return tuple(edges)

    def _lane_geometry(self, edges, width_px, height_px, mm_per_px):
        """算横向偏差与方向角。返回 (offset_mm, heading_deg, near, far)。"""
        if len(edges) < 3:
            return 0.0, 0.0, None, None
        cfg = self.config["geometry"]

        def pick(ratio):
            # 找离目标行最近的一条
            target = height_px * float(ratio)
            return min(edges, key=lambda e: abs(e.y - target))

        near = pick(cfg["near_row_ratio"])
        far = pick(cfg["far_row_ratio"])

        near_mid = (near.left_x + near.right_x) / 2.0
        # 正 = 车偏右 → 状态机应左移
        offset_px = near_mid - width_px / 2.0
        offset_mm = offset_px * mm_per_px

        heading_deg = 0.0
        if far.y != near.y:
            dx = ((far.left_x + far.right_x) / 2.0) - near_mid
            dy = near.y - far.y                     # 画面 y 向下，取正
            if dy > 0:
                # 车道中线"往上"的方向与画面竖直方向的夹角
                heading_deg = math.degrees(math.atan2(dx, dy))
        return offset_mm, heading_deg, near, far

    # ---------------- 障碍 ----------------

    def _black_mask(self, frame, cfg) -> np.ndarray:
        """黑色 mask，同样只算下方 ROI（更远的障碍在这个姿态下也够不着）。"""
        top = int(frame.shape[0] * float(self.config["lane"]["roi_top_ratio"]))
        roi = frame[top:, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower = np.array([0, 0, 0], dtype=np.uint8)
        upper = np.array([179, int(cfg["max_saturation"]),
                          int(cfg["max_value"])], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        full = np.zeros((frame.shape[0], frame.shape[1]), dtype=np.uint8)
        full[top:, :] = mask
        return full

    def _find_obstacles(self, mask, lane_edges, mm_per_px, height_px):
        cfg = self.config["obstacle"]
        rng = self.config["range"]
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        horizon_y = height_px * float(rng["horizon_y_ratio"])
        found = []

        for contour in contours:
            area = float(cv2.contourArea(contour))
            if not (float(cfg["min_area_px"]) <= area <= float(cfg["max_area_px"])):
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if h <= 0 or w <= 0:
                continue
            if float(w) / float(h) > float(cfg["max_aspect_ratio"]) and \
               float(h) / float(w) > float(cfg["max_aspect_ratio"]):
                continue
            hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
            if hull_area <= 0 or area / hull_area < float(cfg["min_solidity"]):
                continue

            moments = cv2.moments(contour)
            if moments["m00"] <= 0:
                continue
            cx = moments["m10"] / moments["m00"]
            cy = moments["m01"] / moments["m00"]
            bottom = y + h

            if bool(cfg["must_be_on_lane"]) and lane_edges:
                near = min(lane_edges, key=lambda e: abs(e.y - bottom))
                margin = float(cfg["on_lane_margin_px"])
                if not (near.left_x - margin <= cx <= near.right_x + margin):
                    continue

            # 距离粗估：底边越低越近
            denom = max(bottom - horizon_y, 1.0)
            distance = float(rng["k_mm_px"]) / denom
            distance = max(float(rng["min_distance_mm"]),
                           min(float(rng["max_distance_mm"]), distance))

            found.append(Obstacle(
                center_x_px=float(cx), center_y_px=float(cy),
                bottom_y_px=float(bottom), width_px=float(w), height_px=float(h),
                width_mm=float(w) * mm_per_px, distance_mm=distance,
                area_px=area, contour=contour,
            ))

        found.sort(key=lambda o: o.bottom_y_px, reverse=True)   # 最近的排前面
        return tuple(found)

    # ---------------- 主流程 ----------------

    def detect(self, frame) -> LaneObstacleDetection:
        started = time.perf_counter()
        if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError("lane detector requires a BGR 3-channel image")

        height_px, width_px = frame.shape[:2]
        geo_cfg = self.config["geometry"]

        lane_mask = self._lane_mask(frame)
        edges = self._scan_lane_edges(lane_mask)

        if len(edges) < 3:
            return LaneObstacleDetection(
                detected=False, lane_found=False, lane_edges=edges,
                processing_ms=(time.perf_counter() - started) * 1000.0,
                detail="no lane found (check gray threshold / cam tilt)",
            )

        # 用"已知车道宽 400mm"反推该行的 mm/px —— 不需要提前标定镜头
        def pick(ratio):
            target = height_px * float(ratio)
            return min(edges, key=lambda e: abs(e.y - target))

        near = pick(geo_cfg["near_row_ratio"])
        near_width_px = float(near.right_x - near.left_x)
        mm_per_px = (float(geo_cfg["lane_width_mm"]) / near_width_px
                     if near_width_px > 1 else 0.0)

        offset_mm, heading_deg, near, far = self._lane_geometry(
            edges, width_px, height_px, mm_per_px)
        far_width_px = float(far.right_x - far.left_x) if far is not None else 0.0

        black_mask = self._black_mask(frame, self.config["obstacle"])
        obstacles = self._find_obstacles(black_mask, edges, mm_per_px, height_px)
        nearest = obstacles[0] if obstacles else None

        # ---- 可通行性 ----
        clear_cfg = self.config["clearance"]
        free_left_px = free_right_px = 0.0
        blocked = False
        action = "GO"
        detail = "lane ok, no obstacle ahead"

        if nearest is not None:
            row = min(edges, key=lambda e: abs(e.y - nearest.bottom_y_px))
            # 用障碍所在那一行自己的尺度：透视下各行的 mm/px 不同，
            # 拿近处的比例去量远处会低估/高估。
            row_width_px = float(row.right_x - row.left_x)
            row_mm_per_px = (float(geo_cfg["lane_width_mm"]) / row_width_px
                             if row_width_px > 1 else 0.0)
            obstacle_left = nearest.center_x_px - nearest.width_px / 2.0
            obstacle_right = nearest.center_x_px + nearest.width_px / 2.0
            free_left_px = max(0.0, float(obstacle_left - row.left_x))
            free_right_px = max(0.0, float(row.right_x - obstacle_right))
            free_left_mm = free_left_px * row_mm_per_px
            free_right_mm = free_right_px * row_mm_per_px

            need_mm = float(clear_cfg["vehicle_width_mm"]) + \
                float(clear_cfg["extra_margin_mm"])
            in_danger = nearest.bottom_y_px >= (
                height_px * float(clear_cfg["danger_zone_row_ratio"]))

            if free_left_mm >= need_mm and free_left_mm >= free_right_mm:
                action = "GO"
                detail = f"obstacle ahead, left has {free_left_mm:.0f}mm -> strafe left"
            elif free_right_mm >= need_mm:
                action = "GO"
                detail = f"obstacle ahead, right has {free_right_mm:.0f}mm -> strafe right"
            elif in_danger:
                blocked = True
                action = ("BLOCKED_LEFT" if free_left_mm > free_right_mm
                          else "BLOCKED_RIGHT")
                detail = (f"blocked at {nearest.distance_mm:.0f}mm: "
                          f"L {free_left_mm:.0f}mm / R {free_right_mm:.0f}mm, "
                          f"need {need_mm:.0f}mm -> replan")
            else:
                detail = (f"obstacle far ahead ({nearest.distance_mm:.0f}mm), "
                          f"no action yet")
        else:
            free_left_mm = free_right_mm = 0.0

        if abs(offset_mm) > float(geo_cfg["center_offset_warn_mm"]) and not blocked:
            detail += f"; lateral offset {offset_mm:+.0f}mm needs correction"

        return LaneObstacleDetection(
            detected=True,
            lane_found=True,
            lane_edges=edges,
            offset_mm=offset_mm,
            heading_deg=heading_deg,
            width_px_near=near_width_px,
            width_px_far=far_width_px,
            mm_per_px_near=mm_per_px,
            far_near_ratio=(far_width_px / near_width_px
                            if near_width_px > 0 else 0.0),
            obstacles=obstacles,
            nearest=nearest,
            free_left_px=free_left_px,
            free_right_px=free_right_px,
            free_left_mm=free_left_mm,
            free_right_mm=free_right_mm,
            blocked=blocked,
            action=action,
            processing_ms=(time.perf_counter() - started) * 1000.0,
            detail=detail,
        )


# ==================== 绘制 ====================

ACTION_COLOR = {
    "GO": (80, 220, 80),
    "BLOCKED_LEFT": (60, 170, 255),
    "BLOCKED_RIGHT": (255, 170, 60),
    "BLOCKED": (60, 60, 255),
    "STOP": (200, 200, 200),
}


def draw_detection(frame, result: LaneObstacleDetection, show_mask=False):
    """画调试图：左/右边界（红）、中线（青）、角度与偏差（绿）、障碍框。"""

    out = frame
    height, width = out.shape[:2]

    # 车道边界 + 中线
    for edge in result.lane_edges:
        cv2.circle(out, (edge.left_x, edge.y), 2, (0, 0, 255), -1, cv2.LINE_AA)
        cv2.circle(out, (edge.right_x, edge.y), 2, (0, 0, 255), -1, cv2.LINE_AA)

    if len(result.lane_edges) >= 2:
        lefts = np.array([(e.left_x, e.y) for e in result.lane_edges], dtype=np.int32)
        rights = np.array([(e.right_x, e.y) for e in result.lane_edges], dtype=np.int32)
        cv2.polylines(out, [lefts], False, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.polylines(out, [rights], False, (0, 0, 255), 2, cv2.LINE_AA)
        mids = ((lefts + rights) // 2).astype(np.int32)
        cv2.polylines(out, [mids], False, (255, 255, 0), 2, cv2.LINE_AA)

    # 障碍
    for index, obs in enumerate(result.obstacles):
        box = (int(obs.center_x_px - obs.width_px / 2),
               int(obs.bottom_y_px - obs.height_px),
               int(obs.width_px), int(obs.height_px))
        cv2.rectangle(out, (box[0], box[1]),
                      (box[0] + box[2], box[1] + box[3]), (0, 0, 255), 2)
        cv2.putText(out, f"O{index + 1} {obs.distance_mm:.0f}mm",
                    (box[0], max(16, box[1] - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 0, 255), 2, cv2.LINE_AA)

    # 顶部信息条
    color = ACTION_COLOR.get(result.action, (255, 255, 255))
    lines = [
        f"action={result.action}  lane={int(result.lane_found)}  "
        f"mm/px={result.mm_per_px_near:.3f}",
        f"offset={result.offset_mm:+.1f}mm  heading={result.heading_deg:+.1f}deg  "
        f"free L/R={result.free_left_mm:.0f}/{result.free_right_mm:.0f}mm",
        result.detail,
    ]
    cv2.rectangle(out, (8, 8), (min(width - 8, 980), 104), (0, 0, 0), -1)
    for index, text in enumerate(lines):
        cv2.putText(out, text, (18, 36 + index * 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.62, color if index == 0 else (235, 235, 235), 2, cv2.LINE_AA)
    return out


def draw_masks(frame_shape, detector: LaneObstacleDetector, frame):
    """把两张 mask 并排画出来，方便调阈值。"""
    lane_mask = detector._lane_mask(frame)
    black_mask = detector._black_mask(frame, detector.config["obstacle"])
    lane_bgr = cv2.cvtColor(lane_mask, cv2.COLOR_GRAY2BGR)
    black_bgr = cv2.cvtColor(black_mask, cv2.COLOR_GRAY2BGR)
    # 车道 mask 显示成绿色：BGR 里清零 B(0) 和 R(2)，只留 G(1)
    lane_bgr[:, :, 0] = 0
    lane_bgr[:, :, 2] = 0
    # 障碍 mask 显示成蓝色：清零 G(1) 和 R(2)，只留 B(0)
    black_bgr[:, :, 1] = 0
    black_bgr[:, :, 2] = 0
    return np.hstack([lane_bgr, black_bgr])


# ==================== 现场调参入口 ====================

def main():
    import argparse

    parser = argparse.ArgumentParser(description="行车视觉：车道 + 黑色障碍（现场调参）")
    parser.add_argument("--device", default="/dev/video2")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--config", default=None, help="参数 JSON，缺省用内置默认")
    parser.add_argument("--show", action="store_true", help="显示实时调试图")
    parser.add_argument("--show-mask", action="store_true", help="同时显示两张 mask")
    parser.add_argument("--dump", default=None, help="存一张调试图到该路径后退出")
    parser.add_argument("--json", action="store_true", help="每帧输出一行 JSON")
    args = parser.parse_args()

    document = load_config(args.config) if args.config else build_default_document()
    detector = LaneObstacleDetector(document)

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit(f"打不开相机：{args.device}")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            print("camera read failed")
            break
        result = detector.detect(frame)

        if args.json:
            print(json.dumps({
                "action": result.action,
                "lane_found": result.lane_found,
                "offset_mm": round(result.offset_mm, 1),
                "heading_deg": round(result.heading_deg, 1),
                "mm_per_px": round(result.mm_per_px_near, 4),
                "width_px_near": round(result.width_px_near, 1),
                "far_near_ratio": round(result.far_near_ratio, 3),
                "obstacles": len(result.obstacles),
                "nearest_mm": round(result.nearest.distance_mm, 0) if result.nearest else None,
                "free_left_mm": round(result.free_left_mm, 0),
                "free_right_mm": round(result.free_right_mm, 0),
                "ms": round(result.processing_ms, 1),
                "detail": result.detail,
            }, ensure_ascii=False), flush=True)

        vis = draw_detection(frame, result)
        if args.show_mask:
            vis = np.vstack([vis, draw_masks(frame.shape, detector, frame)])

        if args.dump:
            cv2.imwrite(args.dump, vis)
            print(f"已保存调试图：{args.dump}")
            break

        if args.show:
            cv2.imshow("lane_obstacle", vis)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
        elif not args.json:
            print(f"{result.action:14s} offset={result.offset_mm:+7.1f}mm "
                  f"lane={result.lane_width_mm_est:6.1f}mm "
                  f"obs={len(result.obstacles)} {result.detail}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
