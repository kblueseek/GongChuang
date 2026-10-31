#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""视觉 demo：单相机识别物料，返回像素坐标。

只做一件事：开一路相机，用 vision/config/material_color_thresholds.json 的 HSV
阈值把「六种颜色里任意一种」的像素抓出来，找面积最大的连通域，把它的质心当成
物料中心发到 /material_target。不做分类、不做转盘逻辑、不发任何运动指令。

话题
----
发布：
    /material_target   std_msgs/Float32MultiArray
                       [detected, cx_px, cy_px, img_w, img_h, color_id]
                         detected = 1.0 有目标 / 0.0 没看到；
                         cx/cy 是物料质心在当前帧的像素坐标（左上角为原点）；
                         img_w/img_h 是当前帧实际宽高（供跟随节点算误差中心）；
                         color_id 是被选中物料的颜色编号（1~6，0 表示无）。

参数
----
  device          相机设备，留空自动试 /dev/video0..7
  width/height    期望分辨率（默认 1280x720，相机不支持会回落到实际值）
  fps             帧率
  rate_hz         发布频率
  target_color    只识别某一种颜色（1~6），0 = 六种任意
  exclude_colors  逗号分隔的颜色号不参与识别（默认 '5' = 排除黑色）
  vision_dir      vision/ 目录（含 颜色标定.py），留空自动找
  debug_dump      true = 每 2 秒存一张画面到 debug_dump_dir
  show_window     true = 开一个实时预览窗口，画出物料轮廓（需图形桌面）

识别筛选（逐色处理，形状筛选默认关闭 = 值 0，避免把真物料误杀）：
  processing_scale 识别前缩放倍数（0.5 更快更稳，1.0 全分辨率）
  open_kernel_px / close_kernel_px   开/闭运算核大小（奇数）
  min_area_px / max_area_px          物料连通域面积范围（max 设 0 = 不设上限）
  min_dimension_px / max_dimension_px 外接框短/长边范围（max 设 0 = 不设上限）
  max_aspect_ratio   长宽比上限，0 = 不查（圆柱斜看会变长，建议保持 0）
  min_solidity       凸包实心度下限，0 = 不查
  min_extent         外接框占比下限，0 = 不查
  ⚠️ 贴边被画面裁掉的轮廓会自动跳过形状筛选（只按颜色+最小面积认），
     避免物料靠近画面边缘时被误判为丢目标。
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


def resolve_vision_dir(param_value: str) -> Path:
    """定位 vision/ 目录（含 颜色标定.py 的那层），找不到就报错。

    源码运行和 colcon 安装后 __file__ 路径不同（src/... vs install/...），所以
    不数固定层数，而是沿着文件所在目录一路向上找「含 颜色标定.py 的 vision/」。
    """
    candidates = []
    if param_value:
        candidates.append(Path(param_value).expanduser())

    here = Path(__file__).resolve()
    # 从本文件目录逐级向上，找祖先里有没有 vision/颜色标定.py
    for parent in (here, *here.parents):
        candidate = parent / "vision"
        if (candidate / "颜色标定.py").is_file():
            return candidate.resolve()

    candidates.append(Path.cwd() / "vision")
    candidates.append(Path("/home/kbluepi/GongChuang/raspberrypi/vision"))
    for candidate in candidates:
        if (candidate / "颜色标定.py").is_file():
            return candidate.resolve()

    raise FileNotFoundError(
        "找不到 vision/ 目录（应含 颜色标定.py）。已尝试："
        f"{[str(c) for c in candidates]}。请用参数 vision_dir 指定。"
    )


def open_camera(device, width, height, fps):
    """打开一路相机，返回 (cap, path, (w, h))。device 留空则从 /dev/video0 依次试。"""
    if device:
        paths = [device]
    else:
        paths = [f"/dev/video{i}" for i in range(8)]

    for path in paths:
        cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            continue
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 4)
        for _ in range(20):
            ok, frame = cap.read()
            if ok and frame is not None:
                return cap, path, (frame.shape[1], frame.shape[0])
        cap.release()
    raise RuntimeError(f"打不开相机：{paths}")


class MaterialDetectNode(Node):
    def __init__(self):
        super().__init__('material_detect')

        self.declare_parameter('device', '')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('target_color', 0)     # 0=任意颜色，1~6=指定
        self.declare_parameter('exclude_colors', '5')  # 逗号分隔，这些颜色号不参与识别（默认排除黑色）
        self.declare_parameter('vision_dir', '')
        self.declare_parameter('debug_dump', False)
        self.declare_parameter('debug_dump_dir', '/tmp/gimbal_track')
        self.declare_parameter('show_window', True)   # 实时预览：画面 + 检测轮廓

        # —— 形状筛选参数（移植自 material_pickup_detector.py 的 detection 段）——
        self.declare_parameter('processing_scale', 0.5)
        self.declare_parameter('open_kernel_px', 5)
        self.declare_parameter('close_kernel_px', 11)
        self.declare_parameter('min_area_px', 300.0)
        self.declare_parameter('max_area_px', 0.0)      # 0 = 不设上限
        self.declare_parameter('min_dimension_px', 10.0)
        self.declare_parameter('max_dimension_px', 0.0)  # 0 = 不设上限
        self.declare_parameter('max_aspect_ratio', 0.0)  # 0 = 不查长宽比
        self.declare_parameter('min_solidity', 0.0)      # 0 = 不查实心度
        self.declare_parameter('min_extent', 0.0)        # 0 = 不查占比

        self._target_color = int(self.get_parameter('target_color').value)
        self._exclude = set()
        for token in str(self.get_parameter('exclude_colors').value).split(','):
            token = token.strip()
            if token:
                self._exclude.add(int(token))
        self._scale = float(self.get_parameter('processing_scale').value)
        self._open_kernel = int(self.get_parameter('open_kernel_px').value)
        self._close_kernel = int(self.get_parameter('close_kernel_px').value)
        self._min_area = float(self.get_parameter('min_area_px').value)
        self._max_area = float(self.get_parameter('max_area_px').value)
        self._min_dim = float(self.get_parameter('min_dimension_px').value)
        self._max_dim = float(self.get_parameter('max_dimension_px').value)
        self._max_aspect = float(self.get_parameter('max_aspect_ratio').value)
        self._min_solidity = float(self.get_parameter('min_solidity').value)
        self._min_extent = float(self.get_parameter('min_extent').value)
        self._debug_dump = bool(self.get_parameter('debug_dump').value)
        self._debug_dir = Path(self.get_parameter('debug_dump_dir').value)
        self._last_dump_s = 0.0
        self._show_window = bool(self.get_parameter('show_window').value)
        self._preview_warned = False

        vision_dir = resolve_vision_dir(self.get_parameter('vision_dir').value)
        if str(vision_dir) not in sys.path:
            sys.path.insert(0, str(vision_dir))
        # 复用 vision/ 里的颜色配置加载器（只 import cv2/numpy，不碰 ROS，安全）
        from 颜色标定 import load_color_config, make_all_color_masks  # noqa: PLC0415

        self._config = load_color_config(
            vision_dir / "config" / "material_color_thresholds.json"
        )
        self._make_masks = make_all_color_masks
        self._color_names = {
            int(item['color_id']): item['name'] for item in self._config['colors']
        }

        try:
            self._cap, self._path, self._size = open_camera(
                str(self.get_parameter('device').value),
                int(self.get_parameter('width').value),
                int(self.get_parameter('height').value),
                float(self.get_parameter('fps').value),
            )
        except RuntimeError as error:
            self._cap = None
            self.get_logger().error(str(error))
            self.get_logger().error('相机不可用，只发布 detected=0。检查接线 / 参数 device。')
        else:
            self.get_logger().info(f'相机已打开：{self._path} {self._size[0]}x{self._size[1]}')

        self._pub = self.create_publisher(Float32MultiArray, '/material_target', 10)
        rate = max(1.0, float(self.get_parameter('rate_hz').value))
        self._timer = self.create_timer(1.0 / rate, self._on_tick)

        self.get_logger().info(
            f'material_detect 就绪：target_color={self._target_color}'
            f'（0=任意）vision/ 目录={vision_dir}'
        )

    @staticmethod
    def _scaled_kernel(full_size: int, scale: float) -> int:
        """核大小按缩放倍率取整，保持正奇数。"""
        value = max(1, int(round(full_size * scale)))
        return value if value % 2 else value + 1

    def _detect(self, frame):
        """逐色识别 + 形状筛选，返回 (cx, cy, w, h, color_id, contour) 或 None。

        不再把六色并成一个 mask：而是每种颜色单独开闭运算、单独找轮廓，
        再用面积/尺寸/长宽比/实心度/占比把背景和噪声过滤掉。最终在通过
        筛选的物料里选面积最大的那件（最显著/最靠近镜头）。
        """
        scale = self._scale
        if scale < 1.0:
            working = cv2.resize(
                frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA
            )
        else:
            working = frame
        masks = self._make_masks(working, self._config)

        open_size = self._scaled_kernel(self._open_kernel, scale)
        close_size = self._scaled_kernel(self._close_kernel, scale)
        open_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (open_size, open_size)
        )
        close_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (close_size, close_size)
        )

        scale_area = scale * scale
        candidates = []
        for color_id, raw_mask in masks.items():
            if self._target_color and int(color_id) != self._target_color:
                continue
            if int(color_id) in self._exclude:
                continue
            mask = cv2.morphologyEx(raw_mask, cv2.MORPH_OPEN, open_kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            for contour in contours:
                work_area = float(cv2.contourArea(contour))
                area = work_area / scale_area
                if area < self._min_area:
                    continue
                bx, by, bw, bh = cv2.boundingRect(contour)
                full_w, full_h = bw / scale, bh / scale
                # 轮廓贴着画面边缘 = 被裁剪，面积/实心度/长宽比都会失真，
                # 这时只按「颜色 + 最小面积」认它，跳过形状筛选——否则物料
                # 一靠近画面边缘（某个 yaw/extend 位置）就会被误判为没目标。
                touches_border = (
                    bx <= 0 or by <= 0
                    or bx + bw >= working.shape[1]
                    or by + bh >= working.shape[0]
                )
                if not touches_border:
                    if self._max_area > 0.0 and area > self._max_area:
                        continue
                    if min(full_w, full_h) < self._min_dim:
                        continue
                    if self._max_dim > 0.0 and max(full_w, full_h) > self._max_dim:
                        continue
                    aspect = max(full_w, full_h) / max(1.0, min(full_w, full_h))
                    if self._max_aspect > 0.0 and aspect > self._max_aspect:
                        continue
                    hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
                    if hull_area > 0.0 and self._min_solidity > 0.0:
                        solidity = work_area / hull_area
                        if solidity < self._min_solidity:
                            continue
                    extent = work_area / max(1.0, float(bw * bh))
                    if self._min_extent > 0.0 and extent < self._min_extent:
                        continue
                moments = cv2.moments(contour)
                if abs(moments['m00']) < 1e-6:
                    continue
                cx = (moments['m10'] / moments['m00']) / scale
                cy = (moments['m01'] / moments['m00']) / scale
                # 轮廓坐标换算回全分辨率，供实时预览绘制
                full_contour = np.rint(
                    contour.astype(np.float32) / scale
                ).astype(np.int32)
                candidates.append(
                    (area, cx, cy, full_w, full_h, int(color_id), full_contour)
                )

        if not candidates:
            return None
        # 面积最大的物料当作跟随目标
        _, cx, cy, w, h, color_id, contour = max(
            candidates, key=lambda item: item[0]
        )
        return cx, cy, float(w), float(h), color_id, contour

    def _maybe_dump(self, frame):
        if not self._debug_dump:
            return
        now = time.monotonic()
        if now - self._last_dump_s < 2.0:
            return
        self._last_dump_s = now
        try:
            self._debug_dir.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(self._debug_dir / 'material.jpg'), frame)
        except (OSError, cv2.error) as error:
            self.get_logger().warn(f'调试图写入失败：{error}', throttle_duration_sec=10.0)

    def _on_tick(self):
        w, h = self._size if self._size else (int(self.get_parameter('width').value),
                                              int(self.get_parameter('height').value))
        if self._cap is None:
            self._pub.publish(Float32MultiArray(data=[0.0, 0.0, 0.0, float(w), float(h), 0.0]))
            return

        ok, frame = self._cap.read()
        if not ok or frame is None:
            self.get_logger().warn('相机读帧失败', throttle_duration_sec=2.0)
            self._pub.publish(Float32MultiArray(data=[0.0, 0.0, 0.0, float(w), float(h), 0.0]))
            return

        w, h = frame.shape[1], frame.shape[0]
        self._maybe_dump(frame)

        result = self._detect(frame)
        if result is None:
            self._pub.publish(Float32MultiArray(data=[0.0, 0.0, 0.0, float(w), float(h), 0.0]))
            self._show(frame, None)
            return

        cx, cy, bw, bh, color_id, contour = result
        self._pub.publish(Float32MultiArray(
            data=[1.0, float(cx), float(cy), float(w), float(h), float(color_id)]
        ))
        self._show(frame, result)

    def _show(self, frame, result):
        """实时预览：画面 + 检测物料轮廓。无图形桌面时静默降级（只告警一次）。"""
        if not self._show_window:
            return
        display = frame.copy()
        if result is not None:
            cx, cy, bw, bh, color_id, contour = result
            cv2.drawContours(display, [contour], -1, (0, 255, 0), 2)
            x, y = int(round(cx - bw / 2)), int(round(cy - bh / 2))
            cv2.rectangle(
                display, (x, y), (x + int(bw), y + int(bh)), (0, 255, 0), 1
            )
            cv2.drawMarker(
                display, (int(round(cx)), int(round(cy))),
                (0, 0, 255), cv2.MARKER_CROSS, 40, 3,
            )
            label = f"#{color_id} {self._color_names.get(color_id, '?')}"
            cv2.putText(
                display, label, (x, max(18, y - 5)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA,
            )
        else:
            cv2.putText(
                display, 'no target', (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2, cv2.LINE_AA,
            )
        try:
            cv2.imshow('gimbal_track preview', display)
            cv2.waitKey(1)
        except cv2.error as error:
            if not self._preview_warned:
                self.get_logger().warn(f'预览窗口不可用（无图形桌面？）：{error}')
                self._preview_warned = True

    def shutdown(self):
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        if self._show_window:
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = MaterialDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
