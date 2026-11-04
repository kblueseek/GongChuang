#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""圆环工位视觉对位 + 里程计重置节点（动作序列 `calibrate` 指令的执行者）。

================ 大白话：这个节点在干什么 ================

上位机动作序列里新增的 `calibrate {"x_m":..,"y_m":..}` 指令，由 motion_planner
转发到本节点完成：

  1. 收到 /ring_fix/target（中间圆环的里程计坐标 x, y）后打开摄像头；
  2. 逐帧跑 RingStationDetector（vision/ring_station_detector.py）识别三个圆环；
  3. 检测器给出"底盘坐标系下的纠偏量"（毫米/度，来自标定的 3x3 纠偏矩阵），
     把它裁成小步（默认 ≤40mm / ≤4°），以"当前里程计位姿 ⊕ 小步"为目标发
     /cmd_pose，让 F407 的位置闭环一步步把中间环对到标定基准（参考图像配置）；
  4. 连续若干帧纠偏量都小于门槛 → 认为对准。此时车体相对中间圆环的位置是
     标定好的固定值，于是：
       车体真实位姿 = (圆环坐标 + align_offset) [+ 可选的 align_yaw 航向]
     先发 /set_odom 让 f407_bridge 把里程计偏移到该真实位姿（清除累计误差），
     再发 /ring_fix/result 通知 motion_planner 同步它的目标位姿。

坐标系注意：/set_odom 之后 ROS 侧所有位姿（/odom、/cmd_pose）都活在"修正后的
里程计系"里——f407_bridge 会在 /odom 上报时加偏移、在 /cmd_pose 下发时减偏移，
F407 内部始终用它自己的原始里程计系，两边自动换算。

================ 使用前提 ================

- 触发前云台和摄像头已摆到标定参考姿态（使用者保证）；
- vision/main.py 等程序不能占用摄像头（本节点只在触发时打开摄像头、做完即释放）；
- vision/config/ring_station_detector.json 里的参考圆心 / 纠偏矩阵
  （correction_calibration.chassis_image_to_motion）必须已标定，否则拒绝执行；
- align_offset_x_m / align_offset_y_m 是一次性标定常数：把车间动到"对准"位置，
  读 odom、减去圆环 odom 坐标，填进启动参数。
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys
import time

import rclpy
from rclpy.node import Node
from car_interfaces.msg import Pose2D
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def _wrap_angle(a: float) -> float:
    """角度归一化到 (-π, π]。"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _find_vision_dir() -> str:
    """从本文件向上找包含 ring_station_detector.py 的 vision/ 目录。

    colcon --symlink-install 时 __file__ 解析到源码树，vision/ 与 ros2_ws/
    同级；找不到就返回空串，由调用方报配置错误。
    """
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / 'vision' / 'ring_station_detector.py').is_file():
            return str(parent / 'vision')
    return ''


class RingFixNode(Node):
    def __init__(self):
        super().__init__('ring_fix')

        # ---- 摄像头 ----
        self.declare_parameter('camera_device', '/dev/video0')
        self.declare_parameter('camera_width', 1280)
        self.declare_parameter('camera_height', 720)
        self.declare_parameter('camera_fps', 30.0)
        # ---- 视觉配置目录（空 = 自动从源码树向上找 vision/） ----
        self.declare_parameter('vision_dir', '')
        # ---- 对准后车体相对中间圆环的偏移（里程计系，米）。一次性标定： ----
        # 对准后 车体odom − 圆环odom。默认 (0,0) = 直接认为"对准位置就是圆环坐标"。
        self.declare_parameter('align_offset_x_m', 0.0)
        self.declare_parameter('align_offset_y_m', 0.0)
        # 对准后车体的航向（度）。留空 = 只修正位置、不修正航向。
        self.declare_parameter('align_yaw_deg', '')
        # ---- 伺服参数 ----
        self.declare_parameter('step_max_xy_m', 0.04)      # 单步最大平移（米）
        self.declare_parameter('step_max_yaw_deg', 4.0)    # 单步最大转向（度）
        self.declare_parameter('step_interval_s', 0.30)    # 相邻两步的最小间隔（秒）
        self.declare_parameter('converge_tol_xy_mm', 4.0)  # 收敛判定：位置（毫米）
        self.declare_parameter('converge_tol_yaw_deg', 1.5)  # 收敛判定：航向（度）
        self.declare_parameter('converge_hold_frames', 3)  # 连续多少帧满足才算对准
        self.declare_parameter('timeout_s', 45.0)          # 单次矫正总超时（秒）

        self._device = str(self.get_parameter('camera_device').value)
        self._width = int(self.get_parameter('camera_width').value)
        self._height = int(self.get_parameter('camera_height').value)
        self._fps = float(self.get_parameter('camera_fps').value)
        self._off_x = float(self.get_parameter('align_offset_x_m').value)
        self._off_y = float(self.get_parameter('align_offset_y_m').value)
        align_yaw_raw = str(self.get_parameter('align_yaw_deg').value).strip()
        self._align_yaw = (math.radians(float(align_yaw_raw))
                           if align_yaw_raw else None)
        self._step_max_xy = abs(float(self.get_parameter('step_max_xy_m').value))
        self._step_max_yaw = abs(float(self.get_parameter('step_max_yaw_deg').value))
        self._step_interval = float(self.get_parameter('step_interval_s').value)
        self._tol_xy = abs(float(self.get_parameter('converge_tol_xy_mm').value))
        self._tol_yaw = abs(float(self.get_parameter('converge_tol_yaw_deg').value))
        self._hold_need = max(1, int(self.get_parameter('converge_hold_frames').value))
        self._timeout = float(self.get_parameter('timeout_s').value)

        # ---- 加载视觉配置（import vision 的检测器） ----
        vision_dir = str(self.get_parameter('vision_dir').value).strip()
        if not vision_dir:
            vision_dir = _find_vision_dir()
        if not vision_dir or not (Path(vision_dir) / 'ring_station_detector.py').is_file():
            raise RuntimeError(
                f'找不到 vision 目录（当前={vision_dir or "空"}）：'
                '请用 vision_dir 参数指定 ring_station_detector.py 所在目录')
        sys.path.insert(0, vision_dir)
        try:
            import cv2  # noqa: E402
        except ImportError as error:
            raise RuntimeError(
                'ring_fix 需要 OpenCV（python3-opencv）才能采集摄像头画面') from error
        self._cv2 = cv2
        from ring_station_detector import (  # noqa: E402
            load_ring_station_config, RingStationDetector,
            RingStationDetectionError,
        )
        self._RingStationDetectionError = RingStationDetectionError
        vision = Path(vision_dir)
        self._ring_config = load_ring_station_config(
            vision / 'config' / 'ring_station_detector.json')
        if self._ring_config.chassis_image_to_motion is None:
            raise RuntimeError(
                'ring_station_detector.json 缺少 correction_calibration.'
                'chassis_image_to_motion 纠偏矩阵，无法计算底盘纠偏量')
        # 摄像头畸变矫正（和 vision/main.py 同一套配置）
        self._map1, self._map2 = self._build_undistort_maps(vision)

        # ---- 话题 ----
        self._pub_pose = self.create_publisher(Pose2D, '/cmd_pose', 10)
        self._pub_set_odom = self.create_publisher(Pose2D, '/set_odom', 10)
        self._pub_result = self.create_publisher(String, '/ring_fix/result', 10)
        self.create_subscription(
            Pose2D, '/ring_fix/target', self._cb_target, 10)
        self.create_subscription(
            Odometry, '/odom', self._cb_odom, 10)

        # ---- 运行状态 ----
        self._odom = None            # 最新里程计 (x, y, yaw)
        self._odom_t = 0.0           # 最近一次收到里程计的时间
        self._phase = 'idle'         # idle / starting / servo
        self._ring_target = None     # (x, y) 中间圆环的里程计坐标
        self._detector = None        # RingStationDetector（每次触发新建）
        self._camera = None          # cv2.VideoCapture（触发时打开、结束时释放）
        self._start_t = 0.0
        self._cam_retry_t = 0.0
        self._hold_frames = 0
        self._last_step_t = 0.0
        self._hold_pose = None       # 当前保持的位姿目标 (x, y, yaw)
        self._last_diag_t = 0.0

        self._timer = self.create_timer(0.05, self._tick)   # 20Hz
        self.get_logger().info('ring_fix 就绪：等待 /ring_fix/target（calibrate 指令）')

    # ---------------- 话题回调 ----------------

    def _cb_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self._odom_t = time.monotonic()

    def _cb_target(self, msg: Pose2D):
        """motion_planner 触发一次圆环矫正。"""
        if self._phase != 'idle':
            self.get_logger().warning(
                f'已有矫正在进行（{self._phase}），新目标 ({msg.x:.3f},{msg.y:.3f}) 忽略')
            return
        self._ring_target = (float(msg.x), float(msg.y))
        self._detector = RingStationDetector(self._ring_config)
        self._release_camera()
        self._phase = 'starting'
        self._start_t = time.monotonic()
        self._cam_retry_t = 0.0
        self._hold_frames = 0
        self._last_step_t = 0.0
        self._hold_pose = None
        self.get_logger().info(
            f'开始圆环矫正：中间环里程计 = ({msg.x:.3f}, {msg.y:.3f}) m')

    # ---------------- 主循环 ----------------

    def _tick(self):
        if self._phase == 'idle':
            return
        now = time.monotonic()
        if now - self._start_t > self._timeout:
            self._fail(f'超时（{self._timeout:.0f}s 未对准）')
            return

        if self._phase == 'starting':
            self._try_open_camera(now)
            return

        # phase == 'servo'
        frame = self._read_frame()
        if frame is None:
            return
        if self._map1 is not None:
            frame = self._cv2.remap(
                frame, self._map1, self._map2, self._cv2.INTER_LINEAR)
        try:
            result = self._detector.detect(frame)
        except self._RingStationDetectionError as error:
            self._fail(f'圆环检测异常：{error}')
            return

        usable = (result.detected and result.stable
                  and len(result.centers_px) == 3
                  and result.correction_body_x_mm is not None)
        if usable:
            cx_mm = result.correction_body_x_mm
            cy_mm = result.correction_body_y_mm
            cyaw = result.correction_yaw_deg
            # 收敛判定：纠偏量持续小于门槛（不依赖 within_tolerance 的
            # 严格验收门槛，避免抖动导致永远等不到稳定窗口）。
            if (max(abs(cx_mm), abs(cy_mm)) <= self._tol_xy
                    and abs(cyaw) <= self._tol_yaw):
                self._hold_frames += 1
            else:
                self._hold_frames = 0
            if self._hold_frames >= self._hold_need:
                self._converge()
                return
            self._maybe_step(now, cx_mm, cy_mm, cyaw)
        else:
            self._hold_frames = 0

        # F407 的位姿目标约 100ms 过期：每拍重发保持位姿，别让它散开。
        if self._hold_pose is not None:
            self._publish_pose(*self._hold_pose)

        if now - self._last_diag_t >= 1.0:
            self._last_diag_t = now
            detail = result.detail if result.detected else '未检测到三个圆环'
            err = (f'纠偏=({result.correction_body_x_mm:+.1f},'
                   f'{result.correction_body_y_mm:+.1f}mm '
                   f'{result.correction_yaw_deg:+.2f}°)'
                   if usable else '--')
            self.get_logger().info(
                f'矫正中 [{self._phase}] {detail} {err} '
                f'稳定帧 {self._hold_frames}/{self._hold_need}')

    # ---------------- 伺服 ----------------

    def _maybe_step(self, now: float, cx_mm: float, cy_mm: float, cyaw_deg: float):
        """把纠偏量裁成小步，以"当前里程计 ⊕ 小步"为目标发 /cmd_pose。"""
        if now - self._last_step_t < self._step_interval:
            return
        if (self._odom is None
                or now - self._odom_t > 0.5):
            return   # 没有新鲜里程计不动车，超时兜底

        dist = math.hypot(cx_mm, cy_mm)
        if dist > self._step_max_xy * 1000.0:
            scale = self._step_max_xy * 1000.0 / dist
            cx_mm *= scale
            cy_mm *= scale
        cyaw = max(-self._step_max_yaw, min(self._step_max_yaw, cyaw_deg))
        if max(abs(cx_mm), abs(cy_mm)) < 0.5 and abs(cyaw) < 0.05:
            return   # 步子小到没意义，等收敛判定

        ox, oy, oyaw = self._odom
        c, s = math.cos(oyaw), math.sin(oyaw)
        tx = ox + (cx_mm * c - cy_mm * s) / 1000.0
        ty = oy + (cx_mm * s + cy_mm * c) / 1000.0
        tyaw = oyaw + math.radians(cyaw)
        self._publish_pose(tx, ty, tyaw)
        self._hold_pose = (tx, ty, tyaw)
        self._last_step_t = now

    def _publish_pose(self, x: float, y: float, yaw: float):
        p = Pose2D()
        p.x = float(x)
        p.y = float(y)
        p.theta = float(yaw)
        self._pub_pose.publish(p)

    # ---------------- 结束路径 ----------------

    def _converge(self):
        rx, ry = self._ring_target
        cx = rx + self._off_x
        cy = ry + self._off_y
        # 对准后车体的真实航向：标定常数优先；没配就用当前里程计航向（只修位置）。
        if self._align_yaw is not None:
            cyaw = self._align_yaw
        elif self._odom is not None:
            cyaw = self._odom[2]
        else:
            cyaw = 0.0
        self._pub_set_odom.publish(Pose2D(x=cx, y=cy, theta=cyaw))
        self.get_logger().info(
            f'圆环对准：里程计重置到 ({cx:.3f}, {cy:.3f}) m '
            f'yaw={math.degrees(cyaw):+.1f}°')
        self._finish({'ok': True, 'x_m': cx, 'y_m': cy, 'yaw_rad': cyaw})

    def _fail(self, reason: str):
        self.get_logger().warning(f'圆环矫正失败：{reason}')
        self._finish({'ok': False, 'reason': reason})

    def _finish(self, payload: dict):
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._pub_result.publish(msg)
        self._phase = 'idle'
        self._hold_pose = None
        self._release_camera()

    # ---------------- 摄像头 ----------------

    def _try_open_camera(self, now: float):
        if now < self._cam_retry_t:
            return
        try:
            camera = self._open_camera()
        except Exception as error:  # noqa: BLE001 - 打不开就按间隔重试到超时
            self.get_logger().warning(
                f'摄像头打不开（{self._device}）：{error}')
            self._cam_retry_t = now + 1.0
            return
        self._camera = camera
        self._phase = 'servo'
        self.get_logger().info('摄像头已打开，开始识别圆环')

    def _read_frame(self):
        try:
            ok, frame = self._camera.read()
        except Exception as error:  # noqa: BLE001
            self.get_logger().warning(f'摄像头读取失败：{error}')
            self._release_camera()
            self._phase = 'starting'
            self._cam_retry_t = time.monotonic() + 1.0
            return None
        if not ok or frame is None:
            self.get_logger().warning('摄像头读取失败（无帧），重连')
            self._release_camera()
            self._phase = 'starting'
            self._cam_retry_t = time.monotonic() + 1.0
            return None
        if (frame.shape[1], frame.shape[0]) != (self._width, self._height):
            self.get_logger().warning(
                f'摄像头输出尺寸变为 {frame.shape[1]}x{frame.shape[0]}，重连')
            self._release_camera()
            self._phase = 'starting'
            self._cam_retry_t = time.monotonic() + 1.0
            return None
        return frame

    def _release_camera(self):
        if self._camera is not None:
            try:
                self._camera.release()
            except Exception:  # noqa: BLE001
                pass
        self._camera = None

    def _open_camera(self):
        """按 vision/main.py 的同一套方式打开摄像头（V4L2 + MJPG）。"""
        cv2 = self._cv2
        camera = cv2.VideoCapture(self._device, cv2.CAP_V4L2)
        if not camera.isOpened():
            raise RuntimeError('VideoCapture 打不开')
        camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        camera.set(cv2.CAP_PROP_FPS, self._fps)
        camera.set(cv2.CAP_PROP_BUFFERSIZE, 4)
        # 预热：自动曝光稳定 + 清掉启动时的坏帧
        for _ in range(10):
            ok, frame = camera.read()
            if ok and frame is not None:
                break
        else:
            camera.release()
            raise RuntimeError('摄像头已打开但读不到画面')
        if (frame.shape[1], frame.shape[0]) != (self._width, self._height):
            camera.release()
            raise RuntimeError(
                f'请求 {self._width}x{self._height}，实际 '
                f'{frame.shape[1]}x{frame.shape[0]}')
        return camera

    def _build_undistort_maps(self, vision: Path):
        """读 camera_processing.json + 相机标定 yaml，生成畸变矫正映射。

        关闭矫正或配置缺失时返回 (None, None)（用原始画面，和 vision/main.py
        行为一致；标定数据缺失只警告，不阻止节点启动）。
        """
        cv2 = self._cv2
        try:
            proc = json.loads((vision / 'config' / 'camera_processing.json')
                              .read_text(encoding='utf-8'))
            if type(proc.get('undistort_enabled')) is not bool:
                raise ValueError('undistort_enabled 必须是布尔值')
            if not proc['undistort_enabled']:
                return None, None
            storage = cv2.FileStorage(
                str(vision / 'config' / 'realtek_rgb_camera.yaml'),
                cv2.FILE_STORAGE_READ)
            if not storage.isOpened():
                raise ValueError('标定文件打不开')
            size = (int(storage.getNode('image_width').real()),
                    int(storage.getNode('image_height').real()))
            if size != (self._width, self._height):
                raise ValueError(
                    f'标定适用于 {size[0]}x{size[1]}，与运行分辨率 '
                    f'{self._width}x{self._height} 不一致')
            camera_matrix = storage.getNode('camera_matrix').mat()
            distortion = storage.getNode('distortion_coefficients').mat()
            new_matrix = storage.getNode('new_camera_matrix').mat()
            storage.release()
            if camera_matrix is None or distortion is None or new_matrix is None:
                raise ValueError('标定文件缺矩阵')
            map1, map2 = cv2.initUndistortRectifyMap(
                camera_matrix, distortion, None, new_matrix,
                size, cv2.CV_16SC2)
            self.get_logger().info('畸变矫正：开')
            return map1, map2
        except Exception as error:  # noqa: BLE001
            self.get_logger().warning(f'畸变矫正配置无效（{error}），用原始画面')
            return None, None

    def destroy_node(self):
        self._release_camera()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RingFixNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
