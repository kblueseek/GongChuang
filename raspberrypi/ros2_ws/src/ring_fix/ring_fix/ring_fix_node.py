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

矫正全程在树莓派桌面弹出实时画面（ring_fix 窗口）：蓝十字 = 标定参考环心，
绿/黄椭圆 = 当前识别到的圆环，顶部文字显示纠偏量和稳定帧——用来确认云台/
相机是否摆到了参考姿态、画面里有没有看到三个环。SSH 无桌面环境下自动
关闭预览，不影响矫正本身。参数 show_preview:=false 可禁用。

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
import os
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


class _PID:
    """和 vision/gimbal_track_pick.py 同一套 PID：输出限幅、积分抗饱和、微分低通。

    ki=kd=0 就是纯 P。kd 提供阻尼，能在不加大振荡的前提下提高 kp；ki 消稳态
    误差，但视觉测量有噪声，先给 0~小值。
    """

    def __init__(self, kp, ki, kd, limit, deriv_tau=0.1):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.limit = abs(float(limit))
        self.deriv_tau = max(0.0, float(deriv_tau))
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_err = 0.0
        self.deriv = 0.0

    def update(self, err, dt):
        if dt <= 0.0:
            dt = 0.01
        raw_deriv = (err - self.prev_err) / dt
        self.prev_err = err
        if self.deriv_tau > 0.0:
            alpha = dt / (self.deriv_tau + dt)
            self.deriv += alpha * (raw_deriv - self.deriv)
        else:
            self.deriv = raw_deriv
        p = self.kp * err
        d = self.kd * self.deriv
        pre = p + self.ki * self.integral + d
        saturated = abs(pre) >= self.limit
        if self.ki != 0.0 and not (saturated and pre * err > 0.0):
            self.integral += err * dt
            bound = self.limit / abs(self.ki)
            self.integral = max(-bound, min(bound, self.integral))
        out = p + self.ki * self.integral + d
        return max(-self.limit, min(self.limit, out))


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
        # ---- 伺服：底盘对位 PID（和 vision/config/track_pid.json 一个思路）----
        # 误差单位：x/y = 毫米，yaw = 度；PID 输出是速度（mm/s、°/s），
        # 再按 tick 周期积分成位姿小步发给 F407。
        # 以前是"每 0.3s 直接走 100% 纠偏量"（等效 Kp=1、无阻尼），相机有
        # ~100ms 延迟，所以来回震荡；现在用可调 PID + 速度限幅压住。
        self.declare_parameter('ctrl_kp_xy', 1.0)
        self.declare_parameter('ctrl_ki_xy', 0.02)
        self.declare_parameter('ctrl_kd_xy', 0.15)
        self.declare_parameter('ctrl_kp_yaw', 1.0)
        self.declare_parameter('ctrl_ki_yaw', 0.0)
        self.declare_parameter('ctrl_kd_yaw', 0.15)
        self.declare_parameter('max_xy_speed_m_s', 0.06)    # 平移速度上限
        self.declare_parameter('max_yaw_speed_deg_s', 15.0)  # 转向速度上限
        self.declare_parameter('ctrl_deadband_xy_mm', 0.5)   # 小于这个误差不再输出
        self.declare_parameter('ctrl_deadband_yaw_deg', 0.15)
        self.declare_parameter('ctrl_deriv_tau', 0.1)        # 微分低通（压视觉噪声）
        # 视觉误差一阶低通：检测本身有 ±0.5~1mm 抖动，直接进 PID 会让车
        # 在原地来回蹭。tau 越大越稳、越慢；0 = 不滤波。
        self.declare_parameter('ctrl_filter_tau', 0.15)
        # 滞环（防抖）：误差进到 stop 以内就完全停车、命令"保持当前位姿"；
        # 只有超过 resume（=验收容差）才重新起步。两个值之间是死区。
        self.declare_parameter('ctrl_stop_xy_mm', 1.0)
        self.declare_parameter('ctrl_stop_yaw_deg', 0.35)
        # 停车后先等这么久再开始计收敛帧，让滤波值稳定下来
        self.declare_parameter('ctrl_settle_s', 0.2)
        # F407 内环增益：位置环 Kp（m/s per m）和航向环 Kp（°/s per °）。
        # PID 输出的是速度，要换算成"领先当前位姿多少"的 POSE 目标：
        #   lead_mm = v_mm_s / pos_kp，lead_deg = ω_dps / head_kp
        # 这两个数要和调参页一致（vehicle.yaml 里 tune_profile 的 pos_kp/head_kp），
        # 填大了车会冲、填小了会慢，但视觉外环会兜住。
        self.declare_parameter('chassis_pos_kp', 1.5)
        self.declare_parameter('chassis_heading_kp', 2.4)
        # 单拍位姿领先量上限（防止增益/标定异常时目标飞出去）
        self.declare_parameter('max_lead_xy_mm', 80.0)
        self.declare_parameter('max_lead_yaw_deg', 8.0)
        self.declare_parameter('converge_tol_xy_mm', 2.0)  # 收敛判定：位置（毫米）
        self.declare_parameter('converge_tol_yaw_deg', 0.8)  # 收敛判定：航向（度）
        self.declare_parameter('converge_hold_frames', 5)  # 连续多少帧满足才算对准
        self.declare_parameter('timeout_s', 45.0)          # 单次矫正总超时（秒）
        # false = 只要三个环稳定可见就开始纠偏；置信度/透视比等只提示不拦截。
        self.declare_parameter('require_quality', False)
        # ---- 实时预览 ----
        # true = 矫正时在树莓派桌面弹出识别画面（窗口名 ring_fix）；
        # 无桌面环境（纯 SSH）首次显示失败会自动关掉预览，不影响矫正。
        self.declare_parameter('show_preview', True)

        self._device = str(self.get_parameter('camera_device').value)
        self._width = int(self.get_parameter('camera_width').value)
        self._height = int(self.get_parameter('camera_height').value)
        self._fps = float(self.get_parameter('camera_fps').value)
        self._off_x = float(self.get_parameter('align_offset_x_m').value)
        self._off_y = float(self.get_parameter('align_offset_y_m').value)
        align_yaw_raw = str(self.get_parameter('align_yaw_deg').value).strip()
        self._align_yaw = (math.radians(float(align_yaw_raw))
                           if align_yaw_raw else None)
        self._max_xy_speed = abs(float(
            self.get_parameter('max_xy_speed_m_s').value)) * 1000.0   # mm/s
        self._max_yaw_speed = abs(float(
            self.get_parameter('max_yaw_speed_deg_s').value))         # °/s
        self._deadband_xy = abs(float(
            self.get_parameter('ctrl_deadband_xy_mm').value))
        self._deadband_yaw = abs(float(
            self.get_parameter('ctrl_deadband_yaw_deg').value))
        self._filter_tau = max(0.0, float(
            self.get_parameter('ctrl_filter_tau').value))
        self._stop_xy = abs(float(
            self.get_parameter('ctrl_stop_xy_mm').value))
        self._stop_yaw = abs(float(
            self.get_parameter('ctrl_stop_yaw_deg').value))
        self._settle_s = max(0.0, float(
            self.get_parameter('ctrl_settle_s').value))
        self._pos_kp = max(0.1, abs(float(
            self.get_parameter('chassis_pos_kp').value)))
        self._heading_kp = max(0.1, abs(float(
            self.get_parameter('chassis_heading_kp').value)))
        self._max_lead_xy = abs(float(
            self.get_parameter('max_lead_xy_mm').value))
        self._max_lead_yaw = abs(float(
            self.get_parameter('max_lead_yaw_deg').value))
        self._require_quality = bool(self.get_parameter('require_quality').value)
        deriv_tau = float(self.get_parameter('ctrl_deriv_tau').value)
        self._pid_x = _PID(self.get_parameter('ctrl_kp_xy').value,
                           self.get_parameter('ctrl_ki_xy').value,
                           self.get_parameter('ctrl_kd_xy').value,
                           self._max_xy_speed, deriv_tau)
        self._pid_y = _PID(self.get_parameter('ctrl_kp_xy').value,
                           self.get_parameter('ctrl_ki_xy').value,
                           self.get_parameter('ctrl_kd_xy').value,
                           self._max_xy_speed, deriv_tau)
        self._pid_yaw = _PID(self.get_parameter('ctrl_kp_yaw').value,
                             self.get_parameter('ctrl_ki_yaw').value,
                             self.get_parameter('ctrl_kd_yaw').value,
                             self._max_yaw_speed, deriv_tau)
        self._tol_xy = abs(float(self.get_parameter('converge_tol_xy_mm').value))
        self._tol_yaw = abs(float(self.get_parameter('converge_tol_yaw_deg').value))
        self._hold_need = max(1, int(self.get_parameter('converge_hold_frames').value))
        self._timeout = float(self.get_parameter('timeout_s').value)
        self._show_preview = bool(self.get_parameter('show_preview').value)
        # 没桌面时不能调 cv2.imshow：OpenCV 的 Qt 后端连不上 display 会**直接
        # abort 整个进程**（不是抛 Python 异常，try/except 拦不住）。SSH 启动
        # 整车时在触发 calibrate 那一刻节点就没了，表现就是"点了没反应、也没画面"。
        if (self._show_preview
                and not os.environ.get('DISPLAY')
                and not os.environ.get('WAYLAND_DISPLAY')):
            self._show_preview = False
            self.get_logger().warning(
                '没有 DISPLAY/WAYLAND_DISPLAY：实时预览已自动关闭'
                '（矫正照常执行；要看画面请在树莓派桌面终端里启动）')
        self._preview_active = False   # 预览窗口当前是否开着（含重试降级）

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
        # 这个 import 写在 __init__ 里（要先把 vision_dir 塞进 sys.path），
        # 名字只是局部变量；回调里要用必须挂到 self 上，否则 NameError。
        self._RingStationDetector = RingStationDetector
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
        self._last_ctrl_t = 0.0      # 上一次 PID 控制时刻（算 dt）
        self._last_usable_t = 0.0    # 上一次拿到可用三环的时刻
        self._hold_pose = None       # 当前保持的位姿目标 (x, y, yaw)
        self._filt = None            # 低通后的纠偏量 [x_mm, y_mm, yaw_deg]
        self._filt_t = 0.0
        self._moving = False         # 滞环：是否处于"纠偏中"
        self._anchor_pose = None     # 停车时锁存的位姿目标（保持用）
        self._settle_until = 0.0     # 停车后多少秒内不计收敛帧
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
        self._detector = self._RingStationDetector(self._ring_config)
        self._release_camera()
        self._phase = 'starting'
        self._start_t = time.monotonic()
        self._cam_retry_t = 0.0
        self._hold_frames = 0
        self._last_ctrl_t = 0.0
        self._last_usable_t = 0.0
        self._hold_pose = None
        self._filt = None
        self._filt_t = 0.0
        self._moving = False
        self._anchor_pose = None
        self._settle_until = 0.0
        for pid in (self._pid_x, self._pid_y, self._pid_yaw):
            pid.reset()
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
            result = self._detector.detect(
                frame, require_quality=self._require_quality)
        except self._RingStationDetectionError as error:
            self._fail(f'圆环检测异常：{error}')
            return
        except Exception as error:  # noqa: BLE001
            # rclpy 的定时器回调里抛异常会直接把整个节点进程带走（之后所有
            # calibrate 都石沉大海、也不会有预览）。检测器的偶发错误只跳过这一帧。
            self.get_logger().warning(f'圆环检测异常（本帧跳过）：{error}')
            return
        self._draw_preview(frame, result)

        usable = (result.detected and result.stable
                  and len(result.centers_px) == 3
                  and result.correction_body_x_mm is not None
                  # 必须是"三环共线"那条路径（残差有限）；否则只是零星圆，
                  # 拿它们配对纠偏会把车带偏。
                  and math.isfinite(result.affine_residual))
        cx_mm = cy_mm = cyaw = fx = fy = fyaw = None
        if usable:
            cx_mm = result.correction_body_x_mm
            cy_mm = result.correction_body_y_mm
            cyaw = result.correction_yaw_deg
            self._last_usable_t = now
            # 先低通再进控制/收敛判定：检测本身 ±0.5~1mm 的抖动如果直接喂
            # PID，车会在目标附近一直蹭，收敛帧也会被单帧噪声打断。
            fx, fy, fyaw = self._filter_correction(now, cx_mm, cy_mm, cyaw)
            self._update_motion_state(now, fx, fy, fyaw)

            # 收敛判定：用滤波后的值，且必须已经停车、过了 settle 时间。
            if (not self._moving
                    and now >= self._settle_until
                    and max(abs(fx), abs(fy)) <= self._tol_xy
                    and abs(fyaw) <= self._tol_yaw):
                self._hold_frames += 1
            else:
                self._hold_frames = 0
            if self._hold_frames >= self._hold_need:
                self._converge()
                return
            if self._moving:
                self._control_step(now, fx, fy, fyaw)
            elif self._anchor_pose is not None:
                # 停车阶段：按住锁存的锚点位姿，位置/航向环持续有误差可修，
                # 不会像"每拍发当前位姿"那样等于放弃保持。
                self._hold_pose = self._anchor_pose
        else:
            self._hold_frames = 0
            # 丢环超过 0.5s 就清 PID/滤波器，免得旧积分/微分在重新看到环时顶一下
            if now - self._last_usable_t > 0.5:
                for pid in (self._pid_x, self._pid_y, self._pid_yaw):
                    pid.reset()
                self._last_ctrl_t = 0.0
                self._filt = None
                self._moving = False

        # F407 的位姿目标约 100ms 过期：每拍重发保持位姿，别让它散开。
        if self._hold_pose is not None:
            self._publish_pose(*self._hold_pose)

        if now - self._last_diag_t >= 1.0:
            self._last_diag_t = now
            detail = result.detail if result.detected else '未检测到三个圆环'
            if usable:
                err = (f'raw=({cx_mm:+.1f},{cy_mm:+.1f}mm {cyaw:+.2f}°) '
                       f'filt=({fx:+.1f},{fy:+.1f}mm {fyaw:+.2f}°) '
                       f'{"纠偏中" if self._moving else "保持"}')
            else:
                err = '--'
            self.get_logger().info(
                f'矫正中 [{self._phase}] {detail} {err} '
                f'稳定帧 {self._hold_frames}/{self._hold_need}')

    # ---------------- 伺服 ----------------

    def _filter_correction(self, now: float, cx_mm: float, cy_mm: float,
                           cyaw_deg: float):
        """视觉纠偏量一阶低通（EMA）。返回滤波后的 (x, y, yaw)。"""
        if self._filt is None:
            self._filt = [cx_mm, cy_mm, cyaw_deg]
            self._filt_t = now
            return cx_mm, cy_mm, cyaw_deg
        dt = min(max(now - self._filt_t, 1e-3), 0.2)
        self._filt_t = now
        alpha = 1.0 if self._filter_tau <= 0.0 else dt / (self._filter_tau + dt)
        for i, value in enumerate((cx_mm, cy_mm, cyaw_deg)):
            self._filt[i] += alpha * (value - self._filt[i])
        return self._filt[0], self._filt[1], self._filt[2]

    def _update_motion_state(self, now: float, fx: float, fy: float, fyaw: float):
        """滞环：进到 stop 带就停车，超过验收容差才重新起步。

        没有滞环时，误差在死区边缘被噪声推来推去，PID 每隔几拍就顶一下，
        车看起来就是原地抖、收敛帧永远攒不够。
        """
        mag = max(abs(fx), abs(fy))
        if self._moving:
            if mag <= self._stop_xy and abs(fyaw) <= self._stop_yaw:
                self._moving = False
                # 锁存停车时的实测位姿做保持目标。不能每拍都发"当前位姿"：
                # 那样位置环误差恒为 0，等于没在保持，车会被地面/轮隙慢慢带跑。
                self._anchor_pose = self._odom
                self._settle_until = now + self._settle_s
                for pid in (self._pid_x, self._pid_y, self._pid_yaw):
                    pid.reset()
        elif mag > self._tol_xy or abs(fyaw) > self._tol_yaw:
            self._moving = True
            self._anchor_pose = None
            self._last_ctrl_t = 0.0
            for pid in (self._pid_x, self._pid_y, self._pid_yaw):
                pid.reset()

    def _control_step(self, now: float, cx_mm: float, cy_mm: float, cyaw_deg: float):
        """PID 速度环：纠偏量(mm/mm/°) → 速度(mm/s、°/s) → 一拍的位姿小步。

        和物料跟随（vision/gimbal_track_pick.py）同一套 PID：输出限幅、积分
        抗饱和、微分低通。以前是每 0.3s 直接走 100% 纠偏量（等效 Kp=1、无
        阻尼），相机有 ~100ms 延迟，所以车在目标两侧来回震荡。
        """
        if self._odom is None or now - self._odom_t > 0.5:
            return   # 没有新鲜里程计不动车，超时兜底
        if self._last_ctrl_t <= 0.0:
            self._last_ctrl_t = now
            for pid in (self._pid_x, self._pid_y, self._pid_yaw):
                pid.reset()
            return
        dt = min(max(now - self._last_ctrl_t, 1e-3), 0.2)
        self._last_ctrl_t = now

        # 死区：已经压线就别再抖
        ex = 0.0 if abs(cx_mm) <= self._deadband_xy else cx_mm
        ey = 0.0 if abs(cy_mm) <= self._deadband_xy else cy_mm
        eyaw = 0.0 if abs(cyaw_deg) <= self._deadband_yaw else cyaw_deg

        vx = self._pid_x.update(ex, dt)          # mm/s（正 = 车体系 +x）
        vy = self._pid_y.update(ey, dt)          # mm/s（正 = 车体系 +y）
        vyaw = self._pid_yaw.update(eyaw, dt)    # °/s
        # 合成平移速度限幅，斜向别超速
        speed = math.hypot(vx, vy)
        if speed > self._max_xy_speed and speed > 1e-6:
            scale = self._max_xy_speed / speed
            vx *= scale
            vy *= scale
        vyaw = max(-self._max_yaw_speed, min(self._max_yaw_speed, vyaw))

        # 速度 → 位姿领先量。F407 的位置环是纯 P（输出 m/s = Kp × 误差 m）：
        # 目标领先当前位姿 L 毫米，车就以 Kp·L 的速度走。所以
        #   L = v / Kp   （mm/s ÷ 1/s = mm）
        # 直接把 v·dt 当步长会被位置环衰减几十倍（3mm 领先只对应 4.5mm/s）。
        lead_x = vx / self._pos_kp
        lead_y = vy / self._pos_kp
        lead = math.hypot(lead_x, lead_y)
        if lead > self._max_lead_xy and lead > 1e-6:
            scale = self._max_lead_xy / lead
            lead_x *= scale
            lead_y *= scale
        lead_yaw = max(-self._max_lead_yaw, min(self._max_lead_yaw,
                                               vyaw / self._heading_kp))
        # 车体系 → 里程计系
        ox, oy, oyaw = self._odom
        c, s = math.cos(oyaw), math.sin(oyaw)
        tx = ox + (lead_x * c - lead_y * s) / 1000.0
        ty = oy + (lead_x * s + lead_y * c) / 1000.0
        tyaw = oyaw + math.radians(lead_yaw)
        # 这一拍的目标；_tick 末尾的 hold 逻辑会把它按 20Hz 重发，F407 的目标
        # 约 100ms 过期，所以必须持续发。
        self._hold_pose = (tx, ty, tyaw)

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
        self._close_preview()
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

    # ---------------- 实时预览 ----------------

    def _draw_preview(self, frame, result):
        """把识别结果叠加到画面上，在树莓派桌面实时显示（窗口名 ring_fix）。

        蓝十字 = 标定参考环心（correction_calibration 的参考图像位置），
        绿/黄椭圆 = 当前帧识别到的圆环（绿 = 稳定观测），让人直接看出云台/
        相机有没有摆到参考姿态、画面里有没有看到三个环。
        """
        if not self._show_preview:
            return
        cv2 = self._cv2
        vis = frame.copy()
        # 标定基准：三个参考环心画蓝色十字 + 圆圈
        for rx, ry in self._ring_config.reference_centers_px:
            pt = (int(round(rx)), int(round(ry)))
            cv2.drawMarker(vis, pt, (255, 160, 0), cv2.MARKER_CROSS, 30, 2)
            cv2.circle(vis, pt, 24, (255, 160, 0), 1)
        # 当前识别：稳定 = 绿，识别到但不稳定 = 黄
        if result.detected:
            color = (0, 255, 0) if result.stable else (0, 220, 255)
            for ellipse in result.ellipses:
                cv2.ellipse(vis, ellipse, color, 2)
            for cx, cy in result.centers_px:
                cv2.circle(vis, (int(round(cx)), int(round(cy))), 5, color, -1)
        # 顶部状态条（cv2.putText 不支持中文，这里只用 ASCII）
        remain = max(0.0, self._timeout - (time.monotonic() - self._start_t))
        line1 = (f'RING FIX  detected:{"Y" if result.detected else "N"}'
                 f'  stable:{"Y" if result.stable else "N"}'
                 f'  conf:{result.confidence:.2f}')
        if (result.correction_body_x_mm is not None
                and result.correction_body_y_mm is not None
                and result.correction_yaw_deg is not None):
            line2 = (f'corr: x={result.correction_body_x_mm:+.1f}mm'
                     f' y={result.correction_body_y_mm:+.1f}mm'
                     f' yaw={result.correction_yaw_deg:+.2f}deg')
        else:
            line2 = 'corr: -- (waiting for stable observation)'
        line3 = f'hold: {self._hold_frames}/{self._hold_need}  left: {remain:.0f}s'
        line4 = 'blue cross = REF center (aim here); green/yellow = detected rings'
        for i, text in enumerate((line1, line2, line3, line4)):
            y = 30 + i * 26
            cv2.rectangle(vis, (4, y - 20), (640, y + 6), (0, 0, 0), -1)
            cv2.putText(vis, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 255), 1, cv2.LINE_AA)
        try:
            if not self._preview_active:
                cv2.namedWindow('ring_fix', cv2.WINDOW_NORMAL)
                self._preview_active = True
            cv2.imshow('ring_fix', vis)
            cv2.waitKey(1)   # 刷新窗口事件，必填
        except Exception as error:  # noqa: BLE001 - 纯 SSH 无桌面时降级关闭
            self._show_preview = False
            self._close_preview()
            self.get_logger().warning(
                f'无法弹出实时画面（无桌面环境？）：{error}；'
                f'预览已关闭，矫正照常进行')

    def _close_preview(self):
        if not self._preview_active:
            return
        try:
            self._cv2.destroyWindow('ring_fix')
        except Exception:  # noqa: BLE001
            pass
        self._preview_active = False

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
        self._close_preview()
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
