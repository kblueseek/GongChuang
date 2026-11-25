#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""云台跟随抓取跟踪器：color_pick 指令的视觉端。

作为 motion_planner 的子程序运行（ROS2 节点 + 摄像头在一个进程里）：

    python3 vision/gimbal_track_pick.py --color 4

做什么：
  1. 打开 USB 摄像头（与 vision/main.py 同一套畸变矫正配置）；
  2. 按颜色标定配置（config/material_color_thresholds.json）分割指定颜色物料；
  3. 实时窗口显示画面 + 识别框 + 十字线 + 当前云台角（--headless 关闭）；
  4. 物料偏离画面中心 → PID 把像素误差转成角速度，发 /cmd_gimbal_vel 让云台
     yaw/extend 平滑跟随（lift 保持不动），把物料"吊"到云台正下方；
  5. 连续稳定若干帧后退出，最后一行打印 JSON 结果：
         {"ok": true, "color": 4, "yaw_deg": …, "lift_deg": …, "extend_deg": …}
     motion_planner 的 color_pick 只跟随不抓取（no_pick），收到 JSON 后继续执行
     序列里后续的抓取/放置指令（你自己写死的 gimbal/gripper/tray 指令）。

需要现场标定的量（改 config/track_pid.json，上位机也能直接改；轴方向反了就翻号）：
  --target-x / --target-y         物料最后要停在哪（默认画面中心）。填成"夹爪降下去
                                  正好夹住物料"时物料所在的像素后，收工即可直接夹；
                                  需要偏移就改这个目标像素，不再做收工后的角度补偿
  --kp-yaw / --kp-extend          像素误差 → 角速度 °/s 的比例（越大跟得越猛，太大振荡）
  --yaw-sign / --ext-sign         轴方向（±1）
  --max-yaw-deg-s / --max-extend-deg-s  速度限幅
  --cam-rotation                  摄像头安装旋转角（180=倒装）

平面映射标定（--calib N，独立运行，不走 motion_planner）：
    python3 vision/gimbal_track_pick.py --calib 100 --color 1
  固定 HOME 姿态采集"N 组 像素 → 收工 (yaw, ext)"，拟合后写
  config/plane_mapping.json，供后续"查表直发 gimbal 绝对目标"（不用 PID）使用。
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import select
import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))          # 颜色标定 等同目录模块
from 颜色标定 import load_color_config  # noqa: E402

COLOR_NAMES = {1: '红', 2: '黄', 3: '蓝', 4: '绿', 5: '黑', 6: '浅蓝'}

# 每次 color_pick / track_debug 的逐帧跟踪数据（覆盖上一次）。
# 用 scripts/gimbal_pid_analyze.py 离线看波形、调 track_pid.json 的 PID。
# 设环境变量 TRACK_CSV='' 可关闭。
_TRACK_CSV = os.environ.get('TRACK_CSV', '/tmp/track_last.csv')
_TRACK_HEADER = (
    't', 'color', 'debug', 'phase', 'found',
    'ex', 'ey', 'yaw_vel', 'ext_vel',
    'gimbal_yaw', 'gimbal_lift', 'gimbal_ext', 'dt',
    'kp_yaw', 'ki_yaw', 'kd_yaw', 'kp_ext', 'ki_ext', 'kd_ext', 'deadband',
)


def _csv_fmt(value):
    return '' if value is None else f'{float(value):.3f}'


class PID:
    """带输出限幅、积分抗饱和、微分低通的小 PID（移植自 gimbal_track 包）。

    ki=kd=0 就是纯 P。加 kd 后能在不增大振荡的前提下把 kp 提得更高（微分提供阻尼）；
    ki 消稳态误差，但像素测量有噪声，建议先用小值或保持 0。
    """

    def __init__(self, kp, ki, kd, limit, deriv_tau=0.05):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.limit = float(limit)
        self.deriv_tau = max(0.0, float(deriv_tau))
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_err = 0.0
        self.deriv = 0.0          # 低通后的微分值

    def update(self, err, dt):
        if dt <= 0.0:
            dt = 0.01

        # 微分：先求原始微分，再做一阶低通，压住像素噪声的抖动
        raw_deriv = (err - self.prev_err) / dt
        self.prev_err = err
        if self.deriv_tau > 0.0:
            alpha = dt / (self.deriv_tau + dt)
            self.deriv += alpha * (raw_deriv - self.deriv)
        else:
            self.deriv = raw_deriv
        d = self.kd * self.deriv
        p = self.kp * err

        # 积分：条件积分抗饱和，饱和且误差还往同方向推时不再累积
        pre = p + self.ki * self.integral + d
        saturated = abs(pre) >= self.limit
        if self.ki != 0.0 and not (saturated and pre * err > 0.0):
            self.integral += err * dt
            bound = self.limit / abs(self.ki)
            self.integral = max(-bound, min(bound, self.integral))

        out = p + self.ki * self.integral + d
        return max(-self.limit, min(self.limit, out))


def build_undistort_maps(width: int, height: int):
    """与 vision/main.py / vision_put_align 同一套：camera_processing.json + 标定 yaml。"""
    import cv2
    proc = json.loads((HERE / 'config' / 'camera_processing.json').read_text(encoding='utf-8'))
    if not proc.get('undistort_enabled'):
        return None, None
    fs = cv2.FileStorage(str(HERE / 'config' / 'realtek_rgb_camera.yaml'),
                         cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        return None, None
    size = (int(fs.getNode('image_width').real()), int(fs.getNode('image_height').real()))
    if size != (width, height):
        fs.release()
        return None, None
    cam = fs.getNode('camera_matrix').mat()
    dist = fs.getNode('distortion_coefficients').mat()
    newc = fs.getNode('new_camera_matrix').mat()
    fs.release()
    if cam is None or dist is None or newc is None:
        return None, None
    return cv2.initUndistortRectifyMap(cam, dist, None, newc, size, cv2.CV_16SC2)


class GimbalTrackPick(Node):
    def __init__(self, args):
        super().__init__('gimbal_track_pick')
        self._args = args
        # 速度模式：发 /cmd_gimbal_vel（°/s），由 F407 的速度环直接执行。
        # 上位机序列里的 gimbal 指令走 /cmd_gimbal（位置环），F407 端"位置优先"，
        # 两条通道互不冲突。
        self._pub_vel = self.create_publisher(Float32MultiArray, '/cmd_gimbal_vel', 10)
        # 位置模式：只有 --calib（平面映射标定）用，让云台回到固定的起始姿态。
        # 序列里的 gimbal 指令走的也是这条通道，两边语义一致。
        self._pub_pos = self.create_publisher(Float32MultiArray, '/cmd_gimbal', 10)
        self.create_subscription(Float32MultiArray, '/gimbal_state',
                                 self._cb_gimbal_state, 10)
        self._gimbal = [0.0, 0.0, 0.0]
        self._gimbal_ok = False
        # 控制权标志：motion_planner 跑序列时用它让手柄让位，--calib 独立运行时
        # 也必须自己占住，否则手柄节点会以 100Hz 补 [0,0,0]，把跟随速度冲散。
        self._pub_active = self.create_publisher(Bool, '/motion_active', 10)
        self._pub_tune = self.create_publisher(Float32MultiArray, '/cmd_tune', 10)

    def publish_active(self, active: bool):
        self._pub_active.publish(Bool(data=bool(active)))

    def send_tune(self, param_id: int, value: float):
        """下发一项 F407 调参（[参数号, 值]），和上位机调参页同一条通道。"""
        self._pub_tune.publish(Float32MultiArray(
            data=[float(param_id), float(value)]))

    def _cb_gimbal_state(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            self._gimbal = [float(v) for v in msg.data[:3]]
            self._gimbal_ok = True

    def send_gimbal_vel(self, yaw: float, lift: float, extend: float):
        self._pub_vel.publish(Float32MultiArray(data=[yaw, lift, extend]))

    def send_gimbal_target(self, yaw: float, lift: float, extend: float):
        """绝对角目标（°/cmd_gimbal）。和上位机序列里的 gimbal 指令同一条通道。"""
        self._pub_pos.publish(Float32MultiArray(data=[float(yaw), float(lift), float(extend)]))


class ControlHold:
    """10Hz 发 /motion_active=True，让 joy_control 让出云台。

    为什么必须有：joy_control 在天上没人的时候每 10ms 发一帧 [0,0,0] 到
    /cmd_gimbal_vel 压住云台。标定脚本 20~30Hz 发的跟随速度会被这些 0 帧
    冲掉，F407 每两帧真正指令之间都收到 0 → 电机走走停停，表现就是
    "一顿一顿"。motion_planner 跑序列时也是发同样的心跳（10Hz），手柄收到
    就不发云台速度了；标定独立运行，只能自己发。

    手柄侧有 0.6s 掉线恢复，本进程崩了控制权也会自己还给手柄。
    """

    def __init__(self, node, period_s: float = 0.1):
        self._node = node
        self._period = float(period_s)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name='motion_active_hb')

    def _run(self):
        while not self._stop.is_set():
            self._node.publish_active(True)
            self._stop.wait(self._period)

    def __enter__(self):
        self._node.publish_active(True)     # 立刻占住，不等第一拍
        self._thread.start()
        return self

    def __exit__(self, *_exc):
        self._stop.set()
        self._thread.join(timeout=1.0)
        self._node.publish_active(False)    # 交还手柄
        return False


_DETECT_DIAG: dict = {}


def detect_target(bgr, color_id: int, document: dict, args,
                  min_cy=None):
    """按颜色标定分割 + 形状筛选，返回跟随目标 dict 或 None。

    min_cy：ROI 下边界（函数内缩放坐标的质心 y 下限），None = 全画面。

    移植自旧 gimbal_track 包的 material_detect_node（其筛选源自
    material_pickup_detector）：半分辨率处理提速；开+闭运算去噪；
    面积/尺寸/长宽比/实心度/占比逐项过滤（对应命令行参数，0 = 关闭该项）；
    贴着画面边缘的轮廓形状会失真，只按"颜色 + 最小面积"认它，免得物料
    一跟到画面边缘就被误判成没目标。通过筛选的候选里取面积最大者。
    """
    import cv2
    import numpy as np

    entry = next((c for c in document['colors'] if int(c['color_id']) == color_id), None)
    if entry is None:
        return None
    scale = args.processing_scale
    if scale < 1.0:
        working = cv2.resize(bgr, None, fx=scale, fy=scale,
                             interpolation=cv2.INTER_AREA)
    else:
        working = bgr
    hsv = cv2.cvtColor(working, cv2.COLOR_BGR2HSV)
    mask = None
    for r in entry['ranges']:
        m = cv2.inRange(hsv, tuple(int(v) for v in r['lower']),
                        tuple(int(v) for v in r['upper']))
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    if mask is None:
        return None

    def _kernel(px: int):
        s = max(1, int(round(px * scale)))
        return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (s, s))

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _kernel(args.open_kernel_px))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _kernel(args.close_kernel_px))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    scale_area = scale * scale
    img_w, img_h = working.shape[1], working.shape[0]
    best = None
    # 诊断用：记下本轮最大的那个色块（不管有没有通过筛选），没找到目标时
    # 打日志能直接看出是"太靠上被 ROI 裁了"还是"太小/太远"。
    _DETECT_DIAG.clear()
    for contour in contours:
        m0 = cv2.moments(contour)
        if abs(m0['m00']) < 1e-6:
            continue
        _area = float(cv2.contourArea(contour)) / scale_area
        if _area > _DETECT_DIAG.get('area', 0.0):
            _DETECT_DIAG['area'] = _area
            _DETECT_DIAG['cy'] = (m0['m01'] / m0['m00']) / scale
        # ROI：只认质心在画面下部的色块（圆盘大，只够得到最近的物料）。
        # 质心y在缩放坐标里比，和 min_cy（同为缩放坐标）直接比。
        if min_cy is not None and (m0['m01'] / m0['m00']) < min_cy:
            continue
        work_area = float(cv2.contourArea(contour))
        area = work_area / scale_area          # 换算回全分辨率像素
        if area < args.min_area:
            continue
        bx, by, bw, bh = cv2.boundingRect(contour)
        fw, fh = bw / scale, bh / scale
        touches_border = (bx <= 0 or by <= 0
                          or bx + bw >= img_w or by + bh >= img_h)
        if not touches_border:
            if 0.0 < args.max_area < area:
                continue
            if min(fw, fh) < args.min_dimension_px:
                continue
            if 0.0 < args.max_dimension_px < max(fw, fh):
                continue
            aspect = max(fw, fh) / max(1.0, min(fw, fh))
            if 0.0 < args.max_aspect < aspect:
                continue
            if args.min_solidity > 0.0:
                hull = float(cv2.contourArea(cv2.convexHull(contour)))
                if hull > 0.0 and work_area / hull < args.min_solidity:
                    continue
            if args.min_extent > 0.0 and \
                    work_area / max(1.0, float(bw * bh)) < args.min_extent:
                continue
        cx = (m0['m10'] / m0['m00']) / scale
        cy = (m0['m01'] / m0['m00']) / scale
        full_contour = np.rint(contour.astype(np.float32) / scale).astype(np.int32)
        if best is None or area > best['area']:
            best = {'cx': cx, 'cy': cy, 'area': area, 'w': fw, 'h': fh,
                    'contour': full_contour, 'border': touches_border}
    return best


def _apply_track_config(args, here: Path):
    """从 config/track_pid.json 读跟踪 PID/方向/旋转，覆盖命令行默认值。

    上位机（motion_gui）直接改这个 JSON，color_pick 每次启动都重新读，
    所以在上位机调完、下次跑 color_pick 就生效。文件不存在或解析失败就用默认值。
    """
    path = here / 'config' / 'track_pid.json'
    if not path.is_file():
        return args
    try:
        doc = json.loads(path.read_text(encoding='utf-8'))
    except Exception as error:  # noqa: BLE001
        print(f'[gimbal_track_pick] track_pid.json 解析失败，用默认值：{error}',
              flush=True)
        return args
    for key, val in doc.items():
        if hasattr(args, key):
            setattr(args, key, val)
    return args


# 调试模式里允许运行中热更新的参数（PID / 限幅 / 死区 / 方向 / 补偿）。
# 相机几何（cam_rotation / width / height / 畸变映射）不能热改：ctx 已经算好了。
_LIVE_TUNABLE = (
    'kp_yaw', 'ki_yaw', 'kd_yaw', 'kp_extend', 'ki_extend', 'kd_extend',
    'max_yaw_deg_s', 'max_extend_deg_s', 'deadband_px', 'deriv_tau',
    'yaw_sign', 'ext_sign', 'stable',
    'still_px',
    # 目标像素也能热改：track_debug 里拖滑杆，十字线当场跟着动，
    # 臂会跟着把物料重新对到新位置，看一眼就知道该填多少。
    'target_x', 'target_y',
)


def _as_float(value, fallback):
    """尽量转成 float；None/空串/非法值都退回 fallback。"""
    try:
        return float(value)
    except (TypeError, ValueError):
        return fallback


def _reload_live_config(args, here: Path):
    """调试模式用：只刷新能运行中改的参数，上位机拖滑块 1 秒内生效。"""
    path = here / 'config' / 'track_pid.json'
    try:
        doc = json.loads(path.read_text(encoding='utf-8'))
    except Exception:  # noqa: BLE001 - 改了一半/文件不存在就沿用上一版
        return args
    for key in _LIVE_TUNABLE:
        if key in doc and hasattr(args, key):
            setattr(args, key, doc[key])
    return args


def _open_camera(args):
    """打开摄像头并预热（头几帧常是空帧）。打不开返回 None。"""
    import cv2
    camera = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not camera.isOpened():
        return None
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    camera.set(cv2.CAP_PROP_FPS, args.fps)
    # 只留 1 帧缓冲：默认排队 4 帧，read() 拿到的可能是 130ms 前的旧画面，
    # 跟随环等于白白多了一段纯延迟。空闲循环会持续取帧，这里不会读不到。
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    for _ in range(10):                      # 预热
        ok, _f = camera.read()
        if ok:
            break
    return camera


def _prepare_frame_ctx(args) -> dict:
    """旋转表 / 畸变映射 / 画面几何量。摄像头开一次算一次，常驻模式复用。"""
    import cv2
    rotate = {0: None, 90: cv2.ROTATE_90_CLOCKWISE,
              180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}[args.cam_rotation]
    # 画面尺寸经旋转后才知道（90/270 宽高互换）
    if args.cam_rotation in (90, 270):
        frame_w, frame_h = args.height, args.width
    else:
        frame_w, frame_h = args.width, args.height
    map1, map2 = build_undistort_maps(args.width, args.height)

    # 跟随目标像素：默认画面中心；填了就用填的值。
    # 这里是"物料最终要停在哪"——把它设成"夹爪降下去正好夹住物料"时物料所在的
    # 像素，跟随收工时夹爪就已经对准物料，序列里不用再补 yaw/ext 偏移。
    return {
        'rotate': rotate, 'map1': map1, 'map2': map2,
        'frame_w': frame_w, 'frame_h': frame_h,
        'cx': _as_float(args.target_x, frame_w / 2.0),
        'cy': _as_float(args.target_y, frame_h / 2.0),
        'roi_y': int(frame_h * (1.0 - max(0.0, min(1.0, args.roi_bottom)))),
    }


def track_color(camera, node, args, color_doc, ctx, color,
                debug: bool = False, should_stop=None) -> dict:
    """跟随一种颜色直到对中，返回结果 dict（就是原来 main 里的那套循环）。

    每次调用都用全新的 PID / 静止判定状态，所以常驻模式里连续 pick 多种颜色
    互不干扰。调用方负责事先设好 args.color（detect_target 和状态行都用它）。

    debug=True 是"纯调参"模式（track_debug 动作）：
      · 不等静止、进死区不判成功，一直跟到超时或被停；
      · 每秒重读 config/track_pid.json，上位机拖 PID 滑块 1 秒内生效；
      · 每帧调 should_stop()，返回 True 就收工（节点发 close 时用）。
    """
    import cv2
    map1, map2 = ctx['map1'], ctx['map2']
    ROTATE = ctx['rotate']
    frame_w, frame_h = ctx['frame_w'], ctx['frame_h']
    cx_target, cy_target = ctx['cx'], ctx['cy']
    roi_y = ctx['roi_y']
    roi_y_scaled = roi_y   # detect_target 内部按同尺度缩放，见调用处

    # ---- 速度环 PID：像素误差 → 角速度 °/s（平滑跟随，Kp 直接决定跟得有多猛）----
    pid_yaw = PID(args.kp_yaw, args.ki_yaw, args.kd_yaw,
                  args.max_yaw_deg_s, args.deriv_tau)
    pid_extend = PID(args.kp_extend, args.ki_extend, args.kd_extend,
                     args.max_extend_deg_s, args.deriv_tau)

    stable = 0
    # wait = 等物料静止（不跟随，不动云台）；track = 静止确认后跟随对中
    phase = 'track' if (debug or args.still_time <= 0.0) else 'wait'
    hist = []                     # (t, cx, cy) 静止判定窗口（wait 阶段积累）
    started = time.monotonic()
    last_t = time.monotonic()     # PID 差分时间戳
    next_cfg = time.monotonic() + 1.0    # 调试模式：下次重读 track_pid.json 的时刻
    next_log = time.monotonic() + 1.0    # 调试模式：下次往日志打状态的时刻
    result = {'ok': False, 'reason': 'timeout'}

    # 逐帧 CSV（调 PID 用；写失败不影响跟踪）
    csv_file = None
    csv_writer = None
    if _TRACK_CSV:
        try:
            csv_file = open(_TRACK_CSV, 'w', newline='', encoding='utf-8')
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(_TRACK_HEADER)
            csv_file.flush()
        except OSError:
            csv_file = None
            csv_writer = None

    def _log_track(now, ex, ey, yaw_vel, ext_vel, dt):
        if csv_writer is None:
            return
        g = node._gimbal if node._gimbal_ok else (None, None, None)
        try:
            csv_writer.writerow((
                f'{now - started:.4f}', args.color, int(bool(debug)), phase,
                int(bool(found)), f'{ex:.2f}', f'{ey:.2f}',
                f'{yaw_vel:.2f}', f'{ext_vel:.2f}',
                _csv_fmt(g[0]), _csv_fmt(g[1]), _csv_fmt(g[2]), f'{dt:.4f}',
                f'{pid_yaw.kp:.4f}', f'{pid_yaw.ki:.4f}', f'{pid_yaw.kd:.4f}',
                f'{pid_extend.kp:.4f}', f'{pid_extend.ki:.4f}',
                f'{pid_extend.kd:.4f}', f'{args.deadband_px:.1f}',
            ))
            csv_file.flush()
        except OSError:
            pass

    def _is_still(now):
        '''最近 still_time 秒内质心抖动 <= still_px = 物料没动。'''
        if args.still_time <= 0.0:
            return False
        win = [(t, x, y) for (t, x, y) in hist if now - t <= args.still_time]
        if len(win) < 3 or now - win[0][0] < args.still_time * 0.8:
            return False
        xs = [p[1] for p in win]
        ys = [p[2] for p in win]
        return (max(xs) - min(xs) <= args.still_px
                and max(ys) - min(ys) <= args.still_px)

    try:
        while time.monotonic() - started < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.0)
            # 统一节点里 stop 是"取消当前业务"；旧的一次性调用没传 should_stop，
            # 行为不变。debug 模式沿用原来的 reason，方便上位机区分。
            if should_stop is not None and should_stop():
                if debug:
                    result = {'ok': True, 'color': args.color,
                              'debug': True, 'reason': 'stopped'}
                else:
                    result = {'ok': False, 'color': args.color,
                              'reason': 'stopped'}
                break
            if debug:
                if time.monotonic() >= next_cfg:
                    # 上位机拖完滑块后 1 秒内生效，不用重跑序列
                    next_cfg = time.monotonic() + 1.0
                    _reload_live_config(args, HERE)
                    pid_yaw.kp, pid_yaw.ki, pid_yaw.kd = (
                        args.kp_yaw, args.ki_yaw, args.kd_yaw)
                    pid_yaw.limit = args.max_yaw_deg_s
                    pid_yaw.deriv_tau = max(0.0, args.deriv_tau)
                    pid_extend.kp, pid_extend.ki, pid_extend.kd = (
                        args.kp_extend, args.ki_extend, args.kd_extend)
                    pid_extend.limit = args.max_extend_deg_s
                    pid_extend.deriv_tau = max(0.0, args.deriv_tau)
            ok, frame = camera.read()
            if not ok or frame is None:
                continue
            if map1 is not None:
                frame = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
            if ROTATE is not None:
                frame = cv2.rotate(frame, ROTATE)

            # 每帧重算目标像素：track_debug 里允许热改（拖滑杆 1 秒内生效），
            # 十字线和误差都跟着走，方便现场找"夹爪能夹住物料"的那个位置。
            cx_target = _as_float(args.target_x, frame_w / 2.0)
            cy_target = _as_float(args.target_y, frame_h / 2.0)

            # ROI 在 detect_target 的缩放坐标系里：detect 内部 scale=processing_scale
            target = detect_target(frame, args.color, color_doc, args,
                                   min_cy=roi_y_scaled * args.processing_scale)
            found = target is not None
            ex = ey = 0.0
            area = target['area'] if found else 0.0
            if found:
                ex = target['cx'] - cx_target
                ey = target['cy'] - cy_target
                if not args.headless:
                    cv2.drawContours(frame, [target['contour']], -1,
                                     (0, 255, 0), 2)
                    cv2.drawMarker(frame, (int(target['cx']), int(target['cy'])),
                                   (0, 255, 255), cv2.MARKER_CROSS, 26, 2)
                    if target['border']:
                        cv2.putText(frame, 'EDGE', (16, 98),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                    (0, 165, 255), 2, cv2.LINE_AA)

            now = time.monotonic()   # 本帧时间戳：静止判定/hist/PID 差分共用
            dt = now - last_t
            last_t = now

            yaw_vel = ext_vel = 0.0

            if found:
                if phase == 'wait':
                    # 不跟随、不动云台，只观察：物料持续静止才算数（转盘场景）
                    hist.append((now, target['cx'], target['cy']))
                    if _is_still(now):
                        phase = 'track'
                        hist.clear()
                        stable = 0
                        pid_yaw.reset()
                        pid_extend.reset()
                        print('[gimbal_track_pick] 物料已静止，开始跟随对中', flush=True)
                    status = (f'WAIT_STILL {now - started:.1f}s '
                              f'({args.still_time}s 内抖动<{args.still_px:.0f}px)')
                else:
                    # track：PID 把像素误差直接转成角速度，每帧连续输出（平滑跟随）
                    if abs(ex) <= args.deadband_px and abs(ey) <= args.deadband_px:
                        # 已进死区：停住并累计"对准"帧数
                        stable += 1
                        pid_yaw.reset()
                        pid_extend.reset()
                    else:
                        stable = 0
                        yaw_vel = args.yaw_sign * pid_yaw.update(ex, dt)
                        ext_vel = args.ext_sign * pid_extend.update(ey, dt)
                    status = (f'TRACK ex={ex:+.0f} ey={ey:+.0f} '
                              f'yaw={yaw_vel:+.0f}°/s ext={ext_vel:+.0f}°/s '
                              f'stable={stable}/{args.stable}')
                    if (not debug) and stable >= args.stable and node._gimbal_ok:
                        result = {'ok': True, 'color': args.color,
                                  'yaw_deg': round(node._gimbal[0], 2),
                                  'lift_deg': round(node._gimbal[1], 2),
                                  'extend_deg': round(node._gimbal[2], 2),
                                  # 收工瞬间的残余像素误差：标定用它判断这一组
                                  # 是"真的对中"还是"卡在死区边上"。
                                  'ex_px': round(ex, 2), 'ey_px': round(ey, 2)}
                        print(f'[gimbal_track_pick] 对准收工：{status}', flush=True)
                        _log_track(now, ex, ey, yaw_vel, ext_vel, dt)
                        break
            else:
                stable = 0
                hist.clear()
                pid_yaw.reset()
                pid_extend.reset()
                status = f'SEARCH color={args.color}({COLOR_NAMES[args.color]}) area={area:.0f}'
                # 连续 1 秒没找到就打一行（每秒最多一次）：排查"物料靠画面上方
                # 就抓不到"这类问题时，能直接看出是没检测到、还是检测到了没跟上。
                # 质心在 ROI 线以上的色块会被直接丢掉（见 detect_target 的 min_cy）。
                if time.monotonic() >= next_log:
                    next_log = time.monotonic() + 1.0
                    diag = dict(_DETECT_DIAG)
                    if not diag:
                        why = '画面里没有这个颜色的色块'
                    elif diag.get('cy', 0.0) < roi_y:
                        why = (f'最大色块质心 y={diag["cy"]:.0f} 在 ROI 线 y={roi_y} '
                               f'上方被裁掉（画面上方 {frame_h - roi_y}px 不认）')
                    elif diag.get('area', 0.0) < args.min_area:
                        why = (f'最大色块面积 {diag["area"]:.0f}px < min_area '
                               f'{args.min_area:.0f}（太远/太小）')
                    else:
                        why = (f'最大色块 {diag.get("area", 0):.0f}px2 '
                               f'y={diag.get("cy", 0):.0f} 被形状筛选排除')
                    print(f'[gimbal_track_pick] 没检测到目标：{why}；'
                          f'画面 {frame_w}x{frame_h}，已搜 '
                          f'{time.monotonic() - started:.1f}s', flush=True)

            # 每帧都发速度：跟踪时发 PID 输出，等停/丢目标/死区时发 0，
            # 既保证 F407 速度环 300ms 超时不会停得一顿一顿，也保证收工瞬间立即停住。
            node.send_gimbal_vel(yaw_vel, 0.0, ext_vel)
            _log_track(now, ex, ey, yaw_vel, ext_vel, dt)

            if debug and time.monotonic() >= next_log:
                # 每秒往 ROS 日志打一行，方便不开窗口也能看出参数有没有生效
                next_log = time.monotonic() + 1.0
                print(f'[gimbal_track_pick] DEBUG {status} '
                      f'kp={args.kp_yaw}/{args.kp_extend} '
                      f'max={args.max_yaw_deg_s:.0f}/{args.max_extend_deg_s:.0f}',
                      flush=True)

            if not args.headless:
                cv2.drawMarker(frame, (int(cx_target), int(cy_target)),
                               (255, 255, 255), cv2.MARKER_CROSS, 30, 1)
                cv2.line(frame, (0, roi_y), (frame_w, roi_y), (120, 120, 255), 1)
                cv2.putText(frame, f'ROI>{args.roi_bottom}', (frame_w - 130, roi_y - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (120, 120, 255), 1,
                            cv2.LINE_AA)
                cv2.putText(frame, ('DEBUG ' + status) if debug else status,
                            (16, 34), cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, (0, 255, 0) if found else (0, 165, 255), 2,
                            cv2.LINE_AA)
                if node._gimbal_ok:
                    cv2.putText(frame,
                                f'gimbal yaw={node._gimbal[0]:+.1f} '
                                f'lift={node._gimbal[1]:+.1f} '
                                f'ext={node._gimbal[2]:+.1f}',
                                (16, 66), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                                (220, 220, 220), 1, cv2.LINE_AA)
                cv2.imshow('gimbal_track_pick', frame)
                if cv2.waitKey(1) & 0xFF in (27, ord('q')):
                    result = {'ok': False, 'reason': 'user quit'}
                    break
    finally:
        # 收工/异常退出：发一帧零速，云台立刻停住，别带着最后一拍速度漂。
        node.send_gimbal_vel(0.0, 0.0, 0.0)
        if csv_file is not None:
            try:
                csv_file.flush()
                csv_file.close()
            except OSError:
                pass
        # 预览窗口也要收掉：业务结束后没人再调 imshow/waitKey，窗口留着
        # 会被系统判定"无响应"，下一次业务切窗口时就会卡死。
        if not args.headless:
            cv2.destroyAllWindows()
            cv2.waitKey(1)

    return result


def _pump_preview(camera, args, ctx, text: str):
    """抓一帧贴到预览窗口上（常驻模式用）。

    常驻模式在两次 pick 之间没有 imshow，窗口只是一块灰色，看起来就像
    "摄像头没启动"。启动时和每次 pick 结束后各贴一帧，open 一执行就能
    看到实时画面，也顺便证明摄像头真的开了。
    """
    if args.headless:
        return
    import cv2
    ok, frame = camera.read()
    if not ok or frame is None:
        print('[gimbal_track_pick] 预览取帧失败（摄像头可能被别的程序占用）',
              flush=True)
        return
    if ctx['map1'] is not None:
        frame = cv2.remap(frame, ctx['map1'], ctx['map2'], cv2.INTER_LINEAR)
    if ctx['rotate'] is not None:
        frame = cv2.rotate(frame, ctx['rotate'])
    cv2.putText(frame, text, (16, 34), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.imshow('gimbal_track_pick', frame)
    cv2.waitKey(1)


def _stdin_ready(timeout_s: float = 0.0) -> bool:
    """stdin 上有没有一整行可读（非阻塞轮询，空闲时用来刷预览）。"""
    try:
        return bool(select.select([sys.stdin], [], [], max(0.0, timeout_s))[0])
    except (OSError, ValueError):
        return False


def _idle_preview(camera, args, ctx):
    """两次请求之间一直取帧刷预览，画面就不会"追完就冻住"。

    顺带把摄像头缓冲排空 —— pick 开始时读到的才是当前画面，而不是几百毫秒
    前压在 V4L2 队列里的旧帧。这里不做畸变矫正（预览不需要精度），省 CPU。
    """
    if args.headless:
        camera.grab()                    # 不看窗口也要排空队列
        time.sleep(0.03)
        return
    import cv2
    ok, frame = camera.read()
    if not ok or frame is None:
        time.sleep(0.01)
        return
    if ctx['rotate'] is not None:
        frame = cv2.rotate(frame, ctx['rotate'])
    cv2.putText(frame, '等待 color_pick（画面实时）', (16, 34),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.imshow('gimbal_track_pick', frame)
    cv2.waitKey(1)
    time.sleep(0.03)                     # 降到 ~15fps，别一直烧满一个核


# ---------------- 平面映射标定（--calib）：像素 → 云台角 ----------------
#
# 目标：在「云台回到固定起始姿态（HOME）」这个前提下，测出
#     物料在画面 (u, v)  →  要让夹爪到它上方的 (Δyaw, Δext)
# 的映射。测出来以后跟踪就不用 PID 了：拍一帧 → 查表 → 一条绝对 gimbal 指令。
#
# 每组样本的流程（--calib N 重复 N 次）：
#   1. 绝对位置指令回 HOME（yaw/lift/ext 三个角，程序启动时读当前姿态当 HOME）；
#   2. 在 HOME 姿态等物料静止（不是"画面里随便静止"，必须从同一个视角看，
#      否则同一个像素在不同 ext 下对应完全不同的世界点，映射就不是单值的了）；
#   3. 记录此刻像素 (u, v)；
#   4. 用现有 PID 跟随对中，记录收工时的 (yaw, lift, ext) 和残余像素；
#   5. 再回 HOME 复核一次像素：和 (u, v) 差太多 = 跟随期间物料被挪过，这组作废；
#   6. 弹窗确认（弹窗期间可以挪物料），点了"是"继续；没桌面环境时退回
#      "自动盯画面等物料挪走"（像素离开 (u, v) 超过 --calib-move-px）。
#      物料还停在上一组的点上会提示再挪一次 —— 原地重复采对拟合没用。
#
# 全程以 10Hz 发 /motion_active=True 占住云台（motion_planner 跑序列同样的
# 协议）：不占的话 joy_control 会 100Hz 往 /cmd_gimbal_vel 补 [0,0,0]，
# 把我们的跟随速度冲散，云台就"一顿一顿"。退出时（含 Ctrl-C）自动交还。
#
# 注意：画面上是 6×4 的格子，每格记数字 = 该格采了几组。**覆盖比数量重要**：
# 100 组全堆在画面中央，拟合出来只在那一个很小区域准。尽量让每格都有样本。

def _read_frame(camera, ctx):
    """取一帧并做和跟踪器完全一样的预处理（畸变矫正 + 旋转）。"""
    ok, frame = camera.read()
    if not ok or frame is None:
        return None
    if ctx['map1'] is not None:
        import cv2
        frame = cv2.remap(frame, ctx['map1'], ctx['map2'], cv2.INTER_LINEAR)
    if ctx['rotate'] is not None:
        import cv2
        frame = cv2.rotate(frame, ctx['rotate'])
    return frame


def _detect_in_frame(frame, args, color_doc, ctx):
    """和 track_color 用同一个检测函数、同一个 ROI。"""
    return detect_target(frame, args.color, color_doc, args,
                         min_cy=ctx['roi_y'] * args.processing_scale)


def _read_gimbal(node, timeout_s: float = 2.0):
    """等 /gimbal_state 来一帧，返回当前三轴角；超时返回 None。"""
    t0 = time.monotonic()
    while not node._gimbal_ok and time.monotonic() - t0 < timeout_s:
        rclpy.spin_once(node, timeout_sec=0.05)
    return list(node._gimbal) if node._gimbal_ok else None


def _move_home(node, home, tol_deg: float = 1.0, timeout_s: float = 15.0) -> bool:
    """发绝对位置目标回 HOME，等三轴实测进容差。每 0.5s 重发一次续命。

    容差比原来的 2° 收紧到 1°：每组样本的起始视角必须和拟合用的 HOME 一致，
    差 2° 在 yaw 上就是图像里 6~7px 的偏差，直接变成样本噪声 —— 34 组数据里
    yaw 残差 ~3°（≈10px）有一大半是这个来的。

    再紧就没意义了：F407 位置环的速度帧是整数 RPM（最小 1 RPM = 6°/s），
    P 输出小于 6°/s 就停住，所以 lift（kp 7.5）最后总留 ±0.8°、ext（kp 12）
    留 ±0.5° 的量化误差。1° 是"能稳定达到"和"再紧就是假精度"之间的值。
    """
    t0 = time.monotonic()
    next_pub = 0.0
    while time.monotonic() - t0 < timeout_s:
        now = time.monotonic()
        if now >= next_pub:
            node.send_gimbal_target(*home)
            next_pub = now + 0.5
        rclpy.spin_once(node, timeout_sec=0.02)
        if all(abs(a - h) <= tol_deg for a, h in zip(node._gimbal, home)):
            return True
    return False


def _key_stop(args) -> bool:
    """预览窗口里按 q / Esc = 提前收工（已采的样本照样保存并拟合）。"""
    if args.headless:
        return False
    import cv2
    try:
        key = cv2.waitKey(1) & 0xFF
    except cv2.error:
        return False
    return key in (ord('q'), ord('Q'), 27)


def _calib_cell(ctx, u, v, cols, rows):
    w, h = ctx['frame_w'], ctx['frame_h']
    c = min(cols - 1, max(0, int(u / w * cols)))
    r = min(rows - 1, max(0, int(v / h * rows)))
    return r, c


def _draw_calib(frame, args, ctx, cover, done, status, target=None):
    """预览叠加：采样网格 + 每格计数 + 当前状态（cv2 只认 ASCII，别写中文）。"""
    import cv2
    w, h = ctx['frame_w'], ctx['frame_h']
    cols, rows = args.calib_grid_cols, args.calib_grid_rows
    for c in range(1, cols):
        x = int(w * c / cols)
        cv2.line(frame, (x, 0), (x, h), (70, 70, 70), 1)
    for r in range(1, rows):
        y = int(h * r / rows)
        cv2.line(frame, (0, y), (w, y), (70, 70, 70), 1)
    for (r, c), n in cover.items():
        x, y = int(w * c / cols + 10), int(h * r / rows + 30)
        cv2.putText(frame, str(n), (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (0, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(frame, f'CALIB {done}/{args.calib}  {status}', (16, 34),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.putText(frame, f'yaw={ctx["home"][0]:+.1f} lift={ctx["home"][1]:+.1f} '
                       f'ext={ctx["home"][2]:+.1f}', (16, 64),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
    if target is not None:
        cv2.drawContours(frame, [target['contour']], -1, (0, 255, 0), 2)
        cv2.drawMarker(frame, (int(target['cx']), int(target['cy'])),
                       (0, 255, 255), cv2.MARKER_CROSS, 26, 2)


def _show(args, frame):
    """只 imshow，不 waitKey —— 按键统一由 _key_stop 读，读两遍会丢按键。"""
    if args.headless:
        return
    import cv2
    cv2.imshow('gimbal_track_pick', frame)


def _show_hint(camera, args, color_doc, ctx, cover, done):
    """贴一帧带网格计数的画面，用来提示"该挪物料了"。"""
    frame = _read_frame(camera, ctx)
    if frame is None:
        return
    target = _detect_in_frame(frame, args, color_doc, ctx)
    _draw_calib(frame, args, ctx, cover, done, 'MOVE MATERIAL', target)
    _show(args, frame)


def _dist(a, b) -> float:
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def _ask_next_group(done: int, total: int, cols: int = 6, rows: int = 4):
    """每组结束后弹窗确认，顺便给用户时间把物料挪到新位置。

    返回 True=继续、False=结束采集、None=没有图形界面（调用方退回自动模式）。
    """
    try:
        import tkinter as tk
        from tkinter import messagebox
        root = getattr(_ask_next_group, '_root', None)
        if root is None:
            root = tk.Tk()
            root.withdraw()
            root.attributes('-topmost', True)
            _ask_next_group._root = root
        return bool(messagebox.askyesno(
            '云台标定',
            f'已完成 {done}/{total} 组。\n\n'
            '请把物料挪到一个新的位置 —— 尽量换到预览窗口里\n'
            f'还没打勾的格子（{cols}×{rows} 网格上有计数）。\n\n'
            '挪好、手离开物料后点【是】继续下一组；\n'
            '点【否】结束采集并拟合保存。\n\n'
            '（弹窗期间预览画面是停住的，点【是】之后恢复实时）',
            parent=root))
    except Exception:  # noqa: BLE001 - 没有 DISPLAY / tkinter 不可用
        return None


def _wait_still_at_home(camera, args, color_doc, ctx, cover, done,
                        still_s: float, timeout_s: float):
    """在 HOME 姿态等物料静止，返回 (u, v)；超时/中止返回 None。

    只认画面里的同一个目标（面积最大那块），且要求没贴边 —— 贴边时轮廓被
    裁掉，质心会偏，这种样本进了拟合就是脏数据。
    """
    hist = []
    last = None
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        if _key_stop(args):
            return None
        t = time.monotonic()
        frame = _read_frame(camera, ctx)
        if frame is None:
            time.sleep(0.01)
            continue
        target = _detect_in_frame(frame, args, color_doc, ctx)
        if target is not None and not target['border']:
            last = target
            hist.append((t, target['cx'], target['cy']))
        else:
            last = None
            hist.clear()
        hist = [p for p in hist if t - p[0] <= still_s]
        still = False
        if len(hist) >= 3 and hist[-1][0] - hist[0][0] >= still_s * 0.8:
            xs = [p[1] for p in hist]
            ys = [p[2] for p in hist]
            still = (max(xs) - min(xs) <= args.calib_still_px
                     and max(ys) - min(ys) <= args.calib_still_px)
        status = 'STILL' if still else (
            'NO TARGET' if last is None else 'WAIT STILL')
        _draw_calib(frame, args, ctx, cover, done, status, last)
        _show(args, frame)
        if still:
            return (last['cx'], last['cy'])
        time.sleep(0.005)
    return None


def _track_to_center(camera, node, args, color_doc, ctx):
    """用现有 PID 跟随对中，返回 track_color 的结果（含收工姿态和残余像素）。"""
    saved = (args.still_time, args.deadband_px, args.stable, args.timeout)
    # 标定要求比平时准：默认死区 8px 的收工姿态，等价于 ±1° 的采样误差。
    args.still_time = 0.0                 # 静止判定上面已经做过了
    args.deadband_px = float(args.calib_deadband_px)
    args.stable = int(args.calib_stable)
    args.timeout = float(args.calib_track_timeout_s)
    try:
        return track_color(camera, node, args, color_doc, ctx, int(args.color))
    finally:
        (args.still_time, args.deadband_px, args.stable, args.timeout) = saved


def _wait_material_moved(camera, args, color_doc, ctx, cover, done,
                         ref_uv, timeout_s: float) -> bool:
    """等用户把物料挪走（相对 ref_uv 移动超过 --calib-move-px，或目标丢失）。"""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        if _key_stop(args):
            return False
        frame = _read_frame(camera, ctx)
        if frame is None:
            time.sleep(0.01)
            continue
        target = _detect_in_frame(frame, args, color_doc, ctx)
        if target is None:
            moved = True
        else:
            d = ((target['cx'] - ref_uv[0]) ** 2
                 + (target['cy'] - ref_uv[1]) ** 2) ** 0.5
            moved = d > args.calib_move_px
        _draw_calib(frame, args, ctx, cover, done,
                    'MOVED' if moved else 'MOVE MATERIAL', target)
        _show(args, frame)
        if moved:
            return True
        time.sleep(0.01)
    return False


def _poly_cols(du, dv, degree: int) -> list:
    """多项式基：1, du, dv[, du², du·dv, dv²][, du³, du²·dv, du·dv²]。

    三次项只给 u 方向（yaw 需要，见 `_fit_plane_mapping` 的说明），v 方向
    三次项实测只会过拟合，所以不做 dv³。
    """
    import numpy as np
    cols = [np.ones_like(du), du, dv]
    if degree >= 2:
        cols += [du * du, du * dv, dv * dv]
    if degree >= 3:
        cols += [du ** 3, du * du * dv, du * dv * dv]
    return cols


def _poly_deriv(co, du, dv, degree: int):
    """基函数对 du / dv 的偏导（归一化坐标，未除以半宽半高）。"""
    import numpy as np
    d_du = np.full_like(du, co[1])
    d_dv = np.full_like(du, co[2])
    if degree >= 2:
        d_du = d_du + 2 * co[3] * du + co[4] * dv
        d_dv = d_dv + co[4] * du + 2 * co[5] * dv
    if degree >= 3:
        d_du = d_du + 3 * co[6] * du * du + 2 * co[7] * du * dv + co[8] * dv * dv
        d_dv = d_dv + co[7] * du * du + 2 * co[8] * du * dv
    return d_du, d_dv


def _axis_degree(fit: dict, axis: str) -> int:
    """取某一轴的多项式阶数；老文件只有 `degree` 时按它的值。"""
    key = 'yaw_degree' if axis == 'yaw' else 'ext_degree'
    return int(fit.get(key, fit.get('degree', 2)))


def _lstsq_fit(samples: list, ctx, degree: int = 2, key_y='d_yaw',
               key_e='d_ext', degree_e: int | None = None) -> dict:
    """最小二乘拟合 像素 → (Δyaw, Δext)，并把残差换算成像素报告。

    坐标先归一化到 [-1, 1]（u 除以半宽、v 除以半高），否则 u² 项的量级会
    把矩阵条件数拉爆，linalg 给的解就不可信了。

    yaw 和 ext 可以**分别取阶数**（`degree` / `degree_e`）。实测两轴的最优
    阶数不一样：yaw 要三次（CV 中位误差 1.9° → 1.0°），ext 三次反而变差
    （25° → 28°），所以默认 ext 用二次。
    """
    import numpy as np
    w, h = ctx['frame_w'], ctx['frame_h']
    cx, cy = ctx['cx'], ctx['cy']
    if degree_e is None:
        degree_e = degree
    uv = np.array([[s['u'], s['v']] for s in samples], float)
    dy = np.array([s[key_y] for s in samples], float)
    de = np.array([s[key_e] for s in samples], float)

    du = (uv[:, 0] - cx) / (w / 2.0)
    dv = (uv[:, 1] - cy) / (h / 2.0)
    mat_y = np.stack(_poly_cols(du, dv, degree), axis=1)
    mat_e = np.stack(_poly_cols(du, dv, degree_e), axis=1)

    cy_coe, *_ = np.linalg.lstsq(mat_y, dy, rcond=None)
    ce_coe, *_ = np.linalg.lstsq(mat_e, de, rcond=None)
    res_y = mat_y @ cy_coe - dy
    res_e = mat_e @ ce_coe - de

    # 角度残差 → 像素残差：用拟合映射的局部雅可比 J = d(Δ角)/d(像素)
    dy_u, dy_v = _poly_deriv(cy_coe, du, dv, degree)
    de_u, de_v = _poly_deriv(ce_coe, du, dv, degree_e)
    jy_u, jy_v = dy_u / (w / 2.0), dy_v / (h / 2.0)   # 对角/像素
    je_u, je_v = de_u / (w / 2.0), de_v / (h / 2.0)
    err_px = []
    for i in range(len(samples)):
        a, b, c, d = jy_u[i], jy_v[i], je_u[i], je_v[i]
        det = a * d - b * c
        if abs(det) < 1e-12:
            continue
        # J⁻¹ · [res_y, res_e]
        pu = (d * res_y[i] - b * res_e[i]) / det
        pv = (-c * res_y[i] + a * res_e[i]) / det
        err_px.append((pu * pu + pv * pv) ** 0.5)
    err_px.sort()
    # 中位数比 RMS 更能说明问题：个别点落在雅可比很病态的位置（画面极边缘）
    # 会把 RMS 拉得很难看，但那些点本来就不该拿来采样。
    def _pct(q):
        if not err_px:
            return None
        k = min(len(err_px) - 1, max(0, int(round(q * (len(err_px) - 1)))))
        return round(float(err_px[k]), 2)
    return {
        'degree': degree,
        'ext_degree': degree_e,
        'yaw_coeff': [round(float(v), 8) for v in cy_coe],
        'ext_coeff': [round(float(v), 8) for v in ce_coe],
        'residual_deg': {
            'yaw_rms': round(float(np.sqrt(np.mean(res_y ** 2))), 3),
            'ext_rms': round(float(np.sqrt(np.mean(res_e ** 2))), 3),
            'yaw_max': round(float(np.max(np.abs(res_y))), 3),
            'ext_max': round(float(np.max(np.abs(res_e))), 3),
        },
        'residual_px': {
            'median': _pct(0.50),
            'p90': _pct(0.90),
            'rms': round(float(np.sqrt(np.mean(np.square(err_px)))), 2)
            if err_px else None,
            'max': round(float(np.max(err_px)), 2) if err_px else None,
        },
        'samples_used': int(len(samples)),
        'samples_size': int(mat_y.shape[1]),
        'samples_size_ext': int(mat_e.shape[1]),
    }


def _jacobian_deg_per_px(fit: dict, u: float, v: float, ctx) -> tuple:
    """拟合映射在 (u, v) 处的局部雅可比：[[∂yaw/∂u, ∂yaw/∂v], [∂ext/∂u, ∂ext/∂v]]。"""
    w, h = ctx['frame_w'], ctx['frame_h']
    du = (u - ctx['cx']) / (w / 2.0)
    dv = (v - ctx['cy']) / (h / 2.0)

    def per_axis(co, deg):
        d_du, d_dv = _poly_deriv(co, du, dv, deg)
        return d_du / (w / 2.0), d_dv / (h / 2.0)

    ay, by = per_axis(fit['yaw_coeff'], _axis_degree(fit, 'yaw'))
    ae, be = per_axis(fit['ext_coeff'], _axis_degree(fit, 'ext'))
    return ay, by, ae, be


def _predict(fit: dict, u: float, v: float, ctx) -> tuple:
    """按拟合系数算 (Δyaw, Δext)——运行时查表也走这里。"""
    w, h = ctx['frame_w'], ctx['frame_h']
    du = (u - ctx['cx']) / (w / 2.0)
    dv = (v - ctx['cy']) / (h / 2.0)
    phi_y = _poly_cols(du, dv, _axis_degree(fit, 'yaw'))
    phi_e = _poly_cols(du, dv, _axis_degree(fit, 'ext'))
    yaw = sum(c * float(p) for c, p in zip(fit['yaw_coeff'], phi_y))
    ext = sum(c * float(p) for c, p in zip(fit['ext_coeff'], phi_e))
    return float(yaw), float(ext)


def _fit_plane_mapping(samples: list, ctx, degree: int = 2,
                       degree_e: int | None = None) -> dict:
    """拟合 + 用每组记录的残余像素把收工姿态回正，然后再拟合一次。

    为什么要回正：F407 的云台速度帧是**整数 RPM**（1 RPM = 6°/s），yaw 环
    kp≈1.4 时误差小于 ~4.4px 的指令会被截断成 0 RPM —— 云台只能停在离中心
    4~5px 的地方，永远进不了 2px 死区（这正是标定一开始一直 timeout 的原因）。
    所以标定死区按 5px 设，收工姿态天生带 4~5px 偏差；好在偏差就是收工那一帧
    实测的 (ex, ey)，用拟合出的雅可比反推一下就能把姿态补正。

    补正方向是 **d + J·(ex, ey)**（实测用 34 组数据验证过：加号把 ext 残差从
    11.9° 降到 10.0°；减号反而升到 14.2°）。直观理解：J = ∂Δ角/∂像素 是
    "物料在画面里更靠右 → 收工姿态要更大"的**全局**映射，不是"姿态增加→画面
    往哪走"的局部运动方向，两者差一个负号，所以这里要加。

    阶数：yaw 用三次、ext 用二次。55 组数据上做 6 折交叉验证，yaw 三次把
    留出点的中位误差从 1.9° 压到 1.0°（p90 6.0°→4.5°），是真实增益；ext
    加三次反而从 25° 涨到 28°，它的误差是被**收工死区**（±5px × 局部 2°/px）
    限住的，不是模型不够。样本少于 24 组时三次项欠定，自动退回二次。
    """
    if degree >= 3 and len(samples) < 24:
        degree = 2
    if degree_e is None:
        degree_e = 2 if degree >= 2 else degree
    raw = _lstsq_fit(samples, ctx, degree, degree_e=degree_e)
    corrected = []
    for s in samples:
        ex, ey = s.get('ex_px'), s.get('ey_px')
        if ex is None or ey is None:
            return raw                    # 老数据没记残余，不回正
        ex, ey = float(ex), float(ey)
        ay, by, ae, be = _jacobian_deg_per_px(raw, s['u'], s['v'], ctx)
        c = dict(s)
        # 收工还差 (ex, ey) 像素没走到：按局部雅可比折成角度补回去
        c['d_yaw_c'] = s['d_yaw'] + (ay * ex + by * ey)
        c['d_ext_c'] = s['d_ext'] + (ae * ex + be * ey)
        corrected.append(c)
    fit = _lstsq_fit(corrected, ctx, degree, key_y='d_yaw_c', key_e='d_ext_c',
                     degree_e=degree_e)
    # 离群剔除：手抖、HOME 没停稳、检测偶尔跳一下，都会让个别组差出十几度
    # （55 组里有 5~6 组 ext 差 30~90°，全是明显采坏的）。多项式项数一多，
    # 一个坏点能拉走一整片，所以按 3σ 剔三轮。
    #
    # 阈值取 max(下限, 3σ)：下限 3°/15° 是**采样本身的精度**（收工死区 ±5px ×
    # 局部 1.6°/10° 每像素，再加 HOME 重复性 1°）。没有下限的话，模型越准 σ
    # 越小、阈值跟着收紧，最后把画面边缘那些"只是噪声大"的点也当成坏点扔掉——
    # 那些点恰恰是运行时最需要映射准的位置。
    kept_idx = list(range(len(corrected)))
    lim_y = lim_e = 0.0
    for _ in range(3):
        rows = [corrected[i] for i in kept_idx]
        ry, re = [], []
        for s in rows:
            py, pe = _predict(fit, s['u'], s['v'], ctx)
            ry.append(py - s['d_yaw_c'])
            re.append(pe - s['d_ext_c'])
        sy = (sum(v * v for v in ry) / len(ry)) ** 0.5
        se = (sum(v * v for v in re) / len(re)) ** 0.5
        lim_y, lim_e = max(3.0, 3.0 * sy), max(15.0, 3.0 * se)
        new_idx = [i for i, a, b in zip(kept_idx, ry, re)
                   if abs(a) <= lim_y and abs(b) <= lim_e]
        if len(new_idx) == len(kept_idx) or len(new_idx) < 0.6 * len(corrected):
            break
        kept_idx = new_idx
        fit = _lstsq_fit([corrected[i] for i in kept_idx], ctx, degree,
                         key_y='d_yaw_c', key_e='d_ext_c', degree_e=degree_e)
    dropped = [corrected[i] for i in range(len(corrected)) if i not in kept_idx]

    fit['refine'] = {'applied': True,
                     'before_residual_deg': raw['residual_deg'],
                     'before_residual_px': raw['residual_px']}
    if dropped:
        fit['trimmed'] = {
            'dropped': [[round(s['u'], 1), round(s['v'], 1)] for s in dropped],
            'kept': len(corrected) - len(dropped),
            'total': len(corrected),
            'limits_deg': [round(lim_y, 2), round(lim_e, 2)],
        }
    return fit


def _save_calibration(path: Path, args, ctx, home: list, samples: list,
                      fit: dict | None, cover: dict):
    doc = {
        'format_version': 1,
        'note': '像素 → 云台角 标定结果。运行时先回 HOME，再按 pixel 查表直接发 '
                'gimbal 绝对目标；Δ 是相对 HOME 的增量。',
        'color': int(args.color),
        'home': {'yaw_deg': round(home[0], 2),
                 'lift_deg': round(home[1], 2),
                 'extend_deg': round(home[2], 2)},
        'image': {'width': int(ctx['frame_w']), 'height': int(ctx['frame_h']),
                  'cam_rotation': int(args.cam_rotation),
                  'processing_scale': float(args.processing_scale)},
        'centering': {'deadband_px': float(args.calib_deadband_px),
                      'stable': int(args.calib_stable)},
        'grid': {'cols': int(args.calib_grid_cols),
                 'rows': int(args.calib_grid_rows),
                 'cover': {f'{r},{c}': int(n) for (r, c), n in sorted(cover.items())}},
        'samples': samples,
        'fit': fit,
    }
    path.write_text(json.dumps(doc, ensure_ascii=False, indent=1), encoding='utf-8')


def run_calibration(args, camera, node, color_doc, ctx) -> int:
    """--calib 模式入口：先占住云台控制权（手柄让位），再跑采集主循环。

    不占的话 joy_control 会 100Hz 补零速，把跟随速度冲成"一顿一顿"。
    """
    with ControlHold(node):
        return _calibration_loop(args, camera, node, color_doc, ctx)


def _calibration_loop(args, camera, node, color_doc, ctx) -> int:
    """--calib 模式主循环：采 N 组样本 → 拟合 → 存 config/plane_mapping.json。"""
    import cv2
    home = _read_gimbal(node, timeout_s=3.0)
    if home is None:
        print('[calib] 收不到 /gimbal_state，云台没上电或 f407_bridge 没跑', flush=True)
        return 1
    if args.calib_home_yaw is not None:
        home[0] = float(args.calib_home_yaw)
    if args.calib_home_lift is not None:
        home[1] = float(args.calib_home_lift)
    if args.calib_home_ext is not None:
        home[2] = float(args.calib_home_ext)
    ctx['home'] = home

    out = Path(args.calib_out)
    print(f'[calib] HOME（检测视角）= yaw {home[0]:+.1f}  lift {home[1]:+.1f}  '
          f'ext {home[2]:+.1f}', flush=True)
    print('[calib] 先用这个姿态把相机对准工作区；采样期间不要用手柄碰云台。',
          flush=True)
    print('[calib] 已占住云台控制权（/motion_active 心跳 10Hz），手柄不会干扰。',
          flush=True)
    print(f'[calib] 物料颜色 {args.color}（{COLOR_NAMES.get(args.color, "?")}），'
          f'目标 {args.calib} 组，按 q/Esc 提前结束。', flush=True)
    print(f'[calib] 跟随沿用 track_pid.json：kp_yaw={args.kp_yaw:.2f} '
          f'kp_extend={args.kp_extend:.2f} '
          f'max={args.max_yaw_deg_s:.0f}/{args.max_extend_deg_s:.0f}°/s '
          f'sign={args.yaw_sign:+.0f}/{args.ext_sign:+.0f}；'
          f'标定死区 {args.calib_deadband_px:.1f}px × {args.calib_stable} 帧',
          flush=True)

    samples, cover = [], {}
    last_uv = None
    attempts = 0
    max_attempts = max(args.calib * 3, args.calib + 10)

    while len(samples) < args.calib and attempts < max_attempts:
        attempts += 1
        if not _move_home(node, home):
            print(f'[calib] 回 HOME 超时（当前 yaw={node._gimbal[0]:+.1f} '
                  f'lift={node._gimbal[1]:+.1f} ext={node._gimbal[2]:+.1f}），继续',
                  flush=True)
        time.sleep(max(0.0, args.calib_home_hold_s))
        if _key_stop(args):
            break

        done = len(samples)
        print(f'[calib] 第 {done + 1}/{args.calib} 组：请把物料放到画面里并保持静止'
              '（从 HOME 视角能看到）…', flush=True)
        uv0 = _wait_still_at_home(camera, args, color_doc, ctx, cover, done,
                                  args.calib_still_s, args.calib_wait_s)
        # 物料还停在上一组的采样点：提示用户挪开，别在原地重复采（拟合用不上）
        while (uv0 is not None and last_uv is not None
               and _dist(uv0, last_uv) < args.calib_move_px):
            print('[calib]   物料还停在上一组的采样点，请挪到画面里别的格子…',
                  flush=True)
            if not _wait_material_moved(camera, args, color_doc, ctx, cover, done,
                                        uv0, args.calib_move_timeout_s):
                uv0 = None
                break
            uv0 = _wait_still_at_home(camera, args, color_doc, ctx, cover, done,
                                      args.calib_still_s, args.calib_wait_s)
        if uv0 is None:
            if _key_stop(args):
                break
            print('[calib] 等不到静止的物料（或超出等待上限），本组跳过', flush=True)
            continue
        print(f'[calib]   像素 ({uv0[0]:.0f}, {uv0[1]:.0f})，开始跟随对中…',
              flush=True)

        result = _track_to_center(camera, node, args, color_doc, ctx)
        if not result.get('ok'):
            print(f'[calib]   跟随失败（{result.get("reason", "?")}），本组作废',
                  flush=True)
            continue

        # 复核：回 HOME 再看一眼，像素应该还在原处（物料在跟随期间没被挪动）
        _move_home(node, home)
        time.sleep(max(0.0, args.calib_home_hold_s))
        uv1 = _wait_still_at_home(camera, args, color_doc, ctx, cover, done,
                                  args.calib_still_s, args.calib_wait_s)
        if uv1 is None:
            print('[calib]   复核时看不到物料（超出 HOME 视野？），本组作废',
                  flush=True)
            continue
        drift = ((uv1[0] - uv0[0]) ** 2 + (uv1[1] - uv0[1]) ** 2) ** 0.5
        if drift > args.calib_verify_px:
            print(f'[calib]   物料在跟随期间移动了 {drift:.0f}px，本组作废'
                  '（跟随过程里别碰物料；把它挪个位置继续）', flush=True)
            _wait_material_moved(camera, args, color_doc, ctx, cover, done,
                                 uv1, args.calib_move_timeout_s)
            continue

        pose = [float(result['yaw_deg']), float(result['lift_deg']),
                float(result['extend_deg'])]
        s = {
            'u': round(uv0[0], 2), 'v': round(uv0[1], 2),
            'd_yaw': round(pose[0] - home[0], 3),
            'd_ext': round(pose[2] - home[2], 3),
            'yaw_deg': round(pose[0], 2),
            'lift_deg': round(pose[1], 2),
            'extend_deg': round(pose[2], 2),
            'ex_px': result.get('ex_px'), 'ey_px': result.get('ey_px'),
        }
        samples.append(s)
        cell = _calib_cell(ctx, uv0[0], uv0[1],
                           args.calib_grid_cols, args.calib_grid_rows)
        cover[cell] = cover.get(cell, 0) + 1
        print(f'[calib]   ✓ 第 {len(samples)} 组：像素 ({s["u"]:.0f},{s["v"]:.0f}) '
              f'→ Δyaw {s["d_yaw"]:+.2f}°  Δext {s["d_ext"]:+.2f}°  '
              f'（残余 {s["ex_px"]:+.1f},{s["ey_px"]:+.1f}px）'
              f'  格 R{cell[0] + 1}C{cell[1] + 1}', flush=True)
        _save_calibration(out, args, ctx, home, samples, None, cover)
        last_uv = uv0

        if len(samples) >= args.calib:
            break
        # 现在人在 HOME 视角、物料就在 uv0：下一组得先把它挪走。
        # 默认弹窗问一句，顺便告诉用户还剩几组；--calib-no-dialog 时退回
        # 自动盯画面等物料移动。
        go_on = None
        if not args.calib_no_dialog:
            _show_hint(camera, args, color_doc, ctx, cover, len(samples))
            go_on = _ask_next_group(len(samples), args.calib,
                                    args.calib_grid_cols, args.calib_grid_rows)
        if go_on is False:
            print('[calib] 用户结束采集，开始拟合保存', flush=True)
            break
        if go_on is None:
            # 没有桌面环境（或 --calib-no-dialog）：自动等物料挪走
            if not args.calib_no_dialog and not getattr(args, '_dlg_warned', False):
                args._dlg_warned = True
                print('[calib] 弹窗用不了（没有 DISPLAY 或 tkinter 不可用），'
                      '改成自动盯画面等物料挪走', flush=True)
            if not _wait_material_moved(camera, args, color_doc, ctx, cover,
                                        len(samples), uv0,
                                        args.calib_move_timeout_s):
                if _key_stop(args):
                    break
                print('[calib] 物料一直没挪动，结束采集', flush=True)
                break

    if not samples:
        print('[calib] 一组都没采到，不写文件', flush=True)
        return 1

    fit = (_fit_plane_mapping(samples, ctx, degree=3, degree_e=2)
           if len(samples) >= 6 else None)
    _save_calibration(out, args, ctx, home, samples, fit, cover)
    print(f'[calib] 已保存 {len(samples)} 组到 {out}', flush=True)
    if fit is None:
        print(f'[calib] 样本太少（{len(samples)} < 6），没拟合，先看 samples。',
              flush=True)
        return 0
    print(f'[calib] 拟合阶数：yaw {fit["degree"]} 次 / ext {fit["ext_degree"]} 次',
          flush=True)
    print(f'[calib] 拟合残差：yaw {fit["residual_deg"]["yaw_rms"]:.2f}° '
          f'(max {fit["residual_deg"]["yaw_max"]:.2f}°)，'
          f'ext {fit["residual_deg"]["ext_rms"]:.2f}° '
          f'(max {fit["residual_deg"]["ext_max"]:.2f}°)，'
          f'≈ {fit["residual_px"]["median"]}px 中位 / '
          f'{fit["residual_px"]["p90"]}px P90', flush=True)
    ref = fit.get('refine') or {}
    if ref.get('applied'):
        # 每组收工时还差那几像素（死区 + 整数 RPM 截断），用记录下来的残余
        # 把采样姿态回正后再拟合，这一遍才是"一步到位"的映射
        print(f'[calib] 已做残余回正拟合：回正前 ≈ '
              f'{ref["before_residual_px"]["median"]}px 中位 → 现在 '
              f'{fit["residual_px"]["median"]}px 中位', flush=True)
    print(f'[calib] 网格覆盖 {len(cover)}/{args.calib_grid_cols * args.calib_grid_rows} 格',
          flush=True)
    return 0


def _run_server(args, camera, node, color_doc, ctx) -> int:
    """常驻模式：摄像头只开这一次，之后按 stdin 请求反复跟随。

    协议是一行一个 JSON（stdin 请求，stdout 回复）：
      请求 {"cmd":"pick","color":1,"timeout_s":30,"still_time":0}
      请求 {"cmd":"debug","color":1,"timeout_s":600}   纯调参：一直跟到 close/超时
      请求 {"cmd":"close"}
      回复 {"ready":true}   —— 摄像头开好、可以 pick 了
           {"ok":true,...} —— 一次 pick 的结果，字段和一次性模式完全一样

    这样 motion_planner 里连着写 color_pick 1、color_pick 2 时，摄像头、
    ROS 节点、畸变映射全都不用重新初始化，省掉每次三四秒的启动时间。

    没有请求时也不闲着：持续取帧刷预览 + 排空摄像头缓冲，画面实时，pick
    第一帧就是新画面（少了历史缓冲的延迟）。
    """
    # 先贴一帧再报 ready：open 这条指令执行完，预览窗口里就应该有画面了
    _pump_preview(camera, args, ctx, '预热完成，等待 color_pick')
    print(json.dumps({'ready': True, 'device': args.device}), flush=True)

    stop = {'requested': False}

    def _stop_requested():
        """调试跟踪每帧调用：stdin 来了 close 就吃掉并返回 True。"""
        while _stdin_ready(0.0):
            line = sys.stdin.readline()
            if not line:                     # EOF：父进程没了
                stop['requested'] = True
                return True
            text = line.strip()
            if not text:
                continue
            try:
                req = json.loads(text)
            except ValueError:
                continue
            if str(req.get('cmd', '')).lower() in ('close', 'quit', 'exit'):
                stop['requested'] = True
                return True
        return stop['requested']

    while True:
        if not _stdin_ready(0.0):
            rclpy.spin_once(node, timeout_sec=0.0)   # 保持 /gimbal_state 新鲜
            _idle_preview(camera, args, ctx)
            continue

        line = sys.stdin.readline()
        if not line:                       # 父进程关管道 / close：正常退出
            break
        line = line.strip()
        if not line:
            continue
        try:
            request = json.loads(line)
        except ValueError:
            print(json.dumps({'ok': False, 'reason': f'请求不是 JSON：{line[:60]}'},
                             ensure_ascii=False), flush=True)
            continue
        action = str(request.get('cmd', 'pick')).strip().lower()
        if action in ('close', 'quit', 'exit'):
            print(json.dumps({'ok': True, 'closed': True}), flush=True)
            return 0
        if action not in ('pick', 'debug'):
            print(json.dumps({'ok': False, 'reason': f'未知请求：{action}'},
                             ensure_ascii=False), flush=True)
            continue
        # 每次请求前重读 track_pid.json 的可热改项：常驻进程只在启动时读一次，
        # 如果进程是 open 过一直复用的，上位机改完补偿量/PID 不重读就会"改了没
        # 生效"，看起来像修正动作时有时无。只刷 _LIVE_TUNABLE（不含 cam_rotation
        # 这类影响畸变映射的几何量，ctx 已经算好了）。
        _reload_live_config(args, HERE)
        try:
            color = int(request.get('color', 0))
        except (TypeError, ValueError):
            color = 0
        if not any(int(c['color_id']) == color for c in color_doc['colors']):
            print(json.dumps({'ok': False, 'color': color,
                              'reason': f'颜色 {color} 没有标定配置'},
                             ensure_ascii=False), flush=True)
            continue

        # 本次请求可以临时覆盖总超时 / 静止判据（序列里按需给）
        args.color = color
        old_timeout, old_still = args.timeout, args.still_time
        try:
            args.timeout = float(request.get('timeout_s', old_timeout))
        except (TypeError, ValueError):
            pass
        try:
            args.still_time = float(request.get('still_time', old_still))
        except (TypeError, ValueError):
            pass
        debug = action == 'debug'
        print(f'[gimbal_track_pick] {"debug" if debug else "pick"} color={color}',
              flush=True)
        try:
            result = track_color(camera, node, args, color_doc, ctx, color,
                                 debug=debug, should_stop=_stop_requested)
        except Exception as error:         # noqa: BLE001 - 一次失败不能带走常驻进程
            # 打印完整堆栈：只有 error 字符串的话，日志里看不出是代码哪一行崩的
            import traceback
            traceback.print_exc()
            result = {'ok': False, 'color': color, 'reason': f'跟随异常：{error}'}
        finally:
            args.timeout, args.still_time = old_timeout, old_still
        print(json.dumps(result, ensure_ascii=False), flush=True)
        if stop['requested']:
            print(json.dumps({'ok': True, 'closed': True}), flush=True)
            return 0
        # 贴一帧"待机"画面（之后空闲循环会继续实时刷新）
        _pump_preview(camera, args, ctx, '等待下一条 color_pick')
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='云台跟随颜色物料（color_pick 视觉端）')
    parser.add_argument('--color', type=int, default=None,
                        help='颜色编号 1~6（--server 模式下由 stdin 下发，可不填）')
    parser.add_argument('--server', action='store_true',
                        help='常驻模式：开一次摄像头，之后从 stdin 反复收 pick/close，'
                             '省掉每次重新初始化的三四秒')
    parser.add_argument('--device', default='/dev/video0')
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--fps', type=float, default=30.0)
    parser.add_argument('--timeout', type=float, default=1000.0, help='跟随总超时 s')
    parser.add_argument('--tol-px', type=float, default=25.0, help='认为对准的像素容差')
    parser.add_argument('--stable', type=int, default=8, help='连续多少帧对准才收工')
    parser.add_argument('--min-area', type=float, default=800.0, help='物料最小像素面积')
    # —— 形状筛选（移植自旧 gimbal_track 检测节点；0 = 关闭该项，默认全关只查面积）——
    parser.add_argument('--processing-scale', type=float, default=0.5,
                        help='半分辨率处理倍率（提速），坐标自动换算回原图')
    parser.add_argument('--open-kernel-px', type=int, default=5, help='开运算核（全分辨率像素）')
    parser.add_argument('--close-kernel-px', type=int, default=11, help='闭运算核（填色块内部孔洞）')
    parser.add_argument('--max-area', type=float, default=0.0, help='最大像素面积（0=不限）')
    parser.add_argument('--min-dimension-px', type=float, default=10.0,
                        help='外接框最短边下限（全分辨率像素）')
    parser.add_argument('--max-dimension-px', type=float, default=0.0,
                        help='外接框最长边上限（0=不限）')
    parser.add_argument('--max-aspect', type=float, default=0.0,
                        help='最长/最短边比上限（0=不查长宽比）')
    parser.add_argument('--min-solidity', type=float, default=0.0,
                        help='实心度下限 轮廓面积/凸包面积（0=不查）')
    parser.add_argument('--min-extent', type=float, default=0.0,
                        help='占比下限 轮廓面积/外接框面积（0=不查）')
    # —— 速度环跟随：像素误差 → 角速度 °/s ——
    parser.add_argument('--kp-yaw', type=float, default=0.2,
                        help='yaw 环 P（px → °/s），越大跟得越猛')
    parser.add_argument('--ki-yaw', type=float, default=0.0)
    parser.add_argument('--kd-yaw', type=float, default=0.0)
    parser.add_argument('--kp-extend', type=float, default=0.8,
                        help='extend 环 P（px → °/s）')
    parser.add_argument('--ki-extend', type=float, default=0.0)
    parser.add_argument('--kd-extend', type=float, default=0.0)
    parser.add_argument('--max-yaw-deg-s', type=float, default=120.0,
                        help='yaw 速度限幅 °/s')
    parser.add_argument('--max-extend-deg-s', type=float, default=300.0,
                        help='extend 速度限幅 °/s')
    parser.add_argument('--deadband-px', type=float, default=10.0,
                        help='误差小于这个像素数就停（也算对准）')
    parser.add_argument('--deriv-tau', type=float, default=0.05,
                        help='微分低通时间常数 s，越大越平滑')
    parser.add_argument('--yaw-sign', type=float, default=1.0, choices=[1.0, -1.0],
                        help='yaw 速度方向反了改成 -1')
    parser.add_argument('--ext-sign', type=float, default=1.0, choices=[1.0, -1.0],
                        help='extend 速度方向反了改成 -1')
    # —— 跟随目标像素 ——
    parser.add_argument('--target-x', type=float, default=None,
                        help='物料最终停下的像素 X（默认画面中心）。'
                             '填"夹爪降下去正好夹住物料"时物料所在的像素')
    parser.add_argument('--target-y', type=float, default=None,
                        help='物料最终停下的像素 Y（默认画面中心）')
    # —— 画面约束 ——
    parser.add_argument('--roi-bottom', type=float, default=0.9,
                        help='只在画面下半这部分找物料：取值 0~1 = 从底部往上占画面比例'
                             '（圆盘大、只够得到最近物料时用；1.0 = 全画面）')
    parser.add_argument('--cam-rotation', type=int, default=180,
                        choices=[0, 90, 180, 270],
                        help='摄像头安装旋转角：倒装 180、正装 0、侧装 90/270。'
                             '旋转在矫正之后做，预览/ROI/误差方向全部基于旋转后的画面')
    # —— 静止前置：先等物料停住再跟随（转盘场景：物料转到抓取位会停一下）——
    parser.add_argument('--still-time', type=float, default=0.3,
                        help='先等物料持续不动这么久，再开始跟随（秒；0 = 不等待'
                             '直接跟随，即动态抓取模式，转盘不停转时才用）')
    parser.add_argument('--still-px', type=float, default=8.0,
                        help='判定"不动"的像素抖动阈值')

    parser.add_argument('--headless', action='store_true', help='不开预览窗口（无显示器环境）')

    # —— 平面映射标定（--calib）：像素 → (Δyaw, Δext)，标好之后跟踪不用 PID ——
    parser.add_argument('--calib', type=int, default=0,
                        help='标定模式：采 N 组样本后拟合并存 plane_mapping.json（0=不标定）')
    parser.add_argument('--calib-out', default=str(HERE / 'config' / 'plane_mapping.json'),
                        help='标定结果输出路径')
    parser.add_argument('--calib-home-yaw', type=float, default=None,
                        help='覆盖 HOME 的 yaw（默认=程序启动时云台所在的角）')
    parser.add_argument('--calib-home-lift', type=float, default=None,
                        help='覆盖 HOME 的 lift（默认=启动时的角）')
    parser.add_argument('--calib-home-ext', type=float, default=None,
                        help='覆盖 HOME 的 ext（默认=启动时的角）')
    parser.add_argument('--calib-home-hold-s', type=float, default=0.8,
                        help='回 HOME 到位后再等多久让画面稳定（秒）')
    parser.add_argument('--calib-still-s', type=float, default=0.6,
                        help='判定物料静止的窗口（秒）')
    parser.add_argument('--calib-still-px', type=float, default=4.0,
                        help='静止窗口内质心抖动上限（像素）')
    parser.add_argument('--calib-wait-s', type=float, default=120.0,
                        help='单次等物料静止/出现的上限（秒）')
    parser.add_argument('--calib-move-px', type=float, default=40.0,
                        help='物料离开采样点多少像素算"用户挪动了"')
    parser.add_argument('--calib-move-timeout-s', type=float, default=180.0,
                        help='等用户挪动物料的上限（秒），超时也继续下一组')
    parser.add_argument('--calib-verify-px', type=float, default=8.0,
                        help='回 HOME 复核时像素漂移超过这个值 → 本组作废')
    parser.add_argument('--calib-deadband-px', type=float, default=5.0,
                        help='标定时对中的像素死区。别设小于 5：F407 速度是整数 RPM'
                             '（1 RPM=6°/s），yaw 环 kp≈1.4 时 <4.4px 的误差指令会被'
                             '截断成 0，云台根本停不进更小的死区（表现为一直 timeout）')
    parser.add_argument('--calib-stable', type=int, default=6,
                        help='标定时连续多少帧进死区才收工')
    parser.add_argument('--calib-track-timeout-s', type=float, default=25.0,
                        help='单组跟随的超时（秒）')
    parser.add_argument('--calib-grid-cols', type=int, default=6)
    parser.add_argument('--calib-grid-rows', type=int, default=4)
    parser.add_argument('--calib-no-dialog', action='store_true',
                        help='每组结束不弹确认框（改成等画面里物料被挪走再自动继续）')

    return parser


def build_args(argv=None):
    """给别的程序（vision_node）复用同一套默认参数。

    不打开摄像头、不初始化 ROS，只把命令行默认值和 track_pid.json 合并好。
    """
    parser = build_parser()
    args = parser.parse_args(argv if argv is not None else [])
    return _apply_track_config(args, HERE)


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    args = _apply_track_config(args, HERE)   # 上位机可改 config/track_pid.json 调 PID/方向/旋转

    if args.color is None and not args.server and args.calib <= 0:
        parser.error('--color 必填（--server 模式下由 stdin 下发，--calib 标定必填）')

    import cv2

    color_doc = load_color_config(HERE / 'config' / 'material_color_thresholds.json')
    if not args.server and not any(int(c['color_id']) == args.color
                                   for c in color_doc['colors']):
        print(json.dumps({'ok': False, 'reason': f'颜色 {args.color} 没有标定配置'}),
              flush=True)
        return 1

    rclpy.init()
    node = GimbalTrackPick(args)

    camera = _open_camera(args)
    if camera is None:
        print(json.dumps({'ok': False, 'reason': f'打不开摄像头 {args.device}'}),
              flush=True)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    if not args.headless:
        try:
            cv2.namedWindow('gimbal_track_pick', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('gimbal_track_pick', 960, 540)
        except cv2.error as error:
            # 通过 SSH 跑 run_car 时没有 DISPLAY，开预览窗口会直接抛异常，
            # 整个脚本没来得及打印 JSON 就退出 → color_pick 看起来"没反应"。
            # 这种情况自动退回 headless，只识别/跟随，不显示画面。
            print(f'[gimbal_track_pick] 没有图形界面，自动 headless：{error}',
                  flush=True)
            args.headless = True

    ctx = _prepare_frame_ctx(args)
    try:
        if args.calib > 0:
            return run_calibration(args, camera, node, color_doc, ctx)
        if args.server:
            return _run_server(args, camera, node, color_doc, ctx)
        result = track_color(camera, node, args, color_doc, ctx, int(args.color))
        print(json.dumps(result, ensure_ascii=False), flush=True)
        return 0 if result.get('ok') else 1
    finally:
        camera.release()
        if not args.headless:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
