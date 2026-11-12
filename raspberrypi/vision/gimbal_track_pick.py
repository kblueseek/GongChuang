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
  --kp-yaw / --kp-extend          像素误差 → 角速度 °/s 的比例（越大跟得越猛，太大振荡）
  --yaw-sign / --ext-sign         轴方向（±1）
  --max-yaw-deg-s / --max-extend-deg-s  速度限幅
  --cam-rotation                  摄像头安装旋转角（180=倒装）
  --ext-offset-deg                对中后再前伸多少度（相机→夹爪的机械偏移）。
                                  只随结果 JSON 回传，由 motion_planner 在 color_pick
                                  结束后自动执行；摄像头改装后只改这个数即可。
"""

from __future__ import annotations

import argparse
import json
import select
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))          # 颜色标定 等同目录模块
from 颜色标定 import load_color_config  # noqa: E402

COLOR_NAMES = {1: '红', 2: '黄', 3: '蓝', 4: '绿', 5: '黑', 6: '浅蓝'}


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
    """与 vision/main.py / ring_fix 同一套：camera_processing.json + 标定 yaml。"""
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
        self.create_subscription(Float32MultiArray, '/gimbal_state',
                                 self._cb_gimbal_state, 10)
        self._gimbal = [0.0, 0.0, 0.0]
        self._gimbal_ok = False

    def _cb_gimbal_state(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            self._gimbal = [float(v) for v in msg.data[:3]]
            self._gimbal_ok = True

    def send_gimbal_vel(self, yaw: float, lift: float, extend: float):
        self._pub_vel.publish(Float32MultiArray(data=[yaw, lift, extend]))


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
    for contour in contours:
        m0 = cv2.moments(contour)
        if abs(m0['m00']) < 1e-6:
            continue
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
    'yaw_sign', 'ext_sign', 'ext_offset_deg', 'stable', 'still_px',
)


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
    return {
        'rotate': rotate, 'map1': map1, 'map2': map2,
        'frame_w': frame_w, 'frame_h': frame_h,
        'cx': frame_w / 2.0, 'cy': frame_h / 2.0,
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
            if debug:
                if should_stop is not None and should_stop():
                    result = {'ok': True, 'color': args.color,
                              'debug': True, 'reason': 'stopped'}
                    break
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
                                  'ext_offset_deg': round(float(args.ext_offset_deg), 2)}
                        print(f'[gimbal_track_pick] 对准收工：{status}', flush=True)
                        break
            else:
                stable = 0
                hist.clear()
                pid_yaw.reset()
                pid_extend.reset()
                status = f'SEARCH color={args.color}({COLOR_NAMES[args.color]}) area={area:.0f}'

            # 每帧都发速度：跟踪时发 PID 输出，等停/丢目标/死区时发 0，
            # 既保证 F407 速度环 300ms 超时不会停得一顿一顿，也保证收工瞬间立即停住。
            node.send_gimbal_vel(yaw_vel, 0.0, ext_vel)

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


def main() -> int:
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
    parser.add_argument('--ext-offset-deg', type=float, default=0.0,
                        help='对中后自动前伸补偿 °（相机→夹爪机械偏移，0=不补偿）')
    parser.add_argument('--yaw-sign', type=float, default=1.0, choices=[1.0, -1.0],
                        help='yaw 速度方向反了改成 -1')
    parser.add_argument('--ext-sign', type=float, default=1.0, choices=[1.0, -1.0],
                        help='extend 速度方向反了改成 -1')
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
    args = parser.parse_args()
    args = _apply_track_config(args, HERE)   # 上位机可改 config/track_pid.json 调 PID/方向/旋转

    if args.color is None and not args.server:
        parser.error('--color 必填（--server 模式下由 stdin 下发）')

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
