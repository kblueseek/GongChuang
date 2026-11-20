#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""vision_put 的视觉端：把指定圆环的圆心对到画面的固定像素。

    python3 vision/vision_put_align.py --ring 0 --target-x 640 --target-y 360

做什么：
  1. 打开 USB 摄像头（和 gimbal_track_pick 同一套畸变矫正 / 安装旋转）；
  2. 每帧跑 vision/ring_station_detector.py 找圆环，取"离目标像素最近"或
     "指定序号"的那个环心；
  3. 像素误差 → PID → /cmd_gimbal_vel（yaw / extend），把环心推到目标像素；
  4. 连续若干帧进死区就停，最后一行打印 JSON 给 motion_planner。

为什么这样做闭环：
  不用识别物料。摄像头和夹爪的相对位置是固定的，所以"环心落在哪个像素"
  等价于"夹爪相对环心偏了多少"。把目标像素标定成"夹爪正对环心时的环心像素"，
  这个动作就能把盲放的误差修掉。目标像素在现场调 --target-x/--target-y 即可。

⚠ 与位置模式的关系：
  F407 收到 gimbal 角度目标后会让位置环占优 3s，但**收到非零速度帧会立即夺回
  控制权**（GimbalTask.c "速度指令夺回控制权"）。所以本脚本发速度就能马上接管，
  不用干等那 3 秒。
  保留一个 0.2s 的"已经不动"确认，是为了别在臂还在半路时就发速度——第一帧非零
  速度会把位置目标作废，臂会停在半路，视觉就从错误的位置开始找环。

方向、限幅、PID 默认值和已经在用的 color_pick 跟踪环（config/track_pid.json）
完全一致，所以第一次跑不用重标方向。

真正要现场定的只有一个量（也可写进 config/vision_put.json，下次启动自动读）：
  --target-x / --target-y    夹爪正对环心时，环心落在画面哪个像素
                             用 --dry-run 把臂摆正后读出来填即可

选环规则：第一帧取离目标像素最近的那个环并**锁定**；之后只跟同一个环。
新观测离上次环心超过 --lock-radius-px 就视为该环丢失，原地等它回来，
不会切到画面里旁边的环（否则云台会来回乱晃）。
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))          # ring_station_detector 同目录
from ring_station_detector import (    # noqa: E402
    RingStationDetectionError,
    RingStationDetector,
    load_ring_station_config,
)


class PID:
    """带输出限幅、积分抗饱和、微分低通的小 PID。

    和 gimbal_track_pick.py 里的实现**逐行一致**——color_pick 调好的增益能
    直接拿来用，就是靠这一点；两边任何一处改了算法，参数就不再等价。
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


class PutAlignNode(Node):
    def __init__(self):
        super().__init__('vision_put_align')
        # 和 gimbal_track_pick 一样走速度模式：F407 的位置模式优先级高，
        # 所以我们只在它的 3s 保持过期之后才发速度（见 --hold-wait-s）。
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


def build_undistort_maps(width: int, height: int):
    """和 vision/main.py / gimbal_track_pick 同一套畸变矫正配置。"""
    import cv2
    proc_path = HERE / 'config' / 'camera_processing.json'
    try:
        proc = json.loads(proc_path.read_text(encoding='utf-8'))
    except Exception:  # noqa: BLE001 - 没配置就用原始画面
        return None, None
    if not proc.get('undistort_enabled'):
        return None, None
    storage = cv2.FileStorage(str(HERE / 'config' / 'realtek_rgb_camera.yaml'),
                              cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        return None, None
    size = (int(storage.getNode('image_width').real()),
            int(storage.getNode('image_height').real()))
    if size != (width, height):
        storage.release()
        return None, None
    cam = storage.getNode('camera_matrix').mat()
    dist = storage.getNode('distortion_coefficients').mat()
    newc = storage.getNode('new_camera_matrix').mat()
    storage.release()
    if cam is None or dist is None or newc is None:
        return None, None
    return cv2.initUndistortRectifyMap(cam, dist, None, newc, size, cv2.CV_16SC2)


def _apply_config(args, provided):
    """读 config/vision_put.json（可选）填默认值。

    **命令行上显式写了的参数优先**：序列里写了 target_x 就不该被 JSON 盖掉，
    否则改一次 profile 会把所有动作的目标像素一起改掉。provided 是命令行里
    出现过的选项名集合（--target-x 这种形式）。
    """
    path = HERE / 'config' / 'vision_put.json'
    if not path.is_file():
        return args
    try:
        doc = json.loads(path.read_text(encoding='utf-8'))
    except Exception as error:  # noqa: BLE001
        print(f'[vision_put_align] vision_put.json 解析失败，用默认值：{error}',
              flush=True)
        return args
    for key, value in doc.items():
        if not hasattr(args, key):
            continue
        if '--' + key.replace('_', '-') in provided:
            continue
        setattr(args, key, value)
    return args


def _select_ring(centers, ring: int, target):
    """挑要对的环：指定序号优先；序号不可用/为 0 时取离目标像素最近的那个。

    探测器的 centers 按 x 升序（左→右），所以 ring=1/2/3 就是左/中/右。
    只看到一个环时序号必然越界，自动退化成"最近"，不会报错。
    """
    if not centers:
        return -1, None
    if 1 <= ring <= len(centers):
        return ring - 1, centers[ring - 1]
    best = min(range(len(centers)),
               key=lambda i: math.hypot(centers[i][0] - target[0],
                                        centers[i][1] - target[1]))
    return best, centers[best]


def _select_predicted(predicted, target, locked_ref, max_distance_px):
    """从完整三环布局预测里选"意图环"。

    第一帧取离目标像素最近、且**不超过 max_distance_px** 的那个预测环心，
    锁定它的**参考环号**；之后一直用同一个环号的预测位置（哪怕这个环本身
    被物料挡住，位置是从旁边环推出来的）。

    距离上限很关键：画面里其他工位/物料/机构也会被检成"环"，没有上限时
    它们可能比真正的目标环更靠近目标像素，一锁就错。
    返回 (ref_index, center, locked_ref)；没有可用预测时 center=None。
    """
    if not predicted or len(predicted) != 3:
        return None, None, locked_ref
    if not any(p is not None for p in predicted):
        return None, None, locked_ref
    if locked_ref is not None:
        if predicted[locked_ref] is not None:
            if math.dist(predicted[locked_ref], target) <= max_distance_px:
                return locked_ref, predicted[locked_ref], locked_ref
            # 锁定环跑得比距离上限还远：锁错了/已经飞了，解锁重选
            locked_ref = None
        else:
            # 锁定环推不出来：原地等，不切到旁边的环
            return locked_ref, None, locked_ref
    candidates = [
        (i, p) for i, p in enumerate(predicted)
        if p is not None and math.dist(p, target) <= max_distance_px
    ]
    if not candidates:
        return None, None, locked_ref
    index, center = min(candidates, key=lambda item: math.dist(item[1], target))
    return index, center, index


def _select_locked(centers, ring: int, target, locked, lock_radius_px: float,
                   max_distance_px: float):
    """第一次取最近；之后只跟踪同一个环，跟丢就停住等它回来。

    只按"离目标像素最近"每帧重选的话，画面里同时出现两三个环时，噪声会让
    选择在两环之间来回跳，云台就跟着乱晃。锁定后只认离上次环心最近的观测，
    距离超过 lock_radius_px 视为这个环丢了（被夹爪挡住/出画面）——此时不动，
    绝不切到旁边的环。

    第一次选择额外受 max_distance_px 限制：离目标像素太远的"环"不可能是
    夹爪下面那个，不理它。

    返回 (index, center, locked)；center=None 表示这一帧不该动。
    """
    if locked is not None:
        if not centers:
            return -1, None, locked
        distances = [math.dist(c, locked) for c in centers]
        best = min(range(len(distances)), key=distances.__getitem__)
        if distances[best] <= lock_radius_px:
            return best, centers[best], centers[best]
        # 锁定环丢失：不重新选别的环，保持锁定，等它重新出现
        return -1, None, locked
    index, center = _select_ring(centers, ring, target)
    if center is not None and math.dist(center, target) > max_distance_px:
        return -1, None, locked
    return index, center, center


def main() -> int:
    parser = argparse.ArgumentParser(description='vision_put 视觉端：圆环圆心对像素')
    parser.add_argument('--lock-radius-px', type=float, default=150.0,
                        help='锁定同一个环的最大像素偏移；超过就认为该环丢失并'
                             '原地等，不会切到旁边别的环')
    parser.add_argument('--max-target-distance-px', type=float, default=300.0,
                        help='第一次选环时，环心离目标像素的最大允许距离；'
                             '超过就不理它（画面里其他工位的环/物料太远，'
                             '误锁会直接跑偏）')
    parser.add_argument('--ring', type=int, default=0,
                        help='要对的环号 1~3（左到右）；0=取离目标像素最近的')
    parser.add_argument('--target-x', type=float, default=None,
                        help='目标像素 x，默认画面中心')
    parser.add_argument('--target-y', type=float, default=None,
                        help='目标像素 y，默认画面中心')
    parser.add_argument('--device', default='/dev/video0')
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--fps', type=float, default=30.0)
    parser.add_argument('--cam-rotation', type=int, default=180,
                        choices=[0, 90, 180, 270])
    parser.add_argument('--timeout', type=float, default=30.0, help='总超时 s')
    parser.add_argument('--tol-px', type=float, default=10.0, help='对准容差（像素）')
    parser.add_argument('--deadband-px', type=float, default=8.0,
                        help='误差小于它就停（控制死区）')
    parser.add_argument('--stable', type=int, default=6, help='连续多少帧进死区算成功')
    # ---- 速度环：默认值和已经在用的 color_pick 跟踪环（track_pid.json）一致 ----
    parser.add_argument('--kp-yaw', type=float, default=1.0)
    parser.add_argument('--ki-yaw', type=float, default=0.01)
    parser.add_argument('--kd-yaw', type=float, default=0.02)
    parser.add_argument('--kp-extend', type=float, default=4.99)
    parser.add_argument('--ki-extend', type=float, default=0.01)
    parser.add_argument('--kd-extend', type=float, default=0.015)
    parser.add_argument('--max-yaw-deg-s', type=float, default=780.0)
    parser.add_argument('--max-extend-deg-s', type=float, default=1155.0)
    parser.add_argument('--deriv-tau', type=float, default=0.05)
    parser.add_argument('--yaw-sign', type=float, default=-1.0, choices=[1.0, -1.0])
    parser.add_argument('--ext-sign', type=float, default=-1.0, choices=[1.0, -1.0])
    # ---- 起跑前只确认"臂已经到位"，不做额外等待 ----
    parser.add_argument('--settle-stable-s', type=float, default=0.2,
                        help='云台角度多久不变就算到位（0=不检查，立刻开始）')
    parser.add_argument('--settle-tol-deg', type=float, default=1.0,
                        help='角度变化小于它算没动')
    parser.add_argument('--config', default=str(HERE / 'config' /
                                                'ring_station_detector.json'))
    parser.add_argument('--dry-run', action='store_true',
                        help='只看不动：打印检测到的环心像素（标定 target 用）')
    parser.add_argument('--headless', action='store_true', help='不开预览窗口')
    parser.add_argument('--roi-fraction', type=float, default=0.65,
                        help='只看画面中心这么大比例的区域（0.65=中心65%%），'
                             '外面涂黑不参与检测；1.0=全画面')
    parser.add_argument('--save-frame', default='',
                        help='把第一帧识别画面（带观测/预测标记）存成图片，'
                             '用于夹着物料时远程排查；例：--save-frame /tmp/ring.jpg')

    args = parser.parse_args()
    # 只把命令行上真出现过的选项当作"已指定"，其余才让 profile 兜底
    provided = {token.split('=')[0] for token in sys.argv[1:]
                if token.startswith('--')}
    args = _apply_config(args, provided)

    import cv2

    try:
        config = load_ring_station_config(Path(args.config))
    except Exception as error:  # noqa: BLE001
        print(json.dumps({'ok': False,
                          'reason': f'圆环配置无效：{error}'}, ensure_ascii=False),
              flush=True)
        return 1

    rclpy.init()
    node = PutAlignNode()
    detector = RingStationDetector(config)

    camera = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not camera.isOpened():
        print(json.dumps({'ok': False, 'reason': f'打不开摄像头 {args.device}'},
                         ensure_ascii=False), flush=True)
        node.destroy_node()
        rclpy.shutdown()
        return 1
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    camera.set(cv2.CAP_PROP_FPS, args.fps)
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 4)
    for _ in range(10):
        ok, _frame = camera.read()
        if ok:
            break

    map1, map2 = build_undistort_maps(args.width, args.height)
    ROTATE = {0: None, 90: cv2.ROTATE_90_CLOCKWISE,
              180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}
    if args.cam_rotation in (90, 270):
        frame_w, frame_h = args.height, args.width
    else:
        frame_w, frame_h = args.width, args.height
    target_x = float(args.target_x if args.target_x is not None else frame_w / 2.0)
    target_y = float(args.target_y if args.target_y is not None else frame_h / 2.0)

    def _draw_overlay(img, det, centers, index, locked_ref, status):
        """把观测环、完整布局预测、目标像素和状态画到图上（预览/存图共用）。"""
        for i, c in enumerate(centers):
            color = (0, 255, 255) if i == index else (0, 255, 0)
            cv2.circle(img, (int(c[0]), int(c[1])), 8, color, 2)
            cv2.putText(img, str(i + 1), (int(c[0]) + 10, int(c[1]) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        # 红色 = 当前锁定/对齐的意图环；蓝色 = 推算出来的其他环
        for pi, p in enumerate(getattr(det, 'predicted_centers_px', ())):
            if p is None:
                continue
            pc = (int(round(p[0])), int(round(p[1])))
            is_target = locked_ref is not None and pi == locked_ref
            col = (0, 0, 255) if is_target else (200, 120, 0)
            cv2.drawMarker(img, pc, col, cv2.MARKER_TILTED_CROSS,
                           18 if is_target else 14, 2)
            cv2.putText(img, f"P{pi + 1}", (pc[0] + 8, pc[1] + 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55 if is_target else 0.45, col, 1)
        cv2.drawMarker(img, (int(target_x), int(target_y)),
                       (255, 255, 255), cv2.MARKER_CROSS, 30, 2)
        state = getattr(det, 'layout_source', '?')
        cv2.putText(img, f"{status}  pred={state}", (16, 34),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    if not args.headless:
        try:
            cv2.namedWindow('vision_put_align', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('vision_put_align', 960, 540)
        except cv2.error as error:
            print(f'[vision_put_align] 没有图形界面，自动 headless：{error}',
                  flush=True)
            args.headless = True

    pid_yaw = PID(args.kp_yaw, args.ki_yaw, args.kd_yaw,
                  args.max_yaw_deg_s, args.deriv_tau)
    pid_ext = PID(args.kp_extend, args.ki_extend, args.kd_extend,
                  args.max_extend_deg_s, args.deriv_tau)

    started = time.monotonic()
    last_t = time.monotonic()
    stable = 0
    settle_since = None          # 云台角度最后一次变化的时刻
    last_angles = None
    # 先确认臂已经走到盲放位置：还在走的时候取消位置目标会把臂停在半路。
    # 停下来的那一刻立刻取消保持、开始跟随，不多等。
    phase = 'settle' if (args.settle_stable_s > 0.0 and not args.dry_run) else 'servo'
    result = {'ok': False, 'reason': 'timeout'}
    frames = 0
    last_centers = []
    last_predicted = ()
    last_layout_source = ''
    saved_frame = False
    locked_center = None          # 旧路径：按观测环心锁定（没有布局预测时用）
    locked_ref = None             # 新路径：锁定的参考环号（0/1/2），按预测布局对齐

    try:
        while time.monotonic() - started < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.0)
            ok, frame = camera.read()
            if not ok or frame is None:
                continue
            if map1 is not None:
                frame = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
            if ROTATE[args.cam_rotation] is not None:
                frame = cv2.rotate(frame, ROTATE[args.cam_rotation])
            # 只看画面中心一块：外面（车身、线束、其他工位）直接涂黑，不参与
            # 检测。坐标保持全图不变，所以 target 像素/纠偏矩阵都不用改。
            if 0.0 < args.roi_fraction < 1.0:
                fh, fw = frame.shape[:2]
                mx = int(round(fw * (1.0 - args.roi_fraction) / 2.0))
                my = int(round(fh * (1.0 - args.roi_fraction) / 2.0))
                frame[:my, :] = 0
                frame[fh - my:, :] = 0
                frame[:, :mx] = 0
                frame[:, fw - mx:] = 0
            frames += 1

            now = time.monotonic()
            dt = now - last_t
            last_t = now

            # ---- 确认臂已到位 → 立刻取消位置保持，进入跟随 ----
            if phase == 'settle':
                angles = list(node._gimbal)
                if (last_angles is None
                        or max(abs(angles[i] - last_angles[i]) for i in range(3))
                        > args.settle_tol_deg):
                    settle_since = now      # 还在动：重新计时
                    last_angles = angles
                elif (settle_since is not None
                        and now - settle_since >= args.settle_stable_s):
                    # 到位了：第一帧非零速度就会顶掉 F407 的位置保持（见 GimbalTask.c
                    # 的"速度指令夺回控制权"），这里可以直接开始。
                    phase = 'servo'
                    pid_yaw.reset()
                    pid_ext.reset()
                    print('[vision_put_align] 臂已到位，开始对环', flush=True)
                if phase == 'settle':
                    continue

            # ---- 找环 ----
            try:
                det = detector.detect(frame, require_quality=False)
            except RingStationDetectionError as error:
                print(f'[vision_put_align] 检测异常：{error}', flush=True)
                continue
            except Exception as error:  # noqa: BLE001 - 单帧异常不该带走进程
                print(f'[vision_put_align] 检测异常（本帧跳过）：{error}', flush=True)
                continue

            centers = det.centers_px
            if locked_ref is None and 1 <= args.ring <= 3:
                locked_ref = args.ring - 1
            ref_index, center, locked_ref = _select_predicted(
                getattr(det, 'predicted_centers_px', ()),
                (target_x, target_y), locked_ref,
                args.max_target_distance_px)
            if center is not None:
                index = ref_index
                locked_center = None
            else:
                # 旧检测器/布局推不出来：退回"按观测环心锁定"的老逻辑
                index, center, locked_center = _select_locked(
                    centers, args.ring, (target_x, target_y),
                    locked_center, args.lock_radius_px,
                    args.max_target_distance_px)

            ex = ey = 0.0
            yaw_vel = ext_vel = 0.0
            if center is not None:
                ex = center[0] - target_x
                ey = center[1] - target_y
                if abs(ex) <= args.deadband_px and abs(ey) <= args.deadband_px:
                    stable += 1
                    pid_yaw.reset()
                    pid_ext.reset()
                else:
                    stable = 0
                    yaw_vel = args.yaw_sign * pid_yaw.update(ex, dt)
                    ext_vel = args.ext_sign * pid_ext.update(ey, dt)
                status = (f'ring{index + 1} ex={ex:+.1f} ey={ey:+.1f} '
                          f'yaw={yaw_vel:+.1f}°/s ext={ext_vel:+.1f}°/s '
                          f'stable={stable}/{args.stable}')
            else:
                stable = 0
                pid_yaw.reset()
                pid_ext.reset()
                if locked_center is not None:
                    status = 'LOCK LOST：锁定环暂时看不到，原地等它回来（不乱切别的环）'
                else:
                    status = 'SEARCH 没找到圆环'

            if args.dry_run and det.detected:
                # 只看不动：记录最后一帧"有检出"的环心和完整布局，方便标定/排错。
                # 必须一起更新，否则会拿到"这一帧的预测 + 上一帧的观测"的混合结果。
                last_centers = [tuple(c) for c in centers]
                last_predicted = getattr(det, 'predicted_centers_px', ())
                last_layout_source = getattr(det, 'layout_source', '')
            else:
                # 每帧都发，丢环/进死区时发 0：F407 速度环 300ms 超时会停，
                # 这里主动发 0 能让它停得更干脆，也不会带着残余速度漂。
                node.send_gimbal_vel(yaw_vel, 0.0, ext_vel)

            if not args.headless:
                _draw_overlay(frame, det, centers, index, locked_ref, status)
                cv2.imshow('vision_put_align', frame)
                if cv2.waitKey(1) & 0xFF in (27, ord('q')):
                    result = {'ok': False, 'reason': 'user quit'}
                    break
            if args.save_frame and not saved_frame:
                vis = frame.copy()
                if args.headless:
                    _draw_overlay(vis, det, centers, index, locked_ref, status)
                if cv2.imwrite(args.save_frame, vis):
                    print(f'[vision_put_align] 已保存识别画面：{args.save_frame}',
                          flush=True)
                else:
                    print(f'[vision_put_align] 保存失败：{args.save_frame}',
                          flush=True)
                saved_frame = True

            if (not args.dry_run) and stable >= args.stable and node._gimbal_ok:
                result = {'ok': True, 'ring': index + 1,
                          'dx_px': round(ex, 1), 'dy_px': round(ey, 1),
                          'yaw_deg': round(node._gimbal[0], 2),
                          'lift_deg': round(node._gimbal[1], 2),
                          'extend_deg': round(node._gimbal[2], 2),
                          'frames': frames}
                print(f'[vision_put_align] 对环完成：{status}', flush=True)
                break
    finally:
        node.send_gimbal_vel(0.0, 0.0, 0.0)
        camera.release()
        if not args.headless:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

    if args.dry_run:
        # 标定用：把"云台停在这个姿态时环心在哪"原样报出来，
        # 夹爪正对环心时读到的值就是 target_x / target_y。
        result = {'ok': False, 'reason': 'dry-run（只看没动）',
                  'centers_px': [[round(c[0], 1), round(c[1], 1)]
                                 for c in last_centers],
                  'predicted_centers_px': [
                      None if p is None else [round(p[0], 1), round(p[1], 1)]
                      for p in last_predicted],
                  'layout_source': last_layout_source,
                  'target_x': target_x, 'target_y': target_y,
                  'frames': frames}

    print(json.dumps(result, ensure_ascii=False), flush=True)
    return 0 if result.get('ok') else 1


if __name__ == '__main__':
    raise SystemExit(main())
