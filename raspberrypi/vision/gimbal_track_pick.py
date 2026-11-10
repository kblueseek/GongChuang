#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""云台跟随抓取跟踪器：color_pick 指令的视觉端。

作为 motion_planner 的子程序运行（ROS2 节点 + 摄像头在一个进程里）：

    python3 vision/gimbal_track_pick.py --color 4

做什么：
  1. 打开 USB 摄像头（与 vision/main.py 同一套畸变矫正配置）；
  2. 按颜色标定配置（config/material_color_thresholds.json）分割指定颜色物料；
  3. 实时窗口显示画面 + 识别框 + 十字线 + 当前云台角（--headless 关闭）；
  4. 物料偏离画面中心 → 发 /cmd_gimbal 让云台 yaw/extend 跟随（lift 保持不动），
     把物料"吊"到云台正下方；
  5. 连续稳定若干帧后退出，最后一行打印 JSON 结果：
         {"ok": true, "color": 4, "yaw_deg": …, "lift_deg": …, "extend_deg": …}
     motion_planner 的 color_pick 只跟随不抓取（no_pick），收到 JSON 后继续执行
     序列里后续的抓取/放置指令（你自己写死的 gimbal/gripper/tray 指令）。

需要现场标定的量（命令行参数，先用默认值试，方向反了就加负号）：
  --yaw-deg-per-px / --yaw-sign   像素误差 → 云台 yaw 增量（符号看机械装配）
  --ext-deg-per-px / --ext-sign   像素误差 → 云台 extend 增量
"""

from __future__ import annotations

import argparse
import json
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
        self._pub_gimbal = self.create_publisher(Float32MultiArray, '/cmd_gimbal', 10)
        self.create_subscription(Float32MultiArray, '/gimbal_state',
                                 self._cb_gimbal_state, 10)
        self._gimbal = [0.0, 0.0, 0.0]
        self._gimbal_ok = False

    def _cb_gimbal_state(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            self._gimbal = [float(v) for v in msg.data[:3]]
            self._gimbal_ok = True

    def send_gimbal(self, yaw: float, lift: float, extend: float):
        self._pub_gimbal.publish(Float32MultiArray(data=[yaw, lift, extend]))


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
            if min(fw, fh) < args.min_dimension:
                continue
            if 0.0 < args.max_dimension < max(fw, fh):
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


def main() -> int:
    parser = argparse.ArgumentParser(description='云台跟随颜色物料（color_pick 视觉端）')
    parser.add_argument('--color', type=int, required=True, help='颜色编号 1~6')
    parser.add_argument('--device', default='/dev/video0')
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--fps', type=float, default=30.0)
    parser.add_argument('--timeout', type=float, default=30.0, help='跟随总超时 s')
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
    parser.add_argument('--yaw-deg-per-px', type=float, default=0.04,
                        help='像素误差→yaw 增量比例（待标定）')
    parser.add_argument('--yaw-sign', type=float, default=1.0, choices=[1.0, -1.0],
                        help='yaw 修正方向反了改成 -1')
    parser.add_argument('--ext-deg-per-px', type=float, default=0.02,
                        help='像素误差→extend 增量比例（待标定）')
    parser.add_argument('--ext-sign', type=float, default=1.0, choices=[1.0, -1.0])
    # —— 画面约束 ——
    parser.add_argument('--roi-bottom', type=float, default=0.5,
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

    import cv2
    import numpy as np

    color_doc = load_color_config(HERE / 'config' / 'material_color_thresholds.json')
    if not any(int(c['color_id']) == args.color for c in color_doc['colors']):
        print(json.dumps({'ok': False, 'reason': f'颜色 {args.color} 没有标定配置'}),
              flush=True)
        return 1

    rclpy.init()
    node = GimbalTrackPick(args)

    camera = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not camera.isOpened():
        print(json.dumps({'ok': False, 'reason': f'打不开摄像头 {args.device}'}),
              flush=True)
        node.destroy_node()
        rclpy.shutdown()
        return 1
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    camera.set(cv2.CAP_PROP_FPS, args.fps)
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 4)
    for _ in range(10):                      # 预热
        ok, _f = camera.read()
        if ok:
            break
    map1, map2 = build_undistort_maps(args.width, args.height)
    ROTATE = {0: None, 90: cv2.ROTATE_90_CLOCKWISE,
              180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}

    if not args.headless:
        cv2.namedWindow('gimbal_track_pick', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('gimbal_track_pick', 960, 540)

    # 画面尺寸经旋转后才知道（90/270 宽高互换）
    if args.cam_rotation in (90, 270):
        frame_w, frame_h = args.height, args.width
    else:
        frame_w, frame_h = args.width, args.height
    cx_target, cy_target = frame_w / 2.0, frame_h / 2.0
    roi_y = int(frame_h * (1.0 - max(0.0, min(1.0, args.roi_bottom))))
    roi_y_scaled = roi_y   # detect_target 内部按同尺度缩放，见调用处

    stable = 0
    # wait = 等物料静止（不跟随，不动云台）；track = 静止确认后一次性跟随对中
    phase = 'track' if args.still_time <= 0.0 else 'wait'
    hist = []                     # (t, cx, cy) 静止判定窗口（wait 阶段积累）
    started = time.monotonic()
    last_send = 0.0
    result = {'ok': False, 'reason': 'timeout'}
    yaw_hold = extend_hold = None

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
            ok, frame = camera.read()
            if not ok or frame is None:
                continue
            if map1 is not None:
                frame = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
            if ROTATE[args.cam_rotation] is not None:
                frame = cv2.rotate(frame, ROTATE[args.cam_rotation])

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

            if found:
                now = time.monotonic()   # 本帧时间戳：静止判定/hist/发送节流共用
                if phase == 'wait':
                    # 不跟随、不动云台，只观察：物料持续静止才算数（转盘场景）
                    hist.append((now, target['cx'], target['cy']))
                    if _is_still(now):
                        phase = 'track'
                        hist.clear()
                        stable = 0
                        print('[gimbal_track_pick] 物料已静止，开始跟随对中', flush=True)
                    status = (f'WAIT_STILL {now - started:.1f}s '
                              f'({args.still_time}s 内抖动<{args.still_px:.0f}px)')
                else:
                    # track：一次性跟随到物料正上方
                    if node._gimbal_ok:
                        yaw_hold, extend_hold = node._gimbal[0], node._gimbal[2]
                        yaw_t = yaw_hold - args.yaw_sign * args.yaw_deg_per_px * ex
                        ext_t = extend_hold + args.ext_sign * args.ext_deg_per_px * ey
                        if now - last_send >= 1.0 / 15.0:    # 15Hz 足够，别刷爆总线
                            node.send_gimbal(yaw_t, node._gimbal[1], ext_t)
                            last_send = now
                    if abs(ex) <= args.tol_px and abs(ey) <= args.tol_px:
                        stable += 1
                    else:
                        stable = 0
                    status = (f'TRACK ex={ex:+.0f} ey={ey:+.0f} stable={stable}'
                              f'/{args.stable}')
                    if stable >= args.stable and node._gimbal_ok:
                        result = {'ok': True, 'color': args.color,
                                  'yaw_deg': round(node._gimbal[0], 2),
                                  'lift_deg': round(node._gimbal[1], 2),
                                  'extend_deg': round(node._gimbal[2], 2)}
                        print(f'[gimbal_track_pick] 对准收工：{status}', flush=True)
                        break
            else:
                stable = 0
                hist.clear()
                status = f'SEARCH color={args.color}({COLOR_NAMES[args.color]}) area={area:.0f}'

            if not args.headless:
                cv2.drawMarker(frame, (int(cx_target), int(cy_target)),
                               (255, 255, 255), cv2.MARKER_CROSS, 30, 1)
                cv2.line(frame, (0, roi_y), (frame_w, roi_y), (120, 120, 255), 1)
                cv2.putText(frame, f'ROI>{args.roi_bottom}', (frame_w - 130, roi_y - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (120, 120, 255), 1,
                            cv2.LINE_AA)
                cv2.putText(frame, status, (16, 34), cv2.FONT_HERSHEY_SIMPLEX,
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
        camera.release()
        if not args.headless:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

    print(json.dumps(result, ensure_ascii=False), flush=True)
    return 0 if result.get('ok') else 1


if __name__ == '__main__':
    raise SystemExit(main())
