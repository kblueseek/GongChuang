"""calibrate 业务：圆环工位视觉对位 + 里程计重置。

从 ring_fix_node.py 搬过来的核心逻辑：
  1. 每帧跑 RingStationDetector 识别三个圆环；
  2. 检测器给出底盘坐标系下的纠偏量（mm/°）；
  3. PID 把纠偏量变成速度，再换算成"领先当前位姿多少"的 /cmd_pose 小步；
  4. 连续若干帧进容差 → 发 /set_odom 重置里程计，返回 {'ok': True, ...}。

区别：摄像头和 ROS 节点由 vision_node 提供，本业务不开关摄像头、不创建节点。
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

from ..business import Business, register


# F407 调参编号（PiComm.h）：位置环 Kp / 转向环 Kp
TUNE_POS_KP = 0x05
TUNE_HEAD_KP = 0x08

# 默认值和 ring_fix_node.py 的 declare_parameter 一致。
# 请求里的 args 可以逐项覆盖（不用改 launch）。
DEFAULTS = {
    # 对准后车体相对中间圆环的偏移（里程计系，米）
    'align_offset_x_m': 0.0,
    'align_offset_y_m': 0.0,
    'align_yaw_deg': '',          # 空 = 只修位置，不修航向
    # 底盘对位 PID
    'ctrl_kp_xy': 1.0,
    'ctrl_ki_xy': 0.02,
    'ctrl_kd_xy': 0.15,
    'ctrl_kp_yaw': 1.0,
    'ctrl_ki_yaw': 0.0,
    'ctrl_kd_yaw': 0.15,
    'max_xy_speed_m_s': 0.06,
    'max_yaw_speed_deg_s': 15.0,
    'ctrl_deadband_xy_mm': 0.5,
    'ctrl_deadband_yaw_deg': 0.15,
    'ctrl_deriv_tau': 0.1,
    'ctrl_filter_tau': 0.15,
    # 滞环
    'ctrl_stop_xy_mm': 1.0,
    'ctrl_stop_yaw_deg': 0.35,
    'ctrl_settle_s': 0.2,
    # 速度 → 位姿领先量（兜底值；正常运行时自动跟随 F407 调参页的 0x05/0x08）
    'chassis_pos_kp': 1.5,
    'chassis_heading_kp': 4.0,
    'max_lead_xy_mm': 80.0,
    'max_lead_yaw_deg': 8.0,
    # 收敛 / 超时
    'converge_tol_xy_mm': 2.0,
    'converge_tol_yaw_deg': 0.8,
    'converge_hold_frames': 5,
    'timeout_s': 45.0,
    'require_quality': False,
}


def _f(value, default=0.0) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _i(value, default=0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return int(default)


def _b(value, default=False) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


class _Calibrator:
    """一次圆环矫正的完整状态机（对应 ring_fix 的 _tick / _control_step）。"""

    def __init__(self, ctx, params: dict, detector, map1, map2,
                 ring_target, stop, pid_cls):
        import cv2
        self._cv2 = cv2
        self.ctx = ctx
        self.detector = detector
        self.map1 = map1
        self.map2 = map2
        self.ring_target = ring_target
        self.stop = stop

        p = params
        self.timeout = _f(p['timeout_s'], 45.0)
        self.require_quality = _b(p['require_quality'], False)
        self.off_x = _f(p['align_offset_x_m'])
        self.off_y = _f(p['align_offset_y_m'])
        raw_value = p.get('align_yaw_deg', '')
        raw = '' if raw_value is None else str(raw_value).strip()
        self.align_yaw = math.radians(_f(raw)) if raw else None

        self.max_xy_speed = abs(_f(p['max_xy_speed_m_s'], 0.06)) * 1000.0   # mm/s
        self.max_yaw_speed = abs(_f(p['max_yaw_speed_deg_s'], 15.0))        # °/s
        self.deadband_xy = abs(_f(p['ctrl_deadband_xy_mm'], 0.5))
        self.deadband_yaw = abs(_f(p['ctrl_deadband_yaw_deg'], 0.15))
        self.filter_tau = max(0.0, _f(p['ctrl_filter_tau'], 0.15))
        self.stop_xy = abs(_f(p['ctrl_stop_xy_mm'], 1.0))
        self.stop_yaw = abs(_f(p['ctrl_stop_yaw_deg'], 0.35))
        self.settle_s = max(0.0, _f(p['ctrl_settle_s'], 0.2))
        self.pos_kp = max(0.1, abs(_f(p['chassis_pos_kp'], 1.5)))
        self.heading_kp = max(0.1, abs(_f(p['chassis_heading_kp'], 2.4)))
        self.max_lead_xy = abs(_f(p['max_lead_xy_mm'], 80.0))
        self.max_lead_yaw = abs(_f(p['max_lead_yaw_deg'], 8.0))
        self.tol_xy = abs(_f(p['converge_tol_xy_mm'], 2.0))
        self.tol_yaw = abs(_f(p['converge_tol_yaw_deg'], 0.8))
        self.hold_need = max(1, _i(p['converge_hold_frames'], 5))

        deriv_tau = _f(p['ctrl_deriv_tau'], 0.1)
        self.pid_x = pid_cls(_f(p['ctrl_kp_xy'], 1.0), _f(p['ctrl_ki_xy'], 0.02),
                             _f(p['ctrl_kd_xy'], 0.15), self.max_xy_speed, deriv_tau)
        self.pid_y = pid_cls(_f(p['ctrl_kp_xy'], 1.0), _f(p['ctrl_ki_xy'], 0.02),
                             _f(p['ctrl_kd_xy'], 0.15), self.max_xy_speed, deriv_tau)
        self.pid_yaw = pid_cls(_f(p['ctrl_kp_yaw'], 1.0), _f(p['ctrl_ki_yaw'], 0.0),
                               _f(p['ctrl_kd_yaw'], 0.15), self.max_yaw_speed, deriv_tau)

        self.hold_frames = 0
        self.last_ctrl_t = 0.0
        self.last_usable_t = 0.0
        self.hold_pose = None
        self.filt = None
        self.filt_t = 0.0
        self.moving = False
        self.anchor_pose = None
        self.settle_until = 0.0
        self.last_diag_t = 0.0

    # ---------------- 主循环 ----------------

    def run(self) -> dict:
        import rclpy
        started = time.monotonic()
        while True:
            rclpy.spin_once(self.ctx.node, timeout_sec=0.0)
            if self.stop is not None and self.stop():
                return {'ok': False, 'reason': 'stopped'}
            now = time.monotonic()
            if now - started > self.timeout:
                return {'ok': False,
                        'reason': f'超时（{self.timeout:.0f}s 未对准）'}

            ok, frame = self.ctx.camera.read()
            if not ok or frame is None:
                continue
            if self.map1 is not None:
                frame = self._cv2.remap(frame, self.map1, self.map2,
                                        self._cv2.INTER_LINEAR)
            try:
                result = self.detector.detect(
                    frame, require_quality=self.require_quality)
            except Exception:  # noqa: BLE001 - 单帧异常只跳过这一帧
                continue

            usable = (result.detected and result.stable
                      and len(result.centers_px) == 3
                      and result.correction_body_x_mm is not None
                      and math.isfinite(result.affine_residual))
            if usable:
                cx_mm = result.correction_body_x_mm
                cy_mm = result.correction_body_y_mm
                cyaw = result.correction_yaw_deg
                self.last_usable_t = now
                fx, fy, fyaw = self._filter(now, cx_mm, cy_mm, cyaw)
                self._update_motion(now, fx, fy, fyaw)

                if (not self.moving
                        and now >= self.settle_until
                        and max(abs(fx), abs(fy)) <= self.tol_xy
                        and abs(fyaw) <= self.tol_yaw):
                    self.hold_frames += 1
                else:
                    self.hold_frames = 0
                if self.hold_frames >= self.hold_need:
                    return self._converge()
                if self.moving:
                    self._control_step(now, fx, fy, fyaw)
                elif self.anchor_pose is not None:
                    # 停车阶段按住锁存的锚点位姿，别让位置环松掉
                    self.hold_pose = self.anchor_pose
            else:
                self.hold_frames = 0
                if now - self.last_usable_t > 0.5:
                    for pid in (self.pid_x, self.pid_y, self.pid_yaw):
                        pid.reset()
                    self.last_ctrl_t = 0.0
                    self.filt = None
                    self.moving = False

            # F407 的位姿目标约 100ms 过期，每拍重发保持位姿
            if self.hold_pose is not None:
                self.ctx.node.publish_pose(*self.hold_pose)

            if now - self.last_diag_t >= 1.0:
                self.last_diag_t = now
                detail = result.detail if result.detected else '未检测到三个圆环'
                self.ctx.node.get_logger().info(
                    f'calibrate {detail} 稳定帧 {self.hold_frames}/{self.hold_need}')

    # ---------------- 伺服 ----------------

    def _filter(self, now, cx_mm, cy_mm, cyaw_deg):
        if self.filt is None:
            self.filt = [cx_mm, cy_mm, cyaw_deg]
            self.filt_t = now
            return cx_mm, cy_mm, cyaw_deg
        dt = min(max(now - self.filt_t, 1e-3), 0.2)
        self.filt_t = now
        alpha = 1.0 if self.filter_tau <= 0.0 else dt / (self.filter_tau + dt)
        for i, value in enumerate((cx_mm, cy_mm, cyaw_deg)):
            self.filt[i] += alpha * (value - self.filt[i])
        return self.filt[0], self.filt[1], self.filt[2]

    def _update_motion(self, now, fx, fy, fyaw):
        mag = max(abs(fx), abs(fy))
        if self.moving:
            if mag <= self.stop_xy and abs(fyaw) <= self.stop_yaw:
                self.moving = False
                odom, _t = self.ctx.node.get_odom()
                self.anchor_pose = odom
                self.settle_until = now + self.settle_s
                for pid in (self.pid_x, self.pid_y, self.pid_yaw):
                    pid.reset()
        elif mag > self.tol_xy or abs(fyaw) > self.tol_yaw:
            self.moving = True
            self.anchor_pose = None
            self.last_ctrl_t = 0.0
            for pid in (self.pid_x, self.pid_y, self.pid_yaw):
                pid.reset()

    def _control_step(self, now, cx_mm, cy_mm, cyaw_deg):
        odom, odom_t = self.ctx.node.get_odom()
        if odom is None or now - odom_t > 0.5:
            return   # 没有新鲜里程计不动车
        if self.last_ctrl_t <= 0.0:
            self.last_ctrl_t = now
            for pid in (self.pid_x, self.pid_y, self.pid_yaw):
                pid.reset()
            return
        dt = min(max(now - self.last_ctrl_t, 1e-3), 0.2)
        self.last_ctrl_t = now

        ex = 0.0 if abs(cx_mm) <= self.deadband_xy else cx_mm
        ey = 0.0 if abs(cy_mm) <= self.deadband_xy else cy_mm
        eyaw = 0.0 if abs(cyaw_deg) <= self.deadband_yaw else cyaw_deg

        vx = self.pid_x.update(ex, dt)          # mm/s（车体系 +x）
        vy = self.pid_y.update(ey, dt)          # mm/s（车体系 +y）
        vyaw = self.pid_yaw.update(eyaw, dt)    # °/s

        speed = math.hypot(vx, vy)
        if speed > self.max_xy_speed and speed > 1e-6:
            scale = self.max_xy_speed / speed
            vx *= scale
            vy *= scale
        vyaw = max(-self.max_yaw_speed, min(self.max_yaw_speed, vyaw))

        # 速度 → 位姿领先量：F407 位置环是纯 P，lead = v / Kp
        lead_x = vx / self.pos_kp
        lead_y = vy / self.pos_kp
        lead = math.hypot(lead_x, lead_y)
        if lead > self.max_lead_xy and lead > 1e-6:
            scale = self.max_lead_xy / lead
            lead_x *= scale
            lead_y *= scale
        lead_yaw = max(-self.max_lead_yaw,
                       min(self.max_lead_yaw, vyaw / self.heading_kp))

        ox, oy, oyaw = odom
        c, s = math.cos(oyaw), math.sin(oyaw)
        tx = ox + (lead_x * c - lead_y * s) / 1000.0
        ty = oy + (lead_x * s + lead_y * c) / 1000.0
        tyaw = oyaw + math.radians(lead_yaw)
        self.hold_pose = (tx, ty, tyaw)

    # ---------------- 结束 ----------------

    def _converge(self) -> dict:
        odom, _t = self.ctx.node.get_odom()
        rx, ry = self.ring_target
        cx = rx + self.off_x
        cy = ry + self.off_y
        if self.align_yaw is not None:
            cyaw = self.align_yaw
        elif odom is not None:
            cyaw = odom[2]
        else:
            cyaw = 0.0
        self.ctx.node.publish_set_odom(cx, cy, cyaw)
        self.ctx.node.get_logger().info(
            f'圆环对准：里程计重置到 ({cx:.3f}, {cy:.3f}) m '
            f'yaw={math.degrees(cyaw):+.1f}°')
        return {'ok': True, 'x_m': cx, 'y_m': cy, 'yaw_rad': cyaw}


@register
class CalibrateBusiness(Business):
    name = 'calibrate'

    def run(self, ctx, args, stop):
        import sys
        vision_dir = str(ctx.vision_dir)
        if vision_dir and vision_dir not in sys.path:
            sys.path.insert(0, vision_dir)

        try:
            rx = float(args['x_m'])
            ry = float(args['y_m'])
        except (KeyError, TypeError, ValueError):
            return {'ok': False,
                    'reason': 'calibrate 需要 x_m / y_m（中间圆环的里程计坐标）'}

        from ring_station_detector import (
            RingStationDetector, load_ring_station_config,
        )
        import vision_put_align as vpa

        config_path = Path(args.get('config') or
                           (ctx.vision_dir / 'config' /
                            'ring_station_detector.json'))
        try:
            cfg = load_ring_station_config(config_path)
        except Exception as error:  # noqa: BLE001
            return {'ok': False, 'reason': f'圆环配置无效：{error}'}
        if getattr(cfg, 'chassis_image_to_motion', None) is None:
            return {'ok': False,
                    'reason': 'ring_station_detector.json 缺少纠偏矩阵，无法计算底盘纠偏量'}

        params = dict(DEFAULTS)
        # 上位机『视觉矫正 PID』页写这份文件；每次 calibrate 开始前读一次。
        cfg_path = ctx.vision_dir / 'config' / 'calibrate.json'
        try:
            doc = json.loads(cfg_path.read_text(encoding='utf-8'))
            for key, value in doc.items():
                if key in params:
                    params[key] = value
        except FileNotFoundError:
            pass
        except Exception as error:  # noqa: BLE001 - 配置坏了用默认值继续
            ctx.node.get_logger().warning(
                f'calibrate.json 读取失败（{error}），用默认参数')

        # 动作序列 args 的优先级最高
        for key, value in args.items():
            if key in params:
                params[key] = value

        # 位置/航向环 Kp 不在视觉矫正面板里重复调：优先用 F407 调参页经
        # /tune_state 回传的生效值（0x05 位置环 / 0x08 转向环）；没收到回传
        # （f407_bridge 没起 / 还没下发调参）才用 JSON 里的兜底值。
        params['chassis_pos_kp'] = ctx.node.get_tune(
            TUNE_POS_KP, params['chassis_pos_kp'])
        params['chassis_heading_kp'] = ctx.node.get_tune(
            TUNE_HEAD_KP, params['chassis_heading_kp'])

        detector = RingStationDetector(cfg)
        map1, map2 = vpa.build_undistort_maps(int(ctx.width), int(ctx.height))

        return _Calibrator(ctx, params, detector, map1, map2,
                           (rx, ry), stop, vpa.PID).run()
