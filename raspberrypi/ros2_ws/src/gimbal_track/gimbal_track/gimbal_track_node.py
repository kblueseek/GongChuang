#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""云台视觉跟随 demo：物料像素坐标 -> yaw/extend 速度（PID）。

手柄的 select 键（joy_control）会把 /vision_enable 置 True，同时让出 /cmd_gimbal_vel；
本节点在这期间接管 /cmd_gimbal_vel，用 PID 把物料质心往画面中心拉，其余时间
完全不发速度，绝不和手柄抢。

映射（摄像头旋转安装时用 cam_rotation 把像素误差转正，再拆到两轴）：
    误差向量 (ex, ey) = (cx - 宽/2, cy - 高/2)，绕画面中心旋转 cam_rotation 度后
      x 分量 -> yaw 角速度，y 分量 -> extend 速度，lift 恒 0
    轴反了 / 发散，再单独用 dir_yaw / dir_extend 翻号

话题
----
订阅：
    /vision_enable    std_msgs/Bool            True = 视觉接管
    /material_target  std_msgs/Float32MultiArray  [detected, cx, cy, img_w, img_h]
发布：
    /cmd_gimbal_vel   std_msgs/Float32MultiArray  [yaw°/s, lift°/s, extend°/s]

参数
----
  kp_yaw / ki_yaw / kd_yaw      yaw 环 PID 增益
  kp_extend / ki_extend / kd_extend  extend 环 PID 增益
                               加 kd 后可以用更高 kp（微分提供阻尼，压振荡）；
                               ki 消稳态误差，但像素有噪声，建议先用小值或 0。
  deriv_tau                    微分低通时间常数 s（0.03~0.1），越大越平滑
  max_yaw_deg_s / max_extend_deg_s   输出限幅（°/s）
  cam_rotation                 摄像头安装旋转角（°），把像素轴转到和 yaw/extend 对齐。
                               摄像头转了 90° 就填 90（或 -90 / 270，看实际往哪边转，
                               填到不再越跟越偏为止）；180° 填 180。默认 0。
  dir_yaw / dir_extend         输出方向（±1，发散就翻）
  deadband_px                  误差小于这个像素数就停
  rate_hz                      控制频率（要 >= 20Hz，否则 F407 300ms 超时会自己停）
  target_timeout_s             /material_target 多久没更新就算丢目标（发零速停住）

  抓取动作（三段状态机）：接管后先不跟踪、保持不动，等物料真正静止（转盘停），
  再跟踪对中，最后 lift 下降再抬起。关键点：判静止时相机必须不动，否则云台跟踪
  会抵消物料运动、让它「看起来」静止。下探用 /gimbal_state 的 lift 位置闭环：
  降到目标深度（或限位）再升回起始高度，保证回原位（纯计时会因重力不对称回不去）。
    dip_enable                 开关（False 只等待静止，不进入对中/下探）
    dip_depth                  下降目标距离（°）
    dip_down_vel_deg_s         下降速度
    dip_up_vel_deg_s           上升速度（慢一点停得准）
    dip_approach_deg           离起始高度这么近时开始线性减速
    dip_creep_vel_deg_s        减速后的最低速度
    dip_timeout_s              单段安全超时（反馈卡死兜底）
    dip_dir                    下降方向（±1，方向反了就翻）
    dip_still_px / dip_still_time_s  判定「静止」：像素抖动 < px 且持续 time 秒
    dip_center_confirm_s       对中后稳住多久才下探（防止「路过」中心就误触发）
    dip_cooldown_s             下探完成后到下次可下探的最小间隔
"""

from __future__ import annotations

import math
import time
from collections import deque

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray


class PID:
    """带输出限幅、积分抗饱和、微分低通的小 PID。

    ki=kd=0 就是纯 P。加 kd 后能在不增大振荡的前提下把 kp 提得更高
    （微分提供阻尼）；ki 消稳态误差，但像素测量有噪声，建议先用小值或保持 0。
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

    def reset_integral(self):
        self.integral = 0.0

    def update(self, err, dt):
        if dt <= 0.0:
            dt = 0.01

        # —— 微分：先求原始微分，再做一阶低通，压住像素噪声的抖动 ——
        raw_deriv = (err - self.prev_err) / dt
        self.prev_err = err
        if self.deriv_tau > 0.0:
            alpha = dt / (self.deriv_tau + dt)
            self.deriv += alpha * (raw_deriv - self.deriv)
        else:
            self.deriv = raw_deriv
        d = self.kd * self.deriv
        p = self.kp * err

        # —— 积分：条件积分抗饱和，饱和且误差还往同方向推时不再累积 ——
        pre = p + self.ki * self.integral + d
        saturated = abs(pre) >= self.limit
        if self.ki != 0.0 and not (saturated and pre * err > 0.0):
            self.integral += err * dt
            bound = self.limit / abs(self.ki)
            self.integral = max(-bound, min(bound, self.integral))

        out = p + self.ki * self.integral + d
        return max(-self.limit, min(self.limit, out))


class GimbalTrackNode(Node):
    def __init__(self):
        super().__init__('gimbal_track')

        self.declare_parameter('kp_yaw', 0.2)
        self.declare_parameter('ki_yaw', 0.0)
        self.declare_parameter('kd_yaw', 0.0)
        self.declare_parameter('kp_extend', 0.8)
        self.declare_parameter('ki_extend', 0.0)
        self.declare_parameter('kd_extend', 0.0)
        self.declare_parameter('max_yaw_deg_s', 120.0)
        self.declare_parameter('max_extend_deg_s', 300.0)
        self.declare_parameter('dir_yaw', 1.0)
        self.declare_parameter('dir_extend', 1.0)
        self.declare_parameter('deadband_px', 10.0)
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('target_timeout_s', 0.5)
        self.declare_parameter('cam_rotation', 0.0)
        self.declare_parameter('deriv_tau', 0.05)   # 微分低通时间常数 s，越大越平滑

        # —— 下探（抓取）动作：目标静止后 lift 下降再抬起（位置闭环，保证回原位）——
        self.declare_parameter('dip_enable', True)
        self.declare_parameter('dip_depth', 1000.0)          # 下降目标距离（°）
        self.declare_parameter('dip_down_vel_deg_s', 600.0)  # 下降速度
        self.declare_parameter('dip_up_vel_deg_s', 250.0)    # 上升速度（慢一点好停准）
        self.declare_parameter('dip_approach_deg', 40.0)     # 离起点这么近时开始减速
        self.declare_parameter('dip_creep_vel_deg_s', 50.0)  # 减速后的最低速度
        self.declare_parameter('dip_timeout_s', 5.0)         # 单段安全超时（反馈卡死兜底）
        self.declare_parameter('dip_dir', 1.0)               # 下降方向（±1，反了就翻）
        self.declare_parameter('dip_still_px', 8.0)          # 判定“静止”的像素抖动阈值
        self.declare_parameter('dip_still_time_s', 0.5)      # 静止持续多久才触发
        self.declare_parameter('dip_cooldown_s', 4.0)        # 两次下探最小间隔
        self.declare_parameter('dip_center_confirm_s', 0.3)  # 对中后稳住多久才下探

        self._dir_yaw = float(self.get_parameter('dir_yaw').value)
        self._dir_extend = float(self.get_parameter('dir_extend').value)
        self._deadband = float(self.get_parameter('deadband_px').value)
        self._timeout = float(self.get_parameter('target_timeout_s').value)
        self._cam_rotation = float(self.get_parameter('cam_rotation').value)
        deriv_tau = float(self.get_parameter('deriv_tau').value)

        self._dip_enable = bool(self.get_parameter('dip_enable').value)
        self._dip_depth = float(self.get_parameter('dip_depth').value)
        self._dip_down_vel = float(self.get_parameter('dip_down_vel_deg_s').value)
        self._dip_up_vel = float(self.get_parameter('dip_up_vel_deg_s').value)
        self._dip_approach = max(0.0, float(self.get_parameter('dip_approach_deg').value))
        self._dip_creep_vel = max(1.0, float(self.get_parameter('dip_creep_vel_deg_s').value))
        self._dip_timeout = max(0.5, float(self.get_parameter('dip_timeout_s').value))
        self._dip_dir = 1.0 if float(self.get_parameter('dip_dir').value) >= 0.0 else -1.0
        self._dip_still_px = float(self.get_parameter('dip_still_px').value)
        self._dip_still_time = max(0.1, float(self.get_parameter('dip_still_time_s').value))
        self._dip_cooldown = max(0.0, float(self.get_parameter('dip_cooldown_s').value))
        self._dip_center_confirm = max(0.0, float(self.get_parameter('dip_center_confirm_s').value))

        self._pid_yaw = PID(
            self.get_parameter('kp_yaw').value,
            self.get_parameter('ki_yaw').value,
            self.get_parameter('kd_yaw').value,
            float(self.get_parameter('max_yaw_deg_s').value),
            deriv_tau,
        )
        self._pid_extend = PID(
            self.get_parameter('kp_extend').value,
            self.get_parameter('ki_extend').value,
            self.get_parameter('kd_extend').value,
            float(self.get_parameter('max_extend_deg_s').value),
            deriv_tau,
        )

        # 状态
        self._enabled = False
        self._planner_active = False # 上位机正在执行动作序列：云台控制权不在视觉/手柄
        self._target = None          # (cx, cy)
        self._img_w = 1280
        self._img_h = 720
        self._target_s = 0.0
        self._last_t = time.monotonic()

        # 抓取状态机：wait_stop(不跟踪等停) -> center(跟踪对中) -> dip(下探，位置闭环)
        self._phase = 'wait_stop'
        self._center_since = 0.0         # 目标进入死区（已对中）的时刻，0 = 还没对中
        self._dip = None                 # 下探状态字典，None 空闲
        self._dip_cooldown_until = 0.0
        self._hist = deque(maxlen=200)   # (t, cx, cy)，用于判定目标是否静止

        # lift 位置反馈（来自 /gimbal_state，用于下探位置闭环）
        self._lift_deg = 0.0
        self._lift_deg_t = 0.0

        self.create_subscription(Bool, '/vision_enable', self._cb_enable, 10)
        self.create_subscription(Float32MultiArray, '/material_target', self._cb_target, 10)
        self.create_subscription(Float32MultiArray, '/gimbal_state', self._cb_gimbal_state, 10)
        # 上位机执行动作序列期间云台归上位机，本节点一个速度都不发（和手柄同一套让位规则）
        self.create_subscription(Bool, '/motion_active', self._cb_motion_active, 10)
        self._pub = self.create_publisher(Float32MultiArray, '/cmd_gimbal_vel', 10)

        rate = max(10.0, float(self.get_parameter('rate_hz').value))
        self._timer = self.create_timer(1.0 / rate, self._tick)

        # 支持运行中改参数（pid_tuner 滑杆走的就是 set_parameters 服务），
        # 否则参数只在 __init__ 读一次，滑杆拖了也不生效。
        self.add_on_set_parameters_callback(self._on_params)

        self.get_logger().info(
            'gimbal_track 就绪：kp_yaw={:.3f} kp_extend={:.3f} '
            'max [yaw {:.0f}°, extend {:.0f}°/s]。select 键切换接管。'.format(
                self._pid_yaw.kp, self._pid_extend.kp,
                self._pid_yaw.limit, self._pid_extend.limit)
        )

    # ---------------- 回调 ----------------

    def _on_params(self, params):
        """运行中改参数即时生效（pid_tuner 滑杆走的就是 set_parameters 服务）。

        改 PID 增益时把 PID 状态清零，免得带着旧积分/旧微分在换增益那一拍跳变。
        下探参数改完立即作用于下一次触发。rate_hz 这类需要重建定时器的不在此处理。
        """
        reset_pid = False
        for param in params:
            name = param.name
            if name in ('kp_yaw', 'ki_yaw', 'kd_yaw'):
                setattr(self._pid_yaw, name.split('_')[0], float(param.value))
                reset_pid = True
            elif name in ('kp_extend', 'ki_extend', 'kd_extend'):
                setattr(self._pid_extend, name.split('_')[0], float(param.value))
                reset_pid = True
            elif name == 'max_yaw_deg_s':
                self._pid_yaw.limit = max(1e-6, float(param.value))
                reset_pid = True
            elif name == 'max_extend_deg_s':
                self._pid_extend.limit = max(1e-6, float(param.value))
                reset_pid = True
            elif name == 'deadband_px':
                self._deadband = float(param.value)
            elif name == 'dip_enable':
                self._dip_enable = bool(param.value)
            elif name == 'dip_depth':
                self._dip_depth = float(param.value)
            elif name == 'dip_down_vel_deg_s':
                self._dip_down_vel = float(param.value)
            elif name == 'dip_up_vel_deg_s':
                self._dip_up_vel = float(param.value)
            elif name == 'dip_approach_deg':
                self._dip_approach = max(0.0, float(param.value))
            elif name == 'dip_creep_vel_deg_s':
                self._dip_creep_vel = max(1.0, float(param.value))
            elif name == 'dip_timeout_s':
                self._dip_timeout = max(0.5, float(param.value))
            elif name == 'dip_dir':
                self._dip_dir = 1.0 if float(param.value) >= 0.0 else -1.0
            elif name == 'dip_still_px':
                self._dip_still_px = float(param.value)
            elif name == 'dip_still_time_s':
                self._dip_still_time = max(0.1, float(param.value))
            elif name == 'dip_cooldown_s':
                self._dip_cooldown = max(0.0, float(param.value))
            elif name == 'dip_center_confirm_s':
                self._dip_center_confirm = max(0.0, float(param.value))
            else:
                continue

        if reset_pid:
            self._pid_yaw.reset()
            self._pid_extend.reset()
            self.get_logger().info(
                'PID 参数已更新：yaw(kp {:.3f} ki {:.3f} kd {:.3f}) '
                'extend(kp {:.3f} ki {:.3f} kd {:.3f}) deadband {:.1f}px'.format(
                    self._pid_yaw.kp, self._pid_yaw.ki, self._pid_yaw.kd,
                    self._pid_extend.kp, self._pid_extend.ki,
                    self._pid_extend.kd, self._deadband),
                throttle_duration_sec=0.5,
            )
        return SetParametersResult(successful=True)

    def _cb_enable(self, msg: Bool):
        enabled = bool(msg.data)
        if enabled == self._enabled:
            return
        self._enabled = enabled
        self._pid_yaw.reset()
        self._pid_extend.reset()
        self._phase = 'wait_stop'
        self._center_since = 0.0
        self._dip = None
        self._hist.clear()
        if enabled:
            self.get_logger().info('视觉接管：先保持不动，等物料静止后再跟踪对中、下探')
        else:
            self.get_logger().info('视觉退出：控制权交还手柄')

    def _cb_target(self, msg: Float32MultiArray):
        data = msg.data
        if len(data) < 5:
            return
        detected = data[0] > 0.5
        self._img_w = float(data[3]) if data[3] > 0 else self._img_w
        self._img_h = float(data[4]) if data[4] > 0 else self._img_h
        self._target_s = time.monotonic()
        if detected:
            cx, cy = float(data[1]), float(data[2])
            self._target = (cx, cy)
            self._hist.append((self._target_s, cx, cy))
        else:
            self._target = None
            self._hist.clear()

    def _cb_gimbal_state(self, msg: Float32MultiArray):
        """记录 lift 当前位置（°/相对 0 点），供下探做位置闭环。"""
        data = msg.data
        if len(data) >= 3:
            self._lift_deg = float(data[1])   # [yaw, lift, extend]
            self._lift_deg_t = time.monotonic()

    def _cb_motion_active(self, msg: Bool):
        """上位机开始/结束执行动作序列。执行期间视觉节点完全让位。"""
        active = bool(msg.data)
        if active == self._planner_active:
            return
        self._planner_active = active
        self._phase = 'wait_stop'
        self._center_since = 0.0
        self._dip = None
        self.get_logger().info(
            '上位机接管云台，视觉跟踪暂停' if active else '上位机结束，视觉跟踪恢复')

    # ---------------- 定时器 ----------------

    def _tick(self):
        now = time.monotonic()
        dt = now - self._last_t
        self._last_t = now

        # 没接管 / 上位机正在跑序列时一个速度都不发，让手柄或上位机控制；
        # 同时中止任何进行中的下探。
        if not self._enabled or self._planner_active:
            self._phase = 'wait_stop'
            self._center_since = 0.0
            self._dip = None
            return

        if self._phase == 'wait_stop':
            self._run_wait_stop(now)
        elif self._phase == 'center':
            self._run_center(now, dt)
        else:  # dip
            self._run_dip(now)

    def _run_wait_stop(self, now):
        """不跟踪、保持不动；等物料真正停住（相机固定，运动才反映到像素上）。"""
        ready = (self._target is not None and self._dip_enable
                 and now >= self._dip_cooldown_until and self._is_stationary(now))
        if ready and (now - self._lift_deg_t) >= 1.0:
            # 没有 lift 位置反馈就没法做位置闭环（下探会瞎走），先不下探
            self.get_logger().warn(
                'lift 位置反馈超时（/gimbal_state 没数据），暂不下探；'
                '确认 run_car / f407_bridge 在跑', throttle_duration_sec=3.0)
        elif ready:
            self._phase = 'center'
            self._center_since = 0.0
            self._pid_yaw.reset()
            self._pid_extend.reset()
            self.get_logger().info('物料已静止，开始跟踪对中')
        # 等待期间保持不动（发零速，云台停在原地）
        self._pub.publish(Float32MultiArray(data=[0.0, 0.0, 0.0]))

    def _run_center(self, now, dt):
        """跟踪物料到画面中心；对中并稳住后进入下探。"""
        # 丢目标 / 超时 -> 回到等待静止
        if self._target is None or (now - self._target_s) > self._timeout:
            self._phase = 'wait_stop'
            self._center_since = 0.0
            self._pid_yaw.reset()
            self._pid_extend.reset()
            self._pub.publish(Float32MultiArray(data=[0.0, 0.0, 0.0]))
            return

        cx, cy = self._target
        error_x = cx - self._img_w / 2.0
        error_y = cy - self._img_h / 2.0

        # 摄像头可能旋转安装：先把像素误差向量旋转 cam_rotation 度，
        # 旋转后的 x 分量给 yaw、y 分量给 extend，这样两轴才和画面对齐。
        rot = math.radians(self._cam_rotation)
        cos_r, sin_r = math.cos(rot), math.sin(rot)
        yaw_err = cos_r * error_x - sin_r * error_y
        ext_err = sin_r * error_x + cos_r * error_y

        if abs(yaw_err) < self._deadband and abs(ext_err) < self._deadband:
            # 已对中：稳住一小段时间再下探，避免“路过”中心就误触发
            yaw = 0.0
            extend = 0.0
            self._pid_yaw.reset()
            self._pid_extend.reset()
            if self._center_since == 0.0:
                self._center_since = now
            elif now - self._center_since >= self._dip_center_confirm:
                self._phase = 'dip'
                self._start_dip(now)
                self._run_dip(now)
                return
        else:
            self._center_since = 0.0
            yaw = self._pid_yaw.update(yaw_err, dt) * self._dir_yaw
            extend = self._pid_extend.update(ext_err, dt) * self._dir_extend

        # 三轴顺序 [yaw, lift, extend]，跟踪阶段 lift 恒 0
        self._pub.publish(Float32MultiArray(data=[float(yaw), 0.0, float(extend)]))

    # ---------------- 下探（抓取）动作 ----------------

    def _is_stationary(self, now):
        """目标在最近 dip_still_time 秒内像素几乎没动 = 转盘停了。"""
        win = self._dip_still_time
        recent = [(t, cx, cy) for (t, cx, cy) in self._hist if now - t <= win]
        if len(recent) < 4:
            return False
        # 首个样本要足够“老”，说明这段时间里目标一直被连续看到（没丢过）
        if now - recent[0][0] < win * 0.8:
            return False
        xs = [p[1] for p in recent]
        ys = [p[2] for p in recent]
        return (max(xs) - min(xs)) <= self._dip_still_px and (max(ys) - min(ys)) <= self._dip_still_px

    def _start_dip(self, now):
        """开始下探：记下当前 lift 高度作为「升回目标」，再降到目标深度（或限位）。"""
        start = self._lift_deg
        self._dip = {
            't0': now,
            'start_lift': start,
            'phase': 'down',
            'phase_t0': now,
            'max_travel': 0.0,       # 离起点最远走了多少（绝对距离，与升降正负号无关）
            'max_travel_t': now,     # 上次刷新最远点的时间（判“到限位/堵转”用）
            'side': 0.0,             # 下探方向（+1 = lift 变大 / -1 = lift 变小），由实际运动学到
        }
        self._hist.clear()
        self._pid_yaw.reset()
        self._pid_extend.reset()
        self.get_logger().info(
            f'下探触发：从 lift {start:.1f}° 下降 {self._dip_depth:.1f}°（或到限位），'
            f'再升回 {start:.1f}°'
        )

    def _run_dip(self, now):
        """下探位置闭环：按绝对距离降到目标深度/限位，再反向升回起始高度。

        不看 lift 的正负号约定，只看「离起点走了多远」：下降方向由 dip_dir 决定，
        回程反向。所以 dip_dir 反了只会变成「往上探」，不会跑飞或回不去。
        """
        dip = self._dip
        lift_cur = self._lift_deg
        delta = lift_cur - dip['start_lift']
        travel = abs(delta)

        if dip['phase'] == 'down':
            if travel > dip['max_travel']:
                dip['max_travel'] = travel
                dip['max_travel_t'] = now
                dip['side'] = 1.0 if delta > 0.0 else -1.0
            reached = travel >= self._dip_depth
            stalled = (now - dip['max_travel_t']) > 0.3      # 0.3s 没再远离起点 = 顶到限位
            timeout = (now - dip['phase_t0']) > self._dip_timeout
            if reached or stalled or timeout:
                dip['phase'] = 'up'
                dip['phase_t0'] = now
                self.get_logger().info(
                    f'lift 到底（离起点 {travel:.1f}°），开始升回 {dip["start_lift"]:.1f}°'
                )
                lift = -self._dip_dir * self._dip_up_vel
            else:
                lift = self._dip_dir * self._dip_down_vel
        else:  # up：往起点回，接近时线性减速，到达/越过起点即结束
            crossed = dip['side'] != 0.0 and delta * dip['side'] < 0.0
            timeout = (now - dip['phase_t0']) > self._dip_timeout
            if travel <= 3.0 or crossed or timeout:
                lift = 0.0
                self._dip = None
                self._phase = 'wait_stop'
                self._center_since = 0.0
                self._dip_cooldown_until = now + self._dip_cooldown
                self._pid_yaw.reset()
                self._pid_extend.reset()
                self.get_logger().info(
                    f'下探完成：lift 回到 {lift_cur:.1f}°（目标 {dip["start_lift"]:.1f}°）'
                )
            else:
                if travel < self._dip_approach:
                    vel = max(self._dip_creep_vel,
                              self._dip_up_vel * travel / max(1e-6, self._dip_approach))
                else:
                    vel = self._dip_up_vel
                lift = -self._dip_dir * vel

        # 下探期间 yaw/extend 冻结，只动 lift
        self._pub.publish(Float32MultiArray(data=[0.0, float(lift), 0.0]))


def main(args=None):
    rclpy.init(args=args)
    node = GimbalTrackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
