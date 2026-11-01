#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""云台视觉跟随 demo：物料像素坐标 -> yaw/extend 速度（PID）。

手柄的 select 键（joy_control）会把 /vision_enable 置 True，同时让出 /cmd_gimbal_vel；
本节点在这期间接管 /cmd_gimbal_vel，用 PID 把物料质心往画面中心拉，其余时间
完全不发速度，绝不和手柄抢。

映射（反了改 dir_* 翻号，不用动别的）：
    水平像素误差 error_x = cx - 画面宽/2  ->  yaw 角速度
    竖直像素误差 error_y = cy - 画面高/2  ->  extend 速度
    lift 恒为 0（先不做升降）

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
  max_yaw_deg_s / max_extend_deg_s   输出限幅（°/s）
  dir_yaw / dir_extend         输出方向（±1，发散就翻）
  deadband_px                  误差小于这个像素数就停
  rate_hz                      控制频率（要 >= 20Hz，否则 F407 300ms 超时会自己停）
  target_timeout_s             /material_target 多久没更新就算丢目标（发零速停住）
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray


class PID:
    """带输出限幅 + 积分限幅的小 PID。默认只用 P（ki=kd=0），需要再加。"""

    def __init__(self, kp, ki, kd, limit):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.limit = float(limit)
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_err = 0.0

    def update(self, err, dt):
        if dt <= 0.0:
            dt = 0.01
        self.integral += err * dt
        if self.ki != 0.0:
            bound = self.limit / abs(self.ki)
            self.integral = max(-bound, min(bound, self.integral))
        deriv = (err - self.prev_err) / dt
        self.prev_err = err
        out = self.kp * err + self.ki * self.integral + self.kd * deriv
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

        self._dir_yaw = float(self.get_parameter('dir_yaw').value)
        self._dir_extend = float(self.get_parameter('dir_extend').value)
        self._deadband = float(self.get_parameter('deadband_px').value)
        self._timeout = float(self.get_parameter('target_timeout_s').value)

        self._pid_yaw = PID(
            self.get_parameter('kp_yaw').value,
            self.get_parameter('ki_yaw').value,
            self.get_parameter('kd_yaw').value,
            float(self.get_parameter('max_yaw_deg_s').value),
        )
        self._pid_extend = PID(
            self.get_parameter('kp_extend').value,
            self.get_parameter('ki_extend').value,
            self.get_parameter('kd_extend').value,
            float(self.get_parameter('max_extend_deg_s').value),
        )

        # 状态
        self._enabled = False
        self._target = None          # (cx, cy)
        self._img_w = 1280
        self._img_h = 720
        self._target_s = 0.0
        self._last_t = time.monotonic()

        self.create_subscription(Bool, '/vision_enable', self._cb_enable, 10)
        self.create_subscription(Float32MultiArray, '/material_target', self._cb_target, 10)
        self._pub = self.create_publisher(Float32MultiArray, '/cmd_gimbal_vel', 10)

        rate = max(10.0, float(self.get_parameter('rate_hz').value))
        self._timer = self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            'gimbal_track 就绪：kp_yaw={:.3f} kp_extend={:.3f} '
            'max [yaw {:.0f}°, extend {:.0f}°/s]。select 键切换接管。'.format(
                self._pid_yaw.kp, self._pid_extend.kp,
                self._pid_yaw.limit, self._pid_extend.limit)
        )

    # ---------------- 回调 ----------------

    def _cb_enable(self, msg: Bool):
        enabled = bool(msg.data)
        if enabled == self._enabled:
            return
        self._enabled = enabled
        if enabled:
            self._pid_yaw.reset()
            self._pid_extend.reset()
            self.get_logger().info('视觉接管：开始跟随物料')
        else:
            self._pid_yaw.reset()
            self._pid_extend.reset()
            self.get_logger().info('视觉退出：控制权交还手柄')

    def _cb_target(self, msg: Float32MultiArray):
        data = msg.data
        if len(data) < 5:
            return
        detected = data[0] > 0.5
        self._img_w = float(data[3]) if data[3] > 0 else self._img_w
        self._img_h = float(data[4]) if data[4] > 0 else self._img_h
        self._target_s = time.monotonic()
        self._target = (float(data[1]), float(data[2])) if detected else None

    # ---------------- 定时器 ----------------

    def _tick(self):
        now = time.monotonic()
        dt = now - self._last_t
        self._last_t = now

        # 没接管时一个速度都不发，让手柄正常控制
        if not self._enabled:
            return

        # 丢目标 / 超时 -> 停住，别按最后一拍误差继续冲
        if self._target is None or (now - self._target_s) > self._timeout:
            self._pid_yaw.reset()
            self._pid_extend.reset()
            self._pub.publish(Float32MultiArray(data=[0.0, 0.0, 0.0]))
            return

        cx, cy = self._target
        error_x = cx - self._img_w / 2.0
        error_y = cy - self._img_h / 2.0

        if abs(error_x) < self._deadband and abs(error_y) < self._deadband:
            yaw = 0.0
            extend = 0.0
        else:
            yaw = self._pid_yaw.update(error_x, dt) * self._dir_yaw
            extend = self._pid_extend.update(error_y, dt) * self._dir_extend

        # 三轴顺序 [yaw, lift, extend]，lift 恒 0
        self._pub.publish(Float32MultiArray(data=[float(yaw), 0.0, float(extend)]))


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
