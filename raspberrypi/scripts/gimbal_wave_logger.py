#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""云台波形记录器：跑原料区动作时在另一个终端录 /gimbal_state + 指令。

不接管控制、不发任何指令，只订阅 + 写 CSV。录完用
scripts/gimbal_pid_analyze.py 分析，调 GimbalTask 的位置环 PID（上位机
PID 页的 gim_yaw_kp / gim_yaw_kd）或 track_pid.json 的视觉跟踪 PID。

用法（先 source 好 ROS2 和本工作区）：
    source /opt/ros/lyrical/setup.bash
    source raspberrypi/ros2_ws/install/setup.bash
    python3 raspberrypi/scripts/gimbal_wave_logger.py --out /tmp/gimbal_wave.csv

然后在另一个终端正常跑原料区动作，做完 Ctrl+C 停止记录。
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

HEADER = (
    't', 'yaw', 'lift', 'ext',
    'cmd_yaw', 'cmd_lift', 'cmd_ext', 'cmd_age_ms',
    'vel_yaw', 'vel_lift', 'vel_ext', 'vel_age_ms',
    'odom_x', 'odom_y', 'odom_yaw_deg', 'state',
)


def _f(values, index):
    if values is None or len(values) <= index:
        return None
    return float(values[index])


class GimbalWaveLogger(Node):
    def __init__(self, out_path: Path, rate_hz: float):
        super().__init__('gimbal_wave_logger')
        self._t0 = time.monotonic()
        self._gimbal = None          # (yaw, lift, ext)
        self._cmd = None             # (yaw, lift, ext)
        self._cmd_t = 0.0
        self._vel = None             # (yaw, lift, ext) °/s
        self._vel_t = 0.0
        self._odom = None            # (x, y, yaw_deg)
        self._state = ''

        self.create_subscription(Float32MultiArray, '/gimbal_state',
                                 self._cb_gimbal, 10)
        self.create_subscription(Float32MultiArray, '/cmd_gimbal',
                                 self._cb_cmd, 10)
        self.create_subscription(Float32MultiArray, '/cmd_gimbal_vel',
                                 self._cb_vel, 10)
        self.create_subscription(Odometry, '/odom', self._cb_odom, 10)
        self.create_subscription(String, '/motion_state', self._cb_state, 10)

        self._file = out_path.open('w', newline='', encoding='utf-8')
        self._writer = csv.writer(self._file)
        self._writer.writerow(HEADER)
        self._file.flush()
        self._n = 0
        self.create_timer(1.0 / max(1.0, rate_hz), self._tick)
        self.get_logger().info(f'云台波形记录到 {out_path}（Ctrl+C 停止）')

    # ---------- 订阅 ----------
    def _cb_gimbal(self, msg):
        if len(msg.data) >= 3:
            self._gimbal = tuple(float(v) for v in msg.data[:3])

    def _cb_cmd(self, msg):
        if len(msg.data) >= 3:
            self._cmd = tuple(float(v) for v in msg.data[:3])
            self._cmd_t = time.monotonic()

    def _cb_vel(self, msg):
        if len(msg.data) >= 3:
            self._vel = tuple(float(v) for v in msg.data[:3])
            self._vel_t = time.monotonic()

    def _cb_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._odom = (msg.pose.pose.position.x,
                      msg.pose.pose.position.y, math.degrees(yaw))

    def _cb_state(self, msg):
        self._state = (msg.data or '').replace('\n', ' ')[:120]

    # ---------- 写行 ----------
    def _tick(self):
        now = time.monotonic()
        t = now - self._t0
        g = self._gimbal or (None, None, None)
        c = self._cmd or (None, None, None)
        v = self._vel or (None, None, None)
        o = self._odom or (None, None, None)
        cmd_age = (now - self._cmd_t) * 1000.0 if self._cmd_t else None
        vel_age = (now - self._vel_t) * 1000.0 if self._vel_t else None

        def fmt(x, nd=3):
            return '' if x is None else f'{x:.{nd}f}'

        self._writer.writerow((
            f'{t:.4f}', fmt(g[0]), fmt(g[1]), fmt(g[2]),
            fmt(c[0]), fmt(c[1]), fmt(c[2]), fmt(cmd_age, 1),
            fmt(v[0]), fmt(v[1]), fmt(v[2]), fmt(vel_age, 1),
            fmt(o[0], 4), fmt(o[1], 4), fmt(o[2], 2), self._state,
        ))
        self._n += 1
        if self._n % 10 == 0:
            self._file.flush()

    def close(self):
        try:
            self._file.flush()
            self._file.close()
        except OSError:
            pass


def main():
    ap = argparse.ArgumentParser(description='云台波形记录器（只订阅，不控制）')
    ap.add_argument('--out', default='/tmp/gimbal_wave.csv')
    ap.add_argument('--rate', type=float, default=100.0, help='采样率 Hz')
    args = ap.parse_args()

    rclpy.init()
    node = GimbalWaveLogger(Path(args.out), args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(f'已保存 {args.out}')


if __name__ == '__main__':
    main()
