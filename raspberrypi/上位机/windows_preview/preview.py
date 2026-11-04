#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""上位机 Windows 离线预览。

不装 ROS、不接树莓派、不接 F407，直接在本机把界面跑起来，并模拟整条链路：

    /motion_load  → 这里按 goto/move/turn/delay... 一步步“开”车
    /odom、/robot_status、/gimbal_state  → 回喂给界面，地图和波形会动
    /cmd_tune     → 立刻回显，PID 页能看到“407 已确认”

只用于看界面/演示流程，**不会驱动任何真实硬件**。
树莓派上仍用 scripts/run_gui.sh，两者互不影响。
"""

import json
import logging
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
GUI_DIR = os.path.dirname(HERE)

# 先注入空壳 rclpy/消息包，再导入真正的 motion_gui
sys.path.insert(0, os.path.join(HERE, 'stubs'))
sys.path.insert(0, GUI_DIR)

import tkinter as tk

import rclpy
import motion_gui
from motion_gui import GuiRosNode
from car_interfaces.msg import Pose2D
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32MultiArray, Int32, Int32MultiArray, String

log = logging.getLogger('preview')

TICK_MS = 50            # 模拟器步长
MAX_V = 0.9             # m/s，演示用，比实车快一点
MAX_W_DEG = 120.0       # °/s
DELAY_SCALE = 0.35      # 动作里的等待时间按比例缩短，演示不干等
TURN_SETTLE = 0.15      # s，到点停稳后再原地转（和 motion_planner 一致）
GIMBAL_RATE = 150.0     # °/s
SCRIPT_WAIT = 0.8       # script/align/vision_pick 在预览里只停一下
EPS_M = 0.004
EPS_DEG = 0.6


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


class PreviewSim:
    """假的 motion_planner + F407，全在主线程里跑（用 tk.after 驱动）。"""

    def __init__(self, node, root):
        self.node = node
        self.root = root

        # ---- 假车状态（里程计系：米 / 弧度）----
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.target = None          # (x, y, yaw_rad)，正在逼近的位姿
        self.wait_yaw = True        # 到点是否必须等车头也转好（中间航点为 False）
        self.hold_yaw = 0.0         # 行进段锁住的车头朝向（出发时的 yaw）
        self.sub = 'move'           # move / settle / turn：先走到位，再原地转
        self.sub_until = 0.0        # settle 的结束时刻
        self.gimbal = [0.0, 0.0, 0.0]
        self.gimbal_tgt = [0.0, 0.0, 0.0]
        self.gripper = 120.0

        # ---- 指令执行 ----
        self.cmds = []
        self.i = 0
        self.wait_until = 0.0
        self.state = 'idle'

        # ---- 轮询 GUI 发出去的消息 ----
        self.load_seen = 0
        self.stop_seen = 0
        self.tune_seen = 0
        self.zero_seen = 0
        self.limit_seen = 0

        self.last_tick = time.monotonic()
        self._after = None

        # 上车即“在线、就绪、4 个电机都在”
        self.node._cb_online(Bool(True))
        self.node._cb_status(Int32MultiArray([1, 0, 0x0F]))
        self._publish_odom()
        self._publish_target()

        # 假 407 报一次已标定的限位，PID 页的零点/限位控件有东西可看
        for axis, (lo, hi) in enumerate(
                ((-1000.0, 1000.0), (-875.0, 875.0), (-580.0, 580.0))):
            self.node._cb_gimbal_limits(Float32MultiArray([axis, 1.0, lo, hi]))

        self._tick()

    # ---------- 主循环 ----------

    def _tick(self):
        now = time.monotonic()
        dt = min(now - self.last_tick, 0.2)
        self.last_tick = now
        try:
            self._drain_outbox()
            self._exec(now, dt)
            self._publish_odom()
            self._feed_gimbal(dt)
            self._feed_status()
        except Exception:
            log.exception('预览模拟器出错')
        self._after = self.root.after(TICK_MS, self._tick)

    def stop(self):
        if self._after is not None:
            try:
                self.root.after_cancel(self._after)
            except Exception:
                pass
            self._after = None

    # ---------- 收 GUI 发出去的指令 ----------

    def _drain_outbox(self):
        # /motion_load：新的任务序列直接替换旧的
        loads = self.node.pub_load.sent
        if len(loads) > self.load_seen:
            self.load_seen = len(loads)
            try:
                cmds = json.loads(loads[-1].data)
            except Exception:
                log.warning('motion_load 不是合法 JSON，忽略')
                return
            self.cmds = [c for c in cmds if isinstance(c, dict)]
            self.i = 0
            self.target = None
            self.sub = 'move'
            self.wait_until = 0.0
            self._set_state('moving' if self.cmds else 'idle')

        # /motion_stop
        stops = self.node.pub_stop.sent
        if len(stops) > self.stop_seen:
            self.stop_seen = len(stops)
            if stops[-1].data:
                self.cmds = []
                self.i = 0
                self.target = None
                self.sub = 'move'
                self.wait_until = 0.0
                self._set_state('idle')

        # /cmd_tune：模拟 407 立即回显
        tunes = self.node.pub_tune.sent
        while self.tune_seen < len(tunes):
            m = tunes[self.tune_seen]
            self.tune_seen += 1
            if len(m.data) >= 2:
                self.node._cb_tune_state(
                    Float32MultiArray(data=[m.data[0], m.data[1]]))

        # /cmd_gimbal_zero：预览里直接把被选中的轴当成新的 0 点
        zeros = self.node.pub_gimbal_zero.sent
        while self.zero_seen < len(zeros):
            mask = int(zeros[self.zero_seen].data) & 0x07
            self.zero_seen += 1
            for k in range(3):
                if mask & (1 << k):
                    self.gimbal[k] = 0.0
                    self.gimbal_tgt[k] = 0.0

        # /cmd_gimbal_limit：模拟 407 接受并在下一帧回传（界面输入框不会跳回旧值）
        limits = self.node.pub_gimbal_limit.sent
        while self.limit_seen < len(limits):
            m = limits[self.limit_seen]
            self.limit_seen += 1
            if len(m.data) >= 3:
                # F407 用 int16 ×0.1° 存限位（向零截断），这里照样模拟
                lo = int(float(m.data[1]) * 10.0) / 10.0
                hi = int(float(m.data[2]) * 10.0) / 10.0
                self.node._cb_gimbal_limits(
                    Float32MultiArray([float(m.data[0]), 1.0, lo, hi]))

    # ---------- 执行指令 ----------

    def _exec(self, now, dt):
        if self.i >= len(self.cmds):
            if self.cmds:
                self.cmds = []
                self.target = None
                self._set_state('idle')
            return
        if now < self.wait_until:
            return

        cmd = self.cmds[self.i]
        name = cmd.get('cmd')

        if name == 'comment':
            self.i += 1
            return

        if name == 'delay':
            ms = float(cmd.get('ms', 0) or 0)
            self.wait_until = now + max(0.0, ms) / 1000.0 * DELAY_SCALE
            self.i += 1
            return

        if name in ('goto', 'move', 'strafe', 'turn'):
            if self.target is None:
                self.target = self._target_of(cmd)
                self.wait_yaw = bool(cmd.get('arrive_yaw', True))
                # 先走到位、再转向：行进段锁住出发朝向，到点停稳后才原地转。
                self.hold_yaw = self.yaw
                self.sub = 'move'
                self._set_state('moving')
            if self._step_toward(now, dt):
                self.i += 1
                self.target = None
            else:
                self._publish_target()
            return

        if name == 'gimbal':
            self.gimbal_tgt = [float(cmd.get(k, 0) or 0)
                               for k in ('yaw_deg', 'lift_deg', 'extend_deg')]
            self.node._cb_gimbal_target(Float32MultiArray(list(self.gimbal_tgt)))
            self.i += 1
            return

        if name == 'gripper':
            self.gripper = float(cmd.get('deg', 0) or 0)
            self.i += 1
            return

        if name in ('pick', 'place'):
            # 预览里不展开宏，只动云台/夹爪并停一下，看起来像在作业
            self.gimbal_tgt = [float(cmd.get('yaw_deg', 0) or 0),
                               float(cmd.get('lift_deg', 0) or 0),
                               float(cmd.get('extend_deg', 0) or 0)]
            self.node._cb_gimbal_target(Float32MultiArray(list(self.gimbal_tgt)))
            self.gripper = float(cmd.get('gripper_close' if name == 'pick'
                                         else 'gripper_open', 120) or 120)
            settle = float(cmd.get('settle_ms', 800) or 800) / 1000.0
            self.wait_until = now + settle * DELAY_SCALE + 0.4
            self.i += 1
            return

        if name in ('script', 'align', 'vision_pick', 'calibrate'):
            # 真机上会启程序/等视觉结果；预览里只停一下，避免卡住演示
            self.wait_until = now + SCRIPT_WAIT
            self.i += 1
            return

        # enable / plate / cam / 未知类型：预览里直接跳过
        self.i += 1

    def _target_of(self, cmd):
        name = cmd.get('cmd')
        if name == 'goto':
            return (float(cmd.get('x_m', 0) or 0),
                    float(cmd.get('y_m', 0) or 0),
                    math.radians(float(cmd.get('yaw_deg', 0) or 0)))
        if name == 'move':
            d = float(cmd.get('dist_m', 0) or 0)
            return (self.x + d * math.cos(self.yaw),
                    self.y + d * math.sin(self.yaw), self.yaw)
        if name == 'strafe':
            d = float(cmd.get('dist_m', 0) or 0)
            a = self.yaw + math.pi / 2.0
            return (self.x + d * math.cos(a), self.y + d * math.sin(a), self.yaw)
        if name == 'turn':
            return (self.x, self.y,
                    self.yaw + math.radians(float(cmd.get('deg', 0) or 0)))
        return (self.x, self.y, self.yaw)

    def _step_toward(self, now, dt):
        """先平移到位（车头锁住不转），停稳后再原地转向；全部完成返回 True。"""
        tx, ty, tyaw = self.target
        dx, dy = tx - self.x, ty - self.y
        dist = math.hypot(dx, dy)

        if self.sub == 'move':
            if dist > EPS_M:
                step = min(MAX_V * dt, dist)
                self.x += dx / dist * step
                self.y += dy / dist * step
                return False
            self.x, self.y = tx, ty
            if not self.wait_yaw or abs(wrap_deg(math.degrees(tyaw - self.yaw))) <= EPS_DEG:
                self.yaw = tyaw
                return True
            # 位置到了但要转向：先停稳
            self.sub = 'settle'
            self.sub_until = now + TURN_SETTLE
            return False

        if self.sub == 'settle':
            if now < self.sub_until:
                return False
            self.sub = 'turn'
            return False

        # turn：原地转，位置保持不变
        dyaw = wrap_deg(math.degrees(tyaw - self.yaw))
        if abs(dyaw) <= EPS_DEG:
            self.yaw = tyaw
            return True
        wstep = MAX_W_DEG * dt
        self.yaw += math.radians(max(-wstep, min(wstep, dyaw)))
        self.yaw = math.radians(wrap_deg(math.degrees(self.yaw)))
        return False

    # ---------- 回喂界面 ----------

    def _publish_odom(self):
        m = Odometry()
        m.pose.pose.position.x = self.x
        m.pose.pose.position.y = self.y
        q = m.pose.pose.orientation
        q.z = math.sin(self.yaw / 2.0)
        q.w = math.cos(self.yaw / 2.0)
        self.node._cb_odom(m)

    def _publish_target(self):
        tx, ty, tyaw = self.target or (self.x, self.y, self.yaw)
        if self.target is not None and self.sub in ('move', 'settle'):
            tyaw = self.hold_yaw     # 行进/停稳阶段车头锁在出发朝向
        m = Pose2D()
        m.x, m.y, m.theta = tx, ty, tyaw
        self.node._cb_pose_target(m)

    def _feed_gimbal(self, dt):
        step = GIMBAL_RATE * dt
        for k in range(3):
            d = self.gimbal_tgt[k] - self.gimbal[k]
            if abs(d) <= step:
                self.gimbal[k] = self.gimbal_tgt[k]
            else:
                self.gimbal[k] += step if d > 0 else -step
        self.node._cb_gimbal_state(Float32MultiArray(list(self.gimbal)))

    def _feed_status(self):
        self.node._cb_online(Bool(True))
        self.node._cb_status(Int32MultiArray([1, 0, 0x0F]))

    def _set_state(self, text):
        if text != self.state:
            self.state = text
            self.node._cb_state(String(text))


def clamp_window(root):
    """窗口按实际屏幕收一下。

    界面是按 1500×940 设计的；笔记本开 125% 缩放时逻辑高度可能只有 800 多，
    底部【执行/停止】会被挤出屏幕。左栏本身可滚动，收小后仍能操作。
    """
    sw, sh = root.winfo_screenwidth(), root.winfo_screenheight()
    root.geometry(f'{min(1500, sw - 40)}x{min(940, sh - 80)}')


def main():
    logging.basicConfig(level=logging.INFO,
                        format='%(levelname)s %(name)s: %(message)s')
    rclpy.init()
    node = GuiRosNode()

    root = tk.Tk()
    app = motion_gui.App(root, node)
    clamp_window(root)

    sim = PreviewSim(node, root)
    root.title(root.title() + ' ｜ Windows 预览：模拟数据，不会驱动真车')

    def on_close():
        try:
            sim.stop()
            app.close()
        finally:
            node.destroy_node()
            rclpy.shutdown()
            root.destroy()

    root.protocol('WM_DELETE_WINDOW', on_close)
    root.mainloop()


if __name__ == '__main__':
    main()
