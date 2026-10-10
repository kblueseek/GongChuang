#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""运动规划节点 = 任务状态机（执行一段"指令序列"）。

================ 大白话：这个节点在干什么 ================

比赛要做的动作（读码、走、转、抓、放……）本质就是一条一条"动作指令"按顺序执行。
这个节点就是那个"执行器"：它读一份指令序列（JSON），一条一条往下做，
每做一条之前先等上一条"到位"，全部做完就停下。

它不自己算"车该往哪走、转向多少"——那是上位机/你在赛前编辑好的。
它只负责：
  1. 把"前进 0.5 米"翻译成世界系里的一个目标点 (x, y) + 目标朝向；
  2. 把目标点通过 /cmd_pose 发给 F407（位置闭环、航向闭环在 F407 上跑）；
  3. 读 /odom 里程计，判断"到了没"，到了就执行下一条。

指令序列的格式（JSON）。相对指令最直观，执行时自动累计成绝对位姿：

  move     {"cmd":"move",   "dist_m":0.5}      前进(负=后退)
  strafe   {"cmd":"strafe", "dist_m":0.3}      横向平移(正=左)
  turn     {"cmd":"turn",   "deg":90}          原地转(正=逆时针/左转，负=右转)
  gimbal   {"cmd":"gimbal", "yaw_deg":-30, "lift_deg":50}  云台轴角度(可缺省)
  gripper  {"cmd":"gripper","deg":45}          夹爪舵机
  plate    {"cmd":"plate",  "deg":90}          载物盘舵机
  cam      {"cmd":"cam",    "deg":0}           相机舵机
  enable   {"cmd":"enable", "en":true}         使能/失能电机
  delay    {"cmd":"delay",  "ms":1000}         原地等多少毫秒
  calibrate{"cmd":"calibrate"}                 视觉校准钩子(占位，待接圆环识别)
  comment  {"cmd":"comment","text":"..."}      注释，什么都不做

坐标约定：车正前方为 +x，左侧为 +y，逆时针为正。和 F407 的里程计/陀螺仪一致。
"""

import json        # 解析/生成 JSON（指令序列就是 JSON）
import math        # 三角函数、弧度换算

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D               # 平面位姿：x, y, theta(弧度)
from nav_msgs.msg import Odometry                  # 里程计消息
from std_msgs.msg import Bool, Float32MultiArray, String


class MotionPlannerNode(Node):
    def __init__(self):
        super().__init__('motion_planner')   # 节点名

        # ---- 可调参数 ----
        # 到位判定阈值：位置误差 < 2cm，朝向误差 < 0.05rad(约3°)
        self.declare_parameter('arrive_tol_xy', 0.02)     # 米
        self.declare_parameter('arrive_tol_yaw', 0.05)    # 弧度
        # 启动时自动加载的序列文件路径（空 = 不加载，等上位机下发）
        self.declare_parameter('sequence_file', '')

        self.arrive_tol_xy = float(self.get_parameter('arrive_tol_xy').value)
        self.arrive_tol_yaw = float(self.get_parameter('arrive_tol_yaw').value)
        sequence_file = str(self.get_parameter('sequence_file').value)

        # ---- 发布的话题（本节点 -> 其他节点） ----
        self._pub_pose = self.create_publisher(Pose2D, '/cmd_pose', 10)                     # 位姿目标
        self._pub_servo = self.create_publisher(Float32MultiArray, '/cmd_servo', 10)        # 舵机 [夹爪,托盘,相机]
        self._pub_gimbal = self.create_publisher(Float32MultiArray, '/cmd_gimbal', 10)      # 云台 [yaw,lift,extend]
        self._pub_enable = self.create_publisher(Bool, '/cmd_enable', 10)                   # 使能
        self._pub_state = self.create_publisher(String, '/motion_state', 10)                # 进度状态(给上位机/屏幕看)

        # ---- 订阅的话题（其他节点 -> 本节点） ----
        self.create_subscription(Odometry, '/odom', self._cb_odom, 10)          # 里程计：判断到位
        self.create_subscription(String, '/motion_load', self._cb_load, 10)     # 上位机下发序列
        self.create_subscription(Bool, '/motion_stop', self._cb_stop, 10)       # 上位机叫停

        # ---- 内部状态 ----
        self.odom = None            # 最新里程计 (x, y, theta)，单位 米/米/弧度
        self.commands = []          # 指令序列（list of dict）
        self.index = -1             # 当前执行到第几条（-1 = 还没开始）
        self.phase = 'idle'         # 状态机状态：idle/moving/waiting/stepping
        self.deadline = 0.0         # delay 指令的到期时间

        # 绝对位姿目标（世界系）：从"当前里程计"出发，逐条指令累计
        self.tx = 0.0
        self.ty = 0.0
        self.tyaw = 0.0

        # 舵机角度镜像（F407 上电默认：夹爪90 托盘0 相机90），供 plate/cam 单独设值时补齐整组
        self.servo = [90.0, 0.0, 90.0]

        # 若有启动参数指定的序列文件，先加载（不自动开始，等上位机下发 start）
        if sequence_file:
            self._load_file(sequence_file)

        # 执行器：每 0.1 秒推进一步（10Hz）
        self._timer = self.create_timer(0.1, self._tick)
        self.get_logger().info('状态机就绪，等待 /motion_load 下发指令序列')

    # ---------------- 订阅回调 ----------------

    def _cb_odom(self, msg: Odometry):
        """里程计：只记录 x、y 和朝向角 theta（从四元数里反解出来）。"""
        q = msg.pose.pose.orientation
        # 绕 Z 轴的偏航角 = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
        theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, theta)

    def _cb_load(self, msg: String):
        """上位机下发序列：解析 JSON，立刻从"当前所在位置"开始执行。"""
        self._load_json(msg.data)

    def _cb_stop(self, msg: Bool):
        """上位机叫停：停在原地（把当前位置设为目标=锁定），状态机回到 idle。"""
        if bool(msg.data):
            self.phase = 'idle'
            self.index = -1
            self._publish_state('已停止')
            # 把当前位置设为目标位姿发给 F407，让它"锁"在原地不动
            if self.odom is not None:
                self._publish_pose(self.odom[0], self.odom[1], self.odom[2])

    # ---------------- 序列加载 ----------------

    def _load_file(self, path: str):
        """从文件读指令序列（启动参数指定时用）。"""
        try:
            with open(path, 'r', encoding='utf-8') as f:
                self._load_json(f.read())
        except Exception as e:
            self.get_logger().error(f'读序列文件失败 {path}: {e}')

    def _load_json(self, text: str):
        """解析一段 JSON 文本为指令序列，并立刻开始执行。"""
        try:
            data = json.loads(text)
        except Exception as e:
            self._publish_state(f'JSON 解析失败: {e}')
            self.get_logger().error(f'JSON 解析失败: {e}')
            return
        # 支持两种写法：直接是命令数组，或 {"commands": [...]}
        self.commands = data['commands'] if isinstance(data, dict) else data
        if not isinstance(self.commands, list):
            self._publish_state('序列格式错误：应为命令数组')
            return
        self._publish_state(f'已加载 {len(self.commands)} 条指令，等待开始')
        self.get_logger().info(f'加载序列：{len(self.commands)} 条')
        self._start()

    def _start(self):
        """开始执行：把绝对目标位姿初始化为"当前里程计位置"，从第 0 条开始。"""
        if self.odom is None:
            self._publish_state('还没收到里程计(F407 未回传)，无法开始，请检查串口')
            self.get_logger().error('无里程计，无法开始')
            return
        self.tx, self.ty, self.tyaw = self.odom   # 从当前真实位置出发
        self.index = -1
        self.phase = 'stepping'                     # 进入执行循环
        self._publish_state('开始执行')

    # ---------------- 状态机主循环 ----------------

    def _tick(self):
        """每 0.1 秒调一次，推进状态机。"""
        if self.phase == 'idle':
            return
        if self.phase == 'moving':
            if not self._arrived():      # 还没到位，继续等
                return
            self.phase = 'stepping'      # 到位了，去取下一条
        elif self.phase == 'waiting':
            if self._now() < self.deadline:   # 还没到时间，继续等
                return
            self.phase = 'stepping'

        # 连续执行：不阻塞的指令（舵机/云台/注释…）一条接一条，遇到阻塞的（走/转/延时）就停下等
        while self.phase == 'stepping':
            self.index += 1
            if self.index >= len(self.commands):
                self._finish()
                return
            self._execute(self.commands[self.index])

    def _execute(self, cmd: dict):
        """执行一条指令。返回后 self.phase 会是：
           - moving/waiting：阻塞型，等完成
           - stepping：非阻塞型，继续下一条
        """
        name = cmd.get('cmd')
        if name == 'move':            # 前进/后退
            d = float(cmd.get('dist_m', 0.0))
            self.tx += d * math.cos(self.tyaw)
            self.ty += d * math.sin(self.tyaw)
            self._publish_pose(self.tx, self.ty, self.tyaw)
            self._publish_state(f'[{self.index+1}/{len(self.commands)}] 前进 {d:+.2f}m')
            self.phase = 'moving'
        elif name == 'strafe':        # 横向平移（正=左）
            d = float(cmd.get('dist_m', 0.0))
            self.tx += d * math.cos(self.tyaw + math.pi / 2.0)
            self.ty += d * math.sin(self.tyaw + math.pi / 2.0)
            self._publish_pose(self.tx, self.ty, self.tyaw)
            self._publish_state(f'[{self.index+1}/{len(self.commands)}] 横移 {d:+.2f}m')
            self.phase = 'moving'
        elif name == 'turn':          # 原地转
            d = math.radians(float(cmd.get('deg', 0.0)))
            self.tyaw += d
            self._publish_pose(self.tx, self.ty, self.tyaw)
            self._publish_state(f'[{self.index+1}/{len(self.commands)}] 转向 {cmd.get("deg", 0):+.0f}°')
            self.phase = 'moving'
        elif name == 'delay':         # 原地等待
            self.deadline = self._now() + float(cmd.get('ms', 0)) / 1000.0
            self._publish_state(f'[{self.index+1}/{len(self.commands)}] 等待 {cmd.get("ms", 0)}ms')
            self.phase = 'waiting'
        elif name == 'gimbal':        # 云台轴（不阻塞，发了就走）
            a = Float32MultiArray()
            a.data = [float(cmd.get('yaw_deg', 0.0)),
                      float(cmd.get('lift_deg', 0.0)),
                      float(cmd.get('extend_deg', 0.0))]
            self._pub_gimbal.publish(a)
            self._publish_state(f'[{self.index+1}/{len(self.commands)}] 云台 yaw={a.data[0]}° lift={a.data[1]}°')
            # 不阻塞：phase 保持 stepping
        elif name in ('gripper', 'plate', 'cam'):   # 三个舵机
            idx = {'gripper': 0, 'plate': 1, 'cam': 2}[name]
            self.servo[idx] = float(cmd.get('deg', 0.0))
            s = Float32MultiArray()
            s.data = list(self.servo)               # 整组下发，避免缺值
            self._pub_servo.publish(s)
            self._publish_state(f'[{self.index+1}/{len(self.commands)}] {name}={self.servo[idx]}°')
        elif name == 'enable':        # 使能/失能
            b = Bool()
            b.data = bool(cmd.get('en', True))
            self._pub_enable.publish(b)
            self._publish_state(f'[{self.index+1}/{len(self.commands)}] 使能={b.data}')
        elif name == 'calibrate':     # 视觉校准钩子（占位）
            self._calibrate()
            self._publish_state(f'[{self.index+1}/{len(self.commands)}] 视觉校准(占位)')
        elif name == 'comment':       # 注释，跳过
            self._publish_state(f'[{self.index+1}/{len(self.commands)}] 注释: {cmd.get("text", "")}')
        else:
            self.get_logger().warn(f'未知指令被跳过: {cmd}')

    def _calibrate(self):
        """视觉校准占位：以后在这里接入"圆环在画面中的位置 -> 修正 tx/ty/tyaw"。
        现在什么都不做，仅打一条日志。"""
        self.get_logger().info('视觉校准：占位实现，待接入圆环识别')

    # ---------------- 工具函数 ----------------

    def _arrived(self) -> bool:
        """判断里程计是否到达绝对目标 (tx, ty, tyaw)。"""
        if self.odom is None:
            return False
        ox, oy, ot = self.odom
        dx = ox - self.tx
        dy = oy - self.ty
        dh = abs(self._angle_diff(ot, self.tyaw))
        return (dx * dx + dy * dy) < self.arrive_tol_xy ** 2 and dh < self.arrive_tol_yaw

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        """两个角度的差，结果归一化到 (-π, π]（处理 ±180° 环绕）。"""
        return (a - b + math.pi) % (2.0 * math.pi) - math.pi

    def _publish_pose(self, x, y, yaw):
        p = Pose2D()
        p.x = float(x)
        p.y = float(y)
        p.theta = float(yaw)
        self._pub_pose.publish(p)

    def _publish_state(self, text: str):
        s = String()
        s.data = text
        self._pub_state.publish(s)

    def _finish(self):
        self.phase = 'idle'
        self._publish_state('全部完成')

    @staticmethod
    def _now() -> float:
        """当前时间（秒）。"""
        import time
        return time.monotonic()


def main(args=None):
    rclpy.init(args=args)
    node = MotionPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
