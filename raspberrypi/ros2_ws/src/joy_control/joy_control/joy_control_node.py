#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""手柄控制节点：把 USB 手柄（WHEELTEC TGZ / 兼容 Xbox 的国产手柄）翻译成整车控制话题。

================ 编号来源（重要） ================
同一个手柄（WHEELTEC TGZ Controller / 兼容 Xbox 的国产手柄）固件里有**两套模式**，
长按 MODE 键切换，插拔/换模式后连内核报的名字和编号都会变：

  【PS2 模式】6 个轴，内核名 "TGZ Controller"（VID:PID 0079:0126，usbhid）
    轴 0=左摇杆左右  1=左摇杆上下  2=右摇杆左右  3=右摇杆上下
    轴 4=方向键左右  5=方向键上下
    键（逐项实测）：0=Y 1=B 2=A 3=X 4=L1 5=R1 6=L2 7=R2 8=select 9=start
    注意面键顺序是 Y/B/A/X，不是常见的 A/B/X/Y；L2/R2 在这里是按键，不是轴。
    指示灯颜色：绿色 = 摇杆连续；红色 = 摇杆分档（实测，两者轴/键编号相同）。

  【Xbox 模式】8 个轴，内核名 "Microsoft X-Box 360 pad"（xpad 类）
    轴 0=左摇杆左右  1=左摇杆上下  2=L2 扳机  3=右摇杆左右
    轴 4=右摇杆上下  5=R2 扳机  6=方向键左右  7=方向键上下
    键 0=A 1=B 2=X 3=Y 4=L1 5=R1 6=select 7=start 8=mode

本节点默认 layout:=auto，按 /joy 的轴数量自动判断（<=6 轴=PS2，>=7 轴=Xbox），
两种模式成套切换，绝不要把一套的编号套到另一套上。
第三方"手柄测试网页"的编号（W3C Gamepad 标准）和 /joy 完全不同，别照那个填。

实测这台手柄的绿灯/红灯**轴和按键编号完全一样**，区别只在摇杆值：
绿灯 = 连续模拟量，红灯 = 只有几档（开车会顿）。所以开车前把指示灯切成绿色：
插好手柄后长按 MODE 键 5 秒即可；手柄休眠后唤醒可能会回到红灯，先看一眼灯色。

================ 控制逻辑 ================
【运动模式】（select 切到运动）
  左摇杆上下        → 前进 / 后退
  右摇杆左右        → 左右平移
  L2 扳机按住       → 向左转
  R2 扳机按住       → 向右转

【云台模式】（select 切到云台）
  左摇杆左右        → yaw 轴 逆 / 顺时针
  右摇杆上下        → X 轴 前后伸
  Y 按住 / A 按住   → Z 轴 升 / 降
  X 按一下          → 夹爪 夹取 / 松开（toggle）
  L1 按住           → 载物盘 顺时针
  L2 扳机按住       → 载物盘 逆时针

【两种模式通用】
  start 按一下      → 使能 / 失能（切换）
  B 按一下          → 立即失能（安全键）
  R1 按一下         → 底盘归位（回坐标原点，保持当前航向）
  左 / 右方向键     → 补光灯亮度 + / -
  上 / 下方向键     → 摄像头舵机 ↑ / ↓

================ 上位机控制权（和 motion_planner 分离） ================
motion_planner 跑序列时会以 10Hz 在 /motion_active 上置 True，表示"上位机在开车"。
这期间本节点**除了 B（立即失能停机）以外，一个指令都不发**：不发光速、不发云台、
不发舵机、不发补光灯、按键也不响应。这样两套控制逻辑完全隔离，不会互相抢。
  · 上位机点"■ 停止"或序列跑完 → /motion_active 置 False，手柄立刻恢复。
  · 按 B → 失能 + 停序列 + 交还控制权，之后按 start 可重新使能手动开。
  · 上位机崩溃/断线时 /motion_active 心跳会停，本节点 0.6s 后自动恢复手柄控制，
    不会被永久锁死。
  · 摇杆回中时本节点不发任何 VEL（只在松手瞬间补一帧零速），
    否则周期零速帧会清掉 F407 的位置目标（PiComm.c 收到 VEL 就清 PosTarget.Valid）。

方向反了不用改代码，用参数翻（例：左摇杆推上其实是后退就改成 1.0）：
  ros2 run joy_control joy_control_node --ros-args -p dir_linear:=1.0

坐标系约定：车正前方 +x（前进），左侧 +y（左平移），逆时针 +wz（左转）。
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray, Int32
from nav_msgs.msg import Odometry
from car_interfaces.msg import Pose2D


class JoyControlNode(Node):
    def __init__(self):
        super().__init__('joy_control')

        # ================= 轴编号（内核实测，ROS /joy 下标） =================
        self.declare_parameter('axis_linear', 1)      # 左摇杆上下 → 前进/后退
        self.declare_parameter('axis_strafe', 3)      # 右摇杆左右 → 平移
        self.declare_parameter('axis_yaw', 0)         # 左摇杆左右 → 云台 yaw（云台模式）
        self.declare_parameter('axis_extend', 4)      # 右摇杆上下 → 云台 extend（云台模式）
        self.declare_parameter('axis_lt', 2)          # L2 扳机（轴！）
        self.declare_parameter('axis_rt', 5)          # R2 扳机（轴！）
        self.declare_parameter('axis_dpad_x', 6)      # 方向键左右
        self.declare_parameter('axis_dpad_y', 7)      # 方向键上下
        self.declare_parameter('trigger_threshold', 0.5)  # 扳机按到多少算"按下"
        self.declare_parameter('trigger_invert', False)   # 内核实测：松开=-1、按到底=+1
        self.declare_parameter('debug_input', False)      # true=打印每个按键的真实序号
        # auto=按轴数量自动判断；ps2=6 轴模式；xbox=8 轴模式
        self.declare_parameter('layout', 'auto')

        # ================= 按键编号 =================
        # 这里的默认值是 Xbox 8 轴模式；PS2 6 轴模式的编号完全不同，
        # 由 _apply_layout('ps2') 在收到第一帧 /joy 时整套覆盖。
        self.declare_parameter('btn_select', 6)       # 切换运动/云台模式
        self.declare_parameter('btn_start', 7)        # 使能/失能切换
        self.declare_parameter('btn_b', 1)            # 立即失能（安全键）
        self.declare_parameter('btn_x', 2)            # 夹爪夹取/松开（toggle，云台模式）
        self.declare_parameter('btn_y', 3)            # 云台 Z 轴上升
        self.declare_parameter('btn_a', 0)            # 云台 Z 轴下降
        self.declare_parameter('btn_l1', 4)           # 载物盘顺时针（云台模式）
        self.declare_parameter('btn_r1', 5)           # 底盘归位（回坐标原点）

        # ================= 方向系数（反了改 ±1，不用动别的） =================
        self.declare_parameter('dir_linear', 1.0)     # 左摇杆推上(-1) = 前进（实测 +Vx 是后退，故取 +1）
        self.declare_parameter('dir_strafe', 1.0)     # 右摇杆推右(+1) = 右移（实测 +Vy 与直觉相反，取 +1 翻转）
        self.declare_parameter('dir_turn', 1.0)       # L2 = 左转(+wz)，R2 = 右转(-wz)
        self.declare_parameter('dir_yaw', 1.0)        # 左摇杆推右 = yaw 增
        self.declare_parameter('dir_extend', -1.0)    # 右摇杆推上(-1) = extend 增
        self.declare_parameter('plate_dir', 1.0)      # 载物盘：L1 顺时针方向
        self.declare_parameter('invert_dpad_x', False)  # 方向键左右如果反了就 True
        self.declare_parameter('invert_dpad_y', False)  # 方向键上下如果反了就 True

        # ================= 运动/步进参数 =================
        self.declare_parameter('max_vx', 1.0)             # 前进满量程 m/s
        self.declare_parameter('max_vy', 1.0)             # 平移满量程 m/s
        self.declare_parameter('turn_wz', 0.3)            # L2/R2 转向角速度 rad/s（≈17°/s，太快容易转过头）
        self.declare_parameter('deadzone', 0.0)           # 摇杆死区（摇杆本身自带死区，这里不再加）
        self.declare_parameter('led_step', 20)            # 补光灯每按一下增减 %
        self.declare_parameter('gimbal_step_deg', 2.0)    # 云台每 tick 步进 °
        self.declare_parameter('servo_step_deg', 5.0)     # 载物盘每 tick 步进 °
        self.declare_parameter('cam_step_deg', 5.0)       # 摄像头每按一下步进 °
        self.declare_parameter('gripper_close_deg', 40.0)  # 夹爪"夹紧"角度
        self.declare_parameter('gripper_open_deg', 90.0)   # 夹爪"松开"角度

        # 读出参数
        self.axis_linear = self.get_parameter('axis_linear').value
        self.axis_strafe = self.get_parameter('axis_strafe').value
        self.axis_yaw = self.get_parameter('axis_yaw').value
        self.axis_extend = self.get_parameter('axis_extend').value
        self.axis_lt = self.get_parameter('axis_lt').value
        self.axis_rt = self.get_parameter('axis_rt').value
        self.axis_dpad_x = self.get_parameter('axis_dpad_x').value
        self.axis_dpad_y = self.get_parameter('axis_dpad_y').value
        self.trigger_threshold = float(self.get_parameter('trigger_threshold').value)
        self.trigger_invert = bool(self.get_parameter('trigger_invert').value)
        self.debug_input = bool(self.get_parameter('debug_input').value)
        self.layout = str(self.get_parameter('layout').value).lower()
        # 由 _apply_layout() 填充：True 表示 L2/R2 是按键（PS2 模式）
        self._trigger_is_button = False
        self.btn_l2 = -1
        self.btn_r2 = -1
        self._applied_layout = None
        self.btn_select = self.get_parameter('btn_select').value
        self.btn_start = self.get_parameter('btn_start').value
        self.btn_b = self.get_parameter('btn_b').value
        self.btn_x = self.get_parameter('btn_x').value
        self.btn_y = self.get_parameter('btn_y').value
        self.btn_a = self.get_parameter('btn_a').value
        self.btn_l1 = self.get_parameter('btn_l1').value
        self.btn_r1 = self.get_parameter('btn_r1').value
        self.dir_linear = float(self.get_parameter('dir_linear').value)
        self.dir_strafe = float(self.get_parameter('dir_strafe').value)
        self.dir_turn = float(self.get_parameter('dir_turn').value)
        self.dir_yaw = float(self.get_parameter('dir_yaw').value)
        self.dir_extend = float(self.get_parameter('dir_extend').value)
        self.plate_dir = float(self.get_parameter('plate_dir').value)
        self.invert_dpad_x = bool(self.get_parameter('invert_dpad_x').value)
        self.invert_dpad_y = bool(self.get_parameter('invert_dpad_y').value)
        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.turn_wz = float(self.get_parameter('turn_wz').value)
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.led_step = int(self.get_parameter('led_step').value)
        self.gimbal_step_deg = float(self.get_parameter('gimbal_step_deg').value)
        self.servo_step_deg = float(self.get_parameter('servo_step_deg').value)
        self.cam_step_deg = float(self.get_parameter('cam_step_deg').value)
        self.gripper_close_deg = float(self.get_parameter('gripper_close_deg').value)
        self.gripper_open_deg = float(self.get_parameter('gripper_open_deg').value)

        # ================= 发布的话题 =================
        self._pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self._pub_enable = self.create_publisher(Bool, '/cmd_enable', 10)
        self._pub_led = self.create_publisher(Int32, '/cmd_led', 10)
        self._pub_gimbal = self.create_publisher(Float32MultiArray, '/cmd_gimbal', 10)
        self._pub_servo = self.create_publisher(Float32MultiArray, '/cmd_servo', 10)
        self._pub_pose = self.create_publisher(Pose2D, '/cmd_pose', 10)
        self._pub_motion_stop = self.create_publisher(Bool, '/motion_stop', 10)

        # ================= 订阅 =================
        self.create_subscription(Joy, '/joy', self._cb_joy, 10)
        self.create_subscription(Odometry, '/odom', self._cb_odom, 10)
        # 使能状态以 /cmd_enable 为准（上位机/状态机也会发），避免两边各记一份、互相打架
        self.create_subscription(Bool, '/cmd_enable', self._cb_enable, 10)
        # 上位机是否正在跑序列：True 期间手柄只留 B（失能）可用
        self.create_subscription(Bool, '/motion_active', self._cb_motion_active, 10)

        # ================= 内部状态 =================
        self._axes = [0.0] * 8                 # 最新轴值
        self._buttons = []                     # 最新按键状态
        self._prev_buttons = []                # 上一拍按键（做上升沿）
        self._prev_dpad = {'left': False, 'right': False, 'up': False, 'down': False}
        self._prev_aux = {'L1': False, 'R1': False, 'L2': False, 'R2': False}
        self._mode = 'drive'                   # drive=运动 / gimbal=云台
        self._enabled = False                  # 当前使能状态（start 切换）
        self._led = 0                          # 补光灯亮度 %
        self._gimbal = [0.0, 0.0, 0.0]         # 云台目标 [yaw, lift, extend] °
        self._servo = [90.0, 0.0, 90.0]        # 舵机目标 [夹爪, 载物盘, 摄像头] °
        self._gripper_closed = False           # 夹爪是否处于"夹紧"档
        self._yaw = 0.0                        # 当前航向 rad（来自 /odom，归位时保持航向用）
        self._homing = False                   # 是否正在底盘归位（归位期间持续发 POSE，不发 VEL）
        self._home_yaw = 0.0                   # 归位时锁存的航向，防止归位过程中被 /odom 带着转
        self._vel_active = False               # 上一拍是否真的在发非零速度（用来"松手补一帧零速"）
        self._planner_active = False           # 上位机状态机是否正在跑序列（True 时手柄除 B 外全部让位）
        self._planner_active_t = 0.0           # 最近一次收到 /motion_active=True 的时刻（做超时自恢复）

        # 定时器：10Hz 推一次（有输入才发速度，见 _tick_drive 的说明）
        self._timer = self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f'joy_control 就绪：layout={self.layout}（收到 /joy 后自动识别 PS2/Xbox 编号）'
        )

    # ---------------- 布局切换 ----------------

    def _apply_layout(self, name: str):
        """按手柄模式成套切换轴/按键编号（两套编号不能混用）。"""
        if name == self._applied_layout:
            return
        self._applied_layout = name
        if name == 'ps2':
            self.axis_linear, self.axis_strafe = 1, 2
            self.axis_yaw, self.axis_extend = 0, 3
            self.axis_dpad_x, self.axis_dpad_y = 4, 5
            self.axis_lt = self.axis_rt = -1
            # 实测（joy_test.py 逐项按键）：面键顺序是 Y/B/A/X，不是 A/B/X/Y
            self.btn_a, self.btn_b, self.btn_x, self.btn_y = 2, 1, 3, 0
            self.btn_l1, self.btn_r1 = 4, 5
            self.btn_l2, self.btn_r2 = 6, 7
            self.btn_select, self.btn_start = 8, 9
            self._trigger_is_button = True
        else:
            self.axis_linear, self.axis_strafe = 1, 3
            self.axis_yaw, self.axis_extend = 0, 4
            self.axis_dpad_x, self.axis_dpad_y = 6, 7
            self.axis_lt, self.axis_rt = 2, 5
            self.btn_a, self.btn_b, self.btn_x, self.btn_y = 0, 1, 2, 3
            self.btn_l1, self.btn_r1 = 4, 5
            self.btn_l2 = self.btn_r2 = -1
            self.btn_select, self.btn_start = 6, 7
            self._trigger_is_button = False
        self.get_logger().info(
            f'手柄布局 -> {name.upper()}：'
            f'select={self.btn_select} start={self.btn_start} B={self.btn_b}；'
            + ('L2/R2=按键 6/7' if self._trigger_is_button else 'L2/R2=轴 2/5')
        )

    # ---------------- 摇杆/按键回调 ----------------

    def _cb_odom(self, msg: Odometry):
        """收到 /odom：只记航向（从朝向四元数取绕 Z 轴的 yaw），归位时用来保持朝向。"""
        q = msg.pose.pose.orientation
        self._yaw = 2.0 * math.atan2(q.z, q.w)

    def _cb_enable(self, msg: Bool):
        """跟随 /cmd_enable 同步使能状态（自己发的、上位机/状态机发的都走这里）。

        为什么必须同步：状态机序列里可能有 enable 指令，如果本节点不跟随，
        就会出现"车已使能但手柄以为没使能"（推摇杆不动）或反过来（一按 start
        反而把车停了）的错位。
        """
        en = bool(msg.data)
        if en == self._enabled:
            return
        self._enabled = en
        if not en:
            self._homing = False
            self._release_chassis()    # 被外部失能时补一帧零速，防止残留速度
        self.get_logger().info(f'使能状态同步 -> {"开" if en else "关"}')

    def _cb_motion_active(self, msg: Bool):
        """上位机状态机开始/结束跑序列。

        约定：上位机工作时，手柄除了 B（失能）以外一律不响应，
        两套控制逻辑彻底分开，不会出现"谁抢谁"的问题。
        上位机发的是 10Hz 心跳，所以这里同时记时间，用来做掉线恢复。
        """
        active = bool(msg.data)
        self._planner_active_t = time.monotonic()
        if active == self._planner_active:
            return
        self._planner_active = active
        if active:
            self._homing = False          # 归位让位给上位机
            self._release_chassis()       # 先补一帧零速，别让残余速度带进上位机控制
            self.get_logger().info('上位机接管：手柄只保留 B（失能）')
        else:
            self.get_logger().info('上位机结束，手柄恢复控制')

    def _cb_joy(self, msg: Joy):
        """收到 /joy：记录轴值、做按键和方向键的上升沿检测（只触发一次）。"""
        if self.layout == 'auto':
            # 6 轴=PS2 模式，8 轴=Xbox 模式；热切换模式也能跟着变
            self._apply_layout('ps2' if len(msg.axes) <= 6 else 'xbox')
        elif self.layout in ('ps2', 'xbox'):
            self._apply_layout(self.layout)
        self._axes = list(msg.axes) + [0.0] * max(0, 8 - len(msg.axes))
        self._buttons = list(msg.buttons)

        # 按键上升沿：0→1 触发一次
        n = max(len(self._buttons), len(self._prev_buttons))
        cur = self._buttons + [0] * max(0, n - len(self._buttons))
        prev = self._prev_buttons + [0] * max(0, n - len(self._prev_buttons))
        for i in range(n):
            if cur[i] and not prev[i]:
                if self.debug_input:
                    self.get_logger().info(f'按键 {i} 按下（debug_input）')
                self._on_button_pressed(i)
        self._prev_buttons = list(self._buttons)

        # 方向键是"轴"，也做上升沿（按一下一档）
        dx = self._axis_raw(self.axis_dpad_x) * (-1.0 if self.invert_dpad_x else 1.0)
        dy = self._axis_raw(self.axis_dpad_y) * (-1.0 if self.invert_dpad_y else 1.0)
        left, right = dx < -0.5, dx > 0.5
        up, down = dy < -0.5, dy > 0.5
        if not self._planner_active:          # 上位机控制中，方向键也不响应
            if left and not self._prev_dpad['left']:
                self._led_up()                # 左方向键：补光灯 +
            if right and not self._prev_dpad['right']:
                self._led_down()              # 右方向键：补光灯 -
            if up and not self._prev_dpad['up']:
                self._cam_up()                # 上方向键：摄像头 ↑
            if down and not self._prev_dpad['down']:
                self._cam_down()              # 下方向键：摄像头 ↓
        self._prev_dpad.update(left=left, right=right, up=up, down=down)

        self._log_aux_buttons()

    def _log_aux_buttons(self):
        """L1/R1/L2/R2 的按下/松开各打一条日志，方便确认键有没有被认到。

        这几个键是"按住持续动作"，不适合在 10Hz 定时器里每拍打日志；
        这里只在状态变化时打。L2/R2 在 6 轴模式是按键、8 轴模式是扳机轴，
        用 _l2()/_r2() 统一判断，两种模式都有日志。
        """
        now = {
            'L1': self._is_pressed(self.btn_l1),
            'R1': self._is_pressed(self.btn_r1),
            'L2': self._l2(),
            'R2': self._r2(),
        }
        for name, pressed in now.items():
            if pressed == self._prev_aux.get(name, False):
                continue
            if not pressed:
                self.get_logger().info(f'{name} 松开')
                continue
            if name == 'L1':
                action = '载物盘顺时针' if self._mode == 'gimbal' else '运动模式未分配'
            elif name == 'L2':
                if self._mode == 'gimbal':
                    action = '载物盘逆时针'
                else:
                    action = '向左转' + ('' if self._enabled else '（未使能，不会转）')
            elif name == 'R2':
                if self._mode == 'gimbal':
                    action = '云台模式未分配'
                else:
                    action = '向右转' + ('' if self._enabled else '（未使能，不会转）')
            else:
                action = '未分配'
            self.get_logger().info(f'{name} 按下 -> {action}')
        self._prev_aux = now

    def _on_button_pressed(self, i: int):
        """某个按键被按下（上升沿），按编号分发。"""
        # 上位机跑序列期间：手柄只保留 B（失能），其余按键一律不响应
        if self._planner_active and i != self.btn_b:
            self.get_logger().info(
                '上位机控制中，手柄按键不响应（按 B 可失能停机）',
                throttle_duration_sec=3.0)
            return
        if i == self.btn_select:
            self._mode = 'gimbal' if self._mode == 'drive' else 'drive'
            if self._mode == 'gimbal':
                self._release_chassis()                 # 切云台前先让底盘停
            self.get_logger().info(f'模式 -> {"云台" if self._mode == "gimbal" else "运动"}')
        elif i == self.btn_start:
            self._enabled = not self._enabled
            self._pub_enable.publish(Bool(data=self._enabled))
            if not self._enabled:
                self._release_chassis()                 # 失能同时停车
            self.get_logger().info('使能' if self._enabled else '失能')
        elif i == self.btn_b:
            self._enabled = False
            self._homing = False
            self._pub_enable.publish(Bool(data=False))
            self._release_chassis()
            # 上位机正在跑序列时，光失能不够：状态机会一直等"到位"，所以连序列一起停
            self._pub_motion_stop.publish(Bool(data=True))
            self.get_logger().info('失能（B 安全键）' + ('，同时停止上位机序列' if self._planner_active else ''))
        elif i == self.btn_x and self._mode == 'gimbal':
            self._gripper_closed = not self._gripper_closed
            self._servo[0] = self.gripper_close_deg if self._gripper_closed else self.gripper_open_deg
            self._send_servo()
            self.get_logger().info(
                f'夹爪 -> {"夹紧" if self._gripper_closed else "松开"} ({self._servo[0]:.0f}°)'
            )
        elif i == self.btn_r1:
            # 底盘归位：走 F407 位置闭环回到坐标原点 (0,0)，保持当前航向不转。
            if self._mode != 'drive':
                self.get_logger().info('R1 归位只在运动模式生效')
            elif not self._enabled:
                self.get_logger().warning('未使能，R1 归位无效，先按 start 使能')
            else:
                self._homing = not self._homing
                if self._homing:
                    self._release_chassis()             # 先停稳，再交给位置闭环
                    # 归位也是发 /cmd_pose，和 motion_planner 会抢；先让状态机停下
                    self._pub_motion_stop.publish(Bool(data=True))
                    self._home_yaw = self._yaw
                    self.get_logger().info(
                        f'归位 -> 原点(0,0)，保持航向 {math.degrees(self._home_yaw):.1f}°'
                    )
                else:
                    self.get_logger().info('取消归位')

    # ---------------- 定时器：运动/云台 ----------------

    def _tick(self):
        # 上位机心跳超时自恢复：状态机崩溃 / 半路退出时不能把手柄永久锁死
        if self._planner_active and time.monotonic() - self._planner_active_t > 0.6:
            self._planner_active = False
            self.get_logger().warning('上位机心跳中断（/motion_active 0.6s 没更新），手柄恢复控制')
        if self._planner_active:
            # 上位机控制中：手柄一个动作都不发（B 失能在按键回调里单独处理）
            self._release_chassis()
            return
        if self._mode == 'gimbal':
            self._tick_gimbal()
        else:
            self._tick_drive()

    def _release_chassis(self):
        """松手/失能时补一帧零速，然后静默。

        为什么不能一直发零速：F407 收到任何 VEL 帧都会清掉 PosTarget.Valid
        （PiComm.c），motion_planner 发的 /cmd_pose 目标 100ms 就没了。
        为什么又不能干脆不发：F407 没有速度超时，停发 VEL 会保持最后一帧速度，
        车会一直跑。所以"有输入才发、松手补一帧零速"是唯一安全的做法。
        """
        if self._vel_active:
            self._pub_vel.publish(Twist())
            self._vel_active = False

    def _tick_drive(self):
        """运动模式：左摇杆上下→vx，右摇杆左右→vy，L2/R2→左右转。

        只有真的有输入时才发 /cmd_vel；摇杆回中就补一帧零速然后停发。
        这样待机时不会用零速帧把上位机/motion_planner 的位置目标冲掉。
        """
        if not self._enabled:                     # 未使能时绝不出速度，且停发
            self._homing = False
            self._release_chassis()
            return
        # 归位期间：持续发 POSE（不发 VEL，否则 VEL 会清掉 F407 的位置目标）。
        # 摇杆或 L2/R2 有输入就退出归位，恢复手动。
        if self._homing:
            moving = (abs(self._axis(self.axis_linear)) > 0.01
                      or abs(self._axis(self.axis_strafe)) > 0.01
                      or self._l2() or self._r2())
            if moving:
                self._homing = False
                self.get_logger().info('有摇杆输入，退出归位')
            else:
                self._pub_pose.publish(Pose2D(x=0.0, y=0.0, theta=self._home_yaw))
                return
        vx = self.dir_linear * self._axis(self.axis_linear) * self.max_vx
        vy = self.dir_strafe * self._axis(self.axis_strafe) * self.max_vy
        wz = 0.0
        if self._l2():
            wz += self.dir_turn * self.turn_wz  # L2 向左转（逆时针为正）
        if self._r2():
            wz -= self.dir_turn * self.turn_wz  # R2 向右转
        if abs(vx) < 1e-6 and abs(vy) < 1e-6 and abs(wz) < 1e-6:
            self._release_chassis()               # 摇杆回中：补一帧零速后停发
            return
        self._vel_active = True

        m = Twist()
        m.linear.x = vx
        m.linear.y = vy
        m.angular.z = wz
        self._pub_vel.publish(m)

    def _tick_gimbal(self):
        """云台模式：摇杆控制 yaw/extend，Y/A 控制 lift，X 夹爪，L1/L2 载物盘。"""
        # 左摇杆左右 → yaw；右摇杆上下 → extend（X 轴伸缩）
        self._gimbal[0] += self.dir_yaw * self._axis(self.axis_yaw) * self.gimbal_step_deg
        self._gimbal[2] += self.dir_extend * self._axis(self.axis_extend) * self.gimbal_step_deg
        # Y/A 按住 → lift 升/降
        if self._is_pressed(self.btn_y):
            self._gimbal[1] += self.gimbal_step_deg
        if self._is_pressed(self.btn_a):
            self._gimbal[1] -= self.gimbal_step_deg
        # 软限位（和 RobotConfig.h 一致，防撞结构）
        self._gimbal[0] = max(-180.0, min(180.0, self._gimbal[0]))
        self._gimbal[1] = max(0.0, min(200.0, self._gimbal[1]))
        self._gimbal[2] = max(0.0, min(200.0, self._gimbal[2]))
        g = Float32MultiArray()
        g.data = [float(x) for x in self._gimbal]
        self._pub_gimbal.publish(g)

        # 载物盘：L1 顺时针 / L2（扳机）逆时针（步进，变了才发）
        # 注：+/- 代表舵机角度增减，具体"顺/逆"取决于机械安装，反了就改 plate_dir:=-1.0
        old_plate = self._servo[1]
        if self._is_pressed(self.btn_l1):
            self._servo[1] += self.plate_dir * self.servo_step_deg
        if self._l2():
            self._servo[1] -= self.plate_dir * self.servo_step_deg
        self._servo[1] = max(0.0, min(180.0, self._servo[1]))
        if self._servo[1] != old_plate:
            self._send_servo()

        # 云台模式下底盘保持停止：切模式时已经补过零速，这里不要再周期发
        # （周期发零速会清掉 motion_planner 的位置目标）
        self._release_chassis()

    # ---------------- 方向键动作（补光灯 / 摄像头） ----------------

    def _led_up(self):
        self._led = min(100, self._led + self.led_step)
        self._pub_led.publish(Int32(data=self._led))
        self.get_logger().info(f'补光灯 -> {self._led}%')

    def _led_down(self):
        self._led = max(0, self._led - self.led_step)
        self._pub_led.publish(Int32(data=self._led))
        self.get_logger().info(f'补光灯 -> {self._led}%')

    def _cam_up(self):
        self._servo[2] = min(180.0, self._servo[2] + self.cam_step_deg)
        self._send_servo()
        self.get_logger().info(f'摄像头 -> {self._servo[2]:.0f}°')

    def _cam_down(self):
        self._servo[2] = max(0.0, self._servo[2] - self.cam_step_deg)
        self._send_servo()
        self.get_logger().info(f'摄像头 -> {self._servo[2]:.0f}°')

    # ---------------- 工具 ----------------

    def _axis_raw(self, idx: int) -> float:
        """读某个轴的原始值，裁剪到 [-1,1]（不做死区）。"""
        if 0 <= idx < len(self._axes):
            return max(-1.0, min(1.0, float(self._axes[idx])))
        return 0.0

    def _axis(self, idx: int) -> float:
        """读某个轴的当前值，做死区处理（|值|<deadzone 视为 0）。"""
        v = self._axis_raw(idx)
        return 0.0 if abs(v) < self.deadzone else v

    def _trigger(self, idx: int) -> bool:
        """扳机（轴）是否按下。

        内核里 ABS_Z/ABS_RZ 的范围是 0..255，joydev 把它线性映射到 -1..+1，
        所以静止 = -1，按到底 = +1。默认不做反转；若你的驱动恰好相反，
        用参数 trigger_invert:=True 翻一下。
        """
        v = self._axis_raw(idx)
        if self.trigger_invert:
            v = -v
        return v > self.trigger_threshold

    def _l2(self) -> bool:
        """L2 是否按下：PS2 模式是按键，Xbox 模式是扳机轴。"""
        return self._is_pressed(self.btn_l2) if self._trigger_is_button \
            else self._trigger(self.axis_lt)

    def _r2(self) -> bool:
        """R2 是否按下：PS2 模式是按键，Xbox 模式是扳机轴。"""
        return self._is_pressed(self.btn_r2) if self._trigger_is_button \
            else self._trigger(self.axis_rt)

    def _is_pressed(self, idx: int) -> bool:
        """某个按键当前是否被按住。"""
        return 0 <= idx < len(self._buttons) and bool(self._buttons[idx])

    def _send_servo(self):
        """把三个舵机的当前目标角度整组发出去 [夹爪, 载物盘, 摄像头]。"""
        a = Float32MultiArray()
        a.data = [float(x) for x in self._servo]
        self._pub_servo.publish(a)


def main(args=None):
    rclpy.init(args=args)
    node = JoyControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
