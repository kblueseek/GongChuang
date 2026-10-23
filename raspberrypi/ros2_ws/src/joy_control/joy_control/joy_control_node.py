#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""手柄控制节点：把 USB 手柄（WHEELTEC PS2）的摇杆/按键翻译成整车控制话题。

================ 大白话：这个节点在干什么 ================

手柄插到树莓派后，由 ROS2 的 joy 包（joy_node）读取，发到 /joy 话题
（sensor_msgs/Joy：一堆轴 axes[] 和按键 buttons[]）。本节点订阅 /joy，
按下面这张"按键表"翻译成整车指令。

================ 控制逻辑（select 切换两个模式） ================

【任何模式】
  start 按一下            → 使能
  B 键 按一下             → 失能/急停
  左方向键 / 右方向键      → 补光灯亮度 + / -（按一下一档）
  上方向键 / 下方向键      → 摄像头舵机 ↑ / ↓（按一下一步）

【运动模式】（select 切到运动）
  左摇杆上下              → 前进/后退 vx
  右摇杆左右              → 左右平移 vy
  L2 按住                 → 向左转 wz
  R2 按住                 → 向右转 wz

【云台模式】（select 切到云台）
  左摇杆左右              → 云台 yaw 轴（逆/顺时针转）
  右摇杆上下              → 云台 X 轴（extend）前后伸
  Y 按住                  → 云台 lift（Z 升降）轴上升
  A 按住                  → 云台 lift（Z 升降）轴下降
  X 按一下                → 夹爪夹取/松开（toggle）
  L1 按住                 → 载物盘舵机逆时针
  R1 按住                 → 载物盘舵机顺时针

================ 按键/轴编号（实测，联调再核对） ================

这些默认值来自你实测的编号（select=8 已确认，start=9 按 PS2 标准推断）。
联调发现不对，用 ros2 参数覆盖即可，不用改代码：
  ros2 run joy_control joy_control_node --ros-args -p btn_start:=9 -p dir_linear:=-1

坐标系约定：车正前方 +x（前进），左侧 +y（左平移），逆时针 +wz（左转）。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray, Int32


class JoyControlNode(Node):
    def __init__(self):
        super().__init__('joy_control')

        # ================= 轴编号（实测） =================
        self.declare_parameter('axis_linear', 1)     # 左摇杆上下 → 前进/后退
        self.declare_parameter('axis_strafe', 2)     # 右摇杆左右 → 平移
        self.declare_parameter('axis_yaw', 0)        # 左摇杆左右 → 云台 yaw（云台模式）
        self.declare_parameter('axis_extend', 3)     # 右摇杆上下 → 云台 extend（云台模式）
        self.declare_parameter('axis_dpad_lr', 4)    # 方向键左右（+1左 -1右）
        self.declare_parameter('axis_dpad_ud', 5)    # 方向键上下（+1上 -1下）

        # ================= 按键编号（实测） =================
        self.declare_parameter('btn_select', 8)      # 切换运动/云台模式
        self.declare_parameter('btn_start', 9)       # 使能（按一下）
        self.declare_parameter('btn_y', 3)           # 云台 lift 上升
        self.declare_parameter('btn_a', 0)           # 云台 lift 下降
        self.declare_parameter('btn_x', 2)           # 夹爪夹取/松开（toggle）
        self.declare_parameter('btn_l1', 4)          # 云台=载物盘逆时针
        self.declare_parameter('btn_r1', 5)          # 云台=载物盘顺时针
        self.declare_parameter('btn_l2', 6)          # 运动=左转
        self.declare_parameter('btn_r2', 7)          # 运动=右转
        self.declare_parameter('btn_estop', 1)       # 失能/急停（B 键，按一下触发一次）

        # ================= 方向系数（联调发现方向反了改成 -1，不用动别的） =================
        self.declare_parameter('dir_linear', 1.0)    # 左摇杆推上 = 前进（若反了改 -1）
        self.declare_parameter('dir_strafe', 1.0)    # 右摇杆推右 = 右移
        self.declare_parameter('dir_yaw', 1.0)       # 左摇杆推右 = 云台 yaw 增
        self.declare_parameter('dir_extend', 1.0)    # 右摇杆推上 = extend 增

        # ================= 运动/步进参数 =================
        self.declare_parameter('max_vx', 1.0)            # 前进满量程 m/s
        self.declare_parameter('max_vy', 1.0)            # 平移满量程 m/s
        self.declare_parameter('turn_wz', 1.0)           # L2/R2 转向角速度 rad/s
        self.declare_parameter('deadzone', 0.05)         # 摇杆死区
        self.declare_parameter('led_step', 20)           # 补光灯每按一下增减 %
        self.declare_parameter('gimbal_step_deg', 2.0)   # 云台每 tick 步进 °
        self.declare_parameter('servo_step_deg', 5.0)    # 载物盘每 tick 步进 °
        self.declare_parameter('cam_step_deg', 5.0)      # 摄像头每按一下步进 °
        self.declare_parameter('gripper_close_deg', 40.0)  # 夹爪"夹紧"角度
        self.declare_parameter('gripper_open_deg', 90.0)   # 夹爪"松开"角度

        # 读出参数
        self.axis_linear = self.get_parameter('axis_linear').value
        self.axis_strafe = self.get_parameter('axis_strafe').value
        self.axis_yaw = self.get_parameter('axis_yaw').value
        self.axis_extend = self.get_parameter('axis_extend').value
        self.axis_dpad_lr = self.get_parameter('axis_dpad_lr').value
        self.axis_dpad_ud = self.get_parameter('axis_dpad_ud').value
        self.btn_select = self.get_parameter('btn_select').value
        self.btn_start = self.get_parameter('btn_start').value
        self.btn_y = self.get_parameter('btn_y').value
        self.btn_a = self.get_parameter('btn_a').value
        self.btn_x = self.get_parameter('btn_x').value
        self.btn_l1 = self.get_parameter('btn_l1').value
        self.btn_r1 = self.get_parameter('btn_r1').value
        self.btn_l2 = self.get_parameter('btn_l2').value
        self.btn_r2 = self.get_parameter('btn_r2').value
        self.btn_estop = self.get_parameter('btn_estop').value
        self.dir_linear = float(self.get_parameter('dir_linear').value)
        self.dir_strafe = float(self.get_parameter('dir_strafe').value)
        self.dir_yaw = float(self.get_parameter('dir_yaw').value)
        self.dir_extend = float(self.get_parameter('dir_extend').value)
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

        # ================= 订阅 =================
        self.create_subscription(Joy, '/joy', self._cb_joy, 10)

        # ================= 内部状态 =================
        self._axes = [0.0] * 8                 # 最新摇杆/方向键轴值
        self._buttons = []                     # 最新按键状态
        self._prev_buttons = []                # 上一拍按键（做边缘检测）
        self._prev_dpad_lr = 0                 # 上一拍方向键左右（-1/0/+1）
        self._prev_dpad_ud = 0                 # 上一拍方向键上下
        self._mode = 'drive'                   # 当前模式：drive=运动 / gimbal=云台
        self._led = 0                          # 补光灯亮度 %
        self._gimbal = [0.0, 0.0, 0.0]         # 云台目标 [yaw, lift, extend] °
        self._servo = [90.0, 0.0, 90.0]        # 舵机目标 [夹爪, 载物盘, 摄像头] °（和 F407 上电默认一致）
        self._gripper_closed = False           # 夹爪是否处于"夹紧"档

        # 定时器：10Hz 持续发运动/步进指令（摇杆回中发 0，避免车保持最后速度继续跑）
        self._timer = self.create_timer(0.1, self._tick)
        self.get_logger().info('joy_control 就绪（select 切模式 / start 使能 / B 失能），等待 /joy')

    # ---------------- 摇杆/按键回调 ----------------

    def _cb_joy(self, msg: Joy):
        """收到 /joy：记录轴值、做按键和方向键的边缘检测（只触发一次）。"""
        # 记录轴值（不够长补 0）
        self._axes = list(msg.axes) + [0.0] * max(0, 8 - len(msg.axes))
        self._buttons = list(msg.buttons)

        # 按键边缘检测：从 0 变 1 的上升沿触发一次
        prev = self._prev_buttons + [0] * max(0, len(self._buttons) - len(self._prev_buttons))
        for i, pressed in enumerate(self._buttons):
            was_pressed = prev[i] if i < len(prev) else 0
            if pressed and not was_pressed:
                self._on_button_pressed(i)
        self._prev_buttons = list(self._buttons)

        # 方向键是"轴"不是"按钮"，这里也做上升沿检测（按一下触发一次）
        lr = self._dpad(self.axis_dpad_lr)   # +1 左 / -1 右
        ud = self._dpad(self.axis_dpad_ud)   # +1 上 / -1 下
        if lr > 0 and self._prev_dpad_lr <= 0:
            self._led_up()                    # 左方向键：补光灯 +
        elif lr < 0 and self._prev_dpad_lr >= 0:
            self._led_down()                  # 右方向键：补光灯 -
        if ud > 0 and self._prev_dpad_ud <= 0:
            self._cam_up()                    # 上方向键：摄像头 ↑
        elif ud < 0 and self._prev_dpad_ud >= 0:
            self._cam_down()                  # 下方向键：摄像头 ↓
        self._prev_dpad_lr = lr
        self._prev_dpad_ud = ud

    def _on_button_pressed(self, i: int):
        """某个按键被按下（边缘触发），按编号分发。"""
        if i == self.btn_select:
            self._mode = 'gimbal' if self._mode == 'drive' else 'drive'
            self.get_logger().info(f'模式 -> {"云台" if self._mode == "gimbal" else "运动"}')
        elif i == self.btn_start:
            self._pub_enable.publish(Bool(data=True))   # start = 使能（点动按键，按一下触发一次）
            self.get_logger().info('使能')
        elif i == self.btn_estop:
            self._pub_enable.publish(Bool(data=False))  # B 键 = 失能/急停（点动按键，按一下触发一次）
            self.get_logger().info('失能')
        elif i == self.btn_x:
            self._gripper_closed = not self._gripper_closed
            self._servo[0] = self.gripper_close_deg if self._gripper_closed else self.gripper_open_deg
            self._send_servo()
            self.get_logger().info(f'夹爪 -> {"夹紧" if self._gripper_closed else "松开"} ({self._servo[0]}°)')

    # ---------------- 定时器：运动/云台 ----------------

    def _tick(self):
        """每 0.1s 执行一次。"""
        # 按当前模式分发
        if self._mode == 'gimbal':
            self._tick_gimbal()
        else:
            self._tick_drive()

    def _tick_drive(self):
        """运动模式：左摇杆上下→vx，右摇杆左右→vy，L2/R2→左右转。"""
        vx = self.dir_linear * self._axis(self.axis_linear) * self.max_vx
        vy = self.dir_strafe * self._axis(self.axis_strafe) * self.max_vy
        wz = 0.0
        if self._is_pressed(self.btn_l2):
            wz = self.turn_wz           # 左转（逆时针为正）
        elif self._is_pressed(self.btn_r2):
            wz = -self.turn_wz          # 右转

        m = Twist()
        m.linear.x = vx
        m.linear.y = vy
        m.angular.z = wz
        self._pub_vel.publish(m)

    def _tick_gimbal(self):
        """云台模式：摇杆控制 yaw/extend，Y/A 控制 lift，L1/L2 控制载物盘。"""
        # 摇杆：左摇杆左右 → yaw，右摇杆上下 → extend（X 轴伸缩）
        self._gimbal[0] += self.dir_yaw * self._axis(self.axis_yaw) * self.gimbal_step_deg
        self._gimbal[2] += self.dir_extend * self._axis(self.axis_extend) * self.gimbal_step_deg
        # 按钮：Y/A → lift 升/降
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

        # 载物盘：L1 逆时针 / R1 顺时针（步进），变了才发
        # 注：+/- 代表舵机角度增减，具体"顺/逆"方向取决于机械安装，联调若反了对调两个 +/- 即可
        old_plate = self._servo[1]
        if self._is_pressed(self.btn_l1):
            self._servo[1] -= self.servo_step_deg   # L1 逆时针
        if self._is_pressed(self.btn_r1):
            self._servo[1] += self.servo_step_deg   # R1 顺时针
        self._servo[1] = max(0.0, min(180.0, self._servo[1]))
        if self._servo[1] != old_plate:
            self._send_servo()

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
        self.get_logger().info(f'摄像头 -> {self._servo[2]}°')

    def _cam_down(self):
        self._servo[2] = max(0.0, self._servo[2] - self.cam_step_deg)
        self._send_servo()
        self.get_logger().info(f'摄像头 -> {self._servo[2]}°')

    # ---------------- 工具 ----------------

    def _axis(self, idx: int) -> float:
        """读某个轴的当前值，做死区处理（|值|<deadzone 视为 0），返回 [-1,1]。"""
        if idx < len(self._axes):
            v = float(self._axes[idx])
            if abs(v) < self.deadzone:
                return 0.0
            return max(-1.0, min(1.0, v))
        return 0.0

    def _dpad(self, idx: int) -> int:
        """读方向键（也是轴），三值化：-1 / 0 / +1。"""
        if idx < len(self._axes):
            v = float(self._axes[idx])
            if abs(v) < self.deadzone:
                return 0
            return 1 if v > 0 else -1
        return 0

    def _is_pressed(self, idx: int) -> bool:
        """某个按键当前是否被按住。"""
        return idx < len(self._buttons) and bool(self._buttons[idx])

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
