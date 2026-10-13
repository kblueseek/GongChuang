#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""手柄控制节点：把 USB 手柄的摇杆/按键翻译成整车控制话题。

================ 大白话：这个节点在干什么 ================

手柄（WHEELTEC PS2 USB 手柄）插到树莓派后，由 ROS2 的 joy 包（joy_node）
读取，发布到 /joy 话题（sensor_msgs/Joy：一堆轴 axes[] 和按键 buttons[]）。

本节点订阅 /joy，把"摇杆推到哪、按下哪个键"翻译成整车控制指令，发到
f407_bridge 已经订阅好的话题上：

  摇杆（运动模式，默认）→ /cmd_vel（速度）
  按键                  → /cmd_enable /cmd_estop /cmd_led /cmd_gimbal_home
  按住 L2（云台模式）   → 摇杆控制云台三轴 /cmd_gimbal
  按住 R2（舵机模式）   → 摇杆控制三个舵机 /cmd_servo

================ 按键/轴编号说明（重要） ================

不同手柄（甚至同一手柄不同驱动）的 axes/buttons 编号可能不一样。
下面的默认值只是"常见布局"的猜测，**联调时必须先用 jstest 确认实际编号**：

  sudo apt install jstest-gtk   # 或命令行 jstest /dev/input/js0
  ros2 run joy joy_node         # 然后 ros2 topic echo /joy

拿到真实编号后，用 ros2 参数覆盖即可，不用改代码，例如：
  ros2 run joy_control joy_control_node --ros-args -p btn_enable:=13

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

        # ================= 参数：轴映射 =================
        self.declare_parameter('axis_linear', 1)     # 前进/后退 vx（左摇杆 Y）
        self.declare_parameter('axis_strafe', 0)     # 平移 vy（左摇杆 X）
        self.declare_parameter('axis_angular', 2)    # 自转 wz（右摇杆 X）
        self.declare_parameter('axis_aux', 3)        # 云台/舵机模式的第二轴（右摇杆 Y）

        # ================= 参数：按键映射 =================
        self.declare_parameter('btn_enable', 9)      # 使能/失能切换（Start）
        self.declare_parameter('btn_estop', 8)       # 急停（Select，按住=急停，松开=解除）
        self.declare_parameter('btn_led_up', 5)      # 补光灯 +一档（R1）
        self.declare_parameter('btn_led_down', 4)    # 补光灯 -一档（L1）
        self.declare_parameter('btn_gimbal_mode', 6) # 按住进入云台模式（L2）
        self.declare_parameter('btn_servo_mode', 7)  # 按住进入舵机模式（R2）
        self.declare_parameter('btn_gimbal_home', 3) # 云台回零（Y/三角键）

        # ================= 参数：运动/步进参数 =================
        self.declare_parameter('max_vx', 1.0)        # 前进满量程 m/s（和 RobotConfig.CHASSIS_MAX_SPEED_X 一致）
        self.declare_parameter('max_vy', 1.0)        # 平移满量程 m/s
        self.declare_parameter('max_wz', 3.0)        # 自转满量程 rad/s
        self.declare_parameter('deadzone', 0.05)     # 摇杆死区：绝对值小于它视为 0
        self.declare_parameter('led_step', 20)       # 补光灯每按一下增减的亮度 %
        self.declare_parameter('gimbal_step_deg', 2.0)  # 云台每 tick 满量程步进角度 °
        self.declare_parameter('servo_step_deg', 5.0)   # 舵机每 tick 满量程步进角度 °

        # 读出参数
        self.axis_linear = self.get_parameter('axis_linear').value
        self.axis_strafe = self.get_parameter('axis_strafe').value
        self.axis_angular = self.get_parameter('axis_angular').value
        self.axis_aux = self.get_parameter('axis_aux').value
        self.btn_enable = self.get_parameter('btn_enable').value
        self.btn_estop = self.get_parameter('btn_estop').value
        self.btn_led_up = self.get_parameter('btn_led_up').value
        self.btn_led_down = self.get_parameter('btn_led_down').value
        self.btn_gimbal_mode = self.get_parameter('btn_gimbal_mode').value
        self.btn_servo_mode = self.get_parameter('btn_servo_mode').value
        self.btn_gimbal_home = self.get_parameter('btn_gimbal_home').value
        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_wz = float(self.get_parameter('max_wz').value)
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.led_step = int(self.get_parameter('led_step').value)
        self.gimbal_step_deg = float(self.get_parameter('gimbal_step_deg').value)
        self.servo_step_deg = float(self.get_parameter('servo_step_deg').value)

        # ================= 发布的话题 =================
        self._pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self._pub_enable = self.create_publisher(Bool, '/cmd_enable', 10)
        self._pub_estop = self.create_publisher(Bool, '/cmd_estop', 10)
        self._pub_led = self.create_publisher(Int32, '/cmd_led', 10)
        self._pub_gimbal_home = self.create_publisher(Int32, '/cmd_gimbal_home', 10)
        self._pub_gimbal = self.create_publisher(Float32MultiArray, '/cmd_gimbal', 10)
        self._pub_servo = self.create_publisher(Float32MultiArray, '/cmd_servo', 10)

        # ================= 订阅 =================
        self.create_subscription(Joy, '/joy', self._cb_joy, 10)

        # ================= 内部状态 =================
        self._axes = [0.0] * 8            # 最新的摇杆值
        self._buttons = []                # 最新的按键状态
        self._prev_buttons = []           # 上一拍的按键状态（用于边缘检测）
        self._enabled = False             # 电机使能状态
        self._led = 0                     # 补光灯亮度 %
        self._gimbal = [0.0, 0.0, 0.0]    # 云台当前目标 [yaw, lift, extend] °
        self._servo = [90.0, 0.0, 90.0]   # 舵机当前目标 [夹爪, 载物盘, 相机] °（和 F407 上电默认一致）

        # 定时器：10Hz 持续发运动指令（摇杆回中发 0，避免车保持最后一次速度继续跑）
        self._timer = self.create_timer(0.1, self._tick)
        self.get_logger().info('joy_control 就绪，等待 /joy（先确认手柄已插、joy_node 在跑）')

    # ---------------- 摇杆/按键回调 ----------------

    def _cb_joy(self, msg: Joy):
        """收到 /joy：记录最新轴值，并做按键的边缘检测（只在"按下瞬间"触发一次）。"""
        # 记录轴值（不够长的补 0）
        self._axes = list(msg.axes) + [0.0] * max(0, 8 - len(msg.axes))
        self._buttons = list(msg.buttons)
        # 补足上一拍数组长度，便于下面按下标比较
        prev = self._prev_buttons + [0] * max(0, len(self._buttons) - len(self._prev_buttons))

        # 遍历每个按键，检测"从 0 变成 1"的上升沿
        for i, pressed in enumerate(self._buttons):
            was_pressed = prev[i] if i < len(prev) else 0
            if pressed and not was_pressed:
                self._on_button_pressed(i)

        self._prev_buttons = list(self._buttons)

    def _on_button_pressed(self, i: int):
        """某个按键被按下（边缘触发），按编号分发到对应动作。"""
        if i == self.btn_enable:
            self._enabled = not self._enabled
            self._pub_enable.publish(Bool(data=self._enabled))
            self.get_logger().info(f'使能 -> {self._enabled}')
        elif i == self.btn_led_up:
            self._led = min(100, self._led + self.led_step)
            self._pub_led.publish(Int32(data=self._led))
            self.get_logger().info(f'补光灯 -> {self._led}%')
        elif i == self.btn_led_down:
            self._led = max(0, self._led - self.led_step)
            self._pub_led.publish(Int32(data=self._led))
            self.get_logger().info(f'补光灯 -> {self._led}%')
        elif i == self.btn_gimbal_home:
            self._pub_gimbal_home.publish(Int32(data=0x07))   # 三轴全回零
            self._gimbal = [0.0, 0.0, 0.0]
            self.get_logger().info('云台回零')

    # ---------------- 定时器：运动/云台/舵机 ----------------

    def _tick(self):
        """每 0.1s 执行一次：发速度（始终发），云台/舵机模式按摇杆步进。"""
        # 急停：按住 btn_estop 时持续发 true，松开发 false 解除
        estop = self._is_pressed(self.btn_estop)
        if estop != getattr(self, '_last_estop', None):
            self._pub_estop.publish(Bool(data=estop))
            self._last_estop = estop
            self.get_logger().info(f'急停 -> {estop}')

        # 模式判断：按住云台键 / 舵机键，否则运动模式
        if self._is_pressed(self.btn_gimbal_mode):
            self._tick_gimbal()
        elif self._is_pressed(self.btn_servo_mode):
            self._tick_servo()
        else:
            self._tick_velocity()

    def _tick_velocity(self):
        """运动模式：摇杆 -> /cmd_vel，死区处理，回中发 0。"""
        vx = self._axis(self.axis_linear) * self.max_vx
        vy = self._axis(self.axis_strafe) * self.max_vy
        wz = self._axis(self.axis_angular) * self.max_wz
        m = Twist()
        m.linear.x = vx
        m.linear.y = vy
        m.angular.z = wz
        self._pub_vel.publish(m)

    def _tick_gimbal(self):
        """云台模式（按住 L2）：摇杆步进云台三轴角度。
        左摇杆 X -> yaw，左摇杆 Y -> lift，右摇杆 X -> extend。"""
        self._gimbal[0] += self._axis(self.axis_strafe) * self.gimbal_step_deg    # yaw
        self._gimbal[1] += self._axis(self.axis_linear) * self.gimbal_step_deg   # lift
        self._gimbal[2] += self._axis(self.axis_angular) * self.gimbal_step_deg  # extend
        # 软限位（和 RobotConfig.h 一致，防撞结构）
        self._gimbal[0] = max(-180.0, min(180.0, self._gimbal[0]))
        self._gimbal[1] = max(0.0, min(200.0, self._gimbal[1]))
        self._gimbal[2] = max(0.0, min(200.0, self._gimbal[2]))
        a = Float32MultiArray()
        a.data = [float(x) for x in self._gimbal]
        self._pub_gimbal.publish(a)

    def _tick_servo(self):
        """舵机模式（按住 R2）：摇杆步进三个舵机角度。
        左摇杆 Y -> 夹爪，右摇杆 Y -> 载物盘，右摇杆 X -> 相机。"""
        self._servo[0] += self._axis(self.axis_linear) * self.servo_step_deg   # 夹爪
        self._servo[1] += self._axis(self.axis_aux) * self.servo_step_deg      # 载物盘
        self._servo[2] += self._axis(self.axis_angular) * self.servo_step_deg  # 相机
        for i in range(3):
            self._servo[i] = max(0.0, min(180.0, self._servo[i]))
        a = Float32MultiArray()
        a.data = [float(x) for x in self._servo]
        self._pub_servo.publish(a)

    # ---------------- 工具 ----------------

    def _axis(self, idx: int) -> float:
        """读某个轴的当前值，做死区处理（|值|<deadzone 视为 0）。返回 [-1,1]。"""
        if idx < len(self._axes):
            v = float(self._axes[idx])
            if abs(v) < self.deadzone:
                return 0.0
            return max(-1.0, min(1.0, v))
        return 0.0

    def _is_pressed(self, idx: int) -> bool:
        """某个按键当前是否被按住。"""
        return idx < len(self._buttons) and bool(self._buttons[idx])


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
