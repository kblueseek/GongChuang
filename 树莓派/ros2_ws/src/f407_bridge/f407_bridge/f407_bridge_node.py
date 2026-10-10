#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""F407 桥接节点：ROS2 世界和 F407 单片机之间的"翻译官"。

================ 大白话：这个节点在干什么 ================

整个树莓派里，只有这个节点直接碰串口（连 F407 的那根线）。
它的工作分两个方向：

  方向1（ROS2 -> F407）：别的节点在话题上发命令（比如 /cmd_vel 速度），
                         这个节点收下，转成协议字节，通过串口发给 F407。

  方向2（F407 -> ROS2）：F407 通过串口回传里程计、状态，
                         这个节点解析后，发到话题上给别的节点用。

所以它像一座桥，把"ROS2 的话题"和"串口的字节"互相翻译。
"""

import math   # 用于算四元数（姿态的一种数学表示）

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Quaternion, Twist   # Pose2D=平面位姿(x,y,朝向), Quaternion=四元数, Twist=速度
from nav_msgs.msg import Odometry                 # 里程计消息
from std_msgs.msg import Bool, Float32MultiArray, Int32MultiArray

# 从本包内导入协议模块（pi_comm.py 是和 comm/pi_comm.py 内容一致的副本）
from f407_bridge.pi_comm import (
    PiCommLink,       # 串口连接类（负责收发、重连、校验）
    RPT_ODOM,         # 回传数据编号：里程计
    RPT_STATUS,       # 回传数据编号：状态
    parse_odom,       # 解析里程计的函数
    parse_status,     # 解析状态的函数
)


class F407BridgeNode(Node):
    def __init__(self):
        super().__init__('f407_bridge')   # 节点名

        # 声明三个可配置参数（串口设备、波特率、坐标系名）
        self.declare_parameter('device', '/dev/serial0')   # 树莓派的串口设备
        self.declare_parameter('baudrate', 115200)         # 波特率，必须和 F407 一致
        self.declare_parameter('frame_id', 'odom')         # 里程计的坐标系名

        # 读出参数值
        device = self.get_parameter('device').value
        baudrate = self.get_parameter('baudrate').value

        # 创建串口连接对象（就是 pi_comm.py 里的那个类）
        self._link = PiCommLink(device=device, baudrate=baudrate)
        self.get_logger().info(f'串口 {device} @ {baudrate}')   # 打一条日志，方便排查

        # ================= 方向1：订阅话题（ROS2 -> F407） =================
        # 订阅 /cmd_vel（速度），收到就调用 _cb_cmd_vel
        self.create_subscription(Twist, '/cmd_vel', self._cb_cmd_vel, 10)
        # 订阅 /cmd_servo（舵机角度数组 [夹爪,托盘,相机]）
        self.create_subscription(Float32MultiArray, '/cmd_servo', self._cb_cmd_servo, 10)
        # 订阅 /cmd_enable（使能/失能）
        self.create_subscription(Bool, '/cmd_enable', self._cb_cmd_enable, 10)
        # 订阅 /cmd_estop（急停）
        self.create_subscription(Bool, '/cmd_estop', self._cb_cmd_estop, 10)
        # 订阅 /cmd_estop_clear（解除急停）
        self.create_subscription(Bool, '/cmd_estop_clear', self._cb_cmd_estop_clear, 10)
        # 订阅 /cmd_pose（位姿目标：走到 (x,y) 并转到朝向 theta）
        self.create_subscription(Pose2D, '/cmd_pose', self._cb_cmd_pose, 10)
        # 订阅 /cmd_gimbal（云台三轴角度 [yaw°, lift°, extend°]）
        self.create_subscription(Float32MultiArray, '/cmd_gimbal', self._cb_cmd_gimbal, 10)

        # ================= 方向2：发布话题（F407 -> ROS2） =================
        # 发布 /odom（里程计）
        self._pub_odom = self.create_publisher(Odometry, '/odom', 10)
        # 发布 /robot_status（整车状态）
        self._pub_status = self.create_publisher(Int32MultiArray, '/robot_status', 10)

        # 两个定时器：
        #   每 1.0 秒调一次 _on_heartbeat（发心跳）
        #   每 0.1 秒调一次 _on_poll（去串口收数据，即 10Hz）
        self._heartbeat_timer = self.create_timer(1.0, self._on_heartbeat)
        self._poll_timer = self.create_timer(0.1, self._on_poll)

    # ---------------- 订阅回调（收到话题消息时自动执行） ----------------

    def _cb_cmd_vel(self, msg: Twist):
        """收到 /cmd_vel：把速度发给 F407。

        Twist 消息里：linear.x/y = 前后/左右的线速度(m/s)，angular.z = 旋转角速度(rad/s)。
        send_vel 内部会自动换算成协议要的 mm/s、mrad/s。
        """
        ok = self._link.send_vel(msg.linear.x, msg.linear.y, msg.angular.z)
        if not ok:
            # 发送失败（比如串口断了）。throttle_duration_sec=2.0 表示同一条日志最多2秒打一次，避免刷屏。
            self.get_logger().debug('VEL 发送失败', throttle_duration_sec=2.0)

    def _cb_cmd_servo(self, msg: Float32MultiArray):
        """收到 /cmd_servo：msg.data = [夹爪角度°, 托盘角度°, 相机角度°]。
        用 len 判断数组有几个元素，只发有的（缺哪个就不发哪个）。
        """
        if len(msg.data) >= 1:
            self._link.send_gripper(msg.data[0])   # 第1个：夹爪
        if len(msg.data) >= 2:
            self._link.send_plate(msg.data[1])     # 第2个：托盘
        if len(msg.data) >= 3:
            self._link.send_cam(msg.data[2])       # 第3个：相机

    def _cb_cmd_enable(self, msg: Bool):
        """收到 /cmd_enable：使能/失能电机。mask=0x7F 表示全部7个电机一起操作。"""
        self._link.send_enable(bool(msg.data), mask=0x7F)

    def _cb_cmd_estop(self, msg: Bool):
        """收到 /cmd_estop：只在"变成 true"时发一次急停（避免每收一条消息都重发）。"""
        if bool(msg.data):
            self._link.send_estop()

    def _cb_cmd_estop_clear(self, msg: Bool):
        """收到 /cmd_estop_clear：解除急停。"""
        if bool(msg.data):
            self._link.send_estop_clear()

    def _cb_cmd_pose(self, msg: Pose2D):
        """收到 /cmd_pose：把目标位姿发给 F407，位置+航向闭环在 F407 上执行。"""
        ok = self._link.send_pose(msg.x, msg.y, msg.theta)
        if not ok:
            self.get_logger().debug('POSE 发送失败', throttle_duration_sec=2.0)

    def _cb_cmd_gimbal(self, msg: Float32MultiArray):
        """收到 /cmd_gimbal：msg.data = [yaw°, lift°, extend°]。
        和舵机一样，有几个发几个。轴编号：0=yaw 1=lift(z升降) 2=extend(x伸缩)。"""
        if len(msg.data) >= 1:
            self._link.send_gimbal(0, msg.data[0])   # yaw 轴
        if len(msg.data) >= 2:
            self._link.send_gimbal(1, msg.data[1])   # lift(z升降) 轴
        if len(msg.data) >= 3:
            self._link.send_gimbal(2, msg.data[2])   # extend(x伸缩) 轴

    # ---------------- 定时器回调（按固定周期自动执行） ----------------

    def _on_heartbeat(self):
        """每秒发一次心跳。

        为什么要心跳？F407 那边有个"看门狗"：如果超过一定时间（默认500ms）没收到
        树莓派的任何数据，就认为树莓派掉线了，进入故障状态。所以哪怕没有运动指令，
        也要定期发个心跳，告诉 F407 "我还活着"。
        """
        self._link.send_heartbeat()

    def _on_poll(self):
        """每 0.1 秒去串口收一次数据。poll() 可能一次返回多帧（可能攒了好几帧）。"""
        for cmd, data in self._link.poll():
            if cmd == RPT_ODOM:                         # 是里程计帧
                self._publish_odom(parse_odom(data))    # 解析后发到 /odom
            elif cmd == RPT_STATUS:                     # 是状态帧
                state, err, mask = parse_status(data)   # 解析出 (状态, 错误码, 电机掩码)
                # 发到 /robot_status，内容是一个整数数组 [state, err, mask]
                self._pub_status.publish(Int32MultiArray(data=[state, err, mask]))

    def _publish_odom(self, pose):
        """把 (x米, y米, 朝向弧度) 组装成标准的里程计消息发出去。"""
        x, y, theta = pose
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()          # 当前时间戳
        msg.header.frame_id = self.get_parameter('frame_id').value  # 坐标系名
        msg.pose.pose.position.x = x    # x 坐标
        msg.pose.pose.position.y = y    # y 坐标
        # 里程计消息里的"朝向"要用四元数表示（4个数）。我们只有绕Z轴的偏航角 theta，
        # 绕Z轴转 theta 的四元数是 (x=0, y=0, z=sin(theta/2), w=cos(theta/2))。
        q = Quaternion()
        q.z = math.sin(theta / 2.0)
        q.w = math.cos(theta / 2.0)
        msg.pose.pose.orientation = q
        self._pub_odom.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = F407BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._link.close()      # 退出前关掉串口，释放资源
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
