#!/usr/bin/env python3
"""f407_bridge：把 ROS2 话题 <-> F407 串口（PiComm，USART6 115200）桥接起来。

输入（订阅）：
    /cmd_vel     geometry_msgs/Twist           -> 0x01 速度（linear.x/y=m/s，angular.z=rad/s）
    /cmd_servo   std_msgs/Float32MultiArray    -> [夹爪°, 托盘°, 相机°] -> 0x04/0x05/0x06
    /cmd_enable  std_msgs/Bool                 -> 0x07 使能/失能（mask=0x7F 全部电机）
    /cmd_estop   std_msgs/Bool                 -> 0x08 急停（true 触发）

输出（发布）：
    /odom         nav_msgs/Odometry            <- 0x81 里程计（x,y,theta）
    /robot_status std_msgs/Int32MultiArray     <- 0x83 [state, err, motor_online_mask]

内部：1Hz 发 0x10 心跳，10Hz 轮询串口。
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32MultiArray, Int32MultiArray

from f407_bridge.pi_comm import (
    PiCommLink,
    RPT_ODOM,
    RPT_STATUS,
    parse_odom,
    parse_status,
)


class F407BridgeNode(Node):
    def __init__(self):
        super().__init__('f407_bridge')

        self.declare_parameter('device', '/dev/serial0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'odom')

        device = self.get_parameter('device').value
        baudrate = self.get_parameter('baudrate').value
        self._link = PiCommLink(device=device, baudrate=baudrate)
        self.get_logger().info(f'串口 {device} @ {baudrate}')

        # 上行订阅
        self.create_subscription(Twist, '/cmd_vel', self._cb_cmd_vel, 10)
        self.create_subscription(Float32MultiArray, '/cmd_servo', self._cb_cmd_servo, 10)
        self.create_subscription(Bool, '/cmd_enable', self._cb_cmd_enable, 10)
        self.create_subscription(Bool, '/cmd_estop', self._cb_cmd_estop, 10)

        # 下行发布
        self._pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self._pub_status = self.create_publisher(Int32MultiArray, '/robot_status', 10)

        self._heartbeat_timer = self.create_timer(1.0, self._on_heartbeat)
        self._poll_timer = self.create_timer(0.1, self._on_poll)  # 10Hz

    # ---- 订阅回调 ----
    def _cb_cmd_vel(self, msg: Twist):
        ok = self._link.send_vel(msg.linear.x, msg.linear.y, msg.angular.z)
        if not ok:
            self.get_logger().debug('VEL 发送失败', throttle_duration_sec=2.0)

    def _cb_cmd_servo(self, msg: Float32MultiArray):
        if len(msg.data) >= 1:
            self._link.send_gripper(msg.data[0])
        if len(msg.data) >= 2:
            self._link.send_plate(msg.data[1])
        if len(msg.data) >= 3:
            self._link.send_cam(msg.data[2])

    def _cb_cmd_enable(self, msg: Bool):
        self._link.send_enable(bool(msg.data), mask=0x7F)

    def _cb_cmd_estop(self, msg: Bool):
        if bool(msg.data):
            self._link.send_estop()

    # ---- 定时器 ----
    def _on_heartbeat(self):
        self._link.send_heartbeat()

    def _on_poll(self):
        for cmd, data in self._link.poll():
            if cmd == RPT_ODOM:
                self._publish_odom(parse_odom(data))
            elif cmd == RPT_STATUS:
                state, err, mask = parse_status(data)
                self._pub_status.publish(Int32MultiArray(data=[state, err, mask]))

    def _publish_odom(self, pose):
        x, y, theta = pose
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').value
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
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
        node._link.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
