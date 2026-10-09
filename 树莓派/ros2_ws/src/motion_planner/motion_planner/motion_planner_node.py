#!/usr/bin/env python3
"""motion_planner：把 /obstacles 转成 /cmd_vel（极简避障，占位实现）。

规则：
    front < stop_range  -> 停，朝更开阔一侧原地转
    front < slow_range  -> 线性减速
    否则                -> 以 max_speed 前进；侧面过近则横向让开

真正的任务规划（循迹/取放/码放/对接）后续替换本节点。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class MotionPlannerNode(Node):
    def __init__(self):
        super().__init__('motion_planner')
        self.declare_parameter('max_speed', 0.3)      # m/s
        self.declare_parameter('slow_range', 0.8)     # m
        self.declare_parameter('stop_range', 0.35)    # m
        self.declare_parameter('turn_speed', 0.6)     # rad/s
        self.declare_parameter('side_margin', 0.25)   # m，侧面安全距离
        self.max_speed = self.get_parameter('max_speed').value
        self.slow_range = self.get_parameter('slow_range').value
        self.stop_range = self.get_parameter('stop_range').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.side_margin = self.get_parameter('side_margin').value

        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Float32MultiArray, '/obstacles', self._cb, 10)

    def _cb(self, msg: Float32MultiArray):
        front, left, right, _back = msg.data
        tw = Twist()

        if front < self.stop_range:
            tw.linear.x = 0.0
            tw.angular.z = self.turn_speed if right >= left else -self.turn_speed
        elif front < self.slow_range:
            ratio = (front - self.stop_range) / (self.slow_range - self.stop_range)
            tw.linear.x = self.max_speed * max(0.0, ratio)
        else:
            tw.linear.x = self.max_speed

        # 侧面让行（浅规则，避免贴着墙走）
        if tw.linear.x > 0.0:
            if left < self.side_margin:
                tw.linear.y = -0.1
            elif right < self.side_margin:
                tw.linear.y = 0.1

        self._pub.publish(tw)


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
