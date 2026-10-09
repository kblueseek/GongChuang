#!/usr/bin/env python3
"""obstacle_detect：订阅 /scan，按 4 个扇区提取最近障碍距离。

发布 /obstacles（std_msgs/Float32MultiArray）：[front, left, right, back]，单位米。
无有效测量点时用 clear_range 封顶（视为开阔）。
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray


class ObstacleDetectNode(Node):
    def __init__(self):
        super().__init__('obstacle_detect')
        self.declare_parameter('half_deg', 60.0)     # 每个扇区半角
        self.declare_parameter('clear_range', 3.0)   # 无有效测量时的"开阔"值（m）
        self.half_deg = self.get_parameter('half_deg').value
        self.clear_range = self.get_parameter('clear_range').value

        self._pub = self.create_publisher(Float32MultiArray, '/obstacles', 10)
        self.create_subscription(LaserScan, '/scan', self._cb, 10)

    def _cb(self, scan: LaserScan):
        front = self._sector(scan, 0.0)
        left = self._sector(scan, 90.0)
        right = self._sector(scan, 270.0)
        back = self._sector(scan, 180.0)
        self._pub.publish(Float32MultiArray(data=[front, left, right, back]))

    def _sector(self, scan: LaserScan, center_deg: float) -> float:
        best = self.clear_range
        for i, r in enumerate(scan.ranges):
            ang_deg = math.degrees(scan.angle_min + i * scan.angle_increment) % 360.0
            diff = (ang_deg - center_deg + 180.0) % 360.0 - 180.0  # 最短角度差
            if abs(diff) <= self.half_deg and math.isfinite(r) and r >= scan.range_min:
                best = min(best, float(r))
        return best


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
