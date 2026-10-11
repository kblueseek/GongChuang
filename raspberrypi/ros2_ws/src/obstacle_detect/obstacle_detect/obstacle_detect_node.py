#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""障碍检测节点：把激光雷达的一圈数据，简化成"前后左右最近距离"。

================ 大白话：这个节点在干什么 ================

激光雷达（YDLIDAR X3 PRO）每转一圈，会给出几百个点的距离（哪个角度、多远）。
这一大堆数据对上层来说太细了。我们这个节点把它"汇总"成 4 个数：

    [前面最近多远, 左边最近多远, 右边最近多远, 后面最近多远]

然后发到话题 /obstacles 上，给后面的 motion_planner 用。

为什么要汇总？因为规划器只关心"哪个方向有障碍、大概多远"，不需要几百个点。
"""

import math   # 数学函数（角度、弧度转换、判断无穷大等）

import rclpy                      # ROS2 的 Python 客户端库
from rclpy.node import Node       # 所有节点的"父类"，我们的节点要继承它
from sensor_msgs.msg import LaserScan        # 激光雷达的标准消息类型
from std_msgs.msg import Float32MultiArray   # 浮点数组消息类型（用来发4个距离）


class ObstacleDetectNode(Node):
    """障碍检测节点。"""

    def __init__(self):
        # 调父类初始化，并给节点起名字 "obstacle_detect"。
        # 这个名字会在 ros2 node list 里看到。
        super().__init__('obstacle_detect')

        # declare_parameter = 声明一个"参数"（可配置项），并给它一个默认值。
        # 好处：以后不用改代码，启动时用命令行就能改，比如 -p half_deg:=45.0
        self.declare_parameter('half_deg', 60.0)     # 每个扇区的半角（±60度）
        self.declare_parameter('clear_range', 3.0)   # 某个方向没测到东西时，默认按3米算（视为开阔）

        # get_parameter(...).value = 把上面声明的参数值读出来，存到 self 里方便用
        self.half_deg = self.get_parameter('half_deg').value
        self.clear_range = self.get_parameter('clear_range').value

        # 创建"发布者"：往话题 /obstacles 发 Float32MultiArray 类型的消息。
        # 最后一个参数 10 是 QoS 队列深度：最多缓存10条待发的消息，防止积压。
        self._pub = self.create_publisher(Float32MultiArray, '/obstacles', 10)

        # 创建"订阅者"：订阅话题 /scan（雷达数据），每收到一帧就调用 self._cb 处理。
        self.create_subscription(LaserScan, '/scan', self._cb, 10)

    def _cb(self, scan: LaserScan):
        """回调函数：每收到一帧雷达数据，就自动调用一次。参数 scan 就是那帧数据。"""
        # 分别求四个方向扇区内的最近距离。角度以"车头朝前"为基准：
        #   0° 前，90° 左，270° 右（= -90°），180° 后
        front = self._sector(scan, 0.0)     # 正前方
        left = self._sector(scan, 90.0)     # 左侧
        right = self._sector(scan, 270.0)   # 右侧（270度就是 -90度，写法上方便取模）
        back = self._sector(scan, 180.0)    # 正后方

        # 把4个数打包成一条消息，发出去。data=[...] 按顺序就是 前、左、右、后。
        self._pub.publish(Float32MultiArray(data=[front, left, right, back]))

    def _sector(self, scan: LaserScan, center_deg: float) -> float:
        """求"以 center_deg 为中心、正负 half_deg 范围"这个扇区里的最近障碍距离。

        返回：扇区里最近的有效距离（米）。如果扇区里没有任何有效点，就返回 clear_range。
        """
        best = self.clear_range   # 先假设"很开阔"，后面遇到更近的就更新
        # 遍历雷达这一圈里的每一个点。r = 该点距离，i = 第几个点。
        for i, r in enumerate(scan.ranges):
            # 算出第 i 个点对应的角度（弧度），再转成角度（0~360）。
            # scan.angle_min 是起始角，scan.angle_increment 是相邻两个点的角度间隔。
            ang_deg = math.degrees(scan.angle_min + i * scan.angle_increment) % 360.0

            # 算这个点和"扇区中心"的角度差，并归一化到 -180~180 之间。
            # 这个公式能正确处理 0° 和 359° 其实只差 1° 的"绕圈"情况。
            diff = (ang_deg - center_deg + 180.0) % 360.0 - 180.0

            # 三个条件同时满足才算有效点：
            #   1. abs(diff) <= half_deg：这个点落在扇区内
            #   2. math.isfinite(r)：距离是有限值（排除 inf/nan；雷达没打到东西常用 inf 表示）
            #   3. r >= scan.range_min：距离不小于最小量程（排除 0 或负值这种无效点）
            if abs(diff) <= self.half_deg and math.isfinite(r) and r >= scan.range_min:
                best = min(best, float(r))   # 取"更近"的那个
        return best


def main(args=None):
    """ROS2 节点的标准入口：初始化 -> 建节点 -> 转起来 -> 退出时清理。"""
    rclpy.init(args=args)              # 初始化 ROS2 客户端库（必须先调）
    node = ObstacleDetectNode()        # 创建我们的节点
    try:
        rclpy.spin(node)               # 让节点"转起来"：不停处理订阅回调，直到被中断
    except KeyboardInterrupt:
        pass                           # 用户按了 Ctrl-C，正常退出
    finally:
        node.destroy_node()            # 释放节点占用的资源
        rclpy.shutdown()               # 关闭 ROS2 客户端库


if __name__ == '__main__':
    main()   # 只有直接运行这个文件时才执行 main()（被 import 时不执行）
