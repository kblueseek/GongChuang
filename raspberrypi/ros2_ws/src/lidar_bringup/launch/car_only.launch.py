# 只启动「F407 桥接 + 运动规划」两个节点，不含激光雷达/障碍检测。
#
# 用于：还没接雷达（或雷达联调先放一边），只想用上位机 / 直接发序列控制小车底盘和云台时。
# 数据链路：上位机 -> /motion_load -> motion_planner -> /cmd_pose -> f407_bridge -> 串口 -> F407
#
# 运行：ros2 launch lidar_bringup car_only.launch.py
# 改串口设备：把下面 f407_bridge 的 device 参数改成实际设备（默认 /dev/ttyAMA0，即 GPIO14/15）。

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # F407 桥接：串口 <-> 话题（设备名/波特率可用参数覆盖）
        Node(
            package='f407_bridge',
            executable='f407_bridge_node',
            name='f407_bridge',
            output='screen',
            parameters=[{'device': '/dev/ttyAMA0', 'baudrate': 115200}],
        ),
        # 运动规划：任务状态机（收上位机 /motion_load 下发序列，逐条执行、判断到位）
        Node(
            package='motion_planner',
            executable='motion_planner_node',
            name='motion_planner',
            output='screen',
        ),
    ])
