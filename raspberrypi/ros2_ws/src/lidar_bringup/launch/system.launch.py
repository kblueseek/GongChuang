# 总启动文件：一次把整车 ROS2 侧的所有节点都拉起来。
#
# 大白话：运行 ros2 launch lidar_bringup system.launch.py 就等于同时启动下面 4 个程序：
#   1. 雷达驱动      -> 产出 /scan
#   2. 障碍检测      -> /scan 变成 /obstacles
#   3. F407 桥接     -> 话题和串口互转
#   4. 运动规划      -> /obstacles 变成 /cmd_vel
# 它们靠"话题"串成一条流水线。

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # 1. 激光雷达驱动（带参数文件）
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            parameters=[FindPackageShare('lidar_bringup') + '/config/x3_pro.yaml'],
        ),
        # 2. 障碍检测节点（我们自己写的）
        Node(
            package='obstacle_detect',
            executable='obstacle_detect_node',   # 这个名字对应 setup.py 里 entry_points 定义的命令
            name='obstacle_detect',
            output='screen',
        ),
        # 3. F407 桥接节点（串口翻译官）
        Node(
            package='f407_bridge',
            executable='f407_bridge_node',
            name='f407_bridge',
            output='screen',
        ),
        # 4. 运动规划节点（避障）
        Node(
            package='motion_planner',
            executable='motion_planner_node',
            name='motion_planner',
            output='screen',
        ),
    ])
