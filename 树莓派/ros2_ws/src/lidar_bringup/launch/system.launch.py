from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """总启动：激光雷达 + 障碍检测 + F407 桥接 + 运动规划。"""
    return LaunchDescription([
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            parameters=[FindPackageShare('lidar_bringup') + '/config/x3_pro.yaml'],
        ),
        Node(
            package='obstacle_detect',
            executable='obstacle_detect_node',
            name='obstacle_detect',
            output='screen',
        ),
        Node(
            package='f407_bridge',
            executable='f407_bridge_node',
            name='f407_bridge',
            output='screen',
        ),
        Node(
            package='motion_planner',
            executable='motion_planner_node',
            name='motion_planner',
            output='screen',
        ),
    ])
