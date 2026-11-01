# PID 滑杆调参上位机（tkinter 窗口，需图形桌面）。
#
# 用法：先跑 run_car + gimbal_track.launch.py，再起这个：
#   ros2 launch pid_tuner pid_tuner.launch.py
# 换配置（调别的节点）：
#   ros2 launch pid_tuner pid_tuner.launch.py config_file:=/path/to/other.yaml

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pid_tuner',
            executable='pid_tuner_node',
            name='pid_tuner',
            output='screen',
            parameters=[{
                'config_file': '',   # 留空 = share/pid_tuner/config/pid_tuner.yaml
            }],
        ),
    ])
