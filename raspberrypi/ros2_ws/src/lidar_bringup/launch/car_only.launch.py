# 只启动「F407 桥接 + 状态屏 + 运动规划 + 手柄」，不含激光雷达/障碍检测。
#
# 用于：还没接雷达（或雷达联调先放一边），只想用手柄 / 上位机 / 序列控制小车。
# 数据链路（有线，全双工 USART6 <-> /dev/ttyAMA0，务必两边共地）：
#   手柄 -> joy_node -> joy_control -> /cmd_vel -> f407_bridge --串口--> F407
#   F407 --串口--> f407_bridge -> /odom -> motion_planner / 状态屏
#
# 运行：ros2 launch lidar_bringup car_only.launch.py
#
# 需要改的常见参数：
#   wire_device    F407 有线串口（默认 /dev/ttyAMA0，GPIO14=TX / GPIO15=RX）
#   width/height   状态屏分辨率：1.96 寸多半是 160x80，128x64 就保持默认

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # F407 桥接：有线串口（上行 + 下行共用一条全双工 USART6）
        Node(
            package='f407_bridge',
            executable='f407_bridge_node',
            name='f407_bridge',
            output='screen',
            parameters=[{
                'uplink': 'wire',
                'wire_device': '/dev/ttyAMA0',
                'downlink': 'wire',
            }],
        ),
        # 状态屏：I2C OLED（GPIO2/3），显示在线状态、坐标、朝向
        Node(
            package='f407_bridge',
            executable='status_display_node',
            name='status_display',
            output='screen',
            parameters=[{'width': 128, 'height': 64}],   # 160x80 的屏改成 160 / 80
        ),
        # 运动规划：任务状态机（收上位机 /motion_load 下发序列，逐条执行、判断到位）
        Node(
            package='motion_planner',
            executable='motion_planner_node',
            name='motion_planner',
            output='screen',
        ),
        # 手柄驱动（USB 手柄 -> /joy，autorepeat 让摇杆按住时持续上报）
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'autorepeat_rate': 20.0}],
        ),
        # 手柄控制（/joy -> /cmd_vel /cmd_enable /cmd_servo /cmd_gimbal /cmd_led）
        Node(
            package='joy_control',
            executable='joy_control_node',
            name='joy_control',
            output='screen',
        ),
    ])
