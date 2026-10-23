# 只启动「F407 桥接 + 状态屏 + 运动规划 + 手柄」，不含激光雷达/障碍检测。
#
# 用于：还没接雷达（或雷达联调先放一边），只想用手柄 / 上位机 / 序列控制小车。
# 数据链路：
#   手柄 -> joy_node -> joy_control -> /cmd_vel -> f407_bridge --蓝牙--> F407
#   F407 --LoRa--> f407_bridge -> /odom -> motion_planner / 状态屏
#
# 运行：ros2 launch lidar_bringup car_only.launch.py
#
# 需要改的常见参数：
#   ble_address    F407 蓝牙模组 MAC（留空=自动扫描；用 scripts/ble_scan.py 查地址）
#   lora_device    LoRa 串口（Pi5 加了 dtoverlay=uart2-pi5 后一般是 /dev/ttyAMA2）
#   width/height   状态屏分辨率：1.96 寸多半是 160x80，128x64 就保持默认

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # F407 桥接：蓝牙上行 + LoRa 下行
        Node(
            package='f407_bridge',
            executable='f407_bridge_node',
            name='f407_bridge',
            output='screen',
            parameters=[{
                'uplink': 'ble',
                'ble_address': '',           # 留空自动扫描；知道 MAC 就填上，启动更快
                'ble_name': '',              # 也可以按名字模糊匹配，例如 'JDY'
                'downlink': 'lora',
                'lora_device': '/dev/ttyAMA2',
                'lora_baudrate': 115200,
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
