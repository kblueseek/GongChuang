# 总启动文件：一次把整车 ROS2 侧的所有节点都拉起来（含激光雷达/障碍检测）。
#
# 用法：bash scripts/bringup.sh   （= ros2 launch car_bringup system.launch.py）
#
# 所有节点的可调参数集中在 config/vehicle.yaml（对标 F407 的 RobotConfig.h），
# 本文件只列节点清单。雷达自身的驱动参数在 config/x3_pro.yaml。

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

VEHICLE_CFG = PathJoinSubstitution([FindPackageShare('car_bringup'), 'config', 'vehicle.yaml'])
X3PRO_CFG = PathJoinSubstitution([FindPackageShare('car_bringup'), 'config', 'x3_pro.yaml'])


def car_node(package, executable, name, **kwargs):
    return Node(package=package, executable=executable, name=name,
                output='screen', parameters=[VEHICLE_CFG], **kwargs)


def generate_launch_description():
    return LaunchDescription([
        # 1. 激光雷达驱动（带参数文件）
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            parameters=[X3PRO_CFG],
        ),
        # 2. 障碍检测节点（/scan -> /obstacles）
        car_node('obstacle_detect', 'obstacle_detect_node', 'obstacle_detect'),
        # 3. F407 桥接节点（有线串口：上行 + 下行共用 USART6）
        car_node('f407_bridge', 'f407_bridge_node', 'f407_bridge'),
        # 4. 运动规划节点（任务状态机）
        car_node('motion_planner', 'motion_planner_node', 'motion_planner'),
        # 5. 圆环工位视觉矫正（calibrate 指令的执行者，触发时才开摄像头）
        car_node('ring_fix', 'ring_fix_node', 'ring_fix'),
        # 6. 扫码模块：UART3 读二维码任务码，校验锁存后发 /task_code(/decoded)
        car_node('qr_scanner', 'qr_scanner_node', 'qr_scanner'),
        # 7. 手柄驱动（USB 手柄 -> /joy）
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'autorepeat_rate': 20.0}],
        ),
        # 8. 手柄控制（/joy -> 整车控制话题：运动/使能/云台/舵机/补光灯）
        car_node('joy_control', 'joy_control_node', 'joy_control'),
        # 9. 状态屏（I2C OLED：三页可翻，按键 GPIO23）
        car_node('f407_bridge', 'status_display_node', 'status_display'),
    ])
