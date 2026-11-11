# 只启动「F407 桥接 + 状态屏 + 运动规划 + 圆环矫正 + 扫码 + 手柄」，不含激光雷达/障碍检测。
#
# 用法：bash scripts/run_car.sh   （= ros2 launch car_bringup car_only.launch.py）
#
# 所有节点的可调参数集中在 config/vehicle.yaml（对标 F407 的 RobotConfig.h），
# 本文件只列节点清单。改参数改 vehicle.yaml，不要在这里加 parameters。

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# 整车集中配置：每个节点只应用与自己名字匹配的那一节
VEHICLE_CFG = PathJoinSubstitution([FindPackageShare('car_bringup'), 'config', 'vehicle.yaml'])


def car_node(package, executable, name, **kwargs):
    return Node(package=package, executable=executable, name=name,
                output='screen', parameters=[VEHICLE_CFG], **kwargs)


def generate_launch_description():
    return LaunchDescription([
        # 1. F407 桥接：有线串口（上行 + 下行共用一条全双工 USART6）
        car_node('f407_bridge', 'f407_bridge_node', 'f407_bridge'),
        # 2. 状态屏：I2C OLED（GPIO2/3），按键 GPIO23 翻页；TEACHING_ENABLED 开关
        #    在 status_display_node.py 顶部（默认关闭示教）
        car_node('f407_bridge', 'status_display_node', 'status_display'),
        # 3. 运动规划：任务状态机（/motion_load 下发序列，逐条执行、判断到位）
        car_node('motion_planner', 'motion_planner_node', 'motion_planner'),
        # 4. 圆环工位视觉矫正（calibrate 指令的执行者，触发时才开摄像头）
        car_node('ring_fix', 'ring_fix_node', 'ring_fix'),
        # 5. 扫码模块：UART3 读二维码任务码，校验锁存后发 /task_code(/decoded)
        car_node('qr_scanner', 'qr_scanner_node', 'qr_scanner'),
        # 6. 手柄驱动（USB 手柄 -> /joy，autorepeat 让摇杆按住时持续上报）
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'autorepeat_rate': 20.0}],
        ),
        # 7. 手柄控制（/joy -> /cmd_vel /cmd_enable /cmd_servo /cmd_gimbal /cmd_led）
        car_node('joy_control', 'joy_control_node', 'joy_control'),
    ])
