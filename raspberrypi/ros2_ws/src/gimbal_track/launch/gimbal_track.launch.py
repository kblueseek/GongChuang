# 视觉跟随云台 demo：物料识别 + 像素误差 PID 控制 yaw/extend。
#
# 依赖：先跑 run_car（= lidar_bringup car_only.launch.py，f407_bridge +
#       joy_control + 状态屏 + 运动规划都在里面），再另开一个终端起本 launch。
#       select 键（手柄）切换视觉接管。
#
# 用法（两个终端）：
#   # 终端 1：bash scripts/run_car.sh
#   # 终端 2：ros2 launch gimbal_track gimbal_track.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 1. 物料识别：单相机 -> /material_target
        Node(
            package='gimbal_track',
            executable='material_detect_node',
            name='material_detect',
            output='screen',
            parameters=[{
                'device': '',        # 留空自动试 /dev/video0..7；也可钉死 '/dev/video0'
                'width': 1280,
                'height': 720,
                'fps': 30.0,
                'rate_hz': 30.0,
                'target_color': 0,   # 0 = 六色任意，1~6 = 只认某一种（联调建议先指定纯色）
                'exclude_colors': '5',  # 黑色干扰大，默认排除；要黑料就改成 ''
                'processing_scale': 0.5,
                'open_kernel_px': 5,
                'close_kernel_px': 11,
                'min_area_px': 300.0,
                'max_area_px': 0.0,        # 0 = 不设上限（物料靠近变大不会再被误拒）
                'min_dimension_px': 10.0,
                'max_dimension_px': 0.0,   # 0 = 不设上限
                'max_aspect_ratio': 0.0,   # 0 = 不查长宽比（圆柱斜看会变长，别卡）
                'min_solidity': 0.0,       # 0 = 不查实心度
                'min_extent': 0.0,         # 0 = 不查占比
                'show_window': True,  # 实时预览窗口（需图形桌面；SSH 无屏就设 False）
                'debug_dump': True,  # 调参期开着：每 2 秒存 /tmp/gimbal_track/material.jpg
            }],
        ),
        # 2. 像素误差 PID -> /cmd_gimbal_vel
        Node(
            package='gimbal_track',
            executable='gimbal_track_node',
            name='gimbal_track',
            output='screen',
            parameters=[{
                'kp_yaw': 0.7,
                'kp_extend': 2.0,
                'max_yaw_deg_s': 240.0,
                'max_extend_deg_s': 600.0,
                'dir_yaw': -1.0,      # 发散（越跑越偏）就翻成 -1.0
                'dir_extend': -1.0,
                'deadband_px': 10.0,
                'rate_hz': 30.0,
            }],
        ),
    ])
