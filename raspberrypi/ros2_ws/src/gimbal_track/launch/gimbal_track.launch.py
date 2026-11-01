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
                # PID：先提 kp 到“快但刚开始抖”，再用 kd 压住抖动，然后 kp 还能再加
                # ki 消稳态误差，但像素有噪声，建议先用小值或保持 0
                'kp_yaw': 1.25,
                'ki_yaw': 0.010,
                'kd_yaw': 0.05,
                'kp_extend': 8.0,
                'ki_extend': 0.007,
                'kd_extend': 0.09,
                'deriv_tau': 0.05,     # 微分低通，0.03~0.1，噪声大就调大
                'max_yaw_deg_s': 240.0,
                'max_extend_deg_s': 600.0,
                'cam_rotation': -90.0,  # 摄像头旋转 90° 安装；越跟越偏就往 -90/180/270 试
                'dir_yaw': -1.0,      # 发散（越跑越偏）就翻成 -1.0
                'dir_extend': -1.0,
                'deadband_px': 10.0,
                'rate_hz': 30.0,
                # 下探（抓取）动作：视觉接管且目标静止（转盘停）后，lift 下降再抬起（位置闭环）
                'dip_enable': True,
                'dip_depth': 1000.0,       # 下降目标距离（°）
                'dip_down_vel_deg_s': 600.0,  # 下降速度
                'dip_up_vel_deg_s': 250.0,    # 上升速度（慢一点停得准）
                'dip_approach_deg': 40.0,     # 离起始高度这么近时开始减速
                'dip_creep_vel_deg_s': 50.0,  # 减速后的最低速度
                'dip_timeout_s': 5.0,         # 单段安全超时（反馈卡死兜底）
                'dip_dir': 1.0,            # 下降方向（±1，反了就翻）
                'dip_still_px': 8.0,       # 判定“静止”的像素抖动阈值
                'dip_still_time_s': 0.5,   # 静止持续多久才触发
                'dip_cooldown_s': 4.0,     # 两次下探最小间隔
                'dip_center_confirm_s': 0.3,  # 对中后稳住多久才下探
            }],
        ),
    ])
