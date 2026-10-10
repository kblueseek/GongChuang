# launch 文件：用来"启动节点"的脚本。文件名后缀 .launch.py 表示它是 Python 写的 launch。
#
# 大白话：这个文件只启动一个东西——激光雷达驱动节点。
# 单独用它（ros2 launch lidar_bringup x3_pro.launch.py）可以只把雷达跑起来，
# 先确认雷达能不能出数据，再去启动整个系统。

from launch import LaunchDescription          # 启动描述：一个"要启动哪些东西"的清单
from launch_ros.actions import Node           # Node：清单里的一项 = 一个节点
from launch_ros.substitutions import FindPackageShare  # 用来找"某个包被安装到哪了"

def generate_launch_description():
    """launch 文件的入口函数：返回要启动的东西的清单。ROS2 会调用它。"""
    return LaunchDescription([
        # 一个节点 = 一个进程，这里就是雷达驱动
        Node(
            package='ydlidar_ros2_driver',          # 节点属于哪个包（第三方驱动）
            executable='ydlidar_ros2_driver_node',  # 要运行的可执行文件/命令名
            name='ydlidar_ros2_driver_node',        # 给这个节点起的名字（要和 yaml 顶层键一致！）
            output='screen',                        # 把日志打到当前终端，方便看
            # 参数：从 lidar_bringup 包的 config/x3_pro.yaml 加载
            # FindPackageShare('lidar_bringup') 会展开成 ".../install/lidar_bringup/share/lidar_bringup"，
            # 再拼上 '/config/x3_pro.yaml'，就找到参数文件了。
            parameters=[FindPackageShare('lidar_bringup') + '/config/x3_pro.yaml'],
        ),
    ])
