#!/usr/bin/env bash
# 只启动控制链路（f407_bridge + motion_planner），不含雷达/视觉。
# 用法：bash scripts/run_car.sh
# 不用 set -u：ROS2 的 setup.bash 会引用未定义的 AMENT_TRACE_SETUP_FILES，-u 会导致 source 失败
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-lyrical}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"

# 修复 ament_python 可执行文件路径：本环境 colcon 装到 bin/，Lyrical 只在 lib/<pkg>/ 里找。
# 幂等，colcon build 之后 / ros2 launch 报 libexec 不存在时也靠它自愈。
bash "$REPO_ROOT/scripts/mk_libexec.sh"

source "$REPO_ROOT/ros2_ws/install/setup.bash"

exec ros2 launch lidar_bringup car_only.launch.py
