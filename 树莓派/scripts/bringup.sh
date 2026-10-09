#!/usr/bin/env bash
# 一键启动整车 ROS2 侧：激光雷达 + 障碍检测 + F407 桥接 + 运动规划。
# 用法：bash scripts/bringup.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-lyrical}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "$REPO_ROOT/ros2_ws/install/setup.bash"

exec ros2 launch lidar_bringup system.launch.py
