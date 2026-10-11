#!/usr/bin/env bash
# 在树莓派本机跑上位机 GUI（需要图形环境：接显示器或 VNC）。
# 用法：bash scripts/run_gui.sh
# 前提：先在一个终端跑 scripts/run_car.sh 把 f407_bridge + motion_planner 拉起来。
# 不用 set -u：ROS2 的 setup.bash 会引用未定义的 AMENT_TRACE_SETUP_FILES，-u 会导致 source 失败
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-lyrical}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "$REPO_ROOT/ros2_ws/install/setup.bash"

# 没有 DISPLAY 时默认用 :0（树莓派本地桌面 / VNC 常见）。多显示器时按需改。
if [ -z "${DISPLAY:-}" ]; then
    export DISPLAY=:0
fi

cd "$REPO_ROOT/上位机"
exec python3 motion_gui.py
