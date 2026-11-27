#!/usr/bin/env bash
# 只启动控制链路：f407_bridge（有线串口 USART6）+ 状态屏 + motion_planner + 手柄。
# 不含激光雷达/障碍检测。手柄插上就能用，不需要另开终端。
#
# 用法：
#   bash scripts/run_car.sh                     # 上位机也在树莓派上跑（同机，走默认组播）
#   bash scripts/run_car.sh 10.23.116.215       # 上位机在本机(PC)上跑，参数 = PC 的 IP
#
# 为什么后面这种要传 IP：跨网段/校园网时组播过不去，传了才能和上位机互相发现
# （见 scripts/_dds_peer.sh）。不传的话上位机收不到 /odom、指令也发不过来。
#
# 不用 set -u：ROS2 的 setup.bash 会引用未定义的 AMENT_TRACE_SETUP_FILES，-u 会导致 source 失败
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-lyrical}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"

# 新 clone 下来（或 build/install 被清掉）时先给句人话，别让 source 报个
# "没有那个文件" 就没了。
if [ ! -f "$REPO_ROOT/ros2_ws/install/setup.bash" ]; then
    echo "[!] ros2_ws 还没构建过，先跑：" >&2
    echo "      cd $REPO_ROOT/ros2_ws && colcon build" >&2
    echo "      bash $REPO_ROOT/scripts/mk_libexec.sh" >&2
    exit 1
fi

# 可选：第一个参数是上位机(PC) 的 IP，传了就强制单播发现。
source "$REPO_ROOT/scripts/_dds_peer.sh"
setup_dds_peer "${1:-}"

# 修复 ament_python 可执行文件路径：本环境 colcon 装到 bin/，Lyrical 只在 lib/<pkg>/ 里找。
# 幂等，colcon build 之后 / ros2 launch 报 libexec 不存在时也靠它自愈。
bash "$REPO_ROOT/scripts/mk_libexec.sh"

source "$REPO_ROOT/ros2_ws/install/setup.bash"

exec ros2 launch car_bringup car_only.launch.py
