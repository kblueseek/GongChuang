#!/usr/bin/env bash
# 跑上位机 GUI。同一个脚本，树莓派和本机（PC）都能用。
#
#   树莓派上（GUI 和 ROS 节点在同一台机器）：
#       bash scripts/run_gui.sh
#   本机上（ROS 节点还在树莓派上）：
#       bash scripts/run_gui.sh 10.23.113.193      ← 参数 = 树莓派的 IP
#
# 为什么要传 IP：默认靠组播发现，跨网段/校园网时组播过不去，
# 传了就让两边用单播直连（见 scripts/_dds_peer.sh）。同机运行不用传。
#
# 前提：
#   · 上位机只依赖 car_interfaces 这一个自定义包，没构建过会自动构建；
#   · 树莓派那边要先跑 bash scripts/run_car.sh。
# 不用 set -u：ROS2 的 setup.bash 会引用未定义的 AMENT_TRACE_SETUP_FILES，-u 会导致 source 失败
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-lyrical}"

# 两边必须一致；树莓派上默认是 0，除非显式设了别的值。
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"

# 本机可能只构建过 car_interfaces（不需要树莓派上那堆节点包），
# 没构建过就自动构建，省得 clone 下来第一次跑就报 import 错。
if [ ! -f "$REPO_ROOT/ros2_ws/install/setup.bash" ]; then
    echo "[i] 工作空间还没构建过，先构建 car_interfaces ……"
    ( cd "$REPO_ROOT/ros2_ws" && colcon build --packages-select car_interfaces )
fi
source "$REPO_ROOT/ros2_ws/install/setup.bash"

# 可选：第一个参数是树莓派 IP，传了就强制单播发现。
source "$REPO_ROOT/scripts/_dds_peer.sh"
setup_dds_peer "${1:-}"

# 没有 DISPLAY 时默认用 :0（树莓派本地桌面 / VNC 常见）。多显示器时按需改。
if [ -z "${DISPLAY:-}" ]; then
    export DISPLAY=:0
fi

# 树莓派的桌面是 GNOME on Wayland，X 由 Xwayland 提供，用的是会话私有的认证文件
# （/run/user/<uid>/.mutter-Xwaylandauth.XXXXXX，后缀每次登录都变）。
# 只设 DISPLAY 不设 XAUTHORITY 会报 "Authorization required, but no authorization protocol specified"。
if [ -z "${XAUTHORITY:-}" ]; then
    if [ -f "$HOME/.Xauthority" ]; then
        export XAUTHORITY="$HOME/.Xauthority"
    else
        for _f in /run/user/"$(id -u)"/.mutter-Xwaylandauth.*; do
            if [ -f "$_f" ]; then
                export XAUTHORITY="$_f"
                break
            fi
        done
    fi
fi

# 连不上就明确说清楚，别让 tkinter 甩一屏 traceback。
# （通过 SSH 敲命令时没有图形环境，要在桌面/VNC 里跑，或者 ssh -X 转发。）
if command -v xdpyinfo >/dev/null 2>&1 && ! xdpyinfo >/dev/null 2>&1; then
    echo "[!] 连不上图形界面：DISPLAY=${DISPLAY}  XAUTHORITY=${XAUTHORITY:-未设置}" >&2
    echo "    如果是在 SSH 里跑的：请改成在桌面/VNC 里执行，或 ssh -X 登录后再跑。" >&2
    exit 1
fi

# 快速自检：发现是异步的，要等几秒才认全；看不到不影响启动，只是提醒网络可能没通。
echo "[i] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}，正在查找其他节点（最多 6 秒，仅供自检）……"
NODES="$(timeout 6 ros2 node list 2>/dev/null | sort -u || true)"
if [ -n "$NODES" ]; then
    echo "[i] 已发现的节点："
    echo "$NODES" | sed 's/^/     /'
else
    echo "[!] 暂时没发现任何节点。检查：树莓派 run_car.sh 起来了吗？两台机器同一个 WiFi 吗？"
    echo "    防火墙：sudo ufw allow 7400:7500/udp"
fi

cd "$REPO_ROOT/上位机"
exec python3 motion_gui.py
