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
    echo "    如果是在 SSH 里跑的：请改成在树莓派桌面/VNC 里执行，或 ssh -X 登录后再跑。" >&2
    exit 1
fi

cd "$REPO_ROOT/上位机"
exec python3 motion_gui.py
