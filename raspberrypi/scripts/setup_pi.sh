#!/usr/bin/env bash
# 树莓派一次性环境准备（需要 sudo 运行）：
#   1) 装 g++（car_interfaces 这个 ROS2 自定义消息包要用 C++ 编译器构建；以后雷达 C++ 驱动也要用）
#   2) 装 python3-tk（上位机 tkinter GUI 需要）
#   3) 把当前用户加进 dialout 组（免 sudo 访问串口 /dev/serial0）
#   4) 打开 UART 并禁用蓝牙，把可靠串口(PL011)映射到 GPIO14/15（连 F407 的那两个脚）
#
# 用法：sudo bash scripts/setup_pi.sh
# 跑完后需要 sudo reboot 让 UART 配置和 dialout 组生效。

set -euo pipefail

[ "$(id -u)" -eq 0 ] || { echo "请用 sudo 运行：sudo bash $0" >&2; exit 1; }

# 真实用户名（sudo 运行时 $SUDO_USER 是执行 sudo 的人）
REAL_USER="${SUDO_USER:-$(logname 2>/dev/null || echo '')}"

echo "==> [1/4] 安装 g++ / build-essential / python3-tk"
apt-get update
apt-get install -y build-essential python3-tk

echo "==> [2/4] 把用户加入 dialout 组"
if [ -n "$REAL_USER" ]; then
    usermod -aG dialout "$REAL_USER"
    echo "    已将 $REAL_USER 加入 dialout 组（重新登录后生效）"
else
    echo "    无法确定用户名，请手动执行：sudo usermod -aG dialout <你的用户名>" >&2
fi

echo "==> [3/4] 打开 UART、禁用蓝牙（config.txt）"
CONFIG=/boot/firmware/config.txt
[ -f "$CONFIG" ] || CONFIG=/boot/config.txt
[ -f "$CONFIG" ] || { echo "    找不到 config.txt" >&2; exit 1; }

append_line() {
    local line="$1"
    grep -qxF "$line" "$CONFIG" || echo "$line" >> "$CONFIG"
}
append_line 'enable_uart=1'
append_line 'dtoverlay=disable-bt'

echo "==> [4/4] 完成。接下来请执行：sudo reboot"
echo "    重启后 /dev/serial0 会指向可靠串口 ttyAMA0（GPIO14=TX, GPIO15=RX）。"
