#!/usr/bin/env bash
# 树莓派（Ubuntu 26.04 + ROS2 Lyrical Luth）环境准备脚本。
# 用法：bash setup.sh    （脚本内部对需要 root 的命令使用 sudo）
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-lyrical}"

echo "==> 1/6 系统依赖"
sudo apt-get update
sudo apt-get install -y \
    python3-pip python3-serial python3-numpy python3-opencv \
    git curl wget cmake \
    python3-colcon-common-extensions

# 让当前用户无需 sudo 即可访问串口/USB
sudo usermod -aG dialout "$USER" || true

echo "==> 2/6 ROS2 ${ROS_DISTRO}"
if ! dpkg -s "ros-${ROS_DISTRO}-desktop" >/dev/null 2>&1; then
    echo "  [!] 未检测到 ROS2 ${ROS_DISTRO}，请先按官方文档安装桌面版："
    echo "      https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debs.html"
    echo "      （URL 中的 rolling 替换为 ${ROS_DISTRO}，对应 Ubuntu 26.04）"
    echo "      安装后重新运行本脚本。"
    exit 1
fi
echo "  ROS2 ${ROS_DISTRO} 已安装。"

echo "==> 3/6 串口（Pi <-> F407 USART6）"
# 使能主 UART；Ubuntu 下 config.txt 可能位于 /boot/firmware 或 /boot
CONFIG_TXT=/boot/firmware/config.txt
[ -f "$CONFIG_TXT" ] || CONFIG_TXT=/boot/config.txt
if [ -f "$CONFIG_TXT" ]; then
    grep -q '^enable_uart=1' "$CONFIG_TXT" 2>/dev/null || \
        echo 'enable_uart=1' | sudo tee -a "$CONFIG_TXT" >/dev/null
    # 释放与蓝牙复用的 UART（Pi 3/4 上需要；Pi 5 无此限制，加了也无害）
    grep -q '^dtoverlay=disable-bt' "$CONFIG_TXT" 2>/dev/null || \
        echo 'dtoverlay=disable-bt' | sudo tee -a "$CONFIG_TXT" >/dev/null
    echo "  已写入 ${CONFIG_TXT}"
else
    echo "  [!] 未找到 config.txt，请手动确认主 UART 已使能（enable_uart=1）。"
fi

echo "==> 4/6 YDLIDAR udev 规则（固定为 /dev/ydlidar）"
# 注意：如 lsusb 显示的 idVendor/idProduct 不同，请据实修改。
sudo bash -c 'cat > /etc/udev/rules.d/ydlidar.rules <<EOF
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", SYMLINK+="ydlidar"
EOF'
sudo udevadm control --reload-rules && sudo udevadm trigger

echo "==> 5/6 YDLIDAR ROS2 驱动（SDK + driver）"
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi
SRC_DIR="$REPO_ROOT/ros2_ws/src"
if [ ! -d "$SRC_DIR/ydlidar_ros2_driver" ]; then
    git clone --recursive https://github.com/YDLIDAR/ydlidar_ros2_driver.git "$SRC_DIR/ydlidar_ros2_driver"
else
    echo "  ydlidar_ros2_driver 已存在，跳过 clone。"
fi

echo "==> 6/6 构建 ROS2 工作区"
cd "$REPO_ROOT/ros2_ws"
colcon build --symlink-install

echo
echo "完成。请执行以下收尾："
echo "  1) sudo reboot            # 使串口/udev 配置生效"
echo "  2) 重新插拔激光雷达 USB，确认 ls -l /dev/ydlidar 存在"
echo "  3) bash scripts/bringup.sh"
