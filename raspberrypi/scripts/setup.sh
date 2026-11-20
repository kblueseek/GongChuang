#!/usr/bin/env bash
# 树莓派（Ubuntu 26.04 + ROS2 Lyrical Luth）环境准备脚本。
# 用法：bash setup.sh    （脚本内部对需要 root 的命令使用 sudo）
# 不用 set -u：ROS2 的 setup.bash 会引用未定义的 AMENT_TRACE_SETUP_FILES，-u 会导致 source 失败
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-lyrical}"

echo "==> 1/6 系统依赖"
sudo apt-get update
sudo apt-get install -y \
    python3-pip python3-serial python3-numpy python3-opencv \
    git curl wget cmake \
    python3-colcon-common-extensions

# 无线链路依赖（蓝牙上行 bleak / LoRa 串口 / I2C 屏）。
# 这几个包在不同发行版里名字/版本不一致，装不上就 pip 兜底，不能让整车环境准备整个失败。
sudo apt-get install -y python3-pil python3-smbus i2c-tools bluez || true
sudo apt-get install -y python3-bleak python3-luma.oled || {
    echo "  [!] apt 缺 python3-bleak / python3-luma.oled，改用 pip 安装"
    python3 -m pip install --break-system-packages bleak luma.oled || \
        echo "  [!] pip 也失败了：手动装 pip3 install bleak luma.oled（或只跑有线链路）"
}

# 让当前用户无需 sudo 即可访问串口/USB/I2C/蓝牙
for group in dialout i2c gpio bluetooth input; do
    getent group "$group" >/dev/null 2>&1 && sudo usermod -aG "$group" "$USER" || true
done

echo "==> 2/6 ROS2 ${ROS_DISTRO}"
if ! dpkg -s "ros-${ROS_DISTRO}-desktop" >/dev/null 2>&1; then
    echo "  [!] 未检测到 ROS2 ${ROS_DISTRO}，请先按官方文档安装桌面版："
    echo "      https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debs.html"
    echo "      （URL 中的 rolling 替换为 ${ROS_DISTRO}，对应 Ubuntu 26.04）"
    echo "      安装后重新运行本脚本。"
    exit 1
fi
echo "  ROS2 ${ROS_DISTRO} 已安装。"

echo "==> 2b/6 手柄驱动 joy 包（USB 手柄 -> /joy）"
sudo apt-get install -y "ros-${ROS_DISTRO}-joy"

echo "==> 3/6 无线链路（蓝牙上行 + LoRa 下行）与 I2C 状态屏"
# config.txt 在 Ubuntu/Raspberry Pi OS 上可能位于 /boot/firmware 或 /boot
CONFIG_TXT=/boot/firmware/config.txt
[ -f "$CONFIG_TXT" ] || CONFIG_TXT=/boot/config.txt
if [ -f "$CONFIG_TXT" ]; then
    # 上行要用树莓派自己的蓝牙，旧脚本写的 disable-bt 必须去掉
    if grep -qxF 'dtoverlay=disable-bt' "$CONFIG_TXT"; then
        sudo sed -i 's/^dtoverlay=disable-bt/# dtoverlay=disable-bt  # 已停用：上行要用树莓派蓝牙/' "$CONFIG_TXT"
        echo "  已注释掉旧的 dtoverlay=disable-bt"
    fi

    # 有线串口保留（备用/调试）
    grep -q '^enable_uart=1' "$CONFIG_TXT" 2>/dev/null || \
        echo 'enable_uart=1' | sudo tee -a "$CONFIG_TXT" >/dev/null

    # LoRa 接 GPIO4/5 -> 打开 UART2（Pi5 与 Pi4 及以下的 overlay 名字不同）
    MODEL="$(tr -d '\0' < /proc/device-tree/model 2>/dev/null || echo '')"
    if echo "$MODEL" | grep -q 'Raspberry Pi 5'; then
        UART_OVERLAY='dtoverlay=uart2-pi5'
    else
        UART_OVERLAY='dtoverlay=uart2'
    fi
    grep -q "^${UART_OVERLAY}$" "$CONFIG_TXT" 2>/dev/null || \
        echo "$UART_OVERLAY" | sudo tee -a "$CONFIG_TXT" >/dev/null

    # 状态屏 I2C（GPIO2=SDA, GPIO3=SCL）
    grep -q '^dtparam=i2c_arm=on' "$CONFIG_TXT" 2>/dev/null || \
        echo 'dtparam=i2c_arm=on' | sudo tee -a "$CONFIG_TXT" >/dev/null

    echo "  已写入 ${CONFIG_TXT}（${UART_OVERLAY}）"
else
    echo "  [!] 未找到 config.txt，请手动使能 UART2、I2C，并删除 dtoverlay=disable-bt。"
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
