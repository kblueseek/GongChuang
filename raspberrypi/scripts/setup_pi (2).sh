#!/usr/bin/env bash
# 树莓派一次性环境准备（需要 sudo 运行）
#
# 这一版对应「树莓派 5 + 无线链路」：
#   上行 树莓派 --蓝牙--> F407        （需要树莓派自己的蓝牙，所以**不能再 disable-bt**）
#   下行 F407  --LoRa--> 树莓派      （LoRa 接 GPIO4/5，需要打开 uart2）
#   状态屏 I2C OLED 接 GPIO2/3       （需要打开 I2C）
#
# 做四件事：
#   1) 装依赖：Python 蓝牙库 bleak、OLED 驱动 luma.oled、串口/编译工具
#   2) 把当前用户加入 dialout / i2c / gpio / bluetooth 组
#   3) 改 config.txt：打开 UART/I2C，映射 GPIO4/5 为 UART2，并**移除 disable-bt**
#   4) 提示重启
#
# 用法：sudo bash scripts/setup_pi.sh
# 跑完后必须 sudo reboot，改动才生效。

set -euo pipefail

[ "$(id -u)" -eq 0 ] || { echo "请用 sudo 运行：sudo bash $0" >&2; exit 1; }

# 真实用户名（sudo 运行时 $SUDO_USER 是执行 sudo 的人）
REAL_USER="${SUDO_USER:-$(logname 2>/dev/null || echo '')}"

echo "==> [1/4] 安装依赖"
apt-get update
apt-get install -y build-essential python3-tk python3-serial python3-pil python3-smbus i2c-tools bluez

# 这两个包在不同发行版里名字/版本不一致，装不上就 pip 兜底
apt-get install -y python3-bleak python3-luma.oled || {
    echo "    [!] apt 缺 python3-bleak / python3-luma.oled，改用 pip 安装"
    apt-get install -y python3-pip
    pip3 install --break-system-packages bleak luma.oled || \
        echo "    [!] pip 也失败了：先跑有线链路，之后再手动装这两个库"
}

# 手柄驱动：ROS2 的 joy 包（joy_node 读 /dev/input/js*，发 /joy 话题）
# 发行版名优先取 $ROS_DISTRO，否则看 /opt/ros 下装了哪个
ROS_DISTRO_NAME="${ROS_DISTRO:-$(ls /opt/ros 2>/dev/null | head -n1)}"
if [ -n "$ROS_DISTRO_NAME" ] && [ -d "/opt/ros/$ROS_DISTRO_NAME" ]; then
    apt-get install -y "ros-${ROS_DISTRO_NAME}-joy" \
        || echo "    [!] 装不上 ros-${ROS_DISTRO_NAME}-joy，手动 sudo apt install ros-<发行版>-joy"
else
    echo "    [!] 没找到 /opt/ros，跳过 joy 包（装好 ROS2 后再跑一次本脚本）"
fi

echo "==> [2/4] 把用户加入串口/I2C/蓝牙相关用户组"
if [ -n "$REAL_USER" ]; then
    for group in dialout i2c gpio bluetooth input; do
        getent group "$group" >/dev/null 2>&1 && usermod -aG "$group" "$REAL_USER"
    done
    echo "    已将 $REAL_USER 加入 dialout / i2c / gpio / bluetooth / input（重新登录后生效）"
    echo "    其中 input 是为了让 joy_node 免 sudo 读 /dev/input/js0"
else
    echo "    无法确定用户名，请手动执行：sudo usermod -aG dialout,i2c,gpio,bluetooth <你的用户名>" >&2
fi

echo "==> [3/4] 修改 config.txt"
CONFIG=/boot/firmware/config.txt
[ -f "$CONFIG" ] || CONFIG=/boot/config.txt
[ -f "$CONFIG" ] || { echo "    找不到 config.txt" >&2; exit 1; }

append_line() {
    local line="$1"
    grep -qxF "$line" "$CONFIG" || echo "$line" >> "$CONFIG"
}

# 3.1 必须保留树莓派自己的蓝牙（上行靠它），把旧的 disable-bt 注释掉
if grep -qxF 'dtoverlay=disable-bt' "$CONFIG"; then
    sed -i 's/^dtoverlay=disable-bt/# dtoverlay=disable-bt  # 已停用：上行要用树莓派蓝牙/' "$CONFIG"
    echo "    已注释掉旧的 dtoverlay=disable-bt（否则树莓派蓝牙不可用）"
fi

# 3.2 有线串口（备用/调试），保留
append_line 'enable_uart=1'

# 3.3 LoRa 用的 UART2：Pi5 和 Pi4 及以下 overlay 名字不同
MODEL="$(tr -d '\0' < /proc/device-tree/model 2>/dev/null || echo '')"
if echo "$MODEL" | grep -q 'Raspberry Pi 5'; then
    append_line 'dtoverlay=uart2-pi5'
    echo "    检测到 $MODEL -> dtoverlay=uart2-pi5（GPIO4/5）"
else
    append_line 'dtoverlay=uart2'
    echo "    检测到 ${MODEL:-未知型号} -> dtoverlay=uart2（GPIO4/5）"
fi

# 3.4 I2C 状态屏
append_line 'dtparam=i2c_arm=on'

echo "==> [4/4] 完成。请执行 sudo reboot，然后按下面几步核对："
echo "    1) ls /dev/ttyAMA*            # 应能看到新增的 UART（LoRa 用哪个以这里为准）"
echo "    2) i2cdetect -y 1             # 应能看到 0x3C（OLED）"
echo "    3) bluetoothctl show          # Powered: yes，说明树莓派蓝牙正常"
echo "    4) 把 LoRa 的串口设备名填到 launch 里的 lora_device 参数"
