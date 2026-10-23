#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""状态屏节点：把整车状态画到树莓派上的 I2C OLED（GPIO2=SDA, GPIO3=SCL）。

================ 大白话：这个节点在干什么 ================

小车上没有显示器，调试时想知道"F407 活着吗、车现在在哪、朝向多少"，就得插电脑。
这个节点用一块 I2C 小屏把这些信息实时画出来，抬头就能看：

    F407 ONLINE   BLE OK PAD
    X  +1.234 m
    Y  -0.056 m
    YAW +45.3 deg
    STATE READY  MOTOR 7F

数据都来自别的节点发布的话题（/odom、/robot_status、/f407_online、/ble_connected、/joy），
本节点只读不写，不参与控制，屏幕挂了也不影响开车。

================ 硬件与依赖 ================

  接线：VCC->3.3V  GND->GND  SDA->GPIO2(物理脚3)  SCL->GPIO3(物理脚5)
  使能 I2C：sudo raspi-config -> Interface Options -> I2C -> Yes（或 raspi-config nonint do_i2c 0）
  确认地址：i2cdetect -y 1   （常见 0x3C）
  装库：sudo apt install python3-luma.oled python3-pil i2c-tools

常见 1.96 寸屏是 160x80 分辨率（SSD1315），默认参数按 128x64 画；
如果你的屏是 160x80，启动时加参数：
    -p width:=160 -p height:=80
屏幕型号不对时改 controller：ssd1306 / ssd1315 / sh1106。

================ 参数 ================

  i2c_port    1            总线号（树莓派默认 1）
  i2c_address 0x3C         I2C 地址
  width       128          屏幕宽（像素）
  height      64           屏幕高（像素）
  controller  ssd1306      控制器型号
  refresh_hz  2            刷新频率
  timeout_s   1.5          多久没收到回传算离线
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Int32MultiArray

# luma.oled 是树莓派上常用的 OLED 驱动库。没装也不让它把节点搞崩——只打日志、不显示。
try:
    from luma.core.interface.serial import i2c
    from luma.oled.device import ssd1306, ssd1315, sh1106
except ImportError:
    i2c = None
    ssd1306 = ssd1315 = sh1106 = None

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:
    Image = ImageDraw = ImageFont = None

# 字体文件：优先等宽字体（数字对齐好看），找不到就用 PIL 自带的小字体
_FONT_CANDIDATES = (
    "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
)

# 状态编号 -> 文字（和 F407 的 ROBOT_DISABLED / ROBOT_READY 对应）
_STATE_TEXT = {0: "DISABLED", 1: "READY", 2: "FAULT"}


class StatusDisplayNode(Node):
    def __init__(self):
        super().__init__('status_display')

        self.declare_parameter('i2c_port', 1)
        self.declare_parameter('i2c_address', 0x3C)
        self.declare_parameter('width', 128)
        self.declare_parameter('height', 64)
        self.declare_parameter('controller', 'ssd1306')
        self.declare_parameter('refresh_hz', 2.0)
        self.declare_parameter('timeout_s', 1.5)

        self._width = int(self.get_parameter('width').value)
        self._height = int(self.get_parameter('height').value)
        self._timeout_s = float(self.get_parameter('timeout_s').value)

        # ================= 屏幕初始化（失败不影响节点其余逻辑） =================
        self._device = None
        self._font = None
        self._small = None
        if i2c is None or Image is None:
            self.get_logger().error(
                '缺少 luma.oled / Pillow，屏幕不可用。请安装：'
                'sudo apt install python3-luma.oled python3-pil'
            )
        else:
            try:
                port = int(self.get_parameter('i2c_port').value)
                address = int(self.get_parameter('i2c_address').value)
                name = str(self.get_parameter('controller').value).lower()
                controller = {'ssd1306': ssd1306, 'ssd1315': ssd1315, 'sh1106': sh1106}.get(name)
                if controller is None:
                    raise ValueError(f'不认识的屏幕型号 {name}')
                serial = i2c(port=port, address=address)
                self._device = controller(serial, width=self._width, height=self._height)
                self._font = self._load_font(max(9, self._height // 6))
                self._small = self._load_font(max(7, self._height // 8))
                self.get_logger().info(
                    f'OLED 就绪：i2c-{port} 0x{address:02X} {self._width}x{self._height} {name}'
                )
            except Exception as error:
                self._device = None
                self.get_logger().error(
                    f'OLED 初始化失败：{error}（检查接线 / i2cdetect -y 1 是否有 0x3C）'
                )

        # ================= 状态缓存 =================
        self._x = self._y = self._yaw = 0.0
        self._odom_s = 0.0                 # 最近一次收到里程计的时间
        self._state = 0
        self._err = 0
        self._motor_mask = 0
        self._f407_online = False
        self._f407_online_s = 0.0
        self._ble = False
        self._ble_s = 0.0
        self._joy_s = 0.0                  # 最近一次收到手柄数据的时间（判断手柄在不在）

        # ================= 订阅（只读） =================
        self.create_subscription(Odometry, '/odom', self._cb_odom, 10)
        self.create_subscription(Int32MultiArray, '/robot_status', self._cb_status, 10)
        self.create_subscription(Bool, '/f407_online', self._cb_f407, 10)
        self.create_subscription(Bool, '/ble_connected', self._cb_ble, 10)
        self.create_subscription(Joy, '/joy', self._cb_joy, 10)

        rate = max(0.2, float(self.get_parameter('refresh_hz').value))
        self.create_timer(1.0 / rate, self._tick)

    # ---------------- 回调 ----------------

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _cb_odom(self, msg: Odometry):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # 四元数 -> 偏航角：绕 Z 轴只有旋转时，yaw = atan2(2(wz+xy), 1-2(y²+z²))
        self._yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._odom_s = self._now()

    def _cb_status(self, msg: Int32MultiArray):
        if len(msg.data) >= 3:
            self._state = int(msg.data[0])
            self._err = int(msg.data[1])
            self._motor_mask = int(msg.data[2])

    def _cb_f407(self, msg: Bool):
        self._f407_online = bool(msg.data)
        self._f407_online_s = self._now()

    def _cb_ble(self, msg: Bool):
        self._ble = bool(msg.data)
        self._ble_s = self._now()

    def _cb_joy(self, msg: Joy):
        self._joy_s = self._now()

    # ---------------- 绘制 ----------------

    def _tick(self):
        if self._device is None:
            return
        try:
            image = Image.new('1', (self._width, self._height))
            self._draw(image)
            self._device.display(image)
        except Exception as error:
            self.get_logger().error(f'OLED 绘制失败：{error}', throttle_duration_sec=5.0)

    def _draw(self, image):
        draw = ImageDraw.Draw(image)
        now = self._now()

        # 离线判定：超过 timeout_s 没更新就当断线（节点可能根本没起来）
        f407_ok = self._f407_online and (now - self._f407_online_s) < self._timeout_s
        ble_ok = self._ble and (now - self._ble_s) < self._timeout_s
        pad_ok = (now - self._joy_s) < 1.0
        odom_ok = (now - self._odom_s) < self._timeout_s

        lh = max(10, self._height // 6)     # 行高：128x64 -> 10px，160x80 -> 13px

        # 第 1 行：链路状态
        draw.text((0, 0),
                  f'F407 {"ON " if f407_ok else "OFF"}'
                  f'  BLE {"OK" if ble_ok else "--"}'
                  f'  {"PAD" if pad_ok else "---"}',
                  fill='white', font=self._small)

        # 第 2、3 行：位置（毫米->米，带符号，方便看是不是在往回走）
        if odom_ok:
            draw.text((0, lh), f'X {self._x:+7.3f} m', fill='white', font=self._font)
            draw.text((0, 2 * lh), f'Y {self._y:+7.3f} m', fill='white', font=self._font)
            # 第 4 行：朝向（弧度->度，方便和手柄/规划对上）
            draw.text((0, 3 * lh), f'YAW {math.degrees(self._yaw):+7.1f} deg',
                      fill='white', font=self._font)
        else:
            draw.text((0, lh), 'waiting F407', fill='white', font=self._font)
            draw.text((0, 2 * lh), 'no odom', fill='white', font=self._font)

        # 第 5 行：整车状态 + 电机在线掩码
        draw.text((0, 4 * lh),
                  f'{_STATE_TEXT.get(self._state, "?")}  M{self._motor_mask:02X}',
                  fill='white', font=self._font)

        # 第 6 行：错误码（0 就是没错误）
        draw.text((0, 5 * lh), f'ERR 0x{self._err:02X}', fill='white', font=self._small)

    def _load_font(self, size: int):
        for path in _FONT_CANDIDATES:
            try:
                return ImageFont.truetype(path, size)
            except Exception:
                continue
        return ImageFont.load_default()


def main(args=None):
    rclpy.init(args=args)
    node = StatusDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
