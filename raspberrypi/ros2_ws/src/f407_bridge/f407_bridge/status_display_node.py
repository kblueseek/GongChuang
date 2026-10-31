#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""状态屏 + 限位示教面板：I2C OLED（GPIO2=SDA, GPIO3=SCL）+ 4 个板载按键。

================ 大白话：这个节点在干什么 ================

小车上没有显示器，调试时想知道"F407 活着吗、车现在在哪、云台转到哪"，就得插电脑。
这个节点用一块 I2C 小屏把这些信息实时画出来，同时用板载的 4 个按键完成云台限位的
示教（把三个轴用手拨到想要的位置，按键记下来）。

  ── 显示（三页，按键 3 / GPIO23 翻页）──
    第 1 页 状态：F407/LINK/PAD、X/Y/朝向、整车状态、错误码
    第 2 页 云台：三轴当前位置（相对上电位置，单位 °）
    第 3 页 限位：三轴当前的软限位（来自示教结果）

  ── 开机限位示教（启动后自动进入）──
    1. 节点先让 F407 把云台三个电机"松轴"（电机控制板失能），并保持整车失能；
    2. 屏幕提示 "YAW -> MIN"，用手把 Yaw 轴拨到想要的最小点，按 按键3 记录；
    3. 提示 "YAW -> MAX"，拨到最大点，再按 按键3；
    4. 依次对 LIFT（Z 升降）、EXTEND（X 伸缩）做同样的事；
    5. 三轴完成后把限位写进树莓派文件，并下发给 F407；F407 立即用新限位。
    示教点就是"人手把轴停住的位置"，所以 min/max 天然和电机方向设置无关。

  按键（全部是上拉输入，按下为低电平）：
    按键1 GPIO17 ：跳过当前轴（保留上次保存的限位），进入下一轴
    按键2 GPIO22 ：当前轴重来（丢掉刚记的最小点，回到 MIN 步骤）
    按键3 GPIO23 ：记录当前点 / 翻页（非示教时）
    按键4 GPIO24 ：退出示教（已经记下的轴生效，剩下的用上次保存值）

================ 重要：限位坐标是"相对 F407 上电位置" ================
F407 在每轴上电后第一次收到编码器反馈时，把那一刻锁存成 0 点，限位就是相对这个 0 点
的角度。所以：
  · 每次 F407 重新上电，0 点都会变（变成上电时轴所在的位置）；
  · 树莓派保存的限位只在"上电姿态和上次一样"时还能用；
  · 正常用法就是每次开机都示教一遍（本节点默认自动进入），保存文件主要用于
    查看/跳过时兜底。
  按 按键4 跳过时，节点会把上次保存的限位发下去——如果这次上电时轴的位置和上次
  装车时不一样，这组限位就是错的，屏幕上会显示 "OLD?" 提醒。

================ 硬件与依赖 ================

  接线：VCC->3.3V  GND->GND  SDA->GPIO2(物理脚3)  SCL->GPIO3(物理脚5)
  使能 I2C：sudo raspi-config -> Interface Options -> I2C -> Yes
  确认地址：i2cdetect -y 1   （常见 0x3C）
  装库：sudo apt install python3-luma.oled python3-pil python3-smbus python3-gpiozero

常见 1.96 寸屏是 160x80 分辨率（SSD1315），默认参数按 128x64 画；
如果你的屏是 160x80，启动时加参数：
    -p width:=160 -p height:=80

================ 参数 ================

  i2c_port          1             总线号（树莓派默认 1）
  i2c_address       0x3C          I2C 地址
  width             128           屏幕宽（像素）
  height            64            屏幕高（像素）
  controller        ssd1306       控制器型号
  refresh_hz        5             屏幕刷新频率
  timeout_s         1.5           多久没收到回传算离线
  auto_setup        true          开机自动进入限位示教
  startup_delay_s   3.0           等 F407 上电、锁存 0 点后再开始示教
  min_span_deg      20.0          示教时最小/最大点至少要差这么多度
  limits_file       ~/.ros/gimbal_limits.json   限位保存位置
  button_enabled    true          是否启用板载按键
  button_confirm_pin 23 / button_skip_pin 17 / button_redo_pin 22 / button_exit_pin 24
  button_bounce_s   0.05          按键消抖
"""

import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32MultiArray, Int32MultiArray

# luma.oled 是树莓派上常用的 OLED 驱动库。没装也不让它把节点搞崩——只打日志、不显示。
try:
    import smbus2
    from luma.core.interface.serial import i2c
    from luma.oled.device import ssd1306, sh1106
    # 新版 luma.oled（3.x）没有独立的 ssd1315 模块。SSD1315 与 SSD1306 寄存器兼容，
    # 找不到就复用 ssd1306 —— 不能因为一个名字缺失就让整个 import 失败、误报"缺库"。
    try:
        from luma.oled.device import ssd1315
    except ImportError:
        ssd1315 = ssd1306
except ImportError:
    smbus2 = None
    i2c = None
    ssd1306 = ssd1315 = sh1106 = None

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:
    Image = ImageDraw = ImageFont = None

# gpiozero 负责读按键。没装时只是不能示教，显示功能不受影响。
try:
    from gpiozero import Button
except ImportError:
    Button = None

# 字体文件：优先等宽字体（数字对齐好看），找不到就用 PIL 自带的小字体
_FONT_CANDIDATES = (
    "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
)

# 状态编号 -> 文字（和 F407 的 ROBOT_DISABLED / ROBOT_READY 对应）
_STATE_TEXT = {0: "DISABLED", 1: "READY", 2: "FAULT"}

# 示教顺序：先 Yaw，再 Z 升降，最后 X 伸缩（下标和 F407 的轴编号一致）
_AXES = (("YAW", 0), ("LIFT", 1), ("EXT", 2))
_PAGE_COUNT = 3


class StatusDisplayNode(Node):
    def __init__(self):
        super().__init__('status_display')

        self.declare_parameter('i2c_port', 1)
        self.declare_parameter('i2c_address', 0x3C)
        self.declare_parameter('width', 128)
        self.declare_parameter('height', 64)
        self.declare_parameter('controller', 'ssd1306')
        self.declare_parameter('refresh_hz', 5.0)
        self.declare_parameter('timeout_s', 1.5)
        # ---- 限位示教 ----
        self.declare_parameter('auto_setup', True)
        self.declare_parameter('startup_delay_s', 3.0)
        self.declare_parameter('min_span_deg', 20.0)
        self.declare_parameter('limits_file', '~/.ros/gimbal_limits.json')
        self.declare_parameter('button_enabled', True)
        self.declare_parameter('button_confirm_pin', 23)   # 按键3
        self.declare_parameter('button_skip_pin', 17)      # 按键1
        self.declare_parameter('button_redo_pin', 22)      # 按键2
        self.declare_parameter('button_exit_pin', 24)      # 按键4
        self.declare_parameter('button_bounce_s', 0.05)

        self._width = int(self.get_parameter('width').value)
        self._height = int(self.get_parameter('height').value)
        self._timeout_s = float(self.get_parameter('timeout_s').value)
        self._auto_setup = bool(self.get_parameter('auto_setup').value)
        self._startup_delay_s = float(self.get_parameter('startup_delay_s').value)
        self._min_span_deg = float(self.get_parameter('min_span_deg').value)
        self._limits_file = os.path.expanduser(
            str(self.get_parameter('limits_file').value))
        self._page = 0

        # ================= 屏幕初始化（失败不影响节点其余逻辑） =================
        self._device = None
        self._font = None
        self._small = None
        if i2c is None or Image is None:
            self.get_logger().error(
                '缺少 luma.oled / Pillow / smbus2，屏幕不可用。请安装：'
                'sudo apt install python3-luma.oled python3-pil python3-smbus'
            )
        else:
            try:
                port = int(self.get_parameter('i2c_port').value)
                address = int(self.get_parameter('i2c_address').value)
                name = str(self.get_parameter('controller').value).lower()
                controller = {'ssd1306': ssd1306, 'ssd1315': ssd1315, 'sh1106': sh1106}.get(name)
                if controller is None:
                    raise ValueError(f'不认识的屏幕型号 {name}')
                # 自己开 smbus2 总线再传给 luma：这样 luma 走"非托管"模式，
                # 显示数据按 32 字节分块写。本机 RP1 的 I2C 单次 i2c_rdwr
                # 超过约 128 字节会回 EREMOTEIO，整页 128B 一次发必然失败。
                bus = smbus2.SMBus(port)
                try:
                    serial = i2c(bus=bus, address=address)
                    self._device = controller(serial, width=self._width, height=self._height)
                except Exception:
                    bus.close()
                    raise
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
        self._gimbal = [0.0, 0.0, 0.0]     # 云台三轴位置 °（相对上电位置）
        self._gimbal_s = 0.0

        # ================= 限位示教状态 =================
        # 'wait' 等 F407 上电锁存基准；'min'/'max' 示教中；'off' 正常
        self._setup_state = 'wait'
        self._setup_axis = 0               # 当前示教的轴 0=yaw 1=lift 2=extend
        self._setup_min = [None, None, None]
        self._limits = self._load_limits()  # 上次保存的 {轴: (min, max)}
        self._start_at = time.monotonic() + self._startup_delay_s
        self._release_off_left = 0         # 示教结束后还要补发几帧"恢复锁轴"
        self._last_limit_tx = 0.0
        self._last_msg = ''                # 屏幕底部临时提示

        # ================= 订阅（只读 + 云台位置） =================
        self.create_subscription(Odometry, '/odom', self._cb_odom, 10)
        self.create_subscription(Int32MultiArray, '/robot_status', self._cb_status, 10)
        self.create_subscription(Bool, '/f407_online', self._cb_f407, 10)
        self.create_subscription(Bool, '/ble_connected', self._cb_ble, 10)
        self.create_subscription(Joy, '/joy', self._cb_joy, 10)
        self.create_subscription(Float32MultiArray, '/gimbal_state', self._cb_gimbal, 10)

        # ================= 发布（示教用） =================
        self._pub_enable = self.create_publisher(Bool, '/cmd_enable', 10)
        self._pub_release = self.create_publisher(Bool, '/cmd_gimbal_release', 10)
        self._pub_limit = self.create_publisher(Float32MultiArray, '/cmd_gimbal_limit', 10)

        # ================= 按键 =================
        self._buttons = []
        self._buttons_ok = self._init_buttons()
        if not self._buttons_ok:
            if self._auto_setup:
                self.get_logger().warning(
                    '按键不可用，跳过开机限位示教（只显示）。检查 gpiozero 和按键接线。')
            self._setup_state = 'off'

        # 示教控制/限位重发 10Hz；屏幕刷新按 refresh_hz。
        self.create_timer(0.1, self._on_setup_tick)
        rate = max(0.2, float(self.get_parameter('refresh_hz').value))
        self.create_timer(1.0 / rate, self._tick)

        if self._setup_state == 'wait':
            self.get_logger().info(
                f'{self._startup_delay_s:.1f}s 后开始云台限位示教（依次 YAW -> LIFT -> EXT）。'
                '按 按键3 立即开始，按 按键4 跳过。')
        else:
            self.get_logger().info('状态屏就绪（限位示教已跳过）')

    # ---------------- 按键 ----------------

    def _init_buttons(self) -> bool:
        """初始化 4 个板载按键（上拉输入，按下为低）。任何一步失败都只降级，不崩节点。"""
        if Button is None:
            self.get_logger().error(
                '缺少 gpiozero，按键不可用。请安装：sudo apt install python3-gpiozero')
            return False
        if not bool(self.get_parameter('button_enabled').value):
            return False
        bounce = float(self.get_parameter('button_bounce_s').value)
        specs = (
            ('button_confirm_pin', self._on_confirm),
            ('button_skip_pin', self._on_skip),
            ('button_redo_pin', self._on_redo),
            ('button_exit_pin', self._on_exit),
        )
        try:
            for param, handler in specs:
                pin = int(self.get_parameter(param).value)
                button = Button(pin, pull_up=True, bounce_time=bounce)
                button.when_pressed = handler
                self._buttons.append(button)
        except Exception as error:
            self.get_logger().error(f'按键初始化失败：{error}（检查引脚是否被占用）')
            for b in self._buttons:
                try:
                    b.close()
                except Exception:
                    pass
            self._buttons = []
            return False
        self.get_logger().info(
            '按键就绪：P23=记录/翻页 P17=跳过本轴 P22=本轴重来 P24=退出示教')
        return True

    def _on_confirm(self):
        """按键3：示教中记录当前点；非示教时翻页。"""
        if self._setup_state == 'wait':
            self._begin_setup()
            return
        if self._setup_state == 'off':
            self._page = (self._page + 1) % _PAGE_COUNT
            return

        pos = self._current_pos()
        name = _AXES[self._setup_axis][0]
        if pos is None:
            self._say('没有云台位置数据，检查 F407 链路')
            return

        if self._setup_state == 'min':
            self._setup_min[self._setup_axis] = pos
            self._setup_state = 'max'
            self.get_logger().info(f'{name} 最小点记录：{pos:+.1f}°，现在拨到最大点再按按键3')
            self._say(f'{name} MIN 已记录')
        else:
            lo, hi = sorted((self._setup_min[self._setup_axis], pos))
            span = hi - lo
            if span < self._min_span_deg:
                self._say(f'行程太小 {span:.0f}° < {self._min_span_deg:.0f}°')
                self.get_logger().warning(
                    f'{name} 行程只有 {span:.1f}°，小于 {self._min_span_deg:.0f}°，'
                    '重新拨到两端再按（按键2 可重来本轴）')
                return
            self._limits[self._setup_axis] = (lo, hi)
            self.get_logger().info(f'{name} 限位记录：{lo:+.1f}° ~ {hi:+.1f}°（行程 {span:.1f}°）')
            self._advance()

    def _on_skip(self):
        """按键1：跳过当前轴，保留上次保存的限位。"""
        if self._setup_state not in ('min', 'max'):
            return
        name = _AXES[self._setup_axis][0]
        self.get_logger().info(f'{name} 跳过示教，沿用上次保存的限位')
        self._say(f'{name} SKIP')
        self._advance()

    def _on_redo(self):
        """按键2：当前轴重来。"""
        if self._setup_state not in ('min', 'max'):
            return
        name = _AXES[self._setup_axis][0]
        self._setup_min[self._setup_axis] = None
        self._setup_state = 'min'
        self.get_logger().info(f'{name} 重来：先拨到最小点再按按键3')
        self._say(f'{name} REDO')

    def _on_exit(self):
        """按键4：退出示教，已记录的轴生效，其余沿用上次保存值。"""
        if self._setup_state == 'wait':
            self._setup_state = 'off'
            self._start_at = 0.0
            self._publish_limits()
            self.get_logger().info('已跳过限位示教，使用上次保存的限位')
            return
        if self._setup_state in ('min', 'max'):
            self.get_logger().info('提前结束限位示教，已记录的轴生效')
            self._finish()

    # ---------------- 示教流程 ----------------

    def _begin_setup(self):
        self._setup_state = 'min'
        self._setup_axis = 0
        self._setup_min = [None, None, None]
        self._release_off_left = 0
        self.get_logger().info(
            '开始限位示教：云台松轴，整车保持失能。把 YAW 拨到最小点，按按键3 记录')

    def _advance(self):
        if self._setup_axis + 1 < len(_AXES):
            self._setup_axis += 1
            self._setup_state = 'min'
            self._setup_min[self._setup_axis] = None
            name = _AXES[self._setup_axis][0]
            self.get_logger().info(f'下一个：{name} —— 拨到最小点，按按键3 记录')
        else:
            self._finish()

    def _finish(self):
        self._setup_state = 'off'
        self._release_off_left = 5          # 连发几帧"恢复锁轴"，防丢一帧一直松着
        self._save_limits()
        self._publish_limits()
        done = ', '.join(
            f'{_AXES[i][0]} {lo:+.0f}~{hi:+.0f}'
            for i, (lo, hi) in sorted(self._limits.items()))
        self.get_logger().info(f'限位示教完成并已下发：{done or "（无，沿用 F407 默认）"}')

    def _on_setup_tick(self):
        now = time.monotonic()

        if self._setup_state == 'wait':
            # 示教前先保持整车失能，避免有人在这几秒里按了 start
            self._pub_enable.publish(Bool(data=False))
            if now >= self._start_at:
                if self._auto_setup:
                    self._begin_setup()
                else:
                    self._setup_state = 'off'
                    self._publish_limits()
                    self.get_logger().info('已关闭自动示教，使用上次保存的限位')
            return

        if self._setup_state in ('min', 'max'):
            # 持续保持"失能 + 松轴"：丢一帧也不会让轴突然锁死
            self._pub_enable.publish(Bool(data=False))
            self._pub_release.publish(Bool(data=True))
            return

        # 'off'：示教结束后补几帧恢复锁轴，之后 1Hz 重发限位（F407 重启也能补上）
        if self._release_off_left > 0:
            self._pub_release.publish(Bool(data=False))
            self._release_off_left -= 1
        if now - self._last_limit_tx >= 1.0:
            self._publish_limits()

    def _publish_limits(self):
        self._last_limit_tx = time.monotonic()
        for axis in range(len(_AXES)):
            if axis in self._limits:
                lo, hi = self._limits[axis]
                self._pub_limit.publish(
                    Float32MultiArray(data=[float(axis), float(lo), float(hi)]))

    def _current_pos(self):
        """当前示教轴的实时位置；数据过期（>0.5s）或从未收到时返回 None。"""
        if self._gimbal_s <= 0.0 or (time.monotonic() - self._gimbal_s) > 0.5:
            return None
        return self._gimbal[self._setup_axis]

    def _say(self, text: str):
        """在屏幕底部显示一条临时提示（几秒后自动消失）。"""
        self._last_msg = text
        self.get_logger().info(text)

    # ---------------- 限位文件 ----------------

    def _load_limits(self) -> dict:
        try:
            with open(self._limits_file, 'r', encoding='utf-8') as f:
                raw = json.load(f)
            limits = {}
            for key, pair in dict(raw.get('limits', {})).items():
                axis = int(key)
                lo, hi = float(pair[0]), float(pair[1])
                if axis in (0, 1, 2) and lo < hi:
                    limits[axis] = (lo, hi)
            if limits:
                self.get_logger().info(
                    f'载入上次保存的限位 {self._limits_file}：'
                    + ', '.join(f'{_AXES[i][0]} {lo:+.1f}~{hi:+.1f}'
                                for i, (lo, hi) in sorted(limits.items())))
            return limits
        except FileNotFoundError:
            return {}
        except Exception as error:
            self.get_logger().warning(f'限位文件读取失败（忽略）：{error}')
            return {}

    def _save_limits(self):
        data = {
            'updated': time.strftime('%Y-%m-%d %H:%M:%S'),
            'frame': 'relative to F407 power-on pose (deg)',
            'limits': {str(i): [lo, hi] for i, (lo, hi) in sorted(self._limits.items())},
        }
        try:
            os.makedirs(os.path.dirname(self._limits_file), exist_ok=True)
            with open(self._limits_file, 'w', encoding='utf-8') as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
            self.get_logger().info(f'限位已保存：{self._limits_file}')
        except Exception as error:
            self.get_logger().error(f'限位保存失败：{error}')

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

    def _cb_gimbal(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            self._gimbal = [float(msg.data[0]), float(msg.data[1]), float(msg.data[2])]
            self._gimbal_s = time.monotonic()

    # ---------------- 绘制 ----------------

    def _tick(self):
        if self._device is None:
            return
        try:
            image = Image.new('1', (self._width, self._height))
            if self._setup_state in ('min', 'max', 'wait'):
                self._draw_setup(image)
            else:
                self._draw_page(image)
            self._device.display(image)
        except Exception as error:
            self.get_logger().error(f'OLED 绘制失败：{error}', throttle_duration_sec=5.0)

    def _draw_page(self, image):
        draw = ImageDraw.Draw(image)
        lh = max(10, self._height // 6)     # 行高：128x64 -> 10px，160x80 -> 13px

        if self._page == 0:
            self._draw_status(draw, lh)
        elif self._page == 1:
            self._draw_gimbal(draw, lh)
        else:
            self._draw_limits(draw, lh)

        # 右下角页码
        draw.text((self._width - 26, self._height - lh), f'{self._page + 1}/{_PAGE_COUNT}',
                  fill='white', font=self._small)

    def _draw_status(self, draw, lh):
        now = self._now()
        # 离线判定：超过 timeout_s 没更新就当断线（节点可能根本没起来）
        f407_ok = self._f407_online and (now - self._f407_online_s) < self._timeout_s
        ble_ok = self._ble and (now - self._ble_s) < self._timeout_s
        pad_ok = (now - self._joy_s) < 1.0
        odom_ok = (now - self._odom_s) < self._timeout_s

        draw.text((0, 0),
                  f'F407 {"ON " if f407_ok else "OFF"}'
                  f'  LINK {"OK" if ble_ok else "--"}'
                  f'  {"PAD" if pad_ok else "---"}',
                  fill='white', font=self._small)

        if odom_ok:
            draw.text((0, lh), f'X {self._x:+7.3f} m', fill='white', font=self._font)
            draw.text((0, 2 * lh), f'Y {self._y:+7.3f} m', fill='white', font=self._font)
            draw.text((0, 3 * lh), f'YAW {math.degrees(self._yaw):+7.1f} deg',
                      fill='white', font=self._font)
        else:
            draw.text((0, lh), 'waiting F407', fill='white', font=self._font)
            draw.text((0, 2 * lh), 'no odom', fill='white', font=self._font)

        draw.text((0, 4 * lh),
                  f'{_STATE_TEXT.get(self._state, "?")}  M{self._motor_mask:02X}',
                  fill='white', font=self._font)
        draw.text((0, 5 * lh), f'ERR 0x{self._err:02X}', fill='white', font=self._small)

    def _draw_gimbal(self, draw, lh):
        fresh = (self._gimbal_s > 0.0 and time.monotonic() - self._gimbal_s < self._timeout_s)
        draw.text((0, 0), 'GIMBAL rel deg', fill='white', font=self._small)
        for row, (name, axis) in enumerate(_AXES):
            if fresh:
                text = f'{name:<5}{self._gimbal[axis]:+8.1f}'
            else:
                text = f'{name:<5}     ---'
            draw.text((0, (row + 1) * lh), text, fill='white', font=self._font)

        have = ' '.join(_AXES[i][0] for i in sorted(self._limits))
        draw.text((0, 4 * lh), f'LIM: {have or "none"}', fill='white', font=self._small)
        draw.text((0, 5 * lh), 'P23 = next page', fill='white', font=self._small)

    def _draw_limits(self, draw, lh):
        draw.text((0, 0), 'SOFT LIMITS (deg)', fill='white', font=self._small)
        for row, (name, axis) in enumerate(_AXES):
            if axis in self._limits:
                lo, hi = self._limits[axis]
                text = f'{name:<5}{lo:+8.1f}{hi:+8.1f}'
            else:
                text = f'{name:<5}{"F407 default":>17}'
            draw.text((0, (row + 1) * lh), text, fill='white', font=self._small)
        draw.text((0, 4 * lh),
                  'OLD? re-teach if pose changed',
                  fill='white', font=self._small)
        draw.text((0, 5 * lh), self._last_msg or 'P23 = next page',
                  fill='white', font=self._small)

    def _draw_setup(self, image):
        draw = ImageDraw.Draw(image)
        lh = max(10, self._height // 6)

        if self._setup_state == 'wait':
            draw.text((0, 0), 'LIMIT SETUP', fill='white', font=self._font)
            draw.text((0, lh), 'preparing...', fill='white', font=self._font)
            draw.text((0, 3 * lh), 'P23 = start now', fill='white', font=self._small)
            draw.text((0, 4 * lh), 'P24 = skip setup', fill='white', font=self._small)
            return

        name = _AXES[self._setup_axis][0]
        draw.text((0, 0), f'SETUP {self._setup_axis + 1}/{len(_AXES)}  {name}',
                  fill='white', font=self._small)
        draw.text((0, lh), '-> MIN point' if self._setup_state == 'min' else '-> MAX point',
                  fill='white', font=self._font)

        pos = self._current_pos()
        draw.text((0, 2 * lh),
                  f'POS {pos:+8.1f}' if pos is not None else 'POS    no data',
                  fill='white', font=self._font)

        min_val = self._setup_min[self._setup_axis]
        if self._setup_state == 'max' and min_val is not None:
            draw.text((0, 3 * lh), f'MIN {min_val:+8.1f}', fill='white', font=self._small)
        draw.text((0, 4 * lh),
                  'P23 = SET' if pos is not None else 'P23 = SET (no data)',
                  fill='white', font=self._small)
        draw.text((0, 5 * lh), self._last_msg or 'P17=SKIP P22=REDO P24=EXIT',
                  fill='white', font=self._small)

    def _load_font(self, size: int):
        for path in _FONT_CANDIDATES:
            try:
                return ImageFont.truetype(path, size)
            except Exception:
                continue
        return ImageFont.load_default()

    def destroy_node(self):
        for b in self._buttons:
            try:
                b.close()
            except Exception:
                pass
        self._buttons = []
        try:
            if self._device is not None:
                self._device.cleanup()
        except Exception:
            pass
        super().destroy_node()


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
