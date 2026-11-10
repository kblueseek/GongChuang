#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""扫码模块节点：读取二维码任务码，校验、锁存并发布。

================ 任务码格式（智能+工程创新赛道） ================

任务码由 4 组三位数组成，用 + 连接，例如：

    452+321+254+312

    第 1 组：第一批物料的搬运颜色和顺序（3 位，每位 = 颜色编号）
    第 2 组：第一批物料在粗加工区/暂存区的放置位置（3 位，每位 = 圆环编号 1~3）
    第 3 组：第二批物料的搬运颜色和顺序
    第 4 组：第二批物料在粗加工区的放置位置

    颜色：红=1  黄=2  蓝=3  绿=4  黑=5  浅蓝=6
    圆环：粗加工区/暂存区各三个，从左到右编号 1,2,3

================ 串口约定 ================

扫码模块识别到二维码后把内容以 ASCII 经串口发出（一般带 \\r\\n 结尾），
"一直识别就一直发"。任务码在一场比赛里不变，所以本节点：

  · 收到任何数据先抠出 `数字+数字+数字+数字` 形态的候选串；
  · 按上面的编号范围严格校验（颜色位 1~6、位置位 1~3），不合法只记日志；
  · 校验通过即锁存：和上次相同不重复发布，**变化才重新发布**；
  · 以 1Hz 重发最后一次有效结果，方便后启动的订阅者。

================ 接线 / 参数 ================

默认串口 /dev/ttyAMA3（树莓派 UART3：TX=GPIO4 / RX=GPIO5）。
Pi5 需要在 /boot/firmware/config.txt 加 dtoverlay=uart3-pi5 后重启才有这个口；
模块如果是 USB 转的，把 device 参数改成 /dev/ttyUSB0 即可。波特率默认 115200，
模块若是 9600 就用启动参数覆盖：

    ros2 run qr_scanner qr_scanner_node --ros-args \
        -p device:=/dev/ttyAMA3 -p baudrate:=9600

发布话题：
  /task_code          （String）原始任务码，如 "452+321+254+312"
  /task_code_decoded  （String）JSON 解析结果：
    {"raw": "...", "batch1_colors": [4,5,2], "batch1_places": [3,2,1],
     "batch2_colors": [2,5,4], "batch2_places": [3,1,2],
     "color_names": ["绿","黑","浅蓝"], ...}
"""

from __future__ import annotations

import json
import re
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import serial
except ImportError as error:  # pragma: no cover
    raise RuntimeError('qr_scanner 需要 pyserial：sudo apt install python3-serial') from error

# 颜色编号 → 名字（浅蓝=6）
_COLOR_NAMES = {1: '红', 2: '黄', 3: '蓝', 4: '绿', 5: '黑', 6: '浅蓝'}

# 任务码形态：四组三位数，+ 连接。校验放在代码里做（颜色位 1~6、位置位 1~3），
# 这里先宽松抠出候选，便于模块回传带前后缀（如 "QR:452+321+254+312\r\n"）也能识别。
_TASK_CODE_RE = re.compile(r'(\d{3})\+(\d{3})\+(\d{3})\+(\d{3})')


def decode_task_code(raw: str) -> dict | None:
    """校验并解析任务码。合法返回 dict，非法返回 None。"""
    m = _TASK_CODE_RE.search(raw.strip())
    if not m:
        return None
    g1, g2, g3, g4 = m.groups()
    # 第 1/3 组是颜色（1~6），第 2/4 组是圆环位置（1~3）
    for ch in g1 + g3:
        if ch not in '123456':
            return None
    for ch in g2 + g4:
        if ch not in '123':
            return None
    b1c = [int(c) for c in g1]
    b2c = [int(c) for c in g3]
    return {
        'raw': f'{g1}+{g2}+{g3}+{g4}',
        'batch1_colors': b1c,            # 第一批搬运颜色顺序
        'batch1_places': [int(c) for c in g2],  # 第一批 粗加工/暂存 放置位
        'batch2_colors': b2c,
        'batch2_places': [int(c) for c in g4],
        'batch1_color_names': [_COLOR_NAMES[c] for c in b1c],
        'batch2_color_names': [_COLOR_NAMES[c] for c in b2c],
    }


class QrScannerNode(Node):
    def __init__(self):
        super().__init__('qr_scanner')

        self.declare_parameter('device', '/dev/ttyAMA3')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('retry_interval_s', 2.0)
        self.declare_parameter('republish_hz', 1.0)   # 锁存结果的周期重发（迟到的订阅者用）
        self.declare_parameter('rx_poll_hz', 20.0)    # 串口收数节拍
        self.declare_parameter('max_buffer', 512)     # 滚动缓冲区上限，防模块异常刷屏撑爆内存

        self._device = str(self.get_parameter('device').value)
        self._baud = int(self.get_parameter('baudrate').value)
        self._retry_s = float(self.get_parameter('retry_interval_s').value)
        self._max_buf = int(self.get_parameter('max_buffer').value)

        self._pub_code = self.create_publisher(String, '/task_code', 10)
        self._pub_decoded = self.create_publisher(String, '/task_code_decoded', 10)

        self._port = None
        self._buf = bytearray()
        self._next_retry = 0.0
        self._last_raw = None        # 已锁存的原始任务码
        self._last_decoded = None
        self._last_publish_t = 0.0

        self.create_timer(1.0 / max(1.0, float(self.get_parameter('rx_poll_hz').value)),
                          self._poll_serial)
        republish = max(0.2, float(self.get_parameter('republish_hz').value))
        self.create_timer(1.0 / republish, self._republish)
        self.get_logger().info(
            f'qr_scanner 就绪：{self._device} @ {self._baud}，等任务码…')

    # ---------------- 串口 ----------------

    def _ensure_port(self) -> bool:
        if self._port is not None:
            return True
        now = time.monotonic()
        if now < self._next_retry:
            return False
        self._next_retry = now + self._retry_s
        try:
            self._port = serial.Serial(self._device, self._baud, timeout=0)
            self._buf.clear()
            self.get_logger().info(f'扫码模块已连接 {self._device}')
        except Exception as error:  # noqa: BLE001 - 打不开就按间隔重试
            self.get_logger().warning(f'扫码模块打不开（{self._device}）：{error}')
        return self._port is not None

    def _poll_serial(self):
        if not self._ensure_port():
            return
        try:
            waiting = int(getattr(self._port, 'in_waiting', 0) or 0)
            if waiting > 0:
                data = self._port.read(waiting)
            else:
                data = b''
        except Exception as error:  # noqa: BLE001
            self.get_logger().warning(f'扫码模块读取失败：{error}，重连')
            try:
                self._port.close()
            except Exception:  # noqa: BLE001
                pass
            self._port = None
            return
        if not data:
            return
        self._buf.extend(data)
        if len(self._buf) > self._max_buf:
            del self._buf[:-self._max_buf]
        self._process_buffer()

    def _process_buffer(self):
        try:
            text = self._buf.decode('ascii', errors='ignore')
        except Exception:  # noqa: BLE001
            self._buf.clear()
            return
        for m in _TASK_CODE_RE.finditer(text):
            decoded = decode_task_code(m.group(0))
            if decoded is None:
                self.get_logger().warning(
                    f'收到形态像任务码但编号越界，已忽略：{m.group(0)}')
                continue
            if decoded['raw'] != self._last_raw:
                self._last_raw = decoded['raw']
                self._last_decoded = decoded
                self._publish(decoded)
                self.get_logger().info(
                    f'任务码锁定：{decoded["raw"]}  '
                    f'一批颜色{decoded["batch1_color_names"]} '
                    f'放{decoded["batch1_places"]} / '
                    f'二批颜色{decoded["batch2_color_names"]} '
                    f'放{decoded["batch2_places"]}')
        # 抠完候选就清缓冲（任务码短，残留在缓冲里没意义）
        self._buf.clear()

    # ---------------- 发布 ----------------

    def _publish(self, decoded: dict):
        self._last_publish_t = time.monotonic()
        self._pub_code.publish(String(data=decoded['raw']))
        self._pub_decoded.publish(String(data=json.dumps(
            decoded, ensure_ascii=False)))

    def _republish(self):
        """1Hz 重发最后一次有效结果：后启动的订阅者（motion_planner/上位机）
        不需要等下一次扫码就能拿到任务码。"""
        if self._last_decoded is None:
            return
        if time.monotonic() - self._last_publish_t >= 0.9:
            self._publish(self._last_decoded)

    def destroy_node(self):
        if self._port is not None:
            try:
                self._port.close()
            except Exception:  # noqa: BLE001
                pass
            self._port = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = QrScannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
