#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""串口屏（淘晶驰 TJC / T1）显示节点。

================ 大白话 ================

和 status_display_node（I2C OLED）干的是同一件事——把车上的状态显示出来，
只是输出设备换成了串口屏。它只订阅话题、往串口写字符串，不改任何控制逻辑。

显示内容（三个文本控件）：

    /task_code_decoded  → 任务码        默认 t0
    /motion_state       → 状态机在干什么  默认 t1
    /robot_status + /odom → 使能/在线/位置 默认 t2

================ 屏幕端要做的 ================

1. USART HMI 里新建工程（T1 3.5" = 480×320），拖三个「文本」控件，
   名字保持 t0 / t1 / t2（想改名就同步改本节点的 obj_* 参数）；
2. 工程设置里波特率设成 115200（和本节点 baudrate 一致）；
3. 字库编码和本节点 encoding 一致：用 UTF-8 就两边都 UTF-8，
   只有 GB2312 就两边都用 gbk；
4. 编译下载。

================ 接线 ================

    USB-TTL TX  → 屏 RX
    USB-TTL RX  → 屏 TX
    USB-TTL GND → 屏 GND   （必须共地）
    屏 5V 独立供电

⚠ 不要占用 /dev/ttyAMA0 —— 那是 F407 的链路。USB 转 TTL 是最省事的方案，
   插上就是 /dev/ttyUSB0。用 `ls /dev/serial/by-id/` 可以看稳定名字。

================ 协议 ================

每条指令都是「文本 + 0xFF 0xFF 0xFF」，UTF-8（或 gbk）编码：

    t0.txt="452+321+254+312"  FF FF FF

屏是慢速 MCU，连续发太快会丢，本节点做了两件事：
  · 只在内容变化时才发（配合 refresh_hz 节流）；
  · 每条指令之间留 20ms。
"""

from __future__ import annotations

import json
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32MultiArray, String

try:
    import serial                      # pyserial
except ImportError:                    # 没装也允许节点启动，只是不显示
    serial = None

TJC_END = b'\xff\xff\xff'              # 每条指令的结束符，少一个字节都不执行
CMD_GAP_S = 0.02                       # 两条指令之间的最小间隔
REOPEN_PERIOD_S = 2.0                  # 串口打不开时的重试间隔


class ScreenDisplayNode(Node):
    def __init__(self):
        super().__init__('screen_display')

        # ---- 串口 ----
        self.declare_parameter('device', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        # 必须和屏幕字库的编码一致：UTF-8 工程用 utf-8，GB2312 工程用 gbk
        self.declare_parameter('encoding', 'utf-8')

        # ---- 显示 ----
        self.declare_parameter('refresh_hz', 2.0)
        self.declare_parameter('obj_task', 't0')     # 任务码
        self.declare_parameter('obj_state', 't1')    # 状态机文字
        self.declare_parameter('obj_info', 't2')     # 使能/位置
        self.declare_parameter('max_chars', 40)      # 单个控件的最大字符数
        self.declare_parameter('page', 0)            # 启动时切到哪一页

        self._device = str(self.get_parameter('device').value)
        self._baud = int(self.get_parameter('baudrate').value)
        self._enc = str(self.get_parameter('encoding').value)
        self._max_chars = int(self.get_parameter('max_chars').value)
        self._obj_task = str(self.get_parameter('obj_task').value)
        self._obj_state = str(self.get_parameter('obj_state').value)
        self._obj_info = str(self.get_parameter('obj_info').value)

        # ---- 数据缓存 ----
        self._task_text = '任务码 --'
        self._state_text = '等待状态机'
        self._info_text = '离线'
        self._state = 0
        self._err = 0
        self._motor_mask = 0
        self._x = self._y = self._yaw = 0.0
        self._odom_s = 0.0
        self._f407_online = False
        self._f407_s = 0.0

        # ---- 串口状态 ----
        self._ser = None
        self._next_open_s = 0.0
        self._last_send = {}            # obj -> 上次发出的文本
        self._last_write_s = 0.0

        self.create_subscription(String, '/task_code_decoded',
                                 self._cb_task, 10)
        self.create_subscription(String, '/motion_state', self._cb_state, 10)
        self.create_subscription(Int32MultiArray, '/robot_status',
                                 self._cb_status, 10)
        self.create_subscription(Odometry, '/odom', self._cb_odom, 10)
        self.create_subscription(Bool, '/f407_online', self._cb_f407, 10)

        hz = max(0.2, float(self.get_parameter('refresh_hz').value))
        self.create_timer(1.0 / hz, self._tick)

        if serial is None:
            self.get_logger().error(
                '没装 pyserial：sudo apt install python3-serial。'
                '本节点照常运行，但不会往屏幕发任何东西')
        else:
            self.get_logger().info(
                f'串口屏：{self._device} @ {self._baud}，编码 {self._enc}，'
                f'控件 {self._obj_task}/{self._obj_state}/{self._obj_info}')

    # ==================== 订阅回调 ====================

    def _cb_task(self, msg: String):
        """任务码。优先用 JSON 里的 raw；解析失败就原样显示。"""
        raw = msg.data.strip()
        try:
            data = json.loads(raw)
        except Exception:               # noqa: BLE001
            self._task_text = f'任务码 {raw}'
            return
        code = str(data.get('raw', '')).strip()
        if not code:
            return
        self._task_text = f'任务码 {code}'

    def _cb_state(self, msg: String):
        self._state_text = msg.data.strip() or '空闲'

    def _cb_status(self, msg: Int32MultiArray):
        if len(msg.data) >= 3:
            self._state = int(msg.data[0])
            self._err = int(msg.data[1])
            self._motor_mask = int(msg.data[2])

    def _cb_odom(self, msg: Odometry):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._odom_s = time.monotonic()

    def _cb_f407(self, msg: Bool):
        self._f407_online = bool(msg.data)
        self._f407_s = time.monotonic()

    # ==================== 串口 ====================

    def _ensure_port(self, now: float) -> bool:
        """按需打开串口；打不开就每 2 秒重试一次，不阻塞别的节点。"""
        if self._ser is not None:
            return True
        if serial is None or now < self._next_open_s:
            return False
        self._next_open_s = now + REOPEN_PERIOD_S
        try:
            self._ser = serial.Serial(self._device, self._baud, timeout=0.1,
                                      bytesize=8, parity='N', stopbits=1)
            time.sleep(0.05)
            self._ser.reset_input_buffer()
            self._last_send.clear()      # 重连后整屏重刷一遍
            self.get_logger().info(f'串口屏已打开：{self._device}')
            self._send_direct(f'page {int(self.get_parameter("page").value)}')
            return True
        except Exception as error:       # noqa: BLE001
            self._ser = None
            self.get_logger().warning(
                f'串口屏打不开（{self._device}）：{error}，{REOPEN_PERIOD_S:.0f}s 后重试',
                throttle_duration_sec=10.0)
            return False

    def _close_port(self, reason: str = ''):
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:            # noqa: BLE001
                pass
            self._ser = None
            self._last_send.clear()
            self.get_logger().warning(f'串口屏连接断开{f"：{reason}" if reason else ""}')

    def _send_direct(self, command: str):
        """发一条原始指令（不带重发/去重逻辑）。"""
        if self._ser is None:
            return
        try:
            payload = command.encode(self._enc, errors='replace') + TJC_END
            self._ser.write(payload)
            self._ser.flush()
            self._last_write_s = time.monotonic()
        except Exception as error:       # noqa: BLE001
            self._close_port(str(error))

    def _pump(self):
        """把变化的字段推给屏幕。每条之间睡 CMD_GAP_S，屏幕才跟得上。

        只在内容变化时发，所以静止时一个字节都不写。本节点跑在独立进程里，
        这里睡几十毫秒不会影响 motion_planner / f407_bridge。
        """
        for obj, text in ((self._obj_task, self._task_text),
                          (self._obj_state, self._state_text),
                          (self._obj_info, self._info_line())):
            if not obj:
                continue
            text = text.replace('"', "'")[:self._max_chars]
            if self._last_send.get(obj) == text:
                continue
            self._send_direct(f'{obj}.txt="{text}"')
            self._last_send[obj] = text
            time.sleep(CMD_GAP_S)

    # ==================== 周期刷新 ====================

    def _info_line(self) -> str:
        now = time.monotonic()
        if self._f407_s and now - self._f407_s < 2.0 and not self._f407_online:
            return 'F407 离线'
        enable = '使能' if self._state == 1 else '失能'
        motors = bin(self._motor_mask & 0x7F).count('1')
        if self._odom_s and now - self._odom_s < 2.0:
            pos = f'({self._x:+.2f},{self._y:+.2f}) {math.degrees(self._yaw):+.0f}deg'
        else:
            pos = '无里程计'
        return f'{enable} 电机{motors}/7 {pos}'

    def _tick(self):
        now = time.monotonic()
        if not self._ensure_port(now):
            return
        self._pump()

    def destroy_node(self):
        self._close_port()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ScreenDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
