#!/usr/bin/env python3
"""树莓派 <-> F407 通信（PiComm 协议，USART6，115200 8N1）。

与 F407 侧 `UniversalCar/ExHardware/PiComm.c` 逐字节对齐，改动前务必两边同步。

帧格式：
    ┌──────┬──────┬─────┬─────┬─────┬─────────┬────────┐
    │ 0xAA │ 0x55 │ LEN │ SEQ │ CMD │  DATA   │ CRC16  │
    └──────┴──────┴─────┴─────┴─────┴─────────┴────────┘
    - LEN = CMD(1) + DATA(N) + CRC(2) = 3 + N
    - SEQ：发送序号，每帧 +1（0~255 回绕），仅用于观测，不影响解析
    - CRC16-MODBUS（poly 0xA001，init 0xFFFF），覆盖 LEN + SEQ + CMD + DATA
      （即 frame[2] 起共 3+N 字节），低字节在前。

命令（树莓派 -> F407）：
    CMD_VEL       0x01  vx vy wz（int16×3，单位 mm/s、mm/s、mrad/s，大端）
    CMD_GIMBAL    0x02  axis(u8: 0=yaw 1=lift 2=extend) + deg(int32×100，大端)
    CMD_HOME      0x03  mask(u8, bit0=yaw bit1=lift bit2=extend)
    CMD_GRIPPER   0x04  pos(u8, 0~180°)
    CMD_PLATE     0x05  pos(u8, 0~180°)
    CMD_CAM       0x06  pos(u8, 0~180°)
    CMD_ENABLE    0x07  en(u8) + mask(u8, bit0-6 = 电机1-7)
    CMD_ESTOP     0x08  无数据
    CMD_HEARTBEAT 0x10  无数据（1Hz）

遥测（F407 -> 树莓派）：
    RPT_ODOM   0x81  x(int32 mm) y(int32 mm) theta(int16 mrad)
    RPT_STATUS 0x83  state(u8) err(u8) motor_online_mask(u8)
"""

from __future__ import annotations

import struct
import time
from typing import Callable, Optional

try:
    import serial
except ImportError:  # 允许只 import 编解码部分而不装 pyserial
    serial = None


# ---------- 命令码（与 PiComm.h 一致） ----------
CMD_VEL = 0x01
CMD_GIMBAL = 0x02
CMD_HOME = 0x03
CMD_GRIPPER = 0x04
CMD_PLATE = 0x05
CMD_CAM = 0x06
CMD_ENABLE = 0x07
CMD_ESTOP = 0x08
CMD_HEARTBEAT = 0x10

# ---------- 遥测码 ----------
RPT_ODOM = 0x81
RPT_STATUS = 0x83

SOF1 = 0xAA
SOF2 = 0x55
MAX_FRAME = 64  # 与 F407 侧 PI_RX_BUF_SIZE 一致

# 错误标志位（与 ReceiveTask.c 一致）
ERR_ESTOP = 1 << 0
ERR_PI_OFFLINE = 1 << 1
ERR_MOTOR_LOST = 1 << 2

# 整车状态机（与 ReceiveTask.h 一致）
ROBOT_DISABLED = 0
ROBOT_READY = 1
ROBOT_FAULT = 2


class PiCommError(ValueError):
    """帧编解码或串口配置错误。"""


def crc16_modbus(data: bytes) -> int:
    """CRC16-MODBUS，poly 0xA001，init 0xFFFF。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else crc >> 1
    return crc & 0xFFFF


def build_frame(cmd: int, data: bytes = b"", seq: int = 0) -> bytes:
    """组一帧：AA 55 LEN SEQ CMD DATA CRC(lo hi)。"""
    if not 0 <= cmd <= 0xFF:
        raise PiCommError(f"功能码越界：{cmd}")
    n = len(data)
    if 3 + n > MAX_FRAME:
        raise PiCommError(f"帧过长：{3 + n} > {MAX_FRAME}")

    frame = bytearray()
    frame.append(SOF1)
    frame.append(SOF2)
    frame.append(3 + n)             # LEN
    frame.append(seq & 0xFF)        # SEQ
    frame.append(cmd)               # CMD
    frame.extend(data)              # DATA
    crc = crc16_modbus(bytes(frame[2:]))  # 覆盖 LEN+SEQ+CMD+DATA
    frame.append(crc & 0xFF)
    frame.append((crc >> 8) & 0xFF)
    return bytes(frame)


# ---------- 各命令的编码（返回 data 部分，单位见 docstring） ----------
def encode_vel(vx_m_s: float, vy_m_s: float, wz_rad_s: float) -> bytes:
    """速度：vx,vy 传 mm/s，wz 传 mrad/s，均 int16 大端。"""
    vx = int(round(vx_m_s * 1000.0))
    vy = int(round(vy_m_s * 1000.0))
    wz = int(round(wz_rad_s * 1000.0))
    for name, val in (("vx", vx), ("vy", vy), ("wz", wz)):
        if not -32768 <= val <= 32767:
            raise PiCommError(f"{name} 超出 int16：{val}")
    return struct.pack(">hhh", vx, vy, wz)


def encode_gimbal(axis: int, deg: float) -> bytes:
    """云台目标：axis(u8) + deg×100(int32 大端)。"""
    if axis not in (0, 1, 2):
        raise PiCommError(f"云台轴越界：{axis}")
    return struct.pack(">Bi", axis, int(round(deg * 100.0)))


def encode_home(mask: int) -> bytes:
    return bytes([mask & 0x07])


def encode_servo(pos_deg: float) -> bytes:
    if not 0.0 <= pos_deg <= 180.0:
        raise PiCommError(f"舵机角度越界：{pos_deg}")
    return bytes([int(round(pos_deg))])


def encode_enable(en: bool, mask: int = 0x7F) -> bytes:
    return bytes([1 if en else 0, mask & 0x7F])


# ---------- 遥测解析 ----------
def parse_odom(data: bytes) -> tuple[float, float, float]:
    """返回 (x_m, y_m, theta_rad)。帧数据为 int32 mm、int32 mm、int16 mrad。"""
    if len(data) != 10:
        raise PiCommError(f"里程计帧长度应为10，实际{len(data)}")
    x_mm, y_mm, theta_mrad = struct.unpack(">iih", data)
    return x_mm / 1000.0, y_mm / 1000.0, theta_mrad / 1000.0


def parse_status(data: bytes) -> tuple[int, int, int]:
    """返回 (state, err, motor_online_mask)。"""
    if len(data) != 3:
        raise PiCommError(f"状态帧长度应为3，实际{len(data)}")
    return data[0], data[1], data[2]


class PiCommLink:
    """维护一个可自动重连的非阻塞全双工串口，并做帧级收发。

    发送：send_* 系列；接收：poll() 返回 (cmd, data) 列表。
    """

    def __init__(
        self,
        device: str = "/dev/serial0",
        baudrate: int = 115200,
        reconnect_interval_s: float = 1.0,
        serial_factory: Callable = serial.Serial if serial else None,
        clock: Callable[[], float] = time.monotonic,
    ):
        if serial_factory is None:
            raise PiCommError("未安装 pyserial，无法创建串口")
        self.device = str(device)
        self.baudrate = int(baudrate)
        self.reconnect_interval_s = float(reconnect_interval_s)
        self._serial_factory = serial_factory
        self._clock = clock
        self._port = None
        self._rx = bytearray()
        self._seq = 0
        self._next_reconnect_s = 0.0
        self.last_error = ""

    # ---------- 连接管理 ----------
    @property
    def connected(self) -> bool:
        return bool(self._port is not None and getattr(self._port, "is_open", True))

    def _disconnect(self, error: str) -> None:
        if self._port is not None:
            try:
                self._port.close()
            except (OSError, Exception):
                pass
        self._port = None
        self._rx.clear()
        self._next_reconnect_s = self._clock() + self.reconnect_interval_s
        self.last_error = str(error)

    def _ensure_connected(self) -> bool:
        if self.connected:
            return True
        if self._clock() < self._next_reconnect_s:
            return False
        try:
            self._port = self._serial_factory(
                port=self.device,
                baudrate=self.baudrate,
                timeout=0,
                write_timeout=0,
            )
        except Exception as error:  # noqa: BLE001
            self._disconnect(str(error))
            return False
        self.last_error = ""
        return True

    # ---------- 发送 ----------
    def _send(self, cmd: int, data: bytes = b"") -> bool:
        if not self._ensure_connected():
            return False
        frame = build_frame(cmd, data, self._seq)
        try:
            written = self._port.write(frame)
            if written != len(frame):
                raise serial.SerialTimeoutException(
                    f"只写入 {written}/{len(frame)} 字节"
                )
        except Exception as error:  # noqa: BLE001
            self._disconnect(str(error))
            return False
        self._seq = (self._seq + 1) & 0xFF
        return True

    def send_vel(self, vx_m_s: float, vy_m_s: float, wz_rad_s: float) -> bool:
        return self._send(CMD_VEL, encode_vel(vx_m_s, vy_m_s, wz_rad_s))

    def send_gimbal(self, axis: int, deg: float) -> bool:
        return self._send(CMD_GIMBAL, encode_gimbal(axis, deg))

    def send_home(self, mask: int) -> bool:
        return self._send(CMD_HOME, encode_home(mask))

    def send_gripper(self, deg: float) -> bool:
        return self._send(CMD_GRIPPER, encode_servo(deg))

    def send_plate(self, deg: float) -> bool:
        return self._send(CMD_PLATE, encode_servo(deg))

    def send_cam(self, deg: float) -> bool:
        return self._send(CMD_CAM, encode_servo(deg))

    def send_enable(self, en: bool, mask: int = 0x7F) -> bool:
        return self._send(CMD_ENABLE, encode_enable(en, mask))

    def send_estop(self) -> bool:
        return self._send(CMD_ESTOP)

    def send_heartbeat(self) -> bool:
        return self._send(CMD_HEARTBEAT)

    # ---------- 接收 ----------
    def poll(self) -> list[tuple[int, bytes]]:
        """非阻塞读取并返回 [(cmd, data), ...]，坏帧自动重新同步。"""
        if not self._ensure_connected():
            return []
        try:
            available = int(getattr(self._port, "in_waiting", 0))
            if available <= 0:
                return []
            data = self._port.read(available)
        except Exception as error:  # noqa: BLE001
            self._disconnect(str(error))
            return []

        self._rx.extend(data or b"")
        frames: list[tuple[int, bytes]] = []

        while True:
            # 找帧头 AA 55
            idx = self._rx.find(b"\xaa\x55")
            if idx < 0:
                # 只保留可能的帧头结尾（末尾单个 0xAA）
                if self._rx and self._rx[-1] == SOF1:
                    del self._rx[:-1]
                else:
                    self._rx.clear()
                break
            if idx > 0:
                del self._rx[:idx]
            if len(self._rx) < 4:  # 需要 LEN 才能知道帧长
                break
            length = self._rx[2]
            total = length + 4  # SOF(2) + LEN + SEQ + LEN 字节
            if length + 3 > MAX_FRAME:
                del self._rx[0]  # 异常 LEN，丢一个字节重新同步
                continue
            if len(self._rx) < total:
                break
            candidate = bytes(self._rx[:total])
            crc = (candidate[-1] << 8) | candidate[-2]
            if crc16_modbus(candidate[2:-2]) == crc:
                frames.append((candidate[4], candidate[5:-2]))
            del self._rx[:total]
        return frames

    def close(self) -> None:
        if self._port is not None:
            try:
                self._port.close()
            except Exception:  # noqa: BLE001
                pass
        self._port = None
        self._rx.clear()


if __name__ == "__main__":
    # 脱离 ROS2 的串口自检：每 1s 发心跳并打印收到的帧。
    import sys

    device = sys.argv[1] if len(sys.argv) > 1 else "/dev/serial0"
    link = PiCommLink(device=device)
    print(f"PiComm 自检：{device} @ {link.baudrate}，Ctrl-C 退出")
    last_hb = 0.0
    try:
        while True:
            for cmd, data in link.poll():
                if cmd == RPT_ODOM:
                    print(f"  里程计: {parse_odom(data)}")
                elif cmd == RPT_STATUS:
                    print(f"  状态: {parse_status(data)}")
                else:
                    print(f"  帧: cmd=0x{cmd:02X} data={data.hex()}")
            now = time.monotonic()
            if now - last_hb >= 1.0:
                last_hb = now
                link.send_heartbeat()
                print(f"[{now:.0f}] heartbeat sent, connected={link.connected}")
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n退出")
    finally:
        link.close()
