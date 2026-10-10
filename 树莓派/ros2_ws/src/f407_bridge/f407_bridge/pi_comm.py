#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""树莓派 <-> F407 通信（PiComm 协议，串口 USART6，波特率 115200）。

================ 用大白话讲清楚这个文件在干什么 ================

这个文件负责一件事：把"命令"变成一串字节，通过串口发给 F407（STM32 单片机），
以及把 F407 发回来的一串字节，还原成"数据"。

为什么是一串字节？因为串口一次只能传一个字节（一个 0~255 的数）。
所以任何命令（比如"往前走 100mm/s"）都得先拆成一串数字，对方收到后再按同样的规则拼回来。
这个"拆和拼的规则"就是协议，这里叫 PiComm。

一帧数据长这样（"帧"就是一封完整的"信"）：

    AA 55 | LEN | SEQ | CMD | DATA | CRC
    头     | 长度 | 序号 | 命令 | 内容  | 校验

   AA 55 = 帧头，两个固定字节，表示"一封信开始了"（防止把噪声当成命令）
   LEN   = 这封信后面还有多少个字节（让接收方知道要收多长）
   SEQ   = 序号，每发一封 +1，只是方便观测，不影响功能
   CMD   = 命令编号，比如 1=速度、4=夹爪、8=急停
   DATA  = 命令的具体内容，比如速度是多少
   CRC   = 校验码，像"防伪码"，接收方算一遍对不上就说明传错了，丢掉

注意：CRC 的算法、覆盖哪些字节、字节先后顺序，都必须和 F407 那边写的代码
（UniversalCar/ExHardware/PiComm.c）完全一致，差一点就通信失败。
"""

# 下面这行是固定写法：让 Python 把类型提示（比如 -> int 这种）推迟处理，避免一些加载顺序问题。
from __future__ import annotations

import struct   # struct 用来做"数字 <-> 字节"的转换（打包/解包），是二进制协议的核心工具
import time     # 提供计时功能，用来做串口断线后的"过几秒再重连"
from typing import Callable, Optional

# 尝试导入串口库 pyserial。如果电脑上没装，也不报错——因为"编解码"这部分不依赖串口，
# 只有真正要打开串口收发时才需要它。
try:
    import serial
except ImportError:
    serial = None


# ==================== 命令编号 ====================
# 这些数字是"命令的名字"，和 F407 侧 PiComm.h 里的定义一一对应，改这里必须改那边。
CMD_VEL = 0x01        # 速度指令（前进/后退/旋转）
CMD_GIMBAL = 0x02     # 云台转到某个角度
CMD_HOME = 0x03       # 云台回零（回原点）
CMD_GRIPPER = 0x04    # 夹爪舵机
CMD_PLATE = 0x05      # 托盘舵机
CMD_CAM = 0x06        # 相机舵机
CMD_ENABLE = 0x07     # 电机使能（上电/断电）
CMD_ESTOP = 0x08      # 急停（不带内容=置急停；带一个字节 0=解除急停）
CMD_POSE = 0x09       # 位姿目标（走到世界系 (x,y) 并转到朝向 yaw，底盘位置+航向闭环）
CMD_HEARTBEAT = 0x10  # 心跳（定期发一次，告诉 F407 "我还活着"）

# ==================== 回传数据编号 ====================
# 这些是 F407 主动发给树莓派的"数据编号"（方向反过来）。编号都 >= 0x80，用最高位区分方向。
RPT_ODOM = 0x81        # 里程计（车走了多远、朝向多少）
RPT_STATUS = 0x83      # 整车状态（状态、错误码、电机在线情况）

# 帧头两个字节
SOF1 = 0xAA
SOF2 = 0x55
MAX_FRAME = 64   # 一帧最长 64 字节，必须和 F407 那边的接收缓冲区一样大

# ==================== 错误标志位 ====================
# F407 会在状态帧里用一个数表示"出了什么错"，每一位代表一种错误。
# 这里只是"照抄一份"，让树莓派端能看懂。真正生效的判断在 F407 的 ReceiveTask.c 里。
ERR_ESTOP = 1 << 0        # 第0位：急停
ERR_PI_OFFLINE = 1 << 1   # 第1位：树莓派离线（心跳断了）
ERR_MOTOR_LOST = 1 << 2   # 第2位：有电机掉线

# ==================== 整车状态机的三种状态 ====================
ROBOT_DISABLED = 0   # 未使能（电机没上电）
ROBOT_READY = 1      # 就绪（可以接收运动指令）
ROBOT_FAULT = 2      # 故障


class PiCommError(ValueError):
    """我们自己定义的异常，专门表示"协议出错"（比如数值越界、帧太长）。

    单独定义这个类，好处是调用方可以只捕获它，而不会误吞别的错误。
    """


def crc16_modbus(data: bytes) -> int:
    """算 CRC16-MODBUS 校验码。

    校验码的作用：发送方根据内容算出一个 16 位的数，接收方收到后用同样算法再算一遍，
    对得上说明数据传输没出错；对不上就说明传错/被干扰了，直接丢弃。

    这个算法是"标准"的，多项式 0xA001、初值 0xFFFF，不能改——因为 F407 那边用同一套。
    """
    crc = 0xFFFF                 # 初值固定为 0xFFFF
    for byte in data:            # 一个一个字节处理
        crc ^= byte              # 把当前字节"混"进校验值里（异或）
        for _ in range(8):       # 每个字节要处理 8 位（一个字节=8位）
            # 看最低位是不是 1：是 1 就先右移再异或 0xA001；是 0 就直接右移
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else crc >> 1
    return crc & 0xFFFF          # 只保留低 16 位（校验码是 16 位）


def build_frame(cmd: int, data: bytes = b"", seq: int = 0) -> bytes:
    """把一条命令打包成一整帧字节（就是那封"信"）。

    参数：
        cmd  = 命令编号（比如 CMD_VEL）
        data = 命令内容（字节串，可为空）
        seq  = 序号
    返回：完整的帧字节串
    """
    # 先做两个合法性检查：命令号必须在 0~255；帧不能太长。
    if not 0 <= cmd <= 0xFF:
        raise PiCommError(f"功能码越界：{cmd}")
    n = len(data)                 # 内容有几个字节
    if 3 + n > MAX_FRAME:         # 整帧长度 = 命令(1) + 内容(n) + 校验(2) = 3+n
        raise PiCommError(f"帧过长：{3 + n} > {MAX_FRAME}")

    # bytearray 是一个可以往里逐个"追加"字节的容器（像可以不断 push 的列表）
    frame = bytearray()
    frame.append(SOF1)            # 第1字节：帧头 AA
    frame.append(SOF2)            # 第2字节：帧头 55
    frame.append(3 + n)           # 第3字节：LEN = 后面还有多少字节（命令+内容+校验）
    frame.append(seq & 0xFF)      # 第4字节：序号（& 0xFF 保证是 0~255）
    frame.append(cmd)             # 第5字节：命令号
    frame.extend(data)            # 接着放：内容
    # 计算校验码：从"LEN"字节开始算（frame[2:] 表示从第3个字节到结尾），
    # 也就是覆盖 LEN + SEQ + CMD + DATA，和协议规定一致。
    crc = crc16_modbus(bytes(frame[2:]))
    frame.append(crc & 0xFF)          # 校验码的低 8 位放前面（小端）
    frame.append((crc >> 8) & 0xFF)   # 校验码的高 8 位放后面
    return bytes(frame)               # 转成不可变字节串返回


# ==================== 各命令的内容打包 ====================
# 下面的函数都只负责把"人看得懂的数值"变成"协议要的字节"，不负责发出去。

def encode_vel(vx_m_s: float, vy_m_s: float, wz_rad_s: float) -> bytes:
    """速度指令的内容：三个方向的速度，各占 2 字节（int16，大端）。

    注意单位换算：我们平时用 m/s、rad/s（弧度/秒），但协议传的是 mm/s、mrad/s（毫弧度/秒）。
    为什么？因为整数比小数好传，而且 1mm/s 的精度对小车足够。
    """
    vx = int(round(vx_m_s * 1000.0))     # m/s 乘 1000 变成 mm/s，再四舍五入成整数
    vy = int(round(vy_m_s * 1000.0))
    wz = int(round(wz_rad_s * 1000.0))   # rad/s 乘 1000 变成 mrad/s
    # int16 的取值范围是 -32768 ~ 32767，超出说明给的速度不合理，直接报错，不要默默溢出。
    for name, val in (("vx", vx), ("vy", vy), ("wz", wz)):
        if not -32768 <= val <= 32767:
            raise PiCommError(f"{name} 超出 int16：{val}")
    # struct.pack(">hhh", ...)：把三个整数打包成字节。
    #   ">"  = 大端（高位字节在前，和 F407 解析顺序一致）
    #   "hhh" = 三个 int16
    return struct.pack(">hhh", vx, vy, wz)


def encode_pose(x_m: float, y_m: float, yaw_rad: float) -> bytes:
    """位姿目标指令内容：x(int32 毫米) + y(int32 毫米) + yaw(int16 毫弧度)，共 10 字节。

    这是状态机的核心指令：告诉 F407 "走到世界系 (x,y)，并把朝向转到 yaw"。
    剩下的位置闭环（4 个电机编码器积分里程计）和航向闭环（陀螺仪）都由 F407 自己做。
    """
    x = int(round(x_m * 1000.0))       # 米 → 毫米
    y = int(round(y_m * 1000.0))
    t = int(round(yaw_rad * 1000.0))   # 弧度 → 毫弧度
    for name, val in (("x", x), ("y", y)):
        if not -2147483648 <= val <= 2147483647:   # int32 范围
            raise PiCommError(f"{name} 超出 int32：{val}")
    if not -32768 <= t <= 32767:                   # int16 范围（±18°，远超 ±π 弧度=±3142）
        raise PiCommError(f"yaw 超出 int16：{t}")
    return struct.pack(">iih", x, y, t)   # >iih = 两个 4 字节整数 + 一个 2 字节整数


def encode_gimbal(axis: int, deg: float) -> bytes:
    """云台目标指令内容：轴编号(1字节) + 角度(4字节 int32，单位是 0.01 度)。

    角度存成"角度×100"的整数，是为了保留两位小数精度（比如 45.5 度存成 4550）。
    """
    if axis not in (0, 1, 2):
        raise PiCommError(f"云台轴越界：{axis}")
    return struct.pack(">Bi", axis, int(round(deg * 100.0)))  # B=1字节无符号, i=4字节整数


def encode_home(mask: int) -> bytes:
    """云台回零指令内容：一个字节，每一位代表一个轴要不要回零。"""
    return bytes([mask & 0x07])   # 只保留低 3 位（3 个轴）


def encode_servo(pos_deg: float) -> bytes:
    """舵机角度内容：一个字节，0~180 度（舵机一般就是 0~180）。"""
    if not 0.0 <= pos_deg <= 180.0:
        raise PiCommError(f"舵机角度越界：{pos_deg}")
    return bytes([int(round(pos_deg))])   # 四舍五入成整数，塞进 1 个字节


def encode_enable(en: bool, mask: int = 0x7F) -> bytes:
    """使能指令内容：第1字节=是否使能(1/0)，第2字节=哪些电机(bit0~6 对应电机1~7)。

    mask 默认 0x7F = 二进制 0111 1111 = 7 个电机全选。
    """
    return bytes([1 if en else 0, mask & 0x7F])


# ==================== 回传数据的解析 ====================
# 下面的函数把 F407 发回来的字节，还原成人能看懂的数。和上面的 encode 正好相反。

def parse_odom(data: bytes) -> tuple[float, float, float]:
    """解析里程计帧（0x81），返回 (x米, y米, 朝向弧度)。

    帧内容是：x(int32 毫米) y(int32 毫米) theta(int16 毫弧度)，共 10 字节。
    """
    if len(data) != 10:
        raise PiCommError(f"里程计帧长度应为10，实际{len(data)}")
    x_mm, y_mm, theta_mrad = struct.unpack(">iih", data)  # 解包：两个4字节整数 + 一个2字节整数
    return x_mm / 1000.0, y_mm / 1000.0, theta_mrad / 1000.0  # 换成 米、米、弧度


def parse_status(data: bytes) -> tuple[int, int, int]:
    """解析状态帧（0x83），返回 (状态, 错误码, 电机在线掩码)。"""
    if len(data) != 3:
        raise PiCommError(f"状态帧长度应为3，实际{len(data)}")
    return data[0], data[1], data[2]   # 三个字节依次对应三个含义


class PiCommLink:
    """维护一个串口连接，负责"自动重连 + 发命令 + 收数据"。

    特点：
      1. 非阻塞——读写不会卡住整个程序（ROS2 节点不能卡）；
      2. 自动重连——串口拔了再插、F407 重启，都会自动恢复；
      3. 收数据时能自动"找帧头、拼帧、校验"，坏数据自动丢。
    """

    def __init__(
        self,
        device: str = "/dev/serial0",   # 串口设备名（树莓派的串口）
        baudrate: int = 115200,         # 波特率，必须和 F407 一致
        reconnect_interval_s: float = 1.0,  # 断线后隔几秒再尝试重连
        serial_factory: Callable = serial.Serial if serial else None,  # 串口"工厂"（测试时可替换）
        clock: Callable[[], float] = time.monotonic,  # 时钟（测试时可替换）
    ):
        if serial_factory is None:
            raise PiCommError("未安装 pyserial，无法创建串口")
        self.device = str(device)
        self.baudrate = int(baudrate)
        self.reconnect_interval_s = float(reconnect_interval_s)
        self._serial_factory = serial_factory
        self._clock = clock
        self._port = None            # 当前串口对象，None 表示未连接
        self._rx = bytearray()       # 接收缓冲区：存还没拼成完整帧的字节
        self._seq = 0                # 发送序号
        self._next_reconnect_s = 0.0 # 下次允许重连的时间点
        self.last_error = ""         # 最近一次错误信息（供外部查看）

    # ---------- 连接管理 ----------
    @property
    def connected(self) -> bool:
        """是否已连上串口。"""
        return bool(self._port is not None and getattr(self._port, "is_open", True))

    def _disconnect(self, error: str) -> None:
        """断开连接：关串口、清缓冲区、推迟下次重连时间。"""
        if self._port is not None:
            try:
                self._port.close()
            except (OSError, Exception):
                pass
        self._port = None
        self._rx.clear()   # 半截数据作废
        self._next_reconnect_s = self._clock() + self.reconnect_interval_s  # 1 秒后再试
        self.last_error = str(error)

    def _ensure_connected(self) -> bool:
        """确保已连接：已连接就返回 True；没连且到时间了就去连；没到时间就返回 False。"""
        if self.connected:
            return True
        if self._clock() < self._next_reconnect_s:   # 还没到重连时间，别急着连
            return False
        try:
            # 打开串口。timeout=0 和 write_timeout=0 表示"非阻塞"——读写立即返回，不等待。
            self._port = self._serial_factory(
                port=self.device,
                baudrate=self.baudrate,
                timeout=0,
                write_timeout=0,
            )
        except Exception as error:
            self._disconnect(str(error))
            return False
        self.last_error = ""
        return True

    # ---------- 发送 ----------
    def _send(self, cmd: int, data: bytes = b"") -> bool:
        """内部发送函数：打包 -> 写入串口 -> 序号+1。返回是否成功。"""
        if not self._ensure_connected():
            return False
        frame = build_frame(cmd, data, self._seq)   # 打包成一帧
        try:
            written = self._port.write(frame)       # 写出去，返回写了几个字节
            if written != len(frame):               # 没写完整 = 异常
                raise serial.SerialTimeoutException(
                    f"只写入 {written}/{len(frame)} 字节"
                )
        except Exception as error:
            self._disconnect(str(error))            # 写失败 -> 断开，等重连
            return False
        self._seq = (self._seq + 1) & 0xFF          # 序号 +1，超过 255 就回到 0
        return True

    # 下面这些是"对外接口"：名字一看就知道发什么命令，内部再调用 _send。
    # 上层代码只需要写 link.send_vel(...)，不用记 0x01 这种数字。
    def send_vel(self, vx_m_s: float, vy_m_s: float, wz_rad_s: float) -> bool:
        return self._send(CMD_VEL, encode_vel(vx_m_s, vy_m_s, wz_rad_s))

    def send_pose(self, x_m: float, y_m: float, yaw_rad: float) -> bool:
        return self._send(CMD_POSE, encode_pose(x_m, y_m, yaw_rad))

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

    def send_estop_clear(self) -> bool:
        return self._send(CMD_ESTOP, b"\x00")   # 带一个字节 0 = 解除急停

    def send_heartbeat(self) -> bool:
        return self._send(CMD_HEARTBEAT)

    # ---------- 接收 ----------
    def poll(self) -> list[tuple[int, bytes]]:
        """非阻塞地收一次数据，返回 [(命令号, 内容), ...] 列表。

        因为串口是"流"（字节一个接一个来），一次 read 可能只收到半帧，
        所以要先把字节攒进缓冲区，再尝试从中"切"出完整的帧。
        """
        if not self._ensure_connected():
            return []
        try:
            available = int(getattr(self._port, "in_waiting", 0))  # 串口里攒了多少字节没读
            if available <= 0:
                return []                        # 没数据，直接返回空
            data = self._port.read(available)    # 把积压的字节一次性读出来
        except Exception as error:
            self._disconnect(str(error))
            return []

        self._rx.extend(data or b"")   # 追加到接收缓冲区
        frames: list[tuple[int, bytes]] = []

        # 循环：尽量从缓冲区里切出完整帧
        while True:
            idx = self._rx.find(b"\xaa\x55")   # 找帧头 AA 55 的位置
            if idx < 0:
                # 没找到帧头：说明这堆字节不是有效数据，全丢掉。
                # 但有个小细节：如果最后恰好残留一个 0xAA（可能是下一帧帧头的前半截），
                # 就把它留着，等下一批数据补上 0x55。
                if self._rx and self._rx[-1] == SOF1:
                    del self._rx[:-1]
                else:
                    self._rx.clear()
                break
            if idx > 0:
                del self._rx[:idx]   # 帧头前面有杂散字节，丢掉它们
            if len(self._rx) < 4:    # 连"长度"字节都还没收齐（至少4字节才能读长度），继续等
                break
            length = self._rx[2]     # 第3个字节是 LEN（帧长）
            total = length + 4       # 整帧总长 = 帧头(2) + LEN + SEQ + LEN字节
            if length + 3 > MAX_FRAME:
                del self._rx[0]      # 长度异常（说明已经失步），丢掉1个字节重新找帧头
                continue
            if len(self._rx) < total:  # 还没收满一整帧，继续等
                break
            candidate = bytes(self._rx[:total])     # 取出一整帧
            crc = (candidate[-1] << 8) | candidate[-2]  # 帧尾两个字节拼成校验码
            if crc16_modbus(candidate[2:-2]) == crc:   # 重新算一遍，和帧里的比对
                # 校验通过：取出 (命令号=第5字节, 内容=中间那段)，收下
                frames.append((candidate[4], candidate[5:-2]))
            # 不管校验过没过，这一帧都要从缓冲区移除（坏帧直接丢弃）
            del self._rx[:total]
        return frames

    def close(self) -> None:
        """关闭串口，释放资源（程序退出前调用）。"""
        if self._port is not None:
            try:
                self._port.close()
            except Exception:
                pass
        self._port = None
        self._rx.clear()


if __name__ == "__main__":
    # 直接运行这个文件（不通过 ROS2），用来单独测试串口和协议是否打通：
    #   python3 comm/pi_comm.py /dev/serial0
    # 它会每秒发一次心跳，并把收到的帧打印出来。
    import sys

    device = sys.argv[1] if len(sys.argv) > 1 else "/dev/serial0"  # 从命令行读设备名
    link = PiCommLink(device=device)
    print(f"PiComm 自检：{device} @ {link.baudrate}，Ctrl-C 退出")
    last_hb = 0.0
    try:
        while True:
            for cmd, data in link.poll():   # 收数据并打印
                if cmd == RPT_ODOM:
                    print(f"  里程计: {parse_odom(data)}")
                elif cmd == RPT_STATUS:
                    print(f"  状态: {parse_status(data)}")
                else:
                    print(f"  帧: cmd=0x{cmd:02X} data={data.hex()}")
            now = time.monotonic()
            if now - last_hb >= 1.0:        # 每秒发一次心跳
                last_hb = now
                link.send_heartbeat()
                print(f"[{now:.0f}] heartbeat sent, connected={link.connected}")
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n退出")
    finally:
        link.close()
