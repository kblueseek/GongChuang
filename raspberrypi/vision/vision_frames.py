#!/usr/bin/env python3
"""视觉 ↔ F407 串口帧的编解码。纯 Python，无第三方依赖。

为什么要单独抽出来
------------------
ROS2 化以后串口由 `serial_gateway` 节点独占，帧的编解码必须和 I/O 分离，
才能同时被 ROS2 节点和旧的纯 Python 程序复用。本文件不 import cv2 /
numpy / serial，可以直接单独跑自测：

    python3 vision_frames.py

帧格式来源：`串口通信协议.md`
-----------------------------
入站（F407 → 视觉），固定 3 字节：      ``AA <功能码> BB``
入站任务码，固定 18 字节：              ``AA 06 <15字节ASCII> BB``
出站（视觉 → F407）：
    转盘中心 7 字节     ``CC 05 X_L X_H Y_L Y_H DD``
    圆环/码垛 11 字节   ``CC 03 X_L X_H Y_L Y_H <float32度> DD``
    物料颜色 6 字节     ``CC 02 L R D DD``
    任务码 18 字节      ``CC 07 <15字节ASCII> DD``
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct


# ---------------------------------------------------------------- 帧常量
RX_HEADER = 0xAA
RX_TAIL = 0xBB
RX_TASK_FUNCTION = 0x06

TX_HEADER = 0xCC
TX_TAIL = 0xDD
TX_MATERIAL_FUNCTION = 0x02
TX_RING_FUNCTION = 0x03
TX_CENTER_FUNCTION = 0x05
TX_TASK_FUNCTION = 0x07

TASK_CODE_BODY_SIZE = 15
TASK_CODE_FRAME_SIZE = 18

CENTER_FRAME_SIZE = 7
RING_FRAME_SIZE = 11
MATERIAL_FRAME_SIZE = 6

COLOR_NAMES = {
    0: "empty",
    1: "red",
    2: "yellow",
    3: "blue",
    4: "green",
    5: "black",
    6: "light_blue",
}


class FrameError(ValueError):
    """帧内容不合协议。"""


# ---------------------------------------------------------------- 出站编码
def encode_center_frame(center_x_px: float, center_y_px: float) -> bytes:
    """``CC 05 X_L X_H Y_L Y_H DD``。``int()`` 截断，负坐标按 0 发。"""

    if not (math.isfinite(center_x_px) and math.isfinite(center_y_px)):
        raise FrameError("转盘中心必须是有限数值")
    x_value = max(0, int(center_x_px))
    y_value = max(0, int(center_y_px))
    if not (0 <= x_value <= 0xFFFF and 0 <= y_value <= 0xFFFF):
        raise FrameError(f"转盘中心超出 uint16：({x_value}, {y_value})")
    return struct.pack(
        "<BBHHB", TX_HEADER, TX_CENTER_FUNCTION, x_value, y_value, TX_TAIL
    )


def encode_ring_frame(center_px, station_yaw_deg: float) -> bytes:
    """``CC 03 X_L X_H Y_L Y_H <float32> DD``，角度单位度、小端。"""

    try:
        center_x, center_y = center_px
    except (TypeError, ValueError) as error:
        raise FrameError("圆心格式无效") from error
    x_float, y_float = float(center_x), float(center_y)
    if not (math.isfinite(x_float) and math.isfinite(y_float)):
        raise FrameError("圆心必须是有限数值")
    x_value, y_value = int(x_float), int(y_float)
    if not (0 <= x_value <= 0xFFFF and 0 <= y_value <= 0xFFFF):
        raise FrameError(f"圆心超出 uint16：({x_value}, {y_value})")
    angle = float(station_yaw_deg)
    if not math.isfinite(angle):
        raise FrameError("角度必须是有限数值")
    try:
        return struct.pack(
            "<BBHHfB",
            TX_HEADER,
            TX_RING_FUNCTION,
            x_value,
            y_value,
            angle,
            TX_TAIL,
        )
    except (OverflowError, struct.error) as error:
        raise FrameError(f"角度无法编码为 float32：{angle}") from error


def encode_material_frame(left: int, right: int, down: int) -> bytes:
    """``CC 02 L R D DD``，颜色编号 0~6，0 表示该位置为空。"""

    values = []
    for name, value in (("left", left), ("right", right), ("down", down)):
        if isinstance(value, bool) or not isinstance(value, int):
            raise FrameError(f"{name} 颜色编号必须是整数")
        if not 0 <= value <= 6:
            raise FrameError(f"{name} 颜色编号超出 0~6：{value}")
        values.append(value)
    return bytes((
        TX_HEADER,
        TX_MATERIAL_FUNCTION,
        values[0],
        values[1],
        values[2],
        TX_TAIL,
    ))


# ---------------------------------------------------------------- 任务码
def validate_task_code_body(body: str) -> tuple:
    """校验 15 字符任务码正文，返回四组整数元组。

    规则（与赛项规定一致）：
        第 1、3 组：三位数，取值 1~6，组内不重复，**两批颜色集合相同（顺序可变）**
        第 2、4 组：必须是 1/2/3 的不重复排列
    """

    if not isinstance(body, str) or len(body) != TASK_CODE_BODY_SIZE:
        raise FrameError("任务码正文长度不是15字符")
    if body[3] != "+" or body[7] != "+" or body[11] != "+":
        raise FrameError("任务码分隔符位置无效")

    groups = (body[0:3], body[4:7], body[8:11], body[12:15])
    colors_a, rings_a, colors_b, rings_b = groups

    for name, colors in (("第一批颜色", colors_a), ("第二批颜色", colors_b)):
        if any(char not in "123456" for char in colors):
            raise FrameError(f"{name}必须为1~6")
        if len(set(colors)) != 3:
            raise FrameError(f"{name}不能重复")
    if set(colors_a) != set(colors_b):
        raise FrameError("两批颜色集合不一致")

    for name, rings in (("第一批圆环", rings_a), ("第二批圆环", rings_b)):
        if set(rings) != {"1", "2", "3"}:
            raise FrameError(f"{name}必须是1/2/3的不重复排列")

    return tuple(tuple(int(char) for char in group) for group in groups)


def encode_task_code_frame(text: str) -> bytes:
    """``CC 07 <15字节ASCII> DD``，共 18 字节。"""

    validate_task_code_body(str(text))
    return (
        bytes((TX_HEADER, TX_TASK_FUNCTION))
        + str(text).encode("ascii")
        + bytes((TX_TAIL,))
    )


# ---------------------------------------------------------------- 入站解析
@dataclass(frozen=True)
class ModeCommand:
    """一条 3 字节模式/上报控制帧：``AA <功能码> BB``。"""

    function: int

    def __str__(self) -> str:
        return f"ModeCommand(0x{self.function:02X})"


@dataclass(frozen=True)
class TaskCodeFrame:
    """一条 18 字节任务码帧：``AA 06 <15字节ASCII> BB``。"""

    text: str
    first_colors: tuple
    first_rings: tuple
    second_colors: tuple
    second_rings: tuple


class FrameParser:
    """把任意分段的字节流解析成完整帧，坏帧不会导致后续失步。

    用法::

        parser = FrameParser()
        for event in parser.feed(data):
            ...
    """

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.invalid_task_count = 0

    def reset(self) -> None:
        """串口重连时调用：丢掉半帧，避免和重连后的数据错误拼接。"""
        self._buffer.clear()

    def feed(self, data: bytes):
        """追加字节并返回本次解析出的完整事件列表。"""

        self._buffer.extend(data)
        events = []

        while self._buffer:
            try:
                start = self._buffer.index(RX_HEADER)
            except ValueError:
                self._buffer.clear()
                break
            if start:
                del self._buffer[:start]
            if len(self._buffer) < 2:
                break

            function = self._buffer[1]
            if function == RX_TASK_FUNCTION:
                if len(self._buffer) < TASK_CODE_FRAME_SIZE:
                    break
                candidate = bytes(self._buffer[:TASK_CODE_FRAME_SIZE])
                try:
                    event = self._decode_task_code_frame(candidate)
                except FrameError:
                    # 坏任务帧只滑过一个字节，重新搜 AA，避免吞掉紧随的合法帧。
                    self.invalid_task_count += 1
                    del self._buffer[0]
                    continue
                del self._buffer[:TASK_CODE_FRAME_SIZE]
                events.append(event)
                continue

            if len(self._buffer) < 3:
                break
            if self._buffer[2] != RX_TAIL:
                del self._buffer[0]
                continue

            del self._buffer[:3]
            events.append(ModeCommand(function))

        return events

    @staticmethod
    def _decode_task_code_frame(frame: bytes) -> TaskCodeFrame:
        if len(frame) != TASK_CODE_FRAME_SIZE:
            raise FrameError(f"任务码帧长度应为18字节，实际{len(frame)}")
        if frame[0] != RX_HEADER or frame[1] != RX_TASK_FUNCTION:
            raise FrameError("任务码帧头或功能码无效")
        if frame[-1] != RX_TAIL:
            raise FrameError("任务码帧尾不是0xBB")
        try:
            body = frame[2:-1].decode("ascii")
        except UnicodeDecodeError as error:
            raise FrameError("任务码正文不是ASCII") from error
        groups = validate_task_code_body(body)
        return TaskCodeFrame(body, groups[0], groups[1], groups[2], groups[3])


# ---------------------------------------------------------------- 自测
def _self_test() -> int:
    failures = []

    def check(label, got, expected):
        if got != expected:
            failures.append(label)
            print(f"FAIL  {label}\n      got      {got!r}\n      expected {expected!r}")
        else:
            print(f"ok    {label}")

    # 出站帧：逐字节对照 串口通信协议.md 里的例子
    check(
        "center (320.9,240.9)",
        encode_center_frame(320.9, 240.9).hex(" "),
        "cc 05 40 01 f0 00 dd",
    )
    check(
        "center (640,-13) 负坐标归零",
        encode_center_frame(640, -13).hex(" "),
        "cc 05 80 02 00 00 dd",
    )
    check(
        "ring (633.9,321.9) -1.25deg",
        encode_ring_frame((633.9, 321.9), -1.25).hex(" "),
        "cc 03 79 02 41 01 00 00 a0 bf dd",
    )
    check(
        "material L=1 R=0 D=3",
        encode_material_frame(1, 0, 3).hex(" "),
        "cc 02 01 00 03 dd",
    )
    check(
        "task code 156+123+651+213",
        encode_task_code_frame("156+123+651+213").hex(" "),
        "cc 07 31 35 36 2b 31 32 33 2b 36 35 31 2b 32 31 33 dd",
    )

    # 入站：协议文档里给的 AA 06 例子
    raw = bytes.fromhex(
        "aa 06 31 35 36 2b 31 32 33 2b 36 35 31 2b 32 31 33 bb"
    )
    events = FrameParser().feed(raw)
    check("task code frame 解出一条", len(events), 1)
    check("task code 正文", events[0].text, "156+123+651+213")
    check("task code 四组", (
        events[0].first_colors, events[0].first_rings,
        events[0].second_colors, events[0].second_rings,
    ), ((1, 5, 6), (1, 2, 3), (6, 5, 1), (2, 1, 3)))

    # 入站：3 字节模式命令 + 噪声 + 拆包 + 粘包
    parser = FrameParser()
    check("噪声后接 AA 05 BB", [str(e) for e in parser.feed(b"\x01\x02\xaa\x05\xbb")],
          ["ModeCommand(0x05)"])
    check("拆包：只给一半不出事件", parser.feed(b"\xaa\x02"), [])
    check("拆包：补齐后出事件", [str(e) for e in parser.feed(b"\xbb")],
          ["ModeCommand(0x02)"])
    check("粘包：一次两帧", [str(e) for e in parser.feed(b"\xaa\x15\xbb\xaa\x14\xbb")],
          ["ModeCommand(0x15)", "ModeCommand(0x14)"])
    check("坏帧尾重新同步", [str(e) for e in parser.feed(b"\xaa\x05\x00\xaa\x05\xbb")],
          ["ModeCommand(0x05)"])

    # 任务码规则
    for bad, why in [
        ("156+123+652+213", "两批颜色集合不一致"),
        ("111+123+651+213", "颜色重复"),
        ("156+123+651+223", "圆环非1-3排列"),
        ("156-123-651-213", "分隔符错"),
        ("156+123+651+2134", "长度错"),
    ]:
        try:
            validate_task_code_body(bad)
        except FrameError:
            check(f"拒绝 {why}", "raised", "raised")
        else:
            check(f"拒绝 {why}", "no raise", "raised")

    check("接受顺序可变的合法码", validate_task_code_body("651+321+156+213")[2], (1, 5, 6))

    print()
    if failures:
        print(f"❌ {len(failures)} 项失败：{failures}")
        return 1
    print("✅ 全部通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(_self_test())
