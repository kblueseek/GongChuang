#!/usr/bin/env python3
"""第二路相机：二维码 → 任务码 → 串口回传给 F407。

单文件、自包含
--------------
不 import 项目里任何模块，也不要求改动 main.py / turntable_serial.py。
只要 pyserial 在环境里，拷进去就能跑。

⚠️ 串口冲突（必读）
------------------
Linux 下同一个 tty 被两个进程同时打开时，收到的字节会被驱动**随机分给其中一个**。
所以本脚本**不要和 main.py 同时运行**，除非给 F407 接了第二路 UART，
再用 ``--serial-device`` 指到那个口。

两种可行用法：

    # A. 分阶段：开机先扫码，把任务码交给 F407，扫完退出；之后再启动识别主程序
    python3 qr_scanner.py --device /dev/video2
    python3 code/main.py --device /dev/video0

    # B. 同时运行：F407 接第二路 UART，两边各占一个口
    python3 qr_scanner.py --device /dev/video2 --serial-device /dev/ttyS3
    python3 code/main.py --device /dev/video0 --serial-device /dev/ttyS7

    # C. 只扫码不发串口（调试用）
    python3 qr_scanner.py --device /dev/video2 --no-serial

协议
----
发送帧 ``CC 07 <15字节ASCII任务码> DD``，共 18 字节，无长度字段、无校验和。
扫到有效码后立即发一次，之后每 ``--resend-interval`` 秒重发一次
（默认 1 秒；用 ``--resend-count`` 限制总次数）。18 字节/秒在 115200 下
只占约 0.16% 带宽，下位机重复收到同一个码忽略即可，不需要做 ACK。

本脚本**只写不读**串口：下位机发来的 AA 开头的控制帧本脚本不处理，
留在内核缓冲里，进程退出时自然丢弃。
"""

from __future__ import annotations

import argparse
import time
from typing import Optional

import cv2
import numpy as np

try:
    import serial
except ImportError:  # 装了 pyserial 才需要串口；纯扫码模式不受影响。
    serial = None


WINDOW_NAME = "QR Scanner"

# ---- 串口帧格式（与 code/串口通信协议.md 的 CC 方向一致） ----
TX_HEADER = 0xCC
TX_TASK_FUNCTION = 0x07
TX_TAIL = 0xDD

# ---- 任务码规则 ----
TASK_CODE_LENGTH = 15
TASK_CODE_SEPARATORS = (3, 7, 11)

# 现有视觉程序在 turntable_serial.py:288 也要求"两批颜色集合一致"，
# 与本赛项规则相同，所以保持 True。若日后规则放开，改这里即可。
REQUIRE_SAME_COLOR_SET = True

# 连续多少帧解出同一个码才采信，用来挡 QRCodeDetector 的偶发误读。
REQUIRED_STABLE_FRAMES = 3


class QrScannerError(RuntimeError):
    """相机、串口或参数不满足要求。"""


# --------------------------------------------------------------------------
# 任务码校验与编码
# --------------------------------------------------------------------------
# 规则（与 turntable_serial.py:253 decode_task_code_frame 一致）：
#   XXX+XXX+XXX+XXX
#   第 1、3 组：三位数，取值 1~6，组内不重复，两组集合相同（顺序可变）
#   第 2、4 组：必须是 1/2/3 的不重复排列
def normalise_task_code(text: object) -> Optional[str]:
    """校验 15 字符任务码；合法返回原串，否则返回 None。"""

    if not isinstance(text, str) or len(text) != TASK_CODE_LENGTH:
        return None
    if any(text[index] != "+" for index in TASK_CODE_SEPARATORS):
        return None

    colors_a, rings_a, colors_b, rings_b = (
        text[0:3], text[4:7], text[8:11], text[12:15]
    )
    for colors in (colors_a, colors_b):
        if any(char not in "123456" for char in colors):
            return None
        if len(set(colors)) != 3:
            return None
    if REQUIRE_SAME_COLOR_SET and set(colors_a) != set(colors_b):
        return None
    for rings in (rings_a, rings_b):
        if set(rings) != {"1", "2", "3"}:
            return None
    return text


def encode_task_code_frame(text: str) -> bytes:
    """组 ``CC 07 <15字节ASCII> DD``，共 18 字节。"""

    body = normalise_task_code(text)
    if body is None:
        raise QrScannerError(f"任务码不合法，拒绝编码：{text!r}")
    return (
        bytes((TX_HEADER, TX_TASK_FUNCTION))
        + body.encode("ascii")
        + bytes((TX_TAIL,))
    )


# --------------------------------------------------------------------------
# 串口推送
# --------------------------------------------------------------------------
class SerialPusher:
    """把任务码推给 F407。打不开或写失败都不终止扫码，只重试。"""

    def __init__(
        self,
        device: str,
        baudrate: int = 115200,
        reconnect_interval_s: float = 1.0,
        resend_interval_s: float = 1.0,
        resend_count: int = 0,
    ):
        if serial is None:
            raise QrScannerError(
                "未安装 pyserial，无法发送任务码；改用 --no-serial 可跳过"
            )
        if not device:
            raise QrScannerError("串口设备节点不能为空")
        if baudrate <= 0:
            raise QrScannerError("串口波特率必须大于0")
        if reconnect_interval_s <= 0.0:
            raise QrScannerError("串口重连间隔必须大于0")
        if resend_interval_s < 0.0:
            raise QrScannerError("重发间隔不能为负数")
        if resend_count < 0:
            raise QrScannerError("重发次数不能为负数")

        self.device = str(device)
        self.baudrate = int(baudrate)
        self.reconnect_interval_s = float(reconnect_interval_s)
        self.resend_interval_s = float(resend_interval_s)
        self.resend_count = int(resend_count)

        self._port = None
        self._next_reconnect_s = 0.0
        self._text: Optional[str] = None
        self._text_sent_count = 0
        self._last_sent_s = 0.0

        self.sent_count = 0
        self.last_error = ""

    @property
    def connected(self) -> bool:
        return bool(
            self._port is not None and getattr(self._port, "is_open", True)
        )

    @property
    def status(self) -> str:
        return "CONNECTED" if self.connected else "RETRYING"

    def _disconnect(self, error: object) -> None:
        if self._port is not None:
            try:
                self._port.close()
            except (OSError, serial.SerialException):
                pass
        self._port = None
        self._next_reconnect_s = time.monotonic() + self.reconnect_interval_s
        self.last_error = str(error)
        print(f"串口不可用 ({self.device})：{error}")

    def _ensure_connected(self) -> bool:
        if self.connected:
            return True
        if time.monotonic() < self._next_reconnect_s:
            return False
        try:
            self._port = serial.Serial(
                port=self.device,
                baudrate=self.baudrate,
                timeout=0,
                write_timeout=0,
            )
        except (OSError, serial.SerialException) as error:
            self._disconnect(error)
            return False
        self.last_error = ""
        print(f"串口已连接：{self.device}，{self.baudrate} baud")
        return True

    def push(self, text: str) -> bool:
        """按需发送一次；真正写入完整帧时返回 True。

        码发生变化 → 立即发；码没变 → 每隔 resend_interval_s 重发一次，
        受 resend_count 限制（0 表示不限次数）。
        """

        now = time.monotonic()
        if text == self._text:
            if self.resend_count > 0 and self._text_sent_count >= self.resend_count:
                return False
            if now - self._last_sent_s < self.resend_interval_s:
                return False
        else:
            self._text = text
            self._text_sent_count = 0

        if not self._ensure_connected():
            return False

        frame = encode_task_code_frame(text)
        try:
            written = self._port.write(frame)
            if written != len(frame):
                raise serial.SerialTimeoutException(
                    f"串口只写入 {written}/{len(frame)} 字节"
                )
        except (OSError, serial.SerialException) as error:
            self._disconnect(error)
            return False

        self._text_sent_count += 1
        self.sent_count += 1
        self._last_sent_s = now
        self.last_error = ""
        return True

    def close(self) -> None:
        if self._port is not None:
            try:
                self._port.close()
            except (OSError, serial.SerialException):
                pass
        self._port = None


# --------------------------------------------------------------------------
# 相机与解码
# --------------------------------------------------------------------------
def open_camera(device: str, width: int, height: int, fps: float):
    """按 MJPG 打开扫码相机。分辨率不一致时只警告，不中止。

    扫码不需要高分辨率，默认请求 640x480；工业相机未必支持这个尺寸，
    而 QR 解码本身与分辨率无关，所以这里不强求尺寸一致。
    """

    camera = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not camera.isOpened():
        raise QrScannerError(f"无法打开扫码相机：{device}")

    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    camera.set(cv2.CAP_PROP_FPS, fps)
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 4)

    for _ in range(10):
        ok, frame = camera.read()
        if ok and frame is not None:
            break
    else:
        camera.release()
        raise QrScannerError(f"相机已打开但读不到画面：{device}")

    actual = (frame.shape[1], frame.shape[0])
    if actual != (width, height):
        print(f"警告：请求 {width}x{height}，实际 {actual[0]}x{actual[1]}，继续运行")
    return camera


def decode_qr(detector, frame):
    """返回 (文本, 四边形)。解不出来时返回 ("", None)。

    单码接口失败时再退到多码接口，对"画面里有多个码"或角度刁钻更稳。
    """

    try:
        text, points, _ = detector.detectAndDecode(frame)
    except (cv2.error, AttributeError):
        text, points = "", None
    if text:
        return text, points

    try:
        ok, texts, multi_points, _ = detector.detectAndDecodeMulti(frame)
    except (cv2.error, AttributeError):
        return "", None
    if not ok:
        return "", None
    for candidate, candidate_points in zip(texts, multi_points):
        if candidate:
            return candidate, candidate_points
    return "", None


def draw_polygon(frame, points) -> None:
    """画出二维码四边形，兼容 (1,4,2) 与 (4,2) 两种形状。"""

    if points is None:
        return
    polygon = np.asarray(points, dtype=np.float32).reshape(-1, 1, 2)
    if polygon.shape[0] < 3:
        return
    cv2.polylines(
        frame, [np.rint(polygon).astype(np.int32)], True, (0, 255, 0), 2
    )


# --------------------------------------------------------------------------
# 主流程
# --------------------------------------------------------------------------
def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="第二路相机扫码：二维码 → 任务码 → CC 07 串口回传"
    )
    parser.add_argument(
        "--device", default="/dev/video2",
        help="扫码相机的 V4L2 节点，建议用 /dev/v4l/by-id/ 下的稳定路径",
    )
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=15.0)

    parser.add_argument(
        "--serial-device", default="/dev/ttyS7",
        help="发任务码用的串口；树莓派上是 /dev/serial0 或 /dev/ttyAMA0",
    )
    parser.add_argument("--serial-baud", type=int, default=115200)
    parser.add_argument(
        "--no-serial", action="store_true",
        help="只扫码不发串口（调试用，避免和 main.py 抢串口）",
    )
    parser.add_argument(
        "--resend-interval", type=float, default=1.0,
        help="同一个码的重发间隔秒数，0 表示每帧都发（不推荐）",
    )
    parser.add_argument(
        "--resend-count", type=int, default=0,
        help="同一个码最多发几次，0 表示不限次数",
    )

    parser.add_argument(
        "--no-show", dest="show", action="store_false",
        help="不开预览窗口（无桌面环境时用）",
    )
    parser.add_argument(
        "--once", action="store_true",
        help="识别到一个有效码并成功发出后立即退出",
    )
    parser.set_defaults(show=True)
    return parser.parse_args()


def run(args: argparse.Namespace) -> None:
    detector = cv2.QRCodeDetector()

    pusher: Optional[SerialPusher] = None
    if not args.no_serial:
        pusher = SerialPusher(
            args.serial_device,
            args.serial_baud,
            resend_interval_s=args.resend_interval,
            resend_count=args.resend_count,
        )

    camera = open_camera(args.device, args.width, args.height, args.fps)

    stable_text = ""
    stable_frames = 0
    accepted: Optional[str] = None
    pushed_once = False

    print(f"扫码进程已启动：{args.device}")
    if pusher is None:
        print("串口：已禁用（--no-serial），任务码只显示不发送")
    else:
        print(f"串口：{pusher.device} @ {pusher.baudrate}")
        print("⚠️ 确认 main.py 没有同时运行，否则两个进程会抢同一个串口")
    print("按 Q 或 Esc 退出。")

    try:
        if args.show:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)

        while True:
            ok, frame = camera.read()
            if not ok or frame is None:
                print("扫码相机读取失败，停止。")
                break

            text, points = decode_qr(detector, frame)
            raw_text = text.strip()

            # 去抖：同一个串连续出现若干帧才采信，滤掉偶发误读。
            if raw_text:
                if raw_text == stable_text:
                    stable_frames += 1
                else:
                    stable_text, stable_frames = raw_text, 1
            else:
                stable_text, stable_frames = "", 0

            if stable_frames >= REQUIRED_STABLE_FRAMES:
                code = normalise_task_code(stable_text)
                if code is not None:
                    accepted = code
                    if pusher is not None and pusher.push(code):
                        pushed_once = True

            if args.once and pushed_once:
                print("已发送任务码，按 --once 退出。")
                break

            if args.show:
                display = frame.copy()
                draw_polygon(display, points)
                serial_status = "OFF" if pusher is None else pusher.status
                sent = 0 if pusher is None else pusher.sent_count
                status = (
                    f"QR: {accepted if accepted else 'SEARCHING'}   "
                    f"Stable={stable_frames}/{REQUIRED_STABLE_FRAMES}   "
                    f"Serial={serial_status} Sent={sent}"
                )
                width = display.shape[1]
                cv2.rectangle(
                    display, (10, 8), (min(width - 10, 1080), 48), (0, 0, 0), -1
                )
                cv2.putText(
                    display, status, (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                    (80, 255, 80) if accepted else (255, 255, 255),
                    2, cv2.LINE_AA,
                )
                if stable_text:
                    cv2.putText(
                        display, stable_text, (20, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 255), 2,
                        cv2.LINE_AA,
                    )
                cv2.imshow(WINDOW_NAME, display)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    break
            else:
                time.sleep(0.05)
    finally:
        camera.release()
        if pusher is not None:
            pusher.close()
        cv2.destroyAllWindows()


def main() -> int:
    args = parse_args()
    if args.width <= 0 or args.height <= 0:
        print("错误：分辨率必须为正数")
        return 1
    if args.fps <= 0:
        print("错误：帧率必须为正数")
        return 1
    try:
        run(args)
    except KeyboardInterrupt:
        print("\n已停止扫码。")
        return 0
    except (QrScannerError, cv2.error) as error:
        print(f"错误：{error}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
