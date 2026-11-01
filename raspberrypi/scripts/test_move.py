#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""台架测试：让小车前进一小段（不需要启动 ROS2）。

用法：
    python3 scripts/test_move.py                      # 默认 /dev/ttyAMA0，0.15 m/s，2 秒
    python3 scripts/test_move.py /dev/ttyAMA0 0.2 3   # 0.2 m/s，持续 3 秒
    python3 scripts/test_move.py COM5 0.15 2          # Windows + USB-TTL 也可以

注意：
    1. 树莓派上先把 ros2 launch / f407_bridge 停掉——同一个串口只能被一个程序打开；
    2. 串口接的是 C8T6 的 USART2（PA2=TX、PA3=RX），115200-8N1，TX/RX 交叉、共地；
    3. C8T6 侧超过 PI_CMD_TIMEOUT_MS=500ms 收不到任何帧会自主失能，所以本脚本
       每 20ms 发一次速度、每 200ms 发一次心跳；脚本退出后车会在 0.5 秒内自动失能。
"""

import os
import sys
import time

# 让 `python3 scripts/test_move.py` 能 import 到仓库根目录下的 comm/pi_comm.py
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from comm.pi_comm import PiCommLink, RPT_STATUS, parse_status   # noqa: E402


def main() -> int:
    device = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyAMA0"
    vx = float(sys.argv[2]) if len(sys.argv) > 2 else 0.15
    duration = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0

    link = PiCommLink(device=device)

    # 先发一帧确认串口能打开；打不开就直接告诉用户原因
    if not link.send_heartbeat():
        print(f"串口打开失败：{device} -> {link.last_error}")
        return 1

    print(f"{device} @ {link.baudrate} 已打开，目标 vx={vx} m/s，持续 {duration} s")
    print("（状态帧里的电机在线掩码应为 0x0F，即 1~4 号电机都在线）")

    # 1) 使能。C8T6 收到 0->1 边沿后会给 4 个轮电机各发一帧使能命令。
    #    重复发几次，防止电机上电慢漏收；命令幂等，多收无副作用。
    for _ in range(3):
        link.send_enable(True)
        time.sleep(0.02)

    # 2) 持续发速度 + 心跳，保持 C8T6 认为树莓派在线
    t0 = time.monotonic()
    last_hb = 0.0
    last_print = 0.0
    try:
        while time.monotonic() - t0 < duration:
            link.send_vel(vx, 0.0, 0.0)

            now = time.monotonic()
            if now - last_hb >= 0.2:
                last_hb = now
                link.send_heartbeat()

            for cmd, data in link.poll():
                if cmd == RPT_STATUS and now - last_print >= 0.5:
                    last_print = now
                    state, err, mask = parse_status(data)
                    print(f"  状态={state} 错误=0x{err:02X} "
                          f"电机在线掩码=0x{mask:02X}")

            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C，提前停车")

    # 3) 停车：连发几帧 0 速度让电机平滑减速，然后失能
    for _ in range(5):
        link.send_vel(0.0, 0.0, 0.0)
        link.send_heartbeat()
        time.sleep(0.02)
    link.send_enable(False)
    link.close()
    print("已发送停车/失能")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
