#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""无线链路自检：不启动 ROS，直接验证「蓝牙上行 + LoRa 下行」是否打通。

用法（不需要 source ROS 环境）：

    # 只看 F407 有没有通过 LoRa 回传（不发任何指令）
    python3 scripts/test_wireless.py --lora /dev/ttyAMA2

    # 指定蓝牙模组地址，持续发心跳（让 F407 保持在线），并打印回传
    python3 scripts/test_wireless.py --ble-address AA:BB:CC:DD:EE:FF

    # 再发一个速度，看车轮动不动（0.2 m/s 前进，0 平移，0 自转）
    python3 scripts/test_wireless.py --ble-address AA:BB:CC:DD:EE:FF --vel 0.2 0 0

屏幕上会持续打印：
    [OK ] odom x=... y=... yaw=...
    [OK ] status state=1 err=0 motor=0x7F
    ---- 2.0s 内没收到任何回传：LoRa 有问题 ----

退出用 Ctrl-C。Ctrl-C 之前不会主动让车停 —— 但如果直接关掉本脚本，
F407 500ms 收不到心跳就会自主失能，车会自动停，这是安全设计。
"""

import argparse
import os
import sys
import time

# 让脚本在没 colcon build 过的情况下也能 import 到包里的 pi_comm
_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.join(_HERE, "..", "ros2_ws", "src", "f407_bridge")
if os.path.isdir(_PKG):
    sys.path.insert(0, _PKG)

from f407_bridge.pi_comm import (      # noqa: E402
    BleLink, PiCommLink, RPT_ODOM, RPT_STATUS, parse_odom, parse_status,
)


def main() -> None:
    parser = argparse.ArgumentParser(description="蓝牙上行 + LoRa 下行 链路自检")
    parser.add_argument("--lora", default="/dev/ttyAMA2", help="LoRa 串口设备")
    parser.add_argument("--baud", type=int, default=115200, help="LoRa 串口波特率")
    parser.add_argument("--ble-address", default="", help="蓝牙模组 MAC（留空=自动扫描）")
    parser.add_argument("--ble-name", default="", help="蓝牙模组名字关键字（可选）")
    parser.add_argument("--write-uuid", default="", help="写入特征值 UUID（留空=自动探测）")
    parser.add_argument("--vel", nargs=3, type=float, metavar=("VX", "VY", "WZ"),
                        help="持续下发速度 m/s, m/s, rad/s（不填就只发心跳）")
    parser.add_argument("--no-ble", action="store_true", help="只收 LoRa，不连蓝牙")
    args = parser.parse_args()

    down = PiCommLink(device=args.lora, baudrate=args.baud)
    up = None
    if not args.no_ble:
        try:
            up = BleLink(address=args.ble_address, name=args.ble_name,
                         write_uuid=args.write_uuid, log=print)
            print("蓝牙后台线程已启动，正在连接/扫描（可能要几秒）...")
        except Exception as error:     # noqa: BLE001
            print(f"[!] 蓝牙不可用：{error}")
            up = None

    print(f"LoRa 收：{args.lora} @ {args.baud}")
    print("Ctrl-C 退出\n")

    last_rx = 0.0
    last_hb = 0.0
    last_vel = 0.0
    last_warn = 0.0
    enable_sent = 0
    try:
        while True:
            now = time.monotonic()

            # ---- 收 LoRa ----
            for cmd, data in down.poll():
                last_rx = now
                if cmd == RPT_ODOM:
                    x, y, yaw = parse_odom(data)
                    print(f"[OK ] odom   x={x:+.3f}m y={y:+.3f}m yaw={yaw:+.3f}rad")
                elif cmd == RPT_STATUS:
                    state, err, mask = parse_status(data)
                    print(f"[OK ] status state={state} err={err} motor=0x{mask:02X}")
                else:
                    print(f"[OK ] cmd=0x{cmd:02X} data={data.hex()}")

            # ---- 发蓝牙 ----
            if up is not None:
                if now - last_hb >= 0.2:            # 心跳 5Hz，快过 F407 的 500ms 失能判定
                    last_hb = now
                    if not up.connected and up.last_scan_report:
                        print("[..] 还没连上蓝牙，扫到的设备：")
                        print(up.last_scan_report)
                    up.send_heartbeat()
                # 连上后先补发几次使能：F407 必须 EnableMask 全置位才驱动轮子，
                # 只发速度不发使能，轮子是不会动的（命令幂等，多收无副作用）。
                if args.vel and up.connected and enable_sent < 3:
                    up.send_enable(True)
                    enable_sent += 1
                if args.vel and now - last_vel >= 0.1:
                    last_vel = now
                    up.send_vel(args.vel[0], args.vel[1], args.vel[2])

            # ---- 断流提示（每 2 秒一次，避免刷屏） ----
            if now - last_rx > 2.0 and now - last_warn > 2.0:
                last_warn = now
                print("---- 2.0s 内没收到任何回传：检查 LoRa 是否上电/波特率/接线 ----")

            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n退出")
    finally:
        if up is not None:
            if args.vel:
                try:
                    up.send_vel(0.0, 0.0, 0.0)   # 先停车
                    up.send_enable(False)        # 再失能
                except Exception:
                    pass
            up.close()
        down.close()


if __name__ == "__main__":
    main()
