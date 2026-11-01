#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""蓝牙调试小工具：扫描附近的 BLE 设备，找出 F407 上那块蓝牙模组。

用法（不需要 source ROS 环境，直接 python3 跑）：

    python3 scripts/ble_scan.py                    # 只扫描，列出地址/名字/广播服务
    python3 scripts/ble_scan.py AA:BB:CC:DD:EE:FF  # 扫描并连接该设备，列出可写特征值

为什么需要它：
  桥接节点自动扫描时，如果附近不止一台 BLE 设备，它没法替你决定连哪个。
  先用这个脚本看清"F407 的蓝牙叫什么、地址是多少"，再填进参数就不会连错：

    ros2 run f407_bridge f407_bridge_node --ros-args -p ble_address:=AA:BB:CC:DD:EE:FF

  NUS / FFE1 这些名字是串口透传模组的常见"服务/特征值"，脚本会把可写的那几个标出来。
"""

import asyncio
import os
import sys

# 让脚本在没 colcon build 过的情况下也能 import 到包里的 pi_comm
_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.join(_HERE, "..", "ros2_ws", "src", "f407_bridge")
if os.path.isdir(_PKG):
    sys.path.insert(0, _PKG)

try:
    from f407_bridge.pi_comm import (          # noqa: E402
        BleakClient, scan_ble_devices,
    )
except Exception as error:                     # noqa: BLE001
    print(f"导入 pi_comm 失败：{error}")
    sys.exit(1)


def find_serial_looking(name, service_uuids):
    """粗略判断这台设备像不像串口透传模组（按名字/服务 UUID 猜）。"""
    marks = ("6e400001", "0000ffe0", "0000ffe1", "0000fff0")
    for uuid in service_uuids:
        for mark in marks:
            if mark in uuid.lower():
                return True
    return any(k in (name or "").lower() for k in ("jdy", "ble", "hc-", "uart", "serial", "at-"))


async def dump_characteristics(address: str) -> None:
    """连上设备，把每个服务的每个特征值打印出来，标出可写的。"""
    print(f"\n正在连接 {address} ...")
    async with BleakClient(address, timeout=10.0) as client:
        print("已连接。服务/特征值如下（W=可写，N=可通知）：")
        for service in client.services:
            print(f"  服务 {service.uuid}")
            for char in service.characteristics:
                props = [p.lower() for p in char.properties]
                flags = ""
                if "write" in props or "write-without-response" in props:
                    flags += "W "
                if "notify" in props or "indicate" in props:
                    flags += "N "
                print(f"    {flags:4s} {char.uuid}  {props}")
        print("\n把可写（W）的那个 UUID 填到桥接节点：")
        print("  ros2 run f407_bridge f407_bridge_node --ros-args \\")
        print(f"    -p ble_address:={address} -p ble_write_uuid:=<上面带W的UUID>")


def main() -> None:
    address = sys.argv[1] if len(sys.argv) > 1 else ""
    print("扫描 BLE 设备（约 8 秒，请让 F407 上电）...")
    try:
        devices = scan_ble_devices(timeout=8.0)
    except Exception as error:                 # noqa: BLE001
        print(f"扫描失败：{error}")
        print("排查：sudo systemctl status bluetooth  /  bluetoothctl show 看 Powered 是否为 yes")
        sys.exit(1)

    if not devices:
        print("没扫到任何 BLE 设备。检查：树莓派蓝牙是否打开、模组是否上电、距离是否太远。")
        return

    print(f"\n共 {len(devices)} 台：")
    for dev_address, name, uuids in sorted(devices, key=lambda d: d[1] or "~"):
        hint = "  <== 可能是串口透传模组" if find_serial_looking(name, uuids) else ""
        print(f"  {dev_address:18s} {name or '(无名)':20s} {','.join(u for u in uuids[:3])}{hint}")

    print("\n如果已经知道哪台是 F407 的模组，把它用 -p ble_address:=<地址> 传给桥接节点。")
    if address:
        asyncio.run(dump_characteristics(address))


if __name__ == "__main__":
    main()
