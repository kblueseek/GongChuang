#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""串口探针：不开界面，直接连雷达试着收 2 秒数据，告诉你到底通不通。

跑法：
    python port_probe.py                  # 自动找 CH340/CP210x 那个口，230400
    python port_probe.py COM22            # 指定口
    python port_probe.py COM22 115200     # 指定口和波特率

输出：收到多少帧、每帧几个点、解码出来的距离范围。
这就是"真机到底行不行"的最快验证方式，比开界面看状态栏直观。
"""

from __future__ import annotations

import sys
import time

import lidar_d6


def pick_port():
    try:
        from serial.tools import list_ports
    except Exception:
        return None, []
    ports = list(list_ports.comports())
    for p in ports:
        d = (p.description or "").upper()
        if any(k in d for k in ("CH340", "CP210", "USB-SERIAL", "FTDI", "SILICON")):
            return p.device, ports
    return (ports[0].device if ports else None), ports


def main():
    argv = [a for a in sys.argv[1:]]
    port = argv[0] if argv else None
    baud = int(argv[1]) if len(argv) > 1 else 230400

    auto, ports = pick_port()
    if ports:
        print("找到这些串口：")
        for p in ports:
            print(f"   {p.device:8s} {p.description}")
    else:
        print("一个串口都没找到（没插？驱动没装？看设备管理器）")
        return 1

    if not port:
        port = auto
        if not port:
            print("没有可用的串口")
            return 1

    print(f"\n连 {port} @ {baud} ...")
    frames = {"n": 0, "pts": 0, "dmin": 1e9, "dmax": 0, "rings": 0, "sample": []}

    def on_frame(pts, ring):
        frames["n"] += 1
        frames["pts"] += len(pts)
        if ring:
            frames["rings"] += 1
        for _, d, _ in pts:
            frames["dmin"] = min(frames["dmin"], d)
            frames["dmax"] = max(frames["dmax"], d)
        if len(frames["sample"]) < 5 and pts:
            frames["sample"].append(pts[0])

    lid = lidar_d6.D6Lidar(port, baud, on_frame=on_frame,
                           on_log=lambda s: print("  [驱动]", s))
    if not lid.open():
        print("打不开：", lid.error)
        print("\n常见原因：")
        print("  1. 串口被占了 —— JCom / 塔克上位机 / 别的脚本还开着，先关掉")
        print("  2. 这个口不是雷达（比如是别的 USB 转串口设备）")
        print("  3. 驱动没装 —— 资料包里有 CH340 驱动")
        return 2

    t0 = time.time()
    while time.time() - t0 < 2.0:
        time.sleep(0.1)
        if frames["n"] > 0 and time.time() - t0 > 1.0:
            break
    lid.close()

    print(f"\n2 秒结果：帧 {frames['n']}，点 {frames['pts']}，整圈标记 {frames['rings']}")
    if frames["n"] == 0:
        print("\n❌ 一个数据帧都没收到。按顺序排查：")
        print("  1. 波特率试 115200 / 153600 / 460800（D6 出厂 230400）")
        print("  2. 雷达有没有转？（电机靠 DTR 启动，线序/供电）")
        print("  3. 换根 USB 线，换个 USB 口")
        print("  4. 用塔克那个 cspc 上位机先确认雷达本身是好的")
        return 3

    print(f"✅ 通了。距离范围 {frames['dmin']:.0f} ~ {frames['dmax']:.0f} mm")
    print("   前几个点（角度, 距离mm, 强度）：")
    for p in frames["sample"]:
        print(f"     {p[0]:7.2f}°, {p[1]:7.0f} mm, {p[2]}")
    if frames["dmax"] < 100:
        print("   ⚠ 距离都很小，可能解错了——核对协议版本")
    return 0


if __name__ == "__main__":
    sys.exit(main())
