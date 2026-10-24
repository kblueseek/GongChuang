#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""摇杆连续性探针：直接读 /dev/input/js0，统计每个轴出现过多少种不同取值。

用途：判断手柄当前模式下的摇杆是「连续模拟量」还是「只有几档」。
用法：
    python3 scripts/joy_probe.py
    python3 scripts/joy_probe.py --seconds 12

跑起来后按提示**慢慢**把左摇杆推到底、回中，再推右摇杆。
结束时每个轴会打印：不同取值个数 / 取值范围 / 最常见取值，并给出判断：

    不同取值 >= 16   连续（正常模拟摇杆）
    不同取值 9~15    有点粗，可能仍有死区或步进
    不同取值 <= 8    分档（不连续，红灯模式实测如此）

切换模式：长按手柄 [MODE] 键 5 秒，指示灯由红变绿。
这台手柄绿灯/红灯都是 6 轴、设备名都是 TGZ Controller、编号也一样，
区别只是绿灯摇杆连续、红灯分档；若设备名变成
"Microsoft X-Box 360 pad"（8 轴），说明换了另一套编号，需重新跑 joy_test.py。
"""

import argparse
import collections
import os
import select
import struct
import sys
import time

JS_EVENT = struct.Struct("<IhBB")   # time(u32), value(s16), type(u8), number(u8)
EVENT_AXIS = 0x02
AXIS_SCALE = 32767.0


def device_name(path):
    """从 /dev/input/jsN 找到对应的内核设备名。"""
    try:
        node = os.path.basename(os.path.realpath(path))       # js0
        with open(f"/sys/class/input/{node}/device/name") as f:
            return f.read().strip()
    except OSError:
        return "?"


def main():
    ap = argparse.ArgumentParser(description="摇杆连续性探针")
    ap.add_argument("--device", default="/dev/input/js0")
    ap.add_argument("--seconds", type=float, default=12.0)
    args = ap.parse_args()

    name = device_name(args.device)
    try:
        fd = os.open(args.device, os.O_RDONLY | os.O_NONBLOCK)
    except OSError as e:
        print(f"打不开 {args.device}: {e}")
        print("先确认手柄插好：ls -l /dev/input/js*")
        return 1

    # 吃掉内核开机补发的初始事件
    while select.select([fd], [], [], 0)[0]:
        os.read(fd, 8)

    print("=" * 58)
    print(f" 设备：{args.device}  内核名：{name}")
    print(f" 采样 {args.seconds:.0f} 秒。请**慢慢**做：")
    print("   1) 左摇杆从中间推到最上，慢慢回中，来回 2 次")
    print("   2) 左摇杆推到最左、最右，各一次")
    print("   3) 右摇杆上下、左右各推一次")
    print("=" * 58)

    counts = collections.defaultdict(collections.Counter)
    latest = {}
    end = time.monotonic() + args.seconds
    last_draw = 0.0

    while time.monotonic() < end:
        r, _, _ = select.select([fd], [], [], 0.1)
        if not r:
            continue
        data = os.read(fd, 8)
        if len(data) != 8:
            continue
        _t, value, typ, num = JS_EVENT.unpack(data)
        if (typ & 0x7F) != EVENT_AXIS:
            continue
        counts[num][value] += 1
        latest[num] = value

        now = time.monotonic()
        if now - last_draw >= 0.15:
            last_draw = now
            shown = " ".join(f"a{n}={v / AXIS_SCALE:+.2f}"
                             for n, v in sorted(latest.items()))
            print(f"\r  实时轴值: {shown:<60}", end="", flush=True)

    os.close(fd)
    print("\r" + " " * 78 + "\r", end="")

    if not counts:
        print("没读到任何轴事件：是不是没推摇杆？")
        return 2

    print(f"{'轴':<4}{'采样数':>8}{'不同取值':>8}{'范围':>18}  判断")
    print("-" * 58)
    for num in sorted(counts):
        c = counts[num]
        values = sorted(c)
        total = sum(c.values())
        distinct = len(values)
        lo, hi = values[0], values[-1]
        if distinct < 2:
            verdict = "完全没动"
        elif distinct <= 8:
            verdict = "分档（不连续）"
        elif distinct <= 15:
            verdict = "偏粗"
        else:
            verdict = "连续"
        common = " ".join(f"{v / AXIS_SCALE:+.2f}x{n}" for v, n in c.most_common(4))
        print(f"a{num:<3}{total:>8}{distinct:>8}"
              f"{lo / AXIS_SCALE:>+8.2f} ~ {hi / AXIS_SCALE:<+7.2f}  {verdict}")
        print(f"     最常见: {common}")

    print()
    print("提示：切绿灯模式后再跑一遍本脚本对比；若仍是分档，把这段输出贴回来。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
