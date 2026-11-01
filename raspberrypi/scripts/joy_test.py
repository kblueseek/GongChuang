#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""手柄按键/轴逐项自检（直接读 /dev/input/js0，编号和 ROS /joy 完全一致）。

用法：
    python3 scripts/joy_test.py

运行时屏幕会提示"请按一下 XXX"，你照做即可。每项：
  - 检测到就打印真实编号，并和内核能力表对比，标 ✓ / ✗
  - 想跳过某一项，直接按 Enter
最后打印一张汇总表，把它贴回来即可。

注意：
  1. 请先停掉 scripts/run_car.sh，否则按键会同时触发 joy_control，可能让车动。
  2. 手柄有两套编号，脚本按轴数量自动识别，并打印用的是哪一套：
       - 6 轴（内核名 TGZ Controller）：L2/R2 是按键 6/7
         实测红灯/绿灯都是这套编号，区别只是绿灯摇杆连续、红灯分档
       - 8 轴（内核名 Microsoft X-Box 360 pad）：L2/R2 是扳机轴 2/5
     两套的编号完全不同，测之前先看一眼脚本打印的模式。
     开车请把指示灯切绿（长按 MODE 5 秒），摇杆才是连续模拟量。
"""

import argparse
import glob
import os
import select
import struct
import sys
import time

JS_EVENT = struct.Struct("<IhBB")   # time(u32), value(s16), type(u8), number(u8)


def device_name(path):
    """从 /dev/input/jsN 找到对应的内核设备名，贴结果时能区分红灯/绿灯。"""
    try:
        node = os.path.basename(os.path.realpath(path))
        with open(f"/sys/class/input/{node}/device/name") as f:
            return f.read().strip()
    except OSError:
        return "?"


EVENT_BUTTON = 0x01
EVENT_AXIS = 0x02
AXIS_SCALE = 32767.0
AXIS_DELTA = 0.6                    # 判定"明显动了"的阈值

# (类型, 提示, 期望编号, 期望符号)
#
# PS2 模式：6 轴，内核名 "TGZ Controller"（VID:PID 0079:0126）
#   轴 0/1=左摇杆，2/3=右摇杆，4/5=方向键；L2/R2 是按键 6/7
#   键（绿灯下逐项实测）：Y0 B1 A2 X3 L1=4 R1=5 L2=6 R2=7 select=8 start=9
#   面键顺序是 Y/B/A/X，不是常见的 A/B/X/Y；剩下 3 个键在 10~12，当前不用。
PS2_STEPS = [
    ("button", "按一下  A  键", 2, None),
    ("button", "按一下  B  键", 1, None),
    ("button", "按一下  X  键", 3, None),
    ("button", "按一下  Y  键", 0, None),
    ("button", "按一下  L1（左肩键）", 4, None),
    ("button", "按一下  R1（右肩键）", 5, None),
    ("button", "按一下  L2（左扳机）", 6, None),
    ("button", "按一下  R2（右扳机）", 7, None),
    ("button", "按一下  select（Back）", 8, None),
    ("button", "按一下  start", 9, None),
    ("axis", "左摇杆推到最上，然后回中", 1, -1),
    ("axis", "左摇杆推到最下，然后回中", 1, +1),
    ("axis", "左摇杆推到最左，然后回中", 0, -1),
    ("axis", "左摇杆推到最右，然后回中", 0, +1),
    ("axis", "右摇杆推到最上，然后回中", 3, -1),
    ("axis", "右摇杆推到最下，然后回中", 3, +1),
    ("axis", "右摇杆推到最左，然后回中", 2, -1),
    ("axis", "右摇杆推到最右，然后回中", 2, +1),
    ("axis", "方向键 按上，然后松开", 5, -1),
    ("axis", "方向键 按下，然后松开", 5, +1),
    ("axis", "方向键 按左，然后松开", 4, -1),
    ("axis", "方向键 按右，然后松开", 4, +1),
]

# Xbox 模式：8 轴，内核名 "Microsoft X-Box 360 pad"
#   轴 0/1=左摇杆，3/4=右摇杆，2/5=扳机，6/7=方向键
#   键 A0 B1 X2 Y3 L1=4 R1=5 select=6 start=7 mode=8
XBOX_STEPS = [
    ("button", "按一下  A  键", 0, None),
    ("button", "按一下  B  键", 1, None),
    ("button", "按一下  X  键", 2, None),
    ("button", "按一下  Y  键", 3, None),
    ("button", "按一下  L1（左肩键）", 4, None),
    ("button", "按一下  R1（右肩键）", 5, None),
    ("button", "按一下  select（Back）", 6, None),
    ("button", "按一下  start", 7, None),
    ("axis", "把 L2（左扳机）按到底，然后松开", 2, +1),
    ("axis", "把 R2（右扳机）按到底，然后松开", 5, +1),
    ("axis", "左摇杆推到最上，然后回中", 1, -1),
    ("axis", "左摇杆推到最下，然后回中", 1, +1),
    ("axis", "左摇杆推到最左，然后回中", 0, -1),
    ("axis", "左摇杆推到最右，然后回中", 0, +1),
    ("axis", "右摇杆推到最上，然后回中", 4, -1),
    ("axis", "右摇杆推到最下，然后回中", 4, +1),
    ("axis", "右摇杆推到最左，然后回中", 3, -1),
    ("axis", "右摇杆推到最右，然后回中", 3, +1),
    ("axis", "方向键 按上，然后松开", 7, -1),
    ("axis", "方向键 按下，然后松开", 7, +1),
    ("axis", "方向键 按左，然后松开", 6, -1),
    ("axis", "方向键 按右，然后松开", 6, +1),
]


class JoyReader:
    def __init__(self, path):
        try:
            self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        except OSError as e:
            print(f"打不开 {path}: {e}")
            sys.exit(1)
        self.axes = {}
        self.buttons = {}
        self._drain()

    def _drain(self):
        """把内核开机时补发的初始状态事件读掉。"""
        while True:
            r, _, _ = select.select([self.fd], [], [], 0)
            if not r:
                return
            if self._read() is None:
                return

    def _read(self):
        try:
            data = os.read(self.fd, 8)
        except BlockingIOError:
            return None
        if len(data) != 8:
            return None
        _t, v, typ, num = JS_EVENT.unpack(data)
        typed = typ & 0x7F
        if typed == EVENT_BUTTON:
            self.buttons[num] = 1 if v else 0
            return (EVENT_BUTTON, 1.0 if v else 0.0, num)
        if typed == EVENT_AXIS:
            val = max(-1.0, min(1.0, v / AXIS_SCALE))
            self.axes[num] = val
            return (EVENT_AXIS, val, num)
        return None

    def wait_change(self, kind, baseline, timeout, allow_skip=True):
        """等一个明显变化。返回 (num, sign) 或 'skip' 或 None(超时)。"""
        # 只有真终端才把 stdin 放进 select：管道/重定向时 stdin 永远可读，
        # 会被误判成"用户按了 Enter"而把每一项都跳过。
        allow_skip = allow_skip and sys.stdin.isatty()
        end = time.monotonic() + timeout
        fds = [self.fd] + ([sys.stdin] if allow_skip else [])
        while time.monotonic() < end:
            r, _, _ = select.select(fds, [], [], 0.1)
            if allow_skip and sys.stdin in r:
                try:
                    sys.stdin.readline()
                except Exception:
                    pass
                return "skip"
            if self.fd not in r:
                continue
            ev = self._read()
            if ev is None:
                continue
            typ, val, num = ev
            if kind == "button":
                if typ == EVENT_BUTTON and val == 1.0:
                    return (num, +1)
            else:
                if typ != EVENT_AXIS:
                    continue
                delta = val - baseline.get(num, 0.0)
                if abs(delta) >= AXIS_DELTA:
                    return (num, 1 if delta > 0 else -1)
        return None

    def wait_return(self, kind, num, baseline_val, timeout=8.0):
        """等按键松开 / 轴回中，避免下一项被残留状态干扰。"""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            r, _, _ = select.select([self.fd], [], [], 0.1)
            if self.fd not in r:
                continue
            ev = self._read()
            if ev is None:
                continue
            typ, val, n = ev
            if kind == "button":
                if typ == EVENT_BUTTON and n == num and val == 0.0:
                    return
            elif typ == EVENT_AXIS and n == num and abs(val - baseline_val) < 0.3:
                return

    def close(self):
        try:
            os.close(self.fd)
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser(description="手柄按键/轴逐项自检")
    ap.add_argument("--device", default="auto",
                    help="手柄设备；auto=自动找第一个 /dev/input/js*")
    ap.add_argument("--timeout", type=float, default=30.0, help="每项等待秒数")
    args = ap.parse_args()

    if args.device == "auto":
        found = sorted(glob.glob("/dev/input/js*"))
        if not found:
            print("没找到 /dev/input/js*：手柄插好了吗？")
            sys.exit(1)
        args.device = found[0]
        if len(found) > 1:
            print(f"发现多个手柄设备 {found}，先用 {args.device}（可用 --device 指定）")

    reader = JoyReader(args.device)
    n_axes = len(reader.axes)
    if n_axes <= 6:
        mode, steps = "PS2（6 轴，TGZ Controller）", PS2_STEPS
    else:
        mode, steps = "Xbox（8 轴）", XBOX_STEPS

    print("=" * 54)
    print(" 手柄逐项自检（直接读 js0，编号 = ROS /joy 下标）")
    print(f" 设备名：{device_name(args.device)}")
    print(f" 检测到模式：{mode}，{n_axes} 个轴 / {len(reader.buttons)} 个按键")
    print(" 提示按什么就按什么；想跳过某项，直接按 Enter。")
    print(" ★ 请先停掉 run_car.sh，否则按键会同时触发 joy_control。")
    print("=" * 54)

    results = []
    total = len(steps)
    for i, (kind, prompt, exp_num, exp_sign) in enumerate(steps, 1):
        reader._drain()                       # 先吃掉积压事件，拿到最新基线
        baseline = dict(reader.axes) if kind == "axis" else {}

        print()
        print(f"----- 第 {i}/{total} 项 -----")
        print(f"  ➜  {prompt}")

        res = reader.wait_change(kind, baseline, args.timeout)
        if res == "skip":
            print("      （已跳过）")
            results.append((kind, prompt, None, None, exp_num, exp_sign, False))
            continue
        if res is None:
            print(f"      （{args.timeout:.0f} 秒没检测到，跳过）")
            results.append((kind, prompt, None, None, exp_num, exp_sign, False))
            continue

        num, sign = res
        if kind == "button":
            reader.wait_return("button", num, 0.0)
            ok = (num == exp_num)
            print(f"      ✓ 检测到 button {num}" if ok
                  else f"      ✗ 检测到 button {num}（预期 {exp_num}）")
        else:
            base = baseline.get(num, 0.0)
            reader.wait_return("axis", num, base)
            sign_txt = "+" if sign > 0 else "-"
            exp_txt = "+" if exp_sign > 0 else "-"
            ok = (num == exp_num and sign == exp_sign)
            print(f"      ✓ 检测到 axis {num}（{sign_txt}）" if ok
                  else f"      ✗ 检测到 axis {num}（{sign_txt}）"
                       f"（预期 axis {exp_num}（{exp_txt}））")
        results.append((kind, prompt, num, sign, exp_num, exp_sign, ok))

    reader.close()

    print()
    print("=" * 54)
    print(" 汇总（把下面这段贴回来）")
    print("=" * 54)
    passed = 0
    skipped = 0
    for kind, prompt, num, sign, exp_num, exp_sign, ok in results:
        if num is None:
            skipped += 1
            print(f"  [跳过] {prompt}")
            continue
        if ok:
            passed += 1
        if kind == "button":
            tail = "✓" if ok else f"✗（预期 {exp_num}）"
            print(f"  {prompt:<30} -> button {num:<2} {tail}")
        else:
            st = "+" if sign > 0 else "-"
            tail = "✓" if ok else f"✗（预期 {exp_num}）"
            print(f"  {prompt:<30} -> axis {num} ({st}) {tail}")
    print(f"\n  符合预期 {passed}/{total - skipped}（跳过 {skipped} 项）")
    print("  若有 ✗，把上面的真实编号贴回来，我改参数即可，不用改代码。")


if __name__ == "__main__":
    main()
