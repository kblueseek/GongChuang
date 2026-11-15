#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""扫码模块串口探测：挨个试常见「波特率 × 校验位」，报出哪种能收到可读 ASCII。

为什么需要它：模块手册上写 115200，实际可能是 9600（出厂默认），接错了串口
收到的就是一串 0x00 —— 把 9600 的数据按 115200 采样，起始位会被切成一个又
一个零字节。看不出是哪一边错了，只能一个个试。

用法（**先 Ctrl+C 停掉 run_car.sh**，否则 qr_scanner 和本脚本抢同一个串口）：

    python3 scripts/qr_baud_probe.py                    # 默认 /dev/ttyAMA2
    python3 scripts/qr_baud_probe.py -d /dev/ttyAMA3    # 换口
    python3 scripts/qr_baud_probe.py -t 4               # 每个组合试 4 秒

测试期间把二维码一直放在模块前面（模块一般只在有码时才发数据）。
找到能用的组合后，把它写进 car_bringup/config/vehicle.yaml 的 qr_scanner 段：

    device: '/dev/ttyAMA2'
    baudrate: 9600

⚠ Pi5 的 UART 编号和 GPIO 不是直觉对应（/boot/firmware/current/overlays/README）：
    uart2-pi5 → GPIO 4/5   → /dev/ttyAMA2
    uart3-pi5 → GPIO 8/9   → /dev/ttyAMA3（config.txt 要另加 dtoverlay=uart3-pi5）
    uart4-pi5 → GPIO 12/13 → /dev/ttyAMA4
    GPIO 14/15 → /dev/ttyAMA0（F407，别碰）
"""

from __future__ import annotations

import argparse
import re
import sys
import time

try:
    import serial
except ImportError:
    raise SystemExit('需要 pyserial：sudo apt install python3-serial')

# 任务码形态（和 qr_scanner_node 里一致）
_TASK_CODE_RE = re.compile(r'\d{3}\+\d{3}\+\d{3}\+\d{3}')

# 常见组合：波特率先试最可能的，校验位放最后（多数模块是 8N1）
_COMBOS = []
for baud in (115200, 9600, 19200, 38400, 57600, 230400):
    _COMBOS.append((baud, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE))
for baud in (115200, 9600):
    for parity, name in ((serial.PARITY_EVEN, 'E'), (serial.PARITY_ODD, 'O')):
        _COMBOS.append((baud, serial.EIGHTBITS, parity, serial.STOPBITS_ONE))
    _COMBOS.append((baud, serial.SEVENBITS, serial.PARITY_EVEN, serial.STOPBITS_ONE))


def probe(port: serial.Serial, seconds: float) -> tuple[int, bytes]:
    """在已打开的串口上收 seconds 秒，返回 (字节数, 原始数据)。"""
    buf = bytearray()
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        data = port.read(4096)
        if data:
            buf.extend(data)
            if len(buf) > 4096:          # 只留最近一段，够判断了
                del buf[:-4096]
    return len(buf), bytes(buf)


def main() -> int:
    parser = argparse.ArgumentParser(description='扫码模块波特率探测')
    parser.add_argument('-d', '--device', default='/dev/ttyAMA2')
    parser.add_argument('-t', '--seconds', type=float, default=2.5,
                        help='每个组合监听多少秒（默认 2.5）')
    parser.add_argument('--all', action='store_true',
                        help='全部组合都试完再报告（默认一找到可读数据就停）')
    args = parser.parse_args()

    print(f'设备 {args.device}，每个组合 {args.seconds}s。'
          '请把二维码一直放在模块前面…\n')
    print(f'{"波特率":>7}  {"格式":>4}  {"字节":>5}  {"可读率":>6}  样本')
    print('-' * 72)

    best = None
    for baud, bits, parity, stop in _COMBOS:
        # pyserial 的 parity 常量本身就是字符串 'N'/'E'/'O'，直接拼
        fmt = f'{bits}{parity}1'
        try:
            port = serial.Serial(args.device, baud, bytesize=bits,
                                 parity=parity, stopbits=stop, timeout=0.1)
        except Exception as error:  # noqa: BLE001
            print(f'打不开 {args.device}：{error}')
            return 1
        try:
            n, raw = probe(port, args.seconds)
        finally:
            port.close()

        printable = sum(1 for b in raw if 32 <= b < 127)
        ratio = printable / n if n else 0.0
        sample = raw[:32]
        text = ''.join(chr(b) if 32 <= b < 127 else '.' for b in raw[:40])
        print(f'{baud:>7}  {fmt:>4}  {n:>5}  {ratio:>6.0%}  {text!r}')

        if n and (ratio >= 0.8 or _TASK_CODE_RE.search(raw.decode('ascii', 'ignore'))):
            best = (baud, fmt)
            print('  ^^^ 这个组合能收到可读 ASCII')
            if not args.all:
                break

    print()
    if best:
        print(f'✅ 用这种配置：baudrate {best[0]}，数据格式 {best[1]}')
        print('   把 vehicle.yaml 的 qr_scanner.baudrate 改成 '
              f'{best[0]}，重启 run_car.sh')
        return 0
    print('❌ 所有组合都没收到可读数据。按顺序查：')
    print('   1) 模块 TX 是否接到 Pi 的 RX（GPIO15/RX 脚），共地了吗')
    print('   2) 二维码有没有一直被模块看到（多数模块只在识别成功时发）')
    print(f'   3) 设备是不是真的存在：ls /dev/ttyAMA*，Pi5 的 uart3-pi5=GPIO8/9')
    print('   4) 模块是不是 USB 键盘模式（插 USB 就直接打字那种），'
          '要改回串口/韦根模式')
    return 1


if __name__ == '__main__':
    sys.exit(main())
