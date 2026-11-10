#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""color_pick / vision_pick 的**测试桩**：假装是转盘视觉程序。

不打开摄像头，按命令行参数直接打印固定的抓取位姿 JSON，用于联调前期验证
"motion_planner → 云台/夹爪" 整条机械链路（不等视觉程序开发）。
真实视觉程序写好后，把 motion_planner 的 vision_program/vision_args
参数指到真程序即可，链路其它部分一行不用改。

手动运行示例：
    python3 vision/test_pick_stub.py --color 4 --yaw 0 --lift -40 --extend 150

输出：先打一行日志（motion_planner 只认**最后一行** JSON）：
    {"yaw_deg": 0.0, "lift_deg": -40.0, "extend_deg": 150.0}
"""

import argparse
import json
import time


def main():
    parser = argparse.ArgumentParser(description='抓取视觉测试桩')
    parser.add_argument('--color', type=int, default=1,
                        help='物料颜色编号（1红 2黄 3蓝 4绿 5黑 6浅蓝），桩只打印日志')
    parser.add_argument('--yaw', type=float, default=0.0, help='抓取 yaw 角°')
    parser.add_argument('--lift', type=float, default=-40.0, help='下降 lift 角°（负=降）')
    parser.add_argument('--extend', type=float, default=150.0, help='伸出 extend 角°')
    parser.add_argument('--delay', type=float, default=1.0,
                        help='模拟视觉识别耗时（秒），顺便观察状态机等待过程')
    args = parser.parse_args()

    print(f'[test_pick_stub] 假装在找颜色 {args.color} 的物料…', flush=True)
    time.sleep(max(0.0, args.delay))
    print(json.dumps({
        'yaw_deg': args.yaw,
        'lift_deg': args.lift,
        'extend_deg': args.extend,
    }), flush=True)


if __name__ == '__main__':
    main()
