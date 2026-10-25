#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""自检 ②：真的把界面跑起来，走一遍"模拟雷达 → 扫描 → 识别 → 规划"，然后截图。

跑法：  python shot_app.py
输出：  app_shot.png

这个脚本是给我（或者你）验证界面没写崩用的。它会自动关窗，不会留进程。
"""

import os
import sys
import time

import tkinter as tk

import app as app_mod
from PIL import ImageGrab

HERE = os.path.dirname(os.path.abspath(__file__))


def pump(root, seconds):
    t0 = time.time()
    while time.time() - t0 < seconds:
        root.update()
        time.sleep(0.02)


def main():
    root = tk.Tk()
    root.geometry("1620x1000+0+0")
    gui = app_mod.App(root, os.path.join(HERE, "field.json"))
    pump(root, 0.5)

    # 1) 连雷达：带 --real 就走真串口
    if "--real" not in sys.argv:
        gui.var_sim.set(True)
    else:
        gui.var_sim.set(False)
        print("串口:", gui.cb_port.get())
    gui.toggle_lidar()
    pump(root, 0.4)

    # 2) 扫描建图（5 圈 @10Hz ≈ 0.5 秒）
    gui.start_scan()
    t0 = time.time()
    while gui.scanning and time.time() - t0 < 8:
        root.update()
        time.sleep(0.02)

    # 3) 全程任务路线
    gui.plan_mission()
    pump(root, 0.6)

    # 模拟鼠标悬停在某点，展示坐标提示
    cx, cy = gui._w2p(300, 300)
    gui._draw_hover(int(cx), int(cy))
    pump(root, 0.3)

    # 把窗口顶到最前，再按窗口矩形截图（DPI 缩放下要换算物理像素）
    root.lift()
    root.attributes("-topmost", True)
    pump(root, 0.6)
    full = ImageGrab.grab()
    sc = full.width / root.winfo_screenwidth()      # 物理/逻辑 像素比
    x, y = root.winfo_rootx(), root.winfo_rooty()
    w, h = root.winfo_width(), root.winfo_height()
    img = full.crop((int(x * sc), int(y * sc), int((x + w) * sc), int((y + h) * sc)))
    out = os.path.join(HERE, "app_shot.png")
    img.save(out)
    print("截图：", out, img.size)
    print("状态：", gui.status.get())
    print("障碍：", len(gui.obstacles), "点云：", len(gui.points))
    print("全程：", gui.mission_info.get())

    gui.stop_lidar()
    root.update()
    root.destroy()


if __name__ == "__main__":
    main()
