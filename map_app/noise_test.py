#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""抗干扰自检：往点云里塞各种"场外/噪声"的东西，看会不会误判成障碍。

跑法：  python noise_test.py

测的场景
--------
A  场外站了个人（界外一大团点）      → 应该全部丢弃
B  场外贴墙的一排点（界外密集墙）    → 应该全部丢弃
C  场内 1 个孤立噪点（反光/飞点）    → 不该封锁整个 450 格
D  场内 3 个挤在一起的噪点            → 按 blob 处理（可接受）
E  近场地面回波（距离 < range_min）   → 应该丢弃
F  真正的 φ50 黑圆柱                 → 应该识别成圆柱
G  贴在右侧墙上的二维码板            → 只能是 blob，且不该封死扫码区

每个场景输出：检出几处、分别是什么、会封锁哪些 5×5 格。
"""

from __future__ import annotations

import math
import os
import sys

import mapping

HERE = os.path.dirname(os.path.abspath(__file__))


def ring_points(field, cx, cy, r, step_deg=0.9):
    """在 (cx,cy) 放一个半径 r 的圆，从雷达位置看能看到的弧，生成点。"""
    pts = []
    lx, ly = field.lidar_x, field.lidar_y
    for k in range(int(360 / step_deg)):
        far = lx + 5000 * math.cos(math.radians(k * step_deg))
        # 简单做法：直接生成圆周上朝向雷达的那半圈
        a = math.radians(k * step_deg)
        x, y = cx + r * math.cos(a), cy + r * math.sin(a)
        if math.hypot(x - lx, y - ly) < math.hypot(cx - lx, cy - ly) + r:
            pts.append((x, y, 120))
    del far
    return pts


def blob_points(cx, cy, n, spread=10.0):
    """在 (cx,cy) 附近造 n 个挨得很近的点（模拟噪声团）。"""
    out = []
    for i in range(n):
        a = 2 * math.pi * i / max(n, 1)
        out.append((cx + spread * math.cos(a), cy + spread * math.sin(a), 120))
    return out


def is_station_cell(field, c, r):
    rc = field.cell_rect(c, r)
    for z in field.work_zones:
        zr = z["rect"]
        if not (zr.x1 < rc.x0 or zr.x0 > rc.x1 or zr.y1 < rc.y0 or zr.y0 > rc.y1):
            return True
    return False


def blocked_of(field, obs):
    cells = set()
    for o in obs:
        cr = field.cell_at(o["x"], o["y"])
        if cr and not is_station_cell(field, *cr):
            cells.add(cr)
    return cells


def run_case(field, cfg, name, points, expect):
    obs = mapping.detect_cylinders(points, cfg, field)
    cells = blocked_of(field, obs)
    labels = "、".join(sorted(field.cell_label(c, r) for (c, r) in cells)) or "无"
    kinds = {}
    for o in obs:
        kinds[o["kind"]] = kinds.get(o["kind"], 0) + 1
    kind_s = "、".join(f"{k}×{v}" for k, v in kinds.items()) or "无"
    print(f"{name}")
    print(f"   输入点 {len(points):4d} → 检出 {len(obs):2d} 处（{kind_s}）")
    print(f"   会封锁的格子：{labels}")
    print(f"   期望：{expect}")
    print()
    return obs, cells


def main():
    cfg = mapping.load_config(os.path.join(HERE, "field.json"))
    f = mapping.FieldModel(cfg)
    lx, ly = f.lidar_x, f.lidar_y
    print(f"雷达 @ ({lx:.0f},{ly:.0f}) yaw={f.lidar_yaw:.0f}°，场地 {f.W:.0f}×{f.H:.0f}\n")

    # A 场外一大团（模拟有人站在场地外/墙）
    outside = []
    for i in range(60):
        outside.append((1500 + (i % 6) * 20, -600 + (i // 6) * 40, 120))   # x>1200，界外
    run_case(f, cfg, "A 场外站着一个人（界外 60 点）", outside, "全部丢弃，不封锁任何格")

    # B 场外贴墙的一排
    wall = [(1240, y, 120) for y in range(-1000, 1000, 20)]
    run_case(f, cfg, "B 场外贴墙的一排点（100 点）", wall, "全部丢弃")

    # C 场内 1 个孤立噪点
    one = [(300, 300, 120)]
    run_case(f, cfg, "C 场内 1 个孤立噪点", one, "不该封锁整个格子（单点不可信）")

    # D 场内 3 个挤在一起的噪点
    three = blob_points(300, 300, 3, 8)
    run_case(f, cfg, "D 场内 3 个挤在一起的噪点", three, "按 blob 处理，可接受")

    # E 近场地面回波：距离小于 range_min
    near = []
    for k in range(20):
        a = math.radians(k * 18)
        d = 60      # < range_min(120)
        near.append((lx + d * math.cos(a), ly + d * math.sin(a), 120))
    run_case(f, cfg, "E 近场地面回波 20 点（d=60mm < range_min）", near,
             "采点时按 range_min 丢弃（这一步在 app 里做，这里只看检测）")

    # F 真圆柱
    true_cyl = ring_points(f, 110, 620, 25)
    run_case(f, cfg, "F 场内真 φ50 黑圆柱", true_cyl, "识别成 circle")

    # G 二维码板（贴右内墙的一条竖板，x≈1190）
    board = [(1190, y, 200) for y in range(-40, 41, 10)]
    run_case(f, cfg, "G 右侧墙上的二维码板（界内 9 点）", board,
             "最多是 blob，且 E3 是工位格应豁免")

    return 0


if __name__ == "__main__":
    sys.exit(main())
