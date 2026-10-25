#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""无界面自检：造一份模拟扫描 → 检测圆柱 → A* 规划 → 输出 PNG 预览。

用途
----
1. 没有雷达时验证整套算法跑得通；
2. 出图给队友看效果；
3. 调参数（聚类阈值、容差）时快速看结果。

跑法：  python selftest.py
输出：  preview.png（和 app.py 里看到的是同一套坐标）
"""

from __future__ import annotations

import math
import os
import sys

from PIL import Image, ImageDraw, ImageFont

import mapping
import lidar_d6

HERE = os.path.dirname(os.path.abspath(__file__))

C_BG      = (255, 255, 255)
C_FIELD   = (203, 203, 203)
C_WHITE   = (248, 248, 246)
C_YELLOW  = (253, 247, 205)
C_BLUE    = (26, 60, 235)
C_CONN    = (226, 226, 226)
C_POINT   = (105, 105, 105)
C_CYL     = (226, 30, 60)
C_BLOB    = (170, 110, 20)
C_PATH    = (20, 110, 220)
C_LIDAR   = (0, 140, 80)
C_TEXT    = (40, 40, 40)


def get_font(size=13, bold=False):
    """PIL 默认字体没有中文，会画成方块。这里按顺序找一个中文字体。"""
    cands = [r"C:\Windows\Fonts\msyhbd.ttc" if bold else r"C:\Windows\Fonts\msyh.ttc",
             r"C:\Windows\Fonts\simhei.ttf",
             "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
             "/System/Library/Fonts/PingFang.ttc"]
    for p in cands:
        if os.path.exists(p):
            try:
                return ImageFont.truetype(p, size)
            except Exception:
                pass
    return ImageFont.load_default()


def make_scan(field, obstacles, revs=5, pts_per_rev=400, noise=6.0):
    """离线跑几圈：从雷达位置按 0.9° 步进打光线，返回世界坐标点云。"""
    sim = lidar_d6.SimLidar(field, obstacles, points_per_rev=pts_per_rev, noise_mm=0.0)
    world = []
    step = 360.0 / pts_per_rev
    for k in range(revs):
        for i in range(pts_per_rev):
            ang = i * step
            d = sim._ray(field.lidar_x, field.lidar_y,
                         field.lidar_yaw + field.angle_sign * ang)
            if d is None:
                continue
            if noise:
                d += noise * math.sin(i * 12.9898 + k * 0.7)
            if d < 120 or d > 8000:
                continue
            x, y = field.lidar_to_world(ang, d)
            world.append((x, y, 120))
    return world


PAL = [(20, 110, 222), (46, 158, 107), (176, 96, 31), (142, 36, 170),
       (194, 42, 82), (11, 110, 138), (90, 125, 31), (122, 79, 191), (208, 129, 20)]


def render(field, points, cylinders, path_world, out_path, title="",
           blocked_cells=(), legs=()):
    S = 0.30
    M = 70
    W = int(field.W * S) + 2 * M
    H = int(field.H * S) + 2 * M
    img = Image.new("RGB", (W, H), C_BG)
    d = ImageDraw.Draw(img)
    f13, f12, f14 = get_font(13), get_font(11), get_font(15, True)

    def T(x, y):
        return (M + (x + field.hw) * S, M + (field.hh - y) * S)

    # 场地
    d.rectangle([T(-field.hw, field.hh), T(field.hw, -field.hh)], fill=C_WHITE)
    for r in field.lane_rects:
        d.rectangle([T(r.x0, r.y1), T(r.x1, r.y0)], fill=C_FIELD)
    for r in field.yellow_rects:
        d.rectangle([T(r.x0, r.y1), T(r.x1, r.y0)], fill=C_YELLOW, outline=(236, 226, 172))
    for z in field.start_zones:
        r = z["rect"]
        d.rectangle([T(r.x0, r.y1), T(r.x1, r.y0)], fill=C_BLUE)
        d.text((T(z["cx"], z["cy"])[0] - 6, T(z["cx"], z["cy"])[1] - 8),
               z["name"][-1] if "启停" in z["name"] else z["name"],
               fill=(255, 255, 255), font=f13)
    for z in field.work_zones:
        r = z["rect"]
        d.rectangle([T(r.x0, r.y1), T(r.x1, r.y0)], outline=(110, 155, 110), width=1)
        p = T(r.x0, r.y1)
        d.text((p[0], p[1] - 15), z["name"], fill=(80, 115, 80), font=f12)
    # 5×5 网格（按黄块边长分格）
    for gx in field.gx:
        d.line([T(gx, field.gy[0]), T(gx, field.gy[-1])], fill=(182, 182, 182), width=1)
    for gy in field.gy:
        d.line([T(field.gx[0], gy), T(field.gx[-1], gy)], fill=(182, 182, 182), width=1)
    for rr in range(field.cell_rows):
        for cc in range(field.cell_cols):
            px, py = T(*field.cell_center(cc, rr))
            d.text((px - 8, py - 7), field.cell_label(cc, rr), fill=(152, 152, 152), font=f12)

    d.rectangle([T(-field.hw, field.hh), T(field.hw, -field.hh)],
                outline=(70, 70, 70), width=2)

    # 雷达原始点
    for x, y, *_ in points:
        px, py = T(x, y)
        d.ellipse([px - 1.2, py - 1.2, px + 1.2, py + 1.2], fill=C_POINT)

    # 任务路线
    for i, leg in enumerate(legs):
        col = PAL[i % len(PAL)]
        if leg["path"]:
            d.line([T(x, y) for x, y in leg["path"]], fill=col, width=4, joint="curve")
        if leg["from_xy"]:
            px, py = T(*leg["from_xy"])
            d.ellipse([px - 8, py - 8, px + 8, py + 8], fill=(255, 255, 255),
                      outline=col, width=2)
            d.text((px - 4, py - 6), str(i + 1), fill=col, font=f12)
    if legs and legs[-1]["to_xy"]:
        px, py = T(*legs[-1]["to_xy"])
        d.ellipse([px - 8, py - 8, px + 8, py + 8], fill=(255, 255, 255),
                  outline=PAL[0], width=2)
        d.text((px - 4, py - 6), str(len(legs) + 1), fill=PAL[0], font=f12)

    # 单点路径
    if len(path_world) > 1:
        d.line([T(x, y) for x, y in path_world], fill=C_PATH, width=4, joint="curve")
        for x, y in (path_world[0], path_world[-1]):
            px, py = T(x, y)
            d.ellipse([px - 5, py - 5, px + 5, py + 5], fill=C_PATH)

    # 检出障碍
    for c in cylinders:
        px, py = T(c["x"], c["y"])
        if c["kind"] == "circle":
            rr = max(c["r"] * S, 3)
            d.ellipse([px - rr - 4, py - rr - 4, px + rr + 4, py + rr + 4],
                      outline=C_CYL, width=2)
            d.line([px - 6, py, px + 6, py], fill=C_CYL, width=1)
            d.line([px, py - 6, px, py + 6], fill=C_CYL, width=1)
            tag = "φ50" + ("" if c.get("r_confident") else "?")
            d.text((px + rr + 7, py - 9),
                   f'{tag} ({c["x"]:.0f},{c["y"]:.0f}) 拟合φ{c["r"]*2:.0f}',
                   fill=C_CYL, font=f12)
        else:
            d.ellipse([px - 5, py - 5, px + 5, py + 5], outline=C_BLOB, width=2)
            d.text((px + 8, py - 8), f'未知 ({c["x"]:.0f},{c["y"]:.0f})',
                   fill=C_BLOB, font=f12)

    # 雷达
    px, py = T(field.lidar_x, field.lidar_y)
    a = math.radians(field.lidar_yaw)
    d.ellipse([px - 5, py - 5, px + 5, py + 5], fill=C_LIDAR)
    d.line([px, py, px + 30 * math.cos(a), py - 30 * math.sin(a)], fill=C_LIDAR, width=3)

    # 标题 + 图例
    d.text((12, 8), title, fill=C_TEXT, font=f14)
    d.text((12, H - 26),
           f'灰点=雷达回波 {len(points)} 个  蓝线=规划路径 {len(path_world)} 个拐点  '
           f'红圈=圆柱  橙圈=未知障碍',
           fill=(90, 90, 90), font=f12)
    # 被圆柱占住、整格不能通车 → 半透明红
    if blocked_cells:
        ov = Image.new("RGBA", img.size, (0, 0, 0, 0))
        od = ImageDraw.Draw(ov)
        for (cc, rr) in blocked_cells:
            r = field.cell_rect(cc, rr)
            od.rectangle([T(r.x0, r.y1), T(r.x1, r.y0)],
                         fill=(226, 30, 60, 85), outline=(226, 30, 60, 255), width=2)
        img = Image.alpha_composite(img.convert("RGBA"), ov).convert("RGB")
        d = ImageDraw.Draw(img)
        for (cc, rr) in blocked_cells:
            r = field.cell_rect(cc, rr)
            p0, p1 = T(r.x0, r.y1), T(r.x1, r.y0)
            d.text(((p0[0] + p1[0]) / 2 - 18, p0[1] + 6), "不通车",
                   fill=(150, 10, 30), font=f13)

    img.save(out_path)
    return out_path


def main():
    cfg = mapping.load_config(os.path.join(HERE, "field.json"))
    field = mapping.FieldModel(cfg)

    # 场上摆东西：2 个黑圆柱（规则 φ50×100）+ 1 个转盘（φ300，用来验证"不像圆"分支）
    truth = [
        {"x": 110,  "y": 620,  "r": 25,  "intensity": 60},   # 车道上偏右，得从左绕
        {"x": -60,  "y": -300, "r": 25,  "intensity": 60},   # 另一个偏左
        {"x": -150, "y": 1150, "r": 150, "intensity": 90},   # 原料区转盘
    ]

    print("模拟扫描 ...")
    pts = make_scan(field, truth, revs=int(cfg["scan"]["revolutions"]))
    print(f"  点云 {len(pts)} 个点")

    cy = mapping.detect_cylinders(pts, cfg, field)
    print(f"  检出 {len(cy)} 处障碍：")
    for c in cy:
        r = f'拟合φ{c["r"]*2:.0f}mm' if c["r"] else "尺寸未知"
        conf = "" if c.get("r_confident", True) else "  (半径不可信)"
        print(f'    {c["kind"]:6s} ({c["x"]:7.1f},{c["y"]:7.1f})  {r}  '
              f'n={c["n"]:2d}  {c["dist"]:.0f}mm{conf}')

    # 工位格豁免：工位里的物体不封锁整格，否则靠泊点进不去
    def is_station(c, r):
        rc = field.cell_rect(c, r)
        for z in field.work_zones:
            zr = z["rect"]
            if not (zr.x1 < rc.x0 or zr.x0 > rc.x1 or zr.y1 < rc.y0 or zr.y0 > rc.y1):
                return True
        return False

    blocked_cells = {c for c in (field.cell_at(o["x"], o["y"]) for o in cy)
                     if c and not is_station(*c)}
    print("  占用格：" + "、".join(sorted(field.cell_label(c, r) for c, r in blocked_cells)))

    grid = mapping.Grid(field)
    mask = grid.build_drivable()
    start = grid.w2c(field.lidar_x, field.lidar_y)
    goal = grid.w2c(0, -900)
    cell_block = grid.build_cell_blocked(blocked_cells)
    cf = mapping.centerline_cost(field)
    plan = mapping.plan_path(grid, mask, cy, start, goal, field.robot_radius,
                             cell_block=cell_block, cost_fn=cf)
    path_world = plan.get("path_world") or [grid.c2w(ix, iy) for ix, iy in plan["path"]]
    print(f'  单点 C1：{len(path_world)} 个拐点，最小余量 {plan["min_clear"]:.0f}mm，{plan["note"]}')

    legs, total = mapping.plan_mission(field, grid, mask, cy, None,
                                      field.robot_radius, cell_block=cell_block,
                                      cost_fn=cf)
    print(f'  全程：{sum(1 for l in legs if l["ok"])}/{len(legs)} 段可通，'
          f'总长 {total/1000:.2f} m')
    for i, l in enumerate(legs, 1):
        print(f'    {i}. {l["from_name"]:>4s}→{l["to_name"]:<4s} '
              f'{l["mm"]/1000:5.2f}m {"OK" if l["ok"] else l["note"]}')

    out = render(field, pts, cy, path_world,
                 os.path.join(HERE, "preview.png"),
                 blocked_cells=blocked_cells, legs=legs,
                 title=f'智能搬运场地 {field.W:.0f}×{field.H:.0f}mm · '
                       f'雷达@{field.lidar_mount}({field.lidar_x:.0f},{field.lidar_y:.0f}) · '
                       f'检出 {sum(1 for c in cy if c["kind"]=="circle")} 个圆柱 · '
                       f'全程 {total/1000:.2f}m')
    print("预览图：", out)


if __name__ == "__main__":
    sys.exit(main())
