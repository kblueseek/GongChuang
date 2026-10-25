#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单目标实测：量一个物体的真实距离和宽度，跟卷尺对照，判断比例对不对。

跑法：
    python measure.py              # 扫 5 圈，列出最靠前的几个物体
    python measure.py 8            # 扫 8 圈
    python measure.py 5 COM22      # 指定口

输出每一项都自带"怎么算出来的"：
    距离      —— 到雷达的直线距离（这个靠距离字节，最可靠）
    弧跨角    —— 这团点在雷达眼里占了多少度
    横向宽度  —— 2 × 距离 × tan(弧跨角/2)，物体"看起来"多宽
    拟合直径  —— 圆拟合出来的直径（小圆弧会偏小，只作参考）

用法：把物体放在雷达正前方 0.8~1.0 m 的空地上，周围别放东西，
      卷尺量一下它的实际直径，跟这里打印的"横向宽度"对一下。
"""

from __future__ import annotations

import math
import sys
import time

import lidar_d6
import mapping

HERE_HINT = "把物体放在雷达正前方 0.8~1.0 m，周围别放别的东西"


def pick_port(preferred=None):
    try:
        from serial.tools import list_ports
    except Exception:
        return preferred
    ports = list(list_ports.comports())
    if preferred:
        return preferred
    for p in ports:
        d = (p.description or "").upper()
        if any(k in d for k in ("CH340", "CP210", "USB-SERIAL", "FTDI", "SILICON")):
            return p.device
    return ports[0].device if ports else None


def main():
    revs = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    port = pick_port(sys.argv[2] if len(sys.argv) > 2 else None)
    if not port:
        print("没找到串口")
        return 1

    cfg = mapping.load_config(__file__.replace("measure.py", "field.json"))
    field = mapping.FieldModel(cfg)

    pts = []
    rings = {"n": 0}

    def on_frame(fr, ring):
        if ring:
            rings["n"] += 1
        for ang, d, inten in fr:
            if d < cfg["scan"]["range_min_mm"] or d > cfg["scan"]["range_max_mm"]:
                continue
            x, y = field.lidar_to_world(ang, d)
            pts.append((x, y, inten))

    lid = lidar_d6.D6Lidar(port, 230400, on_frame=on_frame, on_log=lambda s: None)
    if not lid.open():
        print("打不开：", lid.error)
        return 2
    print(f"{port} 已连上，扫 {revs} 圈 ...")
    t0 = time.time()
    while rings["n"] < revs and time.time() - t0 < 12:
        time.sleep(0.05)
    lid.close()

    print(f"收到 {len(pts)} 个点，{rings['n']} 个整圈标记\n")
    if not pts:
        print("没数据")
        return 3

    # 聚类，按点数排序（点数越多 = 越大/越近）
    clusters = mapping._cluster(pts, float(cfg["cylinder"]["cluster_gap_mm"]))
    clusters.sort(key=len, reverse=True)

    lx, ly = field.lidar_x, field.lidar_y
    print(f"{'#':>2} {'点数':>5} {'距离mm':>7} {'弧跨角°':>8} {'横向宽度mm':>10} "
          f"{'拟合直径mm':>10} {'残差mm':>7}")
    print("-" * 62)
    for i, cl in enumerate(clusters[:6], 1):
        n = len(cl)
        if n < 3:
            continue
        cx = sum(p[0] for p in cl) / n
        cy = sum(p[1] for p in cl) / n
        dist = math.hypot(cx - lx, cy - ly)
        # 弧跨角：以质心方向为基准，取最大偏离 ×2
        base = math.atan2(cy - ly, cx - lx)
        span = 0.0
        for p in cl:
            a = math.atan2(p[1] - ly, p[0] - lx) - base
            a = (a + math.pi) % (2 * math.pi) - math.pi
            span = max(span, abs(a))
        span_deg = math.degrees(span) * 2
        width = 2 * dist * math.tan(math.radians(span_deg / 2))
        fit = mapping._fit_circle(cl)
        fd = fit[2] * 2 if fit else float("nan")
        rms = fit[3] if fit else float("nan")
        print(f"{i:>2} {n:>5} {dist:>7.0f} {span_deg:>8.1f} {width:>10.0f} "
              f"{fd:>10.0f} {rms:>7.1f}")

    print()
    print("怎么用这张表：")
    print("  · 「距离mm」用卷尺量到物体表面比一下，差得多 = 距离字节或安装位置有问题")
    print(f"  · 「横向宽度mm」跟物体实际直径比，差 3 倍 = 角度标定有问题")
    print(f"  · 「拟合直径mm」小圆弧会偏小，只在弧跨角 > 6° 时才可信")
    print(f"\n提示：{HERE_HINT}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
