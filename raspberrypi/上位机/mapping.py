#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""核心算法：场地几何模型 + 占据栅格 + 圆柱检测 + A* 路径规划。

================================ 大白话 ================================

这个文件不碰界面、不碰串口，纯粹是"算"。三件事：

1. 场地长什么样（FieldModel）
   二维雷达看不见地面上的灰色车道和淡黄色方块——它只能看到立在扫描平面里的
   东西（圆柱、转盘、二维码板）。所以地图**必须**用参数画出来，不能靠雷达扫。
   现场尺寸变了，改 field.json 就行。

2. 雷达点怎么变成地图上的东西（detect_cylinders）
   一串点 → 按距离聚成一堆一堆（聚类）→ 对每一堆拟合一个圆 → 半径对得上
   φ50 的就认成圆柱。远一点的小圆柱点太少拟合不出来，退化成"这里有个东西"。

3. 车怎么绕过去（astar）
   把场地切成 20mm 的小格子，车道格子可走、圆柱周围按车半径加一圈禁区，
   然后跑 A*。

坐标系：原点在场心，+x 向右，+y 向上，单位毫米，角度逆时针为正、单位度。
（和赛场示意图一致；和机器人坐标系 x 向前 y 向左不同，转换在 app.py 里做。）
"""

from __future__ import annotations

import json
import math

# ==================== 1. 场地模型 ====================


class Rect:
    """轴对齐矩形，(x0, y0) 左下角，(x1, y1) 右上角，单位 mm。"""

    __slots__ = ("x0", "y0", "x1", "y1")

    def __init__(self, x0, y0, x1, y1):
        self.x0, self.y0 = min(x0, x1), min(y0, y1)
        self.x1, self.y1 = max(x0, x1), max(y0, y1)

    @staticmethod
    def from_center(cx, cy, w, h):
        return Rect(cx - w / 2.0, cy - h / 2.0, cx + w / 2.0, cy + h / 2.0)

    def contains(self, x, y):
        return self.x0 <= x <= self.x1 and self.y0 <= y <= self.y1

    def dist_to(self, x, y):
        """点到矩形的距离（在矩形内部时为 0）。"""
        dx = max(self.x0 - x, 0.0, x - self.x1)
        dy = max(self.y0 - y, 0.0, y - self.y1)
        return math.hypot(dx, dy)

    def __repr__(self):
        return f"Rect({self.x0:.0f},{self.y0:.0f},{self.x1:.0f},{self.y1:.0f})"


class FieldModel:
    """整块场地的几何：车道、淡黄块、启停区、工位区、雷达位置。"""

    def __init__(self, cfg: dict):
        f = cfg["field"]
        self.W = float(f["width_mm"])
        self.H = float(f["height_mm"])
        self.hw, self.hh = self.W / 2.0, self.H / 2.0

        lane_cfg = cfg["lane"]
        lane_w = float(lane_cfg["width_mm"])
        self.lane_w = lane_w
        # 十字车道 = 竖带 ∪ 横带，贯穿整块场地（"田"字中间那一横一竖）
        self.cross_rects = [
            Rect(-lane_w / 2, -self.hh, lane_w / 2, self.hh),   # 竖带
            Rect(-self.hw, -lane_w / 2, self.hw, lane_w / 2),   # 横带
        ]
        # 外圈车道 = 贴着四面墙的一圈（"田"字的外框）。
        # 宽度默认和十字车道一样；ring_mm = 0 就退回只有十字。
        ring_w = float(lane_cfg.get("ring_mm", lane_w))
        self.ring_w = ring_w
        self.ring_rects = []
        if ring_w > 0:
            rw_x = min(ring_w, self.hw)
            rw_y = min(ring_w, self.hh)
            self.ring_rects = [
                Rect(-self.hw, self.hh - rw_y, self.hw, self.hh),      # 上
                Rect(-self.hw, -self.hh, self.hw, -self.hh + rw_y),    # 下
                Rect(-self.hw, -self.hh, -self.hw + rw_x, self.hh),    # 左
                Rect(self.hw - rw_x, -self.hh, self.hw, self.hh),      # 右
            ]
        # 所有灰色车道（十字 + 外圈）
        self.lane_rects = self.cross_rects + self.ring_rects

        # 四个淡黄方块：内边贴车道边
        b = cfg["yellow_blocks"]
        size = float(b["size_mm"])
        gap = float(b.get("gap_from_lane_mm", 0))
        inner = lane_w / 2.0 + gap
        self.yellow_rects = []
        for sx in (-1, 1):
            for sy in (-1, 1):
                x0 = sx * inner
                x1 = sx * (inner + size)
                y0 = sy * inner
                y1 = sy * (inner + size)
                self.yellow_rects.append(Rect(x0, y0, x1, y1))

        # 启停区（蓝色，可放车、也可放雷达）
        self.start_zones = []
        for z in cfg.get("start_zones", []):
            self.start_zones.append({
                "name": z["name"],
                "rect": Rect.from_center(z["x"], z["y"], z["w"], z["h"]),
                "cx": float(z["x"]),
                "cy": float(z["y"]),
            })

        # 工位区（只用于显示和任务路线，不参与可行驶判定）
        self.work_zones = []
        for z in cfg.get("work_zones", []):
            ap = z.get("approach", [z["x"], z["y"]])
            self.work_zones.append({
                "name": z["name"],
                "rect": Rect.from_center(z["x"], z["y"], z["w"], z["h"]),
                "approach": (float(ap[0]), float(ap[1])),
            })

        # 任务路线（名字序列）
        ms = cfg.get("mission", {}) or {}
        self.mission_route = list(ms.get("route", []))
        self.mission_speed = float(ms.get("speed_mm_s", 250))

        # 5×5 细胞网格（按淡黄块边长分格，居中）
        self._build_cell_grid(cfg)

        # 雷达安装位姿
        ld = cfg["lidar"]
        self.lidar_x = float(ld["x"])
        self.lidar_y = float(ld["y"])
        self.lidar_yaw = float(ld["yaw_deg"])
        self.lidar_mount = ld.get("mount", "")
        # 设备角度是"顺时针为正"（文档：雷达顺时针旋转，且帧内角度值递增）。
        # 转成数学的逆时针为正必须取反，否则整个点云是镜像的（南北翻转）。
        self.angle_cw = bool(ld.get("angle_cw", True))
        self.angle_sign = -1.0 if self.angle_cw else 1.0

        # 白区是否可走（默认 True：除了淡黄块，整个场地都能走）
        dr = cfg.get("drivable", {}) or {}
        self.white_drivable = bool(dr.get("white", True))

        self.robot_radius = float(cfg["robot"]["radius_mm"])
        self.cell = float(cfg["grid"]["cell_mm"])
        self.cfg = cfg

    # ---------- 可行驶区域 ----------

    def connector_rects(self):
        """启停区 → 最近一条车道 的走廊。

        规则上说"离开启停区后只能走灰色车道"，但示意图里启停区并不与车道相接，
        中间隔着一段白区。真实赛道上要么有一条灰色连接带，要么允许直接开过去。
        这里按"允许直连"处理，宽度取启停区边长；现场若确认有连接带，把
        corridor 设成 false 并在 field.json 里补一条 lane 矩形即可。
        """
        if not self.cfg.get("corridor", True):
            return []
        out = []
        for z in self.start_zones:
            r = z["rect"]
            best, best_d = None, 1e18
            for lr in self.lane_rects:
                d = min(lr.dist_to(r.x0, r.y0), lr.dist_to(r.x1, r.y0),
                        lr.dist_to(r.x0, r.y1), lr.dist_to(r.x1, r.y1),
                        lr.dist_to(z["cx"], z["cy"]))
                if d < best_d:
                    best, best_d = lr, d
            if best is None:
                continue
            # 走廊：把启停区在"最靠近方向"上延伸到车道
            cx, cy = z["cx"], z["cy"]
            if best.contains(cx, cy):
                continue                             # 启停区就在车道里，不用修走廊
            if cx > best.x1:                         # 车道在左边/右边：横向连
                out.append(Rect(best.x1, r.y0, r.x0, r.y1))
            elif cx < best.x0:
                out.append(Rect(r.x1, r.y0, best.x0, r.y1))
            elif cy > best.y1:                       # 车道在上边/下边：纵向连
                out.append(Rect(r.x0, best.y1, r.x1, r.y0))
            elif cy < best.y0:
                out.append(Rect(r.x0, r.y1, r.x1, best.y0))
        return out

    def drivable_rects(self):
        return self.lane_rects + [z["rect"] for z in self.start_zones] + self.connector_rects()

    def is_drivable(self, x, y):
        """这个点车能不能走。

        white_drivable = True：除了淡黄块，整个场地都能走。
        white_drivable = False：只走灰色【田】字车道（外圈 + 十字）+ 启停区 + 连接走廊。
        """
        if not self.in_field(x, y):
            return False
        # 淡黄块是禁行区，两种口径都要先排除（外圈车道和黄块边线会重叠）
        for r in self.yellow_rects:
            if r.contains(x, y):
                return False
        if self.white_drivable:
            return True
        for r in self.drivable_rects():
            if r.contains(x, y):
                return True
        return False

    def world_bearing_to_device_angle(self, bearing_deg):
        """世界方位角 → 设备角度（SimLidar 打光线用，和 lidar_to_world 严格互逆）。"""
        d = bearing_deg - self.lidar_yaw
        return d / self.angle_sign

    def in_field(self, x, y):
        return -self.hw <= x <= self.hw and -self.hh <= y <= self.hh

    # ---------- 5×5 细胞网格 ----------

    def _build_cell_grid(self, cfg):
        """按淡黄块边长把场地切成 cols×rows 格，居中摆放。

        2400 = 75 + 5×450 + 75  —— 两边各留 75mm（图纸上是 75-85 的量）。
        这样切出来的好处：中心格正好是十字路口的 450×450，四个淡黄块各自占一格，
        和赛场上"第几行第几列"的说法一一对应。

        场地尺寸改了也不用手动算：格边长固定 450，余量自动平分；
        余量变成负数（场地比格子总和小）时，自动把格子缩到刚好塞下。
        """
        cg = cfg.get("cell_grid", {}) or {}
        self.cell_cols = int(cg.get("cols", 5))
        self.cell_rows = int(cg.get("rows", 5))
        want = float(cg.get("cell_mm", cfg["yellow_blocks"]["size_mm"]))
        # 塞不下就缩
        self.cell_mm = min(want, self.W / self.cell_cols, self.H / self.cell_rows)
        mx = (self.W - self.cell_cols * self.cell_mm) / 2.0
        my = (self.H - self.cell_rows * self.cell_mm) / 2.0
        self.gx = [-self.hw + mx + i * self.cell_mm for i in range(self.cell_cols + 1)]
        self.gy = [-self.hh + my + i * self.cell_mm for i in range(self.cell_rows + 1)]
        self.grid_margin = (mx, my)

    def cell_rect(self, c, r):
        """第 c 列、第 r 行的格子（都从 0 数，r=0 是最下面一行）。"""
        return Rect(self.gx[c], self.gy[r], self.gx[c + 1], self.gy[r + 1])

    def cell_center(self, c, r):
        return ((self.gx[c] + self.gx[c + 1]) / 2.0,
                (self.gy[r] + self.gy[r + 1]) / 2.0)

    def cell_at(self, x, y):
        """世界坐标落在哪一格 → (c, r)；在场外或落在边距里返回 None。"""
        if not (self.gx[0] <= x < self.gx[-1] and self.gy[0] <= y < self.gy[-1]):
            return None
        c = r = 0
        for i in range(self.cell_cols):
            if self.gx[i] <= x < self.gx[i + 1]:
                c = i
                break
        for i in range(self.cell_rows):
            if self.gy[i] <= y < self.gy[i + 1]:
                r = i
                break
        return c, r

    def cell_label(self, c, r):
        """第 c 列 → 字母 A..E（从左到右），第 r 行 → 数字 1..5（从下往上）。"""
        return f"{chr(ord('A') + c)}{r + 1}"

    def resolve_waypoint(self, name):
        """把名字换成坐标。START = 雷达所在的那个启停区。找不到返回 None。"""
        if name in ("START", "start", "起点"):
            name = self.lidar_mount or (self.start_zones[0]["name"] if self.start_zones else "")
        for z in self.start_zones:
            if z["name"] == name:
                return (z["cx"], z["cy"])
        for z in self.work_zones:
            if z["name"] == name:
                return z["approach"]
        return None

    def cell_name(self, x, y):
        """世界坐标 → 'C3' 这种格子名；不在格子里返回 '边距' / '场外'。"""
        if not self.in_field(x, y):
            return "场外"
        cr = self.cell_at(x, y)
        return self.cell_label(*cr) if cr else "边距"

    # ---------- 雷达坐标 → 世界坐标 ----------

    def lidar_to_world(self, angle_deg, dist_mm):
        """雷达返回的 (角度, 距离) → 世界系 (x, y)。

        雷达角度是设备给出的"顺时针为正"值；yaw 是设备 0° 在世界系里的朝向。
        angle_sign = -1 时把设备角转成数学的逆时针角（D6 就是这种）。
        """
        a = math.radians(self.lidar_yaw + self.angle_sign * angle_deg)
        return (self.lidar_x + dist_mm * math.cos(a),
                self.lidar_y + dist_mm * math.sin(a))


# ==================== 2. 占据栅格 ====================


class Grid:
    """把场地切成 cell_mm 见方的小格子。mask[iy][ix] = True 表示可通行。"""

    def __init__(self, field: FieldModel):
        self.field = field
        self.cell = field.cell
        self.nx = int(round(field.W / self.cell))
        self.ny = int(round(field.H / self.cell))

    # 坐标换算
    def w2c(self, x, y):
        ix = int((x + self.field.hw) / self.cell)
        iy = int((y + self.field.hh) / self.cell)
        return ix, iy

    def c2w(self, ix, iy):
        return (-self.field.hw + (ix + 0.5) * self.cell,
                -self.field.hh + (iy + 0.5) * self.cell)

    def inside(self, ix, iy):
        return 0 <= ix < self.nx and 0 <= iy < self.ny

    # 构建
    def build_drivable(self):
        """按场地模型生成可通行掩码（逐格问 FieldModel.is_drivable）。"""
        mask = [[False] * self.nx for _ in range(self.ny)]
        for iy in range(self.ny):
            y = -self.field.hh + (iy + 0.5) * self.cell
            row = mask[iy]
            for ix in range(self.nx):
                x = -self.field.hw + (ix + 0.5) * self.cell
                row[ix] = self.field.is_drivable(x, y)
        return mask

    def build_cell_blocked(self, cells):
        """把被圆柱占用的 5×5 整格标成禁行（450mm 见方的 20mm 小格全封）。

        cells: {(c, r), ...}  —— FieldModel.cell_at 的返回值
        """
        blk = [[False] * self.nx for _ in range(self.ny)]
        for (c, r) in cells:
            rect = self.field.cell_rect(c, r)
            ix0, iy0 = self.w2c(rect.x0, rect.y0)
            ix1, iy1 = self.w2c(rect.x1 - 0.1, rect.y1 - 0.1)
            for iy in range(max(0, iy0), min(self.ny, iy1 + 1)):
                for ix in range(max(0, ix0), min(self.nx, ix1 + 1)):
                    blk[iy][ix] = True
        return blk

    def build_blocked(self, cylinders, radius_pad_mm):
        """把圆柱（及其余障碍）标成禁区，并按车半径膨胀。"""
        pad = radius_pad_mm
        blocked = [[False] * self.nx for _ in range(self.ny)]
        if not cylinders:
            return blocked
        reach = pad + max(_ob_radius(c) for c in cylinders)
        for c in cylinders:
            cx, cy = c["x"], c["y"]
            r = _ob_radius(c)
            # 只用包围盒内逐格判断，别全图扫
            ix0, iy0 = self.w2c(cx - reach, cy - reach)
            ix1, iy1 = self.w2c(cx + reach, cy + reach)
            for iy in range(max(0, iy0), min(self.ny, iy1 + 2)):
                wy = -self.field.hh + (iy + 0.5) * self.cell
                for ix in range(max(0, ix0), min(self.nx, ix1 + 2)):
                    wx = -self.field.hw + (ix + 0.5) * self.cell
                    if math.hypot(wx - cx, wy - cy) <= r + pad:
                        blocked[iy][ix] = True
        return blocked


# ==================== 3. 圆柱检测 ====================


def _cluster(points, gap):
    """按"点与点距离 ≤ gap"把点云聚成堆。返回 list[list[(x,y,inten)]]。

    用空间哈希，避免 O(n²)。点数量级几千，其实暴力也够快，但这样更稳。
    """
    if not points:
        return []
    cell = gap
    buckets = {}
    for i, p in enumerate(points):
        k = (int(math.floor(p[0] / cell)), int(math.floor(p[1] / cell)))
        buckets.setdefault(k, []).append(i)

    seen = [False] * len(points)
    clusters = []
    gap2 = gap * gap
    for i in range(len(points)):
        if seen[i]:
            continue
        seen[i] = True
        stack = [i]
        cur = []
        while stack:
            j = stack.pop()
            p = points[j]
            cur.append(j)
            bx = int(math.floor(p[0] / cell))
            by = int(math.floor(p[1] / cell))
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for k in buckets.get((bx + dx, by + dy), ()):
                        if seen[k]:
                            continue
                        q = points[k]
                        if (p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 <= gap2:
                            seen[k] = True
                            stack.append(k)
        clusters.append([points[j] for j in cur])
    return clusters


def _fit_circle(pts):
    """Kåsa 代数圆拟合。返回 (cx, cy, r, rms)；退化时返回 None。

    原理：把 x²+y² = a·x + b·y + c 当线性方程，最小二乘解出 a,b,c，
    圆心就是 (a/2, b/2)，半径 r = sqrt(c + a²/4 + b²/4)。
    """
    n = len(pts)
    if n < 3:
        return None
    sx = sy = sxx = syy = sxy = sxz = syz = sz = 0.0
    for x, y, *_ in ((p[0], p[1]) for p in pts):
        z = x * x + y * y
        sx += x; sy += y; sz += z
        sxx += x * x; syy += y * y; sxy += x * y
        sxz += x * z; syz += y * z
    # 正常方程 [sxx sxy sx; sxy syy sy; sx sy n] · [a b c]ᵀ = [sxz syz sz]ᵀ
    A = [[sxx, sxy, sx],
         [sxy, syy, sy],
         [sx,  sy,  n]]
    b = [sxz, syz, sz]
    sol = _solve3(A, b)
    if sol is None:
        return None
    a, bb, c = sol
    cx, cy = a / 2.0, bb / 2.0
    r2 = c + cx * cx + cy * cy
    if r2 <= 0:
        return None
    r = math.sqrt(r2)
    # 残差
    s = 0.0
    for p in pts:
        d = math.hypot(p[0] - cx, p[1] - cy) - r
        s += d * d
    rms = math.sqrt(s / n)
    return cx, cy, r, rms


def _solve3(A, b):
    """3x3 线性方程，克莱姆法则。奇异返回 None。"""
    def det3(m):
        return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
                - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
    D = det3(A)
    if abs(D) < 1e-9:
        return None
    out = []
    for k in range(3):
        M = [row[:] for row in A]
        for i in range(3):
            M[i][k] = b[i]
        out.append(det3(M) / D)
    return out


def _ob_radius(c):
    """障碍用于膨胀的半径：优先用 r_plan（按规则取 φ50），没有就退回拟合值。"""
    return float(c.get("r_plan") or c.get("r") or 0.0)


def detect_cylinders(points, cfg, field: FieldModel):
    """点云 → 圆柱列表。

    points: [(x, y, intensity), ...] 世界坐标 mm
    返回:   [{"x","y","r","n","rms","kind","dist"}, ...]
            kind = "circle" 拟合成功；"blob" 点太少只能标记"有东西"
    """
    cc = cfg["cylinder"]
    r_nom = float(cc["diameter_mm"]) / 2.0
    min_n = int(cc["min_points"])
    gap = float(cc["cluster_gap_mm"])

    # 三重过滤：① 必须在场地内 ② 距离在量程内（近场地面回波、远处房间墙都挡掉）
    rmin = float(cfg["scan"]["range_min_mm"])
    rmax = float(cfg["scan"]["range_max_mm"])
    lx, ly = field.lidar_x, field.lidar_y
    pts = [p for p in points
           if field.in_field(p[0], p[1])
           and rmin <= math.hypot(p[0] - lx, p[1] - ly) <= rmax]

    min_blob = int(cc.get("blob_min_points", 3))
    out = []
    for cl in _cluster(pts, gap):
        n = len(cl)
        # ③ 点数太少：一个孤立飞点就可能封掉一整格 450×450，代价太大，直接当噪声
        if n < min_blob:
            continue

        cx = sum(p[0] for p in cl) / n
        cy = sum(p[1] for p in cl) / n
        dist = math.hypot(cx - lx, cy - ly)
        fit = _fit_circle(cl) if n >= 3 else None

        if fit is not None:
            fx, fy, r, rms = fit
            # 这一簇在雷达眼里占多大角度（判断半径可不可信）
            base = math.atan2(cy - ly, cx - lx)
            span = 0.0
            for p in cl:
                a = math.atan2(p[1] - ly, p[0] - lx) - base
                a = (a + math.pi) % (2 * math.pi) - math.pi
                span = max(span, abs(a))
            span_deg = math.degrees(2 * span)
            # 半径落在 φ50 的合理范围（小圆弧拟合偏小，下限放到 0.4 倍），
            # 残差够小，点数够多。φ300 转盘这种大圆会因半径大得多被排除。
            if (r_nom * 0.4 <= r <= r_nom * 2.2
                    and rms <= max(12.0, r_nom * 0.6)
                    and n >= min_n):
                out.append({"x": fx, "y": fy, "r": r, "r_plan": r_nom, "n": n,
                            "rms": rms, "kind": "circle", "dist": dist,
                            "span_deg": span_deg,
                            "r_confident": span_deg >= 6.0})
                continue

        # 聚出来了但不像 φ50 的圆：直墙、板子、转盘、墙角。
        # 一律当"未知障碍"标出来（规划时同 r_plan 膨胀），但绝不认成圆柱。
        out.append({"x": cx, "y": cy, "r": None, "r_plan": r_nom * 1.5,
                    "n": n, "rms": (fit[3] if fit else None),
                    "kind": "blob", "dist": dist})
    # 按距离排，近的优先显示
    out.sort(key=lambda c: c["dist"])
    return out


def centerline_cost(field, bias=3.0):
    """给"不在格中线上"的格子加代价，逼 A* 贴着 5×5 的中线走。

    为什么用代价而不是硬限制：整格封锁时（比如 C4 被圆柱占住），
    中线可能整条走不通，这时必须允许临时离开中线绕一下。
    用代价就是"能走中线就走中线，被挡了才偏出去"。

    bias 越大越贴中线；3.0 大约相当于"偏离半格 = 多走半格"。
    """
    xs, ys = field.gx, field.gy
    xc = [(xs[i] + xs[i + 1]) / 2.0 for i in range(len(xs) - 1)]
    yc = [(ys[i] + ys[i + 1]) / 2.0 for i in range(len(ys) - 1)]

    def nearest(v, arr):
        return min(abs(v - a) for a in arr)

    def cost(ix, iy):
        x, y = -field.hw + (ix + 0.5) * field.cell, -field.hh + (iy + 0.5) * field.cell
        off = min(nearest(x, xc), nearest(y, yc))       # 只要求贴着任意一条中线
        return bias * (off / field.cell_mm)

    return cost


def snap_path(path, field, cylinders, pad, cell_block, grid, tol=60.0):
    """把折线吸附到 5×5 格中线上（只吸附 60mm 以内的，且吸附后必须仍然合法）。

    为什么需要这步：A* 在 20mm 栅格上跑，"离中线 10mm"和"离中线 -10mm"代价一样，
    算出来的线会在 ±10mm 之间来回抖。吸一下，线路才是干净的一条直线。
    """
    xc = [(field.gx[i] + field.gx[i + 1]) / 2.0 for i in range(len(field.gx) - 1)]
    yc = [(field.gy[i] + field.gy[i + 1]) / 2.0 for i in range(len(field.gy) - 1)]

    def legal(x, y):
        if not field.is_drivable(x, y):
            return False
        ix, iy = grid.w2c(x, y)
        if cell_block is not None and 0 <= ix < grid.nx and 0 <= iy < grid.ny:
            if cell_block[iy][ix]:
                return False
        for c in cylinders:
            if math.hypot(x - c["x"], y - c["y"]) < _ob_radius(c) + pad:
                return False
        return True

    out = []
    for (x, y) in path:
        sx = min(xc, key=lambda v: abs(v - x))
        sy = min(yc, key=lambda v: abs(v - y))
        nx = sx if abs(sx - x) <= tol else x
        ny = sy if abs(sy - y) <= tol else y
        if (nx, ny) != (x, y) and legal(nx, ny):
            out.append((nx, ny))
        else:
            out.append((x, y))
    # 去重 + 去共线
    ded = [out[0]] if out else []
    for p in out[1:]:
        if abs(p[0] - ded[-1][0]) > 1e-6 or abs(p[1] - ded[-1][1]) > 1e-6:
            ded.append(p)
    if len(ded) < 3:
        return ded
    res = [ded[0]]
    for i in range(1, len(ded) - 1):
        ax, ay = ded[i][0] - ded[i - 1][0], ded[i][1] - ded[i - 1][1]
        bx, by = ded[i + 1][0] - ded[i][0], ded[i + 1][1] - ded[i][1]
        if abs(ax * by - ay * bx) > 1e-6:
            res.append(ded[i])
    res.append(ded[-1])
    return res


def split_backbone(path, field, tol=None):
    """把一条折线切成"主干段"和"靠泊段"。

    主干段 = 还贴着格中线的部分；靠泊段 = 最后离开中线去够工位的那一小段。
    返回 (主干折线, 靠泊折线, 靠泊长度mm)
    """
    if not path:
        return [], [], 0.0
    if tol is None:
        tol = 40.0
    xc = [(field.gx[i] + field.gx[i + 1]) / 2.0 for i in range(len(field.gx) - 1)]
    yc = [(field.gy[i] + field.gy[i + 1]) / 2.0 for i in range(len(field.gy) - 1)]

    def on_line(p):
        return (min(abs(p[0] - v) for v in xc) <= tol) or (min(abs(p[1] - v) for v in yc) <= tol)

    cut = len(path) - 1
    for i in range(len(path) - 1, -1, -1):
        if on_line(path[i]):
            cut = i
            break
    backbone = path[:cut + 1]
    spur = path[cut:]
    spur_mm = sum(math.hypot(spur[k + 1][0] - spur[k][0], spur[k + 1][1] - spur[k][1])
                  for k in range(len(spur) - 1))
    return backbone, spur, spur_mm


# ==================== 4. A* 路径规划 ====================


def astar(mask, blocked, start, goal, allow_diag=True, cost_fn=None):
    """在栅格上找路。

    mask   : 可通行掩码（车道）
    blocked: 障碍掩码（圆柱 + 膨胀）
    start/goal: (ix, iy)
    返回 (path, cost)  path 是 [(ix,iy), ...]；找不到返回 ([], inf)
    """
    import heapq

    nx = len(mask[0])
    ny = len(mask)

    def free(ix, iy):
        return 0 <= ix < nx and 0 <= iy < ny and mask[iy][ix] and not blocked[iy][ix]

    if not free(*start):
        # 起点被卡住（车压到膨胀后的禁区），就近找一个可走格
        best, bd = None, 1e18
        for iy in range(ny):
            for ix in range(nx):
                if free(ix, iy):
                    d = (ix - start[0]) ** 2 + (iy - start[1]) ** 2
                    if d < bd:
                        bd, best = d, (ix, iy)
                        if d == 0:
                            break
        if best is None:
            return [], float("inf")
        start = best
    if not free(*goal):
        best, bd = None, 1e18
        for iy in range(ny):
            for ix in range(nx):
                if free(ix, iy):
                    d = (ix - goal[0]) ** 2 + (iy - goal[1]) ** 2
                    if d < bd:
                        bd, best = d, (ix, iy)
        if best is None:
            return [], float("inf")
        goal = best

    if allow_diag:
        nb = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
              (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
              (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))]
    else:
        nb = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0)]

    def h(a, b):
        dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
        if allow_diag:
            return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)
        return dx + dy

    openq = [(h(start, goal), 0.0, start)]
    g = {start: 0.0}
    parent = {}
    closed = set()
    while openq:
        _, gc, cur = heapq.heappop(openq)
        if cur in closed:
            continue
        closed.add(cur)
        if cur == goal:
            path = [cur]
            while cur in parent:
                cur = parent[cur]
                path.append(cur)
            path.reverse()
            return path, gc
        cx, cy = cur
        for dx, dy, w in nb:
            nxt = (cx + dx, cy + dy)
            if not free(*nxt):
                continue
            # 斜着走要求两个正交邻居都通，避免切角穿墙
            if dx and dy and not (free(cx + dx, cy) and free(cx, cy + dy)):
                continue
            ng = gc + w + (cost_fn(nxt[0], nxt[1]) if cost_fn else 0.0)
            if ng < g.get(nxt, 1e18):
                g[nxt] = ng
                parent[nxt] = cur
                heapq.heappush(openq, (ng + h(nxt, goal), ng, nxt))
    return [], float("inf")


def simplify_path(path):
    """把格路径压成折线（只保留拐点），画图和下发都更好用。"""
    if len(path) < 3:
        return path[:]
    out = [path[0]]
    for i in range(1, len(path) - 1):
        ax, ay = path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1]
        bx, by = path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1]
        if (ax, ay) != (bx, by):
            out.append(path[i])
    out.append(path[-1])
    return out


def line_clear(mask, blocked, a, b):
    """a→b 这条直线上有没有不通行/被挡的格子。用于路径拉直。"""
    nx, ny = len(mask[0]), len(mask)
    dx, dy = b[0] - a[0], b[1] - a[1]
    steps = max(abs(dx), abs(dy))
    if steps == 0:
        return True
    for i in range(steps + 1):
        ix = int(round(a[0] + dx * i / steps))
        iy = int(round(a[1] + dy * i / steps))
        if not (0 <= ix < nx and 0 <= iy < ny):
            return False
        if not mask[iy][ix] or blocked[iy][ix]:
            return False
    return True


def shortcut_path(mask, blocked, path):
    """把 A* 的"台阶"拉直：能直接看到远处那个点就跳过去。

    不拉直的话画出来是锯齿，下发给车也是几十个小段。
    """
    if len(path) < 3:
        return path[:]
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not line_clear(mask, blocked, path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out


def clearance_mm(path, grid, cylinders):
    """沿路径的最小余量（mm）：格子中心到最近障碍表面，再减去车半径的余数。

    小于 0 说明车边贴着障碍了；实际经验值 <20mm 就别指望跑得准。
    """
    if not path or not cylinders:
        return float("inf")
    best = float("inf")
    for ix, iy in path:
        wx, wy = grid.c2w(ix, iy)
        for c in cylinders:
            go = math.hypot(wx - c["x"], wy - c["y"]) - _ob_radius(c)
            if go < best:
                best = go
    return best


def plan_path(grid, mask, cylinders, start, goal, robot_radius,
              min_clear_mm=20.0, cell_block=None, cost_fn=None):
    """带退让的规划：先按理想车半径规划，走不通就逐步缩小膨胀半径重试。

    为什么要退让：车道只有 400 宽，放一个 φ50 圆柱后只剩 350 的缝，
    车 300 宽——理论上过得去，但按"车半径 170 + 余量"去膨胀就没有解了。
    现场的实际情况是"能挤就挤"，所以退化成几档再试，并如实报告最小余量。

    cell_block: 可选的整格禁行掩码（5×5 里被圆柱占住的格子），优先级最高，
                不退让——它代表"这个区域根本不能通车"。

    返回 dict: path / cost / pad / scale / min_clear / note / goal_blocked / start_blocked
    """
    nx, ny = len(mask[0]), len(mask)

    def _free(ix, iy):
        if not (0 <= ix < nx and 0 <= iy < ny):
            return False
        if not mask[iy][ix]:
            return False
        if cell_block is not None and cell_block[iy][ix]:
            return False
        return True

    goal_blocked = not _free(*goal)
    start_blocked = not _free(*start)

    scales = (1.0, 0.9, 0.8, 0.7, 0.6, 0.5)
    best = None
    for sc in scales:
        pad = robot_radius * sc
        blocked = grid.build_blocked(cylinders, pad)
        if cell_block is not None:
            for iy in range(grid.ny):
                row, cb = blocked[iy], cell_block[iy]
                for ix in range(grid.nx):
                    if cb[ix]:
                        row[ix] = True
        # 沿中线的模式用 4 连通：路径严格正交（只有直行和原地转），
        # 正好对上 motion_planner 的 move/turn 指令集。
        path, cost = astar(mask, blocked, start, goal,
                           allow_diag=(cost_fn is None), cost_fn=cost_fn)
        if not path:
            continue
        # 沿中线的模式不能拉直：拉直会连线切角，把"走中线"全毁了。
        # 这里只做"去掉共线点"，台阶就被压成一条干净的正交折线。
        path = shortcut_path(mask, blocked, path) if not cost_fn else simplify_path(path)
        clear = clearance_mm(path, grid, cylinders)
        note = "正常通过" if sc == 1.0 else f"放宽到车半径的 {sc*100:.0f}% 才通得过"
        # 世界坐标折线：沿中线模式要把小幅抖动吸附到格中线上
        world = [grid.c2w(ix, iy) for ix, iy in path]
        if cost_fn:
            world = snap_path(world, grid.field, cylinders, pad, cell_block, grid)
        best = {"path": path, "path_world": world, "cost": cost, "pad": pad, "scale": sc,
                "min_clear": clear, "note": note,
                "goal_blocked": goal_blocked, "start_blocked": start_blocked}
        break
    if best is None:
        return {"path": [], "path_world": [], "cost": float("inf"), "pad": robot_radius,
                "scale": 0.0, "min_clear": 0.0, "note": "无解：路被完全堵死",
                "goal_blocked": goal_blocked, "start_blocked": start_blocked}
    if best["min_clear"] < min_clear_mm:
        best["note"] += f"（余量仅 {best['min_clear']:.0f}mm，很勉强）"
    return best


def plan_mission(field, grid, mask, cylinders, route_names=None,
                 robot_radius=170.0, cell_block=None, cost_fn=None):
    """整条任务路线：逐段 A* + 汇总。

    route_names 默认取 field.mission_route（START → 扫码 → 原料 → 粗加工 → 暂存 → … → START）。
    返回 (legs, total_mm)：
        legs[i] = {from_name, to_name, from_xy, to_xy, path(世界坐标), mm, ok, note, min_clear}
    """
    names = list(route_names or field.mission_route)
    pts = [field.resolve_waypoint(n) for n in names]
    legs = []
    total = 0.0
    for i in range(len(names) - 1):
        a, b = pts[i], pts[i + 1]
        leg = {"from_name": names[i], "to_name": names[i + 1],
               "from_xy": a, "to_xy": b, "path": [], "mm": 0.0,
               "ok": False, "note": "", "min_clear": 0.0}
        if a is None or b is None:
            leg["note"] = "名字解析不到坐标"
            legs.append(leg)
            continue
        res = plan_path(grid, mask, cylinders, grid.w2c(*a), grid.w2c(*b),
                        robot_radius, cell_block=cell_block, cost_fn=cost_fn)
        wp = res.get("path_world") or [grid.c2w(ix, iy) for ix, iy in res["path"]]
        leg["path"] = wp
        leg["backbone"], leg["spur"], leg["spur_mm"] = split_backbone(wp, field)
        leg["mm"] = sum(math.hypot(wp[k + 1][0] - wp[k][0], wp[k + 1][1] - wp[k][1])
                        for k in range(len(wp) - 1))
        leg["ok"] = bool(wp)
        leg["note"] = res["note"] if wp else res["note"]
        leg["min_clear"] = res["min_clear"]
        leg["goal_blocked"] = res.get("goal_blocked", False)
        total += leg["mm"]
        legs.append(leg)
    return legs, total


# ==================== 5. 加载配置 ====================


def load_config(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)
