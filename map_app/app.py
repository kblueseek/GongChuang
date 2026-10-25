#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""智能搬运·场上雷达上位机（单文件 GUI，tkinter，零第三方依赖*）。

* 只有接真雷达时需要 pyserial；用模拟雷达不需要任何库。

================================ 它干什么 ================================

和塔克那套 cspc 上位机同一个思路，但换成本比赛要的东西：

    1. 按参数画出智能搬运场地（尺寸随便改，改 field.json 或界面上直接调）；
    2. 雷达架在启停区，启动时扫几圈，把回波打到地图上；
    3. 自动把 φ50 的黑色圆柱挑出来，在地图上画圈 + 标坐标；
    4. 在灰色车道上跑 A*，给出一条绕开圆柱的路径。

为什么地图是参数画的、不是雷达扫出来的：
    二维雷达只看得见"立在扫描平面里的东西"。地面上的灰色车道、淡黄色方块
    是平的，雷达一点回波都没有。所以地图只能来自参数模型，雷达负责在模型上
    标障碍物。现场尺寸变了，改参数即可。

================================ 怎么用 ================================

    python app.py                       # 打开界面
    python app.py --cfg field.json      # 指定配置文件

界面右边是地图。左键点一下地图 = 把那里设成目标点，自动规划路径。
"""

from __future__ import annotations

import json
import math
import os
import queue
import sys
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import lidar_d6
import mapping

HERE = os.path.dirname(os.path.abspath(__file__))
CFG_PATH = os.path.join(HERE, "field.json")

# ---------------- 配色 ----------------
C_BG      = "#ffffff"
C_WHITE   = "#f8f8f6"
C_LANE    = "#cbcbcb"
C_CONN    = "#e2e2e2"
C_YELLOW  = "#fdf7cd"
C_BLUE    = "#1a3ceb"
C_POINT   = "#696969"
C_CYL     = "#e21e3c"
C_BLOB    = "#aa6e14"
C_PATH    = "#146ede"
C_LIDAR   = "#008c50"
C_GOAL    = "#8e24aa"
C_TEXT    = "#2c2c2c"
C_GRID    = "#dddddd"


class App:
    def __init__(self, root: tk.Tk, cfg_path: str):
        self.root = root
        self.cfg_path = cfg_path
        self.cfg = mapping.load_config(cfg_path)
        self.field = mapping.FieldModel(self.cfg)
        self.grid = mapping.Grid(self.field)
        self.mask = self.grid.build_drivable()

        self.raw_points = []      # 雷达原始 (设备角, 距离, 强度) —— 改 yaw/角度方向时重投影用
        self.points = []          # 世界坐标点云 [(x,y,i), ...]
        self.obstacles = []       # 检出的障碍
        self.blocked_cells = set()  # 被障碍占住、整格不能通车的 5×5 格 {(c,r)}
        self.plan = None          # 最近一次单点规划结果
        self.mission_legs = []    # 整条任务路线的分段结果
        self.mission_total = 0.0  # 总长度 mm
        self._last_hover = None   # 鼠标最后的像素位置（重画后要恢复十字线）
        self.goal_world = None    # 目标点
        self.lidar = None         # 当前雷达对象（真 or 模拟）
        self.lidar_thread_q = queue.Queue()   # 后台 → 界面 的数据队列
        self.scanning = False
        self.scan_revs_done = 0
        self.scan_target_revs = int(self.cfg["scan"]["revolutions"])
        self._rev_hits = 0

        root.title("智能搬运 · 场上雷达上位机")
        root.geometry("1620x1000")
        self._build_ui()
        self._refresh()

    # ==================== 界面 ====================

    def _build_ui(self):
        self.status = tk.StringVar(value="就绪。建议先勾「模拟雷达」点【连接并开始】看效果。")
        ttk.Label(self.root, textvariable=self.status, relief=tk.SUNKEN,
                  anchor="w", padding=4).pack(side=tk.BOTTOM, fill=tk.X)
        body = ttk.Frame(self.root)
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        left = ttk.Frame(body, padding=6)
        left.pack(side=tk.LEFT, fill=tk.Y)

        # ---- 串口 ----
        box = ttk.LabelFrame(left, text="雷达 / 串口", padding=4)
        box.pack(fill=tk.X, pady=(0, 3))

        ports = self._ports()
        real = [p for p in ports if not p.startswith("(")]
        row = ttk.Frame(box); row.pack(fill=tk.X, pady=0)
        ttk.Label(row, text="串口", width=6).pack(side=tk.LEFT)
        self.cb_port = ttk.Combobox(row, width=16, values=ports)
        self.cb_port.set(self._prefer_radar(real) if real else self.cfg["lidar"].get("port", "COM3"))
        self.cb_port.pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(row, text="刷新", width=5,
                   command=lambda: self.cb_port.configure(values=self._ports())).pack(side=tk.LEFT)

        row = ttk.Frame(box); row.pack(fill=tk.X, pady=0)
        ttk.Label(row, text="波特率", width=6).pack(side=tk.LEFT)
        self.cb_baud = ttk.Combobox(row, width=12, values=[115200, 153600, 230400, 460800])
        self.cb_baud.set(self.cfg["lidar"].get("baud", 230400))
        self.cb_baud.pack(side=tk.LEFT, fill=tk.X, expand=True)

        self.var_cw = tk.BooleanVar(value=self.field.angle_cw)
        ttk.Checkbutton(box, text="角度顺时针（D6 默认勾；南北镜像就点它）",
                        variable=self.var_cw,
                        command=self._apply_cw).pack(anchor="w", pady=(2, 0))

        # 有真串口就默认走真机；一个串口都没找到才默认模拟
        self.var_sim = tk.BooleanVar(value=not bool(real))
        ttk.Checkbutton(box, text="用模拟雷达（演示用，不读串口）",
                        variable=self.var_sim).pack(anchor="w", pady=(4, 0))

        row = ttk.Frame(box); row.pack(fill=tk.X, pady=4)
        self.btn_conn = ttk.Button(row, text="连接并开始", command=self.toggle_lidar)
        self.btn_conn.pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(row, text="停止", command=self.stop_lidar).pack(side=tk.LEFT, padx=(4, 0))

        # ---- 雷达安装位 ----
        box = ttk.LabelFrame(left, text="雷达安装位置", padding=4)
        box.pack(fill=tk.X, pady=3)
        row = ttk.Frame(box); row.pack(fill=tk.X)
        ttk.Label(row, text="放在", width=6).pack(side=tk.LEFT)
        self.cb_mount = ttk.Combobox(row, width=14, state="readonly",
                                     values=[z["name"] for z in self.field.start_zones] + ["自定义"])
        self.cb_mount.set(self.cfg["lidar"].get("mount", "启停区1"))
        self.cb_mount.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.cb_mount.bind("<<ComboboxSelected>>", lambda e: self._apply_mount())

        gxy = ttk.Frame(box)
        gxy.pack(fill=tk.X)
        self.ent_lx = self._grid_entry(gxy, 0, 0, "x", self.field.lidar_x, self._apply_lidar)
        self.ent_ly = self._grid_entry(gxy, 0, 1, "y", self.field.lidar_y, self._apply_lidar)
        self.ent_lz = self._row_entry(box, "yaw (°)", self.field.lidar_yaw, self._apply_lidar)
        rw = ttk.Frame(box); rw.pack(fill=tk.X)
        ttk.Label(rw, text="微调", width=6).pack(side=tk.LEFT)
        for txt, d in (("◀ 15°", -15), ("◀ 5°", -5), ("5° ▶", 5), ("15° ▶", 15)):
            ttk.Button(rw, text=txt, width=5,
                       command=lambda dd=d: self._nudge_yaw(dd)).pack(side=tk.LEFT, padx=1)

        # ---- 场地尺寸 ----
        box = ttk.LabelFrame(left, text="场地尺寸（现场量了直接改）", padding=4)
        box.pack(fill=tk.X, pady=3)
        gf = ttk.Frame(box)
        gf.pack(fill=tk.X)
        self.ent_w = self._grid_entry(gf, 0, 0, "宽", self.field.W, self._apply_field)
        self.ent_h = self._grid_entry(gf, 0, 1, "高", self.field.H, self._apply_field)
        self.ent_lane = self._grid_entry(gf, 1, 0, "车道", self.field.lane_w, self._apply_field)
        self.ent_yb = self._grid_entry(gf, 1, 1, "黄块", self.cfg["yellow_blocks"]["size_mm"],
                                       self._apply_field)

        # ---- 圆柱参数 ----
        box = ttk.LabelFrame(left, text="圆柱识别", padding=4)
        box.pack(fill=tk.X, pady=3)
        cc = self.cfg["cylinder"]
        self.ent_dia = self._row_entry(box, "直径 (mm)", cc["diameter_mm"], self._apply_detect)
        self.ent_gap = self._row_entry(box, "聚类间距", cc["cluster_gap_mm"], self._apply_detect)
        self.ent_minn = self._row_entry(box, "最少点数", cc["min_points"], self._apply_detect)

        # ---- 操作 ----
        box = ttk.LabelFrame(left, text="操作", padding=4)
        box.pack(fill=tk.X, pady=3)
        self.btn_scan = ttk.Button(box, text="① 扫描建图（扫几圈后冻结）", command=self.start_scan)
        self.btn_scan.pack(fill=tk.X, pady=1)
        ttk.Button(box, text="② 重新识别障碍", command=self.detect).pack(fill=tk.X, pady=1)
        ttk.Button(box, text="③ 规划到目标（也可直接点地图）",
                   command=self.plan_to_goal).pack(fill=tk.X, pady=1)
        ttk.Button(box, text="清空点云", command=self.clear).pack(fill=tk.X, pady=1)
        ttk.Button(box, text="保存场地参数", command=self.save_cfg).pack(fill=tk.X, pady=1)
        row = ttk.Frame(box); row.pack(fill=tk.X, pady=(2, 0))
        ttk.Label(row, text="目标格").pack(side=tk.LEFT)
        self.ent_cell = ttk.Entry(row, width=6)
        self.ent_cell.insert(0, "C3")
        self.ent_cell.pack(side=tk.LEFT, padx=2)
        self.ent_cell.bind("<Return>", lambda e: self.goto_cell())
        ttk.Button(row, text="去", width=4, command=self.goto_cell).pack(side=tk.LEFT)

        cf0 = ttk.Frame(box)
        cf0.pack(fill=tk.X)
        self.var_strict = tk.BooleanVar(value=True)
        ttk.Checkbutton(cf0, text="整格封锁", variable=self.var_strict,
                        command=self._replan_soft).grid(row=0, column=0, sticky="w")
        cf = ttk.Frame(box)
        cf.pack(fill=tk.X)
        self.var_grid = tk.BooleanVar(value=True)
        self.var_line = tk.BooleanVar(value=True)
        ttk.Checkbutton(cf, text="沿格中线", variable=self.var_line,
                        command=self._replan_soft).grid(row=0, column=0, sticky="w")
        ttk.Checkbutton(cf, text="显示网格", variable=self.var_grid,
                        command=self._redraw).grid(row=0, column=1, sticky="w", padx=(8, 0))
        self.var_snap = tk.BooleanVar(value=True)
        ttk.Checkbutton(cf0, text="吸附格心", variable=self.var_snap,
                        command=self._redraw).grid(row=0, column=1, sticky="w", padx=(8, 0))

        # ---- 检出列表 ----
        box = ttk.LabelFrame(left, text="检出的障碍", padding=4)
        box.pack(fill=tk.BOTH, expand=True, pady=(3, 0))
        self.lst = tk.Listbox(box, height=3, font=("Consolas", 9))
        self.lst.pack(fill=tk.BOTH, expand=True)

        # ---- 右栏：任务路线 ----
        rp = ttk.Frame(body, padding=(6, 8, 8, 8), width=250)
        rp.pack(side=tk.RIGHT, fill=tk.Y)
        rp.pack_propagate(False)

        ttk.Label(rp, text="任务路线（从上到下）",
                  font=("Microsoft YaHei", 10, "bold")).pack(anchor="w")
        self.mission_lst = tk.Listbox(rp, height=10, font=("Microsoft YaHei", 9))
        self.mission_lst.pack(fill=tk.X, pady=2)

        ttk.Button(rp, text="规划全程（逐段避障）", command=self.plan_mission).pack(fill=tk.X, pady=(2, 1))
        ttk.Button(rp, text="导出动作序列 JSON", command=self.export_route).pack(fill=tk.X, pady=1)
        ttk.Button(rp, text="清空路线", command=self.clear_route).pack(fill=tk.X, pady=1)

        self.mission_info = tk.StringVar(value="还没规划")
        ttk.Label(rp, textvariable=self.mission_info, wraplength=230,
                  foreground="#146ede").pack(anchor="w", pady=(6, 2))

        ttk.Label(rp, text="分段结果").pack(anchor="w", pady=(4, 0))
        self.leg_box = tk.Text(rp, height=10, width=28, font=("Consolas", 8),
                               relief="flat", background="#f7f7f5")
        self.leg_box.pack(fill=tk.BOTH, expand=True)

        # ---- 中间画布 ----
        right = ttk.Frame(body)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.canvas = tk.Canvas(right, bg=C_BG, highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Button-1>", self.on_click)
        self.canvas.bind("<Motion>", self.on_motion)
        self.canvas.bind("<Leave>", lambda e: self._clear_hover())
        self.canvas.bind("<Configure>", lambda e: self._redraw())
        self._fill_mission_list()

    def _row_entry(self, parent, label, value, cb):
        row = ttk.Frame(parent); row.pack(fill=tk.X, pady=0)
        ttk.Label(row, text=label, width=9).pack(side=tk.LEFT)
        e = ttk.Entry(row, width=9)
        e.insert(0, f"{float(value):g}")
        e.pack(side=tk.LEFT, fill=tk.X, expand=True)
        e.bind("<Return>", lambda ev: cb())
        e.bind("<FocusOut>", lambda ev: cb())
        return e

    def _grid_entry(self, parent, r, c, label, value, cb):
        """两列布局的一格：标签 + 输入框，省一半纵向空间。"""
        ttk.Label(parent, text=label).grid(row=r, column=c * 2, sticky="w")
        e = ttk.Entry(parent, width=6)
        e.insert(0, f"{float(value):g}")
        e.grid(row=r, column=c * 2 + 1, sticky="we", padx=(2, 8), pady=1)
        e.bind("<Return>", lambda ev: cb())
        e.bind("<FocusOut>", lambda ev: cb())
        parent.columnconfigure(c * 2 + 1, weight=1)
        return e

    @staticmethod
    def _ports():
        """真实串口列表。格式 'COM22 · USB-SERIAL CH340'。

        以前没装 pyserial 时会返回假的 COM3/4/5，让人以为"有串口但连不上"，
        现在直接写清楚原因。
        """
        try:
            from serial.tools import list_ports
        except Exception:
            return ["(未装 pyserial)"]
        out = []
        for p in list_ports.comports():
            desc = (p.description or "").strip()
            out.append(f"{p.device} · {desc}" if desc else p.device)
        return out or ["(没插串口)"]

    def _port_name(self):
        """从 'COM22 · USB-SERIAL CH340' 里取出 'COM22'。"""
        return self.cb_port.get().strip().split("·")[0].split(" ")[0].strip()

    @staticmethod
    def _prefer_radar(ports):
        """优先选 CH340/CP210x/FTDI/SILICON 这类 USB 转串口芯片——雷达一般就是它。"""
        keys = ("CH340", "CP210", "FTDI", "SILICON", "USB-SERIAL", "USB 串行")
        for p in ports:
            if any(k.upper() in p.upper() for k in keys):
                return p
        return ports[0] if ports else "COM3"

    # ==================== 参数改动 ====================

    def _set_status(self, s):
        self.status.set(s)

    def _apply_mount(self):
        name = self.cb_mount.get()
        for z in self.field.start_zones:
            if z["name"] == name:
                self._set(self.ent_lx, z["cx"])
                self._set(self.ent_ly, z["cy"])
                # 朝向场地中心
                yaw = math.degrees(math.atan2(-z["cy"], -z["cx"])) % 360
                self._set(self.ent_lz, round(yaw, 1))
                self._apply_lidar()
                return

    @staticmethod
    def _set(entry, value):
        entry.delete(0, tk.END)
        entry.insert(0, f"{float(value):g}")

    def _num(self, entry, default):
        try:
            return float(entry.get())
        except Exception:
            return float(default)

    def _reproject(self):
        """按当前 yaw / 角度方向把原始点重算成世界坐标。

        有这个，改 yaw 或勾角度方向时点云会**立刻跟着动**，不用重扫——
        标定 yaw 就变成了"拧一下看一眼"。
        """
        x0, y0 = self.field.lidar_x, self.field.lidar_y
        del x0, y0
        self.points = [(*self.field.lidar_to_world(a, d), i)
                       for (a, d, i) in self.raw_points]

    def _apply_lidar(self):
        self.field.lidar_x = self._num(self.ent_lx, self.field.lidar_x)
        self.field.lidar_y = self._num(self.ent_ly, self.field.lidar_y)
        self.field.lidar_yaw = self._num(self.ent_lz, self.field.lidar_yaw)
        self.cfg["lidar"]["x"] = self.field.lidar_x
        self.cfg["lidar"]["y"] = self.field.lidar_y
        self.cfg["lidar"]["yaw_deg"] = self.field.lidar_yaw
        self.cfg["lidar"]["mount"] = self.cb_mount.get()
        self.cfg["lidar"]["angle_cw"] = self.field.angle_cw
        if self.raw_points:
            self._reproject()
            self.detect()
        if hasattr(self, "mission_lst"):
            self._fill_mission_list()
        self._redraw()

    def _apply_cw(self):
        """切换设备角度方向（顺时针为正 / 逆时针为正），立刻重投影。"""
        self.field.angle_cw = bool(self.var_cw.get())
        self.field.angle_sign = -1.0 if self.field.angle_cw else 1.0
        self.cfg["lidar"]["angle_cw"] = self.field.angle_cw
        if self.raw_points:
            self._reproject()
            self.detect()
        else:
            self._redraw()
        self._set_status("角度方向：" + ("顺时针为正（已取反）" if self.field.angle_cw
                                        else "逆时针为正（不取反）"))

    def _nudge_yaw(self, delta):
        """转一下 yaw 立刻重画——拿实物对着地图拧，比算快。"""
        cur = self._num(self.ent_lz, self.field.lidar_yaw)
        self._set(self.ent_lz, (cur + delta) % 360)
        self._apply_lidar()
        self._set_status(f"yaw = {(cur + delta) % 360:.0f}°　"
                         f"（点云和实景对上了就停）")

    def _apply_field(self):
        self.cfg["field"]["width_mm"] = self._num(self.ent_w, self.field.W)
        self.cfg["field"]["height_mm"] = self._num(self.ent_h, self.field.H)
        self.cfg["lane"]["width_mm"] = self._num(self.ent_lane, self.field.lane_w)
        self.cfg["yellow_blocks"]["size_mm"] = self._num(self.ent_yb, 450)
        self._rebuild()

    def _apply_detect(self):
        cc = self.cfg["cylinder"]
        cc["diameter_mm"] = self._num(self.ent_dia, 50)
        cc["cluster_gap_mm"] = self._num(self.ent_gap, 90)
        cc["min_points"] = int(self._num(self.ent_minn, 4))

    def _rebuild(self):
        self.field = mapping.FieldModel(self.cfg)
        self.grid = mapping.Grid(self.field)
        self.mask = self.grid.build_drivable()
        self._redraw()
        self._set_status(f"场地已更新：{self.field.W:.0f}×{self.field.H:.0f}mm，"
                         f"车道宽 {self.field.lane_w:.0f}mm")

    def save_cfg(self):
        try:
            with open(self.cfg_path, "w", encoding="utf-8") as f:
                json.dump(self.cfg, f, ensure_ascii=False, indent=2)
            self._set_status(f"已保存到 {self.cfg_path}")
        except Exception as e:
            messagebox.showerror("保存失败", str(e))

    # ==================== 雷达开关 ====================

    def toggle_lidar(self):
        if self.lidar is not None:
            self.stop_lidar()
            return
        self._apply_lidar()
        if self.var_sim.get():
            obs = self._sim_obstacles()
            self.lidar = lidar_d6.SimLidar(self.field, obs,
                                           on_frame=self._on_frame,
                                           on_log=lambda s: self.lidar_thread_q.put(("log", s)))
        else:
            self.lidar = lidar_d6.D6Lidar(self._port_name(), int(self.cb_baud.get()),
                                          on_frame=self._on_frame,
                                          on_log=lambda s: self.lidar_thread_q.put(("log", s)))
        if not self.lidar.open():
            err = getattr(self.lidar, "error", "打开失败")
            self.lidar = None
            messagebox.showerror("雷达打不开", err)
            return
        self.btn_conn.configure(text="断开")
        mode = "模拟" if self.var_sim.get() else f"真机 {self._port_name()}"
        self._set_status(f"【{mode}】已连接，点【① 扫描建图】开始")

    def stop_lidar(self):
        if self.lidar:
            self.lidar.close()
            self.lidar = None
        self.scanning = False
        self.btn_conn.configure(text="连接并开始")

    def _sim_obstacles(self):
        """模拟用的"真实"障碍：从界面上现有的检出结果造，没有就摆两个。"""
        obs = [{"x": c["x"], "y": c["y"], "r": mapping._ob_radius(c), "intensity": 60}
               for c in self.obstacles]
        if not obs:
            obs = [{"x": 110, "y": 620, "r": 25, "intensity": 60},
                   {"x": -60, "y": -300, "r": 25, "intensity": 60},
                   {"x": -150, "y": 1150, "r": 150, "intensity": 90}]
        return obs

    # ==================== 数据回调（后台线程里） ====================

    def _on_frame(self, pts, ring):
        # 只在扫描阶段收数据。扫完就冻结——这是"启动时扫全场"的设计，
        # 比赛途中不再改地图，也避免点云无限增长拖慢界面。
        if self.scanning:
            self.lidar_thread_q.put(("frame", (pts, ring)))

    def _drain(self):
        """界面线程里把队列里的东西取出来处理。"""
        got = False
        while True:
            try:
                kind, payload = self.lidar_thread_q.get_nowait()
            except queue.Empty:
                break
            if kind == "log":
                self._set_status(str(payload))
            elif kind == "frame":
                pts, ring = payload
                if ring:
                    self._rev_hits = 0
                    if self.scanning:
                        self.scan_revs_done += 1
                for ang, d, inten in pts:
                    if d < self.cfg["scan"]["range_min_mm"] or d > self.cfg["scan"]["range_max_mm"]:
                        continue
                    self.raw_points.append((ang, d, inten))
                    x, y = self.field.lidar_to_world(ang, d)
                    self.points.append((x, y, inten))
                got = True
                if self.scanning and self.scan_revs_done >= self.scan_target_revs:
                    self._finish_scan()
        return got

    # ==================== 扫描流程 ====================

    def start_scan(self):
        if self.lidar is None:
            messagebox.showinfo("先连雷达", "先点【连接并开始】（没硬件就勾模拟雷达）")
            return
        self.raw_points = []
        self.points = []
        self.obstacles = []
        self.plan = None
        self.lst.delete(0, tk.END)
        self.scanning = True
        self.scan_revs_done = 0
        self.scan_target_revs = int(self.cfg["scan"]["revolutions"])
        self._set_status(f"正在扫描 … 目标 {self.scan_target_revs} 圈")

    def _finish_scan(self):
        self.scanning = False
        self._set_status(f"扫描完成：{len(self.points)} 个回波点，开始识别")
        self.detect()

    # ==================== 检测 / 规划 ====================

    def detect(self):
        if not self.points:
            self._set_status("还没有点云，先扫一下")
            return
        self.obstacles = mapping.detect_cylinders(self.points, self.cfg, self.field)
        self.lst.delete(0, tk.END)
        n_cyl = 0
        for c in self.obstacles:
            if c["kind"] == "circle":
                n_cyl += 1
                conf = "" if c.get("r_confident") else " 半径存疑"
                text = (f'[{self.field.cell_name(c["x"], c["y"])}] 圆柱 '
                        f'({c["x"]:7.1f},{c["y"]:7.1f}) 实测φ{c["r"]*2:.0f}mm '
                        f'n={c["n"]}{conf}')
            else:
                text = (f'[{self.field.cell_name(c["x"], c["y"])}] 障碍 '
                        f'({c["x"]:7.1f},{c["y"]:7.1f}) 尺寸未知 n={c["n"]}')
            self.lst.insert(tk.END, text)
        self.blocked_cells = set()
        skipped = []
        for ob in self.obstacles:
            cr = self.field.cell_at(ob["x"], ob["y"])
            if not cr:
                continue
            if self._is_station_cell(*cr):
                # 工位所在的格不封：封锁了就等于"永远到不了工位"。
                # 工位上的物体（转盘等）靠实体半径绕开就够了。
                skipped.append(self.field.cell_label(*cr))
                continue
            self.blocked_cells.add(cr)
        cells = sorted({self.field.cell_name(c["x"], c["y"]) for c in self.obstacles})
        extra = f"（{'、'.join(skipped)} 是工位格，不封锁）" if skipped else ""
        self._set_status(f"识别到 {n_cyl} 个圆柱，共 {len(self.obstacles)} 处障碍；"
                         f"占用格：{'、'.join(cells) if cells else '无'}{extra}")
        self._redraw()

    def plan_to_goal(self):
        if self.goal_world is None:
            self._set_status("先在地图上点一下目标点")
            return
        start = self.grid.w2c(self.field.lidar_x, self.field.lidar_y)
        goal = self.grid.w2c(*self.goal_world)
        cb = None
        if self.var_strict.get() and self.blocked_cells:
            cb = self.grid.build_cell_blocked(self.blocked_cells)
        cost_fn = mapping.centerline_cost(self.field) if self.var_line.get() else None
        self.plan = mapping.plan_path(self.grid, self.mask, self.obstacles,
                                      start, goal, self.field.robot_radius,
                                      cell_block=cb, cost_fn=cost_fn)
        warn = ""
        if self.plan.get("goal_blocked"):
            gname = self.field.cell_name(*self.goal_world)
            warn = f'  ⚠ {gname} 本身被封锁，已改到最近的可通行点'
        if self.plan["path"]:
            self._set_status(
                f'路径 {len(self.plan["path"])} 格，最小余量 '
                f'{self.plan["min_clear"]:.0f}mm —— {self.plan["note"]}{warn}')
        else:
            sealed = "、".join(sorted(self.field.cell_label(c, r)
                                     for (c, r) in self.blocked_cells))
            extra = f"（{sealed} 整格封锁，把路断了）" if sealed and self.var_strict.get() else ""
            self._set_status(f'规划失败：{self.plan["note"]}{extra}')
        self._redraw()

    def goto_cell(self):
        """按格子编号（A1~E5）下目标。列=A..E 从左到右，行=1..5 从下往上。"""
        s = self.ent_cell.get().strip().upper()
        if len(s) < 2 or not s[0].isalpha() or not s[1:].isdigit():
            self._set_status("格子编号写法：A1 ~ E5")
            return
        col = ord(s[0]) - ord("A")
        row = int(s[1:]) - 1
        if not (0 <= col < self.field.cell_cols and 0 <= row < self.field.cell_rows):
            self._set_status(f"超出范围：只有 A1~"
                             f"{chr(ord('A') + self.field.cell_cols - 1)}{self.field.cell_rows}")
            return
        self.goal_world = self.field.cell_center(col, row)
        self._set_status(f"目标 {self.field.cell_label(col, row)} "
                         f"({self.goal_world[0]:.0f}, {self.goal_world[1]:.0f}) mm")
        self.plan_to_goal()

    # ==================== 任务路线 ====================

    def _fill_mission_list(self):
        """把 field.json 里的 mission.route 填进右侧列表，START 显示成具体启停区。"""
        self.mission_lst.delete(0, tk.END)
        for i, name in enumerate(self.field.mission_route):
            if name in ("START", "start", "起点"):
                label = f"{i + 1}. 起点（{self.field.lidar_mount}）"
            else:
                label = f"{i + 1}. {name}"
            self.mission_lst.insert(tk.END, label)

    def plan_mission(self):
        """沿任务路线逐段规划。每段都独立避障，任何一段没有解都会标出来。"""
        if not self.field.mission_route:
            self._set_status("field.json 里没有 mission.route")
            return
        cb = None
        if self.var_strict.get() and self.blocked_cells:
            cb = self.grid.build_cell_blocked(self.blocked_cells)
        cost_fn = mapping.centerline_cost(self.field) if self.var_line.get() else None
        legs, total = mapping.plan_mission(self.field, self.grid, self.mask,
                                           self.obstacles, None,
                                           self.field.robot_radius, cell_block=cb,
                                           cost_fn=cost_fn)
        self.mission_legs = legs
        self.mission_total = total
        self.plan = None

        ok = sum(1 for l in legs if l["ok"])
        eta = total / max(self.field.mission_speed, 1.0)
        self.mission_info.set(f"{ok}/{len(legs)} 段可通　总长 {total / 1000:.2f} m　"
                              f"按 {self.field.mission_speed:.0f} mm/s 约 {eta:.0f} s")

        self.leg_box.delete("1.0", tk.END)
        for i, l in enumerate(legs, 1):
            mark = "OK " if l["ok"] else "✗  "
            spur = l.get("spur_mm", 0.0)
            self.leg_box.insert(tk.END,
                                f'{i:2d}. {l["from_name"][:4]:>4s}→{l["to_name"][:4]:<4s} '
                                f'{l["mm"] / 1000:5.2f}m 靠{spur:4.0f} {mark}\n')
            if not l["ok"]:
                self.leg_box.insert(tk.END, f'      {l["note"]}\n')
            elif l["min_clear"] < 20:
                self.leg_box.insert(tk.END, f'      余量 {l["min_clear"]:.0f}mm 偏紧\n')
        self._set_status(f"全程规划完成：{ok}/{len(legs)} 段可通，总长 {total / 1000:.2f} m")
        self._redraw()

    def clear_route(self):
        self.mission_legs = []
        self.mission_total = 0.0
        self.plan = None
        self.mission_info.set("还没规划")
        self.leg_box.delete("1.0", tk.END)
        self._redraw()

    def export_route(self):
        """把整条路线导成动作序列 JSON（move/turn），格式对齐 motion_planner。

        每段折线：先转向对准，再直行。方向约定：正 = 左转（逆时针）。
        注意：这是"按几何折线反推"的动作，实际下发前必须核对一遍符号和转角。
        """
        if not self.mission_legs:
            messagebox.showinfo("先规划", "先点【规划全程】")
            return
        path = filedialog.asksaveasfilename(defaultextension=".json",
                                            filetypes=[("JSON 文件", "*.json")],
                                            initialfile="route.json")
        if not path:
            return
        yaw = math.radians(self.field.lidar_yaw)      # 假设车出发时朝向和雷达一致
        cmds = [{"cmd": "comment", "text": f"任务路线 由 map_app 生成 "
                                           f"{len(self.mission_legs)} 段"}]
        for i, leg in enumerate(self.mission_legs, 1):
            cmds.append({"cmd": "comment",
                         "text": f'{i}. {leg["from_name"]} -> {leg["to_name"]}'})
            pts = leg["path"]
            for k in range(len(pts) - 1):
                dx = pts[k + 1][0] - pts[k][0]
                dy = pts[k + 1][1] - pts[k][1]
                if abs(dx) < 1 and abs(dy) < 1:
                    continue
                head = math.atan2(dy, dx)
                turn = math.degrees((head - yaw + math.pi) % (2 * math.pi) - math.pi)
                if abs(turn) > 1.0:
                    cmds.append({"cmd": "turn", "deg": round(turn, 1)})
                    yaw = head
                cmds.append({"cmd": "move", "dist_m": round(math.hypot(dx, dy) / 1000.0, 3)})
        with open(path, "w", encoding="utf-8") as f:
            json.dump({"commands": cmds}, f, ensure_ascii=False, indent=2)
        self._set_status(f"已导出 {len(cmds)} 条动作 → {path}（下发前请核对转向符号）")

    def _is_station_cell(self, c, r):
        """这个格子里有没有工位（原料区/粗加工区/暂存区/扫码区）。"""
        rc = self.field.cell_rect(c, r)
        for z in self.field.work_zones:
            zr = z["rect"]
            if not (zr.x1 < rc.x0 or zr.x0 > rc.x1 or zr.y1 < rc.y0 or zr.y0 > rc.y1):
                return True
        return False

    def _replan_soft(self):
        if self.goal_world is not None:
            self.plan_to_goal()
        else:
            self._redraw()

    def clear(self):
        self.raw_points = []
        self.points = []
        self.obstacles = []
        self.blocked_cells = set()
        self.plan = None
        self.scan_revs_done = 0
        self.lst.delete(0, tk.END)
        self._set_status("已清空")
        self._redraw()

    def on_motion(self, ev):
        """鼠标移动：显示十字线 + 坐标/格子/距雷达方位。"""
        self._last_hover = (ev.x, ev.y)
        self._draw_hover(ev.x, ev.y)

    def _clear_hover(self):
        self._last_hover = None
        self.canvas.delete("hover")

    def _draw_hover(self, px, py):
        c = self.canvas
        c.delete("hover")
        w = self._px_to_world(px, py)
        if w is None:
            return
        f = self.field
        x, y = w
        dist = math.hypot(x - f.lidar_x, y - f.lidar_y)
        brg = math.degrees(math.atan2(y - f.lidar_y, x - f.lidar_x)) % 360
        cell = f.cell_name(x, y)

        # 十字虚线
        c.create_line(px, 0, px, c.winfo_height(), fill="#7a7a7a",
                      dash=(3, 5), tags="hover")
        c.create_line(0, py, c.winfo_width(), py, fill="#7a7a7a",
                      dash=(3, 5), tags="hover")
        c.create_oval(px - 3, py - 3, px + 3, py + 3,
                      outline="#3a3a3a", tags="hover")

        text = (f"{x:+.0f}, {y:+.0f} mm   [{cell}]   "
                f"距雷达 {dist:.0f}mm / {brg:.0f}°")
        tid = c.create_text(px + 14, py + 16, text=text, anchor="nw",
                            fill="#1b1b1b", font=("Consolas", 9), tags="hover")
        # 贴个浅底，别被点云糊住看不清
        box = c.bbox(tid)
        if box:
            # 太靠右/下就翻到另一侧
            if box[2] > c.winfo_width() - 4:
                c.coords(tid, px - 14, py + 16)
                c.itemconfig(tid, anchor="ne")
                box = c.bbox(tid)
            if box and box[3] > c.winfo_height() - 4:
                c.coords(tid, px + 14, py - 16)
                c.itemconfig(tid, anchor="nw" if box[2] < c.winfo_width() - 4 else "ne")
                box = c.bbox(tid)
            if box:
                bg = c.create_rectangle(box[0] - 4, box[1] - 2,
                                        box[2] + 4, box[3] + 2,
                                        fill="#fffbe6", outline="#d8d0a0",
                                        tags="hover")
                c.tag_lower(bg, tid)

    def on_click(self, ev):
        w = self._px_to_world(ev.x, ev.y)
        if w is None:
            return
        if self.var_snap.get():
            cr = self.field.cell_at(*w)
            if cr:
                w = self.field.cell_center(*cr)          # 吸附到格子中心
        self.goal_world = w
        self._set_status(f"目标 {self.field.cell_name(*w)}  "
                         f"({w[0]:.0f}, {w[1]:.0f}) mm")
        self.plan_to_goal()

    # ==================== 绘图 ====================

    def _transform(self):
        cw = max(self.canvas.winfo_width(), 200)
        ch = max(self.canvas.winfo_height(), 200)
        m = 40
        s = min((cw - 2 * m) / self.field.W, (ch - 2 * m) / self.field.H)
        ox = (cw - self.field.W * s) / 2
        oy = (ch - self.field.H * s) / 2
        return s, ox, oy

    def _w2p(self, x, y):
        s, ox, oy = self._transform()
        return ox + (x + self.field.hw) * s, oy + (self.field.hh - y) * s

    def _px_to_world(self, px, py):
        s, ox, oy = self._transform()
        x = (px - ox) / s - self.field.hw
        y = self.field.hh - (py - oy) / s
        if not self.field.in_field(x, y):
            return None
        return x, y

    def _refresh(self):
        if self._drain():
            self._redraw()
        self.root.after(100, self._refresh)

    def _redraw(self):
        c = self.canvas
        c.delete("all")
        f = self.field
        s, ox, oy = self._transform()

        def R(r, fill=None, outline=None, width=1):
            p0 = self._w2p(r.x0, r.y1)
            p1 = self._w2p(r.x1, r.y0)
            c.create_rectangle(p0[0], p0[1], p1[0], p1[1],
                               fill=fill, outline=outline, width=width)

        R(mapping.Rect(-f.hw, -f.hh, f.hw, f.hh), fill=C_WHITE, outline="#464646", width=2)
        for r in f.connector_rects():
            R(r, fill=C_CONN)
        for r in f.lane_rects:
            R(r, fill=C_LANE)
        for r in f.yellow_rects:
            R(r, fill=C_YELLOW, outline="#ece2ac")
        for z in f.work_zones:
            R(z["rect"], outline="#6e9b6e")
            x, y = self._w2p(z["rect"].x0, z["rect"].y1)
            c.create_text(x + 2, y - 8, text=z["name"], anchor="w", fill="#50734f",
                          font=("Microsoft YaHei", 9))
            ax, ay = self._w2p(*z["approach"])       # 车应该停的位置
            c.create_line(ax - 4, ay, ax + 4, ay, fill="#2e9e6b", width=2)
            c.create_line(ax, ay - 4, ax, ay + 4, fill="#2e9e6b", width=2)
        for z in f.start_zones:
            R(z["rect"], fill=C_BLUE)
            x, y = self._w2p(z["cx"], z["cy"])
            c.create_text(x, y, text=z["name"][-1], fill="white",
                          font=("Microsoft YaHei", 11, "bold"))

        # 被障碍占住、整格不能通车的格子：标红
        for (cc, rr) in self.blocked_cells:
            r = f.cell_rect(cc, rr)
            p0 = self._w2p(r.x0, r.y1)
            p1 = self._w2p(r.x1, r.y0)
            c.create_rectangle(p0[0], p0[1], p1[0], p1[1],
                               fill=C_CYL, stipple="gray25",
                               outline=C_CYL, width=2)
            c.create_text((p0[0] + p1[0]) / 2, p0[1] + 12,
                          text=f"不通车 {f.cell_mm:.0f}mm",
                          fill="#b3122c", font=("Microsoft YaHei", 9, "bold"))

        # 5×5 网格（按黄块边长分格，居中）
        if self.var_grid.get():
            gx, gy = f.gx, f.gy
            for x in gx:
                p0 = self._w2p(x, gy[0]); p1 = self._w2p(x, gy[-1])
                c.create_line(p0[0], p0[1], p1[0], p1[1], fill="#b4b4b4", dash=(4, 4))
            for y in gy:
                p0 = self._w2p(gx[0], y); p1 = self._w2p(gx[-1], y)
                c.create_line(p0[0], p0[1], p1[0], p1[1], fill="#b4b4b4", dash=(4, 4))
            for rr in range(f.cell_rows):
                for cc in range(f.cell_cols):
                    px, py = self._w2p(*f.cell_center(cc, rr))
                    c.create_text(px, py, text=f.cell_label(cc, rr), fill="#8a8a8a",
                                  font=("Microsoft YaHei", 9))

        # 点云
        for x, y, *_ in self.points[:4000]:
            px, py = self._w2p(x, y)
            c.create_rectangle(px - 1, py - 1, px + 1, py + 1, outline=C_POINT, fill=C_POINT)

        # 路径
        if self.plan and self.plan["path"]:
            pts = [self._w2p(*self.grid.c2w(ix, iy)) for ix, iy in self.plan["path"]]
            flat = [v for p in pts for v in p]
            c.create_line(*flat, fill=C_PATH, width=3, smooth=False)
            for p in (pts[0], pts[-1]):
                c.create_oval(p[0] - 5, p[1] - 5, p[0] + 5, p[1] + 5,
                              fill=C_PATH, outline="")

        # 任务路线：每段一个颜色 + 序号
        if self.mission_legs:
            pal = ["#146ede", "#2e9e6b", "#b0601f", "#8e24aa", "#c22a52",
                   "#0b6e8a", "#5a7d1f", "#7a4fbf", "#d08114"]
            for i, leg in enumerate(self.mission_legs):
                col = pal[i % len(pal)]
                if leg["path"]:
                    pts = [self._w2p(*q) for q in leg["path"]]
                    c.create_line(*[v for p in pts for v in p], fill=col, width=3)
                if leg["from_xy"]:
                    px, py = self._w2p(*leg["from_xy"])
                    c.create_oval(px - 7, py - 7, px + 7, py + 7,
                                  fill="white", outline=col, width=2)
                    c.create_text(px, py, text=str(i + 1), fill=col,
                                  font=("Consolas", 8, "bold"))
            tail = self.mission_legs[-1]["to_xy"]
            if tail:
                px, py = self._w2p(*tail)
                col = pal[0]
                c.create_oval(px - 7, py - 7, px + 7, py + 7,
                              fill="white", outline=col, width=2)
                c.create_text(px, py, text=str(len(self.mission_legs) + 1),
                              fill=col, font=("Consolas", 8, "bold"))

        # 障碍
        for ob in self.obstacles:
            px, py = self._w2p(ob["x"], ob["y"])
            if ob["kind"] == "circle":
                # 半径可信就按实测画，不可信才退回规则值 φ50
                conf = ob.get("r_confident")
                r_mm = ob["r"] if conf else mapping._ob_radius(ob)
                r = max(r_mm * s, 3)
                c.create_oval(px - r, py - r, px + r, py + r, outline=C_CYL, width=2,
                              dash=() if conf else (3, 3))
                tag = f"φ{r_mm * 2:.0f}mm" + ("" if conf else "?")
                c.create_text(px + r + 6, py, anchor="w", fill=C_CYL,
                              text=f'{tag} ({ob["x"]:.0f},{ob["y"]:.0f})',
                              font=("Microsoft YaHei", 9))
            else:
                c.create_oval(px - 4, py - 4, px + 4, py + 4, outline=C_BLOB, width=2)

        # 目标点
        if self.goal_world:
            gx, gy = self._w2p(*self.goal_world)
            c.create_line(gx - 8, gy, gx + 8, gy, fill=C_GOAL, width=2)
            c.create_line(gx, gy - 8, gx, gy + 8, fill=C_GOAL, width=2)

        # 雷达
        lx, ly = self._w2p(f.lidar_x, f.lidar_y)
        c.create_oval(lx - 5, ly - 5, lx + 5, ly + 5, fill=C_LIDAR, outline="")
        a = math.radians(f.lidar_yaw)
        c.create_line(lx, ly, lx + 34 * math.cos(a), ly - 34 * math.sin(a),
                      fill=C_LIDAR, width=3, arrow=tk.LAST)

        # 图例 / 比例尺
        c.create_text(10, 10, anchor="nw", fill="#555555",
                      text=f"场地 {f.W:.0f}×{f.H:.0f}mm   车道 {f.lane_w:.0f}mm   "
                           f"{f.cell_cols}×{f.cell_rows} 网格 格边长 {f.cell_mm:.0f}mm "
                           f"边距 {f.grid_margin[0]:.0f}mm   "
                           f"雷达 @ ({f.lidar_x:.0f},{f.lidar_y:.0f}) yaw={f.lidar_yaw:.0f}°   "
                           f"点 {len(self.points)}　（鼠标移到图上可看坐标）",
                      font=("Microsoft YaHei", 9))
        # 帧体检：角步长正常 ≈0.9°/点。明显不对就红字报警——这是"比例不对"的根因
        step = self.lidar.step_deg() if self.lidar is not None else None
        if step:
            ok = 0.3 <= step <= 1.2
            d = getattr(self.lidar, "direction", None)
            dir_s = {"cw": "顺时针", "ccw": "逆时针"}.get(d, "方向待定")
            rings = self.lidar.stats["rings"] if hasattr(self.lidar, "stats") else 0
            c.create_text(10, 26, anchor="nw",
                          fill="#2e7d32" if ok else "#c62828",
                          text=f"帧体检：{step:.2f}°/点 {dir_s}　"
                               f"{'正常（≈400 点/圈）' if ok else '异常！角度标度或方向有问题'}"
                               f"　{rings} 圈",
                          font=("Microsoft YaHei", 8))

        c.create_text(10, c.winfo_height() - 56, anchor="w", fill="#b3122c",
                      text="红格 = 被占住的整个 450mm 格（不是物体尺寸）；"
                           "取消「整格封锁」只按实体 φ 绕行",
                      font=("Microsoft YaHei", 8))
        if self._last_hover:
            self._draw_hover(*self._last_hover)

        bar = 500 * s
        x0, y0 = 14, c.winfo_height() - 22
        c.create_line(x0, y0, x0 + bar, y0, fill="#555555", width=2)
        c.create_text(x0 + bar / 2, y0 - 8, text="500mm", fill="#555555",
                      font=("Microsoft YaHei", 8))


def main():
    cfg = CFG_PATH
    if "--cfg" in sys.argv:
        cfg = sys.argv[sys.argv.index("--cfg") + 1]
    root = tk.Tk()
    App(root, cfg)
    root.mainloop()


if __name__ == "__main__":
    main()
