#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""上位机：动作序列编辑器 + 雷达地图导航（二合一）。

================ 大白话：这个程序是干什么的 ================

左边两个页签：

  【动作序列】还是原来那套：编辑 move / turn / gimbal / gripper …，
      点"执行整段"发到树莓派，motion_planner 一条一条跑。云台、夹爪、
      载物盘、相机这些**非底盘动作**继续用指令序列写。

  【雷达 / 地图】从 vision 分支的 map_app 整合进来：
      · 按 field.json 的尺寸画出场地（灰色车道、淡黄禁行块、5×5 格）；
      · 接 D6 雷达串口（或模拟雷达）扫几圈，把回波打到地图上；
      · 自动识别圆柱/未知障碍，把占住的 450mm 格标红；
      · 鼠标点地图上的任意位置 → A* 规划绕开障碍的路径 →
        自动换算成里程计坐标，以 goto 指令下发给 motion_planner →
        小车依次走完路点。勾掉"点击即导航"就只规划不下发。

================ 坐标怎么对上 ================

field.json 里有两个场地位姿：

    robot_start : 车起点（上电/复位时车摆的位置），mm/度。
    lidar       : 雷达安装位姿（场地系，mm/度）。

如果雷达就装在车上、原地扫描，这两个值应该一样（界面上有【雷达 = 车起点】）。
雷达是固定在场边的，就把 lidar 改成实际位置。

⚠ F407 的里程计 yaw 不是从 0 开始的：它直接用 JY901S 的绝对 yaw(Imu.Yaw)，
上电时读到多少就是多少。所以除了场地起点，还要记住"车起点对应的里程计位姿"
（界面第一次收到 /odom、且车在原点附近时会自动锁存；车不在起点时请手动点
【此刻车在【车起点】→ 标定里程计】）。设：

    dq = 车起点场地朝向 − 车起点里程计朝向

规划在场地坐标系里做（mm），下发前换算成里程计坐标系（m/弧度）：

    场地 → 里程计： p_odom = R(-dq) · (p_field − start_field) + start_odom
    里程计 → 场地： p_field = R(dq) · (p_odom − start_odom) + start_field

两个坐标系都是右手系（x 前、y 左、逆时针为正），所以只是一个平移+旋转。

界面口径：状态栏位姿、鼠标悬停坐标、目标点读数、下发的 goto，全部用
【里程计口径】（米 / 度，和 `ros2 topic echo /odom` 一模一样）；场地系只留在
field.json 和"车起点 / 雷达安装位姿"这类安装参数里。屏幕上地图仍按场地正向画，
所以点击换算对你不可见。

================ 运行前提 ================

1. 树莓派上先跑 scripts/run_car.sh（f407_bridge + motion_planner 都在）；
2. 雷达串口有权限（一般要把用户加进 dialout 组）；
3. 跑： bash scripts/run_gui.sh

================ 安全 ================

界面上"■ 停止（含导航）" = /motion_stop，状态机会停在原地并交还手柄控制权。
导航过程中手柄除 B（失能）外全部让位，见 joy_control 的说明。
"""

import json
import math
import os
import queue

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32MultiArray, String

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import lidar_d6
import mapping

HERE = os.path.dirname(os.path.abspath(__file__))
CFG_PATH = os.path.join(HERE, "field.json")

# ---------------- 配色（和 map_app 一致） ----------------
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

# ==================== 指令类型的"元信息" ====================
COMMAND_TYPES = {
    'move':      {'params': 'dist_m', 'hint': '前进/后退距离(米)，负=后退'},
    'strafe':    {'params': 'dist_m', 'hint': '横向平移(米)，正=左'},
    'turn':      {'params': 'deg', 'hint': '转向角(度)，正=左转(逆时针)，负=右转'},
    'goto':      {'params': 'x_m,y_m,yaw_deg', 'hint': '绝对位姿(里程计系)：x(米), y(米), 朝向(度)'},
    'gimbal':    {'params': 'yaw_deg,lift_deg,extend_deg', 'hint': '云台三轴(度)，逗号分隔'},
    'gripper':   {'params': 'deg', 'hint': '夹爪角度(0~180)'},
    'plate':     {'params': 'deg', 'hint': '载物盘角度(0~180)'},
    'cam':       {'params': 'deg', 'hint': '相机角度(0~180)'},
    'enable':    {'params': 'en', 'hint': 'true=使能 / false=失能'},
    'delay':     {'params': 'ms', 'hint': '原地等待毫秒数'},
    'calibrate': {'params': '', 'hint': '无参数(视觉校准占位)'},
    'comment':   {'params': 'text', 'hint': '注释文本(可含逗号)'},
}


def cmd_to_text(cmd: dict) -> str:
    """把一条指令字典，变成列表里给人看的一句话。"""
    name = cmd.get('cmd')
    if name == 'move':
        return f"前进 {cmd.get('dist_m', 0):+.2f} 米"
    if name == 'strafe':
        return f"横移 {cmd.get('dist_m', 0):+.2f} 米"
    if name == 'turn':
        return f"转向 {cmd.get('deg', 0):+.0f}°"
    if name == 'goto':
        return (f"去 ({cmd.get('x_m', 0):+.3f}, {cmd.get('y_m', 0):+.3f})m "
                f"朝向 {cmd.get('yaw_deg', 0):+.1f}°")
    if name == 'gimbal':
        return f"云台 yaw={cmd.get('yaw_deg', 0)}° lift={cmd.get('lift_deg', 0)}° ext={cmd.get('extend_deg', 0)}°"
    if name == 'gripper':
        return f"夹爪 {cmd.get('deg', 0)}°"
    if name == 'plate':
        return f"载物盘 {cmd.get('deg', 0)}°"
    if name == 'cam':
        return f"相机 {cmd.get('deg', 0)}°"
    if name == 'enable':
        return f"使能 {'开' if cmd.get('en', True) else '关'}"
    if name == 'delay':
        return f"等待 {cmd.get('ms', 0)} ms"
    if name == 'calibrate':
        return "视觉校准"
    if name == 'comment':
        return f"// {cmd.get('text', '')}"
    return str(cmd)


def build_cmd(name: str, params_str: str) -> dict:
    """根据指令类型 + 用户填的参数串，生成一条指令字典。"""
    if name == 'comment':
        return {'cmd': name, 'text': params_str.strip()}
    parts = [p.strip() for p in params_str.split(',') if p.strip() != '']

    if name in ('move', 'strafe'):
        return {'cmd': name, 'dist_m': float(parts[0]) if parts else 0.0}
    if name == 'turn':
        return {'cmd': name, 'deg': float(parts[0]) if parts else 0.0}
    if name == 'goto':
        vals = [float(p) for p in parts] + [0.0, 0.0, 0.0]
        return {'cmd': name, 'x_m': vals[0], 'y_m': vals[1], 'yaw_deg': vals[2]}
    if name == 'gimbal':
        vals = [float(p) for p in parts] + [0.0, 0.0, 0.0]
        return {'cmd': name, 'yaw_deg': vals[0], 'lift_deg': vals[1], 'extend_deg': vals[2]}
    if name in ('gripper', 'plate', 'cam'):
        return {'cmd': name, 'deg': float(parts[0]) if parts else 0.0}
    if name == 'enable':
        en = (parts[0].lower() in ('1', 'true', '开', 'on')) if parts else True
        return {'cmd': name, 'en': en}
    if name == 'delay':
        return {'cmd': name, 'ms': int(float(parts[0])) if parts else 0}
    if name == 'calibrate':
        return {'cmd': name}
    return {'cmd': name}


def wrap_deg(a: float) -> float:
    """角度归一化到 (-180, 180]。"""
    return (a + 180.0) % 360.0 - 180.0


# ==================== ROS2 节点：负责和树莓派通信 ====================

class GuiRosNode(Node):
    """上位机自己的 ROS2 节点：发指令、收里程计/状态。"""

    def __init__(self):
        super().__init__('motion_gui')
        self.pub_load = self.create_publisher(String, '/motion_load', 10)
        self.pub_stop = self.create_publisher(Bool, '/motion_stop', 10)
        self.create_subscription(Odometry, '/odom', self._cb_odom, 10)
        self.create_subscription(String, '/motion_state', self._cb_state, 10)
        self.create_subscription(Int32MultiArray, '/robot_status', self._cb_status, 10)

        self.odom = None            # (x, y, theta) 米/米/弧度
        self.traj = []              # 历史轨迹 [(x,y), ...]（里程计系）
        self.motion_state = ''
        self.robot_status = None    # [state, err, mask]

    def _cb_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, theta)
        self.traj.append((self.odom[0], self.odom[1]))
        if len(self.traj) > 4000:               # 别无限长
            del self.traj[:len(self.traj) - 4000]

    def _cb_state(self, msg: String):
        self.motion_state = msg.data

    def _cb_status(self, msg: Int32MultiArray):
        if len(msg.data) >= 3:
            self.robot_status = list(msg.data[:3])

    def send_sequence(self, commands: list) -> bool:
        """下发一段指令序列。返回 False = 没有节点在听 /motion_load（motion_planner 没起来）。"""
        if self.pub_load.get_subscription_count() == 0:
            return False
        s = String()
        s.data = json.dumps(commands, ensure_ascii=False)
        self.pub_load.publish(s)
        return True

    def send_stop(self):
        b = Bool()
        b.data = True
        self.pub_stop.publish(b)


# ==================== tkinter 界面 ====================

class App:
    def __init__(self, root: tk.Tk, node: GuiRosNode):
        self.root = root
        self.node = node

        # ---------- 场地 / 地图模型 ----------
        self.cfg_path = CFG_PATH
        self.cfg = mapping.load_config(self.cfg_path)
        self.field = mapping.FieldModel(self.cfg)
        self.grid = mapping.Grid(self.field)
        self.mask = self.grid.build_drivable()

        rs = self.cfg.get("robot_start", {}) or {}
        self.robot_start = (float(rs.get("x", self.field.lidar_x)),
                            float(rs.get("y", self.field.lidar_y)),
                            float(rs.get("yaw_deg", self.field.lidar_yaw)))

        # 车起点对应的【里程计】位姿。x/y 通常就是 (0,0)，但 yaw 不一定是 0：
        # F407 用的是 JY901S 的绝对 yaw(Imu.Yaw)，上电时的值取决于模块姿态。
        # 所以第一次收到 /odom（车还在起点附近）时自动锁存；否则要手动点标定。
        # 三项都齐了（yaw 不是 None）才能把里程计换算到场地。
        self.odom_start = [0.0, 0.0, None]

        # ---------- 雷达 / 规划状态 ----------
        self.raw_points = []        # (设备角, 距离, 强度)
        self.points = []            # 世界坐标 [(x, y, 强度)]
        self.obstacles = []
        self.blocked_cells = set()  # 整格不能通车的 5×5 格
        self.plan = None            # 最近一次单点规划
        self.mission_legs = []
        self.mission_total = 0.0
        self.goal_world = None
        self.lidar = None
        self.lidar_q = queue.Queue()
        self.scanning = False
        self.scan_revs_done = 0
        self.scan_target_revs = int(self.cfg["scan"]["revolutions"])
        self._last_hover = None
        self._last_odom = None
        self._need_redraw = True

        self.commands = []          # 动作序列编辑器里的指令

        root.title('智能搬运上位机 — 动作序列 + 雷达地图导航')
        root.geometry('1500x940')
        self._build_ui()
        self._refresh()

    # ==================== 界面 ====================

    def _build_ui(self):
        self.status = tk.StringVar(
            value='就绪。没雷达就勾【模拟雷达】点【连接并开始】；'
                  '左边【雷达 / 地图】扫完点地图即可导航。')
        ttk.Label(self.root, textvariable=self.status, relief=tk.SUNKEN,
                  anchor='w', padding=4).pack(side=tk.BOTTOM, fill=tk.X)

        bottom = ttk.Frame(self.root, padding=(8, 4))
        bottom.pack(side=tk.BOTTOM, fill=tk.X)
        ttk.Button(bottom, text='▶ 执行整段', command=self.run_all).pack(side=tk.LEFT, padx=4)
        ttk.Button(bottom, text='▶ 执行选中一条',
                   command=self.run_selected).pack(side=tk.LEFT, padx=4)
        ttk.Button(bottom, text='■ 停止（含导航）',
                   command=self.node.send_stop).pack(side=tk.LEFT, padx=4)
        ttk.Label(bottom, foreground='#666',
                  text='  点地图 = 规划并导航；云台/夹爪/舵机仍用左边动作序列').pack(
            side=tk.LEFT, padx=10)

        body = ttk.Frame(self.root)
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        left = ttk.Frame(body, padding=6)
        left.pack(side=tk.LEFT, fill=tk.Y)
        self.nb = ttk.Notebook(left)
        self.nb.pack(fill=tk.BOTH, expand=True)
        self._build_seq_tab()
        self._build_map_tab_controls()

        right = ttk.Frame(body)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.pose_var = tk.StringVar(value='当前位姿: 未收到 /odom')
        self.state_var = tk.StringVar(value='状态机: —')
        self.plan_var = tk.StringVar(value='')
        ttk.Label(right, textvariable=self.plan_var, anchor='w', foreground=C_PATH,
                  wraplength=1000).pack(side=tk.BOTTOM, fill=tk.X, padx=8)
        ttk.Label(right, textvariable=self.state_var, anchor='w',
                  foreground='#444').pack(side=tk.BOTTOM, fill=tk.X, padx=8)
        ttk.Label(right, textvariable=self.pose_var, anchor='w',
                  foreground='#222').pack(side=tk.BOTTOM, fill=tk.X, padx=8)

        self.canvas = tk.Canvas(right, bg=C_BG, highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=8, pady=(8, 2))
        self.canvas.bind('<Button-1>', self.on_click)
        self.canvas.bind('<Motion>', self.on_motion)
        self.canvas.bind('<Leave>', lambda e: self._clear_hover())
        self.canvas.bind('<Configure>', lambda e: self._redraw())

    # ---------- 页签 1：动作序列 ----------

    def _build_seq_tab(self):
        tab = ttk.Frame(self.nb, padding=6)
        self.nb.add(tab, text='动作序列')

        ttk.Label(tab, text='动作序列（从上到下依次执行）',
                  font=('', 11, 'bold')).pack(anchor='w')
        # 1024×768 的树莓派屏上整页要放得下，行数别太多（下面还有导入/导出）
        self.listbox = tk.Listbox(tab, width=38, height=18)
        self.listbox.pack(fill=tk.BOTH, expand=True, pady=4)

        addrow = ttk.Frame(tab)
        addrow.pack(fill=tk.X, pady=2)
        self.cmd_type = tk.StringVar(value='move')
        self.type_menu = ttk.OptionMenu(addrow, self.cmd_type, 'move',
                                        *COMMAND_TYPES.keys(),
                                        command=self._on_type_change)
        self.type_menu.pack(side=tk.LEFT)
        self.param_entry = ttk.Entry(addrow)
        self.param_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        ttk.Button(addrow, text='＋添加', command=self.add_cmd).pack(side=tk.LEFT)

        self.hint_label = ttk.Label(tab, text='', foreground='gray')
        self.hint_label.pack(anchor='w')
        self._on_type_change('move')

        editrow = ttk.Frame(tab)
        editrow.pack(fill=tk.X, pady=4)
        ttk.Button(editrow, text='上移', command=lambda: self.move_cmd(-1)).pack(side=tk.LEFT, padx=2)
        ttk.Button(editrow, text='下移', command=lambda: self.move_cmd(+1)).pack(side=tk.LEFT, padx=2)
        ttk.Button(editrow, text='删除', command=self.del_cmd).pack(side=tk.LEFT, padx=2)
        ttk.Button(editrow, text='清空', command=self.clear_cmds).pack(side=tk.LEFT, padx=2)

        iorow = ttk.Frame(tab)
        iorow.pack(fill=tk.X, pady=2)
        ttk.Button(iorow, text='导入JSON', command=self.import_json).pack(side=tk.LEFT, padx=2)
        ttk.Button(iorow, text='导出JSON', command=self.export_json).pack(side=tk.LEFT, padx=2)

    # ---------- 页签 2：雷达 / 地图 ----------

    def _scrollable_tab(self, title):
        """左栏页签内容比屏幕高时（树莓派屏常见 1024×768）套一层滚动。"""
        outer = ttk.Frame(self.nb)
        self.nb.add(outer, text=title)
        cv = tk.Canvas(outer, highlightthickness=0, borderwidth=0, width=330)
        sb = ttk.Scrollbar(outer, orient='vertical', command=cv.yview)
        cv.configure(yscrollcommand=sb.set)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        cv.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        inner = ttk.Frame(cv, padding=6)
        win = cv.create_window((0, 0), window=inner, anchor='nw')
        self._map_scroll = cv

        def _resize(_=None):
            cv.configure(scrollregion=cv.bbox('all'))
            cv.itemconfigure(win, width=max(cv.winfo_width(), 300))

        inner.bind('<Configure>', _resize)
        cv.bind('<Configure>', _resize)
        return inner

    def _bind_wheel(self, widget):
        """Linux 下滚轮是 Button-4/5（不是 MouseWheel），递归绑到页签里所有控件。"""
        def _wheel(ev):
            if ev.num == 4:
                self._map_scroll.yview_scroll(-3, 'units')
            else:
                self._map_scroll.yview_scroll(3, 'units')
        widget.bind('<Button-4>', _wheel)
        widget.bind('<Button-5>', _wheel)
        for ch in widget.winfo_children():
            self._bind_wheel(ch)

    def _build_map_tab_controls(self):
        tab = self._scrollable_tab('雷达 / 地图')

        # ---- 雷达 / 串口 ----
        box = ttk.LabelFrame(tab, text='雷达 / 串口', padding=4)
        box.pack(fill=tk.X, pady=(0, 3))
        ports = self._ports()
        real = [p for p in ports if not p.startswith('(')]

        row = ttk.Frame(box); row.pack(fill=tk.X)
        ttk.Label(row, text='串口', width=6).pack(side=tk.LEFT)
        self.cb_port = ttk.Combobox(row, width=18, values=ports)
        self.cb_port.set(self._prefer_radar(real) if real else
                         self.cfg['lidar'].get('port', '/dev/ttyUSB0'))
        self.cb_port.pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(row, text='刷新', width=5,
                   command=lambda: self.cb_port.configure(values=self._ports())).pack(side=tk.LEFT)

        row = ttk.Frame(box); row.pack(fill=tk.X)
        ttk.Label(row, text='波特率', width=6).pack(side=tk.LEFT)
        self.cb_baud = ttk.Combobox(row, width=12,
                                    values=[115200, 153600, 230400, 460800])
        self.cb_baud.set(self.cfg['lidar'].get('baud', 230400))
        self.cb_baud.pack(side=tk.LEFT, fill=tk.X, expand=True)

        self.var_cw = tk.BooleanVar(value=self.field.angle_cw)
        ttk.Checkbutton(box, text='角度顺时针（D6 默认；南北镜像就点它）',
                        variable=self.var_cw,
                        command=self._apply_cw).pack(anchor='w')
        self.var_sim = tk.BooleanVar(value=not bool(real))
        ttk.Checkbutton(box, text='用模拟雷达（没插雷达也能演示）',
                        variable=self.var_sim).pack(anchor='w')

        row = ttk.Frame(box); row.pack(fill=tk.X, pady=4)
        self.btn_conn = ttk.Button(row, text='连接并开始', command=self.toggle_lidar)
        self.btn_conn.pack(side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(row, text='停止', command=self.stop_lidar).pack(side=tk.LEFT, padx=(4, 0))

        # ---- 雷达安装位 ----
        box = ttk.LabelFrame(tab, text='雷达安装位置（场地系 mm/°，安装参数）', padding=4)
        box.pack(fill=tk.X, pady=3)
        row = ttk.Frame(box); row.pack(fill=tk.X)
        ttk.Label(row, text='放在', width=6).pack(side=tk.LEFT)
        self.cb_mount = ttk.Combobox(
            row, width=16, state='readonly',
            values=[z['name'] for z in self.field.start_zones] + ['自定义'])
        self.cb_mount.set(self.cfg['lidar'].get('mount', '启停区1'))
        self.cb_mount.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.cb_mount.bind('<<ComboboxSelected>>', lambda e: self._apply_mount())

        gxy = ttk.Frame(box); gxy.pack(fill=tk.X)
        self.ent_lx = self._grid_entry(gxy, 0, 0, 'x', self.field.lidar_x, self._apply_lidar_mount)
        self.ent_ly = self._grid_entry(gxy, 0, 1, 'y', self.field.lidar_y, self._apply_lidar_mount)
        self.ent_lz = self._row_entry(box, 'yaw (°)', self.field.lidar_yaw,
                                      self._apply_lidar_mount)
        rw = ttk.Frame(box); rw.pack(fill=tk.X)
        ttk.Label(rw, text='微调', width=6).pack(side=tk.LEFT)
        for txt, d in (('◀ 15°', -15), ('◀ 5°', -5), ('5° ▶', 5), ('15° ▶', 15)):
            ttk.Button(rw, text=txt, width=5,
                       command=lambda dd=d: self._nudge_yaw(dd)).pack(side=tk.LEFT, padx=1)
        ttk.Button(box, text='雷达 = 车起点（雷达装在车上时用）',
                   command=self._mount_from_start).pack(fill=tk.X, pady=(2, 0))

        # ---- 车起点 / 里程计原点 ----
        box = ttk.LabelFrame(tab, text='车起点（场地系 mm/°，安装参数）', padding=4)
        box.pack(fill=tk.X, pady=3)
        gf = ttk.Frame(box); gf.pack(fill=tk.X)
        self.ent_rx = self._grid_entry(gf, 0, 0, 'x', self.robot_start[0],
                                       self._apply_robot_start)
        self.ent_ry = self._grid_entry(gf, 0, 1, 'y', self.robot_start[1],
                                       self._apply_robot_start)
        gf2 = ttk.Frame(box); gf2.pack(fill=tk.X)
        ttk.Label(gf2, text='yaw (°)').pack(side=tk.LEFT)
        self.ent_rdeg = ttk.Entry(gf2, width=8)
        self.ent_rdeg.insert(0, f'{self.robot_start[2]:g}')
        self.ent_rdeg.pack(side=tk.LEFT, padx=(2, 6))
        self.ent_rdeg.bind('<Return>', lambda e: self._apply_robot_start())
        ttk.Button(gf2, text='应用', command=self._apply_robot_start).pack(side=tk.LEFT)
        ttk.Label(box, foreground='#a05a00', wraplength=330,
                  text='上电/复位那一刻车所在的位姿。改这个值不会改里程计，只改地图↔里程计的换算。'
                  ).pack(anchor='w', pady=(2, 0))
        self.cal_var = tk.StringVar(value='里程计标定: 未标定')
        ttk.Label(box, textvariable=self.cal_var, foreground='#7a4fbf').pack(
            anchor='w', pady=(2, 0))
        ttk.Button(box, text='此刻车在【车起点】→ 标定里程计',
                   command=self._calibrate_odom_start).pack(fill=tk.X, pady=(2, 0))

        # ---- 操作 ----
        box = ttk.LabelFrame(tab, text='扫描 / 规划', padding=4)
        box.pack(fill=tk.X, pady=3)
        ttk.Button(box, text='① 扫描建图（扫几圈后冻结）',
                   command=self.start_scan).pack(fill=tk.X, pady=1)
        ttk.Button(box, text='② 重新识别障碍', command=self.detect).pack(fill=tk.X, pady=1)
        ttk.Button(box, text='③ 规划到目标（也可直接点地图）',
                   command=self.plan_to_goal).pack(fill=tk.X, pady=1)
        ttk.Button(box, text='清空点云', command=self.clear_scan).pack(fill=tk.X, pady=1)
        ttk.Button(box, text='保存场地参数', command=self.save_cfg).pack(fill=tk.X, pady=1)

        row = ttk.Frame(box); row.pack(fill=tk.X, pady=(2, 0))
        ttk.Label(row, text='目标格').pack(side=tk.LEFT)
        self.ent_cell = ttk.Entry(row, width=6)
        self.ent_cell.insert(0, 'C3')
        self.ent_cell.pack(side=tk.LEFT, padx=2)
        self.ent_cell.bind('<Return>', lambda e: self.goto_cell())
        ttk.Button(row, text='去', width=4, command=self.goto_cell).pack(side=tk.LEFT)

        row2 = ttk.Frame(box); row2.pack(fill=tk.X, pady=(2, 0))
        ttk.Label(row2, text='目标 odom').pack(side=tk.LEFT)
        self.ent_gx = ttk.Entry(row2, width=7)
        self.ent_gx.pack(side=tk.LEFT, padx=(2, 0))
        ttk.Label(row2, text=',').pack(side=tk.LEFT)
        self.ent_gy = ttk.Entry(row2, width=7)
        self.ent_gy.pack(side=tk.LEFT, padx=2)
        ttk.Label(row2, text='m').pack(side=tk.LEFT)
        for e in (self.ent_gx, self.ent_gy):
            e.bind('<Return>', lambda ev: self.goto_odom())
        ttk.Button(row2, text='去', width=4, command=self.goto_odom).pack(side=tk.LEFT)

        # ---- 选项 ----
        box = ttk.LabelFrame(tab, text='选项', padding=4)
        box.pack(fill=tk.X, pady=3)
        cf0 = ttk.Frame(box); cf0.pack(fill=tk.X)
        self.var_strict = tk.BooleanVar(value=True)
        self.var_line = tk.BooleanVar(value=True)
        ttk.Checkbutton(cf0, text='整格封锁', variable=self.var_strict,
                        command=self._replan_soft).grid(row=0, column=0, sticky='w')
        ttk.Checkbutton(cf0, text='沿格中线', variable=self.var_line,
                        command=self._replan_soft).grid(row=0, column=1, sticky='w', padx=(8, 0))
        cf1 = ttk.Frame(box); cf1.pack(fill=tk.X)
        self.var_grid = tk.BooleanVar(value=True)
        self.var_snap = tk.BooleanVar(value=True)
        ttk.Checkbutton(cf1, text='显示网格', variable=self.var_grid,
                        command=self._redraw).grid(row=0, column=0, sticky='w')
        ttk.Checkbutton(cf1, text='吸附格心', variable=self.var_snap,
                        command=self._redraw).grid(row=0, column=1, sticky='w', padx=(8, 0))
        self.var_auto = tk.BooleanVar(value=True)
        ttk.Checkbutton(box, text='点击即导航（取消则只规划、不下发）',
                        variable=self.var_auto).pack(anchor='w')
        # field.json 里 drivable.white=false 是"只走灰色田字形车道"。
        # 注意：ring_mm=550 时外圈+十字已经把全场铺满，这个开关没有区别；
        # 只有 ring_mm 调小、黄块和墙之间留出白区时，打开它才能压白区绕障。
        self.var_white = tk.BooleanVar(value=self.field.white_drivable)
        ttk.Checkbutton(box, text='白区可走（关 = 只走灰色车道）',
                        variable=self.var_white,
                        command=self._apply_white).pack(anchor='w')

        # ---- 导航 ----
        box = ttk.LabelFrame(tab, text='导航', padding=4)
        box.pack(fill=tk.X, pady=3)
        row = ttk.Frame(box); row.pack(fill=tk.X)
        ttk.Button(row, text='▶ 导航到目标', command=self.navigate_plan).pack(
            side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(row, text='■', width=3, command=self.node.send_stop).pack(
            side=tk.LEFT, padx=(4, 0))

        # ---- 任务路线 ----
        box = ttk.LabelFrame(tab, text='任务路线（field.json 的 mission.route）', padding=4)
        box.pack(fill=tk.X, pady=3)
        row = ttk.Frame(box); row.pack(fill=tk.X)
        ttk.Button(row, text='规划全程', command=self.plan_mission).pack(
            side=tk.LEFT, fill=tk.X, expand=True)
        ttk.Button(row, text='执行全程', command=self.execute_mission).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        ttk.Button(row, text='清空', width=5, command=self.clear_route).pack(side=tk.LEFT)
        self.mission_info = tk.StringVar(value='还没规划')
        ttk.Label(box, textvariable=self.mission_info, foreground=C_PATH,
                  wraplength=330).pack(anchor='w', pady=(2, 0))

        # ---- 检出障碍 ----
        box = ttk.LabelFrame(tab, text='检出的障碍', padding=4)
        box.pack(fill=tk.BOTH, expand=True, pady=(3, 0))
        self.lst = tk.Listbox(box, height=3, font=('Consolas', 9))
        self.lst.pack(fill=tk.BOTH, expand=True)

        self._fill_mission_list()
        self._bind_wheel(tab)
        wrap = lambda ev: self._map_scroll.yview_scroll(-3 if ev.num == 4 else 3, 'units')
        self._map_scroll.bind('<Button-4>', wrap)
        self._map_scroll.bind('<Button-5>', wrap)

    # ---------- 小工具（布局） ----------

    def _row_entry(self, parent, label, value, cb):
        row = ttk.Frame(parent); row.pack(fill=tk.X, pady=0)
        ttk.Label(row, text=label, width=9).pack(side=tk.LEFT)
        e = ttk.Entry(row, width=9)
        e.insert(0, f'{float(value):g}')
        e.pack(side=tk.LEFT, fill=tk.X, expand=True)
        e.bind('<Return>', lambda ev: cb())
        e.bind('<FocusOut>', lambda ev: cb())
        return e

    def _grid_entry(self, parent, r, c, label, value, cb):
        ttk.Label(parent, text=label).grid(row=r, column=c * 2, sticky='w')
        e = ttk.Entry(parent, width=6)
        e.insert(0, f'{float(value):g}')
        e.grid(row=r, column=c * 2 + 1, sticky='we', padx=(2, 8), pady=1)
        e.bind('<Return>', lambda ev: cb())
        e.bind('<FocusOut>', lambda ev: cb())
        parent.columnconfigure(c * 2 + 1, weight=1)
        return e

    @staticmethod
    def _ports():
        try:
            from serial.tools import list_ports
        except Exception:
            return ['(未装 pyserial)']
        out = []
        for p in list_ports.comports():
            desc = (p.description or '').strip()
            out.append(f'{p.device} · {desc}' if desc else p.device)
        return out or ['(没插串口)']

    def _port_name(self):
        return self.cb_port.get().strip().split('·')[0].split(' ')[0].strip()

    @staticmethod
    def _prefer_radar(ports):
        keys = ('CH340', 'CP210', 'FTDI', 'SILICON', 'USB-SERIAL', 'USB 串行')
        for p in ports:
            if any(k.upper() in p.upper() for k in keys):
                return p
        return ports[0] if ports else '/dev/ttyUSB0'

    @staticmethod
    def _set(entry, value):
        entry.delete(0, tk.END)
        entry.insert(0, f'{float(value):g}')

    @staticmethod
    def _num(entry, default):
        try:
            return float(entry.get())
        except Exception:
            return float(default)

    def _set_status(self, s):
        self.status.set(s)

    # ==================== 参数改动 ====================

    def _apply_mount(self):
        name = self.cb_mount.get()
        for z in self.field.start_zones:
            if z['name'] == name:
                self._set(self.ent_lx, z['cx'])
                self._set(self.ent_ly, z['cy'])
                yaw = math.degrees(math.atan2(-z['cy'], -z['cx'])) % 360
                self._set(self.ent_lz, round(yaw, 1))
                self._apply_lidar_mount()
                return

    def _mount_from_start(self):
        x, y, deg = self.robot_start
        self._set(self.ent_lx, x)
        self._set(self.ent_ly, y)
        self._set(self.ent_lz, deg)
        self.cb_mount.set('自定义')
        self._apply_lidar_mount()
        self._set_status('雷达位姿 = 车起点。雷达装在车上、原地扫描时用这个。')

    def _apply_lidar_mount(self):
        self.field.lidar_x = self._num(self.ent_lx, self.field.lidar_x)
        self.field.lidar_y = self._num(self.ent_ly, self.field.lidar_y)
        self.field.lidar_yaw = self._num(self.ent_lz, self.field.lidar_yaw)
        self.cfg['lidar']['x'] = self.field.lidar_x
        self.cfg['lidar']['y'] = self.field.lidar_y
        self.cfg['lidar']['yaw_deg'] = self.field.lidar_yaw
        self.cfg['lidar']['mount'] = self.cb_mount.get()
        self.cfg['lidar']['angle_cw'] = self.field.angle_cw
        if self.raw_points:
            self._reproject()
            self.detect()
        else:
            self._redraw()

    def _apply_robot_start(self):
        self.robot_start = (self._num(self.ent_rx, self.robot_start[0]),
                            self._num(self.ent_ry, self.robot_start[1]),
                            self._num(self.ent_rdeg, self.robot_start[2]))
        self.cfg['robot_start'] = {'x': self.robot_start[0],
                                   'y': self.robot_start[1],
                                   'yaw_deg': self.robot_start[2]}
        self._set_status(
            f'车起点更新为 ({self.robot_start[0]:.0f}, {self.robot_start[1]:.0f}) '
            f'yaw={self.robot_start[2]:.0f}°。'
            f'注意：必须和 F407 上电时车的真实位姿一致，否则地图上的车会跑偏。')
        self._redraw()

    def _apply_cw(self):
        self.field.angle_cw = bool(self.var_cw.get())
        self.field.angle_sign = -1.0 if self.field.angle_cw else 1.0
        self.cfg['lidar']['angle_cw'] = self.field.angle_cw
        if self.raw_points:
            self._reproject()
            self.detect()
        else:
            self._redraw()
        self._set_status('角度方向：' + ('顺时针为正（已取反）' if self.field.angle_cw
                                        else '逆时针为正（不取反）'))

    def _nudge_yaw(self, delta):
        cur = self._num(self.ent_lz, self.field.lidar_yaw)
        self._set(self.ent_lz, (cur + delta) % 360)
        self._apply_lidar_mount()
        self._set_status(f'yaw = {(cur + delta) % 360:.0f}°（点云和实景对上了就停）')

    def _reproject(self):
        """改 yaw / 角度方向后，用原始回波立刻重算世界坐标，不用重扫。"""
        self.points = [(*self.field.lidar_to_world(a, d), i)
                       for (a, d, i) in self.raw_points]

    def _apply_white(self):
        """切换"白区能不能走"，重建可通行掩码并重新规划。"""
        self.field.white_drivable = bool(self.var_white.get())
        self.cfg['drivable']['white'] = self.field.white_drivable
        self.mask = self.grid.build_drivable()
        self._set_status('白区可走：' + ('是（除淡黄块外都能走）' if self.field.white_drivable
                                        else '否（只走灰色车道 + 启停区）'))
        self._replan_soft()

    def save_cfg(self):
        try:
            with open(self.cfg_path, 'w', encoding='utf-8') as f:
                json.dump(self.cfg, f, ensure_ascii=False, indent=2)
            self._set_status(f'已保存到 {self.cfg_path}')
        except Exception as e:
            messagebox.showerror('保存失败', str(e))

    # ==================== 坐标换算 ====================

    def _odom_to_field(self, x, y, th):
        """里程计(米/弧度) → 场地(mm/度)。

        对应关系：里程计 odom_start ↔ 场地 robot_start。
        dq = 车起点场地朝向 − 车起点里程计朝向，就是两个坐标系的旋转差。
        """
        rx, ry, rdeg = self.robot_start
        ox0, oy0, odeg0 = self.odom_start
        dq_deg = rdeg - (odeg0 if odeg0 is not None else 0.0)
        dq = math.radians(dq_deg)
        c, s = math.cos(dq), math.sin(dq)
        dx, dy = (x - ox0) * 1000.0, (y - oy0) * 1000.0
        return (rx + dx * c - dy * s,
                ry + dx * s + dy * c,
                wrap_deg(math.degrees(th) + dq_deg))

    def _field_to_odom(self, fx, fy, fdeg):
        """场地(mm/度) → 里程计(米/弧度)。"""
        rx, ry, rdeg = self.robot_start
        ox0, oy0, odeg0 = self.odom_start
        dq_deg = rdeg - (odeg0 if odeg0 is not None else 0.0)
        dq = math.radians(dq_deg)
        c, s = math.cos(dq), math.sin(dq)
        dx, dy = fx - rx, fy - ry
        ox = ox0 + (dx * c + dy * s) / 1000.0
        oy = oy0 + (-dx * s + dy * c) / 1000.0
        return ox, oy, math.radians(wrap_deg(fdeg - dq_deg))

    def _calibrate_odom_start(self):
        """把"当前里程计位姿"记成车起点。车此刻必须真的摆在 robot_start。"""
        o = self.node.odom
        if o is None:
            messagebox.showwarning('没有里程计', '还没收到 /odom，等 F407 回传后再标定。')
            return
        self.odom_start = [o[0], o[1], math.degrees(o[2])]
        self.cal_var.set(
            f'里程计标定: ({o[0]:+.3f}, {o[1]:+.3f})m θ={math.degrees(o[2]):+.1f}°')
        self._set_status(
            f'已标定：里程计 ({o[0]:+.3f},{o[1]:+.3f})m θ={math.degrees(o[2]):+.1f}° '
            f'= 场地 ({self.robot_start[0]:.0f},{self.robot_start[1]:.0f})mm '
            f'θ={self.robot_start[2]:.0f}°。现在地图上的车和实际对上了。')
        self._redraw()

    def _robot_field(self):
        """车当前在场地里的位姿。没有里程计或没标定就按"车在起点"算。"""
        o = self.node.odom
        if o is None or self.odom_start[2] is None:
            return self.robot_start
        return self._odom_to_field(*o)

    # ---------- 界面口径：一律里程计（米/度） ----------

    def _odom_of_field(self, fx, fy):
        """场地坐标 → 里程计 (米)。没标定车起点时返回 None。"""
        if self.odom_start[2] is None:
            return None
        ox, oy, _ = self._field_to_odom(fx, fy, 0.0)
        return ox, oy

    def _odom_str(self, fx, fy):
        p = self._odom_of_field(fx, fy)
        if p is None:
            return 'odom 未标定'
        return f'odom ({p[0]:+.3f}, {p[1]:+.3f}) m'

    def _odom_xy_str(self, fx, fy):
        """紧凑版，给列表用。"""
        p = self._odom_of_field(fx, fy)
        return '未标定' if p is None else f'({p[0]:+.2f},{p[1]:+.2f})m'

    # ==================== 雷达开关 ====================

    def toggle_lidar(self):
        if self.lidar is not None:
            self.stop_lidar()
            return
        self._apply_lidar_mount()
        if self.var_sim.get():
            self.lidar = lidar_d6.SimLidar(
                self.field, self._sim_obstacles(), on_frame=self._on_frame,
                on_log=lambda s: self.lidar_q.put(('log', s)))
        else:
            self.lidar = lidar_d6.D6Lidar(
                self._port_name(), int(self.cb_baud.get()), on_frame=self._on_frame,
                on_log=lambda s: self.lidar_q.put(('log', s)))
        if not self.lidar.open():
            err = getattr(self.lidar, 'error', '打开失败')
            self.lidar = None
            messagebox.showerror('雷达打不开', err)
            return
        self.btn_conn.configure(text='断开')
        mode = '模拟' if self.var_sim.get() else f'真机 {self._port_name()}'
        self._set_status(f'【{mode}】已连接，点【① 扫描建图】开始（扫描时车保持静止）')

    def stop_lidar(self):
        if self.lidar:
            self.lidar.close()
            self.lidar = None
        self.scanning = False
        self.btn_conn.configure(text='连接并开始')
        self._need_redraw = True

    def _sim_obstacles(self):
        obs = [{'x': c['x'], 'y': c['y'], 'r': mapping._ob_radius(c), 'intensity': 60}
               for c in self.obstacles]
        if not obs:
            obs = [{'x': 110, 'y': 620, 'r': 25, 'intensity': 60},
                   {'x': -60, 'y': -300, 'r': 25, 'intensity': 60},
                   {'x': -150, 'y': 1150, 'r': 150, 'intensity': 90}]
        return obs

    # ==================== 雷达数据（后台线程 → 队列） ====================

    def _on_frame(self, pts, ring):
        # 只在扫描阶段收数据；扫完冻结，避免比赛途中地图一直变。
        if self.scanning:
            self.lidar_q.put(('frame', (pts, ring)))

    def _drain(self):
        got = False
        while True:
            try:
                kind, payload = self.lidar_q.get_nowait()
            except queue.Empty:
                break
            if kind == 'log':
                self._set_status(str(payload))
            elif kind == 'frame':
                pts, ring = payload
                if ring and self.scanning:
                    self.scan_revs_done += 1
                for ang, d, inten in pts:
                    if (d < self.cfg['scan']['range_min_mm']
                            or d > self.cfg['scan']['range_max_mm']):
                        continue
                    self.raw_points.append((ang, d, inten))
                    x, y = self.field.lidar_to_world(ang, d)
                    self.points.append((x, y, inten))
                got = True
                if self.scanning and self.scan_revs_done >= self.scan_target_revs:
                    self._finish_scan()
        return got

    # ==================== 扫描 / 识别 ====================

    def start_scan(self):
        if self.lidar is None:
            messagebox.showinfo('先连雷达', '先点【连接并开始】（没硬件就勾模拟雷达）')
            return
        self.raw_points = []
        self.points = []
        self.obstacles = []
        self.blocked_cells = set()
        self.plan = None
        self.lst.delete(0, tk.END)
        self.scanning = True
        self.scan_revs_done = 0
        self.scan_target_revs = int(self.cfg['scan']['revolutions'])
        self._set_status(f'正在扫描 … 目标 {self.scan_target_revs} 圈（车保持静止，雷达视野里别站人）')

    def _finish_scan(self):
        self.scanning = False
        self._set_status(f'扫描完成：{len(self.points)} 个回波点，开始识别')
        self.detect()

    def detect(self):
        if not self.points:
            self._set_status('还没有点云，先扫一下')
            return
        self.obstacles = mapping.detect_cylinders(self.points, self.cfg, self.field)
        self.lst.delete(0, tk.END)
        n_cyl = 0
        for c in self.obstacles:
            if c['kind'] == 'circle':
                n_cyl += 1
                conf = '' if c.get('r_confident') else ' 半径存疑'
                text = (f'[{self.field.cell_name(c["x"], c["y"])}] 圆柱 '
                        f'{self._odom_xy_str(c["x"], c["y"])} 实测φ{c["r"]*2:.0f}mm '
                        f'n={c["n"]}{conf}')
            else:
                text = (f'[{self.field.cell_name(c["x"], c["y"])}] 障碍 '
                        f'{self._odom_xy_str(c["x"], c["y"])} 尺寸未知 n={c["n"]}')
            self.lst.insert(tk.END, text)

        self.blocked_cells = set()
        skipped = []
        for ob in self.obstacles:
            cr = self.field.cell_at(ob['x'], ob['y'])
            if not cr:
                continue
            if self._is_station_cell(*cr):
                # 工位所在格不封：封了就永远到不了工位。
                skipped.append(self.field.cell_label(*cr))
                continue
            self.blocked_cells.add(cr)
        cells = sorted({self.field.cell_name(c['x'], c['y']) for c in self.obstacles})
        extra = f"（{'、'.join(skipped)} 是工位格，不封锁）" if skipped else ''
        self._set_status(f'识别到 {n_cyl} 个圆柱，共 {len(self.obstacles)} 处障碍；'
                         f'占用格：{"、".join(cells) if cells else "无"}{extra}')
        self._redraw()

    def _is_station_cell(self, c, r):
        rc = self.field.cell_rect(c, r)
        for z in self.field.work_zones:
            zr = z['rect']
            if not (zr.x1 < rc.x0 or zr.x0 > rc.x1 or zr.y1 < rc.y0 or zr.y0 > rc.y1):
                return True
        return False

    # ==================== 规划 ====================

    def _cell_block(self):
        if self.var_strict.get() and self.blocked_cells:
            return self.grid.build_cell_blocked(self.blocked_cells)
        return None

    def _cost_fn(self):
        return mapping.centerline_cost(self.field) if self.var_line.get() else None

    def plan_to_goal(self):
        if self.goal_world is None:
            self._set_status('先在地图上点一下目标点')
            return
        fx, fy, _ = self._robot_field()
        start = self.grid.w2c(fx, fy)
        goal = self.grid.w2c(*self.goal_world)
        self.plan = mapping.plan_path(
            self.grid, self.mask, self.obstacles, start, goal,
            self.field.robot_radius, cell_block=self._cell_block(),
            cost_fn=self._cost_fn())
        self._report_plan(self.plan)
        self._redraw()

    def _report_plan(self, plan):
        warn = ''
        if plan.get('goal_blocked'):
            gname = self.field.cell_name(*self.goal_world)
            warn = f'  ⚠ {gname} 本身被封锁，已改到最近的可通行点'
        mc = plan.get('min_clear')
        clear = ('无圆柱障碍' if mc is None or not math.isfinite(mc)
                 else f'最小余量 {mc:.0f}mm')
        if plan['path']:
            self._set_status(f'路径 {len(plan["path"])} 格，{clear} —— {plan["note"]}{warn}')
            self.plan_var.set(
                f'规划：{self._odom_str(*self.goal_world)}  '
                f'[{self.field.cell_name(*self.goal_world)}]  '
                f'{len(self._waypoints(plan["path_world"]))} 个路点  '
                f'{clear}  {plan["note"]}{warn}')
        else:
            sealed = '、'.join(sorted(self.field.cell_label(c, r)
                                      for (c, r) in self.blocked_cells))
            extra = f'（{sealed} 整格封锁，把路断了）' if sealed and self.var_strict.get() else ''
            self._set_status(f'规划失败：{plan["note"]}{extra}')
            self.plan_var.set(f'规划失败：{plan["note"]}{extra}')

    def goto_cell(self):
        s = self.ent_cell.get().strip().upper()
        if len(s) < 2 or not s[0].isalpha() or not s[1:].isdigit():
            self._set_status('格子编号写法：A1 ~ E5')
            return
        col = ord(s[0]) - ord('A')
        row = int(s[1:]) - 1
        if not (0 <= col < self.field.cell_cols and 0 <= row < self.field.cell_rows):
            self._set_status(f'超出范围：只有 A1~'
                             f'{chr(ord("A") + self.field.cell_cols - 1)}{self.field.cell_rows}')
            return
        self.goal_world = self.field.cell_center(col, row)
        self._set_status(f'目标 {self.field.cell_label(col, row)}  '
                         f'{self._odom_str(*self.goal_world)}')
        self.plan_to_goal()

    def goto_odom(self):
        """按里程计坐标（米）定目标 —— 和 `ros2 topic echo /odom` 同一套数。"""
        try:
            ox = float(self.ent_gx.get())
            oy = float(self.ent_gy.get())
        except Exception:
            self._set_status('odom 目标按米填，例如 x=1.351 y=0.418')
            return
        if self.odom_start[2] is None:
            self._set_status('还没标定车起点，odom → 场地换算不出来：先点【标定里程计】')
            return
        fx, fy, _ = self._odom_to_field(ox, oy, 0.0)
        if not self.field.in_field(fx, fy):
            self._set_status(f'目标 odom ({ox:+.3f}, {oy:+.3f}) m 换算后不在场地里')
            return
        self.goal_world = (fx, fy)
        self._set_status(f'目标 odom ({ox:+.3f}, {oy:+.3f}) m  '
                         f'[场地格 {self.field.cell_name(fx, fy)}]')
        self.plan_to_goal()

    def _replan_soft(self):
        if self.goal_world is not None:
            self.plan_to_goal()
        else:
            self._redraw()

    # ==================== 下发导航（场地 → 里程计） ====================

    def _waypoints(self, path_mm, min_step_mm=15.0):
        """把世界系折线压成少量路点（去重 + 去共线，不切角）。"""
        if not path_mm:
            return []
        ded = [path_mm[0]]
        for p in path_mm[1:]:
            if math.hypot(p[0] - ded[-1][0], p[1] - ded[-1][1]) >= min_step_mm:
                ded.append(p)
        if ded[-1] != path_mm[-1]:
            ded.append(path_mm[-1])
        if len(ded) <= 2:
            return ded
        out = [ded[0]]
        for i in range(1, len(ded) - 1):
            a, b, c = out[-1], ded[i], ded[i + 1]
            abx, aby = b[0] - a[0], b[1] - a[1]
            acx, acy = c[0] - a[0], c[1] - a[1]
            cross = abx * acy - aby * acx
            perp = abs(cross) / max(math.hypot(acx, acy), 1e-9)
            if perp > 5.0:
                out.append(b)
        out.append(ded[-1])
        return out

    def _path_commands(self, path_mm, title=None):
        """世界系折线 → motion_planner 的 goto 指令（里程计系，米/度）。"""
        wps = self._waypoints(path_mm)
        cmds = []
        if title:
            cmds.append({'cmd': 'comment', 'text': title})
        n = len(wps)
        cur_deg = self._robot_field()[2]
        for i, (x, y) in enumerate(wps):
            if i < n - 1:
                hd = math.degrees(math.atan2(wps[i + 1][1] - y, wps[i + 1][0] - x))
            elif n >= 2:
                hd = math.degrees(math.atan2(y - wps[-2][1], x - wps[-2][0]))
            else:
                hd = cur_deg
            ox, oy, oth = self._field_to_odom(x, y, hd)
            cmds.append({'cmd': 'goto',
                         'x_m': round(ox, 3),
                         'y_m': round(oy, 3),
                         'yaw_deg': round(math.degrees(oth), 1)})
        return cmds

    def _planner_missing(self):
        """motion_planner 没起来（/motion_load 无订阅者）时给个明确的提示。"""
        self._set_status('motion_planner 没在运行：/motion_load 没有订阅者，指令没人执行')
        messagebox.showerror(
            'motion_planner 没在运行',
            '/motion_load 没有订阅者，指令发出去没人执行。\n\n'
            '先在另一个终端跑 scripts/run_car.sh，看到终端里出现\n'
            '"状态机就绪，等待 /motion_load 下发指令序列" 再点导航。')

    def navigate_plan(self):
        if not self.plan or not self.plan.get('path_world'):
            self._set_status('没有可执行的路径：先点地图或点【③ 规划到目标】')
            return
        if self.node.odom is None:
            messagebox.showwarning('没有里程计',
                                   '还没收到 /odom（F407 没回传），不能导航。\n'
                                   '检查 run_car.sh 和串口连接。')
            return
        if self.odom_start[2] is None:
            messagebox.showwarning(
                '还没标定车起点',
                '里程计 yaw 不是 0，现在没法把场地坐标换算成里程计坐标。\n'
                '把车摆到【车起点】，再点左边【此刻车在车起点 → 标定里程计】。')
            return
        if self.node.robot_status and self.node.robot_status[0] != 1:
            if not messagebox.askyesno(
                    '车还没就绪',
                    f'整车状态是"{self.node.robot_status[0]}"（不是就绪/使能），'
                    f'现在下发导航车也不会动。仍要下发吗？'):
                return
        cmds = self._path_commands(
            self.plan['path_world'],
            f'导航 → {self.field.cell_name(*self.goal_world)} '
            f'{self._odom_str(*self.goal_world)}')
        if not self.node.send_sequence(cmds):
            self._planner_missing()
            return
        self._set_status(f'已下发 {len(cmds) - 1} 个路点，motion_planner 接管…')

    # ==================== 任务路线 ====================

    def _fill_mission_list(self):
        if not self.field.mission_route:
            self.mission_info.set('field.json 里没有 mission.route')

    def plan_mission(self):
        if not self.field.mission_route:
            self._set_status('field.json 里没有 mission.route')
            return
        legs, total = mapping.plan_mission(
            self.field, self.grid, self.mask, self.obstacles, None,
            self.field.robot_radius, cell_block=self._cell_block(),
            cost_fn=self._cost_fn())
        self.mission_legs = legs
        self.mission_total = total
        self.plan = None
        ok = sum(1 for leg in legs if leg['ok'])
        eta = total / max(self.field.mission_speed, 1.0)
        self.mission_info.set(f'{ok}/{len(legs)} 段可通  总长 {total / 1000:.2f} m  '
                              f'约 {eta:.0f} s')
        bad = [f'{i + 1}.{leg["note"]}' for i, leg in enumerate(legs) if not leg['ok']]
        self._set_status(f'全程规划：{ok}/{len(legs)} 段可通，总长 {total / 1000:.2f} m'
                         + (f'；不通：{"；".join(bad)}' if bad else ''))
        self._redraw()

    def execute_mission(self):
        if not self.mission_legs:
            self._set_status('先点【规划全程】')
            return
        if self.node.odom is None:
            messagebox.showwarning('没有里程计', '还没收到 /odom，不能导航。')
            return
        if self.odom_start[2] is None:
            messagebox.showwarning(
                '还没标定车起点',
                '把车摆到【车起点】，再点左边【此刻车在车起点 → 标定里程计】。')
            return
        if self.node.robot_status and self.node.robot_status[0] != 1:
            if not messagebox.askyesno(
                    '车还没就绪',
                    '整车状态不是就绪/使能，现在下发车不会动。仍要下发吗？'):
                return
        bad = [i for i, leg in enumerate(self.mission_legs) if not leg['ok']]
        if bad:
            messagebox.showerror('有段不通', f'第 {",".join(str(i + 1) for i in bad)} 段规划失败，'
                                             f'请先处理障碍或改路线。')
            return
        cmds = [{'cmd': 'comment', 'text': '任务路线（雷达地图规划）'}]
        # 车不在任务起点时，先规划一段过去
        first = self.mission_legs[0]['from_xy']
        cur = self._robot_field()[:2]
        if math.hypot(cur[0] - first[0], cur[1] - first[1]) > 150.0:
            res = mapping.plan_path(
                self.grid, self.mask, self.obstacles,
                self.grid.w2c(*cur), self.grid.w2c(*first),
                self.field.robot_radius, cell_block=self._cell_block(),
                cost_fn=self._cost_fn())
            if not res['path']:
                messagebox.showerror('到不了起点', f'从当前位置到任务起点不通：{res["note"]}')
                return
            cmds += self._path_commands(res['path_world'], '先去任务起点')
        for i, leg in enumerate(self.mission_legs):
            cmds += self._path_commands(
                leg['path'], f'{i + 1}. {leg["from_name"]} → {leg["to_name"]}')
        if not self.node.send_sequence(cmds):
            self._planner_missing()
            return
        self._set_status(f'已下发全程路线：{len(cmds)} 条指令（其中 goto '
                         f'{sum(1 for c in cmds if c["cmd"] == "goto")} 个）')

    def clear_route(self):
        self.mission_legs = []
        self.mission_total = 0.0
        self.plan = None
        self.mission_info.set('还没规划')
        self._redraw()

    def clear_scan(self):
        self.raw_points = []
        self.points = []
        self.obstacles = []
        self.blocked_cells = set()
        self.plan = None
        self.scan_revs_done = 0
        self.lst.delete(0, tk.END)
        self._set_status('已清空点云和障碍')
        self._redraw()

    # ==================== 鼠标 ====================

    def on_motion(self, ev):
        self._last_hover = (ev.x, ev.y)
        self._draw_hover(ev.x, ev.y)

    def _clear_hover(self):
        self._last_hover = None
        self.canvas.delete('hover')

    def _draw_hover(self, px, py):
        c = self.canvas
        c.delete('hover')
        w = self._px_to_world(px, py)
        if w is None:
            return
        f = self.field
        x, y = w
        dist = math.hypot(x - f.lidar_x, y - f.lidar_y)
        cell = f.cell_name(x, y)

        c.create_line(px, 0, px, c.winfo_height(), fill='#7a7a7a',
                      dash=(3, 5), tags='hover')
        c.create_line(0, py, c.winfo_width(), py, fill='#7a7a7a',
                      dash=(3, 5), tags='hover')
        c.create_oval(px - 3, py - 3, px + 3, py + 3,
                      outline='#3a3a3a', tags='hover')

        text = (f'{self._odom_str(x, y)}   [{cell}]   '
                f'距雷达 {dist:.0f}mm')
        tid = c.create_text(px + 14, py + 16, text=text, anchor='nw',
                            fill='#1b1b1b', font=('Consolas', 9), tags='hover')
        box = c.bbox(tid)
        if box:
            if box[2] > c.winfo_width() - 4:
                c.coords(tid, px - 14, py + 16)
                c.itemconfig(tid, anchor='ne')
                box = c.bbox(tid)
            if box and box[3] > c.winfo_height() - 4:
                c.coords(tid, px + 14, py - 16)
                c.itemconfig(tid, anchor='nw' if box[2] < c.winfo_width() - 4 else 'ne')
                box = c.bbox(tid)
            if box:
                bg = c.create_rectangle(box[0] - 4, box[1] - 2, box[2] + 4, box[3] + 2,
                                        fill='#fffbe6', outline='#d8d0a0', tags='hover')
                c.tag_lower(bg, tid)

    def on_click(self, ev):
        w = self._px_to_world(ev.x, ev.y)
        if w is None:
            return
        if self.var_snap.get():
            cr = self.field.cell_at(*w)
            if cr:
                w = self.field.cell_center(*cr)
        self.goal_world = w
        self._set_status(f'目标 {self._odom_str(*w)}  '
                         f'[场地格 {self.field.cell_name(*w)}]')
        self.plan_to_goal()
        if self.var_auto.get() and self.plan and self.plan.get('path'):
            self.navigate_plan()

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

    def _redraw(self):
        if not hasattr(self, 'canvas'):
            return
        c = self.canvas
        c.delete('all')
        f = self.field
        s, ox, oy = self._transform()

        def R(r, fill=None, outline=None, width=1):
            p0 = self._w2p(r.x0, r.y1)
            p1 = self._w2p(r.x1, r.y0)
            c.create_rectangle(p0[0], p0[1], p1[0], p1[1],
                               fill=fill, outline=outline, width=width)

        R(mapping.Rect(-f.hw, -f.hh, f.hw, f.hh), fill=C_WHITE,
          outline='#464646', width=2)
        for r in f.connector_rects():
            R(r, fill=C_CONN)
        for r in f.lane_rects:
            R(r, fill=C_LANE)
        for r in f.yellow_rects:
            R(r, fill=C_YELLOW, outline='#ece2ac')
        for z in f.work_zones:
            R(z['rect'], outline='#6e9b6e')
            x, y = self._w2p(z['rect'].x0, z['rect'].y1)
            c.create_text(x + 2, y - 8, text=z['name'], anchor='w', fill='#50734f',
                          font=('', 9))
            ax, ay = self._w2p(*z['approach'])
            c.create_line(ax - 4, ay, ax + 4, ay, fill='#2e9e6b', width=2)
            c.create_line(ax, ay - 4, ax, ay + 4, fill='#2e9e6b', width=2)
        for z in f.start_zones:
            R(z['rect'], fill=C_BLUE)
            x, y = self._w2p(z['cx'], z['cy'])
            c.create_text(x, y, text=z['name'][-1], fill='white',
                          font=('', 11, 'bold'))

        # 被障碍占住、整格不能通车的格子
        for (cc, rr) in self.blocked_cells:
            r = f.cell_rect(cc, rr)
            p0 = self._w2p(r.x0, r.y1)
            p1 = self._w2p(r.x1, r.y0)
            c.create_rectangle(p0[0], p0[1], p1[0], p1[1],
                               fill=C_CYL, stipple='gray25', outline=C_CYL, width=2)
            c.create_text((p0[0] + p1[0]) / 2, p0[1] + 12,
                          text=f'不通车 {f.cell_mm:.0f}mm', fill='#b3122c',
                          font=('', 9, 'bold'))

        # 5×5 网格
        if self.var_grid.get():
            for x in f.gx:
                p0 = self._w2p(x, f.gy[0]); p1 = self._w2p(x, f.gy[-1])
                c.create_line(p0[0], p0[1], p1[0], p1[1], fill='#b4b4b4', dash=(4, 4))
            for y in f.gy:
                p0 = self._w2p(f.gx[0], y); p1 = self._w2p(f.gx[-1], y)
                c.create_line(p0[0], p0[1], p1[0], p1[1], fill='#b4b4b4', dash=(4, 4))
            for rr in range(f.cell_rows):
                for cc in range(f.cell_cols):
                    px, py = self._w2p(*f.cell_center(cc, rr))
                    c.create_text(px, py, text=f.cell_label(cc, rr), fill='#8a8a8a',
                                  font=('', 9))

        # 雷达点云
        for x, y, *_ in self.points[:4000]:
            px, py = self._w2p(x, y)
            c.create_rectangle(px - 1, py - 1, px + 1, py + 1,
                               outline=C_POINT, fill=C_POINT)

        # 单点规划路径
        if self.plan and self.plan.get('path_world'):
            pts = [self._w2p(*q) for q in self.plan['path_world']]
            c.create_line(*[v for p in pts for v in p], fill=C_PATH, width=3)
            for p in (pts[0], pts[-1]):
                c.create_oval(p[0] - 5, p[1] - 5, p[0] + 5, p[1] + 5,
                              fill=C_PATH, outline='')

        # 任务路线
        if self.mission_legs:
            pal = ['#146ede', '#2e9e6b', '#b0601f', '#8e24aa', '#c22a52',
                   '#0b6e8a', '#5a7d1f', '#7a4fbf', '#d08114']
            for i, leg in enumerate(self.mission_legs):
                col = pal[i % len(pal)]
                if leg['path']:
                    pts = [self._w2p(*q) for q in leg['path']]
                    c.create_line(*[v for p in pts for v in p], fill=col, width=3)
                if leg['from_xy']:
                    px, py = self._w2p(*leg['from_xy'])
                    c.create_oval(px - 7, py - 7, px + 7, py + 7,
                                  fill='white', outline=col, width=2)
                    c.create_text(px, py, text=str(i + 1), fill=col,
                                  font=('Consolas', 8, 'bold'))

        # 障碍
        for ob in self.obstacles:
            px, py = self._w2p(ob['x'], ob['y'])
            if ob['kind'] == 'circle':
                conf = ob.get('r_confident')
                r_mm = ob['r'] if conf else mapping._ob_radius(ob)
                r = max(r_mm * s, 3)
                c.create_oval(px - r, py - r, px + r, py + r, outline=C_CYL,
                              width=2, dash=() if conf else (3, 3))
                tag = f'φ{r_mm * 2:.0f}mm' + ('' if conf else '?')
                c.create_text(px + r + 6, py, anchor='w', fill=C_CYL,
                              text=f'{tag} ({ob["x"]:.0f},{ob["y"]:.0f})',
                              font=('', 9))
            else:
                c.create_oval(px - 4, py - 4, px + 4, py + 4,
                              outline=C_BLOB, width=2)

        # 目标点
        if self.goal_world:
            gx, gy = self._w2p(*self.goal_world)
            c.create_line(gx - 8, gy, gx + 8, gy, fill=C_GOAL, width=2)
            c.create_line(gx, gy - 8, gx, gy + 8, fill=C_GOAL, width=2)

        # 车起点（里程计原点对应的场地位置）
        sx, sy = self._w2p(self.robot_start[0], self.robot_start[1])
        c.create_rectangle(sx - 6, sy - 6, sx + 6, sy + 6,
                           outline='#7a4fbf', width=2, dash=(2, 2))
        c.create_text(sx + 9, sy + 8, text='起点', anchor='nw', fill='#7a4fbf', font=('', 8))

        # 车（当前位姿）。没标定就不画——画出来也是错的，反而误导。
        o = self.node.odom
        if o is not None and self.odom_start[2] is not None:
            fx, fy, fd = self._odom_to_field(*o)
            cx, cy = self._w2p(fx, fy)
            a = math.radians(fd)
            tri = [(cx + 14 * math.cos(a), cy - 14 * math.sin(a)),
                   (cx + 9 * math.cos(a + 2.5), cy - 9 * math.sin(a + 2.5)),
                   (cx + 9 * math.cos(a - 2.5), cy - 9 * math.sin(a - 2.5))]
            c.create_polygon(*[v for p in tri for v in p],
                             fill='#d81b60', outline='#8e0038')

        # 雷达
        lx, ly = self._w2p(f.lidar_x, f.lidar_y)
        c.create_oval(lx - 5, ly - 5, lx + 5, ly + 5, fill=C_LIDAR, outline='')
        a = math.radians(f.lidar_yaw)
        c.create_line(lx, ly, lx + 34 * math.cos(a), ly - 34 * math.sin(a),
                      fill=C_LIDAR, width=3, arrow=tk.LAST)

        # 图例 / 比例尺
        c.create_text(10, 10, anchor='nw', fill='#555555',
                      text=f'场地 {f.W:.0f}×{f.H:.0f}mm  十字车道 {f.lane_w:.0f}mm  '
                           f'外圈 {f.ring_w:.0f}mm  '
                           f'{f.cell_cols}×{f.cell_rows} 网格 格边长 {f.cell_mm:.0f}mm  '
                           f'雷达@场地({f.lidar_x:.0f},{f.lidar_y:.0f}) '
                           f'yaw={f.lidar_yaw:.0f}°  '
                           f'点 {len(self.points)}  '
                           f'{"白区可走（整个场地除黄块）" if f.white_drivable else "只走灰色【田】字车道"}',
                      font=('', 9))
        step = self.lidar.step_deg() if self.lidar is not None else None
        if step:
            ok = 0.3 <= step <= 1.2
            c.create_text(10, 26, anchor='nw',
                          fill='#2e7d32' if ok else '#c62828',
                          text=f'帧体检：{step:.2f}°/点  '
                               f'{"正常（≈400 点/圈）" if ok else "异常！角度标度或方向有问题"}',
                          font=('', 8))

        c.create_text(10, c.winfo_height() - 56, anchor='w', fill='#b3122c',
                      text='红格 = 被占住的整个 450mm 格（不是物体尺寸）；'
                           '取消【整格封锁】只按实体 φ 绕行',
                      font=('', 8))
        if self._last_hover:
            self._draw_hover(*self._last_hover)

        bar = 500 * s
        x0, y0 = 14, c.winfo_height() - 22
        c.create_line(x0, y0, x0 + bar, y0, fill='#555555', width=2)
        c.create_text(x0 + bar / 2, y0 - 8, text='500mm', fill='#555555', font=('', 8))

    # ==================== 定时刷新 ====================

    def _update_labels(self):
        o = self.node.odom
        if o:
            # 第一次收到里程计、且车还在起点附近（|x|,|y| < 0.2m）→ 自动锁存
            # 里程计 yaw。JY901S 上电 yaw 不一定是 0，这一步不能省。
            if self.odom_start[2] is None and abs(o[0]) < 0.20 and abs(o[1]) < 0.20:
                self.odom_start = [o[0], o[1], math.degrees(o[2])]
                self._set_status(
                    f'已自动标定里程计起点（θ={math.degrees(o[2]):+.1f}°）。'
                    f'如果车现在不在场地起点，请把车摆回去后重新点【标定里程计】。')
            if self.odom_start[2] is not None:
                fx, fy, fd = self._odom_to_field(*o)
                self.pose_var.set(
                    f'里程计 x={o[0]:+.3f}m y={o[1]:+.3f}m θ={math.degrees(o[2]):+.1f}°    '
                    f'[场地格 {self.field.cell_name(fx, fy)}]')
                self.cal_var.set(
                    f'里程计标定: ({self.odom_start[0]:+.3f}, '
                    f'{self.odom_start[1]:+.3f})m θ={self.odom_start[2]:+.1f}°')
            else:
                self.pose_var.set(
                    f'里程计 x={o[0]:+.3f}m y={o[1]:+.3f}m θ={math.degrees(o[2]):+.1f}°    '
                    f'⚠ 未标定车起点（车不在起点附近，或刚上电）：把车摆到起点后点【标定里程计】')
        else:
            self.pose_var.set('当前位姿: 未收到 /odom（检查 run_car.sh / 串口）')

        st = f'状态机: {self.node.motion_state or "—"}'
        if self.node.robot_status:
            state, err, mask = self.node.robot_status
            state_name = {0: '未使能', 1: '就绪', 2: '故障'}.get(state, state)
            err_names = [n for bit, n in ((0, '树莓派离线'), (1, '急停'), (2, '电机掉线'))
                         if err & (1 << bit)]
            err_text = f'0x{err:02X}' + (f'（{"、".join(err_names)}）' if err_names else '')
            st += f'    整车: {state_name}  错误={err_text}  电机=0x{mask:02X}'
        self.state_var.set(st)

    def _refresh(self):
        got = self._drain()
        o = self.node.odom
        if o is not None:
            if (self._last_odom is None
                    or abs(o[0] - self._last_odom[0]) > 0.003
                    or abs(o[1] - self._last_odom[1]) > 0.003
                    or abs(o[2] - self._last_odom[2]) > 0.01):
                self._need_redraw = True
        self._last_odom = o
        self._update_labels()
        if got or self._need_redraw:
            self._redraw()
            self._need_redraw = False
        self.root.after(100, self._refresh)

    def close(self):
        # 关窗口时先停雷达（发 D6 停止命令、关串口）。窗口可能已经销毁，
        # 这时改控件会抛 TclError，忽略即可。
        try:
            self.stop_lidar()
        except Exception:
            pass

    # ==================== 动作序列编辑（和原来一样） ====================

    def _on_type_change(self, name):
        self.hint_label.config(text=f"参数：{COMMAND_TYPES[name]['hint']}")

    def add_cmd(self):
        try:
            cmd = build_cmd(self.cmd_type.get(), self.param_entry.get())
        except Exception as e:
            messagebox.showerror('参数错误', f'参数解析失败：{e}')
            return
        self.listbox.insert(tk.END, cmd_to_text(cmd))
        self.commands.append(cmd)

    def move_cmd(self, direction):
        sel = self.listbox.curselection()
        if not sel:
            return
        i = sel[0]
        j = i + direction
        if not (0 <= j < len(self.commands)):
            return
        self.commands[i], self.commands[j] = self.commands[j], self.commands[i]
        self._reload_listbox()

    def del_cmd(self):
        sel = self.listbox.curselection()
        if not sel:
            return
        i = sel[0]
        del self.commands[i]
        self._reload_listbox()

    def clear_cmds(self):
        self.commands.clear()
        self._reload_listbox()

    def _reload_listbox(self):
        self.listbox.delete(0, tk.END)
        for c in self.commands:
            self.listbox.insert(tk.END, cmd_to_text(c))

    def import_json(self):
        path = filedialog.askopenfilename(filetypes=[('JSON 文件', '*.json')])
        if not path:
            return
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            cmds = data['commands'] if isinstance(data, dict) else data
            self.commands = list(cmds)
            self._reload_listbox()
        except Exception as e:
            messagebox.showerror('导入失败', str(e))

    def export_json(self):
        path = filedialog.asksaveasfilename(defaultextension='.json',
                                            filetypes=[('JSON 文件', '*.json')])
        if not path:
            return
        try:
            with open(path, 'w', encoding='utf-8') as f:
                json.dump({'commands': self.commands}, f, ensure_ascii=False, indent=2)
            messagebox.showinfo('导出成功', f'已保存到\n{path}')
        except Exception as e:
            messagebox.showerror('导出失败', str(e))

    def run_all(self):
        if not self.commands:
            messagebox.showwarning('空序列', '先添加几条指令')
            return
        if not self.node.send_sequence(self.commands):
            self._planner_missing()

    def run_selected(self):
        sel = self.listbox.curselection()
        if not sel:
            messagebox.showwarning('未选择', '先在列表里选中一条指令')
            return
        if not self.node.send_sequence([self.commands[sel[0]]]):
            self._planner_missing()


def main():
    rclpy.init()
    node = GuiRosNode()

    root = tk.Tk()
    app = App(root, node)

    def spin_ros():
        rclpy.spin_once(node, timeout_sec=0)
        root.after(20, spin_ros)

    def on_close():
        try:
            app.close()
        finally:
            node.destroy_node()
            rclpy.shutdown()
            root.destroy()

    root.protocol('WM_DELETE_WINDOW', on_close)
    root.after(20, spin_ros)
    root.mainloop()


if __name__ == '__main__':
    main()
