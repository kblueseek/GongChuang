#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""上位机：在笔记本上编辑/测试小车的动作序列，调好之后导出 JSON 给 motion_planner 场上跑。

================ 大白话：这个程序是干什么的 ================

它是你调试时的"遥控面板"（跑在笔记本上，通过 WiFi 和树莓派上的 ROS2 通信）。
核心思路：你在界面里编辑一串动作（前进0.5米、右转90°、云台抬升……），
点"执行"就发到树莓派，motion_planner 会一条一条跑；调好之后点"导出"，
把这串动作存成 JSON 文件，赛场上让 motion_planner 直接加载这份文件自动跑。

它还实时画出里程计轨迹，你能肉眼看到小车闭环到底准不准（走0.5米是不是真的走0.5米）。

================ 运行前提 ================

1. 笔记本要装 ROS2（和树莓派同一个版本），并设置和树莓派相同的 ROS_DOMAIN_ID，
   让两边通过 DDS 互相发现。
2. 树莓派上先启动整车（ros2 launch ... system.launch.py），保证 f407_bridge 和
   motion_planner 都在跑。
3. 然后在本机运行：  python3 motion_gui.py

================ 指令类型（和 motion_planner 完全一致） ================

move 前进/后退(米)  strafe 横移(米)  turn 转向(度)  gimbal 云台(yaw,lift,extend度)
gripper/plate/cam 舵机(度)  enable 使能  delay 等待(ms)  calibrate 视觉校准  comment 注释
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32MultiArray, String

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

# ==================== 指令类型的"元信息"：每种指令要填什么参数、怎么提示 ====================
# params 是"逗号分隔"的参数顺序，hint 是给用户看的提示。
COMMAND_TYPES = {
    'move':      {'params': 'dist_m', 'hint': '前进/后退距离(米)，负=后退'},
    'strafe':    {'params': 'dist_m', 'hint': '横向平移(米)，正=左'},
    'turn':      {'params': 'deg', 'hint': '转向角(度)，正=左转(逆时针)，负=右转'},
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
    # 按逗号拆分；comment 的文本可能含逗号，特殊处理（不拆，整体当文本）
    if name == 'comment':
        return {'cmd': name, 'text': params_str.strip()}
    parts = [p.strip() for p in params_str.split(',') if p.strip() != '']

    if name in ('move', 'strafe'):
        return {'cmd': name, 'dist_m': float(parts[0]) if parts else 0.0}
    if name == 'turn':
        return {'cmd': name, 'deg': float(parts[0]) if parts else 0.0}
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


# ==================== ROS2 节点：负责和树莓派通信 ====================

class GuiRosNode(Node):
    """上位机自己的 ROS2 节点：发指令、收里程计/状态。"""

    def __init__(self):
        super().__init__('motion_gui')
        # 发布：发序列 / 叫停 / 使能 / 急停
        self.pub_load = self.create_publisher(String, '/motion_load', 10)
        self.pub_stop = self.create_publisher(Bool, '/motion_stop', 10)
        self.pub_enable = self.create_publisher(Bool, '/cmd_enable', 10)
        self.pub_estop = self.create_publisher(Bool, '/cmd_estop', 10)
        self.pub_estop_clear = self.create_publisher(Bool, '/cmd_estop_clear', 10)
        # 订阅：里程计 / 状态机进度 / 整车状态
        self.create_subscription(Odometry, '/odom', self._cb_odom, 10)
        self.create_subscription(String, '/motion_state', self._cb_state, 10)
        self.create_subscription(Int32MultiArray, '/robot_status', self._cb_status, 10)

        # 最新数据缓存（界面定时来读，不在回调里直接刷 UI）
        self.odom = None            # (x, y, theta)
        self.traj = []              # 历史轨迹 [(x,y), ...]
        self.motion_state = ''
        self.robot_status = None    # [state, err, mask]

    def _cb_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, theta)
        self.traj.append((self.odom[0], self.odom[1]))

    def _cb_state(self, msg: String):
        self.motion_state = msg.data

    def _cb_status(self, msg: Int32MultiArray):
        if len(msg.data) >= 3:
            self.robot_status = list(msg.data[:3])

    # ---- 对外的动作方法 ----
    def send_sequence(self, commands: list):
        s = String()
        s.data = json.dumps(commands, ensure_ascii=False)
        self.pub_load.publish(s)

    def send_stop(self):
        b = Bool()
        b.data = True
        self.pub_stop.publish(b)

    def send_enable(self, en: bool):
        b = Bool()
        b.data = en
        self.pub_enable.publish(b)

    def send_estop(self):
        b = Bool()
        b.data = True
        self.pub_estop.publish(b)

    def send_estop_clear(self):
        b = Bool()
        b.data = True
        self.pub_estop_clear.publish(b)


# ==================== tkinter 界面 ====================

class App:
    def __init__(self, root: tk.Tk, node: GuiRosNode):
        self.root = root
        self.node = node
        self.commands = []   # 当前编辑的指令序列（list of dict）
        root.title('智能搬运上位机 — 动作序列编辑器')
        root.geometry('1000x640')

        # ---------- 左侧：指令序列编辑 ----------
        left = ttk.Frame(root, padding=8)
        left.pack(side=tk.LEFT, fill=tk.BOTH, expand=False)

        ttk.Label(left, text='动作序列（从上到下依次执行）', font=('', 11, 'bold')).pack(anchor='w')
        self.listbox = tk.Listbox(left, width=40, height=24)
        self.listbox.pack(fill=tk.BOTH, expand=True, pady=4)

        # 添加指令：选类型 + 填参数
        addrow = ttk.Frame(left)
        addrow.pack(fill=tk.X, pady=2)
        self.cmd_type = tk.StringVar(value='move')
        self.type_menu = ttk.OptionMenu(addrow, self.cmd_type, 'move', *COMMAND_TYPES.keys(),
                                        command=self._on_type_change)
        self.type_menu.pack(side=tk.LEFT)
        self.param_entry = ttk.Entry(addrow)
        self.param_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        self.hint_label = ttk.Label(left, text='', foreground='gray')
        self.hint_label.pack(anchor='w')
        self._on_type_change('move')

        ttk.Button(addrow, text='＋添加', command=self.add_cmd).pack(side=tk.LEFT)

        # 编辑按钮
        editrow = ttk.Frame(left)
        editrow.pack(fill=tk.X, pady=4)
        ttk.Button(editrow, text='上移', command=lambda: self.move_cmd(-1)).pack(side=tk.LEFT, padx=2)
        ttk.Button(editrow, text='下移', command=lambda: self.move_cmd(+1)).pack(side=tk.LEFT, padx=2)
        ttk.Button(editrow, text='删除', command=self.del_cmd).pack(side=tk.LEFT, padx=2)
        ttk.Button(editrow, text='清空', command=self.clear_cmds).pack(side=tk.LEFT, padx=2)

        iorow = ttk.Frame(left)
        iorow.pack(fill=tk.X, pady=2)
        ttk.Button(iorow, text='导入JSON', command=self.import_json).pack(side=tk.LEFT, padx=2)
        ttk.Button(iorow, text='导出JSON', command=self.export_json).pack(side=tk.LEFT, padx=2)

        # ---------- 右侧：轨迹 + 状态 ----------
        right = ttk.Frame(root, padding=8)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(right, bg='white', width=460, height=380)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        self.pose_label = ttk.Label(right, text='当前位姿: 未收到里程计', font=('', 11))
        self.pose_label.pack(anchor='w', pady=2)
        self.state_label = ttk.Label(right, text='状态机: —', foreground='blue')
        self.state_label.pack(anchor='w')
        self.status_label = ttk.Label(right, text='整车状态: —')
        self.status_label.pack(anchor='w')

        # ---------- 底部：动作按钮 ----------
        bottom = ttk.Frame(root, padding=8)
        bottom.pack(side=tk.BOTTOM, fill=tk.X)

        ttk.Button(bottom, text='▶ 执行整段', command=self.run_all).pack(side=tk.LEFT, padx=4)
        ttk.Button(bottom, text='▶ 执行选中一条', command=self.run_selected).pack(side=tk.LEFT, padx=4)
        ttk.Button(bottom, text='■ 停止', command=self.node.send_stop).pack(side=tk.LEFT, padx=4)
        ttk.Button(bottom, text='使能', command=lambda: self.node.send_enable(True)).pack(side=tk.LEFT, padx=4)
        ttk.Button(bottom, text='失能', command=lambda: self.node.send_enable(False)).pack(side=tk.LEFT, padx=4)
        ttk.Button(bottom, text='急停', command=self.node.send_estop).pack(side=tk.LEFT, padx=4)
        ttk.Button(bottom, text='解除急停', command=self.node.send_estop_clear).pack(side=tk.LEFT, padx=4)

        self._refresh_ui()   # 启动定时刷新

    # ---------- 编辑操作 ----------
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

    # ---------- 导入 / 导出 ----------
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

    # ---------- 执行 ----------
    def run_all(self):
        if not self.commands:
            messagebox.showwarning('空序列', '先添加几条指令')
            return
        self.node.send_sequence(self.commands)

    def run_selected(self):
        sel = self.listbox.curselection()
        if not sel:
            messagebox.showwarning('未选择', '先在列表里选中一条指令')
            return
        self.node.send_sequence([self.commands[sel[0]]])   # 只发这一条

    # ---------- 定时刷新界面 ----------
    def _refresh_ui(self):
        o = self.node.odom
        if o:
            self.pose_label.config(text=f'当前位姿: x={o[0]:.3f}m y={o[1]:.3f}m 朝向={math.degrees(o[2]):.1f}°')
        self.state_label.config(text=f'状态机: {self.node.motion_state or "—"}')
        if self.node.robot_status:
            state, err, mask = self.node.robot_status
            state_name = {0: '未使能', 1: '就绪', 2: '故障'}.get(state, state)
            self.status_label.config(text=f'整车状态: {state_name}  错误码={err}  电机在线掩码=0x{mask:02X}')
        self._draw_traj()
        self.root.after(50, self._refresh_ui)   # 每 50ms 刷一次

    def _draw_traj(self):
        """把历史轨迹画到画布上（自动缩放），起点画绿点、当前画红点。"""
        traj = self.node.traj
        if len(traj) < 2:
            return
        c = self.canvas
        c.delete('all')
        w = c.winfo_width() or 460
        h = c.winfo_height() or 380
        margin = 20
        xs = [p[0] for p in traj]
        ys = [p[1] for p in traj]
        xmin, xmax = min(xs), max(xs)
        ymin, ymax = min(ys), max(ys)
        span = max(xmax - xmin, ymax - ymin, 0.05)   # 防止除以 0
        def sx(x):
            return margin + (x - xmin) / span * (w - 2 * margin)
        def sy(y):
            return h - margin - (y - ymin) / span * (h - 2 * margin)   # y 轴向上
        pts = [(sx(x), sy(y)) for x, y in traj]
        c.create_line(*[v for p in pts for v in p], fill='blue', width=2)   # 轨迹线
        c.create_oval(sx(xs[0]) - 4, sy(ys[0]) - 4, sx(xs[0]) + 4, sy(ys[0]) + 4,
                      fill='green', outline='')                              # 起点
        c.create_oval(sx(xs[-1]) - 5, sy(ys[-1]) - 5, sx(xs[-1]) + 5, sy(ys[-1]) + 5,
                      fill='red', outline='')                                # 当前点


def main():
    # 1. 初始化 ROS2，创建节点
    rclpy.init()
    node = GuiRosNode()

    # 2. 建窗口
    root = tk.Tk()
    App(root, node)

    # 3. 让 ROS2 和 tkinter 的事件循环"共用"：每隔 20ms 旋一次 ROS2
    def spin_ros():
        rclpy.spin_once(node, timeout_sec=0)
        root.after(20, spin_ros)

    def on_close():
        node.destroy_node()
        rclpy.shutdown()
        root.destroy()

    root.protocol('WM_DELETE_WINDOW', on_close)
    root.after(20, spin_ros)
    root.mainloop()


if __name__ == '__main__':
    main()
