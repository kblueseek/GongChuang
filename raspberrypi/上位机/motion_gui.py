#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""上位机：动作序列 + 雷达地图 + 九节点动作序列 + PID 调参（四合一）。

================ 大白话：这个程序是干什么的 ================

左边四个页签：

  【动作序列】通用指令序列：编辑 move / turn / gimbal / gripper / script …
      点底部【▶ 执行】发到树莓派，motion_planner 一条一条跑。

  【雷达 / 地图】从 vision 分支的 map_app 整合进来：
      · 按 field.json 的尺寸画出场地（灰色车道、淡黄禁行块、5×5 格）；
      · 接 D6 雷达串口（或模拟雷达）扫几圈，把回波打到地图上；
      · 自动识别圆柱/未知障碍，把占住的 450mm 格标红；
      · 鼠标点地图上的任意位置 → A* 规划绕开障碍的路径（只规划，不下发）。

  【节点动作序列】整场比赛的 9 个点：每到一个点要做什么，都在这里编辑。
      点的位置来自 field.json 的 mission.route（START → 扫码 → 原料 → … → START，
      正好 9 个）。点底部【▶ 执行】后，上位机先按雷达地图逐段规划路线，
      把车开到第 1 个点，跑该点的动作；再开去第 2 个点…直到第 9 个点。
      动作不只有给 407 的指令，还能：
        · script        启动任意程序（例如视觉识别窗口、标定脚本）；
        · align         跑视觉程序并读回 JSON，用视觉结果修正小车位姿；
        · calibrate     圆环工位视觉矫正：识别三圆环并把中间环对到画面基准，
                        然后把 407 里程计重置到对准位置（参数 = 圆环 odom x,y）；
        · vision_pick   视觉识别物料后自动执行定点抓取；
        · pick / place  抓取/放置宏（云台+夹爪+延时）。

  【PID 调参】在线改小车的所有关键参数，不用重新烧录：
      · 前进/平移最大速度、转向最大角速度、单轮最大转速；
      · 位置环 PID、转向环 PID、云台位置环 PID（位置环在 407 里跑）；
      · 云台零点 / 限位 / 松轴：把云台摆到固定位后【设为零点】，限位相对零点生效；
        限位可直接改数值后【下发限位】，和树莓派状态屏示教的是同一份；
        【松轴】让电机关掉保持力，方便手拨找位置（会先整车失能）；
      · 右边波形可切换 位置环 / 转向环 / 云台环，看目标 vs 实际。

================ 底部按钮 ================

只有两个：【▶ 执行】按当前页签决定执行什么（会先重新使能整车），
          【■ 停止】立刻停车并整车失能（含正在跑的脚本/视觉子程序）。
  动作序列页：执行左边整段序列
  节点页：    执行九点任务（走完 9 个点，每个点跑自己的动作）
  雷达页：    执行当前规划好的路径
  PID 页：    把当前所有调参值重新下发一遍

其他"点一下就动车"的按钮全部取消了，避免误触。

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
from collections import deque

import rclpy
from rclpy.node import Node
from car_interfaces.msg import Pose2D
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32MultiArray, Int32, Int32MultiArray, String

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import lidar_d6
import mapping

HERE = os.path.dirname(os.path.abspath(__file__))
CFG_PATH = os.path.join(HERE, "field.json")
NODES_PATH = os.path.join(HERE, "nodes.json")

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
    'calibrate': {'params': 'x_m,y_m', 'hint': '圆环工位视觉矫正：中间圆环的里程计坐标(米)，'
                 '例：-2.169,1.061。执行时开摄像头识别三个圆环，小步动车把中间环对到'
                 '画面标定基准，然后把 407 里程计重置到对准位置，清除累计误差。'
                 '失败/超时会跳过这一条。触发前确保 vision/main.py 没占用摄像头'},
    'comment':   {'params': 'text', 'hint': '注释文本(可含逗号)'},

    # ---- 复杂动作（motion_planner 里实现）----
    'script': {
        'params': 'program|args|cwd|wait|timeout_s',
        'hint': '启动一个程序。用竖线分隔：程序|参数|工作目录|等结束1/0|超时秒。'
                '例：python3|/home/pi/vision/main.py --device /dev/video0|/home/pi/vision|0|120'
    },
    'align': {
        'params': 'program|args|cwd|timeout_s|result_file',
        'hint': '视觉矫正小车位姿。程序最后一行打印 JSON：'
                '{"dx_m":0.01,"dy_m":-0.02,"dyaw_deg":1.5}；也可写进 result_file。'
    },
    'vision_pick': {
        'params': 'program|args|cwd|timeout_s',
        'hint': '视觉识别物品后抓取。程序打印 JSON：'
                '{"yaw_deg":0,"lift_deg":-40,"extend_deg":150}，缺省用默认抓取位。'
    },
    'pick': {
        'params': 'yaw_deg,lift_deg,safe_lift_deg,extend_deg,gripper_open,gripper_close,settle_ms',
        'hint': '抓取宏：张爪→到安全位→下降到抓取位→夹紧→抬起。'
                '例：0,-40,0,150,120,35,800'
    },
    'place': {
        'params': 'yaw_deg,lift_deg,safe_lift_deg,extend_deg,gripper_open,gripper_close,settle_ms',
        'hint': '放置宏：到安全位→下降到放置位→张爪→抬起。'
                '例：0,-40,0,150,120,35,800'
    },
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
        if 'x_m' in cmd or 'y_m' in cmd:
            return (f"视觉矫正 圆环odom=({cmd.get('x_m', 0):+.3f}, "
                    f"{cmd.get('y_m', 0):+.3f})m")
        return "视觉矫正（未填圆环坐标）"
    if name == 'comment':
        return f"// {cmd.get('text', '')}"
    if name == 'script':
        prog = cmd.get('program', '')
        args = cmd.get('args', '')
        wait = '等结束' if cmd.get('wait', True) else '后台'
        return f"启动程序 {prog} {args}".strip() + f"（{wait}）"
    if name == 'align':
        return f"视觉矫正位姿 ← {cmd.get('program', '')} {cmd.get('args', '')}".strip()
    if name == 'vision_pick':
        return f"视觉识别并抓取 ← {cmd.get('program', '')} {cmd.get('args', '')}".strip()
    if name in ('pick', 'place'):
        verb = '抓取' if name == 'pick' else '放置'
        return (f"{verb} yaw={cmd.get('yaw_deg', 0)}° lift={cmd.get('lift_deg', 0)}° "
                f"ext={cmd.get('extend_deg', 0)}° 夹爪{cmd.get('gripper_close', 35)}°")
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
        vals = [float(p) for p in parts] + [0.0, 0.0]
        return {'cmd': name, 'x_m': vals[0], 'y_m': vals[1]}

    # ---- 复杂动作都用竖线分隔，避免参数里带逗号（比如脚本参数） ----
    if name in ('script', 'align', 'vision_pick'):
        vals = [p.strip() for p in params_str.split('|')]
        vals += [''] * (5 - len(vals))
        cmd = {'cmd': name, 'program': vals[0], 'args': vals[1], 'cwd': vals[2]}
        if name == 'script':
            cmd['wait'] = (vals[3].lower() not in ('0', 'false', '否', 'n', 'no'))
            try:
                cmd['timeout_s'] = float(vals[4]) if vals[4] else 60.0
            except ValueError:
                cmd['timeout_s'] = 60.0
        elif name == 'align':
            try:
                cmd['timeout_s'] = float(vals[3]) if vals[3] else 60.0
            except ValueError:
                cmd['timeout_s'] = 60.0
            cmd['result_file'] = vals[4]
        else:
            try:
                cmd['timeout_s'] = float(vals[3]) if vals[3] else 60.0
            except ValueError:
                cmd['timeout_s'] = 60.0
        return cmd

    if name in ('pick', 'place'):
        vals = [float(p) for p in parts] + [0.0] * 7
        return {'cmd': name,
                'yaw_deg': vals[0],
                'lift_deg': vals[1],
                'safe_lift_deg': vals[2],
                'extend_deg': vals[3],
                'gripper_open': vals[4] if len(parts) > 4 else 120.0,
                'gripper_close': vals[5] if len(parts) > 5 else 35.0,
                'settle_ms': int(vals[6]) if len(parts) > 6 else 800}

    return {'cmd': name}


def wrap_deg(a: float) -> float:
    """角度归一化到 (-180, 180]。"""
    return (a + 180.0) % 360.0 - 180.0


# ==================== 在线调参项 ====================
# (参数编号, 键名, 显示名, 最小, 最大, 步长, 默认值, 是否整数)
# 编号必须和 F407 的 PiComm.h / pi_comm.py 一致。
TUNE_PARAMS = [
    (0x01, 'max_vx',    '前进最大速度 (m/s)',    0.0, 3.0, 0.01, 1.0, False),
    (0x02, 'max_vy',    '平移最大速度 (m/s)',    0.0, 3.0, 0.01, 1.0, False),
    (0x03, 'max_wz',    '转向最大角速度 (rad/s)', 0.0, 8.0, 0.05, 3.0, False),
    (0x04, 'max_rpm',   '单轮最大转速 (RPM)',    0,   600, 5,    300, True),
    (0x05, 'pos_kp',    '位置环 Kp',             0.0,  8.0, 0.01, 1.5, False),
    (0x06, 'pos_ki',    '位置环 Ki',             0.0,  5.0, 0.01, 0.3, False),
    (0x07, 'pos_kd',    '位置环 Kd',             0.0,  5.0, 0.01, 0.2, False),
    (0x08, 'head_kp',   '转向环 Kp',             0.0, 20.0, 0.05, 4.0, False),
    (0x09, 'head_ki',   '转向环 Ki',             0.0, 10.0, 0.01, 0.0, False),
    (0x0A, 'head_kd',   '转向环 Kd',             0.0, 10.0, 0.01, 0.0, False),
    (0x0B, 'gim_kp',    '云台位置环 Kp',         0.0, 100.0, 0.5, 20.0, False),
    (0x0C, 'gim_ki',    '云台位置环 Ki',         0.0, 20.0, 0.05, 0.0, False),
    (0x0D, 'gim_kd',    '云台位置环 Kd',         0.0, 10.0, 0.05, 0.0, False),
]

TUNE_GROUPS = [
    ('车速 / 限幅', ('max_vx', 'max_vy', 'max_wz', 'max_rpm')),
    ('位置环 PID', ('pos_kp', 'pos_ki', 'pos_kd')),
    ('转向环 PID', ('head_kp', 'head_ki', 'head_kd')),
    ('云台位置环 PID', ('gim_kp', 'gim_ki', 'gim_kd')),
]

# 云台标定：轴名、位置显示键
GIMBAL_AXES = (('Yaw', 'yaw'), ('Lift', 'lift'), ('Extend', 'ext'))


# ==================== ROS2 节点：负责和树莓派通信 ====================

class GuiRosNode(Node):
    """上位机自己的 ROS2 节点：发指令、收里程计/状态。"""

    def __init__(self):
        super().__init__('motion_gui')
        self.pub_load = self.create_publisher(String, '/motion_load', 10)
        self.pub_stop = self.create_publisher(Bool, '/motion_stop', 10)
        self.pub_tune = self.create_publisher(Float32MultiArray, '/cmd_tune', 10)
        self.pub_enable = self.create_publisher(Bool, '/cmd_enable', 10)
        # 云台标定：置零 / 限位 / 松轴（和树莓派状态屏共用同一套话题）
        self.pub_gimbal_zero = self.create_publisher(Int32, '/cmd_gimbal_zero', 10)
        self.pub_gimbal_limit = self.create_publisher(Float32MultiArray, '/cmd_gimbal_limit', 10)
        self.pub_gimbal_release = self.create_publisher(Bool, '/cmd_gimbal_release', 10)
        self.create_subscription(Odometry, '/odom', self._cb_odom, 10)
        self.create_subscription(String, '/motion_state', self._cb_state, 10)
        self.create_subscription(Int32MultiArray, '/robot_status', self._cb_status, 10)
        # PID 波形用的"目标值"：
        #   /cmd_pose   motion_planner 下发的目标位姿
        #   /cmd_gimbal 目标云台角度；/gimbal_state 是 F407 回传的实际角度
        self.create_subscription(Pose2D, '/cmd_pose', self._cb_pose_target, 10)
        self.create_subscription(Float32MultiArray, '/cmd_gimbal', self._cb_gimbal_target, 10)
        self.create_subscription(Float32MultiArray, '/gimbal_state', self._cb_gimbal_state, 10)
        self.create_subscription(Bool, '/f407_online', self._cb_online, 10)
        self.create_subscription(Float32MultiArray, '/tune_state', self._cb_tune_state, 10)
        # F407 当前生效的云台限位：[轴号, 是否有效, 下限, 上限]
        self.create_subscription(Float32MultiArray, '/gimbal_limits', self._cb_gimbal_limits, 10)
        # 里程计被视觉矫正重置（/set_odom 生效）时，f407_bridge 发来的坐标系平移量；
        # App 用它同步"车起点"锚点，保证场地↔里程计换算不变
        self.create_subscription(Pose2D, '/odom_shift', self._cb_odom_shift, 10)

        self.odom = None            # (x, y, theta) 米/米/弧度
        self.traj = []              # 历史轨迹 [(x,y), ...]（里程计系）
        self.motion_state = ''
        self.robot_status = None    # [state, err, mask]
        self.pose_target = None     # (x, y, theta) 米/米/弧度
        self.gimbal_target = None   # [yaw, lift, extend] 度
        self.gimbal_state = None    # [yaw, lift, extend] 度（相对 0 点）
        self.gimbal_limits = {}     # 轴号 -> (下限°, 上限°)
        self.gimbal_limits_valid = [False, False, False]
        self.online = False
        self.tune_ack = {}          # 参数编号 -> 407 回显的生效值
        self.odom_shift = None      # 待处理的里程计坐标系平移 (t_x, t_y, t_yaw)

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

    def _cb_pose_target(self, msg: Pose2D):
        self.pose_target = (float(msg.x), float(msg.y), float(msg.theta))

    def _cb_gimbal_target(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            self.gimbal_target = [float(v) for v in msg.data[:3]]

    def _cb_gimbal_state(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            self.gimbal_state = [float(v) for v in msg.data[:3]]

    def _cb_gimbal_limits(self, msg: Float32MultiArray):
        if len(msg.data) >= 4:
            axis = int(round(msg.data[0]))
            if 0 <= axis <= 2:
                self.gimbal_limits[axis] = (float(msg.data[2]), float(msg.data[3]))
                self.gimbal_limits_valid[axis] = bool(msg.data[1])

    def _cb_online(self, msg: Bool):
        self.online = bool(msg.data)

    def _cb_tune_state(self, msg: Float32MultiArray):
        if len(msg.data) >= 2:
            self.tune_ack[int(round(msg.data[0]))] = float(msg.data[1])

    def _cb_odom_shift(self, msg: Pose2D):
        """里程计被视觉矫正重置（calibrate 指令生效）：记下坐标系平移量，由
        App 在刷新循环里同步"车起点"锚点。可能连续来多次，只保留最新一次。"""
        self.odom_shift = (float(msg.x), float(msg.y), float(msg.theta))

    def send_sequence(self, commands: list) -> bool:
        """下发一段指令序列。返回 False = 没有节点在听 /motion_load（motion_planner 没起来）。"""
        if self.pub_load.get_subscription_count() == 0:
            return False
        # 执行前重新使能：底部【停止】会整车失能，不重新使能的话下发后车不动
        self.send_enable(True)
        s = String()
        s.data = json.dumps(commands, ensure_ascii=False)
        self.pub_load.publish(s)
        return True

    def send_stop(self):
        """■ 停止：状态机停在原地，同时整车失能（和手柄 B 键同一套效果）。"""
        b = Bool()
        b.data = True
        self.pub_stop.publish(b)
        self.send_enable(False)

    def send_enable(self, en: bool):
        self.pub_enable.publish(Bool(data=bool(en)))

    def send_gimbal_zero(self, mask: int = 0x07):
        """把云台三轴当前位置定义成 0 点（电机掉电不保存位置）。"""
        self.pub_gimbal_zero.publish(Int32(data=int(mask) & 0x07))

    def send_gimbal_limits(self, limits: dict):
        """limits: 轴号 -> (下限°, 上限°)。和状态屏示教结果走同一条通道。"""
        for axis in range(3):
            if axis not in limits:
                continue
            lo, hi = limits[axis]
            self.pub_gimbal_limit.publish(
                Float32MultiArray(data=[float(axis), float(lo), float(hi)]))

    def send_gimbal_release(self, en: bool):
        self.pub_gimbal_release.publish(Bool(data=bool(en)))

    def send_tune(self, param_id: int, value: float) -> bool:
        """发一项调参。motion 链路没起来时也不要报错，只是没人转发。"""
        m = Float32MultiArray()
        m.data = [float(param_id), float(value)]
        self.pub_tune.publish(m)
        return True


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

        # 车体是 300x300 的正方形（field.json: robot.size_mm）。
        # 只用于画图，按真实尺寸画才能一眼看出会不会蹭墙/蹭黄块。
        self.robot_size = float(self.cfg.get("robot", {}).get("size_mm", 300.0))

        # ---------- 雷达 / 规划状态 ----------
        self.raw_points = []        # (设备角, 距离, 强度)
        self.points = []            # 世界坐标 [(x, y, 强度)]
        self.obstacles = []
        self.blocked_cells = set()  # 整格不能通车的 5×5 格
        self.plan = None            # 最近一次单点规划
        self.mission_legs = []
        self.mission_total = 0.0
        self.goal_world = None
        self._last_plan_kind = None   # 'single' = 单点规划，'mission' = 九点全程
        # 模拟执行用：没有真车时让车在图上沿着规划路径一步步走
        self.sim_pose = None        # (x, y, yaw_deg) 场地系；None = 不模拟
        self.play_pts = None        # 待播放的位姿序列
        self.play_i = 0
        self.play_after = None      # tk.after 的句柄，用来取消
        self.lidar = None
        self.lidar_q = queue.Queue()
        self.scanning = False
        self.scan_revs_done = 0
        self.scan_target_revs = int(self.cfg["scan"]["revolutions"])
        self._last_hover = None
        self._last_odom = None
        self._need_redraw = True

        self.commands = []          # 动作序列编辑器里的指令

        # ---------- 九节点动作序列 ----------
        self.nodes = self._load_nodes()
        self.node_sel = 0
        self._nodes_dirty = False
        if not os.path.exists(NODES_PATH):
            self._save_nodes()      # 第一次运行生成 nodes.json，方便直接改文件

        # ---------- PID 调参 ----------
        # 滑杆值镜像。407 上电默认值必须和 TUNE_PARAMS 的默认值一致；
        # 收到 /f407_online 由 False→True 时自动把全部值重发一遍，
        # 防止 407 重启后参数悄悄回到默认。
        self.tune = {key: float(default) for (_, key, _, _, _, _, default, _) in TUNE_PARAMS}
        self._tune_ids = {key: pid for (pid, key, *_rest) in TUNE_PARAMS}
        self._tune_scales = {}
        self._tune_pending = {}
        self._tune_after = None
        self._tune_was_online = False
        self.tune_status = tk.StringVar(value='调参：等待 /f407_online…')

        # ---------- 波形历史（10Hz 采样，各存 60 秒） ----------
        self.wave_pos = deque(maxlen=600)      # (目标x, 目标y, 实际x, 实际y)
        self.wave_yaw = deque(maxlen=600)      # (目标yaw°, 实际yaw°)
        self.wave_gimbal = deque(maxlen=600)   # (目标y,l,e, 实际y,l,e)
        self.wave_mode = tk.StringVar(value='pos')
        self.wave_axis = tk.IntVar(value=0)

        root.title('智能搬运上位机 — 动作序列 / 雷达地图 / 九节点 / PID 调参')
        root.geometry('1500x940')
        self._build_ui()
        self._refresh()

    # ==================== 界面 ====================

    def _build_ui(self):
        self.status = tk.StringVar(
            value='就绪。没雷达就勾【模拟雷达】点【连接并开始】；'
                  '底部【▶ 执行】按当前页签执行，【■ 停止】随时停车。')
        ttk.Label(self.root, textvariable=self.status, relief=tk.SUNKEN,
                  anchor='w', padding=4).pack(side=tk.BOTTOM, fill=tk.X)

        # ---------- 底部：只留 执行 / 停止 ----------
        bottom = ttk.Frame(self.root, padding=(8, 5))
        bottom.pack(side=tk.BOTTOM, fill=tk.X)
        ttk.Button(bottom, text='▶ 执行', width=12,
                   command=self.execute_current).pack(side=tk.LEFT, padx=(4, 8))
        ttk.Button(bottom, text='■ 停止', width=12,
                   command=self.node.send_stop).pack(side=tk.LEFT)
        self.exec_hint = tk.StringVar(value='执行：动作序列（当前页签）')
        ttk.Label(bottom, textvariable=self.exec_hint,
                  foreground='#555').pack(side=tk.LEFT, padx=16)

        body = ttk.Frame(self.root)
        body.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        left = ttk.Frame(body, padding=6)
        left.pack(side=tk.LEFT, fill=tk.Y)
        self.nb = ttk.Notebook(left)
        self.nb.pack(fill=tk.BOTH, expand=True)
        self._build_seq_tab()
        self._build_map_tab_controls()
        self._build_nodes_tab()
        self._build_tune_tab()
        self.nb.bind('<<NotebookTabChanged>>', lambda e: self._on_tab_changed())

        # ---------- 右侧：地图画面 / 调参波形，按页签切换 ----------
        self.right = ttk.Frame(body)
        self.right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.map_frame = ttk.Frame(self.right)
        self.wave_frame = ttk.Frame(self.right)

        self.pose_var = tk.StringVar(value='当前位姿: 未收到 /odom')
        self.state_var = tk.StringVar(value='状态机: —')
        self.plan_var = tk.StringVar(value='')
        ttk.Label(self.map_frame, textvariable=self.plan_var, anchor='w',
                  foreground=C_PATH, wraplength=1000).pack(side=tk.BOTTOM, fill=tk.X, padx=8)
        ttk.Label(self.map_frame, textvariable=self.state_var, anchor='w',
                  foreground='#444').pack(side=tk.BOTTOM, fill=tk.X, padx=8)
        ttk.Label(self.map_frame, textvariable=self.pose_var, anchor='w',
                  foreground='#222').pack(side=tk.BOTTOM, fill=tk.X, padx=8)

        self.canvas = tk.Canvas(self.map_frame, bg=C_BG, highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=8, pady=(8, 2))
        self.canvas.bind('<Button-1>', self.on_click)
        self.canvas.bind('<Motion>', self.on_motion)
        self.canvas.bind('<Leave>', lambda e: self._clear_hover())
        self.canvas.bind('<Configure>', lambda e: self._redraw())

        self._build_wave_panel()
        self.map_frame.pack(fill=tk.BOTH, expand=True)

    # ---------- 页签 1：动作序列 ----------

    def _build_seq_tab(self):
        tab = ttk.Frame(self.nb, padding=6)
        self.nb.add(tab, text='动作序列')

        ttk.Label(tab, text='动作序列（从上到下依次执行）',
                  font=('', 11, 'bold')).pack(anchor='w')
        self.listbox = tk.Listbox(tab, width=44, height=16)
        self.listbox.pack(fill=tk.BOTH, expand=True, pady=4)

        # 编辑器和【节点动作序列】共用同一套控件生成逻辑，只是目标列表不同
        self.seq_editor = self._make_action_editor(
            tab, self.listbox, lambda: self.commands, lambda: None, width=42)

        iorow = ttk.Frame(tab)
        iorow.pack(fill=tk.X, pady=2)
        ttk.Button(iorow, text='导入JSON', command=self.import_json).pack(side=tk.LEFT, padx=2)
        ttk.Button(iorow, text='导出JSON', command=self.export_json).pack(side=tk.LEFT, padx=2)

    # ---------- 通用动作编辑器（动作序列页 / 节点动作序列页共用） ----------

    def _make_action_editor(self, parent, listbox, get_cmds, on_change, width=44):
        """生成"类型下拉 + 参数输入 + 添加/上移/下移/删除/清空"这一套控件。

        get_cmds() 返回当前页签正在编辑的 list（直接原地修改）；
        on_change() 在内容变化后调用（节点页用它自动存盘）。
        """
        addrow = ttk.Frame(parent)
        addrow.pack(fill=tk.X, pady=2)
        entry = ttk.Entry(addrow)
        hint = ttk.Label(parent, text='', foreground='gray', wraplength=width * 7)

        def on_type(name):
            hint.config(text=f"参数：{COMMAND_TYPES[name]['params']}  —  "
                             f"{COMMAND_TYPES[name]['hint']}")

        cmd_type = tk.StringVar(value='move')
        menu = ttk.OptionMenu(addrow, cmd_type, 'move', *COMMAND_TYPES.keys(),
                              command=on_type)
        menu.configure(width=9)
        menu.pack(side=tk.LEFT)
        entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        hint.pack(anchor='w')
        on_type('move')

        def add():
            try:
                cmd = build_cmd(cmd_type.get(), entry.get())
            except Exception as e:  # noqa: BLE001
                messagebox.showerror('参数错误', f'参数解析失败：{e}')
                return
            cmds = get_cmds()
            cmds.append(cmd)
            listbox.insert(tk.END, cmd_to_text(cmd))
            entry.delete(0, tk.END)
            on_change()

        editrow = ttk.Frame(parent)
        editrow.pack(fill=tk.X, pady=3)

        def move(direction):
            sel = listbox.curselection()
            if not sel:
                return
            i = sel[0]
            j = i + direction
            cmds = get_cmds()
            if not (0 <= j < len(cmds)):
                return
            cmds[i], cmds[j] = cmds[j], cmds[i]
            self._reload_actions(listbox, cmds)
            listbox.selection_set(j)
            on_change()

        def delete():
            sel = listbox.curselection()
            if not sel:
                return
            i = sel[0]
            del get_cmds()[i]
            self._reload_actions(listbox, get_cmds())
            on_change()

        def clear():
            get_cmds().clear()
            self._reload_actions(listbox, get_cmds())
            on_change()

        ttk.Button(addrow, text='＋添加', command=add).pack(side=tk.LEFT)
        ttk.Button(editrow, text='上移', command=lambda: move(-1)).pack(side=tk.LEFT, padx=2)
        ttk.Button(editrow, text='下移', command=lambda: move(+1)).pack(side=tk.LEFT, padx=2)
        ttk.Button(editrow, text='删除', command=delete).pack(side=tk.LEFT, padx=2)
        ttk.Button(editrow, text='清空', command=clear).pack(side=tk.LEFT, padx=2)
        return {'type': cmd_type, 'entry': entry, 'hint': hint}

    @staticmethod
    def _reload_actions(listbox, cmds):
        listbox.delete(0, tk.END)
        for c in cmds:
            listbox.insert(tk.END, cmd_to_text(c))

    # ---------- 页签 3：九节点动作序列 ----------

    def _default_nodes(self):
        """nodes.json 不存在时，按 field.json 的 mission.route 生成默认 9 个节点。"""
        route = list(self.cfg.get('mission', {}).get('route') or [])
        if len(route) < 2:
            route = ['起点', '扫码区', '原料区', '粗加工区', '暂存区',
                     '原料区', '粗加工区', '暂存区', '起点']
        return [{'name': f'{i + 1}. {n}', 'route': n, 'actions': []}
                for i, n in enumerate(route)]

    def _load_nodes(self):
        try:
            with open(NODES_PATH, 'r', encoding='utf-8') as f:
                data = json.load(f)
            nodes = data.get('nodes') if isinstance(data, dict) else data
            if not isinstance(nodes, list) or not nodes:
                raise ValueError('nodes 为空')
            for i, n in enumerate(nodes):
                if not isinstance(n, dict):
                    raise ValueError('节点格式错误')
                n.setdefault('name', f'{i + 1}. {n.get("route", "?")}')
                n.setdefault('route', '')
                acts = n.get('actions')
                n['actions'] = acts if isinstance(acts, list) else []
            return nodes
        except FileNotFoundError:
            return self._default_nodes()
        except Exception as e:  # noqa: BLE001
            if hasattr(self, 'status'):
                self._set_status(f'nodes.json 读取失败（{e}），用默认九点')
            return self._default_nodes()

    def _save_nodes(self):
        try:
            with open(NODES_PATH, 'w', encoding='utf-8') as f:
                json.dump({'nodes': self.nodes}, f, ensure_ascii=False, indent=2)
            self._nodes_dirty = False
        except Exception as e:  # noqa: BLE001
            if hasattr(self, 'status'):
                self._set_status(f'节点序列保存失败：{e}')

    def _build_nodes_tab(self):
        tab = ttk.Frame(self.nb, padding=6)
        self.nb.add(tab, text='节点动作序列')

        ttk.Label(tab, text='九节点动作序列（按 mission.route 顺序走）',
                  font=('', 11, 'bold')).pack(anchor='w')
        ttk.Label(tab, foreground='#666',
                  text='选中一个节点，在下面编辑到达该点后要执行的动作。').pack(anchor='w')

        self.node_list = tk.Listbox(tab, width=44, height=8, exportselection=False)
        self.node_list.pack(fill=tk.X, pady=(4, 2))
        self.node_list.bind('<<ListboxSelect>>', self._on_node_select)

        self.node_info = tk.StringVar(value='')
        ttk.Label(tab, textvariable=self.node_info, foreground=C_PATH,
                  wraplength=340).pack(anchor='w')

        ttk.Label(tab, text='该节点的动作（从上到下）：',
                  font=('', 10, 'bold')).pack(anchor='w', pady=(4, 0))
        self.node_action_list = tk.Listbox(tab, width=44, height=9)
        self.node_action_list.pack(fill=tk.BOTH, expand=True, pady=(2, 2))

        self.node_editor = self._make_action_editor(
            tab, self.node_action_list, self._node_actions, self._node_changed, width=42)

        row = ttk.Frame(tab)
        row.pack(fill=tk.X, pady=(3, 0))
        ttk.Button(row, text='保存', command=lambda: self._save_nodes()).pack(side=tk.LEFT, padx=2)
        ttk.Button(row, text='导入', command=self._import_nodes).pack(side=tk.LEFT, padx=2)
        ttk.Button(row, text='导出', command=self._export_nodes).pack(side=tk.LEFT, padx=2)
        ttk.Button(row, text='用当前序列填充', command=self._copy_seq_to_node).pack(
            side=tk.LEFT, padx=2)
        ttk.Button(row, text='预览完整序列', command=self._preview_mission_sequence).pack(
            side=tk.LEFT, padx=2)

        self._reload_node_list()
        if self.nodes:
            self.node_list.selection_set(0)
            self._on_node_select()

    def _reload_node_list(self):
        self.node_list.delete(0, tk.END)
        for i, n in enumerate(self.nodes):
            self.node_list.insert(tk.END, n.get('name') or f'{i + 1}. {n.get("route", "?")}')

    def _node_actions(self):
        if not self.nodes:
            return []
        self.node_sel = max(0, min(self.node_sel, len(self.nodes) - 1))
        return self.nodes[self.node_sel]['actions']

    def _node_changed(self):
        self._nodes_dirty = True
        self._save_nodes()
        self._update_node_info()

    def _on_node_select(self, _event=None):
        sel = self.node_list.curselection()
        if not sel:
            return
        self.node_sel = sel[0]
        self._reload_actions(self.node_action_list, self._node_actions())
        self._update_node_info()

    def _update_node_info(self):
        if not self.nodes:
            self.node_info.set('还没有节点')
            return
        n = self.nodes[self.node_sel]
        route = n.get('route', '')
        pt = None
        try:
            pt = self.field.resolve_waypoint(route)
        except Exception:  # noqa: BLE001
            pt = None
        where = f'场地 ({pt[0]:.0f}, {pt[1]:.0f})mm' if pt else '路线里解析不到坐标'
        self.node_info.set(
            f'第 {self.node_sel + 1}/{len(self.nodes)} 点 · 路线点「{route}」· {where} · '
            f'{len(n.get("actions", []))} 条动作')

    def _copy_seq_to_node(self):
        if not self.nodes:
            return
        if not messagebox.askyesno('覆盖节点动作',
                                   f'把【动作序列】页的 {len(self.commands)} 条复制到'
                                   f'第 {self.node_sel + 1} 点？会覆盖该节点现有动作。'):
            return
        self.nodes[self.node_sel]['actions'] = [dict(c) for c in self.commands]
        self._reload_actions(self.node_action_list, self._node_actions())
        self._node_changed()

    def _import_nodes(self):
        path = filedialog.askopenfilename(filetypes=[('JSON 文件', '*.json')])
        if not path:
            return
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            nodes = data.get('nodes') if isinstance(data, dict) else data
            if not isinstance(nodes, list) or not nodes:
                raise ValueError('文件里没有节点')
            for i, n in enumerate(nodes):
                n.setdefault('name', f'{i + 1}. {n.get("route", "?")}')
                n.setdefault('route', '')
                n['actions'] = n.get('actions') if isinstance(n.get('actions'), list) else []
            self.nodes = nodes
            self.node_sel = 0
            self._reload_node_list()
            self.node_list.selection_set(0)
            self._on_node_select()
            self._save_nodes()
            self._set_status(f'已导入 {len(nodes)} 个节点')
        except Exception as e:  # noqa: BLE001
            messagebox.showerror('导入失败', str(e))

    def _export_nodes(self):
        path = filedialog.asksaveasfilename(defaultextension='.json',
                                            filetypes=[('JSON 文件', '*.json')])
        if not path:
            return
        try:
            with open(path, 'w', encoding='utf-8') as f:
                json.dump({'nodes': self.nodes}, f, ensure_ascii=False, indent=2)
            messagebox.showinfo('导出成功', f'已保存到\n{path}')
        except Exception as e:  # noqa: BLE001
            messagebox.showerror('导出失败', str(e))

    # ---------- 页签 4：PID 调参 ----------

    def _build_tune_tab(self):
        tab = self._scrollable_tab('PID 调参')

        ttk.Label(tab, text='整车参数在线调整（拖动即下发，不用烧录）',
                  font=('', 11, 'bold')).pack(anchor='w')
        ttk.Label(tab, foreground='#666', wraplength=310,
                  text='波形显示在右边，可切换位置环 / 转向环 / 云台环。').pack(anchor='w')

        self._tune_applying = True
        for title, keys in TUNE_GROUPS:
            box = ttk.LabelFrame(tab, text=title, padding=3)
            box.pack(fill=tk.X, pady=3)
            for key in keys:
                spec = next(s for s in TUNE_PARAMS if s[1] == key)
                pid, _k, label, vmin, vmax, step, default, is_int = spec
                row = ttk.Frame(box)
                row.pack(fill=tk.X)
                var = tk.StringVar(value=f'{default:g}')
                scale = tk.Scale(
                    row, from_=vmin, to=vmax, resolution=step, orient=tk.HORIZONTAL,
                    length=240, showvalue=False, label=label, bd=0,
                    highlightthickness=0,
                    command=lambda v, k=key: self._on_tune_slide(k, v))
                scale.set(default)
                scale.pack(side=tk.LEFT, fill=tk.X, expand=True)
                ttk.Label(row, textvariable=var, width=7, anchor='e').pack(
                    side=tk.LEFT, padx=(2, 0))
                self._tune_scales[key] = (scale, var, is_int)
        self._tune_applying = False

        row = ttk.Frame(tab)
        row.pack(fill=tk.X, pady=(4, 0))
        ttk.Button(row, text='全部下发', command=self._send_all_tune).pack(side=tk.LEFT, padx=2)
        ttk.Button(row, text='恢复默认', command=self._restore_tune_defaults).pack(
            side=tk.LEFT, padx=2)

        ttk.Label(tab, textvariable=self.tune_status, foreground='#7a4fbf',
                  wraplength=310).pack(anchor='w', pady=(3, 0))

        # ---------- 云台零点 / 限位 / 松轴 ----------
        cal = ttk.LabelFrame(tab, text='云台零点 / 限位 / 松轴', padding=4)
        cal.pack(fill=tk.X, pady=(6, 3))
        ttk.Label(cal, foreground='#666', wraplength=310,
                  text='上电后把三个轴摆到固定位置 → 点【设为零点】；'
                       '限位相对零点生效，树莓派状态屏示教过的限位会自动同步过来。'
                  ).pack(anchor='w')

        self.gimbal_now_var = tk.StringVar(value='当前：等待 /gimbal_state…')
        ttk.Label(cal, textvariable=self.gimbal_now_var, font=('Consolas', 9),
                  wraplength=310).pack(anchor='w', pady=(2, 3))

        self.gimbal_limit_vars = {}
        self._limits_user_edited = False
        self._limits_pending = None      # 已下发、等 407 回传确认的限位
        for axis, (name, _key) in enumerate(GIMBAL_AXES):
            row = ttk.Frame(cal)
            row.pack(fill=tk.X)
            ttk.Label(row, text=name, width=7).pack(side=tk.LEFT)
            lo_var = tk.StringVar()
            hi_var = tk.StringVar()
            ttk.Label(row, text='min').pack(side=tk.LEFT)
            e_lo = ttk.Entry(row, textvariable=lo_var, width=7)
            e_lo.pack(side=tk.LEFT, padx=(2, 6))
            ttk.Label(row, text='max').pack(side=tk.LEFT)
            e_hi = ttk.Entry(row, textvariable=hi_var, width=7)
            e_hi.pack(side=tk.LEFT)
            for e in (e_lo, e_hi):
                e.bind('<KeyRelease>', lambda _e: self._limits_edited())
            self.gimbal_limit_vars[axis] = (lo_var, hi_var)

        row = ttk.Frame(cal)
        row.pack(fill=tk.X, pady=(4, 2))
        ttk.Button(row, text='设为零点', command=self._gimbal_set_zero).pack(side=tk.LEFT)
        ttk.Button(row, text='下发限位', command=self._send_gimbal_limits).pack(
            side=tk.LEFT, padx=4)
        self.gimbal_release_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(row, text='松轴（可手拨）', variable=self.gimbal_release_var,
                        command=self._toggle_gimbal_release).pack(side=tk.LEFT, padx=(6, 0))
        self.gimbal_cal_status = tk.StringVar(value='')
        ttk.Label(cal, textvariable=self.gimbal_cal_status, foreground='#2e7d32',
                  wraplength=310).pack(anchor='w')

        wf = ttk.LabelFrame(tab, text='右边波形', padding=3)
        wf.pack(fill=tk.X, pady=(4, 0))
        for text, value in (('位置环', 'pos'), ('转向环', 'yaw'),
                            ('云台环', 'gimbal'), ('XY轨迹', 'xy')):
            ttk.Radiobutton(wf, text=text, value=value, variable=self.wave_mode,
                            command=self._redraw_wave).pack(side=tk.LEFT, padx=2)
        self.cb_wave_axis = ttk.Combobox(wf, width=7, state='readonly',
                                         values=['yaw', 'lift', 'extend'])
        self.cb_wave_axis.current(0)
        self.cb_wave_axis.pack(side=tk.LEFT, padx=(6, 0))
        self.cb_wave_axis.bind('<<ComboboxSelected>>',
                               lambda e: self._set_wave_axis())

        self._bind_wheel(tab)

    def _limits_edited(self):
        self._limits_user_edited = True

    def _sync_gimbal_limits(self):
        """把 407 回传的限位和当前位置刷到界面上（用户正在改限位时不覆盖输入框）。"""
        gs = self.node.gimbal_state
        if gs is not None:
            text = '当前：' + '  '.join(
                f'{name} {gs[i]:+.1f}°' for i, (name, _k) in enumerate(GIMBAL_AXES))
            if self.node.gimbal_limits and not all(self.node.gimbal_limits_valid):
                text += '   [有些轴还没示教，限位显示固件默认]'
            self.gimbal_now_var.set(text)
        else:
            self.gimbal_now_var.set('当前：等待 /gimbal_state…')

        # 刚下发的限位要等 407 回传确认，期间保留输入框里的值，
        # 否则会在"旧回传值 ← → 新输入值"之间来回跳 0.5s。
        if self._limits_pending is not None:
            if self._limits_confirmed():
                self._limits_pending = None
                self._limits_user_edited = False
                self.gimbal_cal_status.set('407 已确认限位')
            else:
                return

        if self._limits_user_edited:
            return
        for axis in range(len(GIMBAL_AXES)):
            if axis not in self.node.gimbal_limits:
                continue
            lo, hi = self.node.gimbal_limits[axis]
            lo_var, hi_var = self.gimbal_limit_vars[axis]
            lo_var.set(f'{lo:g}')
            hi_var.set(f'{hi:g}')

    def _limits_confirmed(self):
        """407 回传的限位是否已经等于刚下发的值（按 0.1° 网格比较）。"""
        for axis, (lo, hi) in self._limits_pending.items():
            got = self.node.gimbal_limits.get(axis)
            if got is None:
                return False
            # F407 收到的是 int16 ×0.1°（向零截断），按同一网格比才算"确认"
            want_lo = int(lo * 10.0) / 10.0
            want_hi = int(hi * 10.0) / 10.0
            if abs(got[0] - want_lo) > 0.049 or abs(got[1] - want_hi) > 0.049:
                return False
        return True

    def _parse_gimbal_limits(self):
        limits = {}
        try:
            for axis in range(len(GIMBAL_AXES)):
                lo_var, hi_var = self.gimbal_limit_vars[axis]
                lo = float(lo_var.get())
                hi = float(hi_var.get())
                if lo > hi:
                    lo, hi = hi, lo
                limits[axis] = (lo, hi)
        except (TypeError, ValueError):
            self.gimbal_cal_status.set(
                '限位还不是数字：等 407 回传一次，或手动填 min / max')
            return None
        return limits

    def _send_gimbal_limits(self):
        limits = self._parse_gimbal_limits()
        if limits is None:
            return
        self.node.send_gimbal_limits(limits)
        self._limits_pending = dict(limits)
        self._limits_user_edited = True     # 等回传确认前不要用旧值覆盖输入框
        self.gimbal_cal_status.set('限位已下发，等 407 回传确认…')

    def _gimbal_set_zero(self):
        """把三轴此刻的位置定义成 0 点，然后把当前限位重新下发。"""
        self.node.send_gimbal_zero(0x07)
        self._limits_user_edited = False
        self.gimbal_cal_status.set('已设 0 点，正在重新下发限位…')
        self.root.after(300, self._send_gimbal_limits)

    def _toggle_gimbal_release(self):
        en = bool(self.gimbal_release_var.get())
        if en:
            # 松轴前先整车失能：手拨的时候底盘/其它轴不会突然动
            self.node.send_enable(False)
        self.node.send_gimbal_release(en)
        self.gimbal_cal_status.set(
            '已松轴：可以手拨电机，上面的当前角度会实时变化'
            if en else '已锁轴')

    def _set_wave_axis(self):
        try:
            self.wave_axis.set(max(0, self.cb_wave_axis.current()))
        except Exception:  # noqa: BLE001
            self.wave_axis.set(0)
        self._redraw_wave()

    def _on_tune_slide(self, key, value):
        try:
            fv = float(value)
        except (TypeError, ValueError):
            return
        entry = self._tune_scales.get(key)
        if entry:
            scale, var, is_int = entry
            var.set(f'{fv:.0f}' if is_int else f'{fv:g}')
        self.tune[key] = fv
        if getattr(self, '_tune_applying', False):
            return
        self._tune_pending[key] = fv
        if self._tune_after is None:
            self._tune_after = self.root.after(60, self._flush_tune)

    def _flush_tune(self):
        self._tune_after = None
        pending, self._tune_pending = self._tune_pending, {}
        for key, value in pending.items():
            self.node.send_tune(self._tune_ids[key], value)
        if pending:
            self.tune_status.set(f'已下发 {len(pending)} 项参数，等待 407 回显…')

    def _send_all_tune(self):
        for key, value in self.tune.items():
            self.node.send_tune(self._tune_ids[key], value)
        self.tune_status.set('已下发全部参数，等待 407 回显…')

    def _restore_tune_defaults(self):
        self._tune_applying = True
        try:
            for (pid, key, _label, _lo, _hi, _step, default, is_int) in TUNE_PARAMS:
                self.tune[key] = float(default)
                scale, var, _ = self._tune_scales[key]
                scale.set(default)
                var.set(f'{default:g}')
        finally:
            self._tune_applying = False
        self._send_all_tune()

    def _update_tune_ack(self):
        acks = self.node.tune_ack
        if not acks:
            self.tune_status.set('调参：还没有收到 407 回显（检查串口 / f407_bridge）')
            return
        names = {pid: key for pid, key, *_ in TUNE_PARAMS}
        if self.node.online:
            self.tune_status.set('407 已确认：' + '  '.join(
                f'{names.get(pid, hex(pid))}={val:g}' for pid, val in
                sorted(acks.items())))
        else:
            self.tune_status.set('调参：407 离线（参数值保留在界面上）')

    # ---------- 右侧波形面板 ----------

    def _build_wave_panel(self):
        top = ttk.Frame(self.wave_frame, padding=(8, 4))
        top.pack(fill=tk.X)
        ttk.Label(top, text='波形：', font=('', 10, 'bold')).pack(side=tk.LEFT)
        ttk.Label(top, foreground='#666',
                  text='虚线=目标值（/cmd_pose、/cmd_gimbal）  实线=实际值（/odom、/gimbal_state）'
                  ).pack(side=tk.LEFT)
        self.wave_info = tk.StringVar(value='等待数据…')
        ttk.Label(self.wave_frame, textvariable=self.wave_info, anchor='w',
                  foreground='#444').pack(side=tk.BOTTOM, fill=tk.X, padx=8, pady=(0, 4))
        self.wave_canvas = tk.Canvas(self.wave_frame, bg=C_BG, highlightthickness=0)
        self.wave_canvas.pack(fill=tk.BOTH, expand=True, padx=8, pady=(2, 2))
        self.wave_canvas.bind('<Configure>', lambda e: self._redraw_wave())

    # ---------- 波形采样 / 绘制 ----------

    def _sample_wave(self):
        """10Hz 采样：目标来自 planner 下发的话题，实际来自 F407 回传。"""
        o = self.node.odom
        t = self.node.pose_target
        if o is not None and t is not None:
            self.wave_pos.append((t[0], t[1], o[0], o[1]))
            # 航向角显示成度；目标/实际可能跨 ±180°，这里各自取原值，
            # 波形上出现"跳一圈"说明确实绕过了 ±180°，不是画错。
            self.wave_yaw.append((math.degrees(t[2]), math.degrees(o[2])))
        gt = self.node.gimbal_target
        gs = self.node.gimbal_state
        if gt is not None and gs is not None:
            self.wave_gimbal.append(tuple(gt) + tuple(gs))
        self._update_wave_info()

    def _update_wave_info(self):
        mode = self.wave_mode.get()
        if mode == 'pos':
            if self.wave_pos:
                tx, ty, ax, ay = self.wave_pos[-1]
                self.wave_info.set(
                    f'位置环   目标 ({tx:+.3f}, {ty:+.3f}) m   实际 ({ax:+.3f}, {ay:+.3f}) m'
                    f'   误差 {math.hypot(tx - ax, ty - ay) * 1000:.0f} mm')
            else:
                self.wave_info.set('位置环：等待 /cmd_pose 与 /odom…')
        elif mode == 'yaw':
            if self.wave_yaw:
                ty, ay = self.wave_yaw[-1]
                self.wave_info.set(
                    f'转向环   目标 {ty:+.1f}°   实际 {ay:+.1f}°   '
                    f'误差 {wrap_deg(ty - ay):+.1f}°')
            else:
                self.wave_info.set('转向环：等待 /cmd_pose 与 /odom…')
        elif mode == 'xy':
            if self.wave_pos:
                tx, ty, ax, ay = self.wave_pos[-1]
                self.wave_info.set(
                    f'XY 轨迹   目标 ({tx:+.3f}, {ty:+.3f}) m   '
                    f'实际 ({ax:+.3f}, {ay:+.3f}) m   '
                    f'误差 {math.hypot(tx - ax, ty - ay) * 1000:.0f} mm   '
                    f'窗口 {len(self.wave_pos) / 10:.0f}s')
            else:
                self.wave_info.set('XY 轨迹：等待 /cmd_pose 与 /odom…')
        else:
            axis = self.wave_axis.get()
            names = ('yaw', 'lift', 'extend')
            if self.wave_gimbal:
                s = self.wave_gimbal[-1]
                self.wave_info.set(
                    f'云台环 {names[axis]}   目标 {s[axis]:+.1f}°   '
                    f'实际 {s[3 + axis]:+.1f}°   误差 {s[axis] - s[3 + axis]:+.1f}°')
            else:
                self.wave_info.set('云台环：等待 /cmd_gimbal 与 /gimbal_state…')

    def _redraw_wave(self):
        c = getattr(self, 'wave_canvas', None)
        if c is None or not c.winfo_ismapped():
            return
        c.delete('all')
        w, h = c.winfo_width(), c.winfo_height()
        if w < 40 or h < 40:
            return
        mode = self.wave_mode.get()
        if mode == 'xy':
            self._draw_wave_xy(c, w, h)
            self._update_wave_info()
            return
        if mode == 'pos':
            panels = [
                ('X 位置 (m)', self.wave_pos, 0, 2, '#1f6fd6'),
                ('Y 位置 (m)', self.wave_pos, 1, 3, '#d62728'),
            ]
        elif mode == 'yaw':
            panels = [('航向 (deg)', self.wave_yaw, 0, 1, '#8e24aa')]
        else:
            axis = self.wave_axis.get()
            names = ('yaw', 'lift', 'extend')
            panels = [(f'云台 {names[axis]} (deg)', self.wave_gimbal,
                       axis, 3 + axis, '#008c50')]
        self._draw_wave_panels(c, w, h, panels)
        self._update_wave_info()

    def _draw_wave_xy(self, c, w, h):
        """XY 轨迹视图：目标与实际在里程计平面上的轨迹（x/y 等比例，不拉伸）。

        实线=实际轨迹，灰色虚线=目标轨迹（阶梯形状是目标逐步下发的自然结果），
        黑十字=当前目标点，红圈=当前实际位置。
        """
        if not self.wave_pos:
            c.create_text(w / 2, h / 2, text='等待数据…', fill='#aaa')
            return
        # 里程计系 → 场地朝向（只旋转：odom = R(-dq)·field，所以显示时用 R(+dq)
        # 转回来；平移差无关紧要，视图自动取包围盒）。这样轨迹图和左边地图页
        # 永远同向；当前校准 dq=180° 时就是整个图像旋转 180°。
        dq = 0.0
        if self.odom_start[2] is not None:
            dq = math.radians(self.robot_start[2] - self.odom_start[2])
        cd, sd = math.cos(dq), math.sin(dq)
        pts = [(cd * p[0] - sd * p[1], sd * p[0] + cd * p[1],
                cd * p[2] - sd * p[3], sd * p[2] + cd * p[3])
               for p in self.wave_pos]
        left, right, top, bottom = 54, 10, 16, 22
        xs, ys = [], []
        for t in pts:
            xs.extend((t[0], t[2]))     # 目标x、实际x
            ys.extend((t[1], t[3]))     # 目标y、实际y
        x0, x1 = min(xs), max(xs)
        y0, y1 = min(ys), max(ys)
        if x1 - x0 < 0.2:
            x0 -= 0.5
            x1 += 0.5
        if y1 - y0 < 0.2:
            y0 -= 0.5
            y1 += 0.5
        x0 -= (x1 - x0) * 0.1
        x1 += (x1 - x0) * 0.1
        y0 -= (y1 - y0) * 0.1
        y1 += (y1 - y0) * 0.1
        # x/y 用同一个比例尺，弧线/直线才是真实形状（不等比例会掩盖跑偏）
        scale = min((w - left - right) / (x1 - x0), (h - top - bottom) / (y1 - y0))
        ox = left + (w - left - right - (x1 - x0) * scale) / 2.0
        oy = top + (h - top - bottom - (y1 - y0) * scale) / 2.0

        def px(x):
            return ox + (x - x0) * scale

        def py(y):
            return oy + (y1 - y) * scale

        c.create_rectangle(px(x0), py(y1), px(x1), py(y0), outline='#dcdcdc')
        c.create_text(left + 4, top - 4, anchor='w', fill='#444',
                      font=('', 9, 'bold'),
                      text='XY 轨迹 (m，场地朝向)   实线=实际   虚线=目标')
        for k in range(5):
            gx = x0 + (x1 - x0) * k / 4.0
            gy = y0 + (y1 - y0) * k / 4.0
            c.create_line(px(gx), py(y0), px(gx), py(y1), fill='#f0f0f0')
            c.create_line(px(x0), py(gy), px(x1), py(gy), fill='#f0f0f0')
            c.create_text(px(gx), py(y0) + 12, fill='#999',
                          font=('Consolas', 8), text=f'{gx:.2f}')
            c.create_text(px(x0) - 5, py(gy), anchor='e', fill='#999',
                          font=('Consolas', 8), text=f'{gy:.2f}')

        tgt = [v for p in pts for v in (px(p[0]), py(p[1]))]
        act = [v for p in pts for v in (px(p[2]), py(p[3]))]
        if len(tgt) >= 4:
            c.create_line(*tgt, fill='#999999', width=1, dash=(4, 3))
        if len(act) >= 4:
            c.create_line(*act, fill='#1f6fd6', width=2)

        lx, ly = px(pts[-1][0]), py(pts[-1][1])
        c.create_line(lx - 7, ly, lx + 7, ly, fill='#333333', width=2)
        c.create_line(lx, ly - 7, lx, ly + 7, fill='#333333', width=2)
        c.create_text(lx + 10, ly - 10, anchor='w', fill='#333', font=('', 8),
                      text='目标')
        ax_, ay_ = px(pts[-1][2]), py(pts[-1][3])
        c.create_oval(ax_ - 4, ay_ - 4, ax_ + 4, ay_ + 4, outline='#d62728', width=2)
        c.create_text(ax_ + 10, ay_ + 6, anchor='w', fill='#d62728', font=('', 8),
                      text='实际')
        # 0.5m 比例尺
        bar = 0.5 * scale
        bx, by = px(x1) - bar - 10, py(y0) - 10
        c.create_line(bx, by, bx + bar, by, fill='#555555', width=2)
        c.create_text(bx + bar / 2, by - 8, fill='#555', font=('', 8), text='0.5m')

    def _draw_wave_panels(self, c, w, h, panels):
        """每个 panel = (标题, 序列, 目标下标, 实际下标, 实际颜色)。"""
        left, right, top, bottom = 54, 10, 10, 8
        gap = 14
        n = len(panels)
        ph = (h - top - bottom - gap * (n - 1)) / n
        for pi, (title, series, ti, ai, color) in enumerate(panels):
            y0 = top + pi * (ph + gap)
            y1 = y0 + ph
            c.create_rectangle(left, y0, w - right, y1, outline='#dcdcdc')
            c.create_text(left + 6, y0 + 10, anchor='w', fill='#444',
                          font=('', 9, 'bold'), text=title)
            c.create_line(w - right - 96, y0 + 10, w - right - 74, y0 + 10,
                          fill='#999', width=2, dash=(4, 3))
            c.create_text(w - right - 70, y0 + 10, anchor='w', fill='#888',
                          font=('', 8), text='目标')
            c.create_line(w - right - 40, y0 + 10, w - right - 18, y0 + 10,
                          fill=color, width=2)
            c.create_text(w - right - 14, y0 + 10, anchor='w', fill='#888',
                          font=('', 8), text='实际')
            if not series:
                c.create_text((left + w - right) / 2, (y0 + y1) / 2,
                              text='等待数据…', fill='#aaa')
                continue
            vals = []
            for s in series:
                vals.extend((s[ti], s[ai]))
            lo, hi = min(vals), max(vals)
            if hi - lo < 1e-6:
                pad = max(abs(hi) * 0.1, 1.0)
            else:
                pad = (hi - lo) * 0.15
            lo -= pad
            hi += pad
            span = max(hi - lo, 1e-9)
            npts = len(series)
            xspan = max(npts - 1, 1)

            def px(i):
                return left + (i / xspan) * (w - left - right)

            def py(v):
                return y1 - (v - lo) / span * (y1 - y0)

            for k in range(5):
                v = lo + (hi - lo) * k / 4.0
                y = py(v)
                if y0 + 1 < y < y1 - 1:
                    c.create_line(left, y, w - right, y, fill='#f0f0f0')
                c.create_text(left - 5, y, anchor='e', fill='#999',
                              font=('Consolas', 8), text=f'{v:.2f}')
            for sel, col, dash in ((ti, '#999', (4, 3)), (ai, color, None)):
                pts = []
                for i, s in enumerate(series):
                    pts.extend((px(i), py(s[sel])))
                if len(pts) >= 4:
                    c.create_line(*pts, fill=col, width=2, dash=dash)
                elif len(pts) == 2:
                    c.create_oval(pts[0] - 3, pts[1] - 3, pts[0] + 3, pts[1] + 3,
                                  fill=col, outline='')

    # ---------- 页签切换 / 底部执行 ----------

    def _current_tab_title(self):
        try:
            return self.nb.tab(self.nb.select(), 'text')
        except Exception:  # noqa: BLE001
            return ''

    def _on_tab_changed(self):
        title = self._current_tab_title()
        if title == 'PID 调参':
            self.map_frame.pack_forget()
            if not self.wave_frame.winfo_ismapped():
                self.wave_frame.pack(fill=tk.BOTH, expand=True)
            self._redraw_wave()
            self.exec_hint.set('执行：把当前全部调参值重新下发到 407')
        else:
            self.wave_frame.pack_forget()
            if not self.map_frame.winfo_ismapped():
                self.map_frame.pack(fill=tk.BOTH, expand=True)
            if title == '节点动作序列':
                self.exec_hint.set('执行：九点任务（依次经过 9 个点，每点跑自己的动作）')
            elif title == '雷达 / 地图':
                self.exec_hint.set('执行：刚规划的单点路径，或规划好的九点全程')
            else:
                self.exec_hint.set('执行：动作序列（当前页签整段）')
        self._redraw()

    def execute_current(self):
        title = self._current_tab_title()
        if title == '节点动作序列':
            self.execute_mission()
        elif title == '雷达 / 地图':
            if (self._last_plan_kind == 'single' and self.plan
                    and self.plan.get('path_world')):
                self.navigate_plan()
            elif self.mission_legs:
                self.execute_mission()      # 依次走九个节点，并执行节点动作
            elif self.plan and self.plan.get('path_world'):
                self.navigate_plan()
            else:
                self._set_status('还没有规划路径：点地图或【③ 规划到目标】，'
                                 '或点【规划全程】走九点路线')
        elif title == 'PID 调参':
            self._send_all_tune()
        else:
            self.run_all()

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
        inner._scroll_canvas = cv          # 滚轮绑定按页签各用各的 canvas
        self._map_scroll = cv

        def _resize(_=None):
            cv.configure(scrollregion=cv.bbox('all'))
            cv.itemconfigure(win, width=max(cv.winfo_width(), 300))

        inner.bind('<Configure>', _resize)
        cv.bind('<Configure>', _resize)
        return inner

    def _bind_wheel(self, widget, cv=None):
        """Linux 下滚轮是 Button-4/5（不是 MouseWheel），递归绑到页签里所有控件。

        cv 必须是本页签自己的滚动 canvas：两个页签都用滚动容器时，不能共用
        self._map_scroll，否则在一个页签上滚滚轮会去滚另一个页签。
        """
        if cv is None:
            cv = getattr(widget, '_scroll_canvas', self._map_scroll)

        def _wheel(ev):
            cv.yview_scroll(-3 if ev.num == 4 else 3, 'units')

        widget.bind('<Button-4>', _wheel)
        widget.bind('<Button-5>', _wheel)
        for ch in widget.winfo_children():
            self._bind_wheel(ch, cv)

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

        # 模拟雷达时手上有抽签位置，就照着实摆；再点一次同一个障碍 = 删掉
        self.var_edit_obs = tk.BooleanVar(value=False)
        ttk.Checkbutton(box, text='手动布置障碍（点地图 加/删 圆柱）',
                        variable=self.var_edit_obs,
                        command=self._redraw).pack(anchor='w', pady=(2, 0))

        # 工位停车点：勾上以后点地图，把下拉框里那个工位的 approach 挪过去
        zrow = ttk.Frame(box); zrow.pack(fill=tk.X, pady=(3, 0))
        self.var_edit_zone = tk.BooleanVar(value=False)
        ttk.Checkbutton(zrow, text='点地图设置工位',
                        variable=self.var_edit_zone).pack(side=tk.LEFT)
        self.cb_zone = ttk.Combobox(
            zrow, width=7, state='readonly',
            values=[z.get('name', '') for z in self.cfg.get('work_zones', [])])
        if self.cb_zone['values']:
            self.cb_zone.current(0)
        self.cb_zone.pack(side=tk.LEFT, padx=(6, 0))

        row = ttk.Frame(box); row.pack(fill=tk.X, pady=(2, 0))
        ttk.Label(row, text='目标格').pack(side=tk.LEFT)
        self.ent_cell = ttk.Entry(row, width=6)
        self.ent_cell.insert(0, 'C3')
        self.ent_cell.pack(side=tk.LEFT, padx=2)
        self.ent_cell.bind('<Return>', lambda e: self.goto_cell())
        ttk.Button(row, text='规划', width=4, command=self.goto_cell).pack(side=tk.LEFT)

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
        ttk.Button(row2, text='规划', width=4, command=self.goto_odom).pack(side=tk.LEFT)

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
        # field.json 里 drivable.white=false 是"只走灰色田字形车道"。
        # 注意：ring_mm=550 时外圈+十字已经把全场铺满，这个开关没有区别；
        # 只有 ring_mm 调小、黄块和墙之间留出白区时，打开它才能压白区绕障。
        self.var_white = tk.BooleanVar(value=self.field.white_drivable)
        ttk.Checkbutton(box, text='白区可走（关 = 只走灰色车道）',
                        variable=self.var_white,
                        command=self._apply_white).pack(anchor='w')

        # ---- 任务路线 ----
        box = ttk.LabelFrame(tab, text='九点任务路线（节点页的路线名）', padding=4)
        box.pack(fill=tk.X, pady=3)
        row = ttk.Frame(box); row.pack(fill=tk.X)
        ttk.Button(row, text='规划全程（只算路，不动车）', command=self.plan_mission).pack(
            side=tk.LEFT, fill=tk.X, expand=True)
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
        map_cv = self._map_scroll
        wrap = lambda ev: map_cv.yview_scroll(-3 if ev.num == 4 else 3, 'units')
        map_cv.bind('<Button-4>', wrap)
        map_cv.bind('<Button-5>', wrap)

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
        """雷达朝向微调。雷达装在车头上，所以**车起点朝向一起改**——
        否则只转雷达箭头、车还斜着停在原地，地图和车对不上。
        如果确实是"雷达和车头有安装偏角"，改完再把车起点 yaw 单独调回去。
        """
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
        # 雷达 = 车头 ⇒ 车起点朝向跟着一起改，车才会真的转过来
        self.robot_start = (self.robot_start[0], self.robot_start[1],
                            self.field.lidar_yaw)
        self.cfg['robot_start'] = {'x': self.robot_start[0],
                                   'y': self.robot_start[1],
                                   'yaw_deg': self.robot_start[2]}
        try:
            self.ent_rdeg.delete(0, tk.END)
            self.ent_rdeg.insert(0, f'{self.robot_start[2]:g}')
        except Exception:
            pass
        self._redraw()

    def _apply_odom_shift(self, tx: float, ty: float, tth: float):
        """里程计坐标系被视觉矫正平移（odom' = t + R(tth)·odom）后，把"车起点"
        锚点做同样的姿态合成。这样 calibrate 重置里程计前后，场地↔里程计换算
        完全一致：车在地图上的位置连续，后续规划/下发的 goto 不会跑偏。"""
        if self.odom_start[2] is None:
            return
        c, s = math.cos(tth), math.sin(tth)
        ox, oy = self.odom_start[0], self.odom_start[1]
        self.odom_start[0] = tx + c * ox - s * oy
        self.odom_start[1] = ty + s * ox + c * oy
        self.odom_start[2] = wrap_deg(self.odom_start[2] + math.degrees(tth))
        self._set_status(
            f'里程计已被视觉矫正重置（平移 {tx:+.3f},{ty:+.3f}m '
            f'旋转 {math.degrees(tth):+.1f}°）：地图锚点已同步，场地坐标不变')
        self._need_redraw = True

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
            # 不再自动塞演示圆柱：障碍摆哪儿由【手动布置障碍】决定。
            # 只保留原料区转盘——它是真实凸起的实体，雷达本来就会扫到它。
            obs = [{'x': -150, 'y': 1150, 'r': 150, 'intensity': 90}]
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
        """这一格是不是"工位格"——只有**包含工位停车点/工位中心**的格子才算。

        ⚠ 不能用"矩形相交"来判断：原料区（x∈[-300,0]）的矩形会擦到相邻的
        B5（x∈[-675,-225]，重叠 75mm），于是 B5 被当成工位格**永久豁免**，
        里面有障碍也永远不封锁 —— 车就会从 B5 中间穿过去。
        """
        rc = self.field.cell_rect(c, r)
        for z in self.field.work_zones:
            # 停车点所在格：必须豁免，否则永远到不了工位
            ap = z.get('approach')
            if ap and rc.contains(ap[0], ap[1]):
                return True
            # 工位矩形中心所在格也豁免一份。
            # 注意 field.work_zones 的条目只有 name/approach/rect，**没有 x/y**
            zr = z.get('rect')
            if zr is not None and rc.contains((zr.x0 + zr.x1) / 2.0,
                                              (zr.y0 + zr.y1) / 2.0):
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
        self._last_plan_kind = 'single'
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

    def _seg_heading(self, seg, prev):
        """段行驶方向 seg（度）→ 车头朝向：seg 或 seg+180 里选离 prev 更近的。

        麦轮正着走/倒着走都比横着走直，所以让车头或车尾对着行驶方向，
        并且选转弯更小的那一头（少打方向、少磨轮子）。
        """
        d_fwd = wrap_deg(seg - prev)
        d_bwd = wrap_deg(seg + 180.0 - prev)
        return wrap_deg(seg + 180.0) if abs(d_bwd) < abs(d_fwd) else wrap_deg(seg)

    def _path_commands(self, path_mm, title=None, fixed_yaw_deg=None,
                       start_yaw_deg=None):
        """世界系折线 → motion_planner 的 goto 指令（里程计系，米/度）。

        start_yaw_deg: 出发时的车头朝向（场地系，度）。None = 车当前朝向。
        fixed_yaw_deg: 终点作业朝向（度）。None = 到站后保持最后一段的行驶朝向。

        麦轮能横移，但横着走没有正着走直、更容易打滑。所以改成**不横移**：
        每个拐点先原地把车头（或车尾，选转弯更小的一头）摆到下一段的行驶
        方向，再沿这段只前/后直行；走到最后一个航点站定后，才原地转到
        fixed_yaw_deg（作业朝向）。所有转体都是原地完成，不边走边转。
        """
        wps = self._waypoints(path_mm)
        cmds = []
        if title:
            cmds.append({'cmd': 'comment', 'text': title})
        if not wps:
            return cmds
        if start_yaw_deg is None:
            start_yaw_deg = self._robot_field()[2]
        prev = wrap_deg(start_yaw_deg)
        n = len(wps)
        hdgs = []
        for i, (x, y) in enumerate(wps):
            if i < n - 1:
                seg = math.degrees(math.atan2(wps[i + 1][1] - y,
                                              wps[i + 1][0] - x))
                hdg = self._seg_heading(seg, prev)
            elif fixed_yaw_deg is not None:
                hdg = wrap_deg(fixed_yaw_deg)
            elif hdgs:
                hdg = hdgs[-1]      # 没有作业朝向：到站就停，别再多转一下
            else:
                hdg = prev
            hdgs.append(hdg)
            prev = hdg
        for (x, y), hdg in zip(wps, hdgs):
            ox, oy, oth = self._field_to_odom(x, y, hdg)
            cmds.append({'cmd': 'goto',
                         'x_m': round(ox, 3),
                         'y_m': round(oy, 3),
                         'yaw_deg': round(math.degrees(oth), 1),
                         'arrive_yaw': True})
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
        # 单点导航：每个拐点原地摆正车头（或车尾）对着下一段行驶方向，
        # 到目标再转成作业朝向（不是作业点就保持到达时的朝向）。
        yaw = self._outward_yaw(self.goal_world)
        cmds = self._path_commands(
            self.plan['path_world'],
            f'导航 → {self.field.cell_name(*self.goal_world)} '
            f'{self._odom_str(*self.goal_world)}',
            fixed_yaw_deg=yaw)
        if not self.node.send_sequence(cmds):
            self._planner_missing()
            return
        self._set_status(f'已下发 {len(cmds) - 1} 个路点，motion_planner 接管…')

    # ==================== 任务路线 ====================

    def _fill_mission_list(self):
        if not self.field.mission_route:
            self.mission_info.set('field.json 里没有 mission.route')

    def _node_routes(self):
        """九节点页里每点填的路线名；节点页才是任务的唯一来源。

        必须保留空名字：节点和路线段是 1:1 的，过滤掉空节点会让后面的
        节点动作错位到别的点上。
        """
        return [str(n.get('route', '')).strip() for n in self.nodes]

    def plan_mission(self):
        routes = self._node_routes()
        if not routes and not self.field.mission_route:
            self._set_status('没有任务路线：在【节点动作序列】页填每点对应的路线名')
            return
        legs, total = mapping.plan_mission(
            self.field, self.grid, self.mask, self.obstacles,
            routes or self.field.mission_route,
            self.field.robot_radius, cell_block=self._cell_block(),
            cost_fn=self._cost_fn())
        self.mission_legs = legs
        self.mission_total = total
        self.plan = None
        self._last_plan_kind = 'mission'
        # 作业点要求"机械臂侧朝场地外侧"，把该朝向挂到每段的终点上
        for leg in legs:
            leg["final_yaw"] = self._outward_yaw(leg.get("to_xy"))
        ok = sum(1 for leg in legs if leg['ok'])
        eta = total / max(self.field.mission_speed, 1.0)
        self.mission_info.set(f'{ok}/{len(legs)} 段可通  总长 {total / 1000:.2f} m  '
                              f'约 {eta:.0f} s')
        bad = [f'{i + 1}.{leg["note"]}' for i, leg in enumerate(legs) if not leg['ok']]
        self._set_status(f'全程规划：{ok}/{len(legs)} 段可通，总长 {total / 1000:.2f} m'
                         + (f'；不通：{"；".join(bad)}' if bad else ''))
        self._redraw()

    def _append_node_actions(self, cmds, index):
        """把第 index 个节点（0 起）到点后要执行的动作整段拼进指令列表。"""
        if not (0 <= index < len(self.nodes)):
            return
        node = self.nodes[index]
        acts = node.get('actions') or []
        if not acts:
            cmds.append({'cmd': 'comment',
                         'text': f'节点 {index + 1}：{node.get("route", "")}（无动作）'})
            return
        cmds.append({'cmd': 'comment',
                     'text': f'节点 {index + 1}：{node.get("route", "")} 动作 {len(acts)} 条'})
        for act in acts:
            if isinstance(act, dict):
                cmds.append(dict(act))

    def execute_mission(self):
        """底部【执行】在节点页：九点任务 = 走路径 + 每点自己的动作序列。

        指令顺序：
          先去第 1 点 → 第 1 点动作 → 第 1→2 段路径 → 第 2 点动作 → … → 第 9 点动作
        pick/place 的宏展开在 motion_planner 载入时做；vision_pick/align 在
        执行到那一条时启动视觉程序，并用程序输出的 JSON 改目标/插动作。
        序列本身由 _build_mission_commands 组装，【预览完整序列】按钮也用它，
        两种入口看到的内容完全一致。
        """
        cmds, err = self._build_mission_commands()
        if cmds is None:
            messagebox.showwarning('无法生成任务序列', err)
            return

        # ---- 没有里程计 / 起点没标定：只填序列，不下发 ----
        if self.node.odom is None or self.odom_start[2] is None:
            why = '还没收到 /odom' if self.node.odom is None else '车起点还没标定'
            self.commands = [dict(c) for c in cmds]
            self._reload_listbox()
            self._set_status(
                f'{why}：九点任务共 {len(cmds)} 条指令，已填进【动作序列】页查看，'
                f'未下发。接上真车并标定起点后再点【▶ 执行】。')
            return

        if self.node.robot_status and self.node.robot_status[0] != 1:
            if not messagebox.askyesno(
                    '车还没就绪',
                    '整车当前不是就绪/使能状态。点【执行】会先重新使能再下发，'
                    '确认周围安全后继续？'):
                return
        if not self.node.send_sequence(cmds):
            self._planner_missing()
            return
        n_goto = sum(1 for c in cmds if c['cmd'] == 'goto')
        n_act = sum(1 for c in cmds if c['cmd'] not in ('goto', 'comment'))
        self._set_status(
            f'已下发九点任务：{n_goto} 个路点 + {n_act} 个节点动作，motion_planner 接管…')

    def _build_mission_commands(self):
        """把"九点任务"组装成 motion_planner 会收到的完整指令序列
        （路点 goto + 每点动作），只组装、不发送。

        返回 (cmds, None)；组装不了返回 (None, 原因)。
        execute_mission 用它下发，【预览完整序列】用它填进动作序列页检查。
        """
        if not self.nodes:
            return None, '【节点动作序列】页还没有任何节点'
        routes = self._node_routes()
        if len(routes) < 2:
            return None, '至少需要 2 个节点才能组成路线'

        self.plan_mission()
        if not self.mission_legs:
            return None, '全程规划失败（看下方提示）'
        bad = [i for i, leg in enumerate(self.mission_legs) if not leg['ok']]
        if bad:
            return None, (f'第 {",".join(str(i + 1) for i in bad)} 段规划失败'
                          f'（共 {len(self.mission_legs)} 段）。'
                          f'先处理障碍，或在节点页改路线点')

        headings = self._mission_headings()
        cmds = [{'cmd': 'comment',
                 'text': f'九点任务：{len(self.nodes)} 个节点 / {len(self.mission_legs)} 段'}]

        # ---- 启停区发车/停车：横移+直行进出，不在角上转向 ----
        # 启停区角上只有 300×300，原地转斜角车角（对角线半长 212mm）会出界；
        # 所以小范围内不转向，把位移拆成"横移 + 直行"两条指令，转向一律放到
        # 格子中心（E5/E1，开阔车道交汇口）再做。第一段/最后一段分别从格子
        # 中心重新规划，不在角上绕外圈矩形车道。
        start_pt = self.mission_legs[0]['from_xy']      # START = 启停区中心
        gateway = self._start_gateway_cell(start_pt)
        cur_f = self._robot_field()
        cur = cur_f[:2]
        car_at_start = (gateway is not None
                        and math.hypot(cur[0] - start_pt[0],
                                       cur[1] - start_pt[1]) <= 150.0)

        self._append_node_actions(cmds, 0)
        if car_at_start:
            gname = self.field.cell_name(*gateway)
            # 发车：横移+直行到格子中心（不转向）
            cmds += self._strafe_move_pair(
                gateway[0] - cur[0], gateway[1] - cur[1], cur_f[2],
                f'发车：横移到 {gname} 中心')
            # 第一段从格子中心重新规划（绕开外圈矩形线）
            leg0 = self.mission_legs[0]
            res = mapping.plan_path(
                self.grid, self.mask, self.obstacles,
                self.grid.w2c(*gateway), self.grid.w2c(*leg0['to_xy']),
                self.field.robot_radius, cell_block=self._cell_block(),
                cost_fn=self._cost_fn())
            if res['path']:
                cmds += self._path_commands(
                    res['path_world'],
                    f'1. {leg0["from_name"]} → {leg0["to_name"]}',
                    start_yaw_deg=cur_f[2],
                    fixed_yaw_deg=leg0.get('final_yaw'))
            else:
                cmds += self._path_commands(
                    leg0['path'], f'1. {leg0["from_name"]} → {leg0["to_name"]}',
                    start_yaw_deg=cur_f[2],
                    fixed_yaw_deg=leg0.get('final_yaw'))
            begin_i = 1
        else:
            # 车不在启停区：保持原逻辑（先规划到起点，再按原计划走）
            if math.hypot(cur[0] - start_pt[0], cur[1] - start_pt[1]) > 150.0:
                res = mapping.plan_path(
                    self.grid, self.mask, self.obstacles,
                    self.grid.w2c(*cur), self.grid.w2c(*start_pt),
                    self.field.robot_radius, cell_block=self._cell_block(),
                    cost_fn=self._cost_fn())
                if not res['path']:
                    return None, f'从当前位置到第 1 个节点不通：{res["note"]}'
                # 到第 1 个节点时，车头已经摆成第一段出发的朝向
                cmds += self._path_commands(res['path_world'], '先去第 1 个节点',
                                            start_yaw_deg=headings[0])
            begin_i = 0

        last_i = len(self.mission_legs) - 1
        for i in range(begin_i, len(self.mission_legs)):
            leg = self.mission_legs[i]
            title = f'{i + 1}. {leg["from_name"]} → {leg["to_name"]}'
            if (i == last_i and last_i >= 1 and gateway is not None
                    and math.hypot(leg['to_xy'][0] - start_pt[0],
                                   leg['to_xy'][1] - start_pt[1]) < 1.0):
                # 停车：最后一段只开到格子中心，到格子的同时把车头摆成停车
                # 朝向（在开阔车道交汇口转，不进启停区小角再转）
                park_yaw = leg.get('final_yaw')
                res = mapping.plan_path(
                    self.grid, self.mask, self.obstacles,
                    self.grid.w2c(*leg['from_xy']), self.grid.w2c(*gateway),
                    self.field.robot_radius, cell_block=self._cell_block(),
                    cost_fn=self._cost_fn())
                if res['path']:
                    cmds += self._path_commands(
                        res['path_world'], title,
                        start_yaw_deg=headings[i],
                        fixed_yaw_deg=park_yaw)
                # 横移+直行进启停区（不转向）。分解用的朝向必须和车停在格子
                # 中心时的实际朝向一致：final_yaw 对 START 是 None（_outward_yaw
                # 只认 work_zones），此时 _path_commands 会保持最后的行驶朝向，
                # 和 headings[i] 可能差 90° 以上——所以直接回读最后一条 goto
                # 的 yaw_deg，别用假设值。
                actual_yaw = None
                for c in reversed(cmds):
                    if c.get('cmd') == 'goto':
                        actual_yaw = float(c['yaw_deg'])
                        break
                if actual_yaw is None:
                    actual_yaw = headings[i]
                # yaw_deg 是里程计系、位移是场地系，差一个 dq 旋转；分解必须
                # 在同一坐标系里做，否则两个坐标系差 180° 时横移/直行全反。
                actual_yaw = self._odom_to_field_yaw_deg(actual_yaw)
                cmds += self._strafe_move_pair(
                    leg['to_xy'][0] - gateway[0], leg['to_xy'][1] - gateway[1],
                    actual_yaw, '停车：横移回启停区')
            else:
                cmds += self._path_commands(
                    leg['path'], title,
                    start_yaw_deg=headings[i],
                    fixed_yaw_deg=leg.get('final_yaw'))
            self._append_node_actions(cmds, i + 1)
        return cmds, None

    def _odom_to_field_yaw_deg(self, yaw_odom_deg):
        """里程计系航向（度）→ 场地系航向。和 _field_to_odom 用同一个 dq，
        保证跨坐标系换算时朝向差恰好抵消。"""
        odeg0 = self.odom_start[2]
        dq_deg = self.robot_start[2] - (odeg0 if odeg0 is not None else 0.0)
        return wrap_deg(yaw_odom_deg + dq_deg)

    def _strafe_move_pair(self, dx_mm, dy_mm, yaw_deg, label):
        """把一段世界系位移拆成"横移 + 直行"两条指令（车体系，**不转向**）。

        用于启停区小范围进出：角上只有 300×300，原地转斜角车角会出界；
        横移+直行对任何目标位移都不用转角度。小于 3cm 的分量省略。
        """
        yaw = math.radians(yaw_deg)
        fwd = dx_mm * math.cos(yaw) + dy_mm * math.sin(yaw)
        lat = -dx_mm * math.sin(yaw) + dy_mm * math.cos(yaw)
        out = []
        if abs(lat) >= 30.0:
            out.append({'cmd': 'strafe', 'dist_m': round(lat / 1000.0, 3)})
        if abs(fwd) >= 30.0:
            out.append({'cmd': 'move', 'dist_m': round(fwd / 1000.0, 3)})
        if out:
            out.insert(0, {'cmd': 'comment',
                           'text': f'{label}（横移{lat:+.0f}mm 行走{fwd:+.0f}mm）'})
        return out

    def _start_gateway_cell(self, zone_xy):
        """启停区发车/停车的斜线接入点：离启停区中心最近的 5×5 格中心。

        启停区1（右上角）→ E5，启停区2（右下角）→ E1。格子中心在外圈车道里，
        和启停区之间是一整片连通灰色车道，斜线不跨黄块。
        """
        best = None
        for r in range(self.field.cell_rows):
            for c in range(self.field.cell_cols):
                p = self.field.cell_center(c, r)
                d = math.hypot(p[0] - zone_xy[0], p[1] - zone_xy[1])
                if best is None or d < best[0]:
                    best = (d, p)
        return best[1] if best else None

    def _preview_mission_sequence(self):
        """【预览完整序列】：把组装好的九点任务（路点 + 每点动作）完整填进
        【动作序列】页，切过去显示，只预览、不发送。检查无误后点底部【▶ 执行】。
        """
        cmds, err = self._build_mission_commands()
        if cmds is None:
            messagebox.showwarning('无法生成任务序列', err)
            return
        self.commands = [dict(c) for c in cmds]
        self._reload_listbox()
        self.nb.select(0)   # 切到【动作序列】页
        n_goto = sum(1 for c in cmds if c['cmd'] == 'goto')
        n_act = sum(1 for c in cmds if c['cmd'] not in ('goto', 'comment'))
        self._set_status(
            f'完整任务序列已填入动作序列页：{n_goto} 个路点 + {n_act} 个动作 '
            f'（只预览未下发；检查无误后点底部【▶ 执行】）')

    # ==================== 模拟执行（没有真车也能看车跑） ====================

    def _play_mission(self):
        """把整条任务路线按固定步长采样成位姿序列，用 tk.after 一步步播放。

        **朝向**：和真车指令一致——每到拐点先**原地**把车头（或车尾，选
        转弯小的一头）摆到下一段行驶方向，再沿这段直着开过去；到作业点
        再原地转成作业朝向（final_yaw）。所以动画里不会出现横移。

        采样按 60mm 定长沿折线插值：路径大多是一大段直线，共线点被压掉后
        两个航点能隔 500mm 以上，按航点播就是"一步跳半米"。
        """
        if not self.mission_legs:
            self._set_status('先点【规划全程】')
            return

        step_mm = 60.0
        spin_deg = 20.0          # 原地转：每步最多转多少度

        pos, tgt = [], []
        yaw = self._robot_field()[2]

        def spin_to(target):
            """原地转到 target（分成若干步，每步 ≤ spin_deg）。"""
            nonlocal yaw
            d = wrap_deg(target - yaw)
            if abs(d) < 0.05:
                return
            n_spin = max(1, int(abs(d) / spin_deg) + 1)
            for k in range(1, n_spin + 1):
                pos.append(pos[-1] if pos else (0.0, 0.0))
                tgt.append(wrap_deg(yaw + d * k / n_spin))
            yaw = wrap_deg(target)

        for leg in self.mission_legs:
            wps = self._waypoints(leg.get('path') or [], min_step_mm=10.0)
            if not wps:
                continue
            # 段与段的衔接点和上一段终点重合：采样从 step_mm 起跳，
            # 不会在原地多插一步（但这一段本身不能丢）
            if not pos:
                pos.append((float(wps[0][0]), float(wps[0][1])))
                tgt.append(yaw)
            for k in range(len(wps) - 1):
                x0, y0 = wps[k]
                x1, y1 = wps[k + 1]
                d = math.hypot(x1 - x0, y1 - y0)
                if d <= 1e-6:
                    continue
                hd = self._seg_heading(
                    math.degrees(math.atan2(y1 - y0, x1 - x0)), yaw)
                spin_to(hd)                       # 先原地摆正车头/车尾
                t = step_mm
                while t < d:
                    r = t / d
                    pos.append((x0 + (x1 - x0) * r, y0 + (y1 - y0) * r))
                    tgt.append(yaw)
                    t += step_mm
                pos.append((float(x1), float(y1)))
                tgt.append(yaw)
            fy = leg.get('final_yaw')
            if fy is not None:
                spin_to(float(fy))                # 到作业点再原地转成作业朝向

        if not pos:
            self._set_status('没有可播放的路径')
            return

        pts = [(pos[k][0], pos[k][1], tgt[k]) for k in range(len(pos))]

        self._stop_play()
        self.play_pts = pts
        self.play_i = 0
        self._set_status(f'模拟执行：{len(pts)} 步 × 130ms ≈ {len(pts) * 0.13:.0f} s')
        self._play_tick()

    def _play_tick(self):
        """每 130ms 走一步。走完自动停。"""
        if not self.play_pts or self.play_i >= len(self.play_pts):
            n = len(self.play_pts or [])
            self._stop_play()
            self._set_status(f'模拟执行结束（走过 {n} 个航点）')
            return
        self.sim_pose = self.play_pts[self.play_i]
        self.play_i += 1
        self._redraw()
        self.play_after = self.root.after(130, self._play_tick)

    def _stop_play(self):
        if self.play_after is not None:
            try:
                self.root.after_cancel(self.play_after)
            except Exception:
                pass
            self.play_after = None
        self.play_pts = None
        self.play_i = 0
        self.sim_pose = None
        self._redraw()

    def _mission_headings(self):
        """每一段路径**出发时**的车头朝向（场地系，度）。

        上一段在作业点已经原地转成本段作业朝向（leg["final_yaw"]），
        这一段就从那个朝向出发；第一段从发车时的车头朝向出发。
        段内拐点由 _path_commands 逐段摆正（车头或车尾对着行驶方向），
        到终点再原地转成本段的 final_yaw。
        """
        yaw = self._robot_field()[2]
        out = []
        for leg in self.mission_legs:
            out.append(yaw)
            fy = leg.get('final_yaw')
            if fy is not None:
                yaw = float(fy)
        return out

    def clear_route(self):
        self.mission_legs = []
        self.mission_total = 0.0
        self.plan = None
        self._last_plan_kind = None
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

    # ---------- 手动布置障碍（模拟雷达时用） ----------

    def _toggle_obstacle(self, wx, wy):
        """点地图加/删一个圆柱障碍。

        落在已有障碍（半径 + 60mm）以内 = 删掉它；否则在该处加一个。
        障碍直接进 self.obstacles，A* 规划立刻生效。
        """
        r_cyl = float(self.cfg.get("cylinder", {}).get("diameter_mm", 50.0)) / 2.0
        hit = None
        for ob in self.obstacles:
            if math.hypot(wx - ob["x"], wy - ob["y"]) <= r_cyl + 60.0:
                hit = ob
                break
        if hit is not None:
            self.obstacles.remove(hit)
            cells = self._rebuild_blocked_cells()
            self._set_status(f'删除障碍 @ ({hit["x"]:.0f}, {hit["y"]:.0f})，'
                             f'剩 {len(self.obstacles)} 个；'
                             f'整格封锁：{self._cells_text(cells)}')
        else:
            self.obstacles.append({"x": float(wx), "y": float(wy), "r": r_cyl,
                                   "n": 0, "rms": 0.0, "kind": "manual", "dist": 0.0})
            cells = self._rebuild_blocked_cells()
            self._set_status(f'添加障碍 @ ({wx:.0f}, {wy:.0f}) φ{r_cyl * 2:.0f}mm，'
                             f'共 {len(self.obstacles)} 个；'
                             f'整格封锁：{self._cells_text(cells)}')
        self.plan = None
        # 模拟雷达的障碍是构造时拷进去的副本，改了 self.obstacles 要同步过去，
        # 否则新摆的圆柱不会出现在点云里。
        if self.lidar is not None and self.var_sim.get():
            self.lidar.obstacles = self._sim_obstacles()
        self._redraw()

    # ---------- 工位停车点 ----------

    def _set_zone_approach(self, wx, wy):
        """把下拉框选中的工位停车点挪到地图上点的那一点，并立刻写回 field.json。

        改的是 `approach`（规划真正用的点）；`x/y/w/h`（画出来的方框、
        以及"机械臂朝哪"的参考中心）不动。
        """
        name = self.cb_zone.get() if hasattr(self, 'cb_zone') else ''
        if not name:
            self._set_status('先在右边下拉框选一个工位')
            return

        raw = self.cfg.get('work_zones', [])
        idx = next((i for i, z in enumerate(raw) if z.get('name') == name), None)
        if idx is None:
            self._set_status(f'field.json 里没有工位「{name}」')
            return

        px, py = round(float(wx), 1), round(float(wy), 1)
        raw[idx]['approach'] = [px, py]
        # 运行中的 FieldModel 是另一份拷贝，规划用的是它，必须同步
        for z in self.field.work_zones:
            if z.get('name') == name:
                z['approach'] = (px, py)
                break

        self._rebuild_blocked_cells()      # 工位格豁免跟着变
        self.plan = None
        self.mission_legs = []
        self.save_cfg()                    # 直接落盘，免得忘了按【保存场地参数】

        yaw = self._outward_yaw((px, py))
        self._set_status(
            f'「{name}」停车点 → ({px:.0f}, {py:.0f})'
            + (f'，机械臂朝向 {yaw:.0f}°' if yaw is not None else '')
            + '，已写入 field.json；重新点【规划全程】生效')
        self._redraw()

    # ---------- 整格封锁 ----------

    def _cells_text(self, cells):
        if not cells:
            return '无'
        return '、'.join(self.field.cell_label(c, r) for (c, r) in sorted(cells))

    def _rebuild_blocked_cells(self):
        """按当前障碍重算"整格禁行"的 5×5 格。

        规则和 detect() 一致：障碍所在的 450×450 格**整格**不许走，
        但**工位所在格不封**（封了就永远到不了工位）。

        注意：手动布置的障碍也必须走这条路，否则 `blocked_cells` 一直是空的，
        「整格封锁」勾了也不生效 —— 车会从有障碍的那一格中间穿过去。
        """
        cells = set()
        for ob in self.obstacles:
            cr = self.field.cell_at(ob['x'], ob['y'])
            if not cr or self._is_station_cell(*cr):
                continue
            cells.add(cr)
        self.blocked_cells = cells
        return cells

    # ---------- 作业点到达朝向 ----------

    def _outward_yaw(self, xy):
        """作业点上要求车摆到的场地朝向：**机械臂侧朝场地外侧**。

        机械臂 + 扫码相机在车头**逆时针 90°**（车体 +y，车左）。
        要让车左指向"从场心指向该作业点"的外法线，车头就要比外法线少 90°：
            外法线   0°(东) → 车头 270°(南) → 车左朝东 ✓
            外法线  90°(北) → 车头   0°(东) → 车左朝北 ✓
            外法线 180°(西) → 车头  90°(北) → 车左朝西 ✓
            外法线 270°(南) → 车头 180°(西) → 车左朝南 ✓

        xy 不是 work_zones 的 approach 点（或就在场心）就返回 None，不加朝向约束。
        """
        if xy is None:
            return None
        zone = None
        for z in self.cfg.get("work_zones", []) or []:
            ap = z.get("approach")
            if ap and math.hypot(xy[0] - ap[0], xy[1] - ap[1]) <= 1.0:
                zone = z
                break
        if zone is None:
            return None
        # 机械臂要指向的方向 = 从停车点指向该作业区中心（原料区的停车点偏了
        # x=-150，用"场心外法线"会差 10°，指作业区本身才准）
        dx = float(zone.get("x", 0.0)) - xy[0]
        dy = float(zone.get("y", 0.0)) - xy[1]
        if math.hypot(dx, dy) < 1.0:
            return None
        arm_dir = math.degrees(math.atan2(dy, dx))
        return wrap_deg(arm_dir - 90.0)

    def on_click(self, ev):
        w = self._px_to_world(ev.x, ev.y)
        if w is None:
            return
        if self.var_edit_obs.get():        # 手动布置障碍模式下，点击只改障碍，不设目标
            self._toggle_obstacle(*w)
            return
        if self.var_edit_zone.get():       # 设置工位模式下，点击改该工位的停车点
            self._set_zone_approach(*w)
            return
        if self.var_snap.get():
            cr = self.field.cell_at(*w)
            if cr:
                w = self.field.cell_center(*cr)
        self.goal_world = w
        self._set_status(f'目标 {self._odom_str(*w)}  '
                         f'[场地格 {self.field.cell_name(*w)}]（只规划，'
                         f'点底部【▶ 执行】才动车）')
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

        # 车（当前位姿）：优先模拟/真实位姿，离线时按起点画虚线示意。
        # 车体尺寸由 field.json 的 robot.size_mm 决定：
        #   车头 = 车体 +x（前进方向，红色粗边 + 红箭头）
        #   机械臂 + 扫码相机 = 车头逆时针 90°（车体 +y，绿色粗边 + 绿箭头）
        o = self.node.odom
        live = o is not None and self.odom_start[2] is not None
        sim = self.sim_pose is not None
        if sim:
            # 模拟执行：没有真车，按播放出来的位姿画（实线，看起来像真在跑）
            fx, fy, fd = self.sim_pose
            live = True
        elif live:
            fx, fy, fd = self._odom_to_field(*o)
        else:
            # 没有里程计时按"车停在起点"画 —— 离线也能核对车体尺寸和朝向
            fx, fy, fd = self.robot_start
        hf = self.robot_size / 2.0
        ca, sa = math.cos(math.radians(fd)), math.sin(math.radians(fd))

        def _corner(lx, ly):
            """车体坐标(mm) → 画布像素（x 朝车头、y 朝车左）。"""
            return self._w2p(fx + lx * ca - ly * sa,
                             fy + lx * sa + ly * ca)

        c.create_polygon(*[v for p in (_corner(+hf, +hf), _corner(+hf, -hf),
                                       _corner(-hf, -hf), _corner(-hf, +hf))
                           for v in p],
                         fill='#f8bbd0',
                         outline='#ad1457' if live else '#8d8d8d',
                         width=2,
                         dash=() if live else (4, 3))   # 没里程计时画虚线
        # 车头：前边加粗 + 一根短箭头
        c.create_line(*_corner(+hf, +hf), *_corner(+hf, -hf), fill='#ad1457', width=4)
        c.create_line(*_corner(0, 0), *_corner(+hf + 90, 0),
                      fill='#ad1457', width=2, arrow=tk.LAST)
        # 机械臂侧（车左）：绿色粗边 + 绿箭头
        c.create_line(*_corner(+hf, +hf), *_corner(-hf, +hf), fill='#2e7d32', width=4)
        c.create_line(*_corner(0, 0), *_corner(0, +hf + 90),
                      fill='#2e7d32', width=2, arrow=tk.LAST)
        if sim:
            c.create_text(*_corner(0, -hf - 40), text='模拟执行中',
                          fill='#d81b60', font=('', 8))
        elif not live:
            c.create_text(*_corner(0, -hf - 40), text='车（未接车，按起点画）',
                          fill='#8d8d8d', font=('', 8))

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
        # 里程计被 calibrate（圆环视觉矫正）重置：把"车起点"锚点做同样的平移，
        # 保持 场地↔里程计 换算不变，否则地图上的车会整体跳一个漂移量。
        shift = self.node.odom_shift
        if shift is not None:
            self.node.odom_shift = None
            self._apply_odom_shift(*shift)
        self._sample_wave()
        self._update_tune_ack()
        if hasattr(self, 'gimbal_limit_vars'):
            self._sync_gimbal_limits()
        # 407 刚上电 / 刚重连：把界面上的参数整套重发，避免 407 静默回默认值
        online = bool(self.node.online)
        if online and not self._tune_was_online:
            self._send_all_tune()
        self._tune_was_online = online
        if got or self._need_redraw:
            self._redraw()
            self._need_redraw = False
        if self._current_tab_title() == 'PID 调参':
            self._redraw_wave()
        self.root.after(100, self._refresh)

    def close(self):
        # 关窗口时先停雷达（发 D6 停止命令、关串口）。窗口可能已经销毁，
        # 这时改控件会抛 TclError，忽略即可。
        try:
            self.stop_lidar()
        except Exception:
            pass
        if self._tune_after is not None:
            try:
                self.root.after_cancel(self._tune_after)
            except Exception:
                pass
            self._tune_after = None

    # ==================== 动作序列编辑 ====================

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
