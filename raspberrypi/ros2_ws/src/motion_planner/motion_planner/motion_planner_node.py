#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""运动规划节点 = 任务状态机（执行一段"指令序列"）。

================ 大白话：这个节点在干什么 ================

比赛要做的动作（读码、走、转、抓、放……）本质就是一条一条"动作指令"按顺序执行。
这个节点就是那个"执行器"：它读一份指令序列（JSON），一条一条往下做，
每做一条之前先等上一条"到位"，全部做完就停下。

它不自己算"车该往哪走、转向多少"——那是上位机/你在赛前编辑好的。
它只负责：
  1. 把"前进 0.5 米"翻译成世界系里的一个目标点 (x, y) + 目标朝向；
  2. 把目标点通过 /cmd_pose 发给 F407（位置闭环、航向闭环在 F407 上跑）；
  3. 读 /odom 里程计，判断"到了没"，到了就执行下一条。

指令序列的格式（JSON）。相对指令最直观，执行时自动累计成绝对位姿：

  move     {"cmd":"move",   "dist_m":0.5}      前进(负=后退)
  strafe   {"cmd":"strafe", "dist_m":0.3}      横向平移(正=左)
  turn     {"cmd":"turn",   "deg":90}          原地转(正=逆时针/左转，负=右转)
  goto     {"cmd":"goto",   "x_m":1.2, "y_m":-0.3, "yaw_deg":45}
                                               绝对位姿(里程计世界系，米/度)。
                                               雷达地图导航就用它：上位机把规划出的
                                               场地路点换算成里程计坐标后下发
  gimbal   {"cmd":"gimbal", "yaw_deg":-30, "lift_deg":50}  云台三轴绝对角(可缺省)
  gimbal_yaw  {"cmd":"gimbal_yaw",  "delta_deg":5}   单轴相对微调：在当前角度上加减，
  gimbal_lift {"cmd":"gimbal_lift", "delta_deg":-20} 只动一个轴。起算点是 /gimbal_state
  gimbal_ext  {"cmd":"gimbal_ext",  "delta_deg":30}  的实测角——所以 color_pick 刚把
                                                     云台跟到物料上方、不知道 yaw 具体
                                                     多少时，直接写 "再降 20°" 就行。
           阻塞到该轴实测角进入 ±tol_deg（默认 2°）才执行下一条，所以后面
           不用再加 delay。可选字段 tol_deg / timeout_ms 覆盖默认值；
           超时只报警不中断整个序列。参数见 gimbal_arrive_tol_deg /
           gimbal_arrive_timeout_s。
  gripper  {"cmd":"gripper","deg":45}          夹爪舵机
  plate    {"cmd":"plate",  "deg":90}          载物盘舵机
  cam      {"cmd":"cam",    "deg":0}           相机舵机
  enable   {"cmd":"enable", "en":true}         使能/失能电机
  delay    {"cmd":"delay",  "ms":1000}         原地等多少毫秒
  calibrate{"cmd":"calibrate","x_m":-2.169,"y_m":1.061}
           圆环工位视觉矫正：ring_fix 节点开摄像头识别三圆环，小步驱动底盘把
           中间环对到标定基准，对准后通过 /set_odom 把里程计重置到"圆环坐标
           (+对准偏移)"——清除累计里程计误差。x_m/y_m 是中间圆环的里程计坐标
           （米）。失败/超时就跳过这一条，不中断任务。
  comment  {"cmd":"comment","text":"..."}      注释，什么都不做

  ===== 复杂动作（本次新增，给节点动作序列用） =====

  script   {"cmd":"script", "program":"python3", "args":"vision/main.py --device /dev/video0",
            "cwd":"/home/pi/vision", "wait":true, "timeout_s":120}
           启动任意程序。wait=false 时只启动、不等退出（适合常驻的视觉窗口）。
  align    {"cmd":"align", "program":"python3", "args":"align.py", "timeout_s":30,
            "result_file":"/tmp/align.json"}
           视觉矫正：跑完程序后读它输出的 JSON（stdout 最后一行，或 result_file），
           形如 {"dx_m":0.012,"dy_m":-0.008,"dyaw_deg":1.5}，在当前目标位姿上叠加
           修正量并重新下发，然后等车到位。程序没输出 JSON 就跳过，不中断任务。
  vision_pick {"cmd":"vision_pick", "program":"python3", "args":"pick.py", ...}
           先跑视觉程序，读它输出的 {"yaw_deg":..,"lift_deg":..,"extend_deg":..}
           （缺省字段沿用下面 pick 的默认值），随后自动执行一次 pick 抓取宏。
  pick     {"cmd":"pick", "yaw_deg":0, "lift_deg":-40, "safe_lift_deg":0,
            "extend_deg":150, "gripper_open":120, "gripper_close":35, "settle_ms":800}
           抓取宏：张开夹爪 → 云台到安全位 → 降到抓取位 → 等稳定 → 夹紧 → 抬起。
  place    参数同 pick，动作反过来：到安全位 → 降下 → 张开 → 抬起。

坐标约定：车正前方为 +x，左侧为 +y，逆时针为正。和 F407 的里程计/陀螺仪一致。
align 的修正量也在这个里程计世界系里（米 / 度）。

================ 动作执行策略：先走到位，再原地转向 ================

F407 的位置环和航向环是同时收敛的：一个 POSE 帧里同时给 (x, y, yaw)，
车会一边平移一边转。麦轮边走边转时轮子打滑，里程计（车体位移按 IMU yaw
旋转到世界系）误差会明显变大。

所以走 / 去某点分两段执行：
  1. 行进段：目标位置 = 终点，目标朝向 = **出发时的当前朝向**。车头锁住、
     只平移（全向底盘可以直接横移），不旋转 → 里程计干净。
  2. 到点后按 turn_settle_s 停稳，再把目标朝向改成终点要求的 yaw，
     原地转向到位；还要连续 turn_hold_s 稳定在容差内才允许走下一步。
中间的航点（goto 带 arrive_yaw=false）只走第 1 段：一路保持出发朝向，
既不在半路转，也不等朝向，直到最后一个航点才原地转。
"""

import json        # 解析/生成 JSON（指令序列就是 JSON）
import math        # 三角函数、弧度换算
import re          # 任务码占位符 $b1c1$ 解析
import os          # 展开路径里的 ~ 和环境变量
import queue       # 子程序输出用后台线程读，避免阻塞状态机
import shlex       # 把 "程序 参数" 拆成 argv
import subprocess  # 动作序列里"启动一个程序"（视觉脚本等）
import threading   # 读子程序 stdout 的线程
import time        # 超时判断

import rclpy
from rclpy.node import Node
from car_interfaces.msg import Pose2D               # 平面位姿：x, y, theta(弧度)，自定义消息
from nav_msgs.msg import Odometry                  # 里程计消息
from std_msgs.msg import Bool, Float32MultiArray, String


class MotionPlannerNode(Node):
    def __init__(self):
        super().__init__('motion_planner')   # 节点名

        # ---- 可调参数 ----
        # 到位判定阈值：位置误差 < 2cm，朝向误差 < 0.05rad(约3°)
        self.declare_parameter('arrive_tol_xy', 0.02)     # 米
        self.declare_parameter('arrive_tol_yaw', 0.05)    # 弧度
        # 单个走/转/去指令最多等多久。到位判据很严（2cm/3°），F407 超调后
        # 可能来回摆很久；不设上限的话状态机会永远卡在这一条。
        self.declare_parameter('arrive_timeout_s', 20.0)  # 秒
        # 位置到位后、原地转向前的停稳时间：让底盘先把平移速度收干净，
        # 避免"还在往前滑的时候开始转"带偏位置。
        self.declare_parameter('turn_settle_s', 0.3)      # 秒
        # 转向到位后的确认时间：必须连续这么久都在容差内，才允许开始下一条
        # （否则车还有角速度、或 odom 噪声恰好扫过容差，就会"没转到位就开走"）。
        self.declare_parameter('turn_hold_s', 0.3)        # 秒
        # gimbal_yaw/lift/ext 单轴相对微调的"到位"判据：/gimbal_state 的实测角
        # 与目标的偏差落进 ±tol 就放行下一条；最多等 timeout 秒，超时只报警
        # 不中断（和 calibrate 一样，单条失败不毁整个任务）。
        self.declare_parameter('gimbal_arrive_tol_deg', 2.0)
        self.declare_parameter('gimbal_arrive_timeout_s', 2.5)
        # 状态机推进频率（Hz）。位姿重发、到位判断、指令切换都在这个节拍上。
        # /gimbal_state 是 20Hz，所以 20 已经吃满；再高只会多占上行（位姿帧）。
        self.declare_parameter('tick_hz', 10.0)
        # 启动时自动加载的序列文件路径（空 = 不加载，等上位机下发）
        self.declare_parameter('sequence_file', '')
        # 视觉程序的默认位置：动作序列里的 script / align / vision_pick 没写
        # program 时就用这里，方便整机换目录时只改一处。
        self.declare_parameter('vision_program', '')
        self.declare_parameter('vision_args', '')
        self.declare_parameter('vision_cwd', '')
        self.declare_parameter('script_timeout_s', 60.0)
        # 圆环视觉矫正（calibrate 指令）最多等多久：要覆盖摄像头打开、识别
        # 稳定、小步对位的全部时间，给足余量。
        self.declare_parameter('calibrate_timeout_s', 60.0)
        # ---- 任务码动作原语的放置位姿（number_put/double_put）----
        # 三环独立查表：calibrate 对位后（车不动），分别实测"放环 1/2/3 成功"时的
        # 云台三轴角度填入 place_ringN_*。不做"基准+步进"的线性假设——旋转云台
        # 跨环的映射本身非线性，量三个填三个最准。double_put = 同环角度 + lift 抬高
        # stack_lift_deg（叠在第一批物料上方）。
        for _ring in (1, 2, 3):
            self.declare_parameter(f'place_ring{_ring}_yaw_deg', 0.0)
            self.declare_parameter(f'place_ring{_ring}_extend_deg',
                                   150.0 if _ring == 2 else 0.0)
            self.declare_parameter(f'place_ring{_ring}_lift_deg', 0.0)
        self.declare_parameter('place_safe_lift_deg', 0.0)
        self.declare_parameter('stack_lift_deg', 0.0)         # 待标定：叠一层 lift 增量°
        self.declare_parameter('place_gripper_open_deg', 120.0)
        self.declare_parameter('place_settle_ms', 800)
        # ---- 载物盘三个料位角度（tray_slot 指令查表；实测填入 vehicle.yaml）----
        self.declare_parameter('tray_slot1_deg', 0.0)
        self.declare_parameter('tray_slot2_deg', 60.0)
        self.declare_parameter('tray_slot3_deg', 120.0)
        # ---- color_pick / vision_pick 的抓取位姿 ----
        # 真值来自视觉程序 JSON（每个物料动态不同）；下面两组参数是"不动视觉
        # 程序"的调节手段：
        #   pick_default_* ：视觉 JSON 缺字段时的兜底值（默认全 0）；
        #   pick_*_offset  ：加在视觉输出上的整体修正（机械标定/系统偏差用，
        #                    比如实际总偏低 5° 就设 pick_lift_offset_deg=5）。
        self.declare_parameter('pick_default_yaw_deg', 0.0)
        self.declare_parameter('pick_default_lift_deg', 0.0)
        self.declare_parameter('pick_default_extend_deg', 0.0)
        self.declare_parameter('pick_safe_lift_deg', 0.0)
        self.declare_parameter('pick_gripper_open_deg', 120.0)
        self.declare_parameter('pick_gripper_close_deg', 35.0)
        self.declare_parameter('pick_settle_ms', 800)
        self.declare_parameter('pick_yaw_offset_deg', 0.0)
        self.declare_parameter('pick_lift_offset_deg', 0.0)
        self.declare_parameter('pick_extend_offset_deg', 0.0)

        self.arrive_tol_xy = float(self.get_parameter('arrive_tol_xy').value)
        self.arrive_tol_yaw = float(self.get_parameter('arrive_tol_yaw').value)
        self.arrive_timeout_s = float(self.get_parameter('arrive_timeout_s').value)
        self.turn_settle_s = float(self.get_parameter('turn_settle_s').value)
        self.turn_hold_s = float(self.get_parameter('turn_hold_s').value)
        self.gimbal_tol_deg = float(
            self.get_parameter('gimbal_arrive_tol_deg').value)
        self.gimbal_timeout_s = float(
            self.get_parameter('gimbal_arrive_timeout_s').value)
        sequence_file = str(self.get_parameter('sequence_file').value)
        self.default_script_timeout = float(self.get_parameter('script_timeout_s').value)
        self.calibrate_timeout_s = float(self.get_parameter('calibrate_timeout_s').value)

        # ---- 发布的话题（本节点 -> 其他节点） ----
        self._pub_pose = self.create_publisher(Pose2D, '/cmd_pose', 10)                     # 位姿目标
        self._pub_servo = self.create_publisher(Float32MultiArray, '/cmd_servo', 10)        # 舵机 [夹爪,托盘,相机]
        self._pub_gimbal = self.create_publisher(Float32MultiArray, '/cmd_gimbal', 10)      # 云台 [yaw,lift,extend]
        self._pub_enable = self.create_publisher(Bool, '/cmd_enable', 10)                   # 使能
        self._pub_state = self.create_publisher(String, '/motion_state', 10)                # 进度状态(给上位机/屏幕看)
        # 控制权占用标志：True 期间手柄（joy_control）除失能键外全部让位。
        # 跑序列时按 10Hz 心跳重复发，手柄那边超时 0.6s 没收到就自动恢复手动，
        # 避免本节点崩溃后把手柄永久锁死。
        self._pub_active = self.create_publisher(Bool, '/motion_active', 10)

        # ---- 订阅的话题（其他节点 -> 本节点） ----
        self.create_subscription(Odometry, '/odom', self._cb_odom, 10)          # 里程计：判断到位
        self.create_subscription(String, '/motion_load', self._cb_load, 10)     # 上位机下发序列
        self.create_subscription(Bool, '/motion_stop', self._cb_stop, 10)       # 上位机叫停
        # 圆环视觉矫正（calibrate 指令）：把目标发给 ring_fix 节点，等它回结果
        self._pub_ring_target = self.create_publisher(Pose2D, '/ring_fix/target', 10)
        self.create_subscription(String, '/ring_fix/result', self._cb_ring_fix, 10)
        # 任务码（qr_scanner 节点）：指令里的 $b1c1$ 等占位符在执行时用它的值替换
        self.create_subscription(String, '/task_code_decoded',
                                 self._cb_task_code, 10)
        # 云台实测角：[yaw, lift, extend]。gimbal_yaw/lift/ext 相对指令要靠它起算，
        # 因为 color_pick 期间是视觉子程序自己在动云台，状态机并不知道最终停在哪。
        self.create_subscription(Float32MultiArray, '/gimbal_state',
                                 self._cb_gimbal_state, 10)

        # ---- 内部状态 ----
        self.odom = None            # 最新里程计 (x, y, theta)，单位 米/米/弧度
        self.commands = []          # 指令序列（list of dict）
        self.index = -1             # 当前执行到第几条（-1 = 还没开始）
        # 状态机状态：
        #   idle 空闲 / stepping 连续执行 / moving 平移中 / settling 到点停稳
        #   turning 原地转向 / waiting 等延时 / proc 等子程序 / fixing 等圆环视觉矫正
        self.phase = 'idle'
        self.deadline = 0.0         # delay 指令的到期时间
        self.move_deadline = 0.0    # 当前阶段（走/停稳/转）的超时时间

        # 圆环视觉矫正（calibrate 指令）等待状态
        self._fix_result = None     # ring_fix 节点回传的 JSON（/ring_fix/result）
        self._fix_deadline = 0.0    # 矫正超时时刻

        # 任务码占位符表：{'b1c1': 4, 'b1p1': 3, ...}，来自 /task_code_decoded
        self._task_params = {}

        # 绝对位姿目标（世界系）：从"当前里程计"出发，逐条指令累计
        self.tx = 0.0
        self.ty = 0.0
        self.tyaw = 0.0
        # 行进段锁定的车头朝向：出发那一刻的里程计 yaw。平移期间 POSE 帧的
        # theta 一直发它，F407 航向环把车头锁住，车只平移不转。
        self.move_yaw = 0.0
        # 转向到位后开始计时的时刻；必须连续稳定 turn_hold_s 才放行。
        self.yaw_stable_since = 0.0
        # 到达当前目标时是否必须等车头也转到位。goto 可以带 arrive_yaw=False：
        # 中间航点只走平移段，不转向、也不等朝向；最后一个航点才原地转。
        self.wait_yaw = True

        # 舵机角度镜像（F407 上电默认：夹爪90 托盘0 相机90），供 plate/cam 单独设值时补齐整组
        self.servo = [90.0, 0.0, 90.0]

        # 云台角度镜像 [yaw, lift, extend]
        #   gimbal_actual：/gimbal_state 回传的实测角
        #   gimbal_target：状态机这边认为"已经命令到位"的角度，供相对指令累加。
        #                   None = 还没同步，下一条相对指令用实测角起算。
        # 为什么要 target 而不是直接用 actual：`gimbal_lift -20` 连写两条时，
        # 第一条还没物理到位就跑第二条，用 actual 会两条都算成 -20（只降一次）。
        self.gimbal_actual = [0.0, 0.0, 0.0]
        self.gimbal_target = None
        # 单轴相对微调的"等到位"状态（phase == 'gimbal' 时有效）
        #   轴号 0=yaw 1=lift 2=extend；_gimbal_wait_deg 是该轴的目标角
        self._gimbal_wait_axis = -1
        self._gimbal_wait_deg = 0.0
        self._gimbal_wait_tol = 2.0
        self.gimbal_deadline = 0.0

        # ---- 动作序列里启动的子程序（script / align / vision_pick）----
        self._proc = None              # subprocess.Popen
        self._proc_lines = queue.Queue()   # 后台线程把 stdout 行丢进来
        self._proc_kind = ''           # script / align / vision_pick
        self._proc_cmd = {}            # 当前子程序对应的指令字典
        self._proc_deadline = 0.0      # 超时时刻
        self._proc_result = None       # 从 stdout / result_file 解析出的 JSON
        self._proc_thread = None       # stdout 读取线程

        # 若有启动参数指定的序列文件，先加载（不自动开始，等上位机下发 start）
        if sequence_file:
            self._load_file(sequence_file)

        # 执行器推进频率。/gimbal_state 只有 20Hz，所以超过 20Hz 意义不大；
        # 提频的收益是"到位判断更跟手"（10Hz = 最多多等 100ms）。
        tick_hz = max(1.0, float(self.get_parameter('tick_hz').value))
        self._timer = self.create_timer(1.0 / tick_hz, self._tick)
        self.get_logger().info('状态机就绪，等待 /motion_load 下发指令序列')

    # ---------------- 订阅回调 ----------------

    def _cb_odom(self, msg: Odometry):
        """里程计：只记录 x、y 和朝向角 theta（从四元数里反解出来）。"""
        q = msg.pose.pose.orientation
        # 绕 Z 轴的偏航角 = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
        theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y, theta)

    def _cb_gimbal_state(self, msg: Float32MultiArray):
        """云台实测角 [yaw, lift, extend]（F407 每帧回传）。

        第一次收到（或刚被重置）时顺手把"目标镜像"对齐到实测值，
        这样相对指令永远是从云台现在的位置起算。
        """
        if len(msg.data) < 3:
            return
        self.gimbal_actual = [float(msg.data[0]), float(msg.data[1]),
                              float(msg.data[2])]
        if self.gimbal_target is None:
            self.gimbal_target = list(self.gimbal_actual)

    def _gimbal_target(self):
        """取相对指令的累加基准；没有实测值时退回全 0。"""
        if self.gimbal_target is None:
            self.gimbal_target = list(self.gimbal_actual)
        return self.gimbal_target

    def _cb_load(self, msg: String):
        """上位机下发序列：解析 JSON，立刻从"当前所在位置"开始执行。"""
        self._load_json(msg.data)

    def _cb_stop(self, msg: Bool):
        """上位机叫停 / 手柄 B 键强制停：停在原地（把当前位置设为目标=锁定），状态机回到 idle。

        已经停了就直接返回：手柄 B 键和上位机 stop 可能连着来，
        重复处理会多发一次 POSE。
        """
        if bool(msg.data) and self.phase != 'idle':
            self._kill_proc()                            # 停止时把视觉/脚本子进程也收掉
            self._fix_result = None                      # 挂起的圆环矫正结果一并作废
            self.phase = 'idle'
            self.index = -1
            self._pub_active.publish(Bool(data=False))   # 交还控制权给手柄
            self._publish_state('已停止')
            # 把当前位置设为目标位姿发给 F407，让它"锁"在原地不动
            if self.odom is not None:
                self._publish_pose(self.odom[0], self.odom[1], self.odom[2])

    def _cb_ring_fix(self, msg: String):
        """ring_fix 节点的圆环矫正结果（JSON 字符串），_tick 里消费。"""
        try:
            data = json.loads(msg.data)
        except Exception:  # noqa: BLE001
            data = {'ok': False, 'reason': f'结果不是合法 JSON：{msg.data[:80]}'}
        self._fix_result = data if isinstance(data, dict) else {'ok': False}

    def _cb_task_code(self, msg: String):
        """qr_scanner 的任务码：展开成 {b1c1: 颜色, b1p1: 位置, ...} 平铺表。"""
        try:
            data = json.loads(msg.data)
        except Exception:  # noqa: BLE001
            return
        if not isinstance(data, dict):
            return
        flat = {}
        for i, v in enumerate(data.get('batch1_colors') or []):
            flat[f'b1c{i + 1}'] = v
        for i, v in enumerate(data.get('batch1_places') or []):
            flat[f'b1p{i + 1}'] = v
        for i, v in enumerate(data.get('batch2_colors') or []):
            flat[f'b2c{i + 1}'] = v
        for i, v in enumerate(data.get('batch2_places') or []):
            flat[f'b2p{i + 1}'] = v
        if flat and flat != self._task_params:
            self.get_logger().info(f'任务码就绪，占位符可用：{flat}')
        self._task_params = flat

    def _resolve_token(self, value, what: str):
        """把 '$b1c1$' 占位符换成任务码数值；普通值原样返回。缺任务码抛 KeyError。"""
        if isinstance(value, str):
            m = re.fullmatch(r'\s*\$\s*(\w+)\s*\$\s*', value)
            if m:
                key = m.group(1)
                if key in self._task_params:
                    return self._task_params[key]
                raise KeyError(f'${key}$（任务码还没扫到？）')
        return value

    def _ring_place_cmd(self, number: int, layer: int) -> dict:
        """由环号(1~3)和层(0/1)查表得到 place 宏参数（三环角度独立标定，见
        vehicle.yaml 注释；车必须已 calibrate 对位且不动）。层 1 = 同环角度 +
        lift 抬高 stack_lift_deg（叠在第一批物料上方）。"""
        gp = self.get_parameter
        n = int(number)
        return {
            'cmd': 'place',
            'yaw_deg': float(gp(f'place_ring{n}_yaw_deg').value),
            'lift_deg': float(gp(f'place_ring{n}_lift_deg').value)
                         + layer * float(gp('stack_lift_deg').value),
            'safe_lift_deg': float(gp('place_safe_lift_deg').value),
            'extend_deg': float(gp(f'place_ring{n}_extend_deg').value),
            'gripper_open': float(gp('place_gripper_open_deg').value),
            'gripper_close': 35.0,
            'settle_ms': int(gp('place_settle_ms').value),
        }

    # ---------------- 序列加载 ----------------

    def _load_file(self, path: str):
        """从文件读指令序列（启动参数指定时用）。"""
        try:
            with open(path, 'r', encoding='utf-8') as f:
                self._load_json(f.read())
        except Exception as e:
            self.get_logger().error(f'读序列文件失败 {path}: {e}')

    def _load_json(self, text: str):
        """解析一段 JSON 文本为指令序列，并立刻开始执行。"""
        try:
            data = json.loads(text)
        except Exception as e:
            self._publish_state(f'JSON 解析失败: {e}')
            self.get_logger().error(f'JSON 解析失败: {e}')
            return
        # 支持两种写法：直接是命令数组，或 {"commands": [...]}
        self.commands = data['commands'] if isinstance(data, dict) else data
        if not isinstance(self.commands, list):
            self._publish_state('序列格式错误：应为命令数组')
            return
        # 抓取/放置宏展开成基本指令，状态机本体不用认识宏
        self.commands = self._expand_macros(self.commands)
        # 新序列从"云台现在实际在哪"开始累加相对指令：两次运行之间可能用手柄
        # 调过云台，不重新对齐的话第一条 gimbal_lift 会跳回上次的旧位置。
        self.gimbal_target = None
        self._publish_state(f'已加载 {len(self.commands)} 条指令，等待开始')
        self.get_logger().info(f'加载序列：{len(self.commands)} 条')
        self._start()

    def _start(self):
        """开始执行：把绝对目标位姿初始化为"当前里程计位置"，从第 0 条开始。"""
        if self.odom is None:
            self._publish_state('还没收到里程计(F407 未回传)，无法开始，请检查串口')
            self.get_logger().error('无里程计，无法开始')
            return
        self._kill_proc()                           # 上一条序列残留的脚本先收掉
        self._fix_result = None                     # 挂起的圆环矫正结果一并作废
        self.tx, self.ty, self.tyaw = self.odom   # 从当前真实位置出发
        self.yaw_stable_since = 0.0
        self.index = -1
        self.phase = 'stepping'                     # 进入执行循环
        self._pub_active.publish(Bool(data=True))   # 立刻占控制权，别等到第一个 _tick
        self._publish_state('开始执行')

    # ---------------- 状态机主循环 ----------------

    def _tick(self):
        """每 0.1 秒调一次，推进状态机。"""
        if self.phase == 'idle':
            return
        self._pub_active.publish(Bool(data=True))   # 占用控制权心跳（手柄靠它判断要不要让位）
        if self.phase == 'proc':
            # 等子程序跑完。完成时 _poll_proc 会把 phase 改成 stepping（继续）
            # 或 moving（视觉校准后重新下发目标，等车到位）。
            if not self._poll_proc():
                return
            if self.phase != 'stepping':
                return
        if self.phase == 'fixing':
            # 等 ring_fix 节点完成圆环对位 + 里程计重置
            if self._fix_result is None:
                if self._now() > self._fix_deadline:
                    self._publish_state(
                        f'[{self.index+1}/{len(self.commands)}] 视觉矫正超时'
                        f'（{self.calibrate_timeout_s:.0f}s），跳过（里程计未重置）')
                    self.phase = 'stepping'
                return
            result = self._fix_result
            self._fix_result = None
            if result.get('ok'):
                # 里程计已被 ring_fix 重置；目标位姿同步到矫正后的真实位姿，
                # 后续指令自动活在修正坐标系里（f407_bridge 负责双向换算）。
                self.tx = float(result.get('x_m', self.tx))
                self.ty = float(result.get('y_m', self.ty))
                self.tyaw = float(result.get('yaw_rad', self.tyaw))
                self._publish_state(
                    f'[{self.index+1}/{len(self.commands)}] 视觉矫正完成：'
                    f'里程计已重置到 ({self.tx:+.3f},{self.ty:+.3f})m '
                    f'{math.degrees(self.tyaw):+.1f}°')
            else:
                self._publish_state(
                    f'[{self.index+1}/{len(self.commands)}] 视觉矫正失败：'
                    f'{result.get("reason", "未知原因")}，跳过（里程计未重置）')
            self.phase = 'stepping'
            return
        if self.phase == 'moving':
            if not self._arrived_xy():   # 位置还没到，继续平移
                if self.move_deadline and self._now() > self.move_deadline:
                    self._abort_timeout()   # 超时：锁定当前位置，交还控制权
                    return
                # 周期重发目标位姿：F407 收到任何 VEL 帧都会清掉 PosTarget.Valid，
                # 重发能保证偶发的速度帧（比如 B 键失能那一帧）不会让闭环中断。
                # 注意 theta 发 move_yaw（出发朝向），行进段不转车头。
                self._publish_pose(self.tx, self.ty, self.move_yaw)
                return
            # 位置到点了。不需要转 / 朝向已经够准 → 直接下一条；
            # 否则先停稳，再原地转向。
            if not self.wait_yaw or self._arrived_yaw():
                self.phase = 'stepping'
            else:
                self._publish_pose(self.tx, self.ty, self.move_yaw)
                self.phase = 'settling'
                self.move_deadline = self._now() + max(0.0, self.turn_settle_s)
                self._publish_state(
                    f'位置到位，停稳 {max(0.0, self.turn_settle_s):.1f}s 后原地转向 '
                    f'{math.degrees(self.tyaw):+.0f}°')
                return
        elif self.phase == 'settling':
            if self._now() < self.move_deadline:   # 还在停稳
                self._publish_pose(self.tx, self.ty, self.move_yaw)
                return
            # 停稳结束：下发最终朝向，车原地转。
            self._publish_pose(self.tx, self.ty, self.tyaw)
            self.phase = 'turning'
            self.move_deadline = self._now() + self.arrive_timeout_s
            self.yaw_stable_since = 0.0
            self._publish_state(f'原地转向 {math.degrees(self.tyaw):+.0f}°')
            return
        elif self.phase == 'turning':
            if self._arrived_yaw():
                # 进入容差只是"可能到了"：F407 可能还带着角速度，odom 噪声
                # 也可能恰好扫过容差。必须连续 turn_hold_s 都在容差内才放行，
                # 否则下一段平移会和残余转动叠在一起（看起来就是没转到位就开走）。
                if self.yaw_stable_since <= 0.0:
                    self.yaw_stable_since = self._now()
                if self._now() - self.yaw_stable_since >= max(0.0, self.turn_hold_s):
                    self.yaw_stable_since = 0.0
                    self.phase = 'stepping'
                else:
                    self._publish_pose(self.tx, self.ty, self.tyaw)
                    return
            else:
                self.yaw_stable_since = 0.0   # 又转出容差了，重新计时
                if self.move_deadline and self._now() > self.move_deadline:
                    self._abort_timeout()
                    return
                self._publish_pose(self.tx, self.ty, self.tyaw)
                return
        elif self.phase == 'waiting':
            if self._now() < self.deadline:   # 还没到时间，继续等
                return
            self.phase = 'stepping'
        elif self.phase == 'gimbal':
            # 等单轴相对微调到位：只盯被命令的那一根轴，偏差进容差就放行。
            # 不返回时 phase 已经是 'stepping'，本拍就直接走下面的 while 继续下一条。
            idx = self._gimbal_wait_axis
            if 0 <= idx < 3:
                label = ('yaw', 'lift', 'extend')[idx]
                err = self.gimbal_actual[idx] - self._gimbal_wait_deg
                if abs(err) <= self._gimbal_wait_tol:
                    self.phase = 'stepping'
                elif self._now() > self.gimbal_deadline:
                    self._publish_state(
                        f'云台 {label} 到位超时：目标 {self._gimbal_wait_deg:+.1f}°'
                        f'，实测 {self.gimbal_actual[idx]:+.1f}°（差 {err:+.1f}°，'
                        f'容差 ±{self._gimbal_wait_tol:.1f}°），继续执行后续指令')
                    self.get_logger().warning(
                        f'云台 {label} 到位超时（差 {err:+.1f}°）')
                    self.phase = 'stepping'
                else:
                    return
            else:
                self.phase = 'stepping'

        # 连续执行：不阻塞的指令（舵机/云台/注释…）一条接一条，遇到阻塞的（走/转/延时）就停下等
        while self.phase == 'stepping':
            self.index += 1
            if self.index >= len(self.commands):
                self._finish()
                return
            self._execute(self.commands[self.index])
            if self.phase in ('moving', 'turning'):
                # 从这一条开始计时。本阶段结束/转段时会重新计时。
                self.move_deadline = self._now() + self.arrive_timeout_s
            else:
                self.move_deadline = 0.0

    def _execute(self, cmd: dict):
        """执行一条指令。返回后 self.phase 会是：
           - moving/turning/waiting：阻塞型，等完成
           - stepping：非阻塞型，继续下一条
        moving 到位后若还要转，会由 _tick 转成 settling → turning。
        """
        # 默认要求位置和朝向都到位；goto 可以显式关掉朝向等待。
        self.wait_yaw = True
        name = cmd.get('cmd')
        if name == 'move':            # 前进/后退
            d = float(cmd.get('dist_m', 0.0))
            self.tx += d * math.cos(self.tyaw)
            self.ty += d * math.sin(self.tyaw)
            # 行进段锁住出发时的车头朝向，只平移不转（tyaw 本身没变）。
            self.move_yaw = self._hold_yaw()
            self._publish_pose(self.tx, self.ty, self.move_yaw)
            self._publish_state(f'[{self.index+1}/{len(self.commands)}] 前进 {d:+.2f}m')
            self.phase = 'moving'
        elif name == 'strafe':        # 横向平移（正=左）
            d = float(cmd.get('dist_m', 0.0))
            self.tx += d * math.cos(self.tyaw + math.pi / 2.0)
            self.ty += d * math.sin(self.tyaw + math.pi / 2.0)
            self.move_yaw = self._hold_yaw()
            self._publish_pose(self.tx, self.ty, self.move_yaw)
            self._publish_state(f'[{self.index+1}/{len(self.commands)}] 横移 {d:+.2f}m')
            self.phase = 'moving'
        elif name == 'turn':          # 原地转：位置不变，直接进转向段
            d = math.radians(float(cmd.get('deg', 0.0)))
            self.tyaw += d
            self._publish_pose(self.tx, self.ty, self.tyaw)
            self._publish_state(f'[{self.index+1}/{len(self.commands)}] 转向 {cmd.get("deg", 0):+.0f}°')
            self.yaw_stable_since = 0.0
            self.phase = 'turning'
        elif name == 'goto':          # 绝对位姿（里程计世界系，米/度）
            self.tx = float(cmd.get('x_m', self.tx))
            self.ty = float(cmd.get('y_m', self.ty))
            if cmd.get('yaw_deg') is not None:
                self.tyaw = math.radians(float(cmd['yaw_deg']))
            # 中间航点 arrive_yaw=False：只走平移段，不转向、也不等朝向
            self.wait_yaw = bool(cmd.get('arrive_yaw', True))
            # 第一段：位置去终点，朝向锁在出发朝向 → 先走到，再转
            self.move_yaw = self._hold_yaw()
            self._publish_pose(self.tx, self.ty, self.move_yaw)
            self._publish_state(
                f'[{self.index+1}/{len(self.commands)}] 去 '
                f'({self.tx:+.2f},{self.ty:+.2f})m '
                f'{math.degrees(self.tyaw):+.0f}°')
            self.phase = 'moving'
        elif name == 'delay':         # 原地等待
            self.deadline = self._now() + float(cmd.get('ms', 0)) / 1000.0
            self._publish_state(f'[{self.index+1}/{len(self.commands)}] 等待 {cmd.get("ms", 0)}ms')
            self.phase = 'waiting'
        elif name == 'gimbal':        # 云台三轴绝对角（不阻塞，发了就走）
            a = Float32MultiArray()
            a.data = [float(cmd.get('yaw_deg', 0.0)),
                      float(cmd.get('lift_deg', 0.0)),
                      float(cmd.get('extend_deg', 0.0))]
            self._pub_gimbal.publish(a)
            self.gimbal_target = list(a.data)   # 绝对指令：镜像直接对齐
            self._publish_state(f'[{self.index+1}/{len(self.commands)}] 云台 yaw={a.data[0]}° lift={a.data[1]}°')
            # 不阻塞：phase 保持 stepping
        elif name in ('gimbal_yaw', 'gimbal_lift', 'gimbal_ext'):
            # 单轴相对微调：只改一个轴，量是"相对当前角度加多少"。
            # 典型用法：color_pick 用视觉把物料吊到正上方之后，不需要知道 yaw
            # 具体是多少，只补一个 lift/extend 的量把夹爪降到抓取高度。
            #
            # 阻塞：发完目标进入 phase='gimbal'，等 /gimbal_state 的实测角进容差
            # 才放行下一条。所以后面不用再写 delay，也不会出现"云台还没降下去
            # 夹爪就合上了"。超时只报警、继续往下走。
            axis, label = {'gimbal_yaw': (0, 'yaw'),
                           'gimbal_lift': (1, 'lift'),
                           'gimbal_ext': (2, 'extend')}[name]
            try:
                delta = float(cmd.get('delta_deg', 0.0))
            except (TypeError, ValueError):
                delta = 0.0
            try:
                tol = abs(float(cmd['tol_deg']))
            except (KeyError, TypeError, ValueError):
                tol = self.gimbal_tol_deg
            try:
                timeout = float(cmd['timeout_ms']) / 1000.0
            except (KeyError, TypeError, ValueError):
                timeout = self.gimbal_timeout_s

            target = self._gimbal_target()
            target[axis] += delta
            a = Float32MultiArray()
            a.data = list(target)
            self._pub_gimbal.publish(a)

            self._gimbal_wait_axis = axis
            self._gimbal_wait_deg = target[axis]
            self._gimbal_wait_tol = tol
            self.gimbal_deadline = self._now() + max(0.0, timeout)
            self._publish_state(
                f'[{self.index+1}/{len(self.commands)}] 云台 {label} '
                f'{delta:+.1f}° → {target[axis]:+.1f}°，等到位（±{tol:.1f}°）')
            self.phase = 'gimbal'
        elif name == 'tray_slot':    # 载物盘转到第 N 个料位（角度查 vehicle.yaml）
            try:
                n = int(self._resolve_token(cmd.get('number'), 'number'))
            except (KeyError, ValueError) as error:
                self._publish_state(
                    f'[{self.index+1}/{len(self.commands)}] tray_slot：参数无效'
                    f'（{error}），跳过')
                return
            if not 1 <= n <= 3:
                self._publish_state(
                    f'[{self.index+1}/{len(self.commands)}] tray_slot：料位 {n} 越界'
                    '（只能 1~3），跳过')
                return
            deg = float(self.get_parameter(f'tray_slot{n}_deg').value)
            self.servo[1] = deg                     # 载物盘 = 舵机组第 2 个
            smsg = Float32MultiArray()
            smsg.data = list(self.servo)            # 整组下发，避免缺值
            self._pub_servo.publish(smsg)
            self._publish_state(
                f'[{self.index+1}/{len(self.commands)}] 载物盘 -> 料位{n} ({deg}°)')
        elif name in ('gripper', 'plate', 'cam'):   # 三个舵机
            idx = {'gripper': 0, 'plate': 1, 'cam': 2}[name]
            self.servo[idx] = float(cmd.get('deg', 0.0))
            s = Float32MultiArray()
            s.data = list(self.servo)               # 整组下发，避免缺值
            self._pub_servo.publish(s)
            self._publish_state(f'[{self.index+1}/{len(self.commands)}] {name}={self.servo[idx]}°')
        elif name == 'enable':        # 使能/失能
            b = Bool()
            b.data = bool(cmd.get('en', True))
            self._pub_enable.publish(b)
            self._publish_state(f'[{self.index+1}/{len(self.commands)}] 使能={b.data}')
        elif name == 'calibrate':     # 圆环工位视觉矫正（ring_fix 节点执行）
            try:
                rx = float(cmd.get('x_m'))
                ry = float(cmd.get('y_m'))
            except (TypeError, ValueError):
                self._publish_state(
                    f'[{self.index+1}/{len(self.commands)}] calibrate 需要圆环里程计坐标 '
                    f'x_m,y_m，参数缺失，跳过')
                return
            target = Pose2D()
            target.x = rx
            target.y = ry
            target.theta = 0.0
            self._pub_ring_target.publish(target)
            self._fix_result = None
            self._fix_deadline = self._now() + self.calibrate_timeout_s
            self._publish_state(
                f'[{self.index+1}/{len(self.commands)}] 圆环视觉矫正 → '
                f'({rx:+.3f},{ry:+.3f})m，识别对位中…')
            self.phase = 'fixing'
        elif name == 'color_pick':     # 按颜色抓物料：vision_pick 封装
            try:
                color = self._resolve_token(cmd.get('color'), 'color')
                if isinstance(color, str) and color.isdigit():
                    color = int(color)
                if isinstance(color, int) and not 1 <= color <= 6:
                    raise ValueError(f'颜色编号 {color} 越界（只能 1~6）')
            except (KeyError, ValueError) as error:
                self._publish_state(
                    f'[{self.index+1}/{len(self.commands)}] color_pick：{error}，跳过')
                return
            # 复用 vision_pick 全链路：跑视觉程序（带 --color），JSON 回来插 pick 宏
            # no_pick：color_pick 只要求云台跟随到物料上方，不自动抓——
            # 抓取/放载物盘等后续动作由序列里写死的 gimbal/gripper/plate 指令完成。
            vp = {'cmd': 'vision_pick',
                  'program': cmd.get('program')
                             or self.get_parameter('vision_program').value or '',
                  'args': (str(cmd.get('args')
                               or self.get_parameter('vision_args').value or '')
                           + f' --color {color}').strip(),
                  'timeout_s': cmd.get('timeout_s', 60.0),
                  'no_pick': True}
            self._execute(vp)
        elif name in ('number_put', 'double_put'):   # 按环号放置 / 叠放
            try:
                number = int(self._resolve_token(cmd.get('number'), 'number'))
            except (KeyError, ValueError) as error:
                self._publish_state(
                    f'[{self.index+1}/{len(self.commands)}] {name}：参数无效（{error}），跳过')
                return
            if not 1 <= number <= 3:
                self._publish_state(
                    f'[{self.index+1}/{len(self.commands)}] {name}：环号 {number} 越界'
                    '（只能 1~3），跳过')
                return
            layer = 1 if name == 'double_put' else 0
            place = self._ring_place_cmd(number, layer)
            expanded = self._macro_pick(place, 'place')
            self.commands[self.index + 1:self.index + 1] = expanded
            self._publish_state(
                f'[{self.index+1}/{len(self.commands)}] {name} → '
                f'环{number} 层{layer}（yaw={place["yaw_deg"]}° '
                f'lift={place["lift_deg"]}° ext={place["extend_deg"]}°），执行 place 宏')
        elif name == 'script':
            wait = bool(cmd.get('wait', True))
            if self._start_proc(cmd, 'script'):
                n = self.index + 1
                if wait:
                    self._publish_state(f'[{n}/{len(self.commands)}] 运行程序（等待结束）')
                    self.phase = 'proc'
                else:
                    self._publish_state(f'[{n}/{len(self.commands)}] 启动程序（后台）')
            else:
                self._publish_state(f'[{self.index+1}/{len(self.commands)}] 程序没启动起来，跳过')
        elif name == 'align':
            if self._start_proc(cmd, 'align'):
                self._publish_state(f'[{self.index+1}/{len(self.commands)}] 视觉校准：运行程序…')
                self.phase = 'proc'
            else:
                self._publish_state(f'[{self.index+1}/{len(self.commands)}] 视觉校准程序没启动，跳过')
        elif name == 'vision_pick':
            if self._start_proc(cmd, 'vision_pick'):
                self._publish_state(f'[{self.index+1}/{len(self.commands)}] 视觉识别物料…')
                self.phase = 'proc'
            else:
                self._publish_state(f'[{self.index+1}/{len(self.commands)}] 视觉识别程序没启动，跳过')
        elif name == 'comment':       # 注释，跳过
            self._publish_state(f'[{self.index+1}/{len(self.commands)}] 注释: {cmd.get("text", "")}')
        else:
            self.get_logger().warning(f'未知指令被跳过: {cmd}')

    # ---------------- 抓取 / 放置宏 ----------------

    def _expand_macros(self, commands: list) -> list:
        """把 pick / place 这类"一条顶好几条"的宏展开成基本指令。

        展开放在加载时做，状态机就不需要认识宏，也方便上位机把展开后的
        完整动作显示出来（可选）。
        """
        out = []
        for cmd in commands:
            if not isinstance(cmd, dict):
                continue
            name = cmd.get('cmd')
            if name in ('pick', 'place'):
                out.extend(self._macro_pick(cmd, name))
            else:
                out.append(cmd)
        return out

    @staticmethod
    def _macro_pick(cmd: dict, kind: str) -> list:
        """抓取宏的具体动作序列。

        参数（都有默认值，节点动作序列里可以只填想改的）：
          yaw_deg / lift_deg / extend_deg  抓取位姿
          safe_lift_deg                    抬起后的安全高度（默认 0）
          gripper_open / gripper_close     夹爪张开/夹紧角度（默认 120 / 35）
          settle_ms                        每个阶段等待时间（默认 800ms）
        """
        yaw = float(cmd.get('yaw_deg', 0.0))
        lift = float(cmd.get('lift_deg', 0.0))
        safe = float(cmd.get('safe_lift_deg', 0.0))
        ext = float(cmd.get('extend_deg', 0.0))
        g_open = float(cmd.get('gripper_open', 140.0))
        g_close = float(cmd.get('gripper_close', 35.0))
        settle = int(cmd.get('settle_ms', 800))

        def gimbal(deg_lift):
            return {'cmd': 'gimbal', 'yaw_deg': yaw,
                    'lift_deg': deg_lift, 'extend_deg': ext}

        def grip(deg):
            return {'cmd': 'gripper', 'deg': deg}

        def wait(ms=settle):
            return {'cmd': 'delay', 'ms': ms}

        if kind == 'pick':
            return [
                grip(g_open),
                gimbal(safe), wait(),
                gimbal(lift), wait(),
                grip(g_close), wait(),
                gimbal(safe),
            ]
        return [                       # place
            gimbal(safe), wait(),
            gimbal(lift), wait(),
            grip(g_open), wait(),
            gimbal(safe),
        ]

    # ---------------- 子程序（script / align / vision_pick） ----------------

    def _start_proc(self, cmd: dict, kind: str) -> bool:
        """启动一条指令对应的子程序。返回 False = 没启动（状态机跳过这条）。"""
        program = str(cmd.get('program') or
                      self.get_parameter('vision_program').value or '').strip()
        args = cmd.get('args')
        if args is None:
            args = self.get_parameter('vision_args').value or ''
        if isinstance(args, (list, tuple)):
            argv = [str(a) for a in args]
        else:
            argv = shlex.split(str(args))

        cwd = str(cmd.get('cwd') or
                  self.get_parameter('vision_cwd').value or '').strip() or None
        if cwd:
            cwd = os.path.expandvars(os.path.expanduser(cwd))
        if not program:
            self.get_logger().warning(f'{kind}: 没有配置 program，跳过')
            return False
        program = os.path.expandvars(os.path.expanduser(program))

        # 同一时刻只保留一个子程序：上一条还是后台模式没退出时，先收掉，
        # 否则 _proc 被覆盖，旧进程就再也停不下来了。
        if self._proc is not None:
            self._kill_proc()

        env = os.environ.copy()
        env.setdefault('PYTHONUNBUFFERED', '1')   # 让脚本 print 立刻出来
        try:
            self._proc = subprocess.Popen(
                [program, *argv], cwd=cwd, env=env,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1,
            )
        except Exception as e:  # noqa: BLE001 - 路径错/没权限都不该拖垮状态机
            self._proc = None
            self.get_logger().error(f'{kind}: 启动 {program} 失败：{e}')
            return False

        self._proc_lines = queue.Queue()
        self._proc_kind = kind
        self._proc_cmd = dict(cmd)
        self._proc_result = None
        try:
            timeout = float(cmd.get('timeout_s', self.default_script_timeout))
        except (TypeError, ValueError):
            timeout = self.default_script_timeout
        self._proc_deadline = time.monotonic() + max(1.0, timeout)


        self._proc_thread = threading.Thread(
            target=self._read_proc, args=(self._proc, self._proc_lines),
            daemon=True, name=f'{kind}-reader')
        self._proc_thread.start()
        self.get_logger().info(f'{kind}: 已启动 {program} {" ".join(argv)}')
        return True

    @staticmethod
    def _read_proc(proc, lines: queue.Queue) -> None:
        """后台线程：把子程序输出逐行塞进队列（不阻塞 ROS 执行器）。"""
        try:
            for line in proc.stdout:
                lines.put(line.rstrip('\n'))
        except Exception:  # noqa: BLE001 - 进程被杀时读管道会抛，忽略
            pass

    def _poll_proc(self) -> bool:
        """检查子程序是否结束。返回 True = 结束并已处理，False = 还在跑。"""
        if self._proc is None:
            self.phase = 'stepping'
            return True

        timed_out = time.monotonic() > self._proc_deadline
        if self._proc.poll() is None and not timed_out:
            return False

        if timed_out and self._proc.poll() is None:
            self.get_logger().warning(f'{self._proc_kind}: 运行超时，强制结束')
            try:
                self._proc.terminate()
                self._proc.wait(timeout=2.0)
            except Exception:  # noqa: BLE001
                try:
                    self._proc.kill()
                except Exception:  # noqa: BLE001
                    pass

        code = self._proc.poll()
        self._proc = None
        kind = self._proc_kind
        self._proc_kind = ''
        # 进程退出和读线程收完管道之间有几毫秒空档；不等它就可能漏掉
        # 最后一行 JSON（视觉结果通常在最后一行）。
        thr = self._proc_thread
        self._proc_thread = None
        if thr is not None:
            thr.join(timeout=1.0)
        self._proc_result = self._parse_proc_result()
        # 子程序（尤其 color_pick 的视觉跟随）会直接动云台，状态机的目标镜像
        # 已经对不上了。重置成 None，下一条相对指令用实测角重新起算。
        self.gimbal_target = None

        if kind == 'align':
            self._apply_align()
        elif kind == 'vision_pick':
            self._apply_vision_pick()
        else:
            self._publish_state(f'程序结束（退出码 {code}）')
            self.phase = 'stepping'
        return True

    def _parse_proc_result(self):
        """从子程序的 stdout 最后一行 / result_file 里找 JSON 对象。

        约定：脚本要回传结果时，在最后一行打印一个 JSON 对象即可，例如
            print(json.dumps({"dx_m": 0.01, "dy_m": -0.02, "dyaw_deg": 1.5}))
        也可以写进 result_file，避免 stdout 混着日志不好挑。
        """
        cmd = self._proc_cmd or {}
        path = str(cmd.get('result_file', '') or '').strip()
        if path:
            try:
                with open(os.path.expanduser(os.path.expandvars(path)),
                          'r', encoding='utf-8') as f:
                    data = json.load(f)
                if isinstance(data, dict):
                    return data
            except Exception:  # noqa: BLE001 - 没有结果文件是正常情况
                pass

        lines = []
        while True:
            try:
                lines.append(self._proc_lines.get_nowait())
            except queue.Empty:
                break
        for line in reversed(lines):
            text = line.strip()
            if not text.startswith('{'):
                continue
            try:
                data = json.loads(text)
            except Exception:  # noqa: BLE001
                continue
            if isinstance(data, dict):
                return data
        return None

    def _apply_align(self):
        """视觉矫正：把结果里的 dx/dy/dyaw 叠加到当前目标位姿，并重新下发。"""
        data = self._proc_result or {}
        try:
            dx = float(data.get('dx_m', 0.0))
            dy = float(data.get('dy_m', 0.0))
            dyaw = math.radians(float(data.get('dyaw_deg', 0.0)))
        except (TypeError, ValueError):
            data = {}
            dx = dy = dyaw = 0.0

        if not data:
            self._publish_state('视觉校准没有返回 JSON，跳过（目标位姿不变）')
            self.phase = 'stepping'
            return

        self.tx += dx
        self.ty += dy
        self.tyaw += dyaw
        # 视觉校准必须把位置和朝向都修正到位。wait_yaw 是上一条指令留下的，
        # 如果上一条是中间航点(arrive_yaw=False)，这里会被误判成"不用转"。
        self.wait_yaw = True
        # 同样先平移到位、再原地修正朝向。
        self.move_yaw = self._hold_yaw()
        self._publish_pose(self.tx, self.ty, self.move_yaw)
        self.move_deadline = time.monotonic() + self.arrive_timeout_s
        self._publish_state(
            f'视觉校准：修正 dx={dx:+.3f}m dy={dy:+.3f}m '
            f'dyaw={math.degrees(dyaw):+.1f}°，重新到位')
        self.phase = 'moving'

    def _apply_vision_pick(self):
        """视觉定点抓取：把识别出的云台角度填进 pick 宏，插到当前指令后面执行。

        位姿优先级：视觉 JSON 值 → pick_default_* 兜底；yaw/lift/extend 再统一
        加 pick_*_offset（机械标定修正，不改视觉程序就能整体平移抓取点）。
        """
        if (self._proc_cmd or {}).get('no_pick'):
            # color_pick 语义：云台已跟随到目标上方，继续执行后续指令即可。
            # 但不能只看"进程结束"就报成功——视觉超时/没找到物料/打不开摄像头
            # 时 JSON 里是 ok:false，必须原样报到界面上，否则后续夹爪指令
            # 会在云台还停在原地的情况下"抓空"，看起来却一切正常。
            data = self._proc_result or {}
            if not data:
                self._publish_state(
                    'color_pick 视觉程序没有返回 JSON（看程序输出：摄像头/路径/'
                    'DISPLAY 问题）——继续执行后续指令')
            elif not data.get('ok', True):
                self._publish_state(
                    f'color_pick 失败：{data.get("reason", "未知原因")}'
                    '——继续执行后续指令')
            else:
                self._publish_state(
                    f'云台已跟随到物料上方（color_pick，不自动抓取）：'
                    f'yaw={data.get("yaw_deg", 0):+.1f}° '
                    f'lift={data.get("lift_deg", 0):+.1f}° '
                    f'ext={data.get("extend_deg", 0):+.1f}°')
            self.phase = 'stepping'
            return
        data = self._proc_result or {}
        gp = self.get_parameter

        def _num(key, default_key):
            return float(data.get(key, gp(default_key).value))

        pick = {'cmd': 'pick',
                'yaw_deg': _num('yaw_deg', 'pick_default_yaw_deg')
                           + float(gp('pick_yaw_offset_deg').value),
                'lift_deg': _num('lift_deg', 'pick_default_lift_deg')
                            + float(gp('pick_lift_offset_deg').value),
                'safe_lift_deg': _num('safe_lift_deg', 'pick_safe_lift_deg'),
                'extend_deg': _num('extend_deg', 'pick_default_extend_deg')
                              + float(gp('pick_extend_offset_deg').value),
                'gripper_open': float(data.get(
                    'gripper_open', gp('pick_gripper_open_deg').value)),
                'gripper_close': float(data.get(
                    'gripper_close', gp('pick_gripper_close_deg').value)),
                'settle_ms': int(data.get(
                    'settle_ms', gp('pick_settle_ms').value))}

        if not data:
            self._publish_state('视觉识别没有返回 JSON，使用默认抓取位')
        else:
            self._publish_state(
                f'视觉识别：yaw={pick.get("yaw_deg", 0)}° '
                f'lift={pick.get("lift_deg", 0)}° '
                f'ext={pick.get("extend_deg", 0)}°，开始抓取')

        expanded = self._macro_pick(pick, 'pick')
        self.commands[self.index + 1:self.index + 1] = expanded
        self.phase = 'stepping'

    def _kill_proc(self):
        """立即结束正在跑的子程序（停止 / 超时 / 任务结束）。"""
        proc = self._proc
        self._proc = None
        self._proc_kind = ''
        self._proc_thread = None
        if proc is None:
            return
        try:
            proc.terminate()
            proc.wait(timeout=1.0)
        except Exception:  # noqa: BLE001
            try:
                proc.kill()
            except Exception:  # noqa: BLE001
                pass

    def _abort_timeout(self):
        """一条走/转/去指令超时：别让状态机永远卡住（比如车被顶住、轮子打滑）。

        把当前位置设成目标位姿，让 F407 原地锁住，然后交还控制权给手柄。
        剩下的指令不再执行——半路停下比硬着头皮跑完安全。
        """
        self._kill_proc()
        self._fix_result = None
        self.phase = 'idle'
        self.index = -1
        self.move_deadline = 0.0
        if self.odom is not None:
            self.tx, self.ty, self.tyaw = self.odom
            self._publish_pose(self.odom[0], self.odom[1], self.odom[2])
        self._pub_active.publish(Bool(data=False))
        self._publish_state(f'超时停止：{self.arrive_timeout_s:.0f}s 没到位')
        self.get_logger().warning(
            f'指令超时（>{self.arrive_timeout_s:.0f}s 未到位），已停止并交还手柄')

    # ---------------- 工具函数 ----------------

    def _arrived_xy(self) -> bool:
        """位置是否到达绝对目标 (tx, ty)。行进段只判位置，不判朝向。"""
        if self.odom is None:
            return False
        ox, oy, _ = self.odom
        dx = ox - self.tx
        dy = oy - self.ty
        return (dx * dx + dy * dy) < self.arrive_tol_xy ** 2

    def _arrived_yaw(self) -> bool:
        """车头是否转到绝对目标 tyaw。"""
        if self.odom is None:
            return False
        return abs(self._angle_diff(self.odom[2], self.tyaw)) < self.arrive_tol_yaw

    def _hold_yaw(self) -> float:
        """行进段要锁住的车头朝向 = 出发那一刻的里程计 yaw。

        还没有里程计（理论上一开始就会拦）时退回当前目标朝向。
        """
        return self.odom[2] if self.odom is not None else self.tyaw

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        """两个角度的差，结果归一化到 (-π, π]（处理 ±180° 环绕）。"""
        return (a - b + math.pi) % (2.0 * math.pi) - math.pi

    def _publish_pose(self, x, y, yaw):
        p = Pose2D()
        p.x = float(x)
        p.y = float(y)
        p.theta = float(yaw)
        self._pub_pose.publish(p)

    def _publish_state(self, text: str):
        s = String()
        s.data = text
        self._pub_state.publish(s)

    def _finish(self):
        self._kill_proc()
        self.phase = 'idle'
        self._pub_active.publish(Bool(data=False))       # 交还控制权给手柄
        self._publish_state('全部完成')

    @staticmethod
    def _now() -> float:
        """当前时间（秒）。"""
        import time
        return time.monotonic()


def main(args=None):
    rclpy.init(args=args)
    node = MotionPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
