#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""旋转扫描：车原地转、雷达边转边扫，把每帧点云按当拍车体位姿放回场地系。

============================ 为什么转车管用 ============================

雷达装在车上，车体两侧（机械臂立柱、扫码相机支架）把左右都挡住，只能看到
车头一个扇区，宽度 W < 180°。遮挡物跟着车一起转，所以"转车"并不是换个角度
看同一个死区 —— 而是把这个**可见扇区**搬到别的朝向上，在场地系里拼起来。

从启停区角落看，整个场地（车体 400mm 以外那条车道）在雷达初始坐标系里的
方位角跨度是 **−14° … +126°，共 140°**（按 field.json 的 2400×2400 场地、
雷达装在车头前 150mm 即 (900,1050)、yaw=180° 实算）。不是想当然的 90° ——
贴右墙那条车道把角度撑宽了 50°。

     视野 W   不转    转30°   转45°   转60°   转90°
      60°   28.0%   62.7%   75.8%   89.8%   99.9%
      90°   46.2%   75.8%   89.8%   98.7%  100%
     120°   62.7%   89.8%   98.7%   99.9%  100%
     150°   75.8%   98.7%   99.9%  100%    100%

  · 转 90° 基本满覆盖（60° 视野也到 99.9%），保守推荐
  · 最小转角 ≈ max(0, 140° − W) 取整：W=150→35°，120→50°，90→65°，60→80°
  · **方向反了等于白转**：从 yaw=180 顺时针转是走 180→90，正好背离场地，
    覆盖率一点不涨。转车方向必须朝向场心，见 prefer_sign()。
  · 雷达离车中心越远（dx 越大），近处张角越大，需要的转角越多 —— 改
    mount_rel.dx 之后这张表要重算。

转一圈也覆盖不了的地方只有一处：**车周围那一圈**。遮挡随车转，近场永远是
盲区，半径大约等于车体+机构外接圆。这部分靠规划里的 robot.radius_mm 兜底，
不指望雷达看见。

============================ 坐标口径 ============================

和三处代码严格一致，别另立一套：

  · 场地系（field.json / mapping.py）：原点场心，+x 向右，+y 向上，
    角度逆时针为正，单位 mm / 度
  · 车体位姿：车头 = 车体 +x，角度 = 场地系里的朝向（度）
  · 雷达设备角：D6 顺时针为正，mapping.FieldModel.angle_sign = -1 负责翻成
    数学角。世界方位角 = 雷达yaw + angle_sign × 设备角

============================ 拖尾 ============================

车在转，雷达一圈 100ms（D6 10Hz）。按"每帧取一次车头角"变换，帧内仍有
speed × 0.1s 的角度畸变：

    30°/s → 3°/帧 → 3m 处横向 157mm
    60°/s → 6°/帧 → 3m 处横向 314mm

所以界面上把角速度限在 45°/s 以内，默认 30°/s。要更快就得按点插值，
那需要雷达逐点时间戳，D6 的包里没有，只能按 FSA/LSA 线性推。
"""

from __future__ import annotations

import math

# 默认参数（界面上可以改，改完存在 field.json 的 scan.spin 里）
DEFAULT_SPEED_DPS = 30.0      # 角速度 °/s
DEFAULT_TARGET_DEG = 90.0     # 总转角 °
DEFAULT_FOV_DEG = 150.0       # 雷达可见扇区宽度（不知道就先按 150 保守填）
DEFAULT_SIGN = -1.0           # -1 = 顺时针（俯视），+1 = 逆时针
SIGN_AUTO = 0.0               # 交给 prefer_sign() 按"朝场心"判方向


def prefer_sign(robot_x: float, robot_y: float, lidar_yaw_deg: float,
                target_x: float = 0.0, target_y: float = 0.0) -> float:
    """选一个让雷达轴朝场心转的方向：+1 逆时针，-1 顺时针。

    启停区1（雷达 yaw=180，场心在 225°）→ +1；启停区2 → -1。
    方向选反的表现不是"扫得慢"，而是**覆盖率一点不涨**（车往空白的一侧转），
    所以默认就按场心判，别让人手猜。
    """
    bearing = math.degrees(math.atan2(target_y - robot_y, target_x - robot_x))
    return 1.0 if wrap_deg(bearing - lidar_yaw_deg) >= 0.0 else -1.0


def wrap_deg(d: float) -> float:
    """角度卷绕到 (-180, 180]。"""
    d = math.fmod(d + 180.0, 360.0)
    if d < 0.0:
        d += 360.0
    return d - 180.0


def rot_xy(x: float, y: float, deg: float) -> tuple:
    """绕原点逆时针旋转 deg 度。"""
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return c * x - s * y, s * x + c * y


def abs_delta(from_deg: float, to_deg: float) -> tuple:
    """从 from 转到 to 的最短弧：返回 (方向, 角度绝对值)。

    例：180° → 270° 得 (+1, 90)；180° → 90° 得 (-1, 90)。
    `to` 指的是**雷达轴朝向**。雷达装车头且 0° 朝车头时它与车头同角；
    装车尾则差 180°。二者关系由 mount_rel.yaw_deg 决定，换算见 lidar_pose()。
    """
    d = wrap_deg(to_deg - from_deg)
    return (1.0 if d >= 0.0 else -1.0), abs(d)


def sweep_radius(robot_size_mm: float) -> float:
    """车原地转一圈扫过的半径（正方形车体的外接圆）＝ 半边长 × √2。

    300mm 车体 → 212mm。车中心离墙小于这个值就顶墙，转不动。
    """
    return abs(float(robot_size_mm)) / 2.0 * math.sqrt(2.0)


def clearance_mm(field_hw: float, field_hh: float,
                 robot_x: float, robot_y: float) -> float:
    """车中心到最近一面墙的距离（mm，方形场地，返回负值表示已越界）。"""
    return min(field_hw - abs(robot_x), field_hh - abs(robot_y))


def lidar_pose(robot_x: float, robot_y: float, robot_yaw_deg: float,
               dx_mm: float = 0.0, dy_mm: float = 0.0,
               dyaw_deg: float = 0.0) -> tuple:
    """车体位姿 + 雷达相对车体的安装偏置 → 雷达在场地系的位姿 (x, y, yaw_deg)。

    dx/dy 是雷达在**车体系**里的位置（车头 +x、车左 +y，mm），
    dyaw 是雷达 0 度方向相对车头的夹角（度）。装在车中心、0 度朝车头就是 0,0,0。
    """
    ox, oy = rot_xy(dx_mm, dy_mm, robot_yaw_deg)
    return robot_x + ox, robot_y + oy, wrap_deg(robot_yaw_deg + dyaw_deg)


def point_from_scan(pose: tuple, angle_deg: float, dist_mm: float,
                    angle_sign: float = -1.0) -> tuple:
    """一个回波点（设备角, 距离）→ 场地系坐标。pose 由 lidar_pose() 给出。"""
    x, y, yaw = pose
    a = math.radians(yaw + angle_sign * angle_deg)
    return x + dist_mm * math.cos(a), y + dist_mm * math.sin(a)


def sector_points(pose: tuple, fov_deg: float, radius_mm: float,
                  step_deg: float = 3.0) -> list:
    """雷达可见扇区的边界点（给界面画扇形用）。返回 [(x, y), ...]，首点是雷达位。"""
    x, y, yaw = pose
    fov = max(1.0, min(float(fov_deg), 359.0))
    n = max(2, int(math.ceil(fov / max(step_deg, 0.5))))
    out = [(x, y)]
    for k in range(n + 1):
        a = math.radians(yaw - fov / 2.0 + fov * k / n)
        out.append((x + radius_mm * math.cos(a), y + radius_mm * math.sin(a)))
    return out


class SpinScan:
    """原地旋转扫描的状态机（只算角度，不碰 ROS / Tk，方便单测）。

    用法：
        s = SpinScan(speed_dps=30, target_deg=90, sign=-1)
        s.start(yaw0)
        每次拿到新里程计：
            wz, done = s.update(yaw, dt)
            if done: 停车
            else:    发 /cmd_vel(0, 0, radians(wz))
    """

    def __init__(self, speed_dps: float = DEFAULT_SPEED_DPS,
                 target_deg: float = DEFAULT_TARGET_DEG,
                 sign: float = DEFAULT_SIGN,
                 tol_deg: float = 1.5,
                 slow_zone_deg: float = 15.0):
        self.speed = abs(float(speed_dps))
        self.target = abs(float(target_deg))
        self.sign = -1.0 if sign < 0 else 1.0
        self.tol = abs(float(tol_deg))
        self.slow_zone = abs(float(slow_zone_deg))

        self._yaw = None
        self._last_yaw = None
        self.progress = 0.0      # 有符号累计转角（逆时针为正，°）
        self.stall_s = 0.0       # 连续多久没有实际转动（s）
        self.done = False

    # ---------- 生命周期 ----------

    def start(self, yaw_deg: float) -> None:
        self._yaw = wrap_deg(yaw_deg)
        self._last_yaw = self._yaw
        self.progress = 0.0
        self.stall_s = 0.0
        self.done = False

    # ---------- 每拍 ----------

    def update(self, yaw_deg: float, dt_s: float) -> tuple:
        """喂当前车头角（度）和距上一拍的间隔（s），返回 (wz_dps, done)。

        wz_dps 已带方向：正 = 逆时针。done 为真时 wz 恒 0。
        """
        if self._yaw is None:
            self.start(yaw_deg)
            return 0.0, False

        d = wrap_deg(yaw_deg - self._yaw)
        self._yaw = wrap_deg(yaw_deg)
        self.progress += d

        # 判卡死：车没转（IMU 离线 / 未使能 / 被 /cmd_pose 抢走）时不能一直发速度
        if abs(d) < 0.05:
            self.stall_s += max(0.0, float(dt_s))
        else:
            self.stall_s = 0.0

        remain = self.target - self.sign * self.progress
        if remain <= self.tol:
            self.done = True
            return 0.0, True

        spd = self.speed * self.sign
        # 最后一段按剩余量收油，减少过冲（F407 那边还有加速度曲线，不会突变）
        if self.slow_zone > 0.0 and remain < self.slow_zone:
            spd *= max(0.25, remain / self.slow_zone)
        return spd, False

    # ---------- 只读 ----------

    @property
    def turned(self) -> float:
        """已转过的角度（带方向，正 = 逆时针），°。"""
        return self.sign * self.progress

    @property
    def remaining(self) -> float:
        return max(0.0, self.target - self.turned)

    def summary(self) -> str:
        return (f'已转 {self.turned:+.1f}° / 目标 {self.target:.0f}°'
                f'（{self.speed:.0f}°/s {"逆时针" if self.sign > 0 else "顺时针"}）')


# ==================== 自测：python3 spin_scan.py ====================

def _selftest() -> int:
    bad = 0

    def chk(name, got, want, tol=1e-6):
        nonlocal bad
        ok = (abs(got - want) <= tol) if isinstance(got, float) else (got == want)
        if not ok:
            bad += 1
            print(f'  ✗ {name}: got {got!r}, want {want!r}')
        return ok

    print('[1] rot_xy 基本旋转')
    for deg, x, y in ((0, 1.0, 0.0), (90, 0.0, 1.0), (180, -1.0, 0.0),
                      (270, 0.0, -1.0), (360, 1.0, 0.0)):
        gx, gy = rot_xy(1.0, 0.0, deg)
        chk(f'rot_xy(1,0,{deg}).x', gx, x, 1e-9)
        chk(f'rot_xy(1,0,{deg}).y', gy, y, 1e-9)

    print('[2] 雷达安装偏置：车在原点朝 +x，雷达装在车头前 100mm')
    p = lidar_pose(0, 0, 0, 100, 0, 0)
    chk('pose.x', p[0], 100.0)
    chk('pose.y', p[1], 0.0)
    chk('pose.yaw', p[2], 0.0)

    print('[3] 车转 90° 后，同一个安装偏置跟着转')
    p = lidar_pose(0, 0, 90, 100, 0, 0)
    chk('pose.x', p[0], 0.0, 1e-9)
    chk('pose.y', p[1], 100.0, 1e-9)

    print('[4] 设备角 → 场地系：雷达在原点朝 +x，D6 顺时针为正（sign=-1）')
    # 设备 0°  → 世界 0°   → (1000, 0)
    x, y = point_from_scan((0, 0, 0), 0.0, 1000.0, -1.0)
    chk('dev0.x', x, 1000.0, 1e-9); chk('dev0.y', y, 0.0, 1e-9)
    # 设备 90°（顺时针）→ 世界 -90° → (0, -1000)
    x, y = point_from_scan((0, 0, 0), 90.0, 1000.0, -1.0)
    chk('dev90.x', x, 0.0, 1e-9); chk('dev90.y', y, -1000.0, 1e-9)

    # 雷达轴朝 -x（yaw=180，就是启停区那个安装角），设备角 45°：
    #   世界角 = 180 + (-1)×45 = 135°（左上方向），y 是正的。
    #   别想成 225° —— 设备角顺时针为正，45° 是把扇区往"上"扳。
    print('[5] 雷达朝 -x（yaw=180）设备角 45° → 世界 135°')
    x, y = point_from_scan((0, 0, 180.0), 45.0, 1000.0, -1.0)
    chk('x', x, -707.1067811865476, 1e-6)
    chk('y', y, 707.1067811865476, 1e-6)

    print('[6] 扇区边界点：中心线与半径对得上')
    sp = sector_points((0, 0, 0), 90.0, 100.0)
    chk('首点=雷达位', sp[0], (0.0, 0.0))
    chk('点数', len(sp), 32)          # 90/3 = 30 段 → 31 点 + 1
    chk('-45°端点.y', sp[1][1], -70.71067811865476, 1e-6)
    chk('+45°端点.y', sp[-1][1], 70.71067811865476, 1e-6)

    print('[7] SpinScan：顺时针转 90°，每拍 1.5°')
    s = SpinScan(speed_dps=30.0, target_deg=90.0, sign=-1.0)
    s.start(0.0)
    yaw, t, steps = 0.0, 0.0, 0
    while True:
        yaw -= 1.5
        wz, done = s.update(yaw, 0.05)
        steps += 1
        if done or steps > 500:
            break
    chk('done', done, True)
    chk('已转角≈90', abs(abs(s.turned) - 90.0) <= 1.5, True)
    chk('末拍速度=0', wz, 0.0)

    print('[8] 跨 ±180 卷绕：从 179° 顺时针转 200°')
    s = SpinScan(speed_dps=30.0, target_deg=200.0, sign=-1.0)
    s.start(179.0)
    yaw, steps = 179.0, 0
    while steps < 1000:
        yaw = wrap_deg(yaw - 1.0)
        _, done = s.update(yaw, 0.05)
        steps += 1
        if done:
            break
    chk('done', done, True)
    # 容差 1.5°：转满 199 拍（差 1°）就判到位，不会为了凑整数多顶一下。
    # 想要"精确 90°"就把 tol_deg 设小（0.5），代价是末尾可能来回蹭一下。
    chk('steps 在 198~201 之间', 198 <= steps <= 201, True)

    print('[9] 卡死检测：车一分钟不动')
    s = SpinScan(target_deg=90.0, sign=-1.0)
    s.start(10.0)
    for _ in range(20):
        _, done = s.update(10.0, 0.1)
    chk('stall_s≈2.0', s.stall_s, 2.0, 1e-6)
    chk('未完成', done, False)

    print('[10] 逆时针 +1 方向')
    s = SpinScan(speed_dps=20.0, target_deg=30.0, sign=+1.0)
    s.start(0.0)
    wz, _ = s.update(0.0, 0.1)
    chk('wz 为正', wz > 0, True)
    s.update(10.0, 0.5)
    chk('turned=+10', s.turned, 10.0, 1e-9)

    print('[11] 自动方向：启停区1 逆时针，启停区2 顺时针')
    # 启停区1 (1050,1050) 雷达 yaw=180 → 场心方位 225° → 要 +90° → 逆时针
    chk('start1', prefer_sign(1050.0, 1050.0, 180.0), 1.0)
    # 启停区2 (1050,-1050) → 场心方位 135° → 要 -45° → 顺时针
    chk('start2', prefer_sign(1050.0, -1050.0, 180.0), -1.0)
    # 雷达已经对着场心：任选一个都行，但必须是 ±1，不能是 0
    chk('toward center', abs(prefer_sign(1050.0, 1050.0, 225.0)), 1.0)

    print('[12] 绝对朝向：正左 180° → 正下 270°')
    sign, mag = abs_delta(180.0, 270.0)
    chk('方向=逆时针', sign, 1.0)
    chk('转角=90', mag, 90.0, 1e-9)
    sign, mag = abs_delta(180.0, 90.0)
    chk('反向=顺时针', sign, -1.0)
    chk('角=90', mag, 90.0, 1e-9)
    # 跨 0°：350° → 10° 应该是 +20°，不是 -340°
    sign, mag = abs_delta(350.0, 10.0)
    chk('跨零方向', sign, 1.0)
    chk('跨零角度', mag, 20.0, 1e-9)

    print('[13] 原地转的外接圆 / 离墙余量')
    chk('300mm 车体扫过半径', sweep_radius(300.0), 212.1320343559643, 1e-9)
    chk('启停区1 离墙', clearance_mm(1200.0, 1200.0, 1050.0, 1050.0), 150.0)
    chk('差多少顶墙', clearance_mm(1200.0, 1200.0, 1050.0, 1050.0)
        - sweep_radius(300.0) < 0, True)

    print()
    print('全部通过' if bad == 0 else f'{bad} 项失败')
    return 0 if bad == 0 else 1


if __name__ == '__main__':
    raise SystemExit(_selftest())
