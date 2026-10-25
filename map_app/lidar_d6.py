#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""国科光芯 D6 激光雷达驱动（AA 55 协议）+ 一个同接口的模拟器。

================================ 协议 ================================

一帧数据（雷达 → 主机），长度 = 10 + 3N 字节。以下全部按官方文档
《COIN-D6 激光雷达数据格式标准说明 V1.0》第 4~5 页，别再自己猜：

    偏移  长度  含义
    0     2    帧头 AA 55
    2     1    M&T      bit0=T：0x00 点云包 / 0x01 起始包；bit7:1 是扫描频率
    3     1    LSN      本包采样点数（起始包为 1）
    4     2    FSA 起始角 小端，bit0 是校验位固定为 1
    6     2    LSA 结束角 小端，同上
    8     2    CS  校验码   双字节异或（在这 10 字节头里，不在帧尾！）
    10    3N   采样点数据

    每点 3 字节 = Si_L, Si_2nd, Si_H：
        距离(mm) = Si_H*64 + (Si_2nd >> 2)              ← 14bit，单位 mm
        强度     = (Si_2nd & 0x03)*64 + (Si_L >> 2)     ← 8bit

    角度：Angle_fsa = (FSA>>1)/64  → 即 FSA/128，单位「度」
          中间点按 (LSA-FSA)/(LSN-1) 线性插值

    ⚠ 两个曾经解错、导致距离放大 4 倍的坑（2026-10-24 修正）：
      1) 点数据起点是 **10**，不是 8。CS 校验码在头里，不在帧尾。
         帧长 8+3N+2 和 10+3N 数值相同，所以同步一直"看起来正常"，
         但数据整体错位 2 字节。
      2) 距离是「高字节×64 + 中字节>>2」，不是「低字节 | 中字节<<8」。
         这两个字节约等于 4 倍关系，所以画出图形状是对的、尺寸全错 4 倍。
    结论：塔克 STM32 例程 ax_laser.c:292/306 的
      ((u16)buf[12+i*3]<<6) + (buf[11+i*3]>>2)
    **是标准公式，完全正确**。之前我说它错，是我错了。

命令（主机 → 雷达）：启动 AA 55 F0 0F，停止 AA 55 F5 0A。

================================ 模拟器 ================================

没有雷达也能把整套流程跑通：SimLidar 按 0.9° 步进从雷达位置打光线，
和场上的圆形物体求最近交点，加上噪声和量化，输出和真雷达一模一样的数据。
调界面、调检测参数、给别人演示都用它。
"""

from __future__ import annotations

import math
import threading
import time

FRAME_HEAD = b"\xAA\x55"
CMD_START = b"\xAA\x55\xF0\x0F"
CMD_STOP = b"\xAA\x55\xF5\x0A"
ANGLE_SCALE = 128.0          # 1 计数 = 1/128 度


# ==================== 真雷达 ====================


class D6Lidar:
    """串口读 D6。回调在后台线程里触发，别在回调里直接碰界面控件。"""

    def __init__(self, port="COM3", baud=230400, on_frame=None, on_log=None):
        self.port = port
        self.baud = baud
        self.on_frame = on_frame
        self.on_log = on_log or (lambda s: None)
        self._serial = None
        self._thread = None
        self._run = False
        self.error = ""
        # 帧体检：用来判断"角度标度对不对"
        #   step_avg 正常应该 ≈ 0.9°/点（400 点/圈）
        #   如果 ≈ 14° 或负数 → 起始/结束角的方向读反了
        self.stats = {"frames": 0, "step_sum": 0.0, "pts": 0, "rings": 0,
                      "a0_first": None, "a0_last": None, "n_last": 0}
        self.direction = None            # None=未判定，"cw"/"ccw"
        self._votes = {"cw": 0, "ccw": 0}

    def open(self):
        try:
            import serial                     # pyserial
        except ImportError:
            self.error = "没装 pyserial，先执行：pip install pyserial"
            return False
        try:
            self._serial = serial.Serial(self.port, self.baud, timeout=0.05)
        except Exception as e:
            self.error = f"打开 {self.port} 失败：{e}"
            return False
        self._serial.write(CMD_START)
        self._run = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self.on_log(f"{self.port} @ {self.baud} 已启动")
        return True

    def close(self):
        self._run = False
        try:
            if self._serial and self._serial.is_open:
                self._serial.write(CMD_STOP)
                time.sleep(0.05)
                self._serial.close()
        except Exception:
            pass
        self.on_log("串口已关闭")

    # ---------- 内部 ----------

    def _loop(self):
        buf = bytearray()
        while self._run:
            try:
                chunk = self._serial.read(512)
            except Exception as e:
                self.on_log(f"读串口异常：{e}")
                break
            if chunk:
                buf += chunk
            while True:
                i = buf.find(FRAME_HEAD)
                if i < 0:
                    if len(buf) > 4:
                        del buf[:-2]                 # 只留最后两字节，防止帧头被截断
                    break
                if i > 0:
                    del buf[:i]
                if len(buf) < 8:
                    break
                n = buf[3]
                if n == 0 or n > 64:
                    del buf[:2]                      # 假的帧头，跳过去
                    continue
                total = 10 + 3 * n          # 头 10 字节（含校验码）+ 3 字节/点
                if len(buf) < total:
                    break
                frame = bytes(buf[:total])
                del buf[:total]
                pts, ring, a0, a1 = self._parse(frame, n)
                if pts is not None and self.on_frame:
                    self.on_frame(pts, ring)

    def step_deg(self):
        """平均角步长（度/点）。正常 ≈ 0.9；≈14 说明角方向读反；0 说明只有单点帧。"""
        f = self.stats["frames"]
        return (self.stats["step_sum"] / f) if f else 0.0

    def _parse(self, frame, n):
        """解一帧。返回 (点列表, 是否整圈帧, 起始角, 结束角)。

        角度方向（顺时针/逆时针）是**投票**定出来的，不能只看一帧：
          · 逆时针雷达：正常帧 a1>a0，只在跨 0° 那一帧 a1<a0
          · 顺时针雷达：正常帧 a1<a0，只在跨 0° 那一帧 a1>a0
        两种情况的多数派正好相反，攒够 12 帧就能判。

        ⚠ 这里以前是 `if step<0: step += 360/(n-1)` —— 顺时针雷达会被算成
        13.5°/点的步长（真实 0.9°），一整帧横跨 338°，物体被拉宽十几倍，
        表现就是"比例不对 + 方向不对"。现在按方向分别处理。
        """
        a0 = (frame[4] | (frame[5] << 8)) / ANGLE_SCALE
        a1 = (frame[6] | (frame[7] << 8)) / ANGLE_SCALE
        ring = (frame[2] == 0xC9)

        if n >= 2:
            if a1 < a0:
                self._votes["cw"] += 1
            else:
                self._votes["ccw"] += 1
            if self.direction is None and sum(self._votes.values()) >= 12:
                self.direction = ("cw" if self._votes["cw"] > self._votes["ccw"]
                                  else "ccw")
        # 步长直接用"最短弧"解卷绕：
        #   正常帧 |d| 很小（约 21°），跨 0° 的那帧 |d| 接近 360°。
        #   所以 |d|>180 就是跨了一圈，加/减 360 拉回来。
        # 这样无论雷达是顺时针还是逆时针都对，而且 step 的符号本身就带了方向，
        # 不需要先"判定方向"再算——之前的写法在判定出方向之前会算错。
        if n <= 1:
            step = 0.0
        else:
            d = a1 - a0
            if d > 180.0:
                d -= 360.0
            elif d < -180.0:
                d += 360.0
            step = d / (n - 1)

        # ⚠ 点数据从偏移 10 开始，不是 8：
        #   AA 55 | M&T | LSN | FSA_L FSA_H | LSA_L LSA_H | CS_L CS_H  ← 头 10 字节
        #   其中 CS 校验码在头里，所以帧长 = 10 + 3×LSN（和原来算的 8+3N+2 数值相同，
        #   但数据起点差了 2 字节 —— 这就是之前距离全错的根因）。
        #
        # 官方《数据格式标准说明V1.0》第5页：
        #   Distance  = Si_H*64 + (Si_2nd>>2)          [mm, 14bit]
        #   Intensity = (Si_2nd & 0x03)*64 + (Si_L>>2)
        pts = []
        for i in range(n):
            o = 10 + i * 3
            if o + 2 >= len(frame):
                break
            s_l, s_2nd, s_h = frame[o], frame[o + 1], frame[o + 2]
            d = s_h * 64 + (s_2nd >> 2)
            inten = (s_2nd & 0x03) * 64 + (s_l >> 2)
            if d <= 0:
                continue
            pts.append((a0 + step * i, float(d), inten))

        st = self.stats                      # 帧体检（放在这里，脱离串口也能统计）
        st["frames"] += 1
        st["pts"] += len(pts)
        if ring:
            st["rings"] += 1
        st["n_last"] = n
        if st["a0_first"] is None:
            st["a0_first"] = a0
        st["a0_last"] = a0
        st["step_sum"] += abs(step)
        return pts, ring, a0, a1


# ==================== 模拟器 ====================


class SimLidar:
    """假的 D6：从雷达位置打光线，输出 (角度, 距离, 强度)。

    obstacles: [{"x":..,"y":..,"r":..,"intensity":..}, ...] 世界坐标 mm
    """

    def __init__(self, field, obstacles=None, on_frame=None, on_log=None,
                 hz=10.0, points_per_rev=400, noise_mm=6.0):
        self.field = field
        self.obstacles = obstacles or []
        self.on_frame = on_frame
        self.on_log = on_log or (lambda s: None)
        self.hz = hz
        self.n = points_per_rev
        self.noise = noise_mm
        self._run = False
        self._thread = None
        self._k = 0

    def step_deg(self):
        return 360.0 / self.n

    def open(self):
        self._run = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self.on_log("模拟雷达已启动（无硬件）")
        return True

    def close(self):
        self._run = False
        self.on_log("模拟雷达已停止")

    def _loop(self):
        period = 1.0 / self.hz
        while self._run:
            t0 = time.time()
            pts = []
            lx, ly = self.field.lidar_x, self.field.lidar_y
            step = 360.0 / self.n
            for i in range(self.n):
                ang = i * step
                d = self._ray(lx, ly, self.field.lidar_yaw + self.field.angle_sign * ang)
                if d is None:
                    continue
                if self.noise:
                    # 固定伪随机，避免同一个角度每圈抖得不一样
                    d += self.noise * math.sin(i * 12.9898 + self._k * 0.7)
                pts.append((ang, d, 120))
            self._k += 1
            if self.on_frame:
                self.on_frame(pts, self._k % 1 == 0)
            dt = period - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)

    def _ray(self, x0, y0, ang_deg):
        """射线与所有圆形障碍求最近交点距离；没打到返回 None。"""
        a = math.radians(ang_deg)
        dx, dy = math.cos(a), math.sin(a)
        best = None
        for ob in self.obstacles:
            ox, oy, r = ob["x"], ob["y"], ob["r"]
            # |P0 + t·d − O|² = r²
            fx, fy = x0 - ox, y0 - oy
            b = 2 * (fx * dx + fy * dy)
            c = fx * fx + fy * fy - r * r
            disc = b * b - 4 * c
            if disc < 0:
                continue
            sq = math.sqrt(disc)
            t = (-b - sq) / 2.0
            if t <= 0:
                continue
            if best is None or t < best:
                best = t
        if best is None or best > 8000:
            return None
        return best
