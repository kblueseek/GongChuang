#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""轮径标定 demo：让车按编码器里程走一段固定距离，用尺子量实际距离，反算轮半径。

为什么这样标：
    F407 的里程计不是"速度×时间"，而是编码器计数：
        WHEEL_M_PER_COUNT = 2π × WHEEL_RADIUS_M / 65536
    （见 UniversalCar/Task/ChassisTask.c 的 Chassis_OdometryUpdate）
    所以 ODOM 帧里的 x/y 就是"轮子转了多少圈"换算出来的距离。
    轮径不准 → 里程计按同样的比例偏大/偏小，位置闭环永远差一截。

    让车走到 ODOM 路程 = --distance（默认 1.000 m）后停车，按编码器算的圈数：
        N = reported / (2π × radius_now)
    再用尺子量出实际距离 D_actual，真实"等效半径"：
        radius_true = radius_now × D_actual / reported

    把结果填回 UniversalCar/Core/Inc/RobotConfig.h 的 WHEEL_RADIUS_M，重烧即可。
    （ZDT 直驱、轮子直接装在电机轴上时，等效半径就是滚动半径；
      如果中间还有减速比，这个比例会一起被吸收进等效半径。）

用法（先停掉 ROS launch：蓝牙和 LoRa 串口不能同时被两个程序占用）：
    python3 scripts/calibrate_wheel.py --ble-address 36:10:00:38:45:F2
    # 按提示放好车、按回车；车走完 1 m 后输入尺子量到的距离

    # 也可以不交互，直接给实际距离：
    python3 scripts/calibrate_wheel.py --ble-address 36:10:00:38:45:F2 --actual 1.05

注意：
  * 地面要平，车要能直线走（航向环工作）。跑偏超过 --max-yaw-deg 会自动停车。
  * 默认 0.1 m/s，慢一点尺子好量、轮子滑移小。
  * 协议里 +Vx 在这台车上实测是后退；标定只关心距离，正负都行。
    想让车远离你，可以加 --speed -0.1。
  * 退出 / 超时 / Ctrl-C 都会先发 0 速度再失能。
"""

import argparse
import math
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.join(_HERE, "..", "ros2_ws", "src", "f407_bridge")
if os.path.isdir(_PKG):
    sys.path.insert(0, _PKG)

from f407_bridge.pi_comm import (  # noqa: E402
    BleLink,
    ERR_PI_OFFLINE,
    PiCommLink,
    RPT_ODOM,
    RPT_STATUS,
    parse_odom,
    parse_status,
)


def wrap_angle(a):
    """把角度卷到 [-π, π]。"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class Car:
    """LoRa 收遥测 + 蓝牙发指令，并记住最新 ODOM / STATUS。"""

    def __init__(self, down, up):
        self.down = down
        self.up = up
        self.odom = None            # (x m, y m, yaw rad)
        self.status = None          # (state, err, motor_mask)
        self.last_odom_t = 0.0
        self._last_hb = 0.0
        self._last_en = 0.0
        self._last_vel = 0.0
        self._vel = (0.0, 0.0, 0.0)

    @property
    def linked(self):
        return (self.up.connected and self.odom is not None
                and time.monotonic() - self.last_odom_t < 1.0)

    @property
    def pi_online(self):
        return self.status is not None and not (self.status[1] & ERR_PI_OFFLINE)

    def poll(self):
        for cmd, data in self.down.poll():
            try:
                if cmd == RPT_ODOM:
                    self.odom = parse_odom(data)
                    self.last_odom_t = time.monotonic()
                elif cmd == RPT_STATUS:
                    self.status = parse_status(data)
            except Exception:
                pass

    def tick(self, now, enable, vel=None):
        """按各自节流周期发心跳 / 使能 / 速度。"""
        if now - self._last_hb >= 0.2:
            self._last_hb = now
            self.up.send_heartbeat()
        if enable and now - self._last_en >= 1.0:
            self._last_en = now
            self.up.send_enable(True)
        if vel is not None:
            self._vel = vel
        if now - self._last_vel >= 0.1:
            self._last_vel = now
            self.up.send_vel(*self._vel)


def wait_link(car, timeout=15.0):
    """等蓝牙连上 + 收到 ODOM。"""
    deadline = time.monotonic() + timeout
    last_report = 0.0
    while time.monotonic() < deadline:
        car.poll()
        if car.linked:
            return True
        now = time.monotonic()
        if not car.up.connected and now - last_report >= 2.0:
            last_report = now
            if car.up.last_scan_report:
                print("[..] 还没连上蓝牙，扫到的设备：")
                print(car.up.last_scan_report)
        time.sleep(0.05)
    return False


def run(args, car):
    print(f"LoRa 收：{args.lora} @ {args.baud}")
    print("等待蓝牙连接和 F407 回传 ...")
    if not wait_link(car):
        print("[!] 超时：没连上蓝牙或没收到 ODOM。检查模组地址 / 供电 / 串口。")
        return

    # 发心跳，等 F407 把"树莓派离线"清掉
    print("等待 F407 在线 ...")
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        car.poll()
        car.tick(time.monotonic(), enable=False)
        if car.pi_online:
            break
        time.sleep(0.02)
    if not car.pi_online:
        print("[!] F407 仍报树莓派离线（err bit0），检查蓝牙上行是否真的发出去。")
        return

    state, err, mask = car.status
    print(f"[OK ] status state={state} err=0x{err:02X} motor=0x{mask:02X}")
    if (mask & 0x0F) != 0x0F:
        names = ["FL(前左,1)", "FR(前右,2)", "RL(后左,3)", "RR(后右,4)"]
        missing = [names[i] for i in range(4) if not (mask & (1 << i))]
        print("\n[中止] 底盘 4 个电机没有全部在线，标定无法进行。")
        print(f"        在线：{', '.join(n for n in names if mask & (1 << (names.index(n)))) or '无'}")
        print(f"        掉线：{', '.join(missing)}")
        print("        里程计要 4 轮编码器都在线才会启动（ODOM 不会动），所以脚本直接退出。")
        print("        先查后轮 CAN 接线 / 供电 / 电机 ID 是否为 3、4（出厂默认常是 1）。")
        return

    print("\n把车放到起点，车头朝直线方向；卷尺零点对准车上同一个参考点（比如车头中心）。")
    try:
        input("准备好后按回车开始（Ctrl-C 取消）...")
    except (EOFError, KeyboardInterrupt):
        print("\n取消")
        return

    # 重新发心跳 / 使能，等电机上锁、ODOM 稳定
    t = time.monotonic()
    while time.monotonic() - t < 0.8:
        car.poll()
        car.tick(time.monotonic(), enable=True, vel=(0.0, 0.0, 0.0))
        time.sleep(0.02)
    if car.odom is None:
        print("[!] 没有 ODOM 数据")
        return
    start = car.odom
    print(f"起点 ODOM: x={start[0]:+.4f} m y={start[1]:+.4f} m "
          f"yaw={math.degrees(start[2]):+.1f}°")
    print(f"开始运动：目标 {args.distance:.3f} m，速度 {args.speed:.3f} m/s\n")

    prev_x, prev_y = start[0], start[1]
    path = 0.0
    t0 = time.monotonic()
    stop_reason = "到达目标"
    while True:
        now = time.monotonic()
        car.poll()
        car.tick(now, enable=True, vel=(args.speed, 0.0, 0.0))
        if car.odom is not None:
            x, y, yaw = car.odom
            path += math.hypot(x - prev_x, y - prev_y)
            prev_x, prev_y = x, y
            if path >= args.distance:
                break
            if abs(wrap_angle(yaw - start[2])) > math.radians(args.max_yaw_deg):
                stop_reason = "跑偏"
                break
        if now - t0 > args.timeout:
            stop_reason = "超时"
            break
        if not car.up.connected:
            stop_reason = "蓝牙断开"
            break
        time.sleep(0.02)

    # 停车：先发 0 速度，保持使能，等车滑停；这段时间的里程也要算进去
    print(f"\n停车（{stop_reason}），等车停稳 ...")
    t = time.monotonic()
    while time.monotonic() - t < 1.5:
        car.poll()
        car.tick(time.monotonic(), enable=True, vel=(0.0, 0.0, 0.0))
        if car.odom is not None:
            x, y, _ = car.odom
            path += math.hypot(x - prev_x, y - prev_y)
            prev_x, prev_y = x, y
        time.sleep(0.02)
    end = car.odom if car.odom is not None else start

    # 停车后失能（失能不松轴，F407 仍会发 0 速度锁住）
    car.up.send_vel(0.0, 0.0, 0.0)
    car.up.send_enable(False)

    reported = path
    straight = math.hypot(end[0] - start[0], end[1] - start[1])
    if reported <= 1e-6:
        print("[!] ODOM 路程为 0：车没动，或编码器 / 里程计没工作。")
        return

    revs = reported / (2.0 * math.pi * args.radius)

    print("\n===== 标定结果 =====")
    print(f"触发距离  : {args.distance:.4f} m")
    print(f"编码器路程: {reported:.4f} m（累计路径 = 弧长）")
    print(f"直线距离  : {straight:.4f} m（起点→终点 = 弦长）")
    print(f"估计圈数  : {revs:.3f} 圈（65536 计数/圈）")
    print(f"当前半径  : {args.radius:.6f} m（直径 {args.radius * 2000.0:.2f} mm）")

    # 关键前提：车必须走直线。跑偏时里程计记的是弧长、尺子量的是弦长，
    # 两者差一个“转弯”的量，任何标定结论都无效。
    if abs(straight - reported) / reported > 0.02:
        print("\n[!] 车跑偏了：直线距离(弦) 明显小于 编码器路程(弧)。")
        print("    这次标定无效 —— 里程计记的是轮子滚过的弧长，尺子量的是弦长，"
              "本来就不能直接比。")
        print("    你之前看到的“走 1m 实际 0.42m”，就是这个：车一边走一边转，"
              "轮子滚了 1m 弧长，但位移只有 0.42m。")
        print("    轮径/里程计本身没问题，先解决“走不直”（方向映射 / 航向保持）。")
        print("    等车能直线走 1m 左右、直线≈路程，再回来标。")
        return

    # 车走直了：直线≈路程，尺子读数 = 实际滚动距离，比例才有意义。
    actual = args.actual
    if actual is None:
        try:
            text = input("\n用尺子量车上同一参考点走过的距离（米）: ").strip()
            actual = float(text)
        except (ValueError, EOFError, KeyboardInterrupt):
            print("没有有效的实际距离，未计算。")
            return
    if actual <= 0:
        print("实际距离必须 > 0")
        return

    radius_new = args.radius * actual / reported
    factor = reported / actual        # 里程计比真实距离放大了多少倍
    print(f"\n尺子实测  : {actual:.4f} m")
    print(f"修正系数  : {actual / reported:.6f}  （等效半径 = 当前半径 × 这个数）")
    print(f"等效半径  : {radius_new:.6f} m（直径 {radius_new * 2000.0:.2f} mm）")
    print(f"里程计放大: {factor:.4f} 倍")

    # 等效半径只有在“误差纯粹来自轮径”时才成立。若算出来的直径和实物差很多，
    # 说明真正的误差在“每圈计数”上（电机有减速箱 / 同步带轮比 / 不是 Emm 65536/圈），
    # 这时改半径会把速度指令也带偏，必须在固件里加减速比。
    if factor > 1.15 or factor < 0.85:
        print("\n[!] 等效半径算出来是", f"{radius_new * 2000.0:.1f} mm，"
              "和实物轮径差得远 —— 说明误差不在轮径，而在“每圈计数”。")
        print("    物理轮径仍按实物填 WHEEL_RADIUS_M，另外在固件里加减速比：")
        print(f"    #define WHEEL_GEAR_RATIO        {factor:.4f}f   /* 电机轴→轮 减速比 */")
        print("    并把 ChassisTask.c 两处都乘上它：")
        print("      WHEEL_M_PER_COUNT = 2πR / (65536 * WHEEL_GEAR_RATIO)")
        print("      rpm = v / (2πR) * 60 * WHEEL_GEAR_RATIO * 方向")
        print("    否则只改半径会同时把速度指令带偏（车会跑快/跑慢同样倍数）。")
        print("    先确认机械：轮子是直接装电机轴上，还是经过减速箱/同步带轮？")
    else:
        print("\n把 UniversalCar/Core/Inc/RobotConfig.h 改成：")
        print(f"#define WHEEL_RADIUS_M          {radius_new:.6f}f")
        print("重烧后再跑一次：ODOM 路程应该和尺子读数基本一致。")


def parse_args():
    p = argparse.ArgumentParser(
        description="轮径标定：走固定 ODOM 距离，用尺子反算轮半径")
    p.add_argument("--lora", default="/dev/ttyAMA2", help="LoRa 串口")
    p.add_argument("--baud", type=int, default=115200, help="LoRa 波特率")
    p.add_argument("--ble-address", default="", help="蓝牙模组 MAC（留空=自动扫描）")
    p.add_argument("--ble-name", default="", help="蓝牙名字关键字（可选）")
    p.add_argument("--write-uuid", default="", help="写入特征值 UUID（留空=自动探测）")
    p.add_argument("--distance", type=float, default=1.0,
                   help="ODOM 路程达到多少米后停车（默认 1.0）")
    p.add_argument("--speed", type=float, default=0.1,
                   help="前进速度 m/s（负值=反方向，默认 0.1）")
    p.add_argument("--radius", type=float, default=0.03925,
                   help="当前 WHEEL_RADIUS_M，要和固件里一致（默认 0.03925）")
    p.add_argument("--actual", type=float, default=None,
                   help="尺子实测距离 m（不给就交互输入）")
    p.add_argument("--max-yaw-deg", type=float, default=20.0,
                   help="跑偏超过多少度自动停车（默认 20）")
    p.add_argument("--timeout", type=float, default=60.0,
                   help="最长运行时间 s（默认 60）")
    return p.parse_args()


def main():
    args = parse_args()

    down = PiCommLink(device=args.lora, baudrate=args.baud)
    try:
        up = BleLink(address=args.ble_address, name=args.ble_name,
                     write_uuid=args.write_uuid, log=print)
    except Exception as e:
        print(f"[!] 蓝牙不可用：{e}")
        down.close()
        return

    car = Car(down, up)
    try:
        run(args, car)
    except KeyboardInterrupt:
        print("\n中断")
    finally:
        try:
            car.up.send_vel(0.0, 0.0, 0.0)
            time.sleep(0.05)
            car.up.send_enable(False)
        except Exception:
            pass
        print("\n正在断开蓝牙并退出 ...")
        car.up.close()
        car.down.close()


if __name__ == "__main__":
    main()
