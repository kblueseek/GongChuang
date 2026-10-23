#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""13 步流程状态机骨架 —— 只调视觉，暂不下发串口。

和 `vision_flow_debug.py` 的分工
--------------------------------
| | 干什么 | 主要用途 |
|---|---|---|
| `vision_flow_debug.py` | **查看器**：按键切模式，看画面 | 调参、看效果 |
| 本文件 | **状态机骨架**：每步有「进入→等待就绪→完成」三段，能在就绪后自动推进 | 以后接串口下发的底座 |

**接串口的做法**：在 `Step.on_act()` 里加下发代码（`/cmd_pose`、`/cmd_gimbal` 等），
其余结构不用动。

每一步的三段
------------
| 相位 | 干什么 |
|---|---|
| `REQUEST` | 进入本步：切视觉模式（并让检测器 `reset()` 清掉上一轮状态） |
| `WAIT`    | 等视觉报就绪；**连续 N 帧**都就绪才算稳定，避免被抖动骗过 |
| `DONE`    | 本步完成。手动模式下等按键；自动模式下立刻进下一步 |

按键
----
    空格 / n   推进到下一步（本步没就绪也强制跳）
    p          退回上一步
    1-9 0      直接跳到第 1~13 步（0 = 行车）
    a          切换「自动推进」：本步就绪后自动进下一步
    r          重跑当前步（重置检测器与计时）
    s          存一张当前调试图
    q / Esc    退出

用法
----
    python3 flow_state_machine.py --top /dev/video2 --front /dev/video0
    python3 flow_state_machine.py --no-front          # 没接前视相机
    python3 flow_state_machine.py --auto              # 启动即自动推进
"""

import argparse
import sys
import time
from enum import Enum
from pathlib import Path

import cv2

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

# 复用 vision_flow_debug 里已经写好的检测器封装和显示工具，避免重复实现。
# 以后要把两个工具拆开，把 MODE_CLASSES / STEPS 抽到独立的 vision_modes.py 即可。
import vision_flow_debug as vfd


# ============================================================ 就绪判据

# 从各模式 process() 返回的结果字典里，取哪个键、满足什么条件算"就绪"。
# ⚠ 键名必须和 vision_flow_debug.py 里各模式返回的字典一致；改了那边要同步这里。
# ⚠⚠ 这些字符串会画到画面上，所以用英文 —— cv2.putText 不支持中文，会全变成 "?"
READY_RULE = {
    "qr":        ("task_code",  lambda v: bool(v) and v != "(not confirmed)"),
    "turntable": ("found",      lambda v: v == "yes"),
    "material":  ("reportable", lambda v: v == "yes"),
    "ring":      ("found",      lambda v: v == "yes"),
    "stacked":   ("found",      lambda v: v == "yes"),
    "lane":      ("found",      lambda v: v == "yes"),
}

REQUIRE_READY_FRAMES = 3     # 连续多少帧就绪才算本步完成
STEP_TIMEOUT_S = 12.0        # 单步超时（秒），超了就在面板上标红


class Phase(Enum):
    REQUEST = "REQ"
    WAIT = "WAIT"
    DONE = "DONE"


# ============================================================ 状态机

class Step:
    """一个流程步骤。视觉部分调 mode_key 对应的检测器。"""

    def __init__(self, num, title, mode_key, note):
        self.num = num
        self.title = title
        self.mode_key = mode_key
        self.note = note

    # ---- 三段 ----
    def on_enter(self, ctx):
        """进入本步：切视觉模式。检测器的 reset() 由 VisionEngine 统一调。"""
        ctx.engine.select(self.mode_key)
        ctx.phase = Phase.WAIT
        ctx.ready_frames = 0
        ctx.started_s = time.monotonic()

    def on_update(self, ctx, result):
        """每帧：判断本步是否就绪。返回 True 表示本步可以结束了。"""
        rule = READY_RULE.get(self.mode_key)
        if rule is None:
            ctx.ready_frames = REQUIRE_READY_FRAMES       # 无视觉的步骤直接算完成
            return True
        key, check = rule
        if check(result.get(key)):
            ctx.ready_frames += 1
        else:
            ctx.ready_frames = 0
        return ctx.ready_frames >= REQUIRE_READY_FRAMES

    def on_act(self, ctx):
        """本步就绪后要做的事 —— **以后在这里加串口下发**。

        例如对准类步骤：
            ctx.face.pose(tx, ty, tyaw)          # 发 /cmd_pose
        例如放料类步骤：
            ctx.face.gimbal(yaw, lift, extend)   # 发 /cmd_gimbal
            ctx.face.servo(gripper=45)           # 发 /cmd_servo
        现在什么都不做。
        """
        return

    def timeout(self, ctx):
        return time.monotonic() - ctx.started_s > STEP_TIMEOUT_S


class Context:
    """状态机运行上下文。"""

    def __init__(self, engine):
        self.engine = engine
        self.step = None
        self.phase = Phase.REQUEST
        self.ready_frames = 0
        self.started_s = 0.0
        self.result = {}
        self.auto = False
        self.last_ms = 0.0


class VisionEngine:
    """按模式键选检测器；切模式时 reset。"""

    def __init__(self):
        self._cache = {}
        self.current = None

    def get(self, mode_key):
        if mode_key in self._cache:
            return self._cache[mode_key]
        try:
            self._cache[mode_key] = vfd.MODE_CLASSES[mode_key]()
        except Exception as exc:            # 某个检测器加载失败不该让整机停摆
            self._cache[mode_key] = vfd.NoVisionMode(
                f"加载失败：{mode_key}", f"{type(exc).__name__}: {exc}")
        return self._cache[mode_key]

    def select(self, mode_key):
        if mode_key == self.current:
            return
        mode = self.get(mode_key)
        mode.reset()                        # ⚠ 切模式必须 reset，否则上一轮的稳定窗口/种子会污染
        self.current = mode_key

    def process(self, frame):
        if self.current is None:
            return {}, frame
        return self.get(self.current).process(frame)


class FlowMachine:
    """13 步流程状态机。"""

    def __init__(self, steps, auto=False):
        self.steps = list(steps)
        self.engine = VisionEngine()
        self.ctx = Context(self.engine)
        self.index = 0
        self.ctx.auto = auto

    # ---- 状态转移 ----
    def goto(self, index, force_restart=True):
        index = max(0, min(len(self.steps) - 1, index))
        self.index = index
        self.ctx.step = self.steps[index]
        self.ctx.phase = Phase.REQUEST
        self.ctx.ready_frames = 0
        self.ctx.result = {}
        if force_restart:
            self.engine.current = None      # 强制重选，触发 reset
        self.ctx.step.on_enter(self.ctx)
        self.announce()

    def next(self):
        if self.index + 1 < len(self.steps):
            self.goto(self.index + 1)
            return True
        return False                        # 已经是最后一步

    def prev(self):
        self.goto(self.index - 1)

    def tick(self, frame):
        """跑一帧。返回 (结果字典, 标注后的画面)。"""
        step = self.ctx.step
        started = time.perf_counter()
        result, frame = self.engine.process(frame)
        self.ctx.last_ms = (time.perf_counter() - started) * 1000.0
        self.ctx.result = result

        if self.ctx.phase == Phase.REQUEST:
            step.on_enter(self.ctx)         # 兜底：正常情况下 goto() 已经做过

        if self.ctx.phase == Phase.WAIT:
            if step.on_update(self.ctx, result):
                step.on_act(self.ctx)       # ← 以后串口下发插在这里
                self.ctx.phase = Phase.DONE
                if self.ctx.auto and not self.next():
                    self.ctx.auto = False   # 已经是最后一步，自动停
                    print("自动推进：已走完最后一步，自动关闭")

        return result, frame

    # ---- 面板显示 ----
    # ⚠ 画到画面上的字符串必须是 ASCII —— cv2.putText 不支持中文，会全部变成 "?"
    #   （原项目的检测器绘图也全是英文，就是这个原因）
    #   中文信息走终端 print（见 announce()），终端是 UTF-8，正常显示。
    def describe(self):
        ctx, step = self.ctx, self.ctx.step
        if ctx.phase == Phase.DONE:
            flag = "READY"
        elif step.timeout(ctx):
            flag = f"TIMEOUT {time.monotonic() - ctx.started_s:.1f}s (SPACE to skip)"
        else:
            flag = f"ready {ctx.ready_frames}/{REQUIRE_READY_FRAMES}"

        tag = "DRIVE" if step.num == 0 else f"{step.num}/13"
        head = (f"[{tag:>5}] {step.title}   det={step.mode_key}   "
                f"phase={ctx.phase.value}   {flag}")
        lines = [head, f"    {step.note}"]

        if ctx.phase == Phase.REQUEST:
            lines.append("    switching mode...")
        elif ctx.phase == Phase.WAIT:
            lines.append("    vision: " + "  ".join(
                f"{k}={v}" for k, v in list(ctx.result.items())[:3]))
        else:
            lines.append("    (next: serial commands go here later)")

        lines.append(f"    {ctx.last_ms:.1f}ms   auto={'on' if ctx.auto else 'off'}   "
                     f"   SPACE/n next  p prev  1-9 0 jump  a auto  r rerun  s save  q quit")
        return lines

    def announce(self):
        """把当前步用中文打到终端（画面画不了中文，终端可以）。"""
        step = self.ctx.step
        cn = vfd.STEPS_CN.get(step.num, step.title)
        print(f"→ 第 {step.num} 步：{cn}    [检测器 {step.mode_key}]")


# ============================================================ 主程序

def build_argparser():
    p = argparse.ArgumentParser(description="13 步流程状态机（只调视觉）")
    p.add_argument("--top", default="/dev/video0", help="俯视相机（默认按当前实测映射；编号会漂，见文件头注释）")
    p.add_argument("--front", default="/dev/video2", help="前视相机（扫码）")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--fps", type=float, default=30.0)
    p.add_argument("--no-front", action="store_true", help="没接前视相机")
    p.add_argument("--no-undistort", action="store_true",
                   help="强制关闭畸变矫正（换了相机、原标定失效时用）")
    p.add_argument("--auto", action="store_true", help="启动即自动推进")
    p.add_argument("--start", type=int, default=1, help="起始步（1~13，0=行车）")
    return p


def main():
    args = build_argparser().parse_args()

    # 把实际用到的设备和起始步打出来 —— 相机编号会漂，不带参数跑时最容易搞混
    print(f"相机：俯视={args.top}   前视={args.front}"
          f"{'   （前视已禁用）' if args.no_front else ''}")
    print(f"起始：第 {args.start} 步"
          f"{'（0=行车）' if args.start == 0 else ''}")

    # ---- 矫正设置必须和 main.py 一致，否则这里调出的参数在主程序里不通用 ----
    if args.no_undistort:
        undistort_on, map1, map2 = False, None, None
        note = "undistort: OFF (forced by --no-undistort)"
    else:
        undistort_on, map1, map2, note = vfd.build_undistorter(
            vfd.CFG / "camera_processing.json", vfd.CFG / "realtek_rgb_camera.yaml",
            args.width, args.height)
    print(note)

    # stacked 检测器要的相机几何签名（对应 main.py:1213-1214）
    try:
        from stacked_ring_detector import camera_geometry_signature
        vfd._CAMERA_GEOMETRY["signature"] = camera_geometry_signature(
            args.width, args.height, undistort_on, vfd.CFG / "realtek_rgb_camera.yaml")
    except Exception as exc:
        vfd._CAMERA_GEOMETRY["signature"] = None
        print(f"⚠ 相机几何签名计算失败（第 13 步会拒跑）：{exc}")

    # ---- 相机 ----
    caps, cap_note = {}, {}
    for key, device in (("top", args.top),
                        ("front", None if args.no_front else args.front)):
        if device is None:
            cap_note[key] = "disabled by --no-front (press SPACE for next step)"
            continue
        cap, msg = vfd.open_camera(device, args.width, args.height, args.fps)
        caps[key] = cap
        cap_note[key] = msg

    # ---- 状态机 ----
    steps = [Step(num, title, mode, note2) for num, title, mode, note2 in vfd.STEPS]
    machine = FlowMachine(steps, auto=args.auto)
    start_index = next((i for i, s in enumerate(steps) if s.num == args.start), 0)
    machine.goto(start_index)

    win = "flow_state_machine"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)

    while True:
        step = machine.ctx.step
        src = "front" if step.mode_key == "qr" else "top"

        cap = caps.get(src)
        if cap is None:
            frame = vfd.blank_card(args.width, args.height, step.title,
                                   f"相机不可用：{src} —— {cap_note.get(src, '')}")
            ok = False
        else:
            ok, frame = cap.read()
            if not ok or frame is None:
                frame = vfd.blank_card(args.width, args.height, step.title,
                                       f"相机读取失败：{src}")
            elif undistort_on and src == "top":
                frame = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)

        if ok:
            try:
                machine.tick(frame)
            except Exception as exc:
                machine.ctx.result = {"error": f"{type(exc).__name__}: {exc}"}

        vfd.draw_panel(frame, machine.describe())
        cv2.imshow(win, frame)
        key = cv2.waitKey(1) & 0xFF

        if key in (ord("q"), 27):
            break
        elif key in (ord(" "), ord("n")):
            if not machine.next():
                print("已经是最后一步")
        elif key == ord("p"):
            machine.prev()
        elif key == ord("a"):
            machine.ctx.auto = not machine.ctx.auto
            print(f"自动推进：{'开' if machine.ctx.auto else '关'}")
            if machine.ctx.auto and machine.ctx.phase == Phase.DONE:
                machine.next()
        elif key == ord("r"):
            machine.goto(machine.index)
            print(f"已重跑第 {step.num} 步")
        elif key == ord("s"):
            name = f"flow_{step.num:02d}_{step.mode_key}.png"
            cv2.imwrite(name, frame)
            print(f"已保存 {name}")
        elif ord("0") <= key <= ord("9"):
            want = key - ord("0")
            target = next((i for i, s in enumerate(steps) if s.num == want), None)
            if target is not None:
                machine.goto(target)

    for cap in caps.values():
        if cap is not None:
            cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
