#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""十步流程的视觉模式轮播 —— 只看，不发串口。

用途
----
按一个键就切到"下一步需要的视觉"，把十个步骤对应的视觉代码依次走一遍，
顺手当场调参、当场看效果。**不碰串口、不依赖 ROS2。**

和 main.py 的区别
-----------------
`main.py` 是工位视觉的主程序（按键 A/R/S/M/D 切工位模式），**没有**车道障碍、
也没有"识别圆环上物料"。这个工具是**按比赛流程顺序**串起来的走查工具，
把每一步需要的视觉放在同一个进程里（相机只能被一个进程占用，所以必须合在一起）。

按键
----
    空格 / n      下一步（按十步流程顺序）
    p             上一步
    1-9 0         直接跳到第 1~10 步（0 = 行车）
    r             重置当前检测器（清稳定窗口/种子）
    s             存一张当前调试图
    q / Esc       退出

用法
----
    python3 vision_flow_debug.py                      # 两路都用默认设备
    python3 vision_flow_debug.py --top /dev/video2 --front /dev/video0
    python3 vision_flow_debug.py --no-front            # 没接前视相机时跳过扫码步
    python3 vision_flow_debug.py --no-undistort        # 换了相机、原标定失效时

已知限制
--------
1. `cv2.FileStorage` 在 **Windows 下读不了中文路径**。想在 Windows 上试这个工具，
   必须把整个 vision 目录放到纯英文路径下，否则：
     - 畸变矫正会退化成"关闭"
     - 叠放位（⑩）会报 `camera geometry not configured`
   树莓派（Linux，路径是 /home/pi/...）没有这个问题。

2. **必须和 main.py 用同一套矫正设置**。检测器吃的是矫正后画面；如果这里不矫正、
   而 main.py 矫正了，两边看到的是不同的像素坐标系，**在这里调出的参数在主程序里不通用**。
"""

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))
CFG = HERE / "config"

# stacked 检测器要求喂一个"相机几何签名"，否则 detect() 直接拒跑。
# 在 main() 里算好放进来，StackedMode 构造时取用。
_CAMERA_GEOMETRY = {"signature": None}
# 运行时的矫正开关，供面板显示
_UNDISTORT = {"on": False, "note": ""}


def build_undistorter(processing_json, calibration_yaml, width, height):
    """按 main.py:1163-1184 的做法建矫正映射。

    必须和主程序用同一套设置：**否则这个工具里调出来的参数在 main.py 里不适用**
    （检测器吃的是矫正后画面，像素坐标会不一样）。

    返回 (是否开启, map1, map2, 说明)。
    """
    try:
        document = json.loads(Path(processing_json).read_text(encoding="utf-8"))
        enabled = document.get("undistort_enabled")
    except Exception as exc:
        return False, None, None, f"undistort: OFF (cannot read cfg: {exc})"
    if type(enabled) is not bool:
        return False, None, None, "undistort: OFF (undistort_enabled must be bool)"

    if not enabled:
        return False, None, None, "undistort: OFF (raw frames)"

    storage = cv2.FileStorage(str(calibration_yaml), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        return False, None, None, "undistort: OFF (cannot open calibration yaml)"
    try:
        cal_w = int(storage.getNode("image_width").real())
        cal_h = int(storage.getNode("image_height").real())
        camera_matrix = storage.getNode("camera_matrix").mat()
        distortion = storage.getNode("distortion_coefficients").mat()
        new_camera_matrix = storage.getNode("new_camera_matrix").mat()
    finally:
        storage.release()

    if (cal_w, cal_h) != (width, height):
        return False, None, None, (f"undistort: OFF (calib {cal_w}x{cal_h} "
                                   f"!= run {width}x{height})")
    if camera_matrix is None or distortion is None or new_camera_matrix is None:
        return False, None, None, "undistort: OFF (calib yaml missing matrix)"
    map1, map2 = cv2.initUndistortRectifyMap(
        camera_matrix, distortion, None, new_camera_matrix,
        (cal_w, cal_h), cv2.CV_16SC2)
    return True, map1, map2, f"undistort: ON ({cal_w}x{cal_h})"



# ============================================================ 各步的视觉模式

class ModeBase:
    needs = "top"          # 'top' = 俯视相机，'front' = 前视相机

    def reset(self):
        pass

    def process(self, frame):
        """返回 (结果 dict, 打好标注的 frame)。"""
        raise NotImplementedError


def _color_list(document):
    """load_color_config 的返回值可能是 dict({'colors': [...]}) 或直接是 list。"""
    if isinstance(document, dict):
        return document.get("colors", [])
    return document or []


# 检测器的失败原因（detail）是中文的，而 cv2.putText 画不了中文（会全变成 "?"）。
# 所以：**画面只放 ASCII 占位，把 detail 打到终端**（终端是 UTF-8，正常显示）。
# 原项目的检测器也是这个设计：画面上全英文，中文只出现在 docstring / 异常 / detail。
_detail_last = {}


def log_detail(mode_key, detail):
    """把检测器的中文 detail 打到终端。同一模式 2 秒最多一次，避免刷屏。"""
    now = time.monotonic()
    if now - _detail_last.get(mode_key, 0.0) < 2.0:
        return
    _detail_last[mode_key] = now
    print(f"[{mode_key}] {detail}")


def _classify_roi(hsv_roi, colors):
    """在 ROI 里按 HSV 阈值判颜色，返回 (color_id, name, 占比)。"""
    total = max(hsv_roi.shape[0] * hsv_roi.shape[1], 1)
    best_id, best_name, best_frac = 0, "empty", 0.0
    for entry in colors:
        mask = None
        for rg in entry.get("ranges", []):
            part = cv2.inRange(hsv_roi,
                               np.array(rg["lower"], np.uint8),
                               np.array(rg["upper"], np.uint8))
            mask = part if mask is None else cv2.bitwise_or(mask, part)
        if mask is None:
            continue
        frac = cv2.countNonZero(mask) / float(total)
        if frac > best_frac:
            best_id = int(entry["color_id"])
            best_name = str(entry["name"])
            best_frac = frac
    return best_id, best_name, best_frac


class QRMode(ModeBase):
    """① 扫二维码（前视相机）"""
    needs = "front"
    label = "① 扫二维码"

    def __init__(self):
        from qr_scanner import (decode_qr, draw_polygon,
                                normalise_task_code, REQUIRED_STABLE_FRAMES)
        self._decode = decode_qr
        self._draw = draw_polygon
        self._norm = normalise_task_code
        self.need = REQUIRED_STABLE_FRAMES
        self.det = cv2.QRCodeDetector()
        self.reset()

    def reset(self):
        self.last = ""
        self.frames = 0

    def process(self, frame):
        text, points = self._decode(self.det, frame)
        if text:
            if text == self.last:
                self.frames += 1
            else:
                self.last, self.frames = text, 1
        else:
            self.last, self.frames = "", 0
        code = self._norm(self.last) if self.frames >= self.need else None
        self._draw(frame, points)
        self._draw_result(frame, text, code)
        return {"raw": text or "--",
                "debounce": f"{self.frames}/{self.need}",
                "task_code": code or "(not confirmed)"}, frame

    # ---- 把识别结果用大字打到画面上（现场对着实物核对最直观）----
    # 位置放画面靠下（0.90 高度）—— 二维码板通常占画面中上部，放低才不压住它
    def _draw_result(self, frame, text, code):
        height = frame.shape[0]
        y = int(height * 0.90)
        if code:
            # 已确认：绿色大字，四组三位数一眼对上
            draw_big_text(frame, f"TASK CODE:  {code}", y, (60, 255, 60), 1.65)
        elif text:
            # 读到了但还没连续 3 帧确认：橙色，先给你看原始串
            draw_big_text(frame, f"RAW:  {text}", y, (0, 210, 255), 1.2)
        else:
            draw_big_text(frame, "SEARCHING QR ...", y, (170, 170, 170), 1.1)


class TurntableMode(ModeBase):
    """②⑧ 转盘对准（俯视）：出圆心和半径"""
    label = "②⑧ 转盘对准"

    def __init__(self):
        from turntable_detector import (load_turntable_detector_config,
                                        TurntableDetector,
                                        draw_turntable_detection)
        self.det = TurntableDetector(
            load_turntable_detector_config(CFG / "turntable_detector.json"))
        self._draw = draw_turntable_detection

    def reset(self):
        self.det.reset()

    def process(self, frame):
        r = self.det.detect(frame)
        self._draw(frame, r)
        if not r.detected:
            log_detail("turntable", r.detail)
            return {"found": "no", "why": "no detection"}, frame
        # 尺度不用标定：转盘直径 300mm 已知
        mm_per_px = 300.0 / (2.0 * r.radius_px) if r.radius_px > 1 else 0.0
        return {"found": "yes", "stable": r.stable,
                "center": f"({r.center_x_px:.1f}, {r.center_y_px:.1f})",
                "radius": f"{r.radius_px:.1f}px",
                "mm/px": f"{mm_per_px:.4f}",
                "conf": f"{r.confidence:.2f}"}, frame


class MaterialMode(ModeBase):
    """③⑧ 识别 down 区物料颜色（俯视），并把 down 区物料的中心坐标显示出来"""
    label = "③⑧ 识别 down 区物料"

    COLOR_NAMES = {0: "empty", 1: "red", 2: "yellow", 3: "blue",
                   4: "green", 5: "black", 6: "lblue"}

    def __init__(self):
        from material_pickup_detector import (load_material_pickup_config,
                                              MaterialPickupDetector,
                                              draw_material_pickup_detection,
                                              _point_in_rectangle)
        from 颜色标定 import load_color_config
        self._in_rect = _point_in_rectangle        # 复用检测器自己的判据
        self.config = load_material_pickup_config(
            CFG / "material_pickup_detector.json")
        self.det = MaterialPickupDetector(
            self.config,
            load_color_config(CFG / "material_color_thresholds.json"))
        self._draw = draw_material_pickup_detection
        self.reset()

    def reset(self):
        self.det.reset()

    def down_zone_material(self, result):
        """挑出落在 down 区的那件物料，返回 MaterialRegion 或 None。

        ⚠ 不能用 result.slot_colors —— 它只给颜色编号（`SLOT_NAMES` 顺序是
          left/right/down，0 = 该区没料），**坐标在 _assign_slots() 里被丢掉了**。
          所以这里拿 result.regions 自己判一遍，并复用检测器的
          `_point_in_rectangle`，保证判据和它内部完全一致
          （含 slot_interior_margin_px 的内缩边距）。
        """
        margin = float(self.config["detection"]["slot_interior_margin_px"])
        rectangle = self.config["regions"]["down"]
        for region in result.regions:
            if self._in_rect(region.center_x_px, region.center_y_px,
                             rectangle, margin):
                return region
        return None

    def process(self, frame):
        r = self.det.detect(frame, timestamp=time.monotonic())
        self._draw(frame, r, self.config)
        if not r.detected:
            log_detail("material", r.detail)

        down = self.down_zone_material(r)
        self._draw_down_marker(frame, down)

        lrd = " / ".join(self.COLOR_NAMES.get(c, str(c)) for c in r.slot_colors)
        return {"found": "yes" if r.detected else "no",
                "reportable": "yes" if r.ready else "no",
                "state": r.state,
                "L/R/D colors": lrd,
                "down_px": (f"({down.center_x_px:.0f}, {down.center_y_px:.0f})"
                            if down else "(no material)"),
                "down_color": down.name if down else "-",
                "speed": f"{r.max_speed_px_s:.0f} px/s",
                "count": r.material_count}, frame

    # ---- 在 down 区物料中心画十字 + 大字标注坐标 ----
    def _draw_down_marker(self, frame, region):
        y = int(frame.shape[0] * 0.90)
        if region is None:
            draw_big_text(frame, "DOWN MATERIAL: (no material)", y,
                          (170, 170, 170), 1.2)
            return
        cx, cy = int(round(region.center_x_px)), int(round(region.center_y_px))
        cv2.drawMarker(frame, (cx, cy), (0, 220, 255), cv2.MARKER_CROSS, 44, 3)
        cv2.circle(frame, (cx, cy), int(max(region.area_px ** 0.5, 20)),
                   (0, 220, 255), 2)
        draw_big_text(frame, f"DOWN MATERIAL: ({cx}, {cy})  {region.name}", y,
                      (0, 220, 255), 1.35)


class RingMode(ModeBase):
    """④⑨ 圆环对准（俯视）：出三环心、工位角、毫米误差、底盘纠偏量"""
    label = "④⑨ 圆环对准"

    def __init__(self):
        from ring_station_detector import (load_ring_station_config,
                                           RingStationDetector,
                                           draw_ring_station_detection)
        self.det = RingStationDetector(
            load_ring_station_config(CFG / "ring_station_detector.json"))
        self._draw = draw_ring_station_detection

    def process(self, frame):
        r = self.det.detect(frame)
        self._draw(frame, r)
        if not r.detected:
            log_detail("ring", r.detail)
            return {"found": "no", "why": "no detection"}, frame
        c2 = r.centers_px[1] if len(r.centers_px) > 1 else (0.0, 0.0)
        return {"found": "yes", "stable": r.stable,
                "ring2 center": f"({c2[0]:.1f}, {c2[1]:.1f})",
                "yaw": f"{r.station_yaw_deg:+.2f} deg",
                "err_mm": f"({r.error_x_mm:+.1f}, {r.error_y_mm:+.1f})",
                "within": r.within_tolerance}, frame


class RingMaterialMode(ModeBase):
    """⑥⑨ 识别圆环上的物料（俯视）—— 实验性

    做法：用 ring_station_detector 给出的三个环心当 ROI，在环心附近跑 HSV 分色。
    这样不用重标 material_pickup_detector 的四个区域（那是转盘场景的）。
    ⚠ 这是起点实现，阈值和 ROI 大小都要拿真画面调。
    """
    label = "⑥⑨ 识别圆环上的物料（实验性）"
    ROI_R = 45

    def __init__(self):
        from ring_station_detector import (load_ring_station_config,
                                           RingStationDetector)
        from 颜色标定 import load_color_config
        self.ring = RingStationDetector(
            load_ring_station_config(CFG / "ring_station_detector.json"))
        self.colors = _color_list(
            load_color_config(CFG / "material_color_thresholds.json"))

    def process(self, frame):
        r = self.ring.detect(frame)
        if not r.detected:
            log_detail("ringmat", r.detail)
            return {"found": "no", "why": "no detection"}, frame
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        found = []
        for index, (cx, cy) in enumerate(r.centers_px, 1):
            x0, x1 = int(cx - self.ROI_R), int(cx + self.ROI_R)
            y0, y1 = int(cy - self.ROI_R), int(cy + self.ROI_R)
            if x0 < 0 or y0 < 0 or x1 > frame.shape[1] or y1 > frame.shape[0]:
                continue
            cid, name, frac = _classify_roi(hsv[y0:y1, x0:x1], self.colors)
            cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 220, 255), 2)
            cv2.putText(frame, f"{index}:{name} {frac:.0%}", (x0, max(18, y0 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 220, 255), 2, cv2.LINE_AA)
            found.append((index, cid, name, frac))
        return {"found": "yes", "rings": len(found),
                "result": "  ".join(f"R{i}={n}({f:.0%})" for i, _, n, f in found) or "--"}, frame


class StackedMode(ModeBase):
    """⑩ 叠放位（俯视）：中环 + 上下长边"""
    label = "⑩ 叠放位（码垛）"

    def __init__(self):
        from stacked_ring_detector import (load_stacked_ring_config,
                                           StackedRingDetector,
                                           draw_stacked_ring_detection)
        from ring_station_detector import load_ring_station_config
        self.det = StackedRingDetector(
            load_stacked_ring_config(CFG / "stacked_ring_detector.json"),
            load_ring_station_config(CFG / "ring_station_detector.json"))
        # ⚠ 必须喂相机几何签名（对应 main.py:1213-1214），否则 detect() 直接返回
        #   "Recalibrate stacking: camera geometry not configured"
        self.det.set_camera_geometry(_CAMERA_GEOMETRY["signature"])
        self._draw = draw_stacked_ring_detection

    def process(self, frame):
        r = self.det.detect(frame)
        self._draw(frame, r)
        if not r.detected:
            log_detail("stacked", r.detail)
            return {"found": "no", "why": "no detection"}, frame
        mid = r.middle_center_px or (0.0, 0.0)
        return {"found": "yes", "stable": r.stable,
                "mid center": f"({mid[0]:.1f}, {mid[1]:.1f})",
                "mid radius": f"{r.middle_radius_px:.1f}px",
                "yaw": f"{r.station_yaw_deg:+.2f} deg"}, frame


class LaneMode(ModeBase):
    """行车（俯视抬起）：车道 + 黑色障碍"""
    label = "行车 · 车道 + 黑色障碍"

    def __init__(self):
        from lane_obstacle_detector import LaneObstacleDetector, draw_detection
        self.det = LaneObstacleDetector()
        self._draw = draw_detection

    def process(self, frame):
        r = self.det.detect(frame)
        self._draw(frame, r)
        return {"found": "yes" if r.lane_found else "no",
                "action": r.action,
                "offset": f"{r.offset_mm:+.1f} mm",
                "lane": f"{r.width_px_near:.0f}px (mm/px={r.mm_per_px_near:.3f})",
                "obstacles": f"{len(r.obstacles)}"
                             + (f", nearest {r.nearest.distance_mm:.0f}mm"
                                if r.nearest else ""),
                "free L/R": f"{r.free_left_mm:.0f} / {r.free_right_mm:.0f} mm"}, frame


class NoVisionMode(ModeBase):
    """本步没有视觉，只显示一张说明卡。"""
    needs = None

    def __init__(self, label, note):
        self.label = label
        self.note = note

    def process(self, frame):
        return {"note": self.note}, frame


MODE_CLASSES = {
    "qr": QRMode,
    "turntable": TurntableMode,
    "material": MaterialMode,
    "ring": RingMode,
    "ringmat": RingMaterialMode,
    "stacked": StackedMode,
    "lane": LaneMode,
}


# ============================================================ 十步流程表

STEPS = [
    (1,  "1  Scan QR code",             "qr",        "front cam; 3 identical frames to confirm"),
    (2,  "2  Turntable align",          "turntable", "top cam; bring down-zone material under gripper"),
    (3,  "3  Read down-zone material",  "material",  "top cam; read L/R/D slot colors"),
    (4,  "4  Rough-zone align ring2",   "ring",      "top cam; bring ring 2 under gripper"),
    (5,  "5  Place onto rings",         "ring",      "same as 4; use ring centers to reach each ring"),
    (6,  "6  Pick back onto plate",     "ring",      "same as 4; ring centers locate the material"),
    (7,  "7  Place at storage (1st)",   "ring",      "same as 4; first layer, no stacking needed"),
    (8,  "8  Back to turntable",        "turntable", "top cam; same as 2"),
    (9,  "9  Read 2nd-batch material",  "material",  "top cam; same as 3"),
    (10, "10 Rough-zone align ring2",   "ring",      "top cam; same as 4"),
    (11, "11 Place onto rings",         "ring",      "same as 4"),
    (12, "12 Pick back onto plate",     "ring",      "same as 6 (2nd batch)"),
    (13, "13 Stack at storage",         "stacked",   "stacking: middle ring + upper/lower lines"),
    (0,  "D  Drive: lane + obstacles",  "lane",      "top cam raised; lane keeping + black cylinders"),
]

# 终端里用的中文标题（cv2 画不了中文，所以只 print 到终端）
STEPS_CN = {
    1: "扫二维码", 2: "转盘对准", 3: "识别 down 区物料", 4: "粗加工区对准圆环2",
    5: "按任务码放料到圆环", 6: "从圆环夹回车上载物台", 7: "到暂存区按序放置",
    8: "回转盘重新校准", 9: "识别第二批物料并夹取", 10: "粗加工区对准圆环2",
    11: "按任务码放料到圆环", 12: "从圆环夹回车上载物台", 13: "暂存区码垛",
    0:  "行车（车道+障碍）",
}


# ============================================================ 相机与显示

def open_camera(device, width, height, fps):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        return None, f"cannot open {device}"
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    for _ in range(20):
        ok, frame = cap.read()
        if ok and frame is not None:
            return cap, f"{frame.shape[1]}x{frame.shape[0]}"
    cap.release()
    return None, f"opened but no frame: {device}"


def draw_panel(frame, lines, color=(60, 255, 60)):
    """在画面上叠一条黑底信息条。"""
    height, width = frame.shape[:2]
    box_h = 20 + 26 * len(lines)
    cv2.rectangle(frame, (8, 8), (min(width - 8, 1080), 8 + box_h), (0, 0, 0), -1)
    for index, text in enumerate(lines):
        cv2.putText(frame, text[:128], (18, 34 + index * 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.60,
                    color if index == 0 else (232, 232, 232), 2, cv2.LINE_AA)
    return frame


def draw_big_text(frame, text, y, color, scale):
    """画带黑描边的大字，深浅背景下都看得清。⚠ 只能传 ASCII（cv2 画不了中文）。"""
    width = frame.shape[1]
    (text_w, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 4)
    x = max(10, (width - text_w) // 2)
    cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale,
                (0, 0, 0), 8, cv2.LINE_AA)
    cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale,
                color, 3, cv2.LINE_AA)


def blank_card(width, height, title, note):
    canvas = np.full((height, width, 3), 28, dtype=np.uint8)
    cv2.putText(canvas, title, (60, height // 2 - 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 220, 255), 3, cv2.LINE_AA)
    cv2.putText(canvas, note, (60, height // 2 + 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2, cv2.LINE_AA)
    return canvas


# ============================================================ 主流程

def main():
    parser = argparse.ArgumentParser(description="十步流程视觉轮播（不发串口）")
    parser.add_argument("--top", default="/dev/video0", help="俯视相机（默认按当前实测映射；编号会漂）")
    parser.add_argument("--front", default="/dev/video2", help="前视相机（扫码）")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--no-front", action="store_true", help="没接前视相机")
    parser.add_argument("--no-undistort", action="store_true",
                        help="强制关闭畸变矫正（换了相机、原标定失效时用）")
    parser.add_argument("--start", type=int, default=1, help="起始步（1~10，0=行车）")
    args = parser.parse_args()

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
        undistort_on, map1, map2, note = build_undistorter(
            CFG / "camera_processing.json", CFG / "realtek_rgb_camera.yaml",
            args.width, args.height)
    _UNDISTORT["on"], _UNDISTORT["note"] = undistort_on, note
    print(note)

    # stacked 检测器要的相机几何签名（对应 main.py:1213-1214）
    try:
        from stacked_ring_detector import camera_geometry_signature
        _CAMERA_GEOMETRY["signature"] = camera_geometry_signature(
            args.width, args.height, undistort_on, CFG / "realtek_rgb_camera.yaml")
    except Exception as exc:
        _CAMERA_GEOMETRY["signature"] = None
        print(f"⚠ 相机几何签名计算失败（叠放位模式会拒跑）：{exc}")

    caps, cap_note = {}, {}
    for key, device in (("top", args.top),
                        ("front", None if args.no_front else args.front)):
        if device is None:
            cap_note[key] = "disabled by --no-front (press SPACE for next step)"
            continue
        cap, note = open_camera(device, args.width, args.height, args.fps)
        caps[key] = cap
        cap_note[key] = note

    mode_cache = {}

    def get_mode(name):
        if name in mode_cache:
            return mode_cache[name]
        try:
            mode_cache[name] = MODE_CLASSES[name]()
        except Exception as exc:
            mode_cache[name] = NoVisionMode(
                f"加载失败：{name}", f"{type(exc).__name__}: {exc}")
        return mode_cache[name]

    index = next((i for i, s in enumerate(STEPS) if s[0] == args.start), 0)
    win = "vision_flow_debug"
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)

    last_ms = 0.0
    last_key_frame = None

    while True:
        num, title, mode_name, note = STEPS[index]

        if mode_name is None:
            mode = NoVisionMode(title, note)
            src = None
        else:
            mode = get_mode(mode_name)
            src = mode.needs

        canvas = None
        if src is None:
            ok = True
            frame = blank_card(args.width, args.height, title, note)
        else:
            cap = caps.get(src)
            if cap is None:
                ok = False
                frame = blank_card(args.width, args.height, title,
                                   f"camera unavailable: {src} - "
                                   f"{cap_note.get(src, '')}")
            else:
                ok, frame = cap.read()
                if not ok or frame is None:
                    frame = blank_card(args.width, args.height, title,
                                       f"camera read failed: {src}")
                elif undistort_on and src == "top":
                    # 俯视相机才做矫正（标定文件是给它的）；二维码不需要
                    frame = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)

        result = {}
        if ok and src is not None:
            started = time.perf_counter()
            try:
                result, frame = mode.process(frame)
            except Exception as exc:
                result = {"error": f"{type(exc).__name__}: {exc}"}
            last_ms = (time.perf_counter() - started) * 1000.0

        header = [
            f"[{num:>2}] {title}   mode={mode_name or 'none'}   "
            f"cam={src or '-'}   {last_ms:.1f}ms",
            f"    {_UNDISTORT['note']}",
        ]
        body = [f"    {k} : {v}" for k, v in result.items()]
        footer = [
            "    SPACE/n next   p prev   1-9 0 jump   r reset   s save   q quit",
        ]
        draw_panel(frame, header + body + footer)

        cv2.imshow(win, frame)
        key = cv2.waitKey(1) & 0xFF

        if key in (ord("q"), 27):
            break
        elif key in (ord(" "), ord("n")):
            index = (index + 1) % len(STEPS)
            last_key_frame = None
        elif key == ord("p"):
            index = (index - 1) % len(STEPS)
            last_key_frame = None
        elif key == ord("r"):
            mode.reset()
            print(f"已重置：{mode_name or title}")
        elif key == ord("s"):
            name = f"flow_{num:02d}_{mode_name or 'none'}.png"
            cv2.imwrite(name, frame)
            print(f"已保存 {name}")
        elif ord("0") <= key <= ord("9"):
            want = key - ord("0")
            index = next((i for i, s in enumerate(STEPS) if s[0] == want), index)
            last_key_frame = None

    for cap in caps.values():
        if cap is not None:
            cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
