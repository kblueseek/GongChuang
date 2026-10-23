#!/usr/bin/env python3
"""检测器结果 → 消息字段的纯转换层。

为什么单独一层
--------------
这一层刻意不 import rclpy、不 import car_interfaces，只吃「有同名属性的对象」、
吐 dict。好处是可以在没有 ROS2 的机器上直接跑测试，改字段映射时不用上机。

`vision_node.py` 只负责把这些 dict 塞进真正的消息对象。
"""

from __future__ import annotations


def _number(value, default=0.0):
    """把可能是 None / 非数值的字段安全转成 float。"""

    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _triplet(values, cast, fill=0):
    """补齐/截断成固定 3 个元素。固定长度数组的消息必须正好 3 个。"""

    items = list(values or ())
    items = (items + [fill, fill, fill])[:3]
    return [cast(item) for item in items]


def ring_fields(result):
    """RingStationDetection 或叠放位结果 → VisionRing 的字段。

    两个检测器的输出结构一致（main.py 里 send_ring_if_ready 共用同一份逻辑），
    所以这里不区分。``detected=False`` 时三个圆心一律填 0。
    """

    detected = bool(getattr(result, "detected", False))
    centers = tuple(getattr(result, "centers_px", ()) or ())
    xs, ys = [], []
    if detected and len(centers) == 3:
        xs = [center[0] for center in centers]
        ys = [center[1] for center in centers]

    return {
        "detected": detected,
        "stable": bool(getattr(result, "stable", False)),
        "center_x_px": _triplet(xs, float),
        "center_y_px": _triplet(ys, float),
        "station_yaw_deg": _number(getattr(result, "station_yaw_deg", 0.0)),
        "error_x_mm": _number(getattr(result, "error_x_mm", 0.0)),
        "error_y_mm": _number(getattr(result, "error_y_mm", 0.0)),
        "angle_error_deg": _number(getattr(result, "angle_error_deg", 0.0)),
        "within_tolerance": bool(getattr(result, "within_tolerance", False)),
    }


def turntable_fields(result):
    """TurntableDetection → VisionTurntable 的字段。"""

    detected = bool(getattr(result, "detected", False))
    return {
        "detected": detected,
        "stable": bool(getattr(result, "stable", False)),
        "center_x_px": _number(getattr(result, "center_x_px", 0.0))
        if detected else 0.0,
        "center_y_px": _number(getattr(result, "center_y_px", 0.0))
        if detected else 0.0,
        "radius_px": _number(getattr(result, "radius_px", 0.0))
        if detected else 0.0,
        "tracking_mode": str(getattr(result, "tracking_mode", "") or ""),
    }


def material_fields(result):
    """MaterialPickupDetection → VisionMaterial 的字段。"""

    slots = _triplet(
        getattr(result, "slot_colors", (0, 0, 0)),
        lambda value: max(0, min(255, int(value))),
    )
    return {
        "calibrated": bool(getattr(result, "calibrated", False)),
        "detected": bool(getattr(result, "detected", False)),
        "ready": bool(getattr(result, "ready", False)),
        "slot_colors": slots,
        "count": max(0, min(255, int(getattr(result, "material_count", 0) or 0))),
        "max_speed_px_s": _number(getattr(result, "max_speed_px_s", 0.0)),
        "state": str(getattr(result, "state", "") or ""),
    }


def task_code_fields(code, valid):
    """二维码结果 → VisionTaskCode 的字段。"""

    return {
        "code": str(code or ""),
        "valid": bool(valid),
    }
