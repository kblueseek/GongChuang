"""vision_put 业务：复用 vision/vision_put_align.py 的对环循环。

和 color_pick 一样，不复制算法：
  · 用 vision_put_align.build_args([]) 取默认参数 + config/vision_put.json；
  · 用请求里的 ring / target_x / target_y / timeout_s 覆盖；
  · 用共享摄像头和节点跑 run_align()。
"""

from __future__ import annotations

from pathlib import Path

from ..business import Business, register


def _coerce(current, value):
    """按 args 里原来的类型转一下请求值。"""
    if isinstance(current, bool):
        return bool(value)
    if isinstance(current, int):
        return int(float(value))
    if isinstance(current, float):
        return float(value)
    return value


@register
class VisionPutBusiness(Business):
    name = 'vision_put'

    def run(self, ctx, args, stop):
        import sys
        vision_dir = str(ctx.vision_dir)
        if vision_dir and vision_dir not in sys.path:
            sys.path.insert(0, vision_dir)

        import vision_put_align as vpa

        # 默认参数 + config/vision_put.json（和独立运行完全一致）
        a = vpa.build_args([])

        # 相机几何以节点参数为准
        a.headless = bool(ctx.headless)
        a.device = ctx.device
        a.width = int(ctx.width)
        a.height = int(ctx.height)
        a.fps = float(ctx.fps)
        a.cam_rotation = int(ctx.cam_rotation)

        # 请求覆盖：ring / target_x / target_y / timeout_s / dry_run / PID…
        for key, value in args.items():
            if key == 'timeout_s':
                try:
                    a.timeout = float(value)
                except (TypeError, ValueError):
                    pass
                continue
            if hasattr(a, key):
                try:
                    setattr(a, key, _coerce(getattr(a, key), value))
                except (TypeError, ValueError):
                    pass

        try:
            config = vpa.load_ring_station_config(Path(a.config))
        except Exception as error:  # noqa: BLE001
            return {'ok': False, 'reason': f'圆环配置无效：{error}'}

        detector = vpa.RingStationDetector(config)
        return vpa.run_align(ctx.camera, ctx.node, a, detector,
                             stop=stop.is_set)
