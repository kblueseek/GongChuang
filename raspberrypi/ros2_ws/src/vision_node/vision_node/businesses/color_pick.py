"""color_pick / track_debug 业务：复用 vision/gimbal_track_pick.py 的算法。

不复制跟踪循环，只负责：
  · 把节点参数 + 请求参数拼成 gimbal_track_pick 认识的 args；
  · 加载颜色阈值、预计算畸变/旋转上下文（每次业务开始算一次）；
  · 用共享摄像头和节点跑 track_color()。
"""

from __future__ import annotations

from ..business import Business, register


def _coerce(current, value):
    """按 args 里原来的类型转一下请求值（JSON 只有数字/字符串/布尔）。"""
    if isinstance(current, bool):
        return bool(value)
    if isinstance(current, int):
        return int(float(value))
    if isinstance(current, float):
        return float(value)
    return value


class _ColorPickBase(Business):
    debug = False

    def run(self, ctx, args, stop):
        # vision/ 目录已在节点启动时塞进 sys.path；这里再兜底一次。
        import sys
        vision_dir = str(ctx.vision_dir)
        if vision_dir and vision_dir not in sys.path:
            sys.path.insert(0, vision_dir)

        import gimbal_track_pick as gtp

        # 和独立运行 gimbal_track_pick.py 完全一样的默认参数（含 track_pid.json）
        a = gtp.build_args([])

        # 相机几何以节点参数为准：这些量不能热改，ctx/畸形映射已经按它算好。
        a.headless = bool(ctx.headless)
        a.device = ctx.device
        a.width = int(ctx.width)
        a.height = int(ctx.height)
        a.fps = float(ctx.fps)
        a.cam_rotation = int(ctx.cam_rotation)

        try:
            color = int(args.get('color'))
        except (TypeError, ValueError):
            return {'ok': False, 'reason': 'color 缺失或不是整数（1~6）'}
        a.color = color

        if 'timeout_s' in args:
            try:
                a.timeout = float(args['timeout_s'])
            except (TypeError, ValueError):
                pass
        if 'still_time' in args:
            try:
                a.still_time = float(args['still_time'])
            except (TypeError, ValueError):
                pass

        # 其余字段（target_x/y、PID、方向、ROI…）有就覆盖，没有用 track_pid.json
        for key, value in args.items():
            if key in ('color', 'timeout_s', 'still_time'):
                continue
            if hasattr(a, key):
                try:
                    setattr(a, key, _coerce(getattr(a, key), value))
                except (TypeError, ValueError):
                    pass

        color_doc = gtp.load_color_config(
            ctx.vision_dir / 'config' / 'material_color_thresholds.json')
        if not any(int(c['color_id']) == color for c in color_doc['colors']):
            return {'ok': False, 'color': color,
                    'reason': f'颜色 {color} 没有标定配置'}

        # 畸变映射 / 旋转 / ROI：每次业务开始算一次（约几十毫秒），
        # 不再像以前那样每条指令起一个进程重新算。
        frame_ctx = gtp._prepare_frame_ctx(a)

        return gtp.track_color(
            ctx.camera, ctx.node, a, color_doc, frame_ctx, color,
            debug=self.debug, should_stop=stop.is_set)


@register
class ColorPickBusiness(_ColorPickBase):
    name = 'color_pick'
    debug = False


@register
class TrackDebugBusiness(_ColorPickBase):
    name = 'track_debug'
    debug = True
