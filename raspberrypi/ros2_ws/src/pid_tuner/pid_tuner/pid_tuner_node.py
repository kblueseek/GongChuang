#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""通用 PID 滑杆调参上位机（tkinter + ROS 2）。

一个窗口做两件事：

1) 通用滑杆：按 YAML 清单给任意节点开一排滑杆，拖动即调 set_parameters 服务，
   目标节点（必须实现了参数变更回调）立刻生效，不用重启。
2) 本地像素曲线：订阅一个像素话题，画「目标像素」vs「实际像素」两条线。
   云台跟随 demo 里就是 /material_target：目标 = 画面中心（img_w/2、img_h/2，
   一条平线），实际 = 物料质心（cx、cy）。两条线重合 = 已对准，来回抖 = kp 过大。

配置全在 config/pid_tuner.yaml，换节点只改 YAML，不动代码：

    node: gimbal_track          # 要调的节点名（不带前导 /）
    params:                     # 每个参数一条滑杆，越界自动夹紧
      - {name: kp_yaw, min: 0.0, max: 6.0, step: 0.05, default: 1.5}
    plot:                       # 曲线段（不需要就把 plot 整段删掉/留空）
      topic: /material_target
      detected_index: 0
      x_index: 1
      ...

话题
----
发布：无
订阅：plot.topic（默认 /material_target，std_msgs/Float32MultiArray）
服务：/<node>/get_parameters（启动时读回真实值）、/<node>/set_parameters（滑杆写入）

参数
----
  config_file    YAML 路径，留空自动找 share/pid_tuner/config/pid_tuner.yaml

用法（需图形桌面，和 cv2 预览窗口一样跑在树莓派本机）：
  ros2 run pid_tuner pid_tuner_node
  ros2 run pid_tuner pid_tuner_node --ros-args -p config_file:=/path/to/other.yaml

调参顺序建议：先只动 kp 往上推，推到「开始来回抖」；再加 kd 把抖压下去；
最后如果稳态还差一点，加一点 ki。每次改完都会清 PID 状态，曲线从零重来是正常的。
"""

from __future__ import annotations

import queue
import threading
import time
import tkinter as tk
from collections import deque
from pathlib import Path

import rclpy
import rclpy.executors
import yaml
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.utilities import try_shutdown
from std_msgs.msg import Float32MultiArray

DEFAULT_CONFIG = "config/pid_tuner.yaml"


def resolve_config(param_value: str) -> Path | None:
    """定位 YAML：显式参数 > share 安装目录 > 源码树。"""
    candidates = []
    if param_value:
        candidates.append(Path(param_value).expanduser())

    try:  # colcon 安装后
        from ament_index_python.packages import get_package_share_directory
        candidates.append(
            Path(get_package_share_directory("pid_tuner")) / DEFAULT_CONFIG
        )
    except Exception:  # noqa: BLE001 - 没装 ament_index 也不该崩
        pass

    here = Path(__file__).resolve()
    for parent in (here, *here.parents):  # 源码运行：<pkg>/pid_tuner/../config/...
        candidates.append(parent / DEFAULT_CONFIG)
    candidates.append(Path.cwd() / DEFAULT_CONFIG)

    for candidate in candidates:
        if candidate.is_file():
            return candidate
    return None


def make_parameter_value(value):
    """按 Python 类型填 ParameterValue（滑杆只会给 float/int/bool/str）。"""
    msg = ParameterValue()
    if isinstance(value, bool):
        msg.type = ParameterType.PARAMETER_BOOL
        msg.bool_value = value
    elif isinstance(value, int):
        msg.type = ParameterType.PARAMETER_INTEGER
        msg.integer_value = value
    elif isinstance(value, float):
        msg.type = ParameterType.PARAMETER_DOUBLE
        msg.double_value = value
    else:
        msg.type = ParameterType.PARAMETER_STRING
        msg.string_value = str(value)
    return msg


class ParamSpec:
    """一个滑杆的规格。default 的 Python 类型决定下发成 int 还是 double。"""

    def __init__(self, raw: dict):
        self.name = str(raw["name"])
        self.vmin = float(raw.get("min", 0.0))
        self.vmax = float(raw.get("max", 1.0))
        self.step = float(raw.get("step", 0.01)) or 0.01
        default = raw.get("default", self.vmin)
        self.is_int = isinstance(default, int) and not isinstance(default, bool)
        self.default = default

    def coerce(self, value: float):
        value = max(self.vmin, min(self.vmax, float(value)))
        return int(round(value)) if self.is_int else float(value)


class TunerNode(Node):
    """只负责 ROS 侧：写参数、读像素。GUI 在另一个类里。"""

    def __init__(self):
        super().__init__("pid_tuner")
        self.declare_parameter("config_file", "")

        config_path = resolve_config(str(self.get_parameter("config_file").value))
        if config_path is None:
            raise FileNotFoundError(
                f"找不到 {DEFAULT_CONFIG}，请用 -p config_file:=<路径> 指定"
            )
        with config_path.open("r", encoding="utf-8") as handle:
            config = yaml.safe_load(handle) or {}

        target = str(config.get("node", "")).strip().lstrip("/")
        if not target:
            raise ValueError(f"{config_path} 里必须写 node: <要调参的节点名>")
        self.target_node = target
        self.specs = [ParamSpec(item) for item in (config.get("params") or [])]
        self.plot_cfg = config.get("plot") or {}
        self.get_logger().info(f"配置：{config_path}，目标节点 /{target}")

        # —— 参数服务 ——
        self._set_cli = self.create_client(SetParameters, f"/{target}/set_parameters")
        self._get_cli = self.create_client(GetParameters, f"/{target}/get_parameters")
        self._pending: dict[str, float] = {}
        self._lock = threading.Lock()
        self._inflight = False
        self._inflight_since = 0.0
        self._requested_initial = False
        self.gui_queue: queue.Queue = queue.Queue()

        # —— 像素曲线数据（executor 线程写，GUI 线程读）——
        self.window = int(self.plot_cfg.get("window", 400))
        self.px = deque(maxlen=self.window)   # (目标x, 实际x or None)
        self.py = deque(maxlen=self.window)   # (目标y, 实际y or None)
        self.img_w = 0.0
        self.img_h = 0.0
        self._x_range = self._parse_range(self.plot_cfg.get("x_range"))
        self._y_range = self._parse_range(self.plot_cfg.get("y_range"))

        topic = str(self.plot_cfg.get("topic", "")).strip()
        if topic:
            self.create_subscription(Float32MultiArray, topic, self._cb_plot, 10)
            self.get_logger().info(f"曲线话题：{topic}")

        self.create_timer(0.1, self._flush)   # 把滑杆的最新值发给目标节点

    @staticmethod
    def _parse_range(value):
        try:
            if value and len(value) == 2:
                return float(value[0]), float(value[1])
        except (TypeError, ValueError):
            pass
        return None

    # ---------------- 参数下发 ----------------

    def set_pending(self, name: str, value):
        """GUI 线程调用：记下最新值，由定时器在 executor 线程真正发出去。"""
        with self._lock:
            self._pending[name] = value

    def _flush(self):
        now = time.monotonic()
        if self._inflight:
            if now - self._inflight_since < 2.0:   # 服务端没响应，超时放行下一条
                return
            self._inflight = False

        if not self._requested_initial and self._get_cli.service_is_ready():
            self._requested_initial = True
            request = GetParameters.Request()
            request.names = [spec.name for spec in self.specs]
            future = self._get_cli.call_async(request)
            future.add_done_callback(self._on_initial_values)

        with self._lock:
            if not self._pending:
                return
            name = next(iter(self._pending))
            value = self._pending.pop(name)

        if not self._set_cli.service_is_ready():
            with self._lock:      # 目标节点还没起来，值留着下次再发
                self._pending.setdefault(name, value)
            self.get_logger().warn(
                f"/{self.target_node} 还没起来，参数暂存中",
                throttle_duration_sec=5.0,
            )
            return

        request = SetParameters.Request()
        request.parameters = [
            Parameter(name=name, value=make_parameter_value(value))
        ]
        future = self._set_cli.call_async(request)
        self._inflight = True
        self._inflight_since = now
        future.add_done_callback(lambda fut, n=name: self._on_set_done(fut, n))

    def _on_initial_values(self, future):
        """启动时读回目标节点真实值，让滑杆初始位置和实际运行值一致。"""
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001
            self.get_logger().warn(f"读取初始参数失败：{error}")
            return
        if response is None:
            return
        values = {}
        for spec, value in zip(self.specs, response.values):
            if value.type == ParameterType.PARAMETER_DOUBLE:
                values[spec.name] = value.double_value
            elif value.type == ParameterType.PARAMETER_INTEGER:
                values[spec.name] = value.integer_value
        if values:
            self.gui_queue.put(("initial", values))

    def _on_set_done(self, future, name):
        self._inflight = False
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001
            self.get_logger().warn(f"写参数 {name} 失败：{error}")
            return
        if response is None or not response.results:
            return
        result = response.results[0]
        if not result.successful:
            self.get_logger().warn(f"写参数 {name} 被拒绝：{result.reason}")

    # ---------------- 像素订阅 ----------------

    def _cb_plot(self, msg: Float32MultiArray):
        cfg = self.plot_cfg
        data = msg.data
        need = max(
            int(cfg.get("detected_index", 0)),
            int(cfg.get("x_index", 1)),
            int(cfg.get("y_index", 2)),
            int(cfg.get("width_index", 3)),
            int(cfg.get("height_index", 4)),
        )
        if len(data) <= need:
            return

        detected = data[int(cfg.get("detected_index", 0))] > 0.5
        img_w = float(data[int(cfg.get("width_index", 3))])
        img_h = float(data[int(cfg.get("height_index", 4))])
        if img_w > 0.0:
            self.img_w = img_w
        if img_h > 0.0:
            self.img_h = img_h

        target_x, target_y = self.img_w / 2.0, self.img_h / 2.0
        if detected:
            self.px.append((target_x, float(data[int(cfg.get("x_index", 1))])))
            self.py.append((target_y, float(data[int(cfg.get("y_index", 2))])))
        else:
            # 丢目标时留空档，不画线，免得看上去像「一下跳到 0」
            self.px.append((target_x, None))
            self.py.append((target_y, None))

    def plot_ranges(self):
        x_range = self._x_range or (0.0, self.img_w or 1280.0)
        y_range = self._y_range or (0.0, self.img_h or 720.0)
        return x_range, y_range


class TunerGui:
    """tkinter 界面：左边滑杆，右边两条像素曲线。"""

    REDRAW_MS = 60

    def __init__(self, root: tk.Tk, node: TunerNode):
        self.root = root
        self.node = node
        self._applying = False      # 程序设置滑杆值时别回写节点
        self._scales = {}

        root.title(f"PID 调参 - /{node.target_node}")
        root.geometry("1180x680")

        left = tk.Frame(root, padx=10, pady=10)
        left.pack(side=tk.LEFT, fill=tk.Y)
        tk.Label(left, text=f"目标节点 /{node.target_node}", font=("TkDefaultFont", 11, "bold")).pack(anchor="w")
        tk.Label(left, text="拖动滑杆立即生效（会重置 PID 状态）", fg="#666").pack(anchor="w", pady=(0, 8))
        self._build_sliders(left)

        tk.Button(left, text="打印当前值", command=self._print_values).pack(anchor="w", pady=(12, 0))

        right = tk.Frame(root, padx=10, pady=10)
        right.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.px_canvas, self.px_info = self._make_panel(right, "水平像素（yaw 方向）：目标=画面中心，实际=物料质心")
        self.py_canvas, self.py_info = self._make_panel(right, "竖直像素（extend 方向）：目标=画面中心，实际=物料质心")

        self._drain_gui()
        self._redraw()

        if not node.specs:
            tk.Label(left, text="（YAML 里没有 params，只有曲线）", fg="#a00").pack(anchor="w")

    # ---------------- 构建 ----------------

    def _build_sliders(self, parent):
        for spec in self.node.specs:
            frame = tk.Frame(parent)
            frame.pack(fill=tk.X, pady=2)
            value_text = tk.StringVar(value=f"{spec.default:g}")
            scale = tk.Scale(
                frame, from_=spec.vmin, to=spec.vmax, resolution=spec.step,
                orient=tk.HORIZONTAL, length=300, label=spec.name,
                showvalue=False,
                command=lambda v, n=spec.name: self._on_slide(n, v),
            )
            scale.set(spec.default)
            scale.pack(side=tk.LEFT)
            tk.Label(frame, textvariable=value_text, width=8, anchor="e").pack(side=tk.LEFT)
            self._scales[spec.name] = (scale, value_text)

    def _make_panel(self, parent, title):
        tk.Label(parent, text=title, anchor="w").pack(fill=tk.X)
        canvas = tk.Canvas(parent, background="#ffffff", height=230, highlightthickness=1,
                           highlightbackground="#cccccc")
        canvas.pack(fill=tk.BOTH, expand=True, pady=(0, 6))
        info = tk.Label(parent, text="等待数据…", anchor="w", fg="#444")
        info.pack(fill=tk.X)
        return canvas, info

    # ---------------- 滑杆 ----------------

    def _spec(self, name):
        for spec in self.node.specs:
            if spec.name == name:
                return spec
        return None

    def _on_slide(self, name, value):
        if self._applying:
            return
        spec = self._spec(name)
        if spec is None:
            return
        coerced = spec.coerce(float(value))
        scale, text = self._scales[name]
        text.set(f"{coerced:g}")
        self.node.set_pending(name, coerced)

    def _print_values(self):
        parts = []
        for name, (scale, _) in self._scales.items():
            parts.append(f"{name}={scale.get():g}")
        self.node.get_logger().info("当前滑杆值：" + " ".join(parts))

    # ---------------- GUI 事件 ----------------

    def _drain_gui(self):
        try:
            while True:
                kind, payload = self.node.gui_queue.get_nowait()
                if kind == "initial":
                    self._apply_initial(payload)
        except queue.Empty:
            pass
        self.root.after(100, self._drain_gui)

    def _apply_initial(self, values):
        """用目标节点里的真实值摆正滑杆（不触发回写）。"""
        self._applying = True
        try:
            for name, value in values.items():
                entry = self._scales.get(name)
                if entry is None:
                    continue
                scale, text = entry
                scale.set(value)
                text.set(f"{value:g}")
        finally:
            self._applying = False

    # ---------------- 曲线 ----------------

    def _redraw(self):
        x_range, y_range = self.node.plot_ranges()
        self._draw(self.px_canvas, self.node.px, x_range, "#d62728")
        self._draw(self.py_canvas, self.node.py, y_range, "#1f6fd6")
        self._update_info()
        self.root.after(self.REDRAW_MS, self._redraw)

    def _draw(self, canvas, samples, value_range, actual_color):
        # deque 会在 executor 线程里被 append，遍历时先拍快照，否则可能
        # 抛 "deque mutated during iteration"
        samples = list(samples)
        canvas.delete("all")
        width = max(canvas.winfo_width(), 50)
        height = max(canvas.winfo_height(), 50)
        pad_x, pad_top, pad_bottom = 46, 16, 18
        plot_w = width - pad_x - 8
        plot_h = height - pad_top - pad_bottom
        if plot_w < 10 or plot_h < 10:
            return

        vmin, vmax = value_range
        if vmax <= vmin:
            vmax = vmin + 1.0

        def to_y(value):
            ratio = (value - vmin) / (vmax - vmin)
            ratio = max(0.0, min(1.0, ratio))
            return pad_top + (1.0 - ratio) * plot_h

        # 网格 + 刻度
        for i in range(5):
            value = vmin + (vmax - vmin) * i / 4.0
            y = to_y(value)
            canvas.create_line(pad_x, y, pad_x + plot_w, y, fill="#eeeeee")
            canvas.create_text(pad_x - 6, y, text=f"{value:.0f}", anchor="e",
                               fill="#888", font=("TkDefaultFont", 8))

        count = len(samples)
        if count < 2:
            canvas.create_text(width / 2, height / 2, text="等待数据…", fill="#aaa")
            return

        def to_x(index):
            return pad_x + plot_w * index / (count - 1)

        # 目标线（画面中心，通常是一条平线）
        target_pts = []
        for index, (target, _) in enumerate(samples):
            target_pts.extend((to_x(index), to_y(target)))
        canvas.create_line(*target_pts, fill="#999999", width=1, dash=(4, 3))

        # 实际线：丢目标的空档断开，不连过去
        segment = []
        for index, (_, actual) in enumerate(samples):
            if actual is None:
                if len(segment) >= 4:
                    canvas.create_line(*segment, fill=actual_color, width=2)
                segment = []
                continue
            segment.extend((to_x(index), to_y(actual)))
        if len(segment) >= 4:
            canvas.create_line(*segment, fill=actual_color, width=2)

        # 图例
        canvas.create_line(pad_x + 4, 10, pad_x + 26, 10, fill="#999999", dash=(4, 3))
        canvas.create_text(pad_x + 30, 10, text="目标", anchor="w", fill="#777",
                           font=("TkDefaultFont", 8))
        canvas.create_line(pad_x + 70, 10, pad_x + 92, 10, fill=actual_color, width=2)
        canvas.create_text(pad_x + 96, 10, text="实际", anchor="w", fill="#555",
                           font=("TkDefaultFont", 8))

    def _update_info(self):
        def latest(samples):
            for target, actual in reversed(list(samples)):
                if actual is not None:
                    return target, actual
            return None

        for canvas_label, samples in ((self.px_info, self.node.px), (self.py_info, self.node.py)):
            pair = latest(samples)
            if pair is None:
                canvas_label.config(text="未检测到目标（实际线断开）")
            else:
                target, actual = pair
                canvas_label.config(
                    text=f"目标 {target:.0f}px   实际 {actual:.0f}px   误差 {actual - target:+.0f}px"
                )


def main(args=None):
    rclpy.init(args=args)
    try:
        node = TunerNode()
    except (FileNotFoundError, ValueError) as error:
        print(f"[pid_tuner] {error}")
        rclpy.shutdown()
        return
    except Exception:  # noqa: BLE001
        rclpy.shutdown()
        raise

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        root = tk.Tk()
    except tk.TclError as error:
        node.get_logger().error(f"打不开图形界面（无桌面环境？）：{error}")
        executor.shutdown()
        node.destroy_node()
        try_shutdown()
        return

    TunerGui(root, node)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        try_shutdown()


if __name__ == "__main__":
    main()
