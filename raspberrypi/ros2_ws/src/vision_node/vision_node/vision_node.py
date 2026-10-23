#!/usr/bin/env python3
"""视觉节点：两路相机 → 识别结果话题。

只做三件事：读相机、跑检测器、发话题。**不发任何运动指令。**
运动指令由 flow_node 根据这些结果算出来，再发给 f407_bridge。

话题
----
发布：
    /vision/turntable   car_interfaces/VisionTurntable   第 2、8 步（俯视）
    /vision/ring        car_interfaces/VisionRing        第 4~7、10~12 步（俯视）
    /vision/stacked     car_interfaces/VisionRing        第 13 步（俯视，码垛）
    /vision/material    car_interfaces/VisionMaterial    第 3、9 步（俯视）
    /vision/task_code   car_interfaces/VisionTaskCode    第 1 步（平视）
订阅：
    /vision/mode        std_msgs/String                  取值见 MODES

相机分工（13 步里只有第 1 步用平视）
------------------------------------
    qr        → front（icSpring，640x480@15）
    turntable / ring / stacked / material → top（DECKXIN，1280x720）

⚠️ 设备号一律用 /dev/v4l/by-id/，不要用 /dev/videoN —— 编号会随上电顺序漂。
"""

from __future__ import annotations

import glob
import os
import re
import time
from pathlib import Path

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, String

from car_interfaces.msg import (
    VisionMaterial,
    VisionRing,
    VisionTaskCode,
    VisionTurntable,
)

from .convert import (
    material_fields,
    ring_fields,
    task_code_fields,
    turntable_fields,
)


# 模式名与 vision_flow_debug.py 的 MODE_CLASSES 保持一致，方便对照。
MODES = ("idle", "qr", "turntable", "ring", "stacked", "material")

# 每个模式用哪一路相机。lane（行车）暂不实现：车道/障碍检测还留在旧脚本里。
CAMERA_FOR_MODE = {
    "idle": None,
    "qr": "front",
    "turntable": "top",
    "ring": "top",
    "stacked": "top",
    "material": "top",
}

# 同一个二维码要连续多少帧解出一致才置 valid=true，滤掉偶发误读。
QR_REQUIRED_STABLE_FRAMES = 3


def resolve_vision_dir(param_value: str) -> Path:
    """定位 vision/ 目录（含 turntable_detector.py 的那层）。"""

    candidates = []
    if param_value:
        candidates.append(Path(param_value).expanduser())
    here = Path(__file__).resolve()
    # .../raspberrypi/ros2_ws/src/vision_node/vision_node/vision_node.py
    #   parents[4] = .../raspberrypi  →  /vision
    if len(here.parents) > 4:
        candidates.append(here.parents[4] / "vision")
    candidates.append(Path.cwd() / "vision")

    for candidate in candidates:
        if (candidate / "turntable_detector.py").is_file():
            return candidate.resolve()
    raise FileNotFoundError(
        "找不到 vision/ 目录（应含 turntable_detector.py）。"
        f"已尝试：{[str(item) for item in candidates]}。"
        "请用参数 vision_dir 指定。"
    )


def load_calibration_arrays(path: Path):
    """读相机内参 / 畸变系数 / 新内参。只为建 remap，不依赖主程序。

    刻意不复用 main.py 里的 load_calibration()：那会把整个主程序
    （连同串口、检测器、全局状态）都 import 进来，对一个节点来说太重。
    """

    resolved = Path(path).expanduser()
    storage = cv2.FileStorage(str(resolved), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise FileNotFoundError(f"无法读取相机标定文件：{resolved}")
    try:
        width = int(storage.getNode("image_width").real())
        height = int(storage.getNode("image_height").real())
        matrix = storage.getNode("camera_matrix").mat()
        distortion = storage.getNode("distortion_coefficients").mat()
        new_matrix = storage.getNode("new_camera_matrix").mat()
    finally:
        storage.release()

    if matrix is None or distortion is None or new_matrix is None:
        raise ValueError(f"标定文件缺少相机矩阵：{resolved}")
    return (width, height), matrix, distortion, new_matrix


def list_video_nodes():
    """当前系统里所有 /dev/video* 节点，按节点号升序。"""

    nodes = glob.glob("/dev/video*")
    return sorted(
        nodes,
        key=lambda path: int(re.search(r"(\d+)$", path).group(1))
        if re.search(r"(\d+)$", path) else 0,
    )


def node_name(node_path: str) -> str:
    """读 /sys 里这个节点报告的名字，例如 'DECKXIN: DECKXIN'。"""

    match = re.search(r"(\d+)$", node_path)
    if not match:
        return ""
    try:
        return Path(
            f"/sys/class/video4linux/video{match.group(1)}/name"
        ).read_text(encoding="utf-8", errors="replace").strip()
    except OSError:
        return ""


def describe_nodes() -> str:
    """给报错用的节点清单：'video0=DECKXIN: DECKXIN'。"""

    parts = []
    for node in list_video_nodes():
        parts.append(f"{os.path.basename(node)}={node_name(node) or '?'}")
    return ", ".join(parts) if parts else "（一个都没有）"


def open_camera(node_path: str, width: int, height: int, fps: float):
    """按 MJPG 打开一个 /dev/videoN。返回 (cap, 说明)；失败时 cap 为 None。

    只接受真实节点路径 —— OpenCV 的 V4L2 后端不认 by-id 软链接，
    传软链接会报 "backend is generally available but can't be used to capture by name"。
    """

    cap = cv2.VideoCapture(node_path, cv2.CAP_V4L2)
    if not cap.isOpened():
        return None, f"cannot open {node_path}"
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 4)

    for _ in range(20):
        ok, frame = cap.read()
        if ok and frame is not None:
            actual = (frame.shape[1], frame.shape[0])
            if actual != (width, height):
                cap.release()
                return None, (
                    f"requested {width}x{height}, got {actual[0]}x{actual[1]}"
                )
            return cap, f"{actual[0]}x{actual[1]}"
    cap.release()
    return None, f"opened but no frame: {node_path}"


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")

        # ---------------- 参数 ----------------
        # 相机选择策略：默认按 /sys/class/video4linux/videoN/name 里的设备名匹配，
        # 不怕节点号随上电顺序漂移。想强制钉死某个节点，把 top_device / front_device
        # 填成 /dev/videoN（注意 OpenCV 不认 /dev/v4l/by-id 软链接）。
        self.declare_parameter("top_device", "")
        self.declare_parameter("front_device", "")
        # 匹配词支持逗号分隔的多个候选；同一台设备在 v4l2-ctl 和 /sys 里
        # 报的名字可能不同（实测 DECKXIN 与 DECXIN），所以写两个兜住。
        self.declare_parameter("top_match", "DECXIN,DECKXIN")
        self.declare_parameter("front_match", "icspring")
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("frame_id", "camera")
        self.declare_parameter("vision_dir", "")
        self.declare_parameter("undistort", True)
        # 每 2 秒把当前参与识别的画面存成 JPEG，路径 debug_dump_dir/<模式名>.jpg。
        # 节点本身没有窗口，detected=false 时靠这个确认「相机到底看到了什么」。
        self.declare_parameter("debug_dump", False)
        self.declare_parameter("debug_dump_dir", "/tmp/vision_node")
        # 两路相机挂在同一个 USB 控制器上时，同时开流可能带宽不够。
        # 置 true 后切模式会关掉另一路。
        self.declare_parameter("close_inactive_cameras", False)

        self._top_device = self.get_parameter("top_device").value
        self._front_device = self.get_parameter("front_device").value
        self._top_match = self.get_parameter("top_match").value
        self._front_match = self.get_parameter("front_match").value
        self._width = int(self.get_parameter("width").value)
        self._height = int(self.get_parameter("height").value)
        self._fps = float(self.get_parameter("fps").value)
        self._frame_id = self.get_parameter("frame_id").value
        self._close_inactive = bool(
            self.get_parameter("close_inactive_cameras").value
        )
        self._undistort = bool(self.get_parameter("undistort").value)
        self._debug_dump = bool(self.get_parameter("debug_dump").value)
        self._debug_dump_dir = Path(
            self.get_parameter("debug_dump_dir").value
        )
        self._vision_dir = resolve_vision_dir(
            self.get_parameter("vision_dir").value
        )

        # ---------------- 状态 ----------------
        self._mode = "idle"
        self._cameras = {}          # "top" / "front" → VideoCapture
        self._maps = None           # 俯视相机的畸变矫正映射
        self._detectors = {}        # 懒加载
        self._qr_detector = None
        self._qr_text = ""
        self._qr_frames = 0
        self._qr_confirmed = ""
        self._last_dump_s = 0.0

        # ---------------- 发布者 ----------------
        self._pub_turntable = self.create_publisher(
            VisionTurntable, "/vision/turntable", 10
        )
        self._pub_ring = self.create_publisher(
            VisionRing, "/vision/ring", 10
        )
        self._pub_stacked = self.create_publisher(
            VisionRing, "/vision/stacked", 10
        )
        self._pub_material = self.create_publisher(
            VisionMaterial, "/vision/material", 10
        )
        self._pub_task_code = self.create_publisher(
            VisionTaskCode, "/vision/task_code", 10
        )

        # ---------------- 订阅者 ----------------
        self.create_subscription(
            String, "/vision/mode", self._on_mode, 10
        )

        self._load_configs()

        rate = float(self.get_parameter("rate_hz").value)
        if rate <= 0.0:
            raise ValueError("rate_hz 必须大于 0")
        self._timer = self.create_timer(1.0 / rate, self._on_tick)

        self.get_logger().info(
            "vision_node 就绪：top 匹配 "
            f"{self._match_patterns('top')}，front 匹配 "
            f"{self._match_patterns('front')}"
        )
        self.get_logger().info(
            f"vision/ 目录 = {self._vision_dir}；模式等待 /vision/mode"
        )

    # ---------------- 配置与检测器 ----------------
    def _load_configs(self):
        """只读配置、建检测器对象，不开相机。配置错了要尽早失败。"""

        import sys

        if str(self._vision_dir) not in sys.path:
            sys.path.insert(0, str(self._vision_dir))

        from turntable_detector import (  # noqa: PLC0415
            TurntableDetector,
            load_turntable_detector_config,
        )
        from turntable_material_seed import (  # noqa: PLC0415
            ThreeMaterialTurntableSeeder,
            load_three_material_seed_config,
        )
        from ring_station_detector import (  # noqa: PLC0415
            RingStationDetector,
            load_ring_station_config,
        )
        from stacked_ring_detector import (  # noqa: PLC0415
            StackedRingDetector,
            camera_geometry_signature,
            load_stacked_ring_config,
        )
        from material_pickup_detector import (  # noqa: PLC0415
            MaterialPickupDetector,
            load_material_pickup_config,
        )
        from 颜色标定 import load_color_config  # noqa: PLC0415

        config_dir = self._vision_dir / "config"
        calibration = config_dir / "realtek_rgb_camera.yaml"

        color_config = load_color_config(
            config_dir / "material_color_thresholds.json"
        )
        ring_config = load_ring_station_config(
            config_dir / "ring_station_detector.json"
        )

        seeder = ThreeMaterialTurntableSeeder(
            load_three_material_seed_config(
                config_dir / "turntable_material_seed.json"
            ),
            color_config,
        )
        stacked = StackedRingDetector(
            load_stacked_ring_config(config_dir / "stacked_ring_detector.json"),
            ring_config,
        )
        # 叠放位检测器必须知道相机几何，否则第 13 步会拒跑。
        stacked.set_camera_geometry(
            camera_geometry_signature(
                self._width, self._height, self._undistort, calibration
            )
        )

        self._detectors = {
            "turntable": TurntableDetector(
                load_turntable_detector_config(
                    config_dir / "turntable_detector.json"
                ),
                seeder,
            ),
            "ring": RingStationDetector(ring_config),
            "stacked": stacked,
            "material": MaterialPickupDetector(
                load_material_pickup_config(
                    config_dir / "material_pickup_detector.json"
                ),
                color_config,
            ),
        }

        if self._undistort:
            size, matrix, distortion, new_matrix = load_calibration_arrays(
                calibration
            )
            if size != (self._width, self._height):
                raise ValueError(
                    f"标定参数适用于 {size[0]}x{size[1]}，"
                    f"与 width/height 不一致"
                )
            self._maps = cv2.initUndistortRectifyMap(
                matrix, distortion, None, new_matrix, size, cv2.CV_16SC2
            )

    # ---------------- 相机 ----------------
    def _match_patterns(self, key: str):
        """解析某一路的匹配词，支持逗号分隔的多个候选（全部小写）。

        同一台设备在 v4l2-ctl --info 和 /sys 里报的名字可能不一样
        （实测：一个是 DECKXIN，另一个是 DECXIN），所以允许多写几个候选。
        """

        raw = self._top_match if key == "top" else self._front_match
        return [
            item.strip().lower()
            for item in str(raw or "").split(",")
            if item.strip()
        ]

    def _candidate_nodes(self, key: str):
        """给某一路相机列出候选 /dev/videoN。

        优先用显式指定的 top_device / front_device；否则按 /sys 里的设备名匹配。

        同一台相机会同时暴露「视频节点」和「元数据节点」，两者名字完全一样，
        单看名字分不出来。所以这里把全部候选返回，由调用方逐个尝试打开 ——
        元数据节点打不开，自然就被跳过了。
        """

        explicit = self._top_device if key == "top" else self._front_device
        if explicit:
            return [explicit]

        patterns = self._match_patterns(key)
        if not patterns:
            return []
        return [
            node
            for node in list_video_nodes()
            if any(
                pattern in node_name(node).lower() for pattern in patterns
            )
        ]

    def _ensure_camera(self, key: str):
        if key in self._cameras:
            return self._cameras[key]

        width, height, fps = self._width, self._height, self._fps
        if key == "front":
            # 扫码不需要高分辨率，降下来省带宽和 CPU。
            width, height, fps = 640, 480, 15.0

        candidates = self._candidate_nodes(key)
        if not candidates:
            self.get_logger().error(
                f"{key} 相机没找到（匹配 {self._match_patterns(key)}）；"
                f"现有节点：{describe_nodes()}"
            )
            return None

        last_note = ""
        for node in candidates:
            cap, note = open_camera(node, width, height, fps)
            if cap is None:
                last_note = note
                continue
            self._cameras[key] = cap
            self.get_logger().info(
                f"{key} 相机已打开：{node} ({note}) 名称={node_name(node)}"
            )
            if self._close_inactive:
                for other in ("top", "front"):
                    if other != key and other in self._cameras:
                        self._cameras.pop(other).release()
                        self.get_logger().info(f"{other} 相机已关闭（节省带宽）")
            return cap

        self.get_logger().error(
            f"{key} 相机候选都打不开：{candidates}；最后错误：{last_note}"
        )
        return None

    def _flush(self, cap, count=4):
        """切模式后丢掉缓冲区里的旧帧，避免拿到上一轮的画面。"""

        for _ in range(count):
            cap.read()

    # ---------------- 模式切换 ----------------
    def _on_mode(self, msg: String):
        mode = str(msg.data or "").strip().lower()
        if mode not in MODES:
            self.get_logger().warn(
                f"未知模式 {mode!r}，可选 {list(MODES)}"
            )
            return
        if mode == self._mode:
            return

        self.get_logger().info(f"模式切换：{self._mode} → {mode}")
        self._mode = mode

        # 复位所有检测器的跨帧状态，避免把上一轮的稳定窗口带过来。
        for detector in self._detectors.values():
            detector.reset()
        self._qr_text, self._qr_frames, self._qr_confirmed = "", 0, ""

        key = CAMERA_FOR_MODE.get(mode)
        if key is not None:
            cap = self._ensure_camera(key)
            if cap is not None:
                self._flush(cap)

    # ---------------- 每帧 ----------------
    def _on_tick(self):
        key = CAMERA_FOR_MODE.get(self._mode)
        if key is None:
            return
        cap = self._ensure_camera(key)
        if cap is None:
            return

        ok, raw = cap.read()
        if not ok or raw is None:
            self.get_logger().warn(
                f"{key} 相机读取失败，释放后重试", throttle_duration_sec=2.0
            )
            stale = self._cameras.pop(key, None)
            if stale is not None:
                stale.release()
            return

        frame = raw
        if key == "top" and self._maps is not None:
            frame = cv2.remap(
                raw, self._maps[0], self._maps[1], cv2.INTER_LINEAR
            )

        self._maybe_dump(frame)
        header = self._make_header()
        if self._mode == "turntable":
            self._handle_turntable(frame, header)
        elif self._mode == "ring":
            self._handle_ring(frame, header)
        elif self._mode == "stacked":
            self._handle_stacked(frame, header)
        elif self._mode == "material":
            self._handle_material(frame, header)
        elif self._mode == "qr":
            self._handle_qr(frame, header)

    def _make_header(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._frame_id
        return header

    def _maybe_dump(self, frame):
        """每 2 秒把参与识别的画面存成 JPEG，供现场确认取景和光照。

        节点没有窗口，detected=false 时只看话题完全不知道发生了什么。
        文件名按模式命名（<mode>.jpg），每次覆盖，不会堆满磁盘。
        """

        if not self._debug_dump:
            return
        now = time.monotonic()
        if now - self._last_dump_s < 2.0:
            return
        self._last_dump_s = now
        try:
            self._debug_dump_dir.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(
                str(self._debug_dump_dir / f"{self._mode}.jpg"), frame
            )
        except (OSError, cv2.error) as error:
            self.get_logger().warn(
                f"调试图写入失败：{error}", throttle_duration_sec=10.0
            )

    # ---------------- 各模式 ----------------
    def _handle_turntable(self, frame, header):
        result = self._detectors["turntable"].detect(frame)
        msg = VisionTurntable()
        msg.header = header
        for key, value in turntable_fields(result).items():
            setattr(msg, key, value)
        self._pub_turntable.publish(msg)

    def _handle_ring(self, frame, header):
        result = self._detectors["ring"].detect(frame)
        msg = VisionRing()
        msg.header = header
        for key, value in ring_fields(result).items():
            setattr(msg, key, value)
        self._pub_ring.publish(msg)

    def _handle_stacked(self, frame, header):
        result = self._detectors["stacked"].detect(frame)
        msg = VisionRing()
        msg.header = header
        for key, value in ring_fields(result).items():
            setattr(msg, key, value)
        self._pub_stacked.publish(msg)

    def _handle_material(self, frame, header):
        result = self._detectors["material"].detect(
            frame, timestamp=time.monotonic()
        )
        msg = VisionMaterial()
        msg.header = header
        for key, value in material_fields(result).items():
            setattr(msg, key, value)
        self._pub_material.publish(msg)

    def _handle_qr(self, frame, header):
        if self._qr_detector is None:
            self._qr_detector = cv2.QRCodeDetector()

        text = ""
        try:
            decoded, _points, _ = self._qr_detector.detectAndDecode(frame)
            text = (decoded or "").strip()
        except (cv2.error, AttributeError):
            text = ""

        if text:
            if text == self._qr_text:
                self._qr_frames += 1
            else:
                self._qr_text, self._qr_frames = text, 1
        else:
            self._qr_text, self._qr_frames = "", 0

        valid = self._qr_frames >= QR_REQUIRED_STABLE_FRAMES
        if valid:
            self._qr_confirmed = self._qr_text

        msg = VisionTaskCode()
        msg.header = header
        for key, value in task_code_fields(
            self._qr_confirmed if valid else "", valid
        ).items():
            setattr(msg, key, value)
        self._pub_task_code.publish(msg)

    # ---------------- 退出 ----------------
    def shutdown(self):
        for cap in self._cameras.values():
            cap.release()
        self._cameras.clear()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VisionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
