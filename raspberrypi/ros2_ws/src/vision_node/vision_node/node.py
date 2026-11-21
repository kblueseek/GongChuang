"""统一视觉节点（vision_node）。

一个进程、一个节点、一台摄像头，业务按注册表挂载：
    color_pick / track_debug / （下一步）vision_put / calibrate

接口（一行一个 JSON，方便和 motion_planner 对接）：

    /vision/command   std_msgs/String
        {"action":"color_pick","args":{"color":4,"timeout_s":60,"still_time":0.3}}
        {"action":"vision_put","args":{"ring":0,"target_x":640,"target_y":360}}
        {"action":"calibrate","args":{"x_m":-2.169,"y_m":1.061}}
        {"action":"stop"}
        {"action":"status"}

    /vision/result    std_msgs/String
        {"action":"color_pick","ok":true,"color":4,"yaw_deg":...}
        {"action":"calibrate","ok":true,"x_m":...}

    /vision/status    std_msgs/String
        {"ready":true,"businesses":[...]}、{"action":"color_pick","started":true} 等

线程模型（为什么这样分）：
  · 空闲：主线程 spin_once 收 /vision/command；
  · 运行：主线程把业务交给 worker，自己只 join，不再 spin；
  · worker 里跑业务，现有跟踪代码内部会自己 spin_once（/gimbal_state、stop）；
  · 同一时刻只有一个 spinner，不会出现两个线程同时 spin 一个节点。
"""

from __future__ import annotations

import json
import os
import sys
import threading
import time
import traceback
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String
from nav_msgs.msg import Odometry
from car_interfaces.msg import Pose2D

from .business import BUSINESSES, BusinessContext
from .camera import Camera
from . import businesses as _businesses  # noqa: F401  （import 时自动注册所有业务）


def _find_vision_dir() -> str:
    """从本文件向上找包含 gimbal_track_pick.py 的 vision/ 目录。

    colcon --symlink-install 时 __file__ 解析到源码树，vision/ 与 ros2_ws/
    同级；找不到就返回空串，由调用方回退到 $HOME 默认路径。
    """
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / 'vision' / 'gimbal_track_pick.py').is_file():
            return str(parent / 'vision')
    return ''


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # ---------------- 参数 ----------------
        self.declare_parameter('vision_dir', '')
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('cam_rotation', 180)
        self.declare_parameter('headless', True)
        self.declare_parameter('idle_grab_hz', 10.0)
        self.declare_parameter('camera_retry_s', 1.0)

        vision_dir = str(self.get_parameter('vision_dir').value).strip()
        if not vision_dir:
            vision_dir = _find_vision_dir()
        if not vision_dir:
            vision_dir = os.path.expanduser('~/GongChuang/raspberrypi/vision')
        self.vision_dir = Path(vision_dir)
        if str(self.vision_dir) not in sys.path:
            sys.path.insert(0, str(self.vision_dir))

        self.device = str(self.get_parameter('device').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.cam_rotation = int(self.get_parameter('cam_rotation').value)
        self.headless = bool(self.get_parameter('headless').value)
        # 没有桌面还开着预览，OpenCV 的 Qt 后端会直接 abort 整个进程
        # （不是 Python 异常，业务里的 try/except 拦不住）。这里提前降级。
        if (not self.headless
                and not os.environ.get('DISPLAY')
                and not os.environ.get('WAYLAND_DISPLAY')):
            self.headless = True
            self.get_logger().warning(
                '没有 DISPLAY/WAYLAND_DISPLAY：预览窗口已自动关闭（headless=true）')
        self._idle_period = 1.0 / max(1.0, float(
            self.get_parameter('idle_grab_hz').value))
        self._cam_retry_s = float(self.get_parameter('camera_retry_s').value)

        # ---------------- 和控制层共用的话题 ----------------
        # 名字和 gimbal_track_pick 完全一致，现有算法可以直接复用。
        self._pub_vel = self.create_publisher(Float32MultiArray, '/cmd_gimbal_vel', 10)
        self._pub_pos = self.create_publisher(Float32MultiArray, '/cmd_gimbal', 10)
        self._pub_active = self.create_publisher(Bool, '/motion_active', 10)
        self._pub_tune = self.create_publisher(Float32MultiArray, '/cmd_tune', 10)
        # 和 gimbal_track_pick.PutAlignNode 的字段同名，vision_put 迁移时直接复用。
        self._gimbal = [0.0, 0.0, 0.0]
        self._gimbal_ok = False
        self.create_subscription(Float32MultiArray, '/gimbal_state',
                                 self._cb_gimbal_state, 10)
        # calibrate 业务用：底盘位姿纠偏
        self._pub_pose = self.create_publisher(Pose2D, '/cmd_pose', 10)
        self._pub_set_odom = self.create_publisher(Pose2D, '/set_odom', 10)
        self._odom = None
        self._odom_t = 0.0
        self.create_subscription(Odometry, '/odom', self._cb_odom, 10)
        # F407 调参生效值：[参数号, 数值]。calibrate 用它取位置/航向环 Kp，
        # 避免和上位机调参页重复维护（位置环 0x05 / 转向环 0x08）。
        self._tune = {}
        self.create_subscription(Float32MultiArray, '/tune_state',
                                 self._cb_tune_state, 10)

        # ---------------- 业务接口 ----------------
        self.create_subscription(String, '/vision/command', self._cb_command, 10)
        self._pub_result = self.create_publisher(String, '/vision/result', 10)
        self._pub_status = self.create_publisher(String, '/vision/status', 10)

        # ---------------- 摄像头 ----------------
        self.camera = None
        self._cam_retry_t = 0.0
        self._last_idle = 0.0

        # ---------------- 调度状态 ----------------
        self._pending = None        # (action, args)
        self._worker = None
        self._stop = threading.Event()
        self._active = ''
        self._hold_stop = threading.Event()
        self._hold_thread = None

        self.get_logger().info(
            f'vision_node 就绪：vision_dir={self.vision_dir}，'
            f'业务={sorted(BUSINESSES)}')
        self._publish_status({'ready': True, 'businesses': sorted(BUSINESSES)})
        # 2Hz 心跳：motion_planner 用它确认 vision_node 还活着
        self.create_timer(2.0, self._status_heartbeat)
        # 启动就把摄像头打开并预热。失败不退出：命令到来时还会重试。
        self._ensure_camera()

    # ---------------- 兼容 gimbal_track_pick 的接口 ----------------

    def _cb_gimbal_state(self, msg):
        if len(msg.data) >= 3:
            self._gimbal = [float(v) for v in msg.data[:3]]
            self._gimbal_ok = True

    def _cb_odom(self, msg):
        q = msg.pose.pose.orientation
        import math
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._odom = (msg.pose.pose.position.x,
                      msg.pose.pose.position.y, yaw)
        self._odom_t = time.monotonic()

    def get_odom(self):
        """最新里程计 (x, y, yaw) 和时间戳；calibrate 业务用。"""
        return self._odom, self._odom_t

    def _cb_tune_state(self, msg):
        if len(msg.data) >= 2:
            self._tune[int(msg.data[0])] = float(msg.data[1])

    def get_tune(self, param_id: int, fallback=None):
        """取 F407 当前生效的调参值；没收到过就返回 fallback。"""
        value = self._tune.get(int(param_id))
        return fallback if value is None else float(value)

    def publish_pose(self, x: float, y: float, yaw: float):
        self._pub_pose.publish(Pose2D(x=float(x), y=float(y), theta=float(yaw)))

    def publish_set_odom(self, x: float, y: float, yaw: float):
        self._pub_set_odom.publish(
            Pose2D(x=float(x), y=float(y), theta=float(yaw)))

    def send_gimbal_vel(self, yaw: float, lift: float, extend: float):
        self._pub_vel.publish(Float32MultiArray(data=[yaw, lift, extend]))

    def send_gimbal_target(self, yaw: float, lift: float, extend: float):
        self._pub_pos.publish(
            Float32MultiArray(data=[float(yaw), float(lift), float(extend)]))

    def publish_active(self, active: bool):
        self._pub_active.publish(Bool(data=bool(active)))

    def send_tune(self, param_id: int, value: float):
        self._pub_tune.publish(
            Float32MultiArray(data=[float(param_id), float(value)]))

    # ---------------- 命令 ----------------

    def _cb_command(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:  # noqa: BLE001 - 一条坏命令不能带走节点
            self._publish_result({'ok': False,
                                  'reason': f'命令不是 JSON：{msg.data[:80]}'})
            return
        if not isinstance(data, dict):
            self._publish_result({'ok': False, 'reason': '命令必须是 JSON 对象'})
            return

        action = str(data.get('action') or data.get('cmd') or '').strip().lower()

        if action in ('stop', 'close', 'quit', 'exit'):
            self._stop.set()
            self._pending = None        # 还没启动的业务也一并取消
            if not self._active and self._worker is None:
                self._publish_result({'action': 'stop', 'ok': True})
            else:
                self._publish_status({'stopping': self._active})
            return

        if action == 'status':
            self._publish_status({
                'active': self._active or 'idle',
                'businesses': sorted(BUSINESSES),
                'camera': ('open' if (self.camera is not None
                                      and not self.camera.closed) else 'closed'),
            })
            return

        if not action:
            self._publish_result({'ok': False, 'reason': '缺少 action'})
            return
        if (self._active or self._worker is not None
                or self._pending is not None):
            busy = self._active or (self._pending[0] if self._pending else '')
            self._publish_result({'action': action, 'ok': False,
                                  'reason': f'忙：{busy}'})
            return
        if action not in BUSINESSES:
            self._publish_result({'action': action, 'ok': False,
                                  'reason': f'未知业务：{action}'
                                            f'（可用 {sorted(BUSINESSES)}）'})
            return

        args = data.get('args') or data.get('params') or {}
        if not isinstance(args, dict):
            self._publish_result({'action': action, 'ok': False,
                                  'reason': 'args 必须是 JSON 对象'})
            return
        self._pending = (action, args)

    # ---------------- 主循环 ----------------

    def run(self):
        while rclpy.ok():
            if self._pending is not None:
                action, args = self._pending
                self._pending = None
                self._run_action(action, args)
                continue

            rclpy.spin_once(self, timeout_sec=0.02)
            self._idle_tick()

    def _idle_tick(self):
        now = time.monotonic()
        if now - self._last_idle < self._idle_period:
            return
        self._last_idle = now
        cam = self._ensure_camera()
        if cam is not None:
            try:
                cam.grab()      # 排空 V4L2 队列，业务第一帧就是新画面
            except Exception:   # noqa: BLE001 - 单帧失败不影响节点
                pass

    def _ensure_camera(self):
        if self.camera is not None and not self.camera.closed:
            return self.camera
        now = time.monotonic()
        if now < self._cam_retry_t:
            return None
        self._cam_retry_t = now + self._cam_retry_s
        try:
            self.camera = Camera(self.device, self.width, self.height, self.fps)
            self.get_logger().info(
                f'摄像头已打开：{self.device} '
                f'{self.width}x{self.height}@{self.fps:g}')
        except Exception as error:  # noqa: BLE001 - 打不开不退出，等下次重试
            self.camera = None
            self.get_logger().warning(f'摄像头打不开（{self.device}）：{error}')
        return self.camera

    def _run_action(self, action: str, args: dict):
        """在主线程里跑一个业务。

        为什么不再放到 worker 线程：
          · OpenCV/Qt 的窗口必须由同一个线程创建和刷新。以前每个业务都在新的
            worker 里 new window，业务结束后窗口没人 waitKey，Qt 事件循环停住，
            系统就把进程标成"无响应"，之后再开窗口/切业务就会卡死甚至被杀。
          · 业务内部本来就会自己 spin_once（收 stop / gimbal_state / 定时器），
            放在主线程不会出现两个 spinner。
        """
        if self._ensure_camera() is None:
            self._publish_result({'action': action, 'ok': False,
                                  'reason': f'摄像头不可用（{self.device}）'})
            return

        cls = BUSINESSES[action]
        try:
            business = cls()
        except Exception as error:  # noqa: BLE001
            self._publish_result({'action': action, 'ok': False,
                                  'reason': f'业务初始化失败：{error}'})
            return

        self._stop.clear()
        self._active = action
        self._hold_stop.clear()
        self._hold_thread = threading.Thread(
            target=self._hold_loop, name=f'hold-{action}', daemon=True)
        self._hold_thread.start()
        self._publish_status({'action': action, 'started': True})
        self.get_logger().info(f'开始业务 {action}：{args}')

        ctx = BusinessContext(
            node=self, camera=self.camera, vision_dir=self.vision_dir,
            headless=self.headless, device=self.device,
            width=self.width, height=self.height, fps=self.fps,
            cam_rotation=self.cam_rotation,
        )

        result = {}
        try:
            result = business.run(ctx, args, self._stop) or {}
            if not isinstance(result, dict):
                result = {'ok': False,
                          'reason': f'{type(result).__name__} 返回的不是 dict'}
        except Exception as error:  # noqa: BLE001 - 一次业务失败不能带走节点
            traceback.print_exc()
            result = {'ok': False,
                      'reason': f'{type(error).__name__}: {error}'}
        finally:
            self._hold_stop.set()
            if self._hold_thread is not None:
                self._hold_thread.join(timeout=1.0)
            self.publish_active(False)
            result.setdefault('action', action)
            self._publish_result(result)
            self.get_logger().info(f'业务 {action} 结束：{result}')
            self._active = ''
            # 业务退出时如果留下预览窗口，主线程这里统一收掉。
            # 否则窗口留在那里没人 waitKey，Qt 事件循环停住 → 系统判定无响应。
            if not self.headless:
                try:
                    import cv2
                    cv2.destroyAllWindows()
                    cv2.waitKey(1)
                except Exception:  # noqa: BLE001 - 没有图形后端时忽略
                    pass

    def _hold_loop(self):
        """业务运行期间 10Hz 发 /motion_active=True，让手柄让出云台。

        和 motion_planner 跑序列用的是同一条约定；业务结束发 False 交还。
        """
        while not self._hold_stop.is_set():
            self.publish_active(True)
            self._hold_stop.wait(0.1)

    def _publish_result(self, payload: dict):
        self._pub_result.publish(
            String(data=json.dumps(payload, ensure_ascii=False)))

    def _publish_status(self, payload: dict):
        self._pub_status.publish(
            String(data=json.dumps(payload, ensure_ascii=False)))

    def _status_heartbeat(self):
        self._publish_status({
            'alive': True,
            'active': self._active or 'idle',
            'businesses': sorted(BUSINESSES),
            'camera': ('open' if (self.camera is not None
                                  and not self.camera.closed) else 'closed'),
        })

    def shutdown(self):
        self._stop.set()
        if self._worker is not None:
            self._worker.join(timeout=2.0)
        if self.camera is not None:
            self.camera.release()
            self.camera = None


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
