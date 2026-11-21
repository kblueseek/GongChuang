"""摄像头封装：整个进程只开一次，所有业务共用。

为什么放在节点里而不是每个业务自己开：
  · 同一时刻只有一个程序能打开 /dev/video0；
  · 反复开关摄像头每次都要重新预热、重新收敛自动曝光，切换业务要等 1~3 秒；
  · 业务只关心“给我最新一帧”，不应该关心 V4L2 细节。
"""

from __future__ import annotations

import threading


class CameraError(RuntimeError):
    """摄像头打不开/读取失败。"""


class Camera:
    def __init__(self, device: str = '/dev/video0', width: int = 1280,
                 height: int = 720, fps: float = 30.0, warmup: int = 15):
        import cv2

        self._cv2 = cv2
        self.device = device
        self.width = int(width)
        self.height = int(height)
        self.fps = float(fps)

        self._cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            raise CameraError(f'打不开摄像头 {device}')

        # 和 gimbal_track_pick / vision_put_align 保持完全一致的采集参数。
        self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap.set(cv2.CAP_PROP_FPS, self.fps)
        # 只留 1 帧缓冲：业务读到的永远是最新画面，不会拿到 V4L2 队列里
        # 几百毫秒前的旧帧（跟随环会平白多一段延迟）。
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # 预热：头几帧常是空帧/曝光还没收敛。
        for _ in range(int(warmup)):
            ok, _frame = self._cap.read()
            if ok:
                break

        self._lock = threading.Lock()
        self._closed = False

    @property
    def closed(self) -> bool:
        return self._closed

    def read(self):
        """读一帧，返回 (ok, frame)。业务直接调用这个。"""
        with self._lock:
            if self._closed or self._cap is None:
                return False, None
            return self._cap.read()

    def grab(self):
        """只取帧不返回画面，用于空闲时排空缓冲。"""
        with self._lock:
            if self._closed or self._cap is None:
                return False
            return self._cap.grab()

    def release(self):
        with self._lock:
            self._closed = True
            if self._cap is not None:
                try:
                    self._cap.release()
                finally:
                    self._cap = None
