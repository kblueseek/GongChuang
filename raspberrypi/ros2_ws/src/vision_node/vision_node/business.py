"""业务接口与注册表。

新视觉动作 = 在 businesses/ 下加一个文件：

    from ..business import Business, register

    @register
    class XxxBusiness(Business):
        name = 'xxx'
        def run(self, ctx, args, stop) -> dict:
            ...

businesses/__init__.py 会自动 import 该目录下的所有模块，节点代码不用改。
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict


@dataclass
class BusinessContext:
    """节点注入给业务的运行环境。

    业务不自己开摄像头、不自己初始化 ROS：
      · ctx.camera   —— 整个进程共用的摄像头（只有一个主人）
      · ctx.node     —— vision_node，提供 send_gimbal_vel 等和控制层同名的接口
      · ctx.vision_dir —— vision/ 目录（算法模块和 config/ 都在这里）
    """

    node: Any
    camera: Any
    vision_dir: Path
    headless: bool = True
    device: str = '/dev/video0'
    width: int = 1280
    height: int = 720
    fps: float = 30.0
    cam_rotation: int = 180
    config: Dict[str, Any] = field(default_factory=dict)


# name -> Business 子类。节点启动时只认这里的东西。
BUSINESSES: Dict[str, type] = {}


def register(cls):
    """把业务类挂到注册表。类必须有非空 name。"""
    name = getattr(cls, 'name', '')
    if not name:
        raise ValueError(f'{cls.__name__} 缺少 name')
    if name in BUSINESSES:
        raise ValueError(f'业务名重复：{name}')
    BUSINESSES[name] = cls
    return cls


class Business:
    """所有视觉业务的基类。

    run() 是阻塞的：从开始执行到返回结果期间，节点只让这一个业务跑。
    stop 是 threading.Event：节点收到 {"action":"stop"} 时置位，
    业务应该尽快退出并返回 {'ok': False, 'reason': 'stopped'} 之类的结构。
    """

    name = ''

    def run(self, ctx: BusinessContext, args: dict,
            stop: threading.Event) -> dict:
        raise NotImplementedError
