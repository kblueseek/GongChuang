"""Windows 预览用的 rclpy 空壳。

只在本机预览时通过 sys.path 注入，树莓派上不会用到。
"""


def init(*args, **kwargs):
    pass


def shutdown(*args, **kwargs):
    pass


def spin_once(node=None, timeout_sec=0):
    """让节点的定时器回调有机会执行（GuiRosNode 本身没有定时器）。"""
    if node is not None:
        node._spin_timers()
