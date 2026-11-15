"""motion_gui 用到的 geometry_msgs/Twist（预览占位）。

只为了让 windows_preview 能在没装 ROS 的 Windows 上开起来，字段和真消息一致。
"""


class _Vec:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class Twist:
    def __init__(self):
        self.linear = _Vec()
        self.angular = _Vec()
