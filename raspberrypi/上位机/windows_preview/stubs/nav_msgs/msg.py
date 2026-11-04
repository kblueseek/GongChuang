"""motion_gui 用到的 nav_msgs/Odometry（预览占位）。"""


class _Header:
    def __init__(self):
        self.stamp = None
        self.frame_id = ''


class _Pos:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _Quat:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 1.0


class _Pose:
    def __init__(self):
        self.position = _Pos()
        self.orientation = _Quat()


class _PoseStamped:
    def __init__(self):
        self.pose = _Pose()


class Odometry:
    def __init__(self):
        self.header = _Header()
        self.pose = _PoseStamped()
