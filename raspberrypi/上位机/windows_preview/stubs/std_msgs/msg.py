"""motion_gui 用到的 std_msgs 消息类型（预览占位）。"""


class Bool:
    def __init__(self, data=False):
        self.data = data


class Int32:
    def __init__(self, data=0):
        self.data = int(data)


class String:
    def __init__(self, data=''):
        self.data = data


class Int32MultiArray:
    def __init__(self, data=None):
        self.data = list(data or [])


class Float32MultiArray:
    def __init__(self, data=None):
        self.data = list(data or [])
