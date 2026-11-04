"""rclpy.node 的最小实现：发布/订阅/参数/定时器都只留在进程内。"""

import logging
import time


class _Pub:
    def __init__(self, topic):
        self.topic = topic
        self.sent = []          # 预览程序靠它拿到 GUI 发出去的指令

    def publish(self, msg):
        self.sent.append(msg)

    def get_subscription_count(self):
        # 预览里总当成"有节点在听"，这样 send_sequence() 会走正常下发分支
        return 1


class _Param:
    def __init__(self, value):
        self.value = value


class Node:
    def __init__(self, name='stub'):
        self._name = name
        self.pubs = {}
        self.params = {}
        self.timers = []
        self._subs = []

    def declare_parameter(self, name, default=None):
        self.params.setdefault(name, default)
        return _Param(self.params[name])

    def get_parameter(self, name):
        return _Param(self.params.get(name))

    def create_timer(self, period, callback):
        self.timers.append([time.monotonic() + period, float(period), callback])
        return len(self.timers)

    def create_publisher(self, msg_type, topic, qos=10):
        pub = _Pub(topic)
        self.pubs[topic] = pub
        return pub

    def create_subscription(self, msg_type, topic, callback, qos=10):
        self._subs.append((topic, callback))
        return None

    def destroy_node(self):
        pass

    def get_logger(self):
        return logging.getLogger('preview')

    def _spin_timers(self):
        now = time.monotonic()
        for t in self.timers:
            if now >= t[0]:
                t[0] = now + t[1]
                try:
                    t[2]()
                except Exception:
                    logging.getLogger('preview').exception('定时器回调出错')
