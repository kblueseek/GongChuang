#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""F407 桥接节点：ROS2 世界和 F407 单片机之间的"翻译官"（无线版）。

================ 大白话：这个节点在干什么 ================

树莓派和 F407 各用一套电源，中间**没有共地线**，所以全部走无线，分两条单向链路：

    上行（树莓派 → F407）：蓝牙。把 /cmd_vel、/cmd_servo 等话题翻译成 PiComm 帧，
                           写进 F407 板载蓝牙模组的串口透传特征值。
    下行（F407 → 树莓派）：LoRa。F407 把里程计/状态帧从 UART4 发给 LoRa 模组，
                           树莓派侧 LoRa 接在 UART 上，本节点当普通串口收。

为什么这么分？蓝牙延迟低、有链路层重传，适合"指令"；LoRa 距离远、能容忍丢包，
而且只发不收，天然避开半双工来回切换的麻烦。两端跑的是同一套 PiComm 帧。

================ 可配置参数（ros2 param） ================

  uplink         ble | rfcomm | wire     上行方式，默认 ble
  ble_address    蓝牙模组 MAC（留空=自动扫描）
  ble_name       蓝牙模组名字关键字（留空=不限）
  ble_write_uuid 写入特征值 UUID（留空=自动探测）
  rfcomm_device  经典蓝牙串口（rfcomm bind 出来的口），默认 /dev/rfcomm0
  wire_device    有线备用串口，默认 /dev/ttyAMA0
  downlink       lora | wire             下行方式，默认 lora
  lora_device    LoRa 串口设备，默认 /dev/ttyAMA2（Pi5 加 dtoverlay=uart2-pi5 后）
  lora_baudrate  LoRa 串口波特率，默认 115200（必须和模组、F407 一致）
  frame_id       里程计坐标系名，默认 odom

改方式举例：
  ros2 run f407_bridge f407_bridge_node --ros-args -p uplink:=rfcomm -p downlink:=wire
"""

import math   # 用于算四元数（姿态的一种数学表示）

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Twist   # Quaternion=四元数, Twist=速度
from car_interfaces.msg import Pose2D             # 自定义平面位姿(x,y,theta)
from nav_msgs.msg import Odometry                 # 里程计消息
from std_msgs.msg import Bool, Float32MultiArray, Int32, Int32MultiArray

# 从本包内导入协议模块（pi_comm.py 是和 comm/pi_comm.py 内容一致的副本）
from f407_bridge.pi_comm import (
    BleLink,          # 蓝牙上行（BLE GATT）
    PiCommLink,       # 串口链路（LoRa 下行 / 有线备用）
    RfcommLink,       # 经典蓝牙 SPP（当串口用）
    RPT_ODOM,         # 回传数据编号：里程计
    RPT_STATUS,       # 回传数据编号：状态
    parse_odom,       # 解析里程计的函数
    parse_status,     # 解析状态的函数
)

# F407 超过这么久没有任何回传，就认为它掉线了（状态 5Hz + 里程计 20Hz，1.5s 足够宽容）
F407_OFFLINE_TIMEOUT_S = 1.5


class F407BridgeNode(Node):
    def __init__(self):
        super().__init__('f407_bridge')   # 节点名

        # ================= 参数声明 =================
        self.declare_parameter('uplink', 'ble')                  # ble / rfcomm / wire
        self.declare_parameter('ble_address', '')                # 留空=自动扫描
        self.declare_parameter('ble_name', '')                   # 留空=不限
        self.declare_parameter('ble_write_uuid', '')             # 留空=自动探测
        self.declare_parameter('ble_scan_timeout', 8.0)
        self.declare_parameter('rfcomm_device', '/dev/rfcomm0')
        self.declare_parameter('wire_device', '/dev/ttyAMA0')
        self.declare_parameter('downlink', 'lora')               # lora / wire
        self.declare_parameter('lora_device', '/dev/ttyAMA2')
        self.declare_parameter('lora_baudrate', 115200)
        self.declare_parameter('frame_id', 'odom')

        # ================= 建链路 =================
        self._up = self._make_uplink()
        self._down = self._make_downlink()

        # ================= 方向1：订阅话题（ROS2 -> F407） =================
        self.create_subscription(Twist, '/cmd_vel', self._cb_cmd_vel, 10)
        self.create_subscription(Float32MultiArray, '/cmd_servo', self._cb_cmd_servo, 10)
        self.create_subscription(Bool, '/cmd_enable', self._cb_cmd_enable, 10)
        self.create_subscription(Pose2D, '/cmd_pose', self._cb_cmd_pose, 10)
        self.create_subscription(Float32MultiArray, '/cmd_gimbal', self._cb_cmd_gimbal, 10)
        self.create_subscription(Int32, '/cmd_gimbal_home', self._cb_cmd_gimbal_home, 10)
        self.create_subscription(Int32, '/cmd_led', self._cb_cmd_led, 10)

        # ================= 方向2：发布话题（F407 -> ROS2） =================
        self._pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self._pub_status = self.create_publisher(Int32MultiArray, '/robot_status', 10)
        self._pub_online = self.create_publisher(Bool, '/f407_online', 10)      # F407 是否有回传
        self._pub_ble = self.create_publisher(Bool, '/ble_connected', 10)       # 蓝牙是否连着

        # ================= 定时器 =================
        # 心跳 5Hz：F407 侧 500ms 收不到任何指令就会自主失能，必须比它快。
        self._heartbeat_timer = self.create_timer(0.2, self._on_heartbeat)
        # 收 LoRa 数据 20Hz：F407 侧里程计 10Hz、状态 2Hz，20Hz 轮询足够且不占 CPU。
        self._poll_timer = self.create_timer(0.05, self._on_poll)
        # 链路状态 2Hz：发布"F407 是否在线 / 蓝牙是否连着"，给状态屏和别的节点用。
        self._link_timer = self.create_timer(0.5, self._on_link_status)

        self._last_rx_s = 0.0          # 最近一次收到 F407 回传的时间
        self._last_online = None       # 上一次发布的在线状态（避免重复打印日志）
        self._last_ble = None

        self.get_logger().info(f'桥接节点就绪：上行={type(self._up).__name__}，下行={self._down.device}')

    # ================= 链路创建 =================

    def _make_uplink(self):
        """按 uplink 参数建上行链路，返回的对象都提供 send_xxx() 方法。"""
        mode = str(self.get_parameter('uplink').value).lower()
        if mode == 'wire':
            device = self.get_parameter('wire_device').value
            self.get_logger().info(f'上行：有线 {device}')
            return PiCommLink(device=device, baudrate=115200)
        if mode == 'rfcomm':
            device = self.get_parameter('rfcomm_device').value
            self.get_logger().info(f'上行：经典蓝牙 SPP {device}')
            return RfcommLink(device=device)
        # 默认走 BLE
        address = self.get_parameter('ble_address').value
        name = self.get_parameter('ble_name').value
        uuid = self.get_parameter('ble_write_uuid').value
        self.get_logger().info(
            f'上行：BLE（地址={address or "自动扫描"} 名字={name or "不限"}）'
        )
        return BleLink(
            address=address,
            name=name,
            write_uuid=uuid,
            scan_timeout=float(self.get_parameter('ble_scan_timeout').value),
            log=self.get_logger().info,
        )

    def _make_downlink(self):
        """按 downlink 参数建下行链路（只收）。"""
        mode = str(self.get_parameter('downlink').value).lower()
        if mode == 'wire':
            device = self.get_parameter('wire_device').value
            baud = 115200
            self.get_logger().info(f'下行：有线 {device} @ {baud}')
        else:
            device = self.get_parameter('lora_device').value
            baud = int(self.get_parameter('lora_baudrate').value)
            self.get_logger().info(f'下行：LoRa {device} @ {baud}')
        return PiCommLink(device=device, baudrate=baud)

    # ---------------- 订阅回调（收到话题消息时自动执行） ----------------

    def _cb_cmd_vel(self, msg: Twist):
        """收到 /cmd_vel：把速度发给 F407（内部换算成 mm/s、mrad/s）。"""
        if not self._up.send_vel(msg.linear.x, msg.linear.y, msg.angular.z):
            self.get_logger().debug('VEL 发送失败', throttle_duration_sec=2.0)

    def _cb_cmd_servo(self, msg: Float32MultiArray):
        """收到 /cmd_servo：msg.data = [夹爪角度°, 托盘角度°, 相机角度°]，有几个发几个。"""
        if len(msg.data) >= 1:
            self._up.send_gripper(msg.data[0])
        if len(msg.data) >= 2:
            self._up.send_plate(msg.data[1])
        if len(msg.data) >= 3:
            self._up.send_cam(msg.data[2])

    def _cb_cmd_enable(self, msg: Bool):
        """收到 /cmd_enable：整车使能/失能。"""
        self._up.send_enable(bool(msg.data))

    def _cb_cmd_pose(self, msg: Pose2D):
        """收到 /cmd_pose：把目标位姿发给 F407，位置+航向闭环在 F407 上执行。"""
        if not self._up.send_pose(msg.x, msg.y, msg.theta):
            self.get_logger().debug('POSE 发送失败', throttle_duration_sec=2.0)

    def _cb_cmd_gimbal(self, msg: Float32MultiArray):
        """收到 /cmd_gimbal：msg.data = [yaw°, lift°, extend°]。0=yaw 1=lift 2=extend。"""
        if len(msg.data) >= 1:
            self._up.send_gimbal(0, msg.data[0])
        if len(msg.data) >= 2:
            self._up.send_gimbal(1, msg.data[1])
        if len(msg.data) >= 3:
            self._up.send_gimbal(2, msg.data[2])

    def _cb_cmd_gimbal_home(self, msg: Int32):
        """收到 /cmd_gimbal_home：云台回零，mask bit0=yaw bit1=lift bit2=extend。"""
        mask = int(msg.data) & 0x07
        if mask:
            self._up.send_home(mask)

    def _cb_cmd_led(self, msg: Int32):
        """收到 /cmd_led：补光灯亮度 0~100%。"""
        self._up.send_led(float(msg.data))

    # ---------------- 定时器回调 ----------------

    def _on_heartbeat(self):
        """5Hz 心跳：告诉 F407 "树莓派还在"，否则它 500ms 就自主失能。"""
        self._up.send_heartbeat()

    def _on_poll(self):
        """20Hz 从 LoRa 串口收数据，可能一次收到多帧。"""
        for cmd, data in self._down.poll():
            self._last_rx_s = self.get_clock().now().nanoseconds / 1e9
            if cmd == RPT_ODOM:
                self._publish_odom(parse_odom(data))
            elif cmd == RPT_STATUS:
                state, err, mask = parse_status(data)
                self._pub_status.publish(Int32MultiArray(data=[state, err, mask]))

    def _on_link_status(self):
        """2Hz 发布链路状态，并在线状态变化时打一条日志。"""
        now_s = self.get_clock().now().nanoseconds / 1e9
        online = (now_s - self._last_rx_s) < F407_OFFLINE_TIMEOUT_S
        ble = bool(getattr(self._up, 'connected', False))

        self._pub_online.publish(Bool(data=online))
        self._pub_ble.publish(Bool(data=ble))

        if online != self._last_online:
            self._last_online = online
            if online:
                self.get_logger().info('F407 回传正常（LoRa 链路通了）')
            else:
                self.get_logger().warn('F407 无回传：检查 LoRa 供电/波特率/接线')
        if ble != self._last_ble:
            self._last_ble = ble
            err = getattr(self._up, 'last_error', '')
            self.get_logger().info(f'蓝牙{"已连接" if ble else "断开"}{"" if ble else ("：" + err if err else "")}')

    def _publish_odom(self, pose):
        """把 (x米, y米, 朝向弧度) 组装成标准的里程计消息发出去。"""
        x, y, theta = pose
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('frame_id').value
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        # 绕 Z 轴转 theta 的四元数是 (0, 0, sin(theta/2), cos(theta/2))
        q = Quaternion()
        q.z = math.sin(theta / 2.0)
        q.w = math.cos(theta / 2.0)
        msg.pose.pose.orientation = q
        self._pub_odom.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = F407BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._up.close()
            node._down.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
