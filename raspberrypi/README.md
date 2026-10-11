# 树莓派端（决策 / 感知）

树莓派是整车的"大脑"，负责三件事：

1. **视觉**（`vision/`）——物料 / 转台 / 圆环工位识别，OpenCV + USB 摄像头，独立进程；
2. **激光雷达避障**（ROS2 + YDLIDAR X3 PRO）——障碍物检测；
3. **运动指令下发**（`comm/pi_comm.py` + `f407_bridge`）——通过 USART 与 F407 通信（PiComm 协议）。

## 目录结构

```
树莓派/
├── vision/          # 视觉代码（原 camera/，独立进程，当前仍用旧串口协议）
├── comm/            # PiComm 协议（与 F407 侧 PiComm.c 对齐），可脱离 ROS2 单独自检
├── ros2_ws/         # ROS2 工作区（Ubuntu 26.04 + Lyrical Luth）
│   └── src/
│       ├── lidar_bringup/    # X3 PRO 参数 + launch（system.launch.py = 总启动）
│       ├── obstacle_detect/  # /scan -> /obstacles（前后左右扇区最近距离）
│       ├── f407_bridge/      # ROS2 话题 <-> F407 串口（PiComm）
│       └── motion_planner/   # /obstacles -> /cmd_vel（避障，占位实现）
├── scripts/         # setup.sh（环境准备）/ bringup.sh（一键启动）
└── docs/            # 树莓派构建指南
```

## 快速开始

完整步骤见 [docs/树莓派构建指南.md](docs/树莓派构建指南.md)，要点：

```bash
bash scripts/setup.sh      # 装依赖、开串口、装 YDLIDAR 驱动、构建
sudo reboot                # 生效
bash scripts/bringup.sh    # 一键启动 4 个节点
```

## 数据流

```
激光雷达 ──/scan──> obstacle_detect ──/obstacles──> motion_planner ──/cmd_vel──┐
                                                                              v
USB摄像头 ──> vision（独立进程，旧协议，待迁移）                          f407_bridge ──PiComm(AA 55)──> F407 USART6
                                                                              ^
                                                     /odom /robot_status <────┘
```

## 通信协议

PiComm 帧格式 `AA 55 | LEN | SEQ | CMD | DATA | CRC16`，与 F407 侧
`UniversalCar/ExHardware/PiComm.c` 逐字节一致，详见
[comm/pi_comm.py](comm/pi_comm.py) 顶部注释。
