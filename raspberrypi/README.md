# 树莓派端（决策 / 感知）

树莓派是整车的"大脑"，负责三件事：

1. **视觉**（`vision/`）——物料 / 转台 / 圆环工位识别，OpenCV + USB 摄像头，独立进程；
2. **激光雷达避障**——两条链路：
   - `上位机/` 里的地图导航：D6 雷达串口扫描 → 识别障碍 → A* 绕障 → 下发 `goto` 路点；
   - `ros2_ws/src/lidar_bringup` + `obstacle_detect`：YDLIDAR X3 PRO（`/scan`，旧链路）；
3. **运动指令下发**（`comm/pi_comm.py` + `f407_bridge`）——通过 USART 与 F407 通信（PiComm 协议）。

## 目录结构

```
树莓派/
├── vision/          # 视觉代码（原 camera/，独立进程，当前仍用旧串口协议）
├── comm/            # PiComm 协议（与 F407 侧 PiComm.c 对齐），可脱离 ROS2 单独自检
├── 上位机/          # 动作序列编辑 + 雷达地图导航（tkinter，跑在树莓派桌面上）
│   ├── motion_gui.py         # 主界面（/motion_load、/motion_stop、/odom）
│   ├── mapping.py            # 场地模型 + 占据栅格 + A* + 圆柱识别（从 vision 分支拷入）
│   ├── lidar_d6.py           # D6 雷达串口驱动 + 模拟雷达
│   └── field.json            # 场地尺寸 / 雷达位姿 / 车起点 / 任务路线
├── ros2_ws/         # ROS2 工作区（Ubuntu 26.04 + Lyrical Luth）
│   └── src/
│       ├── lidar_bringup/    # X3 PRO 参数 + launch（system.launch.py = 总启动）
│       ├── obstacle_detect/  # /scan -> /obstacles（前后左右扇区最近距离）
│       ├── f407_bridge/      # ROS2 话题 <-> F407 串口（PiComm）
│       └── motion_planner/   # /motion_load 指令序列状态机（move/turn/goto/云台…）
├── scripts/         # run_car.sh / run_gui.sh / setup.sh / bringup.sh
└── docs/            # 树莓派构建指南
```

## 快速开始

完整步骤见 [docs/树莓派构建指南.md](docs/树莓派构建指南.md)，要点：

```bash
bash scripts/setup.sh      # 装依赖、开串口、装 YDLIDAR 驱动、构建
sudo reboot                # 生效
bash scripts/run_car.sh    # 启动 f407_bridge + motion_planner
bash scripts/run_gui.sh    # 另开终端跑上位机（动作序列 + 雷达地图）
```

## 上位机使用（动作序列 + 雷达地图导航）

**完整说明见 [docs/上位机使用说明.md](docs/上位机使用说明.md)**（界面、坐标、field.json
全部字段、雷达/识别/规划原理、标定清单、故障排查）。

两个终端：`scripts/run_car.sh` 起控制链路，`scripts/run_gui.sh` 起界面。

**跑一次完整流程**

1. 把车摆到**车起点**（默认启停区1）、上电，`run_car.sh`，再 `run_gui.sh`；
2. 左边【雷达 / 地图】页签 → 串口选 D6（默认 `/dev/ttyUSB0`，230400）→【连接】；
3. 【① 扫描建图】扫几圈自动冻结 →【② 重新识别障碍】，检出的障碍列在下面；
4. 点【此刻车在【车起点】→ 标定里程计】（车没动过、还在原点附近时会自动标定）；
5. 地图上点目标点，或填格子号 `C3`，或填目标 odom 坐标（米）点【去】→ 自动 A* 规划；
6. 【点击即导航】勾着就直接下发，否则点【▶ 导航到目标】；停车用【■】或底部【停止】。

**坐标口径**

界面上的位姿读数、鼠标悬停、目标点读数、下发的 `goto`，全部是**里程计坐标**
（米 / 度，和 `ros2 topic echo /odom` 同一套数）；`field.json` 里的尺寸、车起点、
雷达安装位姿仍然是场地系（mm / 度），因为它们描述的是场地本身。屏幕上的地图
按场地正向画，场地↔里程计的换算是程序内部做的。

**地图约定**

- 灰色**田**字形 = 可走（外圈一圈 + 中间十字，`lane.ring_mm` 和 `lane.width_mm` 可改）；
- 淡黄块 = 禁行；蓝色 = 启停区；绿框 = 工位区（规划只看工位的 `approach` 点）；
- A1~E5 是 5×5 格，A1 在左下、E5 在右上，C3 就是场心；B2/B4/D2/D4 正好是四个黄块，点它们会提示"目标被占住"；
- 红格 = 被障碍占住的整格 450mm（取消【整格封锁】则只按实体 φ 绕行）；
- 车形/雷达图标只在里程计标定之后才按真实位姿画，没标定时停在车起点。

**注意**

- 上位机开车期间手柄自动让位（只有 B 失能键还能用）；上位机停止或断线 0.6s 后手柄自动恢复。
- `goto` 单条 20s 不到位会自动停车并交还手柄（`motion_planner` 的 `arrive_timeout_s` 参数）。
- 地图是用参数画的，不是雷达扫出来的；雷达只负责发现**立起来的东西**。障碍变了要重新【扫描建图】。
- 云台等非导航动作仍在【动作序列】页签里写成一条条指令，执行整段或执行选中一条。

## 数据流

```
D6 雷达 ──串口──> 上位机地图（扫描/识别/A*）──/motion_load(goto)──> motion_planner ──/cmd_pose──┐
                                                                                             v
USB摄像头 ──> vision（独立进程，旧协议，待迁移）                                       f407_bridge ──PiComm(AA 55)──> F407
                                                                                             ^
X3 PRO ──/scan──> obstacle_detect ──/obstacles──>（旧避障占位）        /odom /robot_status <──┘
```

## 通信协议

PiComm 帧格式 `AA 55 | LEN | SEQ | CMD | DATA | CRC16`，与 F407 侧
`UniversalCar/ExHardware/PiComm.c` 逐字节一致，详见
[comm/pi_comm.py](comm/pi_comm.py) 顶部注释。
