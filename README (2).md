# 物流搬运智能车 — 电控代码

基于 **STM32F407VET6**（运动控制）+ **树莓派**（决策感知）+ **7× ZDT Y42 闭环步进电机** 的物流搬运智能车电控工程。

## 系统构成

- **底盘**：4× ZDT Y42 麦轮（Emm 固件，CAN 速度模式，多电机命令同步）
- **三轴臂**：3× ZDT Y42（Yaw 旋转 / 上下升降 / 左右伸缩，Emm 固件，CAN 绝对位置模式）
- **夹爪 / 载物盘 / 摄像头**：3× 舵机（TIM3 PWM）
- **通讯**：电机走 **CAN1**（500K 扩展帧，需收发器）；树莓派走 **USART6**；蓝牙遥控走 **USART2**；JY901S 陀螺仪走 **USART3**；调试输出走 **USART1**（DMA）
- **里程计 / 位置闭环**：电机定时返回编码器位置（0x36），整数计数差分积分成世界系里程计，支持回零位置闭环

## 文档

| 文档 | 说明 |
|---|---|
| [docs/工程结构与模块说明.md](docs/工程结构与模块说明.md) | 工程分层、模块数据流、树莓派/蓝牙协议、ZDT 电机封装、TODO 清单 |
| [docs/引脚分配表.md](docs/引脚分配表.md) | 引脚接线表（CAN 版） |
| [docs/CubeMX配置指南.md](docs/CubeMX配置指南.md) | CubeMX 生成工程的配置步骤 |
| [docs/24V电源板设计.md](docs/24V电源板设计.md) | 24V 电源保护板设计 |
| [docs/ZDT Y42第二代闭环步进电机使用说明V1.1.md](docs/ZDT%20Y42第二代闭环步进电机使用说明V1.1.md) | 厂商手册 |
| [docs/ZDT_X42S第二代闭环步进电机用户手册V1.0.5_260527.md](docs/ZDT_X42S第二代闭环步进电机用户手册V1.0.5_260527.md) | 厂商手册（CAN 协议 / 定时返回 / 多电机命令） |

## 目录规划

```
UniversalCar/
├── Core/                  CubeMX 生成（硬件抽象 + HAL 初始化）
│   ├── Inc/               header.h / RobotConfig.h / main.h / can.h ...
│   └── Src/               main.c / freertos.c / can.c / usart.c ...
├── ExHardware/            外部硬件驱动（ZdtMotor / Servo / JY901S / PiComm / BleControl / Led）
├── Task/                  FreeRTOS 任务（Receive / Imu / Chassis / Gimbal / Debug）
├── UserLibrary/           纯算法库（Universal / UniversalPID）
├── Drivers/               ST HAL 库
├── Middlewares/           FreeRTOS
├── MDK-ARM/               Keil 工程
└── docs/                  设计文档
```

## 开发状态

- [x] 电机库（CAN 版：注册表 + 多电机命令 0xAA + 定时返回解析 + 拆包功能码重复）
- [x] CAN 多电机命令同步（底盘 4 轮一条广播帧同时起步）
- [x] 编码器里程计（整数计数差分积分，最高精度）+ 回零位置闭环
- [x] 麦轮底盘联调（速度模式）
- [x] 蓝牙遥控（树莓派掉线时接管）
- [x] 树莓派协议（USART6）
- [ ] 三轴臂联调（当前 `GIMBAL_MOTOR_INSTALLED=0`，云台未装）
- [ ] 轮径 / 轴距 / 方向系数实车标定
- [ ] 位置闭环 PID 调参
- [ ] 上下位机联通联调
