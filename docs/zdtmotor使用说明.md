# zdtmotor 电机库使用说明

ZDT Y42 第二代闭环步进电机 **CAN 通讯控制封装库**（针对 **X 固件**）。

> ⚠️ **当前项目实际使用的是 UART 版（`zdtmotor_uart.c/.h`），本 CAN 版库暂未启用，仅保留备用**（将来若换 CAN 时使用）。
> 本库文件仍保留在 `Hardware/` 下，但**已从 Keil 工程移除**，can.c 的接收回调和 main.c 的初始化也已删除；要重新启用需按下方「接入步骤」重新加回。

- 适用芯片：STM32F103C8（CAN1，PA11=RX / PA12=TX，500K 波特率）
- 控制对象：7 个电机 —— 4 个麦克纳姆底盘 + 3 轴机械臂
- 文件位置：`Hardware/zdtmotor.h`、`Hardware/zdtmotor.c`

---

## 1. 协议要点（了解即可，库已封装）

CAN 通讯采用**扩展帧**，与手册「4.2 CAN 通讯」一致：

| 项 | 内容 |
|---|---|
| 帧类型 | 扩展帧 |
| 扩展帧 ID | `(电机地址 << 8) \| 包序号` |
| 数据区 | `功能码 + 命令数据 + 校验码(0x6B)` |
| 拆包 | 命令 > 8 字节时自动拆包，包序号从 0 递增 |

> 地址字节**不进数据区**，而是编码进扩展帧 ID 的高 8 位，库已自动处理。

---

## 2. 接入步骤

1. 把 `Hardware/zdtmotor.c`、`Hardware/zdtmotor.h` 加进 Keil 工程（当前已被移除，需重新添加；并确认 Include Path 里有 `..\Hardware`）。
2. `main.c` 中 `MX_CAN_Init()` 之后调用初始化：

```c
MX_CAN_Init();
ZDT_Motor_Init();   // 配置过滤器、启动 CAN、使能接收中断
```

3. 接收回调需加回 `can.c` 的 `HAL_CAN_RxFifo0MsgPendingCallback` 中，收到电机返回帧后解析到 `zdt_state[]`（此回调在切换到 UART 版时已删除，换回 CAN 时需重新补上）。

---

## 3. 电机地址定义

| 宏 | 地址 | 用途 |
|---|---|---|
| `ZDT_MOTOR_FL` | 1 | 底盘 前左轮 |
| `ZDT_MOTOR_FR` | 2 | 底盘 前右轮 |
| `ZDT_MOTOR_RL` | 3 | 底盘 后左轮 |
| `ZDT_MOTOR_RR` | 4 | 底盘 后右轮 |
| `ZDT_MOTOR_A1` | 5 | 机械臂 关节 1 |
| `ZDT_MOTOR_A2` | 6 | 机械臂 关节 2 |
| `ZDT_MOTOR_A3` | 7 | 机械臂 关节 3 |

> 地址 0 为广播地址，发送命令时所有电机同时执行（库中 `ZDT_Motor_SyncAll` 即用广播）。

---

## 4. 函数 API

### 4.1 初始化 / 底层发送

| 函数 | 说明 |
|---|---|
| `void ZDT_Motor_Init(void)` | 配置 CAN 过滤器（接收所有扩展帧）、启动 CAN、使能 FIFO0 接收中断。上电初始化后调用一次。 |
| `void ZDT_Motor_Send(uint8_t addr, const uint8_t *cmd, uint8_t len)` | 底层发送。`cmd` 为「功能码 + 命令数据」（不含地址、不含校验码），自动追加校验码 `0x6B` 并拆包发送。一般不需要直接调用，除非用库未封装的命令。 |

### 4.2 基础命令

| 函数 | 参数 | 说明 |
|---|---|---|
| `ZDT_Motor_Enable(addr, en)` | `en`：`1` 使能锁轴 / `0` 去使能松轴 | 电机使能控制（`F3 AB`） |
| `ZDT_Motor_Stop(addr)` | — | 立即停止（`FE 98`） |
| `ZDT_Motor_SetZero(addr)` | — | 当前位置角度清零（`0A 6D`），设为坐标零点 |
| `ZDT_Motor_Home(addr, mode)` | `mode` 见下表 | 触发回零（`9A`） |
| `ZDT_Motor_SyncAll(void)` | — | 广播触发多机同步运动（`FF 66`） |

回零模式 `mode` 取值：

| 宏 | 值 | 说明 |
|---|---|---|
| `ZDT_HOME_NEAR` | 0 | 单圈就近回零 |
| `ZDT_HOME_DIR` | 1 | 单圈方向回零 |
| `ZDT_HOME_SENSORLESS` | 2 | 无限位碰撞回零 |
| `ZDT_HOME_LIMIT` | 3 | 限位回零 |
| `ZDT_HOME_ABSZERO` | 4 | 回到绝对坐标零点 |
| `ZDT_HOME_POWERCUT` | 5 | 回到上次掉电位置 |

### 4.3 运动命令（X 固件）

**速度模式**（`F6`，适合底盘轮子持续调速）：

```c
void ZDT_Motor_Speed(uint8_t addr, uint8_t dir, uint16_t acc, uint16_t speed);
```

| 参数 | 范围 | 单位 | 说明 |
|---|---|---|---|
| `dir` | `ZDT_DIR_CW` / `ZDT_DIR_CCW` | — | 方向：顺时针 / 逆时针 |
| `acc` | 0~65535 | RPM/s | 加速度 |
| `speed` | 0~30000 | 0.1 RPM | 速度（1500 = 150.0 RPM） |

**梯形位置模式**（`FD`，适合机械臂，带加减速曲线）：

```c
void ZDT_Motor_Pos(uint8_t addr, uint8_t dir, uint16_t speed, uint16_t acc,
                   uint16_t dec, uint32_t pos, uint8_t mode);
```

| 参数 | 范围 | 单位 | 说明 |
|---|---|---|---|
| `dir` | `ZDT_DIR_CW` / `ZDT_DIR_CCW` | — | 方向 |
| `speed` | 0~30000 | 0.1 RPM | 最大速度 |
| `acc` | 0~65535 | RPM/s | 加速加速度 |
| `dec` | 0~65535 | RPM/s | 减速加速度 |
| `pos` | 0~0xFFFFFFFF | 0.1° | 位置角度（3600 = 360°） |
| `mode` | 见下表 | — | 运动模式 |

位置运动模式 `mode` 取值：

| 宏 | 值 | 说明 |
|---|---|---|
| `ZDT_POS_REL_PREV` | 0 | 相对上一输入目标位置运动 |
| `ZDT_POS_ABS` | 1 | 相对坐标零点绝对位置运动 |
| `ZDT_POS_REL_CUR` | 2 | 相对当前实时位置运动 |

### 4.4 读取反馈

| 函数 | 说明 | 解析结果 |
|---|---|---|
| `ZDT_Motor_ReadPos(addr)` | 读实时位置（`0x36`） | 存到 `zdt_state[addr].pos` |
| `ZDT_Motor_ReadStatus(addr)` | 读回零 + 电机状态标志（`0x3C`） | 存到 `zdt_state[addr].home` / `.status` |

---

## 5. 反馈读取

接收回调会自动把电机返回帧解析进全局数组，按地址索引（1~7）：

```c
typedef struct {
    int32_t pos;      // 实时位置，单位 0.1°
    uint8_t home;     // 回零状态标志
    uint8_t status;   // 电机状态标志
} ZDT_MotorState;

extern ZDT_MotorState zdt_state[8];
```

用法：先调用 `ZDT_Motor_ReadPos(addr)`（或 `ReadStatus`），稍后读 `zdt_state[addr]` 即可。

```c
ZDT_Motor_ReadPos(ZDT_MOTOR_A1);
// ... 等待一两个通讯周期后 ...
int32_t angle01 = zdt_state[ZDT_MOTOR_A1].pos;   // 0.1°，1800 = 180°
```

### 电机状态标志位（`status`）

| 位 | 名称 | 说明 |
|---|---|---|
| bit0 | Ens_TF | 使能状态：0 未使能 / 1 已使能 |
| bit1 | Prf_TF | 位置到达：0 未到达 / 1 已到达目标 |
| bit2 | Cgi_TF | 堵转标志：0 未堵转 / 1 已堵转 |
| bit3 | Cgp_TF | 堵转保护：0 未触发 / 1 已触发 |
| bit4 | Esi_LF | 左限位输入电平：0 低 / 1 高 |
| bit5 | Esi_RF | 右限位输入电平：0 低 / 1 高 |
| bit7 | Oac_TF | 掉电标志 |

```c
if (zdt_state[ZDT_MOTOR_A1].status & 0x02) {
    // 到位
}
```

### 回零状态标志位（`home`）

| 位 | 名称 | 说明 |
|---|---|---|
| bit0 | Enc_Rdy | 编码器正常 |
| bit1 | Cal_Rdy | 编码器已校准 |
| bit2 | Org_SF | 正在回零 |
| bit3 | Org_CF | 回零失败 |
| bit4 | Otp_TF | 过热保护触发 |
| bit5 | Ocp_TF | 过流保护触发 |

---

## 6. 使用示例

### 6.1 底盘：麦克纳姆四轮速度控制

```c
// 上电使能四个轮子
ZDT_Motor_Enable(ZDT_MOTOR_FL, 1);
ZDT_Motor_Enable(ZDT_MOTOR_FR, 1);
ZDT_Motor_Enable(ZDT_MOTOR_RL, 1);
ZDT_Motor_Enable(ZDT_MOTOR_RR, 1);

// 前进：四轮同速（具体转向按实际轮子安装方向调整 dir）
ZDT_Motor_Speed(ZDT_MOTOR_FL, ZDT_DIR_CW,  500, 1500);
ZDT_Motor_Speed(ZDT_MOTOR_FR, ZDT_DIR_CCW, 500, 1500);
ZDT_Motor_Speed(ZDT_MOTOR_RL, ZDT_DIR_CW,  500, 1500);
ZDT_Motor_Speed(ZDT_MOTOR_RR, ZDT_DIR_CCW, 500, 1500);

// 停止
ZDT_Motor_Stop(ZDT_MOTOR_FL);
ZDT_Motor_Stop(ZDT_MOTOR_FR);
ZDT_Motor_Stop(ZDT_MOTOR_RL);
ZDT_Motor_Stop(ZDT_MOTOR_RR);
```

### 6.2 机械臂：三轴位置控制

```c
// 关节 1 绝对位置转到 360°，速度 100 RPM，加减速 100 RPM/s
ZDT_Motor_Pos(ZDT_MOTOR_A1, ZDT_DIR_CW, 1000, 100, 100, 3600, ZDT_POS_ABS);

// 关节 2 相对当前位置再转 +90°
ZDT_Motor_Pos(ZDT_MOTOR_A2, ZDT_DIR_CW, 800, 100, 100, 900, ZDT_POS_REL_CUR);

// 读关节 1 位置，判断是否到位
ZDT_Motor_ReadStatus(ZDT_MOTOR_A1);
if (zdt_state[ZDT_MOTOR_A1].status & 0x02) { /* 已到位 */ }
```

### 6.3 多机同步运动

```c
// 先把要同步的电机命令缓存（sync 标志 = 1 的版本见手册；本库默认立即执行）
// 需要同步时，用广播触发：
ZDT_Motor_SyncAll();
```

---

## 7. 注意事项

1. **电机侧必须提前配置好**（用上位机或串口命令），否则 CAN 不通：
   - 每个电机地址设为 **1~7**（出厂默认都是 1）。
   - 通讯端口复用模式设为 **CAN(03)**（默认是 UART）。
   - 通讯校验方式保持默认 **自由协议 6B**。
2. **波特率**：工程 CAN 已配为 500K，与电机默认 CAN 速率一致，无需改。
3. **固件**：本库按 **X 固件**编码。若切换成 Emm 固件，位置/速度命令格式不同，需要另写编码。
4. **单位换算**：位置 0.1°（`3600 = 360°`），速度 0.1 RPM（`1500 = 150 RPM`）。
5. **连续命令**：多电机连续发送命令时，手册建议每条命令间延时几毫秒，防止粘包。
