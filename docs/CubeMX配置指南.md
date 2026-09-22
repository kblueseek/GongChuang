# STM32F103C8T6 运动控制板 — CubeMX 配置指南

> 目标：用 CubeMX 生成一个 Keil（MDK-ARM）工程，配置 **CAN**（控 7 个步进电机）、**USART1**（接收树莓派指令）、**2 路 PWM**（控 2 个舵机）。
> 配套：本工程用到的驱动代码 `ZdtMotor.c/.h`（协议层）+ `ZdtPort_STM32F1.c/.h`（HAL 落地层）。

---

## 0. 引脚分配总表（先看这个，避免冲突）

| 外设 | 引脚 | 说明 |
|---|---|---|
| CAN1_RX | **PA11** | 接 CAN 收发器模块 RX |
| CAN1_TX | **PA12** | 接 CAN 收发器模块 TX |
| USART1_TX | **PA9** | 接树莓派 RX |
| USART1_RX | **PA10** | 接树莓派 TX |
| TIM3_CH1 | **PA6** | 舵机 1 PWM（夹爪） |
| TIM3_CH2 | **PA7** | 舵机 2 PWM（备用/其它） |
| GPIO_Input | **PB0** | 急停按钮输入（下拉/上拉） |
| GPIO_Output | **PC13** | 板载 LED（Blue Pill） |
| SWDIO / SWCLK | **PA13 / PA14** | 烧录调试，必须保留 |

> ⚠️ **关键冲突提醒**：F103C8T6 的 **CAN 和 USB 共用 PA11/PA12**。本项目用 CAN，所以**绝对不能使能 USB**。舵机用 TIM3（PA6/PA7）而不是 TIM2/TIM1，就是为了避开 PA9/PA10（USART1）和 PA11/PA12（CAN）。

---

## 1. 新建工程

1. 打开 STM32CubeMX，`File → New Project`。
2. 在搜索框输入 `STM32F103C8`，选中 **STM32F103C8Tx**（LQFP48），点 Start Project。
3. 如果提示下载固件包，选一个较新的 **STM32Cube FW_F1** 版本（如 V1.8.x）。

---

## 2. 时钟配置（RCC）

**Pinout 页**：左侧 `System Core → RCC`：

| 选项 | 值 |
|---|---|
| High Speed Clock (HSE) | **Crystal/Ceramic Resonator** |

**Clock Configuration 页**：把主频调到 72MHz。Blue Pill 板载 8MHz 晶振，按下表：

| 参数 | 值 | 说明 |
|---|---|---|
| Input frequency | 8 MHz | 板载晶振 |
| PLL Source Mux | HSE | |
| PLLMul | ×9 | 8MHz × 9 = 72MHz |
| System Clock Mux | PLLCLK | |
| SYSCLK | **72 MHz** | 目标 |
| APB1 Prescaler | /2 | → **36 MHz**（上限 36MHz，必须 /2） |
| APB2 Prescaler | /1 | → 72 MHz |

> **重要**：APB1 = 36MHz 意味着挂 APB1 的定时器（TIM2/3/4）实际时钟是 **72MHz**（APB1 分频≠1 时定时器时钟自动 ×2）。这直接影响下面舵机 PWM 的计算，别搞错。

---

## 3. 调试接口（SYS）

`System Core → SYS`：

| 选项 | 值 |
|---|---|
| Debug | **Serial Wire** |

> 不设成 Serial Wire 的话，烧录一次后 PA13/PA14 会变成普通 IO，ST-Link 就再也连不上了。**这步必须设。**

---

## 4. CAN 配置（控电机，重点）

`Connectivity → CAN1`：

1. 勾选 **Activated**。
2. Pinout 页确认：`CAN_RX → PA11`，`CAN_TX → PA12`（默认就是）。

**Parameter Settings 页**：

| 参数 | 值 | 说明 |
|---|---|---|
| Mode | Normal | |
| Prescaler (for Time Quantum) | **6** | |
| Time Quanta in Bit Segment 1 | **9** | |
| Time Quanta in Bit Segment 2 | **2** | |
| ReSynchronization Jump Width | 1 time quantum | |
| Time triggered communication | Disable | |
| Automatic bus-off management | **Enable** | 总线故障自动恢复 |
| Automatic wake-up mode | Disable | |
| No automatic retransmission | Disable | 允许自动重传 |
| Receive FIFO locked mode | Disable | |
| Transmit FIFO priority | Disable | |

> **速率验算**：APB1 = 36MHz。Time Quantum = 6 / 36MHz。位时间 = (1 同步段 + 9 + 2) × TQ = 12 TQ = 2µs → **500 kbps**。与 ZDT Y42 电机默认 CAN 速率（配置码 `07` = 500K）**必须一致**。

**NVIC Settings 页**（Can 的 NVIC 标签，不是全局 NVIC）：

| 中断 | 状态 |
|---|---|
| **CAN1 RX0 interrupt** | ✅ 勾选 |

> ⚠️ **漏勾这个 = 永远收不到电机的返回数据**。`ZdtPort.c` 里的 `HAL_CAN_RxFifo0MsgPendingCallback` 依赖这个中断。

---

## 5. USART1 配置（接收树莓派指令）

`Connectivity → USART1`：

1. Mode 选 **Asynchronous**。
2. 引脚：`USART1_TX → PA9`，`USART1_RX → PA10`。

**Parameter Settings 页**：

| 参数 | 值 |
|---|---|
| Baud Rate | **115200**（先用这个，0% 误差最稳；后面要传里程计可升到 460800） |
| Word Length | 8 Bits |
| Parity | None |
| Stop Bits | 1 |
| Overrun | Disable |

> **波特率参考**（USART1 在 APB2=72MHz 上）：
> - 115200：72M/115200=625 整除，**0% 误差** ← 推荐先跑通
> - 460800：误差约 0.16%，可用
> - 921600：误差约 0.16%，可用

**NVIC Settings 页**：

| 中断 | 状态 |
|---|---|
| USART1 global interrupt | ✅ 勾选 |

> 收树莓派指令要靠 RX 中断 + 空闲中断（IDLE）做帧定界。中断这里勾上，帧解析逻辑在协议代码里做。

---

## 6. TIM3 PWM 配置（2 路舵机）

`Timers → TIM3`：

1. Clock Source 选 **Internal Clock**。
2. Channel1 和 Channel2 都选 **PWM Generation CHx**。
3. 引脚：`TIM3_CH1 → PA6`，`TIM3_CH2 → PA7`。

**Parameter Settings 页（重点，直接决定舵机能不能用）**：

| 参数 | 值 | 说明 |
|---|---|---|
| Prescaler (PSC) | **71** | 72MHz / 72 = **1MHz**（1 tick = 1µs） |
| Counter Mode | Up | |
| Counter Period (ARR) | **19999** | 20000 tick = **20ms = 50Hz** |
| Internal Clock Division | No Division | |
| Repetition Counter | 0 | |
| Pulse (CH1 / CH2) | **500** | 初始 0.5ms（运行时改） |
| Output compare preload | **Enable** | 改脉宽立即生效不跳变 |
| CH Polarity | High | 默认 |

> **舵机 PWM 原理**：50Hz（20ms 周期）下，高电平脉宽决定角度：
> - **500 tick = 0.5ms → 0°**
> - **1500 tick = 1.5ms → 90°**（中位）
> - **2500 tick = 2.5ms → 180°**
>
> 因为 PSC=71 → 1µs/tick，所以脉宽 tick 值 = 微秒数，很好换算。

---

## 7. GPIO 配置（急停 + LED）

`System Core → GPIO`：

| 引脚 | 模式 | 额外设置 |
|---|---|---|
| PB0 | **GPIO_Input** | GPIO Pull-up/Pull-down = **Pull-up**（急停按钮，低电平触发） |
| PC13 | **GPIO_Output** | 初始输出高（Blue Pill LED 低电平亮，按需） |

---

## 8. 中断优先级（可选）

`System Core → NVIC`，确认下面都已勾选：

- CAN1 RX0 interrupt
- USART1 global interrupt

优先级用默认即可；若后面发现 CAN 反馈偶尔丢，可把 **CAN1 RX0 的优先级调得比 USART1 高**（电机反馈更实时）。

---

## 9. Project Manager（生成 Keil 工程）

切到 **Project Manager** 页：

**Project 标签**：

| 项 | 值 |
|---|---|
| Project Name | 如 `MotionBoard` |
| Project Location | 你的工作目录 |
| Toolchain / IDE | **MDK-ARM**（Keil） |
| Min Version | V5 |

**Code Generator 标签**：

| 项 | 值 |
|---|---|
| Copy only the necessary library files | ✅（推荐） |
| Generate peripheral initialization as a pair of '.c/.h' files per peripheral | ✅ |
| Keep User Code when re-generating | ✅ |

> 点右上角 **GENERATE CODE**，生成后点 Open Project 直接用 Keil 打开。

---

## 10. 生成后要做的几件事

### 10.1 把驱动文件加进工程

把下面 4 个文件复制到工程 `Core/Src` / `Core/Inc` 目录，并在 Keil 里 `Add Existing Files` 加入（或直接在 Keil 里右键 Source Group 添加）：

```
ZdtMotor.c        → Core/Src
ZdtMotor.h        → Core/Inc
ZdtPort_STM32F1.c → Core/Src
ZdtPort_STM32F1.h → Core/Inc
```

### 10.2 main.c 集成（关键代码）

```c
#include "ZdtPort_STM32F1.h"

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_CAN_Init();
    MX_USART1_UART_Init();
    MX_TIM3_Init();

    /* 1. 绑定外设 + 配置 CAN 滤波器 */
    ZdtPort_Init(&huart1, &hcan);

    /* 2. 启动 2 路舵机 PWM */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1500);  // 舵机1 到中位 90°
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1500);

    /* 3. 发送一帧 CAN 指令：地址 0x02 电机 500RPM、acc=250 */
    uint8_t f[ZDT_FRAME_MAX];
    uint16_t n = ZdtBuildSpeedEmm(f, 0x02, ZDT_CS_FIXED, ZDT_DIR_CW, 500, 250, ZDT_SYNC_NOW);
    ZdtSend(ZdtPort_Get(), ZDT_BUS_CAN, f, n);

    while (1) {
        /* 你的主循环：解析树莓派指令 → 下发电机命令 */
    }
}

/* 接收电机返回（CAN 中断回调里触发） */
void ZdtPort_OnFrame(uint8_t addr, const uint8_t *frame, uint16_t len)
{
    if (frame[1] == 0x36) {           // 读取实时位置返回
        int32_t pos;
        ZdtParsePos(frame, len, &pos);
    }
}
```

### 10.3 Keil 工程选项

1. `Options → Target`：确认芯片是 STM32F103C8，Xtal = 8.0MHz。
2. `Options → C/C++`：把 `Core/Inc` 加入 Include Paths（CubeMX 默认加了，新增头文件目录要确认）。
3. `Options → Target`：勾选 **Use MicroLIB**（省 Flash/RAM，本项目够用）。
4. 烧录器选 ST-Link，`Debug → Settings → Flash Download → 勾选 Reset and Run`。

---

## 11. 常见坑（对照排查）

| 现象 | 原因 | 解决 |
|---|---|---|
| CAN 电机完全没反应 | 电机「通讯端口复用」没切到 CAN | 用上位机逐个把配置里 `通讯端口复用=CAN(03)` |
| CAN 一直报错/乱 | 速率不一致 | CubeMX 与电机都设 500K |
| CAN 发得出收不回 | 没勾 CAN1 RX0 中断 | 回去勾 NVIC |
| 舵机不动 | 没调 `HAL_TIM_PWM_Start` | 初始化后调一次 Start |
| 舵机角度不对 | ARR/PSC 算错 | 确认 PSC=71、ARR=19999、脉宽 500~2500 |
| 烧录一次后连不上 | 没设 Serial Wire | SYS → Debug → Serial Wire |
| CAN 线接反 | 收发器 TX/RX 交叉 | STM32 TX→模块 TX，STM32 RX→模块 RX，两端加 120Ω |
| 树莓派收不到数据 | 串口线 TX/RX 没交叉 | STM32 PA9(TX)→Pi RXD，PA10(RX)→Pi TXD |

---

## 附：外设时钟速查

| 外设 | 总线 | 时钟 |
|---|---|---|
| CAN1 | APB1 | 36 MHz |
| USART1 | APB2 | 72 MHz |
| TIM3 | APB1（定时器 ×2） | **72 MHz** |
| GPIO | AHB | 72 MHz |

---

*生成工程后，把 ZdtMotor / ZdtPort 四个文件加进去，按 10.2 的 main.c 集成，就可以先跑「CAN 控一个轮子 + 舵机中位」做单机验证了。*
