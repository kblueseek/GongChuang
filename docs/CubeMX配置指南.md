# STM32F407VET6 运动控制板 — CubeMX 配置指南

> 目标：用 CubeMX 生成一个 Keil（MDK-ARM）工程，配置 **USART1**（控 7 个步进电机总线）、**USART2**（蓝牙）、**USART3**（接收树莓派指令）、**2 路 PWM**（控 2 个 180° 舵机）。
> 配套：本工程用到的驱动代码 `zdtmotor_uart.c/.h`（UART 协议层）+ `ble_control.c/.h`（蓝牙）+ `servo.c/.h`（舵机）。

---

## 0. 引脚分配总表（先看这个，避免冲突）

| 外设 | 引脚 | 说明 |
|---|---|---|
| USART1_TX | **PA9** | 接 7 个电机总线（并联，半双工） |
| USART1_RX | **PA10** | 电机总线回读（建议串 100Ω 保护） |
| USART2_TX | **PA2** | 蓝牙模块 RXD |
| USART2_RX | **PA3** | 蓝牙模块 TXD |
| USART3_TX | **PB10** | 树莓派 RXD |
| USART3_RX | **PB11** | 树莓派 TXD |
| TIM3_CH1 | **PA6** | 舵机 1 PWM（180°） |
| TIM3_CH2 | **PA7** | 舵机 2 PWM（180°） |
| GPIO_Output | **PC13** | 板载 LED（低电平亮） |
| SWDIO / SWCLK | **PA13 / PA14** | 烧录调试，必须保留 |

> ⚠️ **关键变化（相对 F103 老版）**：本项目已**弃用 CAN，改走 UART** 控制电机（`zdtmotor_uart.c`），所以 **CAN 整个不配置**。PA11/PA12 空出来（它们同时复用 USB OTG FS 的 DM/DP，将来要用 USB 或留 CAN 后路都行）。

---

## 1. 新建工程

1. 打开 STM32CubeMX，`File → New Project`。
2. 搜索 `STM32F407VE`，选中 **STM32F407VETx**（LQFP100），点 Start Project。
3. 固件包选 **STM32Cube FW_F4**（如 V1.28.x 及以上）。

---

## 2. 时钟配置（RCC）—— 168 MHz

**Pinout 页**：`System Core → RCC`：

| 选项 | 值 |
|---|---|
| High Speed Clock (HSE) | **Crystal/Ceramic Resonator** |

**Clock Configuration 页**（F407 的 PLL 参数名和 F1 完全不同，是 `M/N/P/Q` 制）：

| 参数 | 值 | 说明 |
|---|---|---|
| Input frequency | 8 MHz | 板载晶振 |
| PLL Source Mux | HSE | |
| **PLLM** | **8** | 8MHz / 8 = 1MHz，VCO 输入 |
| **PLLN** | **336** | 1MHz × 336 = 336MHz（VCO） |
| **PLLP** | **2** | 336MHz / 2 = 168MHz |
| PLLQ | 7 | 168MHz / 7 = 48MHz（给 USB/SDIO） |
| System Clock Mux | PLLCLK | |
| SYSCLK | **168 MHz** | 目标 |
| AHB Prescaler | /1 | → 168 MHz |
| APB1 Prescaler | **/4** | → **42 MHz**（上限 42MHz，必须 /4） |
| APB2 Prescaler | **/2** | → **84 MHz** |

> **重要**：APB1=42MHz 意味着挂 APB1 的定时器（TIM2/3/4/5/6/7/12/13/14）实际时钟是 **84MHz**（分频 ≠1 时定时器时钟自动 ×2）；APB2=84MHz 时挂 APB2 的定时器（TIM1/8/9/10/11）是 **168MHz**。这直接影响舵机 PWM 的 PSC 计算。

---

## 3. 调试接口（SYS）

`System Core → SYS`：

| 选项 | 值 |
|---|---|
| Debug | **Serial Wire** |

> 不设成 Serial Wire 的话，烧录一次后 PA13/PA14 变成普通 IO，ST-Link 就再也连不上。**必须设。**

---

## 4. USART1 配置（电机总线）

`Connectivity → USART1`：

1. Mode 选 **Asynchronous**。
2. 引脚：`USART1_TX → PA9`，`USART1_RX → PA10`。

**Parameter Settings 页**：

| 参数 | 值 |
|---|---|
| Baud Rate | **115200**（电机默认，先用这个把电机参数改完再统一切换） |
| Word Length | 8 Bits |
| Parity | None |
| Stop Bits | 1 |
| Overrun | Disable |

> **波特率参考**（USART1 在 APB2=84MHz 上）：
> - 115200：84M/115200=729.17 非整数，但 F407 支持 16×/8× 过采样，误差约 0.02%，可用
> - 500000：84M/500000=168 整除，**0% 误差** ← 电机总线建议升到这个
> - 921600：误差约 0.02%，可用

**NVIC Settings 页**：

| 中断 | 状态 |
|---|---|
| USART1 global interrupt | ✅ 勾选 |

> 电机总线现在是**只发不收**（电机应答方式已设为 None），但 RX 中断仍建议勾上并接好 RX 线 —— 将来做里程计轮询（`0x36` 读位置）要用，事后补线比事前留线贵。

---

## 5. USART2 配置（蓝牙）

`Connectivity → USART2`：

1. Mode 选 **Asynchronous**。
2. 引脚：`USART2_TX → PA2`，`USART2_RX → PA3`。

**Parameter Settings 页**：

| 参数 | 值 |
|---|---|
| Baud Rate | **115200** |
| Word Length | 8 Bits |
| Parity | None |
| Stop Bits | 1 |

**NVIC Settings 页**：

| 中断 | 状态 |
|---|---|
| USART2 global interrupt | ✅ 勾选 |

> 蓝牙（微信小程序）数据包靠 RX 中断 + 一字节一字节收，`ble_control.c` 里已带 `HAL_UART_RxCpltCallback`。

---

## 6. USART3 配置（树莓派）

`Connectivity → USART3`：

1. Mode 选 **Asynchronous**。
2. 引脚：`USART3_TX → PB10`，`USART3_RX → PB11`。

**Parameter Settings 页**：

| 参数 | 值 |
|---|---|
| Baud Rate | **115200**（先跑通，后续里程计遥测可升 460800） |
| Word Length | 8 Bits |
| Parity | None |
| Stop Bits | 1 |

**NVIC Settings 页**：

| 中断 | 状态 |
|---|---|
| USART3 global interrupt | ✅ 勾选 |

> 树莓派下发指令 + STM32 回里程计，双向，所以 TX/RX 都要接。协议帧格式见《需求与架构设计》§6。

---

## 7. TIM3 PWM 配置（2 路舵机，180°）

`Timers → TIM3`：

1. Clock Source 选 **Internal Clock**。
2. Channel1 和 Channel2 都选 **PWM Generation CHx**。
3. 引脚：`TIM3_CH1 → PA6`，`TIM3_CH2 → PA7`。

**Parameter Settings 页（重点，直接决定舵机能不能用）**：

| 参数 | 值 | 说明 |
|---|---|---|
| Prescaler (PSC) | **83** | 84MHz / 84 = **1MHz**（1 tick = 1µs） |
| Counter Mode | Up | |
| Counter Period (ARR) | **19999** | 20000 tick = **20ms = 50Hz** |
| Internal Clock Division | No Division | |
| Repetition Counter | 0 | |
| Pulse (CH1 / CH2) | **500** | 初始 0.5ms（运行时改） |
| Output compare preload | **Enable** | 改脉宽立即生效不跳变 |
| CH Polarity | High | 默认 |

> **⚠️ PSC 是关键差异**：F103 是 71（72MHz），F407 是 **83**（84MHz）。填错 PSC 脉宽会整体缩放，舵机角度全错。
>
> **舵机 PWM 原理**：50Hz（20ms 周期）下，高电平脉宽决定角度（两个都是 180°）：
> - **500 tick = 0.5ms → 0°**
> - **1500 tick = 1.5ms → 90°**（中位）
> - **2500 tick = 2.5ms → 180°**
>
> 因为 PSC=83 → 1µs/tick，所以脉宽 tick 值 = 微秒数。

---

## 8. GPIO 配置（LED）

`System Core → GPIO`：

| 引脚 | 模式 | 额外设置 |
|---|---|---|
| PC13 | **GPIO_Output** | 初始输出高（板载 LED 低电平亮，按需） |

> 急停按钮若用外部中断，任选一个空闲 GPIO 配成 `GPIO_EXTI` + 下拉即可，F407 引脚充足。

---

## 9. 中断优先级（可选）

`System Core → NVIC`，确认勾选：

- USART1 global interrupt
- USART2 global interrupt
- USART3 global interrupt

优先级用默认即可；若电机反馈更实时，可把 USART1 优先级调得比 USART3 高。

---

## 10. Project Manager（生成 Keil 工程）

**Project 标签**：

| 项 | 值 |
|---|---|
| Project Name | 如 `chassiscontrol` |
| Project Location | 你的工作目录 |
| Toolchain / IDE | **MDK-ARM**（Keil） |
| Min Version | V5 |

**Code Generator 标签**：

| 项 | 值 |
|---|---|
| Copy only the necessary library files | ✅（推荐） |
| Generate peripheral initialization as a pair of '.c/.h' files per peripheral | ✅ |
| Keep User Code when re-generating | ✅ |

> 点 **GENERATE CODE**，生成后 Open Project 用 Keil 打开。

---

## 11. 生成后要做的几件事

### 11.1 把驱动文件加进工程

把下面 6 个文件放进 `Hardware/`，并在 Keil 里加入工程 + 加进 Include Paths：

```
zdtmotor_uart.c/.h   → 电机总线（UART 版）
ble_control.c/.h     → 蓝牙
servo.c/.h           → 舵机（两个 180°）
```

> ⚠️ `zdtmotor.c/.h`（CAN 版）和 `zdtmotor_uart.c/.h`（UART 版）**二选一**，别同时包含，枚举/宏会重定义。本项目用 UART 版。

### 11.2 main.c 集成（关键代码）

```c
#include "zdtmotor_uart.h"
#include "ble_control.h"
#include "servo.h"
#include "ControllerTask.h"

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();
    MX_TIM3_Init();

    ZDT_UART_Init();        /* 启动电机总线 1 字节中断接收 */
    BLE_Init();             /* 启动蓝牙接收 */
    Servo_Init();           /* 启动 2 路舵机 PWM，回到初始角度 */
    ControllerTask_Init();  /* 使能电机 */

    while (1) {
        BLE_Process();
        ControllerTask_Loop();
        HAL_Delay(1);
    }
}
```

### 11.3 Keil 工程选项

1. `Options → Target`：确认芯片是 STM32F407VE，Xtal = 8.0MHz。
2. `Options → C/C++`：把 `Core/Inc`、`Hardware`、`Task` 加入 Include Paths，并把 `STM32F407xx`、`USE_HAL_DRIVER` 加进 Define（CubeMX 默认已加，新增目录要确认）。
3. `Options → Target`：勾选 **Use MicroLIB**（省 Flash/RAM，本项目够用）。
4. 烧录器选 ST-Link，`Debug → Settings → Flash Download → 勾选 Reset and Run`。

---

## 12. 常见坑（对照排查）

| 现象 | 原因 | 解决 |
|---|---|---|
| 舵机角度不对 / 抖动 | TIM3 PSC 填错 | 确认 **PSC=83**（不是 F103 的 71）、ARR=19999 |
| 电机完全没反应 | 应答方式没改 None / 波特率不一致 | 上位机逐个把「控制命令应答方式」改 None，波特率统一 |
| 蓝牙收不到 / 乱码 | USART2 引脚或波特率错 | 确认 PA2/PA3、115200，TX/RX 交叉 |
| 树莓派收不到数据 | USART3 线没交叉 | STM32 PB10(TX)→Pi RXD，PB11(RX)→Pi TXD |
| 烧录一次后连不上 | 没设 Serial Wire | SYS → Debug → Serial Wire |
| 电机总线数据乱 | 总线太长/无共地 | 短走线、共地、PA10 串 100Ω 保护 |

---

## 附：外设时钟速查（F407VET6 @168MHz）

| 外设 | 总线 | 时钟 |
|---|---|---|
| USART1 / USART6 | APB2 | 84 MHz |
| USART2 / USART3 | APB1 | 42 MHz |
| UART4 / UART5 | APB1 | 42 MHz |
| CAN1 / CAN2 | APB1 | 42 MHz |
| TIM3（及 APB1 定时器） | APB1（定时器 ×2） | **84 MHz** |
| TIM1 / TIM8（APB2 定时器） | APB2（定时器 ×2） | **168 MHz** |
| GPIO / AHB | AHB | 168 MHz |

---

*生成工程后，把 6 个驱动文件加进去，按 11.2 的 main.c 集成，就可以先跑「UART 控一个轮子 + 两个舵机中位」做单机验证了。*
