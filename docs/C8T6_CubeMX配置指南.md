# STM32F103C8T6 移植与 STM32CubeMX 配置指南

> 适用工程：`UniversalCar`（原主控 STM32F407VET6）
>
> 目标芯片：**STM32F103C8T6**（LQFP48，64 KB Flash / 20 KB SRAM，Cortex-M3，72 MHz，无 FPU）
>
> 工具链：STM32CubeMX 6.16.x + STM32Cube FW_F1 V1.8.x + Keil MDK-ARM V5.32+
>
> 本文假设 407→103 转接板已经做好，目标是**新建一个干净的 F1 工程**，再把现有应用层代码迁过去。

---

## 目录

- [0. 先读这一节：移植边界与两个必须做的选择](#0-先读这一节移植边界与两个必须做的选择)
- [1. 硬件转接板核对](#1-硬件转接板核对)
- [2. 新建 CubeMX 工程](#2-新建-cubemx-工程)
- [3. 时钟与系统配置](#3-时钟与系统配置)
- [4. 外设配置](#4-外设配置)
- [5. NVIC 中断优先级](#5-nvic-中断优先级)
- [6. FreeRTOS 配置](#6-freertos-配置)
- [7. 生成工程](#7-生成工程)
- [8. 应用层代码迁移](#8-应用层代码迁移)
- [9. Keil 工程设置与编译](#9-keil-工程设置与编译)
- [10. 上电验证清单](#10-上电验证清单)
- [11. 性能与实时性评估](#11-性能与实时性评估)
- [12. 常见问题排查](#12-常见问题排查)
- [附录 A：CubeMX 参数速查表](#附录-acubemx-参数速查表)
- [附录 B：关键生成代码核对](#附录-b关键生成代码核对)
- [附录 C：CAN 与串口电机 API 对照](#附录-ccan-与串口电机-api-对照)
- [附录 D：F407 → F103 差异清单](#附录-df407--f103-差异清单)

---

## 快速配置清单（TL;DR）

如果你只想按最短路径把 CubeMX 配完，按下面 10 步做；细节和原因见后文。

1. **新建工程**：`STM32F103C8Tx` / LQFP48 / `MDK-ARM V5` / `FW_F1 V1.8.x` / `Advanced`。
2. **RCC**：HSE = Crystal；**SYS → Debug = Serial Wire，Timebase = TIM4**（C8T6 没有 TIM7）。
3. **时钟**：8 MHz ×9 = 72 MHz；AHB /1、APB1 /2（36 MHz）、APB2 /1（72 MHz）、Flash 2 WS。
4. **CAN1**：PB8 = CAN1_RX、PB9 = CAN1_TX；PSC **6**、BS1 **9TQ**、BS2 **2TQ**、SJW **1TQ** → 500 kbps；NVIC CAN1_RX0 = 5。
5. **串口**：USART1 PA9/PA10、USART2 PA2/PA3、USART3 PB10/PB11，全部 115200-8N1；三个都开 RX 中断，优先级 5。
6. **PWM**：TIM2_CH1 = PA0，PSC **99**、ARR **239**（3 kHz）；TIM3_CH1/2/3 = PA6/PA7/PB0，PSC **71**、ARR **19999**（50 Hz）。
7. **NVIC**：Priority Group = 4 bits；CAN/UART/DMA = 5；TIM4 时基 = 15。
8. **FreeRTOS**：`Middleware → FREERTOS`，Interface 先改成 **CMSIS_V1**；tick 1000 Hz；heap **12288 B**；4 个任务见 6.1 节，其余逐项见 6.2～6.4。
9. **生成后迁移代码**：复制 `ExHardware/`、`Task/`、`UserLibrary/`、`RobotConfig.h`、`header.h`；`huart6→huart2`、`huart5→huart1`、`stm32f4xx_hal.h→stm32f1xx_hal.h`。
10. **Keil**：器件 `STM32F103C8`；宏 `USE_HAL_DRIVER,STM32F103xB`；FreeRTOS port 用 `ARM_CM3`；不勾 FPU。

---

## 0. 先读这一节：移植边界与两个必须做的选择

### 0.1 要保留的功能

按转接板的设计，C8T6 上保留以下功能：

| 功能 | 外设 | C8T6 引脚 |
|---|---|---|
| 陀螺仪 JY901S | USART3 | PB10 / PB11 |
| 树莓派通信 | USART2 | PA2 / PA3 |
| CAN 通信 | CAN1（重映射） | PB8 / PB9 |
| 串口控制电机 | USART1 | PA9 / PA10 |
| 三个舵机 | TIM3 CH1/2/3 | PA6 / PA7 / PB0 |
| 补光灯 PWM | TIM2 CH1 | PA0 |
| SWD 下载调试 | SWD | PA13 / PA14 |

C8T6 只有 **USART1/2/3 三个串口**，没有 USART6、UART4、UART5；也没有 TIM6/TIM7。这两点是本次移植和 F407 最大的区别。

### 0.2 不能直接把 F407 的 `.ioc` 改成 F103

CubeMX 不支持跨芯片系列直接改型号。现有 [UniversalCar.ioc](../UniversalCar.ioc) 是 `STM32F407VETx` + `FW_F4 V1.28.3`，只能作为“参数对照表”使用。正确做法是：

1. 新建一个 F1 工程（建议目录 `UniversalCar_F103/`，不要覆盖原工程）；
2. 按本文第 3～6 节配置；
3. 生成代码后，把 `ExHardware/`、`Task/`、`UserLibrary/` 和少部分 `Core/Inc` 文件复制进去（第 8 节）。

### 0.3 选择一：电机走 CAN 还是串口

现有工程里有两套互斥的 ZDT 电机驱动：

| 文件 | 通信方式 | 说明 |
|---|---|---|
| `ExHardware/ZdtMotor.c` | CAN1 | 当前生效（`MOTOR_USE_UART=0`） |
| `ExHardware/ZdtMotorUart.c` | UART（原 UART5，现 USART1） | 备用版本，已加入 Keil 工程但被 `#if MOTOR_USE_UART` 排除 |

两个文件定义了**同名符号**：`ZdtMotor_Init`、`ZdtMotor_IsOnline`、`ZdtMotor_SetSpeed`、`ZdtMotor_MultiAddSpeed`、`ZdtMotor_SetPosition`、`ZdtMotorList`。因此：

- **逻辑上只能二选一编译**，否则链接报 `L6200E: Symbol ... multiply defined`。当前工程的做法是**两个 .c 都加进 Keil**，但各自整体用条件编译包住：`ZdtMotor.c` 是 `#if !MOTOR_USE_UART`，`ZdtMotorUart.c` 是 `#if MOTOR_USE_UART`，因此只有一份参与编译。
- 方案 A（CAN 模式）：保留 `ZdtMotor.c`，`ZdtMotorUart.c` 不编译；USART1 可空出来做调试口。
- 方案 B（串口模式）：保留 `ZdtMotorUart.c`，`ZdtMotor.c` 不编译；USART1 接电机，调试口需要另想办法。
- 方案 C（混合）：CAN 和串口各控制一部分电机。需要把其中一套的公共函数改名，工作量较大，**不建议第一次上电就这么做**。

> 建议：第一次上电先用**方案 A（CAN）**把板子、时钟、FreeRTOS、树莓派、陀螺仪、舵机、补光灯全部验证通过；确认硬件无误后，再决定电机最终走 CAN 还是串口。转接板已经把两种接口都引出来了，硬件上不需要改。

### 0.4 选择二：调试口怎么办

C8T6 的三个串口被 Pi、陀螺仪、串口电机占满后，就没有独立的 UART 调试口了。可选方案：

1. **CAN 模式**：USART1（PA9/PA10）空着，接 USB-TTL 做调试口，`DebugTask` 可以继续用。
2. **串口电机模式**：USART1 被电机占用，`DebugTask` 必须关闭。调试改用：
   - Keil 调试器 Watch 窗口直接看变量（最省事）；
   - SWO/ITM：PB3 是 `TRACESWO`，SWD 模式下可用，Keil 支持 `ITM printf`；
   - 临时拔掉电机，把 USART1 当调试口用（仅限台架调试）。

本文的 CubeMX 外设配置（时钟、CAN、TIM、三个 USART）对两种模式都适用；**当前 `Universalcar_103` 工程采用方案 A（CAN 电机，`MOTOR_USE_UART=0`），USART1 预留作后续调试口**。串口电机驱动 `ZdtMotorUart.c` 仍留在 Keil 工程里，整体用 `#if MOTOR_USE_UART` 包住，把宏改成 1 即可切换到串口电机，不用改工程文件。两种模式的代码差异见第 8.4 节。

---

## 1. 硬件转接板核对

### 1.1 引脚映射总表

下表中的 F407 引脚号是 **LQFP100** 封装，C8T6 引脚号是 **LQFP48** 封装。转接板已经做好，建议用万用表蜂鸣档逐条核对。

| 功能 | 信号方向 | F407 引脚 / 脚号 | C8T6 引脚 / 脚号 | C8T6 外设 | 备注 |
|---|---|---|---|---|---|
| 陀螺仪 TX → MCU | 输入 | PB11 / 48 | PB11 / 22 | USART3_RX | 直连 |
| 陀螺仪 RX ← MCU | 输出 | PB10 / 47 | PB10 / 21 | USART3_TX | 直连 |
| 树莓派 TX → MCU | 输入 | PC7 / 64 | PA3 / 13 | USART2_RX | 改线 |
| 树莓派 RX ← MCU | 输出 | PC6 / 63 | PA2 / 12 | USART2_TX | 改线 |
| CAN RX | 输入 | PB8 / 95 | PB8 / 45 | CAN1_RX（重映射） | 直连 |
| CAN TX | 输出 | PB9 / 96 | PB9 / 46 | CAN1_TX（重映射） | 直连 |
| 串口电机 TX | 输出 | PC12 / 80 | PA9 / 30 | USART1_TX | 改线 |
| 串口电机 RX | 输入 | PD2 / 83 | PA10 / 31 | USART1_RX | 改线 |
| 舵机 1（夹爪） | 输出 | PA6 / 31 | PA6 / 16 | TIM3_CH1 | 直连 |
| 舵机 2（载物盘） | 输出 | PA7 / 32 | PA7 / 17 | TIM3_CH2 | 直连 |
| 舵机 3（摄像头） | 输出 | PB0 / 35 | PB0 / 18 | TIM3_CH3 | 直连 |
| 补光灯 PWM | 输出 | PA5 / 30 | PA0 / 10 | TIM2_CH1 | 改线 |
| SWDIO | 双向 | PA13 / 72 | PA13 / 34 | SWD | 必须保留 |
| SWCLK | 输入 | PA14 / 76 | PA14 / 37 | SWD | 必须保留 |
| 5 V | 电源 | — | — | — | 见 1.2 |
| GND | 电源 | — | — | — | 必须共地 |

> **重点**：F407 的补光灯在 PA5（TIM2_CH1），但 F103 的 PA5 不是 TIM2_CH1（PA5 是 SPI1_SCK / ADC12_IN5）。F103 的 TIM2_CH1 默认在 **PA0**，所以转接板把补光灯改到了 PA0。软件里 `LED_PWM_CH` 仍然是 `TIM_CHANNEL_1`，只是底层 `htim2` 的 CH1 输出脚变成了 PA0。

### 1.2 电源与电平

- **C8T6 是 3.3 V 器件**。如果用的是 Blue Pill 最小系统板，5 V 应接到板子的 `5V` 引脚，由板载稳压器转 3.3 V；**不要把 5 V 直接接到芯片 VDD（24/36/48 脚）**。
- 三个舵机必须**单独 5～6 V 供电**，不能从 C8T6 的 3.3 V 稳压器取电；舵机电源地与 C8T6 地必须共地。
- JY901S、树莓派 UART、CAN 收发器通常都是 3.3 V 逻辑。若原 407 板上的 CAN 收发器是 5 V 型号（如 TJA1050），其 RXD 输出为 5 V；F103 的 PB8 是 5 V 容忍（FT）引脚，通常可以承受，但更推荐换 **SN65HVD230**（3.3 V）以留足电平裕量。PB9（TXD）输出 3.3 V，TJA1050 的 TXD 高电平门限约 2.0 V，一般也能识别。
- CAN 总线两端需要 **120 Ω 终端电阻**；转接板如果已经焊了，确认整条总线只有两端各一个。
- BOOT0 用 **10 kΩ 下拉到 GND**；BOOT1（PB2）也用 10 kΩ 下拉到 GND；NRST 建议 100 nF 到 GND；8 MHz 晶振配 20 pF 负载电容（按晶振规格书）。
- VBAT（1 脚）无纽扣电池时接 **3.3 V**，不要悬空。
- VDDA/VSSA（9/8 脚）建议加 100 nF + 1 μF 去耦；每个 VDD 脚（24/36/48）附近放 100 nF。

### 1.3 转接板自检

上电前（不插 C8T6）用万用表检查：

1. 5 V 与 GND 不短路，3.3 V 与 GND 不短路；
2. 每个信号脚与对应的 F407 脚印通，且与相邻脚不短路；
3. CAN_H/CAN_L 之间约 60 Ω（两个 120 Ω 并联，仅在整个总线接好时测）；
4. 舵机电源与 C8T6 3.3 V 不连通；
5. SWDIO/SWCLK 与 GND 不短路；
6. **确认 C8T6 的 PA9/PA10 只接到原 UART5（PC12/PD2）的网络**，不要同时接到原 F407 调试口 PA9/PA10，否则串口电机和调试口会互相干扰；
7. **确认 C8T6 的 PA2/PA3 只接到原 USART6（PC6/PC7）的网络**，不要和板上其他串口网络短接；
8. 如果 CAN 模式下要把 USART1 当调试口，确认串口电机已断开或转接板上有跳线，避免调试器和电机 TX 同时驱动 PA10。

---

## 2. 新建 CubeMX 工程

### 2.1 选择芯片

1. 打开 STM32CubeMX → `File` → `New Project`；
2. 在 `Part Number` 搜索框输入 **STM32F103C8Tx**；
3. 选择 **LQFP48** 封装，双击进入；
4. 确认芯片图上是 `STM32F103C8Tx`，Flash 64 KB、RAM 20 KB。

### 2.2 工程参数

在 `Project Manager` 页设置：

| 项目 | 建议值 |
|---|---|
| Project Name | `UniversalCar_F103` |
| Project Location | 与 F407 工程**不同的目录**（例如 `电控代码/UniversalCar_F103`） |
| Application Structure | `Advanced` |
| Toolchain / IDE | `MDK-ARM V5` |
| Firmware Package | `STM32Cube FW_F1 V1.8.x`（用最新可用版本） |
| Minimum Heap Size | `0x200` |
| Minimum Stack Size | `0x400` |

> 选 `Advanced` 是为了得到和原工程一致的 `Core/Inc`、`Core/Src`、`Drivers/` 目录结构，方便直接复制应用层文件。

### 2.3 Code Generator 设置

在 `Project Manager` → `Code Generator` 页：

- ✅ `Copy only the necessary library files`
- ✅ `Generate peripheral initialization as a pair of '.c/.h' files per peripheral`
- ✅ `Keep User Code when re-generating`
- ✅ `Delete previously generated files when not re-generated`（生成前先备份；如果你手动往 `Core/` 放过文件，生成前确认它们不会被删）
- ❌ 不要勾选 `Do not generate the main()`

---

## 3. 时钟与系统配置

### 3.1 RCC

`Pinout & Configuration` → `System Core` → `RCC`：

- `High Speed Clock (HSE)`：**Crystal/Ceramic Resonator**
- `Low Speed Clock (LSE)`：Disable（本项目不用 RTC）
- `Master Clock Output (MCO)`：Disable

### 3.2 SYS

`System Core` → `SYS`：

| 项目 | 值 | 说明 |
|---|---|---|
| Debug | **Serial Wire** | 只占 PA13/PA14，释放 PB3/PB4/PA15 |
| Timebase Source | **TIM4** | **C8T6 没有 TIM6/TIM7**，不能选 TIM7 |

> 这是本次移植最容易踩的坑：原 F407 工程用 TIM7 做 HAL 时基，C8T6 上 TIM7 不存在。CubeMX 的 `Timebase Source` 下拉框里选 **TIM4**。TIM4 只用作 1 ms 计数，不占用任何引脚（不使能输出通道）。

### 3.3 Clock Configuration

在 `Clock Configuration` 页按下面的值填（假设板载 HSE 为 8 MHz）：

| 项目 | 值 |
|---|---|
| Input frequency (HSE) | `8` MHz |
| PLL Source | `HSE` |
| PLLMul | `×9` |
| SYSCLK | **72 MHz** |
| AHB Prescaler | `/1` → HCLK 72 MHz |
| APB1 Prescaler | `/2` → PCLK1 **36 MHz** |
| APB2 Prescaler | `/1` → PCLK2 72 MHz |
| ADC Prescaler | `/6` → 12 MHz |
| USB Prescaler | `/1.5` → 48 MHz |
| Flash Latency | **2 WS** |

如果板载晶振不是 8 MHz：

- 12 MHz → PLLMul 选 `×6`，仍可得到 72 MHz；
- 16 MHz → F1 的 `PLLMUL` 只有整数档（×2～×16），无法用 ×4.5 精确得到 72 MHz；建议换 8 MHz 晶振，或降到 64 MHz（×4）。**不要超到 80 MHz**。

> 注意：F1 的 PLL 只有 `PLLMUL`，没有 F4 的 `PLLM/PLLN/PLLP/PLLQ`。生成后的 `SystemClock_Config()` 里应看到 `RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;` 和 `HAL_RCC_ClockConfig(..., FLASH_LATENCY_2)`。

### 3.4 生成结果核对

生成后打开 `Core/Src/main.c`，`SystemClock_Config()` 应该类似附录 B.1。关键点：

- `RCC_OscInitStruct.HSEState = RCC_HSE_ON;`
- `RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;`
- `RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;`
- `HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);`

同时会生成 `Core/Src/stm32f1xx_hal_timebase_tim.c`，里面用 **TIM4** 做 1 ms 时基。`main.c` 末尾的 `HAL_TIM_PeriodElapsedCallback()` 应该判断 `htim->Instance == TIM4`（附录 B.2）。如果它还在判断 TIM7，说明 Timebase 选错了。

---

## 4. 外设配置

### 4.1 CAN1（PB8/PB9，500 kbps）

#### 引脚

在芯片图上：

1. 点击 `PB8` → 选择 `CAN1_RX`；
2. 点击 `PB9` → 选择 `CAN1_TX`。

CubeMX 会自动启用 **AFIO 重映射**。生成后的 `HAL_CAN_MspInit()` 里应包含：

```c
__HAL_RCC_AFIO_CLK_ENABLE();
__HAL_AFIO_REMAP_CAN1_2();
```

如果没有这两句（不同 CubeMX 版本行为略有差异），手动加在 `USER CODE BEGIN CAN1_MspInit 0` 里：

```c
/* USER CODE BEGIN CAN1_MspInit 0 */
__HAL_RCC_AFIO_CLK_ENABLE();
__HAL_AFIO_REMAP_CAN1_2();     /* CAN1 重映射到 PB8/PB9 */
/* USER CODE END CAN1_MspInit 0 */
```

> 注意：F1 的 CAN 引脚没有 `GPIO_Alternate` 字段。典型配置是 PB8 配成 `GPIO_MODE_INPUT`、PB9 配成 `GPIO_MODE_AF_PP`、`GPIO_SPEED_FREQ_HIGH`。不要照抄 F407 的 `GPIO_AF9_CAN1`，F1 没有这个宏。

#### 参数（Parameter Settings）

| 参数 | 值 |
|---|---|
| Prescaler (for Time Quantum) | **6** |
| Time Quanta in Bit Segment 1 | **9 Times** |
| Time Quanta in Bit Segment 2 | **2 Times** |
| ReSynchronization Jump Width | **1 Time** |
| Time Triggered Communication Mode | Disabled |
| Automatic Bus-Off Management | **Enabled** |
| Automatic Wake-Up Mode | Disabled |
| Automatic Retransmission | **Disabled** |
| Receive Fifo Locked Mode | Disabled |
| Transmit Fifo Priority | Disabled |

计算：APB1 = 36 MHz，PSC = 6 → 6 MHz；1 + 9 + 2 = 12 TQ → **500 kbps**，采样点 83.3%。CubeMX 右侧应显示 `500000 bit/s`。

> F1 的 CAN 位定时寄存器 `CAN_BTR` 在以上参数下为 `0x00180006`，可在调试器里核对。

#### 过滤器与中断

- CubeMX 不需要手动加过滤器；应用层 `ZdtCan_Init()` 会配置 **Filter Bank 0**、32 位掩码模式、全部接收、FIFO0。
- F103 有 14 个过滤器组（0～13），`SlaveStartFilterBank = 14` 保持原值即可（单 CAN 时该字段不生效）。
- 在 `NVIC Settings` 里勾选 **CAN1 RX0 interrupt**，优先级设为 **5**。

#### 生成后的回调

在 `Core/Src/can.c` 的 `USER CODE BEGIN 1` 中保留：

```c
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    CAN_RxHeaderTypeDef RxMessage;
    uint8_t Data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMessage, Data) == HAL_OK)
        ZdtCan_RxCallback(RxMessage.ExtId, Data, (uint8_t)RxMessage.DLC);
}
```

> 如果最终选择串口电机模式（不编译 `ZdtMotor.c`），这个回调会引用不存在的 `ZdtCan_RxCallback`，必须删除或用 `#if` 包住，见第 8.4 节。

### 4.2 USART1（PA9/PA10，115200）

| 参数 | 值 |
|---|---|
| Mode | Asynchronous |
| Baud Rate | 115200 |
| Word Length | 8 Bits |
| Parity | None |
| Stop Bits | 1 |
| Data Direction | Receive and Transmit |
| Over Sampling | 16 |
| Hardware Flow Control | None |

- **CAN 模式**：USART1 用作调试口。可在 `DMA Settings` 里给 `USART1_TX` 加 **DMA1 Channel 4**（Memory → Peripheral，Byte，Normal，Low），`DebugTask` 会用 `HAL_UART_Transmit_DMA`。
- **串口电机模式**：USART1 接电机。必须开启 **USART1 global interrupt**（NVIC 优先级 5），并在 `MX_USART1_UART_Init()` 的 `USER CODE BEGIN USART1_Init 2` 里启动接收：

```c
HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
```

### 4.3 USART2（PA2/PA3，115200，树莓派）

| 参数 | 值 |
|---|---|
| Mode | Asynchronous |
| Baud Rate | 115200 |
| Word Length | 8 Bits |
| Parity | None |
| Stop Bits | 1 |
| Over Sampling | 16 |

- 开启 **USART2 global interrupt**，NVIC 优先级 **5**。
- 在 `USER CODE BEGIN USART2_Init 2` 中启动接收：

```c
HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
```

- 可选优化：给 `USART2_TX` 加 **DMA1 Channel 7**。当前 `PiComm_SendFrame()` 用的是阻塞发送，17 字节 @115200 约 1.5 ms；50 Hz 下约占 7% CPU。CPU 紧张时再改 DMA，第一次上电可先不加。

### 4.4 USART3（PB10/PB11，115200，JY901S）

| 参数 | 值 |
|---|---|
| Mode | Asynchronous |
| Baud Rate | 115200 |
| Word Length | 8 Bits |
| Parity | None |
| Stop Bits | 1 |
| Over Sampling | 16 |

- 开启 **USART3 global interrupt**，NVIC 优先级 **5**。
- 在 `USER CODE BEGIN USART3_Init 2` 中启动接收：

```c
HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
```

### 4.5 TIM2（PA0，补光灯，3 kHz）

在 `Timers` → `TIM2`：

| 参数 | 值 |
|---|---|
| Clock Source | Internal Clock |
| Channel 1 | **PWM Generation CH1** |
| Prescaler | **99** |
| Counter Mode | Up |
| Counter Period (ARR) | **239** |
| Auto-reload Preload | Disable |
| PWM Mode | PWM mode 1 |
| Pulse | 0 |
| Output Polarity | High |
| Fast Mode | Disable |

计算：TIM2 时钟 = 72 MHz（APB1 = 36 MHz，定时器时钟 ×2）；72 MHz / (99+1) / (239+1) = **3 kHz**。

> 引脚必须是 **PA0**，不是 PA5。F103 的 PA5 没有 TIM2_CH1 复用功能。

### 4.6 TIM3（PA6/PA7/PB0，三路舵机，50 Hz）

在 `Timers` → `TIM3`：

| 参数 | 值 |
|---|---|
| Clock Source | Internal Clock |
| Channel 1/2/3 | **PWM Generation CH1/CH2/CH3** |
| Prescaler | **71** |
| Counter Mode | Up |
| Counter Period (ARR) | **19999** |
| Auto-reload Preload | Disable |
| PWM Mode | PWM mode 1 |
| Pulse（三路初始值） | **500** |
| Output Polarity | High |
| Fast Mode | Disable |

计算：72 MHz / (71+1) / (19999+1) = **50 Hz**，周期 20 ms。比较值 500～2500 对应 0.5～2.5 ms 脉宽，即 0°～180°。

### 4.7 DMA（可选）

如果使用 DMA，在 `System Core` → `DMA` 或外设的 `DMA Settings` 中添加：

| 外设 | 方向 | DMA1 通道 | 模式 | 数据宽度 | 优先级 |
|---|---|---|---|---|---|
| USART1_TX（调试） | Memory → Peripheral | **Channel 4** | Normal | Byte | Low |
| USART2_TX（Pi，可选） | Memory → Peripheral | **Channel 7** | Normal | Byte | Low |

对应的 DMA 中断优先级设为 **5**。

> 如果完全不用 DMA，CubeMX 不会生成 `Core/Src/dma.c` 和 `Core/Inc/dma.h`，而现有 [header.h](../Core/Inc/header.h) 里有 `#include "dma.h"`，会编译失败。两种处理方式：
> 1. 保留至少一个 DMA 请求（例如 USART1_TX），让 CubeMX 生成 `dma.c/h`；
> 2. 或从 `header.h` 删除 `#include "dma.h"`，并从 `main.c` 删除 `MX_DMA_Init()`。

### 4.8 GPIO

本项目没有额外的普通 GPIO。CubeMX 的 `GPIO` 页只需要确认：

- PA13/PA14 被 SWD 占用，不要配置成 GPIO；
- 其余未使用引脚可以保持默认。若 CubeMX 提示 “Free pins as analog”，建议启用（降低浮空输入噪声），但不要影响 SWD 和晶振脚。

---

## 5. NVIC 中断优先级

在 `System Core` → `NVIC` 中设置：

| 中断 | 优先级 | 说明 |
|---|---|---|
| TIM4（HAL 时基） | **15** | CubeMX 默认值，保持即可；它不调用 FreeRTOS API |
| CAN1 RX0 | **5** | 与 FreeRTOS 的 `configMAX_SYSCALL_INTERRUPT_PRIORITY = 5` 对齐 |
| USART1 | **5** | |
| USART2 | **5** | |
| USART3 | **5** | |
| DMA1 Channel 4/7（如使用） | **5** | |

- `Priority Group` 设为 **4 bits for pre-emption priority**（`NVIC_PRIORITYGROUP_4`）。
- FreeRTOS 的规则：任何会调用 `...FromISR()` 的中断，其优先级数值必须 **≥ 5**（数值越大优先级越低）。本项目 UART/CAN 回调目前没有直接调用 FreeRTOS API，但仍统一设为 5，给以后留余地。
- 不要把 UART/CAN 中断设成 0～4，否则一旦在回调里调用 FreeRTOS API，会触发 `configASSERT` 或 HardFault。

---

## 6. FreeRTOS 配置（逐项定位到 CubeMX 界面）

本节按 CubeMX 的**界面层级**写：先告诉你左侧树点哪里、右侧哪个 Tab、哪个分组框、控件叫什么，再给值。

### 6.0 找到面板，先改 Interface

1. **左侧分类树**（Category Tree）点 `Middleware` → 展开后点 `FREERTOS`。
2. 右侧配置区出现 `Mode` 分组框，勾选 **`FreeRTOS`**（默认未勾选，勾上后才生效）。
3. 同一面板顶部有 **`Interface`** 下拉框 —— **最关键的选项**。CubeMX 6.x 默认是 `CMSIS_V2`，必须改成 **`CMSIS_V1`**。
   - 现有代码用的是 `osThreadDef()` / `osThreadCreate()` / `osThread()`，这套 API 只存在于 CMSIS-RTOS v1（`Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c`）；选 V2 会走 `cmsis_os2.c`，直接编译报 `osThreadDef` 未定义。
   - **先改 Interface，再填任务**：切换 Interface 会清空 `Tasks and Queues` 页已填的内容。
4. 改完 Interface 后，右侧面板顶部出现一排 Tab（不同版本顺序略有差异）：

```
┌ Tasks and Queues ┬ Timers and Semaphores ┬ Mutexes ┬ Events ┬ FreeRTOS Heap Usage ┬
│ Config parameters ┬ Include parameters ┬ User Constants ┬ Advanced settings      │
└──────────────────────────────────────────────────────────────────────────────────┘
```

| Tab | 是否要动 | 作用 |
|---|---|---|
| `Tasks and Queues` | **要** | 建 4 个任务，见 6.1 |
| `Timers and Semaphores` | 不动 | 软件定时器/信号量，本项目没有 |
| `Mutexes` | 不动 | 互斥量，本项目没有 |
| `Events` | 不动 | 事件标志组，本项目没有 |
| `FreeRTOS Heap Usage` | 只看 | 只读，堆占用预估，见 6.5 |
| `Config parameters` | **要** | tick、堆大小、中断优先级等，见 6.2 |
| `Include parameters` | **要** | `INCLUDE_*` 裁剪，见 6.3 |
| `User Constants` | 不动 | 自定义宏 |
| `Advanced settings` | 不动 | 保持默认，见 6.4 |

> 找不到某个选项时：CubeMX 配置区上方有搜索框，直接输入宏名（如 `TOTAL_HEAP_SIZE`、`vTaskDelayUntil`）即可过滤定位。

### 6.1 `Tasks and Queues` 页：建 4 个任务

页面布局：上半部分是 **`Tasks`** 表，下半部分是 `Queues` 和 `Binary Semaphores` 表（本项目两张都保持空）。

**`Tasks` 表共 9 列**，从左到右：

`Task Name` | `Priority` | `Stack Size` | `Entry Function` | `Code Generation Option` | `Parameter` | `Allocation` | `Buffer Size` | `Control Block`

表下方有 **`Add`** 按钮，点 4 次加 4 行（`Delete` 删当前行）。`Buffer Size` / `Control Block` 两列只在 `Allocation = Static` 时可填，这里留空。

逐列填法：

| 列 | 填什么 | 说明 |
|---|---|---|
| `Task Name` | 见下表 | 也决定生成的任务句柄名（`receiveTaskHandle` 等） |
| `Priority` | 下拉选 `Idle / Low / BelowNormal / Normal / AboveNormal / High / Realtime` | 对应 `osPriority*`，数值见下表 |
| `Stack Size` | 见下表 | **单位是字（Word）**，见下方“单位陷阱” |
| `Entry Function` | 见下表 | **必须与 `Task/*.c` 里的函数名逐字一致（大小写敏感）** |
| `Code Generation Option` | **`As weak`** | 必须，理由见下 |
| `Parameter` | 保持 `NULL` | 传给任务入口的参数，本项目不用 |
| `Allocation` | **`Dynamic`** | 栈 + TCB 从 FreeRTOS 堆里分配 |
| `Buffer Size` | 留空 | 仅 Static 用 |
| `Control Block` | 留空 | 仅 Static 用 |

4 行的具体值：

| Task Name | Priority（下拉项 / `osPriority` 数值） | Stack Size（字） | 实际字节 | Entry Function | 周期 |
|---|---|---|---|---|---|
| `receiveTask` | **AboveNormal** / 1 | **512** | 2048 | `ReceiveTask` | 10 ms |
| `imuTask` | **High** / 2 | **384** | 1536 | `ImuTask` | 5 ms |
| `chassisTask` | **Normal** / 0 | **256** | 1024 | `ChassisTask` | 5 ms |
| `gimbalTask` | **Normal** / 0 | **256** | 1024 | `GimbalTask` | 10 ms |

> **当前工程没有 `debugTask`**（已从 CubeMX 和 Keil 工程删除）。CAN 模式下 USART1 空闲，若以后想加串口调试，可以在 CubeMX 里再建一个低优先级任务、重新加入 `Task/DebugTask.c`；**但切到串口电机模式（`MOTOR_USE_UART=1`）时必须删除它**，否则调试输出会和电机抢 USART1。6.6/6.7 的栈表仍保留 debugTask 一行，是按 5 任务算的上限参考；实际 4 个任务合计 **1,408 字 = 5,632 B**（另加 4 个 TCB）。

**`Code Generation Option` 必须选 `As weak`。** 这样 CubeMX 在 `Core/Src/freertos.c` 里生成的是空壳：

```c
__weak void ReceiveTask(void const * argument)
{
  for(;;)
  {
    osDelay(1);
  }
}
```

`Task/ReceiveTask.c` 里的同名强定义会覆盖它。选错的后果：

- 选 `Default`（强定义）→ 链接报 `L6200E: Symbol ReceiveTask multiply defined`（`freertos.c` 和 `Task/ReceiveTask.c` 撞车）。
- 选 `As external` → `freertos.c` 不生成函数体，也能用；但如果对应 `Task/*.c` 没加进 Keil 工程，链接会报 `Undefined symbol ReceiveTask`。
- `Entry Function` 拼错（如写成 `ReceiveTask_`）→ 不报错，但你自己写的任务函数永远不会被调用，现象是**该任务完全不执行**。

> **单位陷阱**：CubeMX/CMSIS-RTOS **v1** 里填的 `Stack Size` 最终作为 `xTaskCreate()` 的 `usStackDepth`，单位是 **字（4 字节）**，不是字节；CMSIS_V2 的 `osThreadAttr_t.stack_size` 才是字节。以 `cmsis_os.c` 实现为准（`thread_def->stacksize` 原样传给 `xTaskCreate`）。所以 512 表示 512 字 = 2048 字节。

### 6.2 `Config parameters` 页：9 个分组框

这一页是一整页可滚动的**分组框**（带标题的方框），逐组填。下表“分组”列就是界面上的方框标题。

#### ① `API / Version`（只读，确认即可）

| 选项 | 应为 | 说明 |
|---|---|---|
| `FreeRTOS API` | `CMSIS-RTOS v1.02` | 若显示 v2，说明 6.0 的 Interface 没改 |
| `FreeRTOS Kernel Version` | `V10.x` | 只读 |
| `CMSIS Version` | 任意 | 只读 |

#### ② `MPU / FPU`

| 选项 | 设置 | 生成的宏 |
|---|---|---|
| `ENABLE_MPU` | `Disabled` | `configENABLE_MPU 0` |
| `ENABLE_FPU` | `Disabled` | `configENABLE_FPU 0` |

C8T6（Cortex-M3）没有 FPU/MPU，这一组可能整组不显示，属正常。

#### ③ `Kernel settings`

| 选项 | 设置 | 生成的宏 | 说明 |
|---|---|---|---|
| `USE_PREEMPTION` | `Enabled` | `configUSE_PREEMPTION 1` | 抢占式调度，现有代码依赖 |
| `USE_16_BIT_TICKS` | `Disabled` | `configUSE_16_BIT_TICKS 0` | `TickType_t` 用 32 位；开 16 位会在 ~65 s 溢出 |
| `IDLE_SHOULD_YIELD` | `Disabled` | `configIDLE_SHOULD_YIELD 0` | 本项目没有与 Idle 同优先级的任务，影响可忽略 |
| `USE_MUTEXES` | `Enabled` | `configUSE_MUTEXES 1` | 保持与原工程一致 |
| `USE_RECURSIVE_MUTEXES` | `Disabled` | `configUSE_RECURSIVE_MUTEXES 0` | |
| `USE_COUNTING_SEMAPHORES` | `Disabled` | `configUSE_COUNTING_SEMAPHORES 0` | |
| `QUEUE_REGISTRY_SIZE` | `8` | `configQUEUE_REGISTRY_SIZE 8` | 给队列/信号量起名调试用 |
| `USE_PORT_OPTIMISED_TASK_SELECTION` | `Enabled` | `... 1` | Cortex-M3 用 `CLZ` 指令找最高优先级任务，比通用循环快 |
| `USE_TICKLESS_IDLE` | `Disabled` | `configUSE_TICKLESS_IDLE 0` | 低功耗用，本项目不适用 |
| `CPU_CLOCK_HZ` | 保持 `SystemCoreClock` | `configCPU_CLOCK_HZ ( SystemCoreClock )` | 自动跟随时钟树，不要填死数字 |
| `TICK_RATE_HZ` | **`1000`** | `configTICK_RATE_HZ 1000` | 1 tick = 1 ms；`osDelayUntil(..., 5)` 就是 5 ms |
| `MAX_PRIORITIES` | **`7`** | `configMAX_PRIORITIES ( 7 )` | 可用优先级 0～6；CMSIS 的 `osPriorityLow`(-2) 会映射到 0 附近 |
| `MINIMAL_STACK_SIZE` | **`128`** | `configMINIMAL_STACK_SIZE 128` | 单位：**字**；只决定 Idle 任务栈（512 B） |
| `MAX_TASK_NAME_LEN` | **`16`** | `configMAX_TASK_NAME_LEN 16` | 包含结尾 `'\0'` |

#### ④ `Memory management settings`

| 选项 | 设置 | 生成的宏 | 说明 |
|---|---|---|---|
| `Memory Allocation` | **`Dynamic`** | `configSUPPORT_DYNAMIC_ALLOCATION 1` | 任务栈/TCB 走 `pvPortMalloc` |
| `TOTAL_HEAP_SIZE` | **`12288`** | `configTOTAL_HEAP_SIZE 12288` | 单位**字节**，直接占用 SRAM |
| `Memory Management scheme` | **`heap_4`** | 决定工程里编 `heap_4.c` | 带相邻空闲块合并，适合反复创建/删除；本项目任务只创建一次，`heap_3` 也能用，但保持 `heap_4` 与原工程一致 |

> 生成后的 `FreeRTOSConfig.h` 里 `configSUPPORT_STATIC_ALLOCATION` 也是 `1`，这是 CubeMX 给 Idle/Timer 任务做静态分配用的（`vApplicationGetIdleTaskMemory`），不用手动改，也不额外吃堆。

#### ⑤ `Hook function related definitions`

| 选项 | 设置 | 生成的宏 | 说明 |
|---|---|---|---|
| `USE_IDLE_HOOK` | `Disabled` | `configUSE_IDLE_HOOK 0` | |
| `USE_TICK_HOOK` | `Disabled` | `configUSE_TICK_HOOK 0` | |
| `USE_MALLOC_FAILED_HOOK` | **`Enabled`**（调试期） | `configUSE_MALLOC_FAILED_HOOK 1` | 堆不够时进 `vApplicationMallocFailedHook()`，可下断点；量产可关 |
| `USE_DAEMON_TASK_STARTUP_HOOK` | `Disabled` | — | V2 才有，V1 可能不显示 |
| `CHECK_FOR_STACK_OVERFLOW` | **`Option 2`**（调试期） | `configCHECK_FOR_STACK_OVERFLOW 2` | Option 2 比 Option 1 多检查一个哨兵字，最可靠；量产可关 |

> 这两个钩子函数是否由 CubeMX 生成，取决于版本（现有 F407 工程的 `freertos.c` 里没有）。开启后先看 `Core/Src/freertos.c` 末尾有没有空实现，没有就自己补，否则链接报 `Undefined symbol vApplicationMallocFailedHook` / `vApplicationStackOverflowHook`：
>
> ```c
> void vApplicationMallocFailedHook(void)
> {
>   taskDISABLE_INTERRUPTS();
>   for (;;) { }
> }
>
> void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
> {
>   (void)xTask; (void)pcTaskName;
>   taskDISABLE_INTERRUPTS();
>   for (;;) { }
> }
> ```
>
> 不想写就先别开这两项，对应的报错也不会出现。

#### ⑥ `Run time and task stats gathering definitions`

| 选项 | 设置 | 生成的宏 |
|---|---|---|
| `GENERATE_RUN_TIME_STATS` | `Disabled` | `configGENERATE_RUN_TIME_STATS 0` |
| `USE_TRACE_FACILITY` | `Disabled` | `configUSE_TRACE_FACILITY 0` |
| `USE_STATS_FORMATTING_FUNCTIONS` | `Disabled` | `configUSE_STATS_FORMATTING_FUNCTIONS 0` |

开了 `GENERATE_RUN_TIME_STATS` 还要再配一个高精度定时器做时基，C8T6 定时器本来就紧张，不要开。

#### ⑦ `Co-routine related definitions`

| 选项 | 设置 |
|---|---|
| `USE_CO_ROUTINES` | `Disabled` |
| `MAX_CO_ROUTINE_PRIORITIES` | `2`（保持默认，不生效） |

#### ⑧ `Software timer definitions`

| 选项 | 设置 | 生成的宏 | 说明 |
|---|---|---|---|
| `USE_TIMERS` | **`Disabled`** | `configUSE_TIMERS 0` | 本项目没用 `osTimer`。开启会多一个 Timer 服务任务（默认 256 字栈 ≈ 1 KB + 队列），20 KB SRAM 下不划算 |
| `TIMER_TASK_PRIORITY` | （灰显）2 | — | 仅开启后有效 |
| `TIMER_QUEUE_LENGTH` | （灰显）10 | — | |
| `TIMER_TASK_STACK_DEPTH` | （灰显）256 | — | |

#### ⑨ `Interrupt nesting behaviour configuration`

| 选项 | 设置 | 生成的宏 | 说明 |
|---|---|---|---|
| `LIBRARY_LOWEST_INTERRUPT_PRIORITY` | **`15`** | `configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15` | 内核（PendSV/SysTick）用最低优先级 |
| `LIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` | **`5`** | `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5` | 优先级数值 **0～4 的中断禁止调用任何 FreeRTOS API**（`...FromISR` 也不行） |

这两项必须和第 5 节 `NVIC → Priority Group = 4 bits for pre-emption priority` 配套。`configPRIO_BITS` 自动取 `__NVIC_PRIO_BITS`，不用手填。

### 6.3 `Include parameters` 页：`INCLUDE_*` 勾选表

这一页是一长列复选框，直接对应 `FreeRTOSConfig.h` 的 `INCLUDE_xxx`。**只影响编译进固件的函数数量（Flash），不影响 RAM。**

| 复选框 | 勾选 | 原因 |
|---|---|---|
| `INCLUDE_vTaskDelay` | ✅ **必勾** | `osDelay()` 依赖 |
| `INCLUDE_vTaskDelayUntil` | ✅ **必勾** | `osDelayUntil()` 依赖。4 个任务的主循环全用它；**CubeMX 默认不勾**，漏勾会链接报 `Undefined symbol xTaskDelayUntil` |
| `INCLUDE_vTaskSuspend` | ✅（默认） | `osThreadSuspend/Terminate` 相关 |
| `INCLUDE_vTaskDelete` | ✅（默认） | |
| `INCLUDE_vTaskPrioritySet` | ✅（默认） | |
| `INCLUDE_uxTaskPriorityGet` | ✅（默认） | |
| `INCLUDE_xTaskGetSchedulerState` | ✅（默认） | |
| `INCLUDE_uxTaskGetStackHighWaterMark` | ✅ **建议勾** | 调栈神器：`uxTaskGetStackHighWaterMark(handle)` 返回历史最小剩余字数，用它可以确认 512/384/256 到底够不够 |
| `INCLUDE_vTaskCleanUpResources` | ❌ | |
| `INCLUDE_xTaskGetCurrentTaskHandle` | ❌ | 本项目不用 |
| `INCLUDE_xSemaphoreGetMutexHolder` | ❌ | |
| `INCLUDE_xTaskGetIdleTaskHandle` | ❌ | |
| `INCLUDE_eTaskGetState` | ❌ | |
| `INCLUDE_xTimerPendFunctionCall` | ❌ | 依赖 `USE_TIMERS` |
| `INCLUDE_xTaskAbortDelay` | ❌ | |
| `INCLUDE_xTaskGetHandle` | ❌ | |
| `INCLUDE_xTaskResumeFromISR` | ❌ | 本项目中断里不调用 FreeRTOS API |

> 列表项随 FreeRTOS 内核版本略有出入，**原则只有一条：`osDelay` 和 `osDelayUntil` 用到的两个必须勾**。

### 6.4 `Advanced settings` 页

| 选项 | 设置 | 说明 |
|---|---|---|
| `USE_NEWLIB_REENTRANT` | `Disabled` | 开启后每个任务额外分配 `struct _reent`（100+ B），本项目不用 `printf` 走串口，关掉省 RAM |
| `ENABLE_BACKWARD_COMPATIBILITY` | `Disabled` | 保持默认 |
| 其余项目 | 保持默认 | 不同 CubeMX 版本此页内容有差异，未列出的都别动 |

### 6.5 `FreeRTOS Heap Usage` 页：只读核对

配完任务后点这个 Tab，会列出每个对象占用的堆（栈 + TCB），并给出 `Total` 与 `TOTAL_HEAP_SIZE` 的对比条。本项目预期：

| 对象 | 栈（字） | 栈（字节） |
|---|---|---|
| Idle（静态分配，可能不计入堆） | 128 | 512 |
| receiveTask | 512 | 2048 |
| imuTask | 384 | 1536 |
| chassisTask | 256 | 1024 |
| gimbalTask | 256 | 1024 |
| **4 个任务栈合计** | 1408 | **5632** |
| 4 个 TCB（约 92 B/个） | — | **约 368** |
| **堆占用合计** | — | **约 6.0 KB**（6000 B） |

`TOTAL_HEAP_SIZE = 12288`，剩余约 5.6 KB。若这一页显示的 Total 已经超过 12288，说明栈填大了，回到 6.1 调小。

> 该页是估算值，不含你自己运行时再创建的队列/信号量。

### 6.6 为什么把栈和堆调小

原 F407 工程的配置是：

```c
osThreadDef(receiveTask, ReceiveTask, osPriorityAboveNormal, 0, 1024);
osThreadDef(imuTask,     ImuTask,     osPriorityHigh,        0, 512);
osThreadDef(chassisTask, ChassisTask, osPriorityNormal,      0, 256);
osThreadDef(gimbalTask,  GimbalTask,  osPriorityNormal,      0, 256);
osThreadDef(debugTask,   DebugTask,   osPriorityLow,         0, 128);
```

总栈 = (1024+512+256+256+128) × 4 = **8704 字节**，再加 TCB、队列等，原 `configTOTAL_HEAP_SIZE = 16348` 才够。C8T6 只有 20 KB SRAM，必须压缩：

| 项目 | F407 原值 | C8T6 建议值 |
|---|---|---|
| `configTOTAL_HEAP_SIZE` | 16348 | **12288** |
| receiveTask 栈 | 1024 字（4 KB） | **512 字（2 KB）** |
| imuTask 栈 | 512 字（2 KB） | **384 字（1.5 KB）** |
| chassisTask 栈 | 256 字（1 KB） | **256 字（1 KB）** |
| gimbalTask 栈 | 256 字（1 KB） | **256 字（1 KB）** |
| debugTask 栈 | 128 字（512 B） | **删除**（USART1 改作电机总线） |

### 6.7 RAM 预算（重要）

C8T6：20 KB = 20,480 字节。

| 项目 | 估算 |
|---|---|
| FreeRTOS heap | 12,288 |
| 4 个任务栈（从 heap 分配） | 5,632 |
| 4 个 TCB（从 heap 分配） | 约 368 |
| FreeRTOS 链表/堆块头等 | 约 300～600 |
| **heap 剩余** | **约 5.7 KB** |
| Idle 任务栈 + TCB（静态分配，不在 heap） | 512 + 92 |
| 主栈（`ProjectManager.StackSize = 0x400`） | 1,024 |
| `.data` + `.bss`（除 heap 外） | 约 3.5 KB |
| **合计** | **约 17 KB** |

还剩约 3.5 KB 余量。如果链接报 `L6406E: No space in execution regions`：

1. 先把 `configTOTAL_HEAP_SIZE` 降到 **11264**；
2. 再把 `receiveTask` 栈降到 **384 字**；
3. 串口电机版还多了 `ZdtMotorUart.c` 的两个静态缓冲（`s_MultiBuf[256]` + `tx[261]`，共 517 B），不够时可把 `ZdtUart_MultiSend` 的 `tx` 改成复写 `s_MultiBuf` 前方，省 261 B。

> 注意：`configTOTAL_HEAP_SIZE` 必须能容纳所有动态任务栈 + TCB + 队列。如果 heap 太小，`osThreadCreate()` 返回 `NULL`，表现为某个任务完全不运行，或者 `osKernelStart()` 后 HardFault。调试时打开 `configUSE_MALLOC_FAILED_HOOK` 并在 `vApplicationMallocFailedHook()` 里断点。

### 6.8 生成结果核对

生成后的 `Core/Src/freertos.c` 应该包含（顺序可能不同）：

```c
osThreadDef(receiveTask, ReceiveTask, osPriorityAboveNormal, 0, 512);
receiveTaskHandle = osThreadCreate(osThread(receiveTask), NULL);

osThreadDef(imuTask, ImuTask, osPriorityHigh, 0, 384);
imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);

osThreadDef(chassisTask, ChassisTask, osPriorityNormal, 0, 256);
chassisTaskHandle = osThreadCreate(osThread(chassisTask), NULL);

osThreadDef(gimbalTask, GimbalTask, osPriorityNormal, 0, 256);
gimbalTaskHandle = osThreadCreate(osThread(gimbalTask), NULL);
```

（没有 `debugTask`——已从 CubeMX 和 Keil 工程中删除；CAN 模式下 USART1 空闲，以后可再添加。）

并且每个入口函数是 `__weak` 的：

```c
__weak void ReceiveTask(void const * argument)
{
  for(;;) { osDelay(1); }
}
```

`Task/ReceiveTask.c` 等文件里的强符号会覆盖它。**不要复制 F407 的 `Core/Src/freertos.c`**，否则任务栈又变回 1024/512，且可能和 CubeMX 生成的静态 Idle 回调冲突。

再打开 `Core/Inc/FreeRTOSConfig.h` 抽查这几行（值必须一致）：

| 宏 | 期望值 | 对应 CubeMX 位置 |
|---|---|---|
| `configUSE_PREEMPTION` | `1` | Config parameters → Kernel settings |
| `configTICK_RATE_HZ` | `1000` | Config parameters → Kernel settings |
| `configMAX_PRIORITIES` | `( 7 )` | Config parameters → Kernel settings |
| `configMINIMAL_STACK_SIZE` | `128` | Config parameters → Kernel settings |
| `configTOTAL_HEAP_SIZE` | `((size_t)12288)` | Config parameters → Memory management settings |
| `configUSE_TIMERS` | 不出现或 `0` | Config parameters → Software timer definitions |
| `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` | `5` | Config parameters → Interrupt nesting behaviour |
| `configCHECK_FOR_STACK_OVERFLOW` | `2` | Config parameters → Hook function related definitions |
| `INCLUDE_vTaskDelayUntil` | `1` | Include parameters |

另外确认 CubeMX 把 `Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c` 加进了工程（Keil 的 `Middlewares` 组里能看到），且**没有**同时加入 `heap_1.c`～`heap_5.c` 中的第二个。

### 6.9 常见误配 → 现象对照

| 误配位置 | 现象 | 处理 |
|---|---|---|
| `Interface` 选了 `CMSIS_V2` | 编译报 `osThreadDef` / `osThread` 未定义，或 `cmsis_os.h` 里找不到 v1 API | 改回 `CMSIS_V1`，重填任务 |
| `Entry Function` 与 `Task/*.c` 函数名不一致 | 编译链接都通过，但该任务**从不执行**（跑的是 `__weak` 空壳里的 `osDelay(1)`） | 逐字核对 5 个函数名 |
| `Code Generation Option` 选 `Default` | `L6200E: Symbol XxxTask multiply defined` | 改为 `As weak` |
| `Stack Size` 按字节填（如填 2048 想给 2 KB） | 实际给了 8 KB 栈，`osThreadCreate` 返回 `NULL` 或 `vApplicationMallocFailedHook` 触发，任务全不跑 | 按**字**填，512 = 2 KB |
| `TOTAL_HEAP_SIZE` 太小 | 同上：部分任务创建失败；`osKernelStart()` 后 HardFault | 按 6.7 的预算调，不少于 11264 |
| `INCLUDE_vTaskDelayUntil` 没勾 | 链接报 `Undefined symbol xTaskDelayUntil` | 勾上 |
| `LIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` 与 NVIC 分组不匹配 | 中断里调 FreeRTOS API 时进 `configASSERT` 死循环或 HardFault | 保持 5，NVIC 分组保持 4 bits |
| `USE_TIMERS` 误开 | 堆里多出 ~1.1 KB（Timer 任务栈 + 队列），heap 紧张 | 关掉 |

---

## 7. 生成工程

1. 点击右上角 **GENERATE CODE**；
2. 生成完成后点击 `Open Project`，Keil 会自动打开 `MDK-ARM/UniversalCar_F103.uvprojx`；
3. 先不要急着编译，按第 8 节复制应用层代码并改 UART 句柄。

---

## 8. 应用层代码迁移

### 8.1 复制清单

从原 F407 工程复制到新工程：

| 源路径 | 目标 | 说明 |
|---|---|---|
| `ExHardware/ZdtMotor.c/.h` | 新工程 `ExHardware/` | **CAN 模式**；与 UART 版二选一 |
| `ExHardware/ZdtMotorUart.c/.h` | 新工程 `ExHardware/` | **串口模式**；与 CAN 版二选一 |
| `ExHardware/PiComm.c/.h` | 新工程 `ExHardware/` | 树莓派协议 |
| `ExHardware/JY901S.c/.h` | 新工程 `ExHardware/` | 陀螺仪 |
| `ExHardware/Servo.c/.h` | 新工程 `ExHardware/` | 舵机 |
| `ExHardware/Led.c/.h` | 新工程 `ExHardware/` | 补光灯 |
| `ExHardware/BleControl.c/.h` | 新工程 `ExHardware/` | 蓝牙遥控，`BLE_CONTROL_ENABLE=1` 时参与编译（树莓派恢复后改回 0） |
| `Task/*.c/.h` | 新工程 `Task/` | 4 个任务；`DebugTask.c` 已从工程移除（文件可留可删） |
| `UserLibrary/*.c/.h` | 新工程 `UserLibrary/` | PID/工具库 |
| `Core/Inc/RobotConfig.h` | 新工程 `Core/Inc/` | 整车配置宏 |
| `Core/Inc/header.h` | 新工程 `Core/Inc/` | 总头文件，需改 HAL 头 |

**不要复制**这些 F407 生成文件：

- `Core/Src/main.c`、`gpio.c`、`usart.c`、`can.c`、`tim.c`、`dma.c`、`freertos.c`、`stm32f4xx_it.c`、`stm32f4xx_hal_msp.c`、`stm32f4xx_hal_timebase_tim.c`、`system_stm32f4xx.c`
- `Core/Inc/stm32f4xx_hal_conf.h`、`FreeRTOSConfig.h`、`stm32f4xx_it.h`
- `Drivers/`、`Middlewares/`（新工程由 CubeMX 生成 F1 版本）

### 8.2 必改项（逐文件）

| 文件 | 修改 | 原因 |
|---|---|---|
| `Core/Inc/header.h` | `#include "stm32f4xx_hal.h"` → `#include "stm32f1xx_hal.h"` | F1 HAL |
| `Core/Inc/header.h` | 如不用 DMA，删除 `#include "dma.h"` | CubeMX 可能不生成 `dma.h` |
| `Core/Inc/header.h` | 如删除 DebugTask，删除 `#include "DebugTask.h"` | 避免找不到头文件 |
| `Core/Inc/header.h` | 串口模式把 `#include "ZdtMotor.h"` 改成 `#include "ZdtMotorUart.h"` | 两套头文件定义同名类型/宏，不能同时包含 |
| `Task/ChassisTask.h`、`Task/GimbalTask.h` | 串口模式把 `#include "ZdtMotor.h"` 改成 `#include "ZdtMotorUart.h"` | 这两个头文件也直接包含了 `ZdtMotor.h` |
| `ExHardware/PiComm.c` | `huart6` → `huart2` | 树莓派改到 USART2 |
| `ExHardware/PiComm.h` | 注释里的 `USART6` → `USART2` | 文档同步 |
| `ExHardware/ZdtMotorUart.c` | `huart5` → `huart1`（2 处：`ZdtUart_SendFrame`、`ZdtUart_MultiSend`） | 串口电机改到 USART1 |
| `ExHardware/ZdtMotorUart.h` | 注释里的 `UART5` → `USART1` | 文档同步 |
| `Core/Inc/RobotConfig.h` | `LED_PWM_CH` 注释 `PA5` → `PA0`；`PI_CMD_TIMEOUT_MS` 注释 `USART6` → `USART2`；JY901S/BLE 注释确认仍为 `USART3`/`USART2` | 文档同步 |
| `Core/Inc/RobotConfig.h` | 添加 `#define MOTOR_USE_UART 0`（CAN 模式）或 `1`（串口模式） | 模式切换 |
| `Core/Src/usart.c` | 添加 `uart1_rx_byte/uart2_rx_byte/uart3_rx_byte`，重写 `HAL_UART_RxCpltCallback` | 串口映射 |
| `Core/Src/main.c` | `USER CODE BEGIN 2` 调 `ZdtCan_Init()` 或 `ZdtUart_Init()` | 电机初始化 |
| `Core/Src/can.c` | 串口模式删除/包住 `ZdtCan_RxCallback` | 避免链接错误 |
| `Task/ChassisTask.c`、`Task/GimbalTask.c` | **不用改**：`ZdtMotorUart.h` 提供 `ZdtCan_*` → `ZdtUart_*` 兼容别名 | 任务层两种模式共用 |
| `Task/DebugTask.c` | 串口模式从工程移除，或改成 SWO | USART1 被电机占用 |
| `MDK-ARM/*.uvprojx` | 器件、宏、头文件路径、去掉 FPU/ARM_CM4F | 见第 9 节 |

### 8.3 USART 句柄与回调

先在 `Core/Inc/RobotConfig.h` 里加一个模式宏（放在“电机”一节即可）：

```c
#define MOTOR_USE_UART   0   /* 0 = CAN 模式；1 = USART1 串口电机模式 */
```

CubeMX 生成的 `Core/Src/usart.c` 顶部只有 `huart1/2/3`。在 `USER CODE BEGIN 0` 里先加 `#include "header.h"`（生成文件默认没有），再添加接收字节变量：

```c
/* USER CODE BEGIN 0 */
uint8_t uart1_rx_byte;   /* USART1 — 串口电机（CAN 模式下可做调试） */
uint8_t uart2_rx_byte;   /* USART2 — 树莓派 */
uint8_t uart3_rx_byte;   /* USART3 — JY901S */
/* USER CODE END 0 */
```

在三个 `MX_USARTx_UART_Init()` 的 `USER CODE BEGIN ..._Init 2` 中分别启动接收（CAN 模式下 USART1 也可以启动，不影响调试发送）：

```c
HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
```

在 `USER CODE BEGIN 1` 重写回调：

```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        PiComm_RxByte(uart2_rx_byte);
        HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
    }
    else if (huart->Instance == USART3) {
        JY901S_RxByte(uart3_rx_byte);
        HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
    }
    else if (huart->Instance == USART1) {
#if MOTOR_USE_UART
        ZdtUart_RxByte(uart1_rx_byte);
#endif
        HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
    }
}
```

> 这里用了一个建议的宏 `MOTOR_USE_UART`（在 `RobotConfig.h` 里定义，0=CAN，1=串口），方便两种模式切换。如果不想加宏，就直接按最终选择保留/删除 USART1 分支。
>
> **蓝牙与树莓派只能二选一**：C8T6 只有三个串口，陀螺仪占 USART3、电机（串口模式）占 USART1，树莓派和蓝牙都要 USART2。用 `BLE_CONTROL_ENABLE` 切换：`1`=蓝牙接管 USART2（PiComm 停止解析与回传），`0`=树莓派。蓝牙模块直接插原来的树莓派 4 针口即可，不用改线；也可以把 `BLE_UART_PORT` 改成 `1` 接 USART1（此时 `MOTOR_USE_UART` 必须为 0）。

### 8.4 main.c 的初始化顺序

先在 `main.c` 的 `USER CODE BEGIN Includes` 里加 `#include "header.h"`；如果保留 CAN 模式的 ZDT 回调，在 `can.c` 的 `USER CODE BEGIN 0` 里也加 `#include "header.h"`。

CubeMX 生成的 `main()` 里，外设初始化顺序应类似：

```c
MX_GPIO_Init();
MX_DMA_Init();          /* 只有使用 DMA 时才生成 */
MX_CAN1_Init();
MX_USART1_UART_Init();
MX_USART2_UART_Init();
MX_USART3_UART_Init();
MX_TIM2_Init();
MX_TIM3_Init();
/* USER CODE BEGIN 2 */
#if MOTOR_USE_UART
ZdtUart_Init();          /* 只复位串口解析状态机，USART1 已由 CubeMX 初始化 */
#else
ZdtCan_Init();           /* 配置过滤器 + 启动 CAN + 使能 RX FIFO0 中断 */
#endif
/* USER CODE END 2 */

MX_FREERTOS_Init();
osKernelStart();
```

> `MX_TIM2_Init()` / `MX_TIM3_Init()` 必须在 FreeRTOS 启动前完成；`Led_Init()` / `Servo_Init()` 在任务初始化里调用，保持原样即可。

### 8.5 CAN 模式（方案 A，推荐先做）

1. `ExHardware/ZdtMotor.c` 和 `ZdtMotorUart.c` **两个都加进 Keil 工程**：前者整体被 `#if !MOTOR_USE_UART` 包住，后者被 `#if MOTOR_USE_UART` 包住，只有选中的那份参与编译；
2. `main.c` 用 `#if MOTOR_USE_UART` 自动调 `ZdtUart_Init()` 或 `ZdtCan_Init()`；
3. `can.c` 的 `HAL_CAN_RxFifo0MsgPendingCallback()` → `ZdtCan_RxCallback()` 被 `#if !MOTOR_USE_UART` 包住；
4. `Task/ChassisTask.c` / `GimbalTask.c` 保持 `ZdtCan_*` 调用不变（串口模式下由头文件别名转接）；
5. USART1 空闲，可做调试口；若要恢复 `DebugTask`，需在 CubeMX 里重新建任务并把它加回 Keil；
6. 在 `RobotConfig.h` 里把 `MOTOR_USE_UART` 设为 **0**。

### 8.6 串口电机模式（方案 B）

1. 两个驱动文件都留在 Keil 工程里（各自带 `#if` 守卫），只需把 `MOTOR_USE_UART` 改成 **1**；
2. `Core/Inc/header.h`、`Task/ChassisTask.h`、`Task/GimbalTask.h` 已经用 `#if MOTOR_USE_UART` 自动选择 `ZdtMotorUart.h` / `ZdtMotor.h`，不用手改；
3. `main.c` 里已用 `#if MOTOR_USE_UART` 自动调 `ZdtUart_Init()`；
4. `can.c` 删除 `HAL_CAN_RxFifo0MsgPendingCallback()` 里对 `ZdtCan_RxCallback()` 的调用（或用 `#if !MOTOR_USE_UART` 包住）。CAN 外设可以继续初始化，留给以后接其他 CAN 设备；如果暂时不接任何 CAN 设备，也可以在 CubeMX 里取消 `CAN1_RX0` 中断，避免空中断；
5. `usart.c` 的 USART1 分支调用 `ZdtUart_RxByte()`；
6. `Task/ChassisTask.c` / `GimbalTask.c` **不用改**：`ZdtMotorUart.h` 末尾提供了 `ZdtCan_*` → `ZdtUart_*` 的兼容别名，任务层照旧调用即可；
7. 串口版补了 `ZdtUart_SetTimedReturn(addr, 0x36, ms)`（命令 0x11），与 CAN 版语义一致，底盘 4 轮由电机定时主动上报位置。`ZdtMotorUart.c` 的解析器同时写入 `Position.RawCount`（整数计数）和 `Position.Real`（°），里程计精度不受影响。
8. `Task/DebugTask.c` 从 Keil 工程移除（USART1 已被电机占用），或改成 SWO；
9. 在 `RobotConfig.h` 里把 `MOTOR_USE_UART` 设为 **1**。

#### 串口模式的波特率/周期约束（很重要）

以 115200、8N1（每字节 10 bit）计算：

- 4 轮速度命令：4 × (地址 1 + 命令 6 + 校验 1) + 5 字节包头 = **37 字节** ≈ **3.2 ms**；
- 4 轮位置反馈：4 × 8 字节 = **32 字节** ≈ **2.8 ms**；
- 如果反馈靠轮询 `ZdtUart_ReadPos()`，还要加 4 × 3 = **12 字节**请求；
- 合计：电机主动上报时 **69 字节 ≈ 6.0 ms**；轮询时 **81 字节 ≈ 7.0 ms**。

因此 **5 ms 周期 + 115200 在串口模式下是跑不过来的**。建议：

| 方案 | 命令周期 | 反馈周期 | 总线占用（主动上报 / 轮询） |
|---|---|---|---|
| 115200 | 5 ms | **20 ms** | 约 78% / 84% |
| 230400 | 5 ms | 10 ms | 约 65% / 70% |
| 460800 | 5 ms | 5 ms | 约 35% / 40% |

- **切到串口模式（`MOTOR_USE_UART=1`）时采用第一行**：`ZDT_FEEDBACK_PERIOD_MS` 自动取 **20**，底盘任务仍保持 5 ms 控制周期（CAN 模式下自动用 5）；
- 或者把电机串口波特率改成 **230400/460800**（电机侧也要改），再把 `ZDT_FEEDBACK_PERIOD_MS` 设回 5/10；
- `HAL_UART_Transmit()` 是阻塞的：37 字节 @115200 会阻塞 CPU 约 3.2 ms，直接吃掉 5 ms 周期的一大半。串口模式下强烈建议改 DMA 发送，或至少把波特率提到 230400 以上。

### 8.7 调试任务的处理

`Task/DebugTask.c` 当前每 10 ms 调用：

```c
HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&DebugData, sizeof(DebugData));
```

- **CAN 模式**：保留，USART1 + DMA1 Channel 4；
- **串口模式**：USART1 被电机占用，必须从 Keil 工程移除 `Task/DebugTask.c`（或在文件开头 `#if 0`）；
- 如果只是想看几个变量，优先用 Keil 的 Watch 窗口，不需要占用串口；
- 想保留 printf 调试，可用 PB3 的 SWO/ITM，在 Keil 的 `Trace` 设置里开启 `ITM Stimulus Port 0`，并重定向 `ITM_SendChar`。

---

## 9. Keil 工程设置与编译

### 9.1 器件与目标

打开 `Options for Target` → `Device`：

- Device：**STM32F103C8**
- 确认 `Target` 页：
  - Xtal：`8.0` MHz
  - ROM：`0x8000000`，大小 `0x10000`（64 KB）
  - RAM：`0x20000000`，大小 `0x5000`（20 KB）

### 9.2 C/C++ 宏与头文件路径

`Options for Target` → `C/C++`：

**Define**：

```text
USE_HAL_DRIVER,STM32F103xB
```

> C8T6 属于 medium-density，CubeMX 生成的是 `STM32F103xB`，不要写成 `STM32F103xE`。

**Include Paths**（顺序不重要，但必须齐全）：

```text
../Core/Inc
../Drivers/STM32F1xx_HAL_Driver/Inc
../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
../Middlewares/Third_Party/FreeRTOS/Source/include
../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
../Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM3
../Drivers/CMSIS/Device/ST/STM32F1xx/Include
../Drivers/CMSIS/Include
../ExHardware
../Task
../UserLibrary
```

**必须删除** F4 相关路径：

- `../Drivers/STM32F4xx_HAL_Driver/...`
- `../Drivers/CMSIS/Device/ST/STM32F4xx/Include`
- `../Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F`

**其他**：

- `C99 Mode`：勾选
- `Use MicroLIB`：不勾（与原工程一致；勾了也能跑，但 printf/浮点行为会变）
- `Optimization`：`-O2` 或 `-O3`（原工程是 `-O3` 级别）
- `One ELF Section per Function`：可勾选，方便裁剪

### 9.3 目标选项（Target）

- `Use FPU`：**不勾**（Cortex-M3 没有 FPU）
- `Floating Point Hardware`：`Not Used`
- `Use Cross-Module Optimization`：可选

### 9.4 源文件分组

建议在 Keil 工程里建这些 Group：

```text
Application/User
  Core/Src/main.c
  Core/Src/gpio.c
  Core/Src/usart.c
  Core/Src/can.c
  Core/Src/tim.c
  Core/Src/dma.c          (如果生成)
  Core/Src/freertos.c
  Core/Src/stm32f1xx_it.c
  Core/Src/stm32f1xx_hal_msp.c
  Core/Src/stm32f1xx_hal_timebase_tim.c
  Core/Src/system_stm32f1xx.c

Drivers/STM32F1xx_HAL_Driver
  (CubeMX 自动加入的 HAL .c 文件)

Middlewares/FreeRTOS
  (CubeMX 自动加入的 FreeRTOS .c 文件 + ARM_CM3/port.c)

ExHardware
  ExHardware/ZdtMotor.c        (或 ZdtMotorUart.c，二选一)
  ExHardware/PiComm.c
  ExHardware/JY901S.c
  ExHardware/Servo.c
  ExHardware/Led.c

Task
  Task/ReceiveTask.c
  Task/ImuTask.c
  Task/ChassisTask.c
  Task/GimbalTask.c
  Task/DebugTask.c             (串口模式移除)

UserLibrary
  UserLibrary/*.c
```

### 9.5 下载设置

`Options for Target` → `Debug`：

- Use：**ST-Link Debugger**（或你实际使用的调试器）
- Settings → Port：**SW**
- Max Clock：`4 MHz` 起步，稳定后可提高
- Flash Download：勾选 `Reset and Run`
- Flash 算法：`STM32F10x Med-density Flash`，ROM 大小 `0x10000`

---

## 10. 上电验证清单

按顺序做，每一步都确认后再进行下一步。

### 10.1 时钟

1. 下载后连接调试器，全速运行；
2. 在 Watch 窗口看 `SystemCoreClock`，应为 **72,000,000**；
3. 看 `HAL_RCC_GetPCLK1Freq()` 返回 **36,000,000**，`HAL_RCC_GetPCLK2Freq()` 返回 **72,000,000**；
4. 若程序停在 `Error_Handler()`，优先检查 HSE 晶振/负载电容，或 `HSE_VALUE` 是否与实际晶振一致。

### 10.2 FreeRTOS

1. 在 `osKernelStart()` 后暂停，看 4 个任务是否都创建成功（`receiveTaskHandle` 等不为 `NULL`）；
2. 看 `xPortGetFreeHeapSize()`，应大于 **3000** 字节；
3. 用 `uxTaskGetStackHighWaterMark(handle)` 看每个任务剩余栈，建议 **> 64 字**；
4. 如果某个任务不运行，检查：
   - `Task/*.c` 是否加入 Keil 工程；
   - 函数名是否与 CubeMX 任务入口完全一致；
   - 强符号是否覆盖了 `freertos.c` 里的 `__weak` 空函数；
   - heap 是否足够（`osThreadCreate` 返回 `NULL`）。

### 10.3 串口

| 链路 | 验证方法 | 预期 |
|---|---|---|
| USART2 ↔ Pi | Pi 发心跳/指令，F407 侧看 `PiCommand` | `PiComm_IsOnline()` 为真，500 ms 内不超时 |
| USART3 ↔ JY901S | 看 `JY901S` 角度变量 | 角度随转动变化，200 ms 内不判离线 |
| USART1（CAN 模式） | USB-TTL 接 PA9/PA10 | `DebugTask` 每 10 ms 输出一帧 |
| USART1（串口模式） | 接电机，发速度命令 | 电机转动，`ZdtUart_RxByte` 能收到反馈 |

### 10.4 CAN

1. 用示波器/逻辑分析仪看 PB9（CAN_TX）是否有波形；
2. 确认总线两端 120 Ω、CAN_H/CAN_L 不接反；
3. 用 CAN 分析仪看 500 kbps 是否有扩展帧；
4. 看 `ZdtMotor_IsOnline(&Chassis.Wheel[i])`，200 ms 内应收到反馈；
5. 若完全无 ACK，检查：
   - `__HAL_AFIO_REMAP_CAN1_2()` 是否生成；
   - 电机地址/波特率/通信模式是否配置为 CAN 500K；
   - 收发器供电和待机脚（如有 STB 脚，拉低使能）。

### 10.5 PWM

| 信号 | 引脚 | 预期 |
|---|---|---|
| 补光灯 | PA0 | 3 kHz，占空比随亮度变化 |
| 夹爪舵机 | PA6 | 50 Hz，0.5～2.5 ms |
| 载物盘舵机 | PA7 | 50 Hz，0.5～2.5 ms |
| 摄像头舵机 | PB0 | 50 Hz，0.5～2.5 ms |

- 上电时舵机应回到 `SERVO_*_INIT` 角度，不应抖动；
- 若舵机抖动，优先检查舵机电源是否独立、地是否共、脉宽是否在 500～2500 μs。

### 10.6 补光灯

- `Led_Init()` 后 PA0 输出 3 kHz PWM；
- 发 `PI_CMD_LED` 后亮度变化；
- 若灯不亮，检查 PA0 是否被配置成 TIM2_CH1，以及 `Led_Init()` 是否被调用。

---

## 11. 性能与实时性评估

### 11.1 CPU

C8T6 是 72 MHz Cortex-M3、无 FPU。主要开销：

| 开销 | 估算 | 说明 |
|---|---|---|
| FreeRTOS 1 kHz tick + 调度 | 1～3% | 4 个任务，周期 5/10 ms |
| CAN 发送忙等 | 15～30% | `Zdt_SendFrame()` 在 3 个邮箱全满时最多重试 10000 次 |
| Pi 串口阻塞发送 | 约 7% | 17 字节 @50 Hz，@115200 每次约 1.5 ms |
| 软浮点（sinf/cosf/sqrtf） | 3～10% | 底盘 200 Hz 每周期几次三角函数 |
| 其他（PID、解析、协议） | 10～20% | 取决于实际工况 |

总体估计 **40%～70%**，5 ms 底盘周期可行；如果同时启用云台三轴电机（`GIMBAL_MOTOR_INSTALLED=1`），会明显更紧，建议先保持 0。

### 11.2 CAN 总线负载（500 kbps）

- 4 轮位置反馈 @5 ms：4 × 200 = 800 帧/s；每帧扩展帧约 130～140 bit → 约 110 kbps → **22%**；
- 4 轮速度命令 @5 ms：6 帧/5 ms = 1200 帧/s → 约 165 kbps → **33%**；
- 合计约 **55%**，总线本身没问题；瓶颈在 MCU 的发送忙等，不在带宽。

### 11.3 串口总线负载（115200）

见 8.6 节：4 轮命令 + 反馈共 69 字节/周期，115200 下 5 ms 周期跑不过来。建议 10 ms 周期或 230400+ 波特率。

### 11.4 优化建议（按优先级）

1. **CAN 发送忙等**：`Zdt_SendFrame()` 的 `while (ret != HAL_OK && retry < 10000)` 在 72 MHz 上很耗 CPU。可以：
   - 发送前用 `HAL_CAN_GetTxMailboxesFreeLevel()` 判断，邮箱不足时直接丢弃本周期帧（下一周期再发）；
   - 或把 `retry` 降到 1000，并在循环里加 `__NOP()`/`osThreadYield()`（仅在任务上下文）；
   - 或把 `ZDT_FEEDBACK_PERIOD_MS` 从 5 改成 10，减少总线占用。
2. **PiComm 改 DMA**：把 `HAL_UART_Transmit(&huart2, ...)` 改成 `HAL_UART_Transmit_DMA()`，需要处理发送完成回调和缓冲区生命周期（不能传栈上局部数组）。另外，这个阻塞发送发生在 `receiveTask`（AboveNormal）里，会推迟 `chassisTask`（Normal）；如果底盘 5 ms 周期出现抖动，优先处理这里。
3. **串口电机改 DMA**：37 字节的 `HAL_UART_Transmit` 阻塞 3.2 ms，串口模式下必须优化。
4. **减少浮点**：`ChassisTask` 里的 `cosf/sinf` 可以用查表或只在 yaw 变化时重算；`sqrtf` 只在回航判断里用，影响不大。
5. **编译优化**：`-O2`/`-O3` + `One ELF Section per Function`；确认没有把 `-O0` 带进来。
6. **测量 CPU 余量**：在 idle hook 里翻转一个 GPIO，用示波器看高电平比例；或读 `ulTaskGetIdleRunTimeCounter()`（需要开启 `configGENERATE_RUN_TIME_STATS`，会增加开销）。

---

## 12. 常见问题排查

| 现象 | 可能原因 | 处理 |
|---|---|---|
| 编译报 `stm32f4xx_hal.h: No such file` | `header.h` 没改 | 改成 `stm32f1xx_hal.h` |
| 报 `GPIO_SPEED_FREQ_VERY_HIGH` 未定义 | F1 只有 LOW/MEDIUM/HIGH | 改成 `GPIO_SPEED_FREQ_HIGH` |
| 报 `GPIO_AF9_CAN1` 未定义 | F1 没有 `Alternate` 字段 | 删除该行，CAN 重映射用 `__HAL_AFIO_REMAP_CAN1_2()` |
| 报 `PLLM/PLLN/PLLP` 未定义 | F1 只有 `PLLMUL` | 用 CubeMX 重新生成时钟代码，不要手抄 F4 |
| 报 `FLASH_LATENCY_5` 未定义 | F1 最高 2 WS（72 MHz） | 改成 `FLASH_LATENCY_2` |
| 报 `USART6_IRQn` / `UART4` / `UART5` 未定义 | C8T6 没有这些外设 | 删除对应初始化和中断 |
| 报 `TIM7_IRQn` 未定义 | C8T6 没有 TIM6/TIM7 | Timebase 改 TIM4 |
| 报 `L6200E: Symbol ZdtMotor_* multiply defined` | 同时编译了 `ZdtMotor.c` 和 `ZdtMotorUart.c` | 二选一 |
| 报 `ZdtCan_RxCallback` 未定义 | 串口模式下没编译 `ZdtMotor.c` | 删除/包住 `can.c` 里的回调 |
| 报 `vApplicationGetIdleTaskMemory` 重复定义 | 复制了旧 `freertos.c` | 用 CubeMX 生成的，不要复制旧文件 |
| `osThreadCreate` 返回 `NULL` / 某任务不跑 | heap 太小 / 栈太大 | 降栈、增 heap，或开 malloc failed hook |
| `osKernelStart()` 后 HardFault | 栈溢出 / heap 不够 / 中断优先级错误 | 开 stack overflow 检查；确认 UART/CAN 优先级 ≥5 |
| 程序停在 `configASSERT` 的 `for(;;)` | 中断优先级错误 / 参数非法 / heap 不足 | 看调用栈；确认 UART/CAN 优先级 ≥5，任务栈和 heap 够用 |
| CAN 发不出去 | 没重映射 / 收发器 / 终端电阻 / 波特率 | 查 `__HAL_AFIO_REMAP_CAN1_2()`、120 Ω、电机 500K |
| CAN 一直进错误中断 | 总线上没有第二个节点 ACK | 至少接一个电机或 CAN 分析仪 |
| 串口收不到数据 | 没重新 `HAL_UART_Receive_IT` / TX/RX 接反 / 波特率 | 在回调里重新武装接收；核对交叉连接 |
| 舵机不动/抖动 | 电源不足 / 地不共 / 脉宽不对 | 独立 5～6 V 供电，共地，500～2500 μs |
| 补光灯不亮 | PA0 没配成 TIM2_CH1 / `Led_Init` 没调用 | 查 CubeMX 引脚和 `Led_Init()` |
| Pi 一直离线 | 波特率/协议/CRC 不对，或 Pi 没发心跳 | 用逻辑分析仪看 USART2 RX；确认 `PI_CMD_TIMEOUT_MS=500` |
| 电机一直离线 | 反馈周期/地址/`ZdtMotorList` 注册 | 确认 `ZdtMotor_Init()` 被调用，地址 1～4 |

---

## 附录 A：CubeMX 参数速查表

### A.1 时钟

| 项目 | 值 |
|---|---|
| HSE | 8 MHz Crystal |
| PLLMul | ×9 |
| SYSCLK | 72 MHz |
| AHB | /1 |
| APB1 | /2（36 MHz） |
| APB2 | /1（72 MHz） |
| ADC | /6（12 MHz） |
| USB | /1.5（48 MHz） |
| Flash latency | 2 WS |
| Timebase | TIM4 |

### A.2 外设

| 外设 | 引脚 | 关键参数 |
|---|---|---|
| CAN1 | PB8 RX / PB9 TX | PSC 6，BS1 9TQ，BS2 2TQ，SJW 1TQ，500 kbps，ABOM on，AutoRetrans off |
| USART1 | PA9 TX / PA10 RX | 115200，8N1，RX IT（串口电机）或 TX DMA1_CH4（调试） |
| USART2 | PA2 TX / PA3 RX | 115200，8N1，RX IT；可选 TX DMA1_CH7 |
| USART3 | PB10 TX / PB11 RX | 115200，8N1，RX IT |
| TIM2 CH1 | PA0 | PSC 99，ARR 239，PWM1，3 kHz |
| TIM3 CH1/2/3 | PA6 / PA7 / PB0 | PSC 71，ARR 19999，PWM1，50 Hz，Pulse 500 |
| DMA1_CH4 | USART1_TX | Normal，Byte，Low |
| DMA1_CH7 | USART2_TX | Normal，Byte，Low |

### A.3 FreeRTOS

| 参数 | 值 |
|---|---|
| Interface | CMSIS_V1 |
| Tick Rate | 1000 Hz |
| Max Priorities | 7 |
| Minimal Stack | 128 字 |
| Total Heap | 12288 B |
| Max Task Name | 16 |
| Use Mutexes | Enabled |
| Queue Registry | 8 |
| Port Optimized Selection | Enabled |
| Static Allocation | Enabled |
| Timers | Disabled |

### A.4 任务

| Task | Entry | Priority | Stack（字） |
|---|---|---|---|
| receiveTask | ReceiveTask | AboveNormal | 512 |
| imuTask | ImuTask | High | 384 |
| chassisTask | ChassisTask | Normal | 256 |
| gimbalTask | GimbalTask | Normal | 256 |
| debugTask | DebugTask | Low | 128 |

### A.5 生成后 `.ioc` 关键项核对

生成后可以用文本编辑器打开 `.ioc`，确认以下键值（不同 CubeMX 版本格式可能略有差异）：

```text
Mcu.UserName=STM32F103C8Tx
ProjectManager.DeviceId=STM32F103C8Tx
RCC.PLLMUL=RCC_PLL_MUL9
RCC.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK
RCC.APB1CLKDivider=RCC_HCLK_DIV2
RCC.APB2CLKDivider=RCC_HCLK_DIV1
NVIC.TimeBaseIP=TIM4
CAN1.Prescaler=6
CAN1.BS1=CAN_BS1_9TQ
CAN1.BS2=CAN_BS2_2TQ
CAN1.CalculateBaudRate=500000
FREERTOS.configTOTAL_HEAP_SIZE=12288
```

---

## 附录 B：关键生成代码核对

### B.1 `SystemClock_Config()`

```c
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}
```

### B.2 HAL 时基（TIM4）

`Core/Src/stm32f1xx_hal_timebase_tim.c` 里应看到 `htim4.Instance = TIM4;`、`__HAL_RCC_TIM4_CLK_ENABLE();`、`HAL_NVIC_EnableIRQ(TIM4_IRQn);`。

`Core/Src/main.c` 末尾：

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
}
```

### B.3 CAN MspInit（含重映射）

```c
void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    __HAL_AFIO_REMAP_CAN1_2();   /* PB8/PB9 */

    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  }
}
```

> 不同 CubeMX 版本可能把 PB8 也配成 `GPIO_MODE_AF_PP`，功能上也能工作；关键是 **AFIO 时钟 + `__HAL_AFIO_REMAP_CAN1_2()`** 必须在 CAN 启动前执行。

### B.4 TIM2 / TIM3

```c
/* TIM2: 3 kHz */
htim2.Init.Prescaler = 99;
htim2.Init.Period = 239;

/* TIM3: 50 Hz */
htim3.Init.Prescaler = 71;
htim3.Init.Period = 19999;
sConfigOC.Pulse = 500;
```

### B.5 FreeRTOS 任务

```c
osThreadDef(receiveTask, ReceiveTask, osPriorityAboveNormal, 0, 512);
osThreadDef(imuTask,     ImuTask,     osPriorityHigh,        0, 384);
osThreadDef(chassisTask, ChassisTask, osPriorityNormal,      0, 256);
osThreadDef(gimbalTask,  GimbalTask,  osPriorityNormal,      0, 256);
osThreadDef(debugTask,   DebugTask,   osPriorityLow,         0, 128);
```

---

## 附录 C：CAN 与串口电机 API 对照

`ZdtMotor.c` 和 `ZdtMotorUart.c` 的**电机级函数同名**（`ZdtMotor_Init`、`ZdtMotor_SetSpeed`、`ZdtMotor_MultiAddSpeed`、`ZdtMotor_SetPosition`、`ZdtMotor_IsOnline`），所以任务里只改总线级前缀即可。

| CAN 版 | 串口版 | 说明 |
|---|---|---|
| `ZdtCan_Init()` | `ZdtUart_Init()` | 初始化 |
| `ZdtCan_Enable(addr, en)` | `ZdtUart_Enable(addr, en)` | 使能/松轴 |
| `ZdtCan_Stop(addr)` | `ZdtUart_Stop(addr)` | 立即停止 |
| `ZdtCan_SetZero(addr)` | `ZdtUart_SetZero(addr)` | 位置清零 |
| `ZdtCan_Home(addr, mode)` | `ZdtUart_Home(addr, mode)` | 回零 |
| `ZdtCan_SyncAll()` | `ZdtUart_SyncAll()` | 广播同步 |
| `ZdtCan_ReadPos(addr)` | `ZdtUart_ReadPos(addr)` | 读位置 |
| `ZdtCan_ReadStatus(addr)` | `ZdtUart_ReadStatus(addr)` | 读状态 |
| `ZdtCan_MultiBegin()` | `ZdtUart_MultiBegin()` | 多机命令开始 |
| `ZdtCan_MultiAdd(...)` | `ZdtUart_MultiAdd(...)` | 追加子命令 |
| `ZdtCan_MultiSend()` | `ZdtUart_MultiSend()` | 广播发送 |
| `ZdtCan_SetTimedReturn(addr, func, ms)` | `ZdtUart_SetTimedReturn(addr, func, ms)` | 命令 0x11，串口版已补齐 |

任务里的实际替换位置：

- `Task/ChassisTask.c`：`ZdtCan_SetTimedReturn`、`ZdtCan_Enable`、`ZdtCan_MultiBegin`、`ZdtCan_MultiSend`
- `Task/GimbalTask.c`：`ZdtCan_Enable`、`ZdtMotor_SetPosition`（后者同名，不用改）

---

## 附录 D：F407 → F103 差异清单

| 项目 | STM32F407VET6 | STM32F103C8T6 | 影响 |
|---|---|---|---|
| 内核 | Cortex-M4F @168 MHz | Cortex-M3 @72 MHz | 无 FPU；CPU 余量变小 |
| Flash | 512 KB | 64 KB | 当前 33 KB 代码可放下 |
| SRAM | 128 KB + 64 KB CCM | 20 KB | 必须压缩 FreeRTOS 堆/栈 |
| 串口 | USART1/2/3/6 + UART4/5 | USART1/2/3 | 树莓派/陀螺仪/电机重新分配 |
| 定时器 | TIM1-14（含 TIM6/7） | TIM1-4 | HAL 时基改用 TIM4 |
| CAN | CAN1/CAN2（28 个过滤器） | CAN1（14 个过滤器） | 过滤器组 0 仍可用 |
| DMA | DMA1/DMA2（流） | DMA1（通道） | USART1_TX 用 DMA1_CH4，USART2_TX 用 DMA1_CH7 |
| GPIO 复用 | `GPIO_Alternate` 字段 | 无，靠 AFIO 重映射 | CAN 必须 `__HAL_AFIO_REMAP_CAN1_2()` |
| 电压调节 | PWR 电压等级 | 无 | 删除 `__HAL_PWR_VOLTAGESCALING_CONFIG` |
| Flash 等待 | 5 WS @168 MHz | 2 WS @72 MHz | `FLASH_LATENCY_2` |
| PLL | PLLM/N/P/Q | PLLMUL | 时钟代码完全不同 |
| FreeRTOS 端口 | ARM_CM4F | ARM_CM3 | 去掉 FPU 配置，换 port 文件 |
| USB | OTG FS/HS | USB Device | 本项目不用 USB |

---

## 参考

- STM32F103x8/xB 数据手册（DS5319）：C8T6 无 TIM6/TIM7、无 DAC，3 个 USART，CAN1 支持 PB8/PB9 重映射。
- STM32F1 HAL 驱动：`stm32f1xx_hal_can.c`、`stm32f1xx_hal_uart.c`、`stm32f1xx_hal_tim.c`。
- 原工程配置：[UniversalCar.ioc](../UniversalCar.ioc)、[Core/Inc/FreeRTOSConfig.h](../Core/Inc/FreeRTOSConfig.h)、[Core/Src/freertos.c](../Core/Src/freertos.c)。
- ZDT Y42 Emm 固件手册：CAN 扩展帧 ID = `(地址 << 8) | 包序号`；串口帧 = `地址 + 功能码 + 数据 + 0x6B`。

---

> 文档版本：2026-10-23
>
> 适用代码基线：`UniversalCar` 当前 `main` 分支（含使能/失能重构、`BLE_CONTROL_ENABLE=1` 蓝牙遥控、`GIMBAL_MOTOR_INSTALLED=0`）。
