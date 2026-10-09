# STM32F407VET6 整车控制板 — CubeMX 配置指南（FreeRTOS 版）

> 目标：用 CubeMX 生成 Keil（MDK-ARM）工程，配置 **FreeRTOS**（5 任务）、**CAN1**（7 个 ZDT 步进电机）、**USART1**（调试输出）、**USART2**（蓝牙）、**USART3**（JY901S 陀螺仪）、**USART6**（树莓派）、**TIM3**（3 路舵机 PWM）。
> 工程结构参照 RM 标准项目：`Core`（CubeMX 生成）/ `ExHardware`（外设驱动）/ `Task`（FreeRTOS 任务）/ `UserLibrary`（工具库）。
> 更新日期：2026-10-02

---

## 0. 引脚分配总表

| 外设 | 引脚 | 说明 |
|---|---|---|
| CAN1_RX / CAN1_TX | **PB8 / PB9** | 7 个 ZDT 电机总线（并联，500kbps） |
| USART1_TX / RX | **PA9 / PA10** | 调试数据输出（115200，TX 走 DMA） |
| USART2_TX / RX | **PA2 / PA3** | 蓝牙遥控（115200，类比 DBUS） |
| USART3_TX / RX | **PB10 / PB11** | JY901S 陀螺仪（默认 9600，可改 115200） |
| USART6_TX / RX | **PC6 / PC7** | 树莓派（双向，115200） |
| TIM3_CH1 | **PA6** | 夹爪舵机（180°） |
| TIM3_CH2 | **PA7** | 载物盘舵机（180°） |
| TIM3_CH3 | **PB0** | 摄像头舵机（180°） |
| GPIO_Output | **PC13** | 板载 LED |
| SWDIO / SWCLK | **PA13 / PA14** | 烧录调试，必须保留 |

---

## 1. 新建工程

1. `File → New Project`，搜索 `STM32F407VE`，选 **STM32F407VETx**（LQFP100）。
2. 固件包选 **STM32Cube FW_F4**（V1.28.x+）。

## 2. 时钟配置 — 168 MHz

**RCC**：HSE = **Crystal/Ceramic Resonator**（8MHz 晶振）。

**Clock Configuration 页**：

| 参数 | 值 |
|---|---|
| PLL Source | HSE |
| **PLLM** | **4**（8MHz/4 = 2MHz） |
| **PLLN** | **168**（2MHz×168 = 336MHz） |
| **PLLP** | **2**（336/2 = 168MHz） |
| PLLQ | 7（48MHz，给 USB/SDIO 预留） |
| SYSCLK | **168 MHz** |
| AHB | /1 → 168 MHz |
| APB1 | /4 → **42 MHz** |
| APB2 | /2 → **84 MHz** |

> ⚠️ APB1 定时器时钟 = 84MHz（×2），APB2 定时器时钟 = 168MHz（×2）。TIM3 的 PSC 计算依赖这个。

## 3. SYS（调试 + HAL 时基，两个都要设）

| 选项 | 值 | 说明 |
|---|---|---|
| Debug | **Serial Wire** | 不设则烧录一次后连不上 |
| **Timebase Source** | **TIM7** | ⚠️ **FreeRTOS 工程必须改**：SysTick 被 FreeRTOS 占用，HAL 时基改用 TIM7 |

## 4. FreeRTOS（CMSIS_V1，重点）

`Middleware → FREERTOS`，Interface 选 **CMSIS_V1**。

**Config parameters 标签**：

| 参数 | 值 |
|---|---|
| configTOTAL_HEAP_SIZE | **16384** |
| configENABLE_FPU | **Enabled**（F407 有 FPU） |
| INCLUDE_vTaskDelayUntil | **Enabled**（任务周期用 osDelayUntil，必开） |

**Tasks 标签**（加 5 个任务，`Code Generation Option` 都选 **As weak**，Allocation 都选 **Static**）：

| Task Name | Entry Function | Priority (osPriority) | Stack (Words) |
|---|---|---|---|
| receiveTask | ReceiveTask | **AboveNormal** (1) | 256 |
| imuTask | ImuTask | **High** (2) | 512 |
| chassisTask | ChassisTask | **Normal** (0) | 256 |
| gimbalTask | GimbalTask | **Normal** (0) | 256 |
| debugTask | DebugTask | **Low** (-2) | 128 |

> **Task Name 和 Entry Function 用不同名**（CubeMX 会提示同名不行）：
> - **Task Name** 只是线程标签/句柄名（`receiveTask` → `receiveTaskHandle`、`osThreadDef(receiveTask, ...)`），调试器里显示的名字。
> - **Entry Function** 才是真正执行的函数，**必须等于 `Task/` 目录里的函数名**（`ReceiveTask`、`ImuTask`、`ChassisTask`、`GimbalTask`、`DebugTask`），一字不差，否则 `__weak` 壳不会被你的实现覆盖。
> - 这也是 CubeMX 默认任务的写法（`defaultTask` / `StartDefaultTask`）。
>
> `As weak` 是 RM 工程的关键：freertos.c 里生成 `__weak` 任务壳，真正的实现在 `Task/` 目录，之后 CubeMX 重新生成也不会覆盖你的任务代码。

## 5. CAN1（7 个 ZDT 电机，500 kbps）

`Connectivity → CAN1`，勾选 **Activated**，引脚 `CAN1_RX → PB8`、`CAN1_TX → PB9`。

**Parameter Settings**：

| 参数 | 值 | 说明 |
|---|---|---|
| Prescaler | **7** | 42MHz / (7 × 12TQ) = **500 kbps** |
| Time Quanta in Bit Segment 1 | **9** | 位时间 = (1+9+2) TQ = 12 TQ |
| Time Quanta in Bit Segment 2 | **2** | |
| ReSynchronization Jump Width | 1 | |
| Automatic bus-off management | **Enable** | |
| 其余 | 默认 | |

**NVIC Settings**：勾选 **CAN1 RX0 interrupt**。

> ⚠️ 漏勾 = 永远收不到电机反馈。
> ⚠️ F4 双 CAN 注意：`ZdtCan_Init()` 里已写 `SlaveStartFilterBank = 14`，这是 F407 双 CAN 过滤器分段必需的，不加会 `HAL_ERROR`。

## 6. USART6（树莓派，双向）

Mode = Asynchronous，PC6/PC7。参数：**115200** 8N1，Overrun Disable。NVIC 勾选 **USART6 global interrupt**。

> 后续遥测量大可升 460800（APB2=84MHz，460800 误差 0.02%，可用）。

## 7. USART2（蓝牙遥控，类比 DBUS）

Mode = Asynchronous，PA2/PA3。参数：**115200** 8N1。NVIC 勾选 **USART2 global interrupt**。

> 微信小程序端发送 `[joystick,...]` / `[key,...]` / `[slider,...]` 数据包，树莓派不在场时接管整车（详见《工程结构与模块说明》）。

## 8. USART3（JY901S 陀螺仪）

Mode = Asynchronous，PB10/PB11。参数：**9600** 8N1（JY901S 出厂默认；用上位机改成 115200 后此处同步改）。NVIC 勾选 **USART3 global interrupt**。

## 9. USART1（调试输出，DMA）

Mode = Asynchronous，PA9/PA10。参数：**115200** 8N1。

**DMA Settings 标签**：Add → 选 **USART1_TX**：

| 参数 | 值 |
|---|---|
| DMA Request | USART1_TX |
| Stream | DMA2 Stream 7（默认，Channel 4） |
| Direction | Memory To Peripheral |
| Mode | **Normal**（每帧发一次，不用 Circular） |
| 其余 | 默认（Data Width Byte） |

NVIC 勾选 USART1 global interrupt（可不勾，发送不用中断）。

> DebugTask 用 `HAL_UART_Transmit_DMA(&huart1, ...)` 每 10ms 发一帧 DebugData，无阻塞。

## 10. TIM3（3 路舵机 PWM，50Hz）

`Timers → TIM3`，Clock Source = Internal Clock，Channel1/2/3 都选 **PWM Generation CHx**，引脚 PA6/PA7/PB0。

**Parameter Settings**：

| 参数 | 值 | 说明 |
|---|---|---|
| Prescaler (PSC) | **83** | 84MHz / 84 = 1MHz（1 tick = 1µs） |
| Counter Period (ARR) | **19999** | 20000 tick = 20ms = 50Hz |
| Pulse（CH1/CH2/CH3） | **500** | 初始 0.5ms |
| Output compare preload | **Enable** | 改脉宽不跳变 |

> 脉宽 500~2500us ↔ 0~180°，1 tick = 1µs。
> ⚠️ **PSC 千万别填 F103 时代的 71**，F407 TIM3 时钟是 84MHz。

## 11. GPIO

PC13 = GPIO_Output（初始高，板载 LED 低电平亮）。急停按钮如需硬件接入，选空闲 GPIO 配 GPIO_EXTI + 下拉。

## 12. Project Manager

- Toolchain / IDE = **MDK-ARM**，Min Version V5
- Code Generator：Copy only the necessary library files ✅ / Generate peripheral initialization as a pair of '.c/.h' files ✅ / Keep User Code when re-generating ✅

点 **GENERATE CODE**。

---

## 13. 生成后要做的（关键，按顺序）

### 13.1 加入源码文件

把这三个目录整体加进 Keil 工程（新建同名 Group，Add Existing Files）：

```
ExHardware/   ZdtMotor.c/.h  Servo.c/.h  JY901S.c/.h  PiComm.c/.h   （8 个）
Task/         ReceiveTask  ImuTask  ChassisTask  GimbalTask  DebugTask（各 .c/.h，10 个）
UserLibrary/  Universal.c/.h  UniversalPID.c/.h                     （4 个）
```

再把 `Core/Inc/header.h` 和 `Core/Inc/RobotConfig.h` 放进 Core/Inc（它们已在仓库里）。

### 13.2 Keil 工程设置

1. `Options → Target`：芯片 STM32F407VE，Xtal = 8.0，**Use MicroLIB 勾选**。
2. `Options → C/C++ → Include Paths` 加：`ExHardware`、`Task`、`UserLibrary`、`Core/Inc`。
3. `Options → C/C++ → Define` 确认有：`USE_HAL_DRIVER, STM32F407xx`。

### 13.3 can.c — 填入 CAN 接收回调（USER CODE BEGIN 4 段，文件末尾）

```c
/* USER CODE BEGIN 4 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    CAN_RxHeaderTypeDef RxMessage;
    uint8_t Data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMessage, Data) != HAL_OK)
        return;

    /* ZDT 电机扩展帧：ID = (地址<<8)|包序号 */
    ZdtCan_RxCallback(RxMessage.ExtId, Data, RxMessage.DLC);
}

/* USER CODE END 4 */
```

### 13.4 usart.c — 填入接收分发（三处 USER CODE）

**① USER CODE BEGIN 0（文件开头，Includes 后）：**

```c
/* USER CODE BEGIN 0 */
#include "PiComm.h"
#include "JY901S.h"
#include "BleControl.h"

uint8_t uart2_rx_byte;   /* USART2 — 蓝牙遥控 */
uint8_t uart3_rx_byte;   /* USART3 — JY901S */
uint8_t uart6_rx_byte;   /* USART6 — 树莓派 */
/* USER CODE END 0 */
```

**② 每个 MX_USARTx_UART_Init() 末尾的 USER CODE 2 段，启动中断接收：**

```c
  /* USER CODE BEGIN USART6_Init 2 */
  HAL_UART_Receive_IT(&huart6, &uart6_rx_byte, 1);
  /* USER CODE END USART6_Init 2 */
```

```c
  /* USER CODE BEGIN USART2_Init 2 */
  HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
  /* USER CODE END USART2_Init 2 */
```

```c
  /* USER CODE BEGIN USART3_Init 2 */
  HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
  /* USER CODE END USART3_Init 2 */
```

**③ USER CODE BEGIN 1（文件末尾），接收回调分发：**

```c
/* USER CODE BEGIN 1 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6) {
        PiComm_RxByte(uart6_rx_byte);
        HAL_UART_Receive_IT(&huart6, &uart6_rx_byte, 1);
    } else if (huart->Instance == USART2) {
        BLE_RxCallback(uart2_rx_byte);
        HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
    } else if (huart->Instance == USART3) {
        JY901S_RxByte(uart3_rx_byte);
        HAL_UART_Receive_IT(&huart3, &uart3_rx_byte, 1);
    }
}

/* USER CODE END 1 */
```

### 13.5 main.c — 填入集成代码（两处 USER CODE）

**① USER CODE BEGIN Includes：**

```c
/* USER CODE BEGIN Includes */
#include "header.h"
/* USER CODE END Includes */
```

**② USER CODE BEGIN 2（外设初始化后）：**

```c
  /* USER CODE BEGIN 2 */
  ZdtCan_Init();      /* CAN 过滤器 + 启动 + 接收中断 */
  /* USER CODE END 2 */
```

> 其余初始化（电机注册、舵机、串口协议）全部在各任务函数开头做，这是 RM 工程的分工习惯——main 里不堆业务逻辑。

---

## 14. 常见坑

| 现象 | 原因 | 解决 |
|---|---|---|
| HAL_Delay 卡死 | HAL 时基没改 TIM7 | SYS → Timebase Source → TIM7 |
| 舵机角度不对 | TIM3 PSC 填了 71 | 改为 **83** |
| CAN 收不到电机反馈 | NVIC 没勾 CAN1 RX0 | 回去勾 |
| CAN 初始化返回 HAL_ERROR | F4 双 CAN 过滤器分段没设 | 确认 `SlaveStartFilterBank = 14` |
| 任务没跑 | 任务实现没加进工程 | Task/*.c 加入 Keil + Include Paths |
| CubeMX 重新生成后任务实现没了 | 任务没选 As weak | Tasks 标签 Code Generation = As weak |
| 树莓派指令收不到 | USART6 RX 中断没启动 | 确认 13.4 ② 的两段代码 |
| 编译报 hdma_usart1_tx 未定义 | USART1 DMA 没配 | 见第 9 节 |
| 电机不转 | 电机侧没配 CAN 模式 | 上位机把「通讯端口复用」设 CAN(03)、地址 1-7、波特率 500K |

---

## 附：外设时钟速查（168MHz）

| 外设 | 总线 | 时钟 |
|---|---|---|
| CAN1 | APB1 | 42 MHz |
| USART1 / USART6 | APB2 | 84 MHz |
| USART2 / USART3 | APB1 | 42 MHz |
| TIM3（APB1 定时器） | APB1 ×2 | 84 MHz |
| TIM7（HAL 时基） | APB1 ×2 | 84 MHz |
| GPIO / AHB | AHB | 168 MHz |

---

*生成工程 → 13.1 加文件 → 13.3~13.5 填三处代码 → 编译烧录，然后按《工程结构与模块说明》逐个模块验证。*
