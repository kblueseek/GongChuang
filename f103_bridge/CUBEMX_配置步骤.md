# CubeMX 建 F103C8T6 工程 · 一步步配置

目标：生成一个 **Keil5 + HAL** 工程，把本目录的 `picomm.c` / `bridge_app.c` 加进去，
实现「树莓派 → F103 → 电脑串口助手」。

> 全程只需要在 CubeMX 里点鼠标，**不要手改生成的初始化代码**。
> 用户代码只写在 `USER CODE BEGIN/END` 之间，CubeMX 重新生成不会丢。

---

## 0. 先确认一件事：你的板子晶振是多少

翻过板子看那颗银色金属壳，丝印一般是：

| 丝印 | 填多少 |
|---|---|
| `8.000` | **8** MHz（蓝丸/Blue Pill 绝大多数是这个） |
| `12.000` | **12** MHz（部分最小系统板） |

**这一步填错，串口全是乱码。** 后面第 4 步要用。

---

## 1. 新建工程

1. 打开 CubeMX → **File → New Project**
2. 左上角 `Part Number` 搜索框输入 **`STM32F103C8`**
3. 右边列表里点 **`STM32F103C8Tx`**（LQFP48）
4. 右下角 **Start Project**

---

## 2. System Core（两个必须改的）

### SYS —— 先设调试口，否则容易"锁板"

左侧 `System Core → SYS`：

- **Debug** 下拉选 **`Serial Wire`**

> ⚠️ 必须设。默认是 `No Debug`，烧一次程序后 SWD 引脚被当普通 IO，
> 下次就连不上，得按住复位才能救。

### RCC —— 打开外部晶振

左侧 `System Core → RCC`：

- **High Speed Clock (HSE)** 选 **`Crystal/Ceramic Resonator`**
- Low Speed Clock (LSE) 保持 `Disable`

设完右边芯片图上 **PD0/PD1** 会变成绿色（接了晶振）。

---

## 3. Connectivity —— 两个串口

### USART1（接树莓派）

左侧 `Connectivity → USART1`：

- **Mode** 选 **`Asynchronous`**（芯片图上 PA9/PA10 变绿）

下面 `Parameter Settings` 确认：

| 项 | 值 |
|---|---|
| Baud Rate | **115200** |
| Word Length | 8 Bits |
| Parity | None |
| Stop Bits | 1 |
| Data Direction | Receive and Transmit |
| Over Sampling | 16 Samples |

**然后点上面的 `NVIC Settings` 标签页：**

- 找到 **`USART1 global interrupt`**，把 **Enabled** 打勾 ⚠️

> **不勾这个，一个字节都收不到。** 这是最容易漏的一步。

### USART2（接电脑）

左侧 `Connectivity → USART2`：

- **Mode** 选 **`Asynchronous`**（PA2/PA3 变绿）
- `Parameter Settings` 同样 **115200 / 8 / None / 1**
- `NVIC Settings` **不用勾**（只发不收，不占中断）

---

## 4. Clock Configuration（时钟树）

顶部标签切到 **Clock Configuration**。

1. **Input frequency** 填你在第 0 步确认的值（**8** 或 **12**）
2. `PLL Source Mux` 选 **HSE**
3. `PLLMul` 选 **×9**（8MHz 时）
   - 12MHz 晶振要选 **×6**（12×6=72）
4. `System Clock Mux` 选 **PLLCLK**
5. 检查右侧方框里的结果：

```
SYSCLK  = 72 MHz
HCLK    = 72 MHz
PCLK1   = 36 MHz   (APB1 上限 36，超过会红)
PCLK2   = 72 MHz
```

如果有红色/黄色提示，点 **"Resolve Clock Issues"** 让 CubeMX 自己修一遍。

> **为什么必须 72MHz**：串口波特率是从 PCLK 分频出来的，频率不对 → 波特率偏 →
> 电脑端全是乱码。USART1 挂在 PCLK2(72MHz)，USART2 挂在 PCLK1(36MHz)，
> CubeMX 会自动算好分频，只要时钟树对，两边都是准的 115200。

---

## 5. Project Manager（**这一步有个关键勾选项**）

### Project 标签页

- **Project Name**：`F103VisionBridge`
- **Project Location**：自己选个空目录
- **Application Structure**：`Advanced`
- **Toolchain / IDE**：**`MDK-ARM`**，Min Version **`V5`**

### Code Generator 标签页 ⚠️

**必须勾上**：

- ☑ **`Generate peripheral initialization as a pair of '.c/.h' files per peripheral`**

> 勾了才会生成单独的 `usart.c` / `usart.h`，我们的 `bridge_app.c` 靠
> `#include "usart.h"` 拿到 `huart1` / `huart2`。
> **不勾的话 `huart1` 定义在 main.c 里，编译会报未定义。**

其他保持默认（`Keep User Code when re-generating` 默认就是勾的）。

---

## 6. 生成代码

点右上角 **`GENERATE CODE`**。完成后点 **`Open Project`** 打开 Keil。

---

## 7. Keil 里加文件（3 处）

### ① 把两个 .c 加进工程

左侧 `Project` 树 → 右键 **`Application/User/Core`** 组 → `Add Existing Files...`

选：

- `picomm.c`
- `bridge_app.c`
- `picomm.h` / `bridge_app.h` 不用加（头文件靠 include path）

### ② 加头文件搜索路径

`Options for Target`（魔术棒）→ **`C/C++`** 标签 → `Include Paths` 右边 `...`

新增一行：

```
D:/27/GongChuang-vision/f103_bridge/stm32
```

（换成你实际的路径）

### ③ 在 main.c 里贴 3 行

打开 `Core/Src/main.c`，**只改这三个 USER CODE 区**，别动别的地方：

**a) 找 `/* USER CODE BEGIN Includes */`**（大约 30 行处）：

```c
/* USER CODE BEGIN Includes */
#include "bridge_app.h"
/* USER CODE END Includes */
```

**b) 找 `/* USER CODE BEGIN 2 */`**（在 `MX_USART2_UART_Init();` 后面）：

```c
  /* USER CODE BEGIN 2 */
  BridgeApp_Init();
  /* USER CODE END 2 */
```

**c) 找 `while (1)` 里面的 `/* USER CODE BEGIN WHILE */`**：

```c
  while (1)
  {
    /* USER CODE BEGIN WHILE */
    BridgeApp_Poll();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
```

> ⚠️ **HAL 版不要删 `stm32f1xx_it.c` 里的 `USART1_IRQHandler`！**
> 它负责调用 `HAL_UART_IRQHandler(&huart1)`，删了就收不到数据。
> （之前标准库版本才需要删，HAL 版本正好相反。）

---

## 8. 编译下载

1. `Project → Build Target`（F7），应 **0 Error 0 Warning**
2. 下载器选 **ST-Link Debugger** 或 **CMSIS-DAP**，`Settings → Flash Download`
   勾 **`Reset and Run`**
3. 下载

**如果下载后连不上了**（提示 `No target connected`）：

> 按住板子上的 **RESET** 不放 → 点下载 → 看到进度条动起来再松开。
> 这是因为上一次的程序把 SWD 引脚复用掉了，第 2 步设了 `Serial Wire` 就不会再发生。

---

## 9. 验证（和之前一样三步）

**① 只给 F103 上电**，串口助手 115200 8N1 应见：

```
=== F103 vision bridge (HAL) ===
USART1 <- Pi    PA10 / PA9    115200
USART2 -> PC    PA2  / PA3    115200
waiting for frames (AA 55 ...)
```

看不到 → 检查波特率、`huart2` 是否初始化（`MX_USART2_UART_Init` 有没有被 CubeMX 生成）。

**② 接树莓派**（GPIO14→PA10，GPIO15→PA9，GND 共地），Pi 上跑：

```bash
cd raspberrypi && python3 comm/pi_comm.py /dev/ttyAMA0 --vision
```

**③ 电脑串口助手**应刷：

```
[04] rings=3 (380,321) (633,321) (889,321) err=(-0.30,-0.40) yaw=0.00 seq=3
```

---

## 10. CubeMX 版常见问题

| 现象 | 原因 |
|---|---|
| 编译报 `huart1 未定义` | 第 5 步 Code Generator 那个勾没勾，`usart.h` 没生成 |
| 编译报 `cannot open usart.h` | 第 7 步 include path 没加 |
| 电脑一条数据都没有 | 第 3 步 **`USART1 global interrupt` 没勾** |
| **只收到一个字节就停** | 回调里没重新 `HAL_UART_Receive_IT`（本项目的 `bridge_app.c` 已处理） |
| 全是乱码 | 第 4 步 `Input frequency` 和实际晶振不符，或 SYSCLK 不是 72MHz |
| 下载后连不上板子 | 第 2 步没设 `Serial Wire`，按住 RESET 再下载 |
| 电脑端一直只有启动横幅 | 树莓派用的 `/dev/serial0`（Pi 5 上那是调试口），要 `/dev/ttyAMA0` |

---

## 附：为什么代码要分成 picomm.c 和 bridge_app.c

| 文件 | 依赖 | 好处 |
|---|---|---|
| `picomm.c/h` | **纯 C，不依赖 HAL** | 能在电脑上用 gcc 跑协议自测（`hosttest.c`） |
| `bridge_app.c/h` | HAL | 串口初始化、中断回调、格式化输出 |

这样协议逻辑和硬件绑定彻底分开，**协议不用烧板子就能验证**。
