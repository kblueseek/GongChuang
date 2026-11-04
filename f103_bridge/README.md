# F103 视觉链路桥（树莓派 → STM32F103C8T6 → 电脑串口助手）

用途：**先证明数据链路是通的**，再谈接 F407 主控。

```
13 步视觉流程
      │
树莓派 raspberrypi/comm/pi_comm.py  --vision
      │  UART 115200 8N1，PiComm 帧（CMD 0x20）
      ▼
STM32F103C8T6（本目录 stm32/）
      │  USART2 转成 ASCII 一行
      ▼
电脑串口助手（115200 8N1）
```

树莓派侧**没有新增任何 py 文件**——协议和发送都加在已有的
`raspberrypi/comm/pi_comm.py` 里。

---

## 一、文件

```
f103_bridge/
├─ CUBEMX_配置步骤.md   ← **先看这个**，一步步建 CubeMX + Keil 工程
└─ stm32/
   ├─ picomm.h        协议常量 + 解析器声明（纯 C，不依赖 MCU 头）
   ├─ picomm.c        CRC16 + 收帧状态机（可在电脑上 gcc 自测）
   ├─ bridge_app.h    HAL 胶水层声明
   ├─ bridge_app.c    串口中断收、格式化输出（依赖 HAL）
   └─ hosttest.c      【仅电脑用】把字节流喂给解析器的测试驱动
```

`.c/.h` 是 **GBK 编码**（Keil 打开中文注释不乱码）。

> 工程用 **CubeMX 生成 HAL**，配置步骤见 [CUBEMX_配置步骤.md](CUBEMX_配置步骤.md)。
> 只需在 CubeMX 生成的 `main.c` 的 3 个 USER CODE 区各贴 1 行，重新生成不会丢。

树莓派侧改了 `raspberrypi/comm/pi_comm.py`，新增：
- `CMD_VISION_STEP = 0x20`
- `encode_vision_step(step, rings, err_x_mm, err_y_mm, yaw_deg)`
- `PiCommLink.send_vision_step(...)`
- `__main__` 增加 `--vision` 模式（循环发 13 步）

---

## 二、接线

### USART1 ↔ 树莓派（40pin 排针）

| F103 引脚 | 方向 | 树莓派 |
|---|---|---|
| **PA10** (USART1_RX) | ← | **GPIO14 / TXD**（pin 8） |
| **PA9** (USART1_TX) | → | **GPIO15 / RXD**（pin 10） |
| **GND** | — | **GND**（pin 6） |

⚠️ TX 接 RX、RX 接 TX（**交叉**）。**必须共地**，否则一定乱码。
⚠️ 两边都是 3.3V 电平，直连；不要给树莓派灌 5V。

### USART2 ↔ USB-TTL ↔ 电脑

| F103 引脚 | 方向 | USB-TTL |
|---|---|---|
| **PA2** (USART2_TX) | → | **RXD** |
| **PA3** (USART2_RX) | ← | TXD（本程序不用，可不接） |
| **GND** | — | GND |

---

## 三、建 Keil 工程

**完整步骤见 [CUBEMX_配置步骤.md](CUBEMX_配置步骤.md)**（CubeMX 逐步配置 + Keil 加文件 + 贴代码）。

要点速览：
1. CubeMX 选 `STM32F103C8Tx`，SYS→**Serial Wire**，RCC→HSE 晶振
2. USART1/USART2 → Asynchronous，115200 8N1；**USART1 的 NVIC 全局中断必须勾**
3. 时钟树填实际晶振（8 或 12MHz），SYSCLK 做到 **72MHz**
4. Code Generator 勾 **生成 usart.c/.h**
5. Keil 里加 `picomm.c` `bridge_app.c` + include path
6. 在 `main.c` 的 3 个 USER CODE 区各贴 1 行

## 四、树莓派 UART 使能（**本项目用树莓派 5**）

### ⚠️ Pi 5 最大的坑：`/dev/serial0` 不是 GPIO 那一路

| 设备 | 是什么 | 能不能用 |
|---|---|---|
| `/dev/ttyAMA0` | **GPIO14/15 的 PL011 UART** | ✅ **用这个** |
| `/dev/serial0` | Pi 5 上是 `/dev/ttyAMA10`——**板边 3 针调试口** | ❌ 用它会一条都收不到 |
| `/dev/ttyAMA10` | 调试口，同时是 Linux 控制台 | ❌ |

前几代 Pi 上 `/dev/serial0` 指向 GPIO，**照抄老教程就是"接线全对、毫无反应"**。

### 打开 GPIO14/15 的 UART

```bash
sudo raspi-config
#  3 Interface Options → I6 Serial Port
#     "login shell over serial?"        → No
#     "serial port hardware enabled?"   → Yes
sudo reboot
```

Pi 5 上 raspi-config 写的是 `dtparam=uart0=on`。手动等效做法：

```bash
echo 'dtparam=uart0=on' | sudo tee -a /boot/firmware/config.txt && sudo reboot
```

> Pi 5 **不需要** `dtoverlay=disable-bt`（蓝牙走主芯片另一路 UART，跟 GPIO14/15 无关），
> 也不要照 Pi 4 的教程去动 `cmdline.txt` —— Pi 5 的控制台在调试口上，不占 GPIO。

### 验证

```bash
ls -l /dev/ttyAMA0
```

看到 `crw-rw---- 1 root dialout` 就对了（如果显示 `root tty`，说明控制台还占着它，检查 `serial-getty`）。

把用户加进 `dialout` 组，否则会 `权限不够`：

```bash
sudo usermod -aG dialout $USER && sudo reboot
```

---

## 五、三步验证（按顺序做，别跳）

### 第 1 步：F103 自己能出声

只给 F103 上电（不接树莓派），电脑串口助手 **115200 8N1**，应看到：

```
=== F103 vision bridge ===
USART1 <- Pi    PA10 / PA9    115200
USART2 -> PC    PA2  / PA3    115200
waiting for frames (AA 55 ...)
```

**看不到就先解决这个**：波特率不对，或晶振不是 8MHz（PLL 按 8MHz 配的）。

### 第 2 步：树莓派能发

在**不接线**的情况下先看它能不能跑起来（串口打不开会一直重连，不影响打印）：

```bash
cd raspberrypi
python3 comm/pi_comm.py /dev/ttyAMA0 --vision
```

应看到每 200ms 一行 `step N rings=M sent=True/False`。

### 第 3 步：联调

接好 PA10 / PA9 / GND，树莓派重跑同一条命令。电脑串口助手应刷：

```
=== F103 vision bridge ===
[01] rings=0 err=(0.00,0.00) yaw=0.00 seq=0
[04] rings=3 (380,321) (633,321) (889,321) err=(-0.30,-0.40) yaw=0.00 seq=3
[13] rings=1 (633,321) err=(0.00,0.00) yaw=0.00 seq=12
[00] rings=0 err=(0.00,0.00) yaw=0.00 seq=13
```

每满 100 个合法帧多一条 `[STAT] ok=100 bad=0`。**`bad` 一直为 0 就是链路干净。**

---

## 六、协议

与 PiComm 逐字节一致（`pi_comm.py` 的 `build_frame`）：

```
AA 55 | LEN | SEQ | CMD | DATA | CRC_L | CRC_H
LEN = len(DATA) + 3         整帧 = LEN + 4
CRC16-MODBUS（0xFFFF / 0xA001），覆盖 LEN/SEQ/CMD/DATA，不含帧头，低字节先发
DATA 大端
```

**`CMD = 0x20 VISION_STEP`**，DATA 20 字节：

| 偏移 | 类型 | 字段 |
|---|---|---|
| 0 | u8 | step 步骤号 1..13，0=行车 |
| 1 | u8 | rings 本轮检出环数 0..3 |
| 2 | i16 | ring0_x 像素 |
| 4 | i16 | ring0_y |
| 6 | i16 | ring1_x |
| 8 | i16 | ring1_y |
| 10 | i16 | ring2_x |
| 12 | i16 | ring2_y |
| 14 | i16 | err_x_mm × 100 |
| 16 | i16 | err_y_mm × 100 |
| 18 | i16 | yaw_deg × 100 |

> **0x20 是调试旁路命令，F407 的 PiComm.c 不实现它。**
> 正式接主控时要用 `VEL` / `GIMBAL_TARGET` / `POSE` 那些已有命令。

---

## 七、电脑上重跑协议自测（不用板子）

```bash
cd f103_bridge/stm32
gcc -std=c99 -Wall -Wextra -I. -o hosttest.exe hosttest.c picomm.c
```

生成一段字节流再喂进去（用 pi_comm 的编码，和树莓派发的完全一样）：

```bash
cd <仓库根>/raspberrypi
python3 -c "
import sys; sys.path.insert(0, '.')
from comm.pi_comm import build_frame, encode_vision_step, CMD_VISION_STEP
steps = [(4, [(380,321),(633,321),(889,321)], -0.3, -0.4, 0.0),
         (13, [(633,321)], 0.0, 0.0, 0.0),
         (0, [], 0.0, 0.0, 0.0)]
open('frames.bin','wb').write(b''.join(
    build_frame(CMD_VISION_STEP, encode_vision_step(*a), seq=i)
    for i, a in enumerate(steps)))
"
../f103_bridge/stm32/hosttest.exe frames.bin
```

期望：

```
FRAME ok=1 cmd=0x20 seq=0 len=23 n=20 ...
FRAME ok=1 cmd=0x20 seq=1 len=23 n=20 ...
FRAME ok=1 cmd=0x20 seq=2 len=23 n=20 ...
STAT ok=3 bad=0
```

（`hosttest.exe` 是 Windows 程序；树莓派上可以直接用同一条 gcc 命令编出 linux 版。）

---

## 八、常见问题

| 现象 | 原因 |
|---|---|
| 电脑一条都没有 | TX/RX 没交叉；没共地；F103 没跑起来 |
| **Pi 5 上接线全对但一条都没有** | **用了 `/dev/serial0`**（Pi 5 指向调试口），改成 `/dev/ttyAMA0` |
| 报 `Permission denied: /dev/ttyAMA0` | 没在 `dialout` 组：`sudo usermod -aG dialout $USER` |
| 全是乱码 | 波特率不是 115200；晶振不是 8MHz 但 PLL 按 8MHz 配 |
| 偶发 `!! CRC ERR` | 线太长没屏蔽；两边波特率有偏差；电平不匹配 |
| 只收到半行就断 | 树莓派串口被控制台占用（没关 `serial-getty`） |
| `[BAD] vision frame data_len<20` | 帧对上了但内容不够，检查发送端 |
| 数据量很大但解析不出帧 | 波特率对但帧头不对，先看原始字节是不是 `AA 55` |
| Python 报 `未安装 pyserial` | `sudo apt install python3-serial` |

---

## 九、接真实视觉时改哪里

`pi_comm.py` 的 `__main__` 里那张 `VISION_STEPS` 是模拟数据。接真实流程时，
在 `raspberrypi/vision/flow_state_machine.py` 每步拿到检测结果后调用：

```python
link.send_vision_step(
    step=step.num,
    rings=ring_result.centers_px,
    err_x_mm=ring_result.error_x_mm,
    err_y_mm=ring_result.error_y_mm,
    yaw_deg=ring_result.station_yaw_deg,
)
```

`pi_comm.PiCommLink` 自带非阻塞读写和自动重连，不影响视觉主循环的帧率。
