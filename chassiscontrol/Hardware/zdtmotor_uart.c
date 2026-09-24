/* zdtmotor_uart.c — ZDT Y42 闭环步进电机 串口(UART) 控制封装（X 固件） */
#include "zdtmotor_uart.h"
#include "usart.h"

#define ZDT_CHECKSUM 0x6B

ZDT_UARTState zdt_uart_state[8];

/* ---------------- 低层发送 ---------------- */

static void ZDT_UART_Transmit(const uint8_t *buf, uint16_t len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 200);
}

/* 单命令：地址 + 功能码/数据 + 校验码 */
void ZDT_UART_Send(uint8_t addr, const uint8_t *cmd, uint8_t len)
{
    uint8_t buf[32];
    buf[0] = addr;
    for (uint8_t i = 0; i < len; i++) buf[i + 1] = cmd[i];
    buf[len + 1] = ZDT_CHECKSUM;
    ZDT_UART_Transmit(buf, (uint16_t)(len + 2));
}

/* ---------------- 多电机命令(0xAA) ---------------- */

static uint8_t multi_buf[256];
static uint16_t multi_len;

void ZDT_MultiBegin(void)
{
    multi_len = 0;
}

/* 追加一条子命令：地址 + 功能码/数据 + 校验码 */
void ZDT_MultiAdd(uint8_t addr, const uint8_t *cmd, uint8_t len)
{
    multi_buf[multi_len++] = addr;
    for (uint8_t i = 0; i < len; i++) multi_buf[multi_len++] = cmd[i];
    multi_buf[multi_len++] = ZDT_CHECKSUM;
}

/* 广播发送：00 AA 总字节数(2) + 各子命令 + 校验码 */
void ZDT_MultiSend(void)
{
    static uint8_t tx[261];
    uint16_t total = (uint16_t)(4 + multi_len + 1);
    tx[0] = 0x00;
    tx[1] = 0xAA;
    tx[2] = (uint8_t)(total >> 8);
    tx[3] = (uint8_t)(total & 0xFF);
    for (uint16_t i = 0; i < multi_len; i++) tx[4 + i] = multi_buf[i];
    tx[4 + multi_len] = ZDT_CHECKSUM;
    ZDT_UART_Transmit(tx, total);
}

/* ---------------- 定时返回信息(0x11 18) ---------------- */

void ZDT_UART_Periodic(uint8_t addr, uint8_t infoCode, uint16_t ms)
{
    uint8_t c[] = {0x11, 0x18, infoCode, (uint8_t)(ms >> 8), (uint8_t)ms};
    ZDT_UART_Send(addr, c, sizeof(c));
}

/* ---------------- 基础命令 ---------------- */

void ZDT_UART_Enable(uint8_t addr, uint8_t en)
{
    uint8_t c[] = {0xF3, 0xAB, en ? 1 : 0, 0x00};
    ZDT_UART_Send(addr, c, sizeof(c));
}

void ZDT_UART_Stop(uint8_t addr)
{
    uint8_t c[] = {0xFE, 0x98, 0x00};
    ZDT_UART_Send(addr, c, sizeof(c));
}

void ZDT_UART_SetZero(uint8_t addr)
{
    uint8_t c[] = {0x0A, 0x6D};
    ZDT_UART_Send(addr, c, sizeof(c));
}

void ZDT_UART_Home(uint8_t addr, uint8_t mode)
{
    uint8_t c[] = {0x9A, mode, 0x00};
    ZDT_UART_Send(addr, c, sizeof(c));
}

/* ---------------- 运动命令（X 固件） ---------------- */

/* 速度模式(F6)：方向 + 加速度(RPM/s,2B) + 速度(0.1RPM,2B) + 同步标志 */
void ZDT_UART_Speed(uint8_t addr, uint8_t dir, uint16_t acc, uint16_t speed)
{
    uint8_t c[] = {0xF6, dir, acc >> 8, acc, speed >> 8, speed, 0x00};
    ZDT_UART_Send(addr, c, sizeof(c));
}

/* 梯形位置模式(FD)：方向 + 加/减速加速度(2B) + 最大速度(2B) + 位置(0.1°,4B) + 模式 + 同步标志 */
void ZDT_UART_Pos(uint8_t addr, uint8_t dir, uint16_t speed, uint16_t acc,
                  uint16_t dec, uint32_t pos, uint8_t mode)
{
    uint8_t c[] = {
        0xFD, dir,
        acc >> 8, acc, dec >> 8, dec,
        speed >> 8, speed,
        pos >> 24, pos >> 16, pos >> 8, pos,
        mode, 0x00
    };
    ZDT_UART_Send(addr, c, sizeof(c));
}

/* ---------------- 读取反馈 ---------------- */

void ZDT_UART_ReadPos(uint8_t addr)
{
    uint8_t c[] = {0x36};
    ZDT_UART_Send(addr, c, sizeof(c));
}

void ZDT_UART_ReadStatus(uint8_t addr)
{
    uint8_t c[] = {0x3C};
    ZDT_UART_Send(addr, c, sizeof(c));
}

/* ---------------- 接收解析 ---------------- */

/* 按功能码查返回数据区长度（不含地址、功能码、校验码） */
static uint8_t ZDT_RespDataLen(uint8_t code)
{
    switch (code) {
    case 0x1F: case 0x20: return 4;                                  /* 固件/硬件版本、相电阻电感 */
    case 0x24: case 0x26: case 0x27: case 0x31:
    case 0x35: case 0x38: case 0x39: return 2;                      /* 2 字节值 */
    case 0x32: case 0x33: case 0x34: case 0x36: case 0x37: return 5; /* 符号 + 4 字节 */
    case 0x3A: case 0x3B: case 0x3D: case 0x1A: return 1;           /* 1 字节标志 */
    case 0x3C: return 2;                                             /* 回零 + 电机状态 */
    case 0x41: return 2;                                             /* 位置到达窗口 */
    default: return 1;                                               /* 命令确认 02/E2/EE/9F */
    }
}

typedef enum { RX_ADDR, RX_CODE, RX_DATA, RX_CHECK } ZDT_RxState_t;

static ZDT_RxState_t rx_st = RX_ADDR;
static uint8_t rx_addr, rx_code, rx_need, rx_cnt;
static uint8_t rx_data[8];

static void ZDT_UART_Parse(uint8_t addr, uint8_t code, const uint8_t *d, uint8_t len)
{
    if (addr < 1 || addr > 7) return;

    switch (code) {
    case 0x36:                                   /* 实时位置：符号(1) + 位置(4, 大端)，0.1° */
        if (len >= 5) {
            int32_t v = (int32_t)((d[1] << 24) | (d[2] << 16) | (d[3] << 8) | d[4]);
            zdt_uart_state[addr].pos = d[0] ? -v : v;
        }
        break;
    case 0x3C:                                   /* 回零状态(1) + 电机状态(1) */
        if (len >= 2) {
            zdt_uart_state[addr].home = d[0];
            zdt_uart_state[addr].status = d[1];
        }
        break;
    case 0x3A:                                   /* 电机状态(1) */
        if (len >= 1) zdt_uart_state[addr].status = d[0];
        break;
    case 0x3B:                                   /* 回零状态(1) */
        if (len >= 1) zdt_uart_state[addr].home = d[0];
        break;
    default:
        break;
    }
}

/* 状态机逐字节解析（UART 是字节流，需自己切帧，靠"功能码→长度表"定位帧边界） */
void ZDT_UART_ReceiveByte(uint8_t b)
{
    switch (rx_st) {
    case RX_ADDR:
        if (b >= 1 && b <= 7) { rx_addr = b; rx_st = RX_CODE; }
        break;
    case RX_CODE:
        rx_code = b;
        rx_need = ZDT_RespDataLen(b);
        rx_cnt = 0;
        rx_st = RX_DATA;
        break;
    case RX_DATA:
        if (rx_cnt < sizeof(rx_data)) rx_data[rx_cnt++] = b;
        if (rx_cnt >= rx_need) rx_st = RX_CHECK;
        break;
    case RX_CHECK:
        if (b == ZDT_CHECKSUM) ZDT_UART_Parse(rx_addr, rx_code, rx_data, rx_need);
        rx_st = RX_ADDR;
        break;
    }
}

/* ---------------- 初始化 ---------------- */

extern uint8_t uart1_rx_byte;   /* 定义在 usart.c，由接收回调统一分发 */

void ZDT_UART_Init(void)
{
    HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
}
