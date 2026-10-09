/**
 * @attention   采用UTF-8字符集编码
 * @brief       ZDT Y42 闭环步进电机 串口(TTL) 控制封装（Emm 固件）
 * @details     CAN 收发器损坏后的临时替代。命令体与 CAN 版完全一致，区别：
 *               - 地址直接作为帧首字节，不再放进 CAN 扩展帧 ID
 *               - 串口帧整包发送，无需按 8 字节拆包
 *               - 接收端用"帧长状态机"按字节解析（串口是字节流，无帧边界）
 */

#include "header.h"

#define ZDT_CHECKSUM 0x6B

/* 前向声明：ZdtUart_Init 需复位接收状态机，ZdtUart_RxByte 需解析整帧 */
void ZdtUart_ResetParser(void);
static void ZdtUart_ParseFrame(uint8_t* buf, uint8_t total);

ZdtMotor_t* ZdtMotorList[8] = { NULL };

/* 低层发送：整帧 [addr][cmd...][checksum] 通过 UART5 发出 */
static void ZdtUart_SendFrame(uint8_t addr, const uint8_t* cmd, uint8_t len)
{
    uint8_t buf[16];
    uint8_t n = 0;

    buf[n++] = addr;
    for (uint8_t i = 0; i < len; i++)
        buf[n++] = cmd[i];
    buf[n++] = ZDT_CHECKSUM;

    HAL_UART_Transmit(&huart5, buf, n, 10);
}

/* ============ 电机实例维护 ============ */

void ZdtMotor_Init(ZdtMotor_t* motor, uint8_t addr)
{
    motor->Addr = addr;
    motor->Speed.Set = 0;
    motor->Speed.Real = 0;
    motor->Position.Set = 0;
    motor->Position.Real = 0;
    motor->Status = 0;
    motor->HomeStatus = 0;
    motor->LastOnlineTick = 0;

    if (addr >= 1 && addr <= 7)
        ZdtMotorList[addr] = motor;
}

int ZdtMotor_IsOnline(ZdtMotor_t* motor)
{
    return (HAL_GetTick() - motor->LastOnlineTick) < ZDT_OFFLINE_MS;
}

/* ============ 串口总线命令 ============ */

void ZdtUart_Init(void)
{
    /* UART5 外设与接收中断已由 CubeMX 生成（MX_UART5_Init / HAL_UART_Receive_IT），
       这里仅复位接收状态机，保证上电状态干净 */
    ZdtUart_ResetParser();
}

void ZdtUart_Enable(uint8_t addr, uint8_t en)
{
    uint8_t c[] = { 0xF3, 0xAB, en ? 1 : 0, 0x00 };
    ZdtUart_SendFrame(addr, c, sizeof(c));
}

void ZdtUart_Stop(uint8_t addr)
{
    uint8_t c[] = { 0xFE, 0x98, 0x00 };
    ZdtUart_SendFrame(addr, c, sizeof(c));
}

void ZdtUart_SetZero(uint8_t addr)
{
    uint8_t c[] = { 0x0A, 0x6D };
    ZdtUart_SendFrame(addr, c, sizeof(c));
}

void ZdtUart_Home(uint8_t addr, uint8_t mode)
{
    uint8_t c[] = { 0x9A, mode, 0x00 };
    ZdtUart_SendFrame(addr, c, sizeof(c));
}

void ZdtUart_SyncAll(void)
{
    uint8_t c[] = { 0xFF, 0x66 };
    ZdtUart_SendFrame(0, c, sizeof(c));   /* 广播地址 0 */
}

void ZdtUart_ReadPos(uint8_t addr)
{
    uint8_t c[] = { 0x36 };
    ZdtUart_SendFrame(addr, c, sizeof(c));
}

void ZdtUart_ReadStatus(uint8_t addr)
{
    uint8_t c[] = { 0x3C };
    ZdtUart_SendFrame(addr, c, sizeof(c));
}

/* ============ 多电机命令(0xAA) ============ */

static uint8_t  s_MultiBuf[256];
static uint16_t s_MultiLen;

void ZdtUart_MultiBegin(void)
{
    s_MultiLen = 0;
}

/* 追加一条子命令：地址 + 功能码/数据 + 校验码 */
void ZdtUart_MultiAdd(uint8_t addr, const uint8_t* cmd, uint8_t len)
{
    s_MultiBuf[s_MultiLen++] = addr;
    for (uint8_t i = 0; i < len; i++)
        s_MultiBuf[s_MultiLen++] = cmd[i];
    s_MultiBuf[s_MultiLen++] = ZDT_CHECKSUM;
}

/* 广播发送：00 AA 总字节数(2) + 各子命令 + 校验码 */
void ZdtUart_MultiSend(void)
{
    static uint8_t tx[261];
    uint16_t total = (uint16_t)(4 + s_MultiLen + 1);

    tx[0] = 0x00;
    tx[1] = 0xAA;
    tx[2] = (uint8_t)(total >> 8);
    tx[3] = (uint8_t)(total & 0xFF);
    for (uint16_t i = 0; i < s_MultiLen; i++)
        tx[4 + i] = s_MultiBuf[i];
    tx[4 + s_MultiLen] = ZDT_CHECKSUM;

    HAL_UART_Transmit(&huart5, tx, total, 20);
}

/* ============ 电机级命令 ============ */

/* 速度模式（Emm 固件 F6）：dir + 速度(整数RPM,2B) + 加速度(档位,1B) + 同步标志 */
static uint8_t ZdtMotor_BuildSpeed(float rpm, uint8_t* out)
{
    uint8_t dir = (rpm >= 0) ? 0 : 1;
    uint16_t mag = (uint16_t)(fabsf(rpm));   /* RPM（整数，0~3000） */

    out[0] = 0xF6;
    out[1] = dir;
    out[2] = mag >> 8;
    out[3] = mag & 0xFF;
    out[4] = ZDT_WHEEL_ACC;
    out[5] = 0x00;
    return 6;
}

void ZdtMotor_SetSpeed(ZdtMotor_t* motor, float rpm)
{
    uint8_t c[6];
    ZdtMotor_BuildSpeed(rpm, c);
    ZdtUart_SendFrame(motor->Addr, c, sizeof(c));

    motor->Speed.Set = rpm;
}

void ZdtMotor_MultiAddSpeed(ZdtMotor_t* motor, float rpm)
{
    uint8_t c[6];
    ZdtMotor_BuildSpeed(rpm, c);
    ZdtUart_MultiAdd(motor->Addr, c, sizeof(c));

    motor->Speed.Set = rpm;
}

/* 位置模式（Emm 固件 FD）：dir + 速度(整数RPM,2B) + 加速度(档位,1B) + 脉冲(4B,3200/圈) + 模式 + 同步 */
void ZdtMotor_SetPosition(ZdtMotor_t* motor, float deg, uint8_t mode)
{
    /* 方向按目标与当前实时位置的差自动取 */
    uint8_t dir = (deg >= motor->Position.Real) ? 0 : 1;
    uint32_t pulse = (uint32_t)(fabsf(deg) * ZDT_PULSE_PER_REV / 360.0f);   /* ° → 脉冲 */

    uint8_t c[] = {
        0xFD, dir,
        ZDT_POS_SPEED_MAX >> 8, ZDT_POS_SPEED_MAX & 0xFF,   /* 速度，整数 RPM */
        ZDT_ACC_DEFAULT,                                     /* 加速度档位 */
        pulse >> 24, pulse >> 16, pulse >> 8, pulse & 0xFF,  /* 脉冲 4B */
        mode, 0x00
    };
    ZdtUart_SendFrame(motor->Addr, c, sizeof(c));

    motor->Position.Set = deg;
}

/* ============ 接收解析 ============ */

typedef enum
{
    ZDT_RX_WAIT_ADDR = 0,   /* 等待地址字节(1~7) */
    ZDT_RX_WAIT_CODE,       /* 等待功能码，据此确定帧长 */
    ZDT_RX_COLLECT,         /* 收集数据 + 校验码 */
} ZdtRxState_t;

static ZdtRxState_t s_RxState = ZDT_RX_WAIT_ADDR;
static uint8_t s_RxAddr = 0;
static uint8_t s_RxBuf[8];
static uint8_t s_RxIdx = 0;
static uint8_t s_RxTotal = 0;

/* 根据功能码返回整帧字节数（含地址、功能码、校验码） */
static uint8_t ZdtUart_FrameLen(uint8_t code)
{
    switch (code) {
    case 0x35: return 6;   /* 转速：addr+35+符号+速度2B+校验 */
    case 0x36: return 8;   /* 位置：addr+36+符号+位置4B+校验 */
    case 0x3C: return 5;   /* 状态：addr+3C+回零状态+电机状态+校验 */
    default:   return 4;   /* 通用应答：addr+code+02/E2/EE/9F+校验 */
    }
}

void ZdtUart_ResetParser(void)
{
    s_RxState = ZDT_RX_WAIT_ADDR;
    s_RxIdx = 0;
    s_RxTotal = 0;
}

/* 逐字节喂入（由 HAL_UART_RxCpltCallback 调用） */
void ZdtUart_RxByte(uint8_t b)
{
    switch (s_RxState) {
    case ZDT_RX_WAIT_ADDR:
        if (b >= 1 && b <= 7) {      /* 只认 1~7 的返回帧，0 为广播、忽略 */
            s_RxAddr = b;
            s_RxState = ZDT_RX_WAIT_CODE;
        }
        break;

    case ZDT_RX_WAIT_CODE:
        s_RxTotal = ZdtUart_FrameLen(b);
        s_RxBuf[0] = s_RxAddr;
        s_RxBuf[1] = b;
        s_RxIdx = 2;
        s_RxState = ZDT_RX_COLLECT;
        break;

    case ZDT_RX_COLLECT:
        s_RxBuf[s_RxIdx++] = b;
        if (s_RxIdx >= s_RxTotal) {
            ZdtUart_ParseFrame(s_RxBuf, s_RxTotal);
            s_RxState = ZDT_RX_WAIT_ADDR;
        }
        break;

    default:
        s_RxState = ZDT_RX_WAIT_ADDR;
        break;
    }
}

static void ZdtUart_ParseFrame(uint8_t* buf, uint8_t total)
{
    uint8_t addr = buf[0];
    uint8_t code = buf[1];
    ZdtMotor_t* motor = (addr >= 1 && addr <= 7) ? ZdtMotorList[addr] : NULL;

    switch (code) {
    case 0x36:      /* 实时位置：符号(1) + 位置(4B 大端, 0-65535=一圈) */
        if (motor && total >= 8) {
            int32_t v = (int32_t)((buf[3] << 24) | (buf[4] << 16) | (buf[5] << 8) | buf[6]);
            v = buf[2] ? -v : v;
            motor->Position.Real = v * 360.f / 65536.f;    /* Emm：0-65535=一圈 → ° */
        }
        break;
    case 0x35:      /* 实时转速：符号(1) + 速度(2B, 整数RPM) */
        if (motor && total >= 6) {
            int32_t v = (int32_t)((buf[3] << 8) | buf[4]);
            v = buf[2] ? -v : v;
            motor->Speed.Real = v;               /* 整数 RPM */
        }
        break;
    case 0x3C:      /* 回零状态(1) + 电机状态(1) */
        if (motor && total >= 5) {
            motor->HomeStatus = buf[2];
            motor->Status = buf[3];
        }
        break;
    default:        /* 通用应答 02/E2/EE/9F：仅当作心跳 */
        break;
    }

    if (motor)
        motor->LastOnlineTick = HAL_GetTick();
}
