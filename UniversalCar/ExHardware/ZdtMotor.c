/**
 * @attention   采用UTF-8字符集编码
 * @brief       ZDT Y42 闭环步进电机 CAN 控制封装（Emm 固件）
 * @details     电机内部闭环，STM32 侧不跑电流/速度环，直接下发速度或位置指令。
 *              采用注册机制维护地址表，统一了所有电机的维护与反馈解析。
 */

#include "header.h"

#define ZDT_CHECKSUM 0x6B

ZdtMotor_t* ZdtMotorList[8] = { NULL };

/* 调试计数（Keil Watch 里观察，判断 CAN 到底发没发、发没发成功） */
volatile uint32_t ZdtCan_TxCount   = 0;   /* 成功发送的 CAN 帧数 */
volatile uint32_t ZdtCan_TxErr     = 0;   /* HAL_CAN_AddTxMessage 返回非 HAL_OK 的次数 */
volatile uint32_t ZdtCan_TxLastRet = 0;   /* 最后一次 AddTxMessage 返回值 */

/* 发送一帧（单包），扩展帧 ID = (地址<<8)|包序号 */
static void Zdt_SendFrame(uint8_t addr, uint8_t packet, const uint8_t* d, uint8_t n)
{
    CAN_TxHeaderTypeDef tx = { 0 };
    uint32_t mb;
    uint32_t retry = 0;
    tx.ExtId = ((uint32_t)addr << 8) | packet;
    tx.IDE = CAN_ID_EXT;
    tx.RTR = CAN_RTR_DATA;
    tx.DLC = n;

    /* CAN1 只有 3 个 TX 邮箱，全满时 HAL_CAN_AddTxMessage 会直接丢帧（返回 HAL_ERROR）。
       底盘/云台都是背靠背连发 4~6 帧，必须轮询等空闲邮箱，否则后几帧被静默丢弃。
       500k 下一帧约 250us，本循环最坏阻塞约 1ms，对 5ms 周期任务可接受。 */
    ZdtCan_TxLastRet = HAL_CAN_AddTxMessage(&hcan1, &tx, (uint8_t*)d, &mb);
    while (ZdtCan_TxLastRet != HAL_OK && retry < 10000)
    {
        retry++;
        ZdtCan_TxLastRet = HAL_CAN_AddTxMessage(&hcan1, &tx, (uint8_t*)d, &mb);
    }

    if (ZdtCan_TxLastRet == HAL_OK)
        ZdtCan_TxCount++;
    else
        ZdtCan_TxErr++;
}

/* 低层发送：cmd[0] 为功能码，追加校验码后拆包发送。
   手册 4.2：CAN 拆包时每包首字节都要重复功能码，数据每包最多 7 字节，
   例如 12 字节命令拆成 "FD xx xx xx xx xx xx xx" + "FD xx xx xx 6B"。 */
static void Zdt_MotorSend(uint8_t addr, const uint8_t* cmd, uint8_t len)
{
    uint8_t func = cmd[0];
    uint8_t dataLen = len - 1;          /* 功能码之外的数据字节数 */
    uint8_t total = dataLen + 1;        /* 数据 + 校验码 */
    uint8_t sent = 0, packet = 0;

    while (sent < total) {
        uint8_t n = (total - sent > 7) ? 7 : (total - sent);
        uint8_t buf[8];
        buf[0] = func;                  /* 每包首字节重复功能码 */
        for (uint8_t i = 0; i < n; i++) {
            uint8_t idx = sent + i;
            buf[1 + i] = (idx < dataLen) ? cmd[1 + idx] : ZDT_CHECKSUM;
        }
        Zdt_SendFrame(addr, packet, buf, n + 1);
        sent += n;
        packet++;
    }
}

/* ============ 电机实例维护 ============ */

void ZdtMotor_Init(ZdtMotor_t* motor, uint8_t addr)
{
    motor->Addr = addr;
    motor->Speed.Set = 0;
    motor->Speed.Real = 0;
    motor->Position.Set = 0;
    motor->Position.Real = 0;
    motor->Position.RawCount = 0;
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

/* ============ CAN 总线命令 ============ */

void ZdtCan_Init(void)
{
    CAN_FilterTypeDef f = { 0 };
    f.FilterActivation = CAN_FILTER_ENABLE;
    f.FilterBank = 0;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterIdHigh = 0;
    f.FilterIdLow = 0;
    f.FilterMaskIdHigh = 0;
    f.FilterMaskIdLow = 0;
    f.FilterMode = CAN_FILTERMODE_IDMASK;
    f.FilterScale = CAN_FILTERSCALE_32BIT;
    f.SlaveStartFilterBank = 14;    /* F4 双 CAN 时必须写，否则 HAL 返回 HAL_ERROR */
    HAL_CAN_ConfigFilter(&hcan1, &f);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void ZdtCan_Enable(uint8_t addr, uint8_t en)
{
    uint8_t c[] = { 0xF3, 0xAB, en ? 1 : 0, 0x00 };
    Zdt_MotorSend(addr, c, sizeof(c));
}

void ZdtCan_Stop(uint8_t addr)
{
    uint8_t c[] = { 0xFE, 0x98, 0x00 };
    Zdt_MotorSend(addr, c, sizeof(c));
}

void ZdtCan_SetZero(uint8_t addr)
{
    uint8_t c[] = { 0x0A, 0x6D };
    Zdt_MotorSend(addr, c, sizeof(c));
}

void ZdtCan_Home(uint8_t addr, uint8_t mode)
{
    uint8_t c[] = { 0x9A, mode, 0x00 };
    Zdt_MotorSend(addr, c, sizeof(c));
}

void ZdtCan_SyncAll(void)
{
    uint8_t c[] = { 0xFF, 0x66 };
    Zdt_MotorSend(0, c, sizeof(c));   /* 广播地址 0 */
}

void ZdtCan_ReadPos(uint8_t addr)
{
    uint8_t c[] = { 0x36 };
    Zdt_MotorSend(addr, c, sizeof(c));
}

void ZdtCan_ReadStatus(uint8_t addr)
{
    uint8_t c[] = { 0x3C };
    Zdt_MotorSend(addr, c, sizeof(c));
}

/* 定时返回信息命令（手册 5.5.1）：让电机每隔 interval_ms 自动上报指定信息，无需频繁轮询。
   func 为要返回的信息功能码：0x36=实时位置(4B,65536/圈)，0x35=实时转速(RPM)，0x3A=状态标志等。
   interval_ms=0 表示停止返回。 */
void ZdtCan_SetTimedReturn(uint8_t addr, uint8_t func, uint16_t interval_ms)
{
    uint8_t c[] = { 0x11, 0x18, func, interval_ms >> 8, interval_ms & 0xFF };
    Zdt_MotorSend(addr, c, sizeof(c));
}

/* ============ 多电机命令 (0xAA)：一条广播帧发多个电机命令，同时执行 ============ */
/* 手册 5.3.1：00 AA 总字节数(2) + 各子命令(地址+功能码/数据+校验码) + 校验码，
   地址 0 为广播，所有电机同时开始；CAN 上按功能码重复规则拆包。 */

static uint8_t  s_MultiBuf[256];
static uint16_t s_MultiLen;

void ZdtCan_MultiBegin(void)
{
    s_MultiLen = 0;
}

/* 追加一条子命令：地址 + 功能码/数据 + 校验码 */
void ZdtCan_MultiAdd(uint8_t addr, const uint8_t* cmd, uint8_t len)
{
    s_MultiBuf[s_MultiLen++] = addr;
    for (uint8_t i = 0; i < len; i++)
        s_MultiBuf[s_MultiLen++] = cmd[i];
    s_MultiBuf[s_MultiLen++] = ZDT_CHECKSUM;
}

/* 广播发送整帧，按"每包首字节重复功能码 AA、每包最多 8 字节"拆包 */
void ZdtCan_MultiSend(void)
{
    uint16_t total = (uint16_t)(5 + s_MultiLen);   /* 地址1 + 功能码1 + 长度2 + 子命令 + 校验1 */

    static uint8_t tx[261];
    uint16_t n = 0;
    tx[n++] = 0xAA;                       /* 功能码 */
    tx[n++] = (uint8_t)(total >> 8);      /* 长度高 */
    tx[n++] = (uint8_t)(total & 0xFF);    /* 长度低 */
    for (uint16_t i = 0; i < s_MultiLen; i++)
        tx[n++] = s_MultiBuf[i];
    tx[n++] = ZDT_CHECKSUM;

    /* tx[0]=AA 为功能码，tx[1..n-1] 为数据，按功能码重复规则拆包发送（地址0 → ExtId 0x0000） */
    uint16_t dataLen = n - 1;
    uint16_t sent = 0;
    uint8_t packet = 0;
    while (sent < dataLen) {
        uint8_t m = (dataLen - sent > 7) ? 7 : (uint8_t)(dataLen - sent);
        uint8_t buf[8];
        buf[0] = 0xAA;
        for (uint8_t i = 0; i < m; i++)
            buf[1 + i] = tx[1 + sent + i];
        Zdt_SendFrame(0, packet, buf, m + 1);
        sent += m;
        packet++;
    }
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
    Zdt_MotorSend(motor->Addr, c, sizeof(c));

    motor->Speed.Set = rpm;
}

/* 速度子命令追加到多电机缓冲（配合 ZdtCan_MultiBegin/Send，用于底盘 4 轮同步） */
void ZdtMotor_MultiAddSpeed(ZdtMotor_t* motor, float rpm)
{
    uint8_t c[6];
    ZdtMotor_BuildSpeed(rpm, c);
    ZdtCan_MultiAdd(motor->Addr, c, sizeof(c));

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
    Zdt_MotorSend(motor->Addr, c, sizeof(c));

    motor->Position.Set = deg;
}

/* ============ 接收解析 ============ */

void ZdtCan_RxCallback(uint32_t id, uint8_t* data, uint8_t len)
{
    uint8_t addr = (id >> 8) & 0xFF;
    if (addr == 0 || addr > 7)
        return;

    ZdtMotor_t* motor = ZdtMotorList[addr];
    if (motor == NULL)
        return;

    /* 拆包重组：>8 字节的返回帧按包序号拼接（本固件返回帧最长 8 字节，预留兼容） */
    switch (data[0]) {
    case 0x36:      /* 实时位置：符号(1) + 位置(4B 大端)，多圈累计，65536=一圈 */
        if (len >= 6) {
            int32_t v = (int32_t)((data[2] << 24) | (data[3] << 16) | (data[4] << 8) | data[5]);
            v = data[1] ? -v : v;
            motor->Position.RawCount = v;                  /* 原始整数计数，最高精度 */
            motor->Position.Real = v * 360.f / 65536.f;    /* 换算成角度 ° */
        }
        break;
    case 0x35:      /* 实时转速：符号(1) + 速度(2B, 整数RPM) */
        if (len >= 4) {
            int32_t v = (int32_t)((data[2] << 8) | data[3]);
            v = data[1] ? -v : v;
            motor->Speed.Real = v;               /* 整数 RPM */
        }
        break;
    case 0x3C:      /* 回零状态(1) + 电机状态(1) */
        if (len >= 3) {
            motor->HomeStatus = data[1];
            motor->Status = data[2];
        }
        break;
    default:
        break;
    }

    motor->LastOnlineTick = HAL_GetTick();
}
