/**
 * @attention   采用UTF-8字符集编码
 * @brief       树莓派通信协议实现
 */

#include "header.h"

PiCommand_t PiCommand;

/* 接收状态机：每条链路一套上下文（蓝牙上行 / USART6 有线），避免字节交织时互相打断 */
#define PI_RX_BUF_SIZE  64

typedef struct
{
    uint8_t Buf[PI_RX_BUF_SIZE];
    uint8_t Cnt;
    uint8_t Len;
} PiCommRxCtx_t;

static PiCommRxCtx_t s_RxWired;     /* USART6：有线链路（调试/备用） */
static PiCommRxCtx_t s_RxBle;       /* USART2：蓝牙上行 */

/* ============ CRC16-MODBUS ============ */
static uint16_t PiComm_CRC16(const uint8_t* data, uint8_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

/* ============ 发送 ============ */
#define PI_TX_BUF_SIZE  24      /* 最长帧 = 7 + 10(里程计) = 17B，留余量 */

#if LORA_DOWNLINK_ENABLE
/* 遥测帧用中断发送而不是阻塞发送：即使波特率降下来，也不会把调用它的任务卡住
   任务里只把帧丢进待发槽，UART4 中断发完再取下一帧。遥测是周期性的，
   来不及发就覆盖旧帧（最新值才有意义），不排队、不阻塞。 */
static uint8_t  s_TxBuf[PI_TX_BUF_SIZE];        /* 中断正在发送的帧 */
static uint8_t  s_TxPending[PI_TX_BUF_SIZE];    /* 等待发送的最新帧 */
static volatile uint8_t s_TxPendingLen = 0;
static volatile uint8_t s_TxBusy = 0;
volatile uint32_t PiComm_TxDropCount = 0;       /* 被覆盖/启动失败的帧数（调试用） */

/// @brief lora发送函数
/// @param  
static void PiComm_TxPump(void)
{
    uint32_t primask;
    uint8_t  len;

    primask = __get_PRIMASK();
    __disable_irq();
    if (s_TxBusy || s_TxPendingLen == 0) {
        if (!primask) __enable_irq();
        return;
    }
    len = s_TxPendingLen;
    memcpy(s_TxBuf, s_TxPending, len);
    s_TxPendingLen = 0;
    s_TxBusy = 1;
    if (!primask) __enable_irq();

    if (HAL_UART_Transmit_IT(&huart4, s_TxBuf, len) != HAL_OK) {
        s_TxBusy = 0;               /* 启动失败：下一帧还会触发 Pump，不至于卡死 */
        PiComm_TxDropCount++;
    }
}

static void PiComm_TxEnqueue(const uint8_t* frame, uint8_t len)
{
    uint32_t primask;

    if (len > PI_TX_BUF_SIZE)
        return;

    primask = __get_PRIMASK();
    __disable_irq();
    if (s_TxPendingLen != 0)
        PiComm_TxDropCount++;       /* 旧帧还没发出去就被新帧覆盖 */
    memcpy(s_TxPending, frame, len);
    s_TxPendingLen = len;
    if (!primask) __enable_irq();

    PiComm_TxPump();
}
#endif

static void PiComm_SendFrame(uint8_t cmd, const uint8_t* data, uint8_t dataLen)
{
    static uint8_t s_Seq = 0;
    uint8_t frame[PI_TX_BUF_SIZE];

    frame[0] = 0xAA;
    frame[1] = 0x55;
    frame[2] = 3 + dataLen;         /* LEN = CMD + DATA + CRC */
    frame[3] = s_Seq++;
    frame[4] = cmd;
    memcpy(&frame[5], data, dataLen);

    uint16_t crc = PiComm_CRC16(&frame[2], 3 + dataLen);    /* CRC 覆盖 LEN~DATA */
    frame[5 + dataLen] = crc & 0xFF;
    frame[6 + dataLen] = crc >> 8;

#if LORA_DOWNLINK_ENABLE
    PiComm_TxEnqueue(frame, 7 + dataLen);                   /* 非阻塞：LoRa / UART4 */
#else
    HAL_UART_Transmit(&huart6, frame, 7 + dataLen, 100);    /* 回落：USART6 有线 */
#endif
}

#if LORA_DOWNLINK_ENABLE
void PiComm_TxCpltCallback(void)
{
    s_TxBusy = 0;
    PiComm_TxPump();                /* 立刻接上被覆盖等待的那一帧 */
}

void PiComm_ErrorCallback(void)
{
    /* UART4 只发不收，出错时清掉发送状态即可，下一帧自动重新启动 */
    s_TxBusy = 0;
    s_TxPendingLen = 0;
}
#else
void PiComm_TxCpltCallback(void) { }
void PiComm_ErrorCallback(void)  { }
#endif

/* ============ 接收解析 ============ */
static void PiComm_ParseFrame(const uint8_t* f, uint8_t len)
{
    uint8_t  cmd = f[4];
    const uint8_t* d = &f[5];
    uint8_t  dLen = len - 3;        /* LEN 域已含 CMD+CRC */

    switch (cmd) {
    case PI_CMD_VEL:                /* vx vy wz int16×3 */
        if (dLen >= 6) {
            PiCommand.Vel.Vx = (int16_t)((d[0] << 8) | d[1]) / 1000.f;    /* mm/s → m/s */
            PiCommand.Vel.Vy = (int16_t)((d[2] << 8) | d[3]) / 1000.f;
            PiCommand.Vel.Wz = (int16_t)((d[4] << 8) | d[5]) / 1000.f;    /* mrad/s → rad/s */
            PiCommand.PosTarget.Valid = 0;   /* 收到速度指令：退出位置闭环，回到手动速度模式 */
        }
        break;
    case PI_CMD_GIMBAL_TARGET:
        if (dLen >= 5) {
            uint8_t axis = d[0];
            if (axis < 3) {
                int32_t deg = (int32_t)((d[1] << 24) | (d[2] << 16) | (d[3] << 8) | d[4]);
                PiCommand.GimbalTarget[axis].Deg = deg / 100.f;            /* ×100 → ° */
                PiCommand.GimbalTarget[axis].Mask = 1;
                /* 位置指令立即接管：清掉速度模式时间戳，否则这一轴还要在速度模式里
                   再等 GIMBAL_VEL_TIMEOUT_MS 才切回位置，上位机发来的目标被晾着。 */
                PiCommand.GimbalVel[axis].Tick = 0;
            }
        }
        break;
    case PI_CMD_GIMBAL_HOME:
        if (dLen >= 1)
            PiCommand.HomeMask = d[0] & 0x07;
        break;
    case PI_CMD_POSE:               /* x y yaw：int32 mm, int32 mm, int16 mrad */
        if (dLen >= 10) {
            int32_t x = (int32_t)((d[0] << 24) | (d[1] << 16) | (d[2] << 8) | d[3]);
            int32_t y = (int32_t)((d[4] << 24) | (d[5] << 16) | (d[6] << 8) | d[7]);
            int16_t t = (int16_t)((d[8] << 8) | d[9]);
            PiCommand.PosTarget.X = x / 1000.f;                  /* mm → m */
            PiCommand.PosTarget.Y = y / 1000.f;
            PiCommand.TargetYaw = t * (180.f / (PI * 1000.f));   /* mrad → ° */
            PiCommand.TargetYawValid = 1;
            PiCommand.PosTarget.Valid = 1;
        }
        break;
    case PI_CMD_GRIPPER:
        if (dLen >= 1)
            PiCommand.Gripper = d[0];
        break;
    case PI_CMD_PLATE:
        if (dLen >= 1)
            PiCommand.Plate = d[0];
        break;
    case PI_CMD_CAM:
        if (dLen >= 1)
            PiCommand.Cam = d[0];
        break;
    case PI_CMD_LED:                /* 补光灯亮度 %（0~100），仅使能时执行（PWM 无闭环） */
        if (dLen >= 1 && PiCommand.Enabled) {
            PiCommand.Led = d[0];
            Led_SetBrightness((float)d[0]);
        }
        break;
    case PI_CMD_ENABLE:             /* 整车单一使能状态：en 直接决定使能/失能 */
        if (dLen >= 1)
            PiCommand.Enabled = d[0] ? 1 : 0;
        break;
    case PI_CMD_GIMBAL_VEL:         /* axis(u8) + deg/s(int16 大端，整数 °/s) */
        if (dLen >= 3) {
            uint8_t axis = d[0];
            if (axis < 3) {
                int16_t spd = (int16_t)((d[1] << 8) | d[2]);
                PiCommand.GimbalVel[axis].DegPerSec = (float)spd;
                PiCommand.GimbalVel[axis].Tick = HAL_GetTick();   /* 刷新时间戳 = 该轴的新速度 */
            }
        }
        break;
    case PI_CMD_GIMBAL_LIMIT:       /* axis(u8) + min(int16 ×0.1°) + max(int16 ×0.1°) */
        if (dLen >= 5) {
            uint8_t axis = d[0];
            if (axis < 3) {
                int16_t lo = (int16_t)((d[1] << 8) | d[2]);
                int16_t hi = (int16_t)((d[3] << 8) | d[4]);
                if (lo > hi) {          /* 示教时先碰到哪一端都行，这里统一成 min < max */
                    int16_t tmp = lo;
                    lo = hi;
                    hi = tmp;
                }
                PiCommand.GimbalLimit[axis].Min = lo / 10.f;
                PiCommand.GimbalLimit[axis].Max = hi / 10.f;
                PiCommand.GimbalLimit[axis].Valid = 1;
            }
        }
        break;
    case PI_CMD_GIMBAL_RELEASE:     /* en(u8)：1=松轴（电机控制板失能） 0=恢复 */
        if (dLen >= 1)
            PiCommand.GimbalRelease = d[0] ? 1 : 0;
        break;
    case PI_CMD_HEARTBEAT:
        break;
    default:
        return;
    }

    PiCommand.LastOnlineTick = HAL_GetTick();
}

void PiComm_Init(void)
{
    /* 只清状态；UART 接收由 usart.c 的 RX 中断统一管理 */
    memset(&PiCommand, 0, sizeof(PiCommand));
    /* 上电默认「失能」：树莓派端 _enabled 也初始为 False，两边状态才对得上。
       原来默认使能，Pi 侧却以为自己是失能的，要按两次 start 才真正切换一次，
       而且 407 单独上电（没接树莓派）时电机会直接带电锁轴。 */
    PiCommand.Enabled = 0;
    memset(&s_RxWired, 0, sizeof(s_RxWired));
    memset(&s_RxBle, 0, sizeof(s_RxBle));
}

/* 单条链路的字节级帧同步（中断上下文） */
static void PiComm_RxByteCtx(PiCommRxCtx_t* ctx, uint8_t data)
{
    /* 帧头同步 */
    if (ctx->Cnt == 0) {
        if (data != 0xAA)
            return;
    } else if (ctx->Cnt == 1) {
        if (data != 0x55) {
            ctx->Cnt = (data == 0xAA) ? 1 : 0;
            return;
        }
    }

    ctx->Buf[ctx->Cnt++] = data;

    if (ctx->Cnt == 3) {                /* LEN */
        ctx->Len = data;
        if (ctx->Len + 4 > PI_RX_BUF_SIZE) {    /* 帧总长 = SOF(2) + LEN(1) + SEQ(1) + LEN，异常则重新同步 */
            ctx->Cnt = 0;
            return;
        }
    }

    if (ctx->Cnt == ctx->Len + 4) {     /* 收满：SOF(2) + LEN + SEQ + LEN 字节 */
        uint16_t crc = (uint16_t)(ctx->Buf[ctx->Cnt - 1] << 8) | ctx->Buf[ctx->Cnt - 2];
        if (PiComm_CRC16(&ctx->Buf[2], ctx->Len) == crc)
            PiComm_ParseFrame(ctx->Buf, ctx->Len);
        ctx->Cnt = 0;
    }
}

void PiComm_RxByte(uint8_t data)
{
    PiComm_RxByteCtx(&s_RxWired, data);
}

void PiComm_RxBleByte(uint8_t data)
{
    PiComm_RxByteCtx(&s_RxBle, data);
}

int PiComm_IsOnline(void)
{
    return (HAL_GetTick() - PiCommand.LastOnlineTick) < PI_CMD_TIMEOUT_MS;
}

/* ============ 遥测 ============ */
void PiComm_SendOdometry(float x_mm, float y_mm, float theta_mrad)
{
    int32_t x = (int32_t)x_mm;
    int32_t y = (int32_t)y_mm;
    int16_t t = (int16_t)theta_mrad;

    uint8_t d[10] = {
        x >> 24, x >> 16, x >> 8, x,
        y >> 24, y >> 16, y >> 8, y,
        t >> 8, t
    };
    PiComm_SendFrame(PI_RPT_ODOM, d, sizeof(d));
}

void PiComm_SendStatus(uint8_t state, uint8_t err, uint8_t motorOnlineMask)
{
    uint8_t d[3] = { state, err, motorOnlineMask };
    PiComm_SendFrame(PI_RPT_STATUS, d, sizeof(d));
}

/* 角度 → int16（×0.1°），带限幅：限位示教时人手拨动不会越界，但树莓派收到的
   值必须始终落在 int16 里，否则强转会把 3276.8° 变成负数、示教点直接错乱。 */
static int16_t PiComm_DegToInt16x10(float deg)
{
    float v = deg * 10.0f;
    if (v >  32767.0f) v =  32767.0f;
    if (v < -32768.0f) v = -32768.0f;
    return (int16_t)v;
}

void PiComm_SendGimbalPose(float yawDeg, float liftDeg, float extendDeg)
{
    int16_t y = PiComm_DegToInt16x10(yawDeg);
    int16_t l = PiComm_DegToInt16x10(liftDeg);
    int16_t e = PiComm_DegToInt16x10(extendDeg);

    uint8_t d[6] = { y >> 8, y, l >> 8, l, e >> 8, e };
    PiComm_SendFrame(PI_RPT_GIMBAL, d, sizeof(d));
}
