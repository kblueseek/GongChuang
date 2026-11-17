/**
 * @attention   采用UTF-8字符集编码
 * @brief       OPS 里程计（Odom1_lite）驱动实现
 * @details     接收链路：UART4（PC10/PC11）+ DMA1 Stream2 Channel4 循环搬运，
 *              每帧之间靠 IDLE 中断切出来交给解析器。921600 下一个字节只有
 *              10.8µs，逐字节中断会把 CPU 吃掉一大块，所以不用 Receive_IT。
 *
 *              解析器按 PAYLOAD_LEN+8 定长取帧，坏 CRC 只丢 1 字节重新同步，
 *              不扫 A5 当帧尾（载荷里可能出现 A5）。
 *
 *              上电配置：OPS 需要静止 2s 校准，所以等上电 3s 且收到第一帧之后
 *              才依次下发 SEND_TIME / POS_CLEAR / ENC_DIAMETER，每条间隔 100ms
 *              防粘包（手册要求 ≥5~20ms）。
 */

#include "header.h"

#if LORA_DOWNLINK_ENABLE
#error "OPS 占用 UART4，与 LoRa 遥测下行冲突：请把 RobotConfig.h 的 LORA_DOWNLINK_ENABLE 置 0"
#endif

#define OPS_RX_DMA_LEN      256u        /* DMA 循环缓冲，2 帧 38B 绰绰有余 */
#define OPS_RX_BUF_LEN      512u        /* 解析缓冲（半帧最多留一帧多一点） */
#define OPS_CMD_FRAME_LEN   15u
#define OPS_MAX_FRAME_LEN   128u
#define OPS_ODOM_PAYLOAD    30u
#define OPS_CMD_GAP_MS      100u        /* 配置命令之间的间隔，防粘包 */
#define OPS_BOOT_WAIT_MS    3000u       /* 等 OPS 完成上电静止校准 */

Ops_t Ops;

/* ---- 接收链路 ---- */
static DMA_HandleTypeDef s_HdmaRx;
static uint8_t  s_DmaBuf[OPS_RX_DMA_LEN];
static uint16_t s_DmaLastPos;           /* 上一次 IDLE 时 DMA 写到的位置 */
static uint8_t  s_RxArmed;              /* DMA 接收是否已挂上 */

/* ---- 解析缓冲 ---- */
static uint8_t  s_RxBuf[OPS_RX_BUF_LEN];
static uint16_t s_RxLen;

/* ---- 上电配置流程 ---- */
static uint16_t s_CmdSeq;
static uint32_t s_BootTick;
static uint32_t s_CfgTick;
static uint8_t  s_CfgStep;

/* ---- 航向对齐 ---- */
static uint8_t s_OffsetLatched;

/* ============ 小工具 ============ */

static uint16_t Ops_Crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFu;
    uint16_t i;
    uint8_t bit;

    for (i = 0u; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (bit = 0u; bit < 8u; bit++) {
            if (crc & 0x8000u)
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            else
                crc <<= 1;
        }
    }
    return crc;
}

static uint16_t Ops_GetU16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int32_t Ops_GetI32(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0] |
                     ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) |
                     ((uint32_t)p[3] << 24));
}

static void Ops_PutU16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static void Ops_PutU32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static void Ops_DropPrefix(uint16_t count)
{
    if (count >= s_RxLen) {
        s_RxLen = 0u;
        return;
    }
    memmove(s_RxBuf, &s_RxBuf[count], (size_t)(s_RxLen - count));
    s_RxLen = (uint16_t)(s_RxLen - count);
}

/* ============ 帧解析 ============ */

static void Ops_ParseOdometry(const uint8_t *frame)
{
    Ops.Sequence   = Ops_GetU16(&frame[5]);
    Ops.RawX_mm    = (float)Ops_GetI32(&frame[7])  / 100.0f;
    Ops.RawY_mm    = (float)Ops_GetI32(&frame[11]) / 100.0f;
    Ops.RawYaw_deg = (float)Ops_GetI32(&frame[15]) / 1000.0f;
    Ops.Vx_mms     = (float)Ops_GetI32(&frame[19]) / 1000.0f;
    Ops.Vy_mms     = (float)Ops_GetI32(&frame[23]) / 1000.0f;
    Ops.Ax_mms2    = (float)Ops_GetI32(&frame[27]) / 1000.0f;
    Ops.Ay_mms2    = (float)Ops_GetI32(&frame[31]) / 1000.0f;

    Ops.LastFrameTick = HAL_GetTick();
    Ops.FrameCount++;
}

static void Ops_ParseAck(const uint8_t *frame)
{
    Ops.AckCount++;
    Ops.LastAckCmd    = frame[7];
    Ops.LastAckStatus = frame[8];
    Ops.LastAckValue  = (uint32_t)Ops_GetI32(&frame[9]);
}

static uint8_t Ops_ParseFrame(const uint8_t *frame, uint16_t length)
{
    uint8_t  type;
    uint8_t  payloadLen;
    uint16_t crcCalc;
    uint16_t crcRecv;

    (void)length;

    if ((frame[0] != OPS_FRAME_HEAD_0) || (frame[1] != OPS_FRAME_HEAD_1))
        return 0u;
    if (frame[2] != OPS_PROTOCOL_VERSION)
        return 0u;

    type       = frame[3];
    payloadLen = frame[4];

    crcRecv = Ops_GetU16(&frame[5 + payloadLen]);
    crcCalc = Ops_Crc16(&frame[2], (uint16_t)(3u + payloadLen));
    if (crcRecv != crcCalc)
        return 0u;

    if (frame[7 + payloadLen] != OPS_FRAME_TAIL)
        return 0u;

    if ((type == OPS_TYPE_ODOMETRY) && (payloadLen == OPS_ODOM_PAYLOAD))
        Ops_ParseOdometry(frame);
    else if (type == OPS_TYPE_ACK)
        Ops_ParseAck(frame);

    return 1u;
}

void Ops_Receive(const uint8_t *data, uint16_t length)
{
    uint16_t i;
    uint16_t frameLen;
    uint8_t  payloadLen;

    if ((data == NULL) || (length == 0u))
        return;

    if (length >= OPS_RX_BUF_LEN) {             /* 一次给太多，只留最后一截 */
        data += (length - (OPS_RX_BUF_LEN - 1u));
        length = OPS_RX_BUF_LEN - 1u;
    }
    if ((uint16_t)(s_RxLen + length) > OPS_RX_BUF_LEN) {
        Ops_DropPrefix((uint16_t)(s_RxLen + length - OPS_RX_BUF_LEN));
    }

    memcpy(&s_RxBuf[s_RxLen], data, (size_t)length);
    s_RxLen = (uint16_t)(s_RxLen + length);

    for (;;) {
        if (s_RxLen < 5u)
            break;

        /* 找帧头 AA 55 */
        for (i = 0u; i + 1u < s_RxLen; i++) {
            if ((s_RxBuf[i] == OPS_FRAME_HEAD_0) && (s_RxBuf[i + 1u] == OPS_FRAME_HEAD_1))
                break;
        }
        if (i + 1u >= s_RxLen) {
            /* 没找到：只保留末尾可能是半个帧头的 AA */
            if (s_RxBuf[s_RxLen - 1u] == OPS_FRAME_HEAD_0) {
                s_RxBuf[0] = OPS_FRAME_HEAD_0;
                s_RxLen = 1u;
            } else {
                s_RxLen = 0u;
            }
            Ops.SyncErrorCount++;
            break;
        }
        if (i != 0u) {
            Ops.SyncErrorCount += i;
            Ops_DropPrefix(i);
        }
        if (s_RxLen < 5u)
            break;

        payloadLen = s_RxBuf[4];
        frameLen = (uint16_t)payloadLen + 8u;
        if ((frameLen < 8u) || (frameLen > OPS_MAX_FRAME_LEN)) {
            Ops.SyncErrorCount++;
            Ops_DropPrefix(1u);
            continue;
        }
        if (s_RxLen < frameLen)
            break;                              /* 半帧，等后续字节 */

        if (Ops_ParseFrame(s_RxBuf, frameLen)) {
            Ops_DropPrefix(frameLen);
        } else {
            Ops.CrcErrorCount++;
            Ops_DropPrefix(1u);                 /* 只丢 1 字节重新同步 */
        }
    }
}

/* ============ 接收链路初始化 ============ */

uint8_t Ops_SendConfig(uint8_t cmd, uint32_t value)
{
    uint8_t  frame[OPS_CMD_FRAME_LEN];
    uint16_t crc;

    switch (cmd) {
    case OPS_CMD_SEND_TIME:
    case OPS_CMD_MCU_REBOOT:
    case OPS_CMD_POS_CLEAR:
    case OPS_CMD_ENC_DIAMETER:
    case OPS_CMD_SET_POS_X:
    case OPS_CMD_SET_POS_Y:
    case OPS_CMD_SET_YAW:
        break;
    default:
        return 0u;
    }

    frame[0] = OPS_FRAME_HEAD_0;
    frame[1] = OPS_FRAME_HEAD_1;
    frame[2] = OPS_PROTOCOL_VERSION;
    frame[3] = OPS_TYPE_COMMAND;
    frame[4] = (uint8_t)(OPS_CMD_FRAME_LEN - 8u);   /* 7 字节载荷 */
    Ops_PutU16(&frame[5], s_CmdSeq++);
    frame[7] = cmd;
    Ops_PutU32(&frame[8], value);
    crc = Ops_Crc16(&frame[2], 10u);
    Ops_PutU16(&frame[12], crc);
    frame[14] = OPS_FRAME_TAIL;

    /* 15B @921600 ≈ 180µs。只在初始化阶段发，用阻塞发送最简单可靠。 */
    HAL_UART_Transmit(&huart4, frame, OPS_CMD_FRAME_LEN, 20);
    return 1u;
}

static void Ops_DmaInit(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    s_HdmaRx.Instance                 = DMA1_Stream2;
    s_HdmaRx.Init.Channel             = DMA_CHANNEL_4;
    s_HdmaRx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    s_HdmaRx.Init.PeriphInc           = DMA_PINC_DISABLE;
    s_HdmaRx.Init.MemInc              = DMA_MINC_ENABLE;
    s_HdmaRx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    s_HdmaRx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    s_HdmaRx.Init.Mode                = DMA_CIRCULAR;
    s_HdmaRx.Init.Priority            = DMA_PRIORITY_HIGH;
    s_HdmaRx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&s_HdmaRx);

    __HAL_LINKDMA(&huart4, hdmarx, s_HdmaRx);

    HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
}

void Ops_Init(void)
{
    memset(&Ops, 0, sizeof(Ops));
    s_RxLen = 0u;
    s_DmaLastPos = 0u;
    s_CmdSeq = 0u;
    s_CfgStep = 0u;
    s_CfgTick = 0u;
    s_OffsetLatched = 0u;
    s_RxArmed = 0u;
    s_BootTick = HAL_GetTick();

    /* CubeMX 里 UART4 是按 LoRa 115200 生成的，这里改成 OPS 的 921600。
       APB1=42MHz、16 倍过采样时 921600 的分频误差约 1%，用 8 倍过采样可以
       压到 0.2%，所以 OverSampling 一起改。MspInit（GPIO/时钟/NVIC）已经
       由 MX_UART4_Init 做过，这里只改参数，不会重复初始化引脚。 */
    huart4.Init.BaudRate     = OPS_UART_BAUD;
    huart4.Init.OverSampling = UART_OVERSAMPLING_8;
    if (HAL_UART_Init(&huart4) != HAL_OK) {
        Ops.Online = 0u;
        return;
    }

    Ops_DmaInit();

    /* 先清一次 IDLE，避免使能后立刻误触发 */
    __HAL_UART_CLEAR_IDLEFLAG(&huart4);
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);

    if (HAL_UART_Receive_DMA(&huart4, s_DmaBuf, OPS_RX_DMA_LEN) == HAL_OK) {
        s_RxArmed = 1u;
        /* 循环接收不需要半满/全满中断，关掉减少中断负载（错误中断保留） */
        __HAL_DMA_DISABLE_IT(&s_HdmaRx, DMA_IT_HT | DMA_IT_TC);
    }
}

/* ============ 中断 ============ */

void Ops_IRQHandler(void)
{
    uint16_t pos;
    uint16_t len;

    if (__HAL_UART_GET_IT_SOURCE(&huart4, UART_IT_IDLE) == RESET)
        return;
    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_IDLE) == RESET)
        return;

    /* 清 IDLE（读 SR 再读 DR 的固定写法） */
    __HAL_UART_CLEAR_IDLEFLAG(&huart4);

    if (!s_RxArmed)
        return;

    pos = (uint16_t)(OPS_RX_DMA_LEN - __HAL_DMA_GET_COUNTER(&s_HdmaRx));
    if (pos == s_DmaLastPos)
        return;                                  /* 这一段时间没有新字节 */

    if (pos > s_DmaLastPos) {
        len = (uint16_t)(pos - s_DmaLastPos);
        Ops_Receive(&s_DmaBuf[s_DmaLastPos], len);
    } else {
        /* 环形回绕：先喂到缓冲末尾，再从 0 喂到 pos */
        len = (uint16_t)(OPS_RX_DMA_LEN - s_DmaLastPos);
        Ops_Receive(&s_DmaBuf[s_DmaLastPos], len);
        if (pos > 0u)
            Ops_Receive(&s_DmaBuf[0], pos);
    }
    s_DmaLastPos = pos;
}

/* DMA1 Stream2 中断：正常循环接收用不到，留一个空实现防止弱定义跑飞 */
void DMA1_Stream2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&s_HdmaRx);
}

void Ops_ErrorCallback(void)
{
    /* 921600 下偶发溢出（ORE）会让 HAL 把 DMA 接收停掉，而且不会再自己恢复。
       清标志 + 标记未挂，下一拍 Ops_Poll 重新 HAL_UART_Receive_DMA。 */
    __HAL_UART_CLEAR_OREFLAG(&huart4);
    huart4.ErrorCode = HAL_UART_ERROR_NONE;
    s_RxArmed = 0u;
}

/* ============ 周期维护 ============ */

static void Ops_ConfigTick(uint32_t now)
{
    if (Ops.CfgDone)
        return;
    if (Ops.LastFrameTick == 0u)                 /* 还没收到帧，先别发命令 */
        return;
    if ((uint32_t)(now - s_BootTick) < OPS_BOOT_WAIT_MS)
        return;                                  /* 等 OPS 静止校准结束 */

    if (s_CfgTick != 0u && (uint32_t)(now - s_CfgTick) < OPS_CMD_GAP_MS)
        return;

    switch (s_CfgStep) {
    case 0u:
        if (Ops_SendConfig(OPS_CMD_SEND_TIME, OPS_SEND_PERIOD_MS)) {
            Ops.SendPeriodMs = OPS_SEND_PERIOD_MS;
            s_CfgStep = 1u;
        }
        break;
    case 1u:
        /* 当前位置设成起点：和底盘里程计 0,0 对齐 */
        if (Ops_SendConfig(OPS_CMD_POS_CLEAR, 0u))
            s_CfgStep = 2u;
        break;
    case 2u:
        if (Ops_SendConfig(OPS_CMD_ENC_DIAMETER, OPS_WHEEL_DIAMETER_MM)) {
            Ops.WheelDiameterMm = OPS_WHEEL_DIAMETER_MM;
            s_CfgStep = 3u;
        }
        break;
    default:
        Ops.CfgDone = 1u;
        break;
    }
    s_CfgTick = now;
}

void Ops_Poll(uint32_t now)
{
    /* 接收链路万一没挂上（初始化失败/波特率改错），这里补挂一次 */
    if (!s_RxArmed) {
        if (HAL_UART_Receive_DMA(&huart4, s_DmaBuf, OPS_RX_DMA_LEN) == HAL_OK) {
            s_RxArmed = 1u;
            s_DmaLastPos = 0u;          /* DMA 从头开始写，索引同步归零 */
            __HAL_DMA_DISABLE_IT(&s_HdmaRx, DMA_IT_HT | DMA_IT_TC);
            __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
        }
    }

    Ops.Online = (uint8_t)((Ops.LastFrameTick != 0u) &&
                           ((uint32_t)(now - Ops.LastFrameTick) < OPS_TIMEOUT_MS));

    if (!Ops.Online) {
        Ops.PosValid = 0u;
        return;
    }

    Ops_ConfigTick(now);

    /* 航向对齐：把 OPS 的世界系旋到 IMU 的世界系。
       两者各自以自己的上电姿态为零点，差一个固定角；位置要喂给底盘的
       位置环（该环用 IMU 航向做机体系↔世界系旋转），不对齐就会跑偏。

       ⚠ 必须等上电配置跑完（POS_CLEAR 已发）再锁：POS_CLEAR 有可能把
       yaw 一起清零（手册没写死），在那之前锁的偏移量会整体差一个角度。
       IMU 还没出数据时也先不锁，下一拍再试。 */
    if (!s_OffsetLatched && Ops.CfgDone && (Imu.LastOnlineTick != 0u)) {
        Ops.YawOffsetDeg = Imu.Yaw - OPS_YAW_SIGN * Ops.RawYaw_deg;
        s_OffsetLatched = 1u;
    }

    Ops.PosValid = s_OffsetLatched;
}

uint8_t Ops_GetPosition(float *x_m, float *y_m)
{
    float px;
    float py;
    float ang;
    float c;
    float s;

    if ((x_m == NULL) || (y_m == NULL))
        return 0u;
    if (!Ops.PosValid)
        return 0u;

    px = OPS_X_SIGN * Ops.RawX_mm * 0.001f;
    py = OPS_Y_SIGN * Ops.RawY_mm * 0.001f;

#if OPS_SWAP_XY
    {
        float t = px;
        px = py;
        py = t;
    }
#endif

    ang = Ops.YawOffsetDeg * PI / 180.0f;
    c = cosf(ang);
    s = sinf(ang);

    *x_m = px * c - py * s;
    *y_m = px * s + py * c;
    return 1u;
}

uint8_t Ops_IsOnline(void)
{
    return Ops.Online;
}
