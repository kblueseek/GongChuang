/**
 * @attention   采用UTF-8字符集编码
 * @brief       树莓派通信协议实现
 */

#include "header.h"

PiCommand_t PiCommand;

/* 接收状态机 */
#define PI_RX_BUF_SIZE  64
static uint8_t  s_RxBuf[PI_RX_BUF_SIZE];
static uint8_t  s_RxCnt = 0;
static uint8_t  s_RxLen = 0;

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
static void PiComm_SendFrame(uint8_t cmd, const uint8_t* data, uint8_t dataLen)
{
    static uint8_t s_Seq = 0;
    uint8_t frame[PI_RX_BUF_SIZE];

    frame[0] = 0xAA;
    frame[1] = 0x55;
    frame[2] = 3 + dataLen;         /* LEN = CMD + DATA + CRC */
    frame[3] = s_Seq++;
    frame[4] = cmd;
    memcpy(&frame[5], data, dataLen);

    uint16_t crc = PiComm_CRC16(&frame[2], 3 + dataLen);    /* CRC 覆盖 LEN~DATA */
    frame[5 + dataLen] = crc & 0xFF;
    frame[6 + dataLen] = crc >> 8;

    HAL_UART_Transmit(&huart6, frame, 7 + dataLen, 100);
}

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
        }
        break;
    case PI_CMD_GIMBAL_TARGET:
        if (dLen >= 5) {
            uint8_t axis = d[0];
            if (axis < 3) {
                int32_t deg = (int32_t)((d[1] << 24) | (d[2] << 16) | (d[3] << 8) | d[4]);
                PiCommand.GimbalTarget[axis].Deg = deg / 100.f;            /* ×100 → ° */
                PiCommand.GimbalTarget[axis].Mask = 1;
            }
        }
        break;
    case PI_CMD_GIMBAL_HOME:
        if (dLen >= 1)
            PiCommand.HomeMask = d[0] & 0x07;
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
    case PI_CMD_ENABLE:
        if (dLen >= 2) {
            uint8_t mask = d[1] & 0x7F;
            if (d[0])
                PiCommand.EnableMask |= mask;
            else
                PiCommand.EnableMask &= ~mask;
        }
        break;
    case PI_CMD_ESTOP:
        PiCommand.Estop = 1;
        PiCommand.Vel.Vx = PiCommand.Vel.Vy = PiCommand.Vel.Wz = 0;
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
    s_RxCnt = 0;
    s_RxLen = 0;
}

void PiComm_RxByte(uint8_t data)
{
    /* 帧头同步 */
    if (s_RxCnt == 0) {
        if (data != 0xAA)
            return;
    } else if (s_RxCnt == 1) {
        if (data != 0x55) {
            s_RxCnt = (data == 0xAA) ? 1 : 0;
            return;
        }
    }

    s_RxBuf[s_RxCnt++] = data;

    if (s_RxCnt == 3) {                 /* LEN */
        s_RxLen = data;
        if (s_RxLen + 4 > PI_RX_BUF_SIZE) {     /* 帧总长 = SOF(2) + LEN(1) + SEQ(1) + LEN，异常则重新同步 */
            s_RxCnt = 0;
            return;
        }
    }

    if (s_RxCnt == s_RxLen + 4) {       /* 收满：SOF(2) + LEN + SEQ + LEN 字节 */
        uint16_t crc = (uint16_t)(s_RxBuf[s_RxCnt - 1] << 8) | s_RxBuf[s_RxCnt - 2];
        if (PiComm_CRC16(&s_RxBuf[2], s_RxLen) == crc)
            PiComm_ParseFrame(s_RxBuf, s_RxLen);
        s_RxCnt = 0;
    }
}

int PiComm_IsOnline(void)
{
    return (HAL_GetTick() - PiCommand.LastOnlineTick) < PI_CMD_TIMEOUT_MS;
}

void PiComm_TimeoutCheck(void)
{
    if (!PiComm_IsOnline()) {
        /* 树莓派掉线：底盘停车（云台保持，防掉落） */
        PiCommand.Vel.Vx = 0;
        PiCommand.Vel.Vy = 0;
        PiCommand.Vel.Wz = 0;
    }
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
