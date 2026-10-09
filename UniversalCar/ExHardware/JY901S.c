/**
 * @attention   采用UTF-8字符集编码
 * @brief       JY901S 九轴陀螺仪驱动实现
 * @details     逐字节状态机解析 0x55 帧头 + 类型 + 数据 + 校验和。
 *              波特率与模块一致（JY901S 默认 9600，可上位机改成 115200）。
 */

#include "header.h"

JY901S_t JY901S;

/* 接收状态机 */
static uint8_t  s_RxBuf[11];
static uint8_t  s_RxCnt = 0;
static uint8_t  s_RxType = 0;

void JY901S_Init(void)
{
    /* 只清状态；UART 接收由 usart.c 的 RX 中断统一管理（USART3） */
    memset(&JY901S, 0, sizeof(JY901S));
    s_RxCnt = 0;
}

/* 解析一帧（长度 11 字节已收满且校验通过） */
static void JY901S_ParseFrame(uint8_t type, const uint8_t* d)
{
    switch (type) {
    case 0x51:  /* 加速度，量程 ±16g，数据 /32768*16 */
        JY901S.Acc[0] = (int16_t)((d[1] << 8) | d[0]) / 32768.f * 16.f;
        JY901S.Acc[1] = (int16_t)((d[3] << 8) | d[2]) / 32768.f * 16.f;
        JY901S.Acc[2] = (int16_t)((d[5] << 8) | d[4]) / 32768.f * 16.f;
        break;
    case 0x52:  /* 角速度，量程 ±2000°/s */
        JY901S.Gyro[0] = (int16_t)((d[1] << 8) | d[0]) / 32768.f * 2000.f;
        JY901S.Gyro[1] = (int16_t)((d[3] << 8) | d[2]) / 32768.f * 2000.f;
        JY901S.Gyro[2] = (int16_t)((d[5] << 8) | d[4]) / 32768.f * 2000.f;
        break;
    case 0x53:  /* 角度，量程 ±180° */
        JY901S.Angle[0] = (int16_t)((d[1] << 8) | d[0]) / 32768.f * 180.f;
        JY901S.Angle[1] = (int16_t)((d[3] << 8) | d[2]) / 32768.f * 180.f;
        JY901S.Angle[2] = (int16_t)((d[5] << 8) | d[4]) / 32768.f * 180.f;
        break;
    default:
        return;
    }

    JY901S.LastOnlineTick = HAL_GetTick();
}

void JY901S_RxByte(uint8_t data)
{
    s_RxBuf[s_RxCnt++] = data;

    /* 非帧头则重新同步 */
    if (s_RxBuf[0] != 0x55) {
        s_RxCnt = 0;
        return;
    }

    /* 收到类型字节 */
    if (s_RxCnt == 2) {
        s_RxType = data;
    }

    /* 收满 11 字节：校验（前 10 字节求和低 8 位） */
    if (s_RxCnt >= 11) {
        uint8_t sum = 0;
        for (int i = 0; i < 10; i++)
            sum += s_RxBuf[i];
        if (sum == s_RxBuf[10])
            JY901S_ParseFrame(s_RxType, &s_RxBuf[2]);   /* 跳过 0x55 帧头 + 类型字节，d 指向数据区 */
        s_RxCnt = 0;
    }
}
