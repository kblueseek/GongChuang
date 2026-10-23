#ifndef _JY901S_H_
#define _JY901S_H_

/**
 * @attention   采用UTF-8字符集编码
 * @brief       JY901S 九轴陀螺仪模块驱动（USART3 / PB10 PB11）
 * @details     协议：每帧 11 字节 = 0x55 + 0x5X(类型) + 8 数据 + SUM(校验和)
 *               0x51 加速度（g）  0x52 角速度（°/s）  0x53 角度（°）
 *              模块内置姿态解算，0x53 帧直接输出欧拉角。
 *              注：USART2 让给蓝牙遥控，故 JY901S 走 USART3。
 */

#include "stdint.h"

/// @brief 陀螺仪数据（全局维护）
typedef struct
{
    float Angle[3];     //欧拉角 °（0:Roll 1:Pitch 2:Yaw，与模块输出顺序一致）
    float Gyro[3];      //角速度 °/s
    float Acc[3];       //加速度 g

    uint32_t LastOnlineTick;    //最后一次收帧时刻
} JY901S_t;

extern JY901S_t JY901S;

/// @brief 启动 UART 单字节中断接收（main 或任务初始化时调用一次）
void JY901S_Init(void);

/// @brief 字节解析（由 usart.c 的 HAL_UART_RxCpltCallback 调用）
void JY901S_RxByte(uint8_t data);

#endif
