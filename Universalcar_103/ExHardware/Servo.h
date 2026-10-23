#ifndef _SERVO_H_
#define _SERVO_H_

/**
 * @attention   采用UTF-8字符集编码
 * @brief       舵机 PWM 驱动（TIM3，50Hz，均 180° 行程）
 * @details     三个舵机共用 TIM3 时基（PSC=83 → 1us/tick，ARR=19999 → 20ms）：
 *                CH1 = PA6 → 夹爪
 *                CH2 = PA7 → 载物盘
 *                CH3 = PB0 → 摄像头
 *              脉宽 500~2500us 线性对应 0~180°
 */

#include "stdint.h"

/// @brief 舵机句柄
typedef struct
{
    uint32_t Channel;       //TIM 通道（TIM_CHANNEL_1/2/3）
    float Travel;           //行程角（180°）
    uint16_t PulseMin;      //0° 脉宽 us
    uint16_t PulseMax;      //满行程脉宽 us
    float Angle;            //当前角度 °
} Servo_t;

/// @brief 注册舵机并启动对应 PWM 通道（上电初始化时调用一次）
void Servo_Init(Servo_t* servo, uint32_t channel, uint16_t pulseMin, uint16_t pulseMax, float initAngle);

/// @brief 设置目标角度（0~行程角，超范围自动限幅）
void Servo_SetAngle(Servo_t* servo, float angle);

/// @brief 读取当前角度
float Servo_GetAngle(Servo_t* servo);

#endif
