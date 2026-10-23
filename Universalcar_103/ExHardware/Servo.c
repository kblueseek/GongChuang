/**
 * @attention   采用UTF-8字符集编码
 * @brief       舵机 PWM 驱动实现
 */

#include "header.h"

void Servo_Init(Servo_t* servo, uint32_t channel, uint16_t pulseMin, uint16_t pulseMax, float initAngle)
{
    servo->Channel = channel;
    servo->Travel = SERVO_TRAVEL;
    servo->PulseMin = pulseMin;
    servo->PulseMax = pulseMax;
    servo->Angle = 0;

    /* 先给比较值再启动输出，避免上电瞬间跳变 */
    HAL_TIM_PWM_Start(&htim3, channel);
    Servo_SetAngle(servo, initAngle);
}

void Servo_SetAngle(Servo_t* servo, float angle)
{
    /* 限幅到 0 ~ 行程角 */
    if (angle < 0.0f)           angle = 0.0f;
    if (angle > servo->Travel)  angle = servo->Travel;

    float    k     = angle / servo->Travel;    /* 0.0 ~ 1.0 */
    uint16_t pulse = (uint16_t)(servo->PulseMin + k * (servo->PulseMax - servo->PulseMin));

    __HAL_TIM_SET_COMPARE(&htim3, servo->Channel, pulse);

    servo->Angle = angle;
}

float Servo_GetAngle(Servo_t* servo)
{
    return servo->Angle;
}
