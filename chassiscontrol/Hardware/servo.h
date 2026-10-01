/**
 * @file    servo.h
 * @brief   舵机 PWM 驱动 (TIM3, 50Hz) + 蓝牙滑杆控制
 * @note    硬件映射:
 *            TIM3_CH1 = PA6 → servo1 (180° 舵机)
 *            TIM3_CH2 = PA7 → servo2 (180° 舵机)
 *
 *          TIM3 时基: 预分频后 1 个计数 = 1us, Period = 19999 → 20ms (50Hz)
 *          即脉宽 500~2500us 对应 0~180°
 *
 *          蓝牙滑杆 (小程序, 范围 0~1):
 *            [slider,servo1,值] 或 [s,servo1,值] → servo1: 0~1 映射到 0~180°
 *            [slider,servo2,值] 或 [s,servo2,值] → servo2: 0~1 映射到 0~180°
 */
#ifndef __SERVO_H
#define __SERVO_H

#include <stdint.h>

/* ============ 舵机编号 ============ */
typedef enum {
    SERVO_1 = 0,    /* TIM3_CH1 / PA6 */
    SERVO_2 = 1,    /* TIM3_CH2 / PA7 */
    SERVO_NUM       /* 舵机数量 */
} Servo_Id;

/* ============ 外部接口 ============ */

/* 启动两路 PWM 并使舵机回到初始角度 (上电调用一次) */
void  Servo_Init(void);

/* 直接给角度 (0~180°), 超范围自动限幅 */
void  Servo_SetAngle(Servo_Id id, float angle);

/* 读取当前角度 */
float Servo_GetAngle(Servo_Id id);

/* 蓝牙滑杆入口: 按名称匹配 servo1 / servo2, 不匹配则忽略.
 * 在 ControllerTask.c 的 BLE_OnSlider 里调用即可. */
void  Servo_HandleSlider(const char *name, float value);

#endif /* __SERVO_H */
