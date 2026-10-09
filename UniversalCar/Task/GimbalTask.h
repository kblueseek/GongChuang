#ifndef _GIMBAL_TASK_H_
#define _GIMBAL_TASK_H_

/**
 * @attention   采用UTF-8字符集编码
 * @brief       云台控制任务（三轴 ZDT 电机 + 夹爪/摄像头舵机）
 */

#include "stdint.h"
#include "cmsis_os.h"

#include "ZdtMotor.h"
#include "Servo.h"

/// @brief 云台实例
typedef struct
{
    struct { ZdtMotor_t Motor; } Yaw;       /* 旋转轴   ZDT 地址 5 */
    struct { ZdtMotor_t Motor; } Lift;      /* Z 升降轴 ZDT 地址 6 */
    struct { ZdtMotor_t Motor; } Extend;    /* X 伸缩轴 ZDT 地址 7 */

    Servo_t GripperServo;   /* 夹爪舵机 */
    Servo_t CamServo;       /* 摄像头舵机 */

    uint8_t isEnable;       /* 云台使能 */

    uint32_t LastOnlineTick;
} Gimbal_t;

void GimbalTask(void const* argument);

extern Gimbal_t Gimbal;

#endif
