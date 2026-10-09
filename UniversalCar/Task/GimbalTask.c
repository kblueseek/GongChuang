/**
 * @attention   采用UTF-8字符集编码
 * @brief       云台控制任务
 * @details     三轴 ZDT 电机内部闭环，STM32 直接下发绝对位置指令（周期重发，幂等）。
 *              回零请求（PiCommand.HomeMask）在本任务执行，执行后清位。
 *              夹爪/摄像头舵机直接跟随树莓派指令。
 */

#include "header.h"

#define GIMBAL_TASK_UPDATE_TICK 10

Gimbal_t Gimbal;

#if GIMBAL_MOTOR_INSTALLED
/* 云台三轴的软限位（单位 °，见 RobotConfig.h） */
static const float s_MinDeg[3] = { GIMBAL_YAW_MIN,   GIMBAL_LIFT_MIN,   GIMBAL_EXTEND_MIN };
static const float s_MaxDeg[3] = { GIMBAL_YAW_MAX,   GIMBAL_LIFT_MAX,   GIMBAL_EXTEND_MAX };

/* 目标角度限幅后写入电机 */
static void Gimbal_SetAxisTarget(ZdtMotor_t* motor, uint8_t axis, float deg)
{
    motor->Position.Set = ConstrainF(deg, s_MaxDeg[axis], s_MinDeg[axis]);
    ZdtMotor_SetPosition(motor, motor->Position.Set, ZDT_POS_ABS);
}

/* 云台三轴停车 */
static void Gimbal_Disable(void)
{
    ZdtCan_Stop(ZDT_MOTOR_YAW);
    ZdtCan_Stop(ZDT_MOTOR_LIFT);
    ZdtCan_Stop(ZDT_MOTOR_EXTEND);
}
#endif

void GimbalTask(void const* argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();

#if GIMBAL_MOTOR_INSTALLED
    /* ---- 初始化三轴电机（地址 5~7） ---- */
    ZdtMotor_Init(&Gimbal.Yaw.Motor,    ZDT_MOTOR_YAW);
    ZdtMotor_Init(&Gimbal.Lift.Motor,   ZDT_MOTOR_LIFT);
    ZdtMotor_Init(&Gimbal.Extend.Motor, ZDT_MOTOR_EXTEND);
#endif

    /* ---- 初始化夹爪 / 摄像头舵机 ---- */
    Servo_Init(&Gimbal.GripperServo, SERVO_GRIPPER_CH, SERVO_PULSE_MIN, SERVO_PULSE_MAX, SERVO_GRIPPER_INIT);
    Servo_Init(&Gimbal.CamServo,     SERVO_CAM_CH,     SERVO_PULSE_MIN, SERVO_PULSE_MAX, SERVO_CAM_INIT);

    Gimbal.isEnable = 0;

    while (1)
    {
        osDelayUntil(&PreviousWakeTime, GIMBAL_TASK_UPDATE_TICK);

#if GIMBAL_MOTOR_INSTALLED
        /* ---- 使能判定：bit4-6（电机5-7）全置位 且 有指令源（树莓派/蓝牙）在线 且 无急停 ---- */
        uint8_t GimbalEnable = (PiCommand.EnableMask & 0x70) == 0x70;
        uint8_t GimbalCmdOnline = PiComm_IsOnline() || BLE_IsOnline();

        if (GimbalEnable && GimbalCmdOnline && !PiCommand.Estop)
        {
            if (!Gimbal.isEnable) {
                ZdtCan_Enable(Gimbal.Yaw.Motor.Addr,    1);
                ZdtCan_Enable(Gimbal.Lift.Motor.Addr,   1);
                ZdtCan_Enable(Gimbal.Extend.Motor.Addr, 1);
            }
            Gimbal.isEnable = 1;
        }
        else
        {
            if (Gimbal.isEnable) {
                ZdtCan_Enable(Gimbal.Yaw.Motor.Addr,    0);
                ZdtCan_Enable(Gimbal.Lift.Motor.Addr,   0);
                ZdtCan_Enable(Gimbal.Extend.Motor.Addr, 0);
            }
            Gimbal.isEnable = 0;
        }

        if (!Gimbal.isEnable)
        {
            Gimbal_Disable();
        }
        else
        {
            /* ---- 回零请求（执行后清位，避免重复触发） ---- */
            if (PiCommand.HomeMask & (1 << 0)) { ZdtCan_Home(ZDT_MOTOR_YAW,    ZDT_HOME_NEAR); PiCommand.HomeMask &= ~(1 << 0); }
            if (PiCommand.HomeMask & (1 << 1)) { ZdtCan_Home(ZDT_MOTOR_LIFT,   ZDT_HOME_LIMIT); PiCommand.HomeMask &= ~(1 << 1); }
            if (PiCommand.HomeMask & (1 << 2)) { ZdtCan_Home(ZDT_MOTOR_EXTEND, ZDT_HOME_LIMIT); PiCommand.HomeMask &= ~(1 << 2); }

            /* ---- 三轴目标位置（有新指令才更新目标；位置命令周期重发，幂等） ---- */
            if (PiCommand.GimbalTarget[0].Mask) {
                Gimbal_SetAxisTarget(&Gimbal.Yaw.Motor,    0, PiCommand.GimbalTarget[0].Deg);
                PiCommand.GimbalTarget[0].Mask = 0;
            }
            if (PiCommand.GimbalTarget[1].Mask) {
                Gimbal_SetAxisTarget(&Gimbal.Lift.Motor,   1, PiCommand.GimbalTarget[1].Deg);
                PiCommand.GimbalTarget[1].Mask = 0;
            }
            if (PiCommand.GimbalTarget[2].Mask) {
                Gimbal_SetAxisTarget(&Gimbal.Extend.Motor, 2, PiCommand.GimbalTarget[2].Deg);
                PiCommand.GimbalTarget[2].Mask = 0;
            }

            ZdtMotor_SetPosition(&Gimbal.Yaw.Motor,    Gimbal.Yaw.Motor.Position.Set,    ZDT_POS_ABS);
            ZdtMotor_SetPosition(&Gimbal.Lift.Motor,   Gimbal.Lift.Motor.Position.Set,   ZDT_POS_ABS);
            ZdtMotor_SetPosition(&Gimbal.Extend.Motor, Gimbal.Extend.Motor.Position.Set, ZDT_POS_ABS);
        }
#endif

        /* ---- 夹爪 / 摄像头舵机 ---- */
        Servo_SetAngle(&Gimbal.GripperServo, PiCommand.Gripper);
        Servo_SetAngle(&Gimbal.CamServo,     PiCommand.Cam);
    }
}
