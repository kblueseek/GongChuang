#ifndef _CHASSIS_TASK_H_
#define _CHASSIS_TASK_H_

/**
 * @attention   采用UTF-8字符集编码
 * @brief       底盘控制任务（4 麦轮 + 载物盘舵机）
 */

#include "stdint.h"
#include "cmsis_os.h"

#include "ZdtMotor.h"
#include "Servo.h"
#include "UniversalPID.h"

/// @brief 底盘实例
typedef struct
{
    ZdtMotor_t Wheel[4];    /* 0:FL 1:FR 2:RL 3:RR（ZDT 地址 1~4） */

    struct
    {
        float Vx;   //前进速度 m/s
        float Vy;   //平移速度 m/s
        float Wz;   //自转角速度 rad/s（航向 PID 输出，非手动指令）
    } Move;

    struct
    {
        float X;    //世界系 X m（初始朝向为 +X）
        float Y;    //世界系 Y m（初始左移方向为 +Y）
    } Pos;          //里程计位置（编码器整数计数积分，最高精度）

    struct
    {
        float Vx;   //实际前进速度 m/s（编码器正解）
        float Vy;   //实际平移速度 m/s
        float Wz;   //实际自转角速度 rad/s
    } Vel;

    float WheelRpm[4];      //实际轮转速 RPM（编码器差分，与指令同符号约定）
    int32_t WheelLastCount[4];  //各轮上一次原始计数（差分基准）
    uint8_t OdomReady;      //里程计基准是否已用首帧编码器初始化

    PID_t HeadingPid;       //航向 PID（IMU Yaw → 自旋速度 Wz）

    PID_t PosPidX;          //位置环 X（世界系，米域：误差 m → 输出 m/s）
    PID_t PosPidY;          //位置环 Y（世界系）

    Servo_t PlateServo;     //载物盘舵机

    uint8_t isEnable;       //底盘使能（由树莓派指令 + 在线状态决定）

    uint32_t LastOnlineTick;
} Chassis_t;

void ChassisTask(void const* argument);

extern Chassis_t Chassis;

#endif
