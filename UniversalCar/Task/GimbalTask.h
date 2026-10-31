#ifndef _GIMBAL_TASK_H_
#define _GIMBAL_TASK_H_

/**
 * @attention   采用UTF-8字符集编码
 * @brief       云台控制任务（三轴纯速度控制，只计算不发送 + 夹爪/摄像头舵机）
 * @details     云台三轴和底盘轮子一样只收速度（F6），没有任何位置/回零/置零指令。
 *              全总线只允许底盘任务发帧：云台任务把三轴目标转速写进 GimbalTx，
 *              底盘任务在 4 轮 0xAA 广播之后，用 Gimbal_SendAxisSpeedFrames()
 *              给三轴各发一条 F6 单机帧（不放广播里，见该函数注释）。
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

    uint8_t isEnable;       /* 云台使能（速度帧有效） */
    uint8_t isReleased;     /* 松轴中：电机控制板已失能，人手可以拨动（限位示教） */

    uint32_t LastOnlineTick;
} Gimbal_t;

/* ============ 云台 -> 底盘：目标转速 + 待发的一次性配置请求 ============
   为什么不让云台任务自己发帧：CAN1 只有 3 个发送邮箱，底盘（每 10ms 一条广播 +
   三条云台单机帧）和云台同时发会互相把对方挤掉。广播丢一包整帧作废（底盘卡住），
   单机帧丢一帧就是某个轴没反应（X 轴"一点反应都没有"）。全总线只留一个发送者
   就没这问题，顺带也保证 0xAA 广播序列不会被别的帧从中间插开。
   字段由云台任务写、底盘任务读；float/uint8 单次读写是原子的。 */
typedef struct
{
    volatile uint8_t Addr[3];           /* 三轴 CAN 地址（0 = 未初始化/未安装） */
    volatile float   Rpm[3];            /* 目标转速 RPM，顺序 [yaw, lift, extend] */
    volatile uint8_t EnableLeft[3];     /* 还要发几次"使能"，每发一次减 1 */
    volatile uint8_t DisableLeft[3];    /* 还要发几次"松轴"，每发一次减 1 */
    volatile uint8_t ClearLeft[3];      /* 还要发几次"解除堵转"，每发一次减 1 */
    volatile uint8_t FeedbackLeft[3];   /* 还要发几次"定时返回配置"，每发一次减 1 */
} GimbalTx_t;

extern GimbalTx_t GimbalTx;

/* ---- 以下由底盘任务在自己的循环里调用（同一个任务上下文，不跨任务抢邮箱） ---- */

/// @brief 发掉待处理的一次性单机帧（使能 / 解除堵转 / 位置回传配置）
void Gimbal_TxService(void);

/// @brief 给三轴各发一条 F6 速度单机帧（底盘任务每拍调用，失能时 Rpm 为 0）
void Gimbal_SendAxisSpeedFrames(void);

/// @brief 取三轴当前位置（相对上电中位，°）。遥测回传和限位示教都用这个坐标。
void Gimbal_GetRelDeg(float out[3]);

void GimbalTask(void const* argument);

extern Gimbal_t Gimbal;

#endif
