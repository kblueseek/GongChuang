#ifndef _DM_MOTOR_H_
#define _DM_MOTOR_H_

/**
 * @attention   采用UTF-8字符集编码
 * @brief       达妙（DAMIAO）电机 MIT 模式驱动
 * @details     移植自北工大 PIP 战队的 DMMotor 库，按本工程的结构体风格重写：
 *              去掉 UniversalMotor 那套 PWM/DJI 通用层，只保留 MIT 模式。
 *
 *              协议要点：
 *               - 控制帧：标准帧（11 位 ID），ID = 电机 ID（1~16），8 字节
 *               - 回传帧：标准帧，ID = 电机里配置的 Master ID（默认 0）
 *               - 使能/失能/置零/清错：数据区 0xFF×7 + 0xFC/0xFD/0xFE/0xFB
 *               - 本工程用"速度源"用法：kp=0，kd 当速度环增益，torqueFF=0
 *
 *              ⚠ 和 ZDT（张大头）共用一条 CAN 总线没问题：
 *                 ZDT 用**扩展帧**（29 位 ID），达妙用**标准帧**（11 位 ID），
 *                 两者是不同的 ID 空间，硬件上不会冲突。接收侧按 IDE 位分流即可。
 *
 *              使用前必须在达妙上位机里把电机 ID / Master ID / 控制模式配好。
 */

#include "stdint.h"

/* MIT 映射范围（DM4310 V1.1 手册）。换型号要跟着改 RobotConfig.h 里的宏。 */
#define DM4310_P_MIN    (-12.5f)
#define DM4310_P_MAX    ( 12.5f)
#define DM4310_V_MIN    (-30.0f)
#define DM4310_V_MAX    ( 30.0f)
#define DM4310_T_MIN    (-10.0f)
#define DM4310_T_MAX    ( 10.0f)

#define DM_OFFLINE_MS   500     /* 这么久没收到回传就判离线 */
#define DM_LIST_MAX     8       /* 最多注册几台达妙电机 */

/// @brief 达妙电机句柄
typedef struct
{
    uint8_t CanId;      /* 控制帧 ID（MIT 模式 = 电机 ID） */
    uint8_t MasterId;   /* 回传帧 ID（电机里配的 Master ID，默认 0） */

    /* MIT 映射范围 */
    float P_Min, P_Max;
    float V_Min, V_Max;
    float T_Min, T_Max;

    /* MIT 参数：kp=0 时就是纯速度/力矩源，kd 相当于速度环增益 */
    float kP, kD;

    /* 当前下发的目标 */
    struct
    {
        float Pos;      /* 输出轴角度 rad */
        float Vel;      /* 输出轴角速度 rad/s */
        float TorqueFF; /* 前馈力矩 N·m */
    } Target;

    /* 反馈（输出轴） */
    struct
    {
        float Radian;
        float Degree;   /* 保留小数，云台位置环按 0.1° 级别的精度用 */
        float RadianUnwrapped;   /* 多圈累计 rad */
        float DegreeUnwrapped;   /* 多圈累计 ° */
    } Position;

    struct { float Radian; float RPM; } Speed;
    struct { float Real; } Torque;

    float    Temperature;
    uint8_t  isEnable;      /* 1 = 电机已使能（由回传状态解析） */
    uint8_t  ErrorFlag;     /* 0 = 正常；非 0 = 过温/过压等，见手册 */
    uint8_t  isOffline;     /* 1 = 离线 */
    uint32_t LastOnlineTick;

    /* 多圈展开用的上一次原始角度 */
    float LastRawRadian;
} DMMotor_t;

/// @brief 注册并初始化一台达妙电机（CanId = 电机 ID，MasterId = 回传 ID）
void DMMotor_Init(DMMotor_t* motor, uint8_t canId, uint8_t masterId,
                  float kp, float kd,
                  float pMin, float pMax, float vMin, float vMax,
                  float tMin, float tMax);

/// @brief 电机是否在线
int  DMMotor_IsOnline(DMMotor_t* motor);

/* ============ 一次性命令 ============ */
void DMMotor_Enable(DMMotor_t* motor);
void DMMotor_Disable(DMMotor_t* motor);
void DMMotor_SetZero(DMMotor_t* motor);      /* 把当前位置记成电机零点（掉电保存） */
void DMMotor_CleanError(DMMotor_t* motor);

/* ============ 控制 ============ */

/// @brief 速度源用法：kp 保持 0，只给速度目标，kd 提供阻尼
void DMMotor_SetSpeed(DMMotor_t* motor, float radPerSec);

/// @brief 位置目标（kp>0 时才是真正的阻抗/位置控制）
void DMMotor_SetTarget(DMMotor_t* motor, float rad, float radPerSec, float torqueFF);

/// @brief 组一帧 MIT 控制报文并发出去。每拍（5ms）调一次，停发电机就没力矩
void DMMotor_Output(DMMotor_t* motor);

/* ============ 接收解析（由 can.c 的标准帧分支调用） ============ */
void DMMotor_RxCallback(uint32_t stdId, uint8_t* data, uint8_t len);

#endif
