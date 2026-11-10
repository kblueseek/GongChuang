#ifndef _PI_COMM_H_
#define _PI_COMM_H_

/**
 * @attention   采用UTF-8字符集编码
 * @brief       树莓派通信协议（无线：蓝牙上行 + LoRa 下行）
 * @details     上行：树莓派 --BLE--> USART2（PiComm_RxBleByte）
 *              下行：UART4 --LoRa--> 树莓派（仅发送，中断方式，不阻塞任务）
 *              USART6 保留为有线调试链路（PiComm_RxByte / 阻塞发送）。
 *              帧格式：
 *                ┌──────┬──────┬─────┬─────┬─────┬─────────┬──────┐
 *                │ SOF1 │ SOF2 │ LEN │ SEQ │ CMD │  DATA   │ CRC16│
 *                │ 0xAA │ 0x55 │ 1B  │ 1B  │ 1B  │ LEN-3 B │  2B  │
 *                └──────┴──────┴─────┴─────┴─────┴─────────┴──────┘
 *                LEN = CMD(1) + DATA(N) + CRC(2)，CRC16-MODBUS（0xA001/0xFFFF）
 *              树莓派 → STM32 指令，STM32 → 树莓派 遥测（里程计等）
 */

#include "stdint.h"

/* ============ 命令码（树莓派 → STM32） ============ */
#define PI_CMD_VEL           0x01    /* DATA: vx vy wz (int16 ×3, mm/s, mm/s, mrad/s) */
#define PI_CMD_GIMBAL_TARGET 0x02    /* DATA: axis(u8 0=yaw/1=lift/2=extend) + deg(int32 ×100) */
#define PI_CMD_GIMBAL_HOME   0x03    /* DATA: mask(u8, bit0=yaw bit1=lift bit2=extend) */
#define PI_CMD_GRIPPER       0x04    /* DATA: pos(u8, 0~180°) */
#define PI_CMD_PLATE         0x05    /* DATA: pos(u8, 0~180°) */
#define PI_CMD_CAM           0x06    /* DATA: pos(u8, 0~180°) */
#define PI_CMD_ENABLE        0x07    /* DATA: en(u8, 0=失能 1=使能)，整车单一使能状态 */
#define PI_CMD_GIMBAL_VEL    0x08    /* DATA: axis(u8 0=yaw/1=lift/2=extend) + deg/s(int16，整数 °/s) */
#define PI_CMD_POSE          0x09    /* DATA: x y yaw (int32 mm, int32 mm, int16 mrad)，底盘位姿目标(位置+航向闭环) */
#define PI_CMD_LED           0x0A    /* DATA: brightness(u8, 0~100%) 补光灯亮度（PWM 调光） */
#define PI_CMD_GIMBAL_LIMIT  0x0B    /* DATA: axis(u8) + min(int16 ×0.1°) + max(int16 ×0.1°) 示教限位 */
#define PI_CMD_GIMBAL_RELEASE 0x0C   /* DATA: en(u8)，1=松轴（电机控制板失能，可手拨） 0=恢复锁轴 */
#define PI_CMD_TUNE          0x0D    /* DATA: id(u8) + value(float32 大端)，在线调参（见 PI_TUNE_*） */
#define PI_CMD_GIMBAL_ZERO   0x0E    /* DATA: mask(u8, bit0=yaw bit1=lift bit2=extend)，把当前位置记为 0 点 */
#define PI_CMD_HEARTBEAT     0x10    /* DATA: 无，1Hz */

/* ============ 遥测码（STM32 → 树莓派） ============ */
#define PI_RPT_ODOM          0x81    /* DATA: x y theta (int32 mm, int32 mm, int16 mrad) */
#define PI_RPT_STATUS        0x83    /* DATA: state(u8) + err(u8) + motor_online_mask(u8) */
#define PI_RPT_GIMBAL        0x84    /* DATA: yaw lift extend (int16 ×0.1°，相对 0 点) */
#define PI_RPT_TUNE          0x85    /* DATA: id(u8) + value(float32 大端)，回显当前生效值 */
#define PI_RPT_GIMBAL_LIMIT  0x86    /* DATA: axis(u8) + valid(u8) + min(int16 ×0.1°) + max(int16 ×0.1°) */

/* ============ 在线调参 ID（PI_CMD_TUNE 的 id 字节） ============ */
#define PI_TUNE_MAX_VX       0x01    /* 前进最大速度 m/s */
#define PI_TUNE_MAX_VY       0x02    /* 平移最大速度 m/s */
#define PI_TUNE_MAX_WZ       0x03    /* 转向最大角速度 rad/s */
#define PI_TUNE_MAX_RPM      0x04    /* 单轮最大转速 RPM（整数） */
#define PI_TUNE_POS_KP       0x05    /* 位置环 Kp（米域） */
#define PI_TUNE_POS_KI       0x06    /* 位置环 Ki */
#define PI_TUNE_POS_KD       0x07    /* 位置环 Kd */
#define PI_TUNE_HEAD_KP      0x08    /* 转向环 Kp（度域） */
#define PI_TUNE_HEAD_KI      0x09    /* 转向环 Ki */
#define PI_TUNE_HEAD_KD      0x0A    /* 转向环 Kd */
#define PI_TUNE_GIMBAL_KP    0x0B    /* 云台位置环 Kp（度域：误差 ° → 速度 °/s） */
#define PI_TUNE_GIMBAL_KI    0x0C    /* 云台位置环 Ki */
#define PI_TUNE_GIMBAL_KD    0x0D    /* 云台位置环 Kd */
#define PI_TUNE_GIMBAL_LIFT_KP 0x0E  /* 云台 lift 位置环 Kp（0x0B~0x0D 是 yaw 轴） */
#define PI_TUNE_GIMBAL_LIFT_KI 0x0F
#define PI_TUNE_GIMBAL_LIFT_KD 0x10
#define PI_TUNE_GIMBAL_EXT_KP  0x11  /* 云台 extend 位置环 Kp */
#define PI_TUNE_GIMBAL_EXT_KI  0x12
#define PI_TUNE_GIMBAL_EXT_KD  0x13
#define PI_TUNE_GIMBAL_YAW_SPEED   0x14  /* 云台 yaw 最大速度 °/s（位置环输出限幅） */
#define PI_TUNE_GIMBAL_LIFT_SPEED  0x15
#define PI_TUNE_GIMBAL_EXT_SPEED   0x16

/// @brief 在线可调参数（上位机 PID 调参页下发，立即生效，不烧录）
/// @note  Version 每次收到调参帧 +1；底盘任务发现版本变了才重装 PID 增益，
///        避免每 5ms 都去重算 kI*dt / kD/dt 并把积分项清掉。
///        解析在 UART 中断里，回显不在这里发：底盘任务逐拍调用
///        PiComm_SendTune()，不会在中断里阻塞。
typedef struct
{
    float MaxSpeedX;    //前进最大速度 m/s（同时限制树莓派/手柄速度指令）
    float MaxSpeedY;    //平移最大速度 m/s
    float MaxSpeedW;    //转向最大角速度 rad/s（航向环输出也受它限幅）
    float MaxWheelRpm;  //单轮最大转速 RPM（麦轮解算后的等比例限幅）

    float PosKp;        //位置环 Kp（米域：误差 m → 输出 m/s）
    float PosKi;
    float PosKd;

    float HeadingKp;    //转向环 Kp（度域：误差 ° → 输出 °/s）
    float HeadingKi;
    float HeadingKd;

    float GimbalKp;     //云台 yaw 位置环 Kp（度域：误差 ° → 速度 °/s）
    float GimbalKi;
    float GimbalKd;
    float GimbalLiftKp; //云台 lift 位置环（yaw/lift/extend 三环独立整定）
    float GimbalLiftKi;
    float GimbalLiftKd;
    float GimbalExtKp;  //云台 extend 位置环
    float GimbalExtKi;
    float GimbalExtKd;
    float GimbalYawSpeed;   //云台三轴最大速度 °/s（0 或负 = 用固件默认）
    float GimbalLiftSpeed;
    float GimbalExtSpeed;

    uint32_t Version;   //调参版本号，底盘/云台任务用它判断是否需要重装
} PiTune_t;

extern PiTune_t PiTune;

/// @brief 树莓派下发的整车指令（各任务直接读）
typedef struct
{
    struct
    {
        float Vx;   //前进速度 m/s
        float Vy;   //平移速度 m/s
        float Wz;   //自转角速度 rad/s
    } Vel;

    struct
    {
        float Deg;      //目标角度 °（相对云台 0 点，见 PI_CMD_GIMBAL_ZERO）
        uint8_t Mask;   //1 = 本次下发了该轴；云台任务取走后清 0，然后由位置环保持
    } GimbalTarget[3];

    struct
    {
        float    DegPerSec; //目标角速度 °/s（PI_CMD_GIMBAL_VEL 下发，速度模式用）
        uint32_t Tick;      //最后一次收到该轴速度指令的时刻，0 = 从没收到过
    } GimbalVel[3];

    struct
    {
        float Min;      //示教下限 °（相对 0 点）
        float Max;      //示教上限 °
        uint8_t Valid;  //1 = 树莓派下发过，云台用它覆盖 RobotConfig.h 的默认值
    } GimbalLimit[3];

    uint8_t GimbalRelease;  //1 = 云台三轴松轴（电机控制板失能，可手拨） 0 = 正常

    uint8_t GimbalZeroMask; //置零请求 bit0=yaw bit1=lift bit2=extend（执行后由云台任务清除）

    uint8_t HomeMask;   //回零请求 bit0=yaw bit1=lift bit2=extend（执行后由任务清除）
    uint8_t Enabled;    //整车使能状态：1=使能 0=失能（唯一状态，无急停/锁存区分）

    uint8_t ReturnHome; //回到坐标零点请求（蓝牙 return 键触发，到达后由底盘任务清除）

    struct
    {
        float X;        //世界系位置目标 m（PI_CMD_POSE 下发）
        float Y;
        uint8_t Valid;  //1=位置闭环生效；收到速度指令(PI_CMD_VEL)会清掉它，退回手动速度模式
    } PosTarget;

    float   TargetYaw;      //目标航向 °（-180~180），航向 PID 的被控目标
    uint8_t TargetYawValid; //是否收到过有效目标航向（0 = 锁存当前朝向，不做修正）

    float Gripper;      //夹爪舵机角度 °
    float Plate;        //载物盘舵机角度 °
    float Cam;          //摄像头舵机角度 °
    float Led;          //补光灯亮度 %（0~100）

    uint32_t LastOnlineTick;    //最后一次收到有效指令的时刻
} PiCommand_t;

extern PiCommand_t PiCommand;

/// @brief 初始化（清指令状态与两套解析上下文）
void PiComm_Init(void);

/// @brief 有线链路字节解析（USART6，由 usart.c 的 HAL_UART_RxCpltCallback 调用）
void PiComm_RxByte(uint8_t data);

/// @brief 蓝牙上行字节解析（USART2，由 usart.c 的 HAL_UART_RxCpltCallback 调用）
/// @note  与有线链路各用一套帧同步状态机，互不干扰；解析结果写同一个 PiCommand
void PiComm_RxBleByte(uint8_t data);

/// @brief 树莓派是否在线（近 PI_CMD_TIMEOUT_MS 内收到过指令）
int PiComm_IsOnline(void);

/* ============ 遥测发送 ============ */

/// @brief 发送里程计（预留：TODO 等 OPS9 或电机编码器轮询接入后填真实数据）
void PiComm_SendOdometry(float x_mm, float y_mm, float theta_mrad);

/// @brief 发送整车状态（电机在线掩码等）
void PiComm_SendStatus(uint8_t state, uint8_t err, uint8_t motorOnlineMask);

/// @brief 发送云台三轴位置（相对 0 点，单位 °；状态屏显示 + 限位示教用）
void PiComm_SendGimbalPose(float yawDeg, float liftDeg, float extendDeg);

/// @brief 回传单轴当前生效的软限位（相对 0 点，°）
/// @note  valid=0 表示还没收到过示教/上位机限位，值是 RobotConfig.h 的默认值。
///        树莓派状态屏和上位机靠它同步同一套限位。
void PiComm_SendGimbalLimit(uint8_t axis, uint8_t valid, float minDeg, float maxDeg);

/// @brief 回显一项当前生效的调参值（上位机用来确认参数真的进 407 了）
void PiComm_SendTune(uint8_t id, float value);

/// @brief 把 PiTune 恢复成 RobotConfig.h 的默认值（PiComm_Init 调用）
void PiComm_TuneReset(void);

/// @brief UART4（LoRa）发送完成回调，由 usart.c 的 HAL_UART_TxCpltCallback 调用
void PiComm_TxCpltCallback(void);

/// @brief UART4（LoRa）错误回调，由 usart.c 的 HAL_UART_ErrorCallback 调用
void PiComm_ErrorCallback(void);

#endif
