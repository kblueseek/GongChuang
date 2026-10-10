#ifndef _PI_COMM_H_
#define _PI_COMM_H_

/**
 * @attention   采用UTF-8字符集编码
 * @brief       树莓派通信协议（USART6，双向）
 * @details     帧格式：
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
#define PI_CMD_ENABLE        0x07    /* DATA: en(u8) + mask(u8, bit0-6 = 电机1-7) */
#define PI_CMD_ESTOP         0x08    /* DATA: 无=置急停；或1字节(0=解除, 非0=置急停)，最高优先级 */
#define PI_CMD_POSE          0x09    /* DATA: x y yaw (int32 mm, int32 mm, int16 mrad)，底盘位姿目标(位置+航向闭环) */
#define PI_CMD_HEARTBEAT     0x10    /* DATA: 无，1Hz */

/* ============ 遥测码（STM32 → 树莓派） ============ */
#define PI_RPT_ODOM          0x81    /* DATA: x y theta (int32 mm, int32 mm, int16 mrad) */
#define PI_RPT_STATUS        0x83    /* DATA: state(u8) + err(u8) + motor_online_mask(u8) */

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
        float Deg;      //目标角度 °
        uint8_t Mask;   //bit0=yaw bit1=lift bit2=extend，本次下发了哪个轴
    } GimbalTarget[3];

    uint8_t HomeMask;   //回零请求 bit0=yaw bit1=lift bit2=extend（执行后由任务清除）
    uint8_t EnableMask; //电机使能位 bit0-6 = 电机1-7
    uint8_t Estop;      //急停标志（收到即置位，上位机解除前保持）

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

    uint32_t LastOnlineTick;    //最后一次收到有效指令的时刻
} PiCommand_t;

extern PiCommand_t PiCommand;

/// @brief 启动 UART 单字节中断接收（main 里调用一次）
void PiComm_Init(void);

/// @brief 字节解析（由 usart.c 的 HAL_UART_RxCpltCallback 调用）
void PiComm_RxByte(uint8_t data);

/// @brief 树莓派是否在线（近 PI_CMD_TIMEOUT_MS 内收到过指令）
int PiComm_IsOnline(void);

/// @brief 周期回调：超时自动停车保护（由 ReceiveTask 调用）
void PiComm_TimeoutCheck(void);

/* ============ 遥测发送 ============ */

/// @brief 发送里程计（预留：TODO 等 OPS9 或电机编码器轮询接入后填真实数据）
void PiComm_SendOdometry(float x_mm, float y_mm, float theta_mrad);

/// @brief 发送整车状态（电机在线掩码等）
void PiComm_SendStatus(uint8_t state, uint8_t err, uint8_t motorOnlineMask);

#endif
