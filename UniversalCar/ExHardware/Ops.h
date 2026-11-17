#ifndef _OPS_H_
#define _OPS_H_

/**
 * @attention   采用UTF-8字符集编码
 * @brief       OPS 里程计（Odom1_lite）驱动：UART4 + DMA 循环接收 + IDLE 断帧
 *
 * @details     V2 二进制协议，921600 8N1，小端：
 *                AA 55 | VER(02) | TYPE | LEN | PAYLOAD... | CRC16(低先发) | A5
 *              帧长 = LEN + 8。CRC-16/CCITT-FALSE（Poly 0x1021 / Init 0xFFFF /
 *              不反转 / XorOut 0），覆盖 VER 到 PAYLOAD 末字节。
 *
 *              TYPE=0x01 遥测（38B）：
 *                  SEQ(u16) X(i32) Y(i32) YAW(i32) VX(i32) VY(i32) AX(i32) AY(i32)
 *                  X/Y    单位 0.01 mm
 *                  YAW    单位 0.001 °
 *                  VX/VY  单位 0.001 mm/s（机体坐标系）
 *                  AX/AY  单位 0.001 mm/s²
 *              TYPE=0x11 ACK / 0x12 状态：只统计，不参与控制。
 *
 *              本模块只做两件事：把 OPS 的全局位置/航向变成底盘能用的米和度；
 *              用不用它顶替编码器里程计，由 RobotConfig.h 的 OPS_USE_FOR_ODOMETRY 决定。
 */

#include "stdint.h"

#define OPS_FRAME_HEAD_0        0xAAu
#define OPS_FRAME_HEAD_1        0x55u
#define OPS_FRAME_TAIL          0xA5u
#define OPS_PROTOCOL_VERSION    0x02u

#define OPS_TYPE_ODOMETRY       0x01u
#define OPS_TYPE_COMMAND        0x10u
#define OPS_TYPE_ACK            0x11u
#define OPS_TYPE_STATUS         0x12u

/* 命令编号（CMD 0x02 保留，不可用） */
#define OPS_CMD_SEND_TIME       0x03u
#define OPS_CMD_MCU_REBOOT      0x09u
#define OPS_CMD_POS_CLEAR       0x0Eu
#define OPS_CMD_ENC_DIAMETER    0x0Fu
#define OPS_CMD_SET_POS_X       0x10u
#define OPS_CMD_SET_POS_Y       0x11u
#define OPS_CMD_SET_YAW         0x12u

/// @brief OPS 实例
typedef struct
{
    /* ---- 最近一帧遥测（已按协议换算成物理量） ---- */
    float    RawX_mm;           // OPS 全局 X（mm，OPS 自己的世界系）
    float    RawY_mm;           // OPS 全局 Y（mm）
    float    RawYaw_deg;        // OPS 航向（°，未乘方向符号）
    float    Vx_mms;            // 机体前进速度 mm/s
    float    Vy_mms;            // 机体横移速度 mm/s
    float    Ax_mms2;
    float    Ay_mms2;
    uint16_t Sequence;          // 遥测序号，用来发现丢帧

    /* ---- 链路/状态 ---- */
    uint8_t  Online;            // 最近 OPS_TIMEOUT_MS 内收到过遥测帧
    uint8_t  PosValid;          // Online 且已和 IMU 航向对齐 → 可以顶替编码器里程计
    uint8_t  CfgDone;           // 上电初始化命令（周期/清零/轮径）是否已发完
    float    YawOffsetDeg;      // OPS 世界系 → IMU 世界系 的旋转量（上电锁存）
    uint16_t SendPeriodMs;      // 已下发的遥测周期
    uint16_t WheelDiameterMm;   // 已下发的测量轮直径

    uint32_t LastFrameTick;     // 最近一帧遥测的时刻
    uint32_t FrameCount;        // 有效遥测帧总数
    uint32_t CrcErrorCount;     // CRC 错
    uint32_t SyncErrorCount;    // 帧头/长度错（重新同步次数）
    uint32_t AckCount;          // 收到的 ACK 数
    uint8_t  LastAckCmd;
    uint8_t  LastAckStatus;     // 0=成功 1=参数错 2=不支持 3=失败
    uint32_t LastAckValue;
} Ops_t;

extern Ops_t Ops;

/// @brief 初始化 UART4（改波特率）+ DMA 循环接收 + IDLE 中断
void    Ops_Init(void);

/// @brief UART4 中断入口（在 stm32f4xx_it.c 的 USER CODE 里调用）
void    Ops_IRQHandler(void);

/// @brief UART4 错误回调（在 usart.c 的 HAL_UART_ErrorCallback 里调用）
///        HAL 遇到溢出会把 DMA 接收整个停掉，这里只做标记，下一拍 Ops_Poll 重挂
void    Ops_ErrorCallback(void);

/// @brief 周期调用（底盘 5ms 一拍）：维护在线状态、跑上电配置流程、锁存航向对齐
void    Ops_Poll(uint32_t now);

/// @brief 喂入任意长度的串口字节（中断里调用，可处理半帧/粘包/干扰）
void    Ops_Receive(const uint8_t *data, uint16_t length);

/// @brief 取底盘坐标系的绝对位置（米）。返回 0 = OPS 当前不可用，调用方自己兜底
uint8_t Ops_GetPosition(float *x_m, float *y_m);

/// @brief 是否在线（仅看链路，不代表位置可用）
uint8_t Ops_IsOnline(void);

/// @brief 发一条 V2 配置命令，返回 0 = 命令号不合法
uint8_t Ops_SendConfig(uint8_t cmd, uint32_t value);

#endif
