#ifndef _ZDT_MOTOR_UART_H_
#define _ZDT_MOTOR_UART_H_

/**
 * @attention   采用UTF-8字符集编码
 * @brief       ZDT Y42 闭环步进电机 串口(TTL) 控制封装（Emm 固件）
 * @details     CAN 收发器损坏后的临时替代：改用 UART5 与电机通信。
 *              协议要点（手册 4.1）：
 *               - 串口帧 = 地址(1) + 功能码(1) + 命令数据 + 校验码(0x6B)
 *               - 与 CAN 版相比，地址直接作为帧首字节，且整包发送、无需按 8 字节拆包
 *               - 电机为内部闭环（20kHz 位置/速度环），STM32 只需下发速度/位置指令
 *               - 使用前需在电机侧配置：通讯端口复用=串口，地址 1~7，波特率 115200
 */

#include "stdint.h"

/* 位置运动模式 */
#define ZDT_POS_REL_PREV  0  /* 相对上一输入目标位置 */
#define ZDT_POS_ABS       1  /* 绝对位置（相对坐标零点） */
#define ZDT_POS_REL_CUR   2  /* 相对当前实时位置 */

/* 回零模式 */
#define ZDT_HOME_NEAR       0  /* 单圈就近 */
#define ZDT_HOME_DIR        1  /* 单圈方向 */
#define ZDT_HOME_SENSORLESS 2  /* 无限位碰撞 */
#define ZDT_HOME_LIMIT      3  /* 限位 */
#define ZDT_HOME_ABSZERO    4  /* 绝对坐标零点 */
#define ZDT_HOME_POWERCUT   5  /* 掉电位置 */

/* 电机状态标志（status，bit0 使能 / bit1 到位） */
#define ZDT_STATUS_ENABLE   (1 << 0)
#define ZDT_STATUS_ARRIVED  (1 << 1)

/// @brief ZDT 电机句柄
typedef struct
{
    uint8_t Addr;   //电机地址 1~7

    struct
    {
        float Set;      //目标转速 RPM
        float Real;     //实时转速 RPM（Emm：整数 RPM）
    } Speed;

    struct
    {
        float Set;      //目标输出轴角度 °
        float Real;     //实时输出轴角度 °（Emm：0-65535=一圈，×360/65536）
    } Position;

    uint8_t Status;     //电机状态标志（bit0 使能 / bit1 到位）
    uint8_t HomeStatus; //回零状态标志（见手册 5.4.4）

    uint32_t LastOnlineTick;    //最后一次收到反馈的时刻
} ZdtMotor_t;

/* 电机注册表（下标 = 电机地址），由各电机 Init 时注册 */
extern ZdtMotor_t* ZdtMotorList[8];

/* ============ 电机实例维护 ============ */

/// @brief 注册电机（在任务初始化阶段调用一次），清零所有字段
void ZdtMotor_Init(ZdtMotor_t* motor, uint8_t addr);

/// @brief 电机是否在线（近 ZDT_OFFLINE_MS 内收到过反馈）
int ZdtMotor_IsOnline(ZdtMotor_t* motor);

/* ============ 串口总线命令（低层，操作地址） ============ */

void ZdtUart_Init(void);                            /* 复位接收解析器（UART5 已由 CubeMX 初始化，main 里调用一次） */
void ZdtUart_Enable(uint8_t addr, uint8_t en);      /* en: 1 使能锁轴 / 0 去使能松轴 */
void ZdtUart_Stop(uint8_t addr);                    /* 立即停止 */
void ZdtUart_SetZero(uint8_t addr);                 /* 当前位置清零 */
void ZdtUart_Home(uint8_t addr, uint8_t mode);      /* 触发回零 */
void ZdtUart_SyncAll(void);                         /* 广播触发多机同步 */
void ZdtUart_ReadPos(uint8_t addr);                 /* 读取实时位置(0x36) */
void ZdtUart_ReadStatus(uint8_t addr);              /* 读取回零+电机状态(0x3C) */

/* ============ 多电机命令(0xAA)：一次发多条，只有地址1回确认，避免总线冲突 ============ */

void ZdtUart_MultiBegin(void);                                  /* 清空多电机缓冲，开始组帧 */
void ZdtUart_MultiAdd(uint8_t addr, const uint8_t* cmd, uint8_t len);   /* 追加一条子命令（不含地址/校验码） */
void ZdtUart_MultiSend(void);                                   /* 广播发送 00 AA 长度 + 各子命令 + 校验码 */

/* ============ 电机级命令（操作 ZdtMotor_t 句柄，内部转协议） ============ */

/// @brief 速度模式：speed 单位 RPM（整数，方向自动）
void ZdtMotor_SetSpeed(ZdtMotor_t* motor, float rpm);

/// @brief 速度子命令追加到多电机缓冲（配合 ZdtUart_MultiBegin/Send 使用）
void ZdtMotor_MultiAddSpeed(ZdtMotor_t* motor, float rpm);

/// @brief 位置模式：pos 单位 °（内部转脉冲 3200/圈），mode 见 ZDT_POS_*
void ZdtMotor_SetPosition(ZdtMotor_t* motor, float deg, uint8_t mode);

/* ============ 接收解析（由 usart.c 的 HAL_UART_RxCpltCallback 逐字节调用） ============ */
void ZdtUart_RxByte(uint8_t b);

#endif
