/* zdtmotor_uart.h — ZDT Y42 闭环步进电机 串口(UART) 控制封装（X 固件）
 *
 * 协议要点（手册 4.1）：字节流帧 = 地址 + 功能码 + 数据 + 校验码(0x6B)
 * 波特率 115200 / 8N1（与电机默认一致，工程 USART1 已配好）
 *
 * 与 CAN 版 zdtmotor.h 二选一使用，勿同时包含（枚举/宏会重定义）。
 *
 * 集成方法（本库已自带 HAL_UART_RxCpltCallback/ErrorCallback，无需改 usart.c）：
 *   1. 把 zdtmotor_uart.c/.h 加入 Keil 工程，并把本目录加进包含路径；
 *   2. main 里 MX_USART1_UART_Init() 之后调用 ZDT_UART_Init();
 *
 * 多机接线（TTL）：STM32 TX -> 所有电机 RX；所有电机 TX 并在一起 -> STM32 RX。
 * 主从问答式、同一时刻只有一个电机回话，TTL 直接并联可用（走长线建议 RS485）。
 *
 * 用法示例：
 *   // 1) 多电机命令：四个轮子一次发完
 *   ZDT_MultiBegin();
 *   ZDT_MultiAdd(ZDT_MOTOR_FL, (uint8_t[]){0xF6,0x00,0x01,0xF4,0x05,0xDC,0x00}, 7);
 *   ZDT_MultiAdd(ZDT_MOTOR_FR, (uint8_t[]){0xF6,0x01,0x01,0xF4,0x05,0xDC,0x00}, 7);
 *   ...  ZDT_MultiSend();
 *   // 2) 定时返回：电机1 每 10ms 回一次实时位置
 *   ZDT_UART_Periodic(1, ZDT_INFO_POS, 10);   // 停止则 ms=0
 */
#ifndef __ZDTMOTOR_UART_H__
#define __ZDTMOTOR_UART_H__

#include <stdint.h>

/* 电机地址：1-4 麦克纳姆底盘，5-7 机械臂 */
enum {
    ZDT_MOTOR_FL = 1,  /* 底盘 前左 */
    ZDT_MOTOR_FR = 2,  /* 底盘 前右 */
    ZDT_MOTOR_RL = 3,  /* 底盘 后左 */
    ZDT_MOTOR_RR = 4,  /* 底盘 后右 */
    ZDT_MOTOR_A1 = 5,  /* 机械臂 关节1 */
    ZDT_MOTOR_A2 = 6,  /* 机械臂 关节2 */
    ZDT_MOTOR_A3 = 7,  /* 机械臂 关节3 */
};

/* 方向 */
#define ZDT_DIR_CW   0
#define ZDT_DIR_CCW  1

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

/* 定时返回信息命令用到的信息功能码 */
#define ZDT_INFO_POS        0x36  /* 实时位置 */
#define ZDT_INFO_SPEED      0x35  /* 实时转速 */
#define ZDT_INFO_STATUS     0x3A  /* 电机状态标志 */
#define ZDT_INFO_HOME_STATUS 0x3C /* 回零+电机状态 */

/* 电机反馈状态（下标 = 地址 1~7） */
typedef struct {
    int32_t pos;      /* 实时位置，单位 0.1° */
    uint8_t home;     /* 回零状态标志（见手册 5.4.4） */
    uint8_t status;   /* 电机状态标志（bit0 使能 / bit1 到位） */
} ZDT_UARTState;

extern ZDT_UARTState zdt_uart_state[8];

void ZDT_UART_Init(void);                                     /* 启动 1 字节中断接收 */

/* 低层单命令发送：cmd = 功能码+数据（不含地址、校验码），自动加校验码 */
void ZDT_UART_Send(uint8_t addr, const uint8_t *cmd, uint8_t len);

/* 多电机命令(0xAA)：一次发送多条命令，只有地址1回确认，避免总线冲突 */
void ZDT_MultiBegin(void);
void ZDT_MultiAdd(uint8_t addr, const uint8_t *cmd, uint8_t len);
void ZDT_MultiSend(void);

/* 定时返回信息命令(0x11 18)：电机按 ms 周期主动回报，ms=0 停止 */
void ZDT_UART_Periodic(uint8_t addr, uint8_t infoCode, uint16_t ms);

/* 基础命令 */
void ZDT_UART_Enable(uint8_t addr, uint8_t en);               /* en: 1使能/0去使能 */
void ZDT_UART_Stop(uint8_t addr);                             /* 立即停止 */
void ZDT_UART_SetZero(uint8_t addr);                          /* 当前位置清零 */
void ZDT_UART_Home(uint8_t addr, uint8_t mode);               /* 触发回零 */

/* 运动命令（X 固件） */
void ZDT_UART_Speed(uint8_t addr, uint8_t dir, uint16_t acc, uint16_t speed); /* speed 0.1RPM */
void ZDT_UART_Pos(uint8_t addr, uint8_t dir, uint16_t speed, uint16_t acc,
                  uint16_t dec, uint32_t pos, uint8_t mode);  /* speed 0.1RPM，pos 0.1° */

/* 读取反馈（单次） */
void ZDT_UART_ReadPos(uint8_t addr);                          /* 读实时位置(0x36) */
void ZDT_UART_ReadStatus(uint8_t addr);                       /* 读回零+电机状态(0x3C) */

/* 接收字节解析（由本库的 HAL_UART_RxCpltCallback 调用，一般无需手动调） */
void ZDT_UART_ReceiveByte(uint8_t b);

#endif
