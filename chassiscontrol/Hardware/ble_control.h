/**
 * @file    ble_control.h
 * @brief   蓝牙串口模块 — 微信小程序数据包解析 & 绘图/滑杆/摇杆/按键接口
 * @note    USART2, TX=PA2, RX=PA3, 115200 bps（USART1 已给 ZDT 电机用）
 *
 * 小程序数据包格式:
 *   [slider,名称,值]      → 滑杆调参
 *   [s,名称,值]           → 滑杆调参 (短格式)
 *   [joystick,Lx,Ly,Rx,Ry] → 摇杆控制
 *   [key,名称,down|up]    → 按键事件
 *   [plot,v1,v2,...v10]   → MCU 发送绘图 (1~10 条线)
 *   [plot-clear]          → MCU 发送清空绘图
 *   短格式: slider→s, joystick→j, key→k, plot→p, down→d, up→u, plot-clear→p-c
 */
#ifndef __BLE_CONTROL_H
#define __BLE_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

/* ============ 滑杆事件 ============ */
typedef struct {
    char  name[8];    /* 滑杆名称 */
    float value;      /* 滑杆值 */
} BLE_SliderEvent;

/* ============ 摇杆事件 ============ */
typedef struct {
    int Lx, Ly;       /* 左摇杆 */
    int Rx, Ry;       /* 右摇杆 */
} BLE_JoystickEvent;

/* ============ 按键事件 ============ */
typedef struct {
    int  name;        /* 按键名称 */
    bool is_down;     /* true=按下, false=松开 */
} BLE_KeyEvent;

/* ============ 外部接口 ============ */
void BLE_Init(void);
void BLE_RxCallback(uint8_t data);           /* 字节处理 (由 UART Rx 回调调用) */
void BLE_Process(void);                      /* 主循环每帧调用 */
void BLE_SendPlot(float *values, int count); /* 发送绘图包, count=1~10 */
void BLE_SendPlotClear(void);                /* 清空绘图区 */
void BLE_Printf(const char *fmt, ...);       /* 通用 printf, 发往手机 */

/* ============ 事件回调 (用户实现, 在 ControllerTask.c) ============ */
extern void BLE_OnSlider  (BLE_SliderEvent  *e);
extern void BLE_OnJoystick(BLE_JoystickEvent *e);
extern void BLE_OnKey     (BLE_KeyEvent     *e);

/* 接收标志 & 原始数据包 */
extern volatile uint8_t BLE_RxFlag;
extern          char    BLE_RxPacket[];

#endif /* __BLE_CONTROL_H */
