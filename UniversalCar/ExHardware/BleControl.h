#ifndef _BLE_CONTROL_H_
#define _BLE_CONTROL_H_

/**
 * @attention   采用UTF-8字符集编码
 * @brief       蓝牙遥控模块（USART2，类比 RM 的 DBUS 遥控器）
 * @details     微信小程序数据包格式：
 *                [joystick,Lx,Ly,Rx,Ry] 或 [j,...] → 摇杆（-100~100）
 *                [key,名称,down|up] 或 [k,名称,d|u] → 按键
 *                [slider,名称,值] 或 [s,...] → 滑杆（0~1）
 *                [plot,v1,...v10] / [plot-clear] → MCU 发绘图
 *              作用：树莓派不在场时遥控整车，验证底盘/云台/舵机功能。
 *              树莓派在线时遥控输入被忽略（树莓派优先）。
 */

#include "stdint.h"

/* 按键编号约定（小程序端按键名，按下边沿有效） */
enum
{
    BLE_KEY_ENABLE  = 1,    /* 整车使能 toggle */
    BLE_KEY_ESTOP   = 2,    /* 急停（置位 Estop + 清使能） */
    BLE_KEY_RELEASE = 3,    /* 解除急停并重新使能 */
    BLE_KEY_RETURN  = 4,    /* 回到坐标零点（上电位置） */
};

/* 滑杆名称约定 */
#define BLE_SLIDER_GRIPPER "servo1"   /* 夹爪舵机 0~1 → 0~180° */
#define BLE_SLIDER_PLATE   "servo2"   /* 载物盘舵机 */
#define BLE_SLIDER_CAM     "servo3"   /* 摄像头舵机 */
#define BLE_SLIDER_YAW     "yaw"      /* 云台 Yaw 轴 0~1 → 软限位范围 */
#define BLE_SLIDER_LIFT    "lift"     /* 云台升降轴 */
#define BLE_SLIDER_EXTEND  "extend"   /* 云台伸缩轴 */
#define BLE_SLIDER_LED     "led"      /* 补光灯亮度 0~100（直接下发为亮度 %） */
#define BLE_SLIDER_SPIN    "spin"     /* 底盘目标航向 -180~180°（与陀螺仪同量程，直接下发） */

/// @brief 蓝牙遥控数据（各任务直接读）
typedef struct
{
    struct
    {
        int Lx;     //左摇杆横向 -100~100（暂未使用）
        int Ly;     //左摇杆纵向 -100~100 → 前进后退
        int Rx;     //右摇杆横向 -100~100 → 左右平移
        int Ry;     //右摇杆纵向 -100~100（暂未使用）
    } Joy;

    struct
    {
        uint8_t Down;       //当前是否按下
        uint8_t Pressed;    //按下边沿（由消费方处理完后清零）
    } Key[5];

    struct
    {
        float Gripper;      //夹爪舵机角度 °
        float Plate;        //载物盘舵机角度 °
        float Cam;          //摄像头舵机角度 °
        float Yaw;          //云台 Yaw 目标 °
        float Lift;         //云台升降目标 °
        float Extend;       //云台伸缩目标 °
        float Led;          //补光灯亮度 %（0~100，led 滑杆直接下发）
        float Spin;         //底盘目标航向 °（-180~180，spin 滑杆直接下发，与 Yaw 同量程）
        uint8_t SpinValid;  //是否收到过 spin 滑杆（没收到过则锁存当前朝向，不做修正）
        uint8_t GimbalMask; //云台轴更新位（bit0=yaw bit1=lift bit2=extend）
    } Slider;

    uint32_t LastOnlineTick;    //最后一次收到有效包的时刻
} BLE_Remote_t;

extern BLE_Remote_t BleRemote;
extern volatile uint32_t BLE_RxByteCount;   /* USART2 接收字节计数（调试用） */
extern volatile uint8_t  BLE_LastRxByte;     /* USART2 最后收到的原始字节（调试用） */
extern volatile uint32_t BLE_PacketCount;    /* 完成 [ ] 捕获的包数（调试用） */
extern volatile uint8_t  BLE_TagChar;        /* 最近一次捕获包的首字符（调试用） */

/// @brief 初始化（清状态；UART 接收由 usart.c 的 RX 中断统一管理）
void BLE_Init(void);

/// @brief 字节捕获（由 usart.c 的 HAL_UART_RxCpltCallback 调用，中断上下文）
void BLE_RxCallback(uint8_t data);

/// @brief 周期处理：解析收到的包（在 ReceiveTask 里调用）
void BLE_Process(void);

/// @brief 蓝牙是否在线（近 BLE_OFFLINE_MS 内收到过数据包）
int BLE_IsOnline(void);

/// @brief 遥控源适配：把蓝牙输入映射到整车指令（树莓派掉线时替代 PiComm 的位置）
/// @details 摇杆→Vel，按键→EnableMask/Estop，滑杆→舵机/云台目标
void BLE_ApplyToCommand(void);

/// @brief 滑杆分发：0~1 映射到对应执行器（由 BLE_Process 内部调用，亦可外部直接调）
void BLE_HandleSlider(const char* name, float value);

/* ============ 调试发送（发往手机） ============ */
void BLE_Printf(const char* fmt, ...);
void BLE_SendPlot(float* values, int count);
void BLE_SendPlotClear(void);

#endif
