/**
 * @file    MSPM0imu.h
 * @brief   MSPM0G3507 + XV7011 单轴(Z)陀螺仪模组 —— 主机端 UART 驱动
 *
 * @details
 *   适用固件：本仓库 ICMM0 固件的 XV7011 单轴板（MSPM0G3507 + XV70xx）
 *   适用主机：STM32F407 / UniversalCar（也可移植到任意 MCU）
 *
 *   接口：UART，默认 115200 8N1，TTL 3.3V
 *   帧格式（两种都支持，自动识别）：
 *     1) 11 字节标准帧（模组出厂默认）：
 *          55 52 D1L D1H D2L D2H D3L D3H D4L D4H SUM
 *            D1 = 0
 *            D2 = Z 轴角速度（已扣零偏 + 判静锁零，未做单帧死区）
 *            D3 = Z 轴角速度（已扣零偏 + 判静锁零 + 死区，推荐使用）
 *            D4 = 温度 ×100（℃）
 *          55 53 D1..D4 SUM
 *            D1 = 0, D2 = 0
 *            D3 = 偏航角 Yaw（±180° ↔ ±32768，小端）
 *            D4 = 温度 ×100
 *     2) 5 字节兼容帧（模组切到 AX1 帧族后）：
 *          5A AA <int16 角速度> SUM   （已扣零偏）
 *          5A BB <int16 偏航角> SUM
 *
 *   与 JY901S 的字段布局差异、配置命令、移植步骤见配套文档：
 *     docs/JY901S与MSPM0-XV7011使用区别与移植指南.md
 *
 *   移植三步：
 *     ① 把 MSPM0imu.c / MSPM0imu.h 加入工程
 *     ② UART 接收中断里调用 MSPM0IMU_RxByte(byte)
 *     ③ 主循环或任务里周期调用 MSPM0IMU_Poll(当前毫秒)
 *
 *   可选：若要发配置命令（改速率、归零、校准等），实现
 *   MSPM0IMU_Send()，把 5 字节从 UART 发给模组。
 */
#ifndef __MSPM0IMU_H__
#define __MSPM0IMU_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 编译期配置
 * ============================================================ */

/** 角度符号：安装方向与 JY901S 相反时改 -1.0f */
#ifndef MSPM0IMU_YAW_SIGN
#define MSPM0IMU_YAW_SIGN       (1.0f)
#endif

/** 角速度符号：一般与 MSPM0IMU_YAW_SIGN 保持一致 */
#ifndef MSPM0IMU_GYRO_SIGN
#define MSPM0IMU_GYRO_SIGN      (1.0f)
#endif

/** 1 = 同时解析 5 字节兼容帧（0x5A ...）；0 = 只解析 11 字节标准帧 */
#ifndef MSPM0IMU_ENABLE_AX1F
#define MSPM0IMU_ENABLE_AX1F    (1)
#endif

/** 失联判定时间（ms）：超过这么久没收到有效数据帧即判离线 */
#ifndef MSPM0IMU_OFFLINE_MS
#define MSPM0IMU_OFFLINE_MS     (200u)
#endif

/* ============================================================
 * 帧标识
 * ============================================================ */
#define MSPM0IMU_HEAD_STD       (0x55u) /**< 11 字节标准帧帧头 */
#define MSPM0IMU_ID_GYRO        (0x52u) /**< 标准帧：角速度 */
#define MSPM0IMU_ID_ANGLE       (0x53u) /**< 标准帧：角度 */
#define MSPM0IMU_HEAD_AX1F      (0x5Au) /**< 5 字节兼容帧帧头 */
#define MSPM0IMU_AX1F_GYRO      (0xAAu) /**< 兼容帧：角速度 */
#define MSPM0IMU_AX1F_ANGLE     (0xBBu) /**< 兼容帧：角度 */

/* ============================================================
 * 输出速率编码（单轴模组 RRATE 0x03）
 *   注意：单轴表与六轴标准表不同，0x0A 未定义，200Hz 是 0x0B。
 * ============================================================ */
typedef enum
{
    MSPM0IMU_RATE_10HZ   = 0x06,
    MSPM0IMU_RATE_20HZ   = 0x07,
    MSPM0IMU_RATE_50HZ   = 0x08,
    MSPM0IMU_RATE_100HZ  = 0x09,
    MSPM0IMU_RATE_200HZ  = 0x0B,
    MSPM0IMU_RATE_500HZ  = 0x0C,
    MSPM0IMU_RATE_1000HZ = 0x0D
} MSPM0IMU_RateCode;

/* ============================================================
 * 数据结构
 *
 *   Angle / Gyro / Acc / LastOnlineTick 与 JY901S_t 同名，
 *   ImuTask.c 里可以直接把 JY901S 换成 MSPM0IMU。
 * ============================================================ */
typedef struct
{
    /* ---- 与 JY901S 兼容的字段 ---- */
    float Angle[3];             /**< [0]=0, [1]=0, [2]=Yaw(°)；单轴模组只有 Z 轴 */
    float Gyro[3];              /**< [0]=0, [1]=0, [2]=YawRate(°/s)，已扣零偏 */
    float Acc[3];               /**< 恒 0：本模组没有加速度计 */
    uint32_t LastOnlineTick;    /**< 最后一次收到有效帧的时刻（由 Poll 写入） */

    /* ---- 单轴模组专用字段 ---- */
    float Yaw;                  /**< 偏航角 °，等于 Angle[2]，编码后范围 [-180, 180) */
    float YawRate;              /**< Z 轴角速度 °/s，等于 Gyro[2] */
    float Temp;                 /**< 温度 ℃ */
    int16_t YawRaw;             /**< 角度原始码（±180° ↔ ±32768） */
    int16_t GyroRaw;            /**< 0x52 的 D3：扣零偏 + 判静锁零后的原始码 */
    int16_t GyroRawOut;         /**< 0x52 的 D2：扣零偏、判静锁零、未做死区的原始码 */

    /* ---- 链路状态 ---- */
    uint8_t  Online;            /**< 1=在线（由 MSPM0IMU_Poll 维护） */
    uint32_t FrameCnt;          /**< 有效数据帧计数（0x52/0x53/0xAA/0xBB） */
    uint32_t ErrCnt;            /**< 校验失败/坏帧计数 */
} MSPM0IMU_t;

/** 全局数据，与 JY901S 的用法一致 */
extern MSPM0IMU_t MSPM0IMU;

/* ============================================================
 * 核心接口
 * ============================================================ */

/**
 * @brief 初始化（清零数据与解析状态）
 * @note  在使能 UART 接收之前调用一次
 */
void MSPM0IMU_Init(void);

/**
 * @brief 单字节解析，放在 UART 接收中断 / 空闲回调里调用
 * @param data 收到的一个字节
 */
void MSPM0IMU_RxByte(uint8_t data);

/**
 * @brief 周期调用（建议 5~20ms 一次），维护 Online / LastOnlineTick
 * @param now_ms 当前毫秒时基，例如 HAL_GetTick()
 */
void MSPM0IMU_Poll(uint32_t now_ms);

/** @brief 是否在线（等价于 Online != 0） */
bool MSPM0IMU_IsOnline(void);

/** @brief 取偏航角 ° */
float MSPM0IMU_GetYaw(void);

/** @brief 取 Z 轴角速度 °/s */
float MSPM0IMU_GetYawRate(void);

/* ============================================================
 * 配置命令（可选）
 *
 *   这些函数通过 MSPM0IMU_Send() 发 5 字节下行帧。
 *   如果只读数据，可以不实现 MSPM0IMU_Send()，这些函数就是空操作。
 * ============================================================ */

/**
 * @brief 用户实现：把 len 字节从 UART 发给模组
 * @note  驱动内是弱符号，用户在自己的 .c 里定义同名函数即可覆盖。
 *        例：HAL_UART_Transmit(&huart3, (uint8_t *)data, len, 20);
 */
void MSPM0IMU_Send(const uint8_t *data, uint16_t len);

/** @brief 设置输出速率（写入 RRATE 0x03，立即生效；掉电不保存） */
void MSPM0IMU_SetOutputRate(uint8_t rate_code);

/** @brief 当前角度清零（0x76 CALIYAW） */
void MSPM0IMU_SetYawZero(void);

/**
 * @brief 把“当前”角度直接设为 deg（0x77 SETYAW）
 * @param deg 目标角度 °（建议落在 [-180, 180)）
 * @note  不打断积分；掉电不保存；重启后回到 0。
 */
void MSPM0IMU_SetYawDeg(float deg);

/** @brief 开始零偏校准（0x01 CALSE=1），默认采集 10s，期间必须静止 */
void MSPM0IMU_StartBiasCal(void);

/** @brief 提前结束零偏校准（0x01 CALSE=4） */
void MSPM0IMU_StopBiasCal(void);

/** @brief 保存参数到模组 Flash（0x00 SAVE=0） */
void MSPM0IMU_SaveParam(void);

/** @brief 模组软重启（0x00 SAVE=0x00FF） */
void MSPM0IMU_Restart(void);

/** @brief 读版本号（0x27 读 0x2E，回 0x5F 帧；本驱动暂不解析回包） */
void MSPM0IMU_RequestVersion(void);

#ifdef __cplusplus
}
#endif

#endif /* __MSPM0IMU_H__ */
