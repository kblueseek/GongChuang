#ifndef _ZDT_MOTOR_H_
#define _ZDT_MOTOR_H_

/**
 * @attention   采用UTF-8字符集编码
 * @brief       ZDT Y42 闭环步进电机 CAN 控制封装（Emm 固件）
 * @details     协议要点（手册 4.2）：
 *               - CAN 扩展帧 ID = (地址 << 8) | 包序号，地址字节不进数据区
 *               - 数据区 = 功能码 + 命令数据 + 校验码(0x6B)，>8 字节自动拆包
 *               - 电机为内部闭环（20kHz 位置/速度环），STM32 只需下发速度/位置指令
 *               使用前需在电机侧配置：通讯端口复用=CAN(03)，地址 1~7，波特率 500K
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
    uint8_t Addr;   //CAN 地址 1~7

    struct
    {
        float Set;      //目标转速 RPM
        float Real;     //实时转速 RPM（Emm：整数 RPM）
    } Speed;

    struct
    {
        float Set;          //目标输出轴角度 °
        float Real;         //实时输出轴角度 °（多圈，由 RawCount 换算）
        int32_t RawCount;   //实时位置原始计数（65536=一圈，多圈累计，最高精度）
    } Position;

    uint8_t Status;     //电机状态标志（bit0 使能 / bit1 到位）
    uint8_t HomeStatus; //回零状态标志（见手册 5.4.4）

    uint32_t LastOnlineTick;    //最后一次收到反馈的时刻
} ZdtMotor_t;

/* 电机注册表（下标 = CAN 地址），由各电机 Init 时注册 */
extern ZdtMotor_t* ZdtMotorList[8];

/* ============ 电机实例维护 ============ */

/// @brief 注册电机（在任务初始化阶段调用一次），清零所有字段
void ZdtMotor_Init(ZdtMotor_t* motor, uint8_t addr);

/// @brief 电机是否在线（近 ZDT_OFFLINE_MS 内收到过反馈）
int ZdtMotor_IsOnline(ZdtMotor_t* motor);

/* ============ CAN 总线命令（低层，操作地址） ============ */

void ZdtCan_Init(void);                             /* 配置过滤器 + 启动 CAN + 使能接收中断（main 里调用一次） */
void ZdtCan_Enable(uint8_t addr, uint8_t en);       /* en: 1 使能锁轴 / 0 去使能松轴 */
void ZdtCan_Stop(uint8_t addr);                     /* 立即停止 */
void ZdtCan_SetZero(uint8_t addr);                  /* 当前位置清零 */
void ZdtCan_ClearStall(uint8_t addr);               /* 解除堵转保护（堵转后保护锁存，电机在线但不动，必须清） */
void ZdtCan_Home(uint8_t addr, uint8_t mode);       /* 触发回零 */
void ZdtCan_SyncAll(void);                          /* 广播触发多机同步 */
void ZdtCan_ReadPos(uint8_t addr);                  /* 读取实时位置(0x36) */
void ZdtCan_ReadStatus(uint8_t addr);               /* 读取回零+电机状态(0x3C) */
void ZdtCan_SetTimedReturn(uint8_t addr, uint8_t func, uint16_t interval_ms);  /* 定时返回信息命令 */

/* ============ 多电机命令 (0xAA)：一条广播帧发多个电机命令，同时执行 ============ */
/* 组帧上下文：由发送方任务持有。全总线只能有一个发送者——两条 0xAA 序列共用
   CAN ID 0x000/0x001…，电机按包序号重组时会互相串包（0x6B 是固定字节、不是真校验，
   串出来的错帧照样会被执行）；而且 CAN1 只有 3 个发送邮箱，两个任务抢会把多包
   广播挤掉一包、整帧作废。现在只有底盘任务发帧，每 10ms：一条 4 轮速度的 0xAA
   广播（37 字节 / 6 包）+ 云台三轴各一条 F6 单机帧 + 云台的一次性命令
   （Gimbal_TxService 发的单机帧）。7 条子命令的广播（61 字节 / 9 包）实测电机不执行，
   别再把云台塞进广播里。 */

#define ZDT_MULTI_BUF_MAX  128      /* 子命令缓冲（4 轮速度 32B，留足余量） */

typedef struct
{
    uint8_t  Buf[ZDT_MULTI_BUF_MAX];            /* 子命令：地址 + 命令 + 校验码 */
    uint16_t Len;
    uint8_t  Tx[ZDT_MULTI_BUF_MAX + 8];         /* 组好的广播帧：不占任务栈（两个任务栈都只有 1KB） */
} ZdtMulti_t;

void ZdtCan_MultiBegin(ZdtMulti_t* ctx);                                               /* 清空缓冲，开始组帧 */
void ZdtCan_MultiAdd(ZdtMulti_t* ctx, uint8_t addr, const uint8_t* cmd, uint8_t len);  /* 追加一条子命令（不含地址/校验码） */
void ZdtCan_MultiSend(ZdtMulti_t* ctx);                                                /* 广播发送 AA 长度 + 各子命令 + 校验码 */

/* ============ 电机级命令（操作 ZdtMotor_t 句柄，内部转协议） ============ */

/// @brief 速度模式：speed 单位 RPM（整数，方向自动）
void ZdtMotor_SetSpeed(ZdtMotor_t* motor, float rpm);

/// @brief 速度子命令追加到多电机缓冲（配合 ZdtCan_MultiBegin/Send，多轴同时启动）
void ZdtMotor_MultiAddSpeed(ZdtMulti_t* ctx, ZdtMotor_t* motor, float rpm);

/// @brief 位置子命令追加到多电机缓冲（配合 ZdtCan_MultiBegin/Send，多轴同时启动）
void ZdtMotor_MultiAddPosition(ZdtMulti_t* ctx, ZdtMotor_t* motor, float deg, uint8_t mode);

/// @brief 位置模式：pos 单位 °（内部转脉冲 3200/圈），mode 见 ZDT_POS_*
void ZdtMotor_SetPosition(ZdtMotor_t* motor, float deg, uint8_t mode);

/* ============ 接收解析（由 can.c 的 HAL_CAN_RxFifo0MsgPendingCallback 调用） ============ */
void ZdtCan_RxCallback(uint32_t id, uint8_t* data, uint8_t len);

#endif
