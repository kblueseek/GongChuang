/* zdtmotor.h — ZDT Y42 闭环步进电机 CAN 控制封装（X 固件）
 *
 * 协议要点（见手册第 4.2 节）：
 *  - CAN 扩展帧 ID = (地址 << 8) | 包序号；地址字节不进数据区。
 *  - 数据区 = 功能码 + 命令数据 + 校验码(0x6B)；>8 字节自动拆包。
 *
 * 使用前需在电机侧配置：通讯端口复用模式 = CAN(03)，地址设为 1~7。
 */
#ifndef __ZDTMOTOR_H__
#define __ZDTMOTOR_H__

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

/* 电机反馈状态（下标 = 地址 1~7） */
typedef struct {
    int32_t pos;      /* 实时位置，单位 0.1° */
    uint8_t home;     /* 回零状态标志（见手册 5.4.4） */
    uint8_t status;   /* 电机状态标志（见手册 5.5.15，bit0 使能/bit1 到位） */
} ZDT_MotorState;

extern ZDT_MotorState zdt_state[8];

void ZDT_Motor_Init(void);                                     /* 配置过滤器并启动 CAN + 使能接收中断 */

/* 低层发送：cmd = 功能码 + 命令数据（不含地址、不含校验码），自动追加校验码并拆包 */
void ZDT_Motor_Send(uint8_t addr, const uint8_t *cmd, uint8_t len);

/* 基础命令 */
void ZDT_Motor_Enable(uint8_t addr, uint8_t en);               /* en: 1 使能锁轴 / 0 去使能松轴 */
void ZDT_Motor_Stop(uint8_t addr);                             /* 立即停止 */
void ZDT_Motor_SetZero(uint8_t addr);                          /* 当前位置角度清零 */
void ZDT_Motor_Home(uint8_t addr, uint8_t mode);               /* 触发回零，mode 见 ZDT_HOME_* */
void ZDT_Motor_SyncAll(void);                                  /* 广播触发多机同步运动 */

/* 运动命令（X 固件） */
void ZDT_Motor_Speed(uint8_t addr, uint8_t dir, uint16_t acc, uint16_t speed); /* 速度模式，speed 单位 0.1RPM */
void ZDT_Motor_Pos(uint8_t addr, uint8_t dir, uint16_t speed, uint16_t acc,
                   uint16_t dec, uint32_t pos, uint8_t mode);  /* 梯形位置模式，speed 0.1RPM，pos 0.1° */

/* 读取反馈 */
void ZDT_Motor_ReadPos(uint8_t addr);                          /* 读取实时位置(0x36) */
void ZDT_Motor_ReadStatus(uint8_t addr);                       /* 读取回零+电机状态(0x3C) */

/* 由 can.c 接收回调调用，解析电机返回帧 */
void ZDT_Motor_RxCallback(uint32_t id, uint8_t *data, uint8_t len);

#endif
