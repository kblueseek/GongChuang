#ifndef _RECEIVE_TASK_H_
#define _RECEIVE_TASK_H_

/**
 * @attention   采用UTF-8字符集编码
 * @brief       树莓派指令接收任务
 */

#include "stdint.h"
#include "cmsis_os.h"

/* 整车状态机 */
enum
{
    ROBOT_DISABLED = 0,     /* 未使能 */
    ROBOT_READY,            /* 已使能，正常 */
    ROBOT_FAULT,            /* 故障（急停/失联） */
};

/// @brief 整车接收状态
typedef struct
{
    uint8_t State;          /* 见上枚举 */
    uint8_t ErrorFlag;      /* 错误标志：bit0 急停 bit1 树莓派失联 bit2 电机失联 */
    uint32_t LastOnlineTick;
} Receive_t;

void ReceiveTask(void const* argument);

extern Receive_t Receive;

#endif
