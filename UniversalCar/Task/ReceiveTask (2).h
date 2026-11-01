#ifndef _RECEIVE_TASK_H_
#define _RECEIVE_TASK_H_

/**
 * @attention   采用UTF-8字符集编码
 * @brief       树莓派指令接收任务
 */

#include "stdint.h"
#include "cmsis_os.h"

/* 整车状态机（两态，ROBOT_FAULT 保留以兼容上位机映射但不再使用） */
enum
{
    ROBOT_DISABLED = 0,     /* 失能 */
    ROBOT_READY,            /* 使能 */
    ROBOT_FAULT,            /* 保留（不再使用） */
};

/// @brief 整车接收状态
typedef struct
{
    uint8_t State;          /* 见上枚举：0=失能 1=使能 */
    uint8_t ErrorFlag;      /* 错误标志：bit0 树莓派失联 */
    uint32_t LastOnlineTick;
} Receive_t;

void ReceiveTask(void const* argument);

extern Receive_t Receive;

#endif
