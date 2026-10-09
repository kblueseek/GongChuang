#ifndef _DEBUG_TASK_H_
#define _DEBUG_TASK_H_

/**
 * @attention   采用UTF-8字符集编码
 * @brief       调试任务（数据回传上位机）
 * @details     固定格式数据包：Head + Value[DEBUG_VALUE_NUM] + Tail，
 *              通过 USART1 DMA 发送，上位机按格式解析绘制曲线。
 */

#include "stdint.h"
#include "cmsis_os.h"

#include "RobotConfig.h"

/// @brief 调试数据包
typedef struct
{
    uint32_t Head;
    float Value[DEBUG_VALUE_NUM];
    uint32_t Tail;
} DebugData_t;

void DebugTask(void const* argument);

extern DebugData_t DebugData;

#endif
