/**
 * @file    ControllerTask.h
 * @brief   底盘运动控制 — 蓝牙摇杆 → 麦克纳姆运动学 → 电机速度
 */
#ifndef __CONTROLLER_TASK_H
#define __CONTROLLER_TASK_H

void ControllerTask_Init(void);   /* 使能电机、初始化状态 */
void ControllerTask_Loop(void);   /* 主循环调用：看门狗停车 + 下发轮速 */

#endif /* __CONTROLLER_TASK_H */
