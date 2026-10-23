#ifndef _IMU_TASK_H_
#define _IMU_TASK_H_

/**
 * @attention   采用UTF-8字符集编码
 * @brief       惯性测量任务（JY901S）
 */

#include "stdint.h"
#include "cmsis_os.h"

/// @brief 姿态解算结果（供底盘/云台任务使用）
typedef struct
{
    float Yaw;      //航向角 °
    float Pitch;    //俯仰角 °
    float Roll;     //横滚角 °
    float Gyro[3];  //角速度 °/s（0:Roll 1:Pitch 2:Yaw）
    float Acc[3];   //加速度 g
    float Vel[3];   //速度估计 m/s（TODO：卡尔曼/互补滤波后填）

    uint32_t LastOnlineTick;
} Imu_t;

void ImuTask(void const* argument);

extern Imu_t Imu;

#endif
