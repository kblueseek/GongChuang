/**
 * @attention   采用UTF-8字符集编码
 * @brief       惯性测量任务
 * @details     JY901S 模块内置姿态解算，字节解析在 UART 中断完成（JY901S_RxByte）。
 *              本任务把模块数据取到 Imu_t 中，并预留：
 *                TODO 1: 加速度/角速度卡尔曼滤波（UserLibrary 可加 KalmanFilter）
 *                TODO 2: 加速度积分速度估计（漂移大，需与电机编码器/OPS9 里程计融合）
 *                TODO 3: OPS9 光流里程计接入（UART4 预留）
 */

#include "header.h"

#define IMU_TASK_UPDATE_TICK 5

Imu_t Imu;

void ImuTask(void const* argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();

    JY901S_Init();

    while (1)
    {
        osDelayUntil(&PreviousWakeTime, IMU_TASK_UPDATE_TICK);

        /* ---- 模块数据 → 解算结果 ---- */
        Imu.Roll  = JY901S.Angle[0];
        Imu.Pitch = JY901S.Angle[1];
        Imu.Yaw   = JY901S.Angle[2];
        Imu.Gyro[0] = JY901S.Gyro[0];
        Imu.Gyro[1] = JY901S.Gyro[1];
        Imu.Gyro[2] = JY901S.Gyro[2];
        Imu.Acc[0] = JY901S.Acc[0];
        Imu.Acc[1] = JY901S.Acc[1];
        Imu.Acc[2] = JY901S.Acc[2];
        Imu.LastOnlineTick = JY901S.LastOnlineTick;

        /* TODO: 卡尔曼滤波 / 速度估计 */
        /* TODO: OPS9 里程计融合 */
    }
}
