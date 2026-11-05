/**
 * @attention   采用UTF-8字符集编码
 * @brief       惯性测量任务
 * @details     陀螺仪二选一，由 RobotConfig.h 的 IMU_USE_MSPM0 切换（重新编译烧录）：
 *               1 = MSPM0G3507 + XV7011 单轴(Z)模组（USART3 115200，
 *                   驱动 ExHardware/MSPM0imu.c；上电自动把输出率提到 200Hz——
 *                   命令掉电不保存所以每次上电都发；模组已做零偏扣除+判静锁零）
 *               0 = JY901S 九轴（驱动 ExHardware/JY901S.c；输出率取决于模块
 *                   自身配置，默认 ~10Hz）
 *
 *              字节解析在 UART 中断完成，本任务 5ms 周期把数据取到 Imu_t 并维护
 *              在线状态。上层（ChassisTask/ReceiveTask/DebugTask）只认 Imu_t，
 *              换模组对它们无感。
 *
 *              MSPM0 模组注意：
 *               - 装反时（航向跑飞/反向拧）改 MSPM0imu.h 里 MSPM0IMU_YAW_SIGN
 *                 / MSPM0IMU_GYRO_SIGN 为 -1.0f；
 *               - 换陀螺后上位机必须重新【标定里程计】（yaw 零点变了）。
 */

#include "header.h"

#define IMU_TASK_UPDATE_TICK 5

Imu_t Imu;

#if IMU_USE_MSPM0
/* 用户实现的下行发送：把配置命令从 USART3 发给模组（覆盖驱动里的弱符号空实现） */
void MSPM0IMU_Send(const uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)data, len, 20);
}
#endif

void ImuTask(void const* argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();

#if IMU_USE_MSPM0
    MSPM0IMU_Init();
    /* 出厂默认 ~10Hz，上电即提速到 200Hz（底盘任务 200Hz，航向环/里程计坐标
       旋转需要新鲜 yaw）。带宽：200Hz × 0x52/0x53 两帧 × 11B ≈ 4.4KB/s。 */
    MSPM0IMU_SetOutputRate(MSPM0IMU_RATE_200HZ);
#else
    JY901S_Init();
#endif

    while (1)
    {
        osDelayUntil(&PreviousWakeTime, IMU_TASK_UPDATE_TICK);

        /* ---- 模块数据 → 解算结果 ---- */
#if IMU_USE_MSPM0
        Imu.Roll  = MSPM0IMU.Angle[0];
        Imu.Pitch = MSPM0IMU.Angle[1];
        Imu.Yaw   = MSPM0IMU.Angle[2];
        Imu.Gyro[0] = MSPM0IMU.Gyro[0];
        Imu.Gyro[1] = MSPM0IMU.Gyro[1];
        Imu.Gyro[2] = MSPM0IMU.Gyro[2];
        Imu.Acc[0] = MSPM0IMU.Acc[0];
        Imu.Acc[1] = MSPM0IMU.Acc[1];
        Imu.Acc[2] = MSPM0IMU.Acc[2];
        Imu.LastOnlineTick = MSPM0IMU.LastOnlineTick;

        MSPM0IMU_Poll(HAL_GetTick());
#else
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
#endif
    }
}
