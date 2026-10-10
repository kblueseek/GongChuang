/**
 * @attention   采用UTF-8字符集编码
 * @brief       树莓派指令接收任务
 * @details     字节级解析在 UART 中断里完成（PiComm_RxByte），
 *              本任务负责：心跳超时保护、急停响应、整车状态维护、状态回传。
 */

#include "header.h"

#define RECEIVE_TASK_UPDATE_TICK 10

Receive_t Receive;

/* 错误标志位 */
#define ERR_ESTOP      (1 << 0)
#define ERR_PI_OFFLINE (1 << 1)
#define ERR_MOTOR_LOST (1 << 2)

void ReceiveTask(void const* argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();

    PiComm_Init();
    BLE_Init();

    Receive.State = ROBOT_DISABLED;
    Receive.ErrorFlag = 0;
    Receive.LastOnlineTick = 0;

    uint32_t LastReportTick = 0;
    uint32_t LastOdomTick = 0;

    while (1)
    {
        osDelayUntil(&PreviousWakeTime, RECEIVE_TASK_UPDATE_TICK);

        uint32_t NowTick = HAL_GetTick();

        /* ---- 蓝牙包解析（始终解析，保持在线状态新鲜） ---- */
        BLE_Process();

        /* ---- 遥控源切换：树莓派掉线时蓝牙（类比 DBUS）接管整车 ---- */
        if (!PiComm_IsOnline()) {
            PiComm_TimeoutCheck();              /* 树莓派掉线：底盘停车 */
            if (BLE_IsOnline())
                BLE_ApplyToCommand();           /* 蓝牙摇杆/按键/滑杆 → 整车指令 */
        }

        /* ---- 错误标志维护 ---- */
        Receive.ErrorFlag = 0;
        if (PiCommand.Estop)
            Receive.ErrorFlag |= ERR_ESTOP;
        if (!PiComm_IsOnline())
            Receive.ErrorFlag |= ERR_PI_OFFLINE;

        /* ---- 整车状态 ---- */
        if (PiCommand.Estop)
            Receive.State = ROBOT_FAULT;
        else if (PiCommand.EnableMask == 0)
            Receive.State = ROBOT_DISABLED;
        else
            Receive.State = ROBOT_READY;

        /* ---- 低速回传整车状态（10Hz） ---- */
        if (NowTick - LastReportTick >= 100)
        {
            LastReportTick = NowTick;

            /* 电机在线掩码：bit0-6 = 电机1-7 */
            uint8_t mask = 0;
            for (int i = 1; i <= 7; i++)
                if (ZdtMotorList[i] != NULL && ZdtMotor_IsOnline(ZdtMotorList[i]))
                    mask |= (1 << (i - 1));

            PiComm_SendStatus(Receive.State, Receive.ErrorFlag, mask);
        }

        /* ---- 里程计回传（PI_RPT_FREQ_HZ，默认50Hz）：Pi 位置闭环/状态机靠它判断是否到位 ---- */
        if (NowTick - LastOdomTick >= (1000 / PI_RPT_FREQ_HZ))
        {
            LastOdomTick = NowTick;
            PiComm_SendOdometry(Chassis.Pos.X * 1000.f,
                                Chassis.Pos.Y * 1000.f,
                                Imu.Yaw * (PI / 180.f) * 1000.f);   /* ° → mrad */
        }
    }
}
