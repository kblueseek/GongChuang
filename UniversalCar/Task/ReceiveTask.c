/**
 * @attention   采用UTF-8字符集编码
 * @brief       树莓派指令接收任务
 * @details     字节级解析在 UART 中断里完成：
 *                USART2（蓝牙上行）→ PiComm_RxBleByte
 *                USART6（有线备用）→ PiComm_RxByte
 *              本任务负责：树莓派离线自主失能、整车状态维护、遥测回传（LoRa）。
 */

#include "header.h"

#define RECEIVE_TASK_UPDATE_TICK 10

Receive_t Receive;

/* 错误标志位 */
#define ERR_PI_OFFLINE (1 << 0)

void ReceiveTask(void const* argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();

    PiComm_Init();
    Led_Init();             /* 补光灯 PWM：无论哪种控制源都要初始化，否则 PiComm 的 LED 指令无效 */
#if BLE_CONTROL_ENABLE
    BLE_Init();
#endif

    Receive.State = ROBOT_DISABLED;
    Receive.ErrorFlag = 0;
    Receive.LastOnlineTick = 0;

    uint32_t LastReportTick = 0;
    uint32_t LastOdomTick = 0;

    while (1)
    {
        osDelayUntil(&PreviousWakeTime, RECEIVE_TASK_UPDATE_TICK);

        uint32_t NowTick = HAL_GetTick();

        /* ---- 树莓派离线：自主失能（latch，恢复后保持失能，需手柄 start 重新使能） ---- */
        if (!PiComm_IsOnline()) {
            PiCommand.Enabled = 0;
            PiCommand.Vel.Vx = PiCommand.Vel.Vy = PiCommand.Vel.Wz = 0;   /* 清速度，防重连后残留 */
        }

#if BLE_CONTROL_ENABLE
        /* ---- 蓝牙包解析（始终解析，保持在线状态新鲜） ---- */
        BLE_Process();

        /* ---- 遥控源切换：树莓派掉线时蓝牙接管整车 ---- */
        if (!PiComm_IsOnline() && BLE_IsOnline())
            BLE_ApplyToCommand();           /* 蓝牙摇杆/按键/滑杆 → 整车指令 */
#endif

        /* ---- 错误标志维护 ---- */
        Receive.ErrorFlag = 0;
        if (!PiComm_IsOnline())
            Receive.ErrorFlag |= ERR_PI_OFFLINE;

        /* ---- 整车状态（两态：使能/失能） ---- */
        Receive.State = PiCommand.Enabled ? ROBOT_READY : ROBOT_DISABLED;

        /* ---- 低速回传整车状态（PI_STATUS_FREQ_MS，无线链路下降频） ---- */
        if (NowTick - LastReportTick >= PI_STATUS_FREQ_MS)
        {
            LastReportTick = NowTick;

            /* 电机在线掩码：bit0-6 = 电机1-7 */
            uint8_t mask = 0;
            for (int i = 1; i <= 7; i++)
                if (ZdtMotorList[i] != NULL && ZdtMotor_IsOnline(ZdtMotorList[i]))
                    mask |= (1 << (i - 1));

            PiComm_SendStatus(Receive.State, Receive.ErrorFlag, mask);
        }

        /* ---- 里程计回传（PI_RPT_FREQ_HZ，默认 20Hz）：Pi 位置闭环/状态机靠它判断是否到位 ---- */
        if (NowTick - LastOdomTick >= (1000 / PI_RPT_FREQ_HZ))
        {
            LastOdomTick = NowTick;
            PiComm_SendOdometry(Chassis.Pos.X * 1000.f,
                                Chassis.Pos.Y * 1000.f,
                                Imu.Yaw * (PI / 180.f) * 1000.f);   /* ° → mrad */
        }
    }
}
