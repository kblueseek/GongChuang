/**
 * @attention   采用UTF-8字符集编码
 * @brief       调试任务
 * @details     把需要观察的变量填进 DebugData.Value[] 发给上位机。
 *              调车时按需修改填充内容，改完注释掉即可。
 */

#include "header.h"

#define DEBUG_TASK_UPDATE_TICK 10

DebugData_t DebugData = { 0 };

void DebugTask(void const* argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();

    DebugData.Head = DEBUG_HEAD;
    DebugData.Tail = DEBUG_TAIL;

    while (1)
    {
        osDelayUntil(&PreviousWakeTime, DEBUG_TASK_UPDATE_TICK);
        DebugData.Value[0] = PiCommand.Enabled;      // 0=失能 1=使能
        DebugData.Value[1] = JY901S.Angle[2];
        DebugData.Value[2] = Chassis.HeadingPid.Output;
        DebugData.Value[3] = PiCommand.Gripper;
        DebugData.Value[4] = Chassis.Wheel[0].Position.Real;
        DebugData.Value[5] = Chassis.Wheel[1].Position.Real;
#if BLE_CONTROL_ENABLE
        DebugData.Value[6] = BleRemote.Joy.Lx;       // 左摇杆横向值（原始）
        DebugData.Value[7] = BleRemote.Joy.Ly;       // 左摇杆纵向值（原始）
        DebugData.Value[8] = BleRemote.Joy.Rx;       // 右摇杆横向值（原始）
        DebugData.Value[9] = BleRemote.Joy.Ry;       // 右摇杆纵向值（原始）
        DebugData.Value[10] = BLE_RxByteCount;       // USART2 收到的字节数（0=完全没收到）
#else
        /* 云台三轴实时角度（°）：实测行程时看这三个通道，手动推到机械限位读刻度 */
        DebugData.Value[6] = Gimbal.Yaw.Motor.Position.Real;
        DebugData.Value[7] = Gimbal.Lift.Motor.Position.Real;
        DebugData.Value[8] = Gimbal.Extend.Motor.Position.Real;
        DebugData.Value[9] = 0;
        DebugData.Value[10] = 0;
#endif
        DebugData.Value[11] = Receive.ErrorFlag;

        HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&DebugData, sizeof(DebugData));
    }
}
