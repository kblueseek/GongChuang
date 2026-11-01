/**
 * @attention   采用UTF-8字符集编码
 * @brief       遥控指令接收任务（蓝牙 / 树莓派，由 BLE_CONTROL_ENABLE 选择）
 * @details     字节级解析在 UART 中断里完成（BLE_RxCallback / PiComm_RxByte），
 *              本任务负责：遥控源离线自主失能、整车状态维护、（树莓派模式）状态回传。
 */

#include "header.h"

#define RECEIVE_TASK_UPDATE_TICK 10

Receive_t Receive;

/* 错误标志位 */
#define ERR_REMOTE_OFFLINE (1 << 0)

void ReceiveTask(void const* argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();

    PiComm_Init();
#if BLE_CONTROL_ENABLE
    /* 蓝牙模式：USART2 已改由蓝牙占用，PiComm 既收不到也不回传。
       上电先失能，等小程序按下"使能"键再放开，避免一上电就跟着摇杆跑。
       舵机初值取蓝牙模块缓存的上电角度，避免蓝牙接管瞬间跳变。 */
    BLE_Init();
    PiCommand.Enabled = 0;
    PiCommand.Gripper = BleRemote.Slider.Gripper;
    PiCommand.Plate   = BleRemote.Slider.Plate;
    PiCommand.Cam     = BleRemote.Slider.Cam;
#endif

    Receive.State = ROBOT_DISABLED;
    Receive.ErrorFlag = 0;
    Receive.LastOnlineTick = 0;

#if !BLE_CONTROL_ENABLE
    uint32_t LastReportTick = 0;
    uint32_t LastOdomTick = 0;
#endif
#if (BLE_CONTROL_ENABLE && BLE_PLOT_ENABLE)
    uint32_t LastPlotTick = 0;
#endif

    while (1)
    {
        osDelayUntil(&PreviousWakeTime, RECEIVE_TASK_UPDATE_TICK);

        uint32_t NowTick = HAL_GetTick();

#if BLE_CONTROL_ENABLE
        /* ---- 蓝牙遥控：唯一指令源 ----
           每周期解析收到的包并应用到整车；超过 BLE_OFFLINE_MS 没收到包则自主失能
           并清速度，避免遥控断线后残留最后一次摇杆值把车开走。 */
        BLE_Process();

        static uint8_t s_BleWasOnline = 0;
        if (BLE_IsOnline())
        {
            if (!s_BleWasOnline) {
                s_BleWasOnline = 1;
                PiCommand.Enabled = 1;      /* 遥控连上（含断线重连）自动使能，和接收机行为一致 */
#if (BLE_CONTROL_ENABLE && BLE_PLOT_ENABLE)
                BLE_SendPlotClear();        /* 波形页：新连接先清掉上一轮的曲线 */
#endif
            }
            BLE_ApplyToCommand();           /* 摇杆→Vel，按键→Enabled，滑杆→舵机/云台 */
        }
        else {
            s_BleWasOnline = 0;
            PiCommand.Enabled = 0;
            PiCommand.Vel.Vx = PiCommand.Vel.Vy = PiCommand.Vel.Wz = 0;
        }
#else
        /* ---- 树莓派离线：自主失能（latch，恢复后保持失能，需手柄 start 重新使能） ---- */
        if (!PiComm_IsOnline()) {
            PiCommand.Enabled = 0;
            PiCommand.Vel.Vx = PiCommand.Vel.Vy = PiCommand.Vel.Wz = 0;   /* 清速度，防重连后残留 */
        }
#endif

#if (BLE_CONTROL_ENABLE && BLE_PLOT_ENABLE)
        /* ---- IMU 三轴角度波形 → 小程序波形页 ----
           帧：[p,Yaw,Pitch,Roll]\r\n（ASCII，两位小数，单位 °）

           【重要】这里故意不用 BLE_IsOnline() 做门控：
           小程序是"事件驱动"的，只有动摇杆/按键/滑杆时才发数据包，
           停手就一个包都不发。若用 BLE_IsOnline() 判断，波形就会
           "动一下摇杆才走一下"（BLE_OFFLINE_MS=500ms 后判定离线）。
           波形是单向推送，收不到包不代表手机没连着，所以无条件发。 */
        if (NowTick - LastPlotTick >= (1000 / BLE_PLOT_FREQ_HZ))
        {
            LastPlotTick = NowTick;
            float Plot[3];
            Plot[0] = Imu.Yaw;
            Plot[1] = Imu.Pitch;
            Plot[2] = Imu.Roll;
            BLE_SendPlot(Plot, 3);
        }
#endif

        /* ---- 错误标志维护 ---- */
        Receive.ErrorFlag = 0;
#if BLE_CONTROL_ENABLE
        if (!BLE_IsOnline())
            Receive.ErrorFlag |= ERR_REMOTE_OFFLINE;
#else
        if (!PiComm_IsOnline())
            Receive.ErrorFlag |= ERR_REMOTE_OFFLINE;
#endif

        /* ---- 整车状态（两态：使能/失能） ---- */
        Receive.State = PiCommand.Enabled ? ROBOT_READY : ROBOT_DISABLED;

        /* ---- 以下为树莓派协议遥测：蓝牙模式下 USART2 接着蓝牙模块，
               发二进制帧只会被模块当成乱码，且阻塞任务，故整体关闭 ---- */
#if !BLE_CONTROL_ENABLE
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
#else
        (void)NowTick;
#endif
    }
}

