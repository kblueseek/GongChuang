/**
 * @attention   采用UTF-8字符集编码
 * @brief       云台控制任务（三轴纯速度控制，只计算不发送）
 * @details     云台三轴和底盘轮子一样只收速度：本任务算出三轴目标转速写进 GimbalTx，
 *              由底盘任务在每拍的 4 轮 0xAA 广播之后，给三轴各发一条 F6 单机帧。
 *              为什么不让云台自己发：CAN1 只有 3 个发送邮箱，两个任务同时发会互相
 *              挤掉——多包广播丢一包就整帧作废（底盘"卡住"），单机帧丢一帧就是
 *              某个轴没反应。全总线只留一个发送者，这两个问题就都不存在。
 *              控制律：摇杆 → 角速度(°/s)，回中/超时 → 0，线性比例，无软件死区。
 *              限位：每轴第一次位置反馈锁存成 0 点（相对上电位置）；之后按
 *              "相对位置 + 运动方向"掐掉越界方向的速度。
 *              限位值默认取 RobotConfig.h 的宏；树莓派开机示教时用 PI_CMD_GIMBAL_LIMIT
 *              把人工拨到的两端下发过来，运行时以树莓派下发的为准。示教点本身就是
 *              编码器原始坐标，所以限位值和限位判断用的是同一套坐标。
 *              但"判断"还要知道速度指令的正方向对应编码器变大还是变小——编码器
 *              计数方向和电机转向相反的轴（实测 Yaw），正速度是往编码器小的一头走。
 *              所以每轴有一个符号 GIMBAL_xx_CMD_SIGN，判断前先乘上它换到编码器坐标。
 *              符号填反的典型现象：顶到限位后往回拨不动，继续往越界方向反而能动。
 *              松轴示教：PI_CMD_GIMBAL_RELEASE=1 时把三轴电机控制板失能（F3 AB 00），
 *              并且停发 F6（速度帧可能把驱动重新使能），人手才能自由拨动。
 *              失能（正常整车失能，不是松轴）：目标速度直接置 0，底盘广播里每拍都发 0
 *              （不松轴，保持力还在）。
 *              夹爪/摄像头舵机直接跟随树莓派指令。
 */

#include "header.h"

#define GIMBAL_TASK_UPDATE_TICK    5    /* 200 Hz：和底盘同频计算（总线预算见 ChassisTask.c） */
#define GIMBAL_FEEDBACK_MS        10    /* 三轴位置回传周期（ms）：100 Hz，只用于限位判断 */
#define GIMBAL_ENABLE_RETRY_MS   500    /* 使能重发周期（ms） */
#define GIMBAL_CONFIG_RETRY_MS   500    /* 位置回传配置重发周期（ms） */
#define GIMBAL_VEL_TIMEOUT_MS    300    /* 这么久没收到该轴新速度就发 0 */

Gimbal_t  Gimbal;
GimbalTx_t GimbalTx;

#if GIMBAL_MOTOR_INSTALLED
/* 云台三轴的软限位（相对上电位置，单位 °）。
   初值是 RobotConfig.h 的默认值；树莓派下发示教结果后在这里被覆盖。
   示教点是"人手把轴推到想要的位置"，所以坐标就是 (Real - Offset)，限位值和位置
   回传用的是同一套坐标，不需要另外做数值换算。 */
static float s_MinDeg[3] = { GIMBAL_YAW_MIN,   GIMBAL_LIFT_MIN,   GIMBAL_EXTEND_MIN };
static float s_MaxDeg[3] = { GIMBAL_YAW_MAX,   GIMBAL_LIFT_MAX,   GIMBAL_EXTEND_MAX };

/* 每轴"速度指令正方向 → 编码器坐标"的符号（见 RobotConfig.h）。
   限位判断必须在这个坐标里做，否则符号为 -1 的轴会整个反过来。 */
static const float s_CmdSign[3] = {
    GIMBAL_YAW_CMD_SIGN, GIMBAL_LIFT_CMD_SIGN, GIMBAL_EXTEND_CMD_SIGN
};

/* 三轴的顺序固定为 0=Yaw 1=Lift 2=Extend，和 PiCommand.GimbalVel[] 的下标一致 */
static ZdtMotor_t* const s_Axis[3] = {
    &Gimbal.Yaw.Motor, &Gimbal.Lift.Motor, &Gimbal.Extend.Motor
};

/* 各轴是否已装：没装的轴一条子命令都不加 */
static const uint8_t s_AxisInstalled[3] = {
    GIMBAL_YAW_INSTALLED, 1, 1
};

static uint32_t s_EnableTick   = 0;
static uint32_t s_FeedbackTick = 0;

/* 上电中位偏移：每轴第一次收到位置反馈时锁存，限位按 (Real - Offset) 判断 */
static float   s_Offset[3]      = { 0.0f, 0.0f, 0.0f };
static uint8_t s_OffsetValid[3] = { 0, 0, 0 };

/* 把树莓派示教出来的限位同步进来（每拍调用，代价只有 3 次判断）。
   树莓派没下发过的轴保持 RobotConfig.h 的默认值。 */
static void Gimbal_SyncLimits(void)
{
    for (uint8_t i = 0; i < 3; i++)
        if (PiCommand.GimbalLimit[i].Valid) {
            s_MinDeg[i] = PiCommand.GimbalLimit[i].Min;
            s_MaxDeg[i] = PiCommand.GimbalLimit[i].Max;
        }
}

/* 单轴目标转速写入（带限位）。离线 / 中位未锁存时一律给 0。 */
static void Gimbal_SetAxisSpeed(uint8_t i, float degPerSec)
{
    const float stopBand = 5.0f;   /* 离限位多近（°）就强制停 */

    if (!ZdtMotor_IsOnline(s_Axis[i]) || !s_OffsetValid[i]) {
        GimbalTx.Rpm[i] = 0.0f;
        return;
    }

    float rel = s_Axis[i]->Position.Real - s_Offset[i];
    /* 先把速度换到编码器坐标：正值 = 让 rel 变大。这样"往里推/往外拉"的判断
       与电机转向设置无关，编码器方向相反的轴（Yaw）也不会把限位判断弄反。 */
    float cmdInEncoder = degPerSec * s_CmdSign[i];
    if ((cmdInEncoder > 0.0f && rel >= s_MaxDeg[i] - stopBand) ||
        (cmdInEncoder < 0.0f && rel <= s_MinDeg[i] + stopBand))
        degPerSec = 0.0f;

    GimbalTx.Rpm[i] = degPerSec / 6.0f;   /* 1 RPM = 6 °/s */
}

/* 三轴当前位置（相对上电位置，°）。没锁存基准前给 0，遥测不会发 NaN。 */
void Gimbal_GetRelDeg(float out[3])
{
    for (uint8_t i = 0; i < 3; i++)
        out[i] = s_OffsetValid[i]
               ? (s_Axis[i]->Position.Real - s_Offset[i])
               : 0.0f;
}

/* ============ 交给底盘任务发送 ============ */

/* 一次性单机帧：使能 / 解除堵转 / 位置回传配置。
   只由底盘任务调用（同一个任务上下文），不会和广播的包互相抢占邮箱。
   每轴每拍最多发一帧（按 使能 > 解堵转 > 回传配置 排队，剩余次数下一拍继续）：
   三轴 × 十几次连发会在同一拍里塞几十帧，把 3 个发送邮箱和 10ms 周期一起挤爆。 */
void Gimbal_TxService(void)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        uint8_t a = GimbalTx.Addr[i];
        if (!a)
            continue;

        if (GimbalTx.DisableLeft[i]) {
            ZdtCan_Enable(a, 0);            /* 松轴：电机控制板失能，人手可拨 */
            GimbalTx.DisableLeft[i]--;
        }
        else if (GimbalTx.EnableLeft[i]) {
            ZdtCan_Enable(a, 1);
            GimbalTx.EnableLeft[i]--;
        }
        else if (GimbalTx.ClearLeft[i]) {
            ZdtCan_ClearStall(a);
            GimbalTx.ClearLeft[i]--;
        }
        else if (GimbalTx.FeedbackLeft[i]) {
            ZdtCan_SetTimedReturn(a, 0x36, GIMBAL_FEEDBACK_MS);
            GimbalTx.FeedbackLeft[i]--;
        }
    }
}

/* 三轴目标转速各发一条 F6 单机帧（使能中发目标转速，失能时 Rpm 已被压 0）。
   为什么不塞进底盘那条 0xAA 广播：广播只放 4 轮 = 32 字节子命令 / 整帧 37 字节 /
   6 个 CAN 包，这是实测一直好用的尺寸；再加 3 轴会变成 61 字节 / 9 个包，
   现场实测七个电机一条都不执行（手册 5.3.1 的示例最长也只有 34 字节 / 5 个包，
   多电机命令很可能有长度上限）。拆成单机帧后某轴丢一帧只影响那一轴，
   不会再连累整条广播。仍然只由底盘任务调用——同一个任务上下文，不会抢邮箱。 */
void Gimbal_SendAxisSpeedFrames(void)
{
    /* 松轴期间一条 F6 都不发：F6 可能让驱动重新使能，人手就拨不动了。
       正常失能（不松轴）仍然照发 0，保证电机保持锁轴、不会因为停发速度帧
       而记住最后一拍的速度。 */
    if (Gimbal.isReleased)
        return;

    for (uint8_t i = 0; i < 3; i++)
        if (s_AxisInstalled[i] && GimbalTx.Addr[i])
            ZdtMotor_SetSpeed(s_Axis[i], GimbalTx.Rpm[i]);
}
#else
void Gimbal_TxService(void) {}
void Gimbal_SendAxisSpeedFrames(void) {}
void Gimbal_GetRelDeg(float out[3])
{
    out[0] = out[1] = out[2] = 0.0f;
}
#endif

void GimbalTask(void const* argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();

#if GIMBAL_MOTOR_INSTALLED
    /* ---- 初始化三轴电机（地址 5~7），把地址交给底盘任务 ---- */
    ZdtMotor_Init(&Gimbal.Yaw.Motor,    ZDT_MOTOR_YAW);
    ZdtMotor_Init(&Gimbal.Lift.Motor,   ZDT_MOTOR_LIFT);
    ZdtMotor_Init(&Gimbal.Extend.Motor, ZDT_MOTOR_EXTEND);
    for (uint8_t i = 0; i < 3; i++) {
        GimbalTx.Addr[i] = s_Axis[i]->Addr;
        GimbalTx.Rpm[i]  = 0.0f;
        if (s_AxisInstalled[i])
            GimbalTx.FeedbackLeft[i] = 3;    /* 上电分成 3 拍发，确保三轴都收到回传配置 */
    }
#endif

    /* ---- 初始化夹爪 / 摄像头舵机 ---- */
    Servo_Init(&Gimbal.GripperServo, SERVO_GRIPPER_CH, SERVO_PULSE_MIN, SERVO_PULSE_MAX, SERVO_GRIPPER_INIT);
    Servo_Init(&Gimbal.CamServo,     SERVO_CAM_CH,     SERVO_PULSE_MIN, SERVO_PULSE_MAX, SERVO_CAM_INIT);

    Gimbal.isEnable   = 0;
    Gimbal.isReleased = 0;

    while (1)
    {
        osDelayUntil(&PreviousWakeTime, GIMBAL_TASK_UPDATE_TICK);

#if GIMBAL_MOTOR_INSTALLED
        uint32_t now = HAL_GetTick();

        /* ---- 位置回传配置定期补发：上电漏收一帧，这一轴就永远"离线"，
               速度会被压成 0（表现就是"某轴一点反应都没有"）。 ---- */
        if ((uint32_t)(now - s_FeedbackTick) >= GIMBAL_CONFIG_RETRY_MS) {
            s_FeedbackTick = now;
            for (uint8_t i = 0; i < 3; i++)
                if (s_AxisInstalled[i])
                    GimbalTx.FeedbackLeft[i] = 2;
        }

        /* ---- 树莓派示教的限位同步（下发过的轴覆盖默认值） ---- */
        Gimbal_SyncLimits();

        /* ---- 松轴（限位示教）：让电机控制板失能，人手可以拨动三轴 ----
           进入时连发两帧 F3 AB 00；期间不发 F6（见 Gimbal_SendAxisSpeedFrames），
           否则速度帧可能把驱动重新使能。退出时把使能流程重置，下一拍立刻重新使能。 */
        if (PiCommand.GimbalRelease)
        {
            if (!Gimbal.isReleased) {
                for (uint8_t i = 0; i < 3; i++)
                    if (s_AxisInstalled[i]) {
                        /* 清掉还没发完的"使能/解堵转"：它们排在松轴之后发出去，
                           会把刚失能的驱动重新使能，人手就又拨不动了。 */
                        GimbalTx.EnableLeft[i]  = 0;
                        GimbalTx.ClearLeft[i]   = 0;
                        GimbalTx.DisableLeft[i] = 2;
                    }
                Gimbal.isReleased = 1;
                Gimbal.isEnable   = 0;
            }
        }
        else if (Gimbal.isReleased)
        {
            Gimbal.isReleased = 0;
            s_EnableTick      = 0;      /* 松轴结束：下端使能流程立即重跑一遍 */
        }

        /* ---- 使能判定：单一使能状态 PiCommand.Enabled（松轴期间不使能） ---- */
        if (PiCommand.Enabled && !Gimbal.isReleased)
        {
            if (!Gimbal.isEnable || (uint32_t)(now - s_EnableTick) >= GIMBAL_ENABLE_RETRY_MS) {
                /* 刚使能的那一下额外请求发几帧"解除堵转保护"：堵转保护在电机里是
                   锁存的，撞限位后位置反馈照常、但运动指令一律不执行。只在上升沿发，
                   不跟着 500ms 重发（否则轴顶着机械限位会反复解除保护、干磨发热）。 */
                uint8_t risingEdge = !Gimbal.isEnable;
                for (uint8_t i = 0; i < 3; i++)
                    if (s_AxisInstalled[i]) {
                        GimbalTx.EnableLeft[i] = 2;
                        if (risingEdge)
                            GimbalTx.ClearLeft[i] = 6;
                    }
                s_EnableTick = now;
            }
            Gimbal.isEnable = 1;
        }
        else
        {
            Gimbal.isEnable = 0;
        }

        /* ---- 上电中位锁存：每轴第一次收到反馈时记下（不管使能与否） ---- */
        for (uint8_t i = 0; i < 3; i++)
            if (s_AxisInstalled[i] && !s_OffsetValid[i] && s_Axis[i]->LastOnlineTick != 0) {
                s_Offset[i]      = s_Axis[i]->Position.Real;
                s_OffsetValid[i] = 1;
            }

        if (!Gimbal.isEnable)
        {
            /* 失能：目标速度压 0，并作废速度时间戳（重新使能不会接着旧速度转） */
            for (uint8_t i = 0; i < 3; i++) {
                GimbalTx.Rpm[i] = 0.0f;
                PiCommand.GimbalVel[i].Tick = 0;
            }
        }
        else
        {
            for (uint8_t i = 0; i < 3; i++)
            {
                if (!s_AxisInstalled[i]) {
                    GimbalTx.Rpm[i] = 0.0f;
                    PiCommand.GimbalVel[i].Tick = 0;
                    continue;
                }

                /* 新速度指令（未超时）→ 用摇杆速度；回中/超时 → 0 */
                uint32_t tick = PiCommand.GimbalVel[i].Tick;
                float cmd = (tick && (uint32_t)(now - tick) < GIMBAL_VEL_TIMEOUT_MS)
                          ? PiCommand.GimbalVel[i].DegPerSec : 0.0f;

                Gimbal_SetAxisSpeed(i, cmd);
            }
        }

        /* 位置/回零指令本版不支持：收到就直接丢弃，避免标志位一直挂着、以后误执行 */
        PiCommand.HomeMask = 0;
        for (uint8_t i = 0; i < 3; i++)
            PiCommand.GimbalTarget[i].Mask = 0;
#endif

        /* ---- 夹爪 / 摄像头舵机（仅使能时更新，失能保持末值） ---- */
        if (PiCommand.Enabled) {
            Servo_SetAngle(&Gimbal.GripperServo, PiCommand.Gripper);
            Servo_SetAngle(&Gimbal.CamServo,     PiCommand.Cam);
        }
    }
}
