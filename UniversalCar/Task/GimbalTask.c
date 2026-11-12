/**
 * @attention   采用UTF-8字符集编码
 * @brief       云台控制任务（速度环底 + 软件位置环，只计算不发送）
 * @details     云台三轴和底盘轮子一样只收速度：本任务算出三轴目标转速写进 GimbalTx，
 *              由底盘任务在每拍的 4 轮 0xAA 广播之后，给三轴各发一条 F6 单机帧。
 *              为什么不让云台自己发：CAN1 只有 3 个发送邮箱，两个任务同时发会互相
 *              挤掉——多包广播丢一包就整帧作废（底盘"卡住"），单机帧丢一帧就是
 *              某个轴没反应。全总线只留一个发送者，这两个问题就都不存在。
 *
 *              两种控制源：
 *                · PI_CMD_GIMBAL_TARGET（角度）→ 位置环 PID → 速度；收到后保持
 *                  GIMBAL_POS_HOLD_MS，期间速度指令让位，过期后速度重新接管。
 *                · PI_CMD_GIMBAL_VEL（°/s）→ 线性比例，回中/超时 → 0。
 *                  收到**非零**速度帧立即夺回控制权、作废位置目标：视觉跟随是
 *                  纯速度环，不能等位置保持那 3s 过期（否则"看到物料却不动"）。
 *
 *              0 点：电机掉电不保存位置。每轴第一次收到反馈时先锁存一个临时 0 点；
 *              人工把三个轴摆到固定位后，树莓派/上位机发 PI_CMD_GIMBAL_ZERO，
 *              本任务把此刻的编码器值直接改成 0 点（不依赖电机内部零点）。
 *
 *              限位：按"相对 0 点的位置 + 运动方向"掐掉越界方向的速度。
 *              限位值默认取 RobotConfig.h 的宏；树莓派开机示教或上位机标定时用
 *              PI_CMD_GIMBAL_LIMIT 下发，运行时以收到的为准。
 *              但"判断"还要知道速度指令的正方向对应编码器变大还是变小——编码器
 *              计数方向和电机转向相反的轴（实测 Yaw），正速度是往编码器小的一头走。
 *              所以每轴有一个符号 GIMBAL_xx_CMD_SIGN，判断前先乘上它换到编码器坐标。
 *              符号填反的典型现象：顶到限位后往回拨不动，继续往越界方向反而能动。
 *              位置环的 PID 输出也活在这套坐标里，所以下发速度前同样乘符号。
 *
 *              松轴示教：PI_CMD_GIMBAL_RELEASE=1 时把三轴电机控制板失能（F3 AB 00），
 *              并且停发 F6（速度帧可能把驱动重新使能），人手才能自由拨动、读编码器。
 *              失能（正常整车失能，不是松轴）：目标速度直接置 0，底盘广播里每拍都发 0
 *              （不松轴，保持力还在）；位置目标一并作废，重新使能不会自己跑去旧目标。
 *              夹爪/摄像头舵机直接跟随树莓派指令。
 */

#include "header.h"

#define GIMBAL_TASK_UPDATE_TICK    5    /* 200 Hz：和底盘同频计算（总线预算见 ChassisTask.c） */
#define GIMBAL_FEEDBACK_MS        10    /* 三轴位置回传周期（ms）：100 Hz，只用于限位判断 */
#define GIMBAL_ENABLE_RETRY_MS   500    /* 使能重发周期（ms） */
#define GIMBAL_CONFIG_RETRY_MS   500    /* 位置回传配置重发周期（ms） */
#define GIMBAL_VEL_TIMEOUT_MS    300    /* 这么久没收到该轴新速度就发 0 */
#define GIMBAL_POS_HOLD_MS      3000    /* 收到角度目标后位置环保持多久（期间速度指令让位） */
/* 位置环误差限幅（°）。P 项最大值 = 这个值 × Kp，原来 60 会把 Kp=15 的轴
   卡在 900°/s：上位机把该轴最大速度调到 900 以上也提不上去。
   现在放到 200，保证在调参页合法速度（≤2000°/s）内 P 项都够用，
   真正的速度上限只剩 PiTune.Gimbal*Speed 一处（见 Gimbal_SyncSpeedLimit）。 */
#define GIMBAL_POS_MAX_ERROR    200.0f
#define GIMBAL_POS_INTEGRAL_BAND 8.0f   /* 误差小于它才积分，避免大误差时积分饱和 */

Gimbal_t  Gimbal;
GimbalTx_t GimbalTx;

#if GIMBAL_MOTOR_INSTALLED
/* 云台三轴的软限位（相对 0 点，单位 °）。
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

/* 0 点偏移：每轴第一次收到位置反馈时先锁存一个临时 0 点；
   PI_CMD_GIMBAL_ZERO 到达后直接改成"当前编码器值"，限位/遥测按 (Real - Offset)。 */
static float   s_Offset[3]      = { 0.0f, 0.0f, 0.0f };
static uint8_t s_OffsetValid[3] = { 0, 0, 0 };

/* ---- 位置环状态（角度目标 → 速度） ---- */
/* 三轴位置环输出限幅（°/s）。**唯一来源是上位机调参**（0x14~0x16）：
   PiTune 在 PiComm_TuneReset 里给上电初值，之后运行中只认上位机下发的值，
   固件里不再有任何速度宏参与。0 = 该轴输出锁 0。
   运行中改动由 Gimbal_SyncSpeedLimit 热更新（含 PID 限幅）。
   这里初值给 0，真正的值在 Gimbal_InitPosPid 里先 Sync 再建 PID。 */
static float s_MaxSpeedDegPerSec[3] = { 0.0f, 0.0f, 0.0f };

/* 位置环状态要在 Gimbal_SyncSpeedLimit 之前声明：那个函数里会直接改 PID 限幅。 */
static float    s_TargetDeg[3]   = { 0.0f, 0.0f, 0.0f };
static uint8_t  s_TargetValid[3] = { 0, 0, 0 };
static uint32_t s_TargetTick[3]  = { 0, 0, 0 };
static PID_t    s_PosPid[3];
static uint8_t  s_PosPidReady     = 0;

static void Gimbal_SyncSpeedLimit(void)
{
    const float tune[3] = { PiTune.GimbalYawSpeed, PiTune.GimbalLiftSpeed,
                            PiTune.GimbalExtSpeed };
    for (uint8_t i = 0; i < 3; i++) {
        /* 只用上位机值：负值/NaN 按 0 处理，固件里没有可回退的速度宏。
           PiComm_TuneReset() 里的数字只是树莓派连上前的初值。 */
        float spd = tune[i];
        if (!(spd > 0.0f))
            spd = 0.0f;
        if (spd == s_MaxSpeedDegPerSec[i])
            continue;
        s_MaxSpeedDegPerSec[i] = spd;
        if (s_PosPidReady) {                 /* PID 限幅跟着改，立即生效 */
            s_PosPid[i].MaxOut = spd;
            s_PosPid[i].I_Max  = spd;
        }
    }
}

/* yaw/lift/extend 三轴各自记忆最后生效的增益（-1 = 尚未初始化） */
static float    s_LastGimbalKp[3] = { -1.0f, -1.0f, -1.0f };
static float    s_LastGimbalKi[3] = { -1.0f, -1.0f, -1.0f };
static float    s_LastGimbalKd[3] = { -1.0f, -1.0f, -1.0f };

/* 三轴增益表：PiTune 里 yaw/lift/extend 各自独立 */
static void Gimbal_TuneGains(float kp[3], float ki[3], float kd[3])
{
    kp[0] = PiTune.GimbalKp;     ki[0] = PiTune.GimbalKi;     kd[0] = PiTune.GimbalKd;
    kp[1] = PiTune.GimbalLiftKp; ki[1] = PiTune.GimbalLiftKi; kd[1] = PiTune.GimbalLiftKd;
    kp[2] = PiTune.GimbalExtKp;  ki[2] = PiTune.GimbalExtKi;  kd[2] = PiTune.GimbalExtKd;
}

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

/* 三轴当前位置（相对 0 点，°）。没锁存基准前给 0，遥测不会发 NaN。 */
void Gimbal_GetRelDeg(float out[3])
{
    for (uint8_t i = 0; i < 3; i++)
        out[i] = s_OffsetValid[i]
               ? (s_Axis[i]->Position.Real - s_Offset[i])
               : 0.0f;
}

void Gimbal_GetLimits(float minDeg[3], float maxDeg[3])
{
    for (uint8_t i = 0; i < 3; i++) {
        minDeg[i] = s_MinDeg[i];
        maxDeg[i] = s_MaxDeg[i];
    }
}

/* ============ 位置环 ============ */

/* 位置环 PID 初始化：输出直接是 °/s（编码器坐标），限幅用各轴速度上限。
   增益初值取 PiTune，之后每拍检查一次，上位机改了就用 PID_SetGains 热更新。 */
static void Gimbal_InitPosPid(void)
{
    float kp[3], ki[3], kd[3];
    Gimbal_TuneGains(kp, ki, kd);
    Gimbal_SyncSpeedLimit();               /* 先同步速度限幅，PID_Init 用它做 MaxOut */

    for (uint8_t i = 0; i < 3; i++) {
        PID_InitStruct_t p;
        p.kP = kp[i];
        p.kI = ki[i];
        p.kD = kd[i];
        p.DeltaTime = GIMBAL_TASK_UPDATE_TICK / 1000.0f;
        p.DifferentialFreqDiv = 1;
        p.CircleResolution = 0.0f;
        p.MaxError = GIMBAL_POS_MAX_ERROR;
        p.MaxOutput = s_MaxSpeedDegPerSec[i];
        p.I_Max = s_MaxSpeedDegPerSec[i];
        p.DeadBand = GIMBAL_POS_DEADBAND_DEG;
        p.IntegralBand = GIMBAL_POS_INTEGRAL_BAND;
        PID_Init(&s_PosPid[i], &p);
        s_LastGimbalKp[i] = kp[i];
        s_LastGimbalKi[i] = ki[i];
        s_LastGimbalKd[i] = kd[i];
    }
    s_PosPidReady  = 1;
}

/* 上位机调参：每轴独立比对，只有该轴增益真的变了才重装（PID_SetGains 会清积分）。 */
static void Gimbal_ApplyTune(void)
{
    if (!s_PosPidReady)
        return;
    float kp[3], ki[3], kd[3];
    Gimbal_TuneGains(kp, ki, kd);
    for (uint8_t i = 0; i < 3; i++) {
        if (kp[i] == s_LastGimbalKp[i] &&
            ki[i] == s_LastGimbalKi[i] &&
            kd[i] == s_LastGimbalKd[i])
            continue;
        s_LastGimbalKp[i] = kp[i];
        s_LastGimbalKi[i] = ki[i];
        s_LastGimbalKd[i] = kd[i];
        PID_SetGains(&s_PosPid[i], kp[i], ki[i], kd[i]);
    }
    Gimbal_SyncSpeedLimit();               /* 0x14~0x16 改了速度限幅也随版本号热更新 */
}

/* 置零：把三个轴此刻的编码器值定义成 0 点。
   不调用电机内部 ZdtCan_SetZero——固件所有坐标都是 (Real - Offset)，
   软件改偏移就够了，也避免动电机内部状态影响速度模式。 */
static void Gimbal_ApplyZero(void)
{
    uint8_t mask = PiCommand.GimbalZeroMask;
    if (!mask)
        return;
    PiCommand.GimbalZeroMask = 0;

    for (uint8_t i = 0; i < 3; i++) {
        if (!(mask & (1u << i)) || !s_AxisInstalled[i])
            continue;
        if (s_Axis[i]->LastOnlineTick == 0)     /* 从没收到过反馈，不知道当前位置 */
            continue;

        s_Offset[i]      = s_Axis[i]->Position.Real;
        s_OffsetValid[i] = 1;

        /* 置零后旧目标全部作废，否则会突然往旧坐标跑 */
        s_TargetValid[i]            = 0;
        s_TargetTick[i]             = 0;
        PiCommand.GimbalTarget[i].Mask = 0;
        PiCommand.GimbalVel[i].Tick    = 0;
        GimbalTx.Rpm[i]                = 0.0f;
        if (s_PosPidReady)
            PID_Clear(&s_PosPid[i]);
    }
}

/* 把新到的角度目标收进位置环；目标先按软限位夹一刀。 */
static void Gimbal_TakeTargets(uint32_t now)
{
    for (uint8_t i = 0; i < 3; i++) {
        if (!PiCommand.GimbalTarget[i].Mask)
            continue;
        PiCommand.GimbalTarget[i].Mask = 0;

        if (!s_AxisInstalled[i])
            continue;

        float newDeg = ConstrainF(PiCommand.GimbalTarget[i].Deg,
                                  s_MaxDeg[i], s_MinDeg[i]);
        /* 只有目标真的变了才清 PID：上位机会按 1Hz 重发同一个目标，给 F407 的
           3s 位置保持续命。目标没变时保留积分/微分，否则 Ki/Kd 会被每秒清零。
           若位置目标已经过期（期间可能被速度模式带走过），即使角度相同也要清。 */
        uint8_t expired = (uint8_t)(!s_TargetValid[i] ||
                                    (uint32_t)(now - s_TargetTick[i]) >=
                                        GIMBAL_POS_HOLD_MS);
        uint8_t changed = (uint8_t)(expired ||
                                    newDeg > s_TargetDeg[i] + 1e-4f ||
                                    newDeg < s_TargetDeg[i] - 1e-4f);

        s_TargetDeg[i]   = newDeg;
        s_TargetValid[i] = 1;
        s_TargetTick[i]  = now;

        /* 位置指令立即接管：清速度时间戳，别让旧速度再掺和 */
        PiCommand.GimbalVel[i].Tick = 0;
        if (s_PosPidReady && changed)
            PID_Clear(&s_PosPid[i]);
    }
}

/* 该轴的位置目标是否还在有效期内（期间速度指令让位）。 */
static uint8_t Gimbal_PosActive(uint8_t i, uint32_t now)
{
    return (uint8_t)(s_TargetValid[i] &&
                     (uint32_t)(now - s_TargetTick[i]) < GIMBAL_POS_HOLD_MS);
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
void Gimbal_GetLimits(float minDeg[3], float maxDeg[3])
{
    /* 没装云台也回传固件默认值，上位机界面不会显示成一排 0 */
    minDeg[0] = GIMBAL_YAW_MIN;    maxDeg[0] = GIMBAL_YAW_MAX;
    minDeg[1] = GIMBAL_LIFT_MIN;   maxDeg[1] = GIMBAL_LIFT_MAX;
    minDeg[2] = GIMBAL_EXTEND_MIN; maxDeg[2] = GIMBAL_EXTEND_MAX;
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

    /* ---- 位置环 PID（增益初值来自 PiTune，上位机可在线改） ---- */
    Gimbal_InitPosPid();
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

        /* ---- 位置环增益热更新 / 置零（人工把轴摆到固定位后上位机下发） ---- */
        Gimbal_ApplyTune();
        Gimbal_ApplyZero();

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
            /* 失能：目标速度压 0，位置/速度目标一并作废
               （重新使能后不会自己跑去失能前留下的旧目标） */
            for (uint8_t i = 0; i < 3; i++) {
                GimbalTx.Rpm[i] = 0.0f;
                PiCommand.GimbalVel[i].Tick    = 0;
                PiCommand.GimbalTarget[i].Mask = 0;
                s_TargetValid[i] = 0;
                s_TargetTick[i]  = 0;
                if (s_PosPidReady)
                    PID_Clear(&s_PosPid[i]);
            }
        }
        else
        {
            /* 新到的角度目标先收进来（按软限位夹一刀） */
            Gimbal_TakeTargets(now);

            for (uint8_t i = 0; i < 3; i++)
            {
                if (!s_AxisInstalled[i]) {
                    GimbalTx.Rpm[i] = 0.0f;
                    PiCommand.GimbalVel[i].Tick = 0;
                    continue;
                }

                /* 速度指令夺回控制权（只认非零速度）。
                   视觉跟踪（color_pick / track_debug）是纯速度环：它每秒发 30 帧
                   /cmd_gimbal_vel。如果还让位置保持压着，最近一条 gimbal 目标会让
                   这一轴继续按位置环跑满 GIMBAL_POS_HOLD_MS（3s），期间视觉发的速度
                   全被丢掉——表现就是"画面里明明已经看到物料，云台却停三四秒才动"。
                   零速不算：视觉收工会补一帧 0，手柄松手每拍也在发 0，这些不能把
                   后面紧跟的 ext 前伸 / 序列里的 gimbal 目标作废。 */
                {
                    uint32_t vt = PiCommand.GimbalVel[i].Tick;
                    if (vt && (uint32_t)(now - vt) < GIMBAL_VEL_TIMEOUT_MS &&
                        fabsf(PiCommand.GimbalVel[i].DegPerSec) > 0.5f &&
                        s_TargetValid[i]) {
                        s_TargetValid[i] = 0;
                        s_TargetTick[i]  = 0;
                        PID_Clear(&s_PosPid[i]);
                    }
                }

                /* 位置目标有效期内：位置环接管，手柄速度让位 */
                if (Gimbal_PosActive(i, now)) {
                    float rel = s_Axis[i]->Position.Real - s_Offset[i];
                    PID_Calc(&s_PosPid[i], rel, s_TargetDeg[i]);
                    /* PID 输出是"让编码器坐标变大"的速度，换回速度指令坐标 */
                    Gimbal_SetAxisSpeed(i, s_PosPid[i].Output * s_CmdSign[i]);
                    continue;
                }

                /* 目标过期：清掉，退回速度模式（不清的话下次使能会补跑旧目标） */
                if (s_TargetValid[i]) {
                    s_TargetValid[i] = 0;
                    s_TargetTick[i]  = 0;
                    PID_Clear(&s_PosPid[i]);
                }

                /* 新速度指令（未超时）→ 用摇杆速度；回中/超时 → 0 */
                uint32_t tick = PiCommand.GimbalVel[i].Tick;
                float cmd = (tick && (uint32_t)(now - tick) < GIMBAL_VEL_TIMEOUT_MS)
                          ? PiCommand.GimbalVel[i].DegPerSec : 0.0f;

                Gimbal_SetAxisSpeed(i, cmd);
            }
        }

        /* 电机内置寻零本版不支持：收到直接丢弃，避免标志位一直挂着、以后误执行。
           位置目标不再是丢弃，而是由上面的位置环接管。 */
        PiCommand.HomeMask = 0;
#endif

        /* ---- 夹爪 / 摄像头舵机（仅使能时更新，失能保持末值） ---- */
        if (PiCommand.Enabled) {
            Servo_SetAngle(&Gimbal.GripperServo, PiCommand.Gripper);
            Servo_SetAngle(&Gimbal.CamServo,     PiCommand.Cam);
        }
    }
}
