/**
 * @attention   采用UTF-8字符集编码
 * @brief       云台控制任务
 * @details     三轴 ZDT 电机内部闭环，STM32 按轴在两种模式间切换：
 *              · 有新速度指令（PI_CMD_GIMBAL_VEL，150ms 内）→ 速度模式 0xF6（手柄摇杆用）；
 *              · 否则 → 绝对位置模式 0xFD（ZDT_POS_ABS，周期重发，幂等）。
 *              速度模式下目标位置跟着实时位置走，超时/切回位置模式时原地接住，不会跳回旧目标。
 *              首次使能后 20 ms 把当前位置设为零点（上电位置 = 0），之后一切绝对角度都以它为准。
 *              回零请求（PiCommand.HomeMask）在本任务执行，执行后清位。
 *              夹爪/摄像头舵机直接跟随树莓派指令。
 */

#include "header.h"

#define GIMBAL_TASK_UPDATE_TICK 20    /* 50 Hz：位置模式电机内部自持，指令重发只是保活/幂等刷新 */
#define GIMBAL_FEEDBACK_MS      20    /* 各轴位置回传周期（ms）：50 Hz，压低总线负载 */
#define GIMBAL_ENABLE_RETRY_MS  500   /* 使能重发周期（ms）：CAN1 关了自动重传，漏一次要等下次使能才补 */
#define GIMBAL_ZERO_SETTLE_MS   20    /* 使能后等多久再设零点（ms）：让电机的使能帧先被处理完 */
#define GIMBAL_VEL_TIMEOUT_MS  150   /* 速度模式超时：这么久没收到该轴新速度指令就退回位置保持（手柄掉线/切模式兜底）。
                                        手柄按 20Hz 发，单轴 50ms 一帧，150ms = 3 帧余量：
                                        丢一两帧不致于抖，真掉线 150ms 内必停（300°/s 最多空转 45°）。 */

Gimbal_t Gimbal;

#if GIMBAL_MOTOR_INSTALLED
/* 云台三轴的软限位（单位 °，见 RobotConfig.h） */
static const float s_MinDeg[3] = { GIMBAL_YAW_MIN,   GIMBAL_LIFT_MIN,   GIMBAL_EXTEND_MIN };
static const float s_MaxDeg[3] = { GIMBAL_YAW_MAX,   GIMBAL_LIFT_MAX,   GIMBAL_EXTEND_MAX };

static uint32_t s_EnableTick = 0;   /* 上次下发使能帧的时刻（周期重发用） */
static uint32_t s_ZeroTick   = 0;   /* 首次使能的时刻（设零点前的等待起点） */
static uint8_t  s_ZeroDone   = 0;   /* 上电零点是否已设（整个上电周期只做一次） */

/* 三轴的顺序固定为 0=Yaw 1=Lift 2=Extend，和 PiCommand.GimbalTarget[] / HomeMask 的位序一致 */
static ZdtMotor_t* const s_Axis[3] = {
    &Gimbal.Yaw.Motor, &Gimbal.Lift.Motor, &Gimbal.Extend.Motor
};

/* 各轴是否已装：没装的轴一条帧都不发——无 ACK 的帧会抬高 TEC、白占邮箱 */
static const uint8_t s_AxisInstalled[3] = {
    GIMBAL_YAW_INSTALLED, 1, 1
};

static const uint8_t s_HomeMode[3] = {
    ZDT_HOME_NEAR, ZDT_HOME_LIMIT, ZDT_HOME_LIMIT
};

/* 目标角度限幅后写入电机 */
static void Gimbal_SetAxisTarget(ZdtMotor_t* motor, uint8_t axis, float deg)
{
    motor->Position.Set = ConstrainF(deg, s_MaxDeg[axis], s_MinDeg[axis]);
    ZdtMotor_SetPosition(motor, motor->Position.Set, ZDT_POS_ABS);
}

/* 速度模式：把 °/s 限幅后下发 F6 速度帧。接近软限位时按停车带强制减速到 0，
   防止速度模式把轴推出限位撞结构。 */
static void Gimbal_SetAxisSpeed(ZdtMotor_t* motor, uint8_t axis, float degPerSec)
{
    const float stopBand = 5.0f;    /* 离限位多近（°）就开始强制停 */
    float now = motor->Position.Real;

    if ((degPerSec > 0.0f && now >= s_MaxDeg[axis] - stopBand) ||
        (degPerSec < 0.0f && now <= s_MinDeg[axis] + stopBand))
        degPerSec = 0.0f;

    ZdtMotor_SetSpeed(motor, degPerSec / 6.0f);   /* °/s → RPM（1 RPM = 6°/s） */

    /* 速度模式期间让绝对目标跟着实时位置走：速度超时/切回位置模式时，
       位置模式从"当前位置"原地接住，而不是跳回速度模式开始前的旧目标。 */
    motor->Position.Set = now;
}

/* 开定时返回：让已装的轴每 GIMBAL_FEEDBACK_MS(20ms) 主动回一帧实时位置(0x36)。
   不发这条命令，Position.Real 永远是 0、ZdtMotor_IsOnline() 永远为假，
   状态帧的在线掩码、失能时的原位保持、绝对位置的方向判断全都会失效。 */
static void Gimbal_SetupFeedback(void)
{
    for (uint8_t i = 0; i < 3; i++)
        if (s_AxisInstalled[i])
            ZdtCan_SetTimedReturn(s_Axis[i]->Addr, 0x36, GIMBAL_FEEDBACK_MS);
}

/* 云台三轴失能：速度模式下的轴先无条件发速度 0 停住，在线时再锁到当前位置做
   位置保持（不松轴，平滑保持）。
   · 速度 0 不能按"在线"才发：反馈抖动时 IsOnline 变假，一条停止帧都发不出去，
     速度模式下的轴会按最后速度一直转——这正是"云台没有失能逻辑"。
     速度帧是单帧、不依赖位置反馈，任何情况下都发得出去。
   · 只对"之前确实在速度模式"（Tick 非 0）的轴发速度 0：位置模式轴发速度 0 会
     把它切成速度环，保持力反而比位置保持弱，重轴会慢慢往下掉。
   · 离线时 Real 还停在 0，拿它当锚点等于命令轴"回绝对零点"，一按失能云台就会
     自己往零点跑——所以离线只停速度、不发位置。 */
static void Gimbal_Disable(void)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        if (!s_AxisInstalled[i]) {
            PiCommand.GimbalVel[i].Tick = 0;
            continue;
        }

        if (PiCommand.GimbalVel[i].Tick != 0)      /* 这一轴之前处于速度模式 → 先停住 */
            ZdtMotor_SetSpeed(s_Axis[i], 0.0f);

        if (ZdtMotor_IsOnline(s_Axis[i])) {
            ZdtMotor_SetPosition(s_Axis[i], s_Axis[i]->Position.Real, ZDT_POS_ABS);
            s_Axis[i]->Position.Set = s_Axis[i]->Position.Real;
        }

        /* 失能时把速度指令一起作废：否则超时内重新使能，会拿失能前的旧速度
           接着往下转（手柄那边此时可能根本没人按）。 */
        PiCommand.GimbalVel[i].Tick = 0;
    }
}
#endif

void GimbalTask(void const* argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();

#if GIMBAL_MOTOR_INSTALLED
    /* ---- 初始化三轴电机（地址 5~7） ---- */
    ZdtMotor_Init(&Gimbal.Yaw.Motor,    ZDT_MOTOR_YAW);
    ZdtMotor_Init(&Gimbal.Lift.Motor,   ZDT_MOTOR_LIFT);
    ZdtMotor_Init(&Gimbal.Extend.Motor, ZDT_MOTOR_EXTEND);

    Gimbal_SetupFeedback();   /* 先让已装的轴开始回位置，后面才判得了在线 */
#endif

    /* ---- 初始化夹爪 / 摄像头舵机 ---- */
    Servo_Init(&Gimbal.GripperServo, SERVO_GRIPPER_CH, SERVO_PULSE_MIN, SERVO_PULSE_MAX, SERVO_GRIPPER_INIT);
    Servo_Init(&Gimbal.CamServo,     SERVO_CAM_CH,     SERVO_PULSE_MIN, SERVO_PULSE_MAX, SERVO_CAM_INIT);

    Gimbal.isEnable = 0;

    while (1)
    {
        osDelayUntil(&PreviousWakeTime, GIMBAL_TASK_UPDATE_TICK);

#if GIMBAL_MOTOR_INSTALLED
        /* ---- 使能判定：单一使能状态 PiCommand.Enabled（手柄 start=使能 / B=失能，Pi 离线自主失能） ---- */
        if (PiCommand.Enabled)
        {
            /* 使能每 500 ms 重发一次，而不是只在 0→1 那一刻发。
               上电时电机可能比 MCU 晚就绪，或者那一帧正好碰上总线错误被丢掉；
               CAN1 关了自动重传，丢一次就等于这一轴永远失能，表现就是"三轴里有一轴不动"。 */
            if (!Gimbal.isEnable || HAL_GetTick() - s_EnableTick >= GIMBAL_ENABLE_RETRY_MS) {
                for (uint8_t i = 0; i < 3; i++)
                    if (s_AxisInstalled[i])
                        ZdtCan_Enable(s_Axis[i]->Addr, 1);
                Gimbal_SetupFeedback();   /* 电机重新上电会清掉定时返回配置，跟着补发 */
                s_EnableTick = HAL_GetTick();
                if (!s_ZeroDone)
                    s_ZeroTick = HAL_GetTick();
            }
            Gimbal.isEnable = 1;
        }
        else
        {
            Gimbal.isEnable = 0;
        }

        /* ---- 上电零点：使能后等 20 ms（让使能帧先被电机处理完），把"这一刻"的姿态设成 0。
               只做一次。之后所有绝对位置指令都以它为准，上电后云台不会自己往旧零点跑。 ---- */
        uint8_t justZeroed = 0;
        if (Gimbal.isEnable && !s_ZeroDone &&
            HAL_GetTick() - s_ZeroTick >= GIMBAL_ZERO_SETTLE_MS)
        {
            for (uint8_t i = 0; i < 3; i++) {
                if (!s_AxisInstalled[i])
                    continue;
                ZdtCan_SetZero(s_Axis[i]->Addr);
                /* 软件侧一起归零：否则第一条绝对位置指令会把轴拉回清零前的旧坐标 */
                s_Axis[i]->Position.Set      = 0;
                s_Axis[i]->Position.Real     = 0;
                s_Axis[i]->Position.RawCount = 0;
            }
            s_ZeroDone = 1;
            justZeroed = 1;     /* 这一拍不再发位置帧，等下一拍（20ms 后）零点肯定已经生效 */
        }

        if (!Gimbal.isEnable)
        {
            Gimbal_Disable();   /* 失能：发当前位置锁定（不松轴，平滑保持） */
        }
        else if (s_ZeroDone && !justZeroed)
        {
            /* 零点建立之前一条位置指令都不发：此刻 Set=0 指的是电机的旧零点，
               先发出去云台会先往旧零点窜一下，再被新零点拽回来。目标/回零请求会挂着，之后照样执行。 */

            /* ---- 回零请求（执行后清位，避免重复触发） ---- */
            for (uint8_t i = 0; i < 3; i++) {
                if (!(PiCommand.HomeMask & (1 << i)))
                    continue;
                if (s_AxisInstalled[i])
                    ZdtCan_Home(s_Axis[i]->Addr, s_HomeMode[i]);
                PiCommand.HomeMask &= ~(1 << i);   /* 没装的轴也清位，免得装上后补执行一次旧请求 */
            }

            /* ---- 三轴目标位置（有新指令才更新目标；位置命令周期重发，幂等） ---- */
            for (uint8_t i = 0; i < 3; i++) {
                if (!PiCommand.GimbalTarget[i].Mask)
                    continue;
                if (s_AxisInstalled[i])
                    Gimbal_SetAxisTarget(s_Axis[i], i, PiCommand.GimbalTarget[i].Deg);
                PiCommand.GimbalTarget[i].Mask = 0;
            }

            /* 三轴模式分支：有新鲜速度指令 → 速度模式；否则位置模式周期重发（幂等） */
            for (uint8_t i = 0; i < 3; i++) {
                if (!s_AxisInstalled[i])
                    continue;
                uint32_t tick = PiCommand.GimbalVel[i].Tick;
                if (tick && (HAL_GetTick() - tick) < GIMBAL_VEL_TIMEOUT_MS)
                    Gimbal_SetAxisSpeed(s_Axis[i], i, PiCommand.GimbalVel[i].DegPerSec);
                else
                    ZdtMotor_SetPosition(s_Axis[i], s_Axis[i]->Position.Set, ZDT_POS_ABS);
            }
        }
#endif

        /* ---- 夹爪 / 摄像头舵机（仅使能时更新，失能保持末值） ---- */
        if (PiCommand.Enabled) {
            Servo_SetAngle(&Gimbal.GripperServo, PiCommand.Gripper);
            Servo_SetAngle(&Gimbal.CamServo,     PiCommand.Cam);
        }
    }
}
