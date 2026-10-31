/**
 * @attention   采用UTF-8字符集编码
 * @brief       底盘控制任务
 * @details     麦克纳姆 X 布局逆解算（车体坐标系：x 前进，y 左移，ω 逆时针为正）：
 *                v_fl = Vx - Vy - Wz*(Lx+Ly)      v_fr = Vx + Vy + Wz*(Lx+Ly)
 *                v_rl = Vx + Vy - Wz*(Lx+Ly)      v_rr = Vx - Vy + Wz*(Lx+Ly)
 *              线速度 → 轮 RPM：rpm = v/(2πr)*60，再乘安装方向系数。
 *              ZDT 电机内部闭环，本任务直接下发速度指令（周期重发兼作心跳）。
 */

#include "header.h"

/* 200 Hz。总线预算：每拍 5 包广播(8B) + 3 条云台单机帧(7B)，最坏装填 ~1250 bit
   ≈ 2.5ms@500k；7 个电机 10ms 回帧再占 ~21%。200Hz 合计约 76%，留 1/4 余量。
   （100Hz≈48%，250Hz≈90% 已经很挤，理论上限约 285Hz；再快只能把回传周期放长） */
#define CHASSIS_TASK_UPDATE_TICK 5

/* 编码器计数 → 轮位移系数：65536 计数 = 1 圈 = 2πR，即 1 计数 ≈ 3.76µm */
#define WHEEL_M_PER_COUNT  ((2.0f * PI * WHEEL_RADIUS_M) / 65536.0f)

/* 多电机广播（0xAA）组帧缓冲。全总线只有本任务一个发送者，每拍组一条帧：
   只放 4 轮速度（32 字节子命令 → 整帧 37 字节 / 6 个 CAN 包，实测一直好用）。
   云台三轴不塞进这条广播：7 条子命令会撑到 61 字节 / 9 个 CAN 包，现场实测电机
   一条都不执行（手册示例最长 34 字节 / 5 包）。云台三轴改用单机 F6 帧，
   由 Gimbal_SendAxisSpeedFrames() 紧随广播之后、在同一个任务里发出。
   云台的一次性命令（使能/解堵转/回传配置）也由 Gimbal_TxService() 在本任务里发。
   两个任务各发各的会出两种问题：两条 0xAA 序列共用 CAN ID 0x000/0x001… 电机按包
   序号重组会串包；而且抢 CAN1 的 3 个发送邮箱，广播丢一包整帧作废（底盘卡住）。 */
static ZdtMulti_t s_MultiCtx;

/* ==================== 航向保持 PID 参数（度域：误差 ° → 输出 °/s） ====================
   IMU Yaw 作反馈，PID 输出当作车体自旋速度 Wz。目标航向由 spin 滑杆给出（-180~180°）。
   符号搞反会形成正反馈：误差永远顶在 ±180，输出 = kP×180 呈方波、车原地打转。 */

#define CHASSIS_HEADING_ENABLE        1       /* 1=启用航向修正 0=退回手动自旋指令 */

/* 反馈方向系数：Wz>0 = 车体逆时针转（俯视）；JY901S Yaw 也是逆时针为正 → 两者一致取 +1。
   实测确认：手动逆时针转车，Yaw 变大则 +1，变小则 -1。（模块倒装时方向会再反一次） */
#define CHASSIS_HEADING_YAW_DIR       (1.0f)

#define CHASSIS_HEADING_KP            4.0f    /* 比例，°/s 每 °；太小修正无力，太大抖 */
#define CHASSIS_HEADING_KI            0.0f    /* 积分：10Hz 回传下先关，否则助长震荡 */
#define CHASSIS_HEADING_KD            0.0f    /* 微分：回传率低噪声大，先关 */
#define CHASSIS_HEADING_MAX_W         0.6f    /* 修正自旋上限 rad/s（≈34°/s） */
#define CHASSIS_HEADING_DEADBAND      0.0f    /* °，死区，防静止时抖 */
#define CHASSIS_HEADING_INTEGRAL_BAND 15.0f   /* °，积分分离 */
#define CHASSIS_HEADING_I_MAX         0.5f    /* 积分项限幅 rad/s */

/* ==================== 回零（位置闭环）参数 ====================
   世界系位置误差 m → PID → 世界系速度 m/s，再转车体系下发。
   目标恒为里程计零点 (0,0)，即上电位置。KP/KI 单位 (m/s)/m。 */
#define CHASSIS_RETURN_KP        1.5f    /* 比例：1m 误差给 1.5 m/s（被 MAX_V 限住） */
#define CHASSIS_RETURN_KI        0.3f    /* 积分：消除稳态误差，让车真正压到零点 */
#define CHASSIS_RETURN_KD        0.2f    /* 微分：先关 */
#define CHASSIS_RETURN_MAX_V     0.5f    /* 回零最大速度 m/s */
#define CHASSIS_RETURN_I_MAX     0.3f    /* 积分项限幅 m/s */
#define CHASSIS_RETURN_INTEGRAL_BAND 1.0f /* m，误差小于此值才积分，防大误差积分饱和 */
#define CHASSIS_RETURN_ARRIVED   0.02f   /* m，距离零点小于此值判到位并停车 */

Chassis_t Chassis;

/* 每轮方向系数（按实际安装方向标定 ±1，见 RobotConfig.h） */
static const int s_WheelDir[4] = { WHEEL_FL_DIR, WHEEL_FR_DIR, WHEEL_RL_DIR, WHEEL_RR_DIR };

/* 麦克纳姆逆解算：车体速度 → 四轮 RPM */
static void Chassis_MecanumCalc(float* rpm)
{
    float Lx = WHEEL_HALF_WHEELBASE_M;
    float Ly = WHEEL_HALF_TRACK_M;

    float v[4];
    v[0] = Chassis.Move.Vx - Chassis.Move.Vy - Chassis.Move.Wz * (Lx + Ly);    /* FL */
    v[1] = Chassis.Move.Vx + Chassis.Move.Vy + Chassis.Move.Wz * (Lx + Ly);    /* FR */
    v[2] = Chassis.Move.Vx + Chassis.Move.Vy - Chassis.Move.Wz * (Lx + Ly);    /* RL */
    v[3] = Chassis.Move.Vx - Chassis.Move.Vy + Chassis.Move.Wz * (Lx + Ly);    /* RR */

    for (int i = 0; i < 4; i++)
        rpm[i] = v[i] / (2 * PI * WHEEL_RADIUS_M) * 60.f * s_WheelDir[i]; //换算成rpm
}

/* 等比例限幅（保证方向不失真） */
static void Chassis_WheelRpmLimit(float* rpm)
{
    float maxAbs = 0;
    for (int i = 0; i < 4; i++)
        if (fabsf(rpm[i]) > maxAbs)
            maxAbs = fabsf(rpm[i]);

    if (maxAbs > CHASSIS_MAX_WHEEL_RPM) {
        float scale = CHASSIS_MAX_WHEEL_RPM / maxAbs;
        for (int i = 0; i < 4; i++)
            rpm[i] *= scale;
    }
}

#if CHASSIS_HEADING_ENABLE
/* 航向目标：手动模式下由转向指令积分得到，位姿模式下来自 POSE 指令 */
static uint8_t  s_HeadingValid  = 0;
static float    s_HeadingTarget = 0.0f;
static uint32_t s_HeadingTick   = 0;
#endif

/* 底盘停车（清内部状态；电机靠失能分支周期发的速度 0 平滑减速并锁轴） */
static void Chassis_Disable(void)
{
    Chassis.Move.Vx = 0;
    Chassis.Move.Vy = 0;
    Chassis.Move.Wz = 0;
    PID_Clear(&Chassis.HeadingPid);   /* 清航向 PID 积分，避免下次使能时残留 */
    PID_Clear(&Chassis.PosPidX);
    PID_Clear(&Chassis.PosPidY);
    PiCommand.ReturnHome = 0;         /* 失能时取消未完成的回零 */
#if CHASSIS_HEADING_ENABLE
    s_HeadingValid = 0;               /* 下次使能重新锁存当前朝向，不往回拧 */
#endif
}

/* 航向 PID 初始化（度域：误差 ° → 输出 °/s） */
static void Chassis_HeadingPidInit(void)
{
    PID_InitStruct_t Param;

    Param.kP = CHASSIS_HEADING_KP;
    Param.kI = CHASSIS_HEADING_KI;
    Param.kD = CHASSIS_HEADING_KD;
    Param.DeltaTime = CHASSIS_TASK_UPDATE_TICK * 0.001f;
    Param.DifferentialFreqDiv = 1;

    Param.CircleResolution = 360.0f;                          /* Yaw 量程 ±180°，过零走劣弧 */

    Param.MaxError = 180.0f;
    Param.MaxOutput = CHASSIS_HEADING_MAX_W * 180.0f / PI;    /* rad/s → °/s */
    Param.I_Max = CHASSIS_HEADING_I_MAX * 180.0f / PI;

    Param.DeadBand = CHASSIS_HEADING_DEADBAND;
    Param.IntegralBand = CHASSIS_HEADING_INTEGRAL_BAND;

    PID_Init(&Chassis.HeadingPid, &Param);
}

/* 回零位置环初始化（世界系，米域：误差 m → 输出 m/s） */
static void Chassis_PosPidInit(void)
{
    PID_InitStruct_t Param;

    Param.kP = CHASSIS_RETURN_KP;
    Param.kI = CHASSIS_RETURN_KI;
    Param.kD = CHASSIS_RETURN_KD;
    Param.DeltaTime = CHASSIS_TASK_UPDATE_TICK * 0.001f;
    Param.DifferentialFreqDiv = 1;

    Param.CircleResolution = 0;          /* 线性量，不用角度环绕 */
    Param.MaxError = 10.0f;              /* 最大误差 m */
    Param.MaxOutput = CHASSIS_RETURN_MAX_V;
    Param.I_Max = CHASSIS_RETURN_I_MAX;
    Param.DeadBand = 0.0f;
    Param.IntegralBand = CHASSIS_RETURN_INTEGRAL_BAND;

    PID_Init(&Chassis.PosPidX, &Param);
    PID_Init(&Chassis.PosPidY, &Param);
}

/* 里程计更新：编码器整数计数差分 → 轮位移 → 麦轮正解 → 车体位移 → 世界系位置。
   位置用整数计数差分直接积分，不经速度/浮点中转，精度最高（1 计数 ≈ 3.76µm）。
   实际速度(Chassis.Vel / WheelRpm)另算，供阻尼/交叉校验用，不进位置累加。 */
static void Chassis_OdometryUpdate(void)
{
    static uint32_t s_OdomTick = 0;

    /* 首帧：等 4 轮都收到编码器反馈后再初始化差分基准，避免起始跳变 */
    if (!Chassis.OdomReady) {
        for (int i = 0; i < 4; i++)
            if (!ZdtMotor_IsOnline(&Chassis.Wheel[i])) return;
        for (int i = 0; i < 4; i++)
            Chassis.WheelLastCount[i] = Chassis.Wheel[i].Position.RawCount;
        s_OdomTick = HAL_GetTick();
        Chassis.OdomReady = 1;
        return;
    }

    /* 控制周期（5ms）比编码器回传周期（10ms）快：没有新数据的那一拍 dCount 恒为 0，
       按固定 dt 去算会把实际速度打成 0、下一拍再翻倍。所以只在真的收到新计数时积分，
       dt 取两次积分之间实际经过的时间；连续几拍都没有新数据再认为车停了。 */
    uint32_t now = HAL_GetTick();
    int32_t dCount[4];
    uint8_t fresh = 0;

    for (int i = 0; i < 4; i++) {
        dCount[i] = Chassis.Wheel[i].Position.RawCount - Chassis.WheelLastCount[i];
        if (dCount[i] != 0)
            fresh = 1;
    }
    if (!fresh) {
        if ((uint32_t)(now - s_OdomTick) >= 4 * ZDT_FEEDBACK_PERIOD_MS) {
            Chassis.Vel.Vx = Chassis.Vel.Vy = Chassis.Vel.Wz = 0.0f;
            for (int i = 0; i < 4; i++)
                Chassis.WheelRpm[i] = 0.0f;
        }
        return;
    }

    float dt = (now - s_OdomTick) * 0.001f;
    if (dt <= 0.0f)
        return;
    s_OdomTick = now;

    float L = WHEEL_HALF_WHEELBASE_M + WHEEL_HALF_TRACK_M;
    float dv[4];

    for (int i = 0; i < 4; i++) {
        Chassis.WheelLastCount[i] = Chassis.Wheel[i].Position.RawCount;
        Chassis.WheelRpm[i] = (float)dCount[i] * 60.0f / (65536.0f * dt);   /* 实际滚动 RPM */
        dv[i] = (float)dCount[i] * WHEEL_M_PER_COUNT * s_WheelDir[i];        /* 麦轮分量位移 m */
    }

    /* 麦轮正解：轮位移 → 车体位移/转角（系数为常数，位移级也线性成立） */
    float dx = ( dv[0] + dv[1] + dv[2] + dv[3]) * 0.25f;
    float dy = (-dv[0] + dv[1] + dv[2] - dv[3]) * 0.25f;
    float dtheta = (-dv[0] + dv[1] - dv[2] + dv[3]) / (4.0f * L);

    /* 实际车体速度（m/s, rad/s） */
    Chassis.Vel.Vx = dx / dt;
    Chassis.Vel.Vy = dy / dt;
    Chassis.Vel.Wz = dtheta / dt;

    /* 车体位移 → 世界系（IMU yaw 旋转），累加为里程计位置 */
    float yawRad = Imu.Yaw * PI / 180.0f;
    float cosY = cosf(yawRad);
    float sinY = sinf(yawRad);
    Chassis.Pos.X += dx * cosY - dy * sinY;
    Chassis.Pos.Y += dx * sinY + dy * cosY;
}

/* 位置闭环：世界系位置误差 → PID → 世界系速度 → 转车体系，覆盖 Vx/Vy。
   目标可以是 Pi 下发的任意位姿(PI_CMD_POSE)，也可以是蓝牙回零的零点(0,0)。
   Pi 位姿目标到达后由 Pi 侧判定并下发下一目标，这里持续 PID 锁定、不自动停；
   蓝牙回零是"一次性"的：到位即停、清请求、清积分。 */
static void Chassis_PoseControl(float targetX, float targetY)
{
    float yawRad = Imu.Yaw * PI / 180.0f;
    float cosY = cosf(yawRad);
    float sinY = sinf(yawRad);

    /* 世界系误差 → PID → 世界系速度 */
    PID_Calc(&Chassis.PosPidX, Chassis.Pos.X, targetX);
    PID_Calc(&Chassis.PosPidY, Chassis.Pos.Y, targetY);
    float vxWorld = Chassis.PosPidX.Output;
    float vyWorld = Chassis.PosPidY.Output;

    /* 世界系速度 → 车体系速度（旋转矩阵的逆） */
    Chassis.Move.Vx =  vxWorld * cosY + vyWorld * sinY;
    Chassis.Move.Vy = -vxWorld * sinY + vyWorld * cosY;

    /* 蓝牙回零到位判定：距离零点足够近则停车、清请求、清积分 */
    if (PiCommand.ReturnHome &&
        sqrtf(Chassis.Pos.X * Chassis.Pos.X + Chassis.Pos.Y * Chassis.Pos.Y) < CHASSIS_RETURN_ARRIVED)
    {
        Chassis.Move.Vx = 0;
        Chassis.Move.Vy = 0;
        PiCommand.ReturnHome = 0;
        PID_Clear(&Chassis.PosPidX);
        PID_Clear(&Chassis.PosPidY);
    }
}

void ChassisTask(void const* argument)
{
    uint32_t PreviousWakeTime = osKernelSysTick();

    /* ---- 初始化 4 轮电机（地址 1~4） ---- */
    ZdtMotor_Init(&Chassis.Wheel[0], ZDT_MOTOR_FL);
    ZdtMotor_Init(&Chassis.Wheel[1], ZDT_MOTOR_FR);
    ZdtMotor_Init(&Chassis.Wheel[2], ZDT_MOTOR_RL);
    ZdtMotor_Init(&Chassis.Wheel[3], ZDT_MOTOR_RR);

    /* ---- 配置 4 轮定时返回实时位置(0x36)，供里程计/位置闭环使用 ----
       电机每隔 ZDT_FEEDBACK_PERIOD_MS 主动上报一次位置，由 ZdtCan_RxCallback 解析后
       写入 Wheel[i].Position.Real（单位 °，多圈绝对值，65536=一圈）。
       车轮速度可对 Position.Real 差分得到：v = (Pos_now - Pos_last) / dt。 */
    for (int i = 0; i < 4; i++)
        ZdtCan_SetTimedReturn(Chassis.Wheel[i].Addr, 0x36, ZDT_FEEDBACK_PERIOD_MS);

    /* ---- 初始化载物盘舵机 ---- */
    Servo_Init(&Chassis.PlateServo, SERVO_PLATE_CH, SERVO_PULSE_MIN, SERVO_PULSE_MAX, SERVO_PLATE_INIT);

    /* ---- 初始化航向 PID ---- */
    Chassis_HeadingPidInit();

    /* ---- 初始化回零位置环 ---- */
    Chassis_PosPidInit();

    /* ---- 里程计位置清零 + 基准未初始化 ---- */
    Chassis.Pos.X = 0;
    Chassis.Pos.Y = 0;
    Chassis.OdomReady = 0;
    for (int i = 0; i < 4; i++)
        Chassis.WheelLastCount[i] = 0;

    Chassis.isEnable = 0;

    while (1)
    {
        osDelayUntil(&PreviousWakeTime, CHASSIS_TASK_UPDATE_TICK);

        /* ---- 云台的一次性单机帧（使能 / 解除堵转 / 位置回传配置）----
               云台任务只置请求，这里由全总线唯一的发送者发出去：如果云台任务自己
               发，会和本任务连发的广播抢 CAN1 的 3 个发送邮箱，广播丢一包整帧作废
               （底盘卡住），云台的单机帧也可能整条被挤掉（X 轴没反应）。 ---- */
        Gimbal_TxService();

        /* ---- 周期性重发 4 轮定时返回配置（1s 一次）：防止个别电机上电慢/漏收
               导致没有编码器反馈(0x36)。配置命令幂等，重复发无害。 ---- */
        {
            static uint32_t s_LastCfgTick = 0;
            if (HAL_GetTick() - s_LastCfgTick >= 1000) {
                s_LastCfgTick = HAL_GetTick();
                for (int i = 0; i < 4; i++)
                    ZdtCan_SetTimedReturn(Chassis.Wheel[i].Addr, 0x36, ZDT_FEEDBACK_PERIOD_MS);
            }
        }

        /* ---- 使能判定：单一使能状态 PiCommand.Enabled（手柄 start=使能 / B=失能，Pi 离线自主失能） ----
           失能不松轴：周期发 4 轮速度 0，走加速度曲线平滑减速并保持锁轴静止。 */
        if (PiCommand.Enabled)
        {
            if (!Chassis.isEnable) {
                for (int i = 0; i < 4; i++) {
                    ZdtCan_Enable(Chassis.Wheel[i].Addr, 1);
                    ZdtCan_ClearStall(Chassis.Wheel[i].Addr);   /* 刚使能：顺带清掉锁存的堵转保护 */
                }
            }
            Chassis.isEnable = 1;
        }
        else
        {
            if (Chassis.isEnable) {
                Chassis_Disable();   /* 使能→失能边沿：清 PID/回零，仅一次 */
            }
            Chassis.isEnable = 0;

            /* 失能：四轮广播每拍都发速度 0，云台三轴紧跟着各发一条 F6 速度 0，
               平滑减速并锁轴，电机侧保持使能不松轴。整车失能 = 7 个电机全部持续被压住。 */
            ZdtCan_MultiBegin(&s_MultiCtx);
            for (int i = 0; i < 4; i++)
                ZdtMotor_MultiAddSpeed(&s_MultiCtx, &Chassis.Wheel[i], 0);
            ZdtCan_MultiSend(&s_MultiCtx);
            Gimbal_SendAxisSpeedFrames();   /* 云台任务失能时已把 Rpm 压 0 */
            continue;
        }

        /* ---- 车体速度：树莓派(PI_CMD_VEL) 或 蓝牙摇杆(BLE_ApplyToCommand) → PiCommand.Vel ---- */
        Chassis.Move.Vx = PiCommand.Vel.Vx;
        Chassis.Move.Vy = PiCommand.Vel.Vy;

#if CHASSIS_HEADING_ENABLE
        /* ---- 航向控制：目标航向 + 转向输入积分 ----
           目标航向 s_HeadingTarget 这样维护：
             · 位姿模式（PosTarget.Valid）：直接采用 POSE 指令里的目标航向；
             · 手动模式有转向输入（Wz≠0）：把 Wz 积分进目标航向 —— 推着转，
               松手就停在当前朝向（等价于"转到哪保持到哪"）；
             · 手动模式没转向输入：保持上次目标 → 走直线时自动纠偏。
           旧逻辑在手动模式下每周期把目标设成当前朝向，误差恒为 0，
           等于完全没有航向保持，手柄的 L2/R2 转向也会被吃掉。
           IMU 离线时清 PID 并放弃修正，免得拿陈旧角度把车转飞。 */
        if (Imu.LastOnlineTick != 0 &&
            (HAL_GetTick() - Imu.LastOnlineTick) < JY901S_OFFLINE_MS)
        {
            uint32_t Now = HAL_GetTick();
            float dt = (Now - s_HeadingTick) / 1000.0f;
            s_HeadingTick = Now;

            if (!s_HeadingValid)
            {
                /* 首次进入 / IMU 恢复：锁存当前朝向（或已有的 POSE 目标） */
                s_HeadingTarget = PiCommand.TargetYawValid ? PiCommand.TargetYaw : Imu.Yaw;
                s_HeadingValid = 1;
            }
            else if (PiCommand.PosTarget.Valid)
            {
                s_HeadingTarget = PiCommand.TargetYaw;
            }
            else if (fabsf(PiCommand.Vel.Wz) > 0.001f && dt < 0.5f)
            {
                s_HeadingTarget += PiCommand.Vel.Wz * dt * (180.0f / PI);   /* rad/s → °/s */
                while (s_HeadingTarget >  180.0f) s_HeadingTarget -= 360.0f;
                while (s_HeadingTarget < -180.0f) s_HeadingTarget += 360.0f;
            }

            PID_AngleCalc(&Chassis.HeadingPid, Imu.Yaw, s_HeadingTarget);
            Chassis.Move.Wz = CHASSIS_HEADING_YAW_DIR * Chassis.HeadingPid.Output * PI / 180.0f;   /* °/s → rad/s */
        }
        else
        {
            PID_Clear(&Chassis.HeadingPid);
            Chassis.Move.Wz = 0;
            s_HeadingValid = 0;     /* IMU 恢复后重新锁存朝向 */
        }
#else
        Chassis.Move.Wz = PiCommand.Vel.Wz;   /* 手动自旋指令（原逻辑，宏关闭时生效） */
#endif

        /* ---- 位置闭环：Pi 位姿目标(PosTarget.Valid) 或 蓝牙回零(ReturnHome) → 覆盖 Vx/Vy ---- */
        if (PiCommand.PosTarget.Valid)
            Chassis_PoseControl(PiCommand.PosTarget.X, PiCommand.PosTarget.Y);
        else if (PiCommand.ReturnHome)
            Chassis_PoseControl(0.0f, 0.0f);

        /* ---- 里程计：积分车体速度 → 世界系位置 ---- */
        Chassis_OdometryUpdate();

        float rpm[4];
        Chassis_MecanumCalc(rpm);
        Chassis_WheelRpmLimit(rpm);

        /* 4 轮先走 0xAA 广播（一条帧，6 个 CAN 包，实测可用），
           云台三轴随后各发一条单机 F6 帧。同一个任务、同一条时间线，
           全总线仍然只有这一个发送者，云台任务只算不发。 */
        ZdtCan_MultiBegin(&s_MultiCtx);
        for (int i = 0; i < 4; i++)
            ZdtMotor_MultiAddSpeed(&s_MultiCtx, &Chassis.Wheel[i], rpm[i]);
        ZdtCan_MultiSend(&s_MultiCtx);
        Gimbal_SendAxisSpeedFrames();

        /* ---- 载物盘舵机 ---- */
        Servo_SetAngle(&Chassis.PlateServo, PiCommand.Plate);
    }
}
