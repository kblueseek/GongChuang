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

#define CHASSIS_TASK_UPDATE_TICK 5

/* 编码器计数 → 轮位移系数：65536 计数 = 1 圈 = 2πR，即 1 计数 ≈ 3.76µm */
#define WHEEL_M_PER_COUNT  ((2.0f * PI * WHEEL_RADIUS_M) / 65536.0f)

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
#define CHASSIS_RETURN_KD        0.0f    /* 微分：先关 */
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

/* 底盘停车（清速度；电机由失能边沿的 Stop 命令立即停止并保持锁轴） */
static void Chassis_Disable(void)
{
    Chassis.Move.Vx = 0;
    Chassis.Move.Vy = 0;
    Chassis.Move.Wz = 0;
    PID_Clear(&Chassis.HeadingPid);   /* 清航向 PID 积分，避免下次使能时残留 */
    PID_Clear(&Chassis.PosPidX);
    PID_Clear(&Chassis.PosPidY);
    PiCommand.ReturnHome = 0;         /* 失能时取消未完成的回零 */
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
    /* 首帧：等 4 轮都收到编码器反馈后再初始化差分基准，避免起始跳变 */
    if (!Chassis.OdomReady) {
        for (int i = 0; i < 4; i++)
            if (!ZdtMotor_IsOnline(&Chassis.Wheel[i])) return;
        for (int i = 0; i < 4; i++)
            Chassis.WheelLastCount[i] = Chassis.Wheel[i].Position.RawCount;
        Chassis.OdomReady = 1;
        return;
    }

    float dt = CHASSIS_TASK_UPDATE_TICK * 0.001f;
    float L = WHEEL_HALF_WHEELBASE_M + WHEEL_HALF_TRACK_M;
    float dv[4];

    for (int i = 0; i < 4; i++) {
        int32_t dCount = Chassis.Wheel[i].Position.RawCount - Chassis.WheelLastCount[i];
        Chassis.WheelLastCount[i] = Chassis.Wheel[i].Position.RawCount;
        Chassis.WheelRpm[i] = (float)dCount * 60.0f / (65536.0f * dt);   /* 实际滚动 RPM */
        dv[i] = (float)dCount * WHEEL_M_PER_COUNT * s_WheelDir[i];        /* 麦轮分量位移 m */
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

/* 回零：世界系位置误差 → PID → 世界系速度 → 转车体系，覆盖 Vx/Vy。
   到位后清请求并停车。由 return 键触发，目标恒为里程计零点 (0,0)。 */
static void Chassis_ReturnHomeControl(void)
{
    float yawRad = Imu.Yaw * PI / 180.0f;
    float cosY = cosf(yawRad);
    float sinY = sinf(yawRad);

    /* 世界系误差 → PID → 世界系速度 */
    PID_Calc(&Chassis.PosPidX, Chassis.Pos.X, 0.0f);
    PID_Calc(&Chassis.PosPidY, Chassis.Pos.Y, 0.0f);
    float vxWorld = Chassis.PosPidX.Output;
    float vyWorld = Chassis.PosPidY.Output;

    /* 世界系速度 → 车体系速度（旋转矩阵的逆） */
    Chassis.Move.Vx =  vxWorld * cosY + vyWorld * sinY;
    Chassis.Move.Vy = -vxWorld * sinY + vyWorld * cosY;

    /* 到位判定：距离零点足够近则停车、清请求、清积分 */
    if (sqrtf(Chassis.Pos.X * Chassis.Pos.X + Chassis.Pos.Y * Chassis.Pos.Y) < CHASSIS_RETURN_ARRIVED)
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

        /* ---- 使能判定：bit0-3（电机1-4）全置位 且 无急停 ----
           不做蓝牙失联门控：BLE 小程序空闲时不发任何数据，BLE_IsOnline()
           （近500ms收到过数据）会把车误判离线 → 停底盘 → 清航向 PID。
           树莓派失联由 PiComm_TimeoutCheck() 清速度兜底，不在这里关使能。 */
        uint8_t ChassisEnable = (PiCommand.EnableMask & 0x0F) == 0x0F;

        if (ChassisEnable && !PiCommand.Estop)
        {
            if (!Chassis.isEnable) {
                for (int i = 0; i < 4; i++)
                    ZdtCan_Enable(Chassis.Wheel[i].Addr, 1);
            }
            Chassis.isEnable = 1;
        }
        else
        {
            if (Chassis.isEnable) {
                for (int i = 0; i < 4; i++)
                    ZdtCan_Stop(Chassis.Wheel[i].Addr);    /* 立即停止，保持锁轴（电机上电默认锁轴，不松轴） */
            }
            Chassis.isEnable = 0;
        }

        if (!Chassis.isEnable)
        {
            Chassis_Disable();
            continue;
        }

        /* ---- 车体速度：树莓派(PI_CMD_VEL) 或 蓝牙摇杆(BLE_ApplyToCommand) → PiCommand.Vel ---- */
        Chassis.Move.Vx = PiCommand.Vel.Vx;
        Chassis.Move.Vy = PiCommand.Vel.Vy;

#if CHASSIS_HEADING_ENABLE
        /* ---- 航向修正：IMU Yaw → PID → 自旋速度 Wz ----
           目标航向由 spin 滑杆给出；还没收到过滑杆指令时锁存当前朝向（不做修正）。
           IMU 离线时清 PID 并放弃修正，免得拿陈旧角度把车转飞。 */
        if (Imu.LastOnlineTick != 0 &&
            (HAL_GetTick() - Imu.LastOnlineTick) < JY901S_OFFLINE_MS)
        {
            float TargetYaw = PiCommand.TargetYawValid ? PiCommand.TargetYaw : Imu.Yaw;
            PID_AngleCalc(&Chassis.HeadingPid, Imu.Yaw, TargetYaw);
            Chassis.Move.Wz = CHASSIS_HEADING_YAW_DIR * Chassis.HeadingPid.Output * PI / 180.0f;   /* °/s → rad/s */
        }
        else
        {
            PID_Clear(&Chassis.HeadingPid);
            Chassis.Move.Wz = 0;
        }
#else
        Chassis.Move.Wz = PiCommand.Vel.Wz;   /* 手动自旋指令（原逻辑，宏关闭时生效） */
#endif

        /* ---- 回零：return 键触发，位置闭环覆盖 Vx/Vy ---- */
        if (PiCommand.ReturnHome)
            Chassis_ReturnHomeControl();

        /* ---- 里程计：积分车体速度 → 世界系位置 ---- */
        Chassis_OdometryUpdate();

        float rpm[4];
        Chassis_MecanumCalc(rpm);
        Chassis_WheelRpmLimit(rpm);

        /* 四轮速度一次下发：多电机命令 0xAA（一条广播帧），4 轮同时开始执行 */
        ZdtCan_MultiBegin();
        for (int i = 0; i < 4; i++)
            ZdtMotor_MultiAddSpeed(&Chassis.Wheel[i], rpm[i]);
        ZdtCan_MultiSend();

        /* ---- 载物盘舵机 ---- */
        Servo_SetAngle(&Chassis.PlateServo, PiCommand.Plate);
    }
}
