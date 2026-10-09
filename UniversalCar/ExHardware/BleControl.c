/**
 * @attention   采用UTF-8字符集编码
 * @brief       蓝牙遥控模块实现（USART2，类比 RM 的 DBUS）
 * @details     中断只做 [ ] 包捕获，字符串解析在 BLE_Process()（任务上下文）完成。
 */

#include "header.h"

BLE_Remote_t BleRemote;

/* ============ 接收缓冲（中断写入，任务消费） ============ */
#define RX_BUF_SIZE     200
static char           s_RxPacket[RX_BUF_SIZE];
static volatile uint8_t s_RxFlag = 0;
volatile uint32_t BLE_RxByteCount = 0;   /* USART2 收到的字节总数（调试用） */
volatile uint8_t  BLE_LastRxByte = 0;     /* USART2 最后收到的原始字节（调试用） */
volatile uint32_t BLE_PacketCount = 0;    /* 完成 [ ] 捕获的包数（调试用） */
volatile uint8_t  BLE_TagChar = 0;        /* 最近一次捕获包的首字符（'j'=106） */

/* ============ 内部：发送 ============ */
static void BLE_SendByte(uint8_t byte)
{
    HAL_UART_Transmit(&huart2, &byte, 1, 10);
}

static void BLE_SendString(const char* str)
{
    while (*str) {
        BLE_SendByte((uint8_t)*str++);
    }
}

/* ============ 调试发送（发往手机） ============ */
void BLE_Printf(const char* fmt, ...)
{
    char buf[200];
    va_list arg;
    va_start(arg, fmt);
    vsnprintf(buf, sizeof(buf), fmt, arg);
    va_end(arg);
    BLE_SendString(buf);
}

void BLE_SendPlot(float* values, int count)
{
    if (count < 1 || count > 10) return;
    BLE_SendString("[p,");
    for (int i = 0; i < count; i++) {
        char num[32];
        snprintf(num, sizeof(num), "%.2f", values[i]);
        BLE_SendString(num);
        if (i < count - 1) BLE_SendByte(',');
    }
    BLE_SendString("]\r\n");
}

void BLE_SendPlotClear(void)
{
    BLE_SendString("[p-c]\r\n");
}

/* ============ 数据包解析（任务上下文） ============ */
static void BLE_ParsePacket(const char* packet)
{
    char buf[RX_BUF_SIZE];
    strncpy(buf, packet, RX_BUF_SIZE - 1);
    buf[RX_BUF_SIZE - 1] = '\0';

    char* Tag = strtok(buf, ",");
    if (Tag == NULL) return;

    /* ---- 摇杆: [joystick,Lx,Ly,Rx,Ry] 或 [j,...] ---- */
    if (strcmp(Tag, "joystick") == 0 || strcmp(Tag, "j") == 0)
    {
        char* s[4];
        for (int i = 0; i < 4; i++) s[i] = strtok(NULL, ",");
        if (s[0] && s[1] && s[2] && s[3]) {
            BleRemote.Joy.Lx = atoi(s[0]);
            BleRemote.Joy.Ly = atoi(s[1]);
            BleRemote.Joy.Rx = atoi(s[2]);
            BleRemote.Joy.Ry = atoi(s[3]);
        }
    }
    /* ---- 按键: [key,名称,down|up] 或 [k,名称,d|u] ---- */
    else if (strcmp(Tag, "key") == 0 || strcmp(Tag, "k") == 0)
    {
        char* n = strtok(NULL, ",");
        char* a = strtok(NULL, ",");
        if (n && a) {
            int id;
            if (strcmp(n, "return") == 0)
                id = BLE_KEY_RETURN;          /* 名称形式：return */
            else
                id = atoi(n);                 /* 数字形式：1/2/3 */
            if (id >= 1 && id <= BLE_KEY_RETURN) {
                uint8_t down = (strcmp(a, "down") == 0 || strcmp(a, "d") == 0);
                BleRemote.Key[id].Down = down;
                if (down)
                    BleRemote.Key[id].Pressed = 1;
            }
        }
    }
    /* ---- 滑杆: [slider,名称,值] 或 [s,...] ---- */
    else if (strcmp(Tag, "slider") == 0 || strcmp(Tag, "s") == 0)
    {
        char* n = strtok(NULL, ",");
        char* v = strtok(NULL, ",");
        if (n && v)
            BLE_HandleSlider(n, atof(v));
    }

    BleRemote.LastOnlineTick = HAL_GetTick();
}

/* ============ 滑杆语义 ============ */

/// @brief 滑杆分发：0~1 映射到蓝牙缓存（树莓派在线时不生效，由 BLE_ApplyToCommand 统一应用）
void BLE_HandleSlider(const char* name, float value)
{
    if (strcmp(name, BLE_SLIDER_GRIPPER) == 0) {
        BleRemote.Slider.Gripper = value * SERVO_TRAVEL;
    } else if (strcmp(name, BLE_SLIDER_PLATE) == 0) {
        BleRemote.Slider.Plate = value * SERVO_TRAVEL;
    } else if (strcmp(name, BLE_SLIDER_CAM) == 0) {
        BleRemote.Slider.Cam = value * SERVO_TRAVEL;
    } else if (strcmp(name, BLE_SLIDER_YAW) == 0) {
        BleRemote.Slider.Yaw = GIMBAL_YAW_MIN + value * (GIMBAL_YAW_MAX - GIMBAL_YAW_MIN);
        BleRemote.Slider.GimbalMask |= (1 << 0);
    } else if (strcmp(name, BLE_SLIDER_LIFT) == 0) {
        BleRemote.Slider.Lift = GIMBAL_LIFT_MIN + value * (GIMBAL_LIFT_MAX - GIMBAL_LIFT_MIN);
        BleRemote.Slider.GimbalMask |= (1 << 1);
    } else if (strcmp(name, BLE_SLIDER_EXTEND) == 0) {
        BleRemote.Slider.Extend = GIMBAL_EXTEND_MIN + value * (GIMBAL_EXTEND_MAX - GIMBAL_EXTEND_MIN);
        BleRemote.Slider.GimbalMask |= (1 << 2);
    } else if (strcmp(name, BLE_SLIDER_LED) == 0) {
        /* led 滑杆 0~100 直接作为亮度百分比 */
        BleRemote.Slider.Led = value;
    } else if (strcmp(name, BLE_SLIDER_SPIN) == 0) {
        /* 滑杆直接下发 -180~180 目标航向，与陀螺仪 Yaw 同量程（拨到哪车就朝哪） */
        BleRemote.Slider.Spin = value;
        BleRemote.Slider.SpinValid = 1;
    }
    /* 未匹配的滑杆忽略 */
}

/* ============ 遥控源适配 ============ */

void BLE_ApplyToCommand(void)
{
    /* ---- 摇杆 → 底盘速度（带死区，推上=前进）
       左摇杆纵向 = 前进后退，右摇杆横向 = 左右平移 */
    static const float DeadZone = 8;
    static const float JoyRange = 100;

    float Ly = (fabsf((float)BleRemote.Joy.Ly) < DeadZone) ? 0 : BleRemote.Joy.Ly;
    float Rx = (fabsf((float)BleRemote.Joy.Rx) < DeadZone) ? 0 : BleRemote.Joy.Rx;

    PiCommand.Vel.Vx = Ly / JoyRange * CHASSIS_MAX_SPEED_X;     /* 推上(Ly负)→Vx负；实测前后反了，翻转符号 */
    PiCommand.Vel.Vy =  -Rx / JoyRange * CHASSIS_MAX_SPEED_Y;

    /* 自旋速度指令 PiCommand.Vel.Wz 暂不使用 —— 改由 spin 滑杆给目标航向，
       自旋速度由航向 PID 算出（见 ChassisTask）。原摇杆映射保留备用：
         PiCommand.Vel.Wz = Rx / JoyRange * CHASSIS_MAX_SPEED_W; */

    /* ---- spin 滑杆 → 目标航向（航向 PID 的被控目标） ---- */
    if (BleRemote.Slider.SpinValid) {
        PiCommand.TargetYaw = BleRemote.Slider.Spin;
        PiCommand.TargetYawValid = 1;
    }

    /* ---- 按键边沿 ---- */
    if (BleRemote.Key[BLE_KEY_ENABLE].Pressed) {
        PiCommand.EnableMask = (PiCommand.EnableMask == 0x7F) ? 0 : 0x7F;   /* 全 7 电机 toggle */
        BleRemote.Key[BLE_KEY_ENABLE].Pressed = 0;
    }
    if (BleRemote.Key[BLE_KEY_ESTOP].Pressed) {
        PiCommand.Estop = 1;
        PiCommand.EnableMask = 0;
        PiCommand.Vel.Vx = PiCommand.Vel.Vy = PiCommand.Vel.Wz = 0;
        BleRemote.Key[BLE_KEY_ESTOP].Pressed = 0;
    }
    if (BleRemote.Key[BLE_KEY_RELEASE].Pressed) {
        PiCommand.Estop = 0;
        PiCommand.EnableMask = 0x7F;
        BleRemote.Key[BLE_KEY_RELEASE].Pressed = 0;
    }
    if (BleRemote.Key[BLE_KEY_RETURN].Pressed) {
        PiCommand.ReturnHome = 1;   /* 触发回零，底盘任务执行完后清除 */
        BleRemote.Key[BLE_KEY_RETURN].Pressed = 0;
    }

    /* ---- 滑杆缓存 → 舵机 / 云台目标 ---- */
    PiCommand.Gripper = BleRemote.Slider.Gripper;
    PiCommand.Plate   = BleRemote.Slider.Plate;
    PiCommand.Cam     = BleRemote.Slider.Cam;

    /* ---- 补光灯亮度（led 滑杆 0~100 直接应用） ---- */
    Led_SetBrightness(BleRemote.Slider.Led);

    if (BleRemote.Slider.GimbalMask & (1 << 0)) {
        PiCommand.GimbalTarget[0].Deg  = BleRemote.Slider.Yaw;
        PiCommand.GimbalTarget[0].Mask = 1;
    }
    if (BleRemote.Slider.GimbalMask & (1 << 1)) {
        PiCommand.GimbalTarget[1].Deg  = BleRemote.Slider.Lift;
        PiCommand.GimbalTarget[1].Mask = 1;
    }
    if (BleRemote.Slider.GimbalMask & (1 << 2)) {
        PiCommand.GimbalTarget[2].Deg  = BleRemote.Slider.Extend;
        PiCommand.GimbalTarget[2].Mask = 1;
    }
    BleRemote.Slider.GimbalMask = 0;

    /* 注意：不要写 PiCommand.LastOnlineTick！
       否则 PiComm_IsOnline() 会被误判为"树莓派在线"，导致 ReceiveTask 里
       BLE_ApplyToCommand 只在 PiComm 掉线那一刻执行一次（约 2Hz 顿挫）。
       蓝牙是否在线由 BleRemote.LastOnlineTick 单独判定，各任务用
       PiComm_IsOnline() || BLE_IsOnline() 判断是否有指令源即可。 */
}

/* ============ 初始化 / 周期处理 ============ */

void BLE_Init(void)
{
    memset(&BleRemote, 0, sizeof(BleRemote));

    /* 滑杆缓存初始化为舵机上电角度，避免蓝牙接管瞬间舵机跳变 */
    BleRemote.Slider.Gripper = SERVO_GRIPPER_INIT;
    BleRemote.Slider.Plate   = SERVO_PLATE_INIT;
    BleRemote.Slider.Cam     = SERVO_CAM_INIT;

    /* 目标航向：收到 spin 滑杆前 SpinValid=0，底盘会锁存当前朝向、不做修正 */
    BleRemote.Slider.Spin = 0;
    BleRemote.Slider.SpinValid = 0;

    s_RxFlag = 0;

    Led_Init();     /* 补光灯 PWM 初始化（蓝牙摇杆 Ry 控制亮度） */
}

void BLE_Process(void)
{
    if (s_RxFlag) {
        s_RxFlag = 0;
        BLE_ParsePacket(s_RxPacket);
    }
}

int BLE_IsOnline(void)
{
    return (HAL_GetTick() - BleRemote.LastOnlineTick) < BLE_OFFLINE_MS;
}

/* ============ 字节捕获（中断上下文，由 usart.c 调用） ============ */
void BLE_RxCallback(uint8_t data)
{
    static uint8_t  state = 0;      /* 0=等 '['  1=收包内容 */
    static uint16_t index = 0;

    BLE_RxByteCount++;               /* 每收到一个字节计数一次 */
    BLE_LastRxByte = data;           /* 记录最后收到的原始字节 */

    if (state == 0) {
        if (data == '[' && s_RxFlag == 0) {
            state = 1;
            index = 0;
        }
    } else if (state == 1) {
        if (data == ']') {
            state = 0;
            s_RxPacket[index] = '\0';
            s_RxFlag = 1;
            BLE_PacketCount++;
            BLE_TagChar = (uint8_t)s_RxPacket[0];
        } else if (index < RX_BUF_SIZE - 1) {
            s_RxPacket[index++] = (char)data;
        }
    }
}
