/**
 * @file    ControllerTask.c
 * @brief   底盘运动控制实现
 *
 * 数据流: 蓝牙摇杆 [joystick,Lx,Ly,Rx,Ry] → 目标速度(vx,vy,wz) → 麦克纳姆逆解算
 *         → 四个轮子速度 → ZDT 电机速度模式(多电机命令一次下发)
 *
 * 麦克纳姆 X 接法逆解算 (坐标: x前进, y右移, ω逆时针为正):
 *   v_fl =  vx - vy - ω      v_fr =  vx + vy + ω
 *   v_rl =  vx + vy - ω      v_rr =  vx - vy + ω
 * 注: 每个轮子电机实际转向随安装方向不同, 用 FL_DIR 等系数翻转(±1)。
 */
#include "ControllerTask.h"
#include "zdtmotor_uart.h"
#include "ble_control.h"
#include "main.h"

/* ============ 可调参数 ============ */
#define JOY_RANGE     100    /* 摇杆满量程（小程序通常 -100~100，按实际调整） */
#define JOY_DEADZONE  8      /* 摇杆死区 */
#define MAX_SPEED     2000   /* 最大轮速，单位 0.1RPM（2000 = 200 RPM） */
#define ACC           500    /* 加速度，单位 RPM/s */
#define ROT_COEF      1.0f   /* 旋转耦合系数（与轮距/轴距有关，可调） */
#define JOY_TIMEOUT   300    /* 无摇杆输入超时停车，单位 ms */

/* ============ 每轮电机转向系数（按实际安装方向调 ±1） ============
 * 调法：单独给某一轮发正速度，观察轮子是否"前进"，反向就把系数取反。 */
#define FL_DIR   1
#define FR_DIR  -1
#define RL_DIR   1
#define RR_DIR  -1

/* ============ 内部状态 ============ */
static int16_t  g_vx = 0, g_vy = 0, g_wz = 0;  /* 目标速度（0.1RPM） */
static uint32_t g_last_joy = 0;                /* 最后一次摇杆事件时刻 */

static int deadzone(int v)
{
    if (v > -JOY_DEADZONE && v < JOY_DEADZONE) return 0;
    return v;
}

/* 单个轮速（带符号）→ 速度模式子命令（符号转 dir + 幅值） */
static void AddWheelCmd(uint8_t addr, int32_t spd)
{
    uint8_t  dir = (spd >= 0) ? ZDT_DIR_CW : ZDT_DIR_CCW;
    uint16_t mag = (spd >= 0) ? (uint16_t)spd : (uint16_t)(-spd);
    if (mag > MAX_SPEED) mag = MAX_SPEED;
    uint8_t c[7] = {0xF6, dir, (uint8_t)(ACC >> 8), (uint8_t)ACC, (uint8_t)(mag >> 8), (uint8_t)mag, 0x00};
    ZDT_MultiAdd(addr, c, 7);
}

/* 麦克纳姆逆解算 + 一次下发四个轮速 */
static void MecanumApply(void)
{
    int32_t rot = (int32_t)((float)g_wz * ROT_COEF);
    int32_t fl = FL_DIR * (g_vx - g_vy - rot);
    int32_t fr = FR_DIR * (g_vx + g_vy + rot);
    int32_t rl = RL_DIR * (g_vx + g_vy - rot);
    int32_t rr = RR_DIR * (g_vx - g_vy + rot);

    ZDT_MultiBegin();
    AddWheelCmd(ZDT_MOTOR_FL, fl);
    AddWheelCmd(ZDT_MOTOR_FR, fr);
    AddWheelCmd(ZDT_MOTOR_RL, rl);
    AddWheelCmd(ZDT_MOTOR_RR, rr);
    ZDT_MultiSend();
}

void ControllerTask_Init(void)
{
    g_vx = g_vy = g_wz = 0;
    g_last_joy = HAL_GetTick();

    /* 使能四个电机（锁轴） */
    ZDT_UART_Enable(ZDT_MOTOR_FL, 1);
    ZDT_UART_Enable(ZDT_MOTOR_FR, 1);
    ZDT_UART_Enable(ZDT_MOTOR_RL, 1);
    ZDT_UART_Enable(ZDT_MOTOR_RR, 1);
}

void ControllerTask_Loop(void)
{
    /* 看门狗：超时无摇杆输入则停车，防蓝牙断连飞车 */
    if (HAL_GetTick() - g_last_joy > JOY_TIMEOUT) {
        if (g_vx || g_vy || g_wz) {
            g_vx = g_vy = g_wz = 0;
            MecanumApply();
        }
    }
}

/* ============ BLE 回调 ============ */

/* 摇杆 → 目标速度 */
void BLE_OnJoystick(BLE_JoystickEvent *e)
{
    int Lx = deadzone(e->Lx);
    int Ly = deadzone(e->Ly);
    int Rx = deadzone(e->Rx);
    /* e->Ry 暂不用 */

    g_last_joy = HAL_GetTick();

    /* 左摇杆 → 平移，右摇杆 → 旋转 */
    g_vy = (int16_t)((int32_t)Lx * MAX_SPEED / JOY_RANGE);    /* 左右平移 */
    g_vx = (int16_t)((int32_t)-Ly * MAX_SPEED / JOY_RANGE);  /* 前后（推上为负→前进为正） */
    g_wz = (int16_t)((int32_t)Rx * MAX_SPEED / JOY_RANGE);    /* 旋转 */

    MecanumApply();
}

/* 按键：1=使能 2=急停+松轴 */
void BLE_OnKey(BLE_KeyEvent *e)
{
    if (!e->is_down) return;

    if (e->name == 1) {
        ZDT_UART_Enable(ZDT_MOTOR_FL, 1);
        ZDT_UART_Enable(ZDT_MOTOR_FR, 1);
        ZDT_UART_Enable(ZDT_MOTOR_RL, 1);
        ZDT_UART_Enable(ZDT_MOTOR_RR, 1);
    } else if (e->name == 2) {
        ZDT_UART_Stop(ZDT_MOTOR_FL);
        ZDT_UART_Stop(ZDT_MOTOR_FR);
        ZDT_UART_Stop(ZDT_MOTOR_RL);
        ZDT_UART_Stop(ZDT_MOTOR_RR);
        g_vx = g_vy = g_wz = 0;
    }
}

/* 滑杆：本任务只做底盘运动控制，舵机不在这里管。
 * 若要蓝牙调舵机，在这里调用 Servo_HandleSlider(e->name, e->value)。
 * 留空函数是为了保持 ble_control.c 的回调约定不破。 */
void BLE_OnSlider(BLE_SliderEvent *e)
{
    (void)e;
}
