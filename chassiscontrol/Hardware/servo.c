/**
 * @file    servo.c
 * @brief   舵机 PWM 驱动实现 (TIM3 50Hz)
 *
 * 脉宽 → 角度 线性映射:
 *     pulse = pulse_min + (angle / 180) * (pulse_max - pulse_min)
 * 其中 pulse 单位为 us, 因为 TIM3 预分频后 1 计数 = 1us.
 *
 * 调参: 若舵机转到头会"哒哒"响, 说明脉宽超出它实际能接受的范围,
 *       把对应舵机的 SERVOn_PULSE_MIN/MAX 往内收 (例如 600~2400) 即可。
 */
#include "servo.h"
#include "tim.h"
#include <string.h>

/* ============ 可调参数 ============ */

/* 两个舵机都是 180° 行程 */
#define SERVO_TRAVEL        180.0f

/* servo1: TIM3_CH1 / PA6 */
#define SERVO1_PULSE_MIN    500    /* 0°   脉宽, 单位 us */
#define SERVO1_PULSE_MAX    2500   /* 180° 脉宽, 单位 us */
#define SERVO1_INIT_ANGLE   0.0f   /* 上电初始角度 */

/* servo2: TIM3_CH2 / PA7 */
#define SERVO2_PULSE_MIN    500
#define SERVO2_PULSE_MAX    2500
#define SERVO2_INIT_ANGLE   0.0f

/* ============ 配置表 & 状态 ============ */
typedef struct {
    uint32_t    channel;     /* TIM 通道 */
    uint16_t    pulse_min;   /* 0°   脉宽 us */
    uint16_t    pulse_max;   /* 180° 脉宽 us */
    float       init_angle;  /* 上电初始角度 */
    const char *name;        /* 蓝牙滑杆名称 */
} Servo_Cfg;

static const Servo_Cfg g_cfg[SERVO_NUM] = {
    { TIM_CHANNEL_1, SERVO1_PULSE_MIN, SERVO1_PULSE_MAX, SERVO1_INIT_ANGLE, "servo1" },
    { TIM_CHANNEL_2, SERVO2_PULSE_MIN, SERVO2_PULSE_MAX, SERVO2_INIT_ANGLE, "servo2" },
};

static float g_angle[SERVO_NUM];   /* 当前角度记录 */

/* ============ 内部: 把角度直接写进比较寄存器 ============ */
static void Servo_WriteAngle(Servo_Id id, float angle)
{
    const Servo_Cfg *c = &g_cfg[id];

    /* 限幅到 0 ~ 180° */
    if (angle < 0.0f)         angle = 0.0f;
    if (angle > SERVO_TRAVEL) angle = SERVO_TRAVEL;

    float    k     = angle / SERVO_TRAVEL;                                 /* 0.0 ~ 1.0 */
    uint16_t pulse = (uint16_t)(c->pulse_min + k * (c->pulse_max - c->pulse_min));

    __HAL_TIM_SET_COMPARE(&htim3, c->channel, pulse);

    g_angle[id] = angle;
}

/* ============ 初始化 ============ */
void Servo_Init(void)
{
    for (int i = 0; i < SERVO_NUM; i++) {
        g_angle[i] = 0.0f;
    }

    /* 先给比较值再启动输出, 避免上电瞬间跳到 0 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    for (int i = 0; i < SERVO_NUM; i++) {
        Servo_WriteAngle((Servo_Id)i, g_cfg[i].init_angle);
    }
}

/* ============ 角度输入 ============ */
void Servo_SetAngle(Servo_Id id, float angle)
{
    if (id >= SERVO_NUM) return;
    Servo_WriteAngle(id, angle);
}

float Servo_GetAngle(Servo_Id id)
{
    if (id >= SERVO_NUM) return 0.0f;
    return g_angle[id];
}

/* ============ 蓝牙滑杆入口 ============ */
void Servo_HandleSlider(const char *name, float value)
{
    if (name == NULL) return;

    for (int i = 0; i < SERVO_NUM; i++) {
        if (strcmp(name, g_cfg[i].name) == 0) {
            Servo_WriteAngle((Servo_Id)i, value * SERVO_TRAVEL);   /* 滑杆 0~1 → 0~180° */
            return;
        }
    }
    /* 名称不是 servo1/servo2 的滑杆, 直接忽略 */
}
