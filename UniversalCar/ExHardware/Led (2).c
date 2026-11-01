/**
 * @attention   采用UTF-8字符集编码
 * @brief       补光灯 PWM 驱动实现
 */

#include "header.h"

static float s_Brightness = 0.0f;   /* 当前亮度 % */

void Led_Init(void)
{
    /* 先清比较值再启动输出，保证上电熄灭 */
    __HAL_TIM_SET_COMPARE(&htim2, LED_PWM_CH, 0);
    HAL_TIM_PWM_Start(&htim2, LED_PWM_CH);
}

void Led_SetBrightness(float percent)
{
    /* 限幅到 0 ~ 满量程 */
    if (percent < 0.0f)           percent = 0.0f;
    if (percent > LED_BRIGHT_MAX) percent = LED_BRIGHT_MAX;

    /* 占空比 = percent/100 * ARR，用当前 ARR 计算，兼容任意 CubeMX 时基配置 */
    uint32_t arr   = __HAL_TIM_GET_AUTORELOAD(&htim2);
    uint32_t pulse = (uint32_t)((float)arr * percent / LED_BRIGHT_MAX + 0.5f);

    __HAL_TIM_SET_COMPARE(&htim2, LED_PWM_CH, pulse);

    s_Brightness = percent;
}

float Led_GetBrightness(void)
{
    return s_Brightness;
}
