#ifndef _LED_H_
#define _LED_H_

/**
 * @attention   采用UTF-8字符集编码
 * @brief       补光灯 PWM 驱动（TIM2，3kHz，占空比调光）
 * @details     TIM2_CH1 / PA5，CubeMX 里 PSC=99、ARR=279 → 84MHz/100/280 = 3kHz。
 *              亮度 0~100% 线性对应占空比 0~100%。
 */

#include "stdint.h"

/// @brief 初始化补光灯 PWM（上电时调用一次，上电默认熄灭）
void Led_Init(void);

/// @brief 设置亮度（0~100%，超范围自动限幅）
void Led_SetBrightness(float percent);

/// @brief 读取当前亮度（%）
float Led_GetBrightness(void);

#endif
