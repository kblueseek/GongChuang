#ifndef _UNIVERSAL_H_
#define _UNIVERSAL_H_

/**
 * @attention   采用UTF-8字符集编码
 * @brief       通用函数库
 */

#include "stdint.h"

int Constrain(int val, int max, int min);
float ConstrainF(float val, float max, float min);
int AngleWrap(int val, int max);
float AngleWrapF(float val, float max);
int AngleDiff(int a, int b, int max);
float AngleDiffF(float a, float b, float max);

/// @brief 角度解卷绕器
typedef struct
{
    float Value;        //本次输入值
    float Unwraped;     //解卷绕后的累计值
    float PrevVal;      //上次输入值
    float Circle;       //转一圈对应的值
} AngleUnwrap_t;

void AngleUnwrap_Init(AngleUnwrap_t* angleUnwrap, float value, float circle);
float AngleUnwrap_Update(AngleUnwrap_t* angleUnwrap, float value);

#endif
