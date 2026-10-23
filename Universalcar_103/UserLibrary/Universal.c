/**
 * @attention   采用UTF-8字符集编码
 * @brief       通用函数库
 */

#include "Universal.h"
#include "math.h"

/// @brief 整型限幅
int Constrain(int val, int max, int min)
{
    return val > max ? max : val < min ? min : val;
}

/// @brief 浮点限幅
float ConstrainF(float val, float max, float min)
{
    return val > max ? max : val < min ? min : val;
}

/// @brief 相位卷绕 例:AngleWrap(361, 360) => 1
int AngleWrap(int val, int max)
{
    return val > max ? val % max : (val < 0 ? val % max + max : val);
}

/// @brief 相位卷绕（浮点）
float AngleWrapF(float val, float max)
{
    return val > max ? fmodf(val, max) : (val < 0 ? fmodf(val, max) + max : val);
}

/// @brief 计算两角之差 a-b，返回劣弧
int AngleDiff(int a, int b, int max)
{
    int err = AngleWrap(a, max) - AngleWrap(b, max);
    int halfMax = max / 2;
    if (err > halfMax)
        err -= max;
    else if (err < -halfMax)
        err += max;
    return err;
}

/// @brief 计算两角之差 a-b，返回劣弧（浮点）
float AngleDiffF(float a, float b, float max)
{
    float err = AngleWrapF(a, max) - AngleWrapF(b, max);
    float halfMax = max / 2;
    if (err > halfMax)
        err -= max;
    else if (err < -halfMax)
        err += max;
    return err;
}

/// @brief 解卷绕器初始化
void AngleUnwrap_Init(AngleUnwrap_t* angleUnwrap, float value, float circle)
{
    angleUnwrap->Value = value;
    angleUnwrap->Unwraped = value;
    angleUnwrap->PrevVal = value;
    angleUnwrap->Circle = circle;
}

/// @brief 更新解卷绕值，返回解卷绕后的累计值
float AngleUnwrap_Update(AngleUnwrap_t* angleUnwrap, float value)
{
    angleUnwrap->Value = value;
    float DeltaAngle = angleUnwrap->Value - angleUnwrap->PrevVal;
    angleUnwrap->PrevVal = value;

    //过零点时计算劣弧
    float HalfCircle = angleUnwrap->Circle / 2;
    if (DeltaAngle > HalfCircle)
        DeltaAngle -= angleUnwrap->Circle;
    else if (DeltaAngle < -HalfCircle)
        DeltaAngle += angleUnwrap->Circle;

    angleUnwrap->Unwraped += DeltaAngle;

    return angleUnwrap->Unwraped;
}
