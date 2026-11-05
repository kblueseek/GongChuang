/**
 * @file    MSPM0imu.c
 * @brief   MSPM0G3507 + XV7011 单轴陀螺仪模组 —— 主机端 UART 驱动实现
 *
 * 设计要点：
 *   - 逐字节状态机，同时识别 11 字节标准帧（0x55）和 5 字节兼容帧（0x5A）
 *   - 帧尾累加和校验；坏帧自动移位重同步，不依赖帧间静默
 *   - 中断里只做解析与字段更新，Online 判定放在 MSPM0IMU_Poll()
 *   - 不依赖 HAL / FreeRTOS，可直接放进任意 MCU 工程
 */
#include "MSPM0imu.h"

#include <string.h>

/* ============================================================
 * 弱符号（Keil / GCC 通用）
 * ============================================================ */
#if defined(__GNUC__)
#define MSPM0IMU_WEAK   __attribute__((weak))
#elif defined(__CC_ARM) || defined(__ARMCC_VERSION)
#define MSPM0IMU_WEAK   __weak
#elif defined(__ICCARM__)
#define MSPM0IMU_WEAK   __weak
#else
#define MSPM0IMU_WEAK
#endif

/* ============================================================
 * 常量
 * ============================================================ */
#define MSPM0IMU_STD_LEN        (11u)
#define MSPM0IMU_AX1F_LEN       (5u)

#define MSPM0IMU_SCALE_GYRO     (2000.0f / 32768.0f) /* °/s per LSB */
#define MSPM0IMU_SCALE_ANGLE    (180.0f  / 32768.0f) /* °   per LSB */
#define MSPM0IMU_SCALE_TEMP     (0.01f)              /* ℃  per LSB */

/* ============================================================
 * 全局数据与解析状态
 * ============================================================ */
MSPM0IMU_t MSPM0IMU;

static uint8_t  s_buf[MSPM0IMU_STD_LEN];
static uint8_t  s_len;
static uint8_t  s_need;
static uint32_t s_poll_cnt;
static uint8_t  s_seen;

/* ============================================================
 * 工具
 * ============================================================ */
static int16_t mspm0imu_i16le(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static bool mspm0imu_sum_ok(const uint8_t *f, uint8_t len)
{
    uint8_t sum = 0u;
    uint8_t i;

    for (i = 0u; i < (uint8_t)(len - 1u); i++)
    {
        sum = (uint8_t)(sum + f[i]);
    }

    return (sum == f[len - 1u]);
}

/* 坏帧重同步：在已收数据里找下一个合法帧头，找到就把它移到缓冲区开头 */
static void mspm0imu_resync(void)
{
    uint8_t i;

    for (i = 1u; i < s_len; i++)
    {
        if (s_buf[i] == MSPM0IMU_HEAD_STD)
        {
            uint8_t n = (uint8_t)(s_len - i);
            uint8_t k;

            for (k = 0u; k < n; k++)
            {
                s_buf[k] = s_buf[i + k];
            }
            s_len  = n;
            s_need = MSPM0IMU_STD_LEN;
            return;
        }

#if MSPM0IMU_ENABLE_AX1F
        if (s_buf[i] == MSPM0IMU_HEAD_AX1F)
        {
            uint8_t n = (uint8_t)(s_len - i);
            uint8_t k;

            for (k = 0u; k < n; k++)
            {
                s_buf[k] = s_buf[i + k];
            }
            s_len  = n;
            s_need = MSPM0IMU_AX1F_LEN;
            return;
        }
#endif
    }

    s_len  = 0u;
    s_need = 0u;
}

/* ============================================================
 * 帧解析
 * ============================================================ */

/* 11 字节标准帧 */
static bool mspm0imu_parse_std(const uint8_t *f)
{
    uint8_t id = f[1];
    int16_t d1 = mspm0imu_i16le(&f[4]);   /* D2 */
    int16_t d2 = mspm0imu_i16le(&f[6]);   /* D3 */
    int16_t d3 = mspm0imu_i16le(&f[8]);   /* D4 */

    if (id == MSPM0IMU_ID_GYRO)
    {
        /* 单轴模组 0x52：D2=角速度(未死区)，D3=角速度(死区后)，D4=温度 */
        MSPM0IMU.GyroRawOut = d1;
        MSPM0IMU.GyroRaw    = d2;
        MSPM0IMU.Gyro[2]    = MSPM0IMU_GYRO_SIGN * ((float)d2 * MSPM0IMU_SCALE_GYRO);
        MSPM0IMU.YawRate    = MSPM0IMU.Gyro[2];
        MSPM0IMU.Temp       = (float)d3 * MSPM0IMU_SCALE_TEMP;
        return true;
    }

    if (id == MSPM0IMU_ID_ANGLE)
    {
        /* 单轴模组 0x53：D1=0，D2=0，D3=Yaw，D4=温度 */
        MSPM0IMU.YawRaw  = d2;
        MSPM0IMU.Angle[2] = MSPM0IMU_YAW_SIGN * ((float)d2 * MSPM0IMU_SCALE_ANGLE);
        MSPM0IMU.Yaw      = MSPM0IMU.Angle[2];
        MSPM0IMU.Temp     = (float)d3 * MSPM0IMU_SCALE_TEMP;
        return true;
    }

    /* 0x59 四元数 / 0x5F 寄存器回包等：本模组单轴板不产生，直接忽略 */
    return false;
}

/* 5 字节兼容帧 */
#if MSPM0IMU_ENABLE_AX1F
static bool mspm0imu_parse_ax1f(const uint8_t *f)
{
    uint8_t id = f[1];
    int16_t v  = mspm0imu_i16le(&f[2]);

    if (id == MSPM0IMU_AX1F_GYRO)
    {
        MSPM0IMU.GyroRawOut = v;
        MSPM0IMU.GyroRaw    = v;
        MSPM0IMU.Gyro[2]    = MSPM0IMU_GYRO_SIGN * ((float)v * MSPM0IMU_SCALE_GYRO);
        MSPM0IMU.YawRate    = MSPM0IMU.Gyro[2];
        return true;
    }

    if (id == MSPM0IMU_AX1F_ANGLE)
    {
        MSPM0IMU.YawRaw   = v;
        MSPM0IMU.Angle[2] = MSPM0IMU_YAW_SIGN * ((float)v * MSPM0IMU_SCALE_ANGLE);
        MSPM0IMU.Yaw      = MSPM0IMU.Angle[2];
        return true;
    }

    return false;
}
#endif

static void mspm0imu_dispatch(const uint8_t *f, uint8_t len)
{
    bool ok = false;

    if (len == MSPM0IMU_STD_LEN)
    {
        ok = mspm0imu_parse_std(f);
    }
#if MSPM0IMU_ENABLE_AX1F
    else if (len == MSPM0IMU_AX1F_LEN)
    {
        ok = mspm0imu_parse_ax1f(f);
    }
#endif
    /* 其它长度不可能进入（s_need 只取 11 或 5），忽略即可 */

    if (ok)
    {
        MSPM0IMU.FrameCnt++;
    }
}

/* ============================================================
 * 对外接口
 * ============================================================ */
void MSPM0IMU_Init(void)
{
    (void)memset(&MSPM0IMU, 0, sizeof(MSPM0IMU));

    s_len      = 0u;
    s_need     = 0u;
    s_poll_cnt = 0u;
    s_seen     = 0u;
}

void MSPM0IMU_RxByte(uint8_t data)
{
    /* 等帧头 */
    if (s_len == 0u)
    {
        if (data == MSPM0IMU_HEAD_STD)
        {
            s_buf[0] = data;
            s_len    = 1u;
            s_need   = MSPM0IMU_STD_LEN;
        }
#if MSPM0IMU_ENABLE_AX1F
        else if (data == MSPM0IMU_HEAD_AX1F)
        {
            s_buf[0] = data;
            s_len    = 1u;
            s_need   = MSPM0IMU_AX1F_LEN;
        }
#endif
        return;
    }

    /* 收满一帧 */
    s_buf[s_len++] = data;

    if (s_len < s_need)
    {
        return;
    }

    if (mspm0imu_sum_ok(s_buf, s_need))
    {
        mspm0imu_dispatch(s_buf, s_need);
        s_len  = 0u;
        s_need = 0u;
    }
    else
    {
        MSPM0IMU.ErrCnt++;
        mspm0imu_resync();
    }
}

void MSPM0IMU_Poll(uint32_t now_ms)
{
    uint32_t cnt = MSPM0IMU.FrameCnt;

    if (cnt != s_poll_cnt)
    {
        s_poll_cnt            = cnt;
        s_seen                = 1u;
        MSPM0IMU.LastOnlineTick = now_ms;
        MSPM0IMU.Online       = 1u;
        return;
    }

    if (s_seen == 0u)
    {
        MSPM0IMU.Online = 0u;
        return;
    }

    /* 无符号相减，天然处理 32 位 tick 回绕 */
    if ((uint32_t)(now_ms - MSPM0IMU.LastOnlineTick) >= (uint32_t)MSPM0IMU_OFFLINE_MS)
    {
        MSPM0IMU.Online = 0u;
    }
}

bool MSPM0IMU_IsOnline(void)
{
    return (MSPM0IMU.Online != 0u);
}

float MSPM0IMU_GetYaw(void)
{
    return MSPM0IMU.Angle[2];
}

float MSPM0IMU_GetYawRate(void)
{
    return MSPM0IMU.Gyro[2];
}

/* ============================================================
 * 配置命令
 * ============================================================ */

/* 默认空实现；用户在其它 .c 里定义同名函数即可覆盖 */
MSPM0IMU_WEAK void MSPM0IMU_Send(const uint8_t *data, uint16_t len)
{
    (void)data;
    (void)len;
}

static void mspm0imu_send_cmd(uint8_t addr, uint16_t value)
{
    uint8_t f[5];

    f[0] = 0xFFu;                       /* 标准下行帧头 */
    f[1] = 0xAAu;
    f[2] = addr;
    f[3] = (uint8_t)(value & 0xFFu);
    f[4] = (uint8_t)((value >> 8) & 0xFFu);

    MSPM0IMU_Send(f, (uint16_t)sizeof(f));
}

void MSPM0IMU_SetOutputRate(uint8_t rate_code)
{
    mspm0imu_send_cmd(0x03u, (uint16_t)rate_code);
}

void MSPM0IMU_SetYawZero(void)
{
    mspm0imu_send_cmd(0x76u, 0x0000u);
}

void MSPM0IMU_SetYawDeg(float deg)
{
    float native = deg * MSPM0IMU_YAW_SIGN;   /* 符号为 ±1，等价于除法 */
    float v      = native * (32768.0f / 180.0f);

    if (v >= 32768.0f)
    {
        v = 32767.0f;
    }
    else if (v < -32768.0f)
    {
        v = -32768.0f;
    }

    mspm0imu_send_cmd(0x77u, (uint16_t)(int16_t)v);
}

void MSPM0IMU_StartBiasCal(void)
{
    mspm0imu_send_cmd(0x01u, 0x0001u);       /* CALSE = 1：进入，默认 10s */
}

void MSPM0IMU_StopBiasCal(void)
{
    mspm0imu_send_cmd(0x01u, 0x0004u);       /* CALSE = 4：退出 */
}

void MSPM0IMU_SaveParam(void)
{
    mspm0imu_send_cmd(0x00u, 0x0000u);       /* SAVE = 0 */
}

void MSPM0IMU_Restart(void)
{
    mspm0imu_send_cmd(0x00u, 0x00FFu);       /* SAVE = 0x00FF：重启 */
}

void MSPM0IMU_RequestVersion(void)
{
    mspm0imu_send_cmd(0x27u, 0x002Eu);       /* 读 0x2E 起的 4 个寄存器 */
}
