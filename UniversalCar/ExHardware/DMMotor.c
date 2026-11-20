/**
 * @attention   采用UTF-8字符集编码
 * @brief       达妙（DAMIAO）电机 MIT 模式驱动实现
 * @details     移植自北工大 PIP 战队的 DMMotor 库（v1.0），按本工程重写：
 *               - 去掉 UniversalMotor 通用层，只保留 MIT 模式需要的字段
 *               - 发送走和 ZDT 一样的"等空闲邮箱 + 超时"策略，避免和底盘
 *                 多包广播抢 CAN1 的 3 个发送邮箱
 *               - 反馈解析后做多圈展开，云台位置环要的是连续角度
 */

#include "header.h"

#define DM_TX_TIMEOUT_MS   1       /* 等空闲邮箱的最长时间（ms） */

static DMMotor_t* s_DMList[DM_LIST_MAX] = { NULL };
static uint8_t    s_DMCount = 0;

/* ============ MIT 定点映射 ============ */

static uint16_t DM_FloatToUint(float x, float min, float max, uint8_t bits)
{
    float span = max - min;
    if (span <= 0.0f)
        return 0;
    if (x < min) x = min;
    if (x > max) x = max;
    float v = (x - min) * (float)((1u << bits) - 1u) / span;
    return (uint16_t)v;
}

static float DM_UintToFloat(uint16_t x, float min, float max, uint8_t bits)
{
    float span = max - min;
    if (span <= 0.0f)
        return 0.0f;
    return (float)x * span / (float)((1u << bits) - 1u) + min;
}

/* ============ 发送 ============ */

/* 标准帧（11 位 ID），和 ZDT 的扩展帧在总线上分属两个 ID 空间，不会冲突。
   CAN1 只有 3 个发送邮箱，全满时 HAL 直接返回错误；这里和 ZDT 一样等一会儿，
   但按时间封顶，bus-off 时不会把 5ms 任务卡死。 */
static uint8_t DM_SendFrame(uint8_t canId, const uint8_t* data)
{
    CAN_TxHeaderTypeDef tx = { 0 };
    uint32_t mb;

    tx.StdId = canId;
    tx.ExtId = 0;
    tx.IDE   = CAN_ID_STD;
    tx.RTR   = CAN_RTR_DATA;
    tx.DLC   = 8;

    HAL_StatusTypeDef ret = HAL_CAN_AddTxMessage(&hcan1, &tx, (uint8_t*)data, &mb);
    uint32_t t0 = HAL_GetTick();
    while (ret != HAL_OK && (HAL_GetTick() - t0) < DM_TX_TIMEOUT_MS) {
        osThreadYield();
        ret = HAL_CAN_AddTxMessage(&hcan1, &tx, (uint8_t*)data, &mb);
    }
    return (ret == HAL_OK);
}

static void DM_SendCommand(DMMotor_t* motor, uint8_t cmd)
{
    uint8_t data[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd };

    if (motor == NULL)
        return;
    DM_SendFrame(motor->CanId, data);
}

/* ============ 初始化 / 状态 ============ */

void DMMotor_Init(DMMotor_t* motor, uint8_t canId, uint8_t masterId,
                  float kp, float kd,
                  float pMin, float pMax, float vMin, float vMax,
                  float tMin, float tMax)
{
    if (motor == NULL)
        return;

    for (uint8_t i = 0; i < s_DMCount; i++)
        if (s_DMList[i] == motor)
            return;
    if (s_DMCount >= DM_LIST_MAX)
        return;

    memset(motor, 0, sizeof(*motor));

    motor->CanId    = canId;
    motor->MasterId = masterId;
    motor->kP       = kp;
    motor->kD       = kd;
    motor->P_Min = pMin;  motor->P_Max = pMax;
    motor->V_Min = vMin;  motor->V_Max = vMax;
    motor->T_Min = tMin;  motor->T_Max = tMax;

    motor->isEnable  = 0;
    motor->ErrorFlag = 0;
    motor->isOffline = 1;

    s_DMList[s_DMCount++] = motor;
}

int DMMotor_IsOnline(DMMotor_t* motor)
{
    if (motor == NULL || motor->LastOnlineTick == 0)
        return 0;
    return (HAL_GetTick() - motor->LastOnlineTick) < DM_OFFLINE_MS;
}

/* ============ 一次性命令 ============ */

void DMMotor_Enable(DMMotor_t* motor)     { DM_SendCommand(motor, 0xFC); }
void DMMotor_Disable(DMMotor_t* motor)    { DM_SendCommand(motor, 0xFD); }
void DMMotor_SetZero(DMMotor_t* motor)    { DM_SendCommand(motor, 0xFE); }
void DMMotor_CleanError(DMMotor_t* motor) { DM_SendCommand(motor, 0xFB); }

/* ============ 控制 ============ */

void DMMotor_SetSpeed(DMMotor_t* motor, float radPerSec)
{
    if (motor == NULL)
        return;
    /* 速度源用法：位置项不参与，只给速度目标；kd 提供阻尼，误差为 0 时
       输出就是 kd*(v_des - v)，等价于一个 P 型速度环。 */
    motor->Target.Pos      = motor->Position.Radian;
    motor->Target.Vel      = radPerSec;
    motor->Target.TorqueFF = 0.0f;
}

void DMMotor_SetTarget(DMMotor_t* motor, float rad, float radPerSec, float torqueFF)
{
    if (motor == NULL)
        return;
    motor->Target.Pos      = rad;
    motor->Target.Vel      = radPerSec;
    motor->Target.TorqueFF = torqueFF;
}

void DMMotor_Output(DMMotor_t* motor)
{
    uint8_t data[8];
    uint16_t p, v, kp, kd, t;

    if (motor == NULL || motor->CanId == 0)
        return;

    /* 不管在不在线都发：达妙的回传是跟着控制帧回来的，不发就永远收不到。
       失能状态下继续发也不会把它重新使能（使能靠 0xFC 那帧）。 */

    p  = DM_FloatToUint(motor->Target.Pos,      motor->P_Min, motor->P_Max, 16);
    v  = DM_FloatToUint(motor->Target.Vel,      motor->V_Min, motor->V_Max, 12);
    kp = DM_FloatToUint(motor->kP,              0.0f, 500.0f, 12);
    kd = DM_FloatToUint(motor->kD,              0.0f, 5.0f,   12);
    t  = DM_FloatToUint(motor->Target.TorqueFF, motor->T_Min, motor->T_Max, 12);

    data[0] = (uint8_t)(p >> 8);
    data[1] = (uint8_t)p;
    data[2] = (uint8_t)(v >> 4);
    data[3] = (uint8_t)(((v & 0x0F) << 4) | ((kp >> 8) & 0x0F));
    data[4] = (uint8_t)kp;
    data[5] = (uint8_t)(kd >> 4);
    data[6] = (uint8_t)(((kd & 0x0F) << 4) | ((t >> 8) & 0x0F));
    data[7] = (uint8_t)t;

    DM_SendFrame(motor->CanId, data);
}

/* ============ 接收解析 ============ */

/* 单圈角度在 [-P_MAX, P_MAX] 之间来回跳，跳变超过半个量程就认为过零，
   累加出连续角度。云台位置环、限位判断都按连续角度做。 */
static void DM_UpdatePosition(DMMotor_t* motor, float rawRadian)
{
    float delta = rawRadian - motor->LastRawRadian;
    float span  = motor->P_Max - motor->P_Min;

    if (delta >  span * 0.5f) delta -= span;
    else if (delta < -span * 0.5f) delta += span;

    motor->Position.RadianUnwrapped += delta;
    motor->LastRawRadian = rawRadian;

    motor->Position.Radian = motor->Position.RadianUnwrapped;
    motor->Position.Degree = motor->Position.Radian * 57.2957795f;
}

void DMMotor_RxCallback(uint32_t stdId, uint8_t* data, uint8_t len)
{
    DMMotor_t* motor = NULL;
    uint8_t idNibble;

    if (data == NULL || len < 6)
        return;

    idNibble = data[0] & 0x0F;

    /* 先按 Master ID + 低 4 位电机 ID 精确匹配，找不到再退化成只认 Master ID */
    for (uint8_t i = 0; i < s_DMCount; i++) {
        if (s_DMList[i]->MasterId == stdId && s_DMList[i]->CanId == idNibble) {
            motor = s_DMList[i];
            break;
        }
    }
    if (motor == NULL) {
        for (uint8_t i = 0; i < s_DMCount; i++) {
            if (s_DMList[i]->MasterId == stdId) {
                motor = s_DMList[i];
                break;
            }
        }
    }
    if (motor == NULL)
        return;

    uint16_t posRaw = ((uint16_t)data[1] << 8) | (uint16_t)data[2];
    uint16_t velRaw = ((uint16_t)data[3] << 4) | ((uint16_t)data[4] >> 4);
    uint16_t torRaw = (((uint16_t)data[4] & 0x0F) << 8) | (uint16_t)data[5];

    uint8_t status = data[0] >> 4;
    motor->isEnable  = (status == 1) ? 1 : 0;
    motor->ErrorFlag = (status <= 1) ? 0 : status;

    DM_UpdatePosition(motor, DM_UintToFloat(posRaw, motor->P_Min, motor->P_Max, 16));
    motor->Speed.Radian = DM_UintToFloat(velRaw, motor->V_Min, motor->V_Max, 12);
    motor->Speed.RPM    = motor->Speed.Radian * 9.5492966f;      /* rad/s → rpm */
    motor->Torque.Real  = DM_UintToFloat(torRaw, motor->T_Min, motor->T_Max, 12);

    motor->Temperature    = (data[6] > data[7]) ? data[6] : data[7];
    motor->LastOnlineTick = HAL_GetTick();
    motor->isOffline      = 0;
}
