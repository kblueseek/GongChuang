/* zdtmotor.c — ZDT Y42 闭环步进电机 CAN 控制封装（X 固件） */
#include "zdtmotor.h"
#include "can.h"

#define ZDT_CHECKSUM 0x6B

ZDT_MotorState zdt_state[8];

/* 发送一帧（单包），ID = (地址<<8)|包序号 */
static void ZDT_SendFrame(uint8_t addr, uint8_t packet, const uint8_t *d, uint8_t n)
{
    CAN_TxHeaderTypeDef tx;
    uint32_t mb;
    tx.StdId = 0;
    tx.ExtId = ((uint32_t)addr << 8) | packet;
    tx.IDE = CAN_ID_EXT;
    tx.RTR = CAN_RTR_DATA;
    tx.DLC = n;
    tx.TransmitGlobalTime = DISABLE;
    HAL_CAN_AddTxMessage(&hcan, &tx, (uint8_t *)d, &mb);
}

/* 低层发送：cmd 追加校验码后，按每包 8 字节拆包发送 */
void ZDT_Motor_Send(uint8_t addr, const uint8_t *cmd, uint8_t len)
{
    uint8_t buf[8];
    uint8_t total = len + 1;   /* +校验码 */
    uint8_t sent = 0, packet = 0;

    while (sent < total) {
        uint8_t n = (total - sent > 8) ? 8 : (total - sent);
        for (uint8_t i = 0; i < n; i++) {
            uint8_t idx = sent + i;
            buf[i] = (idx < len) ? cmd[idx] : ZDT_CHECKSUM;
        }
        ZDT_SendFrame(addr, packet, buf, n);
        sent += n;
        packet++;
    }
}

void ZDT_Motor_Init(void)
{
    CAN_FilterTypeDef f = {0};
    f.FilterActivation = CAN_FILTER_ENABLE;
    f.FilterBank = 0;
    f.FilterFIFOAssignment = CAN_RX_FIFO0;
    f.FilterIdHigh = 0;
    f.FilterIdLow = 0;
    f.FilterMaskIdHigh = 0;
    f.FilterMaskIdLow = 0;
    f.FilterMode = CAN_FILTERMODE_IDMASK;
    f.FilterScale = CAN_FILTERSCALE_32BIT;
    HAL_CAN_ConfigFilter(&hcan, &f);
    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void ZDT_Motor_Enable(uint8_t addr, uint8_t en)
{
    uint8_t c[] = {0xF3, 0xAB, en ? 1 : 0, 0x00};
    ZDT_Motor_Send(addr, c, sizeof(c));
}

void ZDT_Motor_Stop(uint8_t addr)
{
    uint8_t c[] = {0xFE, 0x98, 0x00};
    ZDT_Motor_Send(addr, c, sizeof(c));
}

void ZDT_Motor_SetZero(uint8_t addr)
{
    uint8_t c[] = {0x0A, 0x6D};
    ZDT_Motor_Send(addr, c, sizeof(c));
}

void ZDT_Motor_Home(uint8_t addr, uint8_t mode)
{
    uint8_t c[] = {0x9A, mode, 0x00};
    ZDT_Motor_Send(addr, c, sizeof(c));
}

void ZDT_Motor_SyncAll(void)
{
    uint8_t c[] = {0xFF, 0x66};
    ZDT_Motor_Send(0, c, sizeof(c));   /* 广播地址 0 */
}

/* 速度模式（X 固件 F6）：dir + 加速度(RPM/s, 2B) + 速度(0.1RPM, 2B) + 同步标志 */
void ZDT_Motor_Speed(uint8_t addr, uint8_t dir, uint16_t acc, uint16_t speed)
{
    uint8_t c[] = {0xF6, dir, acc >> 8, acc, speed >> 8, speed, 0x00};
    ZDT_Motor_Send(addr, c, sizeof(c));
}

/* 梯形位置模式（X 固件 FD）：dir + 加/减速加速度(2B) + 最大速度(2B) + 位置(0.1°, 4B) + 模式 + 同步标志 */
void ZDT_Motor_Pos(uint8_t addr, uint8_t dir, uint16_t speed, uint16_t acc,
                   uint16_t dec, uint32_t pos, uint8_t mode)
{
    uint8_t c[] = {
        0xFD, dir,
        acc >> 8, acc, dec >> 8, dec,
        speed >> 8, speed,
        pos >> 24, pos >> 16, pos >> 8, pos,
        mode, 0x00
    };
    ZDT_Motor_Send(addr, c, sizeof(c));
}

void ZDT_Motor_ReadPos(uint8_t addr)
{
    uint8_t c[] = {0x36};
    ZDT_Motor_Send(addr, c, sizeof(c));
}

void ZDT_Motor_ReadStatus(uint8_t addr)
{
    uint8_t c[] = {0x3C};
    ZDT_Motor_Send(addr, c, sizeof(c));
}

/* 解析电机返回帧（由 can.c 接收回调调用） */
void ZDT_Motor_RxCallback(uint32_t id, uint8_t *data, uint8_t len)
{
    uint8_t addr = (id >> 8) & 0xFF;
    if (addr == 0 || addr > 7) return;

    switch (data[0]) {
    case 0x36:                       /* 实时位置：符号(1) + 位置(4, 大端) */
        if (len >= 6) {
            int32_t v = (int32_t)((data[2] << 24) | (data[3] << 16) |
                                  (data[4] << 8) | data[5]);
            zdt_state[addr].pos = data[1] ? -v : v;
        }
        break;
    case 0x3C:                       /* 回零状态(1) + 电机状态(1) */
        if (len >= 3) {
            zdt_state[addr].home = data[1];
            zdt_state[addr].status = data[2];
        }
        break;
    default:
        break;
    }
}
