#ifndef PICOMM_H
#define PICOMM_H

/* PiComm 帧解析（纯 C，不依赖任何 MCU 头文件）
 *
 * 帧格式（与 PiComm.h 逐字节一致）：
 *   AA 55 | LEN | SEQ | CMD | DATA | CRC_L | CRC_H
 *   LEN = len(DATA) + 3
 *   CRC16-MODBUS，初值 0xFFFF，多项式 0xA001
 *   CRC 覆盖 LEN/SEQ/CMD/DATA（不含帧头 AA 55），低字节先发
 *   DATA 采用大端
 */

#include <stdint.h>

#define PI_SOF1          0xAAu
#define PI_SOF2          0x55u
#define PI_MAX_DATA      32u
#define PI_MAX_FRAME     (PI_MAX_DATA + 7u)

/* 下行命令（树莓派 -> MCU） */
#define PI_CMD_VEL           0x01u
#define PI_CMD_GIMBAL        0x02u
#define PI_CMD_GIMBAL_HOME   0x03u
#define PI_CMD_GRIPPER       0x04u
#define PI_CMD_PLATE         0x05u
#define PI_CMD_CAM           0x06u
#define PI_CMD_ENABLE        0x07u
#define PI_CMD_ESTOP         0x08u
#define PI_CMD_HEARTBEAT     0x10u
#define PI_CMD_VISION_STEP   0x20u   /* 链路测试用：一个视觉步骤的结果 */

typedef struct {
    uint8_t  len;
    uint8_t  seq;
    uint8_t  cmd;
    uint8_t  data[PI_MAX_DATA];
    uint8_t  data_len;
    uint16_t crc_calc;
    uint16_t crc_recv;
    uint8_t  crc_ok;
} pi_frame_t;

typedef struct {
    uint8_t  state;
    uint8_t  len;
    uint8_t  seq;
    uint8_t  cmd;
    uint8_t  index;
    uint8_t  data[PI_MAX_DATA];
    uint16_t crc;
    uint16_t crc_recv;
    uint32_t frames_ok;
    uint32_t frames_bad;
} pi_parser_t;

void     pi_parser_reset(pi_parser_t *p);
uint16_t pi_crc16(const uint8_t *buf, uint16_t len);
int      pi_parser_feed(pi_parser_t *p, uint8_t byte, pi_frame_t *out);

uint8_t  pi_get_u8(const uint8_t *p);
int16_t  pi_get_i16(const uint8_t *p);
int32_t  pi_get_i32(const uint8_t *p);

#endif /* PICOMM_H */
