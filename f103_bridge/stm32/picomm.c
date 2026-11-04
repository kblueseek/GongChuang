#include "picomm.h"

enum {
    PI_ST_SOF1 = 0,
    PI_ST_SOF2,
    PI_ST_LEN,
    PI_ST_SEQ,
    PI_ST_CMD,
    PI_ST_DATA,
    PI_ST_CRC_L,
    PI_ST_CRC_H
};

static void crc_step(uint16_t *crc, uint8_t byte)
{
    uint8_t i;
    *crc ^= byte;
    for (i = 0; i < 8u; i++) {
        if (*crc & 1u) {
            *crc = (uint16_t)((*crc >> 1) ^ 0xA001u);
        } else {
            *crc = (uint16_t)(*crc >> 1);
        }
    }
}

uint16_t pi_crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    uint16_t i;
    for (i = 0; i < len; i++) {
        crc_step(&crc, buf[i]);
    }
    return crc;
}

void pi_parser_reset(pi_parser_t *p)
{
    p->state = PI_ST_SOF1;
    p->len = 0u;
    p->seq = 0u;
    p->cmd = 0u;
    p->index = 0u;
    p->crc = 0xFFFFu;
    p->crc_recv = 0u;
    /* 统计量不清零，便于观察累计效果 */
}

uint8_t pi_get_u8(const uint8_t *p)
{
    return p[0];
}

int16_t pi_get_i16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

int32_t pi_get_i32(const uint8_t *p)
{
    return (int32_t)(((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
                     ((uint32_t)p[2] << 8) | (uint32_t)p[3]);
}

/* 喂一个字节；凑出完整帧时返回 1，并把结果写进 out。 */
int pi_parser_feed(pi_parser_t *p, uint8_t byte, pi_frame_t *out)
{
    uint8_t i;

    switch (p->state) {
    case PI_ST_SOF1:
        if (byte == PI_SOF1) {
            p->state = PI_ST_SOF2;
        }
        break;

    case PI_ST_SOF2:
        if (byte == PI_SOF2) {
            p->state = PI_ST_LEN;
        } else if (byte != PI_SOF1) {
            p->state = PI_ST_SOF1;
        }
        break;

    case PI_ST_LEN:
        /* LEN 至少 3（CMD+CRC），最多 PI_MAX_DATA+3 */
        if (byte < 3u || byte > (uint8_t)(PI_MAX_DATA + 3u)) {
            p->state = PI_ST_SOF1;
            break;
        }
        p->len = byte;
        p->crc = 0xFFFFu;
        crc_step(&p->crc, byte);
        p->state = PI_ST_SEQ;
        break;

    case PI_ST_SEQ:
        p->seq = byte;
        crc_step(&p->crc, byte);
        p->state = PI_ST_CMD;
        break;

    case PI_ST_CMD:
        p->cmd = byte;
        crc_step(&p->crc, byte);
        p->index = 0u;
        p->state = (p->len > 3u) ? PI_ST_DATA : PI_ST_CRC_L;
        break;

    case PI_ST_DATA:
        if (p->index < PI_MAX_DATA) {
            p->data[p->index] = byte;
        }
        p->index++;
        crc_step(&p->crc, byte);
        if (p->index >= (uint8_t)(p->len - 3u)) {
            p->state = PI_ST_CRC_L;
        }
        break;

    case PI_ST_CRC_L:
        p->crc_recv = byte;
        p->state = PI_ST_CRC_H;
        break;

    case PI_ST_CRC_H:
        p->crc_recv |= (uint16_t)((uint16_t)byte << 8);
        p->state = PI_ST_SOF1;

        out->len      = p->len;
        out->seq      = p->seq;
        out->cmd      = p->cmd;
        out->data_len = (uint8_t)(p->len - 3u);
        for (i = 0u; i < out->data_len && i < PI_MAX_DATA; i++) {
            out->data[i] = p->data[i];
        }
        out->crc_calc = p->crc;
        out->crc_recv = p->crc_recv;
        out->crc_ok   = (p->crc == p->crc_recv) ? 1u : 0u;
        if (out->crc_ok) {
            p->frames_ok++;
        } else {
            p->frames_bad++;
        }
        return 1;

    default:
        p->state = PI_ST_SOF1;
        break;
    }
    return 0;
}
