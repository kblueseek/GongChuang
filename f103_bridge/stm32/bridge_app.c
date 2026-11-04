#include "bridge_app.h"

#include "picomm.h"
#include "usart.h"      /* CubeMX 生成的 huart1 / huart2 在这里声明 */

#define RXBUF_SIZE   256u
#define TXLINE_SIZE  160u

/* ---------- 接收：串口中断每次收 1 字节，塞进环形缓冲 ---------- */
static uint8_t  rx_buf[RXBUF_SIZE];
static volatile uint16_t rx_head = 0u;
static volatile uint16_t rx_tail = 0u;
static uint8_t  rx_byte;          /* HAL 的接收目标，必须一直有效 */

static pi_parser_t g_parser;
static pi_frame_t  g_frame;

/* ---------- 发送：先攒进一行，最后一次性发出去（比逐字节发快得多） ---------- */
static char     tx_line[TXLINE_SIZE];
static uint16_t tx_len = 0u;

static void tx_putc(char c)
{
    if (tx_len < TXLINE_SIZE) {
        tx_line[tx_len++] = c;
    }
}

static void tx_puts(const char *s)
{
    while (*s != '\0') {
        tx_putc(*s++);
    }
}

static void tx_u32(uint32_t value)
{
    char tmp[11];
    uint8_t n = 0u;

    if (value == 0u) {
        tx_putc('0');
        return;
    }
    while (value > 0u && n < sizeof(tmp)) {
        tmp[n++] = (char)('0' + (value % 10u));
        value /= 10u;
    }
    while (n > 0u) {
        tx_putc(tmp[--n]);
    }
}

static void tx_i32(int32_t value)
{
    if (value < 0) {
        tx_putc('-');
        value = -value;
    }
    tx_u32((uint32_t)value);
}

/* 两位定长，高位补 0（步骤号 01..13 用） */
static void tx_u32_pad2(uint32_t value)
{
    if (value < 10u) {
        tx_putc('0');
    }
    tx_u32(value);
}

/* 打印 value/100，固定两位小数（毫米、度都用它） */
static void tx_fixed100(int32_t value)
{
    int32_t frac;

    if (value < 0) {
        tx_putc('-');
        value = -value;
    }
    tx_u32((uint32_t)(value / 100));
    frac = value % 100;
    tx_putc('.');
    tx_putc((char)('0' + (frac / 10)));
    tx_putc((char)('0' + (frac % 10)));
}

/* 把攒好的一行发出去，然后清空 */
static void tx_flush(void)
{
    if (tx_len > 0u) {
        HAL_UART_Transmit(&huart2, (uint8_t *)tx_line, tx_len, 50u);
        tx_len = 0u;
    }
}

/* ---------------------------------------------------------------- 打印 */

static void print_vision_step(const pi_frame_t *f)
{
    uint8_t ring_count;
    uint8_t i;

    if (f->data_len < 20u) {
        tx_puts("[BAD] vision frame data_len<20\r\n");
        return;
    }

    tx_putc('[');
    tx_u32_pad2((uint32_t)f->data[0]);        /* 步骤号 */
    tx_puts("] rings=");

    ring_count = f->data[1];
    if (ring_count > 3u) {
        ring_count = 3u;
    }
    tx_u32((uint32_t)ring_count);

    for (i = 0u; i < ring_count; i++) {
        tx_puts(" (");
        tx_i32((int32_t)pi_get_i16(&f->data[2u + i * 4u]));
        tx_putc(',');
        tx_i32((int32_t)pi_get_i16(&f->data[4u + i * 4u]));
        tx_putc(')');
    }

    tx_puts(" err=(");
    tx_fixed100((int32_t)pi_get_i16(&f->data[14]));
    tx_putc(',');
    tx_fixed100((int32_t)pi_get_i16(&f->data[16]));
    tx_puts(") yaw=");
    tx_fixed100((int32_t)pi_get_i16(&f->data[18]));

    tx_puts(" seq=");
    tx_u32((uint32_t)f->seq);
    tx_puts("\r\n");
}

static void print_frame(const pi_frame_t *f)
{
    if (f->crc_ok == 0u) {
        tx_puts("!! CRC ERR cmd=0x");
        tx_u32((uint32_t)f->cmd);
        tx_puts(" seq=");
        tx_u32((uint32_t)f->seq);
        tx_puts("\r\n");
        return;
    }

    if (f->cmd == PI_CMD_VISION_STEP) {
        print_vision_step(f);
    } else if (f->cmd == PI_CMD_HEARTBEAT) {
        tx_puts("[HB] seq=");
        tx_u32((uint32_t)f->seq);
        tx_puts("\r\n");
    } else {
        tx_puts("[CMD 0x");
        tx_u32((uint32_t)f->cmd);
        tx_puts("] n=");
        tx_u32((uint32_t)f->data_len);
        tx_puts(" seq=");
        tx_u32((uint32_t)f->seq);
        tx_puts("\r\n");
    }
}

/* ---------------------------------------------------------------- HAL 回调 */

/* CubeMX 生成的 usart.c 里调 HAL_UART_Receive_IT 后，每收到 1 字节进这里。
 * 这个函数在 HAL 里是 __weak 的，我们在自己的 .c 里给出强定义即可覆盖。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        uint16_t next = (uint16_t)((rx_head + 1u) % RXBUF_SIZE);

        if (next != rx_tail) {          /* 满了就丢，绝不阻塞 */
            rx_buf[rx_head] = rx_byte;
            rx_head = next;
        }
        /* 立刻重新挂上下一个字节的接收，否则只会收到一个字节 */
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1u);
    }
}

/* ---------------------------------------------------------------- 对外接口 */

void BridgeApp_Init(void)
{
    rx_head = 0u;
    rx_tail = 0u;
    tx_len = 0u;

    pi_parser_reset(&g_parser);
    g_parser.frames_ok = 0u;
    g_parser.frames_bad = 0u;

    tx_puts("\r\n");
    tx_puts("=== F103 vision bridge (HAL) ===\r\n");
    tx_puts("USART1 <- Pi    PA10 / PA9    115200\r\n");
    tx_puts("USART2 -> PC    PA2  / PA3    115200\r\n");
    tx_puts("waiting for frames (AA 55 ...)\r\n\r\n");
    tx_flush();

    /* 启动第一次接收；之后靠回调里的 Receive_IT 续上 */
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1u);
}

void BridgeApp_Poll(void)
{
    static uint32_t last_stat_ok = 0u;

    while (rx_tail != rx_head) {
        uint8_t byte = rx_buf[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1u) % RXBUF_SIZE);

        if (pi_parser_feed(&g_parser, byte, &g_frame)) {
            print_frame(&g_frame);
        }
    }

    /* 每累计 100 个合法帧打一条统计，便于观察链路稳定性 */
    if (g_parser.frames_ok >= last_stat_ok + 100u) {
        last_stat_ok = g_parser.frames_ok;
        tx_puts("[STAT] ok=");
        tx_u32(g_parser.frames_ok);
        tx_puts(" bad=");
        tx_u32(g_parser.frames_bad);
        tx_puts("\r\n");
    }

    tx_flush();
}
