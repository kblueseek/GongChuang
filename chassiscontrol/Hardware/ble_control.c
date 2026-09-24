/**
 * @file    ble_control.c
 * @brief   蓝牙 BLE 数据包收发实现
 *          接收: USART2 (PA3 RX) 中断逐字节捕获 [xxx] 数据包
 *          发送: USART2 (PA2 TX) 发往手机
 */
#include "ble_control.h"
#include "usart.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

/* ============ 接收缓冲 ============ */
#define RX_BUF_SIZE     200
char               BLE_RxPacket[RX_BUF_SIZE];
volatile uint8_t   BLE_RxFlag = 0;

/* ============ 内部: 发送单字节 ============ */
static void BLE_SendByte(uint8_t byte)
{
    HAL_UART_Transmit(&huart2, &byte, 1, 10);
}

/* ============ 内部: 发送字符串 ============ */
static void BLE_SendString(const char *str)
{
    while (*str) {
        BLE_SendByte((uint8_t)*str++);
    }
}

/* ============ printf 重定向到 BLE ============ */
void BLE_Printf(const char *fmt, ...)
{
    char buf[200];
    va_list arg;
    va_start(arg, fmt);
    vsnprintf(buf, sizeof(buf), fmt, arg);
    va_end(arg);
    BLE_SendString(buf);
}

/* ============ 发送绘图数据包 ============ */
void BLE_SendPlot(float *values, int count)
{
    if (count < 1 || count > 10) return;
    BLE_SendString("[p,");
    for (int i = 0; i < count; i++) {
        char num[32];
        snprintf(num, sizeof(num), "%.2f", values[i]);
        BLE_SendString(num);
        if (i < count - 1) BLE_SendByte(',');
    }
    BLE_SendString("]\r\n");
}

void BLE_SendPlotClear(void)
{
    BLE_SendString("[p-c]\r\n");
}

/* ============ 数据包解析 ============ */
static void ParsePacket(const char *packet)
{
    char buf[RX_BUF_SIZE];
    strncpy(buf, packet, RX_BUF_SIZE - 1);
    buf[RX_BUF_SIZE - 1] = '\0';

    char *Tag = strtok(buf, ",");
    if (Tag == NULL) return;

    /* ---- 滑杆: [slider,name,value] 或 [s,name,value] ---- */
    if (strcmp(Tag, "slider") == 0 || strcmp(Tag, "s") == 0)
    {
        BLE_SliderEvent e;
        char *n = strtok(NULL, ",");
        char *v = strtok(NULL, ",");
        if (n && v) {
            strncpy(e.name, n, sizeof(e.name) - 1);
            e.name[sizeof(e.name) - 1] = '\0';
            e.value = atof(v);
            BLE_OnSlider(&e);
        }
    }
    /* ---- 摇杆: [joystick,Lx,Ly,Rx,Ry] 或 [j,...] ---- */
    else if (strcmp(Tag, "joystick") == 0 || strcmp(Tag, "j") == 0)
    {
        BLE_JoystickEvent e;
        char *s[4];
        for (int i = 0; i < 4; i++) s[i] = strtok(NULL, ",");
        if (s[0] && s[1] && s[2] && s[3]) {
            e.Lx = atoi(s[0]);
            e.Ly = atoi(s[1]);
            e.Rx = atoi(s[2]);
            e.Ry = atoi(s[3]);
            BLE_OnJoystick(&e);
        }
    }
    /* ---- 按键: [key,name,down|up] 或 [k,name,d|u] ---- */
    else if (strcmp(Tag, "key") == 0 || strcmp(Tag, "k") == 0)
    {
        BLE_KeyEvent e;
        char *n = strtok(NULL, ",");
        char *a = strtok(NULL, ",");
        if (n && a) {
            e.name    = atoi(n);
            e.is_down = (strcmp(a, "down") == 0 || strcmp(a, "d") == 0);
            BLE_OnKey(&e);
        }
    }
}

void BLE_Process(void)
{
    if (BLE_RxFlag) {
        BLE_RxFlag = 0;
        ParsePacket(BLE_RxPacket);
    }
}

/* ============ 字节回调 (由 usart.c 的 HAL_UART_RxCpltCallback 调用) ============ */
void BLE_RxCallback(uint8_t data)
{
    static uint8_t state  = 0;   /* 0=等 '[', 1=收包内容 */
    static uint16_t index = 0;

    if (state == 0) {
        if (data == '[' && BLE_RxFlag == 0) {
            state = 1;
            index = 0;
        }
    } else if (state == 1) {
        if (data == ']') {
            state = 0;
            BLE_RxPacket[index] = '\0';
            BLE_RxFlag = 1;
        } else if (index < RX_BUF_SIZE - 1) {
            BLE_RxPacket[index++] = (char)data;
        }
    }
}

/* 来自 usart.c 的 UART RX 字节缓冲 */
extern uint8_t uart2_rx_byte;

/* ============ 初始化: 启动 USART2 中断接收 ============ */
void BLE_Init(void)
{
    HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
}
