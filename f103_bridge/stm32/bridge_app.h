#ifndef BRIDGE_APP_H
#define BRIDGE_APP_H

/* 视觉链路桥的 HAL 胶水层。
 *
 * 为什么单独一个文件：picomm.c 是**纯 C**（不依赖 HAL），所以能在电脑上用 gcc
 * 跑协议自测；HAL 相关的部分全部收在这里，两边互不污染。
 *
 * 在 CubeMX 生成的 main.c 里只需要动三处（都在 USER CODE 区，重新生成不会丢）：
 *   1) USER CODE BEGIN Includes : #include "bridge_app.h"
 *   2) USER CODE BEGIN 2        : BridgeApp_Init();      // 所有 MX_*_Init() 之后
 *   3) USER CODE BEGIN WHILE    : BridgeApp_Poll();      // while(1) 里
 */

/* 在 MX_USART1_UART_Init() / MX_USART2_UART_Init() 之后调用一次。 */
void BridgeApp_Init(void);

/* 在 while(1) 里反复调用。非阻塞：没数据就立刻返回。 */
void BridgeApp_Poll(void);

#endif /* BRIDGE_APP_H */
