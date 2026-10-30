#ifndef UART_COMM_H
#define UART_COMM_H

#include "main.h"
#include <stdint.h>

/*
 * 串口收发封装：
 *   - 收：中断逐字节收，交给 protocol.c 组帧；
 *   - 发：中断发送（不阻塞中断上下文），上一帧没发完就丢弃这一帧。
 *
 * 注意：不要在中断里用阻塞发送，一帧最长 37 字节 @115200 约 3.2ms，
 *       会把 10ms 的控制周期吃掉三分之一。
 */
#ifdef __cplusplus
extern "C" {
#endif

void UARTComm_Init(UART_HandleTypeDef *huart);
void UARTComm_StartReceive(void);
HAL_StatusTypeDef UARTComm_SendFrame(uint8_t cmd, uint8_t len, const uint8_t *data);

/* 调试用：累计收到的字节数（OLED 系统页显示，用来确认串口通不通） */
uint32_t UARTComm_GetRxBytes(void);

#ifdef __cplusplus
}
#endif

#endif
