#ifndef __USER_UART_H__
#define __USER_UART_H__

#include "ring_buffer.h"
#include "usart.h"

#define USART_RECV_BUF_SIZE 600
#define USART_SEND_BUF_SIZE 600

typedef struct
{
    UART_HandleTypeDef *huartX;
    RingBufferTypeDef *sendBuf;
    RingBufferTypeDef *recvBuf;
} Usart_DataTypeDef;

extern UART_HandleTypeDef huart1;       // 调试串口

void Debug_Printf(const char *fmt, ...);

#endif
