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

extern Usart_DataTypeDef FSUS_usart2;  // 舵机串口
extern UART_HandleTypeDef huart1;       // 调试串口

void User_Uart_Init(UART_HandleTypeDef *huartx);
void Usart_SendAll(Usart_DataTypeDef *usart);
void Debug_Printf(const char *fmt, ...);

#endif
