#include "user_uart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

uint8_t usart2SendBuf[USART_SEND_BUF_SIZE + 1];
uint8_t usart2RecvBuf[USART_RECV_BUF_SIZE + 1];
RingBufferTypeDef usart2SendRingBuf;
RingBufferTypeDef usart2RecvRingBuf;
Usart_DataTypeDef FSUS_usart2;
uint8_t rc2;

// maixcam2 DMA接收（在main.c中处理）

void User_Uart_Init(UART_HandleTypeDef *huartx)
{
    // 创建缓冲组
    RingBuffer_Init(&usart2SendRingBuf, USART_SEND_BUF_SIZE, usart2SendBuf);
    RingBuffer_Init(&usart2RecvRingBuf, USART_RECV_BUF_SIZE, usart2RecvBuf);
    // 初始化自定义用户串口结构体
    FSUS_usart2.recvBuf = &usart2RecvRingBuf;
    FSUS_usart2.sendBuf = &usart2SendRingBuf;
    FSUS_usart2.huartX = huartx;
    // 开启接收中断
    HAL_UART_Receive_IT(FSUS_usart2.huartX, (uint8_t *)&rc2, 1);
}

// 发送数据
void Usart_SendAll(Usart_DataTypeDef *usart)
{
    uint8_t value;
    while (RingBuffer_GetByteUsed(usart->sendBuf))
    {
        value = RingBuffer_Pop(usart->sendBuf);
        HAL_UART_Transmit(usart->huartX, &value, 1, 1);
    }
}

// 调试串口发送
void Debug_Printf(const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, strlen(buf), HAL_MAX_DELAY);
}

// 串口调参缓冲区（USART1）
extern uint8_t dbg_rx_buf[32];
extern volatile uint8_t dbg_rx_idx;
extern volatile uint8_t dbg_cmd_len;
extern volatile uint8_t dbg_cmd_ready;

// 中断接收（USART1调参 / USART2舵机 / USART3已改用DMA接收）
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // USART2 舵机接收
    if (huart->Instance == FSUS_usart2.huartX->Instance)
    {
        uint8_t ucTemp = rc2;
        RingBuffer_Push(FSUS_usart2.recvBuf, ucTemp);
        HAL_UART_Receive_IT(FSUS_usart2.huartX, (uint8_t *)&rc2, 1);
    }
    // USART1 串口调参接收
    else if (huart->Instance == USART1)
    {
        uint8_t ch = dbg_rx_buf[dbg_rx_idx];

        if (ch == '\r' || ch == '\n') {
            if (dbg_rx_idx > 0) {
                dbg_rx_buf[dbg_rx_idx] = '\0';
                dbg_cmd_len = dbg_rx_idx;
                dbg_cmd_ready = 1;
                dbg_rx_idx = 0;  // 立即重置，防止\r\n二次触发打乱接收指针
            }
        } else {
            if (dbg_rx_idx < 32 - 1) {
                dbg_rx_idx++;
            }
        }

        HAL_UART_Receive_IT(&huart1, &dbg_rx_buf[dbg_rx_idx], 1);
    }
}
