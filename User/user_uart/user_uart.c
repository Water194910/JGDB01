#include "user_uart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// maixcam2 DMA接收（在main.c中处理）

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

// 中断接收（USART1调参 / USART3已改用DMA接收）
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // USART1 串口调参接收
    if (huart->Instance == USART1)
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
