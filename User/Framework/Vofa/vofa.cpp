//
// Created by ZanePioneer on 2026/8/27.
//
#include "usart.h"
#include "Motor.hpp"
#include <stdlib.h>

#define RX_DMA_BUF_SIZE 64

static uint8_t            rx_dma_buf[RX_DMA_BUF_SIZE];
static volatile uint8_t   rx_dma_ready = 0;
static volatile uint16_t  rx_dma_len   = 0;

extern float   target_speed;
extern float   target_angle;

/*
* @name   Vofa_RxStart
* @brief  启动 DMA+空闲中断接收
* @param  none
* @retval none
*/
void Vofa_RxStart()
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma_buf, RX_DMA_BUF_SIZE);
}

/*
* @name   HAL_UART_RxCpltCallback
* @brief  串口接收完成  中断回调函数，作用是一个一个接收字节
* @param  huart uart1的句柄
* @retval none
*/

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    //不是usart1不管
    if (huart->Instance != USART1)
    {
        return;
    }
    rx_dma_len   = Size;   // 本次收到多少字节
    rx_dma_ready = 1;      // 只置标志，不解析、不重启接收
}


/*
* @name   Vofa_ParseCommand
* @brief  解析收到的命令的函数
* @param  none
* @retval none
* 00:atof: 字符串转化为浮点数
*/
void Vofa_Command_analyze(void)
{
    if (rx_dma_ready == 0)
    {
        return;
    }
    rx_dma_ready = 0;


    char line[32];
    uint16_t n = 0;
    for (uint16_t i = 0; i < rx_dma_len && n < (uint16_t)sizeof(line) - 1; i++)
    {
        char c = (char)rx_dma_buf[i];
        if (c == '\r' || c == '\n')
        {
            if (n > 0) break;      // 行结束
            continue;              // 跳过行首的空白
        }
        line[n++] = c;
    }
    line[n] = '\0';

    if (n > 0)
    {
        switch (line[0])
        {
        case 's': case 'S':
            target_speed = (float)atof(&line[1]);
            break;
        case 'm': case 'M':
            if (line[1] == '0') motor_mode = MODE_PROTECT;
            if (line[1] == '1') motor_mode = MODE_SPEED;
            if (line[1] == '2') motor_mode = MODE_POSITION;
            break;
        case 'p': case 'P':
            target_angle = (float) atof ( &line[1] );
            break;
        default:
            break;
        }
    }

    Vofa_RxStart();   // 解析完毕才重启接收，保证缓冲不被覆盖
}