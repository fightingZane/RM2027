//
// Created by ZanePioneer on 2026/8/27.
//
#include "usart.h"
#include "Motor.hpp"    // target_speed / motor_mode / MODE_*
#include <stdlib.h>

static uint8_t  rx_line[32];  //用于存放vofa收到的命令
static uint8_t  rx_idx = 0;   //用于存放字节数量
static volatile uint8_t rx_line_ready = 0;  //标志位，当收到换行符，该变量写为1，是主函数读取收到的命令 的指令
static uint8_t  rx_byte;  //用于临时存放刚从串口收到的那一个字节。因为中断接收一次只接收一个字节

/*
* @name   Vofa_RxStart
* @brief  作用是启动串口中断接收
* @param  none
* @retval none
*/
void Vofa_RxStart(void)
{
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

/*
* @name   HAL_UART_RxCpltCallback
* @brief  串口接收完成中断回调函数，作用是一个一个接收字节
* @param  huart uart1的句柄
* @retval none
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    if (rx_byte == '\n' || rx_byte == '\r')
    {
        rx_line[rx_idx] = '\0';
        rx_line_ready = 1;
        rx_idx = 0;
    }
    else if (rx_idx < sizeof(rx_line) - 1)
    {
        rx_line[rx_idx++] = rx_byte;
    }
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);   // 继续收下一字节
}


/*
* @name   Vofa_ParseCommand
* @brief  命令解析函数
* @param  none
* @retval none
*/
void Vofa_ParseCommand(void)
{
    if (!rx_line_ready) return;
    rx_line_ready = 0;

    switch (rx_line[0])
    {
    case 's': case 'S':
        target_speed = (float)atof((char *)&rx_line[1]);
        break;
    case 'm':
        if (rx_line[1] == '0') motor_mode = MODE_PROTECT;
        if (rx_line[1] == '1') motor_mode = MODE_SPEED;
        break;
    default:
        break;
    }
}