//
// Created by ZanePioneer on 2026/8/27.
//
#include "usart.h"
#include "Motor.hpp"
#include <stdlib.h>

//中断里改，主循环里读取的变量都要加volatile，防止被编译器优化！
static volatile uint8_t  rx_line[32];
//用于存放vofa收到的命令,注意这里不能使用uint16_t
//并且uint8_t是储存文本字符,而不是数字本身,所以8191->四个字符四个字节 ,8个字节 完全够用
static volatile uint8_t  rx_num = 0;    //用于存放字符个数和下一个写入字符的索引
static volatile uint8_t rx_line_ready = 0;  //标志位，当收到换行符，该变量写为1，是主函数读取收到的命令 的指令
static uint8_t  rx_byte;  //用于临时存放刚从串口收到的那一个字节。因为中断接收一次只接收一个字节

extern float   target_speed;
extern float   target_angle;

/*
* @name   Vofa_RxStart
* @brief  作用是启动串口中断接收
* @param  none
* @retval none
*/
void Vofa_RxStart()
{
    HAL_UART_Receive_IT (&huart1, &rx_byte, 1);
}

/*
* @name   HAL_UART_RxCpltCallback
* @brief  串口接收完成  中断回调函数，作用是一个一个接收字节
* @param  huart uart1的句柄
* @retval none
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    //不是usart1不管
    if (huart->Instance != USART1)
    {
        return;
    }

    if (rx_byte == '\n' || rx_byte == '\r')
    {
        rx_line[rx_num] = '\0';  //字符串最后都是有一个\0来表示字符串的的结束
        rx_line_ready = 1;  //标志
        rx_num = 0;
    }
    else if (rx_num < sizeof(rx_line) - 1)
    {
        rx_line[rx_num++] = rx_byte;
        //不能写成这个：会漏rx_line[0]
        // rx_num++;//计数
        // rx_line[rx_num] = rx_byte;
    }


    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);   // 继续收下一字节
}


/*
* @name   Vofa_ParseCommand
* @brief  解析收到的命令的函数
* @param  none
* @retval none
* 00:atof: 字符串转化为浮点数
*/
void Vofa_Command_analyze()
{
    if (rx_line_ready == 0)
    {
        return;
    }
    //如果rx_line_ready不是0,就是1,则先将rx_line_ready置为0
    rx_line_ready = 0;

    switch (rx_line[0])
    {
    case 's': case 'S':
        target_speed =  ( float ) atof ( (char *) &rx_line[1] );  //rx_line[1]是字符串,转化为浮点数
        break;

    case 'm': case 'M':
        if (rx_line[1] == '0') motor_mode = MODE_PROTECT;
        if (rx_line[1] == '1') motor_mode = MODE_SPEED;
        if (rx_line[1] == '2') motor_mode = MODE_POSITION;
        break;

    case 'p':case 'P':
        target_angle =  ( float ) atof ( (char *) &rx_line[1] );
        break;

    default:
        break;
    }
}