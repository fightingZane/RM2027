//
// Created by ZanePioneer on 2026/8/25.
//
#include "cmsis_os.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

#include "vofa.h"
#include "can_io.hpp"     // Motor_Data
#include "Motor.hpp"

extern float   target_speed;
extern float   target_angle;



//snprintf用于将格式化的数据写入字符串，并限制写入的最大字符数，防止缓冲区溢出。它是 sprintf 的安全版本（AI
static void vofa_send_firewater(float ch0, float ch1, float ch2 ,float ch3 )
{
    char buf[64];
    //这个函数的返回值就是字符串的长度，所以len就是长度，直接用
    int len = snprintf(buf, sizeof(buf), "%.3f,%.3f,%.3f,%.3f\r\n", ch0, ch1, ch2, ch3);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, HAL_MAX_DELAY);
}

extern "C" void Print_task(void const * argument)
{
    (void)argument;
    TickType_t current_tick = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(10);   // 10ms = 100Hz

    for (;;)
    {
        Vofa_Command_analyze();
        vofa_send_firewater(target_speed, (float)Motor_Data_1.Speed,target_angle, (float)Motor_Data_1.Angle);
        vTaskDelayUntil(&current_tick, xPeriod);
    }
}
