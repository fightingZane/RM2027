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

//使用dma发送
static uint8_t tx_buf[64];

static void vofa_send_firewater(float ch0, float ch1, float ch2, float ch3)
{

    int len = snprintf((char*)tx_buf, sizeof(tx_buf),
                       "%.3f,%.3f,%.3f,%.3f\n", ch0, ch1, ch2, ch3);
    HAL_UART_Transmit_DMA(&huart1, tx_buf, len);
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