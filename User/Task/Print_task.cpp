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

//使用dma发送
static uint8_t tx_buf[128];

static void vofa_send_firewater  (const int32_t *ch)
{

    int len = snprintf((char *)tx_buf, sizeof(tx_buf),
                      "%d,%d,%d,%d,%d,%d,%d,%d\n",
                      (int)ch[0], (int)ch[1], (int)ch[2], (int)ch[3],
                      (int)ch[4], (int)ch[5], (int)ch[6], (int)ch[7]);
    HAL_UART_Transmit_DMA(&huart1, tx_buf, (uint16_t) len);
}

extern "C" void Print_task(void const * argument)
{
    (void)argument;
    TickType_t current_tick = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(50);

    for (;;)
    {
        Vofa_Command_analyze();

        int32_t ch[8] = {
            //上电机，gimbal[0],pitch
            (int32_t) pitch_motor.target_angle,
            (int32_t) gimbal_data[0].Angle,
            (int32_t) pitch_motor.target_speed,
            (int32_t) gimbal_data[0].Speed,

            //下电机，gimbal[1],yaw
            (int32_t) yaw_motor.target_angle,
            (int32_t) gimbal_data[1].Angle,
            (int32_t) yaw_motor.target_speed,
            (int32_t) gimbal_data[1].Speed
        };

        vofa_send_firewater(ch);

        vTaskDelayUntil(&current_tick, xPeriod);
    }
}
