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
static uint8_t tx_buf[128];

static void vofa_send_firewater  (const float *ch)
{

    int len = snprintf((char *)tx_buf, sizeof(tx_buf),
                      "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                      ch[0], ch[1], ch[2], ch[3],
                      ch[4], ch[5], ch[6], ch[7]);
    HAL_UART_Transmit_DMA(&huart1, tx_buf, len);
}

extern "C" void Print_task(void const * argument)
{
    (void)argument;
    TickType_t current_tick = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(20);   //20ms一次

    for (;;)
    {
        Vofa_Command_analyze();

        float ch[8] = {
            // ---- Yaw ----
            yaw_motor.target_angle,
            (float)gimbal_data[0].Angle,
            yaw_motor.target_speed,
            (float)gimbal_data[0].Speed,
            // ---- Pitch ----
            pitch_motor.target_angle,
            (float)gimbal_data[1].Angle,
            pitch_motor.target_speed,
            (float)gimbal_data[1].Speed
        };

        vofa_send_firewater(ch);

        vTaskDelayUntil(&current_tick, xPeriod);
    }
}
