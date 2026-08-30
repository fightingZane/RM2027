//
// Created by ZanePioneer on 2026/8/25.
//
#include "cmsis_os.h"
#include "usart.h"
#include "can_io.hpp"     // Motor_Data
#include <string.h>

void Vofa_ParseCommand(void);

extern float   target_speed;
extern Motor_Data Motor_Data_1;

// vofa+ JustFloat：N 个 float32(小端) + 帧尾 00 00 80 7F
//将两个 float 值打包成 VOFA+ 的 JustFloat 协议帧，并通过 UART1 发送出去

static void vofa_send_2float(float ch0, float ch1)
{
    uint8_t buf[12];
    memcpy(buf + 0, &ch0, 4);
    memcpy(buf + 4, &ch1, 4);
    buf[8]  = 0x00;
    buf[9]  = 0x00;
    buf[10] = 0x80;
    buf[11] = 0x7F;
    HAL_UART_Transmit(&huart1, buf, sizeof(buf), HAL_MAX_DELAY);
}

extern "C" void Print_task(void const * argument)
{
    (void)argument;
    TickType_t current_tick = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(10);   // 10ms = 100Hz，波形更平滑

    for (;;)
    {
        Vofa_ParseCommand();
        vofa_send_2float(target_speed, (float)Motor_Data_1.Speed);
        vTaskDelayUntil(&current_tick, xPeriod);
    }
}
