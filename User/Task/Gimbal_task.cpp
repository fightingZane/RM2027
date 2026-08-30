//
// Created by ZanePioneer on 2026/8/25.
//
#include "cmsis_os.h"
#include "Motor.hpp"

  // 目标角度  GM6020 编码器 0~8191
  static const float target_angle = 1000.0f;

extern "C" void Gimbal_task(void const * argument)
{
    (void)argument;

    TickType_t current_tick= xTaskGetTickCount();

    const TickType_t cycle_tick = pdMS_TO_TICKS(2);   // 2ms = 500Hz

    for (;;)
    {
        //motor.motor_S_loop( 300 );            // 速度环 PID + CAN 发送
        motor.motor_S_P_loop(1000);

        HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_10);
        vTaskDelayUntil(&current_tick, cycle_tick);  // 严格 500Hz
    }
}
