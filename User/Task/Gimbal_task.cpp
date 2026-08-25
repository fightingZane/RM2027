//
// Created by ZanePioneer on 2026/8/25.
//
#include "cmsis_os.h"
#include "Motor.hpp"

  // 目标角度（GM6020 编码器 0~8191，这里手动写死）
  static const float TARGET_ANGLE = 1000.0f;

extern "C" void Gimbal_task(void const * argument)
{
    (void)argument;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(2);   // 2ms = 500Hz

    for (;;)
    {
        motor.motor_loop(TARGET_ANGLE);            // 角度串级 PID + CAN 发送
        vTaskDelayUntil(&xLastWakeTime, xPeriod);  // 严格 500Hz
    }
}