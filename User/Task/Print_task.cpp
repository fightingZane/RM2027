//
// Created by ZanePioneer on 2026/8/25.
//
#include "cmsis_os.h"
#include "usart.h"

//
//freertos.c 和 Print_task.cpp函数名不同：所以+extern "C"
extern "C" void Print_task(void const * argument)
{
    //显式将未使用的参数标记为“已忽略”，避免编译器产生未使用参数的警告（ai教的
    (void)argument;

    //portTickType和TickType_t大致是类似的，ai：TickType_t新一点，就用TickType_t
    //TickType_t 是 FreeRTOS 定义的类型，通常为 uint32_t（在 32 位 MCU 上），表示系统节拍（tick）计数值。
    //xTaskGetTickCount() 返回当前系统节拍计数器的值
    TickType_t current_tick = xTaskGetTickCount();

    //pdMS_TO_TICKS(ms) 是一个宏，将毫秒转换为系统节拍数,这里就是将50hz的20ms时间间隔转化为系统的节拍数，方便系统计算，和前面对应
    const TickType_t xPeriod = pdMS_TO_TICKS(20);   // 20ms = 50Hz

    for (;;)
    {
        uint8_t massage[] = "RM2027";
        HAL_UART_Transmit(&huart1, massage, sizeof(massage) - 1, HAL_MAX_DELAY);
        vTaskDelayUntil(&current_tick, xPeriod);
    }
}
