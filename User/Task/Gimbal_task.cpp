//
// Created by ZanePioneer on 2026/8/25.
//
#include "cmsis_os.h"
#include "Motor.hpp"
#include "pid.hpp"

//定义在motor.cpp文件中
extern float target_speed;
extern float target_angle;
/*
 *FREERTOS小课堂喵：
 *
 *00：extern "C"：C++ 编译器这个函数按 C 语言规则生成符号名（这里就是Gimbal_task），便于 FreeRTOS 的 C 接口调用，这样 C 和 C++ 代码就可以互相调用
 *（因为c++有函数重构，就是编cpp译器会在函数名字里加入一下参数，像这样：函数 void foo(int) 在 C++ 中可能被修饰成 _Z3fooi 之类的，防止重构，要用）
 *
 *01：void const * argument：freertos固定格式，可以接收一个指针
 *(void)argument：表示这个指针我不用
 *
 *02：xTaskGetTickCount()->获取当前系统tick数，类似gettick()
 *pdMS_TO_TICKS->将毫秒数转化为tick数量
 *vTaskDelayUntil(上次任务被唤醒的tick数,每个任务之间差的tick即周期):精确周期性任务 的延时函数
 */
extern "C" void Gimbal_task(void const * argument)
{
    (void)argument;

    TickType_t current_tick= xTaskGetTickCount();

    const TickType_t cycle_tick = pdMS_TO_TICKS(2);


    for (;;)
    {
        yaw_motor.motor_Update();
        pitch_motor.motor_Update();

        can.can_send(0x1FE, pitch_motor.Get_Out(), yaw_motor.Get_Out(), 0, 0);

        vTaskDelayUntil(&current_tick, cycle_tick);
    }
}
