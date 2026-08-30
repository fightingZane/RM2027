//
// Created by ZanePioneer on 2026/8/18.
//

#ifndef RM2027_MOTOR_H
#define RM2027_MOTOR_H

#include "can_io.hpp"
#include "stm32f4xx_hal.h"
#include "pid.hpp"

//定义两种模式
#define MODE_PROTECT  0   // 保护模式：不输出扭矩
#define MODE_SPEED    1   // 速度模式：速度闭环

class MOTORc
{
private:
    PIDc PID;

public:
    void motor_S_P_PID_Init();
    void motor_S_PID_Init();
    void motor_S_P_loop(float target_angle);
    void motor_S_loop(float target_speed);
};

//因为 MOTORc motor 是 main() 里的局部变量，但 Gimbal_task 里也要调 motor.motor_loop()，必须让两个文件共享同一个对象（AI
extern MOTORc motor;
extern float   target_speed;   // 目标转速
extern uint8_t motor_mode;     // 当前模式


#endif //RM2027_MOTOR_H
