//
// Created by ZanePioneer on 2026/8/18.
//

#ifndef RM2027_MOTOR_H
#define RM2027_MOTOR_H

#include "can_io.hpp"
#include "stm32f4xx_hal.h"
#include "pid.hpp"

class MOTORc
{
private:
    PIDc PID;


public:
    void motor_Init();
    void motor_loop(float target_angle);

};

//因为 MOTORc motor 是 main() 里的局部变量，但 Gimbal_task 里也要调 motor.motor_loop()，必须让两个文件共享同一个对象（AI
extern MOTORc motor;

#endif //RM2027_MOTOR_H
