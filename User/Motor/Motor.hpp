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


#endif //RM2027_MOTOR_H
