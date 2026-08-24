//
// Created by ZanePioneer on 2026/8/18.
//

#ifndef RM2027_MOTOR_H
#define RM2027_MOTOR_H

#include "can_io.hpp"
#include "stm32f4xx_hal.h"
#include "pid.hpp"

class Motor
{
private:
    PIDc PID;


public:
    void PID_Init(void);
    void PID_loop();

};


#endif //RM2027_MOTOR_H
