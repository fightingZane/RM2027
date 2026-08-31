//
// Created by ZanePioneer on 2026/8/18.
//

#ifndef RM2027_MOTOR_H
#define RM2027_MOTOR_H

#include "can_io.hpp"
#include "stm32f4xx_hal.h"
#include "pid.hpp"

//宏定义通常是大写，为了方便和一般的代码变量啥的区分开来，黄色
//考核题A：定义三种模式
#define MODE_PROTECT  0   // 保护模式
#define MODE_SPEED    1   // 速度模式
#define MODE_POSITION 2   // 位置模式

typedef struct
{
    uint16_t Angle;
    int16_t Speed;
    int16_t Current;
    int16_t Temperature;
}Motor_Data;


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
extern float target_speed;     // 目标转速
extern uint8_t motor_mode;     // 当前模式
extern PID_t Posparam , Spdparam;
extern Motor_Data Motor_Data_1; //在头文件中extern前要保证在源文件中是变量的定义的
#endif //RM2027_MOTOR_H
