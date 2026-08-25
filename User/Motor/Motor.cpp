//
// Created by ZanePioneer on 2026/8/18.
//

#include "Motor.hpp"

#include "can_io.hpp"
#include "pid.hpp"

extern Motor_Data Motor_Data_1;

PID_t Posparam,Spdparam;

CANc can;
PIDc pid;

void MOTORc::motor_Init()
{
    can.can_init();
    pid.PID_Init(&Posparam,0.0f,0.0f,0.0f);
    pid.PID_Init(&Spdparam,0.0f,0.0f,0.0f);
}

//前提是已经调用了can_recevice来获取上一次数据 motor_data(其中包含电机的 当前转速speed,当前位置angle)
void MOTORc::motor_loop(float target_angle)
{
    can.can_send(0x1FE,0,
    (int16_t)pid.Pos_Spd_PID(&Spdparam,&Posparam,target_angle,Motor_Data_1.Angle, Motor_Data_1.Speed),
    0,0);
}
