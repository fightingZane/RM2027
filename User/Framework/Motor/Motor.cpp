//
// Created by ZanePioneer on 2026/8/18.
//

#include "Motor.hpp"

#include "can_io.hpp"
#include "pid.hpp"

extern Motor_Data Motor_Data_1;

PID_t Posparam , Spdparam;

MOTORc motor;
CANc can;
PIDc pid;

//默认上电后是0输出,就是保护模式
float   target_speed;
uint8_t motor_mode   = MODE_PROTECT;

/*
* @part   01角度模式
* @brief  保持在要求的特定的角度
*/

void MOTORc::motor_S_P_PID_Init()
{
    can.can_init();
    pid.PID_Init(&Spdparam,20.0f,0.02f,0.0f);
    pid.PID_Init(&Posparam,0.3f,0.001f,0.0f);
}

void MOTORc::motor_S_P_loop(float target_angle)
{
    can.can_send(0x1FE,
        0,(int16_t)pid.Pos_Spd_PID(&Spdparam,&Posparam,target_angle,Motor_Data_1.Angle, Motor_Data_1.Speed),
        0,0);
}



/*
* @part   02速度模式
* @brief  按照设定速度旋转，这里是只有一个速度环
*/

void MOTORc::motor_S_PID_Init()
{
    can.can_init();
    pid.PID_Init(&Spdparam, 30.0f, 1.0f, 0.0f);
}


void MOTORc::motor_S_loop(float target_speed)
{
    int16_t output = 0;

        // 单速度环：目标转速 target_speed，反馈 Motor_Data_1.Speed（单位 RPM）
        output = (int16_t)pid.SingleLoop_PID(&Spdparam,
                                             (float)Motor_Data_1.Speed,
                                             target_speed);

    // 0x1FF 控制 ID1~4 四个电机；本电机 ID=2，所以电压放第 2 通道(Mess_2)
    can.can_send(0x1EF,0,output,0, 0);
}
