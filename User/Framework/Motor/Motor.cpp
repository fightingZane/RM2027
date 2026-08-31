//
// Created by ZanePioneer on 2026/8/18.
//

#include "Motor.hpp"
#include "can_io.hpp"
#include "pid.hpp"



PID_t Posparam , Spdparam;

MOTORc motor;
CANc can;
PIDc pid;
Motor_Data Motor_Data_1;

//默认上电后是0输出,默认第一种保护模式
float target_speed = 0.0f;
float target_angle = 0.0f;
uint8_t motor_mode   = MODE_PROTECT;

/*
* @part   01角度模式 初始化&串级pid
* @brief  保持在要求的特定的角度
*/

void MOTORc::motor_S_P_PID_Init()
{
    can.can_init();
    pid.PID_Init(&Spdparam,20.0f,0.02f,0.0f
        ,1.0f,30000.0f,2000.0f,300.0f);
    pid.PID_Init(&Posparam,0.3f,0.001f,0.0f
        ,1.0f,30000.0f,2000.0f,300.0f);
}

void MOTORc::motor_S_P_loop(float target_angle)
{
    //最短路径解决:
    //方法1是修改旋转方向 ,方法2是直接让angle向前后者向后多转一圈!!(如下:nb!!
    float feedback_round = (float) Motor_Data_1.Angle;
    float diff = target_angle - feedback_round ;
    if (diff > 4096.0f)
    {
        feedback_round = feedback_round + 8192.0f;  //目标值太超前了,往前挪一圈
    }
    else if (diff < -4096.0f)
    {
        feedback_round = feedback_round - 8192.0f;   //目标值太落后了,往后挪一圈
    }

    int16_t output = 0;
    output = (int16_t)pid.Pos_Cascade_PID  //包含pid_update在此
            (&Spdparam,&Posparam,target_angle,feedback_round,Motor_Data_1.Speed);

    can.can_send(0x1FE,0,output,0,0);
}

/*
* @part   02速度模式
* @brief  按照设定速度旋转，这里是只有一个速度环
*/

void MOTORc::motor_S_PID_Init()
{
    can.can_init();
    pid.PID_Init(&Spdparam, 30.0f, 1.0f, 0.0f
        ,1.0f,30000.0f,2000.0f,300.0f);
}


void MOTORc::motor_S_loop(float target_speed)
{
    int16_t output = 0;
    output = (int16_t)pid.SingleLoop_PID
                (&Spdparam,(float)Motor_Data_1.Speed,target_speed);

    can.can_send(0x1EF,0,output,0, 0);
}
