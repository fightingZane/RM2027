//
// Created by ZanePioneer on 2026/8/18.
//

#include "Motor.hpp"

#include "can_io.hpp"
#include "pid.hpp"

extern Motor_Data Motor_Data_1;

PID_t Posparam,Spdparam;

MOTORc motor;
CANc can;
PIDc pid;

//默认上电后是0输出
float   target_speed = 0.0f;
uint8_t motor_mode   = MODE_PROTECT;

#define SPEED_KP  0.0f
#define SPEED_KI  0.0f
#define SPEED_KD  0.0f

void MOTORc::motor_S_P_PID_Init()
{
    can.can_init();
    pid.PID_Init(&Posparam,0.0f,0.0f,0.0f);
    pid.PID_Init(&Spdparam,0.0f,0.0f,0.0f);
}

void MOTORc::motor_S_PID_Init()
{
    can.can_init();
    pid.PID_Init(&Spdparam, SPEED_KP, SPEED_KI, SPEED_KD);
}


//前提是已经调用了can_recevice来获取上一次数据 motor_data(其中包含电机的 当前转速speed,当前位置angle)
void MOTORc::motor_S_P_loop(float target_angle)
{
    can.can_send(0x1FF,0,
    (int16_t)pid.Pos_Spd_PID(&Spdparam,&Posparam,target_angle,Motor_Data_1.Angle, Motor_Data_1.Speed),
    0,0);
}

void MOTORc::motor_S_loop()
{
    int16_t output = 0;

    if (motor_mode == MODE_SPEED)
    {
        // 单速度环：目标转速 target_speed，反馈 Motor_Data_1.Speed（单位 RPM）
        output = (int16_t)pid.SingleLoop_PID(&Spdparam,
                                             (float)Motor_Data_1.Speed,
                                             target_speed);
    }
    else
    {
        pid.PID_Clear(&Spdparam);   // 清积分，防止切回速度模式瞬间突变
        output = 0;                 // 保护模式：输出 0 电压 = 无扭矩
    }

    // 0x1FF 控制 ID1~4 四个电机；本电机 ID=2，所以电压放第 2 通道(Mess_2)
    can.can_send(0x1FF, 0, output, 0, 0);
}