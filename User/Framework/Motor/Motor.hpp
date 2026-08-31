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

class MOTORc
{

public:
    void motor_S_P_PID_Init ();
    void motor_S_PID_Init ();
    void motor_S_P_loop (float target_angle);
    void motor_S_loop (float target_speed);

    //用于得知自己电机反馈值（motor——data）的位置的指针
    void bind_fb_pointor (Motor_Data *fb) { feedback = fb; }          // 绑定自己的反馈槽
    void Set_Limit (float mn, float mx) { limit_min = mn; limit_max = mx; }
    void motor_Update ();                                   // 统一入口：模式分支在类内部
    int16_t Get_Out () const { return out; }                 // 电压交给任务统一发送
    float Get_Cont_Angle () const { return cont_angle; }      // 供 vofa 打印

    float target_speed = 0.0f;
    float target_angle = 5000.0f;

private:
    void Update_cont_Angle();   // 连续角度跟踪

    PID_t Spdparam;
    PID_t Posparam;

    Motor_Data *feedback = nullptr;//给feedback空指针，所以使用fb前一定要先调用get_fb_pointor
    int16_t out = 0;
    float cont_angle = 0.0f;    // 连续角度（跨零点不跳变）
    float last_raw   = 0.0f;
    //这里位置和速度都用一个变量,是因为同一个电机:位置和速度模式不同时运行,所以不冲突
    float limit_min  = -1e9f;   // 默认不限位
    float limit_max  =  1e9f;
    uint8_t last_mode = 0xFF;

};
//电机当前模式
extern uint8_t motor_mode;

extern MOTORc pitch_motor ; //上电机，pitch，改变俯仰角，一号电机
extern MOTORc yaw_motor  ; //下电机，yaw，改变旋转角，二号电机
#endif //RM2027_MOTOR_H
