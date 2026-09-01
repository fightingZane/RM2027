//
// Created by ZanePioneer on 2026/8/18.
//

#include "Motor.hpp"
#include "can_io.hpp"
#include "pid.hpp"


CANc can;
PIDc pid;

MOTORc pitch_motor;
MOTORc yaw_motor;

//默认上电后是0输出,默认第一种保护模式
uint8_t motor_mode   = MODE_PROTECT;


//双环初始化
void MOTORc::motor_S_P_PID_Init()
{
    can.can_init();
    pid.PID_Init(&Spdparam,20.0f,0.02f,0.0f
        ,1.0f,30000.0f,2000.0f,300.0f);
    pid.PID_Init(&Posparam,0.3f,0.005f,0.0f
        ,1.0f,30000.0f,2000.0f,300.0f);
}


//连续角度追踪,让所有的旋转都是在一个连续的角度上cont_angle就是开机到现在旋转的总角度
void MOTORc::Update_cont_Angle()
{
    //此处的feedback只不过是特定电机的motor_data的指针罢了
    float raw   = (float) feedback->Angle;

    // 第一次进来：把当前原始角度作为连续角度的起点
    if (!angle_initialized)
    {
        cont_angle = raw;
        last_raw   = raw;
        angle_initialized = true;
        return;
    }

    float delta = raw - last_raw;//delta->差值
    if (delta > 4096.0f)  //说明是从0->8191,计算会出现8000+,所以要减去一个8192
    {
        delta -= 8192.0f;
    }
    if (delta < -4096.0f) //同上,这个就是 8191->0 所以加上一个8191
    {
        delta += 8192.0f;
    }

    cont_angle += delta; //把正确的角度变化加在总的角度上

    last_raw = raw; //更新一下
}


//速度模式+限位
void MOTORc::motor_S_loop(float target_speed)
{
    float raw = (float) feedback->Angle;

    //此处为反弹代码，限位+反弹巡航
    if (raw >= limit_max - limit_margin && speed_dir > 0)
    {
        speed_dir = -1;
        pid.PID_Clear(&Spdparam);
    }

    else if (raw <= limit_min + limit_margin && speed_dir < 0)
    {
        speed_dir = 1;
        pid.PID_Clear(&Spdparam);
    }

    float spd = target_speed * (float)speed_dir;//速度加上方向
    out = (int16_t)pid.SingleLoop_PID
                (&Spdparam,(float)feedback->Speed,spd);
}


//位置模式+限位+最短路径
void MOTORc::motor_S_P_loop(float target_angle)
{
    Update_cont_Angle();

    //限位
    if (target_angle >= limit_max)  target_angle = limit_max;
    if (target_angle <= limit_min)  target_angle = limit_min;

    //最短路径
    float feedback_round = cont_angle;
    float diff = target_angle - feedback_round ;
    if (diff > 4096.0f)
    {
        feedback_round = feedback_round + 8192.0f;  //目标值太超前了,往前挪一圈
    }
    else if (diff < -4096.0f)
    {
        feedback_round = feedback_round - 8192.0f;   //目标值太落后了,往后挪一圈
    }

    out = (int16_t)pid.Pos_Cascade_PID  //包含pid_update在此
            (&Spdparam,&Posparam,target_angle,feedback_round,feedback->Speed);

}

void MOTORc::motor_Update()
{
    if (motor_mode != last_mode) // 模式切换瞬间清积分
    {
        pid.PID_Clear(&Spdparam);
        pid.PID_Clear(&Posparam);
        last_mode = motor_mode;
    }

    if (motor_mode == MODE_POSITION)
        motor_S_P_loop(target_angle);
    else if (motor_mode == MODE_SPEED)
        motor_S_loop(target_speed);
    else  // 保护模式：零扭矩就是out=0;
        out = 0;
}

