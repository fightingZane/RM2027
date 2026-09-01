//
// Created by ZanePioneer on 2026/8/18.
//

#ifndef RM2027_PID_H
#define RM2027_PID_H

//pid通用结构体，大部分初始化为0，保护机制和精度初始化为常规值
typedef struct
{
    float Kp = 0 ;
    float Ki = 0;
    float Kd = 0;

    float PID_Err_now = 0;    //这次误差
    float PID_Err_last = 0;   //上次误差（算ki用的）
    float PID_Err_all = 0;    //总误差

    float PID_Out = 0;
    float PID_last_Out = 0;
    float PID_Target = 0;
    float PID_feedback = 0;  //pid的输入反馈值

    //很接近时不再计算
    float PID_Precision = 1.0f ; //控制精度，当目标速度与实际速度的差值小于此值时，认为没有误差，pid不再计算
    //保护机制
    float PID_ErrAllMax = 30000.0f ; //控制ERR_ALL最大值（积分最大值），因为每秒500hz，误差累计的还是比较快的，所以给大一点
    float PID_OutMax = 150.0f ; //PID输出限幅
    float PID_OutStep = 300.0f ; //PID输出步幅限制

} PID_t;

class PIDc {
private:
    void PID_Update(PID_t *WhichPID,float feedback,float Target);
    float PID_Calculate(PID_t *WhichPID);

public:
    void PID_Init(PID_t *WhichPID,float Kp,float Ki,float Kd,
                    float PID_Precision,float PID_ErrAllMax,float PID_OutMax ,float PID_OutStep);
    float Pos_Cascade_PID(PID_t *SpdParam,PID_t *PosParam,float Pos_Target,float Pos_feedback,float Spd_Input);
    float SingleLoop_PID(PID_t *Which_PID, float Pos_Input, float Pos_Target);
    void PID_Clear(PID_t *WhichPID);

};
extern PIDc pid;

#endif //RM2027_PID_H
