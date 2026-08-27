//
// Created by ZanePioneer on 2026/8/18.
//

#ifndef RM2027_PID_H
#define RM2027_PID_H


#define PID_DEFAULT_PRECISION      1.0    //控制精度，当目标速度与实际速度的差值小于此值时，认为没有误差，pid不再计算
#define PID_DEFAULT_ERRALL_MAX    30000.0f   //控制ERR_ALL最大值（积分最大值），否则ERR_ALL最大值过大，会使PID反应慢，不稳
#define PID_DEFAULT_OUTPUT_MAX     30000.0f   //输出限幅
#define PID_DEFAULT_OUTPUT_STEP_MAX  30000.0f   //输出微分限幅

//
//pid通用结构体
//

typedef struct
{
    float Kp = 0;
    float Ki = 0;
    float Kd = 0;

    float PID_Err_now = 0;
    float PID_Err_last = 0;
    float PID_Err_all = 0;

    float PID_Out = 0;
    float PID_last_Out = 0;
    float PID_Target = 0;
    float PID_feedback = 0;  //pid的输入反馈值

    //很接近时不再计算
    float PID_Precision = 0; //pid最小精度
    //保护机制
    float PID_ErrAllMax = 0; //pid积分限幅
    float PID_OutMax = 0; //PID输出限幅
    float PID_OutStep = 0; //PID输出步幅限制

} PID_t;


class PIDc {

private:
    void PID_Update(PID_t *WhichPID,float NowInput,float Target);
    float PID_Calculate(PID_t *WhichPID);

public:
    void PID_Init(PID_t *WhichPID,float Kp,float Ki,float Kd);
    float Pos_Spd_PID(PID_t *SpdParam,PID_t *PosParam,float Pos_Target,float Pos_feedback,float Spd_Input);
    float SingleLoop_PID(PID_t *Which_PID, float Pos_Input, float Pos_Target);
    void PID_Clear(PID_t *WhichPID);
    PID_t SpdParam;
    PID_t PosParam;
};


#endif //RM2027_PID_H
