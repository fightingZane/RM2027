//
// Created by ZanePioneer on 2026/8/18.
//

 #include "pid.hpp"

/*
* @name   PID_Init
* @brief  初始化PID速度环的参数
* @param(输入函数的参数)  WhichPID PID结构体指针
* @retval(函数返回值)  None
*/
void PIDc:: PID_Init(PID_t *WhichPID,float Kp,float Ki,float Kd)
{
    //初始化PID的默认参数
    WhichPID->Kp = Kp;
    WhichPID->Ki = Ki;
    WhichPID->Kd = Kd;

    WhichPID->PID_Precision = PID_DEFAULT_PRECISION;
    WhichPID->PID_ErrAllMax = PID_DEFAULT_ERRALL_MAX;
    WhichPID->PID_OutMax = PID_DEFAULT_OUTPUT_MAX;
    WhichPID->PID_OutStep = PID_DEFAULT_OUTPUT_STEP_MAX;
}


/*
* @name   PID_Update
* @brief  更新PID的数据，即更新PID的输入值 (calculate后update)
* @param  WhichPID PID结构体指针,feedback反馈值, Target目标值
* @retval None
*/
void PIDc::PID_Update(PID_t *WhichPID, float feedback,float Target)
{
    //update
    WhichPID->PID_feedback = feedback;
    WhichPID->PID_Target = Target;
    WhichPID->PID_Err_last = WhichPID->PID_Err_now;
    WhichPID->PID_Err_now = WhichPID->PID_Target - WhichPID->PID_feedback;

    //最后err达到设定的理想范围内(+-内)(即最小精度内),视为计算结束,err置零,结束运算
    if (WhichPID->PID_Err_now < WhichPID->PID_Precision && WhichPID->PID_Err_now > -WhichPID->PID_Precision){
        WhichPID->PID_Err_now = 0;
    }

    //积分限幅
    WhichPID->PID_Err_all += WhichPID->PID_Err_now;
    if (WhichPID->PID_Err_all > WhichPID->PID_ErrAllMax){
        WhichPID->PID_Err_all = WhichPID->PID_ErrAllMax;
    }
    else if (WhichPID->PID_Err_all < -WhichPID->PID_ErrAllMax){
        WhichPID->PID_Err_all = -WhichPID->PID_ErrAllMax;
    }
}

/*
* @name   PID_Calculate
* @brief  位置式PID(绝对控制量)，注意不是指位置环PID(诸如电机位置)
* @param  WhichPID PID结构体指针
* @retval PID_Out pid计算之后的输出值
*/
float PIDc::PID_Calculate(PID_t *WhichPID){
    WhichPID->PID_Out =
            WhichPID->Kp * WhichPID->PID_Err_now +
            WhichPID->Kd * (WhichPID->PID_Err_now - WhichPID->PID_Err_last)+
            WhichPID->PID_Err_all * WhichPID->Ki;

    //PID输出限幅
    if (WhichPID->PID_Out >= WhichPID->PID_OutMax)
        WhichPID->PID_Out = WhichPID->PID_OutMax;
    if (WhichPID->PID_Out <= -WhichPID->PID_OutMax)
        WhichPID->PID_Out = -WhichPID->PID_OutMax;

    //PID输出步长限制
    if (WhichPID->PID_Out - WhichPID->PID_last_Out > WhichPID->PID_OutStep)
        WhichPID->PID_Out = WhichPID->PID_last_Out + WhichPID->PID_OutStep;
    if (WhichPID->PID_Out - WhichPID->PID_last_Out < -WhichPID->PID_OutStep)
        WhichPID->PID_Out = WhichPID->PID_last_Out + -WhichPID->PID_OutStep;

    WhichPID->PID_last_Out = WhichPID->PID_Out;
    return WhichPID->PID_Out;
}

/*
* @name   Pos_Spd_PID
* @brief  串级PID
* @param  WhichPID PID结构体指针
* @retval 返回计算结果PID_Out
*/
//Pos = Position，位置；Spd = Speed，速度；param = parameter，参数/状态结构体
//详细讲解：
//首先更新当下的新位置（算出误差等信息），用位置环pid计算出下一步速度的期望值（就是位置环的输出，即posparam->output）
//然后再用位置环的输出作为速度环的期望输入，更新速度环的信息，计算出下一步电机应该如何转动（pwm）
float PIDc::Pos_Spd_PID(PID_t *Spdparam,PID_t *Posparam, float Pos_Target, float Pos_feedback, const float Spd_feedback)
{
    //01:
     PID_Update(Posparam,Pos_feedback,Pos_Target);
     PID_Calculate(Posparam);//先进行位置环PID运算（外环）

     PID_Update(Spdparam,Spd_feedback,Posparam->PID_Out);
     PID_Calculate(Spdparam);//后进行速度环PID运算（内环）

    // //02:由外到内的计算pid
    // SingleLoop_PID (Posparam,Pos_feedback, Pos_Target ); //位置(位置环算出的output就是速度target)
    // SingleLoop_PID (Spdparam,Spd_feedback,Posparam->PID_Out );//速度

    return Spdparam->PID_Out;
}

float PIDc::SingleLoop_PID(PID_t *WhichPID, float feedback, float Target)
{
    PID_Update(WhichPID, feedback, Target);
    return PID_Calculate(WhichPID);
}

void PIDc::PID_Clear(PID_t *WhichPID)
{
    WhichPID->PID_Err_now  = 0.0f;
    WhichPID->PID_Err_last = 0.0f;
    WhichPID->PID_Err_all  = 0.0f;
    WhichPID->PID_Out      = 0.0f;
    WhichPID->PID_last_Out = 0.0f;
}