//
// Created by ZanePioneer on 26-8-25.
//

#include "can_io.hpp"
#include "motor.hpp"
#include "main.h"
#include "vofa.h"

#define YAW_LIMIT_MIN    (1500.0f)
#define YAW_LIMIT_MAX    (6500.0f)

#define PITCH_LIMIT_MIN  (3200.0f)
#define PITCH_LIMIT_MAX  (7500.0f)



int main(){

  main_init();

  yaw_motor.motor_S_P_PID_Init();
  pitch_motor.motor_S_P_PID_Init();

  yaw_motor.bind_fb_pointor (&gimbal_data[1]);//下电机2 yaw
  pitch_motor.bind_fb_pointor (&gimbal_data[0]);//上电机1 pitch

  yaw_motor.Set_Limit(YAW_LIMIT_MIN, YAW_LIMIT_MAX);
  pitch_motor.Set_Limit(PITCH_LIMIT_MIN, PITCH_LIMIT_MAX);

  Vofa_RxStart();       // 开始 UART 接收
  Rtos_Init();          // 启动调度器
  while (1)
  {

  }
}