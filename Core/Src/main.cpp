//
// Created by ZanePioneer on 26-8-25.
//

#include "main.h"
#include "can_io.hpp"
#include "motor.hpp"

void Vofa_RxStart(void);
void Vofa_ParseCommand(void);

int main(){

  main_init();          // HAL + 时钟 + GPIO + CAN + USART
  motor.motor_S_P_PID_Init(); //pid初始化
  Vofa_RxStart();       // 开始 UART 接收
  Rtos_Init();          // 启动调度器
  while (1)
  {

  }
}