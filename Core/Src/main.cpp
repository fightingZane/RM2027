//
// Created by ZanePioneer on 26-8-25.
//

#include "main.h"
#include "can_io.hpp"
#include "motor.hpp"

int main(){

  main_init();
  motor.motor_Init();
  Rtos_Init();

  while (1)
  {

  }
}