//
// Created by ZanePioneer on 2026/8/30.
//

#ifndef RM2027_THREE_MODE_USER_H
#define RM2027_THREE_MODE_USER_H
#include <sys/_stdint.h>

//A题目的三个模式，把所有初始化执行啥的都封装到一个函数
void speed_mode (int  target_speed);
void angle_mode (uint16_t target_angle);
void protect_mode ();



#endif //RM2027_THREE_MODE_USER_H
