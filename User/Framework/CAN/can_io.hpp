//
// Created by ZanePioneer on 2026/8/10.
//

#ifndef RM2027_CAN_IO_H
#define RM2027_CAN_IO_H

#include <stm32f4xx_hal.h>
#define Motor_receive_ID_2 0x206   //0x204+2(id为2)下云台是2号电机
#define Motor_receive_ID_1 0x205   //0x204+1(id为1)上云台是1号电机
#define MOTOR_NUM 2

//数据结构体
typedef struct
{
    uint16_t Angle;        // 角度 0~8191
    int16_t  Speed;        // 转速 RPM
    int16_t  Current;      // 电流
    int16_t  Temperature;  // 温度
} Motor_Data;


//can类
class CANc
{
private:

public:
    void can_init();
    void can_send(int16_t ID,int16_t Mess_1,int16_t Mess_2,int16_t Mess_3,int16_t Mess_4 );
    void can_receive();
};


extern CANc can;
extern Motor_Data gimbal_data[MOTOR_NUM] ; //方便修改1-n个电机，只需要改宏定义的数量就ok


#endif //RM2027_CAN_IO_H
