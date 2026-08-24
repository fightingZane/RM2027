//
// Created by ZanePioneer on 2026/8/10.
//

#ifndef RM2027_CAN_IO_H
#define RM2027_CAN_IO_H

#include <stm32f4xx_hal.h>
#define Yaw_receive_ID 0x206   //0x204+2(id为2)

//结构体命名
typedef struct
{
    uint16_t Angle;
    int16_t Speed;
    int16_t Current;
    int16_t Temperature;
}Motor_Data;

class CANc {
private:

public:

    void can_init();
    void can_send(int16_t ID,int16_t Mess_1,int16_t Mess_2,int16_t Mess_3,int16_t Mess_4 );
    void can_receive();
};


#endif //RM2027_CAN_IO_H
