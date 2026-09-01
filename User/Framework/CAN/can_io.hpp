//
// Created by ZanePioneer on 2026/8/10.
//

#ifndef RM2027_CAN_IO_H
#define RM2027_CAN_IO_H

#include <stm32f4xx_hal.h>

//第二道题；
#define Motor_receive_ID_2 0x206   //0x204+2(id为2)下云台是2号电机
#define Motor_receive_ID_1 0x205   //0x204+1(id为1)上云台是1号电机
#define MOTOR_NUM 2

//第三道题：
//ab板子
#define BOARD_ROLE_A  0
#define BOARD_ROLE_B  1
#define BOARD_ROLE    BOARD_ROLE_B
// 板间通信 ID
#define BOARD_ID_DEG 0x301 // A→B 角度
#define BOARD_ID_RAD  0x302   // B→A 弧度

#define BOARD_ID_RAD_IN 0x303   // A→B 弧度
#define BOARD_ID_DEG_OUT 0x304   // B→A 角度

#define BOARD_ID_HELLO  0x305   // A→B 问好
#define BOARD_ID_REPLY  0x306   // B→A 回礼


// 板间消息储存区
typedef struct
{
    float   deg_in;   // 收到的角度(度)
    float   rad_in;   // 收到的弧度
    float   result_rad; // B 算出的弧度
    float   result_deg;  // B 算出的角度
    volatile uint8_t hello_flag; // B: 收到问好
    volatile uint8_t reply_flag;  // A: 收到回礼
    volatile uint32_t hello_count; // 握手次数

    volatile uint8_t deg_new;//等待换算的角度
    volatile uint8_t rad_new;
} Board_Msg;

extern Board_Msg board_msg;





//1-2题电机结构体：
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
public:
    void can_init();
    void can_send(int16_t ID,int16_t Mess_1,int16_t Mess_2,int16_t Mess_3,int16_t Mess_4 );
    void can_receive();
private:
};

//第三题：
void board_send_float(uint16_t id, float v);
void board_send_deg(float deg);
void board_send_rad(float rad);
void board_send_hello(void);


extern CANc can;
extern Motor_Data gimbal_data[MOTOR_NUM] ; //方便修改1-n个电机，只需要改宏定义的数量就ok


#endif //RM2027_CAN_IO_H
