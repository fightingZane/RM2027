//
// Created by ZanePioneer on 2026/8/10.
//

#include "can_io.hpp"

#include "can.h"
#include "stm32f4xx_hal_can.h"

uint8_t rx_data[8];
Motor_Data Motor_Data_1;

void CANc::can_init()
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterBank = 0;         //选择过滤器序号
    can_filter_st.FilterActivation = CAN_FILTER_ENABLE;              //使用过滤器
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;     //使用掩码
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;    //掩码长度 可选32或者16

    //过滤器掩码是0x0000，则无论什么id和掩码“按位与”都是0x0000，则必满足接受的id，则过滤器没有过滤作用
    can_filter_st.FilterIdHigh = 0x0000;       //过滤器接受id的高16位
    can_filter_st.FilterIdLow = 0x0000;        //过滤器接受id的低16位
    can_filter_st.FilterMaskIdHigh = 0x0000;   //过滤器掩码的高16位
    can_filter_st.FilterMaskIdLow = 0x0000;    //过滤器掩码的低16位
    can_filter_st.FilterFIFOAssignment = CAN_FILTER_FIFO0;   //卧槽这好像写错了CAN_RX_FIFO0改为CAN_FILTER_FIFO0
    can_filter_st.SlaveStartFilterBank = 0;

    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);    //写入配置
    HAL_CAN_Start(&hcan1);   //开启can外设
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //开启FIFO0队列中断

}

 void CANc::can_send(int16_t ID,int16_t Mess_1,int16_t Mess_2,int16_t Mess_3,int16_t Mess_4 )
 {


     uint32_t Sent_Mailbox = 0;
     CAN_TxHeaderTypeDef Tx_message;
     uint8_t can_send_message[8];

     Tx_message.DLC = 0x08;          //数据长度为8个字节
     Tx_message.IDE = CAN_ID_STD;    //是否为拓展帧标记点位，std——>标准帧格式
     Tx_message.StdId = ID;          //帧id
     Tx_message.RTR = CAN_RTR_DATA;  //数据帧or遥控帧的选择——>data 数据帧

     //数据的拼接，左右移动
     can_send_message[0] = Mess_1 >> 8 ;  //取高8位
     can_send_message[1] = Mess_1;        //高8位，直接无视
     can_send_message[2] = Mess_2 >> 8 ;
     can_send_message[3] = Mess_2;
     can_send_message[4] = Mess_3 >> 8 ;
     can_send_message[5] = Mess_3;
     can_send_message[6] = Mess_4 >> 8 ;
     can_send_message[7] = Mess_4;

     //传入的内容是can外设的句柄（指针）、can数据的句柄（指针）、发送的数据、是个邮箱？
     HAL_CAN_AddTxMessage(&hcan1, &Tx_message, can_send_message, &Sent_Mailbox);
}

void CANc::can_receive()
{
    CAN_RxHeaderTypeDef rx_header;
    static int16_t ID ;
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data );

    ID =(int16_t) rx_header.StdId;
    switch (ID)
    {
        case Motor_receive_ID : {
        Motor_Data_1.Angle     = (int16_t)((rx_data[0] << 8) | rx_data[1]);
        Motor_Data_1.Speed     = (int16_t)((rx_data[2] << 8) | rx_data[3]);
        Motor_Data_1.Current   = (int16_t)((rx_data[4] << 8) | rx_data[5]);
        Motor_Data_1.Temperature = rx_data[6];
        break;
        }
        default:
            break;
    }


}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    can.can_receive();
}