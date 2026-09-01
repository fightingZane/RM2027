//
// Created by ZanePioneer on 2026/8/25.
//
#include "cmsis_os.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

#include "vofa.h"
#include "can_io.hpp"     // Motor_Data
#include "Motor.hpp"

//使用dma发送
static uint8_t tx_buf[128];

static void vofa_send_firewater(const float *ch)
{
    int len = snprintf((char *)tx_buf, sizeof(tx_buf),
                       "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                       ch[0], ch[1], ch[2], ch[3],
                       ch[4], ch[5], ch[6], ch[7]);
    HAL_UART_Transmit_DMA(&huart1, tx_buf, (uint16_t)len);
}

extern "C" void Print_task(void const * argument)
{
    (void)argument;
    TickType_t current_tick = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(20);

    for (;;)
    {
        Vofa_Command_analyze();

    //不同于一般的if语句，这个是预编译编译指令，在编译之前就会预处理
    //通常用于不同的开发板？母鸡
    #if BOARD_ROLE == BOARD_ROLE_A
        {
            static uint32_t tick_count = 0;   // 只在板A编译，板B不会出现

            // ① 发角度：有电机用真实编码器角度，无电机演示可改用固定值
            float deg = (float)gimbal_data[0].Angle * 360.0f / 8192.0f;
            board_send_deg(deg);

            // ② 每 1s（50 次×20ms）问好一次
            if (++tick_count >= 50)
            {
                tick_count = 0;
                board_send_hello();
            }
            // ③ 收到的结果（result_rad / reply_flag）直接在下面 vofa 打印
        }
#else   // ===== 板 B =====
        // ① 收到角度 → 换弧度 → 回发
        if (board_msg.deg_new)
        {
            board_msg.deg_new = 0;
            board_send_rad(board_msg.deg_in * 3.14159265f / 180.0f); // 发 0x302
        }
        // ② 收到弧度 → 换角度 → 回发
        if (board_msg.rad_new)
        {
            board_msg.rad_new = 0;
            board_send_float(BOARD_ID_DEG_OUT,
                             board_msg.rad_in * 180.0f / 3.14159265f);
        }
        // ③ 收到问好 → 回礼
        if (board_msg.hello_flag)
        {
            board_msg.hello_flag = 0;
            can.can_send(BOARD_ID_REPLY, 0x0055, 0, 0, 0);
        }
#endif
        // ===== 周期逻辑结束 =====

        // vofa 打印（按角色选不同内容）
#if BOARD_ROLE == BOARD_ROLE_A
        float ch[8] = {
            (float)gimbal_data[0].Angle,   // 发出的角度(编码值)
            board_msg.result_rad,          // 收到的弧度
            board_msg.result_deg,          // 收到的角度结果
            (float)board_msg.reply_flag,   // 回礼标志
            0, 0, 0, 0
        };
#else
        float ch[8] = {
            board_msg.deg_in,              // 收到的角度(度)
            board_msg.deg_in * 3.14159265f / 180.0f,  // 算出的弧度
            board_msg.rad_in,              // 收到的弧度
            (float)board_msg.hello_count,  // 握手次数
            0, 0, 0, 0
        };
#endif
        vofa_send_firewater(ch);


        // int32_t ch[8] = {
        //     //上电机，gimbal[0],pitch
        //     (int32_t) pitch_motor.target_angle,
        //     (int32_t) gimbal_data[0].Angle,
        //     (int32_t) pitch_motor.target_speed,
        //     (int32_t) gimbal_data[0].Speed,
        //
        //     //下电机，gimbal[1],yaw
        //     (int32_t) yaw_motor.target_angle,
        //     (int32_t) gimbal_data[1].Angle,
        //     (int32_t) yaw_motor.target_speed,
        //     (int32_t) gimbal_data[1].Speed
        // };
        //
        // vofa_send_firewater(ch);

        vTaskDelayUntil(&current_tick, xPeriod);
    }
}
