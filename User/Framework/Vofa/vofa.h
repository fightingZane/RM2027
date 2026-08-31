//
// Created by ZanePioneer on 2026/8/31.
//

#ifndef RM2027_VOFA_H
#define RM2027_VOFA_H

void Vofa_RxStart();
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void Vofa_Command_analyze();

#endif //RM2027_VOFA_H
