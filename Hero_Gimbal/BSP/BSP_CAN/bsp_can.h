#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "stm32f4xx.h"
#include "can.h"

#define LK_Pitch_Motor_ID 0x141
uint8_t Bsp_canInit(void);

extern uint8_t LK_Pitch_Motor_Send_Data[8];
extern uint8_t LK_Pitch_Motor_Receive_Data[8];

#endif
