#ifndef __sent_task_H
#define __sent_task_H

#include "stm32f4xx.h"

#define Chassis_Shoot_Task_Tx_ID	0x012
#define CAN_Invert_Flag_Trans_ID	0x015
#define Relay_Mode_Set_ID			0x017 


uint8_t canTX_chassis(int16_t x,int16_t y,int8_t z,int8_t deviation);
uint8_t canTX_fric(int16_t left,int16_t right);
uint8_t canTX_trigger(uint8_t trigger);
uint8_t canTX_pitch(int16_t pitch);
uint8_t canTX_yaw(int16_t yaw,int16_t yaw_current);
uint8_t canTX_Yaw_Current(int16_t yaw_current);
uint8_t CAN_Tx_Mode(uint8_t Chassis_Mode, int Precision_Mode, int Chassis_Follow_Mode);
void canTX_Invert_Flag(uint8_t Bool_Invert_Flag);

void canTX_LK_Pitch_Motor(void);
void canTX_Relay_Set_Mode(void);
uint8_t canTX_UI(int pitch, int mode, int Pitch_Temp);

#endif
