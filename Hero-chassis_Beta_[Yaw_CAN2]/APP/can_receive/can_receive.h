#ifndef __CAN_RECEIVE_H
#define __CAN_RECEIVE_H

#include "stm32f4xx.h"

typedef struct
{
	uint16_t    lastangle; /*上次电机角度*/
	
	uint16_t 			sumangle;/*角度和*/
	
	uint16_t 		angle;/*电机实际角度*/
	
	int16_t		  speed;/*电机实时速度*/
	
	uint8_t     temp; /*电机温度*/
	
	int32_t 		turns;/*电机从上电开始转过的圈数*/
}Motor_HandleTypeDef;		


extern Motor_HandleTypeDef yaw_can_rx,pitch_can_rx,shoot_can_rx[2];



#endif
