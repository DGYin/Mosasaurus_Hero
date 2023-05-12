#ifndef __can_receive_H
#define __can_receive_H

#include "main.h"

#include "bsp_can.h"
//#define PITCH_MOTOR_ID				 0x206
#define PITCH_MOTOR_ID				 0x205
#define SHOOT_LEFT_MOTOR_ID    0x201
#define SHOOT_RIGH_MOTOR_ID    0x202
#define SHOOT_MOTOR_TRIGGER_ID 0x207

#define CAN_Yaw_Raw_Angle		  0x014

#define YAW_ID    0x209

typedef struct
{
	float    lastangle; /*上次电机角度*/
	
	float 		Total_Angle;/*角度和*/
	
	float 		angle;/*电机实际角度*/
	
	int16_t		  speed;/*电机实时速度*/
	
	uint8_t     temp; /*电机温度*/
	
	int32_t 		turns;/*电机从上电开始转过的圈数*/
}Motor_HandleTypeDef;		


extern Motor_HandleTypeDef yaw_can_rx,pitch_can_rx,shoot_can_rx[2];
extern int shoot_flag;


#endif
