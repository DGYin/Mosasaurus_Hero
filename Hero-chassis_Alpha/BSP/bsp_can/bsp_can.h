#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "can.h"
#include "chassis_move.h"
#include "trigger_turn.h"
#include "yaw_turn.h"
#include "Briter_Encoder.h"

#define MODE_SWITCH        1
#define MODE_NO_SWITCH     0

#define CAN_3508Motor1_ID         0x201
#define CAN_3508Motor2_ID         0x202
#define CAN_3508Motor3_ID         0x203
#define CAN_3508Motor4_ID         0x204
#define CAN_3508MotorA_ID         0x205
#define CAN_3508MotorB_ID         0x206
#define CAN_3508MotorC_ID         0x207
#define CAN_3508MotorD_ID         0x208
#define YAW_ID    				  0x209

#define CAN_GIMBAL_Y_ID           	  0x20E
#define LOOP_BACK_ID              	  0x003
#define YAW_CONTROL_ID            	  0x006
#define GIMBAL_CONTROL_ID         	  0x007
#define SHOOT_MOTOR_TRIGGER_ID    	  0x208
#define MODE_RECEIVE_ID           	  0X009
#define UI_ID                     	  0x010
#define CAN_RefereeData_ID        	  0x011
#define Chassis_Shoot_Task_Rx_ID       	  0x012
#define Chassis_Motor_Speed_ID	  	  0x013
#define CAN_Yaw_Raw_Angle		  	  0x014
#define CAN_Yaw_Invert_Flag_Trans_ID  0x015
#define CAN_Beta_Power_Limit_ID		  0x016
#define Relay_Mode_Set_ID			  0x017

#define Briter_Encoder1_ID 0x0A
#define Briter_Encoder2_ID 0x0B
#define Briter_Encoder3_ID 0x0C//C
#define Briter_Encoder4_ID 0x0D

//设置两板编号，用于布瑞特编码器的信息传输
#define Alpha_Board 0x111
#define Beta_Board 0x222
#define Board_Code Alpha_Board

uint8_t bsp_can_init(void);
uint8_t Can_Tx_Message(CAN_HandleTypeDef *hcan,uint8_t *mdata);

void Encoder_Data_Process(uint8_t *rxdata, Briter_Encoder_t *Encoder, MOTOR_t *Motor);
void canTX_To_Gimbal_Yaw_Callback(int16_t Angle,int16_t Speed,int16_t shoot_flag_t);
void send_gimbal_data_2(void);
void canTX_Briter_Encoder(Briter_Encoder_t Paramater,CAN_HandleTypeDef *hcan);
void UartTX_To_BetaBoard_Briter_Encoder(Briter_Encoder_t Encoder,int Typecode);
void canTX_To_BetaBoard_WheelVel(void);
void canTX_AGV_Chassis_Motor_Current(void);
void record_yaw_callback(float angle,float speed);
void canTX_To_Beta_Power_Limit(int Power_Mode);
	
extern float pich_angle;
extern int mode_now;
extern char M_DAta[5];

#endif

	
	

