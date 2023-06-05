#include "can_receive.h"
#include "stm32f4xx.h"
#include "gimbal_task.h"
#include "string.h"
#include "math.h"
#include "led.h"
#include "shoot_task.h"
#include "bsp_can.h"
#include "lk_pitch_turn.h"
#include "gimbal_calibration_task.h"

Motor_HandleTypeDef	yaw_can_rx = {0}, pitch_can_rx = {0}, shoot_can_rx[2] = {0};

void Yaw_Angle_Process(void);

int16_t euler_angle[3];
int shoot_flag = 0;
/**
  * @brief          can总线的接收
  * @param[*hcan]   CAN_HandleTypeDef类型的指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxFrame;
    uint8_t rxData[8] = {0};
    led_on(&led[2]);
    if(hcan == &hcan1)
    {

        HAL_CAN_GetRxMessage(&hcan1,  CAN_RX_FIFO0, &rxFrame,  rxData);
        switch (rxFrame.StdId)
        {
			//			case SHOOT_MOTOR_TRIGGER_ID:
			//								{
			//										Trigger_Motor_Callback(&rc_shoot.trigger,(int16_t)rxData[0]<<8|rxData[1],(int16_t) rxData[2]<<8|rxData[3]);
			//								}
			case PITCH_MOTOR_ID:
				pitch_can_rx.angle = (int16_t) rxData[7] << 8 | rxData[6];
				pitch_can_rx.speed = (int16_t) rxData[5] << 8 | rxData[4];
				pitch_can_rx.temp  = (int16_t) rxData[1];
				break;
			case SHOOT_LEFT_MOTOR_ID:
				shoot_can_rx[0].angle = (int16_t) rxData[0] << 8 | rxData[1];
				shoot_can_rx[0].speed = (int16_t) rxData[2] << 8 | rxData[3];
				rc_shoot.left_fric.actual_speed = shoot_can_rx[0].speed ;
				break;
			case SHOOT_RIGH_MOTOR_ID:
				shoot_can_rx[1].angle = (int16_t) rxData[0] << 8 | rxData[1];
				shoot_can_rx[1].speed = (int16_t) rxData[2] << 8 | rxData[3];
				rc_shoot.right_fric.actual_speed = shoot_can_rx[1].speed;
				break;
			case LK_Pitch_Motor_ID:
				for(int i = 0; i < 8; i++)
					LK_Pitch_Motor_Receive_Data[i] = rxData[i];
				if (rxData[0] == Get_MultiRound_Angle_ID)
				{
					Motor_Alive_Flag++;
					LK_Pitch_Motor.SingleRound_Angle =(int16_t) rxData[4] | rxData[5]<<8 | rxData[6]<<16 | rxData[7]<<24;
					//Total_Angle计算
					if ((int)LK_Pitch_Motor.SingleRound_Angle - (int)LK_Pitch_Motor.Last_SingleRound_Angle < -108000) LK_Pitch_Motor.Round_Cnt++;
					if ((int)LK_Pitch_Motor.SingleRound_Angle - (int)LK_Pitch_Motor.Last_SingleRound_Angle > 108000) LK_Pitch_Motor.Round_Cnt--;
					LK_Pitch_Motor.Total_Angle = LK_Pitch_Motor.Round_Cnt*216000 + (int)LK_Pitch_Motor.SingleRound_Angle - Gimbal_Encoder_Horizontal_Angle;
					LK_Pitch_Motor.Last_SingleRound_Angle = LK_Pitch_Motor.SingleRound_Angle;
				
					if (Gimbal_Calibration_Read_Angle_Flag) 
					{
						Gimbal_Encoder_Horizontal_Angle = LK_Pitch_Motor.Total_Angle;
						Gimbal_Calibration_Read_Angle_Flag = 0;
					}
					if (Gimbal_Calibration_Target_Times == Gimbal_Calibration_Times)
						LK_Pitch_Motor.Calibrated_Angle = LK_Pitch_Motor.Total_Angle - Gimbal_Encoder_Horizontal_Angle;
					LK_Pitch_Motor.Converted_Calibrated_Angle = (LK_Pitch_Motor.Calibrated_Angle/216000.f)*360.f;
					if (LK_Pitch_Motor.Converted_Calibrated_Angle > 200.f) LK_Pitch_Motor.Converted_Calibrated_Angle = LK_Pitch_Motor.Converted_Calibrated_Angle - 360.f;
					
				}
					
				if (rxData[0] == 0x9A)
					LK_Pitch_Motor.Tempreture = rxData[1];
				break;
        }
    }
    else if(hcan == &hcan2)
    {
        HAL_CAN_GetRxMessage(&hcan2,  CAN_RX_FIFO0, &rxFrame,  rxData);
        switch (rxFrame.StdId)
        {
			//Yaw电机数据接收
			case YAW_ID:
				yaw_can_rx.lastangle = yaw_can_rx.angle;
				yaw_can_rx.angle = (int16_t)((rxData[0]<<8)|rxData[1]);
				yaw_can_rx.speed = (int16_t)((rxData[2]<<8)|rxData[3]);
				Yaw_Angle_Process();
			break;
        }
    }
}

float Temp_Total_Angle;
void Yaw_Angle_Process(void)
{
	if (yaw_can_rx.angle - yaw_can_rx.lastangle < -4096) yaw_can_rx.turns++;
	if (yaw_can_rx.angle - yaw_can_rx.lastangle >  4096) yaw_can_rx.turns--;
	yaw_can_rx.Total_Angle = (float)yaw_can_rx.turns * 8192 + yaw_can_rx.angle;//得到编码器角度的Total_Angle
	float temp;
	temp = gimbal_y.CAN_Total_Angle;
	while (temp > 16384.f) temp =temp - 16384.f; while (temp <0) temp = temp + 16384.f;
	gimbal_y.CAN_actual_angle=temp*360.0f/16384.0f;
	gimbal_y.CAN_Total_Angle = yaw_can_rx.Total_Angle*360.0f/16384.0f;
}
