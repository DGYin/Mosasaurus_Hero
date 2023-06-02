#include <cstdlib>
#include "chassis_task.h"
#include "sent_task.h"
#include "remote_control.h"
#include "gimbal_task.h"
#include "can_receive.h"
#include "mode_control.h"
#include "gimbal_calibration_task.h"
/*
	* @ brief       ���Ƶ���
	* @ param				none
	* @ retvel      none
*/
extern int Gimbal_Precision_Mode;

int Chassis_Mode, Chassis_Follow_Switch = Chassis_Follow_ON;
void remote_chassis(void)
{
	if (rc_sent.r_speed > 2) rc_sent.r_speed =2; if (rc_sent.r_speed < -2) rc_sent.r_speed = -2;
	rc_sent.r_speed = rc_sent.r_speed*10.0f;
	rc_sent.x_speed = rc_sent.x_speed*gimbal_y.Valuence_Invert_Flag;
	rc_sent.y_speed = rc_sent.y_speed*gimbal_y.Valuence_Invert_Flag;
	if (Gimbal_Precision_Mode == 0)
	{
		if (Gimbal_Calibration_Target_Times > Gimbal_Calibration_Times) //У׼ģʽ�����̲����˶�
		{
			canTX_chassis(rc_sent.x_speed, rc_sent.y_speed, 0, 0);
			CAN_Tx_Mode(CHASSIS_REMOTE_CLOSE, Gimbal_Precision_Mode, Chassis_Follow_Switch);
		}
		if(gimbal_set_mode == GIMBAL_ZERO_FORCE || gimbal_set_mode == GIMBAL_RELATIVE_ANGLE)//����
		{
			canTX_chassis(rc_sent.x_speed, rc_sent.y_speed, 0, 0);
			CAN_Tx_Mode(CHASSIS_REMOTE_CLOSE, Gimbal_Precision_Mode, Chassis_Follow_Switch);
		}
		else if(Chassis_Mode == CHASSIS_SPIN)//С����
		{
			canTX_chassis(rc_sent.x_speed, rc_sent.y_speed, 0, 0);
			CAN_Tx_Mode(CHASSIS_SPIN, Gimbal_Precision_Mode, Chassis_Follow_Switch);
		}
		else if(Chassis_Mode == CHASSIS_NORMAL)//����
		{
			canTX_chassis(rc_sent.x_speed, rc_sent.y_speed, rc_sent.r_speed, 0);
			CAN_Tx_Mode(CHASSIS_NORMAL, Gimbal_Precision_Mode, Chassis_Follow_Switch);
		}
	}
	else//����ģʽ
	{
		canTX_chassis(0, 0, 0, 0);
		CAN_Tx_Mode(CHASSIS_REMOTE_CLOSE, Gimbal_Precision_Mode, Chassis_Follow_Switch);
	}
	canTX_Invert_Flag(gimbal_y.Bool_Invert_Flag);
}
