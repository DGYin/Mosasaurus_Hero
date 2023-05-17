#include <cstdlib>
#include "chassis_task.h"
#include "sent_task.h"
#include "remote_control.h"
#include "gimbal_task.h"
#include "can_receive.h"
#include "mode_control.h"
/*
	* @ brief       ���Ƶ���
	* @ param				none
	* @ retvel      none
*/
extern int Gimbal_Precision_Mode;
void remote_chassis(void)
{
	if (rc_sent.r_speed > 2) rc_sent.r_speed =2; if (rc_sent.r_speed < -2) rc_sent.r_speed = -2;
	rc_sent.r_speed = rc_sent.r_speed*10.0f;
	rc_sent.x_speed = rc_sent.x_speed*gimbal_y.Valuence_Invert_Flag;
	rc_sent.y_speed = rc_sent.y_speed*gimbal_y.Valuence_Invert_Flag;
	if (!Gimbal_Precision_Mode)
	{
		if(gimbal_set_mode == GIMBAL_ZERO_FORCE || gimbal_set_mode == GIMBAL_RELATIVE_ANGLE)//����
		{
			canTX_chassis(rc_sent.x_speed, rc_sent.y_speed, 0, 0);
			canTX_mode(CHASSIS_REMOTE_CLOSE);
		}
		else if(gimbal_set_mode == GIMBAL_TOP_ANGLE)//С����
		{
			canTX_chassis(rc_sent.x_speed, rc_sent.y_speed, rc_sent.r_speed, 0);
			canTX_mode(CHASSIS_SPIN);
		}
		else if(gimbal_set_mode == GIMBAL_ABSOLUTE_ANGLE)//����
		{
			canTX_chassis(rc_sent.x_speed, rc_sent.y_speed, rc_sent.r_speed, 0);
			canTX_mode(CHASSIS_NORMAL);
		}
	}
	else//����ģʽ
	{
		canTX_chassis(0, 0, 0, 0);
		canTX_mode(CHASSIS_REMOTE_CLOSE);
	}
	canTX_Invert_Flag(gimbal_y.Bool_Invert_Flag);
}
