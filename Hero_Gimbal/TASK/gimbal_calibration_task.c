#include "gimbal_task.h"
#include "lk_pitch_turn.h"
#include "bsp_imu.h"
#include "gimbal_calibration_task.h"

//共外部调用的变量
int Gimbal_Calibration_Read_Angle_Flag = 0;
int Gimbal_Encoder_Horizontal_Angle = 0;
int Gimbal_Calibration_Times = 0;
int Gimbal_Calibration_Target_Times = 1;
int Motor_Alive_Flag = 0;
//仅用于.c文件中的全局变量
int Gimbal_Calibration_Start_Time = 0, Gimbal_Calibration_Start_Flag = 0;

void Gimbal_Calibration_Task(int S_Cnt, int MS_Cnt)
{
	if (Gimbal_Calibration_Target_Times > Gimbal_Calibration_Times)
	{
		int Global_Time;
		Global_Time = S_Cnt*1000 + MS_Cnt;
		if (Gimbal_Calibration_Start_Flag == 0)
		{
			Gimbal_Calibration_Start_Time = Global_Time;
			Gimbal_Calibration_Start_Flag = 1;
		}
		else 
		{
			if (Motor_Alive_Flag <=  0) Gimbal_Calibration_Start_Time = Global_Time; //电机未上电，继续等待
			if (Global_Time-Gimbal_Calibration_Start_Time<4000) //3s时间供云台回到水平角度
			{
				//IMU数据获取
				gimbal_p.IMU_actual_angle = 1.0f * INS_angle[1] / (2 * 3.141590f) * 360.0f;
				gimbal_p.IMU_actual_speed = -1.0f * INS_gyro[0] + 0.0069813f; ///(2*3.141590f)*360.0f +0.4f;
				//使用IMU数据进行运动
				gimbal_p.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
				PID_Calc(&gimbal_p.gimbal_gyro_pid, 0, gimbal_p.IMU_actual_angle);
				Send_Pitch_Motor_Add_Angle(-gimbal_p.gimbal_gyro_pid.out);
			}
			if (Global_Time-Gimbal_Calibration_Start_Time==4000)
			{
				Gimbal_Calibration_Read_Angle_Flag = 1;
			}
			if (Global_Time-Gimbal_Calibration_Start_Time>4000)
			{	
				Gimbal_Calibration_Start_Flag = 0;//完成校准，标志位置零
				Gimbal_Calibration_Times++;
			}
		}
	}
}
