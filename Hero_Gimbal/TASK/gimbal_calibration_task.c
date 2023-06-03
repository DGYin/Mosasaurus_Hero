#include "gimbal_task.h"
#include "lk_pitch_turn.h"
#include "bsp_imu.h"
#include "gimbal_calibration_task.h"

//���ⲿ���õı���
int Gimbal_Calibration_Read_Angle_Flag = 0;
int Gimbal_Encoder_Horizontal_Angle = 0;
int Gimbal_Calibration_Times = 0;
int Gimbal_Calibration_Target_Times = 1;
int Motor_Alive_Flag = 0;
//������.c�ļ��е�ȫ�ֱ���
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
			if (Motor_Alive_Flag <=  0) Gimbal_Calibration_Start_Time = Global_Time; //���δ�ϵ磬�����ȴ�
			if (Global_Time-Gimbal_Calibration_Start_Time<4000) //3sʱ�乩��̨�ص�ˮƽ�Ƕ�
			{
				//IMU���ݻ�ȡ
				gimbal_p.IMU_actual_angle = 1.0f * INS_angle[1] / (2 * 3.141590f) * 360.0f;
				gimbal_p.IMU_actual_speed = -1.0f * INS_gyro[0] + 0.0069813f; ///(2*3.141590f)*360.0f +0.4f;
				//ʹ��IMU���ݽ����˶�
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
				Gimbal_Calibration_Start_Flag = 0;//���У׼����־λ����
				Gimbal_Calibration_Times++;
			}
		}
	}
}
