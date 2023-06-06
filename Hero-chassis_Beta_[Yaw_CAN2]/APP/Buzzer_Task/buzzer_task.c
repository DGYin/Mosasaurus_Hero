#include "bsp_buzzer.h"
#include "buzzer_task.h"
//.c内部的全局变量
uint16_t pwm = 2700;	//调整音色
int Beep_Busy_Flag = 0; //如果有蜂鸣器任务为1，否则为0
int Gimbal_Zero_Force_Flag, Last_Gimbal_Zero_Force_Flag;
int Shoot_Flag_For_Buzzer;
int Beep_Now_Priority = 0; //用于区分任务优先级，数值越大优先级越高
//函数声明顺序重定义
void Power_On_Beep(int S_Cnt, int MS_Cnt);
void Zero_Force_Beep(int S_Cnt, int MS_Cnt);
void Shoot_Beep(int S_Cnt, int MS_Cnt, int Shoot_Flag);
void Pitch_Calibration_Beep(int S_Cnt, int MS_Cnt);
void Pitch_Motor_Offline_Beep(int S_Cnt, int MS_Cnt);


void DJI_Beep(int S_Cnt, int MS_Cnt)
{
	Beep_Busy_Flag = 1;
	int Global_Time;
	Global_Time = S_Cnt*1000 + MS_Cnt;
	int psc;
	if (Global_Time==100)
	{
		psc=8;
		buzzer_on(psc, pwm);
	}
	if (Global_Time==350)
	{
		psc=7;
		buzzer_on(psc, pwm);
	}
	if (Global_Time==600)
	{
		psc=5;
		buzzer_on(psc, pwm);
	}

	if (Global_Time==900)
			buzzer_off();
}
void Buzzer_Task(int S_Cnt, int MS_Cnt)
{
	Beep_Busy_Flag = 0;
	DJI_Beep(S_Cnt, MS_Cnt);
	if (Beep_Busy_Flag == 0) //如果没有任务，默认关掉蜂鸣器
		buzzer_off();
	//Shoot_Beep(S_Cnt, MS_Cnt,Shoot_Flag_For_Buzzer);
	//Zero_Force_Beep(S_Cnt, MS_Cnt);
	
}
