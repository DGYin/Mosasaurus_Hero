#include "buzzer.h"
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


void Buzzer_Task(int S_Cnt, int MS_Cnt)
{
	Beep_Busy_Flag = 0;
	Power_On_Beep(S_Cnt, MS_Cnt);//开机提示音
	Pitch_Calibration_Beep(S_Cnt, MS_Cnt); //开机提示音后校准
	if (Beep_Busy_Flag == 0) //如果没有任务，默认关掉蜂鸣器
		buzzer_off();
	//Shoot_Beep(S_Cnt, MS_Cnt,Shoot_Flag_For_Buzzer);
	//Zero_Force_Beep(S_Cnt, MS_Cnt);
	
}

void Pitch_Calibration_Beep(int S_Cnt, int MS_Cnt)
{
	int psc;
	extern int Gimbal_Calibration_Times, Gimbal_Calibration_Target_Times, Motor_Alive_Flag;
	if (Gimbal_Calibration_Target_Times > Gimbal_Calibration_Times)
	{
		if (Pitch_Calibration_Priority > Beep_Now_Priority)
			Beep_Now_Priority = Pitch_Calibration_Priority;
		Beep_Busy_Flag = 1;
	}
	int Global_Time;
	Global_Time = S_Cnt*1000 + MS_Cnt;
	if (Beep_Now_Priority == Pitch_Calibration_Priority) //当前Pitch校准优先级最高，才控制发声
	{
		if (Motor_Alive_Flag>0)
		{
			if (Global_Time % 1000 == 0)
			{
				psc=4;
				buzzer_on(psc, pwm);
			}
			if (Global_Time % 1000 == 300)
			{
				psc=5;
				buzzer_on(psc, pwm);
			}
			if (Global_Time % 1000 == 700)
				buzzer_off();
		}
		else if (Motor_Alive_Flag<=0)
		{
			if (Global_Time % 1500 == 0)
			{
				psc=5;
				buzzer_on(psc, pwm);
			}
			if (Global_Time % 1500 == 300)
			{
				psc=6;
				buzzer_on(psc, pwm);
			}
			if (Global_Time % 1500 == 700)
				buzzer_off();
		}
	}
	if (Gimbal_Calibration_Target_Times == Gimbal_Calibration_Times) //如果蜂鸣器任务不再需要执行
		if (Beep_Now_Priority == Pitch_Calibration_Priority) //且当前正在执行该蜂鸣器任务
			Beep_Now_Priority = 0; //任务结束，优先级清零
}

int Shoot_Beep_Start_MS;
void Shoot_Beep(int S_Cnt, int MS_Cnt, int Shoot_Flag)
{
	int Total_MS;
	Total_MS = S_Cnt*1000 + MS_Cnt;
	if (Shoot_Flag)
	{
		Shoot_Beep_Start_MS = S_Cnt*1000 + MS_Cnt;
		int psc = 2;
		buzzer_on(psc, pwm);
	}
	if (Total_MS - Shoot_Beep_Start_MS == 400)
		buzzer_off();
}

void Zero_Force_Beep(int S_Cnt, int MS_Cnt)
{
	int psc = 3;
	int Start_Flag, End_Flag;
	int Total_MS, Start_MS, End_MS;
	Start_MS = 0; End_MS = 0;
	Start_Flag = 0; End_Flag = 0;
	Total_MS = S_Cnt*1000 + MS_Cnt;
	//当开启了无力模式时
	if (Last_Gimbal_Zero_Force_Flag==0 && Gimbal_Zero_Force_Flag==1) Start_MS = S_Cnt*1000 + MS_Cnt;
	if (Start_MS>0) 
	{
		psc = 3;
		buzzer_on(psc, pwm);
		Start_Flag = 1;
		
	}
	if (Total_MS - Start_MS == 400)
	{
		psc = 4;
		buzzer_on(psc, pwm);
		Start_Flag = 1;
	}
	if (Total_MS - Start_MS == 800)
	{
		psc = 4;
		buzzer_off();
		Start_Flag = 1;
	}
	//打开无力模式后
	if (((Total_MS - Start_MS)%1000 == 0) && (Start_Flag==0) && (End_Flag==0) && (Gimbal_Zero_Force_Flag))
	{
		psc = 4;
		buzzer_on(psc, pwm);
	}
	if (((Total_MS - Start_MS)%1000 == 300) && (Start_Flag==0) && (End_Flag==0) && (Gimbal_Zero_Force_Flag))
	{
		buzzer_off();
	}
	//关闭了无力模式时
	if (Last_Gimbal_Zero_Force_Flag==1 && Gimbal_Zero_Force_Flag==0) End_MS = S_Cnt*1000 + MS_Cnt;
	if (End_MS>0)
	{
		psc = 4;
		buzzer_on(psc, pwm);
		End_Flag = 1;
	}
	if (Total_MS - End_MS == 400)
	{
		psc = 3;
		buzzer_on(psc, pwm);
		End_Flag = 1;
	}
	if (Total_MS - End_MS == 800)
		buzzer_off();
}
	
void Power_On_Beep(int S_Cnt, int MS_Cnt)
{
	if (S_Cnt <=3) //只在前三秒进行
	{
		if (Power_On_Priority > Beep_Now_Priority)
		{
			Beep_Now_Priority = Power_On_Priority;
			Beep_Busy_Flag = 1;
		}
		if (Beep_Now_Priority == Power_On_Priority)
		{
			int psc;//调整音调
			if (S_Cnt == 0)
			{
				psc = 4;
				if (MS_Cnt==0)
				buzzer_on(psc, pwm);
				if (MS_Cnt==80)
				buzzer_off();
				
				psc = 3;
				if (MS_Cnt==160)
					buzzer_on(psc, pwm);
				if (MS_Cnt==240)
					buzzer_off();	
				
				psc = 3;
				if (MS_Cnt==320)
					buzzer_on(psc, pwm);
				if (MS_Cnt==400)
					buzzer_off();	
			}
			if (S_Cnt == 1)
			{
				psc = 4;
				if (MS_Cnt==0)
				buzzer_on(psc, pwm);
				if (MS_Cnt==80)
				buzzer_off();
				
				psc = 3;
				if (MS_Cnt==160)
					buzzer_on(psc, pwm);
				if (MS_Cnt==240)
					buzzer_off();		
			}	
			
			if (S_Cnt == 2)
			{
				psc = 4;
				if (MS_Cnt==0)
					buzzer_on(psc, pwm);
				if (MS_Cnt==100)
					buzzer_off();	
			}
			
			if (S_Cnt == 3)
			{
				psc = 4;
				if (MS_Cnt==0)
					buzzer_on(psc, pwm);
				psc = 3;
				if (MS_Cnt==400)
					buzzer_on(psc, pwm);
				
				if (MS_Cnt==800)
					buzzer_off();
			}
		}
	}
	if (S_Cnt == 4 && MS_Cnt == 0)
		Beep_Now_Priority = 0; //任务结束，优先级清零
}