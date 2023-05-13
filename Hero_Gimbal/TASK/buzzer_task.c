#include "buzzer.h"
#include "buzzer_task.h"

void Power_On_Beep(int S_Cnt, int MS_Cnt);
void Zero_Force_Beep(int S_Cnt, int MS_Cnt);
void Shoot_Beep(int S_Cnt, int MS_Cnt, int Shoot_Flag);

uint16_t pwm = 2700;	//调整音色
int Gimbal_Zero_Force_Flag, Last_Gimbal_Zero_Force_Flag;
int Shoot_Flag_For_Buzzer;
void Buzzer_Task(int S_Cnt, int MS_Cnt)
{
	if (S_Cnt<=3) Power_On_Beep(S_Cnt, MS_Cnt);//开机提示音
	//Shoot_Beep(S_Cnt, MS_Cnt,Shoot_Flag_For_Buzzer);
	//Zero_Force_Beep(S_Cnt, MS_Cnt);
	
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
	int psc;//调整音调
	if (S_Cnt == 0)
	{
		psc = 4;
		if (MS_Cnt==0)
		buzzer_on(psc, pwm);
		if (MS_Cnt==100)
		buzzer_off();
		
		psc = 3;
		if (MS_Cnt==200)
			buzzer_on(psc, pwm);
		if (MS_Cnt==300)
			buzzer_off();	
		
		psc = 3;
		if (MS_Cnt==400)
			buzzer_on(psc, pwm);
		if (MS_Cnt==500)
			buzzer_off();	
	}
	if (S_Cnt == 1)
	{
		psc = 4;
		if (MS_Cnt==0)
		buzzer_on(psc, pwm);
		if (MS_Cnt==100)
		buzzer_off();
		
		psc = 3;
		if (MS_Cnt==200)
			buzzer_on(psc, pwm);
		if (MS_Cnt==300)
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