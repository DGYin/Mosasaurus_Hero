#include "buzzer.h"
#include "buzzer_task.h"
void Power_On_Beep(int S_Cnt, int MS_Cnt);

uint16_t pwm = 2700;	//调整音色

void Buzzer_Task(int S_Cnt, int MS_Cnt)
{
	Power_On_Beep(S_Cnt,MS_Cnt);//开机提示音
	
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