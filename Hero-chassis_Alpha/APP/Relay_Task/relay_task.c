#include "relay_task.h"
#include "gpio.h"

/***********************************************************
*@Brief	���ⲿ���õı���
***********************************************************/
int Relay_Set_State = Supercap_Power_Supply;

/***********************************************************
*@Brief	relay_task.c�ڲ��ı���
***********************************************************/
int Relay_State = Supercap_Power_Supply;

/***********************************************************
*@Brief	relay_task.c����˳���ض���
***********************************************************/
void Supercap_Power_Mode(void);
void Battery_Power_Mode(void);

void Relay_Task(int S_Cnt, int MS_Cnt)
{
	//ʱ�����
	int Global_Time;
	Global_Time = S_Cnt*1000 + MS_Cnt;
	
	//�̵������ƣ�100ms����һ��
	if (Global_Time%300 == 0)  //ȷ���̵���ģʽ�л��������300ms����ֹ��
	{
		Relay_State = Relay_Set_State;
		switch(Relay_State)
		{
			case Supercap_Power_Supply:
				Supercap_Power_Mode();
				break;
			case Direct_Power_Supply:
				Battery_Power_Mode();
				break;
		}
	}
}
	
void Supercap_Power_Mode(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
}

void Battery_Power_Mode(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
}