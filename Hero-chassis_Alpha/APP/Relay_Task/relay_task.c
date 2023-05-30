#include "relay_task.h"
#include "gpio.h"

int Relay_Set_State;


	
void Relay_Task(int S_Cnt, int MS_Cnt)
{
	int Global_Time;
	Global_Time = S_Cnt*1000 + MS_Cnt;
	if (Global_Time%2000==0)
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
	if (Global_Time%2000==1000)
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
}
	