/**
  ******************************************************************************
  * @file    Project/APP/chassis_task.c 
  * @author  NEUQRM
  * @version V1.0.0
  * @date    12.2021
  * @brief   该文件是战车输出任务汇总
  ******************************************************************************
  * @attention
  ******************************************************************************
  ****************************(C) COPYRIGHT 2021 NEUQRM****************************

  ...........  革命尚未成功，同志仍需努力  ...........
*/

#include "chassis_task.h"
#include "bsp_can.h"
#include "supercap.h"
#include "referee_UI.h"
#include "bsp_uart.h"
#include "yaw_turn.h"
#include "buzzer_task.h"

int S_Count=0, MS_Count=0;
void CHASSIS_TASK()//TIM3定时器中断控制战车（已经过宏定义）
{
	MS_Count++;
	if(MS_Count==1000)			//清除计数标志    1s
	{
		MS_Count=0;
		S_Count++;
	}
	Buzzer_Task(S_Count, MS_Count);
//	if(time_count%3==0)
//		canTX_Gimbal_Yaw_Data(yaw_can_rx.angle, yaw_can_rx.speed); //转发原始数据
	if(MS_Count%10==5)
	{
		yaw_turn(); //yaw轴旋转任务
	}	
	if(MS_Count%10==0)
	{		
		chassis_move();
		trigger_turn(); //拨弹轮旋转任务
	}	
	if(MS_Count%10==8)
	{
		supercap(); //超级电容通讯
	}		
	
}

