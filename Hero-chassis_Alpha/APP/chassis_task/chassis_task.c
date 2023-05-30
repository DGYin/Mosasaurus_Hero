/**
  ******************************************************************************
  * @file    Project/APP/chassis_task.c 
  * @author  NEUQRM
  * @version V1.0.0
  * @date    12.2021
  * @brief   ���ļ���ս������������
  ******************************************************************************
  * @attention
  ******************************************************************************
  ****************************(C) COPYRIGHT 2021 NEUQRM****************************

  ...........  ������δ�ɹ���ͬ־����Ŭ��  ...........
*/

#include "chassis_task.h"
#include "bsp_can.h"
#include "supercap.h"
#include "referee_UI.h"
#include "bsp_uart.h"
#include "relay_task.h"

int MS_Count=0;
int S_Count=0;

void Time_Service_Task(void);
/***********************************************************
*@fuction	:CHASSIS_TASK
*@brief		:
*@param		:--
*@return	:void
*@author	:--
*@date		:2023-04-04
***********************************************************/

void CHASSIS_TASK()//TIM3��ʱ���жϿ���ս�����Ѿ����궨�壩
{

	Time_Service_Task();
	if(MS_Count%10==4)
	{
		remote_control(); 				//ң�����ݽ���
		chassis_move();   				//�����ƶ�����
		referee_unpack_fifo_data();	//����ϵͳ���ݽ���
	}
	if(MS_Count%3==0)
	{
		referee_usart_task();
	}	
	Relay_Task(S_Count, MS_Count);	
	Briter_Encoder_Task(S_Count, MS_Count);	
	supercap(S_Count, MS_Count);

}

void Time_Service_Task(void)
{
	MS_Count++;
	if(MS_Count>999)			
	{
		MS_Count=0;
		S_Count++;
	}
}
