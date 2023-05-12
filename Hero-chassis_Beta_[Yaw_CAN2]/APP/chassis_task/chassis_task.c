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
#include "yaw_turn.h"

void CHASSIS_TASK()//TIM3��ʱ���жϿ���ս�����Ѿ����궨�壩
{
	static int time_count=1;
	time_count++;
//	if(time_count%3==0)
//		canTX_Gimbal_Yaw_Data(yaw_can_rx.angle, yaw_can_rx.speed); //ת��ԭʼ����
	if(time_count%10==5)
	{
		yaw_turn(); //yaw����ת����
	}	
	if(time_count%10==0)
	{		
		chassis_move();
		trigger_turn(); //��������ת����
	}	
	if(time_count%10==8)
	{
		supercap(); //��������ͨѶ
	}		
	if(time_count>=1000)			//���������־    1s
	{time_count=1;}
}

