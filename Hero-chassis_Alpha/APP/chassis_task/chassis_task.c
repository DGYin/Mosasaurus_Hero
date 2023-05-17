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
//int Briter_Encoder_Enable_Quest[]={1, 0, 0, 0}

static int MS_Count=1;
static int S_Count=0;

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
	MS_Count++;
	if (MS_Count%3==0)
	{
		int temp;
		temp = chassis_center.actual_angle*100;
	}
	if(MS_Count%10==4)
	{
		remote_control(); 				//ң�����ݽ���
		chassis_move();   				//�����ƶ�����
		referee_unpack_fifo_data();	//����ϵͳ���ݽ���
	}
	if(MS_Count%10==5)
	{
		//UartTX_To_BetaBoard_Yaw_Control(Yaw_Control_Current_Mode, gimbal_y.given_current);//yaw����ת����
	}	
	if(MS_Count%10==0)
	{
		send_gimbal_data_2();
	}	

	if (MS_Count%10==0)
	{
		Get_Encoder_Position(1);
	}
	if (MS_Count%10==1)
	{
		Get_Encoder_Position(2);
	}
	if (MS_Count%10==2)
	{
		Get_Encoder_Position(3);
	}	
	if (MS_Count%10==3)
	{
		Get_Encoder_Position(4);
	} 	
	if(MS_Count%400==0)
		UI_Display();
	if (MS_Count % 10 == 0)
		supercap();
	if(MS_Count>=1000)			//���������־    1s
	{
		MS_Count=1;
		S_Count++;
	}
}

