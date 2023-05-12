/**
  ******************************************************************************
  * @file    blueteeth.c 
  * @author  NEUQRM
  * @version V1.0.0
  * @date    4.2022
  * @brief   ���������ļ�
  ******************************************************************************
  * @attention
  ******************************************************************************
  ****************************(C) COPYRIGHT 2021 NEUQRM****************************

  ...........  ������δ�ɹ���ͬ־����Ŭ��  ...........
*/  
#include "blueteeth.h"
#include <stdio.h>
#include "usart.h"
#include "gimbal_task.h"
#include "vision_task.h"


//����ʹ�ô���6������PC6
char bt_imu_yaw_angle[25];
char bt_imu_pitch_angle[25];
char bt_mode[20];

char bt_gimbal_mode;
char bt_vision_mode;
/**
	* @brief       �ڲ�������������ö����������ת���ɶ�Ӧ�ַ��������Ķ�
	* @param				none
	* @retvel      none
*/
void num_to_char(void)
{
	switch(vision_mode)
	{
		case 0:
		bt_vision_mode='T';
		break;
		case 1:
		bt_vision_mode='F';
		break;
		default:break;
	}
	switch(gimbal_set_mode)
	{
		case 2:
			bt_gimbal_mode='S';
		break;
		case 3:
			bt_gimbal_mode='T';
		break;
		case 5:
			bt_gimbal_mode='Z';
		break;
		default:break;
	}
}
/**
	* @brief       �������亯�����������������ڴ����ַ���
	* @param				none
	* @retvel      none
*/
void blueteeth_trans(void)
{
		num_to_char();
	
		sprintf(bt_imu_yaw_angle,"yaw act:%.2f tar:%.2f",gimbal_y.IMU_actual_angle,gimbal_y.target_angle);
		sprintf(bt_imu_pitch_angle,"pit act:%.2f tar:%.2f",gimbal_p.IMU_actual_angle,gimbal_p.target_angle);
		sprintf(bt_mode,"vis:%c gim:%c",bt_vision_mode,bt_gimbal_mode);

		
    HAL_UART_Transmit(&huart6, (uint8_t *)bt_imu_yaw_angle, 25, 100);
		HAL_UART_Transmit(&huart6, (uint8_t *)bt_imu_pitch_angle, 25, 100);
		HAL_UART_Transmit(&huart6, (uint8_t *)bt_mode, 20, 100);

}


