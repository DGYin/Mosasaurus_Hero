/**
  ******************************************************************************
  * @file    Project/APP/vision.c 
  * @author  NEUQRM
  * @version V1.0.0
  * @date    12.2021
  * @brief   �Ƿ�����Ӿ�ģʽ����Ϊ����Ҫ������̨�����ݣ����������λ����ֱ�ӽӵ������ϸ��ļ��Ϳ��Ժ��ԣ�
  ******************************************************************************
  * @attention
  ******************************************************************************
  ****************************(C) COPYRIGHT 2021 NEUQRM****************************

  ...........  ������δ�ɹ���ͬ־����Ŭ��  ...........
*/

#include "vision.h"

VISION_t vision_mode=VISION_OFF;
VISION_GET_t vision_sent;

//void Vision_Task(void)
//{
//	if(vision_mode==VISION_OFF)
//	{
//		gimbal_y.add_angle=rc_sent.yaw.Target_Angle;
//		gimbal_y.Target_Speed=rc_sent.yaw.Target_Speed*0.5f;
//	}
//	else
//	{
//		gimbal_y.add_angle=vision_sent.yaw.Target_Angle;
//	}
//}

