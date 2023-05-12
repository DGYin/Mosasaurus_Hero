/**
  ******************************************************************************
  * @file    Project/APP/trigger_turn.c 
  * @author  NEUQRM
  * @version V1.0.0
  * @date    12.2021
  * @brief   ���ļ������˲����ֵ�pid����ֵ����ֵ��pid���㣬�����һϵ�и��������йصĺ���
  ******************************************************************************
  * @attention
  ******************************************************************************
  ****************************(C) COPYRIGHT 2021 NEUQRM****************************

  ...........  ������δ�ɹ���ͬ־����Ŭ��  ...........
*/

#include "trigger_turn.h"



trigger_t trigger;

float TriggerSpeed[5] 	= {5.0f,			0.05f,			0.0f,		30000,			1500};  //�������ٶȻ�
float TriggerAngle[5] 	= {0.15f,			0.0f,			0.000f,		3000,			0.0f};  //�����ֽǶȻ�

//���·��ӵ�����
void shoot_angle_clc(void);
//������PID����
static void trigger_pid(void);
//�����ָ���yaw������
void trigger_turn(void)
{
	shoot_angle_clc();    //�ڽ�����̨��Ӧָ��ʱִ�иú��������ﲻִ�У�
	if(shoot_flag>=1)
	{
		shoot_flag++;
	}
	if(shoot_flag>=150)
	{
	   trigger.Target_Angle-=((17.5*360/7.f)*8191/360);
	   shoot_flag=0;
	}
	trigger_pid();
	canTX_trigger(trigger.set_currunt);
}

void trigger_all_init(void)//������pid������״̬��ʼ��
{
	PID_Init(&trigger.speed_pid,TriggerSpeed);
	PID_Init(&trigger.angle_pid,TriggerAngle);
	
	trigger.actual_angle = 0.0f;
	trigger.actual_speed = 0.0f;
	
	trigger.last_shoot_flag = 0;
	
	trigger.Target_Angle = 13.0f;
	trigger.Target_Speed = 0.0f;
}
float Target_Angle;
void shoot_angle_clc(void)
{
	
	if(trigger.last_shoot_flag!=shoot_flag)
	{
		if(trigger.rounds>=400)
		{
				trigger.Target_Angle-=(trigger.rounds-3)*8192;
				if(trigger.Target_Angle<=0) trigger.Target_Angle=1;
				trigger.Total_Angle -=(trigger.rounds-3)*8192 ;
				trigger.begin_angle=trigger.Total_Angle;
				trigger.rounds=3;
		}
		Target_Angle=((SHOOT_NUM*19.2*360/6.f)*8191/360);          //����1��
//		Target_Angle+=((9.5*360/7.f)*8191/360);
		trigger.Target_Angle-=Target_Angle;
	}
	trigger.last_shoot_flag=shoot_flag;
}

int16_t sped;
static void trigger_pid(void)//������pid��ֵ����
{
	apid_vpid_realize(&trigger.angle_pid,trigger.Target_Angle,trigger.Total_Angle);

	trigger.Target_Speed = trigger.angle_pid.out;
//	
	apid_vpid_realize(&trigger.speed_pid,trigger.Target_Speed,trigger.actual_speed);
	
	trigger.set_currunt = trigger.speed_pid.out;
}


uint8_t canTX_trigger(int16_t trigger)//�����ֵ������
{
	CAN_TxHeaderTypeDef canFrame;
	uint8_t data[8]={0};
	uint32_t temp=0;
	
	canFrame.IDE=CAN_ID_STD;
	canFrame.StdId=0x1ff;
	canFrame.RTR=CAN_RTR_DATA;
	canFrame.DLC=8;
	canFrame.TransmitGlobalTime=DISABLE;
	data[0]=0;
	data[1]=0;
	data[2]=0;
	data[3]=0;
	data[4]=0;
	data[5]=0;
	data[6]=trigger>>8;
	data[7]=trigger&0xff;
	HAL_CAN_AddTxMessage(&hcan1, &canFrame, data,&temp);
	
	return temp;
}

void Trigger_Motor_Callback(trigger_t *motor,uint16_t angle, int16_t speed)//�����ֵ���ص�
{
	motor->last_angle = motor->actual_angle;
	motor->actual_angle = angle;
	motor->actual_speed = 0.5*(speed + motor->last_speed);
	motor->last_speed = speed;

	if(motor->record_begin_angle_status==0)
	{
		motor->begin_angle = angle;
		motor->record_begin_angle_status++;
	}
	if(motor->actual_angle - motor->last_angle > 4096)
		motor->rounds --;
	else if (motor->actual_angle - motor->last_angle < -4096)
		motor->rounds ++;
	motor->Total_Angle = motor->rounds * 8192 + motor->actual_angle;

	
}
