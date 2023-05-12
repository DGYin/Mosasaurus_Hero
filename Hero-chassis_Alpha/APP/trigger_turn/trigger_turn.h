#ifndef __TRIGGER_TURN_H
#define __TRIGGER_TURN_H

#include "pid.h"
#include "remote_control.h"
#include "stm32f4xx.h"
#include "yaw_turn.h"


#define SHOOT_NUM 1          //����1��

typedef struct{
	
	float Target_Speed;
	float actual_speed;
	
	float Target_Angle;
	float actual_angle;
	
	int last_shoot_flag;
	
	float set_currunt;
	
	float last_angle;			//��һ�λ�еת�ӽǶ�
	int rounds;				    //ת����Ȧ��
	int Total_Angle;			//�ܹ�ת���ĽǶ�
	int last_speed;       		//��һ����ʵת��
	int record_begin_angle_status;//�Ƿ��¼�˵����ʼ�Ƕ� 0����û�м�¼��1�����¼�ɹ�
	int begin_angle;            //�����ʼ�Ƕ�
	
	PidTypeDef speed_pid;
	PidTypeDef angle_pid;
}trigger_t;

extern trigger_t trigger;
void trigger_turn(void);
void trigger_all_init(void);
uint8_t canTX_trigger(int16_t trigger);
void Trigger_Motor_Callback(trigger_t *motor,uint16_t angle, int16_t speed);
void shoot_angle_clc(void);
#endif
