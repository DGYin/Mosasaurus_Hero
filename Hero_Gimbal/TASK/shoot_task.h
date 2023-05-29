#ifndef __SHOOT_TASK_H
#define __SHOOT_TASK_H

#include "pid.h"

#define SHOOT_NUM 1

#define SHOOT_FRIC_HIGH_SPEED 5700	//����Ħ���֣�Ч���š����ñ��������16m/s
//#define SHOOT_FRIC_HIGH_SPEED 5900 //����ѵ�������16m/s
#define SHOOT_FRIC_LOW_SPEED 1600
#define SHOOT_FRIC_MIDDLE_SPEED 1600



#define SHOOT_ERROR 3

typedef struct{
	
	float target_speed;
	float actual_speed;
	
	float target_angle;
	float actual_angle;
	
	int last_shoot_flag;
	
	float set_currunt;
	
	float last_angle;			//��һ�λ�еת�ӽǶ�
	int rounds;				    //ת����Ȧ��
	int total_angle;			//�ܹ�ת���ĽǶ�
	int last_speed;       		//��һ����ʵת��
	int record_begin_angle_status;//�Ƿ��¼�˵����ʼ�Ƕ� 0����û�м�¼��1�����¼�ɹ�
	int begin_angle;            //�����ʼ�Ƕ�
	
	PidTypeDef speed_pid;
	PidTypeDef angle_pid;
}trigger_t;

typedef struct{
	
	float target_speed;
	float actual_speed;
	
	float set_currunt;
	

	PidTypeDef speed_pid;
}fric_t;


typedef struct{
	
	fric_t left_fric;
	fric_t right_fric;
	
	trigger_t trigger;
}shoot_task_t;

void shoot_init(void);
void shoot_task(void);
void Trigger_Motor_Callback(trigger_t *motor,uint16_t angle, int16_t speed);
extern shoot_task_t rc_shoot;

#endif
