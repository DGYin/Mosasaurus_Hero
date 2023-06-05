/**
  ************************************* Copyright ******************************   
  *                 (C) Copyright 2022,Hebei,China, NEUQRM.
  *                            All Rights Reserved
  *                              
  *                     By DGYin,
  *                     https://--
  *      
  * FileName   : chassis_move.h
  * Version    : v1.1.0-alpha.1
  * Author     : NEUQRM, DGYin
  * Date       : 2022-11-07         
  * Description: ͷ�ļ�   
  ******************************************************************************
**/
#ifndef __MOTOR_H
#define __MOTOR_H

#include "pid.h"

#include "can.h"
#include <math.h>
#include "referee.h"


/*******************************************************************************  
                                  ���������õ�������
*******************************************************************************/
#define Mecanum_Wheel_Chassis 0	//���ֵ���
#define AGV_Chassis						1	//���ֵ���
#define Chassis_Type AGV_Chassis

//Ҳ������ֱ̨�Ӵ���ָ��
#define CHASSIS_REMOTE_CLOSE		1	//�ر�ң����                                ���λ
#define CHASSIS_NORMAL				3	//����ģʽ                                  ����λ
#define CHASSIS_SPIN				2	//С����ģʽ                                    ���λ

#define Chassis_Follow_ON		1
#define Chassis_Follow_OFF		0

#define GIMBAL_HEAD_ANGLE  -30

#define POWER_LIMIT         80.0f
#define WARNING_POWER       40.0f   
#define WARNING_POWER_BUFF  50.0f  

/*Motor_Power_Prosumption��Motor_Power_Limitation������صĺ궨��*/
#define Tau_Coefficient 0.3 //�����3508�ֲ� https://rm-static.djicdn.com/tem/17348/RoboMaster%20M3508%E7%9B%B4%E6%B5%81%E6%97%A0%E5%88%B7%E5%87%8F%E9%80%9F%E7%94%B5%E6%9C%BA%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8EV1.0%EF%BC%88%E4%B8%AD%E8%8B%B1%E6%97%A5%EF%BC%89.pdf
#define Power_K1 0
#define Power_K2 0
#define Power_Prosumption_Converting_Coefficient 0

/*Target_Velocity_Smoothen������صĺ궨��*/
#define Uniform_Acceleration 0 //�ȼ���ģʽ������ƽ���ٶ�����
#define Acceleration 0 //���ٶȣ�����ƽ���ٶ����ߵ��ȼ���ģʽ

#define Directive_Motor_Max_Err_Angle 80000

#define follow_pid_realize() \
				do{ \
						switch_flag=FOLLOW;      \
						pid_realize(&(chassis_center.pid));   \
					  switch_flag=NUL;  \
				}while(0)
				
#define vpid_chassis_realize() \
				do{ \
						switch_flag=CHASSIS;      \
						pid_realize(&(Chassis_Motor1.pid));   \
						pid_realize(&(Chassis_Motor2.pid));   \
						pid_realize(&(Chassis_Motor3.pid));   \
						pid_realize(&(Chassis_Motor4.pid));   \
					  switch_flag=NUL;  \
				}while(0)
				

			
typedef struct{
	
	float start_angle;			//�����ʼ�Ƕ�ֵ
	int Zero_Position;
	int start_angle_flag;	//��¼�����ʼ�Ƕ�ֵ��flag
	int switch_mode_flag;  //��¼ģʽת���Ƕ�ֵ��flag
	int stop_angle;				//����ֹͣ����ʱ��ĽǶ�ֵ
	float Last_Target_Angle;
	float Target_Angle;
	float Find_Min;
	
	int Start_Angle;
	float actual_angle;			//��ǰ��ʵ�Ƕ�ֵ
	float last_angle;				//��һ�η��صĽǶ�ֵ
	float switch_mode_angle;  //��¼ģʽת���Ƕ�ֵ
	int round_cnt;				//��Կ���ʱת����Ȧ��
	int Total_Angle;			//�ܹ�ת���ļ���
	int Last_Total_Angle;
	float ChassisCoordinate_Angle;
	float AGV_Heading_Angle; //
	
	float actual_speed;			//�����ʵ�ٶ�,rpm
	int Target_Speed;			//���Ŀ���ٶ�,rpm  ת/min
	int last_speed;       //�����һ�λش����ٶ�ֵ
	int actual_current;		//�����ʵ����
	int target_current;		//���Ŀ�����
	float Actual_Torque;
	float Target_Torque;
	//int temp;							//����¶ȣ�2006�����֧�֣�3508֧�֣�
	
	int Invert_Flag;
	float Power_Prosumption;
	
	PID_t pid;
}MOTOR_t;

typedef struct
{
	int16_t vx_set;
	int16_t vy_set;
	int16_t wz_set;
	uint8_t chassis_mode;
	int Precision_Mode;
	uint8_t last_chassis_mode;
	uint16_t gimbal_6020_angle;
}CHASSIS_CONTROL_ORDER_t;

typedef struct
{
	int16_t real_vx;
	int16_t real_vy;
}REAl_CHASSIS_SPEED_t;

typedef enum
{
	NO_STEP=0,
	X_STEP,
	Y_STEP,	
	XY_STEP,
}STEPSTAR;


extern MOTOR_t Chassis_Motor1,Chassis_Motor2,Chassis_Motor3,Chassis_Motor4,chassis_center;
extern MOTOR_t Chassis_MotorA,Chassis_MotorB,Chassis_MotorC,Chassis_MotorD;
extern CHASSIS_CONTROL_ORDER_t chassis_control_order;
extern POWER_PID_t p_pid;
extern BUFFER_PID_t b_pid;
extern int shoot_flag;
extern int Chassis_Follow_Switch;

float Square(float Input);
void vpid_chassis_realize_F(void);
void chassis_move(void);
int All_Directive_Motor_Angle_Ready(void);
#endif

