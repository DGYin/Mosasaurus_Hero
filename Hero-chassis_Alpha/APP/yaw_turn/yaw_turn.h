#ifndef __YAW_TURN_H
#define __YAW_TURN_H

#include "stm32f4xx.h"
#include "remote_control.h"
#include "pid.h"
#include "can_receive.h"

#define yaw_angle gimbal_y.add_angle
#define YAW_DEADLINE 0.3

#define CH_YAW_SPEED_MAXX 10
#define CH_YAW_SPEED_MINN -10
#define CH_YAW_ANGLE_MAXX 2
#define CH_YAW_ANGLE_MINN -2

typedef enum{
	GIMBAL_MOTOR_RAW= 0, //����������ٶȻ�����
    GIMBAL_MOTOR_GYRO,    //��������ǽǶȿ���
    GIMBAL_MOTOR_ENCONDE, //�������ֵ�Ƕȿ���
}MODE_t;

typedef struct{
	
	PidTypeDef gimbal_raw_pid;
	PidTypeDef gimbal_gyro_pid;
	PidTypeDef gimbal_enconde_pid_speed;//�ٶ�
	PidTypeDef gimbal_enconde_pid;//�Ƕ�
	//λ��ʽ
	float Target_Angle;
	float IMU_actual_angle;
	int Invert_Flag;
	   
	float Target_Speed;
	float IMU_actual_speed;
	
	float CAN_actual_speed;
	float CAN_actual_angle;
	float CAN_Total_Angle;
	
	float add_angle;
	
	int16_t given_current;
	
	MODE_t gimbal_motor_mode;
}GIMBAL_t;


float Float_Abs(float a);
extern float yaw_Target_Speed;
extern float yaw_Target_Angle;
float limits_change(int maxx,int minn,int a,int maxx_actual,int minn_actual);
void yaw_ch2_to_yaw_target(void);
extern GIMBAL_t gimbal_y;
extern int16_t yaw_ch2;
extern int16_t yaw_spin_current;
uint8_t canTX_yaw(int16_t yaw_current);
void yaw_turn(void);
void Yaw_PIDinit(void);

#endif
