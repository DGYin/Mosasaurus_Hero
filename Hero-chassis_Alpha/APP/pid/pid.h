#ifndef __PID_H
#define __PID_H

#include "stdint.h"
#include <stdlib.h>


//�ǶȻ��궨��
#define aPID_OUT_MAX          1800		//������ٶ�
#define aPID_OUT_MAX_CHASSIS          1000		//������ٶ�
#define aPID_OUT_MAX_FOLLOW          2		//������ٶ�
#define aP_OUT_MAX_FOLLOW          	2
#define FOLLOW_Integral_amax         200// 3000
#define FOLLOW_aIntegralSeparation    6//20

//�ٶȻ��궨��
#define CHASSIS_Integral_max         10000            //�����ֱ���
#define CHASSIS_IntegralSeparation   700             //���ַ���
//#define CHASSIS_vPID_max             6000            //����޷�

#define FOLLOW_Integral_max         500// 3000
#define FOLLOW_IntegralSeparation    15//20f
#define FOLLOW_vPID_max              100
#define AVERAGE                      10              //ƽ��������鳤�� 

//���ʻ��궨��
#define POWER_Integral_max         120// 3000
#define POWER_IntegralSeparation    14//20
#define PID_OUT_MAX_POWER          2000		//������ٶ�
#define P_OUT_MAX_POWER          	2000


#define P_OUT_MAX_BUFFER   1000
#define POSITION_LOOP     1
#define SPEED_LOOP     2

#define POSITION_LOOP     1
#define SPEED_LOOP     2

#define pid_init() \
				do{ \
					/*����λ�û�*/                        \
					chassis_center.pid.position_loop.pid_Parameter.Akp = 0.08; \
					chassis_center.pid.position_loop.pid_Parameter.Aki = 0; \
					chassis_center.pid.position_loop.pid_Parameter.Akd = 0; \
					chassis_center.pid.position_loop.pid_Parameter.Vkp = 1.1; \
					chassis_center.pid.position_loop.pid_Parameter.Vki = 0.1; \
					chassis_center.pid.position_loop.pid_Parameter.Vkd = 0.1; \
					                                     \
					/*�����ٶȻ�*/                        \
					chassis_center.pid.speed_loop.pid_Parameter.Vkp = 4; \
					chassis_center.pid.speed_loop.pid_Parameter.Vki = 0.1; \
					chassis_center.pid.speed_loop.pid_Parameter.Vkd = 0.1; \
					                   \
					/*�����ٶȻ�*/       \
					Chassis_Motor1.pid.speed_loop.pid_Parameter.Vkp = 0.76;   \
					Chassis_Motor2.pid.speed_loop.pid_Parameter.Vkp = 0.76;   \
					Chassis_Motor3.pid.speed_loop.pid_Parameter.Vkp = 0.76;   \
					Chassis_Motor4.pid.speed_loop.pid_Parameter.Vkp = 0.76;   \
					Chassis_Motor1.pid.speed_loop.pid_Parameter.Vki = 0.00265;   \
					Chassis_Motor2.pid.speed_loop.pid_Parameter.Vki = 0.00265;   \
					Chassis_Motor3.pid.speed_loop.pid_Parameter.Vki = 0.00265;   \
					Chassis_Motor4.pid.speed_loop.pid_Parameter.Vki = 0.00265;   \
					Chassis_Motor1.pid.speed_loop.pid_Parameter.Vkd = 0.1;   \
					Chassis_Motor2.pid.speed_loop.pid_Parameter.Vkd = 0.1;   \
					Chassis_Motor3.pid.speed_loop.pid_Parameter.Vkd = 0.1;   \
					Chassis_Motor4.pid.speed_loop.pid_Parameter.Vkd = 0.1;   \
                    \
                    Chassis_MotorA.pid.speed_loop.pid_Parameter.Vkp = 4;   \
					Chassis_MotorB.pid.speed_loop.pid_Parameter.Vkp = 4;   \
					Chassis_MotorC.pid.speed_loop.pid_Parameter.Vkp = 4;   \
					Chassis_MotorD.pid.speed_loop.pid_Parameter.Vkp = 4;   \
					Chassis_MotorA.pid.speed_loop.pid_Parameter.Vki = 0.6;   \
					Chassis_MotorB.pid.speed_loop.pid_Parameter.Vki = 0.6;   \
					Chassis_MotorC.pid.speed_loop.pid_Parameter.Vki = 0.6;   \
					Chassis_MotorD.pid.speed_loop.pid_Parameter.Vki = 0.6;   \
					Chassis_MotorA.pid.speed_loop.pid_Parameter.Vkd = 0;   \
					Chassis_MotorB.pid.speed_loop.pid_Parameter.Vkd = 0;   \
					Chassis_MotorC.pid.speed_loop.pid_Parameter.Vkd = 0;   \
					Chassis_MotorD.pid.speed_loop.pid_Parameter.Vkd = 0;   \
                    \
                    Chassis_MotorA.pid.position_loop.pid_Parameter.Akp = 0.08;   \
					Chassis_MotorB.pid.position_loop.pid_Parameter.Akp = 0.08;   \
					Chassis_MotorC.pid.position_loop.pid_Parameter.Akp = 0.08;   \
					Chassis_MotorD.pid.position_loop.pid_Parameter.Akp = 0.08;   \
					Chassis_MotorA.pid.position_loop.pid_Parameter.Aki = 0.01;   \
					Chassis_MotorB.pid.position_loop.pid_Parameter.Aki = 0.01;   \
					Chassis_MotorC.pid.position_loop.pid_Parameter.Aki = 0.01;   \
					Chassis_MotorD.pid.position_loop.pid_Parameter.Aki = 0.01;   \
					Chassis_MotorA.pid.position_loop.pid_Parameter.Akd = 0;   \
					Chassis_MotorB.pid.position_loop.pid_Parameter.Akd = 0;   \
					Chassis_MotorC.pid.position_loop.pid_Parameter.Akd = 0;   \
					Chassis_MotorD.pid.position_loop.pid_Parameter.Akd = 0;   \
                    Chassis_MotorA.pid.position_loop.pid_Parameter.Vkp = 15;   \
					Chassis_MotorB.pid.position_loop.pid_Parameter.Vkp = 15;   \
					Chassis_MotorC.pid.position_loop.pid_Parameter.Vkp = 15;   \
					Chassis_MotorD.pid.position_loop.pid_Parameter.Vkp = 15;   \
					Chassis_MotorA.pid.position_loop.pid_Parameter.Vki = 0.2;   \
					Chassis_MotorB.pid.position_loop.pid_Parameter.Vki = 0.2;   \
					Chassis_MotorC.pid.position_loop.pid_Parameter.Vki = 0.2;   \
					Chassis_MotorD.pid.position_loop.pid_Parameter.Vki = 0.2;   \
					Chassis_MotorA.pid.position_loop.pid_Parameter.Vkd = 0;   \
					Chassis_MotorB.pid.position_loop.pid_Parameter.Vkd = 0.0;   \
					Chassis_MotorC.pid.position_loop.pid_Parameter.Vkd = 0.0;   \
					Chassis_MotorD.pid.position_loop.pid_Parameter.Vkd = 0.0;   \
					/*����*/           \
					p_pid.kp=1.6; \
					p_pid.ki=0.75; \
					p_pid.kd=0; \
					/*����*/           \
					b_pid.kp=4.55; \
					b_pid.ki=2.0; \
					b_pid.kd=0; \
				}while(0)

				
typedef enum
{
	
	CHASSIS = 1,
	FOLLOW =2,
	NUL=0,
}switch_flag_t;


typedef struct{
	
	float err;
	float last_err;
	float err_integration;
	float actual_angle;
	float Target_Angle;
	float P_OUT;
	int16_t I_OUT;
	int16_t D_OUT;
	float PID_OUT;
	float last_actual_angle;
}APID_t;

typedef struct{
	int err;
	int last_err;
	int err_integration;
	int Target_Speed;
	int actual_speed;
	
	int P_OUT;
	int I_OUT;
	int D_OUT;
	int PID_OUT;
	
	int pid_count;           //pid���������ڼ���ƽ�����
	float average_err;           //ƽ����ʹ������߸���ƽ��
	float last_average_err;      //��һ��ƽ�����
}VPID_t;

typedef struct
{
    //PID ������
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //������
    float max_iout; //���������

    float ref;
    float fdb;
	float lastfdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    float error[3]; //����� 0���� 1��һ�� 2���ϴ�

}PidTypeDef;

typedef struct
{
	float Akp;			
	float Aki;	
  float Akd;	
	float Vkp;			
	float Vki;	
  float Vkd;	
}Parameter_t;

typedef struct
{
	APID_t apid;
	VPID_t vpid;
	Parameter_t pid_Parameter;
}PID_Loop_t;

typedef struct{
	uint8_t loop_flag;
	PID_Loop_t position_loop;
	PID_Loop_t speed_loop;
}PID_t;

typedef struct{
	float err;
	float last_err;
	float err_integration;
	float actual_power;
	float target_power;
	float P_OUT;
	float I_OUT;
	float D_OUT;
	float PID_OUT;
	float last_actual_power;
	float kp;			
	float ki;	
  float kd;
}POWER_PID_t;


typedef struct{
	float err;
	float last_err;
	float err_integration,err_integration_max;
	float err_max;
	float actual_buffer;
	float target_buffer;
	float P_OUT;
	float I_OUT;
	float D_OUT;
	float PID_OUT;
	float last_actual_buffer;
	float kp;			
	float ki;	
  float kd;
}BUFFER_PID_t;


extern switch_flag_t switch_flag;
extern int CHASSIS_vPID_max;
void motor_pid_init(PID_t *pid);
void pid_realize(PID_t *pid);
void POWER_PID_Init(POWER_PID_t *pid);
void power_pid_realize(POWER_PID_t *pid);
void BUFFER_PID_Init(BUFFER_PID_t *pid);
void buffer_pid_realize(BUFFER_PID_t *pid);
float apid_vpid_realize(PidTypeDef *pid,  float ref,float fdb);
void PID_Init(PidTypeDef *pid,  const float PID[5]);
#endif
