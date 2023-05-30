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
  * Description: 头文件   
  ******************************************************************************
**/
#ifndef __MOTOR_H
#define __MOTOR_H

#include "pid.h"
#include "kinematics.h"
#include "can.h"
#include <math.h>
#include "referee.h"


/*******************************************************************************  
													在这里配置底盘类型
*******************************************************************************/
#define Mecanum_Wheel_Chassis 0	//麦轮底盘
#define AGV_Chassis						1	//舵轮底盘
#define Chassis_Type AGV_Chassis

//也可由云台直接传输指令
#define CHASSIS_REMOTE_CLOSE   	 1    //关闭遥控器                                左高位
#define CHASSIS_NORMAL           3    //正常模式                                  左中位
#define CHASSIS_SPIN             2    //小陀螺模式                                    左低位

#define GIMBAL_HEAD_ANGLE  0 

/*Target_Velocity_Smoothen函数相关的宏定义*/
#define Low_Voltage_Mode 		0
#define High_Voltage_Mode 		2
#define Supercap_Disconnected_Mode		1

#define Smoothen_Off 0 //不进行曲线平缓
#define Uniform_Acceleration 1 //匀加速模式，用于平滑速度曲线
#define Ease_Out 2 //缓出模式

#define Acceleration 300 //加速度，用于平滑速度曲线的匀加速模式

#define POWER_LIMIT         80.0f
#define WARNING_POWER       40.0f   
#define WARNING_POWER_BUFF  50.0f  

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
	
	float start_angle;			//电机初始角度值
	int start_angle_flag;	//记录电机初始角度值的flag
	int switch_mode_flag;  //记录模式转换角度值的flag
	int stop_angle;				//发送停止命令时候的角度值
	float Target_Angle;
	
	float actual_angle;			//当前真实角度值
	float last_angle;				//上一次返回的角度值
	float switch_mode_angle;  //记录模式转换角度值
	int round_cnt;				//相对开机时转过的圈数
	int total_angle;			//总共转过的计数
	
	float actual_speed;			//电机真实速度,rpm
	int Target_Speed;			//电机目标速度,rpm  转/min
	int last_speed;       //电机上一次回传的速度值
	int actual_current;		//电机真实电流
	int target_current;		//电机目标电流
	//int temp;							//电机温度（2006电机不支持，3508支持）
	

	
	PID_t pid;
}MOTOR_t;

typedef struct
{
	int16_t vx_set;
	int16_t vy_set;
	int16_t wz_set;
	uint8_t chassis_mode;
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
void vpid_chassis_realize_F(void);
void chassis_move(void);
int Target_Velocity_Smoothen(int Target_Speed, int Current_Speed, int Smoothen_Method, int Power_Mode);
#endif

