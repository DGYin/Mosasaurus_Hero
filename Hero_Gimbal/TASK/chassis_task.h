#ifndef __chassis_task_H
#define __chassis_task_H

#define CHASSIS_REMOTE_CLOSE	1	//�ر�ң����                                ���λ
#define CHASSIS_NORMAL			3	//����ģʽ                                  ����λ
#define CHASSIS_SPIN			2	//С����ģʽ  

#define Chassis_Follow_ON		1
#define Chassis_Follow_OFF		0

extern int Chassis_Mode;
extern int Chassis_Follow_Switch;
void remote_chassis(void);
#endif
