#ifndef __pid_H
#define __pid_H
#include "main.h"
#include "usart.h"
#include <string.h>

typedef struct
{
    //PID ������
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //������
    float max_iout; //���������
		float deadband;
    float ref;
    float fdb;
	float lastfdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    float error[3]; //����� 0���� 1��һ�� 2���ϴ�

} PidTypeDef;


extern void PID_Init(PidTypeDef *pid,  const float PID[5]);
extern float PID_Calc(PidTypeDef *pid, float ref, float fdb);
extern void PID_clear(PidTypeDef *pid);

#endif
