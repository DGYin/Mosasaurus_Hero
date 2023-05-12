#ifndef __MODE_CONTROL_H
#define __MODE_CONTROL_H
/*
��̨ģʽ
*/
typedef enum
{
    GIMBAL_INIT = 0, //�����ʼ��ģʽ
    GIMBAL_CALI,    //����ٶȿ���ģʽ
    GIMBAL_ABSOLUTE_ANGLE, //��̨�涯 2
    GIMBAL_TOP_ANGLE,       //С���� 3
    GIMBAL_RELATIVE_ANGLE, //����������Ƕȿ��� 4
    GIMBAL_ZERO_FORCE,   //����״̬
} GIMBAL_MODE_t;

typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //����������ٶȻ�����
    GIMBAL_MOTOR_GYRO,    //��������ǽǶȿ���
    GIMBAL_MOTOR_ENCONDE, //�������ֵ�Ƕȿ���
} MODE_t;


#endif
