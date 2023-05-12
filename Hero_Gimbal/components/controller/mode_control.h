#ifndef __MODE_CONTROL_H
#define __MODE_CONTROL_H
/*
云台模式
*/
typedef enum
{
    GIMBAL_INIT = 0, //电机初始化模式
    GIMBAL_CALI,    //电机速度控制模式
    GIMBAL_ABSOLUTE_ANGLE, //云台随动 2
    GIMBAL_TOP_ANGLE,       //小陀螺 3
    GIMBAL_RELATIVE_ANGLE, //电机编码器角度控制 4
    GIMBAL_ZERO_FORCE,   //无力状态
} GIMBAL_MODE_t;

typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //电机陀螺仪速度环控制
    GIMBAL_MOTOR_GYRO,    //电机陀螺仪角度控制
    GIMBAL_MOTOR_ENCONDE, //电机编码值角度控制
} MODE_t;


#endif
