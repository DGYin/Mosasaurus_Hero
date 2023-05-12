/**
  ******************************************************************************
  * @file    Project/APP/yaw_turn.c
  * @author  NEUQRM
  * @version V1.0.0
  * @date    12.2021
  * @brief   ���ļ�������yaw���pid����ֵ����ֵ��pid���㣬�����һϵ�и��������йصĺ���
  ******************************************************************************
  * @attention
  ******************************************************************************
  ****************************(C) COPYRIGHT 2021 NEUQRM****************************

  ...........  ������δ�ɹ���ͬ־����Ŭ��  ...........
*/

#include "yaw_turn.h"

//���յ���yawͨ��ֵ
int16_t yaw_ch2;
//���յ���yaw����ֵ
int16_t yaw_spin_current;

static void GIMBAL_CALBACK_GET(void);
static void GIMBAL_CALBACK_GET(void);
static void GIMBAL_Set_Mode(void);
static void GIMBAL_Set_Contorl(void);
static void GIMBAL_PID(void);
static void gimbal_motor_encode_pid(GIMBAL_t *gimbal_);
static int deadline_judge(float a, int flag);

GIMBAL_t gimbal_y;

float YawGyroPid[5] 	= {0.0f,			0.0f,			0.0f,		0.0f,			0.0f};  //imu�ǶȻ�
float YawEncondePid[5] 	= {1.5f,			0.0f,			0.0f,		500.0f,			0.0f};  //�������ǶȻ�
float YawEncondePidSpeed[5] 	= {150.0f,			15.0f,			0.1f,		15000.0f,			9000.0f};  //�������ǶȻ�
float YawSpeedPid[5] 			= {2100.0f,			300.0f,			0.0f,		13000.0f,			7000.0f};     //�ٶȻ�
float yaw_Target_Angle = 0.0f;
float	yaw_Target_Speed = 0.0f;

//��Χת��
float limits_change(int maxx, int minn, int a, int maxx_actual, int minn_actual)
{
    float b = (float)(a - minn_actual) / (float)(maxx_actual - minn_actual);
    float c = (float)b * (float)(maxx - minn);
    float ans = c + (float)minn;
    return ans;
}

void yaw_ch2_to_yaw_target(void)//yawͨ��ֵת��ΪĿ��ֵ�����Ƕȼ��ɣ�
{
    yaw_Target_Speed = limits_change(CH_YAW_SPEED_MAXX, CH_YAW_SPEED_MINN, yaw_ch2, RC_MAXX, RC_MINN);
    yaw_Target_Angle = limits_change(CH_YAW_ANGLE_MAXX, CH_YAW_ANGLE_MINN, yaw_ch2, RC_MAXX, RC_MINN);
}

uint8_t canTX_yaw(int16_t yaw_current)//yaw��������
{
    CAN_TxHeaderTypeDef canFrame;
    uint8_t data[8] = {0};
    uint32_t temp = 0;

    canFrame.IDE = CAN_ID_STD;
    canFrame.StdId = 0x2ff;
    canFrame.RTR = CAN_RTR_DATA;
    canFrame.DLC = 8;
    canFrame.TransmitGlobalTime = DISABLE;
    data[0] = yaw_current >> 8;
    data[1] = yaw_current & 0Xff;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    HAL_CAN_AddTxMessage(&hcan1, &canFrame, data, &temp);

    return temp;
}


float Trans_Yaw_Ang;
void yaw_turn(void)//yaw��ת�����ӽӵ�ͨ��ֵ�����ֵ���������ֵ��
{
	GIMBAL_Set_Mode();
	GIMBAL_CALBACK_GET();
    //��������ĵ���ֵ����Yaw���
    //canTX_yaw(gimbal_y.given_current);
    
}

static void GIMBAL_PID(void)
{
    while(gimbal_y.Target_Angle - gimbal_y.CAN_total_angle > 360)
        gimbal_y.Target_Angle = gimbal_y.Target_Angle - 720;
    while(gimbal_y.Target_Angle - gimbal_y.CAN_total_angle < -360)
        gimbal_y.Target_Angle = gimbal_y.Target_Angle + 720;

    gimbal_motor_encode_pid(&gimbal_y);
}

static void GIMBAL_Set_Mode(void)
{
    gimbal_y.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
}

static void GIMBAL_CALBACK_GET(void)
{

    gimbal_y.CAN_actual_angle = yaw_can_rx.angle / 8191.0f * 360.0f;
    gimbal_y.CAN_total_angle = yaw_can_rx.turns * 360.0f + gimbal_y.CAN_actual_angle;		//����yaw��total_angle
    gimbal_y.CAN_actual_speed = yaw_can_rx.speed;
}

//static void GIMBAL_Set_Contorl(void)
//{
//	//ѡ������ֵ��Դ(�Ƿ�����)
////	Vision_Task();
//	 gimbal_y.add_angle=yaw_Target_Angle;//�˴��к궨��gimbal_y.add_angle    yaw_angle
//		if(deadline_judge(yaw_angle,1)!=0)
//		{
//			gimbal_y.Target_Angle+=yaw_angle;
//		}

//}

static int deadline_judge(float a, int flag) //ң���������ж�
{
    if(abs(a) <= YAW_DEADLINE) return 0;
    else return 1;
}

int abs(int a)
{
    if(a < 0) return -a;
    else return a;
}

void Gimbal_Init(void)
{
    float yaw_Target_Angle = 0.0f;
    float	yaw_Target_Speed = 0.0f;
    int16_t yaw_spin_current = 0;
    gimbal_y.IMU_actual_angle = 0.0f;
    gimbal_y.IMU_actual_speed = 0.0f;
    gimbal_y.CAN_actual_angle = 0.0f;
    gimbal_y.CAN_actual_speed = 0.0f;
    gimbal_y.Target_Angle = 239.0f;
    gimbal_y.Target_Speed = 0.0f;
    gimbal_y.add_angle = 0.0f;
    gimbal_y.given_current = 0;

    gimbal_y.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
}

void Yaw_PIDinit(void)//yaw��pid��ʼ��
{
    //PID_Init(&gimbal_y.gimbal_raw_pid,YawSpeedPid);
    //PID_Init(&gimbal_y.gimbal_gyro_pid,YawGyroPid);
    PID_Init(&gimbal_y.gimbal_enconde_pid, YawEncondePid);
    PID_Init(&gimbal_y.gimbal_enconde_pid_speed, YawEncondePidSpeed);
    Gimbal_Init();
}

//��̨��ʼ��


static void gimbal_motor_encode_pid(GIMBAL_t *gimbal_)//��yaw��
{
    apid_vpid_realize(&gimbal_->gimbal_enconde_pid, gimbal_->Target_Angle, gimbal_->CAN_total_angle);   //pitch�ĵ���������Ϊgimbal_->CAN_actual_angle
    gimbal_->Target_Speed = gimbal_->gimbal_enconde_pid.out;
    apid_vpid_realize(&gimbal_->gimbal_enconde_pid_speed, gimbal_->Target_Speed, gimbal_->CAN_actual_speed);
    gimbal_->given_current = gimbal_->gimbal_enconde_pid_speed.out;
}


