#include "tim3_cnt_task.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "RC_task.h"
#include "tim.h"
#include "bsp_imu.h"
#include "bsp_uart.h"
#include "can_receive.h"
#include "sent_task.h"
#include "buzzer_task.h"
int MS_Count = 0;
int IMU_cnt = 0, start_flag = 0, S_Count = 0;
//0.1ms
/*
	* @ brief       TIM3��ѭ���жϺ���
	* @ param				none
	* @ retvel      none
*/
void TIM3_CNT_TASK()
{
	Buzzer_Task(S_Count,MS_Count);
	//�ȴ�IMU�����ȶ�
	if(IMU_cnt > 3)
    {
        start_flag = 1;
    }
    else
    {
        canTX_chassis(0, 0, 0, 0);
        canTX_mode(CHASSIS_REMOTE_CLOSE);
    }
    MS_Count++;
	//IMU����
    INS_task();
    if(MS_Count % 20 == 0)
    {
        control_mode_judge();
        //DMA_Send();
    }
    if(MS_Count % 7 == 2 && start_flag == 1)
    {
        Gimbal_Task();		//������̨
        shoot_task();		//���Ʋ����֡�Ħ���ֵ��˶�
        DMA_Send();			//����λ����������
        remote_chassis();	//���Ƶ��̵�ģʽ���˶�
    }
    if(MS_Count % 7 == 0)
    {
        //remote_chassis();
        if(KEY_MODE == KEY_OFF)
            remote_control_data();	//����ң��������
        else
            key_control_data();	//���ͼ�������
    }

    if(MS_Count % 70 == 0)
    {
        if(gimbal_set_mode == GIMBAL_ABSOLUTE_ANGLE) //A
            canTX_UI(gimbal_p.IMU_actual_angle * 100, 1);
        else  if(gimbal_set_mode == GIMBAL_RELATIVE_ANGLE) //F
            canTX_UI(gimbal_p.IMU_actual_angle * 100, 2);
        else  if(gimbal_set_mode == GIMBAL_TOP_ANGLE) //T
            canTX_UI(gimbal_p.IMU_actual_angle * 100, 3);
    }
	//��ʱ����
    if(MS_Count >= 1000)
    {
        MS_Count = 0;
        S_Count++;
        if(start_flag == 0)
            IMU_cnt++;
    }

}
