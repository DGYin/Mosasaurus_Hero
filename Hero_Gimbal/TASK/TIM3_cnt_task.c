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
	* @ brief       TIM3主循环中断函数
	* @ param				none
	* @ retvel      none
*/
void TIM3_CNT_TASK()
{
	Buzzer_Task(S_Count,MS_Count);
	//等待IMU数据稳定
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
	//IMU任务
    INS_task();
	if (start_flag == 1)
	{
	    if(MS_Count % 7 == 2)
		{
			Gimbal_Task(S_Count, MS_Count);		//控制云台
			shoot_task();		//控制拨弹轮、摩擦轮的运动
			DMA_Send();			//向上位机发送数据
			remote_chassis();	//控制底盘的模式和运动
		}
	}
    if(MS_Count % 20 == 0)
    {
        control_mode_judge();
        //DMA_Send();
    }

    if(MS_Count % 7 == 0)
    {
        //remote_chassis();
        if(KEY_MODE == KEY_OFF)
            remote_control_data();	//发送遥控器数据
        else
            key_control_data();	//发送键盘数据
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
	//计时部分
    if(MS_Count >= 1000)
    {
        MS_Count = 0;
        S_Count++;
        if(start_flag == 0)
            IMU_cnt++;
    }

}
