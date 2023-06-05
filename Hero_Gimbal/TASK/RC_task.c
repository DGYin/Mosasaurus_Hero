#include "stm32f4xx.h"    
#include "rc_task.h"
#include "remote_control.h"
#include "gimbal_task.h"
#include "math.h"
#include "vision_task.h"        
#include "bsp_uart.h"
#include "shoot_task.h"
#include "can_receive.h"
#include "chassis_task.h"
#include "gimbal_calibration_task.h"

int vision_switch_flag=0;//视觉开关的标识符，用来实现拨一下开关视觉，再播一下开关视觉的功能 
static int deadline_judge(uint8_t a);
float rc_cali(int max,int min, int RC_actual, int RC_max, int RC_mid, int RC_min,int deadline);
int F_flag=0;//判断F是否开启
int frie_first_flag=0;//判断第一次发单
int Fric_Switch_Flag = 0;
int Shoot_Num = 0;
int MOUSE_pre_left_cnt=0;	
//计时变量
int Chassis_Spin_Delay_Cnt					= 0;
int Gimbal_Reverse_Bottom_Delay_Cnt			= 0;
int Chassis_Reverse_Bottom_Delay_Cnt		= 0;
int Gimbal_Precision_Mode_Delay_Cnt			= 0;
int Chassis_TurnAround_Delay_Cnt			= 0;
int Fric_Switch_Delay_Cnt					= 0;
int Pitch_Calibration_Delay_Cnt				= 0;

KEY_CONTROL KEY_MODE=KEY_OFF;
extern int Sent_dataC;
//修改外部模式变量
extern int Gimbal_Precision_Mode;
extern int Last_Gimbal_Precision_Mode;
extern int Gimbal_Precision_Activated_Flag;
extern int Gimbal_Precision_Inactivated_Flag;


void control_mode_judge(void)
{
	if(rc_ctrl.rc.ch[0]!=0||rc_ctrl.rc.ch[1]!=0||rc_ctrl.rc.ch[2]!=0||rc_ctrl.rc.ch[3]!=0||rc_ctrl.rc.ch[4]!=0)
		KEY_MODE=KEY_OFF;
	if(KEY_board||MOUSE_x||MOUSE_y||MOUSE_z)
		KEY_MODE=KEY_ON;
}
int shoot_true=0,shoot_true_cnt=0;
void remote_control_data(void)
{
//	if(!(KEY_board||MOUSE_x||MOUSE_y||MOUSE_z)==0)
//		return;
//	if(switch_is_down(SW_R)&&vision_switch_flag==0) //视觉模式选择
//	{
//		if(vision_mode==VISION_OFF)
//		{
//			vision_mode=VISION_ON;
////			gimbal_set_mode=GIMBAL_RELATIVE_ANGLE;
//			gimbal_set_mode=GIMBAL_ABSOLUTE_ANGLE;
//			vision_sent.yaw.target_angle=gimbal_y.CAN_actual_angle;
//			vision_sent.pitch.target_angle=gimbal_p.IMU_actual_angle;
//		    gimbal_y.target_angle=gimbal_y.CAN_actual_angle;
//			gimbal_p.target_angle=gimbal_p.IMU_actual_angle;
//		}
//			
//		else 
//		{
//			gimbal_set_mode=GIMBAL_ABSOLUTE_ANGLE;
//		    gimbal_y.target_angle=gimbal_y.IMU_actual_angle;
//			vision_mode=VISION_OFF;
//		}
//		vision_switch_flag=1;
//	}
//	else if(switch_is_up(SW_R) || switch_is_mid(SW_R)) vision_switch_flag=0;
	
	if(switch_is_up(SW_R)&&frie_first_flag == 0)
	{
		frie_first_flag=1;
		rc_shoot.left_fric.target_speed = -SHOOT_FRIC_HIGH_SPEED;
		rc_shoot.right_fric.target_speed = SHOOT_FRIC_HIGH_SPEED;
	}

	if(switch_is_up(SW_L) && gimbal_set_mode!=GIMBAL_RELATIVE_ANGLE) //小陀螺模式
		gimbal_set_mode = GIMBAL_TOP_ANGLE; 

	if(switch_is_mid(SW_L) && gimbal_set_mode!=GIMBAL_RELATIVE_ANGLE) //随动
	{
		gimbal_set_mode = GIMBAL_ABSOLUTE_ANGLE;  
		Sent_dataC=OFF_FAR;
	}
	if(switch_is_down(SW_L)) //脱力状态
		gimbal_set_mode = GIMBAL_ZERO_FORCE;  
		Sent_dataC=ON_FAR;
	
	//遥控器摇杆通道值映射到底盘和云台的控制量
	rc_sent.x_speed=rc_ctrl.rc.ch[3]/2.0f;
	rc_sent.y_speed=rc_ctrl.rc.ch[2]/2.0f;
	//if(!(rc_ctrl.rc.ch[4])==0)
	rc_sent.r_speed=-rc_ctrl.rc.ch[4]/660.0f*5.0f;
	
	//rc_sent.r_speed=-rc_cali(Z_SPEED_MAXX,Z_SPEED_MINN,rc_ctrl.rc.ch[4],RC_MAXX,RC_MIDD,RC_MINN,DEADLINE);
	
	rc_sent.yaw.target_angle=1.0f*rc_cali(RC_YAW_ANGLE_MAXX,RC_YAW_ANGLE_MINN,rc_ctrl.rc.ch[0],RC_MAXX,RC_MIDD,RC_MINN,DEADLINE);
	rc_sent.pitch.target_angle=rc_cali(RC_PITCH_ANGLE_MAXX,RC_PITCH_ANGLE_MINN,rc_ctrl.rc.ch[1],RC_MAXX,RC_MIDD,RC_MINN,DEADLINE);
		
	rc_sent.yaw.target_speed=-1.0f*rc_cali(RC_YAW_SPEED_MAXX,RC_YAW_SPEED_MINN,rc_ctrl.rc.ch[0],RC_MAXX,RC_MIDD,RC_MINN,DEADLINE);
	rc_sent.pitch.target_speed=rc_cali(RC_PITCH_SPEED_MAXX,RC_PITCH_SPEED_MINN,rc_ctrl.rc.ch[1],RC_MAXX,RC_MIDD,RC_MINN,DEADLINE);
	
	if(rc_shoot.trigger.last_shoot_flag==0 && frie_first_flag==1)
	{
		if(switch_is_up(SW_R) && rc_shoot.trigger.last_shoot_flag==0)  
		{			
			rc_shoot.trigger.target_angle=SHOOT_NUM;
			rc_shoot.trigger.last_shoot_flag=1;
		}
	}
	
	//遥控器打前哨站
	if((MOUSE_pre_left==1 || shoot_vision_flag==1 ) && rc_shoot.trigger.last_shoot_flag==0) 
    {			
		if (Fric_Switch_Flag)
		{
			rc_shoot.trigger.target_angle=SHOOT_NUM;
			rc_shoot.trigger.last_shoot_flag=1;
			shoot_true=0;
		}
	}
	
	if(switch_is_mid(SW_R))
	{
		rc_shoot.trigger.last_shoot_flag=0;
	}
	
}
int R_flag=0,R_cnt=0;

void key_control_data(void)
{	
//	if((KEY_board||MOUSE_x||MOUSE_y||MOUSE_z)==0)
//	return;
	if(R_flag==1)
	R_cnt++;
	if(R_cnt>=1000) {R_flag=0;R_cnt=0;}
	rc_sent.x_speed=0;
	rc_sent.y_speed=0;
	//Shift键低速
	if(KEY_board & KEY_PRESSED_OFFSET_SHIFT)
	{
		if(KEY_board & KEY_PRESSED_OFFSET_W)
			rc_sent.x_speed=50;
		if(KEY_board & KEY_PRESSED_OFFSET_S)
			rc_sent.x_speed=-50;
		if(KEY_board & KEY_PRESSED_OFFSET_A)
			rc_sent.y_speed=-50;
		if(KEY_board & KEY_PRESSED_OFFSET_D)
			rc_sent.y_speed=50;
		rc_sent.yaw.target_angle=limits_change(KEY_YAW_ANGLE_MAXX_RUN,KEY_YAW_ANGLE_MINN_RUN,MOUSE_x,KEY_MAXX,KEY_MINN);
		if (Gimbal_Precision_Mode == 0)
			rc_sent.pitch.target_angle=limits_change(KEY_PITCH_ANGLE_MAXX_RUN,KEY_PITCH_ANGLE_MINN_RUN,MOUSE_y,KEY_MAXX,KEY_MINN);
		else rc_sent.pitch.target_angle=limits_change(KEY_PITCH_ANGLE_MAXX_RUN,KEY_PITCH_ANGLE_MINN_RUN,MOUSE_z,KEY_MAXX,KEY_MINN);
	}
	else
	{
		if(KEY_board & KEY_PRESSED_OFFSET_W)
			rc_sent.x_speed=KEY_X_SPEED_MAXX;
		if(KEY_board & KEY_PRESSED_OFFSET_S)
			rc_sent.x_speed=KEY_X_SPEED_MINN;
		if(KEY_board & KEY_PRESSED_OFFSET_A)
			rc_sent.y_speed=KEY_Y_SPEED_MINN;
		if(KEY_board & KEY_PRESSED_OFFSET_D)
			rc_sent.y_speed=KEY_Y_SPEED_MAXX;
		rc_sent.yaw.target_angle=limits_change(KEY_YAW_ANGLE_MAXX_ON,KEY_YAW_ANGLE_MINN_ON,MOUSE_x,KEY_MAXX,KEY_MINN);
		if (Gimbal_Precision_Mode == 0)
			rc_sent.pitch.target_angle=limits_change(KEY_PITCH_ANGLE_MAXX_ON,KEY_PITCH_ANGLE_MINN_ON,MOUSE_y,KEY_MAXX,KEY_MINN);
		else rc_sent.pitch.target_angle=limits_change(KEY_PITCH_ANGLE_MAXX_ON,KEY_PITCH_ANGLE_MINN_ON,MOUSE_z,KEY_MAXX,KEY_MINN);
	}
	if(F_flag==1 && gimbal_set_mode != GIMBAL_TOP_ANGLE ) gimbal_set_mode=GIMBAL_RELATIVE_ANGLE;
	else if(F_flag==0 && gimbal_set_mode != GIMBAL_TOP_ANGLE) gimbal_set_mode=GIMBAL_ABSOLUTE_ANGLE;
	//Ctrl+R键底盘随动开关
	if(KEY_board & KEY_PRESSED_OFFSET_R)
	{
		if (KEY_board & KEY_PRESSED_OFFSET_CTRL)
		{
			if (Chassis_Spin_Delay_Cnt == 0)
			{
				Chassis_Spin_Delay_Cnt = 100;
				if (Chassis_Follow_Switch == Chassis_Follow_ON) Chassis_Follow_Switch = Chassis_Follow_OFF;
				else Chassis_Follow_Switch = Chassis_Follow_ON;
			}
		}
	}
	//R键小陀螺模式开关
	if(KEY_board & KEY_PRESSED_OFFSET_R)
	{
		if (Chassis_Spin_Delay_Cnt == 0)
		{
			Chassis_Spin_Delay_Cnt = 100;
			gimbal_set_mode = GIMBAL_TOP_ANGLE;
			if (Chassis_Mode !=  CHASSIS_SPIN)
			{
				Chassis_Follow_Switch = Chassis_Follow_OFF;
				Chassis_Mode = CHASSIS_SPIN;
			}
			else if (Chassis_Mode ==  CHASSIS_SPIN)
			{
				Chassis_Follow_Switch = Chassis_Follow_ON;
				Chassis_Mode = CHASSIS_NORMAL;
			}
		}
	}
	//Q键阿克曼底盘模式，底盘左转
	rc_sent.r_speed = 0;
	if(KEY_board & KEY_PRESSED_OFFSET_Q)
	{
		if (rc_sent.x_speed >= 0)
			rc_sent.r_speed = -2.0;
		else rc_sent.r_speed = 2.0;
	}
	//E键阿克曼底盘模式，底盘右转
	if(KEY_board & KEY_PRESSED_OFFSET_E)
	{
		if (rc_sent.x_speed >= 0)
			rc_sent.r_speed = 2.0;
		else 
			rc_sent.r_speed = -2.0;
	}

	//摩擦轮开关
	if (MOUSE_pre_right)
	{
		if (Fric_Switch_Delay_Cnt==0)
		{
			if (Fric_Switch_Flag == 0) Fric_Switch_Flag=1;
			else if (Fric_Switch_Flag == 1) Fric_Switch_Flag=0;
			rc_shoot.left_fric.target_speed = -SHOOT_FRIC_HIGH_SPEED*Fric_Switch_Flag;
		    rc_shoot.right_fric.target_speed = SHOOT_FRIC_HIGH_SPEED*Fric_Switch_Flag;
			Fric_Switch_Delay_Cnt = 100;
		}
	}
	//发射及发射延迟
	if((MOUSE_pre_left==1 || shoot_vision_flag==1 ) && rc_shoot.trigger.last_shoot_flag==0) 
    {			
		if (Fric_Switch_Flag)
		{
			rc_shoot.trigger.target_angle=SHOOT_NUM;
			rc_shoot.trigger.last_shoot_flag=1;
			shoot_true=0;
			Shoot_Num++;
		}
		else rc_shoot.trigger.target_angle=0;
	}
	if(MOUSE_pre_left==0) 
	{
		MOUSE_pre_left_cnt++;
		if(MOUSE_pre_left_cnt>=100)
			{rc_shoot.trigger.last_shoot_flag=0;MOUSE_pre_left_cnt=0;}
	}
    
	//一键掉头
	if(KEY_PRESSED_OFFSET_C&KEY_board)
	{
		if (Chassis_TurnAround_Delay_Cnt == 0) 
		{
			if(KEY_PRESSED_OFFSET_CTRL&KEY_board) //头和底盘一起转
			{
				if (Gimbal_Precision_Mode == 0)
					gimbal_y.target_angle = gimbal_y.target_angle + 180.f;
			}
			else //底盘不转，头转
			{
				if (gimbal_y.Bool_Invert_Flag == 0) gimbal_y.Bool_Invert_Flag = 1;
				else if (gimbal_y.Bool_Invert_Flag == 1) gimbal_y.Bool_Invert_Flag = 0;
				gimbal_y.Valuence_Invert_Flag = -gimbal_y.Valuence_Invert_Flag;
			}
			Chassis_TurnAround_Delay_Cnt = 100;
		}
	}
	//云台头部反转180度
	if(KEY_PRESSED_OFFSET_X&KEY_board)
	{ 
		
		if ((gimbal_y.Bool_Invert_Flag == 0)&&(Gimbal_Reverse_Bottom_Delay_Cnt == 0)) {gimbal_y.Bool_Invert_Flag = 1; Gimbal_Reverse_Bottom_Delay_Cnt = 100;}
		if ((gimbal_y.Bool_Invert_Flag == 1)&&(Gimbal_Reverse_Bottom_Delay_Cnt == 0)) {gimbal_y.Bool_Invert_Flag = 0; Gimbal_Reverse_Bottom_Delay_Cnt = 100;}
		
	}
	//底盘运动方向反转
	if(KEY_PRESSED_OFFSET_Z&KEY_board)
	{ 
		if (Chassis_Reverse_Bottom_Delay_Cnt == 0) 
		{
			gimbal_y.Valuence_Invert_Flag = -gimbal_y.Valuence_Invert_Flag;
			Chassis_Reverse_Bottom_Delay_Cnt = 100; //计时重置
		}
		
	}	
	if(KEY_PRESSED_OFFSET_B&KEY_board)
	{
		if (Gimbal_Precision_Mode_Delay_Cnt == 0) 
		{
			//模式切换
			if (Gimbal_Precision_Mode==1) Gimbal_Precision_Mode = 0;
			else Gimbal_Precision_Mode = 1;
			
			//计时重置
			Gimbal_Precision_Mode_Delay_Cnt = 100;
		}
	}
	if(KEY_PRESSED_OFFSET_F&KEY_board)
	{
		if (Pitch_Calibration_Delay_Cnt == 0) 
		{
			//进行校准
			Gimbal_Calibration_Target_Times++;
			//计时重置
			Pitch_Calibration_Delay_Cnt = 1000;
		}
	}
	extern int Relay_Set_State;
	if (KEY_board&KEY_PRESSED_OFFSET_CTRL)
	{
		if (KEY_board&KEY_PRESSED_OFFSET_G)	//Ctrl+G	电池供电模式
			Relay_Set_State = 2;
	}
	if (KEY_board&KEY_PRESSED_OFFSET_SHIFT)
	{
		if (KEY_board&KEY_PRESSED_OFFSET_G)	//Shift+G	电容供电模式
			Relay_Set_State = 1;
	}	

	//计时用
	if (Chassis_Spin_Delay_Cnt > 0)				Chassis_Spin_Delay_Cnt--;
	if (Gimbal_Precision_Mode_Delay_Cnt > 0)	Gimbal_Precision_Mode_Delay_Cnt--;
	if (Gimbal_Reverse_Bottom_Delay_Cnt > 0)	Gimbal_Reverse_Bottom_Delay_Cnt--; 
	if (Chassis_Reverse_Bottom_Delay_Cnt > 0)	Chassis_Reverse_Bottom_Delay_Cnt--;
	if (Chassis_TurnAround_Delay_Cnt>0)			Chassis_TurnAround_Delay_Cnt--;
	if (Fric_Switch_Delay_Cnt>0)				Fric_Switch_Delay_Cnt--;
	if (Pitch_Calibration_Delay_Cnt>0)			Pitch_Calibration_Delay_Cnt--;
}
/**
	* @brief       幅度判断函数
	* @param[a]		想要判断的数值
	* @retvel      1（不超限幅），0（超限幅）
*/
static int deadline_judge(uint8_t a)
{
	if(abs(a-RC_MIDD)<=DEADLINE) return 1;
	else return 0;
}

/**
	* @brief       			遥控器映射到实际速度或者实际角度
	* @param[max]		    需要映射的最大值
	* @param[min]		    需要映射的最小值
	* @param[RCactual]	遥控器通道实际值
	* @param[RCmax]		  遥控器通道最大值
	* @param[RCmid]		  遥控器通道中值
	* @param[RCmin]		  遥控器通道最小值
	* @param[deadline]	遥控器死区
	* @retvel      1（不超限幅），0（超限幅）
*/
float rc_cali(int max,int min, int RC_actual, int RC_max, int RC_mid, int RC_min, int deadline)
{
	float value;
	if((RC_actual-RC_mid)>=deadline)
	{
		value=(float)(RC_actual-RC_mid-deadline)/(RC_max-RC_min);
		value*=max;
	}
	else if((RC_actual-RC_mid)<-deadline)
	{
		value=(float)(RC_actual-RC_mid+deadline)/(RC_max-RC_min);
		value*=max;
	}
	else
		value=0;
	return value;
}




