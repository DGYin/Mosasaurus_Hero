/**
  ************************************* Copyright ****************************** 
  *
  *                 (C) Copyright 2022,Hebei,China, NEUQRM.
  *                            All Rights Reserved
  *                              
  *                     By DGYin, 
  *                     https://--
  *    
  * FileName   : chassis_move.c   
  * Version    : v1.1.0-alpha.1	
  * Author     : NEUQRM, DGYin, 
  * Date       : 2022-11-07         
  * Description: 该文件是针对得到的模式来进行对应的输出和底盘功率限制
  * Function List:  
  	1. ....
			<version>       : 		
			<modify staff>	:
			<data>          :
			<description>   : 
  	2. ...
  ******************************************************************************
 */
 
/*include----------------------------------------------------------------------*/
#include "chassis_move.h"
#include "bsp_can.h"
/*define-----------------------------------------------------------------------*/


/*variate----------------------------------------------------------------------*/
float total_current_limit,total_current,power;
uint16_t buffer,max_power;
BUFFER_PID_t b_pid;
float current_scale;

CHASSIS_CONTROL_ORDER_t chassis_control_order;
MOTOR_t Chassis_Motor1,Chassis_Motor2,Chassis_Motor3,Chassis_Motor4,chassis_center,Chassis_MotorA,Chassis_MotorB,Chassis_MotorC,Chassis_MotorD;
POWER_PID_t p_pid;
int shoot_flag=0;

POWER_PID_t p_pid;
BUFFER_PID_t b_pid;
REAl_CHASSIS_SPEED_t real_chassis_speed;
int8_t max_d_speed_x;
int8_t max_d_speed_y;
float avx,avy,awz;
float last_vx,last_vy;

//int Chassis_Type;
/*statement--------------------------------------------------------------------*/


/*Function prototype Begin*****************************************************/
void chassis_spin(float *vx,float *vy); 
static float chassis_follow(void);
static void chassis_speed_control(float speed_x, float speed_y, float speed_r);
static void chassis_move_mode(void);			
static void can_send_chassis_current(void);
static float Get_chassis_theta(void);
static void power_limitation_jugement(void);
static void power_limitation_jugement(void);
static float chassis_buffer_loop(uint16_t buffer);
static void chassis_fly(uint16_t buffer);
static float chassis_power_loop(uint16_t target_power,float actual_power,float last_power);

/*Function prototype End*******************************************************/
/*
 * **************************************************************************
 * ********************                                  ********************
 * ********************      ESSESTIAL INFORMATION       ********************
 * ********************                                  ********************
 * **************************************************************************
 *                                                                          *
 *                                   _oo8oo_                                *
 *                                  o8888888o                               *
 *                                  88" . "88                               *
 *                                  (| -_- |)                               *
 *                                  0\  =  /0                               *
 *                                ___/'==='\___                             *
 *                              .' \\|     |// '.                           *
 *                             / \\|||  :  |||// \                          *
 *                            / _||||| -:- |||||_ \                         *
 *                           |   | \\\  -  /// |   |                        *
 *                           | \_|  ''\---/''  |_/ |                        *
 *                           \  .-\__  '-'  __/-.  /                        *
 *                         ___'. .'  /--.--\  '. .'___                      *
 *                      ."" '<  '.___\_<|>_/___.'  >' "".                   *
 *                     | | :  `- \`.:`\ _ /`:.`/ -`  : | |                  *
 *                     \  \ `-.   \_ __\ /__ _/   .-` /  /                  *
 *                 =====`-.____`.___ \_____/ ___.`____.-`=====              *
 *                                   `=---=`                                *
 * **************************************************************************
 * ********************                                  ********************
 * ********************      				 										 ********************
 * ********************         佛祖保佑 永远无BUG       ********************
 * ********************                                  ********************
 * **************************************************************************
 *         .............................................
 *                  佛祖镇楼                  BUG辟易
 *          佛曰:
 *                  写字楼里写字间，写字间里程序员；
 *                  程序人员写程序，又拿程序换酒钱。
 *                  酒醒只在网上坐，酒醉还来网下眠；
 *                  酒醉酒醒日复日，网上网下年复年。
 *                  但愿老死电脑间，不愿鞠躬老板前；
 *                  奔驰宝马贵者趣，公交自行程序员。
 *                  别人笑我忒疯癫，我笑自己命太贱；
 *                  不见满街漂亮妹，哪个归得程序员？
 */

/**
 **************************************************************
 *                                                            *
 *   .=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-.       *
 *    |                     ______                     |      *
 *    |                  .-"      "-.                  |      *
 *    |                 /            \                 |      *
 *    |     _          |              |          _     |      *
 *    |    ( \         |,  .-.  .-.  ,|         / )    |      *
 *    |     > "=._     | )(__/  \__)( |     _.=" <     |      *
 *    |    (_/"=._"=._ |/     /\     \| _.="_.="\_)    |      *
 *    |           "=._"(_     ^^     _)"_.="           |      *
 *    |               "=\__|IIIIII|__/="               |      *
 *    |              _.="| \IIIIII/ |"=._              |      *
 *    |    _     _.="_.="\          /"=._"=._     _    |      *
 *    |   ( \_.="_.="     `--------`     "=._"=._/ )   |      *
 *    |    > _.="                            "=._ <    |      *
 *    |   (_/                                    \_)   |      *
 *    |                                                |      *
 *    '-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-='      *
 *                                                            *
 *           LASCIATE OGNI SPERANZA, VOI CH'ENTRATE           *
 **************************************************************
 */
/*<!--
                       ::
                      :;J7, :,                        ::;7:
                      ,ivYi, ,                       ;LLLFS:
                      :iv7Yi                       :7ri;j5PL
                     ,:ivYLvr                    ,ivrrirrY2X,
                     :;r@Wwz.7r:                :ivu@kexianli.
                    :iL7::,:::iiirii:ii;::::,,irvF7rvvLujL7ur
                   ri::,:,::i:iiiiiii:i:irrv177JX7rYXqZEkvv17
                ;i:, , ::::iirrririi:i:::iiir2XXvii;L8OGJr71i
              :,, ,,:   ,::ir@mingyi.irii:i:::j1jri7ZBOS7ivv,
                 ,::,    ::rv77iiiriii:iii:i::,rvLq@huhao.Li
             ,,      ,, ,:ir7ir::,:::i;ir:::i:i::rSGGYri712:
           :::  ,v7r:: ::rrv77:, ,, ,:i7rrii:::::, ir7ri7Lri
          ,     2OBBOi,iiir;r::        ,irriiii::,, ,iv7Luur:
        ,,     i78MBBi,:,:::,:,  :7FSL: ,iriii:::i::,,:rLqXv::
        :      iuMMP: :,:::,:ii;2GY7OBB0viiii:i:iii:i:::iJqL;::
       ,     ::::i   ,,,,, ::LuBBu BBBBBErii:i:i:i:i:i:i:r77ii
      ,       :       , ,,:::rruBZ1MBBqi, :,,,:::,::::::iiriri:
     ,               ,,,,::::i:  @arqiao.       ,:,, ,:::ii;i7:
    :,       rjujLYLi   ,,:::::,:::::::::,,   ,:i,:,,,,,::i:iii
    ::      BBBBBBBBB0,    ,,::: , ,:::::: ,      ,,,, ,,:::::::
    i,  ,  ,8BMMBBBBBBi     ,,:,,     ,,, , ,   , , , :,::ii::i::
    :      iZMOMOMBBM2::::::::::,,,,     ,,,,,,:,,,::::i:irr:i:::,
    i   ,,:;u0MBMOG1L:::i::::::  ,,,::,   ,,, ::::::i:i:iirii:i:i:
    :    ,iuUuuXUkFu7i:iii:i:::, :,:,: ::::::::i:i:::::iirr7iiri::
    :     :rk@Yizero.i:::::, ,:ii:::::::i:::::i::,::::iirrriiiri::,
     :      5BMBBBBBBSr:,::rv2kuii:::iii::,:i:,, , ,,:,:i@petermu.,
          , :r50EZ8MBBBBGOBBBZP7::::i::,:::::,: :,:,::i;rrririiii::
              :jujYY7LS0ujJL7r::,::i::,::::::::::::::iirirrrrrrr:ii:
           ,:  :@kevensun.:,:,,,::::i:i:::::,,::::::iir;ii;7v77;ii;i,
           ,,,     ,,:,::::::i:iiiii:i::::,, ::::iiiir@xingjief.r;7:i,
        , , ,,,:,,::::::::iiiiiiiiii:,:,:::::::::iiir;ri7vL77rrirri::
         :,, , ::::::::i:::i:::i:i::,,,,,:,::i:i:::iir;@Secbone.ii:::

--*/


/**原readme
  ******************************************************************************
  * @file    Project/APP/chassis_move.c 
  * @author  NEUQRM
  * @version V1.0.0
  * @date    12.2021
  * @brief   该文件是针对得到的模式来进行对应的输出和底盘功率限制
  ******************************************************************************
  * @attention
  ******************************************************************************
  ****************************(C) COPYRIGHT 2021 NEUQRM****************************

  ...........  革命尚未成功，同志仍需努力  ...........
*/  

void chassis_move(void)//底盘移动过程
{
    
	//chassis_move_mode(); //对应模式的底盘状态赋值
//	power_limitation_jugement(); //功率判断与限制
	//vpid_chassis_realize(); //pid计算
    vpid_chassis_realize_F();
	can_send_chassis_current();	//输出底盘电流值
}

int Power_Mode;
void vpid_chassis_realize_F(void)
{
    switch_flag=CHASSIS;  
	//控制电机的加速度
	Chassis_Motor1.Target_Speed = Target_Velocity_Smoothen(Chassis_Motor1.Target_Speed, Chassis_Motor1.actual_speed, Ease_Out, Power_Mode);
	Chassis_Motor2.Target_Speed = Target_Velocity_Smoothen(Chassis_Motor2.Target_Speed, Chassis_Motor2.actual_speed, Ease_Out, Power_Mode);
	Chassis_Motor3.Target_Speed = Target_Velocity_Smoothen(Chassis_Motor3.Target_Speed, Chassis_Motor3.actual_speed, Ease_Out, Power_Mode);
	Chassis_Motor4.Target_Speed = Target_Velocity_Smoothen(Chassis_Motor4.Target_Speed, Chassis_Motor4.actual_speed, Ease_Out, Power_Mode);
	
	//PID参数注入
	Chassis_Motor1.pid.speed_loop.vpid.Target_Speed = Chassis_Motor1.Target_Speed;
	Chassis_Motor2.pid.speed_loop.vpid.Target_Speed = Chassis_Motor2.Target_Speed;
	Chassis_Motor3.pid.speed_loop.vpid.Target_Speed = Chassis_Motor3.Target_Speed;
	Chassis_Motor4.pid.speed_loop.vpid.Target_Speed = Chassis_Motor4.Target_Speed;
	pid_realize(&(Chassis_Motor1.pid));   
	pid_realize(&(Chassis_Motor2.pid));   
	pid_realize(&(Chassis_Motor3.pid));   
	pid_realize(&(Chassis_Motor4.pid));   
     
	switch_flag=NUL;  
}
int flag3=0;
static void chassis_speed_control(float speed_x, float speed_y, float speed_r)
{
	flag3=1;
	int max;
		//速度换算
		if (Chassis_Type == Mecanum_Wheel_Chassis) Mecanum_Wheel_BaseVel_To_WheelVel(speed_x, speed_y, speed_r);
		if (Chassis_Type == AGV_Chassis) 
        {
            //AGV_BaseVel_To_WheelVel(speed_x, speed_y, speed_r);
            int angle;
            angle = AGV_DirectiveMotor_RobotMotion_To_TargetStatus(speed_x, speed_y, 0, 0, 0);
            //AGV_DirectiveMotor_TargetStatus_To_MotorAngle(angle, 0);
        }
        
        //Speed_Limitation();
//		max=find_max();
//		if(max>MAX_MOTOR_SPEED)
//		{
//			Chassis_Motor1.Target_Speed=(int)(Chassis_Motor1.Target_Speed*MAX_MOTOR_SPEED*1.0/max);
//			Chassis_Motor2.Target_Speed=(int)(Chassis_Motor2.Target_Speed*MAX_MOTOR_SPEED*1.0/max);
//			Chassis_Motor3.Target_Speed=(int)(Chassis_Motor3.Target_Speed*MAX_MOTOR_SPEED*1.0/max);
//			Chassis_Motor4.Target_Speed=(int)(Chassis_Motor4.Target_Speed*MAX_MOTOR_SPEED*1.0/max);
//		}
	//set_chassis_speed(Chassis_Motor1.Target_Speed, Chassis_Motor2.Target_Speed, Chassis_Motor3.Target_Speed, Chassis_Motor4.Target_Speed, Chassis_MotorA.Target_Speed, Chassis_MotorB.Target_Speed, Chassis_MotorC.Target_Speed, Chassis_MotorD.Target_Speed );
        
}	

static float chassis_follow(void)
{
	chassis_center.pid.position_loop.apid.Target_Angle=GIMBAL_HEAD_ANGLE;
	chassis_center.pid.position_loop.apid.actual_angle=chassis_center.actual_angle;
	follow_pid_realize();
	return (float)chassis_center.pid.position_loop.apid.PID_OUT;
}

STEPSTAR step_flag;
float K_VX,K_VY,B_VX,B_VY; //分别代表K和B 一次函数
int step_times_x=0,step_times_y=0; //时间
float TIME_LIMIT_X=180,TIME_LIMIT_Y=280; //斜坡的时间 
int STEP_VALUE=50; //差值大于step_value就用斜坡
//斜坡函数状态判断
void step_flag_judge(float VX_,float VY_,float LAST_VX_,float LAST_VY_)
{
	if(step_flag==NO_STEP)
	{
		if(abs(VX_-LAST_VX_)>STEP_VALUE) step_flag=X_STEP;
	    else if(abs(VY_-LAST_VY_)>STEP_VALUE*0.4) step_flag=Y_STEP;
		return;
	}
	if(step_flag==X_STEP)
	{
		if(step_times_x>TIME_LIMIT_X)
	    {
		    step_times_x=0;
		    step_flag=NO_STEP;
			return;
	    }
		 if(abs(VX_)<=1.0f) 
	     {
		    step_flag=NO_STEP;
		    step_times_x=0;
			return;
	     }
		if(abs(VY_-LAST_VY_)>STEP_VALUE&&abs(VY_)>90) 
		{
			step_flag=XY_STEP;
		}
		return;
	}
	
	if(step_flag==Y_STEP)
	{
		if(step_times_y>TIME_LIMIT_Y)
	    {
		    step_times_y=0;
		    step_flag=NO_STEP;
			return;
	    }
		 if(abs(VY_)<=1.0f) 
	     {
		    step_flag=NO_STEP;
		    step_times_y=0;
			return;
	     }
		if(abs(VX_-LAST_VX_)>STEP_VALUE&&abs(VX_)>90) 
		{
			step_flag=XY_STEP;
		}
		return;
	}
	
	if(step_flag==XY_STEP)
	{
		if(step_times_y>TIME_LIMIT_Y &&step_times_x>TIME_LIMIT_X)
	    {
		    step_times_y=0;
			step_times_x=0;
		    step_flag=NO_STEP;
			return;
	    }
		if(step_times_x>TIME_LIMIT_X)
	    {
		    step_times_x=0;
		    step_flag=Y_STEP;
			return;
	    }
		if(step_times_y>TIME_LIMIT_Y)
	    {
		    step_times_y=0;
		    step_flag=X_STEP;
			return;
	    }
		 if(abs(VY_)<=1.0f&&abs(VX_)<=1.0f) 
	     {
		    step_flag=NO_STEP;
		    step_times_y=0;
			 step_times_x=0;
			 return;
	     }
		 if(abs(VY_)<=1.0f) 
	     {
		    step_flag=X_STEP;
		    step_times_y=0;
	     }
		 if(abs(VX_)<=1.0f) 
	     {
		    step_flag=Y_STEP;
		    step_times_x=0;
	     }
		return;
	}
	
	
}

void step_star(float *VX_,float *VY_,float LAST_VX_,float LAST_VY_)
{
	step_flag_judge(*VX_,*VY_,LAST_VX_,LAST_VY_);

	if(step_flag==NO_STEP)  return;

	
	if(step_flag==X_STEP)
	{
		step_times_x++;
		if(step_times_x<=1)
		{
			K_VX=(*VX_-LAST_VX_)/TIME_LIMIT_X;
			B_VX=LAST_VX_;
		}
		
		*VX_=(float)(K_VX*(float)step_times_x)+B_VX;
		
	}
	if(step_flag==Y_STEP)
	{
		step_times_y++;
		if(step_times_y<=1)
		{
			K_VY=(*VY_-LAST_VY_)/TIME_LIMIT_Y;
			B_VY=LAST_VY_;
		}
		
		*VY_=K_VY*(float)step_times_y+B_VY;
	}
	if(step_flag==XY_STEP)
	{
		step_times_y++;
		if(step_times_y<=1)
		{
			K_VY=(*VY_-LAST_VY_)/TIME_LIMIT_Y;
			B_VY=LAST_VY_;
		}
		step_times_x++;
		if(step_times_x<=1)
		{
			K_VX=(*VX_-LAST_VX_)/TIME_LIMIT_X;
			B_VX=LAST_VX_;
		}
		
		*VX_=(float)(K_VX*(float)step_times_x)+B_VX;
		
		*VY_=K_VY*(float)step_times_y+B_VY;
	}
}


int Time_Cnt;
/***********************************************************
*@fuction	:Target_Velocity_Smoothen
*@brief		:
*@param		:--
*@return	:void
*@author	:--
*@date		:2023-04-05
***********************************************************/

int Target_Velocity_Smoothen(int Target_Speed, int Current_Speed, int Smoothen_Method, int Power_Mode)
{
	Time_Cnt++;
	int Delta;
	switch (Smoothen_Method)
	{
		//匀加速模式
		case Uniform_Acceleration:
			Delta = Acceleration;
			//不能影响刹车性能
			if (Target_Speed != 0)
			{
				if (Target_Speed > Current_Speed)
					if (Target_Speed - Current_Speed > Delta)
						Target_Speed = Current_Speed + Delta;
				if (Target_Speed < Current_Speed)
					if (Current_Speed - Target_Speed > Delta)
						Target_Speed = Current_Speed - Delta;
			}
			return Target_Speed;
		break;
		//缓出模式
		case Ease_Out:
			//不能影响刹车性能
			if (Target_Speed != 0)
			{
				//分段选择缓动参数
				float Parameter_C; //范围0~1，越接近1加速越快
				if (Power_Mode == High_Voltage_Mode)
				{
					if (abs(Target_Speed - Current_Speed) < 500)
						Parameter_C = 0.03;
					else Parameter_C = 0.17 ;
				}
				if (Power_Mode == Supercap_Disconnected_Mode)
				{
					if (abs(Target_Speed - Current_Speed) < 500)
						Parameter_C = 0.05;
					else Parameter_C = 0.25 ;
				}
				if (Power_Mode == Low_Voltage_Mode)
				{
					Parameter_C = 0.05;
				}
				//缓出公式
				Delta = (Target_Speed - Current_Speed)*Parameter_C;
				
				return Current_Speed+Delta;
			}
			return Target_Speed;
		break;
		//不进行缓动调节
		case Smoothen_Off:
			return Target_Speed;
		break;
	}

}

float vx,vy,wz;
float yu;
static void chassis_move_mode(void)//对应模式下的底盘动作
{
	
	vx=(float)chassis_control_order.vx_set;
	vy=(float)chassis_control_order.vy_set;
	wz=(float)chassis_control_order.wz_set;
//	vx=yu;
	avx=vx;
	avy=vy;
//	step_star(&avx,&avy,last_vx,last_vy);
//	if(step_flag!=NO_STEP)
//	{
//		vx=avx;
//	    vy=avy;
//	}
//	chassis_control_order.chassis_mode=CHASSIS_REMOTE_CLOSE;

	if(chassis_control_order.chassis_mode==CHASSIS_REMOTE_CLOSE)
	{
	}
  else if(chassis_control_order.chassis_mode==CHASSIS_NORMAL)
	{
		chassis_spin(&vx,&vy);
		wz = -1.0f*chassis_follow();
	}
	else if(chassis_control_order.chassis_mode==CHASSIS_SPIN)
	{
		chassis_spin(&vx,&vy);
		//wz=1.5;
	}
	last_vx=(float)chassis_control_order.vx_set;
//	last_vx=yu;
	last_vy=(float)chassis_control_order.vy_set;
	//chassis_speed_control(vx,vy,wz);
}

int flag1=0;
static void can_send_chassis_current(void)//输出底盘电流值
{
	flag1=1;
    canTX_Chassis_Motor_Current();
    //canTX_AGV_Chassis_Motor_Current();
}

float theta;       
void chassis_spin(float *vx,float *vy) 
{
	theta=Get_chassis_theta();   
	*vx = (float)(avy*sin(theta) + avx*cos(theta));    
	*vy = (float)(avy*cos(theta) - avx*sin(theta));    
}

static float Get_chassis_theta(void)//读取底盘相对云台转角（需要陀螺仪）
{
	float temp,angle;
	if(chassis_center.actual_angle<chassis_center.switch_mode_angle)
		angle=chassis_center.actual_angle+360.0f;
	else
		angle=chassis_center.actual_angle;
	temp=angle-chassis_center.switch_mode_angle;	
	angle=temp/360.0f*2*PI;
	return angle;
}

/**
  * @breif         底盘功率限制
  * @param[in]     none 
	* @param[out]    输出限制后的四个电机电流值
  * @retval        none     
  */
float current_scale,BUFFER_MAX=60.0f,POWER_TOTAL_CURRENT_LIMIT=9000.0f;
float temp3,temp1,temp2,speed1,speed2,speed3,speed4,total_current_limit,total_current,power,last_power;
float power_scale,buffer_scale;
uint8_t fly_flag;
static void power_limitation_jugement(void)
{
    total_current=0;
	last_power=power;
	get_chassis_power_and_buffer_and_max(&power,&buffer,&max_power);
	canTX_RefereeData(&power,&buffer,&max_power);
	

	power_scale=chassis_power_loop(max_power-8,power,last_power);
	buffer_scale=chassis_buffer_loop(buffer);
	temp1=Chassis_Motor1.pid.speed_loop.vpid.Target_Speed*buffer_scale*power_scale;
	Chassis_Motor1.pid.speed_loop.vpid.Target_Speed=(int16_t)temp1;
	temp1=Chassis_Motor2.pid.speed_loop.vpid.Target_Speed*buffer_scale*power_scale;
	Chassis_Motor2.pid.speed_loop.vpid.Target_Speed=(int16_t)temp1;
	temp1=Chassis_Motor3.pid.speed_loop.vpid.Target_Speed*buffer_scale*power_scale;
	Chassis_Motor3.pid.speed_loop.vpid.Target_Speed=(int16_t)temp1;
	temp1=Chassis_Motor4.pid.speed_loop.vpid.Target_Speed*buffer_scale*power_scale;
	Chassis_Motor4.pid.speed_loop.vpid.Target_Speed=(int16_t)temp1;
//	if(power>max_power*2)CHASSIS_vPID_max=5000;
//	else if(power<max_power*1.6)CHASSIS_vPID_max=8000;
	if(buffer<BUFFER_MAX*0.15f) CHASSIS_vPID_max=500;
	if(buffer<BUFFER_MAX*0.25f&& buffer>=BUFFER_MAX*0.15f) CHASSIS_vPID_max=900;
	if(buffer<BUFFER_MAX*0.36f&&CHASSIS_vPID_max<=8000) CHASSIS_vPID_max=4000;  //原来是1200
	if(CHASSIS_vPID_max<=1900 && buffer>=BUFFER_MAX*0.55f ) CHASSIS_vPID_max=1600;
	if(CHASSIS_vPID_max<=2100 && buffer>=BUFFER_MAX*0.75f ) CHASSIS_vPID_max=3200;
	if(CHASSIS_vPID_max<=4500 && buffer>=BUFFER_MAX*0.96f ) CHASSIS_vPID_max=8000;
}
/**
  * @breif         底盘功率环函数
  * @param[in]     target_power：设定的目标值
	* @param[in]     target_power：返回的真实值  
	* @param[in]     last_power：上一次返回的真实值
	* @param[out]    四个电机的输出电流
  * @retval        none     
  */
static float chassis_power_loop(uint16_t target_power,float actual_power,float last_power)
{
	float temp;
	p_pid.target_power=(float)target_power;
	p_pid.actual_power=actual_power;
	//此处进行pid运算
	power_pid_realize(&p_pid);
	//此处计算比例系数
	temp=1.07f+((float)p_pid.PID_OUT/1000);
//	if(temp>1.2f)  temp-=0.2f;
//	temp=temp/2.0*0.45f+0.1f;
	temp*=0.7f;

	return temp;
}

static float chassis_buffer_loop(uint16_t buffer)
{
	float temp;
	b_pid.target_buffer=50;
	b_pid.actual_buffer=buffer;
	buffer_pid_realize(&b_pid);
	temp=1.07f-((float)b_pid.PID_OUT/1000.0f);
	temp*=0.75f;
//	if(temp>1.2f)  temp-=0.2f;

	return temp;
}
/**
  * @breif         底盘飞坡函数，防止因飞坡后缓冲能量用完
  * @param[in]     buffer：底盘缓冲能量
	* @param[out]    四个电机的输出电流
  * @retval        none     
  */
static void chassis_fly(uint16_t buffer)
{
	if(buffer<20)
	{
		Chassis_Motor1.pid.speed_loop.vpid.PID_OUT*=0.5f;
		Chassis_Motor2.pid.speed_loop.vpid.PID_OUT*=0.5f;
		Chassis_Motor3.pid.speed_loop.vpid.PID_OUT*=0.5f;
		Chassis_Motor4.pid.speed_loop.vpid.PID_OUT*=0.5f;
	}
}
