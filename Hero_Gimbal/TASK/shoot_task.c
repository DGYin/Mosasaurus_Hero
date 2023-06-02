#include "shoot_task.h"
#include "remote_control.h"
#include "sent_task.h"

/*
   实现拨弹加发射功能
   左边  往上拨一下 发射N发
*/

shoot_task_t rc_shoot;

float TriggerSpeed[5] 	= {3.0f,			0.1f,			0.1f,				8000,			500};  //拨弹轮速度环
float TriggerAngle[5] 	= {0.4f,			0.0f,			0.00015f,		4000,			0.0f};  //拨弹轮角度环

//float FricLeftSpeed[5]	= {18.0f,			0.05f,			0.0f,				15000.0f,			4000.0f};  //适用训练弹丸的左摩擦轮速度环
//float FricRightSpeed[5]	= {18.0f,			0.05f,			0.0f,				15000.0f,			4000.0f};  //适用训练弹丸的右摩擦轮速度环
float FricLeftSpeed[5]	= {25.5f,			0.2f,			0.0f,				15000.0f,			4000.0f};  //【新摩擦轮，效果优】适用比赛的左摩擦轮速度环
float FricRightSpeed[5] = {25.5f,			0.2f,			0.0f,				15000.0f,			4000.0f};  //【新摩擦轮，效果优】适用比赛的右摩擦轮速度环
//float FricLeftSpeed[5]	= {15.5f,			0.2f,			0.0f,				15000.0f,			4000.0f};  //适用比赛的左摩擦轮速度环
//float FricRightSpeed[5] = {15.5f,			0.2f,			0.0f,				15000.0f,			4000.0f};  //适用比赛的右摩擦轮速度环

//摩擦轮PID计算
int LR_Error;

static void fric_pid(void);
static int16_t speed_counter=0;
extern int Shoot_Flag_For_Buzzer;
void shoot_task(void)
{
	fric_pid();
	canTX_fric(rc_shoot.left_fric.set_currunt,rc_shoot.right_fric.set_currunt);
	if(canTX_trigger(rc_shoot.trigger.target_angle))
		rc_shoot.trigger.target_angle = 0;
	Shoot_Flag_For_Buzzer = rc_shoot.trigger.target_angle;

}


void shoot_init(void)
{
	
	PID_Init(&rc_shoot.trigger.speed_pid,TriggerSpeed);
	PID_Init(&rc_shoot.trigger.angle_pid,TriggerAngle);
	
	PID_Init(&rc_shoot.left_fric.speed_pid,FricLeftSpeed);
	PID_Init(&rc_shoot.right_fric.speed_pid,FricRightSpeed);
	
	rc_shoot.trigger.actual_angle = 0.0f;
	rc_shoot.trigger.actual_speed = 0.0f;
	
	rc_shoot.trigger.last_shoot_flag = 0;
	
	rc_shoot.trigger.target_angle = 0.0f;
	rc_shoot.trigger.target_speed = 0.0f;
	
	rc_shoot.left_fric.actual_speed = 0.0f;
	rc_shoot.left_fric.target_speed = 0.0f;
	
	rc_shoot.right_fric.actual_speed = 0.0f;
	rc_shoot.right_fric.target_speed = 0.0f;
}



static void fric_pid(void)
{
	LR_Error = rc_shoot.left_fric.actual_speed + rc_shoot.right_fric.actual_speed;
	PID_Calc(&rc_shoot.left_fric.speed_pid,rc_shoot.left_fric.target_speed,rc_shoot.left_fric.actual_speed);
	rc_shoot.left_fric.set_currunt=rc_shoot.left_fric.speed_pid.out;
//	if (abs(rc_shoot.left_fric.target_speed-rc_shoot.left_fric.actual_speed)>500)
//	rc_shoot.left_fric.set_currunt = -15000;
	
	PID_Calc(&rc_shoot.right_fric.speed_pid,rc_shoot.right_fric.target_speed,rc_shoot.right_fric.actual_speed);
	rc_shoot.right_fric.set_currunt=rc_shoot.right_fric.speed_pid.out;
//	if (abs(rc_shoot.right_fric.target_speed-rc_shoot.right_fric.actual_speed)>500)
//	rc_shoot.right_fric.set_currunt = 15000;
}

void Trigger_Motor_Callback(trigger_t *motor,uint16_t angle, int16_t speed)
{
	motor->last_angle = motor->actual_angle;
	motor->actual_angle = angle;
	motor->actual_speed = 0.5*(speed + motor->last_speed);
	motor->last_speed = speed;

	if(motor->record_begin_angle_status==0)
	{
		motor->begin_angle = angle;
		motor->record_begin_angle_status++;
	}
	if(motor->actual_angle - motor->last_angle > 4096)
		motor->rounds --;
	else if (motor->actual_angle - motor->last_angle < -4096)
		motor->rounds ++;
	motor->total_angle = motor->rounds * 8192 + motor->actual_angle;

	
}

