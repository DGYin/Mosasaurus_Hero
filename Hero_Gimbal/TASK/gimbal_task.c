/**
  ************************************* Copyright ******************************
  *
  *                 (C) Copyright 2022,Hebei,China, NEUQRM.
  *                            All Rights Reserved
  *
  *                     By DGYin,
  *                     https://--
  *
  * FileName   : gimbal_task.c
  * Version    : v1.0
  * Author     : NEUQRM, DGYin,
  * Date       : 2022-11-21
  * Description: ��̨����
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
//#include "*.h"
#include "gimbal_task.h"
#include "vision_task.h"
#include "gimbal_task_behaviour.h"
#include "sent_task.h"
#include "bsp_imu.h"
#include "can_receive.h"
#include "RC_task.h"
#include "lk_pitch_turn.h"
#include "gimbal_calibration_task.h"
/*define-----------------------------------------------------------------------*/
#define yaw_angle gimbal_y.add_angle
#define pitch_angle gimbal_p.add_angle

/*variate----------------------------------------------------------------------*/
GIMBAL_t gimbal_y, gimbal_p;
GIMBAL_MODE_t gimbal_set_mode;

//����������ÿһλ�ֱ��Ӧ	   KP��		KI��	KD��  MAX_OUT�� MAX_IOUT��deadband
float YawGyroPid[6] 		= {1.0f,	0.2f,	0.1f, 500.0f,   0.0f,   0.0f};	//Yaw imu�ǶȻ�
float YawEncondePid[6] 		= {25.0f,	1.0f,	0.1f, 500.0f,   100.0f,	0.005};	//Yaw �������ǶȻ�

float YawGyroPIDSpeed[5]    = {3000.0f,	70.0f,	0.0f, 15000.0f,	7500.0f};	//imu�ٶȻ�
float YawEncondePidSpeed[5] = {170.0f,	0.2f,	0.0f, 15000.0f,	7500.0f};   //�������ٶȻ�
float YawSpeedPid[5] 		= {2100.0f,	300.0f,	0.0f, 13000.0f,	7000.0f};     //�ٶȻ�int test_set_speed = 10;

float PitchGyroPid[6] 		  = {20.f,		40.0f,			0.0f,		240.0f,			0.0f,	0.07f};  	//imu�ǶȻ�
float PitchEncondePid[6] 	  = {0.1f,		0.0f,			0.001f,		100.0f,		0.0f,		0.0f}; //�������ǶȻ�
float PitchEncondePidSpeed[6] = {0.0f,	0.0f,		0.0f,	0.0f,	0.0f, 	0.0f}; //�������ٶȻ�
//float PitchSpeedPid[6] 	  = {0.2f,		0.03f,			0.8f,		12.0f,	8.0f, 0.0f};    //
float PitchSpeedPid[6] 		  = {12000.0f,		500.0f,			5000.0f,		3000.0f,	700.0f,		0.2f};    //8900.0f,		320.0f,			0.0f

//ԭ6020pitch��������
//float PitchGyroPid[5] 				= {10.0f,		5.0f,			0.0f,		20.0f,			1.0f};  	//imu�ǶȻ�
//float PitchEncondePid[5] 			= {10.0f,		5.0f,			1.0f,		500.0f,		5000.0f}; //�������ǶȻ�
//float PitchEncondePidSpeed[5] = {3000.0f,	100.0f,		10.0f,	30000.0f,	5000.0f}; //�������ٶȻ�
//float PitchSpeedPid[5] 				= {380.0f,		220.0f,			150.0f,		15000.0f,	9500.0f};    //

int Gimbal_Precision_Mode = 0, Last_Gimbal_Precision_Mode;
int Gimbal_Precision_Activated_Flag = 0, Gimbal_Precision_Inactivated_Flag = 0;
int gimbal_imu_cnt = 0;
int yu = 0; //���Ա���
float xuan = 0;

/*statement--------------------------------------------------------------------*/
//PID��ʼ��
static void YawPitch_PIDinit(void);
//��̨���ģʽѡ��
static void GIMBAL_Set_Mode(void);
//��̨����
static void GIMBAL_Set_Contorl(void);
//PID����
static void GIMBAL_PID(void);
//IMU���ݽ���
static void GIMBAL_CALBACK_GET(void);
//�ٶ�PID
static void gimbal_motor_raw_pid(GIMBAL_t *gimbal_);
static void Yaw_Gyro_PID(GIMBAL_t *gimbal_);
static void Yaw_Encoder_PID(GIMBAL_t *gimbal_);
void Pitch_Gyro_PID(GIMBAL_t *gimbal_);
void Pitch_Encoder_PID(GIMBAL_t *gimbal_);

/*Function prototype Begin*****************************************************/


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
 * ********************      				 			 ********************
 * ********************         ���汣�� ��Զ��BUG        ********************
 * ********************                                  ********************
 * **************************************************************************
 *         .............................................
 *                  ������¥                  BUG����
 *          ��Ի:
 *                  д��¥��д�ּ䣬д�ּ������Ա��
 *                  ������Աд�������ó��򻻾�Ǯ��
 *                  ����ֻ���������������������ߣ�
 *                  ��������ո��գ����������긴�ꡣ
 *                  ��Ը�������Լ䣬��Ը�Ϲ��ϰ�ǰ��
 *                  ���۱������Ȥ���������г���Ա��
 *                  ����Ц��߯��񲣬��Ц�Լ���̫����
 *                  ��������Ư���ã��ĸ���ó���Ա��
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
void Gimbal_Task(int S_Cnt, int MS_Cnt)
{
	if (MS_Cnt % 4==0 && Motor_Alive_Flag > -5) Motor_Alive_Flag--;
	if (Motor_Alive_Flag > 5) Motor_Alive_Flag = 5;
	//��ȡPitch�������Ƕ�
	Get_Pitch_Motor_SingleRound_Angle();
	//forѭ����֤겿ص�����ᶪ֡
	for (int i=0; i<11000; i++)
	i=i;
	//����У׼ʱ
	if (Gimbal_Calibration_Target_Times == Gimbal_Calibration_Times && MS_Cnt%7==0)
	{
		extern int Gimbal_Precision_Mode, Last_Gimbal_Precision_Mode;
		//�����л���־
		if (Last_Gimbal_Precision_Mode == 0&&Gimbal_Precision_Mode==1) Gimbal_Precision_Activated_Flag = 1;
		if (Last_Gimbal_Precision_Mode == 1&&Gimbal_Precision_Mode==0)Gimbal_Precision_Inactivated_Flag = 1;
		Last_Gimbal_Precision_Mode = Gimbal_Precision_Mode;
		
		if (MS_Cnt==7) Get_Pitch_Motor_Error_Status();//ÿ���ȡһ��Pitch����¶�
		
		Pitch_Motor_Model = MOTOR_LKTECH;//ѡ��Pitch����ͺ�
		GIMBAL_CALBACK_GET(); //����������
		GIMBAL_Set_Mode();//ģʽѡ��
		GIMBAL_Set_Contorl();//ģʽ����
		GIMBAL_PID();//PID����
		//Pitch�Ƕ����ƣ���ֹ��̨�Ƕȹ����ƻ���е�ṹ
		if (Gimbal_Precision_Mode == 0)
		{
			if(gimbal_p.IMU_actual_angle <= -33.f)
			{
				gimbal_p.target_angle = -33.f;
				gimbal_p.target_speed = 0;
				if (gimbal_p.gimbal_gyro_pid.out>0) gimbal_p.gimbal_gyro_pid.out= 100; //���Ŀ����̧ͷ��Ҫ��ͷ
			}
			if(gimbal_p.IMU_actual_angle >= 22)
			{
				gimbal_p.target_angle = 22;
				gimbal_p.target_speed = 0;
				if (gimbal_p.gimbal_gyro_pid.out<0) gimbal_p.gimbal_gyro_pid.out= -100; //���Ŀ���ǵ�ͷ��Ҫ̧ͷ
			
			}
		}

		else if (Gimbal_Precision_Mode)
		{
			if(gimbal_p.IMU_actual_angle <= -33.f || gimbal_p.IMU_actual_angle >= 22)
				Gimbal_Precision_Mode = 0;
			//�����л���־
			if (Last_Gimbal_Precision_Mode == 0&&Gimbal_Precision_Mode==1)	Gimbal_Precision_Activated_Flag = 1;
			if (Last_Gimbal_Precision_Mode == 1&&Gimbal_Precision_Mode==0)	Gimbal_Precision_Inactivated_Flag = 1;
			Last_Gimbal_Precision_Mode = Gimbal_Precision_Mode;

		}
		//��̨������ݷ���
		if (gimbal_set_mode != GIMBAL_ZERO_FORCE)//������ģʽ
		{
			if (Gimbal_Precision_Mode == 0)
			{
				canTX_Yaw_Current(gimbal_y.given_current);
				Send_Pitch_Motor_Add_Angle(-gimbal_p.gimbal_gyro_pid.out);//����������Ƕȵ�ͷ�Ǽ���̧ͷ�Ǽӣ���IMU�෴
			}
			else 
			{
				canTX_Yaw_Current(gimbal_y.given_current);
				Send_Pitch_Motor_Add_Angle(gimbal_p.gimbal_enconde_pid.out);
			}
			
		}
	}
	//��Ҫ����У׼
	else if (Gimbal_Calibration_Target_Times > Gimbal_Calibration_Times && MS_Cnt%7==0) 
	{
		Gimbal_Calibration_Task(S_Cnt, MS_Cnt);
	}
	//Send_Pitch_Motor_Add_Angle(1000);
	//Send_Pitch_Motor_Start_Instruction();
	//Send_Pitch_Motor_Target_Speed(-100000); //debug��
}
/*
	* @ brief       ��̨��ʼ������
	* @ param				none
	* @ retvel      none
*/
void Gimbal_Init(void)
{
	
    YawPitch_PIDinit(); //PID��ʼ��
    gimbal_set_mode = GIMBAL_ABSOLUTE_ANGLE;
    gimbal_y.IMU_actual_angle = 0.0f;
    gimbal_y.IMU_actual_speed = 0.0f;
    gimbal_y.CAN_actual_angle = 0.0f;
    gimbal_y.CAN_actual_speed = 0.0f;
    gimbal_y.target_angle = 0.0f;
    gimbal_y.target_speed = 0.0f;
    gimbal_y.add_angle = 0.0f;
    gimbal_y.given_current = 0;
	gimbal_y.Zero_Position = 66.0f;
	gimbal_y.Valuence_Invert_Flag = -1;

    gimbal_y.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE; //GIMBAL_MOTOR_RAW GIMBAL_MOTOR_GYRO

    gimbal_p.IMU_actual_angle = 0.0f;
    gimbal_p.IMU_actual_speed = 0.0f;
    gimbal_p.CAN_actual_speed = 0.0f;

    gimbal_p.CAN_actual_angle = 0.0f;
    gimbal_p.target_angle = 0.0f; //(RC_PITCH_ANGLE_MAXX + RC_PITCH_ANGLE_MINN)/2.0f;  //��ʼʱ��pitch��������
    gimbal_p.target_speed = 0.0f;
    gimbal_p.add_angle = 0.0f;
    gimbal_p.given_current = 0;

    gimbal_p.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
}

/*
	* @ brief       yaw��pitch���ʼ��
	* @ param				none
	* @ retvel      none
*/
static void YawPitch_PIDinit(void)
{
    PID_Init(&gimbal_y.gimbal_raw_pid, YawSpeedPid);
    PID_Init(&gimbal_y.gimbal_gyro_pid, YawGyroPid);
	PID_Init(&gimbal_y.gimbal_gyro_pid_speed, YawGyroPIDSpeed);
    PID_Init(&gimbal_y.gimbal_enconde_pid, YawEncondePid);
    PID_Init(&gimbal_y.gimbal_enconde_pid_speed, YawEncondePidSpeed);

    PID_Init(&gimbal_p.gimbal_raw_pid, PitchSpeedPid);
    PID_Init(&gimbal_p.gimbal_gyro_pid, PitchGyroPid);
    PID_Init(&gimbal_p.gimbal_enconde_pid, PitchEncondePid);
    PID_Init(&gimbal_p.gimbal_enconde_pid_speed, PitchEncondePidSpeed);
}

/*
	* @ brief       ��̨ģʽѡ��
	* @ param				none
	* @ retvel      none
*/
extern int Gimbal_Zero_Force_Flag;
static void GIMBAL_Set_Mode(void)
{
    if (gimbal_set_mode == NULL)
    {
        return;
    }
	Gimbal_Zero_Force_Flag = 0;
    //���ģʽѡ��
    if(gimbal_set_mode == GIMBAL_INIT)
    {
        gimbal_y.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
        gimbal_p.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    }
    else if(gimbal_set_mode == GIMBAL_ZERO_FORCE)
    {
		Gimbal_Zero_Force_Flag = 1;
        gimbal_y.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_p.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if(gimbal_set_mode == GIMBAL_ABSOLUTE_ANGLE)
    {
        //
        gimbal_y.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_p.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    }
    else if(gimbal_set_mode == GIMBAL_TOP_ANGLE) //С���ݣ�������ת����̨���ֽǶ�
    {

        gimbal_y.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
        gimbal_p.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    }
    else if(gimbal_set_mode == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_y.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_p.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    }
    else if(gimbal_set_mode == GIMBAL_CALI) //У׼ģʽ
    {
        gimbal_y.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
        gimbal_p.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    }
    //	gimbal_p.gimbal_motor_mode=GIMBAL_MOTOR_RAW; //����ʱѡ��
}

/*
	* @ brief       ��̨��������
	* @ param				none
	* @ retvel      none
*/
static void GIMBAL_Set_Contorl(void)
{
    Vision_Task();

    if(vision_mode == VISION_OFF)
    {
		if (Gimbal_Precision_Mode)
		{
			gimbal_y.target_angle += gimbal_y.add_angle/6.0f;
			gimbal_p.target_angle += gimbal_p.add_angle*300.0f;
		}
		else 
		{
			gimbal_y.target_angle += gimbal_y.add_angle;
			gimbal_p.target_angle += gimbal_p.add_angle;
		}
    }
    else
    {
        gimbal_y.target_angle = gimbal_y.add_angle;
        gimbal_p.target_angle = gimbal_p.add_angle;
    }
//    if(gimbal_set_mode != GIMBAL_RELATIVE_ANGLE)
//    {
//        if(gimbal_y.target_angle > 180)
//            gimbal_y.target_angle -= 360;
//        if(gimbal_y.target_angle < -180)
//            gimbal_y.target_angle += 360;
//    }//360���ƵĽǶ�

}
/*
	* @ brief       ��̨��������
	* @ param				none
	* @ retvel      none
*/
static void GIMBAL_CALBACK_GET(void)
{
    //Yaw������ݴ���
//    gimbal_y.CAN_actual_angle = (float)yaw_can_rx.angle;
//    gimbal_y.CAN_Total_Angle = (float)yaw_can_rx.turns * 360.0f + gimbal_y.CAN_actual_angle;		//����yaw��total_angle
    gimbal_y.CAN_actual_speed = (float)yaw_can_rx.speed;

    //Yaw��IMU���ݴ���
	gimbal_y.IMU_Last_Actual_Angle = gimbal_y.IMU_actual_angle;
    gimbal_y.IMU_actual_angle = INS_angle[0] / (2 * 3.141590f) * 360.0f;
	if (gimbal_y.IMU_actual_angle - gimbal_y.IMU_Last_Actual_Angle > 180.f) gimbal_y.IMU_Round_Cnt--;
	if (gimbal_y.IMU_actual_angle - gimbal_y.IMU_Last_Actual_Angle < -180.f) gimbal_y.IMU_Round_Cnt++;
	gimbal_y.IMU_Total_Angle = gimbal_y.IMU_Round_Cnt*360.f + gimbal_y.IMU_actual_angle;
    gimbal_y.IMU_actual_speed = INS_gyro[2];

    if(fabs(gimbal_y.IMU_actual_speed) < 0.01f)
        gimbal_y.IMU_actual_speed = 0;
    //Pitch��3508���������
    if (Pitch_Motor_Model == MOTOR_3508)
    {
        gimbal_p.CAN_actual_angle = pitch_can_rx.angle / 8191.0f * 360.0f;
        gimbal_p.CAN_actual_speed = pitch_can_rx.speed;
    }
    //Pitch��겿ص��������
    if (Pitch_Motor_Model == MOTOR_LKTECH)
    {
        gimbal_p.CAN_actual_angle = LK_Pitch_Motor.Encoder_Position / 16383.0f * 360.0f;
        gimbal_p.CAN_actual_speed = LK_Pitch_Motor.Speed;
    }
    //Pitch��IMU���ݴ���
    gimbal_p.IMU_actual_angle = 1.0f * INS_angle[1] / (2 * 3.141590f) * 360.0f;
    gimbal_p.IMU_actual_speed = -1.0f * INS_gyro[0] + 0.0069813f; ///(2*3.141590f)*360.0f +0.4f;
}


/*
	* @ brief       ��̨pid����
	* @ param				none
	* @ retvel      none
*/

float Yaw_Target_Angle, Pitch_Target_Angle;
float Yaw_IMU_Angle, Pitch_IMU_Angle;
float Yaw_IMU_Speed, Pitch_IMU_Speed;
static void GIMBAL_PID(void)
{
	//���ݶ�ȡ
	if (Gimbal_Precision_Activated_Flag)
	{
		gimbal_y.target_angle = gimbal_y.CAN_Total_Angle;
		gimbal_p.target_angle = 18.25/360.f*216000;
		//gimbal_p.target_angle = 18.25/360.f*216000 - Gimbal_Encoder_Horizontal_Angle;
//		while (gimbal_p.target_angle - LK_Pitch_Motor.Total_Angle < -216000) gimbal_p.target_angle = gimbal_p.target_angle+216000;
//		while (gimbal_p.target_angle - LK_Pitch_Motor.Total_Angle >  216000) gimbal_p.target_angle = gimbal_p.target_angle-216000;
		Gimbal_Precision_Activated_Flag = 0;
	}
	if (Gimbal_Precision_Inactivated_Flag)
	{
		gimbal_y.target_angle = -gimbal_y.IMU_actual_angle + 180*gimbal_y.Bool_Invert_Flag ;
		//gimbal_p.target_angle = gimbal_p.IMU_actual_angle;
		gimbal_p.target_angle = 0;
		Gimbal_Precision_Inactivated_Flag = 0;
	}
	/**************************/
    /****  Yaw���PID����  ****/
	/**************************/
	//����ע��
	Yaw_Target_Angle = gimbal_y.target_angle;// + gimbal_y.Zero_Position;
	Yaw_IMU_Angle = -gimbal_y.IMU_Total_Angle + 180*gimbal_y.Bool_Invert_Flag;
	Yaw_IMU_Speed = -gimbal_y.IMU_actual_speed;
	Pitch_Target_Angle = gimbal_p.target_angle;
	Pitch_IMU_Angle = gimbal_p.IMU_actual_angle;
	Pitch_IMU_Speed = gimbal_p.IMU_actual_speed;
	//����ģʽδ������ʹ��IMU���ݿ���
	if (Gimbal_Precision_Mode==0) 
	{
		/**************************/
		/**** Yaw���PID����  ****/
		/**************************/
		//���ӻ�����
		while(Yaw_Target_Angle - Yaw_IMU_Angle > 180) Yaw_Target_Angle -= 360;
		while(Yaw_Target_Angle - Yaw_IMU_Angle < -180) Yaw_Target_Angle += 360;
		Yaw_Gyro_PID(&gimbal_y);
		/**************************/
		/**** Pitch���PID���� ****/
		/**************************/
		Pitch_Gyro_PID(&gimbal_p);
	}
	else
	//����ģʽ������ʹ�ñ�����ģʽ
	{
		Pitch_Encoder_PID(&gimbal_p);
		Yaw_Encoder_PID(&gimbal_y);
	}

}

static void gimbal_motor_raw_pid(GIMBAL_t *gimbal_)
{
    PID_Calc(&gimbal_->gimbal_raw_pid, gimbal_->target_speed, gimbal_->IMU_actual_speed);
    gimbal_->given_current = gimbal_->gimbal_raw_pid.out;
}

//ʹ��IMU����ΪĿ��Ƕȵ�PID
static void Yaw_Gyro_PID(GIMBAL_t *gimbal_)
{
    PID_Calc(&gimbal_->gimbal_gyro_pid, Yaw_Target_Angle, Yaw_IMU_Angle);
    gimbal_->target_speed = gimbal_->gimbal_gyro_pid.out;
    PID_Calc(&gimbal_->gimbal_gyro_pid_speed, gimbal_->target_speed, Yaw_IMU_Speed);
    gimbal_->given_current = gimbal_->gimbal_gyro_pid_speed.out;
}

//ʹ�õ������������ΪĿ��Ƕȵ�PID
int TP1, TP2;
static void Yaw_Encoder_PID(GIMBAL_t *gimbal_)
{
	
	while (Yaw_Target_Angle - gimbal_->CAN_Total_Angle<-360.f)	Yaw_Target_Angle = Yaw_Target_Angle + 360.f;
	while (Yaw_Target_Angle - gimbal_->CAN_Total_Angle>360.f)	Yaw_Target_Angle = Yaw_Target_Angle - 360.f;
	TP1 = Yaw_Target_Angle; TP2 = gimbal_->CAN_Total_Angle;
    PID_Calc(&gimbal_->gimbal_enconde_pid, Yaw_Target_Angle, gimbal_->CAN_Total_Angle);   //pitch�ĵ���������Ϊgimbal_->CAN_actual_angle
	gimbal_->target_speed = gimbal_->gimbal_enconde_pid.out;
    PID_Calc(&gimbal_->gimbal_enconde_pid_speed, gimbal_->target_speed, gimbal_->CAN_actual_speed);
    gimbal_->given_current = gimbal_->gimbal_enconde_pid_speed.out;
}

void Pitch_Gyro_PID(GIMBAL_t *gimbal_)
{
    PID_Calc(&gimbal_->gimbal_gyro_pid, Pitch_Target_Angle, Pitch_IMU_Angle);
}

void Pitch_Encoder_PID(GIMBAL_t *gimbal_)
{
	PID_Calc(&gimbal_->gimbal_enconde_pid, Pitch_Target_Angle, LK_Pitch_Motor.Total_Angle);   //pitch�ĵ���������Ϊgimbal_->CAN_actual_angle
}




