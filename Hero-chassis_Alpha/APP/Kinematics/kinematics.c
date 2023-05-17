/**
  ************************************* Copyright ******************************
  *
  *                 (C) Copyright 2022,Hebei,China,NEUQRM.
  *                            All Rights Reserved
  *
  *                     By DGYin,
  *                     https://--
  *
  * FileName   : kinematics.c
  * Version    : v1.1.0-alpha.1
  * Author     : NEUQRM, DGYin,
  * Date       : 2022-11-07
  * Description: ���ļ��������˶�ѧ��ʽ���ɵ������ĺ��ĸ����ӵ��ٶ�֮��Ļ���ת��
  * Function List:
  	1. ....
			<version>				:
			<modify staff>	:
			<data>					:
			<description>		:
  	2. ...
  ******************************************************************************
 */

/*include----------------------------------------------------------------------*/
#include "kinematics.h"

/*define-----------------------------------------------------------------------*/


/*variate----------------------------------------------------------------------*/
Kinematics_t Kinematics;

/*statement--------------------------------------------------------------------*/


/*Function prototype Begin*****************************************************/

void Mecanum_Wheel_BaseVel_To_WheelVel(float linear_x, float linear_y, float angular_z);
void Mecanum_Wheel_Get_Base_Velocities(void);
void AGV_Vector_Composition_In_ChassisCoordinate(float linear_x, float linear_y, float angular_z, float Theta, float Motion_Angle);
void DirectiveMotor(float linear_x, float linear_y, float angular_z);
int find_max(void);
void Speed_Limitation(void);

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
 * ********************         ���汣�� ��Զ��BUG       ********************
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


/***************************************************************
 * @brief      ͨ�����̼������ĵ������ٶȼ������ֵ��ٶ�
 * @author     NEUQRM,
 * @version    1.0.0
 * @date       2022.1.1
 **************************************************************/
void Mecanum_Wheel_BaseVel_To_WheelVel(float linear_x, float linear_y, float angular_z)
{
	Kinematics.Wheel_1.Target_Speed.linear_vel = -linear_x - linear_y + angular_z * (half_width + half_length);
	Kinematics.Wheel_2.Target_Speed.linear_vel = -linear_x + linear_y - angular_z * (half_width + half_length);
	Kinematics.Wheel_3.Target_Speed.linear_vel = -linear_x + linear_y + angular_z * (half_width + half_length);
	Kinematics.Wheel_4.Target_Speed.linear_vel = -linear_x -  linear_y - angular_z * (half_width + half_length);    //���ٶ� cm/s  תת��  RPM
	Kinematics.Wheel_1.Target_Speed.rpm = Kinematics.Wheel_1.Target_Speed.linear_vel * VEL2RPM;
	Kinematics.Wheel_2.Target_Speed.rpm = Kinematics.Wheel_2.Target_Speed.linear_vel * VEL2RPM;
	Kinematics.Wheel_3.Target_Speed.rpm = Kinematics.Wheel_3.Target_Speed.linear_vel * VEL2RPM;
	Kinematics.Wheel_4.Target_Speed.rpm = Kinematics.Wheel_4.Target_Speed.linear_vel * VEL2RPM;
	Chassis_Motor1.Target_Speed = - (int)(Kinematics.Wheel_1.Target_Speed.rpm * M3508_REDUCTION_RATIO);
	Chassis_Motor2.Target_Speed =   (int)(Kinematics.Wheel_2.Target_Speed.rpm * M3508_REDUCTION_RATIO);
	Chassis_Motor3.Target_Speed = - (int)(Kinematics.Wheel_3.Target_Speed.rpm * M3508_REDUCTION_RATIO);
	Chassis_Motor4.Target_Speed =  	(int)(Kinematics.Wheel_4.Target_Speed.rpm * M3508_REDUCTION_RATIO);

}

/***************************************************************
 * @brief      ͨ�����̼������ĵ������ٶȼ����������ǣ��������ٶ�
 * @author     NEUQRM, DGYin,
 * @version    1.0.0
 * @date       2022.11.6
 **************************************************************/


void AGV_Vector_Composition_In_ChassisCoordinate(float linear_x, float linear_y, float angular_z, float Theta, float Motion_Angle)
{
	//��������
	float Radian_Theta, Radian_Conversion_Angle;
	float X_Components, Y_Components, Sine, Cosine;
	float Gimbal_V, Chassis_Vr; 
	extern int Chassis_Angle;
	
	Gimbal_V = sqrt(linear_x * linear_x + linear_y * linear_y);//��̨����ϵ��ƽ���ٶ�
	Chassis_Vr = angular_z * (half_width + half_length);//�ṩ���̽��ٶȵĵ���ٶ�
	Radian_Theta = Theta/360.00f *(2.00f*3.14f);  //������ThetaΪ������̨��ԽǵĽǶ�ֵ��Ҫת��Ϊ�����ơ�
	Motion_Angle = Motion_Angle /81920.0f *(2.00f*3.14f); //������Motion_AngleΪ�������Ƕ��ƣ�Ҫת��Ϊ�����ơ����˱�����û���õ���
	
	/*ʸ���ϳ�*/
	//��̨�ٶȺ͵�����Խ�
	Radian_Conversion_Angle = Radian_Theta+atan2(linear_y,linear_x);
	while (Radian_Conversion_Angle >  3.14) Radian_Conversion_Angle  = Radian_Conversion_Angle - 2*3.14;
	while (Radian_Conversion_Angle < -3.14) Radian_Conversion_Angle = Radian_Conversion_Angle + 2*3.14;
	//������任
	Sine   = sin(Radian_Conversion_Angle); 
	Cosine = cos(Radian_Conversion_Angle); 
	//�޸�sin���������������������
	if (linear_y>0 && linear_x==0) Sine   = 1; if (linear_y<0 && linear_x==0) Sine   = -1;
	//if (linear_x>0 && linear_y==0) Cosine = 1; if (linear_x<0 && linear_y==0) Cosine = -1;
	
	/******Aת���******/
	//ʸ���ϳɺ�X��Y��������ֱ���㡣
	if (Radian_Conversion_Angle > 0)
		 Y_Components = Gimbal_V*  Float_Abs(Sine)  + Chassis_Vr;
	else Y_Components = Gimbal_V*(-Float_Abs(Sine)) + Chassis_Vr;
	
	if (Radian_Conversion_Angle < 3.1415/2 && Radian_Conversion_Angle > -3.1415/2)
		 X_Components = Gimbal_V*  Float_Abs(Cosine)  + Chassis_Vr;
	else X_Components = Gimbal_V*(-Float_Abs(Cosine)) + Chassis_Vr;
	//ʸ���ϳɺ���ʸ���ĽǶȡ�
	Chassis_MotorA.ChassisCoordinate_Angle = -atan2(Y_Components,X_Components)/3.1415/2* 8191*10;
	//���ɶ��������ʸ�����ȡ�
	Kinematics.Wheel_1.Target_Speed.linear_vel = Square(X_Components) + Square(Y_Components);
	Kinematics.Wheel_1.Target_Speed.linear_vel = sqrt(Kinematics.Wheel_1.Target_Speed.linear_vel);
	/*******************/
	/******Bת���******/
	//ʸ���ϳɺ�X��Y�������
	if (Radian_Conversion_Angle > 0)
		 Y_Components = Gimbal_V*  Float_Abs(Sine)  + Chassis_Vr;
	else Y_Components = Gimbal_V*(-Float_Abs(Sine)) + Chassis_Vr;
	
	if (Radian_Conversion_Angle < 3.1415/2 && Radian_Conversion_Angle > -3.1415/2)
		 X_Components = Gimbal_V*  Float_Abs(Cosine)  - Chassis_Vr;
	else X_Components = Gimbal_V*(-Float_Abs(Cosine)) - Chassis_Vr;
	//�����ϳɺ��������ĽǶȡ�
	Chassis_MotorB.ChassisCoordinate_Angle = -atan2(Y_Components,X_Components)/3.1415/2* 8191*10;
	//���ɶ���������������ȡ�
	Kinematics.Wheel_2.Target_Speed.linear_vel = Square(X_Components) + Square(Y_Components);
	Kinematics.Wheel_2.Target_Speed.linear_vel = sqrt(Kinematics.Wheel_2.Target_Speed.linear_vel);
	/*******************/
	/******Cת���******/
	//ʸ���ϳɺ�X��Y�������
	if (Radian_Conversion_Angle > 0)
		 Y_Components = Gimbal_V*  Float_Abs(Sine)  - Chassis_Vr;
	else Y_Components = Gimbal_V*(-Float_Abs(Sine)) - Chassis_Vr;
	
	if (Radian_Conversion_Angle < 3.1415/2 && Radian_Conversion_Angle > -3.1415/2)
		 X_Components = Gimbal_V*  Float_Abs(Cosine)  - Chassis_Vr;
	else X_Components = Gimbal_V*(-Float_Abs(Cosine)) - Chassis_Vr;
	//�����ϳɺ��������ĽǶȡ�
	Chassis_MotorC.ChassisCoordinate_Angle = -atan2(Y_Components,X_Components)/3.1415/2* 8191*10;
	//���ɶ���������������ȡ�
	Kinematics.Wheel_3.Target_Speed.linear_vel = Square(X_Components) + Square(Y_Components);
	Kinematics.Wheel_3.Target_Speed.linear_vel = sqrt(Kinematics.Wheel_3.Target_Speed.linear_vel);
	/*******************/
	/******Dת���******/
	//ʸ���ϳɺ�X��Y�������
	if (Radian_Conversion_Angle > 0)
		 Y_Components = Gimbal_V*  Float_Abs(Sine)  - Chassis_Vr;
	else Y_Components = Gimbal_V*(-Float_Abs(Sine)) - Chassis_Vr;
	
	if (Radian_Conversion_Angle < 3.1415/2 && Radian_Conversion_Angle > -3.1415/2)
		 X_Components = Gimbal_V*  Float_Abs(Cosine)  + Chassis_Vr;
	else X_Components = Gimbal_V*(-Float_Abs(Cosine)) + Chassis_Vr;
	//�����ϳɺ��������ĽǶȡ�
	Chassis_MotorD.ChassisCoordinate_Angle = -atan2(Y_Components,X_Components)/3.1415/2* 8191*10;
	//���ɶ���������������ȡ�
	Kinematics.Wheel_4.Target_Speed.linear_vel = Square(X_Components) + Square(Y_Components);
	Kinematics.Wheel_4.Target_Speed.linear_vel = sqrt(Kinematics.Wheel_4.Target_Speed.linear_vel);
	
	/*��ʸ���ϳ�ģʽ*/
//	Kinematics.Wheel_1.Target_Speed.linear_vel = Gimbal_V + Chassis_Vr;
//	Kinematics.Wheel_2.Target_Speed.linear_vel = Gimbal_V + Chassis_Vr;
//	Kinematics.Wheel_3.Target_Speed.linear_vel = Gimbal_V + Chassis_Vr;
//	Kinematics.Wheel_4.Target_Speed.linear_vel = Gimbal_V + Chassis_Vr;
//	Chassis_MotorA.ChassisCoordinate_Angle = Chassis_Angle;
//	Chassis_MotorB.ChassisCoordinate_Angle = Chassis_Angle;
//	Chassis_MotorC.ChassisCoordinate_Angle = Chassis_Angle;
//	Chassis_MotorD.ChassisCoordinate_Angle = Chassis_Angle;
	
	//���ٶȽ��ٶ�ת�������ٶ� cm/s  תת��  RPM
	Kinematics.Wheel_1.Target_Speed.rpm = Kinematics.Wheel_1.Target_Speed.linear_vel * VEL2RPM;
	Kinematics.Wheel_2.Target_Speed.rpm = Kinematics.Wheel_2.Target_Speed.linear_vel * VEL2RPM;
	Kinematics.Wheel_3.Target_Speed.rpm = Kinematics.Wheel_3.Target_Speed.linear_vel * VEL2RPM;
	Kinematics.Wheel_4.Target_Speed.rpm = Kinematics.Wheel_4.Target_Speed.linear_vel * VEL2RPM;

}
/***************************************************************
 * @brief      ͨ��ң�������ݺ���̨������Խǣ������Ե���Ϊ����ϵ���˶�����
 * @author     NEUQRM, DGYin,
 * @version    1.0.0
 * @date       2022.11.6
 **************************************************************/
float AGV_tan, AGV_ang;
float AGV_DirectiveMotor_RobotMotion_To_TargetStatus(float linear_x, float linear_y, float Theta)
{
	AGV_ang = -atan2(linear_y,linear_x)/3.1415/2* 8191*10;// - Theta/360.0f * 8191*10;//�������Ƕ�ֵ�ĽǶ�
    return AGV_ang;
}

/***************************************************************
 * @brief      ͨ��������������˶��������������ת������Ŀ��Ƕȡ�
 * @author     NEUQRM, DGYin,
 * @version    1.0.0
 * @date       2022.11.6
 **************************************************************/

void AGV_DirectiveMotor_TargetStatus_To_MotorAngle_In_ChassisCoordinate(int break_mode)
{
	//������ֵ�Ŀ��Ƕ�ֵ
	Chassis_MotorA.Target_Angle = Chassis_MotorA.ChassisCoordinate_Angle + Chassis_MotorA.Zero_Position ; 
	Chassis_MotorB.Target_Angle = Chassis_MotorB.ChassisCoordinate_Angle + Chassis_MotorB.Zero_Position ;
	Chassis_MotorC.Target_Angle = Chassis_MotorC.ChassisCoordinate_Angle + Chassis_MotorC.Zero_Position ;
	Chassis_MotorD.Target_Angle = Chassis_MotorD.ChassisCoordinate_Angle + Chassis_MotorD.Zero_Position ;
	
	//�Ż��ӻ�����
	while (Chassis_MotorA.Target_Angle - Chassis_MotorA.Total_Angle >= 8192*5)  Chassis_MotorA.Target_Angle = Chassis_MotorA.Target_Angle - 8192*10; while (Chassis_MotorA.Target_Angle - Chassis_MotorA.Total_Angle <= -8192*5) Chassis_MotorA.Target_Angle = Chassis_MotorA.Target_Angle + 8192*10;
	while (Chassis_MotorB.Target_Angle - Chassis_MotorB.Total_Angle >= 8192*5)  Chassis_MotorB.Target_Angle = Chassis_MotorB.Target_Angle - 8192*10; while (Chassis_MotorB.Target_Angle - Chassis_MotorB.Total_Angle <= -8192*5) Chassis_MotorB.Target_Angle = Chassis_MotorB.Target_Angle + 8192*10;
	while (Chassis_MotorC.Target_Angle - Chassis_MotorC.Total_Angle >= 8192*5)  Chassis_MotorC.Target_Angle = Chassis_MotorC.Target_Angle - 8192*10; while (Chassis_MotorC.Target_Angle - Chassis_MotorC.Total_Angle <= -8192*5) Chassis_MotorC.Target_Angle = Chassis_MotorC.Target_Angle + 8192*10;
	while (Chassis_MotorD.Target_Angle - Chassis_MotorD.Total_Angle >= 8192*5)  Chassis_MotorD.Target_Angle = Chassis_MotorD.Target_Angle - 8192*10; while (Chassis_MotorD.Target_Angle - Chassis_MotorD.Total_Angle <= -8192*5) Chassis_MotorD.Target_Angle = Chassis_MotorD.Target_Angle + 8192*10;

	
	//ɲ��ģʽ
	if (break_mode)
	{
		Chassis_MotorA.Target_Angle = 8192*10*315/360+ Chassis_MotorA.Zero_Position;
		Chassis_MotorB.Target_Angle = 8192*10*225/360+ Chassis_MotorB.Zero_Position;
		Chassis_MotorC.Target_Angle = 8192*10*135/360+ Chassis_MotorC.Zero_Position;
		Chassis_MotorD.Target_Angle = 8192*10*45/360 + Chassis_MotorD.Zero_Position;
		//�Ż��ӻ�����
		while (Chassis_MotorA.Target_Angle - Chassis_MotorA.Total_Angle >= 8192*5)  Chassis_MotorA.Target_Angle = Chassis_MotorA.Target_Angle - 8192*10; while (Chassis_MotorA.Target_Angle - Chassis_MotorA.Total_Angle <= -8192*5) Chassis_MotorA.Target_Angle = Chassis_MotorA.Target_Angle + 8192*10;
		while (Chassis_MotorB.Target_Angle - Chassis_MotorB.Total_Angle >= 8192*5)  Chassis_MotorB.Target_Angle = Chassis_MotorB.Target_Angle - 8192*10; while (Chassis_MotorB.Target_Angle - Chassis_MotorB.Total_Angle <= -8192*5) Chassis_MotorB.Target_Angle = Chassis_MotorB.Target_Angle + 8192*10;
		while (Chassis_MotorC.Target_Angle - Chassis_MotorC.Total_Angle >= 8192*5)  Chassis_MotorC.Target_Angle = Chassis_MotorC.Target_Angle - 8192*10; while (Chassis_MotorC.Target_Angle - Chassis_MotorC.Total_Angle <= -8192*5) Chassis_MotorC.Target_Angle = Chassis_MotorC.Target_Angle + 8192*10;
		while (Chassis_MotorD.Target_Angle - Chassis_MotorD.Total_Angle >= 8192*5)  Chassis_MotorD.Target_Angle = Chassis_MotorD.Target_Angle - 8192*10; while (Chassis_MotorD.Target_Angle - Chassis_MotorD.Total_Angle <= -8192*5) Chassis_MotorD.Target_Angle = Chassis_MotorD.Target_Angle + 8192*10;

	}
	//������ת�Ż�
	Directive_Motor_Angle_Optimize(&Chassis_MotorA);
	Directive_Motor_Angle_Optimize(&Chassis_MotorB);
	Directive_Motor_Angle_Optimize(&Chassis_MotorC);
	Directive_Motor_Angle_Optimize(&Chassis_MotorD);
	//��¼�ϴε�Ŀ��Ƕ�ֵ
	Chassis_MotorA.Last_Target_Angle = Chassis_MotorA.Target_Angle;
	Chassis_MotorB.Last_Target_Angle = Chassis_MotorB.Target_Angle;
	Chassis_MotorC.Last_Target_Angle = Chassis_MotorC.Target_Angle;
	Chassis_MotorD.Last_Target_Angle = Chassis_MotorD.Target_Angle;
	
//	Chassis_Motor1.Target_Speed = (int)(Kinematics.Wheel_1.Target_Speed.rpm* M3508_REDUCTION_RATIO)*1.0f;
//	Chassis_Motor2.Target_Speed = (int)(Kinematics.Wheel_2.Target_Speed.rpm* M3508_REDUCTION_RATIO)*1.0f;
//	Chassis_Motor3.Target_Speed = (int)(Kinematics.Wheel_3.Target_Speed.rpm* M3508_REDUCTION_RATIO)*1.0f;
//	Chassis_Motor4.Target_Speed = (int)(Kinematics.Wheel_4.Target_Speed.rpm* M3508_REDUCTION_RATIO)*1.0f;
	
	Chassis_Motor1.Target_Speed = (int)(Kinematics.Wheel_1.Target_Speed.rpm * M3508_REDUCTION_RATIO*Chassis_MotorA.Invert_Flag)*1.0f;
	Chassis_Motor2.Target_Speed = (int)(Kinematics.Wheel_2.Target_Speed.rpm * M3508_REDUCTION_RATIO*Chassis_MotorB.Invert_Flag)*1.0f;
	Chassis_Motor3.Target_Speed = (int)(Kinematics.Wheel_3.Target_Speed.rpm * M3508_REDUCTION_RATIO*Chassis_MotorC.Invert_Flag)*1.0f;
	Chassis_Motor4.Target_Speed = (int)(Kinematics.Wheel_4.Target_Speed.rpm * M3508_REDUCTION_RATIO*Chassis_MotorD.Invert_Flag)*1.0f;
}
void AAA_AGV_DirectiveMotor_TargetStatus_To_MotorAngle(MOTOR_t *Motor, float Target_Motion_Angle, int Spining_Mode)
{
	//������ֵ�Ŀ��Ƕ�ֵ
	Motor->Target_Angle = Target_Motion_Angle + Spining_Mode * 8192*10*315/360  + Motor->Zero_Position; 
	//�Ż��ӻ�����
	while (Motor->Target_Angle - Motor->Total_Angle >= 8192*5)  Motor->Target_Angle = Motor->Target_Angle - 8192*10;
	while (Motor->Target_Angle - Motor->Total_Angle <= -8192*5) Motor->Target_Angle = Motor->Target_Angle + 8192*10;
	//������ת�Ż�
	while (Motor->Target_Angle>8192*10) Motor->Target_Angle = Motor->Target_Angle - 8192*10;
	while (Motor->Target_Angle<0) 		Motor->Target_Angle = Motor->Target_Angle + 8192*10;
	while (Motor->Last_Target_Angle>8192*10) Motor->Last_Target_Angle = Motor->Last_Target_Angle - 8192*10;
	while (Motor->Last_Target_Angle<0) 		 Motor->Last_Target_Angle = Motor->Last_Target_Angle + 8192*10;
	if (Float_Abs(Motor->Target_Angle - Motor->Last_Target_Angle) > 8192*2.5f) 
	{
		Motor->Target_Angle = Motor->Target_Angle + 8192*5;
		Motor->Invert_Flag = -Motor->Invert_Flag;
	}
	while (Motor->Target_Angle - Motor->Total_Angle >= 8192*5)  Motor->Target_Angle = Motor->Target_Angle - 8192*10; 
	while (Motor->Target_Angle - Motor->Total_Angle <= -8192*5) Motor->Target_Angle = Motor->Target_Angle + 8192*10;
	//��¼�ϴε�Ŀ��Ƕ�ֵ
	Motor->Last_Target_Angle = Motor->Target_Angle;
	
}
/***************************************************************
 * @brief      ͨ����������ת������Ŀ��Ƕȼ����Ӧ�ı�����λ�á�
 * @author     NEUQRM, DGYin,
 * @version    1.0.0
 * @date       2022.11.6
 **************************************************************/
void AGV_DirectiveMotor_MotorAngle_To_EncoderAngle(float Target_Motion_Angle, float Spining_Mode)
{
	Chassis_MotorA.Target_Angle = Chassis_MotorA.Target_Angle ; 
    Chassis_MotorB.Target_Angle = Chassis_MotorB.Target_Angle * 4500 / 360.00f;
    Chassis_MotorC.Target_Angle = Chassis_MotorC.Target_Angle * 4500 / 360.00f;
    Chassis_MotorD.Target_Angle = Chassis_MotorD.Target_Angle * 4500 / 360.00f;
}

/***************************************************************
 * @brief      ͨ�����ֵ�ʵ��ת�ټ�����̼������ĵ������ٶ�
 * @author     NEUQRM,
 * @version    1.0.0
 * @date       2022.1.1
 **************************************************************/
void Mecanum_Wheel_Get_Base_Velocities(void)
{
    //���ݵ��ת�ٲ�������ת��
	Kinematics.Wheel_1.actual_speed.rpm = - Chassis_Motor1.actual_speed / M3508_REDUCTION_RATIO;
	Kinematics.Wheel_2.actual_speed.rpm =   Chassis_Motor2.actual_speed / M3508_REDUCTION_RATIO;
	Kinematics.Wheel_3.actual_speed.rpm =   Chassis_Motor3.actual_speed / M3508_REDUCTION_RATIO;
	Kinematics.Wheel_4.actual_speed.rpm = - Chassis_Motor4.actual_speed / M3508_REDUCTION_RATIO;  //����ת��ת��Ϊ�������ٶ�
	Kinematics.Wheel_1.actual_speed.linear_vel = Kinematics.Wheel_1.actual_speed.rpm * RPM2VEL;
	Kinematics.Wheel_2.actual_speed.linear_vel = Kinematics.Wheel_2.actual_speed.rpm * RPM2VEL;
	Kinematics.Wheel_3.actual_speed.linear_vel = Kinematics.Wheel_3.actual_speed.rpm * RPM2VEL;
	Kinematics.Wheel_4.actual_speed.linear_vel = Kinematics.Wheel_4.actual_speed.rpm * RPM2VEL;  //�������ٶ�ת��Ϊ��������������ٶ�
	Kinematics.actual_velocities.angular_z = ( Kinematics.Wheel_1.actual_speed.linear_vel - Kinematics.Wheel_2.actual_speed.linear_vel\
					- Kinematics.Wheel_3.actual_speed.linear_vel + Kinematics.Wheel_4.actual_speed.linear_vel) / (4.0f * (half_width + half_length));
	Kinematics.actual_velocities.linear_x  = (-Kinematics.Wheel_1.actual_speed.linear_vel + Kinematics.Wheel_2.actual_speed.linear_vel\
					- Kinematics.Wheel_3.actual_speed.linear_vel + Kinematics.Wheel_4.actual_speed.linear_vel) / (4.0f);
	Kinematics.actual_velocities.linear_y  = ( Kinematics.Wheel_1.actual_speed.linear_vel + Kinematics.Wheel_2.actual_speed.linear_vel\
					+ Kinematics.Wheel_3.actual_speed.linear_vel + Kinematics.Wheel_4.actual_speed.linear_vel) / (4.0f);
}

int find_max(void)
{
	int temp = 0;
	temp = Float_Abs(Chassis_Motor1.Target_Speed);
	if(Float_Abs(Chassis_Motor2.Target_Speed) > temp)
			temp = Float_Abs(Chassis_Motor2.Target_Speed);
	if(Float_Abs(Chassis_Motor3.Target_Speed) > temp)
			temp = Float_Abs(Chassis_Motor3.Target_Speed);
	if(Float_Abs(Chassis_Motor4.Target_Speed) > temp)
			temp = Float_Abs(Chassis_Motor4.Target_Speed);
	return temp;
}

void Speed_Limitation(void)
{
    int temp = 0;
	temp = Float_Abs(Chassis_Motor1.Target_Speed);
	if(Float_Abs(Chassis_Motor2.Target_Speed) > temp)
			temp = Float_Abs(Chassis_Motor2.Target_Speed);
	if(Float_Abs(Chassis_Motor3.Target_Speed) > temp)
			temp = Float_Abs(Chassis_Motor3.Target_Speed);
	if(Float_Abs(Chassis_Motor4.Target_Speed) > temp)
			temp = Float_Abs(Chassis_Motor4.Target_Speed);
	extern int Power_Mode;
	if (Power_Mode == 0) temp = 7000;
	//��������С�ٶȣ���ֱֹ������
    if(temp>MAX_MOTOR_SPEED)
    {
        Chassis_Motor1.Target_Speed=(int)(Chassis_Motor1.Target_Speed*MAX_MOTOR_SPEED*1.0/temp);
        Chassis_Motor2.Target_Speed=(int)(Chassis_Motor2.Target_Speed*MAX_MOTOR_SPEED*1.0/temp);
        Chassis_Motor3.Target_Speed=(int)(Chassis_Motor3.Target_Speed*MAX_MOTOR_SPEED*1.0/temp);
        Chassis_Motor4.Target_Speed=(int)(Chassis_Motor4.Target_Speed*MAX_MOTOR_SPEED*1.0/temp);
    }
    
    if (Chassis_Type == AGV_Chassis) //����ǣ�����ҲҪ�����ٶ�
    {
        temp = Float_Abs(Chassis_MotorA.Target_Speed);
        if(Float_Abs(Chassis_MotorB.Target_Speed) > temp)
                temp = Float_Abs(Chassis_MotorB.Target_Speed);
        if(Float_Abs(Chassis_MotorC.Target_Speed) > temp)
                temp = Float_Abs(Chassis_MotorC.Target_Speed);
        if(Float_Abs(Chassis_MotorD.Target_Speed) > temp)
                temp = Float_Abs(Chassis_MotorD.Target_Speed);
        if(temp>MAX_MOTOR_SPEED)
        {
            Chassis_MotorA.Target_Speed=(int)(Chassis_MotorA.Target_Speed*MAX_MOTOR_SPEED*1.0/temp);
            Chassis_MotorB.Target_Speed=(int)(Chassis_MotorB.Target_Speed*MAX_MOTOR_SPEED*1.0/temp);
            Chassis_MotorC.Target_Speed=(int)(Chassis_MotorC.Target_Speed*MAX_MOTOR_SPEED*1.0/temp);
            Chassis_MotorD.Target_Speed=(int)(Chassis_MotorD.Target_Speed*MAX_MOTOR_SPEED*1.0/temp);
        }
    }
}

void Directive_Motor_Angle_Optimize(MOTOR_t *Motor)
{
	float Now_Angle;
	float Target_Angle;
	Target_Angle = Motor->Target_Angle;
	Now_Angle = Motor->Last_Target_Angle;
	//���Ƕ�ת��Ϊ��Ȧ�Ƕ�
	while(Target_Angle>81920)Target_Angle = Target_Angle-81920; while(Target_Angle<0)Target_Angle = Target_Angle+81920;
	while(Now_Angle>81920)Now_Angle = Now_Angle-81920; while(Now_Angle<0)Now_Angle = Now_Angle+81920;
	//����ǶȲ�ֵ����ֵ
	if (Float_Abs(Target_Angle - Now_Angle) < Float_Abs(Now_Angle - Target_Angle))
		Motor->Find_Min = Float_Abs(Target_Angle - Now_Angle);
	else 
		Motor->Find_Min = Float_Abs(Now_Angle - Target_Angle);
	//�Ƕ��Ż�
	if (Motor->Find_Min > 8192*2.5f)
	{
		Motor->Target_Angle = Motor->Target_Angle + 8192*5;
		Motor->Invert_Flag = -1;
	}
	else Motor->Invert_Flag = 1;
	
	//���ӻ�����
	while (Motor->Target_Angle - Motor->Total_Angle >= 8192*5)  Motor->Target_Angle = Motor->Target_Angle - 8192*10; while (Motor->Target_Angle - Motor->Total_Angle <= -8192*5) Motor->Target_Angle = Motor->Target_Angle + 8192*10;

}

float Square(float Input) //�����ڸ�������ƽ������
{
    float Ans;
    Ans = Input * Input;
    return Ans;
}
