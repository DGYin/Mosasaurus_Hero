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
void AGV_BaseVel_To_WheelVel(float linear_x, float linear_y, float angular_z);
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
void AGV_BaseVel_To_WheelVel(float linear_x, float linear_y, float angular_z)
{
	Kinematics.Wheel_1.Target_Speed.linear_vel = sqrt(linear_x * linear_x + linear_y * linear_y) + angular_z * (half_width + half_length);  //ע�������������Գư�װ��ͬʱ����ת�����򲢲���ͬ��
	Kinematics.Wheel_2.Target_Speed.linear_vel = sqrt(linear_x * linear_x + linear_y * linear_y) + angular_z * (half_width + half_length);
	Kinematics.Wheel_3.Target_Speed.linear_vel = sqrt(linear_x * linear_x + linear_y * linear_y) + angular_z * (half_width + half_length);
	Kinematics.Wheel_4.Target_Speed.linear_vel = sqrt(linear_x * linear_x + linear_y * linear_y) + angular_z * (half_width + half_length);	//���ٶ� cm/s  תת��  RPM
	Kinematics.Wheel_1.Target_Speed.rpm = Kinematics.Wheel_1.Target_Speed.linear_vel * VEL2RPM;
	Kinematics.Wheel_2.Target_Speed.rpm = Kinematics.Wheel_2.Target_Speed.linear_vel * VEL2RPM;
	Kinematics.Wheel_3.Target_Speed.rpm = Kinematics.Wheel_3.Target_Speed.linear_vel * VEL2RPM;
	Kinematics.Wheel_4.Target_Speed.rpm = Kinematics.Wheel_4.Target_Speed.linear_vel * VEL2RPM;
	Chassis_Motor1.Target_Speed =  (int)(Kinematics.Wheel_1.Target_Speed.rpm * M3508_REDUCTION_RATIO);
	Chassis_Motor2.Target_Speed =  (int)(Kinematics.Wheel_2.Target_Speed.rpm * M3508_REDUCTION_RATIO);
	Chassis_Motor3.Target_Speed =  (int)(Kinematics.Wheel_3.Target_Speed.rpm * M3508_REDUCTION_RATIO);
	Chassis_Motor4.Target_Speed =  (int)(Kinematics.Wheel_4.Target_Speed.rpm * M3508_REDUCTION_RATIO);
}
/***************************************************************
 * @brief      ͨ���Ƿ��������ݡ�ң�������ݼ��������������˶����򡣹涨��̨������Ϊ0�Ƚǣ�˳ʱ��������360�ȡ�
 * @author     NEUQRM, DGYin,
 * @version    1.0.0
 * @date       2022.11.6
 **************************************************************/
float AGV_DirectiveMotor_RobotMotion_To_TargetStatus(float linear_x, float linear_y, float Gimbal_Angle, float Chassis_Angle, int Spining_Mode)
{
    float tan, ang;
    tan = linear_y * 1.00f / linear_x * 1.00f;
    ang = atan(tan) * 180.00f * PI + Spining_Mode * (Gimbal_Angle * 1.00f - Chassis_Angle * 1.00f);
    if (ang < 0)  ang = ang + 360;
    if (ang >360) ang = ang - 360;
    return ang;
}

/***************************************************************
 * @brief      ͨ��������������˶��������������ת������Ŀ��Ƕȡ��涨��̨������Ϊ0�Ƚǣ�˳ʱ��������360�ȡ�
 * @author     NEUQRM, DGYin,
 * @version    1.0.0
 * @date       2022.11.6
 **************************************************************/
void AGV_DirectiveMotor_TargetStatus_To_MotorAngle(float Target_Motion_Angle, int Spining_Mode)
{
	Chassis_MotorA.Target_Angle = Target_Motion_Angle + Spining_Mode * 45.00f;
    Chassis_MotorB.Target_Angle = Target_Motion_Angle + Spining_Mode * (45.00f + 90.00f);
    Chassis_MotorC.Target_Angle = Target_Motion_Angle + Spining_Mode * (45.00f + 270.00f);
    Chassis_MotorD.Target_Angle = Target_Motion_Angle + Spining_Mode * (45.00f + 180.00f);
}

/***************************************************************
 * @brief      ͨ����������ת������Ŀ��Ƕȼ����Ӧ�ı�����λ�á�
 * @author     NEUQRM, DGYin,
 * @version    1.0.0
 * @date       2022.11.6
 **************************************************************/
void AGV_DirectiveMotor_MotorAngle_To_EncoderAngle(float Target_Motion_Angle, float Spining_Mode)
{
	Chassis_MotorA.Target_Angle = Chassis_MotorA.Target_Angle * 4500 / 360.00f; //�������ֱ���Ϊ0.08�ȣ�Ҳ����һȦ����Ϊ4500�ݡ�
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
	temp = abs(Chassis_Motor1.Target_Speed);
	if(abs(Chassis_Motor2.Target_Speed) > temp)
			temp = abs(Chassis_Motor2.Target_Speed);
	if(abs(Chassis_Motor3.Target_Speed) > temp)
			temp = abs(Chassis_Motor3.Target_Speed);
	if(abs(Chassis_Motor4.Target_Speed) > temp)
			temp = abs(Chassis_Motor4.Target_Speed);
	return temp;
}

void Speed_Limitation(void)
{
    int temp = 0;
	temp = abs(Chassis_Motor1.Target_Speed);
	if(abs(Chassis_Motor2.Target_Speed) > temp)
			temp = abs(Chassis_Motor2.Target_Speed);
	if(abs(Chassis_Motor3.Target_Speed) > temp)
			temp = abs(Chassis_Motor3.Target_Speed);
	if(abs(Chassis_Motor4.Target_Speed) > temp)
			temp = abs(Chassis_Motor4.Target_Speed);
    if(temp>MAX_MOTOR_SPEED)
    {
        Chassis_Motor1.Target_Speed=(int)(Chassis_Motor1.Target_Speed*MAX_MOTOR_SPEED*1.0/temp);
        Chassis_Motor2.Target_Speed=(int)(Chassis_Motor2.Target_Speed*MAX_MOTOR_SPEED*1.0/temp);
        Chassis_Motor3.Target_Speed=(int)(Chassis_Motor3.Target_Speed*MAX_MOTOR_SPEED*1.0/temp);
        Chassis_Motor4.Target_Speed=(int)(Chassis_Motor4.Target_Speed*MAX_MOTOR_SPEED*1.0/temp);
    }
    
    if (Chassis_Type == AGV_Chassis) //����ǣ�����ҲҪ�����ٶ�
    {
        temp = abs(Chassis_MotorA.Target_Speed);
        if(abs(Chassis_MotorB.Target_Speed) > temp)
                temp = abs(Chassis_MotorB.Target_Speed);
        if(abs(Chassis_MotorC.Target_Speed) > temp)
                temp = abs(Chassis_MotorC.Target_Speed);
        if(abs(Chassis_MotorD.Target_Speed) > temp)
                temp = abs(Chassis_MotorD.Target_Speed);
        if(temp>MAX_MOTOR_SPEED)
        {
            Chassis_MotorA.Target_Speed=(int)(Chassis_MotorA.Target_Speed*MAX_MOTOR_SPEED*1.0/temp);
            Chassis_MotorB.Target_Speed=(int)(Chassis_MotorB.Target_Speed*MAX_MOTOR_SPEED*1.0/temp);
            Chassis_MotorC.Target_Speed=(int)(Chassis_MotorC.Target_Speed*MAX_MOTOR_SPEED*1.0/temp);
            Chassis_MotorD.Target_Speed=(int)(Chassis_MotorD.Target_Speed*MAX_MOTOR_SPEED*1.0/temp);
        }
    }
}