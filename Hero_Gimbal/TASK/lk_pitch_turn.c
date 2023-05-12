/**
  ************************************* Copyright ****************************** 
  *
  *                 (C) Copyright 2022, Hebei, China, NEUQRM.
  *                            All Rights Reserved
  *                              
  *                     By DGYin, 
  *                     https://--
  *    
  * FileName   : lk_pitch_turn.c   
  * Version    : v1.0		
  * Author     : NEUQRM, DGYin, 
  * Date       : 2022-11-21         
  * Description: 封装了瓴控Pitch电机的基本操作    
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
#include "bsp_can.h"
#include "lk_pitch_turn.h"
#include "sent_task.h"
/*define-----------------------------------------------------------------------*/


/*variate----------------------------------------------------------------------*/
LK_Pitch_Motor_t LK_Pitch_Motor;
LK_Pitch_Motor_PID_t LK_Pitch_Motor_PID;
int Pitch_Motor_Model;
/*statement--------------------------------------------------------------------*/


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

//读取电机单圈角度值、速度、温度、转矩电流
void Get_Pitch_Motor_SingleRound_Angle(void)
{
	//向电机发送读取申请
	LK_Pitch_Motor_Send_Data[0] = Get_MultiRound_Angle_ID;
	for (int i=1; i<8; i++)
		LK_Pitch_Motor_Send_Data[i] = 0;
	canTX_LK_Pitch_Motor();
}
//基于电机MCU的位置闭环控制
uint16_t maxspeed=3500;							
void Send_Pitch_Motor_Add_Angle(int32_t pitch)
{
	CAN_TxHeaderTypeDef canFrame;
	uint8_t data[8]={0};
	uint32_t temp=0;
	
	canFrame.IDE=CAN_ID_STD;
	canFrame.StdId=LK_Pitch_Motor_ID;
	canFrame.RTR=CAN_RTR_DATA;
	canFrame.DLC=8;
	canFrame.TransmitGlobalTime=DISABLE;
	data[0]=0xA8;
	data[1]=0;
	data[2]=(uint8_t)(maxspeed);
	data[3]=(uint8_t)(maxspeed>>8);
	data[4]=(uint8_t)((pitch));
	data[5]=(uint8_t)((pitch)>>8);
	data[6]=(uint8_t)((pitch)>>16);
	data[7]=(uint8_t)((pitch)>>24);

	HAL_CAN_AddTxMessage(&hcan1, &canFrame, data,&temp);
}
void Send_Pitch_Motor_Target_Speed(int32_t pitch)
{
	CAN_TxHeaderTypeDef canFrame;
	uint8_t data[8]={0};
	uint32_t temp=0;
	
	canFrame.IDE=CAN_ID_STD;
	canFrame.StdId=LK_Pitch_Motor_ID;
	canFrame.RTR=CAN_RTR_DATA;
	canFrame.DLC=8;
	canFrame.TransmitGlobalTime=DISABLE;
	data[0]=0xA2;
	data[1]=0;
	data[2]=0;
	data[3]=0;
	data[4]=(uint8_t)((pitch));
	data[5]=(uint8_t)((pitch)>>8);
	data[6]=(uint8_t)((pitch)>>16);
	data[7]=(uint8_t)((pitch)>>24);

	HAL_CAN_AddTxMessage(&hcan1, &canFrame, data,&temp);
}

							//发送基于电机MCU的PID参数，使之写入RAM。
void Send_Pitch_Motor_PID_Parameter(void)
{

	LK_Pitch_Motor_Send_Data[0] = 0x31;
	LK_Pitch_Motor_Send_Data[2] = LK_Pitch_Motor_PID.Position_Kp; 
	LK_Pitch_Motor_Send_Data[3] = LK_Pitch_Motor_PID.Position_Ki;
	LK_Pitch_Motor_Send_Data[4] = LK_Pitch_Motor_PID.Speed_Kp;
	LK_Pitch_Motor_Send_Data[5] = LK_Pitch_Motor_PID.Speed_Ki;
	LK_Pitch_Motor_Send_Data[4] = LK_Pitch_Motor_PID.IQ_Kp;	//转矩环P参数（没用到）
	LK_Pitch_Motor_Send_Data[5] = LK_Pitch_Motor_PID.IQ_Ki;	//转矩环I参数（没用到）
	canTX_LK_Pitch_Motor();	
	for(int i=0 ;i<8 ;i++) LK_Pitch_Motor_Send_Data[i]=0;
}

							//发送电机停止指令，但保留之前的控制数据（如目标角）
void Send_Pitch_Motor_Shutdown_Instruction(void)
{
	LK_Pitch_Motor_Send_Data[0] = 0x81;
    for(int i=1 ;i<8 ;i++) LK_Pitch_Motor_Send_Data[i]=0;
	canTX_LK_Pitch_Motor();	
	for(int i=0 ;i<8 ;i++) LK_Pitch_Motor_Send_Data[i]=0;
}

							//发送电机开始运行指令
void Send_Pitch_Motor_Start_Instruction(void)
{
	LK_Pitch_Motor_Send_Data[0] = 0x88;
    for(int i=1 ;i<8 ;i++) LK_Pitch_Motor_Send_Data[i]=0;
	canTX_LK_Pitch_Motor();	
	for(int i=0 ;i<8 ;i++) LK_Pitch_Motor_Send_Data[i]=0;
}
							//读取电机错误状态
void Get_Pitch_Motor_Error_Status(void)
{
	//向电机发送读取申请
	LK_Pitch_Motor_Send_Data[0] = 0x9A;
	canTX_LK_Pitch_Motor();
	//初始化状态指示值
	LK_Pitch_Motor.Tempreture_Status = 0; 
	//接收电机数据
	//if (LK_Pitch_Motor_Receive_Data[7] == 0) LK_Pitch_Motor.Tempreture_Status = 1; 
}