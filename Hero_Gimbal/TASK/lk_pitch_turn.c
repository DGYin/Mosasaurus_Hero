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
  * Description: ��װ��겿�Pitch����Ļ�������    
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

//��ȡ�����Ȧ�Ƕ�ֵ���ٶȡ��¶ȡ�ת�ص���
void Get_Pitch_Motor_SingleRound_Angle(void)
{
	//�������Ͷ�ȡ����
	LK_Pitch_Motor_Send_Data[0] = Get_MultiRound_Angle_ID;
	for (int i=1; i<8; i++)
		LK_Pitch_Motor_Send_Data[i] = 0;
	canTX_LK_Pitch_Motor();
}
//���ڵ��MCU��λ�ñջ�����
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

							//���ͻ��ڵ��MCU��PID������ʹ֮д��RAM��
void Send_Pitch_Motor_PID_Parameter(void)
{

	LK_Pitch_Motor_Send_Data[0] = 0x31;
	LK_Pitch_Motor_Send_Data[2] = LK_Pitch_Motor_PID.Position_Kp; 
	LK_Pitch_Motor_Send_Data[3] = LK_Pitch_Motor_PID.Position_Ki;
	LK_Pitch_Motor_Send_Data[4] = LK_Pitch_Motor_PID.Speed_Kp;
	LK_Pitch_Motor_Send_Data[5] = LK_Pitch_Motor_PID.Speed_Ki;
	LK_Pitch_Motor_Send_Data[4] = LK_Pitch_Motor_PID.IQ_Kp;	//ת�ػ�P������û�õ���
	LK_Pitch_Motor_Send_Data[5] = LK_Pitch_Motor_PID.IQ_Ki;	//ת�ػ�I������û�õ���
	canTX_LK_Pitch_Motor();	
	for(int i=0 ;i<8 ;i++) LK_Pitch_Motor_Send_Data[i]=0;
}

							//���͵��ָֹͣ�������֮ǰ�Ŀ������ݣ���Ŀ��ǣ�
void Send_Pitch_Motor_Shutdown_Instruction(void)
{
	LK_Pitch_Motor_Send_Data[0] = 0x81;
    for(int i=1 ;i<8 ;i++) LK_Pitch_Motor_Send_Data[i]=0;
	canTX_LK_Pitch_Motor();	
	for(int i=0 ;i<8 ;i++) LK_Pitch_Motor_Send_Data[i]=0;
}

							//���͵����ʼ����ָ��
void Send_Pitch_Motor_Start_Instruction(void)
{
	LK_Pitch_Motor_Send_Data[0] = 0x88;
    for(int i=1 ;i<8 ;i++) LK_Pitch_Motor_Send_Data[i]=0;
	canTX_LK_Pitch_Motor();	
	for(int i=0 ;i<8 ;i++) LK_Pitch_Motor_Send_Data[i]=0;
}
							//��ȡ�������״̬
void Get_Pitch_Motor_Error_Status(void)
{
	//�������Ͷ�ȡ����
	LK_Pitch_Motor_Send_Data[0] = 0x9A;
	canTX_LK_Pitch_Motor();
	//��ʼ��״ָ̬ʾֵ
	LK_Pitch_Motor.Tempreture_Status = 0; 
	//���յ������
	//if (LK_Pitch_Motor_Receive_Data[7] == 0) LK_Pitch_Motor.Tempreture_Status = 1; 
}