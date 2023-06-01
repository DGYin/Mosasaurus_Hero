/**
  ************************************* Copyright ****************************** 
  *
  *                 (C) Copyright 2022,Hebei,China, NEUQRM.
  *                            All Rights Reserved
  *                              
  *                     By DGYin, 
  *                     https://--
  *    
  * FileName   : supercap.c   
  * Version    : v1.0		
  * Author     : NEUQRM, DGYin, 		
  * Date       : 2023-05-29         
  * Description:    
  * Function List:  
  	1. ....
  	   <version>: 		
  <modify staff>:
  		  <data>:
   <description>:  
  	2. ...
  ******************************************************************************
 */
 
/*include----------------------------------------------------------------------*/
//#include "*.h"

/*define-----------------------------------------------------------------------*/


/*variate----------------------------------------------------------------------*/


/*statement--------------------------------------------------------------------*/


/*Function prototype Begin*****************************************************/


/*Function prototype End*******************************************************/
/*
 * **************************************************************************
 * ********************                                  ********************
 * ********************      COPYRIGHT INFORMATION       ********************
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

#include "usart.h"
#include "supercap.h"
#include "referee.h"

/***********************************************************
*@Brief	���ⲿ���õı���
***********************************************************/
int Supercap_Connection_Status = Supercap_Disconnected;
int Supercap_UART_RX_KeepAlive_Flag = 0;
int Capacitor_State = 0; 					//����ʣ������״̬��0Ϊ�ͣ�1Ϊ��
int supercap_volt = 150; 					//�������ݵ�ѹ
int Power_Mode = 0, Last_Power_Mode = 0;	//����ָʾ���̼���ģʽ
float supercap_per = 0; 					//�������ݵ����ٷֱ�

/***********************************************************
*@Brief	����supercap.c�ļ���ʹ�õı���
***********************************************************/
int Global_Time = 0;

/***********************************************************
*@Brief	����supercap.c��������˳��
***********************************************************/
void Supercap_Trans_RefereeData(void);
void Supercap_Keep_Alive(void);
void PowerMode_Judgement(void);
void BetaBoard_Trans_PowerMode(void);

/***********************************************************
  * @breif         �������ݣ����ݲ�ͬ�Ĺ������޸������ϵ�Ƭ����������
  * @param[in]     none
	* @param[out]    ����ͬ���ʵ��ַ�
  * @retval        none
***********************************************************/

extern void UartTX_Super_Capacitor(int Power_Limitation, fp32 Power);
extern void canTX_To_Beta_Power_Limit(int Power_Mode);
void supercap(int S_Cnt, int MS_Cnt)
{
	Global_Time = S_Cnt*1000 + MS_Cnt;	//����ȫ��ʱ��
	if (Global_Time%200 == 0)
		Supercap_Keep_Alive();
	if (Global_Time%100 == 0)
	{
		Supercap_Trans_RefereeData();
		PowerMode_Judgement();
		BetaBoard_Trans_PowerMode();
	}
}
/***********************************************************
*@fuction	:Supercap_Keep_Alive
*@brief		:ȷ�ϳ�������״̬
*@param		:
*@return	:void
*@author	:DGYin
*@date		:2023-05-29
***********************************************************/
extern void UART_TX_Supercap_Connection_Check(int Global_Time);
int Keep_Alive_Time_Cnt;
void Supercap_Keep_Alive(void)
{
	Keep_Alive_Time_Cnt++;	//��ʱ
	UART_TX_Supercap_Connection_Check(Global_Time);	//���Ͳ�ѯ����
	//��bsp_uart.c�Ĵ����ж��У����յ���ѯ������Keep_Alive_Time_Cnt������
	if (Keep_Alive_Time_Cnt > 10)//1.2sû���յ����練����
		Supercap_Connection_Status = Supercap_Disconnected;
	else Supercap_Connection_Status = Supercap_Connected;
}

/***********************************************************
*@fuction	:Supercap_Trans_RefereeData
*@brief		:�򳬵緢�Ͳ���ϵͳ����
*@param		:
*@return	:void
*@author	:DGYin
*@date		:2023-05-29
***********************************************************/
void Supercap_Trans_RefereeData(void)
{
	//�򳬵緢�͹�����Ϣ
	uint16_t UART_TX_Power_Max = 0, UART_TX_Buffer = 0;
	fp32 UART_TX_Power = 0;
    get_chassis_power_and_buffer_and_max(&UART_TX_Power, &UART_TX_Buffer, &UART_TX_Power_Max);  //��ȡ����ϵͳ����
    UartTX_Super_Capacitor(UART_TX_Power_Max, UART_TX_Power);	//�򳬵緢�͹���������������̵�ǰ����
}

/***********************************************************
*@fuction	:PowerMode_Judgement
*@brief		:���ݳ���״̬���Ƶ��̹���ģʽ
*@param		:
*@return	:void
*@author	:DGYin
*@date		:2023-05-29
***********************************************************/
void PowerMode_Judgement(void)
{
	//���̹���ģʽ�жϣ��ֳ����Ƿ������������
	if (Supercap_Connection_Status == Supercap_Connected)
	{
		if (supercap_volt > 200)
			Power_Mode = High_Voltage_Mode;
		if (supercap_volt <= 150)
			Power_Mode = Low_Voltage_Mode;
		if (Last_Power_Mode == Low_Voltage_Mode && supercap_volt < 180) Power_Mode = Low_Voltage_Mode; 
		else Power_Mode = High_Voltage_Mode;
		
	}
	else 
	{
		Power_Mode = Medium_Voltage_Mode;
		
	}
	//��¼�ϴι���ģʽ
    Last_Power_Mode = Power_Mode;
}

void BetaBoard_Trans_PowerMode(void)
{
	canTX_To_Beta_Power_Limit(Power_Mode);
}