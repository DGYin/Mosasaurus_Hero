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
#include "relay_task.h"

/*define-----------------------------------------------------------------------*/


/*variate----------------------------------------------------------------------*/


/*statement--------------------------------------------------------------------*/
void Yaw_Angle_Process(void);
static int16_t* read_motor_data(uint8_t *rxdata,CAN_HandleTypeDef *hcan);
void Encoder_Data_Process(uint8_t *rxdata, Briter_Encoder_t *Encoder, MOTOR_t *Motor);
static void get_motor_data(MOTOR_t *motor, uint16_t angle, int16_t speed, int16_t current);
static void record_chassis_callback(MOTOR_t *motor, uint16_t angle, int16_t current);


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

uint8_t bsp_can_init(void)
{
	uint8_t status=0;
	CAN_FilterTypeDef canFilter;
	
	
	canFilter.FilterBank=1;    																//ɸѡ����1
	canFilter.FilterIdHigh=0;
	canFilter.FilterIdLow=0;
	canFilter.FilterMaskIdHigh=0;
	canFilter.FilterMaskIdLow=0;
	canFilter.FilterMode=CAN_FILTERMODE_IDMASK;  							//����ģʽ
	canFilter.FilterActivation=CAN_FILTER_ENABLE;							//����
	canFilter.FilterScale=CAN_FILTERSCALE_32BIT; 							//32λģʽ
	canFilter.FilterFIFOAssignment=CAN_FILTER_FIFO0; 					//���ӵ�fifo0
	canFilter.SlaveStartFilterBank=14;												//can2ɸѡ����ʼ���
	
	status=HAL_CAN_ConfigFilter(&hcan1,&canFilter);					//���ù�����
	
	canFilter.FilterBank=15;    															//ɸѡ����15
	status=HAL_CAN_ConfigFilter(&hcan2,&canFilter);					//���ù�����
	
	/*�뿪��ʼģʽ*/
	HAL_CAN_Start(&hcan1);				
	HAL_CAN_Start(&hcan2);
	
	
	/*���ж�*/
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);       //can1 ����fifo 0��Ϊ���ж�
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);       //can2 ����fifo 0��Ϊ���ж�
	return status;
}

int flag9=0;
uint8_t data[8];
//uint8_t Can_Tx_Message(CAN_HandleTypeDef *hcan,uint8_t *mdata)
//{
//	int i;
//	uint8_t status;
//	CAN_TxHeaderTypeDef CAN_TxHeaderStruct;
//	for(i=0;i<8;i++)
//	data[i]=mdata[i];
//	uint32_t  pTxMailbox;
//	if(hcan==&hcan1)
//	{
//		flag9=1;
//		CAN_TxHeaderStruct.StdId=0x200;
//		CAN_TxHeaderStruct.ExtId=0;
//		CAN_TxHeaderStruct.DLC=8;
//		CAN_TxHeaderStruct.IDE=CAN_ID_STD;
//		CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
//		CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
//	}
//	else if(hcan==&hcan2)
//	{
//		CAN_TxHeaderStruct.StdId=0x007;
//		CAN_TxHeaderStruct.ExtId=0;
//		CAN_TxHeaderStruct.DLC=8;
//		CAN_TxHeaderStruct.IDE=CAN_ID_STD;
//		CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
//		CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
//	}
//	status=HAL_CAN_AddTxMessage(hcan,&CAN_TxHeaderStruct,mdata,&pTxMailbox);
//	return status;
//}
int flag4=0;
float pich_angle;
int mode_now=1;
char M_DAta[5];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//CAN���պ���
{
	flag4=1;
	CAN_RxHeaderTypeDef CAN_RxHeaderStruct;
	uint8_t rxdata[8];
	int16_t speed,*gdata,current;
	float angle;
	if(hcan==&hcan1)//CAN1�����Ӧ�ص�
	{
		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&CAN_RxHeaderStruct,rxdata);
		gdata=read_motor_data(rxdata,&hcan1);
		angle=gdata[0];
		speed=gdata[1];
		current=gdata[2];
		switch(CAN_RxHeaderStruct.StdId)
		{
			case CAN_3508MotorA_ID:
				get_motor_data(&Chassis_MotorA,angle,speed,current);
			break;
			case CAN_3508MotorB_ID:
				get_motor_data(&Chassis_MotorB,angle,speed,current);
			break;
			case CAN_3508MotorC_ID:
				get_motor_data(&Chassis_MotorC,angle,speed,current);
			break;
			case CAN_3508MotorD_ID:
				get_motor_data(&Chassis_MotorD,angle,speed,current);
			break;
			break;
			//���������ݽ���
			case Briter_Encoder2_ID:
				Encoder_Data_Process(rxdata, &Briter_Encoder2,&Chassis_MotorB);              
			break;	            
			case Briter_Encoder4_ID:
				Encoder_Data_Process(rxdata, &Briter_Encoder4,&Chassis_MotorD);              
			break;  
		}
	}
	else if(hcan==&hcan2)//�˴���CAN2������ָ̨�ID����̨�����Ѷ�Ӧ
	{
        //��ֹ�賵
		if(HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO0,&CAN_RxHeaderStruct,rxdata)!=HAL_OK)
			chassis_control_order.chassis_mode=CHASSIS_REMOTE_CLOSE;
		gdata=read_motor_data(rxdata,&hcan2);
		
		switch(CAN_RxHeaderStruct.StdId)
		{

			case MODE_RECEIVE_ID:
				chassis_control_order.chassis_mode		=	(uint8_t)rxdata[0];
				chassis_control_order.Precision_Mode 	= 	(uint8_t)rxdata[1];
				Chassis_Follow_Switch					= 	(uint8_t)rxdata[2];		
				chassis_control_order.last_chassis_mode=chassis_control_order.chassis_mode;
				break;		
			case GIMBAL_CONTROL_ID: //ң��ֵ����
				chassis_control_order.vx_set=(int16_t)((gdata[0]<<8)|gdata[1]); 
				chassis_control_order.vy_set=(int16_t)((gdata[2]<<8)|gdata[3]); 
				chassis_control_order.wz_set=(int16_t)((gdata[4]<<8)|gdata[5]);	
				break;	
			case Chassis_Shoot_Task_Rx_ID: //�Ƿ���
				extern int Shoot_Num, Fric_State;
				Fric_State = gdata[1];
				Shoot_Num = gdata[2];
				break;		
			case YAW_ID:
				yaw_can_rx.lastangle = yaw_can_rx.angle;
				yaw_can_rx.angle = (int16_t)((gdata[0]<<8)|gdata[1]);
				yaw_can_rx.speed = (int16_t)((gdata[2]<<8)|gdata[3]);
				Yaw_Angle_Process();
				break;
			case UI_ID:  //UI����
				pich_angle=(int16_t)((gdata[0]<<8)|gdata[1]);
				pich_angle=(float)pich_angle/100.0f;
				mode_now=(int16_t)((gdata[2]<<8)|gdata[3]);
				mode_now=(float)mode_now;
				extern int Pitch_Tempreture;
				Pitch_Tempreture = (int16_t)((gdata[4]<<8)|gdata[5]);
				// 1 �涯 2 f 3 С����
//				if(mode_now==1) {M_DAta[0]='A';M_DAta[1]='A';}
//				if(mode_now==2) {M_DAta[0]='F';M_DAta[1]='F';}
//				if(mode_now==3) {M_DAta[0]='T';M_DAta[1]='T';}
				break;
			//���������ݽ���
			case Briter_Encoder1_ID:
				Encoder_Data_Process(rxdata, &Briter_Encoder1,&Chassis_MotorA);              
				break;			
			case Briter_Encoder3_ID:
				Encoder_Data_Process(rxdata, &Briter_Encoder3,&Chassis_MotorC);              
				break;     
			//Yaw����Flag��������
			case CAN_Yaw_Invert_Flag_Trans_ID:
				gimbal_y.Invert_Flag = rxdata[0];
				break;
			//���ռ̵�������ģʽ
			case Relay_Mode_Set_ID:
				Relay_Set_State = rxdata[0];
				break;
		}
	}
}
int16_t adata[8];
static int16_t* read_motor_data(uint8_t *rxdata,CAN_HandleTypeDef *hcan)//��λ��ȡ���ݺ���
{
//	static int16_t adata[4];
	if(hcan==&hcan1)
	{
		adata[0]=(int16_t)((rxdata[0]<<8)|rxdata[1]);
		adata[1]=(int16_t)((rxdata[2]<<8)|rxdata[3]);
		adata[2]=(int16_t)((rxdata[4]<<8)|rxdata[5]);
		adata[3]=(int16_t)((rxdata[6]<<8)|rxdata[7]);
	}
	else if(hcan==&hcan2)//CAN2��ȫ����
	{
		adata[0]=(int16_t)(rxdata[0]);
		adata[1]=(int16_t)(rxdata[1]);
		adata[2]=(int16_t)(rxdata[2]);
		adata[3]=(int16_t)(rxdata[3]);
		adata[4]=(int16_t)(rxdata[4]);
		adata[5]=(int16_t)(rxdata[5]);
		adata[6]=(int16_t)(rxdata[6]);
		adata[7]=(int16_t)(rxdata[7]);
	}
	return adata;
}

void Encoder_Data_Process(uint8_t *rxdata, Briter_Encoder_t *Encoder, MOTOR_t *Motor)
{
    Encoder->Last_Angle =  Encoder->Absolute_Angle;
	//���ݱ���
    for (int i=0; i<8; i++)
        Encoder->Received_Data[i] = rxdata[i];
	//�ǶȽ���
    Encoder->Raw_Present_Angle = (int) ((Encoder->Received_Data[6]<<24)|(Encoder->Received_Data[5]<<16)|(Encoder->Received_Data[4]<<8)|Encoder->Received_Data[3]);        
	//Ȧ������
	Encoder->Raw_Round =  Encoder->Raw_Present_Angle/5120;
    Encoder->Absolute_Angle = (int) ((Encoder->Raw_Present_Angle % 5120) / 5120.0f *81920.0f);
	if (Encoder->Absolute_Angle - Encoder->Last_Angle > 40960)
		Encoder->Total_Round --;
	else if (Encoder->Absolute_Angle - Encoder->Last_Angle < -40960)  
		Encoder->Total_Round ++;
	Encoder->Total_Angle = Encoder->Total_Round*81920 + Encoder->Absolute_Angle; 

		//���ڱ�������3508�ļ��ٱȲ�ͬ����Ҫ�����ݽ���ת�����˶���������3508�ļ��ٱȵõ��ĽǶȼ���ģ�
//		Motor->round_cnt =  Encoder->Raw_Present_Angle/512;
//		Motor->last_angle = Motor->actual_angle;
//		Motor->actual_angle = Encoder->Raw_Present_Angle%512 / 512.0f *8191.0f;
//		if(Motor->actual_angle - Motor->last_angle > 4096)h 
//			Motor->round_cnt--;
//		else if (Motor->actual_angle - Motor->last_angle < -4096)
//			Motor->round_cnt++;		
		
		//����ע��
//		Motor->Total_Angle = Motor->round_cnt * 8192 + Motor->actual_angle;
//		Motor->Total_Angle = -Motor->Total_Angle;
		Motor->Total_Angle = -Encoder->Total_Angle;
		//��ʼλ�ü�¼
//		if (Encoder->Request_Flag == 0)
//		{
//			Motor->start_angle = Motor->Total_Angle;
//			while (Motor->start_angle > 81920) Motor->start_angle = Motor->start_angle - 81920;
//			while (Motor->start_angle < 0) Motor->start_angle = Motor->start_angle + 81920;
//		}
		Motor->pid.position_loop.apid.actual_angle =  Motor->Total_Angle;//��Ϊ����������ת����͵������ת�������෴�ģ�����Ҫ���д���
		Encoder->Request_Flag = 1;
	
    
}
static void get_motor_data(MOTOR_t *motor, uint16_t angle, int16_t speed, int16_t current)
{
	//if (All_Initial_Position_Recorded())
	//{
		motor->last_angle = motor->actual_angle;
		motor->actual_angle = angle;
		
		motor->actual_speed = 0.5*(speed + motor->last_speed);
		motor->pid.position_loop.vpid.actual_speed=motor->actual_speed;
		motor->pid.speed_loop.vpid.actual_speed=motor->actual_speed;
		//motor->actual_speed = KalmanFilter(speed,Q,R);
		motor->last_speed = speed;
		motor->actual_current = current;
		//motor1.temp = temp;
//		if(motor->actual_angle - motor->last_angle > 4096)
//			motor->round_cnt--;
//		else if (motor->actual_angle - motor->last_angle < -4096)
//			motor->round_cnt++;
		//motor->Total_Angle = motor->round_cnt * 8192 + motor->actual_angle + motor->start_angle;
		
		//motor->pid.position_loop.apid.actual_angle = motor->Total_Angle;
	//}
}

static void record_chassis_callback(MOTOR_t *motor, uint16_t angle, int16_t current)
{
	motor->last_angle = motor->actual_angle;
	motor->actual_angle = (float)angle/16384.0f;
	motor->actual_angle*=360.0f;
	motor->actual_current = current;
	//motor1.temp = temp;
	if(motor->start_angle_flag==0)
	{
//		motor->start_angle = (float)angle/8191.0f*360.0f;
//		motor->start_angle_flag++;	//ֻ������ʱ��¼һ�γ�ʼ�Ƕ�
	}
	
}

void record_yaw_callback(float angle,float speed)
{	
	yaw_can_rx.lastangle = yaw_can_rx.angle;
	yaw_can_rx.angle = angle;
	yaw_can_rx.speed = speed;
	yaw_can_rx.turns=(yaw_can_rx.angle-yaw_can_rx.lastangle)<-180.0f?yaw_can_rx.turns+1:yaw_can_rx.turns;
	yaw_can_rx.turns=(yaw_can_rx.angle-yaw_can_rx.lastangle)>180.0f? yaw_can_rx.turns-1:yaw_can_rx.turns;	
	chassis_center.actual_angle=yaw_can_rx.angle; //���ڵ�����̬����
}


void send_gimbal_data_2(void)
{
	int Angle,Speed;
	Angle=(int16_t)(yaw_can_rx.angle*100); //Ϊ�˴���С������ԭʼ��������һ�ٱ�
	Speed=(int16_t)yaw_can_rx.speed;
	//canTX_To_Gimbal_Yaw_Callback(Angle, Speed, shoot_flag);
}

//����һ����ӷ��Ͳ���ϵͳ��������
void canTX_RefereeData(int16_t *power,int16_t *buffer,int16_t *max_power)
{
	uint8_t data[8];
	uint8_t status;
	CAN_TxHeaderTypeDef CAN_TxHeaderStruct;
	uint32_t  pTxMailbox;
	
	CAN_TxHeaderStruct.StdId=CAN_RefereeData_ID;
	CAN_TxHeaderStruct.ExtId=0;
	CAN_TxHeaderStruct.DLC=8;
	CAN_TxHeaderStruct.IDE=CAN_ID_STD;
	CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
	CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
	
	data[0]=*power>>8;
	data[1]=*power&0xff;
	data[2]=*buffer>>8;
	data[3]=*buffer&0xff;
	data[4]=*max_power>>8;
	data[5]=*max_power&0xff;
	status=HAL_CAN_AddTxMessage(&hcan2,&CAN_TxHeaderStruct,data,&pTxMailbox);
}

void canTX_To_Gimbal_Yaw_Callback(int16_t Angle,int16_t Speed,int16_t shoot_flag_t)
{
	uint8_t data[8];
	uint8_t status;
	CAN_TxHeaderTypeDef CAN_TxHeaderStruct;
	uint32_t  pTxMailbox;
	
	CAN_TxHeaderStruct.StdId=CAN_GIMBAL_Y_ID;
	CAN_TxHeaderStruct.ExtId=0;
	CAN_TxHeaderStruct.DLC=8;
	CAN_TxHeaderStruct.IDE=CAN_ID_STD;
	CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
	CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
	data[0]=Angle>>8;
	data[1]=Angle&0xff;
	data[2]=Speed>>8;
	data[3]=Speed&0xff;
	data[4]=shoot_flag_t>>8;
	data[5]=shoot_flag_t&0xff;
	status=HAL_CAN_AddTxMessage(&hcan2,&CAN_TxHeaderStruct,data,&pTxMailbox);
}

void canTX_To_Beta_Power_Limit(int Power_Mode)
{
	uint8_t data[8];
	uint8_t status;
	CAN_TxHeaderTypeDef CAN_TxHeaderStruct;
	uint32_t  pTxMailbox;
	
	CAN_TxHeaderStruct.StdId=CAN_Beta_Power_Limit_ID;
	CAN_TxHeaderStruct.ExtId=0;
	CAN_TxHeaderStruct.DLC=8;
	CAN_TxHeaderStruct.IDE=CAN_ID_STD;
	CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
	CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
	data[0]=Power_Mode;
	status=HAL_CAN_AddTxMessage(&hcan2,&CAN_TxHeaderStruct,data,&pTxMailbox);
}

void canTX_Briter_Encoder(Briter_Encoder_t Paramater,CAN_HandleTypeDef *hcan)
{
	uint8_t data[8];
	uint8_t status;
	CAN_TxHeaderTypeDef CAN_TxHeaderStruct;
	uint32_t  pTxMailbox;
	
	CAN_TxHeaderStruct.StdId=Paramater.ID;
	CAN_TxHeaderStruct.ExtId=0;
	CAN_TxHeaderStruct.DLC=8;
	CAN_TxHeaderStruct.IDE=CAN_ID_STD;
	CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
	CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
	
	data[0]=Paramater.Length; //ָ���
	data[1]=Paramater.ID; //���ڱ�������ID
	data[2]=Paramater.Order_Code; //ָ�����
	for (int i=3; i<8; i++)
        data[i]=Paramater.Send_Data[i];
	status=HAL_CAN_AddTxMessage(hcan,&CAN_TxHeaderStruct,data,&pTxMailbox);
    status = status;
}

void canTX_To_BetaBoard_WheelVel(void)
{
	uint8_t data[8];
	uint8_t status;
	CAN_TxHeaderTypeDef CAN_TxHeaderStruct;
	uint32_t  pTxMailbox;
	
    CAN_TxHeaderStruct.StdId = Chassis_Motor_Speed_ID;
    CAN_TxHeaderStruct.ExtId=0;
    CAN_TxHeaderStruct.DLC=8;
    CAN_TxHeaderStruct.IDE=CAN_ID_STD;
    CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
    CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
	data[0]=(Chassis_Motor1.Target_Speed)>>8;
	data[1]=(Chassis_Motor1.Target_Speed)&0xFF;
	data[2]=(Chassis_Motor2.Target_Speed)>>8;
	data[3]=(Chassis_Motor2.Target_Speed)&0xFF;
	data[4]=(Chassis_Motor3.Target_Speed)>>8;
	data[5]=(Chassis_Motor3.Target_Speed)&0xFF;
	data[6]=(Chassis_Motor4.Target_Speed)>>8;
	data[7]=(Chassis_Motor4.Target_Speed)&0xFF;
	
	HAL_CAN_AddTxMessage(&hcan2,&CAN_TxHeaderStruct,data,&pTxMailbox);
}

void canTX_AGV_Chassis_Motor_Current(void)
{
	uint8_t data[8];
	uint8_t status;
	CAN_TxHeaderTypeDef CAN_TxHeaderStruct;
	uint32_t  pTxMailbox;
    CAN_TxHeaderStruct.StdId=0x1FF;
    CAN_TxHeaderStruct.ExtId=0;
    CAN_TxHeaderStruct.DLC=8;
    CAN_TxHeaderStruct.IDE=CAN_ID_STD;
    CAN_TxHeaderStruct.RTR=CAN_RTR_DATA;
    CAN_TxHeaderStruct.TransmitGlobalTime=DISABLE;
	if (Chassis_MotorA.pid.loop_flag == SPEED_LOOP)
	{
		data[0]=(Chassis_MotorA.pid.speed_loop.vpid.PID_OUT)>>8;
		data[1]=(Chassis_MotorA.pid.speed_loop.vpid.PID_OUT)&0xFF;
	}
	else if(Chassis_MotorA.pid.loop_flag == POSITION_LOOP)
	{
		data[0]=(Chassis_MotorA.pid.position_loop.vpid.PID_OUT)>>8;
		data[1]=(Chassis_MotorA.pid.position_loop.vpid.PID_OUT)&0xFF;
	}
	
    if (Chassis_MotorB.pid.loop_flag == SPEED_LOOP)
	{
		data[2]=(Chassis_MotorB.pid.speed_loop.vpid.PID_OUT)>>8;
		data[3]=(Chassis_MotorB.pid.speed_loop.vpid.PID_OUT)&0xFF;
	}
	else if(Chassis_MotorB.pid.loop_flag == POSITION_LOOP)
	{
		data[2]=(Chassis_MotorB.pid.position_loop.vpid.PID_OUT)>>8;
		data[3]=(Chassis_MotorB.pid.position_loop.vpid.PID_OUT)&0xFF;
	}
	
    if (Chassis_MotorC.pid.loop_flag == SPEED_LOOP)
	{
		data[4]=(Chassis_MotorC.pid.speed_loop.vpid.PID_OUT)>>8;
		data[5]=(Chassis_MotorC.pid.speed_loop.vpid.PID_OUT)&0xFF;
	}
	else if(Chassis_MotorC.pid.loop_flag == POSITION_LOOP)
	{
		data[4]=(Chassis_MotorC.pid.position_loop.vpid.PID_OUT)>>8;
		data[5]=(Chassis_MotorC.pid.position_loop.vpid.PID_OUT)&0xFF;
	}

	if (Chassis_MotorD.pid.loop_flag == SPEED_LOOP)
	{
		data[6]=(Chassis_MotorD.pid.speed_loop.vpid.PID_OUT)>>8;
		data[7]=(Chassis_MotorD.pid.speed_loop.vpid.PID_OUT)&0xFF;
	}
	else if(Chassis_MotorD.pid.loop_flag == POSITION_LOOP)
	{
		data[6]=(Chassis_MotorD.pid.position_loop.vpid.PID_OUT)>>8;
		data[7]=(Chassis_MotorD.pid.position_loop.vpid.PID_OUT)&0xFF;
	}
    
    //if (All_Initial_Position_Recorded())
		status=HAL_CAN_AddTxMessage(&hcan1,&CAN_TxHeaderStruct,data,&pTxMailbox);
}  
    
void Yaw_Angle_Process(void)
{
	if (yaw_can_rx.angle - yaw_can_rx.lastangle < -4096) yaw_can_rx.turns++;
	if (yaw_can_rx.angle - yaw_can_rx.lastangle >  4096) yaw_can_rx.turns--;
	gimbal_y.CAN_Total_Angle = (float)yaw_can_rx.turns * 8192 + yaw_can_rx.angle;//�õ�Total_Angle
	float temp;
	temp = gimbal_y.CAN_Total_Angle;
	while (temp > 16384.f) temp =temp - 16384.f; while (temp <0) temp = temp + 16384.f;
	chassis_center.actual_angle=temp*360.0f/16384.0f;
}