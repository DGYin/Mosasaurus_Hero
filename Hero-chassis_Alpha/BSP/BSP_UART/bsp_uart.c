#include <stdio.h>
#include <string.h>
#include "dma.h"
#include "usart.h"
#include <math.h>
#include "bsp_can.h"
#include "Briter_Encoder.h"
#include "bsp_uart.h"
#include "chassis_move.h"


/***********************************************************
*@Brief	����bsp_uart.c�ļ���ʹ�õı���
***********************************************************/
uint8_t KeepAlive_SentData[4];
//union Receive_data{int get_data[2];char char_data[8];}receive_data;
uint8_t receive_data[40];
uint8_t UART1_Sent_Data[10];
uint8_t UART1_Data_Type;

union Send_Data
{
    int data[12];
    char char_data[8];
} Send_Data;
uint8_t start_receive_flag = 0;

static int i = 0;
float a, b = 0;

uint8_t ch;
int int_get[2];
int iii = -1;
uint8_t dma_rx_buff[20];

void uart_init(void)
{
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart1, dma_rx_buff, DMA_REC_LEN);
}

uint8_t length;
int temp_1;
void USART1_IRQHandler(void)
{
    int j;
    if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET) //�����жϣ�������һ֡���ݴ������ˣ�
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        HAL_UART_DMAStop(&huart1);
        length = DMA_REC_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
        for(j = 0; i < length; j++)
        {
            if(dma_rx_buff[j] == ';')//��������
            {
				uint8_t Temp[8];
				for (int i=1; i<=8; i++)
					Temp[i-1] = receive_data[i];
				switch(receive_data[0]) //�ж���ʲô���͵�����
				{
					case Briter_Encoder2_TypeCode:
						Encoder_Data_Process(Temp, &Briter_Encoder2,&Chassis_MotorB);
						break;

					case Briter_Encoder4_TypeCode:
						Encoder_Data_Process(Temp, &Briter_Encoder4,&Chassis_MotorD);
						break;
					case Yaw_Data_Trans_Typecode:
						float Angle, Speed;
						Angle = (uint16_t) ((Temp[0]<<8)|(Temp[1]));
						Angle = Angle / 100.0f;
						temp_1 = Angle;
						Speed = (uint16_t) ((Temp[2]<<8)|(Temp[3]));
						record_yaw_callback(Angle, Speed);
						break;
					case Super_Cap_RX_Typecode:
						extern int supercap_volt; //�������ݵ�ѹ
						extern float supercap_per; //�������ݵ����ٷֱ�
						extern int Capacitor_State;
						Capacitor_State   = Temp[0];
						supercap_per  = Temp[1];
						supercap_volt = Temp[2];
						break;
					case SuperCap_Status_RX_Typecode:
						if (KeepAlive_SentData[0] = Temp[0])
							if (KeepAlive_SentData[1] = Temp[1])
								if (KeepAlive_SentData[2] = Temp[2])
									if (KeepAlive_SentData[3] = Temp[3])
									{
										extern int Keep_Alive_Time_Cnt;
										Keep_Alive_Time_Cnt = 0;
									}
						break;
            	}
				//�������ݴ���

				//������ϣ���λ
                memset(receive_data, 0, sizeof(receive_data));
                start_receive_flag = 0;
                i = 0;
                break;
            }
            if(start_receive_flag == 1)   //��������ת��
            {
                if(i < length)
                {
                    receive_data[i] = dma_rx_buff[j];
                    i++;

                }

            }
			//��ʼ����
            if(dma_rx_buff[j] == '*')
                start_receive_flag = 1;
        }
        memset(dma_rx_buff, 0, sizeof(dma_rx_buff));
        HAL_UART_Receive_DMA(&huart1, dma_rx_buff, DMA_REC_LEN);
    }
}


//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	HAL_UART_Receive_IT(&huart1, dma_rx_buff, 11);
//}
uint8_t Buffer[11];

void UartTX_Super_Capacitor(int Power_Limitation, fp32 Power)
{
	//С��ת��
	int IntIze_Power;
	IntIze_Power = (int) (Power*10);
    Buffer[0] =  '*';	//��ʼ֡
	Buffer[1] =  SuperCap_Power_TX_Typecode;	//���ʷ��ͱ�־λ
	//���͹�������ֵ
    Buffer[2] =  (uint8_t)(Power_Limitation / 100);
	Power_Limitation = Power_Limitation - Buffer[2]*100;
    Buffer[3] =  (uint8_t)(Power_Limitation / 10);
    Buffer[4] =  (uint8_t)(Power_Limitation % 10);
	//���͵�ǰ����
    Buffer[5] =  (uint8_t)(IntIze_Power/1000);
	IntIze_Power=IntIze_Power- Buffer[5]*1000;
	Buffer[6] =  (uint8_t)(IntIze_Power/100);
	IntIze_Power=IntIze_Power- Buffer[6]*100;
	Buffer[7] =  (uint8_t)(IntIze_Power/10);
	Buffer[8] =  (uint8_t)(IntIze_Power%10);
	Buffer[9] = 0; //Ԥ��λ����ʱû��
	Buffer[10] = ';';	//����֡
    uint8_t status;
    status = HAL_UART_Transmit(&huart1, Buffer, 11, 0xff);
}

/***********************************************************
*@fuction	:Uart_TX_Supercap
*@brief		:
*@param		:Typecode		�������ݰ�����
*@param		:Sent_Data[8]	���͵����ݣ���8�ֽ�
*@return	:void
*@author	:DGYin
*@date		:2023-05-29
***********************************************************/
void Uart_TX_Supercap(int Typecode, uint8_t Sent_Data[8])
{
	uint8_t Data[11];
	//д����ʼ֡������֡
	Data[0] = '*';
	Data[10] = ';';
	//д�뷢�����ݰ�����
	Data[1] = Typecode;
	//д�����ݰ�
	for (int i=2; i<=9; i++)
	Data[i] = Sent_Data[i];
	//���ݷ��ͣ�����Status���ӷ��ͳɹ����
	uint8_t status;
    status = HAL_UART_Transmit(&huart1, Data, 11, 0xff);
}


void UART_TX_Supercap_Connection_Check(int Global_Time)
{
	uint8_t Sent_Data[8];
	int Keep_Alive_Typecode = SuperCap_KeepAlive_TX_Typecode;
	Sent_Data[0] = (uint8_t)(Global_Time>>24);
	Sent_Data[1] = (uint8_t)((Global_Time>>16)&0xff);
	Sent_Data[2] = (uint8_t)((Global_Time>>8)&0xff);
	Sent_Data[3] = (uint8_t)(Global_Time&0xff);
	for (int i=0; i<=3; i++)
		KeepAlive_SentData[i] = Sent_Data[i];
	Uart_TX_Supercap(Keep_Alive_Typecode, Sent_Data);
}


void DMA_Send(void)
{
    Buffer[0] = '*';
    Buffer[1] = UART1_Data_Type;
    for (int i = 2; i <= 9; i++)
        Buffer[i] = UART1_Sent_Data[i - 2];
    Buffer[10] = ';';
    uint8_t status;
    status = HAL_UART_Transmit(&huart1, Buffer, 11, 0xff);

    for (int i = 0; i < 8; i++)
        UART1_Sent_Data[i] = 0;
    UART1_Data_Type = 0;
}

void UartTX_trigger(void)
{
    UART1_Data_Type = Trigger_TypeCode;
    for (int i = 0; i <= 5; i++)
        UART1_Sent_Data[i] = 0;
    UART1_Sent_Data[0] = 1;
    DMA_Send();
}

void UartTX_To_BetaBoard_Briter_Encoder(Briter_Encoder_t Encoder, int Typecode)
{
    if (Typecode == Briter_Encoder4_TypeCode)
        UART1_Data_Type = Briter_Encoder4_TypeCode;
    else if (Typecode == Briter_Encoder2_TypeCode)
        UART1_Data_Type = Briter_Encoder2_TypeCode;
    for (int i = 0; i < 8; i++)
        UART1_Sent_Data[i] = Encoder.Send_Data[i];
    DMA_Send();
}

void UartTX_To_BetaBoard_Chassis_Motor_Current(void)
{

    UART1_Data_Type = Chassis_Motor_Current_TypeCode;
    uint8_t data[8];

    data[0] = (Chassis_Motor1.pid.speed_loop.vpid.PID_OUT) >> 8;
    data[1] = (Chassis_Motor1.pid.speed_loop.vpid.PID_OUT) & 0xFF;
    data[2] = (Chassis_Motor2.pid.speed_loop.vpid.PID_OUT) >> 8;
    data[3] = (Chassis_Motor2.pid.speed_loop.vpid.PID_OUT) & 0xFF;
    data[4] = (Chassis_Motor3.pid.speed_loop.vpid.PID_OUT) >> 8;
    data[5] = (Chassis_Motor3.pid.speed_loop.vpid.PID_OUT) & 0xFF;
    data[6] = (Chassis_Motor4.pid.speed_loop.vpid.PID_OUT) >> 8;
    data[7] = (Chassis_Motor4.pid.speed_loop.vpid.PID_OUT) & 0xFF;
    for (int i = 0; i < 8; i++)
        UART1_Sent_Data[i] = data[i];
    DMA_Send();
}

void UartTX_To_BetaBoard_Shoot_Info(int16_t vx, int16_t vy, int16_t shoot_flag_t)
{

    UART1_Data_Type = Shoot_Info_TypeCode;
    uint8_t data[8];

    data[0] = vx >> 8;
    data[1] = vx & 0xFF;
    data[2] = vy >> 8;
    data[3] = vy & 0xFF;
    data[4] = shoot_flag_t >> 8;
    data[5] = shoot_flag_t & 0xFF;
    for (int i = 0; i < 8; i++)
        UART1_Sent_Data[i] = data[i];
    DMA_Send();
}

int Test;
void UartTX_To_BetaBoard_Yaw_Control(int Control_Mode, float Value)
{
    UART1_Data_Type = YAW_CONTROL_TypeCode;
    uint8_t data[8] = {0};
    int IntIze_Value;

    data[0] = Control_Mode;
    switch (Control_Mode)
    {
    case Yaw_Control_Current_Mode:
        IntIze_Value = (int) Value;
        Test = IntIze_Value;
        data[1] = IntIze_Value >> 8;
        data[2] = IntIze_Value & 0xFF;
        break;
    case Yaw_Control_Target_Angle_Mode:
        Value = Value * 100;
        IntIze_Value = (int) Value;
        data[1] = IntIze_Value >> 8;
        data[2] = IntIze_Value & 0xFF;
        break;
    case Yaw_Control_Add_Angle_Mode:
        Value = Value * 100;
        IntIze_Value = (uint16_t) Value;
        data[1] = IntIze_Value >> 8;
        data[2] = IntIze_Value & 0xFF;
        break;
    }
    for (int i = 0; i < 8; i++)
        UART1_Sent_Data[i] = data[i];
    DMA_Send();
}

void UartTX_To_BetaBoard_WheelVel(float Speed1, float Speed2, float Speed3, float Speed4)
{
    UART1_Data_Type = WheelVel_TypeCode;
    uint8_t data[8] = {0};
    int IntIze_Value;

    IntIze_Value = (int) (Speed1);
    data[0] = IntIze_Value >> 8;
    data[1] = IntIze_Value & 0xFF;
    IntIze_Value = (int) (Speed2);
    data[2] = IntIze_Value >> 8;
    data[3] = IntIze_Value & 0xFF;
    IntIze_Value = (int) (Speed3);
    data[4] = IntIze_Value >> 8;
    data[5] = IntIze_Value & 0xFF;
    IntIze_Value = (int) (Speed4);
    data[6] = IntIze_Value >> 8;
    data[7] = IntIze_Value & 0xFF;

    for (int i = 0; i < 8; i++)
        UART1_Sent_Data[i] = data[i];
    DMA_Send();
}
