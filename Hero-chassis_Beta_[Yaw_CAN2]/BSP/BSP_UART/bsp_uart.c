
#include <stdio.h>
#include <string.h>
#include "dma.h"
#include "usart.h"
#include <math.h>
#include "bsp_can.h"
#include "Briter_Encoder.h"
#include "bsp_uart.h"
#include "chassis_move.h"
#include "trigger_turn.h"
//union Receive_data{int get_data[2];char char_data[8];}receive_data;
uint8_t receive_data[40];
uint8_t UART1_Sent_Data[10];
uint8_t UART1_Data_Type;
int UART1_Briter_Encoder2_Send_Status;
int UART1_Briter_Encoder4_Send_Status;

union Send_Data{int data[12];char char_data[8];} Send_Data;
uint8_t start_receive_flag = 0;



static int i = 0;
float a,b = 0;

uint8_t ch;
int int_get[2];
int iii=-1;
uint8_t dma_rx_buff[40];

void uart_init(void)
{
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
		HAL_UART_Receive_DMA(&huart1, dma_rx_buff, DMA_REC_LEN); 	
}

uint8_t length;
int temp_1;
void USART1_IRQHandler(void)
{
	length=0;
	int j;
	 if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE) != RESET)
	 {
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);
		HAL_UART_DMAStop(&huart1);
		length=DMA_REC_LEN-__HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
		 
		for(j=0;i<length;j++)
		{
			if(dma_rx_buff[j] == ';')//结束接收
			{	
				uint8_t Temp[8];	
				for (int i=1; i<=8; i++)
					Temp[i-1] = receive_data[i];
				switch(receive_data[0]) //判断是什么类型的数据
				{
					case WheelVel_TypeCode:				
						Chassis_Motor1.Target_Speed = (Temp[0]<<8)+(Temp[1]);					
						Chassis_Motor2.Target_Speed = (Temp[2]<<8)|(Temp[3]);									
						Chassis_Motor3.Target_Speed = (Temp[4]<<8)|(Temp[5]);
						Chassis_Motor4.Target_Speed = (Temp[6]<<8)|(Temp[7]);
						if (Chassis_Motor1.Target_Speed>20000)
							Chassis_Motor1.Target_Speed = Chassis_Motor1.Target_Speed-65535;
						if (Chassis_Motor2.Target_Speed>20000)
							Chassis_Motor2.Target_Speed = Chassis_Motor2.Target_Speed-65535;
						if (Chassis_Motor3.Target_Speed>20000)
							Chassis_Motor3.Target_Speed = Chassis_Motor3.Target_Speed-65535;
						if (Chassis_Motor4.Target_Speed>20000)
							Chassis_Motor4.Target_Speed = Chassis_Motor4.Target_Speed-65535;
						
						
					break;		
//					case Chassis_Motor_Current_TypeCode: //为节约串口资源，没有用这个
//						Chassis_Motor1.pid.speed_loop.vpid.PID_OUT = (Temp[0]<<8)|(Temp[1]);
//						Chassis_Motor2.pid.speed_loop.vpid.PID_OUT = (Temp[2]<<8)|(Temp[3]);
//						Chassis_Motor3.pid.speed_loop.vpid.PID_OUT = (Temp[4]<<8)|(Temp[5]);
//						Chassis_Motor4.pid.speed_loop.vpid.PID_OUT = (Temp[6]<<8)|(Temp[7]);
//						canTX_Chassis_Motor_Current();
//					break;
					
//					case Trigger_TypeCode:
//						if(Temp[0]==1) 
//						{
//							shoot_flag=1;
//							shoot_angle_clc();
//							Temp[0]=0;
//						}
//					break;			
					//Yaw控制指令接收
//					case YAW_CONTROL_TypeCode:
//						switch (Temp[0])
//						{	
//							//接收电流模式
//							case Current_Mode:
//								gimbal_y.Control_Mode = Current_Mode;
//								gimbal_y.given_current = (Temp[1]<<8)|(Temp[2]);
//								if (gimbal_y.given_current > 20000) gimbal_y.given_current = gimbal_y.given_current - 65535;//无符号数正负处理
//							break;
//							//接收绝对角度模式
//							case Target_Angle_Mode:
//								gimbal_y.Control_Mode = Target_Angle_Mode;
//								gimbal_y.Target_Angle = (Temp[1]<<8)|(Temp[2]);
//								gimbal_y.Target_Angle = gimbal_y.Target_Angle*2;
//							break;
//							//接收增量角度模式
//							case Add_Angle_Mode:
//								float Add_Angle;
//								gimbal_y.Control_Mode = Add_Angle_Mode;
//								Add_Angle = (Temp[1]<<8)|(Temp[2]);
//								gimbal_y.Target_Angle = gimbal_y.Target_Angle + Add_Angle/100.00f;
//							break;
//						}
//					break;
				}	
				memset(receive_data,0,sizeof(receive_data));
				start_receive_flag = 0;
				i = 0;
				break;
			}
			else if(dma_rx_buff[j] == '%')//结束接收，数据处理
			{
				memset(receive_data,0,sizeof(receive_data));
				start_receive_flag = 0;
				i = 0;				
				break;
			}
			if(start_receive_flag == 1)   //进行数据转移
			{ 
				if(i<length)
				{
					receive_data[i]=dma_rx_buff[j];
					i++;
				}

			}
			if((dma_rx_buff[j] == '*')||(dma_rx_buff[j] == '#'))//开始接收
			{
				start_receive_flag = 1;
			}
		}
		memset(dma_rx_buff,0,sizeof(dma_rx_buff));
		HAL_UART_Receive_DMA(&huart1, dma_rx_buff, DMA_REC_LEN);
	}
}


uint8_t Buffer[11];



void DMA_Send(void)
{ 
	Buffer[0] = '*';
	Buffer[1] = UART1_Data_Type;
	for (int i=2; i<=9; i++)
		Buffer[i] = UART1_Sent_Data[i-2];
	Buffer[10] = ';';
	uint8_t status;
	status=HAL_UART_Transmit(&huart1,Buffer,11,0xff);
	
	for (int i=0; i<8; i++)
		UART1_Sent_Data[i] = 0;
}

void UartTX_trigger(int16_t trigger)
{
	UART1_Data_Type = Trigger_TypeCode;
	for (int i=0; i<=5; i++)
		UART1_Sent_Data[i] = 0;
	UART1_Sent_Data[6] = trigger>>8;
	UART1_Sent_Data[7] = trigger&0xFF;
	DMA_Send();
}
	
void UartTX_To_AlphaBoard_Briter_Encoder(Briter_Encoder_t Encoder,int Typecode)
{
	if (Typecode == Briter_Encoder4_TypeCode)
		UART1_Data_Type = Briter_Encoder4_TypeCode;
	else if (Typecode == Briter_Encoder2_TypeCode)
		UART1_Data_Type = Briter_Encoder2_TypeCode;
	for (int i=0; i<8; i++)
		UART1_Sent_Data[i] = Encoder.Received_Data[i];
	if (UART1_Sent_Data[2]==0x01) UART1_Sent_Data[0]=0x07;
	DMA_Send();
}

//将Yaw电机的角度值、速度值发送给Alpha板
int IntIze_Angle = 0;
void UartTX_Yaw_Data_Trans(float angle, int16_t speed)
{
	UART1_Data_Type = Yaw_Data_Trans_Typecode;
	
	IntIze_Angle = (float) (angle * 100.0f);
	IntIze_Angle = IntIze_Angle;
	UART1_Sent_Data[0] = IntIze_Angle>>8;
	UART1_Sent_Data[1] = IntIze_Angle&0xFF;
	UART1_Sent_Data[2] = speed>>8;
	UART1_Sent_Data[3] = speed&0xFF;
	DMA_Send();
}
