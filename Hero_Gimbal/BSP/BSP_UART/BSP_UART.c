#include "bsp_uart.h"
#include "gimbal_task.h"
#include <stdio.h>
#include <string.h>
#include "dma.h"
#include "usart.h"
#include "vision_task.h"

//union Receive_data{int get_data[2];char char_data[8];}receive_data;
uint8_t receive_data[20];
union Send_Data{int data[12];char char_data[8];} send_data;
uint8_t start_receive_flag = 0;

static int i = 0;
float a,b = 0;

uint8_t ch;
int int_get[2];
int iii=-1;
uint8_t dma_rx_buff[8];
int shoot_vision_flag=0;

void uart_init(void)
{
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
		HAL_UART_Receive_DMA(&huart1, dma_rx_buff, DMA_REC_LEN); 	
}

uint8_t length;
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
					vision_sent.yaw.target_angle =(int16_t)(receive_data[1]<<8|receive_data[0])/100.0f;
					vision_sent.pitch.target_angle =(int16_t)(receive_data[3]<<8|receive_data[2])/100.0f;
					shoot_vision_flag=(int16_t)(receive_data[5]<<8|receive_data[4]);
					start_receive_flag = 0;
					i = 0;
					break;
				}
			 if(start_receive_flag == 1)   //进行数据转移
        { 
				if(i<9)
				{
					receive_data[i]=dma_rx_buff[j];
				  i++;
				}
	    }
	    if(dma_rx_buff[j] == '*')//开始接收
	    {
		    start_receive_flag = 1;
	    }

		 }
		 HAL_UART_Receive_DMA(&huart1, dma_rx_buff, DMA_REC_LEN);
	 }
}
VISION_RESET VISION_RESET_FLAG=OFF;
VISION_FAR_SHOOT VISION_RAR_FLAG=OFF_FAR;

uint8_t Buffer[14],status;

int Sent_dataA = 0;
int Sent_dataB = 0;
int Sent_dataC = 0;
int Sent_dataD = 0;
int Sent_dataE = 0;

void DMA_Send(void)
{ 
	//c=100,d=200;
	//vision_getSpeed();
	Sent_dataA=100*gimbal_y.IMU_actual_angle;
	Sent_dataB=100*gimbal_p.IMU_actual_angle;
	//Sent_dataC=VISION_RAR_FLAG;      在rc_task赋值，左摇杆往下开启打前哨站
	Sent_dataD=0;
	Sent_dataE=VISION_RESET_FLAG;

	
	Buffer[0] = '*';
	Buffer[2] = (Sent_dataA>>8);
	Buffer[1] =  Sent_dataA&0xff;
	Buffer[4] = (Sent_dataB>>8);
	Buffer[3] =  Sent_dataB&0xff;
	Buffer[6] = (Sent_dataC>>8);
	Buffer[5] =  Sent_dataC&0xff;
	Buffer[8] = (Sent_dataD>>8);
	Buffer[7] =  Sent_dataD&0xff;
	Buffer[10] = (Sent_dataE>>8);
	Buffer[9] =  Sent_dataE&0xff;

	  for(int i=11;i<13;i++)
	{
	  Buffer[i] = 0;
	}

//  for(int i=1;i<(7-1);i++)
//	{
//	  Buffer[i] = send_data.data[i-1];
//	}
	Buffer[13] = ';';
	status=HAL_UART_Transmit(&huart1,Buffer,14,0xff);

}

