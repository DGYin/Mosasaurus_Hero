/**
  ******************************************************************************
  * @file    Project/APP/remote_control.c 
  * @author  NEUQRM
  * @version V1.0.0
  * @date    12.2021
  * @brief   ���ļ��ǽ���ң������Ӧ��ͨ��ֵ�͸���ң�������ж϶�Ӧ��ģʽ����ң�������������ڵ�����ʱ��
  ******************************************************************************
  * @attention
  ******************************************************************************
  ****************************(C) COPYRIGHT 2021 NEUQRM****************************

  ...........  ������δ�ɹ���ͬ־����Ŭ��  ...........
*/

#include "remote_control.h"
#include "stdarg.h"
#include "dma.h"
#include "usart.h"
#include "math.h"

#define RC_huart    huart3
#define RC_UART		USART3
#define RC_dma		hdma_usart3_rx

RC_ctrl_t rc_ctrl = {{0x0400,0x0400,0x0400,0x0400,2,2,0x0400},{0,0,0,0,0,},{0}};

/*******
ch
0  ��ͨ�� ����
1  ��ͨ�� ǰ��
2  ��ͨ�� ����
3  ��ͨ�� ǰ��
4  ����
s
0   ��
1   ��
*/

//����ԭʼ���ݣ�Ϊ18���ֽڣ�����36���ֽڳ��ȣ���ֹDMA����Խ��
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
static void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
static int16_t RC_abs(int16_t value);
static void RC_restart(uint16_t dma_buf_num);
static float caculate_linear_speed(int width,int mid,int min,int max);

static float caculate_linear_speed(int width,int mid,int min,int max)
{
  float speed=0;
  if(width>=(mid+2))		//�м���������
    speed=(3.0*(width-(mid+2))/(max-(mid+2))*NORMAL_LINEAR_SPEED);
  else if(width<=(mid-2))
    speed=(3.0*(width-(mid-2))/((mid-2)-min)*NORMAL_LINEAR_SPEED);
  else
    speed=0;
  return speed;                
}

static float caculate_rotational_speed(int width,int mid,int min,int max)
{
  float speed=0;
  if(width>=(mid+2))		//�м���������
    speed=(3.0*(width-(mid+2))/(max-(mid+2))*NORMAL_ROTATIONAL_SPEED);
  else if(width<=(mid-2))
    speed=(3.0*(width-(mid-2))/((mid-2)-min)*NORMAL_ROTATIONAL_SPEED);
  else
    speed=0;
  return speed*3.5f;
}
//��������ʼ������
void remote_control_init(void)
{
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

static void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
	//enable the DMA transfer for the receiver request
	//ʹ�� DMA ���ڽ���
	SET_BIT(RC_huart.Instance->CR3, USART_CR3_DMAR);
	//enalbe idle interrupt
	//ʹ�ܿ����ж�
	__HAL_UART_ENABLE_IT(&RC_huart, UART_IT_IDLE);
	//disable DMA
	//ʧЧ DMA
	__HAL_DMA_DISABLE(&RC_dma);

	while(RC_dma.Instance->CR & DMA_SxCR_EN)
	{
			__HAL_DMA_DISABLE(&RC_dma);
	}

	RC_dma.Instance->PAR = (uint32_t) & (RC_UART->DR);
	//memory buffer 1
	//�ڴ滺���� 1
	RC_dma.Instance->M0AR = (uint32_t)(rx1_buf);
	//memory buffer 2
	//�ڴ滺���� 2
	RC_dma.Instance->M1AR = (uint32_t)(rx2_buf);
	//data length
	//���ݳ���
	RC_dma.Instance->NDTR = dma_buf_num;
	//enable double memory buffer
	//ʹ��˫������
	SET_BIT(RC_dma.Instance->CR, DMA_SxCR_DBM);
	//enable DMA
	//ʹ�� DMA
	__HAL_DMA_ENABLE(&RC_dma);
}

//�ⲿ����
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}


//�ж�ң���������Ƿ����
uint8_t RC_data_is_error(void)
{
    //ʹ����go to��� �������ͳһ����ң�����������ݹ���
    if (RC_abs(rc_ctrl.rc.ch[0]) > 1684)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[1]) > 1684)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[2]) > 1684)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[3]) > 1684)
    {
        goto error;
    }
//    if (rc_ctrl.rc.s[0] == 0)
//    {
//        goto error;
//    }
//    if (rc_ctrl.rc.s[1] == 0)
//    {
//        goto error;
//    }
    return 0;

error:
    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
    rc_ctrl.rc.s[0] = 2;
    rc_ctrl.rc.s[1] = 2;
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.key.v = 0;
    return 1;
}

//д��stm32f4xx_it.c�����ж�

void USART3_IRQHandler(void)
{
	if(RC_huart.Instance->SR & UART_FLAG_RXNE)//���յ�����
	{
		__HAL_UART_CLEAR_PEFLAG(&RC_huart);
	}
	else if(RC_UART->SR & UART_FLAG_IDLE)
	{
		static uint16_t this_time_rx_len = 0;
		__HAL_UART_CLEAR_PEFLAG(&RC_huart);
		if ((RC_dma.Instance->CR & DMA_SxCR_CT) == RESET)
		{
			/* Current memory buffer used is Memory 0 */
			//disable DMA
			//ʧЧ DMA
			__HAL_DMA_DISABLE(&RC_dma);
			//get receive data length, length = set_data_length - remain_length
			//��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
			this_time_rx_len = SBUS_RX_BUF_NUM - RC_dma.Instance->NDTR;
			//reset set_data_lenght
			//�����趨���ݳ���
			RC_dma.Instance->NDTR = SBUS_RX_BUF_NUM;
			//set memory buffer 1
			//�趨������ 1
			RC_dma.Instance->CR |= DMA_SxCR_CT;
			//enable DMA
			//ʹ�� DMA
			__HAL_DMA_ENABLE(&RC_dma);
		
			if(this_time_rx_len == RC_FRAME_LENGTH)
			{
				sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
				if(RC_data_is_error()==1)
				{
					RC_restart(SBUS_RX_BUF_NUM);
				}
			}
		}
		else
		{
			/* Current memory buffer used is Memory 1 */
			//disable DMA
			//ʧЧ DMA
			__HAL_DMA_DISABLE(&RC_dma);
			//get receive data length, length = set_data_length - remain_length
			//��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
			this_time_rx_len = SBUS_RX_BUF_NUM - RC_dma.Instance->NDTR;
			//reset set_data_lenght
			//�����趨���ݳ���
			RC_dma.Instance->NDTR = SBUS_RX_BUF_NUM;
			//set memory buffer 0
			//�趨������ 0
			DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
			//enable DMA
			//ʹ�� DMA
			__HAL_DMA_ENABLE(&RC_dma);
		
			if(this_time_rx_len == RC_FRAME_LENGTH)
			{
				//����ң��������
				sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
				if(RC_data_is_error()==1)
				{
					RC_restart(SBUS_RX_BUF_NUM);
				}
			}
		}
	}
}

static void RC_restart(uint16_t dma_buf_num)
{
	//disable UART
	__HAL_UART_DISABLE(&RC_huart);
	//disable DMA
	__HAL_DMA_DISABLE(&RC_dma);
	//reset set_data_lenght
	RC_dma.Instance->NDTR = SBUS_RX_BUF_NUM;
	__HAL_UART_CLEAR_IDLEFLAG(&RC_huart);
	__HAL_DMA_CLEAR_FLAG(&RC_dma,DMA_FLAG_TCIF2_6);
	// DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);
	//ensable UART
	__HAL_UART_ENABLE(&RC_huart);
	//ensable DMA
	__HAL_DMA_ENABLE(&RC_dma);
}

//ȡ������
static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}

static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
	if (sbus_buf == NULL || rc_ctrl == NULL)
	{
			return;
	}
	
	rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff; //!< Channel 0
	rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
	rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) &0x07ff;//!< Channel 2
	rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
	
	rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003); //!< Switch left
	rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2; //!< Switch right
	
	rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8); //!< Mouse X axis
	rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8); //!< Mouse Y axis
	rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8); //!< Mouse Z axis
	
	rc_ctrl->mouse.press_l = sbus_buf[12]; //!< Mouse Left Is Press 
	rc_ctrl->mouse.press_r = sbus_buf[13]; //!< Mouse Right Is Press 
	
	rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8); //!< KeyBoard value
	
	rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8); //NULL
	chassis_control_order.vx_set=(int16_t)rc_ctrl->rc.ch[3] - 0x0400; 
	chassis_control_order.vy_set=(int16_t)rc_ctrl->rc.ch[2] - 0x0400; 
	chassis_control_order.wz_set=(int16_t)rc_ctrl->rc.ch[0] - 0x0400;		
	chassis_control_order.vx_set = chassis_control_order.vx_set/10;
	chassis_control_order.vy_set = chassis_control_order.vy_set/10;
	chassis_control_order.wz_set = chassis_control_order.wz_set/10;
}

void remote_control(void)//ң����ͨ��ֵת���Լ�ģʽѡ��
{
		//�˲�
		//chassis_control_order.vy_set=caculate_linear_speed(rc_ctrl.rc.ch[0],RC_MIDD,RC_MINN,RC_MAXX);
		//chassis_control_order.vx_set=caculate_linear_speed(rc_ctrl.rc.ch[1],RC_MIDD,RC_MINN,RC_MAXX);
		//chassis_control_order.wz_set=caculate_rotational_speed(rc_ctrl.rc.ch[2],RC_MIDD,RC_MINN,RC_MAXX);
	
	switch(rc_ctrl.rc.s[1])                                //S1���󲦸ˣ�
	{
		case 3: {chassis_control_order.chassis_mode=CHASSIS_NORMAL;} break;//����ģʽ
		case 1: {chassis_control_order.chassis_mode=CHASSIS_REMOTE_CLOSE;} break;//��������ģʽ
		case 2: {chassis_control_order.chassis_mode=CHASSIS_SPIN;} break;//С����ģʽ
		default:break;
	}
	switch(rc_ctrl.rc.s[0])                                //S1���󲦸ˣ�
	{
		case 3: {shoot_flag=1;} break;//����ģʽ
		case 1: {shoot_flag=0;} break;//��������ģʽ
		//case 2: {chassis_control_order.chassis_mode=CHASSIS_SPIN;} break;//С����ģʽ
		default:break;
	}
}
