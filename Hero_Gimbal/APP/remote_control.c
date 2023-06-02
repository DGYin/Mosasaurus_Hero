#include "remote_control.h"
#include "stdarg.h"
#include "dma.h"
#include "usart.h"
#include "math.h"
#include "referee.h"

#define RC_huart    huart3
#define RC_UART		USART3
#define RC_dma		hdma_usart3_rx

int Transmission_Mode = Transmission_Mode_OFF;

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
RC_ctrl_t rc_ctrl;
RC_ctrl_t global_ctrl;
RC_GET_t rc_sent;

//����ԭʼ���ݣ�Ϊ18���ֽڣ�����36���ֽڳ��ȣ���ֹDMA����Խ��
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl,RC_ctrl_t *global_ctrl);
static void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
static void RC_restart(uint16_t dma_buf_num);

//��������ʼ������
void remote_control_init(void)
{
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}


/**
  * @brief          ��ȡң��������ָ��
  * @param[in]      none
  * @retval         ң��������ָ��
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}
/**
  * @brief          				��ʼ��ң����
  * @param[*rx1_buf]      	��һ����������
  * @param[*rx2_buf]      	�ڶ�����������
  * @param[dma_buf_num]     DMA����������
  * @retval         ��				none
  */
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


//�ж�ң���������Ƿ����
uint8_t RC_data_is_error(void)
{
    //ʹ����go to��� �������ͳһ����ң�����������ݹ���
    if (int16_abs(rc_ctrl.rc.ch[0]) > 660)
    {
        goto error;
    }
    if (int16_abs(rc_ctrl.rc.ch[1]) > 660)
    {
        goto error;
    }
    if (int16_abs(rc_ctrl.rc.ch[2]) > 660)
    {
        goto error;
    }
    if (int16_abs(rc_ctrl.rc.ch[3]) > 660)
    {
        goto error;                      
    }
    if (rc_ctrl.rc.s[0] == 0)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[1] == 0)
    {
        goto error;
    }
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

/**
  * @brief        д��stm32f4xx_it.c�����ж�
	*	@param				none
  * @retval       none
  */
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
				sbus_to_rc(sbus_rx_buf[0], &rc_ctrl,&global_ctrl);
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
				sbus_to_rc(sbus_rx_buf[1], &rc_ctrl,&global_ctrl);
				if(RC_data_is_error()==1)
				{
					RC_restart(SBUS_RX_BUF_NUM);
				}
			}
		}
	}
}

/**
  * @brief        						ң������������
	*	@param[dma_buf_num]				DMA����������
  * @retval      							 none
  */
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


/**
  * @brief        					�������õ�ң����ֵ
	*	@param[*sbus_buf]				ң����������
	* @param[*rc_ctrl]				ң����ֵ��ָ��
	*	@param[*global_ctrl]    ȫ�ֿ��Ƶ�ָ��
  * @retval    							none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl, RC_ctrl_t *global_ctrl)
{
	if (sbus_buf == NULL || rc_ctrl == NULL)
	{
			return;
	}
	
	rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff; //��ҡ������ -+
	rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //��ҡ������ -+
	rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) &0x07ff;//��ҡ������ -+
	rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //��ҡ������ -+
	rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8); //����ť���� -+
	rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003); //�󲦸� -+
	rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2; //�Ҳ��� -+
	
	if (Transmission_Mode == Transmission_Mode_OFF)
	{
		rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8); //���x������
		rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8); //���y������
		rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);//���z������
		rc_ctrl->mouse.press_l = sbus_buf[12]; //����������1
		rc_ctrl->mouse.press_r = sbus_buf[13]; //����Ҽ�����1
		rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8); //���̼�ֵ
	}	

	rc_ctrl->rc.ch[0] -= 1024;
	rc_ctrl->rc.ch[1] -= 1024;
	rc_ctrl->rc.ch[2] -= 1024;
	rc_ctrl->rc.ch[3] -= 1024;
	rc_ctrl->rc.ch[4] -= 1024;
	
	if(rc_ctrl->rc.s[1] == 1)
	{
		
	}
	else
	{
		global_ctrl->rc = rc_ctrl->rc;
	}
		
}
