/**
  ******************************************************************************
  * @file    Project/APP/bsp_referee.c 
  * @author  NEUQRM
  * @version V1.0.0
  * @date    12.2021
  * @brief   ���ò���ϵͳ
  ******************************************************************************
  * @attention
  ******************************************************************************
  ****************************(C) COPYRIGHT 2021 NEUQRM****************************

  ...........  ������δ�ɹ���ͬ־����Ŭ��  ...........
*/

#include "bsp_referee.h"

#include <stdio.h>
#include <string.h>
#include "dma.h"
#include "referee.h"
#include "usart.h"

#define RE_huart  huart6
#define RE_UART		USART6
#define RE_dma_rx	hdma_usart6_rx
#define RE_dma_tx   hdma_usart6_tx


//����ԭʼ���ݣ�Ϊ18���ֽڣ�����36���ֽڳ��ȣ���ֹDMA����Խ��
uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];
fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
static void REFEREE_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

uint8_t canTX_gimbal(int8_t robot_id, int16_t heat0, int16_t heat0_limit ,int16_t bullet_speed, uint8_t bullet_speedlimit);




//��������ʼ������
void referee_usart_fifo_init(void)
{
    REFEREE_init(usart6_buf[0], usart6_buf[1], USART_RX_BUF_LENGHT);
		fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
}

static void REFEREE_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
		//enable the DMA transfer for the receiver and tramsmit request
    //ʹ��DMA���ڽ��պͷ���
    SET_BIT(RE_huart.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(RE_huart.Instance->CR3, USART_CR3_DMAT);

    //enalbe idle interrupt
    //ʹ�ܿ����ж�
    __HAL_UART_ENABLE_IT(&RE_huart, UART_IT_IDLE);



    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&RE_dma_rx);
    
    while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&RE_dma_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&RE_dma_rx, DMA_LISR_TCIF2);

    RE_dma_rx.Instance->PAR = (uint32_t) & (RE_UART->DR);
    //memory buffer 1
    //�ڴ滺����1
    RE_dma_rx.Instance->M0AR = (uint32_t)(usart6_buf);
    //memory buffer 2
    //�ڴ滺����2
    RE_dma_rx.Instance->M1AR = (uint32_t)(usart6_buf);
    //data length
    //���ݳ���
    __HAL_DMA_SET_COUNTER(&RE_dma_rx, dma_buf_num);

    //enable double memory buffer
    //ʹ��˫������
    SET_BIT(RE_dma_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //ʹ��DMA
    __HAL_DMA_ENABLE(&RE_dma_rx);


    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&RE_dma_tx);

    while(RE_dma_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&RE_dma_tx);
    }

    RE_dma_tx.Instance->PAR = (uint32_t) & (RE_UART->DR);
}


//д��stm32f4xx_it.c����usart6�ж�
void USART6_IRQHandler(void)
{
	static volatile uint8_t res;
    if(RE_UART->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&RE_huart);

        static uint16_t this_time_rx_len = 0;

        if ((RE_huart.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(RE_huart.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(RE_huart.hdmarx);
            __HAL_DMA_SET_COUNTER(RE_huart.hdmarx, USART_RX_BUF_LENGHT);
            RE_huart.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(RE_huart.hdmarx);
            fifo_s_puts(&referee_fifo, (char*)usart6_buf[0], this_time_rx_len);
            //detect_hook(REFEREE_TOE);
        }
        else
        {
            __HAL_DMA_DISABLE(RE_huart.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(RE_huart.hdmarx);
            __HAL_DMA_SET_COUNTER(RE_huart.hdmarx, USART_RX_BUF_LENGHT);
            RE_huart.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(RE_huart.hdmarx);
            fifo_s_puts(&referee_fifo, (char*)usart6_buf[1], this_time_rx_len);
            //detect_hook(REFEREE_TOE);
        }
			}
}

/*���ڷ���*/
void RE_usart_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&RE_dma_tx);

    while(RE_dma_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&RE_dma_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&RE_dma_tx, DMA_HISR_TCIF7);

    RE_dma_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&RE_dma_tx, len);

    __HAL_DMA_ENABLE(&RE_dma_tx);
}


