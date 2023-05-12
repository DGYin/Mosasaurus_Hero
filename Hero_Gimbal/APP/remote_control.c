#include "remote_control.h"
#include "stdarg.h"
#include "dma.h"
#include "usart.h"
#include "math.h"

#define RC_huart    huart3
#define RC_UART		USART3
#define RC_dma		hdma_usart3_rx


/*******
ch
0  右通道 左右
1  右通道 前后
2  左通道 左右
3  左通道 前后
4  左上
s
0   左
1   右
*/
RC_ctrl_t rc_ctrl;
RC_ctrl_t global_ctrl;
RC_GET_t rc_sent;

//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl,RC_ctrl_t *global_ctrl);
static void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
static void RC_restart(uint16_t dma_buf_num);

//主函数初始化调用
void remote_control_init(void)
{
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}


/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}
/**
  * @brief          				初始化遥控器
  * @param[*rx1_buf]      	第一个缓冲数组
  * @param[*rx2_buf]      	第二个缓冲数组
  * @param[dma_buf_num]     DMA缓冲区长度
  * @retval         ·				none
  */
static void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
	//enable the DMA transfer for the receiver request
	//使能 DMA 串口接收
	SET_BIT(RC_huart.Instance->CR3, USART_CR3_DMAR);
	//enalbe idle interrupt
	//使能空闲中断
	__HAL_UART_ENABLE_IT(&RC_huart, UART_IT_IDLE);
	//disable DMA
	//失效 DMA
	__HAL_DMA_DISABLE(&RC_dma);

	while(RC_dma.Instance->CR & DMA_SxCR_EN)
	{
			__HAL_DMA_DISABLE(&RC_dma);
	}

	RC_dma.Instance->PAR = (uint32_t) & (RC_UART->DR);
	//memory buffer 1
	//内存缓冲区 1
	RC_dma.Instance->M0AR = (uint32_t)(rx1_buf);
	//memory buffer 2
	//内存缓冲区 2
	RC_dma.Instance->M1AR = (uint32_t)(rx2_buf);
	//data length
	//数据长度
	RC_dma.Instance->NDTR = dma_buf_num;
	//enable double memory buffer
	//使能双缓冲区
	SET_BIT(RC_dma.Instance->CR, DMA_SxCR_DBM);
	//enable DMA
	//使能 DMA
	__HAL_DMA_ENABLE(&RC_dma);
}


//判断遥控器数据是否出错，
uint8_t RC_data_is_error(void)
{
    //使用了go to语句 方便出错统一处理遥控器变量数据归零
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
  * @brief        写入stm32f4xx_it.c串口中断
	*	@param				none
  * @retval       none
  */
void USART3_IRQHandler(void)
{
	if(RC_huart.Instance->SR & UART_FLAG_RXNE)//接收到数据
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
			//失效 DMA
			__HAL_DMA_DISABLE(&RC_dma);
			//get receive data length, length = set_data_length - remain_length
			//获取接收数据长度,长度 = 设定长度 - 剩余长度
			this_time_rx_len = SBUS_RX_BUF_NUM - RC_dma.Instance->NDTR;
			//reset set_data_lenght
			//重新设定数据长度
			RC_dma.Instance->NDTR = SBUS_RX_BUF_NUM;
			//set memory buffer 1
			//设定缓冲区 1
			RC_dma.Instance->CR |= DMA_SxCR_CT;
			//enable DMA
			//使能 DMA
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
			//失效 DMA
			__HAL_DMA_DISABLE(&RC_dma);
			//get receive data length, length = set_data_length - remain_length
			//获取接收数据长度,长度 = 设定长度 - 剩余长度
			this_time_rx_len = SBUS_RX_BUF_NUM - RC_dma.Instance->NDTR;
			//reset set_data_lenght
			//重新设定数据长度
			RC_dma.Instance->NDTR = SBUS_RX_BUF_NUM;
			//set memory buffer 0
			//设定缓冲区 0
			DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
			//enable DMA
			//使能 DMA
			__HAL_DMA_ENABLE(&RC_dma);
		
			if(this_time_rx_len == RC_FRAME_LENGTH)
			{
				//处理遥控器数据
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
  * @brief        						遥控器重启函数
	*	@param[dma_buf_num]				DMA缓冲区长度
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
  * @brief        					缓冲区得到遥控器值
	*	@param[*sbus_buf]				遥控器缓冲区
	* @param[*rc_ctrl]				遥控器值的指针
	*	@param[*global_ctrl]    全局控制的指针
  * @retval    							none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl, RC_ctrl_t *global_ctrl)
{
	if (sbus_buf == NULL || rc_ctrl == NULL)
	{
			return;
	}
	
	rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff; //右摇杆左右 -+
	rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //右摇杆下上 -+
	rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) &0x07ff;//左摇杆左右 -+
	rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //左摇杆下上 -+
	rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8); //左旋钮右左 -+
	rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003); //左拨杆 -+
	rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2; //右拨杆 -+
	
	rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8); //鼠标x轴坐标
	rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8); //鼠标y轴坐标
	rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);//鼠标z轴坐标
	
	rc_ctrl->mouse.press_l = sbus_buf[12]; //鼠标左键按下1
	rc_ctrl->mouse.press_r = sbus_buf[13]; //鼠标右键按下1
	
	rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8); //键盘键值
		
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
