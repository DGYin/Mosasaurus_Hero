#ifndef __BSP_UART_H
#define __BSP_UART_H


#define DMA_REC_LEN    10


void uart_init(void);
void DMA_Send(void);

typedef enum
{
	ON_RESET=1,
	OFF=0,
}VISION_RESET; //重启标志位

typedef enum
{
	ON_FAR=1,
	OFF_FAR=0,
}VISION_FAR_SHOOT; //前哨站开启标志位

extern VISION_RESET VISION_RESET_FLAG;
extern VISION_FAR_SHOOT VISION_RAR_FLAG;
extern int shoot_vision_flag;
#endif