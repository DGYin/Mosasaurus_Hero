#ifndef __BSP_UART_H
#define __BSP_UART_H

#include <stdint.h>
#include "Briter_Encoder.h"

#define DMA_REC_LEN    20

#define Briter_Encoder1_TypeCode 1
#define Briter_Encoder2_TypeCode 2
#define Briter_Encoder3_TypeCode 3
#define Briter_Encoder4_TypeCode 4


#define Trigger_TypeCode 11
#define Chassis_Motor_Current_TypeCode 5
#define Shoot_Info_TypeCode 6
#define Yaw_Data_Trans_Typecode  7
#define YAW_CONTROL_TypeCode 	  8
#define WheelVel_TypeCode 9

extern uint8_t UART1_Sent_Data[10];
extern uint8_t UART1_Data_Type;
extern int UART1_Briter_Encoder3_Send_Status;
extern int UART1_Briter_Encoder4_Send_Status;
extern int UART1_Briter_Encoder1_Send_Status;
extern int UART1_Briter_Encoder2_Send_Status;

void uart_init(void);
void DMA_Send(void);
void UartTX_trigger(int16_t trigger);
void UartTX_To_AlphaBoard_Briter_Encoder(Briter_Encoder_t Encoder, int Typecode);
void UartTX_Yaw_Data_Trans(float angle, int16_t speed);
#endif

