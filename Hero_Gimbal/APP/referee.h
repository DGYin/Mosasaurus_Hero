
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __REFEREE_H__
#define __REFEREE_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "protocol.h"
#include "fifo.h"

/* Defines -------------------------------------------------------------------*/
#define Referee_UART huart6
#define Referee_IRQHandler USART6_IRQHandler

/* Referee Defines -----------------------------------------------------------*/
/* 0x030X --------------------------------------------------------------------*/
typedef __packed struct
{
int16_t mouse_x;
int16_t mouse_y;
int16_t mouse_z;
int8_t left_button_down;
int8_t right_button_down;
uint16_t keyboard_value;
uint16_t reserved;
} ext_robot_command_t;

/* Structs -------------------------------------------------------------------*/
/* protocol包头结构体 */
extern frame_header_struct_t Referee_Receive_Header;

/* 0x030X */
extern ext_robot_command_t                   Robot_Command;

			
extern void referee_usart_task();

#ifdef __cplusplus
}
#endif

#endif /* __REFEREE_H__ */
