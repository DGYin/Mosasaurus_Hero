#ifndef __REMOTE_CONTROL_H
#define __REMOTE_CONTROL_H

#include "dma.h"
#include "usart.h"

#define SBUS_RX_BUF_NUM 36u
#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */

#define SW_L                    rc_ctrl.rc.s[1]
#define SW_R                    rc_ctrl.rc.s[0]
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)

/* ----------------------- PC Key Definition-------------------------------- */
#define MOUSE_x               rc_ctrl.mouse.x     
#define MOUSE_y               rc_ctrl.mouse.y
#define MOUSE_z               rc_ctrl.mouse.z
#define MOUSE_pre_left        rc_ctrl.mouse.press_l
#define MOUSE_pre_right       rc_ctrl.mouse.press_r
#define KEY_board             rc_ctrl.key.v

#define KEY_PRESSED_NULL                ((uint16_t)0     )
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL          0x000020
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            0x000100
#define KEY_PRESSED_OFFSET_F            0x000200
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)
/* ----------------------- Data Struct ------------------------------------- */

//遥控器数据结构体
typedef __packed struct
{
	__packed struct
	{
			int16_t ch[5];
			char s[2];
	} rc;
	__packed struct
	{
			int16_t x;
			int16_t y;
			int16_t z;
			uint8_t press_l;
			uint8_t press_r;
	} mouse;
	__packed struct
	{
			uint16_t v;
	} key;

} RC_ctrl_t;
//遥控器接收值
typedef struct{
	
	float target_angle;
	float target_speed;
}GIMBAL_RC_t;

typedef struct{
	GIMBAL_RC_t yaw;
	GIMBAL_RC_t pitch;
	
	float x_speed;
	float y_speed;
	float r_speed;
	
}RC_GET_t;
extern const RC_ctrl_t *get_remote_control_point(void);
extern RC_GET_t rc_sent;
extern RC_ctrl_t rc_ctrl;
extern RC_ctrl_t global_ctrl;
void remote_control_init(void);
void RC_IRQHandler(void);

#endif
