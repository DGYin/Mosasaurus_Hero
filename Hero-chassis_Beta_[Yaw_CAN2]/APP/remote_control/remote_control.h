#ifndef __REMOTE_CONTROL_H
#define __REMOTE_CONTROL_H

#include "dma.h"
#include "usart.h"
#include "chassis_move.h"

#define SBUS_RX_BUF_NUM 36u
#define RC_FRAME_LENGTH 18u
//����ֵ
#define DEADLINE 100
//ң����ֵ��Χ          (�Ѹ�)
#define RC_MIDD 0
#define RC_MAXX 660
#define RC_MINN -660
//��̨�Ƕ��ٶȷ�Χ
#define RC_YAW_SPEED_MAXX 50
#define RC_YAW_SPEED_MINN -50
#define RC_YAW_ANGLE_MAXX 180
#define RC_YAW_ANGLE_MINN -180

#define RC_PITCH_SPEED_MAXX 30
#define RC_PITCH_SPEED_MINN -30
#define RC_PITCH_ANGLE_MAXX 25
#define RC_PITCH_ANGLE_MINN -25

//�����ٶȷ�Χ
#define X_SPEED_MAXX 255
#define X_SPEED_MINN -255

#define Y_SPEED_MAXX 255
#define Y_SPEED_MINN -255

#define Z_SPEED_MAXX 255
#define Z_SPEED_MINN -255

#define NORMAL_LINEAR_SPEED          70.0f
#define NORMAL_ROTATIONAL_SPEED      0.5f

//ң�������ݽṹ��
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
//ң��������ֵ

extern RC_ctrl_t rc_ctrl;
void remote_control_init(void);
void RC_IRQHandler(void);
void remote_control(void);
#endif
