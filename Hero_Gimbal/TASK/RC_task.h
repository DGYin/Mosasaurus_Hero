#ifndef __RC_TASK_H
#define __RC_TASK_H


//����ֵ
#define DEADLINE 20
//ң����ֵ��Χ
#define RC_MIDD 0
#define RC_MAXX 660
#define RC_MINN -660
//��̨�Ƕ��ٶȷ�Χ
#define RC_YAW_SPEED_MAXX 6
#define RC_YAW_SPEED_MINN -6
#define RC_YAW_ANGLE_MAXX 3
#define RC_YAW_ANGLE_MINN -3

#define RC_PITCH_SPEED_MAXX 6
#define RC_PITCH_SPEED_MINN -6
#define RC_PITCH_ANGLE_MAXX 1
#define RC_PITCH_ANGLE_MINN -1

//���������ٶȷ�Χ
#define X_SPEED_MAXX 400
#define X_SPEED_MINN -400

#define Y_SPEED_MAXX 400
#define Y_SPEED_MINN -400

#define Z_SPEED_MAXX 5
#define Z_SPEED_MINN -5

//����
#define KEY_DEADLINE  0
//ս��ģʽʱ�Ŀ��Ʒ�Χ
#define KEY_PITCH_ANGLE_MAXX_ON 1
#define KEY_PITCH_ANGLE_MINN_ON -1

#define KEY_YAW_ANGLE_MAXX_ON 2
#define KEY_YAW_ANGLE_MINN_ON -2

//���ģʽʱ�Ŀ��Ʒ�Χ
#define KEY_PITCH_ANGLE_MAXX_RUN 2
#define KEY_PITCH_ANGLE_MINN_RUN -2
#define KEY_X_SPEED_MAXX 300
#define KEY_X_SPEED_MINN -300
#define KEY_Y_SPEED_MAXX 300
#define KEY_Y_SPEED_MINN -300


#define KEY_YAW_ANGLE_MAXX_RUN 4
#define KEY_YAW_ANGLE_MINN_RUN -4

//ң����ֵ��Χ
#define KEY_MIDD 0
#define KEY_MAXX 427
#define KEY_MINN -427


typedef enum{
    KEY_ON= 0, //���̿�
    KEY_OFF,    //
}KEY_CONTROL;

void remote_control_data(void);
void key_control_data(void);
void control_mode_judge(void);


extern KEY_CONTROL KEY_MODE;

#endif
