#/**
  ******************************************************************************
  * @file    referee.h
  * @author  Karolance Future
  * @version V1.3.0
  * @date    2022/03/21
  * @brief   Header file of referee.c
  ******************************************************************************
  * @attention
	*
	*   ���ݲ���ϵͳ ����Э�鸽¼ V1.3
	*
  ******************************************************************************
  */

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
/* �������� */
#define Game_Type_RMUC     1 //�����Կ���
#define Game_Type_RMUT     2 //������
#define Game_Type_RMUA     3 //�˹�������ս��
#define Game_Type_RMUL_3V3 4 //��У������3V3
#define Game_Type_RMUL_1V1 5 //��У������1V1

/* �����׶� */
#define Game_Progress_Unstart   0 //δ��ʼ����
#define Game_Progress_Prepare   1 //׼���׶�
#define Game_Progress_SelfCheck 2 //�Լ�׶�
#define Game_Progress_5sCount   3 //5s����ʱ
#define Game_Progress_Battle    4 //��ս��
#define Game_Progress_Calculate 5 //����������

/* ������� */
#define Game_Result_Draw    0 //ƽ��
#define Game_Result_RedWin  1 //�췽ʤ��
#define Game_Result_BlueWin 2 //����ʤ��

/* ������Ϣ */
#define Warning_Yellow  1 //���ƾ���
#define Warning_Red     2 //���ƾ���
#define Warning_Failure 3 //�и�

/* ������ID */
#define Robot_ID_Red_Hero         1 //�췽Ӣ��
#define Robot_ID_Red_Engineer     2 //�췽����
#define Robot_ID_Red_Infantry3    3 //�췽����3
#define Robot_ID_Red_Infantry4    4 //�췽����4
#define Robot_ID_Red_Infantry5    5 //�췽����5
#define Robot_ID_Red_Aerial       6 //�췽���˻�
#define Robot_ID_Red_Sentry       7 //�췽�ڱ�
#define Robot_ID_Red_Darts        8 //�췽����
#define Robot_ID_Red_Radar        9 //�췽�״�
#define Robot_ID_Blue_Hero      101 //����Ӣ��
#define Robot_ID_Blue_Engineer  102 //��������
#define Robot_ID_Blue_Infantry3 103 //��������3
#define Robot_ID_Blue_Infantry4 104 //��������4
#define Robot_ID_Blue_Infantry5 105 //��������5
#define Robot_ID_Blue_Aerial    106 //�������˻�
#define Robot_ID_Blue_Sentry    107 //�����ڱ�
#define Robot_ID_Blue_Darts     108 //��������
#define Robot_ID_Blue_Radar     109 //�����״�

/* �����˵ȼ� */
#define Robot_Level_1 1 //1��
#define Robot_Level_2 2 //2��
#define Robot_Level_3 3 //3��

/* ��Ѫ���� */
#define Hurt_Type_ArmoredPlate     0 //װ�װ��˺�
#define Hurt_Type_ModuleOffline    1 //ģ������
#define Hurt_Type_OverShootSpeed   2 //ǹ�ڳ�����
#define Hurt_Type_OverShootHeat    3 //ǹ�ܳ�����
#define Hurt_Type_OverChassisPower 4 //���̳�����
#define Hurt_Type_Collision        5 //װ��ײ��

/* ���������� */
#define Shooter_ID1_17mm 1 //1��17mm�������
#define Shooter_ID2_17mm 2 //2��17mm�������
#define Shooter_ID1_42mm 3 //1��42mm�������

/* ������Ϣ */
#define Dart_State_Open     0 //����բ�ſ���
#define Dart_State_Close    1 //����բ�Źر�
#define Dart_State_Changing 2 //���ڿ������߹ر���
#define Dart_Target_Outpost 0 //����Ŀ��Ϊǰ��վ
#define Dart_Target_Base    1 //����Ŀ��Ϊ����

/* ������ID */
#define Cilent_ID_Red_Hero       0x0101 //�췽Ӣ�۲�����
#define Cilent_ID_Red_Engineer   0x0102 //�췽���̲�����
#define Cilent_ID_Red_Infantry3  0x0103 //�췽����3������
#define Cilent_ID_Red_Infantry4  0x0104 //�췽����4������
#define Cilent_ID_Red_Infantry5  0x0105 //�췽����5������
#define Cilent_ID_Red_Aerial     0x0106 //�췽����
#define Cilent_ID_Blue_Hero      0x0165 //����Ӣ�۲�����
#define Cilent_ID_Blue_Engineer  0x0166 //�������̲�����
#define Cilent_ID_Blue_Infantry3 0x0167 //��������3������
#define Cilent_ID_Blue_Infantry4 0x0168 //��������4������
#define Cilent_ID_Blue_Infantry5 0x0169 //��������5������
#define Cilent_ID_Blue_Aerial    0x016A //��������

/* UI��������cmdID */
#define UI_DataID_Delete   0x100 //�ͻ���ɾ��ͼ��
#define UI_DataID_Draw1    0x101 //�ͻ��˻���1��ͼ��
#define UI_DataID_Draw2    0x102 //�ͻ��˻���2��ͼ��
#define UI_DataID_Draw5    0x103 //�ͻ��˻���5��ͼ��
#define UI_DataID_Draw7    0x104 //�ͻ��˻���7��ͼ��
#define UI_DataID_DrawChar 0x110 //�ͻ��˻����ַ�ͼ��

/* UIɾ������ */
#define UI_Delete_Invalid 0 //�ղ���
#define UI_Delete_Layer   1 //ɾ��ͼ��
#define UI_Delete_All     2 //ɾ������

/* UIͼ�β��� */
#define UI_Graph_invalid 0 //�ղ���
#define UI_Graph_Add     1 //����ͼ��
#define UI_Graph_Change  2 //�޸�ͼ��
#define UI_Graph_Delete  3 //ɾ��ͼ��

/* UIͼ������ */
#define UI_Graph_Line      0 //ֱ��
#define UI_Graph_Rectangle 1 //����
#define UI_Graph_Circle    2 //��Բ
#define UI_Graph_Ellipse   3 //��Բ
#define UI_Graph_Arc       4 //Բ��
#define UI_Graph_Float     5 //������
#define UI_Graph_Int       6 //����
#define UI_Graph_String    7 //�ַ���

/* UIͼ����ɫ */
#define UI_Color_Main   0 //������ɫ
#define UI_Color_Yellow 1 //��ɫ
#define UI_Color_Green  2 //��ɫ
#define UI_Color_Orange 3 //��ɫ
#define UI_Color_Purple 4 //��ɫ
#define UI_Color_Pink   5 //��ɫ
#define UI_Color_Cyan   6 //��ɫ
#define UI_Color_Black  7 //��ɫ
#define UI_Color_White  8 //��ɫ

/* 0x000X --------------------------------------------------------------------*/
typedef __packed struct  //0x0001 ����״̬����
{
    uint8_t  game_type : 4;
    uint8_t  game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} ext_game_status_t;

typedef __packed struct  //0x0002 �����������
{
    uint8_t winner;
} ext_game_result_t;

typedef __packed struct  //0x0003 ������Ѫ������
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP_t;

/* 0x010X --------------------------------------------------------------------*/
typedef __packed struct  //0x0101 �����¼�����
{
    uint32_t event_type;
} ext_event_data_t;

typedef __packed struct  //0x0102 ����վ������ʶ
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

typedef __packed struct //0x0103 ���󲹸�վ�������ݣ��ɲ����ӷ��ͣ�RM �Կ�����δ���ţ�
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_num;
} ext_supply_projectile_booking_t;

typedef __packed struct  //0x0104 ���о�����Ϣ
{
    uint8_t level;
    uint8_t foul_robot_id;
} ext_referee_warning_t;

typedef __packed struct  //0x0105 ���ڷ���ڵ���ʱ
{
    uint8_t dart_remaining_time;
} ext_dart_remaining_time_t;

/* 0x020X --------------------------------------------------------------------*/
typedef __packed struct  //0x0201 ����������״̬
{
    uint8_t  robot_id;
    uint8_t  robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;
    uint16_t shooter_id1_17mm_cooling_rate;
    uint16_t shooter_id1_17mm_cooling_limit;
    uint16_t shooter_id1_17mm_speed_limit;
    uint16_t shooter_id2_17mm_cooling_rate;
    uint16_t shooter_id2_17mm_cooling_limit;
    uint16_t shooter_id2_17mm_speed_limit;
    uint16_t shooter_id1_42mm_cooling_rate;
    uint16_t shooter_id1_42mm_cooling_limit;
    uint16_t shooter_id1_42mm_speed_limit;
    uint16_t chassis_power_limit;
    uint8_t  mains_power_gimbal_output  : 1;
    uint8_t  mains_power_chassis_output : 1;
    uint8_t  mains_power_shooter_output : 1;
} ext_game_robot_state_t;

typedef __packed struct  //0x0202 ʵʱ������������
{
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float    chassis_power;
    uint16_t chassis_power_buffer;
    uint16_t shooter_id1_17mm_cooling_heat;
    uint16_t shooter_id2_17mm_cooling_heat;
    uint16_t shooter_id1_42mm_cooling_heat;
} ext_power_heat_data_t;

typedef __packed struct  //0x0203 ������λ��
{
    float x;
    float y;
    float z;
    float yaw;
} ext_game_robot_pos_t;

typedef __packed struct  //0x0204 ����������
{
    uint8_t power_rune_buff;
} ext_buff_musk_t;

typedef __packed struct  //0x0205 ���л���������״̬
{
    uint8_t attack_time;
} aerial_robot_energy_t;

typedef __packed struct  //0x0206 �˺�״̬
{
    uint8_t armor_type : 4;
    uint8_t hurt_type  : 4;
} ext_robot_hurt_t;

typedef __packed struct  //0x0207 ʵʱ�����Ϣ
{
    uint8_t bullet_type;
    uint8_t shooter_id;
    uint8_t bullet_freq;
    float   bullet_speed;
} ext_shoot_data_t;

typedef __packed struct  //0x0208 �ӵ�ʣ�෢����
{
    uint16_t bullet_remaining_num_17mm;
    uint16_t bullet_remaining_num_42mm;
    uint16_t coin_remaining_num;
} ext_bullet_remaining_t;

typedef __packed struct  //0x0209 ������RFID״̬
{
    uint32_t rfid_status;
} ext_rfid_status_t;

typedef __packed struct  //0x020A ���ڻ����˿ͻ���ָ������
{
    uint8_t  dart_launch_opening_status;
    uint8_t  dart_attack_target;
    uint16_t target_change_time;
    uint16_t operate_launch_cmd_time;
} ext_dart_client_cmd_t;

/* 0x030X --------------------------------------------------------------------*/
typedef __packed struct  //0x0301 �����˼�ͨ�� ͷ�ṹ��
{
    uint16_t data_cmd_id;
    uint16_t sender_ID;
    uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

typedef __packed struct  //0x0301 �����˼�ͨ�� ���ݽṹ��
{
    uint8_t *data;
} robot_interactive_data_t;

typedef __packed struct  //0x0303 С��ͼ�·���Ϣ��ʶ
{
    float    target_position_x;
    float    target_position_y;
    float    target_position_z;
    uint8_t  commd_keyboard;
    uint16_t target_robot_ID;
} ext_robot_command_t;

typedef __packed struct  //0x0305 С��ͼ������Ϣ��ʶ
{
    uint16_t target_robot_ID;
    float    target_position_x;
    float    target_position_y;
} ext_client_map_command_t;

/* �Զ������UI�ṹ�� -------------------------------------------------------*/
typedef __packed struct  //����UI UIͼ������
{
    uint8_t  graphic_name[3];
    uint32_t operate_tpye: 3;
    uint32_t graphic_tpye: 3;
    uint32_t layer: 4;
    uint32_t color: 4;
    uint32_t start_angle: 9;
    uint32_t end_angle: 9;
    uint32_t width: 10;
    uint32_t start_x: 11;
    uint32_t start_y: 11;
    uint32_t radius: 10;
    uint32_t end_x: 11;
    uint32_t end_y: 11;
} graphic_data_struct_t;

typedef __packed struct  //����UI UI�ַ�������
{
    uint8_t  string_name[3];
    uint32_t operate_tpye: 3;
    uint32_t graphic_tpye: 3;
    uint32_t layer: 4;
    uint32_t color: 4;
    uint32_t start_angle: 9;
    uint32_t end_angle: 9;
    uint32_t width: 10;
    uint32_t start_x: 11;
    uint32_t start_y: 11;
    uint32_t null;
    uint8_t stringdata[30];
} string_data_struct_t;

typedef __packed struct  //����UI UIɾ��ͼ������
{
    uint8_t operate_tpye;
    uint8_t layer;
} delete_data_struct_t;

typedef __packed struct //����UI ����1��ͼ�������ṹ��
{
    frame_header_struct_t Referee_Transmit_Header;
    uint16_t CMD_ID;
    ext_student_interactive_header_data_t Interactive_Header;
    graphic_data_struct_t Graphic[1];
    uint16_t CRC16;
} UI_Graph1_t;

typedef __packed struct //����UI ����2��ͼ�������ṹ��
{
    frame_header_struct_t Referee_Transmit_Header;
    uint16_t CMD_ID;
    ext_student_interactive_header_data_t Interactive_Header;
    graphic_data_struct_t Graphic[2];
    uint16_t CRC16;
} UI_Graph2_t;

typedef __packed struct //����UI ����5��ͼ�������ṹ��
{
    frame_header_struct_t Referee_Transmit_Header;
    uint16_t CMD_ID;
    ext_student_interactive_header_data_t Interactive_Header;
    graphic_data_struct_t Graphic[5];
    uint16_t CRC16;
} UI_Graph5_t;

typedef __packed struct //����UI ����7��ͼ�������ṹ��
{
    frame_header_struct_t Referee_Transmit_Header;
    uint16_t CMD_ID;
    ext_student_interactive_header_data_t Interactive_Header;
    graphic_data_struct_t Graphic[7];
    uint16_t CRC16;
} UI_Graph7_t;

typedef __packed struct //����UI ����1�ַ��������ṹ��
{
    frame_header_struct_t Referee_Transmit_Header;
    uint16_t CMD_ID;
    ext_student_interactive_header_data_t Interactive_Header;
    string_data_struct_t String;
    uint16_t CRC16;
} UI_String_t;

typedef __packed struct  //����UI UIɾ��ͼ�������ṹ��
{
    frame_header_struct_t Referee_Transmit_Header;
    uint16_t CMD_ID;
    ext_student_interactive_header_data_t Interactive_Header;
    delete_data_struct_t Delete;
    uint16_t CRC16;
} UI_Delete_t;

/* Structs -------------------------------------------------------------------*/
/* protocol��ͷ�ṹ�� */
extern frame_header_struct_t Referee_Receive_Header;

/* 0x000X */
extern ext_game_status_t   Game_Status;
extern ext_game_result_t   Game_Result;
extern ext_game_robot_HP_t Game_Robot_HP;

/* 0x010X */
extern ext_event_data_t                Event_Data;
extern ext_supply_projectile_action_t  Supply_Projectile_Action;
extern ext_supply_projectile_booking_t Supply_Projectile_Booking;
extern ext_referee_warning_t           Referee_Warning;
extern ext_dart_remaining_time_t       Dart_Remaining_Time;

/* 0x020X */
extern ext_game_robot_state_t Game_Robot_State;
extern ext_power_heat_data_t  Power_Heat_Data;
extern ext_game_robot_pos_t   Game_Robot_Pos;
extern ext_buff_musk_t        Buff_Musk;
extern aerial_robot_energy_t  Aerial_Robot_Energy;
extern ext_robot_hurt_t       Robot_Hurt;
extern ext_shoot_data_t       Shoot_Data;
extern ext_bullet_remaining_t Bullet_Remaining;
extern ext_rfid_status_t      RFID_Status;
extern ext_dart_client_cmd_t  Dart_Client_Cmd;

/* 0x030X */
extern ext_student_interactive_header_data_t Student_Interactive_Header_Data;
extern robot_interactive_data_t              Robot_Interactive_Data;
extern ext_robot_command_t                   Robot_Command;
extern ext_client_map_command_t              Client_Map_Command;

/* ����UIר�ýṹ�� */
extern UI_Graph1_t UI_Graph1;
extern UI_Graph2_t UI_Graph2;
extern UI_Graph5_t UI_Graph5;
extern UI_Graph7_t UI_Graph7;
extern UI_String_t UI_String;
extern UI_Delete_t UI_Delete;

/* Functions -----------------------------------------------------------------*/
void Referee_StructInit(void);
void Referee_UARTInit(uint8_t *Buffer0, uint8_t *Buffer1, uint16_t BufferLength);

void Referee_UnpackFifoData(unpack_data_t *p_obj, fifo_s_t *referee_fifo);
void Referee_SolveFifoData(uint8_t *frame);

void UI_Draw_Line(graphic_data_struct_t *Graph,        //UIͼ�����ݽṹ��ָ��
                  char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
                  uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
                  uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
                  uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
                  uint16_t               Width,        //�߿�
                  uint16_t               StartX,       //��ʼ����X
                  uint16_t               StartY,       //��ʼ����Y
                  uint16_t               EndX,         //��ֹ����X
                  uint16_t               EndY);        //��ֹ����Y
void UI_Draw_Rectangle(graphic_data_struct_t *Graph,        //UIͼ�����ݽṹ��ָ��
                       char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
                       uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
                       uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
                       uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
                       uint16_t               Width,        //�߿�
                       uint16_t               StartX,       //��ʼ����X
                       uint16_t               StartY,       //��ʼ����Y
                       uint16_t               EndX,         //��ֹ����X
                       uint16_t               EndY);        //��ֹ����Y
void UI_Draw_Circle(graphic_data_struct_t *Graph,        //UIͼ�����ݽṹ��ָ��
                    char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
                    uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
                    uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
                    uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
                    uint16_t               Width,        //�߿�
                    uint16_t               CenterX,      //Բ������X
                    uint16_t               CenterY,      //Բ������Y
                    uint16_t               Radius);      //�뾶
void UI_Draw_Ellipse(graphic_data_struct_t *Graph,        //UIͼ�����ݽṹ��ָ��
                     char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
                     uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
                     uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
                     uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
                     uint16_t               Width,        //�߿�
                     uint16_t               CenterX,      //Բ������X
                     uint16_t               CenterY,      //Բ������Y
                     uint16_t               XHalfAxis,    //X���᳤
                     uint16_t               YHalfAxis);   //Y���᳤
void UI_Draw_Arc(graphic_data_struct_t *Graph,        //UIͼ�����ݽṹ��ָ��
                 char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
                 uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
                 uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
                 uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
                 uint16_t               StartAngle,   //��ʼ�Ƕ� [0,360]
                 uint16_t               EndAngle,     //��ֹ�Ƕ� [0,360]
                 uint16_t               Width,        //�߿�
                 uint16_t               CenterX,      //Բ������X
                 uint16_t               CenterY,      //Բ������Y
                 uint16_t               XHalfAxis,    //X���᳤
                 uint16_t               YHalfAxis);   //Y���᳤
void UI_Draw_Float(graphic_data_struct_t *Graph,        //UIͼ�����ݽṹ��ָ��
                   char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
                   uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
                   uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
                   uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
                   uint16_t               NumberSize,   //�����С
                   uint16_t               Significant,  //��Чλ��
                   uint16_t               Width,        //�߿�
                   uint16_t               StartX,       //��ʼ����X
                   uint16_t               StartY,       //��ʼ����Y
                   float                  FloatData);   //��������
void UI_Draw_Int(graphic_data_struct_t *Graph,        //UIͼ�����ݽṹ��ָ��
                 char                   GraphName[3], //ͼ���� ��Ϊ�ͻ��˵�����
                 uint8_t                GraphOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
                 uint8_t                Layer,        //UIͼ��ͼ�� [0,9]
                 uint8_t                Color,        //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
                 uint16_t               NumberSize,   //�����С
                 uint16_t               Width,        //�߿�
                 uint16_t               StartX,       //��ʼ����X
                 uint16_t               StartY,       //��ʼ����Y
                 int32_t                IntData);     //��������
void UI_Draw_String(string_data_struct_t *String,        //UIͼ�����ݽṹ��ָ��
                    char                  StringName[3], //ͼ���� ��Ϊ�ͻ��˵�����
                    uint8_t               StringOperate, //UIͼ�β��� ��ӦUI_Graph_XXX��4�ֲ���
                    uint8_t               Layer,         //UIͼ��ͼ�� [0,9]
                    uint8_t               Color,         //UIͼ����ɫ ��ӦUI_Color_XXX��9����ɫ
                    uint16_t              CharSize,      //�����С
                    uint16_t              StringLength,  //�ַ�������
                    uint16_t              Width,         //�߿�
                    uint16_t              StartX,        //��ʼ����X
                    uint16_t              StartY,        //��ʼ����Y
                    char                 *StringData);   //�ַ�������

void UI_PushUp_Graphs(uint8_t Counter, void *Graphs, uint8_t RobotID);
void UI_PushUp_String(UI_String_t *String, uint8_t RobotID);
void UI_PushUp_Delete(UI_Delete_t *Delete, uint8_t RobotID);
void get_chassis_power_and_buffer_and_max(fp32 *power, uint16_t *buffer,uint16_t *power_max);
extern void referee_usart_task();

#ifdef __cplusplus
}
#endif

#endif /* __REFEREE_H__ */
