#include "sent_task.h"
#include "can.h"
#include "usart.h"
#include "bsp_can.h"
#include "lk_pitch_turn.h"


int Relay_Set_State = 1; //1为电容供电，2为电池直连

uint8_t canTX_chassis(int16_t x, int16_t y, int8_t z, int8_t deviation)
{
    CAN_TxHeaderTypeDef canFrame;
    uint8_t data[8] = {0};
    uint32_t temp = 0;

    canFrame.IDE = CAN_ID_STD;
    canFrame.StdId = 0x007;
    canFrame.RTR = CAN_RTR_DATA;
    canFrame.DLC = 8;
    canFrame.TransmitGlobalTime = DISABLE;
    data[0] = x >> 8;
    data[1] = x & 0xff;
    data[2] = y >> 8;
    data[3] = y & 0xff;
    data[4] = z >> 8;
    data[5] = z & 0xff;
    data[6] = deviation >> 8;
    data[7] = deviation & 0xff;
    HAL_CAN_AddTxMessage(&hcan2, &canFrame, data, &temp);

    return temp;
}

uint8_t CAN_Tx_Mode(uint8_t Chassis_Mode, int Precision_Mode, int Chassis_Follow_Mode)
{
    CAN_TxHeaderTypeDef canFrame;
    uint8_t data[8] = {0};
    uint32_t temp = 0;

    canFrame.IDE = CAN_ID_STD;
    canFrame.StdId = 0x009;
    canFrame.RTR = CAN_RTR_DATA;
    canFrame.DLC = 8;
    canFrame.TransmitGlobalTime = DISABLE;
    data[0] = Chassis_Mode;
    data[1] = Precision_Mode;
    data[2] = Chassis_Follow_Mode;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    HAL_CAN_AddTxMessage(&hcan2, &canFrame, data, &temp);

    return temp;
}

uint8_t canTX_UI(int pitch, int mode, int Pitch_Temp)
{
    CAN_TxHeaderTypeDef canFrame;
    uint8_t data[8] = {0};
    uint32_t temp = 0;

    canFrame.IDE = CAN_ID_STD;
    canFrame.StdId = 0x010;
    canFrame.RTR = CAN_RTR_DATA;
    canFrame.DLC = 8;
    canFrame.TransmitGlobalTime = DISABLE;
    data[0] = pitch >> 8;
    data[1] = pitch & 0xff;
    data[2] = mode >> 8;
    data[3] = mode & 0xff;
    data[4] = Pitch_Temp >> 8;
    data[5] = Pitch_Temp & 0xff;
    data[6] = 0;
    data[7] = 0;
    HAL_CAN_AddTxMessage(&hcan2, &canFrame, data, &temp);

    return temp;
}


uint8_t canTX_pitch(int16_t pitch)              //pitch采用3508电机驱动方案
{
    CAN_TxHeaderTypeDef canFrame;
    uint8_t data[8] = {0};
    uint32_t temp = 0;

    canFrame.IDE = CAN_ID_STD;
    canFrame.StdId = 0x1ff;
    canFrame.RTR = CAN_RTR_DATA;
    canFrame.DLC = 8;
    canFrame.TransmitGlobalTime = DISABLE;
    data[0] = pitch >> 8;
    data[1] = pitch & 0xff;
    HAL_CAN_AddTxMessage(&hcan1, &canFrame, data, &temp);

    return temp;
}

//用于向瓴控Pitch电机发送消息。
void canTX_LK_Pitch_Motor(void)
{
    uint8_t data[8];
    uint32_t temp = 0;
    CAN_TxHeaderTypeDef canFrame;
    canFrame.IDE = CAN_ID_STD;
    canFrame.StdId = LK_Pitch_Motor_ID;
    canFrame.RTR = CAN_RTR_DATA;
    canFrame.DLC = 8;
    canFrame.ExtId = 0;
    canFrame.TransmitGlobalTime = DISABLE;
    for(int i = 0; i < 8; i++)
        data[i] = LK_Pitch_Motor_Send_Data[i];

    HAL_CAN_AddTxMessage(&hcan1, &canFrame, LK_Pitch_Motor_Send_Data, &temp);

}

uint8_t canTX_yaw(int16_t Yaw_Angle, int16_t yaw_current)
{
    CAN_TxHeaderTypeDef canFrame;
    uint8_t data[8] = {0};
    uint32_t temp = 0;

    canFrame.IDE = CAN_ID_STD;
    canFrame.StdId = 0x006;
    canFrame.RTR = CAN_RTR_DATA;
    canFrame.DLC = 8;
    canFrame.TransmitGlobalTime = DISABLE;
    data[0] = Yaw_Angle >> 8; //角度值发送
    data[1] = Yaw_Angle & 0xff;
    data[2] = yaw_current >> 8; //电流值发送
    data[3] = yaw_current & 0xff;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    HAL_CAN_AddTxMessage(&hcan2, &canFrame, data, &temp);

    return temp;
}

uint8_t canTX_Yaw_Current(int16_t yaw_current)//直接控制Yaw轴电机
{
    CAN_TxHeaderTypeDef canFrame;
    uint8_t data[8] = {0};
    uint32_t temp = 0;

    canFrame.IDE = CAN_ID_STD;
    canFrame.StdId = 0x2ff;
    canFrame.RTR = CAN_RTR_DATA;
    canFrame.DLC = 8;
    canFrame.TransmitGlobalTime = DISABLE;
    data[0] = yaw_current >> 8;
    data[1] = yaw_current & 0Xff;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    HAL_CAN_AddTxMessage(&hcan2, &canFrame, data, &temp);

    return temp;
}

uint8_t canTX_fric(int16_t left, int16_t right)
{
    CAN_TxHeaderTypeDef canFrame;
    uint8_t data[8] = {0};
    uint32_t temp = 0;

    canFrame.IDE = CAN_ID_STD;
    canFrame.StdId = 0x200;
    canFrame.RTR = CAN_RTR_DATA;
    canFrame.DLC = 8;
    canFrame.TransmitGlobalTime = DISABLE;
    data[0] = left >> 8;
    data[1] = left & 0xff;
    data[2] = right >> 8;
    data[3] = right & 0xff;
    //	data[4]=0;
    //	data[5]=0;
    data[6] = 0;
    data[7] = 0;
    HAL_CAN_AddTxMessage(&hcan1, &canFrame, data, &temp);

    return temp;
}

uint8_t canTX_trigger(uint8_t trigger)
{
    CAN_TxHeaderTypeDef canFrame;
    uint8_t data[8] = {0};
    uint32_t temp = 0;

    canFrame.IDE = CAN_ID_STD;
    canFrame.StdId = Chassis_Shoot_Task_Tx_ID;
    canFrame.RTR = CAN_RTR_DATA;
    canFrame.DLC = 8;
    canFrame.TransmitGlobalTime = DISABLE;
	
	extern int Fric_Switch_Flag;
	extern int Shoot_Num;
    data[0] = trigger;
    data[1] = Fric_Switch_Flag;
    data[2] = Shoot_Num;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    HAL_CAN_AddTxMessage(&hcan2, &canFrame, data, &temp);

    return temp;
}

void canTX_Invert_Flag(uint8_t Bool_Invert_Flag)
{
    CAN_TxHeaderTypeDef canFrame;
    uint8_t data[8] = {0};
    uint32_t temp = 0;

    canFrame.IDE = CAN_ID_STD;
    canFrame.StdId = CAN_Invert_Flag_Trans_ID;
    canFrame.RTR = CAN_RTR_DATA;
    canFrame.DLC = 8;
    canFrame.TransmitGlobalTime = DISABLE;

    data[0] = Bool_Invert_Flag;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    HAL_CAN_AddTxMessage(&hcan2, &canFrame, data, &temp);
}

void canTX_Relay_Set_Mode(void)
{
    CAN_TxHeaderTypeDef canFrame;
    uint8_t data[8] = {0};
    uint32_t temp = 0;

    canFrame.IDE = CAN_ID_STD;
    canFrame.StdId = Relay_Mode_Set_ID;
    canFrame.RTR = CAN_RTR_DATA;
    canFrame.DLC = 8;
    canFrame.TransmitGlobalTime = DISABLE;

    data[0] = Relay_Set_State;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    HAL_CAN_AddTxMessage(&hcan2, &canFrame, data, &temp);
}