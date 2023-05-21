#include "bsp_can.h"
/*
	* @ brief   canͨ���˲�����ʼ��
	* @ param   none
	* @ retvel  ״ֵ̬
*/

uint8_t LK_Pitch_Motor_Receive_Data[8];
uint8_t LK_Pitch_Motor_Send_Data[8];

uint8_t Bsp_canInit(void)    
{
	uint8_t status=0;
	CAN_FilterTypeDef canFilter;
	
	/*can1��ʼ��*/
	//MX_CAN1_Init();             								//MX���ɵĴ���
	canFilter.FilterBank=1;    																//ɸѡ����1
	canFilter.FilterIdHigh=0;
	canFilter.FilterIdLow=0;
	canFilter.FilterMaskIdHigh=0;
	canFilter.FilterMaskIdLow=0;
	canFilter.FilterMode=CAN_FILTERMODE_IDMASK;  							//����ģʽ
	canFilter.FilterActivation=CAN_FILTER_ENABLE;							//����
	canFilter.FilterScale=CAN_FILTERSCALE_32BIT; 							//32λģʽ
	canFilter.FilterFIFOAssignment=CAN_FILTER_FIFO0; 					//���ӵ�fifo0
	canFilter.SlaveStartFilterBank=14;												//can2ɸѡ����ʼ���
	
	status=HAL_CAN_ConfigFilter(&hcan1,&canFilter);					//���ù�����
	
	/*can2��ʼ��*/
	//MX_CAN2_Init();             								//MX���ɵĴ���
	canFilter.FilterBank=15;    															//ɸѡ����15
	status=HAL_CAN_ConfigFilter(&hcan2,&canFilter);					//���ù�����
	
	/*�뿪��ʼģʽ*/
	HAL_CAN_Start(&hcan1);				
	HAL_CAN_Start(&hcan2);
	
	/*���ж�*/
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);       //can1 ����fifo 0��Ϊ���ж�
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);       //can2 ����fifo 0��Ϊ���ж�
	return status;

}



