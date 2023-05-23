#include "supercap.h"
#include "referee.h"
int Capacitor_State = 0; 	//����ʣ������״̬��0Ϊ�ͣ�1Ϊ��

int supercap_volt = 24; 		//�������ݵ�ѹ
int Power_Mode = 0, Last_Power_Mode = 0;
float supercap_per = 0; 	//�������ݵ����ٷֱ�
uint8_t rec_super = 0;
uint8_t cnt = 0;
uint8_t send_data[4] = {0};
/**
  * @breif         �������ݣ����ݲ�ͬ�Ĺ������޸������ϵ�Ƭ����������
  * @param[in]     none
	* @param[out]    ����ͬ���ʵ��ַ�
  * @retval        none
  */
uint16_t UART_TX_Power_Max = 0, UART_TX_Buffer = 0;
fp32 UART_TX_Power = 0;
extern void UartTX_Super_Capacitor(int Power_Limitation, fp32 Power);
extern void canTX_To_Beta_Power_Limit(int Power_Mode);
void supercap(void)
{
    get_chassis_power_and_buffer_and_max(&UART_TX_Power, &UART_TX_Buffer, &UART_TX_Power_Max);  //��ȡ����ϵͳ����
	//UART_TX_Power_Max = 60; //������
	UartTX_Super_Capacitor(UART_TX_Power_Max, UART_TX_Power);
	//����Beta��ļ��ٶ�
	supercap_volt = 240;
	if (supercap_volt>20)
		Power_Mode = High_Voltage_Mode;
	if (supercap_volt<=15)
		Power_Mode = Low_Voltage_Mode;
	if (Last_Power_Mode==Low_Voltage_Mode && supercap_volt<18) Power_Mode = Low_Voltage_Mode;
	canTX_To_Beta_Power_Limit(Power_Mode);
	Last_Power_Mode = Power_Mode;
}
