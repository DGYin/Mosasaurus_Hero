#include "supercap.h"
#include "referee.h"
int Relay_State = 0; //�̵�������ֵ��1Ϊ���ݹ��磬0Ϊ��ع��硣

uint16_t supercap_volt = 0; //�������ݵ�ѹ
float supercap_per = 0; //�������ݵ����ٷֱ�
uint8_t rec_super = 0;
uint8_t cnt = 0;
uint8_t send_data[4] = {0};
/**
  * @breif         �������ݣ����ݲ�ͬ�Ĺ������޸������ϵ�Ƭ����������
  * @param[in]     none
	* @param[out]    ����ͬ���ʵ��ַ�
  * @retval        none
  */
extern void UartTX_Super_Capacitor(int Power_Limitation, fp32 Power);
void supercap(void)
{
	uint16_t Power_Max = 0, Buffer = 0;
	fp32 Power = 0;
    get_chassis_power_and_buffer_and_max(&Power, &Buffer, &Power_Max);  //��ȡ����ϵͳ����
	UartTX_Super_Capacitor(Power_Max, Power);
}


void get_supercap_data(void)  //ת��
{
    supercap_volt = (uint16_t)recvStr[0];
    supercap_per = (uint16_t)recvStr[1];
}