#include "supercap.h"
#include "referee.h"
uint16_t power_limit=0;
uint16_t supercap_volt=0;  //�������ݵ�ѹ
float supercap_per=0;   //�������ݵ����ٷֱ�
uint8_t rec_super=0;
uint8_t cnt=0;
uint8_t send_data[4]={0};
uint8_t state=0;
/**
  * @breif         �������ݣ����ݲ�ͬ�Ĺ������޸������ϵ�Ƭ����������
  * @param[in]     none 
	* @param[out]    ����ͬ���ʵ��ַ�
  * @retval        none     
  */
void supercap(void)
{
	get_chassis_power_limit(&power_limit);  //��ȡ����ϵͳ����
	power_limit = 40;
//	power_limit=68;
//	power_limit++;
//	if(power_limit>999)power_limit=0;
	send_data[0]=((uint8_t)(power_limit/100))%10;  //��λ
	send_data[1]=((uint8_t)(power_limit/10))%10;   //ʮλ
	send_data[2]=((uint8_t)power_limit)%10;        //��λ
	send_data[3]='#';                              //����֡
	state=HAL_UART_Transmit(&huart6, send_data, 4, 100);
	get_supercap_data();
	
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART6)  //�ж�Ϊ�������ݴ����ж�
	{
		/*���ݴ���*/
		if(rec_super!='#')
		{
			if(cnt<=1)
			{
				recvStr[cnt]=rec_super;  //��ѹ ����
				cnt++;
			}			
		}
		else
		{
			
			cnt=0;
		}
		//�����жϺ��ٴ��ֶ����������ж�
		HAL_UART_Receive_IT(&huart6,&rec_super,1); 
	}
}


void get_supercap_data(void)  //ת��
{
	supercap_volt=(uint16_t)recvStr[0];
	supercap_per=(supercap_volt*supercap_volt-13.0*13.0)/(23.0*23.0-13.0*13.0); //���������������㹫ʽ
}