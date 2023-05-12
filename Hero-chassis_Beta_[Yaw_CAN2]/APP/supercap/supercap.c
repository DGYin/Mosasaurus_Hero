#include "supercap.h"
#include "referee.h"
uint16_t power_limit=0;
uint16_t supercap_volt=0;  //超级电容电压
float supercap_per=0;   //超级电容电量百分比
uint8_t rec_super=0;
uint8_t cnt=0;
uint8_t send_data[4]={0};
uint8_t state=0;
/**
  * @breif         超级电容，根据不同的功率上限给电容上单片机发送数据
  * @param[in]     none 
	* @param[out]    代表不同功率的字符
  * @retval        none     
  */
void supercap(void)
{
	get_chassis_power_limit(&power_limit);  //获取裁判系统数据
	power_limit = 40;
//	power_limit=68;
//	power_limit++;
//	if(power_limit>999)power_limit=0;
	send_data[0]=((uint8_t)(power_limit/100))%10;  //百位
	send_data[1]=((uint8_t)(power_limit/10))%10;   //十位
	send_data[2]=((uint8_t)power_limit)%10;        //个位
	send_data[3]='#';                              //结束帧
	state=HAL_UART_Transmit(&huart6, send_data, 4, 100);
	get_supercap_data();
	
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART6)  //判断为超级电容串口中断
	{
		/*数据处理*/
		if(rec_super!='#')
		{
			if(cnt<=1)
			{
				recvStr[cnt]=rec_super;  //电压 电量
				cnt++;
			}			
		}
		else
		{
			
			cnt=0;
		}
		//进入中断后再次手动开启接收中断
		HAL_UART_Receive_IT(&huart6,&rec_super,1); 
	}
}


void get_supercap_data(void)  //转换
{
	supercap_volt=(uint16_t)recvStr[0];
	supercap_per=(supercap_volt*supercap_volt-13.0*13.0)/(23.0*23.0-13.0*13.0); //超级电容容量计算公式
}