#include "supercap.h"
#include "referee.h"
int Relay_State = 1; //继电器控制值。1为电容供电，0为电池供电。
uint16_t power_limit = 0;
uint16_t supercap_volt = 0; //超级电容电压
float supercap_per = 0; //超级电容电量百分比
uint8_t rec_super = 0;
uint8_t cnt = 0;
uint8_t send_data[4] = {0};
/**
  * @breif         超级电容，根据不同的功率上限给电容上单片机发送数据
  * @param[in]     none
	* @param[out]    代表不同功率的字符
  * @retval        none
  */
extern void UartTX_Super_Capacitor(int Power_Limitation,int Relay_State);
void supercap(void)
{
    get_chassis_power_limit(&power_limit);  //获取裁判系统数据
	UartTX_Super_Capacitor(power_limit, Relay_State);
}


void get_supercap_data(void)  //转换
{
    supercap_volt = (uint16_t)recvStr[0];
    supercap_per = (uint16_t)recvStr[1];
}