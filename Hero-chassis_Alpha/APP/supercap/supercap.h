#ifndef __SUPERCAP_H
#define __SUPERCAP_H



#define Low_Voltage_Mode 		0
#define Medium_Voltage_Mode		1
#define High_Voltage_Mode 		2

#define	Supercap_Connected		1
#define Supercap_Disconnected	0
/***********************************************************
*@Brief	���ⲿ���õı���
***********************************************************/
extern int supercap_volt;
extern float supercap_per;
extern int Supercap_Connection_Status;
/***********************************************************
*@Brief	���ⲿ���õĺ���
***********************************************************/
void supercap(int S_Cnt, int MS_Cnt);
#endif

