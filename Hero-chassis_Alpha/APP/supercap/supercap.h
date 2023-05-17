#ifndef __SUPERCAP_H
#define __SUPERCAP_H

#include "usart.h"

#define Low_Voltage_Mode 		0
#define Medium_Voltage_Mode		1
#define High_Voltage_Mode 		2
extern uint8_t recvStr[53];
extern int supercap_volt;
extern float supercap_per;
void supercap(void);
void get_supercap_data(void);
#endif

