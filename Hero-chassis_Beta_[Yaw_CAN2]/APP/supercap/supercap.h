#ifndef __SUPERCAP_H
#define __SUPERCAP_H

#include "usart.h"

extern uint8_t recvStr[53]; 
extern uint16_t supercap_volt;
extern float supercap_per;
void supercap(void);
void get_supercap_data(void);
#endif

