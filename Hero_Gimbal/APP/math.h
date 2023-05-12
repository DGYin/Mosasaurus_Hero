#ifndef __math_H
#define __math_H

#include "stm32f4xx.h"

int abs(int a);
float fabs(float a);
int16_t int16_abs(int16_t a);

uint16_t limits_(uint16_t maxx,uint16_t minn,uint16_t a);
float limits_change(int maxx,int minn,int a,int maxx_actual,int minn_actual);

#endif
