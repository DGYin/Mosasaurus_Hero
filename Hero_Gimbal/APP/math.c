#include "math.h"
/**
  * @brief         绝对值函数，用来获取绝对值
  * @param[a]      原始数据
  * @retval        取绝对值后的值
  */
int abs(int a)
{
	if(a<0) return -a;
	else return a;
}
/**
  * @brief         绝对值函数，用来获取绝对值
  * @param[a]      原始数据
  * @retval        取绝对值后的值
  */
float fabs(float a)
{
	if(a<0.0f) return -a;
	else return a;
}
/**
  * @brief         绝对值函数，用来获取绝对值
  * @param[a]      原始数据
  * @retval        取绝对值后的值
  */
int16_t int16_abs(int16_t a)
{
	if(a<0) return -a;
	else return a;
}
/**
  * @brief         大小范围限制
  * @param[maxx]   范围最大值
  * @param[minn]   范围最小值
	* @param[a]   	 所要限制的值
	* @retval        受限后返回的值
  */

uint16_t limits_(uint16_t maxx,uint16_t minn,uint16_t a)
{
	if(a>maxx) return maxx;
	else if(a<minn) return minn;
	else return a;
}
/**	
  * @brief         						功率限制
  * @param[maxx]   						范围最大值
  * @param[minn]   						范围最小值
	* @param[a]   	 						所要限制的值
	* @param[maxx_actual]   		实际最大值
	* @param[minn_actual]   	 	实际最小值
	* @retval        						限制后的值
  */
//范围转换
float limits_change(int maxx,int minn,int a,int maxx_actual,int minn_actual)
{
	float b=(float)(a-minn_actual)/(float)(maxx_actual-minn_actual);
	float c=(float)b*(float)(maxx-minn);
	float ans=c+(float)minn;
	return ans;
}

