#include "math.h"
/**
  * @brief         ����ֵ������������ȡ����ֵ
  * @param[a]      ԭʼ����
  * @retval        ȡ����ֵ���ֵ
  */
int abs(int a)
{
	if(a<0) return -a;
	else return a;
}
/**
  * @brief         ����ֵ������������ȡ����ֵ
  * @param[a]      ԭʼ����
  * @retval        ȡ����ֵ���ֵ
  */
float fabs(float a)
{
	if(a<0.0f) return -a;
	else return a;
}
/**
  * @brief         ����ֵ������������ȡ����ֵ
  * @param[a]      ԭʼ����
  * @retval        ȡ����ֵ���ֵ
  */
int16_t int16_abs(int16_t a)
{
	if(a<0) return -a;
	else return a;
}
/**
  * @brief         ��С��Χ����
  * @param[maxx]   ��Χ���ֵ
  * @param[minn]   ��Χ��Сֵ
	* @param[a]   	 ��Ҫ���Ƶ�ֵ
	* @retval        ���޺󷵻ص�ֵ
  */

uint16_t limits_(uint16_t maxx,uint16_t minn,uint16_t a)
{
	if(a>maxx) return maxx;
	else if(a<minn) return minn;
	else return a;
}
/**	
  * @brief         						��������
  * @param[maxx]   						��Χ���ֵ
  * @param[minn]   						��Χ��Сֵ
	* @param[a]   	 						��Ҫ���Ƶ�ֵ
	* @param[maxx_actual]   		ʵ�����ֵ
	* @param[minn_actual]   	 	ʵ����Сֵ
	* @retval        						���ƺ��ֵ
  */
//��Χת��
float limits_change(int maxx,int minn,int a,int maxx_actual,int minn_actual)
{
	float b=(float)(a-minn_actual)/(float)(maxx_actual-minn_actual);
	float c=(float)b*(float)(maxx-minn);
	float ans=c+(float)minn;
	return ans;
}

