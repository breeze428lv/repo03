/************************************��Ȩ����********************************************
**                             ���ݴ�ʹ��Ƽ����޹�˾
**                             http://www.gz-dc.com
**-----------------------------------�ļ���Ϣ--------------------------------------------
** �ļ�����:   ulitity.c
** �޸�ʱ��:   2018-05-18
** �ļ�˵��:   �û�MCU��������������
** ����֧�֣�  Tel: 020-82186683  Email: hmi@gz-dc.com Web:www.gz-dc.com
--------------------------------------------------------------------------------------*/
#include "ulitity.h"

extern volatile  uint32 timer_tick_count;


/*!
* ��    �ƣ� delay_ms
* ��    �ܣ� ��ʱn����
* ��ڲ����� n-��ʱʱ��
* ���ڲ����� ��
*/
void delay_ms(uint32 delay)
{
    uint32 tick = timer_tick_count;
    while(1)
    {
        if(timer_tick_count-tick>delay/10)
            break;
    }
}
/*!
*��    �ƣ� systicket_init
*��    �� �����Ķ�ʱ�� 10ms
*��ڲ����� ��
*���ڲ����� ��
*/
void systicket_init(void)
{
   /*Ҫע�⺯�����õĴΣ�
     ��SysTick_Config(uint32_t ticks)��
	 ��SysTick_CLKSourceConfig(uint32_t SysTick_CLKSource)*/ 

	SysTick_Config(48000000/100);                              //һ���ӽ���100���ж� Period: 10ms
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);	  //48M
   /* SysTick_Config(6000000/100); 
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	  //48/8=6M		*/
}
