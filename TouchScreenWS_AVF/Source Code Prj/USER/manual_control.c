/************************************版权申明********************************************
**                            深圳英捷思科技有限公司
**                             http://www.
**-----------------------------------文件信息--------------------------------------------
--------------------------------------------------------------------------------------*/
#include "hmi_driver.h"

#include "cmd_queue.h"
#include "cmd_process.h"
#include "stdio.h"
#include "stdbool.h"   
#include "hw_config.h"
#include "ulitity.h"
#include "string.h"

#include "stm32f10x.h"
#include "usart.h"
#include "relays.h"
#include "delay.h"
#include "adc.h"
#include "dac.h"
#include "macros.h"
#include "error_handle.h"
#include "manual_control.h"
#include "time_control.h"

#ifdef USE_PWM
extern uint8 DutyCycle;
void PWM_manager(void)	  //every 100us
{
	static uint8 cnt1 = 0;
	static uint8 cnt2 = 0;
	if(DutyCycle == 10)
	{

		led_run = ON;
	}
	else if(DutyCycle == 0)
	{
		led_run = OFF;

	}
	else
	{
		if(cnt1 < DutyCycle)
		{
			cnt1++;
			if(cnt1 == DutyCycle)
			{
				cnt2 = 0;
				led_run = OFF;
			}
	
		}
		else if(cnt2 < 10 - DutyCycle)
		{
			cnt2++;
			if(cnt2 == 10 - DutyCycle)
			{
				cnt1 = 0;
				led_run = ON;
			}
	
		}
	}		

}
#endif
void HeartBeating()
{
	led_run = 1 - led_run;
}
