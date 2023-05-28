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
#include "settings.h"
#include "pump_running.h"

#define ICON_IND_START            0
#define ICON_IND_STOP             1
#define ICON_IND_MANUAL           2
#define ICON_IND_PARA_SETTING     3
#define ICON_IND_SYSTEM_SETTING   4

extern uint8 pump_enable_tbl[];

extern uint8 TempMonitorEnable;

extern uint8 ErrorCounter;
extern uint8 SysRunning;
extern uint8 SoftStopping;

extern MY_DATE StopDate;
extern uint8 pump_usablilty_tbl[];
extern uint8 PumpGroupMode;

const uint8 button_id_tbl[5][2] = {							  
							   {ICON_START_MASK       , BTN_START_AUTO_SCREEN_MAIN },
							   {ICON_STOP_MASK        , BTN_STOP_AUTO_SCREEN_MAIN  },	 
							   {ICON_MANUAL_MASK      , BTN_MANUAL_SETTING         },
							   {ICON_PARA_SETTING_MASK, BTN_PARAMETER_SETTING      },
							   {ICON_SYS_SETTING_MASK , BTN_SYSTEM_SETTING         }

};

int button_frozen[6] = {0, 1, 0, 0, 0, 0}; 

int weight[4][6] = {
						
						{1, -1,  1,  1,  0,  0},  //start 0/1
						{0,  0,  1,  0,  0,  0},  //disable manual 2/3
						{1,  1,  1,  0,  0,  0},      //expire   4/5
						{1,  1,  1,  0,  0,  0},      //error   6/7


};

const char* cmd_str[8] = {
							"Start",
							"Stop",
							"Disable manual",
							"Enable manual",
							"Expire",
							"Release expire",
							"error",
							"remove error"

};

/*
typedef struct{
    unsigned char icon_pump_fc;
    unsigned char icon_pump_pf;
    unsigned char icon_pump_mask;
    unsigned char icon_temp_mask;
    unsigned char icon_freq;
    unsigned char icon_err;
   
     
} TYPE_PUMP_ICONS;
*/
									//  mode  fc     pf    flow    pump   temp   freq     err
									//                             mask   mask
const TYPE_PUMP_ICONS PumpIcons[6] ={
										{73,   2 , 	1  ,  68,     40, 	120, 	25, 	59 },
										{67,   4 , 	3  ,  62,     41, 	121, 	28, 	58 },
										{69,   6 , 	5  ,  63,  	  42, 	122, 	29, 	60 },
										{70,   45, 	46 ,  64,  	  47, 	56 , 	32, 	49 },
										{71,   51, 	50 ,  65,	  52, 	57 , 	39, 	53 },
										{72,   8 , 	255,  66,	  43, 	123, 	44, 	61 }


};

void display_int(int* a, int len)
{
	int i;
	
  	for(i = 0; i < len; i++)
    {
    	printf("  %d  ", a[i]);
    }
    printf("\n\n");
}

int get_sig(uint8 i)//0---1   1----  -1       a * 0 + b = 1   a + b = -1
{
	return 1 - (i % 2) * 2;
}



void refresh_controls(void)
{
	uint8 i, status;
	for(i = 0; i < 4; i++)
	{
		status = (button_frozen[i] >= 1)? OFF : ON;

		SetControlVisiable(SCREEN_MAIN_1, button_id_tbl[i][0], 1 - status);
		delay_ms(100);
		SetControlEnable(SCREEN_MAIN_1, button_id_tbl[i][1], status);
		delay_ms(100);

	}
	SetControlEnable(SCREEN_MAIN_1, BTN_ERROR_CHECKING, TRUE);
	delay_ms(100);
	SetControlVisiable(SCREEN_MAIN_1, ICON_SYS_SETTING_MASK, INVISIBLE);
	delay_ms(100);
	SetControlEnable(SCREEN_MAIN_1, BTN_SYSTEM_SETTING, TRUE);
	delay_ms(100);

}


void Pump_Icons_Initialization()	  //???
{
	update_pump_icon(0, MOD_NUL);
	update_pump_icon(1, MOD_NUL);
	update_pump_icon(2, MOD_NUL);
	update_pump_icon(3, MOD_NUL);
}

/****************************************************************
 *	Hide Controls:
 *      SCREEN_TIME_CTR_3-> TXT_TC_INPUT_ERROR 
 *      SCREEN_TIME_CTR_3-> ICON_TC_INPUT_ERROR
 *      SCREEN_MAIN_1-> TXT_ERROR_REMINDING, 
 *		SCREEN_MAIN_1-> ICON_TIMER_ON
 *		SCREEN_MAIN_1-> ICON_MANUAL_RUNNING



*****************************************************************/
void InitVisibility()
{
#ifndef FATIGE_CLOCK
//	SetControlVisiable(1, 28, 0);
//	delay_ms(100);
#endif
#ifdef TRAFFIC_LIGHT
//	SetControlVisiable(1, 47, 1);
#endif


	SetControlVisiable(SCREEN_TIME_CTR_3, ICON_TC_INPUT_ERROR, INVISIBLE);
	delay_ms(100);
    SetControlVisiable(SCREEN_PUMP_GROUPING, ICON_INPUT_ERROR1, INVISIBLE);
	delay_ms(100);
	SetControlVisiable(SCREEN_PUMP_GROUPING, ICON_INPUT_ERROR2, INVISIBLE);
	delay_ms(100);
	SetControlVisiable(SCREEN_PUMP_GROUPING, ICON_INPUT_ERROR3, INVISIBLE);
	delay_ms(100);

	SetControlEnable(SCREEN_MAIN_1, BTN_STOP_AUTO_SCREEN_MAIN, FALSE);
	delay_ms(100);

/*	Pump_Icons_Initialization();	  2020-6-9
	delay_ms(100);*/

#ifdef DBG_WDG_TEST
	SetControlVisiable(SCREEN_MAIN_1, 50, VISIBLE);
#endif
#ifdef DBG_MANUAL_FSW
	delay_ms(100);
	SetControlVisiable(SCREEN_MAIN_1, 51, VISIBLE);
#endif



}

void set_button_visible(uint8 ind, uint8 usable)
{
	SetControlVisiable(SCREEN_MAIN_1, button_id_tbl[ind][0], 1 - usable);
	delay_ms(100);
	SetControlEnable(SCREEN_MAIN_1, button_id_tbl[ind][1], usable);
	delay_ms(100);
}
extern uint8 AvfMode;
uint8 pump_err_filter(void)
{			//  pump1 error under fixed-fc mode is excluded					   //at least one pump is workable
   if ((AvfMode == FALSE) && (PumpGroupMode == 2) && (pump_usablilty_tbl[0] == FALSE)) return 0;
   if ((pump_usablilty_tbl[0] == TRUE) || (pump_usablilty_tbl[1] == TRUE) ||\
       (pump_usablilty_tbl[2] == TRUE) || (pump_usablilty_tbl[3] == TRUE) ||\
	   (pump_usablilty_tbl[4] == TRUE))
   {
   		return 1;
   } 
   else
   {
   		return 0;
   }
   
   // (!((AvfMode == FALSE) && (PumpGroupMode == 2) && (pump_usablilty_tbl[0] == FALSE))) && ((pump_usablilty_tbl[0] == TRUE) || (pump_usablilty_tbl[1] == TRUE) || (pump_usablilty_tbl[2] == TRUE)); 
	//20210625  
}
void RefreshButtons(void)
{
   uint8 filter;
   if(SysRunning)
   {
	   set_button_visible(ICON_IND_STOP, TRUE);
	   set_button_visible(ICON_IND_START, FALSE);
	   set_button_visible(ICON_IND_MANUAL, FALSE);
	   set_button_visible(ICON_IND_PARA_SETTING, FALSE);
	   set_button_visible(ICON_IND_SYSTEM_SETTING, FALSE);

	   button_frozen[ICON_IND_START] = TRUE;
	   button_frozen[ICON_IND_STOP] = FALSE;

   }
   else	  //not running
   {
		set_button_visible(ICON_IND_STOP, FALSE);
   		if(ErrorCounter > 0)
		{			
			set_button_visible(ICON_IND_START, FALSE);
			set_button_visible(ICON_IND_MANUAL, FALSE);
			set_button_visible(ICON_IND_PARA_SETTING, TRUE);
			set_button_visible(ICON_IND_SYSTEM_SETTING, TRUE);

			button_frozen[ICON_IND_START] = TRUE;
		}
		else
		{

			if(SoftStopping)
		    {
				set_button_visible(ICON_IND_START, FALSE);
				button_frozen[ICON_IND_START] = TRUE;
		    }
	   		else	 //steadily	 stopped
		    {
				filter = pump_err_filter();
				set_button_visible(ICON_IND_START, filter);
				button_frozen[ICON_IND_START] = 1 - filter;

				set_button_visible(ICON_IND_PARA_SETTING, TRUE);				
				set_button_visible(ICON_IND_SYSTEM_SETTING, TRUE);

				if(StopDate.disable_manual == FALSE)
				{ 
					set_button_visible(ICON_IND_MANUAL, TRUE);
				}
				else
				{
					set_button_visible(ICON_IND_MANUAL, FALSE);
				}
			}
		}
		
		button_frozen[ICON_IND_STOP] = TRUE;		
   }
#ifdef DBG_FUNC_INFO
/*	printf("button_frozen[]:");
    display_int(button_frozen, 6);	  */
#endif
}


void SetButtonUsability(uint8 status)    	
{
	if(status == ON)
	{
		SetControlVisiable(SCREEN_MAIN_1, ICON_STOP_MASK, INVISIBLE);
		delay_ms(100);
		SetControlEnable(SCREEN_MAIN_1, BTN_STOP_AUTO_SCREEN_MAIN, TRUE);
		delay_ms(100);

		SetControlVisiable(SCREEN_MAIN_1, ICON_START_MASK, VISIBLE);
		delay_ms(100);
		SetControlEnable(SCREEN_MAIN_1, BTN_START_AUTO_SCREEN_MAIN, FALSE);
		delay_ms(100);

		SetControlVisiable(SCREEN_MAIN_1, ICON_MANUAL_MASK, VISIBLE);
		delay_ms(100);
		SetControlEnable(SCREEN_MAIN_1, BTN_MANUAL_SETTING, FALSE);
		delay_ms(100);

		SetControlVisiable(SCREEN_MAIN_1, ICON_PARA_SETTING_MASK, VISIBLE);
		delay_ms(100);
		SetControlEnable(SCREEN_MAIN_1, BTN_PARAMETER_SETTING, FALSE);
		delay_ms(100);

		SetControlVisiable(SCREEN_MAIN_1, ICON_SYS_SETTING_MASK, VISIBLE);
		delay_ms(100);
		SetControlEnable(SCREEN_MAIN_1, BTN_SYSTEM_SETTING, FALSE);
		delay_ms(100);	
	}
	else if(status == OFF)
	{
		SetControlVisiable(SCREEN_MAIN_1, ICON_STOP_MASK, VISIBLE);
		delay_ms(100);
		SetControlEnable(SCREEN_MAIN_1, BTN_STOP_AUTO_SCREEN_MAIN, FALSE);
		delay_ms(100);

		SetControlVisiable(SCREEN_MAIN_1, ICON_START_MASK, INVISIBLE);
		delay_ms(100);
		SetControlEnable(SCREEN_MAIN_1, BTN_START_AUTO_SCREEN_MAIN, TRUE);
		delay_ms(100);
		
		if(StopDate.disable_manual == FALSE)
		{ 
			SetControlVisiable(SCREEN_MAIN_1, ICON_MANUAL_MASK, INVISIBLE);
			delay_ms(100);
			SetControlEnable(SCREEN_MAIN_1, BTN_MANUAL_SETTING, TRUE);
			delay_ms(100);
		}
		
		SetControlVisiable(SCREEN_MAIN_1, ICON_PARA_SETTING_MASK, INVISIBLE);
		delay_ms(100);
		SetControlEnable(SCREEN_MAIN_1, BTN_PARAMETER_SETTING, TRUE);
		delay_ms(100);
		
		SetControlVisiable(SCREEN_MAIN_1, ICON_SYS_SETTING_MASK, INVISIBLE);
		delay_ms(100);
		SetControlEnable(SCREEN_MAIN_1, BTN_SYSTEM_SETTING, TRUE);
		delay_ms(100);		


	}
	else if(status == BUTTON_STATUS_ERROR)				 //ERROR
	{
		//Start
		SetControlVisiable(SCREEN_MAIN_1, ICON_START_MASK, VISIBLE); 
		delay_ms(100);
		SetControlEnable(SCREEN_MAIN_1, BTN_START_AUTO_SCREEN_MAIN, FALSE);
		delay_ms(100);

		//Stop
		SetControlVisiable(SCREEN_MAIN_1, ICON_STOP_MASK, VISIBLE);
		delay_ms(100);
		SetControlEnable(SCREEN_MAIN_1, BTN_STOP_AUTO_SCREEN_MAIN, FALSE);
		delay_ms(100);

		SetControlVisiable(SCREEN_MAIN_1, ICON_MANUAL_MASK, VISIBLE);
		delay_ms(100);
		SetControlEnable(SCREEN_MAIN_1, BTN_MANUAL_SETTING, FALSE);
		delay_ms(100);
		
		SetControlVisiable(SCREEN_MAIN_1, ICON_PARA_SETTING_MASK, INVISIBLE);
		delay_ms(100);
		SetControlEnable(SCREEN_MAIN_1, BTN_PARAMETER_SETTING, TRUE);
		delay_ms(100);
		
		SetControlVisiable(SCREEN_MAIN_1, ICON_SYS_SETTING_MASK, INVISIBLE);
		delay_ms(100);
		SetControlEnable(SCREEN_MAIN_1, BTN_SYSTEM_SETTING, TRUE);
		delay_ms(100);		
	 
	}
	else if(status == BUTTON_STATUS_MANUAL_DISABLED)				 //ERROR
	{
		printf("status == BUTTON_STATUS_MANUAL_DISABLED\n");
		SetControlVisiable(SCREEN_MAIN_1, ICON_MANUAL_MASK, VISIBLE);
		delay_ms(100);
		SetControlEnable(SCREEN_MAIN_1, BTN_MANUAL_SETTING, FALSE);
		delay_ms(100);
		
	}
	else if(status == BUTTON_STATUS_MANUAL_ENABLED)				 //ERROR
	{

		SetControlVisiable(SCREEN_MAIN_1, ICON_MANUAL_MASK, INVISIBLE);
		delay_ms(100);
		SetControlEnable(SCREEN_MAIN_1, BTN_MANUAL_SETTING, TRUE);
		delay_ms(100);
		
	}  
	else if(status == BUTTON_STATUS_LOCK_ALL)
	{
		SetControlVisiable(SCREEN_MAIN_1, ICON_START_MASK, VISIBLE);
		delay_ms(100);
		SetControlEnable(SCREEN_MAIN_1, BTN_START_AUTO_SCREEN_MAIN, FALSE);
		delay_ms(100);

		SetControlVisiable(SCREEN_MAIN_1, ICON_STOP_MASK, VISIBLE);
		delay_ms(100);
		SetControlEnable(SCREEN_MAIN_1, BTN_STOP_AUTO_SCREEN_MAIN, FALSE);
		delay_ms(100);

		SetControlVisiable(SCREEN_MAIN_1, ICON_MANUAL_MASK, VISIBLE);
		delay_ms(100);
		SetControlEnable(SCREEN_MAIN_1, BTN_MANUAL_SETTING, FALSE);
		delay_ms(100);

		SetControlVisiable(SCREEN_MAIN_1, ICON_PARA_SETTING_MASK, VISIBLE);
		delay_ms(100);
		SetControlEnable(SCREEN_MAIN_1, BTN_PARAMETER_SETTING, FALSE);
		delay_ms(100);

		SetControlEnable(SCREEN_MAIN_1, BTN_ERROR_CHECKING, FALSE);
		delay_ms(100);	

		SetControlVisiable(SCREEN_MAIN_1, ICON_SYS_SETTING_MASK, VISIBLE);
		delay_ms(100);
		SetControlEnable(SCREEN_MAIN_1, BTN_SYSTEM_SETTING, FALSE);
		delay_ms(100);	

		SetControlEnable(SCREEN_MAIN_1, 31, FALSE);
		delay_ms(100);	

	}
}

void ButtonUsabilityManager(uint8 cmd, uint8 status)
{
	static uint8 status_in_stack = OFF;
	uint8 new_status;

	new_status = (cmd == CMD_USABILTY_RECOVER)? status_in_stack : status;
	printf("new_status = %d\n",new_status);

	SetButtonUsability(new_status);

	if((status != BUTTON_STATUS_ERROR) && (status != BUTTON_STATUS_NUL)) status_in_stack = status;
	printf("status_in_stack = %d\n",status_in_stack);

}

void RefreshDateControls(void)
{
	char buff[5];

	sprintf(buff, "%d", StopDate.month);	
	SetTextValue(SCREEN_TRAIL_DATE_SETTING, TXT_MONTH, (uchar*)buff);	   

	delay_ms(100);
	sprintf(buff, "%d", StopDate.day);	
	SetTextValue(SCREEN_TRAIL_DATE_SETTING, TXT_DAY, (uchar*)buff);	  

	delay_ms(100);
}

void UiButtonManager(uint8 ind)
{
	RefreshButtons();

}

/**************************************************************************************************************************
  Pump tempature screen: Set the controls according to pump_enable_tbl[] 

	Control id:	 
		14~17                             --- Mask of pump1~pump4         Use mask if the relevant pump is unselected
	    (2, 3),(5, 6),(8, 9),(11, 12)     --- Temp limit, Temp bias       Not editable if the relevant pump is unselected  
***************************************************************************************************************************/

void SetVisibiltyBumpTemp(void)
{
	uint8 i;	
	SetControlVisiable(SCREEN_PUMP_TEM_CONTROL, 19, 1 - TempMonitorEnable);	  //cover
	SetControlEnable(SCREEN_PUMP_TEM_CONTROL, 2, TempMonitorEnable);
	SetControlEnable(SCREEN_PUMP_TEM_CONTROL, 3, TempMonitorEnable);
	SetControlEnable(SCREEN_PUMP_TEM_CONTROL, 4, TempMonitorEnable);
					
/*	if(TempMonitorEnable)
	{

			SetControlVisiable(SCREEN_PUMP_TEM_CONTROL, 14 + i, 1 - pump_enable_tbl[i]);
			SetControlEnable(SCREEN_PUMP_TEM_CONTROL, i * 3 + 2, pump_enable_tbl[i]);
			SetControlEnable(SCREEN_PUMP_TEM_CONTROL, i * 3 + 3, pump_enable_tbl[i]);

	}
	else   //unselected
	{
		for(i = 0; i < 4; i++)
		{
			SetControlVisiable(SCREEN_PUMP_TEM_CONTROL, 14 + i, 1);	  //use cover
			SetControlEnable(SCREEN_PUMP_TEM_CONTROL, i * 3 + 2, 0);
			SetControlEnable(SCREEN_PUMP_TEM_CONTROL, i * 3 + 3, 0);
		}
	}  */
}
