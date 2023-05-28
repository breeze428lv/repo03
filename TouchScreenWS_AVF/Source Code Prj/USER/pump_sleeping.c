/*****************************************************************************************
**                            深圳英捷思科技有限公司
**                             
**-----------------------------------文件信息--------------------------------------------
**	This file has a class of functions dealing with issures related to sleeping/wake-up
**  If the work loading is low enough sleeping mode is taking control to protect pumps from
**  working overtime. On the other side, if consuming of water climbs over a certain threshold
**  (outlet pressure drops lower under target value), the system is waken up.
**  There are 2 diffrent ways of sleeping: full sleeping and half sleeping.
**  In full sleeping mode no pump is wroking at all, while half sleeping mode means only small pump 
**  keeps running.
**  Both of them can jump to Main-Pump-Running mode through wake-up
--------------------------------------------------------------------------------------*/

#include "stm32f10x.h"
#include "delay.h"
#include "hmi_driver.h"

#include "cmd_queue.h"
#include "cmd_process.h"
#include "stdio.h"
#include "stdbool.h"
#include "hw_config.h"
#include "ulitity.h"
#include "string.h"
#include "usart.h"
#include "relays.h"
#include "macros.h"
#include "measurement.h"
#include "pump_sleeping.h"

#include "relays.h"
#include "time_control.h"
#include "pump_running.h"
#include "security.h"
#include "error_handle.h"
#define STOP_ALL i > 4

#define ERROR_STOPPER    	  0
#define ERROR_NON_STOPPER     1

extern uint8 Sys_Switches[];

#define ERROR_FREE 	 (Sys_Switches[0] == ON) && (Sys_Switches[1] == ON) && (Sys_Switches[2] == ON) && (Sys_Switches[3] == ON)

#define SLEEP_MANAGER_GREENLIGHT (pump_half_waking_up == FALSE) && (pump_waking_up == FALSE) && (switch_value() == ON) && ERROR_FREE

extern uint8 AvfMode;

extern uint8 AllPumpsDead;

extern uint8 Pump_Switch_Condtion[];

extern uint8 SettingsLoaded;

extern uint8 PumpGroupMode;

extern uint8 WarmUpDone;

extern volatile  uint32 timer_tick_count; 

extern uint32  timer_tick_count_rec; 

uint8 pump_waking_up = FALSE;

uint8 pump_wake_up_confirmed = FALSE;

uint8 pump_waking_up_from_half_phase1 = FALSE;

uint8 pump_waking_up_from_half_phase2 = FALSE;

uint8 pump_half_waking_up = FALSE;

uint8 pump_falling_asleep = FALSE;

uint8 focused_pump_index_backup;

uint16 sleep_chk_timer;

uint16 wakeup_chk_timer;


extern uint8 pump_error_tbl[];

extern uint8 Pump_Status;

extern uint16 RunningCounter;

extern uint8 ErrorCounter;

extern uint8 SleepingEnableMode;

extern uint8 amount_of_running_pumps;

extern uint8 focused_pump_index;

extern uint8 pump_usablilty_tbl[];

extern uint8 pump_on_off_tbl[];

extern uint8 pump_running_mode_tbl[]; 

extern float Tartget_Pressure;

extern float OutletRealTimePressure;

extern uint8 Entrance_sensor0[];

extern uint8 Valve_Control1[];

extern uint8 Valve_Control[];

extern uint8 Power_Up_Setting[];

extern uint8 Sleep_Setting[];

extern uint8 Outlet_LP_Protection_Selected;
extern float Outlet_LP_Protection_Value;

extern uint8 Outlet_HP_Protection_Selected;
extern float Outlet_HP_Protection_Value;
extern bool AllPumpsOn;


extern uint8 WaterSourceMode;

extern float WaterLevel;

extern uint8 WorkingMode;

extern uint8 Error_Channel_Disconnected;

extern uint8 ManualIsRunning;

extern uint8 Low_Pressure_Protection_Selected;
extern uint8 Low_Pressure_Protection_Timing;
extern uint16 low_pressure_check_counter;

extern uint16 low_entrance_pressure_check_counter;
extern uint8 low_pressure_check_ready;

extern uint8 low_entrance_pressure_check_ready;


extern uint8 time_control_enabled;

extern uint8 SysRunning;
extern uint8 TaskIsRunning;

extern TYPE_PUMP_ICONS PumpIcons[];


/********************************************************************************************
 * Name： SleepFrequency
 * Brief：Get configured SleepFrequency
 *           
 * Para：   null	
 * Return ：SleepFrequency
 * Caller(s):  SleepManager()
 **********************************************************************************************/
uint8 SleepFrequency()
{
	if(pump_usablilty_tbl[5] == ON) //Small Pump is workable
		return Sleep_Setting[6];
	return Sleep_Setting[2];
}

/********************************************************************************************
 * Name： WakeUpPressureBias/WakeUpDelay
 * Brief：Get configured WakeUpPressureBias/WakeUpDelay: if current outlet pressure keeps lower than 
 *        TargetPressure - WakeUpPressureBias for WakeUpDelay(), an wake-up occures
 *           
 * Para：   null	
 * Return ：WakeUpPressureBias/WakeUpDelay
 * Caller(s):  SleepManager()
 **********************************************************************************************/

uint8 WakeUpPressureBias()
{
	if(IsSmallPump()) return Sleep_Setting[7];
	return Sleep_Setting[3];
}
uint16 WakeUpDelay()
{
	if(IsSmallPump()) return Sleep_Setting[8];
	return Sleep_Setting[4] * 256 + Sleep_Setting[5];
}

void SwitchPump(int src, int dest)
{
	if(src >= 0)
	{
		control_pump(src, MOD_PF, OFF);
		pump_on_off_tbl[src] = OFF;
		pump_running_mode_tbl[src] = MOD_PF;
		update_pump_icon(src, MOD_NUL);
	}
	if(dest >= 0)
	{
		// fc->foucused pump
		control_pump(dest, MOD_FC, ON);
		pump_on_off_tbl[dest] = ON;
		pump_running_mode_tbl[dest] = MOD_FC;
		//icon
		update_pump_icon(dest, MOD_FC);
		focused_pump_index = dest;
	}

}



/*****************************************************
 * Manager of wake_up
 *
 * scan the timer, after a specified time gap, turn fc on
 *
 ******************************************************/
 
void wake_up_server(void)
{
	 if(pump_waking_up == TRUE)
	 {														 // time gap between fc-pump connection and FC on
		if((timer_tick_count - timer_tick_count_rec) * 10 >= (Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14]))
		{	//Do wake up

			Pump_Status = MAIN_PUMP_ON;
			RefreshErrorInfo(0, ADD_ERROR_NO);
					
			PIDParament_Init();

			if (AvfMode == FALSE) FC_SWITCH = ON;		//Swicth FC on

		  	pump_waking_up = FALSE;

		}
	 }
}

/*****************************************************
 * get the index ready for a wake up
 * normally it is the one recorded before falling asleep,
 * in case it can not work, a new one will be found.
 * if fails to find one, set it to 9.

******************************************************/
void refresh_focused_ind_for_wake_up(void)
{
   uint8 ind;
   int i;
   //focused_pump_index = ((PumpGroupMode == 1) || (AvfMode == TRUE))? focused_pump_index_backup:0; //fp mode !!!???
   //20210805
   /* floating fc mode */
   if (PumpGroupMode == 1)	 
   {
   	   i = FindLeastUsedPump();	
	   if (i >= 0)
	   {
	   		focused_pump_index = i;
			printf("Wake up! Start the least used pump%d\n", i + 1);
	   }	   
	   else	    //almost  impossible            
	   {
		    focused_pump_index = focused_pump_index_backup;
	   }
   }
   /* fixed-fc mode */
   else
   {
	   focused_pump_index = 0;
   }

   if(pump_usablilty_tbl[focused_pump_index] == FALSE)	 //not usable any more, go find a new one
   {
		ind = focused_pump_index;
   		update_focused_pump_index();
		if(ind == focused_pump_index)  focused_pump_index = 9; //no free pump available
   }
}


/******************************************************************
 * State machine of wake_up_from_half_sleep
 *---|<<------------------------------->>|-------------------------
 *	 |     phase1	         phase2		 |
 *   |<------T1----->|<--------T2------->|<------T1----->|
 *   |               |		  		     |				 |
 *FC-SWITCH OFF      |				main pump on	   	 |
 *				small pump off						FC-SWITCH ON
 ******************************************************************/
void wake_up_from_half_server(void)
{
//#define DBG_WAKE_UP_FROM_HALF_SERVER          //Enabled for debugging only
//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_WAKE_UP_FROM_HALF_SERVER
    printf("\n\n\n Entrance: void wake_up_from_half_server(void)...............\n\n");
#endif  
//--------------------------------------------------------------------------------------------------


	 if(pump_waking_up_from_half_phase1 == TRUE)  //Phase1
	 {
	 	//delay time out, ready to stop small pump
		if((timer_tick_count - timer_tick_count_rec) * 10 >= (Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14]))
		{
		    //phase1 done
		    if (AvfMode == FALSE)	//not AVF
			{
				Set_FC_Zero();
#ifdef DBG_WAKE_UP_FROM_HALF_SERVER
				printf("Set FC to 0: dc-out = 0v\n");
#endif
				//disconnect small pump
				SwitchPump(IND_SMALL_PUMP, -1);	

#ifdef DBG_WAKE_UP_FROM_HALF_SERVER
				printf("disconnect small pump from FC, LED-blue OFF\n");
				printf("变频暂停延时: %d ms\n", Pump_Switch_Condtion[8] * 1000);
#endif
			}
			else	//AVF
			{
				stop_a_pump(IND_SMALL_PUMP); 
				if(amount_of_running_pumps > 0) amount_of_running_pumps--;
			}
	
			timer_tick_count_rec = timer_tick_count; 
	
		  	pump_waking_up_from_half_phase1 = FALSE;
			pump_waking_up_from_half_phase2 = TRUE;
		}		
	 }
	 else if(pump_waking_up_from_half_phase2 == TRUE)	 //Phase2
	 {
	 	//delay time out
		if((timer_tick_count - timer_tick_count_rec) * 10 >= Pump_Switch_Condtion[8] * 1000)
		{
		    //phase2 done

			//select a main pump
			refresh_focused_ind_for_wake_up();	
						
			if(focused_pump_index != 9)	 //normal pump found
			{
				if (AvfMode == FALSE)	//not AVF	 
				{
					//connect a main pump to fc
					SwitchPump(-1, focused_pump_index);
					amount_of_running_pumps = 1;

#ifdef DBG_WAKE_UP_FROM_HALF_SERVER
					printf("connect a main to fc\n");
#endif	
					//Set fc to start frequency
				 	Set_FC_StartValue();
			    	show_frequency();
				}
				else   //AVF
				{
					switch_to_fc(focused_pump_index);
					amount_of_running_pumps = 1; 

					//Set fc[focused_pump_index] to start frequency
				    UpdatePumpFreq(Pump_Switch_Condtion[4], focused_pump_index);
					show_frequency();
				}


#ifdef DBG_WAKE_UP_FROM_HALF_SERVER	
				printf("Set fc to start frequency\n");		
				printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~WAKEUP MAIN PUMP\n");
#endif	
				
				RefreshErrorInfo(0, ADD_ERROR_NO);
		
#ifdef DBG_WAKE_UP_FROM_HALF_SERVER		
				printf("变频转工频延时: %d ms\n", Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14]);
#endif	
		
				pump_waking_up = TRUE;
				timer_tick_count_rec = timer_tick_count; 
		
			  	pump_waking_up_from_half_phase1 = FALSE;
				pump_waking_up_from_half_phase2 = FALSE;

#ifdef DBG_WAKE_UP_FROM_HALF_SERVER
				debugger("Wake up a main pump", ON);
	            OSD_DbgInfo();
#endif
			}
		 	else
			{
			
				Start_Stopper(STOP_TYPE_SYSTEM);
			}
		}
	 }
}

/**************************************************************************************************
 * After a certain length of time has elapsed, turn on FC and small pump starts running in FC mode.
 * Context: wake up from full-sleeping is confirmed and small-pump is connected to fc already.
 *          a time buffer is needed before FC is actually turned on.
 *
***************************************************************************************************/
void half_wake_up_server(void)
{
//#define DBG_HALF_WAKE_UP_SERVER          //Enabled for debugging only
//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_HALF_WAKE_UP_SERVER

    printf("\n\n\n Entrance: void half_wake_up_server(void)...............\n\n");
#endif  
//--------------------------------------------------------------------------------------------------


	 if(pump_half_waking_up == TRUE)  //In half-waking Inspection window
	 {
		if((timer_tick_count - timer_tick_count_rec) * 10 >= (Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14]))
		{
#ifdef DBG_HALF_WAKE_UP_SERVER
			printf("half wkp Done\n");

#endif
			
			Pump_Status = SMALL_PUMP_ON;

			if (AvfMode == FALSE) FC_SWITCH = ON;

			PIDParament_Init(); 

			RefreshErrorInfo(0, ADD_ERROR_NO);

		  	pump_half_waking_up = FALSE;

#ifdef DBG_HALF_WAKE_UP_SERVER	
				debugger("Enetr half sleep now", ON);
	            OSD_DbgInfo();
#endif
		}
	 }
}

/*****************************************************************************************************************
 * Manager of fall-asleep
 *
 * On condition that sleep cmd is issued: after a certain length of time has elapsed, disconnect the running pump.
 * Context: falling asleep from main pump or half sleep mode is confirmed and FC output is switched off.
 *          a time buffer is needed before the running pump is actually disconnected.
 * 
******************************************************************************************************************/
void sleep_server(void)
{
	 if(pump_falling_asleep == TRUE)
	 {
		if((timer_tick_count - timer_tick_count_rec) * 10 >= (Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14]))
		{
#ifdef DBG_FUNC_INFO
			printf("Sleep Done\n");

#endif
			if (AvfMode == TRUE)
			{
				UpdatePumpFreq(0, focused_pump_index);
				SetTextValue(SCREEN_MAIN_1, PumpIcons[focused_pump_index].icon_freq, "--");
				SetControlVisiable(SCREEN_MAIN_1,PumpIcons[focused_pump_index].icon_pump_fc, INVISIBLE);
				SetControlVisiable(SCREEN_MAIN_1,PumpIcons[focused_pump_index].icon_pump_pf, INVISIBLE);
//				CLR_MODE_TXT(focused_pump_index)
//				STOP_WATERFLOW(focused_pump_index)
			}
			else
			{
				Set_FC_Zero();
			}
			CLR_MODE_TXT(focused_pump_index)
			STOP_WATERFLOW(focused_pump_index)

			//Disconnect the Pump, sleeping now
			if ((PumpGroupMode == 1) || (AvfMode == TRUE))
			{
				SwitchPump(focused_pump_index, -1);	
			}
			else 
			{
				if(focused_pump_index != 3) SwitchPump(0, -1);
				else SwitchPump(3, -1);
			}	
			if(amount_of_running_pumps > 0) amount_of_running_pumps--;	 //20200304
		  	pump_falling_asleep = FALSE;

#ifdef DBG_PUMP_MANAGER	//	
			debugger("Sleep confirmed, Done", ON);
            OSD_DbgInfo();
#endif

		}
	 }
}


/********************************************************************************************
//Sleep is confirmed and switch off FC. (a time gap later the only running will be cut off
//  in  sleep_server())
*********************************************************************************************/
void FallAsleep(void)
{
//#define DBG_FALLASLEEP          //Enabled for debugging only
//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_FALLASLEEP

       printf("\n\n\n Entrance: void FallAsleep(void)...............\n\n");
#endif  
//--------------------------------------------------------------------------------------------------
	
		
		FC_SWITCH = OFF;
		Set_FC_StartValue();

#ifdef DBG_FALLASLEEP
		printf("Switch off FC, Set FC to start value\n");	
		printf("focused_pump_index = %d\n",focused_pump_index);
#endif

		//if(focused_pump_index != 3) focused_pump_index_backup = focused_pump_index;
		if(focused_pump_index != 5) focused_pump_index_backup = focused_pump_index;	   //is main pump  20210518
	

		Pump_Status = ALL_SLEEP;
		SetTextValue(SCREEN_MAIN_1, TXT_CURRENT_FREQUENCY, "--"); 

		RefreshErrorInfo(0, ADD_ERROR_NO);

		pump_falling_asleep = TRUE;

		pump_wake_up_confirmed = FALSE;

#ifdef DBG_FALLASLEEP
		printf("变频转工频延时: %d ms\n", Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14]);
#endif
		timer_tick_count_rec = timer_tick_count; 
}

void ShortCutToSleep(void)
{
	if(focused_pump_index != 5) focused_pump_index_backup = focused_pump_index;

	Pump_Status = ALL_SLEEP;
	SetTextValue(SCREEN_MAIN_1, TXT_CURRENT_FREQUENCY, "--"); 
	RefreshErrorInfo(0, ADD_ERROR_NO);
}


/************************************************************************************
* Keep an eye on the fc requency and outlet pressure for sleeping signal
* as soon as the condition is met it issues a 1st alert, which starts a counting down
* timer, once the reading of the timer reaches 0, sleeping is confirmed and FallAsleep()
* is executed.
* Caller(s): SleepManager
*************************************************************************************/
void sleep_watchdog(uint8 status)
{

//#define DBG_SLEEP_WATCHDOG          //Enabled for debugging only
//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_SLEEP_WATCHDOG

    printf("\n\n\n Entrance: void sleep_watchdog(uint8 status)...............\n\n");
#endif  
//--------------------------------------------------------------------------------------------------


	if(sleep_chk_timer > 0)
	{
#ifdef DBG_SLEEP_WATCHDOG
 		printf("sleep_chk_timer = %d\n",sleep_chk_timer);
#endif
		sleep_chk_timer--;
														                      //ADDED ON 2020-5-27
		if((sleep_chk_timer == 0) && (SleepPressureConfirmed() == TRUE) && (amount_of_running_pumps <= 1))
		{
			FallAsleep();
		}			
	}
	else if(SleepingEnableMode == SLEEP_MODE_SLEEP_ENABLE)
	{
#ifdef DBG_SLEEP_WATCHDOG
		printf("-------------------SleepingEnableMode = %d\n", SleepingEnableMode);
		printf("-------------------GetCurrentFrequency = %d, SleepFrequency = %d, WakeUpPressureAlert(0) = %d\n",\
						GetCurrentFrequency(), SleepFrequency(), WakeUpPressureAlert(0));
#endif																						 //ADDED ON 2020-5-27
		if((GetCurrentFrequency() < SleepFrequency()) && (WakeUpPressureAlert(0) == FALSE) && (amount_of_running_pumps <= 1))	 //original sleep signal is issued 
		{
		
			//START_SLEEP_CHK_WINDOW;
			sleep_chk_timer = Sleep_Setting[0] * 256 + Sleep_Setting[1];   //issue 1st alert of sleeping
#ifdef DBG_SLEEP_WATCHDOG
			printf("CurrentFrequency  < SleepFrequency\n");
			printf("Sleeping to be confirmed: sleep_chk_timer = %d\n",sleep_chk_timer);	
			printf("amount_of_running_pumps = %d\n",amount_of_running_pumps);
#endif
		}
	}
}

void wake_up(void)
{
//#define DBG_WAKE_UP          //Enabled for debugging only
//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_WAKE_UP

    printf("\n\n\n Entrance: void wake_up(void)...............\n\n");
#endif  
//--------------------------------------------------------------------------------------------------

	refresh_focused_ind_for_wake_up();

	if(focused_pump_index != 9)
	{
		FC_SWITCH = ON;
		SwitchPump(IND_SMALL_PUMP, focused_pump_index);
	
		PIDParament_Init();
#ifdef DBG_WAKE_UP	
		printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~WAKEUP MAIN PUMP\n");
#endif
		Pump_Status = MAIN_PUMP_ON;

		RefreshErrorInfo(0, ADD_ERROR_NO);

	}
	else
	{
		Start_Stopper(STOP_TYPE_SYSTEM);
	}
}

/**************************************************
* Jump directly from deep sleep to main pump running
*****************************************************/
uint8 full_wake_up(void)
{
//#define DBG_FULL_WAKE_UP          //Enabled for debugging only
//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_FULL_WAKE_UP

    printf("\n\n\n Entrance: uint8 full_wake_up(void)...............\n\n");
#endif  
//--------------------------------------------------------------------------------------------------
	//connect to fc
	refresh_focused_ind_for_wake_up();//???focused_pump_index = (PumpGroupMode == 1)? focused_pump_index_backup:0;
	if(focused_pump_index != 9)	  //main pump available for wakeup
	{
#ifdef DBG_FULL_WAKE_UP
		printf("normal wkp\n");
#endif
		if (AvfMode == FALSE)	//not AVF
		{
			SwitchPump(IND_SMALL_PUMP, focused_pump_index);	 //switch to a main pump in fc
			amount_of_running_pumps++;
			
			Set_FC_StartValue();
			show_frequency();
		}
		else   //AVF
		{
			switch_to_fc(focused_pump_index);  //switch a main pump to fc
			amount_of_running_pumps = 1; 

			//Set fc[focused_pump_index] to start frequency
		    UpdatePumpFreq(Pump_Switch_Condtion[4], focused_pump_index);
			show_frequency();
		}

		RESET_FATIGE_SWITCHING_TIMER

#ifdef DBG_FULL_WAKE_UP	
		printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~WAKEUP MAIN PUMP\n");
#endif
		
		RefreshErrorInfo(0, ADD_ERROR_NO);
	
		return TRUE;
	}
	else
	{
#ifdef DBG_FULL_WAKE_UP
		printf("all error! escape...\n");
#endif
		Start_Stopper(STOP_TYPE_SYSTEM);
		return FALSE;
	}
}

/*
   WKP Paths:
   --case1:  xx-->full wkp(forcrd full wkp)
                small pump not workable
   --case2:	 half slp-->full wkp(wkp from half)
                
   --case3:	 all slp-->full wkp(non-stop wkp)

   all-sleep --【low】-->half-sleep -->【very low】---->wkp
       ||                                  /\
	   ||__________________________________/\
	   |_________non-stop__________________||
*/
void wake_up_watchdog(uint8 status)
{
#define DBG_WAKE_UP_WATCHDOG          //Enabled for debugging only
//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_WAKE_UP_WATCHDOG

    printf("\n\n\n Entrance: void wake_up_watchdog(uint8 status)...............\n\n");
#endif  
//--------------------------------------------------------------------------------------------------


	if(wakeup_chk_timer > 0)   //in wake-up alert phase
	{
		wakeup_chk_timer--;

#ifdef DBG_WAKE_UP_WATCHDOG
		printf("wakeup_chk_timer = %d\n", wakeup_chk_timer);
#endif

		if(pump_usablilty_tbl[5] == FALSE)		//small pump is not workable			 20210517
		{
			if(PATH_TO_MAIN_PUMP_ON_IS_BLOCKED)
			{
#ifdef DBG_WAKE_UP_WATCHDOG
				printf("PATH_TO_FULL_WAKE_UP_IS_BLOCKED\n");
#endif
			}
			else //able to go main pump state directly(full wake up)
			{

				if((wakeup_chk_timer == 0) && (WakeUpPressureAlert(Sleep_Setting[3]) == TRUE))
				{
	
					// ---------------------Full wake up----------------------------------------
					if(full_wake_up())
					{
#ifdef DBG_WAKE_UP_WATCHDOG	
						printf("变频转工频延时: %d ms\n", Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14]);
#endif		
						pump_waking_up = TRUE;
						timer_tick_count_rec = timer_tick_count; 
					}		
				}	
			}
		}
		else if(status == SMALL_PUMP_ON)	 //in shallow sleep
		{											 //no main pump can work
			if((PATH_TO_MAIN_PUMP_ON_IS_BLOCKED) || (pump_usablilty_tbl[0] + pump_usablilty_tbl[1] + pump_usablilty_tbl[2]\
			                                         + pump_usablilty_tbl[3] + pump_usablilty_tbl[4] == 0))
			{
#ifdef DBG_WAKE_UP_WATCHDOG
				printf("PATH_TO_FULL_WAKE_UP_IS_BLOCKED\n");
#endif
			}
			else	  //path to full wake-up is open
			{					
				if((wakeup_chk_timer == 0) && (WakeUpPressureAlert(Sleep_Setting[3]) == TRUE)) //low pressure alert
				{
					//----------------wake_up_from_half_sleep confirmed-------------------------------------
#ifdef DBG_WAKE_UP_WATCHDOG
					printf("已经唤醒\n");
#endif
					pump_wake_up_confirmed = TRUE;
	
					if (AvfMode == FALSE) FC_SWITCH = OFF;	  //no general FC_SWITCH for avf 20210518

					RESET_FATIGE_SWITCHING_TIMER

#ifdef DBG_WAKE_UP_WATCHDOG	
					printf("Switch fc off\n");	
					printf("变频转工频延时: %d ms\n", Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14]);
#endif	
					pump_waking_up_from_half_phase1 = TRUE;
					timer_tick_count_rec = timer_tick_count; 
	
				}
			}			
		}		
		else if(status == ALL_SLEEP)
		{											   //to half-wake-up
			if((wakeup_chk_timer == 0) && (WakeUpPressureAlert(Sleep_Setting[7]) == TRUE))
			{
				if(WakeUpPressureAlert(Sleep_Setting[3]) == TRUE)//Even lower than full-wkp threshold, go full wkp			
				{
#ifdef DBG_WAKE_UP_WATCHDOG
					printf("Even lower than full-wkp threshold, go full wkp\n");
#endif															//at least one main pump can work
					if((PATH_TO_MAIN_PUMP_ON_IS_BLOCKED) || (pump_usablilty_tbl[0] + pump_usablilty_tbl[1] + pump_usablilty_tbl[2] +\
					                                         pump_usablilty_tbl[3] + pump_usablilty_tbl[4] == 0))  //pump3 4 are added 20210520
					{

					}
					else //workable pump available
					{
						
						if(full_wake_up())
						{
#ifdef DBG_WAKE_UP_WATCHDOG
							printf("Skip half wake up---变频转工频延时: %d ms\n", Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14]);	
#endif

							pump_waking_up = TRUE;
							timer_tick_count_rec = timer_tick_count; 
						}
					}
				}
				else	//lower than half-wkp threshold, go half-wkp
				{
				     printf("lower than half-wkp threshold, go half-wkp\n");

		    		if (AvfMode == FALSE)
					{
						SwitchPump(-1, IND_SMALL_PUMP);
						Set_FC_StartValue();
					}
					else   //AVF
					{
						

					    switch_to_fc(IND_SMALL_PUMP);
						focused_pump_index = IND_SMALL_PUMP;
						printf("switch_to_fc(%d);\n", focused_pump_index);

						//Set fc[focused_pump_index] to start frequency
					    UpdatePumpFreq(Pump_Switch_Condtion[4], focused_pump_index);
					}
					show_frequency();

#ifdef DBG_WAKE_UP_WATCHDOG
					printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~WAKEUP SMALL PUMP\n");
					printf("变频转工频延时: %d ms\n", Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14]);
#endif	
					pump_half_waking_up = TRUE;	   //Enter half-waking Inspection window
					timer_tick_count_rec = timer_tick_count; 
				}
			}	
		}		

	}
	else if(SleepingEnableMode != SLEEP_MODE_SLEEP_DISABLE)   //waiting for 1st alert of wake up
	{
		if((status == SMALL_PUMP_ON)||(pump_usablilty_tbl[5] == FALSE))//in shallow sleep or small-pump is not workable, waiting for main pump wake-up
		{
			if((WakeUpPressureAlert(Sleep_Setting[3]) == TRUE) && (pump_wake_up_confirmed == FALSE))  //当前压力低于设定压力减去唤醒压力偏差				
			{
				//	RELOAD_TIMER, WAKE UP ALERT
				wakeup_chk_timer = Sleep_Setting[4] * 256 + Sleep_Setting[5]; //issue 1st alert of wkp for main pump delay		

#ifdef DBG_WAKE_UP_WATCHDOG
				printf("RELOAD_TIMER, WAKE UP ALERT\n");
#endif
			}
		}
		else if(status == ALL_SLEEP)//in deep sleep , waiting for small-pump wake-up
		{
			if(WakeUpPressureAlert(Sleep_Setting[7]) == TRUE)  //当前压力低于设定压力减去唤醒压力偏差
			{
				wakeup_chk_timer = Sleep_Setting[8]; 

#ifdef DBG_WAKE_UP_WATCHDOG
				printf("Start wkp from all_sleep:RELOAD_TIMER,wakeup_chk_timer = %d\n",wakeup_chk_timer);
#endif

			}
#ifdef DBG_WAKE_UP_WATCHDOG
			printf("No WKP Alert!!\n");
#endif
		}
	}	
	else    
	{
		 if(OutletRealTimePressure < Tartget_Pressure)
		 {
		 	wake_up();
		 }	
	}   		
}

void EscapeSleeper(void)
{
   Pump_Status = SYS_STOP;

}

/****************Every 100ms *********************/	
void SleepManager(void)
{
	sleep_server();
	wake_up_server();
	wake_up_from_half_server();
	half_wake_up_server();
	
	if(SLEEP_MANAGER_GREENLIGHT)
	{
		switch(Pump_Status)
		{
		   case ALL_SLEEP:
		   		wake_up_watchdog(ALL_SLEEP);
		
		   		break;
	
		   case SMALL_PUMP_ON:
		   
				wake_up_watchdog(SMALL_PUMP_ON);
				sleep_watchdog(SMALL_PUMP_ON);
	
		   		break;
	
		   case MAIN_PUMP_ON:
		   	    sleep_watchdog(MAIN_PUMP_ON);
		   		break;
	
		   default:
				break;
		}
	}
}
