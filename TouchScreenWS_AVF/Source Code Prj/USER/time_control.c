/************************************版权申明********************************************
**                            深圳英捷思科技有限公司
**                             http://www.
**-----------------------------------文件信息--------------------------------------------
**
**
** 文件名称:   time_control.c
**
** 修改时间:   2020-02-25  
** 文件说明:   TC functions are included. 
**             User can specify working days and up to 6 time segments within each working day.
**             at the power-up, this config data is loaded from flash and will be saved into the
**         	   same address once it is edited by user.
**               
**
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
#include "stm32_flash.h"
#include "error_handle.h"
#include "macros.h"
#include "time_control.h"
#include "measurement.h"
#include "pump_running.h"

#define  IN_TC_WINDOW	now_hour * 60 + now_min	 >= start_hour * 60 + start_min


void NotifyScreen(uint16 screen_id);

#ifdef DBG_FUNC_INFO
extern uint8 func_dbg_info;
#endif

extern uint8 SysStandby;

extern uint8 SysRunning;
extern uint8 pump_mode;

extern uint8 Low_Pressure_Protection_Timing;
extern uint16 low_pressure_check_counter;

extern uint8 low_pressure_check_ready;

extern uint16 low_entrance_pressure_check_counter;

extern uint8 low_entrance_pressure_check_ready;


extern uint8 CMD_Disable_Heaters;

extern uint8 Sys_Switches[];

extern uint8 Valve_Control1[];

extern uint8 WaterSourceMode;

extern uint16 Entrance_LP_Protection_Delay;

extern uint8 RemoteTargetEnable;

bool AllPumpsOn;

uint8 Entrance_Pressure_Checking = FALSE;

uint8 schedule_loaded = 0; 
uint8 task_phase;
uint8 task_index;
uint8 new_day_starts;
uint8 now_hour;
uint8 now_min;
uint8 now_sec;
uint8 start_hour;
uint8 start_min;
uint8 stop_hour;
uint8 stop_min;
uint8 schedule_loaded;
uint8 scheduler_today_is_working = 0;
uint8 now_weekday;
uint8 time_control_enabled = TRUE;
uint8 working_weekdays[8] = {0, 0, 0, 0, 0, 0, 0, 0};
float tc_target_pressure[6];

/*

*/
float DefaultTargetPressure = 1.0;


float RemoteTargetPressure;
bool RemoteMode = false;

uint8 previous_sec;
uint8 TaskIsRunning = FALSE;


typedef struct
{
	uint8 time_cell[5];		  //{enabled, start hour, start minute, stop hour, stop minute}
}WORK_SEGMENT;

WORK_SEGMENT schedules[6];



/**********************************************************************************************
 * Name：pump_pressure_check_startup_monitor()
 * Brief：Once all pumps are running check if Low_Pressure_Protection_Timing has elapsed,
 *        if yes give green light signal- low_pressure_check_ready= TRUE for outlet low 
 *         pressure checking.
 *  
 * Para：    	
 * Return ： 
 * Timing :    every 1s
 * Caller(s):  main.c/main() 
 **********************************************************************************************/
void pump_pressure_check_startup_monitor()
{
   if(AllPumpsOn && (low_pressure_check_ready == FALSE))
   {
#ifdef  DBG_PUMP_PRESSURE_CHECK
      	printf("low_pressure_check_counter = %d\n",low_pressure_check_counter);
#endif

   		low_pressure_check_counter++;
		if(low_pressure_check_counter >= Low_Pressure_Protection_Timing)
		{
			 low_pressure_check_ready = TRUE;
#ifdef  DBG_PUMP_PRESSURE_CHECK
      		printf("low_pressure_check_ready\n");
#endif

		}
   }	
}


/**********************************************************************************************
 * Name：reset_pump_pressure_check_startup_monitor()
 * Brief：Initialize the data for outlet low-pressure checking
 *  
 * Para：    	
 * Return ： 
 * 
 * Caller(s): pump_running.c/use_new_bucket()  
 **********************************************************************************************/
void reset_pump_pressure_check_startup_monitor()
{
	low_pressure_check_counter = 0;
	low_pressure_check_ready = FALSE;
}


uint16 get_entrance_LP_protection_delay()
{
	if(WaterSourceMode == TANK_NONNEGTIVE_MODE)
	{
	   return  Valve_Control1[1];
	}
	else
	{
	   return  Entrance_LP_Protection_Delay;
	}

}


void pump_entrance_pressure_check_startup_monitor()
{
   if(Entrance_Pressure_Checking == TRUE)
   {
#ifdef  DBG_PUMP_ENTRANCE_PRESSURE_CHECK
		printf("low_entrance_pressure_check_counter = %d\n",low_entrance_pressure_check_counter);
#endif

   		low_entrance_pressure_check_counter++;
		if(low_entrance_pressure_check_counter >= get_entrance_LP_protection_delay())
		{
			 low_entrance_pressure_check_ready = TRUE;
			 Entrance_Pressure_Checking = FALSE;		//stop buffering
		}
   }	
}

void reset_pump_entrance_pressure_check_startup_monitor()
{
	low_entrance_pressure_check_counter = 0;
	low_entrance_pressure_check_ready = FALSE;
}


#ifdef DBG_MASTER_SWITCH
/**********************************************************************************************
 * Name：  load_schedule()
 * Brief： print the schedule data,  For debugging only
 * Para    ：  
 * Return  : null
 * Example :  
 * Caller(s):   main.c/NotifyText()/SCREEN_TIME_CTR_3
 **********************************************************************************************/
void load_schedule()
{
	uint8 i, j;
    
	for(i = 0; i < 7; i++)
		printf("%d  ",working_weekdays[i]);
	printf("\nTo PC-API:.............................\n");
	for(i = 0; i < 6; i++)
	{
		for(j = 0; j < 5; j++)
		{
			printf(" %d  ",schedules[i].time_cell[j]);
		}
		printf("  %0.2f",tc_target_pressure[i]);
		printf("\n");
	}
}
#endif

uint8 ReadScheduleCell(uint8 row, uint8 column)
{
	return schedules[row].time_cell[column];
}
/**********************************************************************************************
 * Name：  UpdatScheduleCell(uint8 data_byte, uint8 row, uint8 column)
 * Brief： Update the byte at schedules[row][column] with data_byte
 * Para    ： data_byte, row,  column 
 * Return  : null
 * Example :  
 * Caller(s):   main.c/NotifyText()/SCREEN_TIME_CTR_3
 **********************************************************************************************/
void UpdatScheduleCell(uint8 data_byte, uint8 row, uint8 column)
{
	schedules[row].time_cell[column] = data_byte;
}

/**********************************************************************************************
 * Name：  CompressData(uint8* data, uint8 len)
 * Brief： Compress an array into 1 byte
 * Para    ：data,  len --- the array and its length 
 * Return  :
 * Example : {1, 0, 1, 0, 0, 1, 1, 1} -> 10100111(0xa7)	
 * Caller(s):  SaveScheduleToFlash()
 **********************************************************************************************/
uint8 CompressData(uint8* data, uint8 len)
{
	uint8 x, i;
	for(i = 0; i < len; i++)
    {
    	x = x << 1;
    	if(data[i] == 1) x += 1;
	}
	return x;	
}


/**********************************************************************************************
 * Name： SaveScheduleToFlash()
 * Brief：Save schedule	data(32 bytes) into Flash memory.
 * Para    ： 
 * Return  : 	
 * Caller(s):  main.c/NotifyButton()/SCREEN_TIME_CTR_3
 **********************************************************************************************/
//#define DBG_SAVESCHEDULETOFLASH
extern uint8 UseIoTargetPressure;
extern float DefaultTargetPressureMenu;

void SaveScheduleToFlash()
{

	uint8 i, j;
	uint8 data[46];
	uint8 remote_target[2];


//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_SAVESCHEDULETOFLASH
    printf("\n\n\n Entrance: void SaveScheduleToFlash()...............\n\n");
#endif  
//--------------------------------------------------------------------------------------------------
#ifdef DBG_SAVESCHEDULETOFLASH
    printf("SaveScheduleToFlash();\n");
	printf("\ntime_control_enabled:%d\n", time_control_enabled);
	for(i = 0; i < 7; i++)
		printf("%d  ",working_weekdays[i]);
	printf("\n.............................\n");
#endif

	working_weekdays[7] = time_control_enabled;

	data[0] =  CompressData(working_weekdays, 8);
	//Schedule data
	for(i = 0; i < 6; i++)
	{
		for(j = 0; j < 5; j++)
		{
#ifdef DBG_SAVESCHEDULETOFLASH		
			printf("%d  ",schedules[i].time_cell[j]);
#endif

			data[i * 5 + j + 1] = schedules[i].time_cell[j];
		}
#ifdef DBG_SAVESCHEDULETOFLASH
		printf("\n");
#endif
	}

	//DefaultTargetPressureMenu 			  20210703
	data[31] = (int)(DefaultTargetPressureMenu * 100)/256;
	data[32] = (int)(DefaultTargetPressureMenu * 100) % 256;

	for(i = 0; i < 6; i++)				 //target pressure
	{
		data[33 + i * 2] = (int)(tc_target_pressure[i] * 100)/256;
		data[34 + i * 2] = (int)(tc_target_pressure[i] * 100)%256;
	}
 	
	STM32_FLASH_Write(STM32_FLASH_SAVE_ADDR_USER_SETTINGS + STM32_FLASH_SAVE_ADDR_USER_TC_INTERNAL,(u16*)data, 23);

	remote_target[0] = (int)(RemoteTargetPressure * 100) / 256;
	remote_target[1] = (int)(RemoteTargetPressure * 100) % 256;
	STM32_FLASH_Write(STM32_FLASH_SAVE_REMOTE_TARGET,(u16*)remote_target, 1);

	SaveRemoteTargetEnable(RemoteTargetEnable);	//20210618
	SaveUseIoTargetPressure(UseIoTargetPressure);	//20210618

} 
		

uint8 weekday_selected()
{
	uint8 i;
    for(i = 0; i < 7; i++)
	{
		if(working_weekdays[i] == 1) return TRUE;
	}
	return FALSE;
}
/**********************************************************************************************
 * Name： check_schedule_ok()
 * Brief：check the validity of the schedule
 * Para    ：null
 * Return  : FALSE --- invalid. There are 3 cases regarded as invalid setting.
 *                 i>    the sequence is not correct, which means the ending time is no later than 
 			      	     starting time within the same segment
				   ii>   overlapping of different segments
				   iii>  time_control_enabled is TRUE but no time segment or weekday is selected at all.
 *           TRUE --- valid 		
 * Caller(s):  main.c/NotifyButton()/SCREEN_TIME_CTR_3
 **********************************************************************************************/
int check_schedule_ok(void)
{
	uint8 i, j, nbr_of_sel_segment = 0;
	int previous = -1, current;

	for(i = 0; i < 6; i++)			 //scan each row
	{
		if(schedules[i].time_cell[0]) //only check enabled row
		{
			nbr_of_sel_segment++;
			for(j = 0; j < 2; j++)	 //start and stop
			{
				current = schedules[i].time_cell[j * 2 + 1] * 60 + schedules[i].time_cell[j * 2 + 2];
				if(current <= previous)		//invalid data found
				{
					
					return FALSE;
				}
				else
				{
					previous = current;
				}	
			}	
		}
	}

	return TRUE;
}

/**********************************************************************************************
 * Name： time_window_expired(uint8 start_hour, uint8 start_min)
 * Brief：check rtc time against task time window to see if it has expired.
 * Para    ：stop_hour, stop_min 	-- range of the time segment to be checked
 * Return  : 0 --- valid
 *           1 --- expired 		
 * Caller(s):   start_task_scheduler()|task_scheduler()  
 **********************************************************************************************/
int time_window_expired(uint8 stop_hour, uint8 stop_min)
{
	uint16 time_in_int;

	time_in_int = now_hour * 60 + now_min;

    if(time_in_int >= stop_hour * 60 + stop_min)
	{
		printf("\nwindow [*-* ---> %d-%d  has expired\n",stop_hour, stop_min);
		return 1;
	}
	return 0;
}

/**********************************************************************************************
 * Name： load_next_time_window()
 * Brief：load the next valid and enabled time segment from  schedules[] ， update task_index
 * Para    ：null  
 * Return  : null 		
 * Caller(s):   start_task_scheduler()|task_scheduler()  
 **********************************************************************************************/
int load_next_time_window()
{

	while(schedules[task_index].time_cell[0] == 0 || 
		time_window_expired(schedules[task_index].time_cell[3], schedules[task_index].time_cell[4]))//skip disabled or expired window 
	{ 
		task_index++;

		if(task_index >= 6)	//overflow no window is available
		{
			return 0;
		}
	}

    start_hour = schedules[task_index].time_cell[1];
	start_min = schedules[task_index].time_cell[2];
	stop_hour = schedules[task_index].time_cell[3];
	stop_min = schedules[task_index].time_cell[4];
    
	return 1;	
}


/**********************************************************************************************
 * Name： time_window_check(uint8 hour, uint8 min)
 * Brief：check rtc time against task time window, return 0/1/2 as an event
 * Para：    hour,  min	--RTC time
 * Return  : 0 -- no event
 *		  	 1 -- enter the window
 *		  	 2 -- leave the window		
 * Caller(s):   task_scheduler()  
 **********************************************************************************************/
int time_window_check(uint8 hour, uint8 min)
{
	uint16 time_in_int;
	time_in_int = hour * 60 + min;
	if(task_phase == 0)//on the left of the window previously
	{
		if(time_in_int >= start_hour * 60 + start_min)//enter the window
		{
			return 1;
		} 
	}
	else if(task_phase == 1)//In the window previously 
	{
		if(time_in_int >= stop_hour * 60 + stop_min)//leave the window
		{
			return 2;
		} 
	}
	return 0;
}

void ShowTargetPressure(float target)
{
	char buff[5];

	sprintf(buff, "%.2f", target);
	SetTextValue(SCREEN_MAIN_1, TXT_TARGET_PRESSURE, (uchar*)buff);
}


/*********************************************************************
 *-- Today is not selected: Return 0.0  
 *-- Today is selected:	    
 *   --The segment which now belongs to is selected, return target[i], 
 *      i denotes to segment index(0~5)
 *   --Now is out of all segments, return default pressure
**********************************************************************/
float LoadTargetPressure(void)
{  
    uint8 i;
	uint16 time_in_int;
//#define DBG_LOADTARGETPRESSURE
//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_LOADTARGETPRESSURE

    printf("\n\n\n Entrance: float LoadTargetPressure(void)...............\n\n");
#endif  
//--------------------------------------------------------------------------------------------------
	time_in_int = now_hour * 60 + now_min;

#ifdef DBG_LOADTARGETPRESSURE
	printf("time_in_int = %d\n",time_in_int);
	printf("DefaultTargetPressure = %.1f\n",DefaultTargetPressure);
#endif

	if(working_weekdays[now_weekday] == 1)	   //Today is enable
	{
		for(i = 0; i < 6; i++)
		{
			if(schedules[i].time_cell[0] == 1)	  //task i is selected
			{
#ifdef DBG_LOADTARGETPRESSURE
				printf("tc_target_pressure[i] = %.1f\n",tc_target_pressure[i]);
#endif	
				//in idle gap, use default
				if(time_in_int < schedules[i].time_cell[1] * 60 + schedules[i].time_cell[2]) return DefaultTargetPressure;

				//in a task, use segment target pressure
			    if(time_in_int < schedules[i].time_cell[3] * 60 + schedules[i].time_cell[4]) return tc_target_pressure[i];
			}
		}
		return DefaultTargetPressure;	  //now is later than all tasks today, use default target
	}
	return 0.0;	   //today is not working day at all, use 0.0 MPa
	
}

void StopTaskScheduler(void)
{
   scheduler_today_is_working = FALSE;
}


void ScheduleUpdate(void)
{
	float target_pressure;


//#define DBG_SCHEDULEUPDATE
//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_SCHEDULEUPDATE

    printf("\n\n\n Entrance: void ScheduleUpdate(void)...............\n\n");
#endif  
//--------------------------------------------------------------------------------------------------



#ifdef DBG_SCHEDULEUPDATE
    load_schedule();
#endif  
 
	target_pressure = DefaultTargetPressure;
	scheduler_today_is_working = 0;
	//--------------------------fileter:weekday---------------------------
    if(working_weekdays[now_weekday] == 1)	 //today is selected
	{
#ifdef DBG_SCHEDULEUPDATE
    printf(" today is selected\n");
#endif
		task_phase = 0;
		task_index = 0;
	    if(load_next_time_window())	 //next window of today exists
		{
			scheduler_today_is_working = 1;

		    /*--------------load_tc_target_pressure----------------------*/
			if(IN_TC_WINDOW)		  //in tc window already, load the relevant target pressure
			{
				target_pressure = tc_target_pressure[task_index];
			}
			
#ifdef DBG_SCHEDULEUPDATE
			{
				printf("To PC-API:  in start_task_scheduler();	TaskIsRunning = TRUE;\n");
				printf("\nTo PC-API:Prepare for next task\n");
				printf("To PC-API:%d:%02d ---------> %d:%02d, %0.2fMpa\n ",schedules[task_index].time_cell[1], schedules[task_index].time_cell[2],
													 schedules[task_index].time_cell[3],schedules[task_index].time_cell[4], tc_target_pressure[task_index]);
		
			}						
#endif	
		
		}
		else	   //end of task today
		{
			task_phase = PHASE_END_OF_TASK;
#ifdef DBG_SCHEDULEUPDATE
            printf("\n.........\nTo PC-API:End of the task today\n");
#endif
		}
	}

	else		  //today is not selected
	{
		target_pressure	= 0.0;
#ifdef DBG_SCHEDULEUPDATE
		printf("To PC-API:today is unselected\n");
#endif
	}
	//pid setting/TargetPressure/main screen display
	if ((!RemoteMode) || (RemoteTargetEnable == FALSE)) PidSetTargetPressure(target_pressure);		 //2020-7-16
	else  PidSetTargetPressure(RemoteTargetPressure);
		
#ifdef DBG_SCHEDULEUPDATE
	{		
		printf("To PC-API:PID target_pressure = %f\n",target_pressure);
	}
#endif

}
/**********************************************************************************************
 * Name： start_task_scheduler()
 * Brief：Kick off the scheduled tasks:	load the next enabled task
 * Context: Daily schedule is already loaded in schedules[]
 * Para：    null
 * Return  : null
 * Caller(s):  main.c/main()   
 **********************************************************************************************/
void start_task_scheduler(void)
{
//#define DBG_START_TASK_SCHEDULER          //Enabled for debugging only
//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_START_TASK_SCHEDULER
    printf("\n\n\n Entrance: void start_task_scheduler(void)...............\n\n");
#endif  
//--------------------------------------------------------------------------------------------------

	ScheduleUpdate();	   //20200313
	TaskIsRunning = TRUE;
	SysRunning = TRUE;//start pumps  
	SysStandby = FALSE;

	Sys_Switches[IND_SYSTEM] = ON;
	
#ifdef DBG_START_TASK_SCHEDULER
    printf("\n\n\n Exit: void start_task_scheduler(void)...............\n\n");
#endif  
	 
}
/********************************************************************************************************************
 * Name： task_scheduler()
 * Brief：Control the running phase and update target pressure according to current time and scheduled time windows.
 *        say, it is the time to enter task windows or leave it or stay where it was.  
 * Timing: Executed every second
 * Para：    null
 * Return  : null
 * Caller(s):  main.c/main()   
 *******************************************************************************************************************/
void task_scheduler(void)
{
//#define DBG_TASK_SCHEDULER          //Enabled for debugging only
//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_TASK_SCHEDULER

    printf("\n\n\n Entrance: void task_scheduler(void)...............\n\n");
#endif  
//--------------------------------------------------------------------------------------------------


	if(scheduler_today_is_working)
	{
		if(task_phase != PHASE_END_OF_TASK)
		{
#ifdef DBG_TASK_SCHEDULER
	    if(func_dbg_info) printf("\nnow_hour = %d, now_min = %d, now_sec = %d\n",now_hour, now_min, now_sec);
#endif

			if(time_window_check(now_hour,now_min) == 1)	 //enter window
			{
				
				/*--------------load_tc_target_pressure----------------------*/	   if (UseIoTargetPressure == FALSE)
				if ((!RemoteMode) && (UseIoTargetPressure == FALSE)) 
					PidSetTargetPressure(tc_target_pressure[task_index]);		

				task_phase = PHASE_RUNNING_IN_WINDOW_STARTED;

#ifdef DBG_TASK_SCHEDULER
			    if(func_dbg_info) printf("To PC-API:%d:%02d:%02d\nTo PC-API:Start to do a task\n\n", now_hour, now_min, now_sec);
#endif
				
			}
			else if(time_window_check(now_hour,now_min) == 2)	 //leave window
			{
				/*--------------restore target_pressure to default----------------------*/
				if(!RemoteMode) PidSetTargetPressure(DefaultTargetPressure);		
	
				task_phase = PHASE_RUNNING_IN_WINDOW_STOPPED;

#ifdef DBG_TASK_SCHEDULER
			    printf("To PC-API:%d:%02d:%02d\nEnd of a task\n\n", now_hour, now_min, now_sec);
#endif		
				
				task_index++;
#ifdef DBG_TASK_SCHEDULER
			    printf("To PC-API:task_index = %d\n\n",task_index);
#endif	
				if(task_index <= 5)
				{ 
					if(load_next_time_window() == 1)	 //next window exists
					{
#ifdef DBG_TASK_SCHEDULER

						printf("\nTo PC-API:Prepare for next task\n");
						printf("To PC-API:%d:%2d ---------> %d:%2d  %0.2fMpa\n ",schedules[task_index].time_cell[1], schedules[task_index].time_cell[2],
														 schedules[task_index].time_cell[3],schedules[task_index].time_cell[4], tc_target_pressure[task_index]);				
#endif				
					}
					else	   //end of task
					{
						task_phase = PHASE_END_OF_TASK;
#ifdef DBG_TASK_SCHEDULER
			    		printf("\nTo PC-API:...Empty tasks are left..\nEnd of all tasks\n");
#endif
					}
				}
				else	   //end of tasks of the day
				{
					task_phase = PHASE_END_OF_TASK;
#ifdef DBG_TASK_SCHEDULER
			    	printf("\nTo PC-API:..the 6th task.\nEnd of all tasks\n");
#endif
				}
			}	
		}
	}
#ifdef DBG_TASK_SCHEDULER
    printf("\n\n\n Exit: void task_scheduler(void)...............\n\n");
#endif  

}
