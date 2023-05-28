/**********************************************************
* 	File name:    error_handle.c                          *
*                                                         *
* 	Author:       Breeze Ji                               *
* 													      *
*	Version:      1.1                                     *
* 													      *
* 	Revision history:                                     *
*                                                         *
* 	--1.0 July 5, 2020,   Breeze Ji                       *
* 	      Initial Version.                                *
* 														  *
*   --1.1 Mar 18, 2021,   Breeze Ji, line 0396~0462       *
**********************************************************/

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
#include "settings.h"
#include "UserInterface.h"
#include "error_handle.h"

#include "relays.h"
#include "time_control.h"
#include "pump_running.h"
#include "security.h"
#include "ui_controls.h"
#include "error_handle.h"
#include "pump_sleeping.h"
#include "pump_running.h"


#define STOP_ALL i > 4

#define ERROR_STOPPER    	  0
#define ERROR_NON_STOPPER     1

#define NOT_OCCUPIED(x)	      (old_pump_index != x) && (focused_pump_index != x) 
uint16 SysStatusWord;

extern TYPE_PUMP_ICONS PumpIcons[];

extern uint8 Manual_Setting;

extern uint8   ApiCommSysStatus;

extern bool RemoteMode;

extern float temp_limit[]; 
extern uint8 TempMonitorEnable;

extern uint8 fast_mode;
extern uint8 pump_switch_phase_mod2;
extern uint8 adding_pump_executing;
extern uint8 adding_pump_executing;
extern uint32  timer_tick_count_rec_mod2; 
extern uint8 Entrance_Sensor_type;

extern float pump_temp[];

extern char DbgBuff[];

extern uint16 PowerUpAutoRunTimer;

extern uint8 func_dbg_info;

extern uint8 Pump_Switch_Condtion[];

extern uint8 SettingsLoaded;

extern uint8 PumpGroupMode;

extern uint8 WarmUpDone;

extern volatile  uint32 timer_tick_count; 
extern uint32  timer_tick_count_rec; 

#ifdef USE_CURVES
extern uint8 up_crossed;	
#endif

uint8 AllPumpsDead = FALSE;

uint8 FatalErrorOccured = FALSE;

uint8 Outlet_HP_Protected = FALSE;

uint8 PumpErrStatus;

extern uint8 pump_error_tbl[];

extern uint8 pump_enable_tbl[];

extern uint8 focused_pump_index_backup;

extern uint8 old_pump_index;
extern uint8 focused_pump_index;

extern uint8 pump_switch_phase;


uint8 CMD_Disable_Heaters = INACTIVE;

uint8 Sys_Switches[5] = {
							ON,		//IND_ERR_LP   				outlet low pressure
							ON,		//IND_ERR_HP   				outlet high pressure
							ON, 	//IND_ERR_WL_L 				low water level
							ON, 	//IND_ERR_ENTRANCE_LP       entrance low pressure 
							OFF		//IND_SYSTEM                system is intended to run(decided by MAIN_SCREEN/START BUTTON)
									//							will be	reset to OFF only if STOP BUTTON is pressed
						};


uint8 Error_Switch_Status;

uint8 HeatingTrafficLight = GREEN;

uint8 error_occured = 0;
uint8 error_never_occured = 1;
uint8 ErrorCounter = 0;

uint8 remote_cmd_status = 1;
uint8 remote_cmd_pre_status = 1;

uint8 now_year;
uint8 now_month;
uint8 now_day;
extern uint8 now_hour;
extern uint8 now_min;
extern uint8 now_sec;

uint8 error_info_index = 0;

uint8 Pump_Status;

extern uint8 ErrorCounter;

extern uint8 SleepingEnableMode;

extern uint8 amount_of_running_pumps;

extern uint8 focused_pump_index;

extern uint8 pump_usablilty_tbl[];
extern uint8 pump_on_off_tbl[]; 
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

uint8 TankTemperatureLimit;
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



//extern uint8 Pressure_Roof;
//extern uint8 Pressure_Floor;



//extern uint8 PT_Selected;
extern float PT_pressure;
extern float PM_pressure;
extern float phy_quantities[]; 

extern float EntranceRealTimePressure;

extern float Entrance_LP_Protection_Value;
extern float Entrance_LP_Protection_Restore_Value;

extern uint8 Entrance_Pressure_Checking;

extern uint8 pump_waking_up_from_half_phase1;

extern uint8 pump_waking_up_from_half_phase2;

extern uint8 pump_waking_up;


uint8 warning_bar_index = 0;
uint8 error_status_changed = 0;


uint8 TankTempSuperLow_selected;

#ifdef DBG_VIRTUAL_ERR
	uint8 VirtualErr[6];
#endif

#ifdef ENGLISH_VERSION
char* channel_name[] =  {
                               "Outlet low pressure",    //Error channel 1	    bit15
                               "Outlet high pressure",    //Error channel 2				    nit14
                               "Outlet high pressure canceled ",    //Error channel 3				bit13
                               "Inlet low water level ",    //Error channel 4				bit12
                               "Inlet low pressure",    //Error channel 5					bit11
							   "Water shortage",    //Error channel 6			 		bit10   100 0000 0000
                               "Safety protection",    //Error channel 7				    bit9
                               "Frequency converter error",    //Error channel 8				bit8
                               "Pump1 error",    //Error channel 9			 0x80	bit7   1000 0000
                               "Pump2 error",    //Error channel 10			 0x40   bit6    
                               "Pump3 error",    //Error channel 11		  	 0x20	bit5
                               "Auxiliary error",    //Error channel 12		 0x10	bit4
                               "Remote control",    //Error channel 13
							   "No running",    //Error channel 14		  		bit2
							                  //trial expired 15		  		bit1
											  //nul			  16		  		bit0


};
#else
char* channel_name[] =  {
                               "出口欠压(需重启恢复)",    //Error channel 1	    bit15
                               "出口超压",    //Error channel 2				    nit14
                               "出口超压消除",    //Error channel 3				bit13
                               "入口低水位",    //Error channel 4				bit12
                               "入口欠压",    //Error channel 5					bit11
							   "缺水信号",    //Error channel 6			 		bit10   100 0000 0000
                               "安全保护",    //Error channel 7				    bit9
                               "变频器故障",    //Error channel 8				bit8
                               "泵1故障",    //Error channel 9			 0x80	bit7   1000 0000
                               "泵2故障",    //Error channel 10			 0x40   bit6    
                               "泵3故障",    //Error channel 11		  	 0x20	bit5
                               "泵4故障",     //Error channel 12		 0x10	bit4
                               "远程控制",    //Error channel 13
							   "禁止运行",    //Error channel 14		  		bit2
							                  //trial expired 15		  		bit1
											  //nul			  16		  		bit0
                               "泵5故障",    // 
                               "泵6故障",    // 
							   "过热故障",    // 
};
#endif

char* status_name[] =  {
							"自动运行状态",
							"等待状态",
							"自动运行状态-浅度休眠",
							"自动运行状态-深度休眠",

};


 /*	  厂缺水信号常闭有效
厂泵l故障常闭有效
厂泵4故障常闭有效
厂安全保护常闭有效
厂泵2故障常闭有效
厂辅泵故障常阔有效
  变频器故障常闭有效
  泵3故障常闭有效

  启用进水阀门：即水箱液位低于设定的“液位下限”值，接通电磁阀，打开进水阀门进行水箱注水。
  当液位值高于设定的“液位上限”值，断开电磁阀，关闭进水阀门停止水箱注水。
3. 启用泄压阀门：即水泵出口压力高于设定的“压力上限”值，接通电磁阀，打开管网泄压阀门进行管网泄水。
当压力值低于设定的“压力下限”值时，断开电磁阀，关闭泄压阀门停止管网泄压。	  */
                             
typedef struct
{
	uint8 channel;
	uint8 error_code;		  
	char time_stamp[50];

}ERROR_INFO;

ERROR_INFO errors[WARNING_NBR];

char err_record[WARNING_NBR] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};		

char StatusBuff[WARNING_NBR] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
char Error_Type[WARNING_NBR] = {
										ERROR_NON_STOPPER, 	
										ERROR_NON_STOPPER, 	
										ERROR_NON_STOPPER, 	
										ERROR_NON_STOPPER, 	

										ERROR_NON_STOPPER, 	
										ERROR_STOPPER, 	
										ERROR_STOPPER, 	
										ERROR_STOPPER, 	

										ERROR_NON_STOPPER, 	
										ERROR_NON_STOPPER, 	
										ERROR_NON_STOPPER, 	
										ERROR_NON_STOPPER, 

										ERROR_NON_STOPPER, 	
										ERROR_STOPPER, 	
										ERROR_NON_STOPPER, 	
										ERROR_NON_STOPPER,																			
										ERROR_STOPPER
							         };

uint8 Restarter[WARNING_NBR] = {
									1, 0, 0, 0,
									0, 0, 0, 0,
									0, 0, 0, 0,
									0, 0, 0, 0, 0
							   };



									

 /*
 * 函数名：Key_GPIO_Config
 * 描述  ：配置按键用到的I/O口
 * 输入  ：无
 * 输出  ：无
 */
void Status_GPIO_Config(void)
{ 
 	GPIO_InitTypeDef GPIO_InitStructure;

    //开启DIN端口（PB）的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	 
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//上拉输入
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  //开启DIN端口（PC）的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //上拉输入
 	GPIO_Init(GPIOC, &GPIO_InitStructure);

 //开启DIN端口（PD）的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_2|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //上拉输入
 	GPIO_Init(GPIOD, &GPIO_InitStructure);
		 		 	   
}

void escape_sleep(void)
{

	Pump_Status = MAIN_PUMP_ON;
	AllPumpsDead = FALSE;
	pump_waking_up_from_half_phase1 = FALSE;
	pump_waking_up_from_half_phase2 = FALSE;
	pump_waking_up = FALSE; 

}

char ErrorCheckOK(uint8 channel)
{
	 return StatusBuff[channel];
}

extern uint8 FC_is_on;



/* 
 *
 *	 		enable			cmd_id:0							        cmd_id:1~7
 *             1       all buttons off and enabled 		     button-cmd_id off  enable button-cmd_id  
 *             0       all buttons off and disabled 		 button-cmd_id off  disable button-cmd_id  
 */
void reset_manual_controls_non_avf(uint8 cmd_id, uint8 enable)	  
{
	uint8 i;


//	printf("cmd_id = %d, enable = %d\n", cmd_id, enable);
	if ((cmd_id >= 1) && (cmd_id <= 11))	//individual
	{
		SetButtonValue(SCREEN_MANUAL_CTR_2, cmd_id, OFF);

		NotifyButton(SCREEN_MANUAL_CTR_2, cmd_id, OFF);

		if(enable == FALSE)		//Disable a pump
		{
		//	printf("SetControlEnable(SCREEN_MANUAL_CTR_2,%d, 0);\n", cmd_id);
			SetControlEnable(SCREEN_MANUAL_CTR_2, cmd_id, 0);
		
		    //show disabling cover
			SetControlVisiable(SCREEN_MANUAL_CTR_2, 20 + cmd_id, 1);

			//FC-button: Set its pf-partner
			if ((is_fc_button(cmd_id) != 0) && (cmd_id != 7))
			{
				SetControlEnable(SCREEN_MANUAL_CTR_2, cmd_id - 1, 0);	
				SetControlVisiable(SCREEN_MANUAL_CTR_2, 20 + cmd_id - 1, 1);
			}
		}
		else   //Enable a pump
		{
			 if ((is_fc_button(cmd_id) != 0) && (FC_is_on == TRUE))
			 {
			 	
			 }
			 else
			 {
				SetControlEnable(SCREEN_MANUAL_CTR_2, cmd_id, 1);
				SetControlVisiable(SCREEN_MANUAL_CTR_2, 20 + cmd_id, 0);
			 }
		}
	}
	else  //cmd_id = 0  for all pumps
	{
		for(i = 1; i <= 11; i++)		//???
		{
			SetButtonValue(SCREEN_MANUAL_CTR_2, i, 0);	 //off
			NotifyButton(SCREEN_MANUAL_CTR_2, i, 0);	

			//disable/enable a button and show the disabling cover
			SetControlEnable(SCREEN_MANUAL_CTR_2, i, enable);
			SetControlVisiable(SCREEN_MANUAL_CTR_2, 20 + i, 1 - enable);

			if(enable == 0)	   //disable
			{
				if ((is_fc_button(i) != 0) && (i != 11))
				{
					SetControlEnable(SCREEN_MANUAL_CTR_2, i - 1, 0);	
					SetControlVisiable(SCREEN_MANUAL_CTR_2, 20 + i - 1, 1);
				}
			}
		}	 
	}
}

void reset_manual_controls_avf(uint8 cmd_id, uint8 enable)	  
{
	uint8 i;
//	printf("cmd_id = %d, enable = %d\n", cmd_id, enable);
	if ((cmd_id >= 1) && (cmd_id <= 6))	//individual
	{
		//Switch off a button
		SetButtonValue(SCREEN_MANUAL_CTR_AVF, cmd_id, OFF);
		NotifyButton(SCREEN_MANUAL_CTR_AVF, cmd_id, OFF);

		SetControlEnable(SCREEN_MANUAL_CTR_AVF, i, enable);
		SetControlVisiable(SCREEN_MANUAL_CTR_AVF, 20 + i, 1 - enable);

	}
	else  //cmd_id = 0  for all pumps
	{
		for(i = 1; i <= 6; i++)		 
		{
			SetButtonValue(SCREEN_MANUAL_CTR_AVF, i, 0);
			NotifyButton(SCREEN_MANUAL_CTR_AVF, i, 0);	

			SetControlEnable(SCREEN_MANUAL_CTR_AVF, i, enable);
			SetControlVisiable(SCREEN_MANUAL_CTR_AVF, 20 + i, 1 - enable);
		}	 

	}
}
extern uint8 AvfMode;
void reset_manual_controls(uint8 cmd_id, uint8 enable)	
{
	  if (AvfMode) 
	  {
	  	  reset_manual_controls_avf(cmd_id, enable);	
	  } 
	  else
	  {
	  	  reset_manual_controls_non_avf(cmd_id, enable);
	  }		 
}


/*
 * After all pumps selected have been running for 10 minutes,
 * if the outlet pressure is still lower
 * than Outlet_LP_Protection_Value and
 * target pressure is higher than Outlet_LP_Protection_Value,
 * issue an error
 */

char outlet_pressure_low()
{
	if((Outlet_LP_Protection_Selected == TRUE) && (low_pressure_check_ready == TRUE))
	{
		if((AllPumpsOn == true) && \
		        (OutletRealTimePressure < Outlet_LP_Protection_Value) &&\
		        (Tartget_Pressure > Outlet_LP_Protection_Value)) //20220325
		{
#ifdef  DBG_PUMP_PRESSURE_CHECK
			printf("Low pressure protection occured\n");
		    printf("Stop all pumps\n");
#endif
			escape_sleep();
			Start_Stopper(STOP_TYPE_SYSTEM);
			SysRunning = FALSE;
			low_pressure_check_ready = FALSE;
			AllPumpsOn = FALSE;


			return 0;
		}
		else
		{
			if(Sys_Switches[IND_ERR_LP] == OFF)
			{
				Sys_Switches[IND_ERR_LP] = ON;
			}
			return 1;
		}
	}
	else
	{
		return 2;
	}	  
}


char outlet_pressure_high()
{
	if(Outlet_HP_Protection_Selected == TRUE)
	{
	   	if((Manual_Setting == TRUE) &&(OutletRealTimePressure > Outlet_HP_Protection_Value)) 
		{
			Sys_Switches[IND_ERR_HP] = OFF;
			reset_manual_controls(0, 0);
		}
	    
		if((SysRunning == TRUE) &&(OutletRealTimePressure > Outlet_HP_Protection_Value))  //HP_Protection occurs
		{

			Sys_Switches[IND_ERR_HP] = OFF;
			escape_sleep();
			Start_Stopper(STOP_TYPE_SYSTEM);
			SysRunning = FALSE;

			Outlet_HP_Protected = TRUE;


			printf("High pressure protection occured\n");
			return 0;
		}
		else
		{
			if(Outlet_HP_Protected == FALSE) return 1;		//HP_Protection not released yet
			return 0;			                            //not protected yet or released
		}
	}
	else
	{
		return 1;
	}	  
}

char outlet_pressure_restore_from_high()
{

	if(Sys_Switches[IND_ERR_HP] == OFF)
	{
		if(OutletRealTimePressure <= Tartget_Pressure)	
		{
			Sys_Switches[IND_ERR_HP] = ON;
			TryToRestoreSysRunning();
			Outlet_HP_Protected = FALSE;

		   	if(Manual_Setting == TRUE) 
			{
				reset_manual_controls(0, 1);
			}

			printf("Restore from High pressure protection\n");
		}
	}

	return 1;
}


char water_level_low()
{
	uint8 event;

	event = WaterLevelMonitor();
	if(event == EVENT_WL_UNDERFLOW)
	{
		if(Manual_Setting)	 //20210318
		{
			reset_manual_controls(0, 0);
		}		
		return 0;
	}
	return 1;
}

char entrance_pressure_low()
{
	if(Entrance_Sensor_type == ENTRANCE_SENSOR_TYPE_3) return 1;

	if(EntranceRealTimePressure < Entrance_LP_Protection_Value)	   //low pressure
	{
		if(low_entrance_pressure_check_ready == TRUE)	   //ready for final check
		{	
			if(SysRunning)			//20200319
			{	
				escape_sleep();					
				Start_Stopper(STOP_TYPE_SYSTEM);
				SysRunning = FALSE;
			}

			low_entrance_pressure_check_ready = FALSE;

			Sys_Switches[IND_ERR_ENTRANCE_LP] = OFF;
			PumpErrStatus = LOW_ENTRANCE_PRESSURE_PROTECTED;

			if(Manual_Setting)	 //20210318
			{
				reset_manual_controls(0, 0);
			}
			printf("Low pressure entrance protection occured\n");
			return 0;
		}
		           
			     //     error not reported  yet 					     not in the checking window
		else if((PumpErrStatus != LOW_ENTRANCE_PRESSURE_PROTECTED) && (Entrance_Pressure_Checking == FALSE))  //pumps on and not check yet
		{
			Entrance_Pressure_Checking = TRUE;			//start buffering
			return 1;
		}
	}
	if(PumpErrStatus == LOW_ENTRANCE_PRESSURE_PROTECTED) return 0;

	return 1;
}

void entrance_pressure_restore_from_low()
{
	if(Sys_Switches[IND_ERR_ENTRANCE_LP] == OFF)
	{
		if(EntranceRealTimePressure >= Entrance_LP_Protection_Restore_Value)	
		{
			Sys_Switches[IND_ERR_ENTRANCE_LP] = ON;
			TryToRestoreSysRunning();
			PumpErrStatus = PUMP_RUNNING;

			reset_pump_entrance_pressure_check_startup_monitor();//20200417

			printf("EntranceRealTimePressure = %f\n",EntranceRealTimePressure);
			printf("Restore from low entrance pressure protection\n");
		}
	}
}
/*
1. 进入参数设置后，在组泵方式参数页面设置“箱式无负压模式”后，点击进入“阀门控制”，即显示此阀门控制界面；
2. 水箱切换阀门设置： 入口欠压切换、欠压切换延时：泵入口压力（市政压力）低于入口欠压切换设定值并且延时超过设定的欠压切换延时时间后，
   将接通电磁阀，水泵切换为从水箱取水。 入口恢复压力：泵入口压力（市政压力）欠压切换水箱取水后，
   入口压力值高于入口恢复压力设定值并且延时超过设定的欠压切换延时时间后，将关闭电磁阀，水泵恢复到从无负压罐取水。
3. 切换水箱时间：此时间值是周期性的从水箱取水，以保持水箱中的水质新鲜。
4. 水箱工作时间：周期性地从水箱取水的维持时间。
*/

void entrance_pressure_low_dual()
{
	if(EntranceRealTimePressure < (float)(Valve_Control1[0]/100.0))	   //low pressure
	{
		if(low_entrance_pressure_check_ready == TRUE)	   //ready for final check
		{							
			ValveSwitchToTank = ON;
			reset_pump_entrance_pressure_check_startup_monitor();
		}
		else if((SysRunning == TRUE) && (Entrance_Pressure_Checking == FALSE)) //pumps on and not check yet
		{
			Entrance_Pressure_Checking = TRUE;			//start buffering
			printf("EntranceRealTimePressure = %f\n",EntranceRealTimePressure);
		}
	}
}

void entrance_pressure_restore_from_low_dual()
{
	if(EntranceRealTimePressure >=  (float)(Valve_Control1[4]/100.0))	
	{
		ValveSwitchToTank = OFF;
		reset_pump_entrance_pressure_check_startup_monitor();
	}
}
/*
1. 进入参数设置后，在组泵方式参数页面设置“水箱恒压模式”或“无负压模式”后，
	点击进入“阀门控制”，即显示此阀门控制界面；

2. 启用进水阀门：即水箱液位低于设定的“液位下限”值，接通电磁阀，
	打开进水阀门进行水箱注水。当液位值高于设定的“液位上限”值，断开电磁阀，关闭进水阀门停止水箱注水。

3. 启用泄压阀门：即水泵出口压力高于设定的“压力上限”值，接通电磁阀，打开管网泄压阀门进行管网泄水。
	当压力值低于设定的“压力下限”值时，断开电磁阀，关闭泄压阀门停止管网泄压。

*/
void water_entrance_monitor()
{
	static uint8 valve_state = OFF;
	if((Entrance_sensor0[0] & 0x0f) == OFF) // not floating ball
	{
		if(Valve_Control[0] == ON)
		{
			if((WaterLevel < VALVE_WATER_LEVEL_FLOOR) && (valve_state == OFF))
			{
				ValveIntoTank = ON;
				valve_state = ON;
#ifdef DBG_EVENT_AS_ERROR
				EventRecorder("水位偏低，打开进水阀门");
#endif
			}
			else if((WaterLevel > VALVE_WATER_LEVEL_CEILING) && (valve_state == ON))
			{
			    ValveIntoTank = OFF;	
				valve_state = OFF; 
#ifdef DBG_EVENT_AS_ERROR
				EventRecorder("水位恢复，关闭进水阀门");
#endif 
			}
		}
	}
}

void decompression_monitor()
{
	static uint8 valve_state = OFF;
	if(Valve_Control[1] == ON)	  //enabled
	{
		if((OutletRealTimePressure > VALVE_PRESSURE_CEILING) && (valve_state == OFF))
		{
			ValveDecompression = ON;
			valve_state = ON;
#ifdef DBG_EVENT_AS_ERROR
			EventRecorder("管压过高，打开泄压阀门");
#endif
		}
		else if((OutletRealTimePressure < VALVE_PRESSURE_FLOOR) && (valve_state == ON))
		{
		    ValveDecompression = OFF;
			valve_state = OFF; 
#ifdef DBG_EVENT_AS_ERROR
			EventRecorder("管压恢复，关闭泄压阀门");
#endif
		}
	}
}
char lack_of_water()
{
	if(WATER_SHORTAGE_SC_EFFECTIVE == OFF)	//常开有效	o/c effective
	{
		return StatusCH1;
	}
	else return 1 - StatusCH1;
}

char safety_protection()
{
	if(SAFETY_PROTECT_SC_EFFECTIVE == OFF)	//常开有效	o/c effective
	{
		return StatusCH2;
	}
	else return 1 - StatusCH2;
}

char fc_error()
{
	char result;
	if(FC_DEFECT_SC_EFFECTIVE == OFF)	//常开有效	o/c effective
	{
		result =  StatusCH3;
	}
	else
	{
	 	result = 1 - StatusCH3;
	}
	if((result == DIN_ERROR) && (FC_DEFECT_STOP == 1)) FatalErrorOccured = TRUE;
	return result;
}

#define CMD_CHK                             0
#define CMD_SET_TIMER_ERR_ON                1
#define CMD_SET_TIMER_ERR_OFF               2

/********************************************************************************
*  There are 2 counter teams for confirmation of error-adding and 
*  error-canceling respectively.
* -- When an error signal is captured, the relevant counter1 is loaded
*    with 1(s) and starts counting down. the next second it reaches 0, 
*     1 is returned for new error-confirmation 
* 
*  --When an error-canceliing signal is captured, the relevant counter2
*     is loaded with 5(s) and starts counting down. the next second it reaches 0, 
*     -1 is returned for error-canceling confirmation  
*
*  --Otherwise it returns 0 showing that none of the above occurs
*********************************************************************************/
int bomb_checker(uint8 ind, uint8 cmd)
{
	uint8 status;
	static uint8 counter1[6] = {0, 0, 0, 0, 0, 0};
	static uint8 counter2[6] = {0, 0, 0, 0, 0, 0};

//#define DBG_BOMB_CHECKER
//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_BOMB_CHECKER

    printf("\n\n\n Entrance: int bomb_checker(uint8 ind, uint8 cmd)...............\n\n");
#endif  
//--------------------------------------------------------------------------------------------------

	if(cmd == CMD_SET_TIMER_ERR_ON)
	{
		if(counter1[ind] == 0)	//+  2020-6-2 out of checking window, enter it
		{
			counter1[ind] = 1;
			counter2[ind] = 0;
		}
#ifdef DBG_BOMB_CHECKER
		else
		{
			printf("in err-confirming check window already, skip...\n");
		}
#endif

	}
	else if(cmd == CMD_SET_TIMER_ERR_OFF)
	{
		if(counter2[ind] == 0)	//+  2020-6-2 out of checking window, enter it
		{
			counter1[ind] = 0;
			counter2[ind] = 2;
		}
#ifdef DBG_BOMB_CHECKER
		else
		{
			printf("in err-cancel-confirming check window already, skip...\n");
		}
#endif
	}
	else
	{
		
		if(counter1[ind] > 0)
		{
			
			counter1[ind]--;
			printf("                                                 counter1[%d] = %d\n\n", ind, counter1[ind]); 
			if(counter1[ind] == 0)  
			{
#ifdef DBG_BOMB_CHECKER
			    display(counter1, 4);
				display(counter2, 4);
				printf("check pump%d, return 1;\n", ind);

#endif
				status = cmd / 10;
				printf("status = %d\n",status);

				if(status == 0)				   //+  2020-6-2
				{
#ifdef DBG_BOMB_CHECKER
					printf("error  confirmed\n");
#endif
					return 1;
				}
#ifdef DBG_BOMB_CHECKER
				else
				{
					printf("fake error, skip...\n");
				}
#endif
			}
	
		}
		if(counter2[ind] > 0)
		{
			
			counter2[ind]--;
			printf("                                                 counter2[%d] = %d\n\n", ind, counter2[ind]); 
			if(counter2[ind] == 0)  
			{
#ifdef DBG_BOMB_CHECKER
			    display(counter1, 4);
				display(counter2, 4);
				printf("check pump%d, return -1;\n", ind);

#endif
				status = cmd / 10;
				printf("status = %d\n",status);

				if(status == 1)				   //+  2020-6-2
				{
#ifdef DBG_BOMB_CHECKER
					printf("error-cancel confirmed\n");
#endif
					return -1;
				}
#ifdef DBG_BOMB_CHECKER
				else
				{
					printf("fake error-cancel, skip...\n");
				}
#endif
			}	
		}
	}
#ifdef DBG_BOMB_CHECKER
	printf("counter1: ");
    display(counter1, 4);
	printf("counter2: ");
	display(counter2, 4);
	printf("check pump%d, return 0;\n", ind);

#endif
	return 0;
}

#define ERROR_OCCUR_SIGNAL_CAPTURED 				  (status == 0) && (pump_prev_status[i] == 1)	      
#define	ERROR_CANCEL_SIGNAL_CAPTURED				  (status == 1) && (pump_prev_status[i] == 0)

#define NEW_STATUS_CONFIRMED                           bomb_status != 0
#define ERROR_CONFIRMED                                bomb_status == 1

#define ERROR_ADDING                                   1
#define ERROR_REMOVAL                                  -1

void update_ui_sys_variables(uint8 i, int err_status)
{
	static uint8 err_counter = 0;
	uint8 info_type;

//#define DBG_UPDATE_UI_SYS_VARIABLES          //Enabled for debugging only
//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_UPDATE_UI_SYS_VARIABLES

    printf("\n\n\n Entrance: void update_ui_sys_variables(uint8 i, int err_status)...............\n\n");
#endif  
//--------------------------------------------------------------------------------------------------

	//update system variables
	pump_usablilty_tbl[i] = (err_status == ERROR_ADDING)? FALSE : TRUE;
	pump_error_tbl[i]     = (err_status == ERROR_REMOVAL)? FALSE : TRUE;

	//Error indicator
	if(err_status == ERROR_ADDING)
	{
		err_counter++;
		if(err_counter == 1) warning_out = ON;		//1st error issues the warning signal
#ifdef DBG_UPDATE_UI_SYS_VARIABLES
    	printf("-------------------------error number = %d\n\n", err_counter + ErrorCounter);
#endif 
			
	}
	else   //error cancelled
	{	
		if(err_counter > 0)  err_counter--;	
		if(err_counter + ErrorCounter == 0) warning_out = OFF;		
#ifdef DBG_UPDATE_UI_SYS_VARIABLES
    	printf("-------------------------error number = %d\n\n", err_counter + ErrorCounter);
#endif 
	}

	//update UI: buttons/info bar/error report screen
	RefreshButtons();
	info_type = (err_status == ERROR_ADDING)? ADD_ERROR_YES : ADD_ERROR_NO;
	RefreshErrorInfo(i, info_type);

}

void sys_restart(void)
{
	PIDParament_Init();
				
	start_task_scheduler();

	RefreshButtons();	
		 
	start_mod_1_2();
	PumpErrStatus = PUMP_RUNNING;	

}

void fc_pump1_restart(void)
{
 	Set_FC_StartValue();
	show_frequency();

	switch_to_fc(0);		   
	amount_of_running_pumps++;
			
	pump_switch_phase_mod2 = 3;
#ifdef DBG_STATE_MACHINE
    SetControlBackColor(SCREEN_MAIN_1, 63, 0x07e0);	   //2 -> 3
#endif
	
	fast_mode = TRUE;
	adding_pump_executing = TRUE;
	timer_tick_count_rec_mod2 = timer_tick_count; 

}




char pump_error(uint8 i, uint8 status)
{
	int bomb_status;	
	static uint8 pump_prev_status[6] = {1, 1, 1, 1, 1, 1};

//#define DBG_PUMP_ERROR          //Enabled for debugging only
//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_PUMP_ERROR

    printf("\n\n\n Entrance: char pump_error(uint8 i, uint8 status)...............\n\n");
#endif  
//--------------------------------------------------------------------------------------------------


	IWDG_FEED
	bomb_status = bomb_checker(i, CMD_CHK + status * 10);

	if(NEW_STATUS_CONFIRMED ) 
	{
		update_ui_sys_variables(i, bomb_status);
		if(ERROR_CONFIRMED) 
		{
			//Response: use a spare pump
			printf("                                                 new pump%d error  confirmed\n", i + 1);
		    printf("pump_on_off_tbl[%d] = %d\n", i, pump_on_off_tbl[i]);

			if(pump_on_off_tbl[i] == ON)	 //Replace it with a spare pump
			{
				UseSparePump(i);
			}
			else							 //pump is off	
			{								//
				printf("the pump is not on, do nothing\n");
			}
			
			//show red cross pump-err icon on main screen
			SetControlVisiable(SCREEN_MAIN_1, PumpIcons[i].icon_err, VISIBLE);
			pump_prev_status[i] = 0;		 

			if(Manual_Setting)	 //20210318
			{
				if (AvfMode)
				{
					reset_manual_controls(i + 1, 0);  	         //Disable Manual pf button for pump[i]  0~5 --> 1~11
				}
				else
				{
					printf("reset_manual_controls(%d, 0);\n", i * 2 + 1);
					reset_manual_controls(i * 2 + 1, 0);	//Disable Manual pf button for pump[i]  0~5 --> 1~11
					
					//Small pump(pump5) is an exception, it has no fc button
					if(i != 5) reset_manual_controls(i * 2 + 2, 0);	  //Disable Manual fc button for pump[i]: 0~4 --> 2~10
				}
			}	
		}
		else	 //error canceled
		{
			//Hide red cross
			SetControlVisiable(SCREEN_MAIN_1,PumpIcons[i].icon_err, INVISIBLE);

	    	if(AllPumpsDead == TRUE)	 //recover from all-dead.	 //PATH_TO_MAIN_PUMP_ON_IS_BLOCKED
			{
				if(PATH_TO_MAIN_PUMP_ON_IS_BLOCKED)			 //must be a pf pump being rocovered, do nothing
				{
					printf("PATH_TO_MAIN_PUMP_ON_IS_BLOCKED\n");
				}
				else //if(i == 0)		  //a fc pump is recovered
				{								
					AllPumpsDead = FALSE;
					printf("ALL DEAD STATUS IS UNLOCKED, Restart...\n\n"); 
							  
					sys_restart();
				}	
			}  	
			else if(Pump_Status == MAIN_PUMP_ON)	//partly running already
			{
				if (AvfMode == FALSE)
				{
					if((PumpGroupMode == 2) && (i == 0))		//Fixed fc mode, pump1 recovered, restart
					{
						fc_pump1_restart();
					}
				}
			}
			pump_prev_status[i] = 1;		//Error relieved

			if(Manual_Setting)	 //20210318
			{
				if (AvfMode)
				{
					reset_manual_controls(i + 1, 1);  	         //Enable Manual pf button for pump[i]  0~5 --> 1~11
				}
				else
				{
					reset_manual_controls(i * 2 + 1, 1);  	         //Enable Manual pf button for pump[i]  0~5 --> 1~11
					if(i != 5) reset_manual_controls(i * 2 + 2, 1);	 //Disable Manual fc button for pump[i]: 0~4 --> 2~10
				}
			}				
		}
	}

	//Error radar for the first occurence or canceling
	if(ERROR_OCCUR_SIGNAL_CAPTURED ) 		            // new pump error, to be confirmed 
	{
#ifdef DBG_PUMP_ERROR
		printf("                                                 new pump1 error, not confirmed\n timer is set\n");
#endif
		bomb_checker(i, CMD_SET_TIMER_ERR_ON);				 //switch to error-confirming timer
		
	}		
	else if(ERROR_CANCEL_SIGNAL_CAPTURED)				   //error canceling found, not confirmed yet
	{
#ifdef DBG_PUMP_ERROR	
		printf("error free found, not confirmed yet\n\n"); 
#endif
		bomb_checker(i, CMD_SET_TIMER_ERR_OFF);				//switch to error-cancel confirming timer

	}

#ifdef DBG_FUNC_INFO
	func_dbg_info = OFF;
    if(func_dbg_info) printf("\n\n\n Exit: char pump_error(uint8 i, uint8 status)...............\n\n");
#endif  

	return 1;
		
}

char pump1_error()
{

	uint8 status;
	if(pump_enable_tbl[0] == FALSE) return 1;

#ifdef DBG_VIRTUAL_ERR
	if(PUMP1_DEFECT_SC_EFFECTIVE == OFF)	//常开有效	o/c effective
	{
		status =  VirtualErr[0];
	}
	else
	{
	 	status =  1 - VirtualErr[0];
	}
#else

	if(PUMP1_DEFECT_SC_EFFECTIVE == OFF)	//常开有效	o/c effective
	{
		status =  StatusCH4;
	}
	else
	{
	 	status =  1 - StatusCH4;
	}
#endif

/*	if(TempMonitorEnable)
	{		 			
		if((pump_temp[0] > temp_limit[0]) || (status == 0))   status = 0;
		else status = 1;
	}  */
	return pump_error(0, status);
}
char pump2_error()
{
	uint8 status;
	if(pump_enable_tbl[1] == FALSE) return 1;
	if(PUMP2_DEFECT_SC_EFFECTIVE == OFF)	//常开有效	o/c effective
	{
		status =  StatusCH5;
	}
	else
	{
	 	status =  1 - StatusCH5;
	}
	if(TempMonitorEnable)
	{		 			
		if((pump_temp[1] > temp_limit[1]) || (status == 0))   status = 0;
		else status = 1;
	}

	return pump_error(1, status);
}
//#define DBG_PUMP_ERR
char pump3_error()
{
	uint8 status;
	if(pump_enable_tbl[2] == FALSE) return 1;


	if(PUMP3_DEFECT_SC_EFFECTIVE == OFF)	//常开有效	o/c effective
	{
		status =  StatusCH6;
	}
	else
	{
	 	status =  1 - StatusCH6;
	}
#ifdef DBG_PUMP_ERR
	if(status == 0)	SetTextValue(SCREEN_MAIN_1, 70, "故障");	 
	else 	SetTextValue(SCREEN_MAIN_1, 70, "正常");	 

#endif
	 if(status == 0) printf("====================pump3 error!!!\n");

	if(TempMonitorEnable)
	{		 				
		if((pump_temp[2] > temp_limit[2]) || (status == 0))   status = 0;
		else status = 1;
	}

	return pump_error(2, status);
}

char pump4_error()
{
	uint8 status;

#ifdef DBG_FUNC_INFO
	func_dbg_info = OFF;
#endif	
	 	
	if(pump_enable_tbl[3] == FALSE) 
	{
#ifdef DBG_FUNC_INFO
	 	if(func_dbg_info) printf("pump_enable_tbl[3] == FALSE\n\n");
#endif
		return 1;		
	}

	if(PUMP4_DEFECT_SC_EFFECTIVE == OFF)	//常开有效	o/c effective
	{
		status =  StatusCH7;
	}
	else
	{
	 	status =  1 - StatusCH7;
	}
	if(TempMonitorEnable)
	{		 					
		if((pump_temp[3] > temp_limit[3]) || (status == 0))   status = 0;
		else status = 1;
	}

	return pump_error(3, status);
}

char pump5_error()
{
	uint8 status;
	if(pump_enable_tbl[4] == FALSE) return 1;


	if(PUMP5_DEFECT_SC_EFFECTIVE == OFF)	//常开有效	o/c effective
	{
		status =  StatusCH10;
	}
	else
	{
	 	status =  1 - StatusCH10;
	}

	 if(status == 0) printf("====================pump5 error!!!\n");

	if(TempMonitorEnable)
	{		 				
		if((pump_temp[4] > temp_limit[4]) || (status == 0))   status = 0;
		else status = 1;
	}

	return pump_error(4, status);
}
char pump6_error()
{
	uint8 status;
	if(pump_enable_tbl[5] == FALSE) return 1;


	if(SMALL_PUMP_DEFECT_SC_EFFECTIVE == OFF)	//常开有效	o/c effective
	{
		status =  StatusCH11;
	}
	else
	{
	 	status =  1 - StatusCH11;
	}

	 if(status == 0) printf("====================pump6 error!!!\n");

	if(TempMonitorEnable)
	{		 				
		if((pump_temp[5] > temp_limit[5]) || (status == 0))   status = 0;
		else status = 1;
	}

	return pump_error(5, status);
}

extern uint8 RemoteTargetEnable;
extern float RemoteTargetPressure;
void StartStopResponsor(uint8 cmd)
{
	if(cmd == ON)
	{
		RemoteMode = true;

		if (RemoteTargetEnable)				 //20210625
		{
  			Tartget_Pressure = RemoteTargetPressure;
			ShowTargetPressure(Tartget_Pressure);
			PidSetTargetPressure(Tartget_Pressure);//20210707
			return;
		}

		if(SysRunning == FALSE)		   //stopped now
		{
			
		 	POWER_UP_AUTORUN_DISABLE
	
			
			PIDParament_Init();
						
			start_task_scheduler();
			RefreshButtons();		 
			start_mod_1_2();
			PumpErrStatus = PUMP_RUNNING;

#ifdef USE_CURVES
			up_crossed = FALSE;	
			SetTextValue(SCREEN_CONTROL_CURVES, 23, "--");
#endif
		}	
		else  //running already
		{
			UpdateTargetPressure();
		}

	}
//	else if((cmd == OFF) && (SysRunning == TRUE)) 
	else if (cmd == OFF)						 //20210625
	{
		RemoteMode = false;

		if (RemoteTargetEnable)
		{
  			Tartget_Pressure = LoadTargetPressure();
			ShowTargetPressure(Tartget_Pressure);
			PidSetTargetPressure(Tartget_Pressure);//20210707
			return;
		}
		if (SysRunning == TRUE)
		{
			SysRunning = FALSE;
			Sys_Switches[IND_SYSTEM] = OFF;
	
			UpdateTargetPressure();
			
			Start_Stopper(STOP_TYPE_SYSTEM);
			EscapeSleeper();
		}
	}
}

char remote_start_stop()
{
	uint8 cmd;
	static uint8 remote_cmd_status = 1;
	static uint8 remote_cmd_pre_status = 1;

    remote_cmd_status = StatusCH8;
    if(remote_cmd_status != remote_cmd_pre_status)
    {
	    remote_cmd_pre_status = remote_cmd_status;
	    cmd = 1 - StatusCH8;	    
	    StartStopResponsor(cmd);
	    printf("Remote control\n");

    }
    return 1;
}

char no_running()
{
   if(StatusCH9 == 0)
   {
		return 0;
   }
   return 1;
}

void load_time(uint8* time)
{
	time[0] = now_year;
	time[1] = now_month;
	time[2] = now_day;
	time[3] = now_hour; 
	time[4] = now_min; 
	time[5] = now_sec;
}

/*--------------------Record the new error--------------------------------*/
void general_recorder(uint8 i)
{
	uint8 time[6];
	if(err_record[i] == FALSE)	   //channel i(0~15) is not recorded yet
	{	
		 warning_out = ON;

		 error_status_changed = TRUE;
		 errors[error_info_index ].channel = i + 1;	   //error channel i+1
         errors[error_info_index ].error_code = 1;
		 err_record[i] = TRUE;	
		 ErrorCounter++;

		 RefreshButtons();
         sprintf(errors[error_info_index ].time_stamp, "20%d-%02d-%02d %02d:%02d:%02d---%s",  now_year, now_month, now_day, now_hour, now_min, now_sec, channel_name[i]);
	
	 	 printf("error_info_index = %d\n\n", error_info_index);
		 SetTextValue(SCREEN_FAILURE_INQUIRY_2, error_info_index + 1, (uchar*)errors[error_info_index].time_stamp); 

		 error_info_index = (error_info_index + 1) % 10; //0~9
		 delay_ms(100);

		 load_time(time);
		// display(time, 6);
		 save_error(time, i);
	}	  
}

void show_an_err_record(uint8* err)
{
	char buff[50];
    sprintf(buff, "20%d-%02d-%02d %02d:%02d:%02d---%s",  err[0], err[1],\
					 err[2], err[3], err[4], err[5], channel_name[err[6]]);

 	printf("error_info_index = %d\n\n", error_info_index);
	printf("20%02d-%02d-%02d %02d:%02d:%02d---%s",  err[0], err[1],\
					 err[2], err[3], err[4], err[5], channel_name[err[6]]);
	delay_ms(100);
	SetTextValue(SCREEN_FAILURE_INQUIRY_2, error_info_index + 1, (uchar*)buff); 
	delay_ms(100);

	error_info_index = (error_info_index + 1) % 10; //0~9
  
}
 //Tartget_Pressure 
 void manual_screen_monitor(void)
 {
 	if (Manual_Setting)	
	{
	 	if (OutletRealTimePressure > Tartget_Pressure)
		{
			 reset_manual_controls(0, 1);		//stop, Enable all manual buttons
		}
	}
 }
 extern float temp_restore;
 char over_heat_error(void)
 {
	if(TempMonitorEnable)
	{		 			
		if (pump_temp[0] > temp_limit[0])  return 0;	   //overheat
		if ((pump_temp[0] > temp_restore) && (err_record[16] == TRUE))  return 0; //overheat not relieved		
	}  
	return 1;
 }

//pump_error_tbl[i]

#define NO_PUMP_ERR	   (pump_error_tbl[0] == 0) && (pump_error_tbl[1] == 0) && (pump_error_tbl[2] == 0) &&\
					   (pump_error_tbl[3] == 0) && (pump_error_tbl[4] == 0) && (pump_error_tbl[5] == 0)

void event_monitor(void)
{
	uint8 i;
	/* 
#ifdef ENGLISH_VERSION
	char str[80];
#else	 */
	char str[50];
//#endif

//*****************************UNIT TEST HEADER*****************************************************
 
//--------------------------------------------------------------------------------------------------

	manual_screen_monitor();
/*-------------------------------Scan all the channels----------------------------------*/	
	StatusBuff[0] = outlet_pressure_low();
	StatusBuff[1] = outlet_pressure_high();
	StatusBuff[2] = outlet_pressure_restore_from_high();

	if(WaterSourceMode == TANK_MODE)  //20201015
	{
		StatusBuff[3] = water_level_low();
	}
	else if(WaterSourceMode == NONNEGTIVE_MODE)	   //clear TANK MODE wl error if any
	{
	   //20201015-start
		Sys_Switches[IND_ERR_WL_L] = ON;

		StatusBuff[3] = 1;

		//20201015-end
	}

	//entrance
	if(WaterSourceMode == NONNEGTIVE_MODE)
	{
		StatusBuff[4] = entrance_pressure_low();
		entrance_pressure_restore_from_low();	
	}	
	else if(WaterSourceMode == TANK_MODE)
	{
	   //20201009-start
		Sys_Switches[IND_ERR_ENTRANCE_LP] = ON;
	/*	TryToRestoreSysRunning();
		PumpErrStatus = PUMP_RUNNING;	 */
		StatusBuff[4] = 1;

		reset_pump_entrance_pressure_check_startup_monitor();//20200417		  
		//20201009-end
		water_entrance_monitor();	
	}
	//outlet
	decompression_monitor();

	StatusBuff[5] = lack_of_water();
	StatusBuff[6] = safety_protection();
	StatusBuff[7] = fc_error();
	StatusBuff[8] = pump1_error();
	StatusBuff[9] = pump2_error();
	StatusBuff[10] = pump3_error();
	StatusBuff[11] = pump4_error();;//small_pump_error();	 	
	StatusBuff[12] =  remote_start_stop();		   //远程启停
	StatusBuff[13] =  no_running();		           //禁止运行  
	StatusBuff[14] = pump5_error();
	StatusBuff[15] = pump6_error();
	StatusBuff[16] = over_heat_error();

/////////////////////////DBG
if (lack_of_water() == 0) printf("                             lack_of_water() = %d\n",lack_of_water());
if (safety_protection() == 0) printf("                             safety_protection() = %d\n",safety_protection());
if (fc_error() == 0) printf("                             fc_error() = %d\n",fc_error());
if (pump1_error() == 0) printf("                             pump1_error() = %d\n",pump1_error());
if (pump2_error() == 0) printf("                             pump2_error() = %d\n",pump2_error());
if ( pump3_error() == 0) printf("                              pump3_error() = %d\n", pump3_error());
if ( pump4_error() == 0) printf("                              pump4_error() = %d\n", pump4_error());
if (StatusBuff[14] == 0) printf("                              pump5_error() = %d\n", StatusBuff[14]);

if ( StatusBuff[15] == 0) printf("                              pump6_error() = %d\n", StatusBuff[15]);

if (  remote_start_stop() == 0) printf("                               remote_start_stop() = %d\n",  remote_start_stop());
if (  no_running() == 0) printf("                               no_running() = %d\n\n\n\n\n",  no_running());



	/*----------------------check each channel-----------------------------------*/

	for(i = 0; i < WARNING_NBR; i++)			   // for each channel
	{
		if(StatusBuff[i] == DIN_ERROR)	  		   //error in channel i
		{
			error_occured = TRUE;			   //only need to flag once
			error_never_occured = FALSE;

#ifdef DBG_FUNC_INFO
            if(func_dbg_info) printf("===============================error%d: %s occured\n\n", i, channel_name[i]);
#endif 
			
#ifdef DBG_FUNC_INFO
		    if(func_dbg_info) 
			{
				sprintf(DbgBuff, "error%d: %s occured", i, channel_name[i]);
				debugger(DbgBuff, ON);
			}
#endif 
	
			if(Error_Type[i] == ERROR_STOPPER) //the error will stop the system
			{
				if(SysRunning == TRUE)
				{
					escape_sleep();
					Start_Stopper(STOP_TYPE_SYSTEM);
					SysRunning = FALSE;
				}

				else if(Manual_Setting)	 //20210318
				{
					reset_manual_controls(0, 0);	//Disbale all manual buttons
				}
			}

			SetControlVisiable(SCREEN_MAIN_1, ICON_ERRORS_EXIST, VISIBLE);	
			delay_ms(100);
			if(SysRunning == TRUE)
			{
#ifdef ENGLISH_VERSION
				sprintf(str, "AUTO-RUNNING-%s", channel_name[i]);
#else
				sprintf(str, "自动运行状态-%s", channel_name[i]);
#endif
				ApiCommSysStatus = 1;	//DC_CPWS-Ver-1.3.9
			
			}
			else
			{
#ifdef ENGLISH_VERSION
				sprintf(str, "STANDBY-%s", channel_name[i]);
#else
				sprintf(str, "等待状态-%s", channel_name[i]);
#endif
				ApiCommSysStatus = 0;	//DC_CPWS-Ver-1.3.9
			}
			SetTextValue(SCREEN_MAIN_1, TXT_RUNNING_STATUS, (uchar*)str); 
			delay_ms(100);

			SetTextValue(SCREEN_FAILURE_INQUIRY_2, TXT_ERROR_FREE , "");	 //erase 'error-free' msg
			delay_ms(100);
			
			general_recorder(i);	
				  
		}
		/*如果启用“变频器故障时停机保护”，则故障将一直保持，直到控制器上电重新启动；
		如果不启用，则变频器故障信号消失，控制器将自动进入自动控制方式运行。*/
		else if(StatusBuff[i] == 1)// correction				   //the channel is working well
		{
			if(err_record[i] == TRUE)	   // recorded as an error already. error canceled
			{	   
			     error_status_changed = TRUE;
				 err_record[i] = FALSE;	
				 if(ErrorCounter > 0)
				 {
				  	ErrorCounter --;					
							
					if(ErrorCounter == 0)
					{
						printf("/*----------------non-pump error free now---------------------------*/\n");
						//warning_out = OFF;
						if(NO_PUMP_ERR) warning_out = OFF;

						SetControlVisiable(SCREEN_MAIN_1, ICON_ERRORS_EXIST, INVISIBLE);
						delay_ms(100);

						RefreshErrorInfo(0, ADD_ERROR_NO);
																						
						if(SysRunning == FALSE)
						{
							TryToRestoreSysRunning();		
						}

						if(Manual_Setting)	 //20210318
						{
							reset_manual_controls(0, 1);  	//error free now. Enable all manual buttons
						}
						
						printf("ButtonUsabilityManager(CMD_USABILTY_RECOVER, BUTTON_STATUS_NUL);\n");	
				
					} 	
					else	 //Still some errors
					{
						if(Error_Channel_Disconnected == FALSE)		//need to update SCREEN_MANUAL_CTR_2
						{
						}
					}				
				 }
				 RefreshButtons();
				 printf("an error canceled\n\n");			
			}
		}
	}
}

void show_fatal_error(void)
{
	SetControlVisiable(SCREEN_MAIN_1, ICON_START_MASK, VISIBLE);
	delay_ms(100);
	SetControlEnable(SCREEN_MAIN_1, BTN_START_AUTO_SCREEN_MAIN, FALSE);
	delay_ms(100);
	SetControlForeColor(SCREEN_BALANCE_WARNING, 2, 0xf800);
	delay_ms(100);
	ShowMessage(INFO_FATAL_FC_ERROR);
}

void Status_Scan(void)
{
#ifndef MUTE
	static uint8 counter = 2;
			 
#endif
	if(FatalErrorOccured == FALSE) 
	{	
		event_monitor();
	}
	else 
	{
		show_fatal_error();
	}
#ifndef MUTE
	if(counter)
	{ 
		counter--;
		if(counter == 0)
		{
			counter = 2;
			if((ErrorCounter > 0) || (ErrorsExist() == TRUE)) SetBuzzer(100);				   // beep for alerting every 1s
		}
	}	 
#endif
}

void clear_error(void) 
{
	uint8 buff[2] = {0, 0};

	STM32_FLASH_Write(STM32_FLASH_SAVE_ERROR_RECORD, (u16*)buff, 1);
}


void ClearErrorMsg()
{
  uint8 i;
 
  for(i = 1; i <= 10; i++)
  {     
  	SetTextValue(SCREEN_FAILURE_INQUIRY_2, i, "");  
	delay_ms(100);	                                                                  
  }
  clear_error();
  error_info_index = 0;
}
 /*
 1、 进入参数设置后，确认组泵方式里选择的是“水箱恒压模式”，点击“入口传
感器”，进入水箱液位设置界面(即恒压供水设置)；
2、 入口液位传感器：液位开关指示，即未连接液位传感器，使用浮球开关作为水
箱有水和无水的判断。入口变送器连接了液位变送器，即选中 4~20mA 输入；
3、 传感器量程：输入框内输入实际连接的液位变送器量程最高液位；
4、 液位偏差修正：可微调控制器显示水箱液位与实际压力相匹配。
5、 满液位时高度：输入框内输入的数值即为水箱液位的最高高度；
6、 停机时保护液位：水箱液位低于此值时，所有水泵停泵；
7、 恢复运行液位：低液位保护停泵后，水箱液位高于此值后，水泵开始自动启动。
 */		
 
uint8 water_level_event(uint8 status)
{
	uint16 low_height, restore_height;
	low_height = Entrance_sensor0[8] * 256 + Entrance_sensor0[9];
	restore_height = Entrance_sensor0[10] * 256 + Entrance_sensor0[11];

	if(WaterLevel < low_height) return EVENT_WL_UNDERFLOW;

	if(status == LOW_WATER_LEVEL_PROTECTED)
	{ 
		if(WaterLevel >= restore_height) return EVENT_WL_RESTORE;
		else  return EVENT_WL_UNDERFLOW;
	}
	return EVENT_WL_NONE;
}

uint8 WaterLevelMonitor()
{
	uint8 event = EVENT_WL_NONE;
	if(WaterSourceMode == TANK_MODE)
	{	 
#ifdef  DBG_WATER_LEVEL_CHECK
		printf("WaterLevel = %f\n",WaterLevel);
#endif

		if((Entrance_sensor0[0] & 0x0f) == TRUE)	  //use floating ball
		{
		  
		}
		else				  //use HM
		{
			event = water_level_event(PumpErrStatus);	
		}	
		//event processor	
		if(event == EVENT_WL_UNDERFLOW)
		{
			if(PumpErrStatus == PUMP_RUNNING)
			{
				//STOP ALL PUMPS
				Sys_Switches[IND_ERR_WL_L] = OFF;
				escape_sleep();	
				SysRunning = FALSE;
		
				Start_Stopper(STOP_TYPE_SYSTEM);
				printf("-------------------------------------STOP ALL PUMPS\n");
		
				PumpErrStatus = LOW_WATER_LEVEL_PROTECTED;				
			}
		}		
		else if(event == EVENT_WL_RESTORE)
		{
			//RESTART/CONTINUE PUMPS
			Sys_Switches[IND_ERR_WL_L] = ON;
			TryToRestoreSysRunning();
			printf("------------------------------------RESTART/CONTINUE PUMPS\n");
	
			PumpErrStatus = PUMP_RUNNING;
		}	
	}
	return event;		
}


/********************************************************************************************
 * Name： switch_value()
 * Brief：Scan Sys_Switches[] whose elements are connected through AND logic , which means
 *        only if they are all ON, an ON is returned
 *           
 * Para：   null	
 * Return ：AND(Sys_Switches[i]), where 0 <= i <= 4
 * Caller(s):  TryToRestoreSysRunning()
 *
 **********************************************************************************************/
uint8 switch_value()
{
	uint8 i, value = 1;
	for(i = 0; i < 5; i++)
	{
		value *= Sys_Switches[i];
	}
	return value;
}

/********************************************************************************************
 * Name： TryToRestoreSysRunning()
 * Brief：Scan Sys_Switches[] and start pumps if an ON is returned
 *           
 * Para：   null	
 * Return ：null
 * Caller(s):  outlet_pressure_restore_from_high()| entrance_pressure_restore_from_low() |
 *             Status_Scan() | water_level_event()
 *
 **********************************************************************************************/
void TryToRestoreSysRunning()
{
	uint8 sw_val;
	sw_val = switch_value();

	if(ErrorCounter == 0)  //error free
	{
		if(SysRunning == FALSE)
		{
			if(sw_val == ON)
			{
				PIDParament_Init();
		   //option 1
				SysRunning = TRUE;//2020-6-1
				start_mod_1_2();
				start_task_scheduler();
				RefreshButtons();		  
				SysRunning = TRUE;

			//option2		 //2020-6-1	   This could be a better choice, while test is still needed.
			/*
				start_task_scheduler();
				RefreshButtons();		 
				start_mod_1_2();
	
			*/
			}
		}
	}
}

#ifdef  ENGLISH_VERSION
const char* tail[2] = {"", "Error"};
const char* headl[2] = {"", "-"};
const char* strs[4][2] = {
							{"", " Pump1 "}, 
							{"", " Pump2 "}, 
							{"", " Pump3 "}, 
							{"", " Pump4 "}, 
							{"", " Pump5 "},
							{"", " Auxiliary "} 


};
#else
const char* tail[2] = {"", "故障"};
const char* headl[2] = {"", "-"};
const char* strs[6][2] = {
							{"", " 泵1"}, 
							{"", " 泵2"}, 
							{"", " 泵3"}, 
							{"", " 泵4"}, 
							{"", " 泵5"}, 
							{"", " 辅泵"} 


};
#endif

uint8 ErrorsExist(void)
{
	uint8 sum ;
	sum = pump_error_tbl[0] + pump_error_tbl[1] + pump_error_tbl[2] + pump_error_tbl[3] + pump_error_tbl[4] + pump_error_tbl[5];  
	return  (sum + 9) / 10;
}

#ifdef DBG_EVENT_AS_ERROR
void recorder(uint8* event_str, uint8 show_time)
{
	char str[50];
	if(show_time)
	{
		sprintf(str, "20%d-%02d-%02d %02d:%02d:%02d---%s",  now_year, now_month,\
					 now_day, now_hour, now_min, now_sec, event_str);
		SetTextValue(SCREEN_FAILURE_INQUIRY_2, error_info_index % 10  + 1, (uchar*)str); 
	}
	else
	{
		SetTextValue(SCREEN_FAILURE_INQUIRY_2, error_info_index % 10  + 1, (uchar*)event_str); 
	}
	delay_ms(100);
	error_info_index = (error_info_index + 1) % 10;


}

void EventRecorder(uint8* event_str)
{
	recorder(event_str, TRUE);
}

void EventRecorderWithoutTimeStamp(uint8* event_str)
{
	recorder(event_str, FALSE);
}

#endif

void pump_error_recorder(uint8 ind)
{
 	char str[50];
	uint8 time[6];
/*	if(ind == 3)
	{	
#ifdef ENGLISH_VERSION
 		sprintf(str, "20%d-%02d-%02d %02d:%02d:%02d---Auxiliary pump error",  now_year, now_month, now_day, now_hour, now_min, now_sec);
#else  
 		sprintf(str, "20%d-%02d-%02d %02d:%02d:%02d---辅泵故障",  now_year, now_month, now_day, now_hour, now_min, now_sec);
		 		 
#endif
	}
	else	  */
	{
#ifdef ENGLISH_VERSION
		sprintf(str, "20%d-%02d-%02d %02d:%02d:%02d---Pump%d error",  now_year, now_month,\
					 now_day, now_hour, now_min, now_sec, ind + 1);
#else
		sprintf(str, "20%d-%02d-%02d %02d:%02d:%02d---泵%d故障",  now_year, now_month,\
					 now_day, now_hour, now_min, now_sec, ind + 1);
#endif
	}

	SetTextValue(SCREEN_FAILURE_INQUIRY_2, error_info_index % 10  + 1, (uchar*)str); 
	delay_ms(100);
	error_info_index = (error_info_index + 1) % 10;

	load_time(time);
	// display(time, 6);
	save_error(time, 8 + ind);


#ifndef MUTE
    SetBuzzer(100);
#endif	  
}
//ADD_ERROR_YES
void RefreshErrorInfo(uint8 ind, uint8 error_type)
{
 	char str[50];
	uint8 error_status;
//*****************************UNIT TEST HEADER*************************************************************
#ifdef DBG_FUNC_INFO
	 func_dbg_info = OFF;
#endif
#ifdef DBG_UNIT_TEST	
	 if(func_dbg_info) printf("\n\n\n...............Entrance: 【RefreshErrorInfo(uint8 ind, uint8 on_off)】................\n\n");
#endif
//-----------------------------------------------------------------------------------------------------------
#define  ERROR_STR headl[error_status], strs[0][pump_error_tbl[0]], strs[1][pump_error_tbl[1]],\
		        strs[2][pump_error_tbl[2]], strs[3][pump_error_tbl[3]], \
				strs[4][pump_error_tbl[4]], strs[5][pump_error_tbl[5]], tail[error_status]


#ifdef DBG_FUNC_INFO
	if(func_dbg_info) show_on_off();
#endif
    error_status = ErrorsExist();
	SetControlVisiable(SCREEN_MAIN_1, ICON_ERRORS_EXIST, error_status);	
	delay_ms(100);
	printf("SysRunning = %d\n",SysRunning);

	if(SysRunning == TRUE)
	{
		if(Pump_Status == MAIN_PUMP_ON)
		{
#ifdef ENGLISH_VERSION
			sprintf(str, "AUTO-RUNNING%s%s%s%s%s%s%s%s", ERROR_STR);	

#else
			sprintf(str, "自动运行状态%s%s%s%s%s%s%s%s",ERROR_STR);	
#endif
			ApiCommSysStatus = 1;	//DC_CPWS-Ver-1.3.9
		}
		else if(Pump_Status == ALL_SLEEP)
		{
#ifdef ENGLISH_VERSION
			sprintf(str, "DEEP SLEEP%s%s%s%s%s%s%s%s", ERROR_STR);	
#else		
			sprintf(str, "深度休眠状态%s%s%s%s%s%s%s%s", ERROR_STR);	
#endif
			ApiCommSysStatus = 2;	//DC_CPWS-Ver-1.3.9
		}	
		else if(Pump_Status == SMALL_PUMP_ON)
		{
#ifdef ENGLISH_VERSION
			sprintf(str, "LIGHT SLEEP%s%s%s%s%s%s%s%s",ERROR_STR);	
#else		
			sprintf(str, "浅度休眠状态%s%s%s%s%s%s%s%s", ERROR_STR);	
#endif
			ApiCommSysStatus = 2;	//DC_CPWS-Ver-1.3.9
		}	
	
	}
	else
	{
#ifdef ENGLISH_VERSION
		sprintf(str, "STANDBY%s%s%s%s%s%s%s%s", ERROR_STR);	

#else
		sprintf(str, "等待状态%s%s%s%s%s%s%s%s", ERROR_STR);
#endif
		ApiCommSysStatus = 0;	//DC_CPWS-Ver-1.3.9
	}
	SetTextValue(SCREEN_MAIN_1, TXT_RUNNING_STATUS, (uchar*)str); 
	delay_ms(100);

	//Only added errors are saved into error-checking page and flash, 
	//while the realtime status is updated on the ifo bar at the bottom of Main Screen
	if(error_type == ADD_ERROR_YES)
	{
		pump_error_recorder(ind);
	}

#ifdef DBG_UNIT_TEST
	 if(func_dbg_info) printf("\n\n\n...............Exit: 【RefreshErrorInfo(uint8 ind, uint8 on_off)】................\n\n");
#endif

}

void IWDG_Config(uint8_t prv ,uint16_t rlv)
{    
    // 使能 预分频寄存器PR和重装载寄存器RLR可写
    IWDG_WriteAccessCmd( IWDG_WriteAccess_Enable );
    
    // 设置预分频器值
    IWDG_SetPrescaler( prv );
    
    // 设置重装载寄存器值
    IWDG_SetReload( rlv );
    
    // 把重装载寄存器的值放到计数器中
    IWDG_ReloadCounter();
    
    // 使能 IWDG
    IWDG_Enable();    
}
