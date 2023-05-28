/*************************************************************************************
**                            深圳英捷思科技有限公司
**                              
**-----------------------------------Breif--------------------------------------------
**  This is mainly a gateway to Local Flash, where configuration data set by user is
**  sent to flash and read out of it before refresh relevant variables and UI controls.
**	----Flash Operation
**           --Reading
**           --Writing
**
**  ----Data Processing
**
**  ----Controls Updating
**
**  
--------------------------------------------------------------------------------------*/
/*
 	----Upgrading Guideline---


---------------------------------------------------------------------------------------*/
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
#include "pump_running.h"
#include "pump_sleeping.h"
#include "error_handle.h"
#include "macros.h"
#include "settings.h"
#include "stm32_flash.h"
#include "measurement.h"
#include "UserInterface.h"
#include "time_control.h"
#include "security.h"
#include "ui_controls.h"

#define LOAD_VERSION_NBR SetTextValue(SCREEN_SYS_SETTINGS_2, 32, VERSION_NO);

extern float PumpMaxFreq;	//20210911
	
extern  TYPE_PUMP_ICONS PumpIcons[];

extern uint8 UseIoTargetPressure;
float DefaultTargetPressureMenu;

extern uint8 Use60HzAsMaxFreq;
extern uint8 MaxFreqOut;

extern uint8 DeviceAddress;

extern uint8 ApiCommSysStatus;

extern float RemoteTargetPressure;
extern bool RemoteMode;

extern uint8 FactoryInfoCheckPassed;
extern float phy_quantities_bias[];
extern int WL_Bias;
extern float FcFreqOutBias;
extern uint16 pFactor, iFactor, dFactor;
extern uint8 pump_usablilty_tbl[];
extern uint8 Pin_Enabled;
extern uint8 Sys_Pin_Enabled;
extern uint8 Sys_Switches[];
extern uint8 ErrorCounter;
extern uint8 SysRunning;
extern uint8 error_info_index;
extern uint8 now_year;
extern uint8 now_month;
extern uint8 now_day;
extern uint8 now_weekday;
extern uint8 new_day_starts;
extern uint8 now_hour;
extern uint8 now_min;
extern uint8 now_sec;

extern uint16 PowerUpAutoRunTimer;

extern uint8 focused_pump_index;
extern uint8 pin_ok;
extern uint8 Pin_Enabled;

extern uint8 PumpGroupMode;


extern uint8 PumpPointer;

extern uint16 ReturnScreenId;
extern uint8 time_control_enabled;
extern uint8 working_weekdays[];

extern uint8 BL_Control_Enable;
extern uint8 BL_On_Time;

MY_DATE StopDate;
MY_DATE RTC_Date;

extern uint8 TargetLockerTiming;
extern float TargetLockerDelta;

uint8 Use60HzAsMaxFreq;	
uint8 MaxFreqOut = 50;								
uint8 AvfMode = 0;	
uint8 PumpCancelFreq;							


uint8 TrialExpired = FALSE;

uint8 SleepingEnableMode;

uint8 SettingsLoaded = FALSE;


uint8 Pressure_Full_range = 16;
int Pressure_Bias;

uint8 Low_Pressure_Protection_Selected;
uint8 Low_Pressure_Protection_Timing = 10;


uint8 flash_data_loaded = FALSE;

extern uint8 UserPin[];
extern uint8 FactoryPin[];

extern uint8 WaterSourceMode;
extern uint8 Max_Pump_Nbr;

extern uint8 Fixed_FC_Pump;

extern uint8 pump_enable_tbl[];


extern float DefaultTargetPressure;
extern float tc_target_pressure[];

uint8 Mileage[5];
uint16 MileageLimit;
uint16 HourMileage;
uint8 FactoryInfoLocked;
uint8 MileageEnabled = FALSE;

//SCREEN_PUMP_SETTING_3
extern uint8 pump_mode;
extern uint8 WarmUpDone;

//SCREEN_OUTLET_SENSOR
extern uint8 Outlet_Sensor_type;
extern float Outlet_Sensor_Range;
extern float Outlet_Pressure_Bias;
extern uint8 Outlet_LP_Protection_Selected;
extern float Outlet_LP_Protection_Value;
extern uint8 Outlet_HP_Protection_Selected;
extern float Outlet_HP_Protection_Value;

float OutletPressureCaliCoeff;	
float EntrancePressureCaliCoeff;
uint8 RemoteTargetEnable;

//SCREEN_ENTRANCE_SENSOR
extern uint8 Entrance_Sensor_type;
extern float Entrance_Sensor_Range;
extern float Entrance_Pressure_Bias;
extern float Entrance_LP_Protection_Value;
extern uint16 Entrance_LP_Protection_Delay;
extern float Entrance_LP_Protection_Restore_Value;

//SCREEN_ENTRANCE_SENSOR0
extern uint8 Entrance_sensor0[];

//SCREEN_ENTRANCE_SENSOR2
extern uint8 Entrance_sensor2[];

//SCREEN_PUMP_SWITCH_CONDTION
extern uint8 Pump_Switch_Condtion[];
extern int VarIndex_SCREEN_PUMP_SWITCH_CONDTION[];

//SCREEN_SLEEP_SETTING
extern uint8 Sleep_Setting[];
extern int VarIndex_SCREEN_SLEEP_SETTING[];

//SCREEN_SCREEN_POWER_UP_SETTING
extern uint8 Power_Up_Setting[];

//SCREEN_VALVE_CONTROL
extern uint8 Valve_Control[];

//SCREEN_VALVE_CONTROL1
extern uint8 Valve_Control1[];



//SCREEN_PID_SETTING
extern uint8 PID_Setting[];

extern uint8 SettingsLoaded;

extern float SpeedUpFactor;//20210712

uint8 FactoryInfoChecked = 0;

char PhoneNumber[14];
char CompanyName[42];

#ifdef USE_PIN_GENERATOR   //
	uint8 PW_ErrIndex;
#endif

#define IS_TIME_SEG_SELECTOR    (ctr_id >= 10) && (ctr_id <= 15)
#define IS_TIME_SEG_EN_DIS       ctr_id == 1
#define IS_TARGET_PRESSURE		((ctr_id >= 48) && (ctr_id <= 53))||(ctr_id == 54)

typedef struct{
				uint8 head_ind;
				uint8 err_nbr;
			//	ERR_CELL err_queue[16];
	
	
	
} ERROR_RECORD;

ERROR_RECORD flash_err_record;

typedef struct
{
	uint8 control_type;		  //
	uint8 control_id;
}CONTROL_INFO;

const CONTROL_INFO control_index_tbltbl[8] = {
				{0,1},
				{0,2},
				{0,3},
				{0,4},
				{0,5},
				{0,6},
				{0,7},
				{0,8}

};

const uint8 control_index_tbl[39] = {
				0, 			            
				1,
				10, 16, 17, 18, 19,
				11, 20, 21, 22, 23,
				12,	24, 25, 26, 27,
				13,	28, 29, 30, 31,
				14,	32, 33, 34, 35,
				15,	36, 37, 38, 39,
				54, 48, 49, 50, 51,
				52, 53

};	


#ifdef DBG_FACTORY_SETTINGS		//DBG ONLY!!!
const int DefaultSettings[185] = {  
   //                                        SCREEN_PUMP_GROUPING,        //
                                           0,        //Pump_Grouping_Mode		  0
                                           3,        //Max_Pump_Nbr
                                           0,        //Small_Pump_Selected
                                           0,        //Fixed_FC_Pump
                                           1,        //pump_enable_tbl[0]
                                           1,        //pump_enable_tbl[1]
                                           1,        //pump_enable_tbl[2]
                                           0,        //pump_enable_tbl[3]		  7
   //                                        SCREEN_OUTLET_SENSOR,        //
                                           1,        //Outlet_Sensor_type
                                           0,        //Outlet_Sensor_Range_H
                                           160,        //Outlet_Sensor_Range_L
                                           0,        //Outlet_Pressure_Bias_H
                                           0,        //Outlet_Pressure_Bias_L
                                           0,        //Outlet_LP_Protection_Selected
                                           20,        //Outlet_LP_Protection_Value
                                           0,        //Outlet_HP_Protection_Selected
                                           0,        //Outlet_HP_Protection_Value_H
                                           50,        //Outlet_HP_Protection_Value_L  10		17
   //                                        SCREEN_ENTRANCE_SENSOR,        //
                                           0,        //Entrance_Sensor_type
                                           1,        //Entrance_Sensor_Range_H
                                           64,        //Entrance_Sensor_Range_L
                                           0,        //Entrance_Pressure_Bias_H
                                           0,        //Entrance_Pressure_Bias_L
                                           2,        //Entrance_LP_Protection_Value
                                           0,        //Entrance_LP_Protection_Delay_H
                                           10,        //Entrance_LP_Protection_Delay_L
                                           3,        //Entrance_LP_Protection_Restore_Value
                                           0,        //										   	  27
   //                                        SCREEN_ENTRANCE_SENSOR0,        //
                                           1,        //Entrance_sensor0[0]
                                           0,        //Entrance_sensor0[1]
                                           3,        //Entrance_sensor0[2]
                                           0xe8,        //Entrance_sensor0[3]
                                           0,        //Entrance_sensor0[4]
                                           0,        //Entrance_sensor0[5]
                                           2,        //Entrance_sensor0[6]
                                           0x58,        //Entrance_sensor0[7]
                                           0,        //Entrance_sensor0[8]
                                           200,        //Entrance_sensor0[9]
                                           1,        //Entrance_sensor0[10]
                                           44,        //Entrance_sensor0[11]						   39
   //                                        SCREEN_ENTRANCE_SENSOR2,        //
                                           0,        //Entrance_sensor2[0]
                                           0,        //Entrance_sensor2[1]
                                           160,        //Entrance_sensor2[2]
                                           0,        //Entrance_sensor2[3]							  43
   //                                        SCREEN_PUMP_SWITCH_CONDTION,        //
                                           0,        //Pump_Switch_Condtion[0]
                                           5,        //Pump_Switch_Condtion[1]
                                           0,        //Pump_Switch_Condtion[2]
                                           5,        //Pump_Switch_Condtion[3]
                                           20,        //Pump_Switch_Condtion[4]
                                           0,        //Pump_Switch_Condtion[5]
                                           8,        //Pump_Switch_Condtion[6]
                                           0,        //Pump_Switch_Condtion[7]
                                           2,        //Pump_Switch_Condtion[8]
                                           0,        //Pump_Switch_Condtion[9]
                                           2,        //Pump_Switch_Condtion[10]
                                           0,        //Pump_Switch_Condtion[11]
                                           2,        //Pump_Switch_Condtion[12]
                                           1,        //Pump_Switch_Condtion[13]
                                           244,        //Pump_Switch_Condtion[14]
                                           3,        //Pump_Switch_Condtion[15]							59
   //                                        SCREEN_SLEEP_SETTING,        //
                                           0,        //Sleep_Setting[0]	  SLEEP_ENABLE_MODE/SLEEP_JUDGE_DELAY_H
                                           5,        //Sleep_Setting[1]   SLEEP_JUDGE_DELAY_L        20210903
                                           25,       //Sleep_Setting[2]   MAIN_PUMP_SLEEP_JUDGE_FREQ 20210903
                                           6,        //Sleep_Setting[3]
                                           0,        //Sleep_Setting[4]	  MAIN_PUMP_WAKE_UP_DELAY_H
                                           5,        //Sleep_Setting[5]   MAIN_PUMP_WAKE_UP_DELAY_L	 20210903
                                           40,        //Sleep_Setting[6]
                                           3,        //Sleep_Setting[7]
                                           3,        //Sleep_Setting[8]
										   0,		//												   69
   //                                        SCREEN_POWER_UP_SETTING,        //
 										   0,        //Power_Up_Setting[0]	 FC_DEFECT_STOP
                                           0,        //Power_Up_Setting[1]	 WATER_SHORTAGE_SC_EFFECTIVE
                                           0,        //Power_Up_Setting[2]	 SAFETY_PROTECT_SC_EFFECTIVE
                                           0,        //Power_Up_Setting[3]	 FC_DEFECT_SC_EFFECTIVE
                                           0,        //Power_Up_Setting[4]	 PUMP1_DEFECT_SC_EFFECTIVE
                                           0,        //Power_Up_Setting[5]	 PUMP2_DEFECT_SC_EFFECTIVE
                                           0,        //Power_Up_Setting[6]	 PUMP3_DEFECT_SC_EFFECTIVE
                                           0,        //Power_Up_Setting[7]	 SMALL_PUMP_DEFECT_SC_EFFECTIVE
                                           0,        //Power_Up_Setting[8]
                                           0,        //Power_Up_Setting[9]
                                           0,        //Power_Up_Setting[10] 
										   0,        //Power_Up_Setting[11]							   81
   //                                        SCREEN_VALVE_CONTROL,        //Valve_Control[0]
                                           0,        //Valve_Control[0]
                                           0,        //Valve_Control[1]
                                           1,        //Valve_Control[2]
                                           144,        //Valve_Control[3]
                                           2,        //Valve_Control[4]
                                           88,        //Valve_Control[5]
                                           48,        //Valve_Control[6]
                                           35,        //Valve_Control[7]								 89
   //                                        SCREEN_VALVE_CONTROL1,        //Valve_Control1[0]
                                           10,        //Valve_Control1[0]
                                           10,        //Valve_Control1[1]
                                           240,        //Valve_Control1[2]
                                           24,        //Valve_Control1[3]
                                           15,        //Valve_Control1[4]
										   0,															  // 95
   //                                        SCREEN_PID_SETTING,        
                                           0,        //PID_Setting[0]									 SELF_DEF_PID_PAR+PUMP_POINTER*10 
                                           1,        //PID_Setting[1]
                                           1,        //PID_Setting[2]
                                           44,        //PID_Setting[3]
                                           0,        //PID_Setting[4]
                                           2,        //PID_Setting[5]
                                           0,        //PID_Setting[6]
                                           0,        //PID_Setting[7]									   //103
 
                                           //PHONE_NBR,        //
                                           ' ',        //
                                           ' ',        //
                                           ' ',        //
                                           ' ',        //
                                           ' ',        //
                                           ' ',        //
                                           ' ',        //
                                           ' ',        //
                                           ' ',        //
                                           ' ',        //
                                           ' ',        //
                                           ' ',        //
                                           ' ',        //
                                           ' ',        //											   //117
										   //expire date
                                           20,        //
                                           50,        //
                                           12,        //
                                           31,        //
                                           0,        //
                                           0,        //											  123
										   //  PW_SWITCH
                                           0,        //
                                           0,        //
										   //   POWER_SAVING
                                           1,        //
                                           30,        //										  127
   //                                        SCREEN_TIME_CTR_3 ,        //
                                           255,        //WKD_EN
                                           0,        //Task1:enable
                                           6,        //Task1:start hour
                                           30,        //Task1:start minute
                                           7,        //Task1:stop hour
                                           30,        //Task1:stop minute
                                           0,        //Task2:enable
                                           8,        //Task2:start hour
                                           30,        //Task2:start minute
                                           9,        //Task2:stop hour
                                           30,        //Task2:stop minute
                                           0,        //Task3:enable
                                           10,        //Task3:start hour
                                           30,        //Task3:start minute
                                           11,        //Task3:stop hour
                                           30,        //Task3:stop minute
                                           0,        //Task4:enable
                                           12,        //Task4:start hour
                                           30,        //Task4:start minute
                                           13,        //Task4:stop hour
                                           30,        //Task4:stop minute
                                           0,        //Task5:enable
                                           14,        //Task5:start hour
                                           30,        //Task5:start minute
                                           15,        //Task5:stop hour
                                           30,        //Task5:stop minute
                                           0,        //Task6:enable
                                           16,        //Task6:start hour
                                           30,        //Task6:start minute
                                           17,        //Task6:stop hour
                                           30,        //Task6:stop minute
                                           0,        //DefaultTargetPressure_H
                                           40,        //DefaultTargetPressure_L
                                           0,        //tc_target_pressure[0]
                                           40,        //
                                           0,        //tc_target_pressure[1]
                                           40,        //
                                           0,        //tc_target_pressure[2]
                                           40,        //
                                           0,        //tc_target_pressure[3]
                                           40,        //
                                           0,        //tc_target_pressure[4]
                                           40,        //
                                           0,        //tc_target_pressure[5]
                                           40,        //
                                           0,        //	                                                 173
													 //FACTORY DATA
                                           0x5a,        //CHKSUM
                                           88,        //PIN-USER
                                           88,        //
                                           88,        //
                                           66,        //PIN-FACTORY
                                           66,        //
                                           66,        //
										   0,					                                        //181
                                           0,        //PW_ERR_INDEX
                                           0,        //
                                           0xff,        //FACTORY_INFO_LOCKED
};

#else		 //for formal use
const int DefaultSettings[185] = {  
   //                                        SCREEN_PUMP_GROUPING,        //
                                           0,        //Pump_Grouping_Mode		  0
                                           3,        //Max_Pump_Nbr
                                           0,        //Small_Pump_Selected
                                           0,        //Fixed_FC_Pump
                                           1,        //pump_enable_tbl[0]
                                           1,        //pump_enable_tbl[1]
                                           1,        //pump_enable_tbl[2]
                                           3,        //pump_enable_tbl[3]		  7
   //                                        SCREEN_OUTLET_SENSOR,        //
                                           1,        //Outlet_Sensor_type
                                           0,        //Outlet_Sensor_Range_H
                                           160,        //Outlet_Sensor_Range_L
                                           0,        //Outlet_Pressure_Bias_H
                                           0,        //Outlet_Pressure_Bias_L
                                           0,        //Outlet_LP_Protection_Selected
                                           20,        //Outlet_LP_Protection_Value
                                           0,        //Outlet_HP_Protection_Selected
                                           0,        //Outlet_HP_Protection_Value_H
                                           50,        //Outlet_HP_Protection_Value_L  10		17
   //                                        SCREEN_ENTRANCE_SENSOR,        //
                                           0,        //Entrance_Sensor_type
                                           1,        //Entrance_Sensor_Range_H
                                           64,        //Entrance_Sensor_Range_L
                                           0,        //Entrance_Pressure_Bias_H
                                           0,        //Entrance_Pressure_Bias_L
                                           2,        //Entrance_LP_Protection_Value
                                           0,        //Entrance_LP_Protection_Delay_H
                                           10,        //Entrance_LP_Protection_Delay_L
                                           3,        //Entrance_LP_Protection_Restore_Value
                                           0,        //										   	  27
   //                                        SCREEN_ENTRANCE_SENSOR0,        //
                                           1,        //Entrance_sensor0[0]
                                           0,        //Entrance_sensor0[1]
                                           3,        //Entrance_sensor0[2]
                                           0xe8,        //Entrance_sensor0[3]
                                           0,        //Entrance_sensor0[4]
                                           0,        //Entrance_sensor0[5]
                                           2,        //Entrance_sensor0[6]
                                           0x58,        //Entrance_sensor0[7]
                                           0,        //Entrance_sensor0[8]
                                           200,        //Entrance_sensor0[9]
                                           1,        //Entrance_sensor0[10]
                                           44,        //Entrance_sensor0[11]						   39
   //                                        SCREEN_ENTRANCE_SENSOR2,        //
                                           0,        //Entrance_sensor2[0]
                                           0,        //Entrance_sensor2[1]
                                           160,        //Entrance_sensor2[2]
                                           0,        //Entrance_sensor2[3]							  43
   //                                        SCREEN_PUMP_SWITCH_CONDTION,        //
                                           0,        //Pump_Switch_Condtion[0]
                                           35,        //Pump_Switch_Condtion[1]
                                           0,        //Pump_Switch_Condtion[2]
                                           35,        //Pump_Switch_Condtion[3]
                                           20,        //Pump_Switch_Condtion[4]
                                           0,        //Pump_Switch_Condtion[5]
                                           60,        //Pump_Switch_Condtion[6]
                                           0,        //Pump_Switch_Condtion[7]
                                           2,        //Pump_Switch_Condtion[8]
                                           0,        //Pump_Switch_Condtion[9]
                                           20,        //Pump_Switch_Condtion[10]
                                           0,        //Pump_Switch_Condtion[11]
                                           10,        //Pump_Switch_Condtion[12]
                                           1,        //Pump_Switch_Condtion[13]
                                           244,        //Pump_Switch_Condtion[14]
                                           3,        //Pump_Switch_Condtion[15]							59
   //                                        SCREEN_SLEEP_SETTING,        //
                                           0,        //Sleep_Setting[0]
                                           5,        //Sleep_Setting[1]   SLEEP_JUDGE_DELAY_L        20210903
                                           25,       //Sleep_Setting[2]   MAIN_PUMP_SLEEP_JUDGE_FREQ 20210903
                                           6,        //Sleep_Setting[3]
                                           0,        //Sleep_Setting[4]	  MAIN_PUMP_WAKE_UP_DELAY_H
                                           5,        //Sleep_Setting[5]   MAIN_PUMP_WAKE_UP_DELAY_L	 20210903
                                           40,        //Sleep_Setting[6]
                                           3,        //Sleep_Setting[7]
                                           3,        //Sleep_Setting[8]
										   0,		//												   69
   //                                        SCREEN_POWER_UP_SETTING,        //
 										   0,        //Power_Up_Setting[0]	 FC_DEFECT_STOP
                                           0,        //Power_Up_Setting[1]	 WATER_SHORTAGE_SC_EFFECTIVE
                                           0,        //Power_Up_Setting[2]	 SAFETY_PROTECT_SC_EFFECTIVE
                                           0,        //Power_Up_Setting[3]	 FC_DEFECT_SC_EFFECTIVE
                                           0,        //Power_Up_Setting[4]	 PUMP1_DEFECT_SC_EFFECTIVE
                                           0,        //Power_Up_Setting[5]	 PUMP2_DEFECT_SC_EFFECTIVE
                                           0,        //Power_Up_Setting[6]	 PUMP3_DEFECT_SC_EFFECTIVE
                                           0,        //Power_Up_Setting[7]	 SMALL_PUMP_DEFECT_SC_EFFECTIVE
                                           0,        //Power_Up_Setting[8]
                                           0,        //Power_Up_Setting[9]
                                           0,        //Power_Up_Setting[10] 
										   0,        //Power_Up_Setting[11]							   81
   //                                        SCREEN_VALVE_CONTROL,        //Valve_Control[0]
                                           0,        //Valve_Control[0]
                                           0,        //Valve_Control[1]
                                           1,        //Valve_Control[2]
                                           144,        //Valve_Control[3]
                                           2,        //Valve_Control[4]
                                           88,        //Valve_Control[5]
                                           48,        //Valve_Control[6]
                                           35,        //Valve_Control[7]								 89
   //                                        SCREEN_VALVE_CONTROL1,        //Valve_Control1[0]
                                           10,        //Valve_Control1[0]
                                           10,        //Valve_Control1[1]
                                           240,        //Valve_Control1[2]
                                           24,        //Valve_Control1[3]
                                           15,        //Valve_Control1[4]
										   0,															  // 95
   //                                        SCREEN_PID_SETTING,        
                                           1,        //PID_Setting[0]				updated from 0  20210416		 SELF_DEF_PID_PAR+PUMP_POINTER*10 
                                           1,        //PID_Setting[1]							      
                                           0,        //PID_Setting[2]						             P-H	updated from 1 	 20210413
                                           100,        //PID_Setting[3]                   		  	     P-L  	updated from 44  20210413	
                                           0,        //PID_Setting[4]								     I-H
                                           10,        //PID_Setting[5]									 I-L    updated from 2  20210413
                                           0,        //PID_Setting[6]									 D-H
                                           0,        //PID_Setting[7]									 D-L
 
                                           //PHONE_NBR,        //
                                           ' ',        //
                                           ' ',        //
                                           ' ',        //
                                           ' ',        //
                                           ' ',        //
                                           ' ',        //
                                           ' ',        //
                                           ' ',        //
                                           ' ',        //
                                           ' ',        //
                                           ' ',        //
                                           ' ',        //
                                           ' ',        //
                                           ' ',        //											   //117
										   //expire date
                                           20,        //
                                           50,        //
                                           12,        //
                                           31,        //
                                           0,        //
                                           0,        //											  123
										   //  PW_SWITCH
                                           0,        //
                                           0,        //
										   //   POWER_SAVING
                                           1,        //
                                           30,        //										  127
   //                                        SCREEN_TIME_CTR_3 ,        //
                                           255,        //WKD_EN
                                           0,        //Task1:enable
                                           6,        //Task1:start hour
                                           30,        //Task1:start minute
                                           7,        //Task1:stop hour
                                           30,        //Task1:stop minute
                                           0,        //Task2:enable
                                           8,        //Task2:start hour
                                           30,        //Task2:start minute
                                           9,        //Task2:stop hour
                                           30,        //Task2:stop minute
                                           0,        //Task3:enable
                                           10,        //Task3:start hour
                                           30,        //Task3:start minute
                                           11,        //Task3:stop hour
                                           30,        //Task3:stop minute
                                           0,        //Task4:enable
                                           12,        //Task4:start hour
                                           30,        //Task4:start minute
                                           13,        //Task4:stop hour
                                           30,        //Task4:stop minute
                                           0,        //Task5:enable
                                           14,        //Task5:start hour
                                           30,        //Task5:start minute
                                           15,        //Task5:stop hour
                                           30,        //Task5:stop minute
                                           0,        //Task6:enable
                                           16,        //Task6:start hour
                                           30,        //Task6:start minute
                                           17,        //Task6:stop hour
                                           30,        //Task6:stop minute
                                           0,        //DefaultTargetPressure_H
                                           40,        //DefaultTargetPressure_L
                                           0,        //tc_target_pressure[0]
                                           40,        //
                                           0,        //tc_target_pressure[1]
                                           40,        //
                                           0,        //tc_target_pressure[2]
                                           40,        //
                                           0,        //tc_target_pressure[3]
                                           40,        //
                                           0,        //tc_target_pressure[4]
                                           40,        //
                                           0,        //tc_target_pressure[5]
                                           40,        //
                                           0,        //	                                                 173
													 //FACTORY DATA
                                           0x5a,        //CHKSUM
                                           88,        //PIN-USER
                                           88,        //
                                           88,        //
                                           66,        //PIN-FACTORY
                                           66,        //
                                           66,        //
										   0,					                                        //181
                                           0,        //PW_ERR_INDEX
                                           0,        //
                                           0xff,        //FACTORY_INFO_LOCKED
};

#endif


const uint16 setting_screens[12] = {
                                           SCREEN_PUMP_GROUPING,
                                           SCREEN_OUTLET_SENSOR,
                                           SCREEN_ENTRANCE_SENSOR,
                                           SCREEN_ENTRANCE_SENSOR0,
                                           SCREEN_ENTRANCE_SENSOR2,
                                           SCREEN_PUMP_SWITCH_CONDTION,
                                           SCREEN_SLEEP_SETTING,
                                           SCREEN_POWER_UP_SETTING,
                                           SCREEN_VALVE_CONTROL,
                                           SCREEN_VALVE_CONTROL1,
                                           SCREEN_PID_SETTING,
                                           SCREEN_TIME_CTR_3 ,
};
const uint8 flash_settings[12][2] = {
                                           {0,   9},        //0   SCREEN_PUMP_GROUPING
                                           {9,   10},        //1   SCREEN_OUTLET_SENSOR
                                           {19,   9},        //2   SCREEN_ENTRANCE_SENSOR
                                           {28,   12},        //3   SCREEN_ENTRANCE_SENSOR0
                                           {40,   5},        //4   SCREEN_ENTRANCE_SENSOR2
                                           {45,   17},        //5   SCREEN_PUMP_SWITCH_CONDTION
                                           {62,   10},        //6   SCREEN_SLEEP_SETTING
                                           {72,   12},        //7   SCREEN_POWER_UP_SETTING
                                           {84,   9},        //8   SCREEN_VALVE_CONTROL
                                           {93,   6},        //9   SCREEN_VALVE_CONTROL1
                                           {99,   7},        //10   SCREEN_PID_SETTING
                                           {106,   46},        //11   SCREEN_TIME_CTR_3 
};

const uint8 pointer_flash_settings[28] = {  
                                           0,        //0   SCREEN_START
                                           0,        //1   SCREEN_MAIN_1  
                                           0,        //2   SCREEN_MANUAL_CTR_2
                                           0,        //3   SCREEN_USER_SETTINGS_2
                                           0,        //4   SCREEN_SYS_SETTINGS_2
                                           0,        //5   SCREEN_FAILURE_INQUIRY_2
                                           0,        //6   SCREEN_OPERATION_MANUAL
                                           11,        //7   SCREEN_TIME_CTR_3
                                           0,        //8   SCREEN_TIME_SETTING_3
                                           0,        //9   SCREEN_PIN_SETTING_3
                                           0,        //10   SCREEN_SCREEN_SETTING
                                           0,        //11   SCREEN_SUPPLIER_INFO_3
                                           0,        //12   SCREEN_PIN_CONTROL
                                           0,        //13   SCREEN_PIN_CONTROL_FACTORY
                                           0,        //14   SCREEN_BALANCE_WARNING
                                           0,        //15   SCREEN_SCREEN_USER_PIN_MANAGER
                                           0,        //16   SCREEN_PUMP_GROUPING
                                           1,        //17   SCREEN_OUTLET_SENSOR
                                           2,        //18   SCREEN_ENTRANCE_SENSOR
                                           5,        //19   SCREEN_PUMP_SWITCH_CONDTION
                                           6,        //20   SCREEN_SLEEP_SETTING
                                           7,        //21   SCREEN_POWER_UP_SETTING
                                           8,        //22   SCREEN_VALVE_CONTROL
                                           10,        //23   SCREEN_PID_SETTING
                                           0,        //24   SCREEN_CONTROL_CURVES
                                           3,        //25   SCREEN_ENTRANCE_SENSOR0
                                           4,        //26   SCREEN_ENTRANCE_SENSOR2
                                           9,        //27   SCREEN_VALVE_CONTROL1

};


uint8 Temp;
#ifdef  DBG_SCREEN_NAMING
const uint8* screen_name_tbl[20] = {
									"SCREEN_START",
									"SCREEN_MAIN_1  ",
									"SCREEN_MANUAL_CTR_2",
									"SCREEN_USER_SETTINGS_2",
									"SCREEN_SYS_SETTINGS_2",
									"SCREEN_FAILURE_INQUIRY_2",
									"SCREEN_OPERATION_MANUAL",
									"SCREEN_GROUP_SETTING_3",
									"SCREEN_TEMP_SETTING_3",
									"SCREEN_TIME_CTR_3",
									"SCREEN_VALVE_SETTING_3",
									"SCREEN_PUMP_SETTING_3",
									"SCREEN_TIME_SETTING_3",
									"SCREEN_PIN_SETTING_3",
									"SCREEN_SCREEN_SETTING",
									"SCREEN_SUPPLIER_INFO_3",
									"SCREEN_MAIN_ANIMATION",
									"SCREEN_PIN_CONTROL",
									"SCREEN_PIN_CONTROL_FACTORY",
									"SCREEN_BALANCE_WARNING",
};

#endif

/*-------------FOR DEBUG ONLY--------------*/
#ifdef DBG_DISP_INFO
void display(uint8* a, int len)
{
	int i;
	printf("[");
  	for(i = 0; i < len; i++)
    {
    	printf("  %d  ", a[i]);
    }
    printf("]\n\n");
}
void display_str(uint8* a, int len)
{
	int i;
	printf("[");
  	for(i = 0; i < len; i++)
    {
    	printf("  %c  ", a[i]);
    }
    printf("]\n\n");
}
#endif


/**********************************************************************************************
 * Name：UnCompressData(uint8 data_byte, uint8* data_array)
 * Brief：Convert a byte into an array of 8 elements
 * 
 * Para：   data_byte	
 * Return ：data_array[]
 * Example:  data_byte = 0x5a(01011010)=> data_array[] = {0, 1, 0, 1, 1, 0, 1, 0}
 * Caller(s):  
 **********************************************************************************************/

void UnCompressData(uint8 data_byte, uint8* data_array)
{
	uint8 i;
 	for(i = 0; i < 8; i++)
    {   
    	if(data_byte & 0x80) 
		{
			data_array[i] = 1;
		}
		else
		{
			data_array[i] = 0;
		}
    	data_byte = data_byte << 1;
	}
}

 
void set_usability()
{
	uint8 i;
	for(i = 0; i < 6; i++)
	{
		pump_usablilty_tbl[i] = pump_enable_tbl[i];	
	}
} 

/************************************************************************************************
   Read company_name from flash and update text both in	SCREEN_SUPPLIER_INFO_3 and SCREEN_MAIN_1
*************************************************************************************************/
void load_company_name(void)
{
	uint8 read_buff[42];
	uint8 name[42];
	uint8 i, j = 0;
	
	STM32_FLASH_Read(STM32_FLASH_SAVE_COMPANY_NAME, (u16*)read_buff, 21);

	if(read_buff[41] != '#')
	{
		SetTextValue(SCREEN_MAIN_1,  9 , " ");	  

		SetTextValue(SCREEN_SUPPLIER_INFO_3, TXT_COMPANY_NAME, " ");	  

	}
	else
	{
	
		i = strlen((char*)read_buff) - 1;

		while(i > 0)
		{
			printf("(read_buff[%d] = %c\n", i, read_buff[i]);
			if(read_buff[i] == '#')  
			{
				j++;
				i--;
			}
			else 
			{
				break;
			}
		}
	
		sscanf((char*)read_buff, "%[^#]", (char*)name);
		
 		strcpy(CompanyName, (char*)name);	
	
		SetTextValue(SCREEN_SUPPLIER_INFO_3, TXT_COMPANY_NAME, (uchar*)name);	  

		SetTextValue(SCREEN_MAIN_1, SCREEN_MAIN_TXT_COMPANY_NAME, (uchar*)name);


	}
}

/************************************************************************

-----------------------------------------------------
  Test cases
  	--phoneNbr = "0"
	--phoneNbr = "0123"
  	--phoneNbr = "1234567890123"
	--phoneNbr = "12345678901234"
  Ref info:	  flash ind: 104

*************************************************************************/
void load_phone_nbr(uint8* data)
{
	uint8 read_buff[14];
	uint8 phone[14];
	uint8 i, j = 0;

	for(i = 0; i < 14; i++)
	{
		read_buff[i] = data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_PHONE_NBR_INTERNAL + i];

	}

	if(read_buff[13] != '#')
	{
		SetTextValue(SCREEN_MAIN_1,  26 , " ");	  

		SetTextValue(SCREEN_SUPPLIER_INFO_3, TXT_PHONE_NO , " ");	  

		SetTextValue(SCREEN_MAIN_1, TXT_PHONE_NUMBER_MAIN, " ");

	//	SetTextValue(SCREEN_MAIN_1, TXT_QR_CODE_MAIN, "");

	}
	else
	{
	
		i = strlen((char*)read_buff) - 1;
		printf("i = %d\n", i);
		while(i > 0)
		{
			printf("(read_buff[%d] = %c\n", i, read_buff[i]);
			if(read_buff[i] == '#')  
			{
				j++;
				i--;
				printf("i = %d\n", i);
			}
			else 
			{
				break;
			}
		}
	
		sscanf((char*)read_buff, "%[^#]", (char*)phone);
	
		printf("length = %d\n\n", strlen((char*)phone));
		printf("%s@@@@@@@@@@@@@", phone);
		
		strcpy(PhoneNumber, (char*)phone);	
	
		SetTextValue(SCREEN_SUPPLIER_INFO_3, TXT_PHONE_NO , (uchar*)phone);	  


    	SetTextValue(SCREEN_MAIN_1,  26 , "服务热线");	 	                       //  

    	SetTextValue(SCREEN_MAIN_1, TXT_PHONE_NUMBER_MAIN, (uchar*)phone);	       //  

	//	SetTextValue(SCREEN_MAIN_1, TXT_QR_CODE_MAIN, (uchar*)phone);		       // UNUSED


	}
}

 
void load_expire_date(uint8* data)
{
	uint8 test_read_buff[6];
	char buff[5];
	uint8 i;

	for(i = 0; i < 6; i++)
	{
		test_read_buff[i] = data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_EXPIRE_DATE_INTERNAL + i];
	}
	
	date_init_check(test_read_buff);

	StopDate.year = test_read_buff[0] * 100 + test_read_buff[1];
	StopDate.month = test_read_buff[2];
	StopDate.day = test_read_buff[3];
	StopDate.disable_manual = test_read_buff[4];
	StopDate.enable_expire_date = test_read_buff[5];

	sprintf(buff, "%d", test_read_buff[0] * 100 + test_read_buff[1]);	
	SetTextValue(SCREEN_TRAIL_DATE_SETTING, 3 , (uchar*)buff);	  //


	sprintf(buff, "%d", test_read_buff[2]);	
	SetTextValue(SCREEN_TRAIL_DATE_SETTING, 4 , (uchar*)buff);	  //


	sprintf(buff, "%d", test_read_buff[3]);	
	SetTextValue(SCREEN_TRAIL_DATE_SETTING, 5 , (uchar*)buff);	  //


	SetButtonValue(SCREEN_TRAIL_DATE_SETTING, 1, test_read_buff[4]);

	RefreshButtons();
	
	SetButtonValue(SCREEN_TRAIL_DATE_SETTING, 2, test_read_buff[5]);

}

void load_pw_switches(uint8* data)
{
	uint8 read_buff[2];
	uint8 i;

	for(i = 0; i < 2; i++)
	{
		read_buff[i] = data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_PW_SWITCH_INTERNAL + i];

	}

	if(read_buff[0] > 1)  read_buff[0] = 0;
	if(read_buff[1] > 1)  read_buff[1] = 0;

	Pin_Enabled = read_buff[0];
	Sys_Pin_Enabled = read_buff[1];

}

void LoadPowerSavingData(uint8* data)
{
	uint8 read_buff[2];
	uint8 i;

	for(i = 0; i < 2; i++)
	{
		read_buff[i] = data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_POWER_SAVING_INTERNAL + i];
	}

	BL_Control_Enable = read_buff[0];
	BL_On_Time = read_buff[1];
	if(BL_Control_Enable > 1) BL_Control_Enable = 1;
	if((BL_On_Time > 60) || (BL_On_Time == 0)) BL_On_Time = 30;

	printf("BL_Control_Enable = %d\n",BL_Control_Enable);
	printf("BL_On_Time = %d\n",BL_On_Time);



} 

//#define DBG_FLASH_ERR_BLOCK

#ifdef DBG_FLASH_ERR_BLOCK
void show_flash_err_block(void)
{
	uint8 i, j;
	uint8 err[8];
	
	printf("err_record.head_ind = %d\n", flash_err_record.head_ind);
	printf("err_record.err_nbr = %d\n", flash_err_record.err_nbr);
	
	for(i = 0; i < 10; i++)
	{	
		if(i == flash_err_record.head_ind)
		{ 
			printf("[");
			if(0 == flash_err_record.err_nbr) printf("Empty]\n->");
		}
		STM32_FLASH_Read(STM32_FLASH_SAVE_ERROR_RECORD + 2 + 8 * i, (u16*)err, 4); 
		for(j = 0; j < 8; j++)
		{	
			printf("%d ", err[j]);
		}
		if(i == (flash_err_record.head_ind + flash_err_record.err_nbr - 1) % 10) printf("]");
		printf("\n");	
	}
	printf("-----------------------------------------\n");
	
}

#endif

//#define DBG_FLASH_EDITOR
void load_err_record(void)
{
	uint8 i;
	uint32 ind;
	uint8 err[8];
	
	STM32_FLASH_Read(STM32_FLASH_SAVE_ERROR_RECORD, (u16*)err, 1); 
    if((err[0] > 9) || (err[1] > 10))
	{
		flash_err_record.head_ind = 0;
		flash_err_record.err_nbr = 0;
	}
	else
	{
		flash_err_record.head_ind = err[0];
	 	flash_err_record.err_nbr = err[1];
	}

	
	if(flash_err_record.err_nbr == 0)
	{
		printf("No error!\n");
		flash_err_record.head_ind = 0;
		flash_err_record.err_nbr = 0;

		return;
	}
	for(i = 0; i < flash_err_record.err_nbr; i++)
	{
		ind = STM32_FLASH_SAVE_ERROR_RECORD + 2 + 8 * ((flash_err_record.head_ind + i) % 10);

		STM32_FLASH_Read(ind, (u16*)err, 4);
		
		show_an_err_record(err);
		printf("20%02d-%02d-%02d %02d:%02d:%02d---故障%d\n",  err[0], err[1],\
				 err[2], err[3], err[4], err[5],err[6]);
	}

#ifdef DBG_FLASH_ERR_BLOCK
	show_flash_err_block();
#endif


}


/*----------------------------------IN-FLASH DATA STRUCTURE OF ERR-RECORDS--------------------------------------------

              ---In-flash Addr---                                     		---Data---

 Start--	 STM32_FLASH_SAVE_ERROR_RECORD + 0                			   【head_ind】  0..9	  
	 		 STM32_FLASH_SAVE_ERROR_RECORD + 8                			   【err_nbr 】	 0..10 
	 		 STM32_FLASH_SAVE_ERROR_RECORD + 16         *(old head_ind)---->(err_code1 )<--<tail>---new err_code..[OVERFLOW!]
	 		 STM32_FLASH_SAVE_ERROR_RECORD + 24         *(new head_ind)---->(err_code2 )	 
	 	 	   				...									   					...
 End ---	 STM32_FLASH_SAVE_ERROR_RECORD + 88               				(err_code10)
	
----------------------------------------------------------------------------------------------------------------------*/
void save_error(uint8* time, uint8 err_code) 
{
	uint8 j;
	uint32 new_ind;
	uint8 buff[8];
	
	//Tail address: address of the new err-record(8 bytes)
	new_ind = STM32_FLASH_SAVE_ERROR_RECORD + 2 + 8 * ((flash_err_record.head_ind + flash_err_record.err_nbr) % 10);
	if(flash_err_record.err_nbr == 10) 
	{
		//Overflow! start over: override the old head-byte with the new tail-byte and move the head pointer a byte forward
		flash_err_record.head_ind = (flash_err_record.head_ind + 1) % 10;	
	}
	
	if(flash_err_record.err_nbr < 10) flash_err_record.err_nbr++;
	
	
	buff[0] = flash_err_record.head_ind;
	buff[1] = flash_err_record.err_nbr;
	STM32_FLASH_Write(STM32_FLASH_SAVE_ERROR_RECORD, (u16*)buff, 1);		
	
	for(j = 0; j < 6; j++)
	{	
		buff[j] = time[j];
	}
	buff[6] = err_code;
	buff[7] = 0;
	
	STM32_FLASH_Write(new_ind, (u16*)buff, 4);	

}

void SaveDeviceAddress(void)
{

	uint8 buff[2];

	buff[0] = DeviceAddress;
	
	STM32_FLASH_Write(STM32_FLASH_SAVE_DEVICE_ADDRESS, (u16*)buff, 1); 		
}

void SaveUse60Hz(uint8 x)
{

	uint8 buff[2];

	buff[0] = x;
	
	STM32_FLASH_Write(STM32_FLASH_SAVE_USE_60HZ, (u16*)buff, 1); 		
}

void SaveCaliCoeffs(uint8 x)
{

	uint8 buff[2]; 

	buff[0] = x;
	
	STM32_FLASH_Write(STM32_FLASH_SAVE_OUTLET_PRESSURE_CALI_COEFF, (u16*)buff, 1); 	
	STM32_FLASH_Write(STM32_FLASH_SAVE_ENTRANCE_PRESSURE_CALI_COEFF, (u16*)buff, 1); 
		
}

void SaveRemoteTargetEnable(uint8 x)
{

	uint8 buff[2];

	buff[0] = x;
	
	STM32_FLASH_Write(STM32_FLASH_SAVE_REMOTE_TARGET_ENABLE, (u16*)buff, 1); 		
}

void SaveUseIoTargetPressure(uint8 x)
{

	uint8 buff[2];

	buff[0] = x;
	
	STM32_FLASH_Write(STM32_FLASH_SAVE_USE_IO_TARGET_PRESSURE, (u16*)buff, 1); 		
}

void save_target_locker(uint8 timing) 
{
	uint8 j;
	uint32 new_ind;
	uint8 buff[2];

	buff[0] = timing;//TargetLockerTiming;
	
	STM32_FLASH_Write(STM32_FLASH_SAVE_TARGET_LOCKER, (u16*)buff, 1); 		
}

void save_target_locker_delta(float delta) 
{
	uint8 buff[2];

	buff[0] = (int)(delta * 100);	
	STM32_FLASH_Write(STM32_FLASH_SAVE_TARGET_LOCKER_DELTA, (u16*)buff, 1); 
	printf("STM32_FLASH_Write a byte to  %x :  %d\n", STM32_FLASH_SAVE_TARGET_LOCKER_DELTA, buff[0]); 		
}

void save_speed_up_factor_value(float val)	  //20210712
{
	uint8 buff[2];

	buff[0] = (int)(val * 10);	
	STM32_FLASH_Write(STM32_FLASH_SAVE_SPEED_UP_FACTOR, (u16*)buff, 1); 
}

void save_speed_up_factor(void)	  //20210712
{
 	save_speed_up_factor_value(SpeedUpFactor);		
}

void save_pump_cancel_freq(uint8 freq) 
{
	uint8 buff[2];

	buff[0] = freq;	
	buff[1] = 0;	
	STM32_FLASH_Write(STM32_FLASH_SAVE_PUMP_CANCEL_FREQ, (u16*)buff, 1); 
}

//20210911
void save_pump_max_freq(float freq) 
{
	uint8 buff[2];

    /* Save Pump Max Freq*/
    buff[0] = ((int)(freq * 10))>>8;
	buff[1] = ((int)(freq * 10)) & 0xff;

	STM32_FLASH_Write(STM32_FLASH_SAVE_PUMP_MAX_FREQ, (u16*)buff, 1);
}


void save_avf_mode(uint8 avf) 
{
	uint8 buff[2];
	buff[0] = avf;
	buff[1] = 0;
	STM32_FLASH_Write(STM32_FLASH_SAVE_AVF_MODE, (u16*)buff, 1);
}

void load_target_locker(void)
{
	uint8 i;
	uint32 ind;
	uint8 buff[2];
	
	STM32_FLASH_Read(STM32_FLASH_SAVE_TARGET_LOCKER, (u16*)buff, 1); 
    if(buff[0] > 180)
	{
		TargetLockerTiming = 30;
	}
	else
	{
		TargetLockerTiming = buff[0];
	}

	printf("TargetLockerTiming = %d\n",  TargetLockerTiming);



}

float temp_limit[6] = {90.0, 90.0, 90.0, 90.0, 90.0, 90.0};
float temp_bias[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  
float temp_restore = 65.0;
uint8 TempMonitorEnable;
/*
	---Data description---
	  【example: temp limit = 92.5, temp_bias = -0.3】
	
	  buff[0]      integer of temp_limit          92
	  buff[1]      decimal of temp_limit(* 100)   50
	  buff[2]      sign of temp_bias(1:-; 0:+)	  1
	  buff[3]      absolute of temp bias * 10     3    	    
*/
void load_pump_temp_setting(void)
{
	float t_limit, bias, t_restore;
	uint8 buff[18], str[10];
	
	STM32_FLASH_Read(STM32_FLASH_SAVE_PUMP_TEMP, (u16*)buff, 9); 	//18 bytes, only 6 bytes are used 
	delay_ms(100);
	TempMonitorEnable = (buff[16] > 0)? 1:0;
	printf("TempMonitorEnable= %d\n", TempMonitorEnable);


	t_limit = buff[0] + buff[1] / 100.0;	//t_limit: buff[0..1] 92 + 50/100.0 = 92.5 
	if(t_limit <= 100.0)  temp_limit[0] = t_limit;

	bias = (1 - buff[2] * 2) *  buff[3] / 10.0;		//bias:	buff[2..3]
	if(bias * bias <= 100.0) temp_bias[0] = bias;	//within valid range
	phy_quantities_bias[3] = temp_bias[0];     

	temp_restore = buff[4] + buff[5] / 100.0;	//t_limit: buff[4..5] 92 + 50/100.0 = 92.5 
	if ((temp_restore > 150.0) || (temp_restore < 30.0)) temp_restore = 65.0;

	sprintf(str, "%.1f", temp_limit[0]);
	SetTextValue(SCREEN_PUMP_TEM_CONTROL, 2, (uchar*)str);

	sprintf(str, "%.1f", temp_restore);
	SetTextValue(SCREEN_PUMP_TEM_CONTROL, 4, (uchar*)str);

	sprintf(str, "%.1f", temp_bias[0]);
	SetTextValue(SCREEN_PUMP_TEM_CONTROL, 3, (uchar*)str);

	//for unkown reason this has to be repeated
	sprintf(str, "%.1f", temp_limit[0]);
	SetTextValue(SCREEN_PUMP_TEM_CONTROL, 2, (uchar*)str);

	SetButtonValue(SCREEN_PUMP_TEM_CONTROL, 26, TempMonitorEnable);

}

void save_pump_temp_setting(void) 
{
	uint8 i;
	uint8 buff[18];
	
	i = 0;
//	for(i = 0; i < 4; i++)
	{
		printf("temp_limit[%d] = %f\n", i, temp_limit[i]);
		printf("temp_bias[%d] = %f\n", i, temp_bias[i]);
		buff[i * 4] = (int)temp_limit[i];
	    buff[i * 4 + 1] = (int)((temp_limit[i] - buff[i * 4]) * 100);

		buff[i * 4 + 2] = (temp_bias[i] < 0)? 1:0;	  //positive or negative
	    buff[i * 4 + 3] = (temp_bias[i] < 0)? (int)(-temp_bias[i] * 10):(int)(temp_bias[i] * 10);
		printf("buff[%d] = %d,", i*4, buff[i*4]);
		printf("buff[%d] = %d,", i*4+1, buff[i*4+1]);
		printf("buff[%d] = %d,", i*4+2, buff[i*4+2]);
		printf("buff[%d] = %d\n\n", i*4+3, buff[i*4+3]);

	}
	buff[4] = (int)(temp_restore);
	buff[5] = (int)((temp_restore - buff[4]) * 100);

	buff[16] = TempMonitorEnable;
	
	STM32_FLASH_Write(STM32_FLASH_SAVE_PUMP_TEMP, (u16*)buff, 9); 	
	
}
/*
temp_limit:60.5        bias:  -9.8

{ 60, 50, 1, 98}*/
void save_pump_temp_setting_default(void) 
{
	uint8 i;

	for(i = 0; i < 4; i++)
	{
		temp_limit[i] = 90.0;
		temp_bias[i] = 0.0;
	}
	temp_restore = 65.0;
	TempMonitorEnable = 0;
	save_pump_temp_setting();

}
	
void save_remote_target_default(void) 
{
	uint8 data[2] = {0, 40};
	STM32_FLASH_Write(STM32_FLASH_SAVE_REMOTE_TARGET, (u16*)data, 1); 
}
/**********************************************************************************************
 * Name：variables_update(uint8 *_data, uint16 screen_id)
 * Brief：Update variables with data from flash
 *  
 * Para：  *_data: an array of bytes from flash 
 *         screen_id	
 * Return ：null
 * 
 * Caller(s): UseSettingsInFlash()
 **********************************************************************************************/


void variables_update(uint8 *_data, uint16 screen_id)
{
	uint8 i;

	switch(screen_id)
	{
		case SCREEN_PUMP_GROUPING:
/*------------------------SCREEN_PUMP_GROUPING FLASH DATA---------------------------------------	 
	              Relative Addr.	       Data
				  --------------------------------------
	                    0			 WaterSourceMode
						1			 Max_Pump_Nbr
						2            ...UNUSED...
						3			 Fixed_FC_Pump
	 				    4			 pump_enable_tbl[0]
						5			 pump_enable_tbl[1]
						6			 pump_enable_tbl[2]
						7           【X X X X X O O O】
									            | | |
						                        | | +--bit0------pump_enable_tbl[3]
					     		      	        | +----bit1------pump_enable_tbl[4]
									            +------bit2------pump_enable_tbl[5]

*/	
			WaterSourceMode = _data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_PUMP_GROUPING_INTERNAL + 0];
			Max_Pump_Nbr = _data[1];
 
			Fixed_FC_Pump = _data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_PUMP_GROUPING_INTERNAL + 3];
			PumpGroupMode = _data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_PUMP_GROUPING_INTERNAL + 3] + 1;
			pump_enable_tbl[0] = _data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_PUMP_GROUPING_INTERNAL + 4];
			pump_enable_tbl[1] = _data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_PUMP_GROUPING_INTERNAL + 5];
			pump_enable_tbl[2] = _data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_PUMP_GROUPING_INTERNAL + 6];

			
			printf("_data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_PUMP_GROUPING_INTERNAL + 7] = %d\n",\
			      _data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_PUMP_GROUPING_INTERNAL + 7]);

			if ( _data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_PUMP_GROUPING_INTERNAL + 7] > 7)  //out of range  //20210507
				 _data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_PUMP_GROUPING_INTERNAL + 7] = 3; 
		

			pump_enable_tbl[3] = _data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_PUMP_GROUPING_INTERNAL + 7] & 0x01;
			pump_enable_tbl[4] = (_data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_PUMP_GROUPING_INTERNAL + 7] & 0x02)>>1;
			pump_enable_tbl[5] = (_data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_PUMP_GROUPING_INTERNAL + 7] & 0x04)>>2;

			set_usability();
			RefreshButtons();

			break;

		case SCREEN_OUTLET_SENSOR:
		
			Outlet_Sensor_type = _data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_OUTLET_SENSOR_INTERNAL + 0];
			Outlet_Sensor_Range = (float)(_data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_OUTLET_SENSOR_INTERNAL + 1] * 256 + \
					_data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_OUTLET_SENSOR_INTERNAL + 2]) / 100.0;
			printf("Outlet_Sensor_Range = %f\n",Outlet_Sensor_Range);

			if((_data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_OUTLET_SENSOR_INTERNAL + 3] & 0x80) != 0) //negative
			{
			   Outlet_Pressure_Bias = 0 - (float)((_data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_OUTLET_SENSOR_INTERNAL + 3] & 0x7f) * 256\
			   								 + _data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_OUTLET_SENSOR_INTERNAL + 4]) / 100.0;
			   	printf("Outlet_Pressure_Bias < 0\n");
			}
			else
			{
			   Outlet_Pressure_Bias = (float)((_data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_OUTLET_SENSOR_INTERNAL + 3] * 256 +\
			   								 _data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_OUTLET_SENSOR_INTERNAL + 4]) / 100.0);
			   printf("Outlet_Pressure_Bias > 0\n");
			}


			SetBias(Outlet_Pressure_Bias, IND_CH_OUTLET_PRESSURE);

			Outlet_LP_Protection_Selected = _data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_OUTLET_SENSOR_INTERNAL + 5];
			Outlet_LP_Protection_Value =(float)(_data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_OUTLET_SENSOR_INTERNAL + 6] / 100.0);
			Outlet_HP_Protection_Selected = _data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_OUTLET_SENSOR_INTERNAL + 7];
			Outlet_HP_Protection_Value = (float)(_data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_OUTLET_SENSOR_INTERNAL + 8] * 256\
			                                 + _data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_OUTLET_SENSOR_INTERNAL + 9]) / 100.0;

			break;

		case SCREEN_ENTRANCE_SENSOR:
      		printf("start variables_update SCREEN_ENTRANCE_SENSOR\n");
		
			Entrance_Sensor_type = _data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_ENTRANCE_SENSOR_INTERNAL + 0];
			Entrance_Sensor_Range = (float)(_data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_ENTRANCE_SENSOR_INTERNAL + 1] * 256 +\
							 _data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_ENTRANCE_SENSOR_INTERNAL + 2]) / 100.0;

			if((_data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_ENTRANCE_SENSOR_INTERNAL + 3] & 0x80) != 0) //negative
			{
			    Entrance_Pressure_Bias = 0 - (float)((_data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_ENTRANCE_SENSOR_INTERNAL + 3] & 0x7f)\
								 * 256 + _data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_ENTRANCE_SENSOR_INTERNAL + 4]) / 100.0;
			   	printf("Entrance_Pressure_Bias < 0\n");
			}
			else
			{
			   Entrance_Pressure_Bias = (float)((_data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_ENTRANCE_SENSOR_INTERNAL + 3] * 256 + \
			   									 _data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_ENTRANCE_SENSOR_INTERNAL + 4]) / 100.0);
			   printf("Entrance_Pressure_Bias > 0\n");
			}

			SetBias(Entrance_Pressure_Bias, IND_CH_ENTRANCE_PRESSURE);
			Entrance_LP_Protection_Value = (float)(_data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_ENTRANCE_SENSOR_INTERNAL + 5] / 100.0);
			Entrance_LP_Protection_Delay = _data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_ENTRANCE_SENSOR_INTERNAL + 6] * 256\
			 									+ _data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_ENTRANCE_SENSOR_INTERNAL + 7];
			Entrance_LP_Protection_Restore_Value = (float)(_data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_ENTRANCE_SENSOR_INTERNAL + 8] / 100.0);

			printf("End variables_update SCREEN_ENTRANCE_SENSOR\n");


			break;

		case SCREEN_ENTRANCE_SENSOR0:
		
			for(i = 0; i < 12; i++)
			{
			   	Entrance_sensor0[i] = _data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_ENTRANCE_SENSOR0_INTERNAL + i];
			}
			if((Entrance_sensor0[4] & 0x80) != 0) //negative
			{
			   WL_Bias = 0 - ((Entrance_sensor0[4] & 0x7f) * 256 + Entrance_sensor0[5]);
			   printf("WL_Bias < 0\n");
			}
			else
			{
			   WL_Bias = Entrance_sensor0[4] * 256 + Entrance_sensor0[5];
			   printf("WL_Bias > 0\n");
			}
			SetBias((float)WL_Bias, IND_CH_WATER_LEVEL);
			printf("WL_Bias = %d\n", WL_Bias);

			printf("Entrance_sensor0[0] = %02x\n",Entrance_sensor0[0]);

			break;

		case SCREEN_ENTRANCE_SENSOR2:
		
			for(i = 1; i <= 4; i++)
			{
			   	Entrance_sensor2[i - 1] = _data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_ENTRANCE_SENSOR2_INTERNAL + i - 1];
			}
			break;

		case SCREEN_PUMP_SWITCH_CONDTION:
		
			for(i = 1; i <= 16; i++)
			{
			   	Pump_Switch_Condtion[i - 1] = _data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_PUMP_SWITCH_CONDTION_INTERNAL + i - 1];
				
			}		

			break;

		case SCREEN_SLEEP_SETTING:
		
			for(i = 1; i <= 9; i++)
			{
			   	Sleep_Setting[i - 1] = _data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_SLEEP_SETTING_INTERNAL + i - 1];
			}
			SleepingEnableMode = Sleep_Setting[0] / 16;
			printf("SleepingEnableMode = %d\n",SleepingEnableMode);
			Sleep_Setting[0] = Sleep_Setting[0] % 16;

			break;

		case SCREEN_POWER_UP_SETTING:
					
			for(i = 1; i <= 12; i++)
			{
			   	Power_Up_Setting[i - 1] = _data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_POWER_UP_SETTING_INTERNAL + i - 1];
			}
			if((Power_Up_Setting[9] & 0x80) != 0) //negative
			{
			   FcFreqOutBias = 0 - (float)((Power_Up_Setting[9] & 0x7f) * 256 + Power_Up_Setting[10]) / 10.0;
			   printf("FcFreqOutBias < 0\n");
			}
			else
			{
			   FcFreqOutBias = (float)((Power_Up_Setting[9] * 256 + Power_Up_Setting[10]) / 10.0);
			   printf("FcFreqOutBias > 0\n");
			}

		 	Power_Up_Setting[8] = (Power_Up_Setting[8] > 2)? 0:Power_Up_Setting[8];

			break;

		case  SCREEN_VALVE_CONTROL:
		
			for(i = 1; i <= 8; i++)
			{
			   	Valve_Control[i - 1] = _data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_VALVE_CONTROL_INTERNAL + i - 1];
			}
			break;

		case  SCREEN_VALVE_CONTROL1:
		
			for(i = 1; i <= 5; i++)
			{
			   	Valve_Control1[i - 1] = _data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_VALVE_CONTROL1_INTERNAL + i - 1];
			}

			break;

		case  SCREEN_PID_SETTING:
		
			for(i = 1; i <= 8; i++)
			{
			   	PID_Setting[i - 1] = _data[STM32_FLASH_SAVE_ADDR_USER_SCREEN_PID_SETTING_INTERNAL + i - 1];
			}
			PumpPointer = PID_Setting[0] / 10;	  //extract PumpPointer
			PID_Setting[0] = PID_Setting[0] % 10;

			printf("Read PumpPointer = %d\n", PumpPointer);

			pFactor = PID_Setting[2] *256 + PID_Setting[3]; 
			iFactor = PID_Setting[4] *256 + PID_Setting[5]; 
			dFactor = PID_Setting[6] *256 + PID_Setting[7]; 
			load_target_locker();


			break;
		
 		case  SCREEN_SUPPLIER_INFO_3:
	 		load_company_name();
			load_phone_nbr(_data);
		
		 	break;

			
 		case  SCREEN_TRAIL_DATE_SETTING:
			load_expire_date(_data);
			break;

 		case  SCREEN_USER_PIN_MANAGER:
			load_pw_switches(_data);
			break;

 		case  SCREEN_SCREEN_SETTING:
			LoadPowerSavingData(_data);
			break;

		default:
			break;	
	}	



}

void controls_update_SCREEN_SCREEN_SETTING()
{ 
 	 char buff[5];


     SetButtonValue(SCREEN_SCREEN_SETTING, BTN_BL_CONTROL_ENABLE, BL_Control_Enable);


     sprintf(buff, "%d", BL_On_Time);

     SetTextValue(SCREEN_SCREEN_SETTING, TXT_BL_ON_TIME, (uchar*)buff);

     SetPowerSaving(BL_Control_Enable, 255, 0, BL_On_Time * 60);
 
}


void controls_update_SCREEN_PUMP_GROUPING()
{
	char buff[5];
	uint8 i;
	
	printf(" controls_update_SCREEN_PUMP_GROUPING()\n");
	
	sprintf(buff, "%d", Max_Pump_Nbr);
	SetTextValue(SCREEN_PUMP_GROUPING, TXT_MAX_PUMP_NBR, (uchar*)buff);


	if(WaterSourceMode == TANK_MODE)  
	{	
		SetControlVisiable(SCREEN_MAIN_1, ICON_TITLE_NN, INVISIBLE);
		
		SetButtonValue(SCREEN_PUMP_GROUPING, BTN_TANK_MODE, ON);

		SetButtonValue(SCREEN_PUMP_GROUPING, BTN_NONNEGTIVE_MODE, OFF);	

		SetButtonValue(SCREEN_PUMP_GROUPING, BTN_TANK_NONNEGTIVE_MODE, OFF);

		SetControlVisiable(SCREEN_MAIN_1, ICON_NON_NEG, INVISIBLE);

//		SetControlVisiable(SCREEN_MAIN_1, 39, INVISIBLE);	 //???

		SetControlVisiable(SCREEN_MAIN_1, 11, INVISIBLE);

		SetControlVisiable(SCREEN_MAIN_1, 55, VISIBLE);


/*		SetControlVisiable(SCREEN_MANUAL_CTR_2, 11, INVISIBLE);

		SetControlVisiable(SCREEN_MANUAL_CTR_2, 12, INVISIBLE);		   */

		SetControlVisiable(SCREEN_MANUAL_CTR_2, 33, INVISIBLE);

		SetControlVisiable(SCREEN_MANUAL_CTR_2, 34, INVISIBLE);		
			
	}
	else if(WaterSourceMode == NONNEGTIVE_MODE)  
	{	
		SetControlVisiable(SCREEN_MAIN_1, ICON_TITLE_NN, VISIBLE);

		SetButtonValue(SCREEN_PUMP_GROUPING, BTN_TANK_MODE, OFF);

		SetButtonValue(SCREEN_PUMP_GROUPING, BTN_NONNEGTIVE_MODE, ON);	

		SetButtonValue(SCREEN_PUMP_GROUPING, BTN_TANK_NONNEGTIVE_MODE, OFF);

		SetControlVisiable(SCREEN_MAIN_1, ICON_NON_NEG, VISIBLE);

//		SetControlVisiable(SCREEN_MAIN_1, 39, VISIBLE);		 ???

		SetControlVisiable(SCREEN_MAIN_1, 11, VISIBLE);
		
		SetControlVisiable(SCREEN_MAIN_1, 55, INVISIBLE);

		SetControlVisiable(SCREEN_MANUAL_CTR_2, 33, VISIBLE);

		SetControlVisiable(SCREEN_MANUAL_CTR_2, 34, VISIBLE);
	
	}
	else if(WaterSourceMode == TANK_NONNEGTIVE_MODE)  
	{	
		SetButtonValue(SCREEN_PUMP_GROUPING, BTN_TANK_MODE, OFF);

		SetButtonValue(SCREEN_PUMP_GROUPING, BTN_NONNEGTIVE_MODE, OFF);	

		SetButtonValue(SCREEN_PUMP_GROUPING, BTN_TANK_NONNEGTIVE_MODE, ON);	
	}
    delay_ms(100);

	for(i = 0; i < 3; i++) //pump1~3
	{
	    SetButtonValue(SCREEN_PUMP_GROUPING, 4 + i, pump_enable_tbl[i]);	

		SetControlVisiable(SCREEN_MAIN_1, 40 + i, 1 - pump_enable_tbl[i]);

		SetControlVisiable(SCREEN_MAIN_1, 120 + i, 1 - pump_enable_tbl[i]);

	}
	//pump4
    SetButtonValue(SCREEN_PUMP_GROUPING, 3, pump_enable_tbl[3]);	
	SetControlVisiable(SCREEN_MAIN_1,PumpIcons[3].icon_pump_mask, 1 - pump_enable_tbl[3]);
	SetControlVisiable(SCREEN_MAIN_1,PumpIcons[3].icon_temp_mask, 1 - pump_enable_tbl[3]);

	//pump5
    SetButtonValue(SCREEN_PUMP_GROUPING, 14, pump_enable_tbl[4]);	
	SetControlVisiable(SCREEN_MAIN_1,PumpIcons[4].icon_pump_mask, 1 - pump_enable_tbl[4]);
	SetControlVisiable(SCREEN_MAIN_1,PumpIcons[4].icon_temp_mask, 1 - pump_enable_tbl[4]);

	//pump6
    SetButtonValue(SCREEN_PUMP_GROUPING, 7, pump_enable_tbl[5]);	
	SetControlVisiable(SCREEN_MAIN_1,PumpIcons[5].icon_pump_mask, 1 - pump_enable_tbl[5]);
	SetControlVisiable(SCREEN_MAIN_1,PumpIcons[5].icon_temp_mask, 1 - pump_enable_tbl[5]);


	SetButtonValue(SCREEN_PUMP_GROUPING, BTN_FIXED_FC_PUMP_MODE, Fixed_FC_Pump);	

} 

void controls_update_SCREEN_OUTLET_SENSOR()
{
	char buff[5];
	
	printf(" controls_update_SCREEN_OUTLET_SENSOR()\n");
	
	sprintf(buff, "%.2f", Outlet_Sensor_Range);
	SetTextValue(SCREEN_OUTLET_SENSOR, TXT_OUTLET_SENSOR_RANGE, (uchar*)buff);

	printf("Outlet_Pressure_Bias = %0.2f\n",Outlet_Pressure_Bias);
	sprintf(buff, "%.2f", Outlet_Pressure_Bias);
	SetTextValue(SCREEN_OUTLET_CALI, TXT_OUTLET_PRESSURE_CALI_BIAS, (uchar*)buff);

	sprintf(buff, "%.2f", OutletPressureCaliCoeff);
	SetTextValue(SCREEN_OUTLET_CALI, TXT_OUTLET_PRESSURE_CALI_COEFF, (uchar*)buff);

	sprintf(buff, "%.2f", Outlet_LP_Protection_Value);
	SetTextValue(SCREEN_OUTLET_SENSOR, TXT_OUTLET_LP_PROTECTION_VALUE, (uchar*)buff);

	sprintf(buff, "%.2f", Outlet_HP_Protection_Value);
	SetTextValue(SCREEN_OUTLET_SENSOR, TXT_OUTLET_HP_PROTECTION_VALUE, (uchar*)buff);

	if(Outlet_Sensor_type == OUTLET_SENSOER_TYPE_1)  
	{	
		SetButtonValue(SCREEN_OUTLET_SENSOR, BTN_OUTLET_SENSOR_RANGE1, ON);

		SetButtonValue(SCREEN_OUTLET_SENSOR, BTN_OUTLET_SENSOR_RANGE2, OFF);
	}
	else if(Outlet_Sensor_type == OUTLET_SENSOER_TYPE_2)  
	{	
		SetButtonValue(SCREEN_OUTLET_SENSOR, BTN_OUTLET_SENSOR_RANGE1, OFF);

		SetButtonValue(SCREEN_OUTLET_SENSOR, BTN_OUTLET_SENSOR_RANGE2, ON);
	}


	SetButtonValue(SCREEN_OUTLET_SENSOR, BTN_OUTLET_LOW_PRESSURE_PROTECTION, Outlet_LP_Protection_Selected);


	SetButtonValue(SCREEN_OUTLET_SENSOR, BTN_OUTLET_HIGH_PRESSURE_PROTECTION, Outlet_HP_Protection_Selected);


} 

void controls_update_SCREEN_ENTRANCE_SENSOR()
{
	char buff[5];
	
	sprintf(buff, "%.2f", Entrance_Sensor_Range);
	SetTextValue(SCREEN_ENTRANCE_SENSOR, TXT_ENTRANCE_SENSOR_RANGE, (uchar*)buff);

	sprintf(buff, "%.2f", Entrance_Pressure_Bias);
	SetTextValue(SCREEN_ENTRANCE_CALI, TXT_ENTRANCE_PRESSURE_CALI_BIAS, (uchar*)buff);

	sprintf(buff, "%.2f", EntrancePressureCaliCoeff);
	SetTextValue(SCREEN_ENTRANCE_CALI, TXT_ENTRANCE_PRESSURE_CALI_COEFF, (uchar*)buff);

	sprintf(buff, "%.2f", Entrance_LP_Protection_Value);
	SetTextValue(SCREEN_ENTRANCE_SENSOR, TXT_ENTRANCE_LP_PROTECTION_VALUE, (uchar*)buff);

	sprintf(buff, "%d", Entrance_LP_Protection_Delay);
	SetTextValue(SCREEN_ENTRANCE_SENSOR, TXT_ENTRANCE_LP_PROTECTION_DELAY, (uchar*)buff);

	sprintf(buff, "%.2f", Entrance_LP_Protection_Restore_Value);
	SetTextValue(SCREEN_ENTRANCE_SENSOR, TXT_ENTRANCE_LP_PROTECT_RESTORE_VAL, (uchar*)buff);


	if(Entrance_Sensor_type == ENTRANCE_SENSOR_TYPE_1)  
	{	
		SetButtonValue(SCREEN_ENTRANCE_SENSOR, BTN_ENTRANCE_SENSOR_INPUT1_SELECTED, ON);

		SetButtonValue(SCREEN_ENTRANCE_SENSOR, BTN_ENTRANCE_SENSOR_INPUT2_SELECTED, OFF);

		SetButtonValue(SCREEN_ENTRANCE_SENSOR, BTNE_JOINT_PM_SELECTED, OFF);
	}
	if(Entrance_Sensor_type == ENTRANCE_SENSOR_TYPE_2)  
	{	
		SetButtonValue(SCREEN_ENTRANCE_SENSOR, BTN_ENTRANCE_SENSOR_INPUT1_SELECTED, OFF);

		SetButtonValue(SCREEN_ENTRANCE_SENSOR, BTN_ENTRANCE_SENSOR_INPUT2_SELECTED, ON);

		SetButtonValue(SCREEN_ENTRANCE_SENSOR, BTNE_JOINT_PM_SELECTED, OFF);
	}
	if(Entrance_Sensor_type == ENTRANCE_SENSOR_TYPE_3)  
	{	
		SetButtonValue(SCREEN_ENTRANCE_SENSOR, BTN_ENTRANCE_SENSOR_INPUT1_SELECTED, OFF);

		SetButtonValue(SCREEN_ENTRANCE_SENSOR, BTN_ENTRANCE_SENSOR_INPUT2_SELECTED, OFF);

		SetButtonValue(SCREEN_ENTRANCE_SENSOR, BTNE_JOINT_PM_SELECTED, ON);
		SetTextValue(SCREEN_MAIN_1, TXT_ENTRANCE_PRESSURE, "--");

		SetTextValue(SCREEN_MANUAL_CTR_2, TXT_MANUAL_ENTRANCE_PRESSURE, "--"); 		  //20200319

	}
} 

void controls_update_SCREEN_ENTRANCE_SENSOR0()
{
	char buff[5];
	uint8 i, wl_unit;
	uint16 value;
	
	for(i = 4; i <= 8; i++)
	{
		 if(i == 5)
	     {
			sprintf(buff, "%d", WL_Bias);
	     }
	     else if(i == 4)
	     {
		 	value = Entrance_sensor0[2] * 256 + Entrance_sensor0[3];
			sprintf(buff, "%d", value);
		 }
	     else 
	     {
		 	value = Entrance_sensor0[i * 2 - 6] * 256 + Entrance_sensor0[i * 2 - 5];
			sprintf(buff, "%d", value);
		 }
		 SetTextValue(SCREEN_ENTRANCE_SENSOR0, i, (uchar*)buff);
	 
	}

	SetButtonValue(SCREEN_ENTRANCE_SENSOR0, 1, (Entrance_sensor0[0] & 0x0f));
	
	wl_unit = Entrance_sensor0[0] >> 4;	 
	if(wl_unit > 2)	wl_unit = 0;

	if(wl_unit == 0) 
	{
		SetButtonValue(SCREEN_ENTRANCE_SENSOR0, 10, ON);					 
        SetButtonValue(SCREEN_ENTRANCE_SENSOR0, 11, OFF); 
        SetButtonValue(SCREEN_ENTRANCE_SENSOR0, 12, OFF); 

		SetTextValue(SCREEN_MAIN_1, 10, "(mm)");  



    }
	else if(wl_unit == 1) 
	{
		SetButtonValue(SCREEN_ENTRANCE_SENSOR0, 10, OFF);					 
        SetButtonValue(SCREEN_ENTRANCE_SENSOR0, 11, ON); 
        SetButtonValue(SCREEN_ENTRANCE_SENSOR0, 12, OFF); 

		SetTextValue(SCREEN_MAIN_1, 10, "(cm)");  
    }
	else if(wl_unit == 2) 
	{
		SetButtonValue(SCREEN_ENTRANCE_SENSOR0, 10, OFF);					 
        SetButtonValue(SCREEN_ENTRANCE_SENSOR0, 11, OFF); 
        SetButtonValue(SCREEN_ENTRANCE_SENSOR0, 12, ON); 

		SetTextValue(SCREEN_MAIN_1, 10, "(m)");  
    }

	if((Entrance_sensor0[0] & 0x0f) == 1)  //floating ball
	{
		SetButtonValue(SCREEN_ENTRANCE_SENSOR0, 1, 1);	 


		SetButtonValue(SCREEN_ENTRANCE_SENSOR0, 2, 0);	 

		SetButtonValue(SCREEN_ENTRANCE_SENSOR0, 3, 0);	 

		SetTextValue(SCREEN_MAIN_1, TXT_WATER_LEVEL, "--");

   	    SetTextValue(SCREEN_MANUAL_CTR_2, TXT_TANK_WATER_LEVEL, "--"); //floating ball ON, no need to display water level

		SetProgressValue(SCREEN_MAIN_1, SCREEN_MAIN_WATER_TANK, WL_DEFAULT_PERCENTAGE); //half full

		/////////
		printf("SetProgressValue(SCREEN_MAIN_1, 55, 100 / 2);\n");

	}
	else
	{
		SetButtonValue(SCREEN_ENTRANCE_SENSOR0, 1, 0);	 


		if(Entrance_sensor0[1] == 0)  //range1
		{
			SetButtonValue(SCREEN_ENTRANCE_SENSOR0, 2, 1);	 

			SetButtonValue(SCREEN_ENTRANCE_SENSOR0, 3, 0);	 

	
		}
		else	//range2
		{
			SetButtonValue(SCREEN_ENTRANCE_SENSOR0, 2, 0);	 

			SetButtonValue(SCREEN_ENTRANCE_SENSOR0, 3, 1);	 

		}
	}

} 

void controls_update_SCREEN_ENTRANCE_SENSOR2()
{
	char buff[5];

	float f_value;

 	f_value = (float)((Entrance_sensor2[1] * 256 + Entrance_sensor2[2])/100.0);
	sprintf(buff, "%.2f", f_value);
	SetTextValue(SCREEN_ENTRANCE_SENSOR2, 3, (uchar*)buff);
		 

 	f_value = (float)(Entrance_sensor2[3] / 100.0);
	sprintf(buff, "%.2f", f_value);
	SetTextValue(SCREEN_ENTRANCE_SENSOR2, 4, (uchar*)buff);
		 

	if(Entrance_sensor2[0] == 0)  //range1
	{
		SetButtonValue(SCREEN_ENTRANCE_SENSOR2, 1, 1);	 

		SetButtonValue(SCREEN_ENTRANCE_SENSOR2, 2, 0);	 


	}
	else	//range2
	{
		SetButtonValue(SCREEN_ENTRANCE_SENSOR2, 1, 0);	 

		SetButtonValue(SCREEN_ENTRANCE_SENSOR2, 2, 1);	 

	}
} 

void controls_update_SCREEN_PUMP_SWITCH_CONDTION()
{
	char buff[5];
	uint8 i;
	uint16 value;
	float f_value;	
			
	for(i = 1; i <= 10; i++)
	{
		 if((i == 1) || (i == 2) || (i == 4) || (i == 7) || (i == 8) || (i == 9))
	     {
		 	value = Pump_Switch_Condtion[VarIndex_SCREEN_PUMP_SWITCH_CONDTION[i]] * 256 + Pump_Switch_Condtion[VarIndex_SCREEN_PUMP_SWITCH_CONDTION[i] + 1];
			sprintf(buff, "%d", value);
	     }
	     else if((i == 3) || (i == 5) || (i == 6))
	     {
		 	value = Pump_Switch_Condtion[VarIndex_SCREEN_PUMP_SWITCH_CONDTION[i]];
			sprintf(buff, "%d", value);
		 }
	     else if(i == 10)
	     {
		 	f_value = (float)(Pump_Switch_Condtion[15] / 100.0);
			sprintf(buff, "%.2f", f_value);
		 }
		 SetTextValue(SCREEN_PUMP_SWITCH_CONDTION, i, (uchar*)buff);
	 
	}

	//
	sprintf(buff, "%d", PumpCancelFreq);
	SetTextValue(SCREEN_PUMP_SWITCH_CONDTION, TXT_PUMP_CANCEL_FREQ, (uchar*)buff);
	
	sprintf(buff, "%.2f", Entrance_Sensor_Range);
	SetTextValue(SCREEN_ENTRANCE_SENSOR, TXT_ENTRANCE_SENSOR_RANGE, (uchar*)buff);


	sprintf(buff, "%.2f", Entrance_Pressure_Bias);
	SetTextValue(SCREEN_ENTRANCE_SENSOR,  TXT_ENTRANCE_PRESSURE_BIAS, (uchar*)buff);


	sprintf(buff, "%.2f", Entrance_LP_Protection_Value);
	SetTextValue(SCREEN_ENTRANCE_SENSOR, TXT_ENTRANCE_LP_PROTECTION_VALUE, (uchar*)buff);


	sprintf(buff, "%d", Entrance_LP_Protection_Delay);
	SetTextValue(SCREEN_ENTRANCE_SENSOR, TXT_ENTRANCE_LP_PROTECTION_DELAY, (uchar*)buff);


	sprintf(buff, "%.2f", Entrance_LP_Protection_Restore_Value);
	SetTextValue(SCREEN_ENTRANCE_SENSOR, TXT_ENTRANCE_LP_PROTECT_RESTORE_VAL, (uchar*)buff);

	//20210911
	sprintf(buff, "%.1f", PumpMaxFreq);
	SetTextValue(SCREEN_PUMP_SWITCH_CONDTION, TXT_PUMP_MAX_FREQ, (uchar*)buff);

	if(Entrance_Sensor_type == ENTRANCE_SENSOR_TYPE_1)  
	{	
		SetButtonValue(SCREEN_ENTRANCE_SENSOR, BTN_ENTRANCE_SENSOR_INPUT1_SELECTED, ON);

		SetButtonValue(SCREEN_ENTRANCE_SENSOR, BTN_ENTRANCE_SENSOR_INPUT2_SELECTED, OFF);

		SetButtonValue(SCREEN_ENTRANCE_SENSOR, BTNE_JOINT_PM_SELECTED, OFF);
	}
	if(Entrance_Sensor_type == ENTRANCE_SENSOR_TYPE_2)  
	{	
		SetButtonValue(SCREEN_ENTRANCE_SENSOR, BTN_ENTRANCE_SENSOR_INPUT1_SELECTED, OFF);

		SetButtonValue(SCREEN_ENTRANCE_SENSOR, BTN_ENTRANCE_SENSOR_INPUT2_SELECTED, ON);

		SetButtonValue(SCREEN_ENTRANCE_SENSOR, BTNE_JOINT_PM_SELECTED, OFF);
	}
	if(Entrance_Sensor_type == ENTRANCE_SENSOR_TYPE_3)  
	{	
		SetButtonValue(SCREEN_ENTRANCE_SENSOR, BTN_ENTRANCE_SENSOR_INPUT1_SELECTED, OFF);

		SetButtonValue(SCREEN_ENTRANCE_SENSOR, BTN_ENTRANCE_SENSOR_INPUT2_SELECTED, OFF);

		SetButtonValue(SCREEN_ENTRANCE_SENSOR, BTNE_JOINT_PM_SELECTED, ON);
	}


} 


void controls_update_SCREEN_SLEEP_SETTING()
{
	char buff[5];

	uint8 i;
	uint16 value;
	float f_value;	
			
	for(i = 1; i <= 7; i++)
	{
		 if((i == 1) || (i == 4))
	     {
		 	value = Sleep_Setting[VarIndex_SCREEN_SLEEP_SETTING[i]] * 256 + Sleep_Setting[VarIndex_SCREEN_SLEEP_SETTING[i] + 1];
			sprintf(buff, "%d", value);
	     }
	     else if((i == 3) || (i == 6))
	     {
		 	f_value = (float)(Sleep_Setting[VarIndex_SCREEN_SLEEP_SETTING[i]] / 100.0);
			sprintf(buff, "%.2f", f_value);
		 }
	     else
	     {

		 	value = Sleep_Setting[VarIndex_SCREEN_SLEEP_SETTING[i]];
			sprintf(buff, "%d", value);
		 }
		 SetTextValue(SCREEN_SLEEP_SETTING, i, (uchar*)buff);
	 
	}

	switch(SleepingEnableMode)
	{
		case 0:
			SetButtonValue(SCREEN_SLEEP_SETTING, 9, ON);

			SetButtonValue(SCREEN_SLEEP_SETTING, 10, OFF);

			SetButtonValue(SCREEN_SLEEP_SETTING, 11, OFF);

			break;
		case 1:
			SetButtonValue(SCREEN_SLEEP_SETTING, 9, OFF);

			SetButtonValue(SCREEN_SLEEP_SETTING, 10, ON);

			SetButtonValue(SCREEN_SLEEP_SETTING, 11, OFF);

			break;
		case 2:
			SetButtonValue(SCREEN_SLEEP_SETTING, 9, OFF);

			SetButtonValue(SCREEN_SLEEP_SETTING, 10, OFF);

			SetButtonValue(SCREEN_SLEEP_SETTING, 11, ON);

			break;
		default:
			break;	
	}
} 

void controls_update_SCREEN_POWER_UP_SETTING()
{
	char buff[5];
	uint8 i, i_value;
	uint16 i16_value;
			
	for(i = 1; i <= 7; i++)		 //0-6
	{
	 	i_value = Power_Up_Setting[i - 1];
		SetButtonValue(SCREEN_POWER_UP_SETTING, i, i_value);	 
	}
	SetButtonValue(SCREEN_POWER_UP_SETTING, 8,  (Power_Up_Setting[7] & 0x04)>>2); //pump4	 
	SetButtonValue(SCREEN_POWER_UP_SETTING, 9,  (Power_Up_Setting[7] & 0x02)>>1); //pump5
	SetButtonValue(SCREEN_POWER_UP_SETTING, 10, (Power_Up_Setting[7] & 0x01));    //pump6

	SetButtonValue(SCREEN_POWER_UP_SETTING, BTN_AVF_MODE, AvfMode);	

	//Show/Hide Fixed-FC mode selection
	SetControlVisiable(SCREEN_PUMP_GROUPING, ICON_BLUE_MASK, AvfMode);
	SetControlEnable(SCREEN_PUMP_GROUPING, BTN_FIXED_FC_PUMP_MODE, 1 - AvfMode);	

	//USE_60HZ_AS_MAX_FREQUENCY
	SetButtonValue(SCREEN_POWER_UP_SETTING, BTN_USE_60HZ_AS_MAX_FREQUENCY, Use60HzAsMaxFreq);

	//FcFreqOutBias
	sprintf(buff, "%.1f", FcFreqOutBias);
	SetTextValue(SCREEN_POWER_UP_SETTING, 10, (uchar*)buff);

	//PWR_UP_AUTORUN_DELAY_
 	i16_value = Power_Up_Setting[8] * 256 + Power_Up_Setting[11];;
	sprintf(buff, "%d", i16_value);
	SetTextValue(SCREEN_POWER_UP_SETTING, 11, (uchar*)buff);

} 

void controls_update_SCREEN_VALVE_CONTROL()
{
	char buff[5];
	uint8 i, i_value;
	uint16 value;
	float f_value;	
			
	for(i = 1; i <= 6; i++)
	{
		 if((i == 3) || (i == 4))
	     {
		 	value = Valve_Control[i * 2 - 4] * 256 + Valve_Control[i * 2 - 3];
			sprintf(buff, "%d", value);
			SetTextValue(SCREEN_VALVE_CONTROL, i, (uchar*)buff);
	     }
		 else if((i == 5) || (i == 6))
	     {
		 	f_value	= (float)(Valve_Control[i + 1] / 100.0);
			sprintf(buff, "%.2f", f_value);
			SetTextValue(SCREEN_VALVE_CONTROL, i, (uchar*)buff);
	     }
	     else
	     {
			i_value = Valve_Control[i - 1];
			SetButtonValue(SCREEN_VALVE_CONTROL, i, i_value);	 
		 }
	}
} 

void controls_update_SCREEN_VALVE_CONTROL1()
{
	char buff[5];
	uint8 i;
	uint16 value;
	float f_value;	
			
	for(i = 1; i <= 5; i++)
	{
		 if((i == 1) || (i == 5))
	     {
		 	f_value	= (float)(Valve_Control1[i - 1] / 100.0);
			sprintf(buff, "%.2f", f_value);
			SetTextValue(SCREEN_VALVE_CONTROL1, i, (uchar*)buff);
	     }
		 else
	     {
		 	value = Valve_Control1[i - 1];
			sprintf(buff, "%d", value);
			SetTextValue(SCREEN_VALVE_CONTROL1, i, (uchar*)buff);
	     }
	 
	}
} 

void controls_update_SCREEN_PID_SETTING()
{
	char buff[5];
	uint16* pid_arr[3] = {&pFactor, &iFactor, &dFactor};
	uint8 i;
	uint16 value;
	float f_value;
			
	for(i = 1; i <= 5; i++)
	{
		 if(i == 1)
	     {
			SetButtonValue(SCREEN_PID_SETTING, i, PID_Setting[0]);	 
	     }
		 else if(i == 2)
		 {
		 	f_value = (float)(PID_Setting[1]/10.0);
			sprintf(buff, "%0.1f", f_value);
			SetTextValue(SCREEN_PID_SETTING, i, (uchar*)buff);
		 }	   
		 else
		 {
		 	value = *(pid_arr[i - 3]);
			sprintf(buff, "%d", value);
			SetTextValue(SCREEN_PID_SETTING, i, (uchar*)buff);

		 }
	
	 
	}
	value = TargetLockerTiming;
	sprintf(buff, "%d", value);
	SetTextValue(SCREEN_PID_SETTING, TXT_TARGET_LOCKER_TIMING, (uchar*)buff);

	sprintf(buff, "%.1f", SpeedUpFactor);		   //20210712
	SetTextValue(SCREEN_PID_SETTING, TXT_SPEED_UP_FACTOR, (uchar*)buff);

} 
/**********************************************************************************************
 * Name：controls_update(uint16 screen_id)
 * Brief：Update hmi controls with variables 
 *  
 * Para：  screen_id
 *        
 * Return ：null
 * 
 * Caller(s):  UseSettingsInFlash()
 **********************************************************************************************/
void controls_update(uint16 screen_id)
{
	printf("==============================start:controls_update\n");
	switch(screen_id)
	{																	
		case SCREEN_PUMP_GROUPING:
 			controls_update_SCREEN_PUMP_GROUPING();
			break;
			
 		case SCREEN_OUTLET_SENSOR:
 			controls_update_SCREEN_OUTLET_SENSOR();
			break;
 		case SCREEN_ENTRANCE_SENSOR:
 			controls_update_SCREEN_ENTRANCE_SENSOR();
			break;
		case SCREEN_ENTRANCE_SENSOR0:
 			controls_update_SCREEN_ENTRANCE_SENSOR0();
			break;
 		case SCREEN_ENTRANCE_SENSOR2:
 			controls_update_SCREEN_ENTRANCE_SENSOR2();
			break;
 
 		case SCREEN_PUMP_SWITCH_CONDTION:
 			controls_update_SCREEN_PUMP_SWITCH_CONDTION();
			break;
		case SCREEN_SLEEP_SETTING:
 			controls_update_SCREEN_SLEEP_SETTING();
			break;
		case SCREEN_POWER_UP_SETTING:
 			controls_update_SCREEN_POWER_UP_SETTING();
			break;
		
		case SCREEN_VALVE_CONTROL:
 			controls_update_SCREEN_VALVE_CONTROL();
			break;
		case SCREEN_VALVE_CONTROL1:
 			controls_update_SCREEN_VALVE_CONTROL1();
			break;
 
		case SCREEN_PID_SETTING:
 			controls_update_SCREEN_PID_SETTING();
			break;

		case SCREEN_SCREEN_SETTING:
 			controls_update_SCREEN_SCREEN_SETTING();
			break;

		default:
			break;
	}	
			
}

 
#ifdef DBG_FLASH_IMAGE
void copy_data(uint8* a, uint8* b, uint8 j, uint8 len)
{
   uint8 i;
   for(i = 0; i < 10; i++)
   {
   		a[i] = b[j + i];
   }
}

void add_flash_data_to_table(uint8 *data, uint16 length)
{
   uint8 j, tail_len;
   uint8 buff[10];
   tail_len = length;
   j = 0;

   while(tail_len > 10)
   {
	   copy_data(buff, data, j, 10);
	   delay_ms(100);
	   Record_Add_Byte(37, 1, buff, 10);
	   	   
	   j += 10;
	   tail_len -= 10;
	}
	if(tail_len > 0)
	{
	   copy_data(buff, data, j, tail_len);
	   Record_Add_Byte(37, 1, buff, tail_len);

	}
	IWDG_FEED
}
#endif	  

  
/**********************************************************************************************
 * Name：UseSettingsInFlash(uint8 *_data,uint16 length)
 * Brief：Update variables and hmi controls with data from flash
 *  
 * Para：  *_data: an array of bytes from flash 
 *         length: number of byte	
 * Return ：null
 * 
 * Caller(s):  main.c/ NotifyReadFlash() 
 **********************************************************************************************/

void UseSettingsInFlash(uint8 *_data,uint16 length)
{
	uint8 i, ctr_id, row, column;
	uint16 screen_id;
	char buff[5];
	
//#define DBG_USESETTINGSINFLASH
//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_USESETTINGSINFLASH
    printf("\n\n\n Entrance: void UseSettingsInFlash(uint8 *_data,uint16 length)...............\n\n");
#endif  
//--------------------------------------------------------------------------------------------------
#ifdef DBG_USESETTINGSINFLASH	
	printf("Received --------------\n");
	display(_data, 8);	
#endif

#ifdef DBG_FLASH_IMAGE
	add_flash_data_to_table(_data, length);
#endif			 
	
   	screen_id = _data[0]; 

#ifdef  DBG_USESETTINGSINFLASH
    printf("screen_id = %d\n",screen_id);
#endif

	if(screen_id == SCREEN_TIME_CTR_3)
	{
#ifdef DBG_USESETTINGSINFLASH
		printf("SCREEN_TIME_CTR_3 FLASH DATA:");
		for(i = 1; i < length; i++)
		{
			printf("_data[%d] = %d\n",  i, _data[i]);		 
		} 
		printf("----END OF SCREEN_TIME_CTR_3 FLASH DATA\n");
#endif
		UnCompressData(_data[1], working_weekdays);	//load working_weekdays

#ifdef  DBG_USESETTINGSINFLASH
	    printf("working days\n");
		display(working_weekdays, 8);
#endif

	    BatchBegin(SCREEN_TIME_CTR_3);
		for(i = 1; i < 32; i++)
		{
			row = (i - 2)/5;
			column = (i - 2)%5;

			ctr_id = control_index_tbl[i];

			if(IS_TIME_SEG_EN_DIS)  	 //Button time_control_enabled
			{
				//BatchSetButtonValue(ctr_id, time_control_enabled);
			}
			else if(IS_TIME_SEG_SELECTOR) 	 //Button time segment selector
			{
				BatchSetButtonValue(ctr_id, _data[i]);	
			}
			else  // Hour/minute
			{
				if(column % 2 == 0) 	 //always 2 digits for minute
				{
					sprintf(buff, "%02d", _data[i]);
				}
				else
				{			
					sprintf(buff, "%d",  _data[i]);   
				}
				BatchSetText(ctr_id, (uchar*)buff);
			}


			UpdatScheduleCell(_data[i], row, column);
						
		}

		//DefaultTargetPressure
		DefaultTargetPressure = (_data[32] * 256 + _data[33]) / 100.0;
	 
		sprintf(buff, "%.2f", DefaultTargetPressure);
		BatchSetText(TXT_TC_DEFAULT_PRESSURE, (uchar*)buff);

		for(i = 0; i < 6; i++)				 //target pressure
		{
			tc_target_pressure[i] = (_data[34 + i * 2] * 256 + _data[35 + i * 2]) / 100.0;

			ctr_id = 48 + i;		
			sprintf(buff, "%.2f", tc_target_pressure[i]);
			BatchSetText(ctr_id, (uchar*)buff);


		}
		for(i = 0; i < 7; i++)				 //working_weekdays[]/time_control_enabled
		{
			ctr_id = i + 3;
		
			BatchSetButtonValue(ctr_id, working_weekdays[i]);

		}

#ifdef DBG_USESETTINGSINFLASH
		load_schedule();
#endif
		SettingsLoaded = TRUE;


		PowerUpAutoRunTimer	= Power_Up_Setting[11] + Power_Up_Setting[8] * 256;		

	}
	else
	{
		variables_update(_data, screen_id);

#ifdef   DBG_USESETTINGSINFLASH
	    printf( "Flash Data--screen: %d  \nlength: %d\n", _data[0], length - 1);
#endif

     	controls_update(screen_id);
	}
 
	
	flash_data_loaded = TRUE;	

#ifdef DBG_USESETTINGSINFLASH
    printf("\n\n\n Exit: void UseSettingsInFlash(uint8 *_data,uint16 length)...............\n\n");
#endif  

} 



void SaveSettingsToFlash(uint16 screen_id)
{
	uint8 start_addr, len;
  	
	if(screen_id ==  SCREEN_PUMP_GROUPING)
    {
		uint8 temp[8]; 
		
		temp[0] = WaterSourceMode;
		temp[1] = Max_Pump_Nbr;
		temp[2] = 0;				  //unused
		temp[3] = Fixed_FC_Pump;
		temp[4] = pump_enable_tbl[0];
		temp[5] = pump_enable_tbl[1];
		temp[6] = pump_enable_tbl[2];
	//	temp[7] = pump_enable_tbl[3];	//to be replaced with following
		temp[7] = pump_enable_tbl[3] | (pump_enable_tbl[4] << 1) | (pump_enable_tbl[5] << 2);

		printf("temp[7] = %d\n",temp[7]);
	
		STM32_FLASH_Write(STM32_FLASH_SAVE_ADDR_USER_SETTINGS + STM32_FLASH_SAVE_ADDR_USER_SCREEN_PUMP_GROUPING_INTERNAL,(u16*)temp, 4);


    }
	else if(screen_id == SCREEN_OUTLET_SENSOR)
    {
		uint8 temp[10]; 
		uint8  Outlet_Sensor_Range_H, Outlet_Sensor_Range_L, Outlet_HP_Protection_Value_H, Outlet_HP_Protection_Value_L;

		Outlet_Sensor_Range_H = (int)(Outlet_Sensor_Range * 100) / 256;
		Outlet_Sensor_Range_L = (int)(Outlet_Sensor_Range * 100) % 256;

		Outlet_HP_Protection_Value_H = (int)(Outlet_HP_Protection_Value * 100) / 256;
		Outlet_HP_Protection_Value_L = (int)(Outlet_HP_Protection_Value  * 100)% 256;;
				
		temp[0] = Outlet_Sensor_type;
		temp[1] = Outlet_Sensor_Range_H;
		temp[2] = Outlet_Sensor_Range_L;

	 	if(Outlet_Pressure_Bias < 0)
		{
			temp[3] = (int)(-Outlet_Pressure_Bias * 100) / 256 + 0x80;
			temp[4] = (int)(-Outlet_Pressure_Bias * 100) % 256;
		}
		else
		{
			temp[3] = (int)(Outlet_Pressure_Bias * 100) / 256;
			temp[4] = (int)(Outlet_Pressure_Bias * 100) % 256;
		}

		temp[5] = Outlet_LP_Protection_Selected;
		temp[6] = (int)(Outlet_LP_Protection_Value * 100);
		temp[7] = Outlet_HP_Protection_Selected;
		temp[8] = Outlet_HP_Protection_Value_H;
		temp[9] = Outlet_HP_Protection_Value_L;

	
		STM32_FLASH_Write(STM32_FLASH_SAVE_ADDR_USER_SETTINGS + STM32_FLASH_SAVE_ADDR_USER_SCREEN_OUTLET_SENSOR_INTERNAL,\
																 (u16*)temp, 5);
		temp[0] = (int)(OutletPressureCaliCoeff * 100);
		STM32_FLASH_Write(STM32_FLASH_SAVE_OUTLET_PRESSURE_CALI_COEFF,(u16*)temp, 1);
		
    }
	else if(screen_id == SCREEN_ENTRANCE_SENSOR)
    {
		uint8 temp[9]; 
		uint8 Entrance_Sensor_Range_H, Entrance_Sensor_Range_L, Entrance_LP_Protection_Delay_H, Entrance_LP_Protection_Delay_L;

		Entrance_Sensor_Range_H = (int)(Entrance_Sensor_Range * 100) / 256;
		Entrance_Sensor_Range_L = (int)(Entrance_Sensor_Range * 100) % 256;

		Entrance_LP_Protection_Delay_H = Entrance_LP_Protection_Delay / 256;
		Entrance_LP_Protection_Delay_L = Entrance_LP_Protection_Delay % 256;;
				
		temp[0] = Entrance_Sensor_type;
		temp[1] = Entrance_Sensor_Range_H;
		temp[2] = Entrance_Sensor_Range_L;

		if(Entrance_Pressure_Bias < 0)
		{
			temp[3] = (int)(-Entrance_Pressure_Bias * 100) / 256 + 0x80;
			temp[4] = (int)(-Entrance_Pressure_Bias * 100) % 256;
		}
		else
		{
			temp[3] = (int)(Entrance_Pressure_Bias * 100) / 256;
			temp[4] = (int)(Entrance_Pressure_Bias * 100) % 256;
		}

		temp[5] = (int)(Entrance_LP_Protection_Value * 100);
		temp[6] = Entrance_LP_Protection_Delay_H;
		temp[7] = Entrance_LP_Protection_Delay_L;
		temp[8] = (int)(Entrance_LP_Protection_Restore_Value * 100);
	
		STM32_FLASH_Write(STM32_FLASH_SAVE_ADDR_USER_SETTINGS + STM32_FLASH_SAVE_ADDR_USER_SCREEN_ENTRANCE_SENSOR_INTERNAL,\
												 (u16*)temp, 5);
		//save coeff
		temp[0] = (int)(EntrancePressureCaliCoeff * 100);		
		STM32_FLASH_Write(STM32_FLASH_SAVE_ENTRANCE_PRESSURE_CALI_COEFF, (u16*)temp, 1);

    }
	else if(screen_id == SCREEN_ENTRANCE_SENSOR0)
    {
		uint8 temp[12]; 
		uint8 i;
		if(WL_Bias < 0)
		{
			Entrance_sensor0[4] = (int)(-WL_Bias) / 256 + 0x80;
			Entrance_sensor0[5] = (int)(-WL_Bias) % 256;
		}
		else
		{
			Entrance_sensor0[4] = (int)(WL_Bias) / 256;
			Entrance_sensor0[5] = (int)(WL_Bias) % 256;
		}
	
		for(i = 0; i < 12; i++)
		{
		   	temp[i] = Entrance_sensor0[i];

		}

		STM32_FLASH_Write(STM32_FLASH_SAVE_ADDR_USER_SETTINGS + STM32_FLASH_SAVE_ADDR_USER_SCREEN_ENTRANCE_SENSOR0_INTERNAL,\
												 (u16*)temp, 6);
    }
	else if(screen_id == SCREEN_ENTRANCE_SENSOR2)
    {
		uint8 temp[5]; 
		uint8 i;
				
		temp[0] = SCREEN_ENTRANCE_SENSOR2;
		for(i = 1; i <= 4; i++)
		{
		   	temp[i] = Entrance_sensor2[i - 1];

		}

		start_addr = flash_settings[4][0];
		len =  flash_settings[4][1];
		WriteUserFlash(start_addr, len, temp);
	
    }
	else if(screen_id == SCREEN_PUMP_SWITCH_CONDTION)
    {
		uint8 temp[16]; 
		uint8 i;
				
		for(i = 0; i < 16; i++)
		{
		   	temp[i] = Pump_Switch_Condtion[i];
		}
		STM32_FLASH_Write(STM32_FLASH_SAVE_ADDR_USER_SETTINGS + STM32_FLASH_SAVE_ADDR_USER_SCREEN_PUMP_SWITCH_CONDTION_INTERNAL,\
												 (u16*)temp, 8);
	
	    temp[0] = PumpCancelFreq;
		temp[1] = 0;
		STM32_FLASH_Write(STM32_FLASH_SAVE_PUMP_CANCEL_FREQ, (u16*)temp, 1);

		//20210911
	    /* Save Pump Max Freq*/
	    temp[0] = ((int)(PumpMaxFreq * 10))>>8;
		temp[1] = ((int)(PumpMaxFreq * 10)) & 0xff;

		printf("Save Pump Max Freq: %.1f\n", PumpMaxFreq);
		printf("temp[0] = %d\n",temp[0]);
		printf("temp[1] = %d\n",temp[1]);


		STM32_FLASH_Write(STM32_FLASH_SAVE_PUMP_MAX_FREQ, (u16*)temp, 1);
	
    }
	else if(screen_id == SCREEN_SLEEP_SETTING)
    {
		uint8 temp[10]; 
		uint8 i;
				
		for(i = 0; i < 10; i++)
		{
		   	temp[i] = Sleep_Setting[i];

		}
		temp[0] += SleepingEnableMode * 16;
		STM32_FLASH_Write(STM32_FLASH_SAVE_ADDR_USER_SETTINGS + STM32_FLASH_SAVE_ADDR_USER_SCREEN_SLEEP_SETTING_INTERNAL,\
												 (u16*)temp, 5);

	
    }
	else if(screen_id == SCREEN_POWER_UP_SETTING)
    {
		uint8 temp[12]; 
		uint8 i;
		if(FcFreqOutBias < 0)
		{
			Power_Up_Setting[9] = (int)(-FcFreqOutBias * 10) / 256 + 0x80;
			Power_Up_Setting[10] = (int)(-FcFreqOutBias * 10) % 256;
		}
		else
		{
			Power_Up_Setting[9] = (int)(FcFreqOutBias * 10) / 256;
			Power_Up_Setting[10] = (int)(FcFreqOutBias * 10) % 256;
		}
	
		for(i = 0; i < 12; i++)
		{
		   	temp[i] = Power_Up_Setting[i];

		}
		STM32_FLASH_Write(STM32_FLASH_SAVE_ADDR_USER_SETTINGS + STM32_FLASH_SAVE_ADDR_USER_SCREEN_POWER_UP_SETTING_INTERNAL,\
												 (u16*)temp, 6);

		temp[0] = AvfMode;
		temp[1] = 0;
		STM32_FLASH_Write(STM32_FLASH_SAVE_AVF_MODE, (u16*)temp, 1);

		SaveUse60Hz(Use60HzAsMaxFreq);


    }
	else if(screen_id == SCREEN_VALVE_CONTROL)
    {
		uint8 temp[8]; 
		uint8 i;
				
		for(i = 0; i < 8; i++)
		{
		   	temp[i] = Valve_Control[i];
		}

		STM32_FLASH_Write(STM32_FLASH_SAVE_ADDR_USER_SETTINGS + STM32_FLASH_SAVE_ADDR_USER_SCREEN_VALVE_CONTROL_INTERNAL,\
												 (u16*)temp, 4);
    }
	else if(screen_id == SCREEN_VALVE_CONTROL1)
    {
		uint8 temp[6]; 
		uint8 i;
				
		for(i = 0; i < 6; i++)
		{
		   	temp[i] = Valve_Control1[i];
		}

		STM32_FLASH_Write(STM32_FLASH_SAVE_ADDR_USER_SETTINGS + STM32_FLASH_SAVE_ADDR_USER_SCREEN_VALVE_CONTROL1_INTERNAL,\
												 (u16*)temp, 3);

    }
	else if(screen_id == SCREEN_PID_SETTING)
    {
		uint8 temp[8]; 
				
		temp[0] = PID_Setting[0];
		temp[1] = PID_Setting[1];
		temp[2] = pFactor / 256;
		temp[3] = pFactor % 256;
		temp[4] = iFactor / 256;
		temp[5] = iFactor % 256;
		temp[6] = dFactor / 256;
		temp[7] = dFactor % 256;
		STM32_FLASH_Write(STM32_FLASH_SAVE_ADDR_USER_SETTINGS + STM32_FLASH_SAVE_ADDR_USER_SCREEN_PID_SETTING_INTERNAL,\
												 (u16*)temp, 4);

		save_speed_up_factor();	  //20210712
    }
	else if(screen_id == SCREEN_SCREEN_SETTING)
    { 		
		SavePowerSavingData();	 
	} 
#ifdef DBG_EVENT_AS_ERROR
			EventRecorder("数据已存入flash");
#endif
}



void SavePowerSavingData(void)	  
{
	uint8 write_buff[2];
	write_buff[0] = BL_Control_Enable;
	write_buff[1] = BL_On_Time;

	STM32_FLASH_Write(STM32_FLASH_SAVE_ADDR_POWER_SAVING , (u16*)write_buff, 1);
	
}


#ifdef USE_PIN_GENERATOR   //PW_ErrIndex

void SavePW_ErrIndex(uint8 ind)
{
	uint8 write_buff[2];
	write_buff[0] = ind;
	write_buff[1] = 0;
	STM32_FLASH_Write(STM32_FLASH_SAVE_ADDR_PW_ERR_IND, (u16*)write_buff,1);
} 
#endif

void SaveCompanyName(uint8* name)
{
	uint8 i, j;
	char write_buff[42]; 

	i = strlen((char*)name);
	

    strcpy(write_buff, (char*)name);
	if((i < 42) && (i > 1))
	{
		for(j = i + 1; j <= 42; j++)
		{
		 	strcat(write_buff, "#");	
		}
	}
	printf("%s\n", write_buff);	

	STM32_FLASH_Write(STM32_FLASH_SAVE_COMPANY_NAME, (u16*)write_buff,21);


}


void SavePhoneNbr(uint8* phone)
{
	uint8 i, j;
	char write_buff[14]; 

	i = strlen((char*)phone);
	printf("SavePhoneNbr(): i = %d\n", i);

    strcpy(write_buff, (char*)phone);
	if(i < 14)
	{
		for(j = i + 1; j <= 14; j++)
		{
		 	strcat(write_buff, "#");	
		}
	}
	printf("%s\n", write_buff);	

	STM32_FLASH_Write(STM32_FLASH_SAVE_ADDR_USER_SETTINGS + STM32_FLASH_SAVE_ADDR_USER_SCREEN_PHONE_NBR_INTERNAL,\
														(u16*)write_buff,7);


}

uint8 is_leap_year(uint16 year) 
{
	return (year % 400 == 0) || ((year % 4 == 0) && (year % 100 != 0));
	
}

uint8 is_big_month(uint8 mth)
{
	return (mth % 2 + mth/8) % 2;
	
}

/************************************************************
 * Check the validity of the input date, do correction if  
 * it is out of range and return TRUE. otherwise return FALSE.
 * eg. 2020-2-30->2020-3-1    
 *     2020-4-31->2020-5-1
 ************************************************************/
uint8 DateCorrection(MY_DATE* date)
{
	if(date->month == 2)
	{
		if(is_leap_year(date->year)) 
		{
			if(date->day > 29) 
			{
				date->month = 3;
				date->day -= 29;
				return TRUE;							
			}
		}
		else
		{
			if(date->day > 28)
			{
				date->month = 3;
				date->day -= 28;
				return TRUE;				
			} 
		}
		
	}
	else
	{
		if(is_big_month(date->month) == FALSE)
		{
			if(date->day > 30)
			{
				date->month += 1;				
				date->day = 1;	
				return TRUE;							
			} 
		}
		
	}
	return FALSE;
	
}

void SaveExpireDate(uint16 year, uint8 month, uint8 day, uint8 disable_manual, uint8 enable_expire_date)
{
	uint8 expire_date[6];

	expire_date[0] = year / 100;
	expire_date[1] = year % 100;
	expire_date[2] = month;
	expire_date[3] = day;
	expire_date[4] = disable_manual;
	expire_date[5] = enable_expire_date;	

	STM32_FLASH_Write(STM32_FLASH_SAVE_ADDR_USER_SETTINGS + STM32_FLASH_SAVE_ADDR_USER_SCREEN_EXPIRE_DATE_INTERNAL,\
													(u16*)expire_date,3);

} 

#define INVALID_YEAR  	(date[0] > 21) || (date[0] < 20) || (date[1] > 99)
#define INVALID_MONTH  	(date[2] > 12) || (date[2] == 0)
#define INVALID_DATE  	(date[3] > 31) || (date[3] == 0)
#define INVALID_FLAGS  	(date[4] > 1)  || (date[5] > 1)

void date_init_check(uint8* date)
{
//#define DBG_DATE_INIT_CHECK
//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_DATE_INIT_CHECK

    printf("\n\n\n Entrance: void date_init_check(uint8* date)...............\n\n");
#endif  
//--------------------------------------------------------------------------------------------------
#ifdef DBG_DATE_INIT_CHECK
	printf("%2d%2d-%2d-%2d flag: [%d|%d]  \n", date[0], date[1], date[2], date[3], date[4], date[5]);
#endif

	if(INVALID_YEAR || INVALID_MONTH || INVALID_DATE || INVALID_FLAGS)
	{
		date[0] = 20;
		date[1] = 50;
		date[2] = 12;
		date[3] = 31;
		date[4] = 0;
		date[5] = 0;
#ifdef DBG_DATE_INIT_CHECK
		printf("Invalid Date! Reset to default value\n");
#endif
	}
#ifdef DBG_DATE_INIT_CHECK
	else
	{
	    printf("Pass checking\n");
	}
#endif

#ifdef DBG_DATE_INIT_CHECK
    printf("\n\n\n Exit: void date_init_check(uint8* date)...............\n\n");
#endif  


}
 

void UseDataTC(uint8* _data)
{
	 char buff[5];
	 uint8 read_buf[2];
	 uint8 i, ctr_id, row, column;
	{
		UnCompressData(_data[STM32_FLASH_SAVE_ADDR_USER_TC_INTERNAL + 0], working_weekdays);	//load working_weekdays

	    BatchBegin(SCREEN_TIME_CTR_3);
		for(i = 1; i < 32; i++)
		{
			row = (i - 2)/5;
			column = (i - 2)%5;

			ctr_id = control_index_tbl[i];

			if(IS_TIME_SEG_EN_DIS)  	 //Button time_control_enabled
			{
				//BatchSetButtonValue(ctr_id, time_control_enabled);
			}
			else if(IS_TIME_SEG_SELECTOR) 	 //Button time segment selector
			{
				BatchSetButtonValue(ctr_id, _data[STM32_FLASH_SAVE_ADDR_USER_TC_INTERNAL + i - 1]);	
			}
			else  // Hour/minute
			{
				if(column % 2 == 0) 	 //always 2 digits for minute
				{
					sprintf(buff, "%02d", _data[STM32_FLASH_SAVE_ADDR_USER_TC_INTERNAL + i - 1]);
				}
				else
				{			
					sprintf(buff, "%d",  _data[STM32_FLASH_SAVE_ADDR_USER_TC_INTERNAL + i - 1]);   
				}
				BatchSetText(ctr_id, (uchar*)buff);
			}


			UpdatScheduleCell(_data[STM32_FLASH_SAVE_ADDR_USER_TC_INTERNAL + i - 1], row, column);
						
		}


		DefaultTargetPressureMenu = (_data[STM32_FLASH_SAVE_ADDR_USER_TC_INTERNAL + 31] * 256 +\
		                             _data[STM32_FLASH_SAVE_ADDR_USER_TC_INTERNAL + 32]) / 100.0;
		/*
			DefaultTargetPressure is initially set to menu value at this moment, 
			it might be overriden by IO setting in case UseIoTargetPressure is enabled after wram-up
		*/
 		DefaultTargetPressure = DefaultTargetPressureMenu; //20210703

		//Remote target	 read_buf[2];
		STM32_FLASH_Read(STM32_FLASH_SAVE_REMOTE_TARGET, (u16*)read_buf, 1);
		RemoteTargetPressure = (read_buf[0] * 256 + read_buf[1]) / 100.0;
		if(RemoteTargetPressure > 50) RemoteTargetPressure = 0.4;

	 
		sprintf(buff, "%.2f", DefaultTargetPressure);
		BatchSetText(TXT_TC_DEFAULT_PRESSURE, (uchar*)buff);

		for(i = 0; i < 6; i++)				 //target pressure
		{
			tc_target_pressure[i] = (_data[STM32_FLASH_SAVE_ADDR_USER_TC_INTERNAL + 33 + i * 2] * 256 +\
			 _data[STM32_FLASH_SAVE_ADDR_USER_TC_INTERNAL + 34 + i * 2]) / 100.0;

			ctr_id = 48 + i;		
			sprintf(buff, "%.2f", tc_target_pressure[i]);
			BatchSetText(ctr_id, (uchar*)buff);


		}
		for(i = 0; i < 7; i++)				 //working_weekdays[]/time_control_enabled
		{
			ctr_id = i + 3;
		
			BatchSetButtonValue(ctr_id, working_weekdays[i]);

		}

		BatchSetButtonValue(BTN_REMOTE_TARGET_ENABLE, RemoteTargetEnable);
		BatchSetButtonValue(BTN_USE_IO_TARGET_PRESSURE, UseIoTargetPressure);

		SettingsLoaded = TRUE;
		PowerUpAutoRunTimer	= Power_Up_Setting[11] + Power_Up_Setting[8] * 256;	

	}

}

void UseData(uint8* _data, uint16 screen_id)
{

 	variables_update(_data, screen_id);
 	controls_update(screen_id);
	delay_ms(100);

} 

void load_pid_target_delta(void)
{
	uint8 i;
	uint32 ind;
	uint8 buff[2];
	char str[10];
	
	STM32_FLASH_Read(STM32_FLASH_SAVE_TARGET_LOCKER_DELTA, (u16*)buff, 1); 
    if((buff[0] > 3) || (buff[0] == 0))
	{
		TargetLockerDelta = 0.01;
	}
	else
	{
		TargetLockerDelta = buff[0] * 0.01;
	}

}
void load_device_address(void)
{
	uint8 i;
	uint32 ind;
	uint8 buff[2];
	
	STM32_FLASH_Read(STM32_FLASH_SAVE_DEVICE_ADDRESS, (u16*)buff, 1); 
    DeviceAddress = buff[0];

}

void load_use60hz(void)
{
	uint8 i;
	uint8 buff[2];
	
	STM32_FLASH_Read(STM32_FLASH_SAVE_USE_60HZ, (u16*)buff, 1); 

	printf("load_use60hz(void): buff[0] = %d\n",buff[0]);

	if (buff[0] > 1) buff[0] = 0;
    Use60HzAsMaxFreq = buff[0];
	MaxFreqOut = (Use60HzAsMaxFreq == TRUE)?60:50;
	printf("MaxFreqOut = %dHz\n",MaxFreqOut);


}

void load_avf_mode(void)
{
	uint8 i;
	uint8 buff[2];
	
	STM32_FLASH_Read(STM32_FLASH_SAVE_AVF_MODE, (u16*)buff, 1); 

	printf("load_avf_mode(void): buff[0] = %d\n",buff[0]);

	if (buff[0] > 1) buff[0] = 0;
    AvfMode = buff[0];
	printf("AvfMode = %d\n",AvfMode);

//							

}

								
//
void load_pump_cancel_freq(void)
{
	uint8 i;
	uint8 buff[2];
	
	STM32_FLASH_Read(STM32_FLASH_SAVE_PUMP_CANCEL_FREQ, (u16*)buff, 1); 

	printf("load_pump_cancel_freq: buff[0] = %d\n",buff[0]);

	if (buff[0] > 50) buff[0] = 20;
    PumpCancelFreq = buff[0];
	printf("PumpCancelFreq = %d\n", PumpCancelFreq);

}

void load_outlet_pressure_cali_coeff(void)
{

	uint8 buff[2];
	
	STM32_FLASH_Read(STM32_FLASH_SAVE_OUTLET_PRESSURE_CALI_COEFF, (u16*)buff, 1); 

	printf("load_outlet_pressure_cali_coeff: buff[0] = %d\n",buff[0]);

	if ((buff[0] > 150) || (buff[0] < 50)) buff[0] = 100;  //default
	OutletPressureCaliCoeff = (float)(buff[0] / 100.0);
	printf("OutletPressureCaliCoeff = %.2f\n",OutletPressureCaliCoeff);


}

void load_entrance_pressure_cali_coeff(void)
{

	uint8 buff[2];
	
	STM32_FLASH_Read(STM32_FLASH_SAVE_ENTRANCE_PRESSURE_CALI_COEFF, (u16*)buff, 1); 

	printf("load_entrance_pressure_cali_coeff: buff[0] = %d\n",buff[0]);

	if ((buff[0] > 150) || (buff[0] < 50)) buff[0] = 100;  //default
	EntrancePressureCaliCoeff = (float)(buff[0] / 100.0);
	printf("EntrancePressureCaliCoeff = %.2f\n",EntrancePressureCaliCoeff);


}

void load_remote_target_enable(void)	  //20210618
{

	uint8 buff[2];

	STM32_FLASH_Read(STM32_FLASH_SAVE_REMOTE_TARGET_ENABLE, (u16*)buff, 1); 

	printf("load_remote target enable: buff[0] = %d\n",buff[0]);

	if (buff[0] > 1)  buff[0] = 0;  //default
	RemoteTargetEnable = buff[0];
	printf("RemoteTargetEnable = %d\n", RemoteTargetEnable);


}

void load_use_io_target_pressure(void)	  //20210702
{

	uint8 buff[2];

	STM32_FLASH_Read(STM32_FLASH_SAVE_USE_IO_TARGET_PRESSURE, (u16*)buff, 1); 

	printf("load_use_io_target_pressure: buff[0] = %d\n",buff[0]);

	if (buff[0] > 1)  buff[0] = 0;  //default
	UseIoTargetPressure = buff[0];
	printf("UseIoTargetPressure = %d\n", UseIoTargetPressure);


}

void load_speed_up_factor(void)	  //20210712
{
	uint8 buff[2];

	STM32_FLASH_Read(STM32_FLASH_SAVE_SPEED_UP_FACTOR, (u16*)buff, 1); 

	printf("load_speed_up_factor: buff[0] = %d\n",buff[0]);

	if ((buff[0] > 100) || (buff[0] < 10)) buff[0] = 20;  //default
	SpeedUpFactor = buff[0] / 10.0;
	printf("Speed Up Factor = %.1f\n", SpeedUpFactor);
}


void load_pump_max_freq(void)	  //20210911	
{
	uint8 buff[2];

	STM32_FLASH_Read(STM32_FLASH_SAVE_PUMP_MAX_FREQ, (u16*)buff, 1); 

	printf("load_pump_max_freq: buff[ ] = {%d, %d}\n", buff[0], buff[1]);

	PumpMaxFreq = ((buff[0]<<8) | buff[1]) / 10.0;

	if (PumpMaxFreq > MaxFreqOut) PumpMaxFreq = MaxFreqOut; //default

	printf("PumpMaxFreq = %.1f\n",PumpMaxFreq);
}

void LoadAllSettingsInFlash()
{ 
	uint8 read_buff[STM32_FLASH_USER_INIT_DATA_LEN];
	uint8 str[10];
#ifdef DBG_FLASH_IMAGE
 	uint8 buff[10];
#endif

	printf("\n\nRead %d bytes from on-chip Flash, Address Range: 0x%x -- 0x%x\n", STM32_FLASH_USER_INIT_DATA_LEN, \
						STM32_FLASH_SAVE_ADDR_USER_SETTINGS,  STM32_FLASH_SAVE_ADDR_USER_SETTINGS + STM32_FLASH_USER_INIT_DATA_LEN - 1);
	
	
		 
	STM32_FLASH_Read(STM32_FLASH_SAVE_ADDR_USER_SETTINGS, (u16*)read_buff, STM32_FLASH_USER_INIT_DATA_LEN / 2);
	IWDG_FEED

	load_use60hz();
	load_avf_mode();
	load_pump_cancel_freq();
	load_outlet_pressure_cali_coeff();
	load_entrance_pressure_cali_coeff();

	load_remote_target_enable();	//20210618
	load_use_io_target_pressure();
	load_speed_up_factor();	

	load_pump_max_freq();	//20210911

	UseData(read_buff, SCREEN_PUMP_GROUPING);	
	UseData(read_buff, SCREEN_OUTLET_SENSOR);	
	UseData(read_buff, SCREEN_ENTRANCE_SENSOR);	
	UseData(read_buff, SCREEN_ENTRANCE_SENSOR0);	
	UseData(read_buff, SCREEN_ENTRANCE_SENSOR2);
	IWDG_FEED	
	UseData(read_buff, SCREEN_PUMP_SWITCH_CONDTION);	
	UseData(read_buff, SCREEN_SLEEP_SETTING);	
	UseData(read_buff, SCREEN_POWER_UP_SETTING);
	UseData(read_buff, SCREEN_VALVE_CONTROL);	
	UseData(read_buff, SCREEN_VALVE_CONTROL1);
	IWDG_FEED	

	UseData(read_buff, SCREEN_PID_SETTING);	
	UseData(read_buff, SCREEN_SUPPLIER_INFO_3);	
	UseData(read_buff, SCREEN_TRAIL_DATE_SETTING);	
	UseData(read_buff, SCREEN_USER_PIN_MANAGER);	
	UseData(read_buff, SCREEN_SCREEN_SETTING);	

	UseDataTC(read_buff);

	load_pid_target_delta();

	load_speed_up_factor();	  //20210712

	load_pump_temp_setting();

#ifndef FIXED_DEVICE_ADDRESS
	load_device_address();
	sprintf(str, "%d", DeviceAddress);
	SetTextValue(SCREEN_DEVICE_ADDRESS, 1, (uchar*)str);
#endif



	LOAD_VERSION_NBR
	IWDG_FEED




 	//load_err_record();

	printf("RemoteTargetPressure = %.2f\n", RemoteTargetPressure);

	sprintf(str, "%.2f", RemoteTargetPressure);
	SetTextValue(SCREEN_TIME_CTR_3, TXT_TC_REMOTE_PRESSURE, (uchar*)str);

	printf("TargetLockerDelta = %0.2f\n",  TargetLockerDelta);

	sprintf(str, "%.2f", TargetLockerDelta);
	SetTextValue(SCREEN_PID_SETTING, TXT_TARGET_LOCKER_DELTA, (uchar*)str);

	printf("%s\n",  str);



	flash_data_loaded = TRUE;	


#ifdef DBG_FLASH_IMAGE

	delay_ms(100);
	add_flash_data_to_table(buff, 10);	//dummy
	add_flash_data_to_table(read_buff, sizeof(read_buff));
#endif	


}

void user_kw_melt_down(uint8 kw)
{
	uint8 write_buff[2];
	write_buff[0] = kw;
	STM32_FLASH_Write(STM32_FLASH_SAVE_CHKSUM_ADDR , (u16*)write_buff, 1);
}

#if(AUTHORITY_ADMIN)
/**********************************************************************************************
 * Name：FactorySettings(uint8 kw)
 * Brief：The function implements 1 of 2 tasks:
 *        kw != 0x5a:  mess up user data(thus send a user-data-tidy-up cmd to the next power-up) 
                       and unlocks factory data 
 *  	  kw == 0x5a, tidy up user data(reset user data) and unlocks factory data(set factory
 *                     pin to 666666) 
 * Para：   null	
 * Return ：null
 * 
 * Caller(s):    main.c/NotifyReadFlash()
 **********************************************************************************************/
void FactorySettings(uint8 kw)
{
	uint8 i, data[1];

	data[0] = kw;
	WriteUserFlash(CHKSUM_ADDR, 1, data);			//user data chksm
	delay_ms(120);

	data[0] = 0xff;
	WriteUserFlash(FACTORY_INFO_LOCKED_START_ADDR, 1, data);   //factory-lock word is messed up, SPW is virtually reset to default
	delay_ms(120);
	
}

#else

/**********************************************************************************************
 * Name：FactorySettings(uint8 kw)
 * Brief：The function implements 1 of 2 tasks:
 *        kw != 0x5a:  mess up user data(thus send a user-data-tidy-up cmd to the next power-up)                       
 *  	  kw == 0x5a, tidy up user data(reset user data), including para settings and kw|user-pin 
 * Para：   kw: flag indicating whether user data has been initialized. 0x5a: yes.	
 * Return ：null
 * 
 * Caller(s):    main.c/NotifyReadFlash()
 **********************************************************************************************/
void FactorySettings(uint8 kw)		//STM32_FLASH_SAVE_ADDR_USER_SETTINGS
{
	uint8 i, data[STM32_FLASH_USER_INIT_DATA_LEN];
	for(i = 0; i < STM32_FLASH_USER_INIT_DATA_LEN; i++)
	{
		data[i] = DefaultSettings[i];
	}
	if(kw != 0x5a)
	{
		user_kw_melt_down(kw);
	}
	else
	{
		STM32_FLASH_Write(STM32_FLASH_SAVE_ADDR_USER_SETTINGS, (u16*)data, STM32_FLASH_USER_INIT_DATA_LEN / 2);	

		clear_error(); 
		save_target_locker(30);
		save_pump_temp_setting_default();
		SaveCompanyName(" ");
		save_remote_target_default();
		save_target_locker_delta(0.01);
		save_pump_cancel_freq(20);
	 	//save_avf_mode(OFF);			  20210703

 #ifndef FIXED_DEVICE_ADDRESS
 		DeviceAddress = 1;
 		SaveDeviceAddress();
 #endif
 		SaveUse60Hz(0);
		SaveCaliCoeffs(100);
		SaveRemoteTargetEnable(0);	     //20210618
		SaveUseIoTargetPressure(0);      //20210702
		save_speed_up_factor_value(3.0); //20210712	
		save_pump_max_freq(50.0);	     //20210911
	}

} 
#endif
	
/**********************************************************************************************
	Read background data(kw/pins/errCounter/lockflag) and do some initializtions if needed, 
	use the BKG data( eg. extract pins) before load in user data and apply it to relevant 
	variables and UI controls of the screen module. 
	--if kw != 0x5a, reset user-data zone to default
***********************************************************************************************/

void FactoryCheck(void)
{
	uint8 read_buff[STM32_FLASH_SAVE_BKG_DATA_LEN];

//#define DBG_FACTORYCHECK
//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_FACTORYCHECK
	#ifdef DBG_EVENT_AS_ERROR
		   char buff[10];
	#endif	
#endif 

 
	printf("\n\nRead 12 bytes from on-chip Flash, Address Range: 0x%x -- 0x%x\n", STM32_FLASH_SAVE_ADDR_BKG_DATA_START, \
								                   STM32_FLASH_SAVE_ADDR_BKG_DATA_START + STM32_FLASH_SAVE_BKG_DATA_LEN - 1);	 
	STM32_FLASH_Read(STM32_FLASH_SAVE_ADDR_BKG_DATA_START,(u16*)read_buff, STM32_FLASH_SAVE_BKG_DATA_LEN / 2);

#ifdef USE_PIN_GENERATOR

#ifdef DBG_FACTORYCHECK
	#ifdef DBG_EVENT_AS_ERROR
		   sprintf(buff, "buf[8]:0x%x", read_buff[8]); 
		   EventRecorderWithoutTimeStamp((uchar*)buff);
	#endif	
#endif 
	if((read_buff[8] == 0) || (read_buff[8] > 10)) read_buff[8] = 1;  //make sure that it is within the range [1~10]
	PW_ErrIndex = read_buff[8];

#ifdef DBG_FACTORYCHECK
	#ifdef DBG_EVENT_AS_ERROR
		   sprintf(buff, "buf[8]:0x%x", read_buff[8]); 
		   EventRecorderWithoutTimeStamp((uchar*)buff);
	#endif	
#endif 
#endif

#ifdef DBG_FACTORYCHECK
	#ifdef DBG_EVENT_AS_ERROR
		   sprintf(buff, "KW:0x%x", read_buff[0]); 
		   EventRecorderWithoutTimeStamp((uchar*)buff);
	#endif	
#endif 

	if(FactoryInfoCheckPassed==0)	  //At startup, BKG data is to be checked: load FactoryInfoLocked and check checksum
	{
		printf("Factory checking...\n");

			
		//check……...
		if(read_buff[0] != 0x5a)	   // not initialized yet, need to reset to factory settings 
		{
				
			printf("kw in flash is %d instead of expected 0x5a, check failed…reset flash to factory\n", read_buff[0]);
#ifdef DBG_EVENT_AS_ERROR
	    	EventRecorderWithoutTimeStamp("check failed…reset flash to factory");
#endif			
			
			FactorySettings(0x5a);	  //send cmd: write user init-data(to be read back in NotifyWriteFlash) into flash

		
		}
#ifdef DBG_EVENT_AS_ERROR
	    EventRecorderWithoutTimeStamp("check ok, jump to next phase");
#endif		
		printf("check ok, jump to next phase\n");
		UseBKG_SettingsInFlash(read_buff);
		
		FactoryInfoCheckPassed = 1;
		LoadAllSettingsInFlash();

	}
}



void WritePinToFlash(void)
{
	uint8 data[4];
	data[0] = 0x5a;
	data[1] = UserPin[0];
	data[2] = UserPin[1];
	data[3] = UserPin[2];

	STM32_FLASH_Write(STM32_FLASH_SAVE_ADDR_PIN_USER,(u16*)data, 2);
}
void WriteFactoryPinToFlash(void)
{
	uint8 data[4];
	data[0] = FactoryPin[0];
	data[1] = FactoryPin[1];
	data[2] = FactoryPin[2];

	STM32_FLASH_Write(STM32_FLASH_SAVE_ADDR_PIN_FACTORY,(u16*)data, 2);
	
	data[0] = 0xa5;			//Initialized with factory pin
	data[1] = 0x00;

	STM32_FLASH_Write(STM32_FLASH_SAVE_ADDR_INFO_LOCKED,(u16*)data, 1);
}
/*
void WriteMileageToFlash()
{
	uint8 date_bytes[2];
	date_bytes[0] = HourMileage / 256;
	date_bytes[1] = HourMileage % 256;

	WriteUserFlash(MILEAGE_START_ADDR, MILEAGE_LEN, date_bytes);
	delay_ms(120);
	printf("WriteMileageToFlash()\n");

}
*/ /*
void WriteMileageSetToFlash()
{
	Mileage[0] = MileageLimit / 256;
	Mileage[1] = MileageLimit % 256;
	Mileage[2] = HourMileage / 256;
	Mileage[3] = HourMileage % 256;
	Mileage[4] = MileageEnabled;

	WriteUserFlash(MILEAGE_SET_START_ADDR, MILEAGE_SET_LEN, Mileage);
	delay_ms(120);
	printf("WriteMileageSetToFlash()\n");

} */

/************************************************************
 * Do not activate User-pin if it's value is default
*************************************************************/
void PinChecker()
{

	if((UserPin[0] == 88) && (UserPin[0] == 88) && (UserPin[0] == 88))
	{
		Pin_Enabled = FALSE;
	}
	else
	{
		Pin_Enabled = TRUE;
	}
	SetButtonValue(SCREEN_USER_PIN_MANAGER, BTN_ENABLE_PIN, Pin_Enabled);
	delay_ms(100);

}





/**************************************************************
 * Read User-pin and Factory-pin 
 *  -- If Factory-pin is not initialized yet, reset it to default
 *	-- If Factory-pin has been initialized but somehow out of valid range, also reset it to default
 *  -- If User-pin is default, do not activate it
 ***************************************************************/
void UseBKG_SettingsInFlash(uint8 *_data)
{
#define DBG_USEBKG_SETTINGSINFLASH
//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_USEBKG_SETTINGSINFLASH

    printf("\n\n\n Entrance: void UseBKG_SettingsInFlash(uint8 *_data)...............\n\n");
#endif  
//--------------------------------------------------------------------------------------------------


	UserPin[0] = _data[1];
	UserPin[1] = _data[2];
	UserPin[2] = _data[3];

	FactoryInfoLocked = _data[10];
	
	printf("FactoryInfoLocked = %d\n",FactoryInfoLocked);

	if(FactoryInfoLocked != 0xa5)	//uninitialized yet, use default 
	{
	   	FactoryPin[0] = 66;
		FactoryPin[1] = 66;
		FactoryPin[2] = 66;
		

#ifdef DBG_USEBKG_SETTINGSINFLASH

        printf("FACT PIN NOT SET YET\n");
#endif  


	}
	else
	{
	  	FactoryPin[0] = (_data[4] < 100)?_data[4]: 66;		//in case the value is out of range: 0~99, use default
		FactoryPin[1] = (_data[5] < 100)?_data[5]: 66;
		FactoryPin[2] = (_data[6] < 100)?_data[6]: 66;
		
#ifdef DBG_USEBKG_SETTINGSINFLASH

        printf("FACT PIN ALREADY SET \n");
#endif  


	}

#ifdef DBG_USEBKG_SETTINGSINFLASH
	 	printf("UserPin[0] = %d\n",UserPin[0]);
	  	printf("UserPin[1] = %d\n",UserPin[1]);
	
	   	printf("UserPin[2] = %d\n",UserPin[2]);
	
	  	printf("FactoryPin[0] = %d\n",FactoryPin[0]);
	    printf("FactoryPin[1] = %d\n",FactoryPin[1]);
		printf("FactoryPin[2] = %d\n",FactoryPin[2]);
#endif 
   

//	variables_update(_data,  SCREEN_TIME_SETTING_3);	  //Update MilageLimit and HourMilage
//	controls_update_SCREEN_TIME_SETTING_3();

//	UpdateMileageStatus();

	if((UserPin[0] == 88) &&(UserPin[1] == 88)  &&(UserPin[2] == 88)) pin_ok = TRUE;
	PinChecker();
			   
}


uint8 nbr_of_enabled_pump()
{
   uint8 i, nbr = 0;
   for(i = 0; i < 6; i++)
   {
	  nbr += pump_enable_tbl[i];

   }
   return nbr;

}
/****************************************************************************************************
   Monitor of expiration event for trial user
 
   para: TrialExpired, StopDate, ErrorCounter 
   Callers: meaurement.c/RealtimeMeasurement | 
   			UserInterface.c/(NotifyReadRTC|SCREEN_RTC_SETTING|SCREEN_TRAIL_DATE_SETTING)
***************************************************************************************************/
void ExpireDateManager(void)
{
	char time_stamp[50];

	UpdateRTC();
	if((StopDate.enable_expire_date == FALSE) || (TrialTimeOut(RTC_Date, StopDate) == FALSE))	 //RELIEVE EXPIRE
	{
		if(TrialExpired == TRUE) 
		{
			TrialExpired = FALSE;
			if(ErrorCounter > 0)
			{
				ErrorCounter--;
				if(ErrorCounter == 0) warning_out = OFF;
			}

			SetControlVisiable(SCREEN_MAIN_1, ICON_ERRORS_EXIST, INVISIBLE);	 //Blue Bar
			delay_ms(100);
	
			//Status info: error
			if(SysRunning == TRUE)
			{
#ifdef ENGLISH_VERSION
				SetTextValue(SCREEN_MAIN_1, TXT_RUNNING_STATUS, "AUTO-RUNNING"); 
#else
				SetTextValue(SCREEN_MAIN_1, TXT_RUNNING_STATUS, "自动运行状态"); 
#endif
				ApiCommSysStatus = 1;	//DC_CPWS-Ver-1.3.9	
			}
			else
			{
#ifdef ENGLISH_VERSION
				SetTextValue(SCREEN_MAIN_1, TXT_RUNNING_STATUS, "STANDBY"); 
#else
				SetTextValue(SCREEN_MAIN_1, TXT_RUNNING_STATUS, "等待状态"); 
#endif
				ApiCommSysStatus = 0;	//DC_CPWS-Ver-1.3.9			
			}
			delay_ms(100);
	
			RefreshButtons();
	
		}	
	  
	}
	else if(TrialExpired == FALSE)// && (TrialTimeOut(RTC_Date, StopDate)))	//newly expired	   20200320
	{
		printf("!!!Trial period has expired\n\n");
		TrialExpired = TRUE;
		ErrorCounter++;
		
		warning_out = ON;

		SetControlVisiable(SCREEN_MAIN_1, ICON_ERRORS_EXIST, VISIBLE);	 //Red Bar
		delay_ms(100);

		//Status info: error
		if(SysRunning == TRUE)
		{
			SysRunning = FALSE;
			Sys_Switches[IND_SYSTEM] = OFF;

			Start_Stopper(STOP_TYPE_SYSTEM);
			EscapeSleeper();			
		}
#ifdef ENGLISH_VERSION
		SetTextValue(SCREEN_MAIN_1, TXT_RUNNING_STATUS, "STANDBY-Error"); 
#else
		SetTextValue(SCREEN_MAIN_1, TXT_RUNNING_STATUS, "等待状态-故障"); 
#endif
		ApiCommSysStatus = 0;	//DC_CPWS-Ver-1.3.9

		delay_ms(100);
	
		//Inquiry screen
		SetTextValue(SCREEN_FAILURE_INQUIRY_2, TXT_ERROR_FREE , "");	 //erase 'error-free' msg
		delay_ms(100);	

#ifdef ENGLISH_VERSION
		sprintf(time_stamp, "20%d-%02d-%02d %02d:%02d:%02d---%s",  now_year, now_month, now_day, now_hour, now_min, now_sec, "Error");
#else
		sprintf(time_stamp, "20%d-%02d-%02d %02d:%02d:%02d---%s",  now_year, now_month, now_day, now_hour, now_min, now_sec, "故障");
#endif		
		SetTextValue(SCREEN_FAILURE_INQUIRY_2, error_info_index + 1, (uchar*)time_stamp); 
		error_info_index = (error_info_index + 1) % 10;
		
		RefreshButtons();
	  
	}
}
