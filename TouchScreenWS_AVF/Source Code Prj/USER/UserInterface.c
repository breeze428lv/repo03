/************************************版权申明********************************************
**                            深圳英捷思科技有限公司
**                             http://www.
**-----------------------------------文件信息--------------------------------------------
** 文件名称:   main.c
** Version: 2.25-Dev
** 修改时间:   2019-11-24 13:13
** 文件说明:   
**
--------------------------------------------------------------------------------------*/
#include <math.h>
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
#include "pump_sleeping.h"
#include "error_handle.h"
#include "manual_control.h"
#include "measurement.h"
#include "settings.h"
#include "time_control.h"
#include "security.h"
#include "pump_running.h"
#include "ui_controls.h"
#include "UserInterface.h"
#include "stm32_flash.h"
#include "api_comm.h"



#define TIME_100MS 10   
//#define TICK_BIAS  5



#define TELL_FC_CONTROLLER_FREQ_CHANGE_IN_MANUAL_MODE         if(FC_is_on) Manual_FC(frequency);	//change output to fc only if FC is running.
#ifdef DBG_WDG_TEST
extern uint8 DogFood; 

#endif
#ifdef DBG_FUNC_INFO
	 extern uint8  func_dbg_info;
#endif
#ifdef DBG_FUNC_INFO
	 extern uint8 FuncSwitches[];
#endif
#ifdef DBG_USE_UART1_RECEIVING
uint8 ApiCmd;

#endif
#ifdef USE_MODBUS
uint8 ApiCmd;

#endif

#ifdef DBG_FLASH_IMAGE	
uint8 flash_index;
uint8 flash_cell_value0;
#endif

extern uint8 MaxFreqOut;
extern uint8 Use60HzAsMaxFreq;

extern uint8 AvfMode;

extern uint8 PumpCancelFreq;

extern TYPE_PUMP_ICONS PumpIcons[];

extern uint8 DeviceAddress;
int WL_Bias;
extern uint8 RemoteTargetEnable;
extern float RemoteTargetPressure;
extern bool RemoteMode;

float FcFreqOutBias;

extern float TargetLockerDelta;
extern float RemoteTargetPressure;
extern bool RemoteMode;

extern float temp_limit[]; 
extern float temp_bias[];
extern float phy_quantities_bias[];
extern uint8 TempMonitorEnable;

uint8 FC_is_on;

extern uint8 TargetLockerTiming;

extern uint16 RunningCounter;

extern uint16 pFactor, iFactor, dFactor;

extern char DbgBuff[];

extern uint8 pump_usablilty_tbl[];

extern uint8 WorkingMode;

extern MY_DATE StopDate;
extern MY_DATE RTC_Date;

extern uint16 t_counter, i_counter;                                                             //100毫秒(10个单位)

#ifdef USE_PIN_GENERATOR   //
extern uint8 PW_ErrIndex;
#endif
 
uint8 NewMinuteEvent;
int OldHour = 99;
int OldMinute = 99;
uint16 ReturnScreenId;
bool unsaved_changes_exist = false;		  //unsaved_changes_exist

uint8 BL_Control_Enable;
uint8 BL_On_Level;
uint8 BL_Off_Level;
uint8 BL_On_Time = 30;

float manual_freq_setting;

extern uint8 ErrorCounter;

extern uint8 SysRunning;

uint8 SysStandby = FALSE;

extern uint8 WarmUpDone;

//Timing
uint8 nHoursCounter = 0;

extern uint8 SleepingEnableMode;

extern uint8 ManualIsRunning;

extern uint16 PowerUpAutoRunTimer;

#ifdef DUAL_WATER_SOURCE
extern uint16 TankOnCounter;
#endif
uint8 pump_mode;

extern uint8 Manual_Setting;

extern uint8 SoftStopping;

#ifdef USE_CURVES
extern uint8 up_crossed;	
#endif

extern uint8 PumpErrStatus;

extern uint8 PumpGroupMode;

extern uint8 MileageEnabled;

extern uint8 Error_Channel_Disconnected;

extern uint8 Error_Switch_Status;

extern float DefaultTargetPressure;

//extern uint8 ValveControlGap;
extern uint8 task_index;
extern uint8 Sys_Pin_Enabled;
extern uint8 Pin_Enabled;
extern uint8 scheduler_today_is_working;
extern uint8 TaskIsRunning;
extern uint8 error_occured;
extern uint8 error_never_occured;

extern uint8 FactoryPin[];
extern uint8 Mileage[];                             //MileageLimit[0:1], HourMileage[2:3]
extern uint16 MileageLimit;
extern uint16 HourMileage;
extern uint8 MileageStatus;

extern uint8 FactoryInfoLocked;

//extern uint8 PT_Selected;
//extern uint8 Pressure_Roof;
//extern uint8 Pressure_Floor;
extern uint8 Pressure_Full_range;
extern int Pressure_Bias;

extern uint8 Low_Pressure_Protection_Selected;
extern uint8 Low_Pressure_Protection_Timing;
//extern uint8 skip_restart;
extern float tc_target_pressure[];

extern uint8 Sys_Switches[];
extern float Tartget_Pressure;
extern float OutletRealTimePressure;

extern uint8  cmd_buffer[];                                                     //指令缓存

static uint16 current_screen_id = 0;                                                 //当前画面ID

                                                            
static int sec = 1;                                                                  //时间秒
                                                           //滑动选择分钟 
extern uint8 Temp;
extern uint8 now_year;
extern uint8 now_month;
extern uint8 now_day;
extern uint8 now_weekday;
extern uint8 new_day_starts;
extern uint8 now_hour;
extern uint8 now_min;
extern uint8 now_sec;
extern uint8 time_control_enabled;
extern uint8 schedule_loaded;
extern uint8 flash_data_loaded;

extern uint8 PinMode;

extern uint16 FuelMeter[];

extern uint8 SettingsLoaded;

extern char PhoneNumber[];
extern char CompanyName[];
void UpdateUI(void);                                                                 //更新UI数据
uint8 FactoryInfoCheckPassed = 0;

void delay(unsigned int count);
float ADC_ConvertedValue; 
float ADC_ConvertedValueLocal; 
float temp;      
float DACx;
uint8 UserPin[3];
uint8 flag = 0;
uint8 now_vs_time_window = 3;
uint8 pin_ok = FALSE;

uint16 low_pressure_check_counter = 0;
uint16 low_entrance_pressure_check_counter = 0;

uint8 low_pressure_check_ready = FALSE;
uint8 low_entrance_pressure_check_ready = FALSE;

//SCREEN_PUMP_GROUPING
uint8 WaterSourceMode = 0;
uint8 Max_Pump_Nbr;
//uint8 Small_Pump_Selected;
uint8 Fixed_FC_Pump;
extern uint8 pump_enable_tbl[];

//SCREEN_OUTLET_SENSOR
uint8 Outlet_Sensor_type;
float Outlet_Sensor_Range;
float Outlet_Pressure_Bias;
uint8 Outlet_LP_Protection_Selected;
float Outlet_LP_Protection_Value;
uint8 Outlet_HP_Protection_Selected;
float Outlet_HP_Protection_Value;

//SCREEN_ENTRANCE_SENSOR
uint8 Entrance_Sensor_type;
float Entrance_Sensor_Range;
float Entrance_Pressure_Bias;
float Entrance_LP_Protection_Value;
uint16 Entrance_LP_Protection_Delay;
float Entrance_LP_Protection_Restore_Value;

//SCREEN_ENTRANCE_SENSOR0
uint8 Entrance_sensor0[12];

//SCREEN_ENTRANCE_SENSOR2
uint8 Entrance_sensor2[4];


//SCREEN_PUMP_SWITCH_CONDTION
uint8 Pump_Switch_Condtion[16];


//SCREEN_SLEEP_SETTING
uint8 Sleep_Setting[9];

//SCREEN_SCREEN_POWER_UP_SETTING
uint8 Power_Up_Setting[12];

//SCREEN_VALVE_CONTROL
uint8 Valve_Control[8];

//SCREEN_VALVE_CONTROL1
uint8 Valve_Control1[6];

//SCREEN_PID_SETTING
uint8 PID_Setting[8];

//SCREEN_MANUAL_AVF
uint8 manual_avf_button_state[6] = {0, 0, 0, 0, 0, 0}; //added 20220513

uint8 debt_tbl[7] = {0, 0, 0, 0, 0, 0, 0};

extern char StatusBuff[];

extern uint8 working_weekdays[];

extern int PinChkType;

extern char* UserPinReminder[];

/*! 
*  \brief  消息处理流程
*  \param msg 待处理消息
*  \param size 消息长度
*/


int VarIndex_SCREEN_PUMP_SWITCH_CONDTION[10] = {
													0,
													0,	  //1
													2,	  //2
													4,	  //3
													5,	  //4
													7,	  //5
													8,	  //6
													9,	  //7
													11,	  //8
													13,	  //9




};

int VarIndex_SCREEN_SLEEP_SETTING[8] = {
													0,
													0,	  //1	2
													2,	  //2	1
													3,	  //3	1
													4,	  //4	2
													6,	  //5	1
													7,	  //6	1
													8,	  //7	1

};

void WarningTagManager(uint8 cmd);
void ProcessMessage( PCTRL_MSG msg, uint16 size )
{
    uint8 cmd_type = msg->cmd_type;                                                  //指令类型
    uint8 ctrl_msg = msg->ctrl_msg;                                                  //消息的类型
    uint8 control_type = msg->control_type;                                          //控件类型
    uint16 screen_id = PTR2U16(&msg->screen_id);                                     //画面ID
    uint16 control_id = PTR2U16(&msg->control_id);                                   //控件ID
    uint32 value = PTR2U32(msg->param);                                              //数值

 
    switch(cmd_type)
    {  
	    case NOTIFY_TOUCH_PRESS:                                                        //触摸屏按下
	    case NOTIFY_TOUCH_RELEASE:                                                      //触摸屏松开
	        NotifyTouchXY(cmd_buffer[1],PTR2U16(cmd_buffer+2),PTR2U16(cmd_buffer+4)); 
	        break;      
			                                                              
	    case NOTIFY_WRITE_FLASH_OK:                                                     //写FLASH成功
	        NotifyWriteFlash(1);                                                      
	        break;     
			                                                               
	    case NOTIFY_WRITE_FLASH_FAILD:                                                  //写FLASH失败
	        NotifyWriteFlash(0);                                                      
	        break;      
			                                                              
	    case NOTIFY_READ_FLASH_OK:                                                      //读取FLASH成功
	        NotifyReadFlash(1,cmd_buffer+2,size-6);                                     //去除帧头帧尾
	        break;      
			                                                              
	    case NOTIFY_READ_FLASH_FAILD:                                                   //读取FLASH失败
	        NotifyReadFlash(0,0,0);                                                   
	        break;     
			                                                               
	    case NOTIFY_READ_RTC:                                                           //接收当前RTC时间
	        NotifyReadRTC(cmd_buffer[2],cmd_buffer[3],cmd_buffer[4],cmd_buffer[5],cmd_buffer[6],cmd_buffer[7],cmd_buffer[8]);
	        break;
	        
	    case NOTIFY_CONTROL:
        {
            if(ctrl_msg==MSG_GET_CURRENT_SCREEN)                                    //画面ID变化通知
            {
                NotifyScreen(screen_id);                                            //画面切换调动的函数
            }
            else
            {
                switch(control_type)
                {
	                case kCtrlButton:                                                   //按钮控件
	                    NotifyButton(screen_id,control_id,msg->param[1]);                  
	                    break;    
						                                                         
	                case kCtrlText:                                                     //文本控件
	                    NotifyText(screen_id,control_id,msg->param);                       
	                    break;  
						                                                           
                	case kCtrlProgress:                                                 //进度条控件
                   	 	NotifyProgress(screen_id,control_id,value);                        
                    	break; 
					                                                            
                	case kCtrlSlider:                                                   //滑动条控件
                    	NotifySlider(screen_id,control_id,value);                          
                    	break;  
					                                                           
                	case kCtrlMeter:                                                    //仪表控件
                    	NotifyMeter(screen_id,control_id,value);                           
                    	break;  
					                                                           
                	case kCtrlMenu:                                                     //菜单控件
                    	NotifyMenu(screen_id,control_id,msg->param[0],msg->param[1]);      
                    	break;  
					                                                            
                	case kCtrlSelector:                                                 //选择控件
                    	NotifySelector(screen_id,control_id,msg->param[0]);                
                    	break;   
					                                                           
                	case kCtrlRTC:                                                      //倒计时控件
                    	NotifyTimer(screen_id,control_id);
                    	break;
                    
                	default:
                    	break;
                }
            } 
            break;  
        } 
    	case NOTIFY_HandShake:                                                          //握手通知                                                     
        	NOTIFYHandShake();
        	break;
        	
    	default:
        	break;
    }
}
/*! 
*  \brief  握手通知
*/
void NOTIFYHandShake()
{
 
}

/*! 
*  \brief  画面切换通知
*  \details  当前画面改变时(或调用GetScreen)，执行此函数
*  \param screen_id 当前画面ID
*/
void NotifyScreen(uint16 screen_id)
{
    //TODO: 添加用户代码
    current_screen_id = screen_id;   			  //在工程配置中开启画面切换通知，记录当前画面ID
	   	                             
}

/*! 
*  \brief  触摸坐标事件响应
*  \param press 1按下触摸屏，3松开触摸屏
*  \param x x坐标
*  \param y y坐标
*/
void NotifyTouchXY(uint8 press,uint16 x,uint16 y)
{ 
    //TODO: 添加用户代码
	char strr[50];
 
	sprintf(strr, "Position x=%x, y=%x  is touched!",x, y);
	
    SendStrings((uchar*)strr);
}

/*! 
*  \brief  更新数据
*/ 
void UpdateUI(void)
{
    //文本设置和显示  定时20ms刷新一次
#if(AUTHORITY_ADMIN)
	if(current_screen_id == SCREEN_START)	   //Jump to "Reset to Factory settings"
	{
		
		SetScreen(SCREEN_BALANCE_WARNING);
		NotifyScreen(SCREEN_BALANCE_WARNING);
		delay_ms(100);
		SetTextValue(SCREEN_BALANCE_WARNING, 2, "所有数据将恢复默认值");

	} 	 

#else

  
#endif      
}

uint8 next_fc_button(uint8 i)
{
	return (i + 2 - i / 6 + (7 - (i + 2 - i / 6) % 8) / 7 * 2) % 8;
}

uint8 is_fc_button(uint8 id)
{
   // return 1 - (id + id / 7) % 2;
   return (id == 2) || (id == 4) ||(id == 6) || (id == 8) || (id == 10) || (id == 11);

}



#ifdef DBG_SHOW_NAMES
	 const char* control_state_names[2] = {
											  "OFF",
			 							      "ON"
	 };
	 const char* control_names[11] = {
											  "工频1",
											  "变频1",
											  "工频2",
											  "变频2",
											  "工频3",
											  "变频3",
											  "工频4",
											  "变频4",
											  "工频5",
											  "变频5",
											  "辅变"

	 };
#endif

void button_controller(uint8 i, uint8 cmd)
{
   if(cmd == ON)
   {
	  	if(debt_tbl[i] > 0) 
		{
			debt_tbl[i]--;
			if(debt_tbl[i] == 0)
			{
				SetControlEnable(SCREEN_MANUAL_CTR_2, i, ON);
				//delay_ms(100);
				SetControlVisiable(SCREEN_MANUAL_CTR_2, 20 + i, OFF);
		  	    //delay_ms(100);
			}
		}
   }
   else
   {
		debt_tbl[i]++;
		SetControlEnable(SCREEN_MANUAL_CTR_2, i, OFF);
		//delay_ms(100);
		SetControlVisiable(SCREEN_MANUAL_CTR_2, 20 + i, ON);
		//delay_ms(100);

   }
}

void SetEnablityForAllManualButtons(uint8 on_off)
{
	 uint8 i, max_nbr;
 	 max_nbr = (AvfMode == 0)? 11:6;

	 for (i = 1; i <= max_nbr; i++)
	 {
		  button_controller(i, on_off);
	 }
}
#define  CONFIG_MAUANL_BUTTONS  SetEnablityForAllManualButtons((OutletRealTimePressure < Tartget_Pressure));   

void manual_fc_button_manager(uint8 id, uint8 status)
{
	uint8 i, j, k;

	for (i = 1; i <= 11; i++)
	{
		 if ((is_fc_button(i)) && (i != id))  button_controller(i, 1 - status);
	}

}

uint8 get_partner(uint8 id) 
{
	if (id == 0) return 0;
	if (id >= 11) return 0;
	if (id < 3) return 3 - id;
	if (id < 5) return 7 - id;
	if (id < 7) return 11 - id;
	if (id < 9) return 15 - id;
	return 19 - id;
}

//#define DBG_FUNC_INFO
void partner_button_manager(uint8 id, uint8 status)
{
	uint8 i;
	i = get_partner(id);

#ifdef DBG_FUNC_INFO

		printf("%s's partner is %s\n\n", control_names[id - 1], control_names[i - 1]);

#endif

	button_controller(i, 1 - status);

#ifdef DBG_FUNC_INFO

		printf("%s's usability: %s\n\n", control_names[i - 1], control_state_names[1 - status]);

#endif

}

void all_pump_off(void)
{
	uint8 i;
	Manual_FC(0.0);	 //fc - out = 0
	FC_SWITCH = OFF;

	for(i = 1; i <= 11; i++)
	{
		push_cmd_pump(CMD_SET_BUTTON, SCREEN_MANUAL_CTR_2, i, OFF);
		push_cmd_pump(CMD_SET_USABILITY, SCREEN_MANUAL_CTR_2, i, ON);
		push_cmd_pump(CMD_SET_VISIBILITY, SCREEN_MANUAL_CTR_2, 20 + i, OFF);	//masks
		debt_tbl[i] = 0;		
	}

	for(i = 0; i < 6; i++)
	{
		Manual_Control_Pump(i, MOD_FC, OFF);
		Manual_Control_Pump(i, MOD_PF, OFF);
	}
}

void all_pump_off_avf(void)
{
	uint8 i;

	for(i = 1; i <= 6; i++)
	{
		push_cmd_pump(CMD_SET_BUTTON, SCREEN_MANUAL_CTR_AVF, i, OFF);
		push_cmd_pump(CMD_SET_USABILITY, SCREEN_MANUAL_CTR_AVF, i, ON);
		push_cmd_pump(CMD_SET_VISIBILITY, SCREEN_MANUAL_CTR_AVF, 20 + i, OFF);	//masks
		debt_tbl[i] = 0;
		
	}

	for(i = 0; i < 6; i++)
	{
		Manual_Control_Pump(i, MOD_FC, OFF);
		Manual_Control_Pump(i, MOD_PF, OFF);
	}
}

extern uint8 TrialExpired;
extern float OutletRealTimePressureForCali;
extern float OutletPressureCaliCoeff ;															
extern float EntranceRealTimePressureForCali;
extern float EntrancePressureCaliCoeff ;
extern uint8 UseIoTargetPressure;
extern float DefaultTargetPressureMenu;

/*! 
*  \brief  按钮控件通知
*  \details  当按钮状态改变(或调用GetControlValue)时，执行此函数
*  \param screen_id 画面ID
*  \param control_id 控件ID
*  \param state 按钮状态：0弹起，1按下
*/
void NotifyButton(uint16 screen_id, uint16 control_id, uint8  state)
{ 

#ifdef    DBG_FLASH 
	int i;
#endif
	char buff[5];
  
//#define DBG_SCREEN_MANUAL_CTR_2	
/*******************************************SCREEN_MANUAL_CTR_2【手动控制-2】*******************************************/
	if(screen_id == SCREEN_MANUAL_CTR_2)
    {
		uint8 pump_index;


#ifdef DBG_SCREEN_MANUAL_CTR_2	
	 printf("\n\n\n...............Unit test of 【NotifyButton-SCREEN_MANUAL_CTR_2】................\n\n");
#endif  

		if((control_id == BTN_BACK_SCREEN_MANUAL_CTR)&& (state == ON))	   //Return, all but Pump are turned off
		{		
		 	Manual_Setting = FALSE;
			
			FC_SWITCH = OFF;
			FC_is_on = FALSE;
			delay_ms(100);
			Set_FC_Zero();
			SetTextValue(SCREEN_MANUAL_CTR_2, TXT_RUNNING_FREQUENCY, "--");

		    all_pump_off();   

		}	
        else if((control_id >= 1) && (control_id <= 11))  //For each fc/pf-buttons(1 ~ 11)
		{
#ifdef DBG_FUNC_INFO
			if(func_dbg_info)
			{
				printf("%s is %s\n\n", control_names[control_id - 1], control_state_names[state]);
			}
#endif		
					
			partner_button_manager(control_id, state); //update enablity of partners

			//get index and mode
			pump_index = (control_id - 1) / 2;

		    if(is_fc_button(control_id))   //FC on/off
			{
				// set fc 
				if(state == ON)	  
				{
					Manual_Control_Pump(pump_index, MOD_FC, ON); //connect pump to FC
					Manual_FC(manual_freq_setting);	 //set FC
					FC_SWITCH = ON; //fc 
					FC_is_on = TRUE;
				}
				else
				{
					FC_SWITCH = OFF;
					FC_is_on = FALSE;
					Manual_FC(0.0);
					Manual_Control_Pump(pump_index, MOD_FC, OFF); //disconnect pump from FC
				}

			 	manual_fc_button_manager(control_id, state);//set usablilty of other buttons

				
			}
			else   //PF on/off
			{
				Manual_Control_Pump(pump_index, MOD_PF, state);  //set the working state for the selected pump
			}
		}			  			 	  
	}
/*******************************************SCREEN_MANUAL_CTR_2【手动控制-AVF】*******************************************/
	if(screen_id == SCREEN_MANUAL_CTR_AVF)
    {
		uint8 pump_index, i; 

		if((control_id == BTN_BACK_SCREEN_MANUAL_CTR)&& (state == ON))	   //Return, all but Pump are turned off
		{		
		 	Manual_Setting = FALSE;

			FC_is_on = FALSE;
			delay_ms(100);

			for (i = 0; i < 6; i++)
			{
			    UpdatePumpFreq(0.0, i);
			}
			SetTextValue(SCREEN_MANUAL_CTR_AVF, TXT_RUNNING_FREQUENCY, "--");	

		    all_pump_off_avf();   

		}	
        else if((control_id >= 1) && (control_id <= 6))  //For each fc-buttons(1 ~ 6)
		{		
			//get index(0~5) and mode   
			pump_index = control_id - 1;

			// set fc 
			if(state == ON)	 //Set Pump[pump_index]: ON 
			{
				Manual_Control_Pump(pump_index, MOD_FC, ON); //connect pump to FC
				
				UpdatePumpFreq(manual_freq_setting, pump_index);   //set frequency
			
				FC_is_on = TRUE;

				manual_avf_button_state[pump_index] = ON; //20220513
			}
			else             //Set Pump[pump_index]: OFF
			{
				FC_is_on = FALSE;
				UpdatePumpFreq(0.0, pump_index);			  //Reset frequency
				Manual_Control_Pump(pump_index, MOD_FC, OFF); //disconnect pump from FC

				manual_avf_button_state[pump_index] = OFF; //20220513
			}

		 	manual_fc_button_manager(control_id, state);//set usablilty of other buttons


		}			  			 	  
	}
/*******************************************SCREEN_TIME_CTR_3【时控开关-3】*******************************************/
	else if(screen_id == SCREEN_TIME_CTR_3)	 
    {
		if((control_id >= BTN_MONDAY_ENABLED) && (control_id <= BTN_SUNDAY_ENABLED))   //Button weekdays variables updating
		{
			working_weekdays[control_id - 3] = state;
			printf("working_weekdays[%d] = %d\n",control_id - 3, state);
			
			if(unsaved_changes_exist == false) unsaved_changes_exist = true;
		}
		else if((control_id >= BTN_FIRST_TASK_ENABLED) && (control_id <= BTN_LAST_TASK_ENABLED))   //Task selector variables updating
		{
			UpdatScheduleCell(state, control_id - 10, 0);
			
			if(unsaved_changes_exist == false) unsaved_changes_exist = true;		
		}
		else if (control_id == BTN_REMOTE_TARGET_ENABLE)  		  //20210618
		{
			RemoteTargetEnable = state;

			RefreshTarget(); //20210707
			
			if(unsaved_changes_exist == false) unsaved_changes_exist = true;		
		}
		else if (control_id == BTN_USE_IO_TARGET_PRESSURE)  		  //USE_IO_TARGET_PRESSURE  20210702
		{
			UseIoTargetPressure = state;
			if (UseIoTargetPressure == OFF)		 //
			{
				DefaultTargetPressure = DefaultTargetPressureMenu;

				sprintf(buff, "%.2f", DefaultTargetPressure);

				SetTextValue(SCREEN_TIME_CTR_3, TXT_TC_DEFAULT_PRESSURE, buff);

				SetTextValue(SCREEN_MAIN_1, TXT_TARGET_PRESSURE, buff);
			}
			
			if(unsaved_changes_exist == false) unsaved_changes_exist = true;		
		}
		else if((control_id == BTN_BACK_SCREEN_TIME_CTR)&&(state == OFF))		//【返回】
		{	
			if(PassStopperMonitor())			//pass balance check
			{		
				if(check_schedule_ok())			 //pass schedule data check
				{

					if(!RemoteMode) ScheduleUpdate();	 //?? 2020-7-16
	

			   		SetScreen(SCREEN_MAIN_1);									   //Back to the parent menu
					delay_ms(120);

					SetControlVisiable(SCREEN_TIME_CTR_3, ICON_TC_INPUT_ERROR, INVISIBLE);   //warning for invalid input	  

					if(unsaved_changes_exist) 
					{
						SaveScheduleToFlash();											       //Save schedule to flash
						printf("  SaveScheduleToFlash();\n");
						unsaved_changes_exist = false;
					}					
				}
		        else																	   //Invalid schedule
			    {
				
					SetControlVisiable(SCREEN_TIME_CTR_3, ICON_TC_INPUT_ERROR, VISIBLE);   //warning for invalid input	  
				}
			}
			else																	      //No enough working hours balance, go home screen 
			{
				ReturnScreenId = SCREEN_MAIN_1;											  // Balance warning screen --> main screen 
				unsaved_changes_exist = false;
			}				
		}
	}

/*******************************************SCREEN_MAIN_1  【主界面-1】*******************************************/
	else if(screen_id == SCREEN_MAIN_1)	
	{	
		 char buff[5]; 
		 if((control_id ==  BTN_START_AUTO_SCREEN_MAIN) && (state == OFF)) 	 	//【启动】
		 {
			 if((WarmUpDone == TRUE) && (TrialExpired == FALSE))	//20210604	Do nothing if warm-up is not done or expiration occurs
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
		 }
		 else if((control_id ==  BTN_STOP_AUTO_SCREEN_MAIN) && (state == OFF)) 	 	//【停止】
		 {
			SetScreen(SCREEN_STOP_CONFIRM1); 

		 }
		 else if((control_id ==  BTN_MANUAL_SETTING) && (state == OFF)) 	 	//【手动】
		 {

			if(SysRunning == FALSE)
			{
				// 压力已达目标值, 禁止手动开泵
				if (OutletRealTimePressure >= Tartget_Pressure)
				{
					SetTextValue(SCREEN_MAIN_1, 9, "压力已达目标值, 禁止手动开泵");
					WarningTagManager(ON);
					return;
				}

				Manual_Setting = TRUE;

				//Switch to manual screen
				if (AvfMode) 
					SetScreen(SCREEN_MANUAL_CTR_AVF);
				else
					SetScreen(SCREEN_MANUAL_CTR_2);
				delay_ms(100); 
			
		   		manual_freq_setting = Pump_Switch_Condtion[4];

				Manual_FC(0.0);//no control voltage to FC
				FC_is_on = FALSE;

				//refresh frequency value in both  SCREEN_MANUAL_CTR_2 and SCREEN_MANUAL_AVF
				sprintf(buff, "%.1f", manual_freq_setting);
				SetTextValue(SCREEN_MANUAL_CTR_2, TXT_RUNNING_FREQUENCY, (uchar*)buff);
				SetTextValue(SCREEN_MANUAL_CTR_AVF, TXT_RUNNING_FREQUENCY, (uchar*)buff);

			}
		 }
		 else if((control_id ==  BTN_PARAMETER_SETTING) && (state == OFF)) 	 	//【参数设置】
		 {
			if(Pin_Enabled)	  
			{
				ReturnScreenId = SCREEN_USER_SETTINGS_2;
				SetScreen(SCREEN_PIN_CONTROL);
			}
			else
			{
			    SetScreen(SCREEN_USER_SETTINGS_2);
			}
		 }
		 else if((control_id ==  BTN_SYSTEM_SETTING) && (state == OFF)) 	 	//【系统设置】
		 {

		 	if(Sys_Pin_Enabled) 
			{
				ReturnScreenId = SCREEN_SYS_SETTINGS_2; 
				SetScreen(SCREEN_PIN_CONTROL_FACTORY);
#ifdef USE_PIN_GENERATOR
				delay_ms(100);
				SetControlEnable(SCREEN_PIN_CONTROL_FACTORY, TXT_PIN_INPUT, TRUE);
#endif
			}
			else
			{			
				SetScreen(SCREEN_SYS_SETTINGS_2); 
			}			 
		 }
	
#ifdef DBG_MANUAL_FSW
		 else if((control_id ==  52) && (state == ON)) 	  
		 {
		 	NewMinuteEvent = TRUE;
			RunningCounter = 1;
		 }
#endif
#ifdef DBG_WDG_TEST
		 else if((control_id ==  53) && (state == ON)) 	  
		 {
		 	DogFood = OFF; 
		 }
#endif


	}
/*******************************************SCREEN_STOP_CONFIRM1【停机确认1】*******************************************/
	else if(screen_id == SCREEN_STOP_CONFIRM1)	
	{	
		 if((control_id ==  BTN_STOP_CONFIRM) && (state == ON)) 	 	//【确认】
		 {
		 		RemoteMode = false;
				SysRunning = FALSE;
				Sys_Switches[IND_SYSTEM] = OFF;
				UpdateTargetPressure();
				Start_Stopper(STOP_TYPE_SYSTEM);
				EscapeSleeper();
		 }
	}
/*******************************************SCREEN_USER_SETTINGS_2【参数设置-2】*******************************************/
	else if(screen_id == SCREEN_USER_SETTINGS_2)		//【系统设置】
    {
		if((control_id == 11) && (state == OFF))  //【入口参数】
		{			
			if(WaterSourceMode == 0)
			{
			   SetScreen(SCREEN_ENTRANCE_SENSOR0); 
			}
			else if(WaterSourceMode == 1)
			{
			   SetScreen(SCREEN_ENTRANCE_SENSOR); 
			}
			else
			{
 			   SetScreen(SCREEN_ENTRANCE_SENSOR2); 
			}

		}
	    else if((control_id == 13) && (state == OFF))	  //【阀门控制】
		{			
			if(WaterSourceMode == 2)
			{
			   SetScreen(SCREEN_VALVE_CONTROL1); 
			}
			else
			{
			   SetScreen(SCREEN_VALVE_CONTROL); 
			}
		
		}

	}
/*******************************************SCREEN_SYS_SETTINGS_2【系统设置-2】*******************************************/
	else if(screen_id == SCREEN_SYS_SETTINGS_2)		//【系统设置】
    {
	/*
		if((control_id == BTN_RESET_TO_FACTORY) && (state == OFF))  //【恢复出厂设置】
		{
			ReturnScreenId = SCREEN_MAIN_1;
			ShowMessage(INFO_FACTORY_SETTING_OK);			
		    FactorySettings(0x99);
			printf("reset flash to chaos\n");

		
			
			SetButtonUsability(BUTTON_STATUS_LOCK_ALL);
		} 
		*/
	    if((control_id == BTN_TIME_SETTING) && (state == OFF))
		{			
		    PinMode = PIN_MODE_FACTORY;
		}
	    else if((control_id ==  BTN_PIN_SETTING) && (state == OFF))
		{			 
			SetScreen(SCREEN_PASSWORD_SELECT); 
		}
	    else if((control_id ==  14) && (state == OFF))
		{
			printf(	"control_id=%d\n", control_id);		 
			SetScreen(SCREEN_MAIN_1); 
		}
	}	
/*******************************************SCREEN_FACTORY_SETTING_CONFIRM【恢复出厂设置】*******************************************/
	else if(screen_id == 39)//SCREEN_FACT_SETTING_CONFIRM 
    {
		if((control_id == 1) && (state == OFF))  //【恢复出厂设置】
		{
			ReturnScreenId = SCREEN_MAIN_1;
			ShowMessage(INFO_FACTORY_SETTING_OK);			
		    FactorySettings(0x99);
			printf("reset flash to chaos\n");
		
			SetButtonUsability(BUTTON_STATUS_LOCK_ALL);
		}
	   
	}
	
/*******************************************SCREEN_USER_PIN_MANAGER【用户密码管理】*******************************************/
	else if(screen_id == SCREEN_USER_PIN_MANAGER)		//   /*	#define PARA_PW      			0 	#define SYS_PW       			1*/

    {
		char write_buff[2]; 
		if(control_id == BTN_ENABLE_PIN)  //
		{			
			if(PinMode == PIN_MODE_USER) 
			{
				Pin_Enabled = state;
			}
			else if(PinMode == PIN_MODE_FACTORY)
			{
			    Sys_Pin_Enabled = state;
			}

			ShowCurrentPassWord(PinMode, state);
		}
	    else if((control_id == BTN_CHANGE_PIN) && (state == OFF))
		{			
			
			write_buff[0] = Pin_Enabled;
			write_buff[1] = Sys_Pin_Enabled;		
			
		    STM32_FLASH_Write(STM32_FLASH_SAVE_ADDR_USER_SETTINGS + STM32_FLASH_SAVE_ADDR_USER_SCREEN_PW_SWITCH_INTERNAL,\
												 (u16*)write_buff, 1);

				
			SetScreen(SCREEN_PIN_SETTING_3); 	  

		}
	    else if((control_id ==  BTN_BACK_SCREEN_USER_PIN_MANAGER) && (state == OFF))
		{
			write_buff[0] = Pin_Enabled;
			write_buff[1] = Sys_Pin_Enabled;		
		//	STM32_FLASH_Write(STM32_FLASH_SAVE_ADDR_PW_SWITCH,(u16*)write_buff,1);
		    STM32_FLASH_Write(STM32_FLASH_SAVE_ADDR_USER_SETTINGS + STM32_FLASH_SAVE_ADDR_USER_SCREEN_PW_SWITCH_INTERNAL,\
												 (u16*)write_buff, 1);
			
			SetScreen(SCREEN_PASSWORD_SELECT); 

		}
	}	 

/*******************************************SCREEN_PASSWORD_SELECT【密码选择】*******************************************/
 	else if(screen_id == SCREEN_PASSWORD_SELECT)	 
	{
		if((control_id == 5) && (state == OFF))  //【SYSTEM PASSWORD】
		{			
		    PinMode = PIN_MODE_FACTORY;
		
			SetButtonValue(SCREEN_USER_PIN_MANAGER, BTN_PW_SWITCH, Sys_Pin_Enabled);
			delay_ms(100);
		 	SetScreen(SCREEN_USER_PIN_MANAGER);
			delay_ms(100);
			ShowCurrentPassWord(PIN_MODE_FACTORY, Sys_Pin_Enabled);
						
		}
	    else if((control_id == 6) && (state == OFF))
		{			
		    PinMode = PIN_MODE_USER;

			SetButtonValue(SCREEN_USER_PIN_MANAGER, BTN_PW_SWITCH, Pin_Enabled);
			delay_ms(100);
		 	SetScreen(SCREEN_USER_PIN_MANAGER);  
			delay_ms(100);
			ShowCurrentPassWord(PIN_MODE_USER, Pin_Enabled);
		}
	}
/*******************************************SCREEN_TIME_SETTING_3【厂家设置-3】*******************************************/
/*	else if(screen_id == SCREEN_TIME_SETTING_3)	 
	{
		 char buff[8];
		 if((control_id == BTN_CLEAR_MILEAGE) && (state == ON)) 
		 {		  	                                                           
        	HourMileage = 0;

			sprintf(buff, "%d",  HourMileage);
			SetTextValue(SCREEN_TIME_SETTING_3,  TXT_MILEAGE , (uchar*)buff);
			if(unsaved_changes_exist == false) unsaved_changes_exist = true;
		 }
		 else if(control_id ==  BTN_MILEAGE_ENABLED)
		 {	
			if(state == ON) 
			{							
				MileageEnabled = TRUE;
			}
			else
			{							
				MileageEnabled = FALSE;
			}
			if(unsaved_changes_exist == false) unsaved_changes_exist = true;							   		 		 	  	                                                          		
		 }		
		 else if((control_id ==  BTN_BACK_SCREEN_TIME_SETTING_3) && (state == ON)) 
		 {	
			if(unsaved_changes_exist) 
			{							
				WriteMileageSetToFlash();
				UpdateMileageStatus();
				unsaved_changes_exist = false;
			}				   		 		 	  	                                                          		
		 }		  
	}	*/
/*******************************************SCREEN_BALANCE_WARNING【余额报警】*******************************************/
#if(AUTHORITY_ADMIN)
	else if(screen_id == SCREEN_BALANCE_WARNING)	 
	{
		 char buff[8];
		 uint16 value = 0;
         if((control_id ==   BTN_BACK_SCREEN_BALANCE_WARNING) && (state == 1)) 
		 {		  	                                                           
 			SetTextValue(SCREEN_BALANCE_WARNING, 2,"正在恢复出厂设置, 请稍候...");
			delay_ms(100);
			SetControlVisiable(SCREEN_BALANCE_WARNING, BTN_BACK_SCREEN_BALANCE_WARNING, INVISIBLE);
			delay_ms(100);

			FactorySettings(0x99);
			delay_ms(100);
			
			SetTextValue(SCREEN_BALANCE_WARNING, 2,"设备已恢复出厂设置, 请重新上电");
		 }		  
	}	
#else
	else if(screen_id == SCREEN_BALANCE_WARNING)	 
	{
         if((control_id ==   BTN_BACK_SCREEN_BALANCE_WARNING) && (state == 1)) 
		 {		  	                                                           
			SetScreen(ReturnScreenId); 
		 }		    
	}	
#endif			  	  

/*******************************************SCREEN_FAILURE_INQUIRY_2【故障查询-2】*******************************************/
	else if(screen_id == SCREEN_FAILURE_INQUIRY_2)	 
	{
         if((control_id == BTN_CLEAR_ERROR_INFO) && (state == ON)) 
		 {		  	                                                           
 			ClearErrorMsg();
		 }		   
	}	

/*******************************************SCREEN_SCREEN_SETTING【屏幕设置-3】*******************************************/	
	else if(screen_id == SCREEN_SCREEN_SETTING)	 
	{

		if(control_id == BTN_BL_CONTROL_ENABLE)  
		{			
			BL_Control_Enable = state;
			if(unsaved_changes_exist == false) unsaved_changes_exist = true;
		}
        else if((control_id == BTN_BACK_SCREEN_SCREEN_SETTING) && (state == ON)) 
		{		                                                       
			SetPowerSaving(BL_Control_Enable, 255, 0, BL_On_Time * 60);
			SaveSettingsToFlash(SCREEN_SCREEN_SETTING);		
		}		   
	}	
/*******************************************SCREEN_PIN_CONTROL【密码控制】*******************************************/
	else if(screen_id == SCREEN_PIN_CONTROL)	 
	{
         if((control_id == 14) && (state == ON)) 
		 {
	  	    printf("ReturnScreenId = %d\n",ReturnScreenId);
                                                       
 		 	SetScreen(SCREEN_MAIN_1); 

			delay_ms(100);
			SetTextValue(SCREEN_PIN_CONTROL, TXT_USER_PIN_REMINDER, "请输入6位密码");


		 }		   	
	}	
/*******************************************SCREEN_PIN_SETTING_3【密码设置-3】*******************************************/
	else if(screen_id == SCREEN_PIN_SETTING_3)	 
	{
	    if(control_id == BTN_CANCEL_PIN)
		{
			PinChkType = CHK_VALIDITY;
			SetTextValue(SCREEN_PIN_SETTING_3, TXT_USER_PIN_REMINDER , (uchar*)UserPinReminder[PIN_INPUT]);
		}
	}  
/*******************************************SCREEN_PUMP_GROUPING【组泵方式】*******************************************/
	else if(screen_id == SCREEN_PUMP_GROUPING)	 
	{
	    // TANK_MODE
		if((control_id == BTN_TANK_MODE) && (state == ON))   
		{			
			WaterSourceMode = TANK_MODE;

			//Hide Title and Icon for NN
			SetControlVisiable(SCREEN_MAIN_1, ICON_TITLE_NN, INVISIBLE);
			SetControlVisiable(SCREEN_MAIN_1, ICON_NON_NEG, INVISIBLE);
			delay_ms(100);

			//Hide City/MPa
			SetControlVisiable(SCREEN_MAIN_1, SCREEN_MAIN_ICON_CITY, INVISIBLE);
			delay_ms(100);
			SetControlVisiable(SCREEN_MANUAL_CTR_2, MANUAL_ICON_CITY, INVISIBLE);
			delay_ms(100);
			SetControlVisiable(SCREEN_MANUAL_CTR_2, MANUAL_ICON_MPA, INVISIBLE);
			delay_ms(100);
			SetControlVisiable(SCREEN_MANUAL_CTR_AVF, MANUAL_ICON_CITY, INVISIBLE);
			delay_ms(100);
			SetControlVisiable(SCREEN_MANUAL_CTR_AVF, MANUAL_ICON_MPA, INVISIBLE);
			delay_ms(100);

			//Show Water Tank Icon
			SetControlVisiable(SCREEN_MAIN_1, SCREEN_MAIN_WATER_TANK, VISIBLE);
			delay_ms(100);

			//Floating ball, show -- for water level
			if((Entrance_sensor0[0] & 0x0f) == ON) SetTextValue(SCREEN_MAIN_1, TXT_WATER_LEVEL, "--");	   

			if(unsaved_changes_exist == false) unsaved_changes_exist = true;

		}
		//NONNEGTIVE_MODE
        else if((control_id == BTN_NONNEGTIVE_MODE) && (state == ON))   
		{			
			WaterSourceMode = NONNEGTIVE_MODE;

			//Show Title and Icon for NN
			SetControlVisiable(SCREEN_MAIN_1, ICON_TITLE_NN, VISIBLE);
			SetControlVisiable(SCREEN_MAIN_1, ICON_NON_NEG, VISIBLE);
			delay_ms(100);

			//Show City/MPa
			SetControlVisiable(SCREEN_MAIN_1, SCREEN_MAIN_ICON_CITY, VISIBLE);
			delay_ms(100);
			SetControlVisiable(SCREEN_MANUAL_CTR_2, MANUAL_ICON_CITY, VISIBLE);
			delay_ms(100);
			SetControlVisiable(SCREEN_MANUAL_CTR_2, MANUAL_ICON_MPA, VISIBLE);
			delay_ms(100);
			SetControlVisiable(SCREEN_MANUAL_CTR_AVF, MANUAL_ICON_CITY, VISIBLE);
			delay_ms(100);
			SetControlVisiable(SCREEN_MANUAL_CTR_AVF, MANUAL_ICON_MPA, VISIBLE);
			delay_ms(100);

			//Hide Water Tank Icon
			SetControlVisiable(SCREEN_MAIN_1, SCREEN_MAIN_WATER_TANK, INVISIBLE);		//TANK

			//BTNE_JOINT_PM, show -- for entrance pressure
			if(Entrance_Sensor_type == ENTRANCE_SENSOR_TYPE_3)	 
			{
				SetTextValue(SCREEN_MAIN_1, TXT_ENTRANCE_PRESSURE, "--");
				delay_ms(100);
				SetTextValue(SCREEN_MANUAL_CTR_2, TXT_MANUAL_ENTRANCE_PRESSURE, "--"); 		  //20200319
			}  

			if(unsaved_changes_exist == false) unsaved_changes_exist = true;
		}
#ifdef DUAL_WATER_SOURCE
        else if((control_id == BTN_TANK_NONNEGTIVE_MODE) && (state == ON))   
		{			
			WaterSourceMode = TANK_NONNEGTIVE_MODE;
			if(unsaved_changes_exist == false) unsaved_changes_exist = true;
		}
#endif
        else if((control_id >= 4) && (control_id < 7)) 		//pump1~3
		{			
			printf("Pump%d is selected?%d\n",control_id - 3, state);

		
			pump_enable_tbl[control_id - 4] = state;
			pump_usablilty_tbl[control_id - 4] = state;
			RefreshButtons();
			
			SetControlVisiable(SCREEN_MAIN_1, 40 + control_id - 4, 1 - state);	 //show/hide masks

			printf("pump_enable_tbl[%d] = %d\n", control_id - 4, pump_enable_tbl[control_id - 4]);
			if(unsaved_changes_exist == false) unsaved_changes_exist = true;
		}
        else if(control_id == 3) 		//pump4
		{			
			pump_enable_tbl[3] = state;
			pump_usablilty_tbl[3] = state;
			RefreshButtons();
			
			SetControlVisiable(SCREEN_MAIN_1,PumpIcons[3].icon_pump_mask, 1 - pump_enable_tbl[3]);

			if(unsaved_changes_exist == false) unsaved_changes_exist = true;
		}
        else if(control_id == 14) 		//pump5
		{			
			pump_enable_tbl[4] = state;
			pump_usablilty_tbl[4] = state;
			RefreshButtons();
			
			SetControlVisiable(SCREEN_MAIN_1,PumpIcons[4].icon_pump_mask, 1 - pump_enable_tbl[4]);

			if(unsaved_changes_exist == false) unsaved_changes_exist = true;
		}
        else if(control_id == 7) 		//pump6
		{			
			pump_enable_tbl[5] = state;
			pump_usablilty_tbl[5] = state;
			RefreshButtons();
			
			SetControlVisiable(SCREEN_MAIN_1,PumpIcons[5].icon_pump_mask, 1 - pump_enable_tbl[5]);

			if(unsaved_changes_exist == false) unsaved_changes_exist = true;
		}
	    else if(control_id == BTN_FIXED_FC_PUMP_MODE) //FIXED_FC_PUMP_MODE
	    {
			Fixed_FC_Pump = state;
			printf("Fixed_FC_Pump = %d\n",Fixed_FC_Pump);
			PumpGroupMode = state + 1;		//f-fc selected: Mode2, f-fc unselected: Mode1
			if(unsaved_changes_exist == false) unsaved_changes_exist = true;
	    }
	    else if((control_id == BTN_BACK_SCREEN_PUMP_GROUPING) && (state == ON))	  //【返回】
	    {
			uint8 nbr;
			nbr = nbr_of_enabled_pump();
		
		    if(NUMBER_OF_SELECTED_MAIN_PUMPS == 0)
			{
				//At least 1 main pump must be selected
				SetControlVisiable(SCREEN_PUMP_GROUPING, ICON_INPUT_ERROR1, VISIBLE);
			}
			else if(Max_Pump_Nbr > NUMBER_OF_SELECTED_MAIN_PUMPS)
			{
			    //Max should not greater than nbr of selected main pumps
			    SetControlVisiable(SCREEN_PUMP_GROUPING, ICON_INPUT_ERROR2, VISIBLE);
			}
			else if((Fixed_FC_Pump == ON) && (pump_enable_tbl[0] == OFF))
			{
				//pump 1 must be selected in Fixed-Fc mode
				SetControlVisiable(SCREEN_PUMP_GROUPING, ICON_INPUT_ERROR3, VISIBLE);
			}	
			else //error free
			{
				if(unsaved_changes_exist) 
				{							
				    SaveSettingsToFlash(SCREEN_PUMP_GROUPING);
					unsaved_changes_exist = false;
				    printf("    SaveSettingsToFlash(SCREEN_PUMP_GROUPING);\n");
				
					//hide all warnings
					SetControlVisiable(SCREEN_PUMP_GROUPING, ICON_INPUT_ERROR1, INVISIBLE);
					delay_ms(100);
					SetControlVisiable(SCREEN_PUMP_GROUPING, ICON_INPUT_ERROR2, INVISIBLE);
					delay_ms(100);
					SetControlVisiable(SCREEN_PUMP_GROUPING, ICON_INPUT_ERROR3, INVISIBLE);
					delay_ms(100);	
	
				}
				//jump to the parent screen
				SetScreen(SCREEN_USER_SETTINGS_2);		
			}
	    }
	}		
/*******************************************SCREEN_OUTLET_SENSOR【出口传感器】*******************************************/
	else if(screen_id == SCREEN_OUTLET_SENSOR)
	{
	     if((control_id == BTN_OUTLET_SENSOR_RANGE1) && (state == ON))
	     {
		 	Outlet_Sensor_type = OUTLET_SENSOER_TYPE_1;	 //PM
			if(unsaved_changes_exist == false) unsaved_changes_exist = true;
	     }
	     else if((control_id == BTN_OUTLET_SENSOR_RANGE2) && (state == ON))
	     {
		 	Outlet_Sensor_type = OUTLET_SENSOER_TYPE_2;
			if(unsaved_changes_exist == false) unsaved_changes_exist = true;
	     }
	     else if(control_id == BTN_OUTLET_LOW_PRESSURE_PROTECTION)
	     {
		 	Outlet_LP_Protection_Selected = state;
			if(unsaved_changes_exist == false) unsaved_changes_exist = true;
			printf("Outlet_LP_Protection_Selected = %d\n",Outlet_LP_Protection_Selected);


	     }
	     else if(control_id == BTN_OUTLET_HIGH_PRESSURE_PROTECTION)
	     {
		 	Outlet_HP_Protection_Selected = state;
			if(unsaved_changes_exist == false) unsaved_changes_exist = true;
			printf("Outlet_HP_Protection_Selected = %d\n",Outlet_HP_Protection_Selected);


	     }
	     else if((control_id == BTN_BACK_SCREEN_OUTLET_SENSOR) && (state == ON))
	     {
		 	if(unsaved_changes_exist) 
			{							
			    SaveSettingsToFlash(SCREEN_OUTLET_SENSOR);
				unsaved_changes_exist = false;
			    printf("    SaveSettingsToFlash(SCREEN_PUMP_GROUPING);\n");

			}	
	     }
	     else if((control_id == 10) && (state == ON))
	     {			
			Outlet_Pressure_Bias = Outlet_Pressure_Bias - OutletRealTimePressureForCali;
			SetBias(Outlet_Pressure_Bias, IND_CH_OUTLET_PRESSURE);
			sprintf(buff, "%.2f", Outlet_Pressure_Bias);
			SetTextValue(SCREEN_OUTLET_SENSOR, 6, (uchar*)buff); 
			if(unsaved_changes_exist == false) unsaved_changes_exist = true;
	     }
		 printf("Outlet_Sensor_type = %d\n",Outlet_Sensor_type);


		 

	}
/*******************************************SCREEN_OUTLET_CALI【出口压力校准】*******************************************/
	else if(screen_id == SCREEN_OUTLET_CALI)
	{
	     if ((control_id == BTN_OUTLET_ZERO_CALI) && (state == ON))
	     {			
			Outlet_Pressure_Bias = Outlet_Pressure_Bias - OutletRealTimePressureForCali;
			SetBias(Outlet_Pressure_Bias, IND_CH_OUTLET_PRESSURE);
			sprintf(buff, "%.2f", Outlet_Pressure_Bias);
			SetTextValue(SCREEN_OUTLET_CALI, TXT_OUTLET_PRESSURE_CALI_BIAS, (uchar*)buff); 
			if(unsaved_changes_exist == false) unsaved_changes_exist = true;
	     }
	     else if ((control_id == BTN_OUTLET_FULL_RANGE_CALI) && (state == ON))
	     {			
			OutletPressureCaliCoeff = OutletPressureCaliCoeff * Outlet_Sensor_Range / OutletRealTimePressureForCali;
			sprintf(buff, "%.2f", OutletPressureCaliCoeff);
			SetTextValue(SCREEN_OUTLET_CALI, TXT_OUTLET_PRESSURE_CALI_COEFF, (uchar*)buff); 
			if(unsaved_changes_exist == false) unsaved_changes_exist = true;
	     }
	}
/*******************************************SCREEN_ENTRANCE_CALI【入口压力校准】*******************************************/
	else if(screen_id == SCREEN_ENTRANCE_CALI)
	{
	     if ((control_id == BTN_ENTRANCE_ZERO_CALI) && (state == ON))
	     {			
			Entrance_Pressure_Bias = Entrance_Pressure_Bias - EntranceRealTimePressureForCali;
			SetBias(Entrance_Pressure_Bias, IND_CH_ENTRANCE_PRESSURE);
			sprintf(buff, "%.2f", Entrance_Pressure_Bias);
			SetTextValue(SCREEN_ENTRANCE_CALI, TXT_ENTRANCE_PRESSURE_CALI_BIAS, (uchar*)buff); 
			if(unsaved_changes_exist == false) unsaved_changes_exist = true;
	     }
	     else if ((control_id == BTN_ENTRANCE_FULL_RANGE_CALI) && (state == ON))
	     {			
  	    	EntrancePressureCaliCoeff = EntrancePressureCaliCoeff * Entrance_Sensor_Range / EntranceRealTimePressureForCali;
			sprintf(buff, "%.2f", EntrancePressureCaliCoeff);
			SetTextValue(SCREEN_ENTRANCE_CALI, TXT_ENTRANCE_PRESSURE_CALI_COEFF, (uchar*)buff); 
			if(unsaved_changes_exist == false) unsaved_changes_exist = true;
	     }
	}

/*******************************************SCREEN_ENTRANCE_SENSOR【入口传感器】*******************************************/
	else if(screen_id == SCREEN_ENTRANCE_SENSOR)
	{
	     if((control_id == BTN_ENTRANCE_SENSOR_INPUT1_SELECTED) && (state == ON))
	     {
		 	Entrance_Sensor_type = ENTRANCE_SENSOR_TYPE_1;
			if(unsaved_changes_exist == false) unsaved_changes_exist = true;
	     }
	     else if((control_id == BTN_ENTRANCE_SENSOR_INPUT2_SELECTED) && (state == ON))
	     {
		 	Entrance_Sensor_type = ENTRANCE_SENSOR_TYPE_2;
			if(unsaved_changes_exist == false) unsaved_changes_exist = true;
	     }
	     else if((control_id == BTNE_JOINT_PM_SELECTED) && (state == ON))
	     {
		 	Entrance_Sensor_type = ENTRANCE_SENSOR_TYPE_3;
			SetTextValue(SCREEN_MAIN_1, TXT_ENTRANCE_PRESSURE, "--");
			delay_ms(100);
			SetTextValue(SCREEN_MANUAL_CTR_2, TXT_MANUAL_ENTRANCE_PRESSURE, "--"); 		  //20200319


			if(unsaved_changes_exist == false) unsaved_changes_exist = true;
	     }
	     else if((control_id == BTN_BACK_SCREEN_ENTRANCE_SENSOR ) && (state == ON))
	     {
		 	if(unsaved_changes_exist) 
			{							
			    SaveSettingsToFlash(SCREEN_ENTRANCE_SENSOR);
				unsaved_changes_exist = false;
			    printf("    SaveSettingsToFlash(SCREEN_ENTRANCE_SENSOR);\n");
			}	
	     }
	}
/*******************************************SCREEN_PUMP_SWITCH_CONDTION【切泵条件】*******************************************/
	else if(screen_id == SCREEN_PUMP_SWITCH_CONDTION)
	{
	     if((control_id == BTN_BACK_SCREEN_PUMP_SWITCH_CONDTION) && (state == ON))
	     {
		 	if(unsaved_changes_exist) 
			{							
			    SaveSettingsToFlash(SCREEN_PUMP_SWITCH_CONDTION);
				unsaved_changes_exist = false;
			    printf("    SaveSettingsToFlash(SCREEN_PUMP_SWITCH_CONDTION);\n");
			}	

	     }
	}
/*******************************************SCREEN_SLEEP_SETTING【休眠设置】*******************************************/	
	else if(screen_id == SCREEN_SLEEP_SETTING)
	{
	     if((control_id == BTN_BACK_SCREEN_SLEEP_SETTING) && (state == ON))
	     {
		 	if(unsaved_changes_exist) 
			{							

			    SaveSettingsToFlash(SCREEN_SLEEP_SETTING);
				unsaved_changes_exist = false;
			    printf("    SaveSettingsToFlash(SCREEN_SLEEP_SETTING);\n");
			}
	     }	   
		 else if(control_id == 9)
		 {
			 SleepingEnableMode = 0;
			 if(unsaved_changes_exist == false) unsaved_changes_exist = true;
		 }
		 else if(control_id == 10)
		 {
			 SleepingEnableMode = 1;
			 if(unsaved_changes_exist == false) unsaved_changes_exist = true;
		 }
		 else if(control_id == 11)
		 {
			 SleepingEnableMode = 2;
			 if(unsaved_changes_exist == false) unsaved_changes_exist = true;
		 }
		 printf("SleepingEnableMode = %d\n",SleepingEnableMode);

	}
/*******************************************SCREEN_POWER_UP_SETTING【上电运行】*******************************************/
	else if(screen_id == SCREEN_POWER_UP_SETTING)
	{
	     if((control_id >= 1) && (control_id <= 7))	 //1-7
		 {
		 	 Power_Up_Setting[control_id - 1] = state;	//0-6
			 if(unsaved_changes_exist == false) unsaved_changes_exist = true;
		 }	
	     if (control_id == 8)	   //pump4
		 {
		 	if (state)	
				Power_Up_Setting[7] |=  0x04;	 
		 	else
				Power_Up_Setting[7] &=  0xfb; 

			 if(unsaved_changes_exist == false) unsaved_changes_exist = true;
		 }
	     if (control_id == 9)	   //pump5
		 {
		 	if (state)	
				Power_Up_Setting[7] |=  0x02;	 
		 	else
				Power_Up_Setting[7] &=  0xfd; 

			 if(unsaved_changes_exist == false) unsaved_changes_exist = true;
		 }
	     if (control_id == 10)	   //pump6
		 {
		 	if (state)	
				Power_Up_Setting[7] |=  0x01;	 
		 	else
				Power_Up_Setting[7] &=  0xfe; 

			 if(unsaved_changes_exist == false) unsaved_changes_exist = true;
		 }
	     else if (control_id == BTN_USE_60HZ_AS_MAX_FREQUENCY)
		 {
		 	 Use60HzAsMaxFreq = state;
			 printf("Use60HzAsMaxFreq = %d\n",Use60HzAsMaxFreq);

			 if(unsaved_changes_exist == false) unsaved_changes_exist = true;
		 }
		 //
	     else if (control_id == BTN_AVF_MODE)
		 {
		 	 AvfMode = state;

		 	 //Show/Hide Fixed-FC mode selection
			 SetControlVisiable(SCREEN_PUMP_GROUPING, ICON_BLUE_MASK, AvfMode);
			 SetControlEnable(SCREEN_PUMP_GROUPING, BTN_FIXED_FC_PUMP_MODE, 1 - AvfMode);
			 if(unsaved_changes_exist == false) unsaved_changes_exist = true;
		 }
		 else if((control_id == BTN_BACK_SCREEN_POWER_UP_SETTING) && (state == ON))	   //Return
	     {
		 	if(unsaved_changes_exist) 
			{							
			    SaveSettingsToFlash(SCREEN_POWER_UP_SETTING);
				MaxFreqOut = (Use60HzAsMaxFreq == TRUE)?60:50;
				printf("MaxFreqOut = %dHz\n",MaxFreqOut);


				unsaved_changes_exist = false;
			    printf("    SaveSettingsToFlash(SCREEN_POWER_UP_SETTING);\n");
///////////////////////////////////
			/*	printf("PUMP1_DEFECT_SC_EFFECTIVE: %d\n", PUMP1_DEFECT_SC_EFFECTIVE);    
 			    printf("PUMP2_DEFECT_SC_EFFECTIVE: %d\n", PUMP2_DEFECT_SC_EFFECTIVE);    
				printf("PUMP3_DEFECT_SC_EFFECTIVE: %d\n", PUMP3_DEFECT_SC_EFFECTIVE);    
				printf("PUMP4_DEFECT_SC_EFFECTIVE: %d\n", PUMP4_DEFECT_SC_EFFECTIVE);    
				printf("PUMP5_DEFECT_SC_EFFECTIVE: %d\n", PUMP5_DEFECT_SC_EFFECTIVE);    
				printf("PUMP6_DEFECT_SC_EFFECTIVE: %d\n", SMALL_PUMP_DEFECT_SC_EFFECTIVE);    */
 

////////////////////////////////
			}
		 	
	     }
		 else if((control_id ==  46) && (state == ON)) 	  
		 {
		 	SetVisibiltyBumpTemp();
		 }
	}
/*******************************************SCREEN_DEVICE_ADDRESS【本机地址】*******************************************/
	else if(screen_id == SCREEN_DEVICE_ADDRESS)
	{
		 if((control_id == 2) && (state == ON)) //RETURN
	     {
		 	if(unsaved_changes_exist) 
			{							
 #ifndef FIXED_DEVICE_ADDRESS
		 		SaveDeviceAddress();
 #endif
				unsaved_changes_exist = false;
			    printf("    SaveSettingsToFlash( SCREEN_DEVICE_ADDRESS);\n");
			}		 	
	     }
	}
	//
/*******************************************SCREEN_VALVE_CONTROL【阀门控制】*******************************************/
	else if(screen_id == SCREEN_VALVE_CONTROL)
	{
	     if((control_id == BTN_BACK_SCREEN_VALVE_CONTROL) && (state == ON))
	     {
		 	if(unsaved_changes_exist) 
			{							
			    SaveSettingsToFlash(SCREEN_VALVE_CONTROL);
				unsaved_changes_exist = false;
			    printf("    SaveSettingsToFlash(SCREEN_VALVE_CONTROL);\n");
			}

	     }
		 else if((control_id >= 1) && (control_id <= 2))
	     {
			  Valve_Control[control_id - 1] = state;
			  if(state == OFF)
			  {
				  if(control_id == 1) 
				  {
					ValveIntoTank = OFF;
				  }
				  else
				  {
					ValveDecompression = OFF;
				  }
			  }
			  if(unsaved_changes_exist == false) unsaved_changes_exist = true;

	     }
	}	
/*******************************************SCREEN_PID_SETTING【PID设置】*******************************************/
	else if(screen_id == SCREEN_PID_SETTING)
	{
	     if((control_id == BTN_BACK_PID_SETTING) && (state == ON))
	     {
		 	if(unsaved_changes_exist) 
			{							
			    SaveSettingsToFlash(SCREEN_PID_SETTING);
				save_target_locker(TargetLockerTiming);
				save_target_locker_delta(TargetLockerDelta); 


				unsaved_changes_exist = false;
			    printf("    SaveSettingsToFlash(SCREEN_PID_SETTING);\n");
			}


	     }
	     else if(control_id == BTN__SELF_DEF_PID_PAR)
	     {
			 PID_Setting[0] = (PID_Setting[0] / 10) * 10 + state;
			 unsaved_changes_exist = true;
	     }
	}
/*******************************************SCREEN_ENTRANCE_SENSOR0【入口传感器0】*******************************************/
	else if(screen_id == SCREEN_ENTRANCE_SENSOR0 )
	{
	     if((control_id == BTN_BACK_SCREEN_ENTRANCE_SENSOR0 ) && (state == ON))
	     {
 		 	if(unsaved_changes_exist) 
			{							
			    SaveSettingsToFlash(SCREEN_ENTRANCE_SENSOR0);
				unsaved_changes_exist = false;
			    printf("    SaveSettingsToFlash(SCREEN_ENTRANCE_SENSOR0);\n");

			}
	     }
		 else if((control_id == 1) && (state == ON))	//floating ball
	     {
			  Entrance_sensor0[0] = (Entrance_sensor0[0] & 0xf0) | 0x01;
			  
			  if(state == ON)
			  {
				    printf("******************************floating ball is selcted\n");
					push_cmd_pump(CMD_SET_TEXT, SCREEN_MAIN_1, TXT_WATER_LEVEL, TXT_VALUE_FOR_NUL);
					push_cmd_pump(CMD_SET_TEXT, SCREEN_MANUAL_CTR_2, TXT_TANK_WATER_LEVEL, TXT_VALUE_FOR_NUL);

					push_cmd_pump(CMD_SET_PROGRESS, SCREEN_MAIN_1, SCREEN_MAIN_WATER_TANK, WL_DEFAULT_PERCENTAGE); 
			  }
			  unsaved_changes_exist = true;


	     }
		 else if(((control_id == 2) || (control_id == 3)) && (state == ON))
	     {
		 	  Entrance_sensor0[0] &= 0xf0;	
			  Entrance_sensor0[1] = control_id - 2;
			  unsaved_changes_exist = true;
	     }
		 else if(((control_id >= 10) && (control_id <= 12)) && (state == ON))
	     {
		 	  Entrance_sensor0[0] = ((control_id - 10) << 4) | (Entrance_sensor0[0] & 0x0f) ;	
			  printf("Entrance_sensor0[0] = %02x\n",Entrance_sensor0[0]);

			  if(control_id == 10) SetTextValue(SCREEN_MAIN_1, 10, "(mm)");
			  else if(control_id == 11) SetTextValue(SCREEN_MAIN_1, 10, "(cm)");
			  else if(control_id == 12) SetTextValue(SCREEN_MAIN_1, 10, "(m)");
			  unsaved_changes_exist = true;
	     }
	}

	
/*******************************************SCREEN_ENTRANCE_SENSOR2【入口传感器2】*******************************************/

	else if(screen_id == SCREEN_ENTRANCE_SENSOR2 )
	{
	     if((control_id == BTN_BACK_SCREEN_ENTRANCE_SENSOR2 ) && (state == ON))
	     {
 		 	if(unsaved_changes_exist) 
			{							
			    SaveSettingsToFlash(SCREEN_ENTRANCE_SENSOR2);
				unsaved_changes_exist = false;
			    printf("    SaveSettingsToFlash(SCREEN_ENTRANCE_SENSOR2);\n");
			}

	     }
		 else if(((control_id == 1) || (control_id == 2)) && (state == ON))
	     {
			  Entrance_sensor0[1] = control_id - 1;
			  unsaved_changes_exist = true;
	     }
	}
/*******************************************SCREEN_VALVE_CONTROL1【阀门控制1】*******************************************/
	else if(screen_id == SCREEN_VALVE_CONTROL1)
	{
	     if((control_id == BTN_BACK_SCREEN_VALVE_CONTROL1) && (state == ON))
	     {
		 	if(unsaved_changes_exist) 
			{							
			    SaveSettingsToFlash(SCREEN_VALVE_CONTROL1);
				unsaved_changes_exist = false;
			    printf("    SaveSettingsToFlash(SCREEN_VALVE_CONTROL1);\n");
			}

	     }
	}	
/*******************************************SCREEN_CONTROL_CURVES【控制曲线】*******************************************/
	else if(screen_id == SCREEN_CONTROL_CURVES)
	{
	     if((control_id == 7) && (state == ON))
	     {
		 	GraphChannelDataClear(SCREEN_CONTROL_CURVES,1,0);
			delay_ms(100);
			GraphChannelDataClear(SCREEN_CONTROL_CURVES,1,1);
			delay_ms(100);
			GraphChannelDataClear(SCREEN_CONTROL_CURVES,1,2);
	     }
	}	
/*******************************************SCREEN_TRAIL_DATE_SETTING【试用期设置3】*******************************************/	
	else if(screen_id == SCREEN_TRAIL_DATE_SETTING)
	{	

	    if((control_id == 14) && (state == ON))	//Return button			
	    {
			if(unsaved_changes_exist) 
			{	
				if(DateCorrection(&StopDate)) RefreshDateControls();
				ExpireDateManager();	
								
			    SaveExpireDate(StopDate.year, StopDate.month, StopDate.day, StopDate.disable_manual, StopDate.enable_expire_date);
				unsaved_changes_exist = false;
			    printf("    SaveExpireDate(StopDate.year, StopDate.month, StopDate.day);\n");
			}	     
		}
		else if(control_id == 1)
		{
			StopDate.disable_manual = state;

			RefreshButtons();

			if(unsaved_changes_exist == false) unsaved_changes_exist = true;
		}
		else if(control_id == 2)	  //Enable trial control
		{
			StopDate.enable_expire_date = state;
		//	ExpireDateManager();
			if(unsaved_changes_exist == false) unsaved_changes_exist = true;
		}
	}	
/*******************************************SCREEN_PUMP_TEM_CONTROL【泵温监控】*******************************************/
/*temp_limit[4] = {60.0, 60.0, 60.0, 60.0};
float temp_bias[4] = {0.0, 0.0, 0.0, 0.0}; */
	else if(screen_id == SCREEN_PUMP_TEM_CONTROL)
	{	
	     if((control_id == BTN_BACK_SCREEN_PUMP_TEM_CONTROL) && (state == ON))
	     {
 		 	if(unsaved_changes_exist) 
			{							
			    save_pump_temp_setting();
				unsaved_changes_exist = false;
			    printf("    SaveSettingsToFlash(SCREEN_ENTRANCE_SENSOR2);\n");
			}

	     }
	     else if(control_id == 26)
	     {
		 	TempMonitorEnable = state;
			SetVisibiltyBumpTemp();
			printf("TempMonitorEnable= %d\n", TempMonitorEnable);
			if(unsaved_changes_exist == false) unsaved_changes_exist = true;
	     }

		 


	}
/*******************************************SCREEN_RTC_SETTING【系统时钟设置】*******************************************/
	else if(screen_id == SCREEN_RTC_SETTING)
	{	

	    if((control_id == 1) && (state == ON))	//Return button			
	    {
			ExpireDateManager();   
		}
	}
	else if(screen_id == 33)	 //Debugger screen
	{	
	    if((control_id == 27) && (state == ON))	//Return button			
	    {  
			debugger("", OFF);	   //clear dbg screen
		}
#ifdef DBG_USE_SCREEN_COPY	
	    if((control_id == 33) && (state == ON))	//Return button			
	    {  
			SaveScreenCopy(50);
		}
	    if((control_id == 34) && (state == ON))	//Return button			
	    {  
			LoadScreenCopy(50);
		}
#endif	   
	}	
#ifdef DBG_FUNC_INFO
/*******************************************SCREEN_FUNC_DBG_SWITCHES【函数调试开关】*******************************************/
	else if(screen_id == SCREEN_FUNC_DBG_SWITCHES)
	{	

	    if((control_id >= 7) && (control_id <= 9))	//Return button			
	    {
			FuncSwitches[control_id - 7] = state; 
			printf("FuncSwitches[]: [%d %d %d]\n\n", FuncSwitches[0], FuncSwitches[1], FuncSwitches[2]);
		}
	}
	 

#endif			                                                                                    				 			  	  
}


//
/*! 
*  \brief  文本控件通知
*  \details  当文本通过键盘更新(或调用GetControlValue)时，执行此函数
*  \details  文本控件的内容以字符串形式下发到MCU，如果文本控件内容是浮点值，
*  \details  则需要在此函数中将下发字符串重新转回浮点值。
*  \param screen_id 画面ID
*  \param control_id 控件ID
*  \param str 文本控件内容
*/
extern float temp_restore;
extern float SpeedUpFactor;	 //20210712
extern float PumpMaxFreq;	//20210911
void NotifyText(uint16 screen_id, uint16 control_id, uint8 *str)
{
	/******************SCREEN_TIME_CTR_3_【时控开关】****************************************/ 
    if(screen_id == SCREEN_TIME_CTR_3)	   
    {
		uint8 row, column;
		int value;
		float pressure_value;
	    char buff[5]; 
		                                                      
        sscanf((char*)str,"%ld", &value);    
		row =  (control_id - 16)/4;
		column =  (control_id - 16)%4 + 1;
		{	
			if((control_id >= 16 ) && (control_id <= 39))   //
			{
				 UpdatScheduleCell(value, row, column);
				 printf("   UpdatScheduleCell(value, row, column);\n");

				 if(unsaved_changes_exist == false) unsaved_changes_exist = true;

				 printf("schedules[%d].time_cell[%d] = %d\n\n", row, column, value);
				 if(column % 2 == 0) 	 //always 2 digits for minute
				 {
				     sprintf(buff, "%02d",  value);
				     SetTextValue(SCREEN_TIME_CTR_3, control_id, (uchar*)buff);
				 }
			}
			else if((control_id >= 48 ) && (control_id <= 53))   //
			{                                   
        		sscanf((char*)str,"%f",&pressure_value);
				tc_target_pressure[control_id - 48] = pressure_value;
				if(unsaved_changes_exist == false) unsaved_changes_exist = true;
				printf("tc_target_pressure[%d] = %.2f\n", control_id - 48, tc_target_pressure[control_id - 48]);
			}
			else if(control_id == TXT_TC_DEFAULT_PRESSURE)  //
			{                                   
        		sscanf((char*)str,"%f",&pressure_value);
				DefaultTargetPressureMenu = pressure_value;

				if (UseIoTargetPressure == FALSE) DefaultTargetPressure = DefaultTargetPressureMenu;
				/*This might affect Target_Pressure, which needs to be refreshed here 20210703*/
				RefreshTarget();	//20210707

			//	PidSetTargetPressure(DefaultTargetPressure);
			//	if(!RemoteMode) SetTextValue(SCREEN_MAIN_1, TXT_TARGET_PRESSURE, str);	deleted on 20210703
				if(unsaved_changes_exist == false) unsaved_changes_exist = true;
				printf("tc_target_pressure[%d] = %.2f\n", control_id - 48, tc_target_pressure[control_id - 48]);
			} 
			else if(control_id == TXT_TC_REMOTE_PRESSURE)  //
			{                                   
        		sscanf((char*)str,"%f",&pressure_value);
				RemoteTargetPressure = pressure_value;

				/*This might affect Target_Pressure, which needs to be refreshed here 20210703*/
				RefreshTarget();             //20210707
				/*
				PidSetTargetPressure(RemoteTargetPressure);
				if(RemoteMode) SetTextValue(SCREEN_MAIN_1, TXT_TARGET_PRESSURE, str);  */
				if(unsaved_changes_exist == false) unsaved_changes_exist = true;

			} 

		}
	
	}

/*******************************************SCREEN_PIN_CONTROL【密码控制】*******************************************/
	else if(screen_id == SCREEN_PIN_CONTROL)	 
	{
		 uint32 value = 0, correct_pin;
		 if(control_id == TXT_PIN_INPUT) 
		 {		  	                                                           
        	sscanf((char*)str,"%ld",&value);  
			correct_pin = UserPin[0]*10000 + UserPin[1]*100 + UserPin[2];

		    printf("correct_pin = %d\n",correct_pin);
			if(correct_pin == value)  
			{
				pin_ok = TRUE;
#ifdef ENGLISH_VERSION
				SetTextValue(SCREEN_PIN_CONTROL, TXT_USER_PIN_REMINDER, "Correct password");
#else
				SetTextValue(SCREEN_PIN_CONTROL, TXT_USER_PIN_REMINDER, "密码正确");
#endif
				delay_ms(100);
			    SetScreen(ReturnScreenId);
				delay_ms(100);
#ifdef ENGLISH_VERSION
				SetTextValue(SCREEN_PIN_CONTROL, TXT_USER_PIN_REMINDER, "Please enter a 6-digit password");
#else
				SetTextValue(SCREEN_PIN_CONTROL, TXT_USER_PIN_REMINDER, "请输入6位密码");
#endif
				
				
				if(ReturnScreenId == SCREEN_MAIN_1) SetControlVisiable(SCREEN_PIN_CONTROL,14, VISIBLE);	  //Power-up checking

			}
			else
			{
#ifdef ENGLISH_VERSION
				SetTextValue(SCREEN_PIN_CONTROL, TXT_USER_PIN_REMINDER, "Incorrect password, please re-enter");
#else
				SetTextValue(SCREEN_PIN_CONTROL, TXT_USER_PIN_REMINDER, "密码错误！请重试");
#endif

			}		
			printf("--------------pin = %d\n", value);
		 }	
	} 
/*******************************************SCREEN_PIN_CONTROL_FACTORY【密码控制-厂家】*******************************************/
	else if(screen_id == SCREEN_PIN_CONTROL_FACTORY)	 
	{
		 uint32 value = 0, correct_pin;
#ifdef USE_PIN_GENERATOR
		 char buff[30];
		 static uint8 error_counter = 0;
		 uint32 unlock_pin;
		 unlock_pin = (int)(pow(PW_ErrIndex * 973, 2) + 13) % 1000000;
#endif
		 if(control_id == TXT_PIN_INPUT) 
		 {		  	                                                           
        	sscanf((char*)str,"%ld",&value);  
			correct_pin = FactoryPin[0]*10000 + FactoryPin[1]*100 + FactoryPin[2];

		    printf("correct_pin = %d\n",correct_pin);

#ifdef USE_PIN_GENERATOR
			if((correct_pin == value) || (unlock_pin == value)) 
#else
			if(correct_pin == value) 
#endif 
			{
#ifdef USE_PIN_GENERATOR
		        error_counter = 0;
				if(unlock_pin == value)			 //successfully unlocked, point at next error index
				{
				   PW_ErrIndex++;
				   if(PW_ErrIndex > 10) PW_ErrIndex = 1;
				   SavePW_ErrIndex(PW_ErrIndex);			//sycronize value in flash with that in RAM

				} 
#endif
				SetScreen(ReturnScreenId);
				delay_ms(100);
#ifdef ENGLISH_VERSION
				SetTextValue(SCREEN_PIN_CONTROL_FACTORY, TXT_USER_PIN_REMINDER, "Please enter a 6-digit password");		   
#else
				SetTextValue(SCREEN_PIN_CONTROL_FACTORY, TXT_USER_PIN_REMINDER, "请输入6位密码");
#endif	
			}
			else
			{
#ifdef USE_PIN_GENERATOR
		        error_counter++;
				if(error_counter >= 3) 
				{
#ifdef ENGLISH_VERSION
					sprintf(buff, "The 3 inputs are all incorrect,error code E0%d, please contact customer serive", PW_ErrIndex);
#else
					sprintf(buff, "连错3次,错误码 E0%d, 请联系客服", PW_ErrIndex);
#endif
					SetTextValue(SCREEN_PIN_CONTROL_FACTORY, TXT_USER_PIN_REMINDER, (uchar*)buff);
					delay_ms(100);
					SetControlEnable(SCREEN_PIN_CONTROL_FACTORY, TXT_PIN_INPUT, FALSE);
				}
				else
				{
#endif

#ifdef ENGLISH_VERSION
					SetTextValue(SCREEN_PIN_CONTROL_FACTORY, TXT_USER_PIN_REMINDER, "Incorrect password！Please re-enter");
#else
					SetTextValue(SCREEN_PIN_CONTROL_FACTORY, TXT_USER_PIN_REMINDER, "密码错误！请重试");
#endif

#ifdef USE_PIN_GENERATOR
				}
#endif
			}
		
			printf("--------------pin = %d\n", value);
		 }	
	} 
/*******************************************SCREEN_PIN_SETTING_3【密码设置-3】*******************************************/
	else if(screen_id == SCREEN_PIN_SETTING_3)	 
	{
	    if(control_id == TXT_NEW_PIN_INPUT)
		{
			PinMonitor((char*)str);
		}
	}  
/*******************************************SCREEN_TIME_SETTING_3【时间设置-3】*******************************************/
/*	else if(screen_id == SCREEN_TIME_SETTING_3)	 
	{
		 int value = 0;
		 if(control_id == TXT_MILEAGE_LIMIT) 
		 {		  	                                                           
        	sscanf((char*)str,"%ld",&value);  
			MileageLimit = value;
		    printf("MileageLimit = %d\n",MileageLimit);
			if(unsaved_changes_exist == false) unsaved_changes_exist = true;
		 }
	}	*/ 
/*******************************************SCREEN_SCREEN_SETTING【屏幕设置-3】*******************************************/	
	else if(screen_id == SCREEN_SCREEN_SETTING)	 
	{
 
		 int value = 0;
		 sscanf((char*)str,"%ld",&value); 
 
		 if(control_id == TXT_BL_ON_TIME) 
		 {		  	                                                                  	 
			BL_On_Time = value;
		    printf("BL_On_Time = %d min\n",BL_On_Time);
		 }
		 if(unsaved_changes_exist == false) unsaved_changes_exist = true;

	}				  	  
/*******************************************SCREEN_PUMP_GROUPING【组泵方式】*******************************************/
	else if(screen_id == SCREEN_PUMP_GROUPING)	 
	{
		int i_value;   

	    if(control_id == TXT_MAX_PUMP_NBR)
	    {
			sscanf((char*)str,"%d",&i_value); 
			Max_Pump_Nbr = i_value;
			printf("Max_Pump_Nbr = %d\n",Max_Pump_Nbr);

			if(unsaved_changes_exist == false) unsaved_changes_exist = true;
	    }
	} 
		    
/*******************************************SCREEN_OUTLET_SENSOR【出口传感器】*******************************************/
	else if(screen_id == SCREEN_OUTLET_SENSOR)
	{
		 float f_value;

	     if(control_id == TXT_OUTLET_SENSOR_RANGE)
	     {
		 	sscanf((char*)str,"%f",&f_value); 
			Outlet_Sensor_Range = f_value;
			printf("Outlet_Sensor_Range = %f\n",Outlet_Sensor_Range);
			if(unsaved_changes_exist == false) unsaved_changes_exist = true;


	     }
	     else if(control_id == TXT_OUTLET_PRESSURE_BIAS)
	     {
			sscanf((char*)str,"%f",&f_value); 
			Outlet_Pressure_Bias = f_value;
			SetBias(f_value, IND_CH_OUTLET_PRESSURE);
			printf("Outlet_Pressure_Bias = %f\n",Outlet_Pressure_Bias);

		 	if(unsaved_changes_exist == false) unsaved_changes_exist = true;
	     }
	     else if(control_id == TXT_OUTLET_LP_PROTECTION_VALUE)
	     {
			sscanf((char*)str,"%f",&f_value); 
			Outlet_LP_Protection_Value = f_value;
			printf("Outlet_LP_Protection_Value = %f\n",Outlet_LP_Protection_Value);


		 	if(unsaved_changes_exist == false) unsaved_changes_exist = true;

	     }
	     else if(control_id == TXT_OUTLET_HP_PROTECTION_VALUE)
	     {
			sscanf((char*)str,"%f",&f_value); 
			Outlet_HP_Protection_Value = f_value;
			printf("Outlet_HP_Protection_Value = %f\n",Outlet_HP_Protection_Value);


		 	if(unsaved_changes_exist == false) unsaved_changes_exist = true;

	     }
	}	
/*******************************************SCREEN_OUTLET_CALI【出口压力校准】*******************************************/
	else if(screen_id == SCREEN_OUTLET_CALI)
	{
		 float f_value;
	     if (control_id == TXT_OUTLET_PRESSURE_CALI_BIAS)
	     {
			sscanf((char*)str,"%f",&f_value); 
			Outlet_Pressure_Bias = f_value;
			SetBias(f_value, IND_CH_OUTLET_PRESSURE);
			printf("Outlet_Pressure_Bias = %f\n",Outlet_Pressure_Bias);

		 	if(unsaved_changes_exist == false) unsaved_changes_exist = true;
	     }
	     if (control_id == TXT_OUTLET_PRESSURE_CALI_COEFF)
	     {
			sscanf((char*)str,"%f",&f_value); 
			OutletPressureCaliCoeff = f_value;

			printf("OutletPressureCaliCoeff = %f\n",OutletPressureCaliCoeff);

		 	if(unsaved_changes_exist == false) unsaved_changes_exist = true;
	     }
	}
/*******************************************SCREEN_ENTRANCE_SENSOR【入口传感器】*******************************************/
	else if(screen_id == SCREEN_ENTRANCE_SENSOR )
	{
		 float f_value;
		 int i_value;   

	     if(control_id == TXT_ENTRANCE_SENSOR_RANGE)
	     {
		 	sscanf((char*)str,"%f",&f_value); 
			Entrance_Sensor_Range = f_value;
		 	if(unsaved_changes_exist == false) unsaved_changes_exist = true;

	     }
	     else if(control_id == TXT_ENTRANCE_PRESSURE_BIAS)
	     {
			sscanf((char*)str,"%f",&f_value); 
			Entrance_Pressure_Bias = f_value;
			SetBias(f_value, IND_CH_ENTRANCE_PRESSURE);
		 	if(unsaved_changes_exist == false) unsaved_changes_exist = true;

	     }
	     else if(control_id == TXT_ENTRANCE_LP_PROTECTION_DELAY)
	     {
			sscanf((char*)str,"%d",&i_value); 
			Entrance_LP_Protection_Delay = i_value;
		 	if(unsaved_changes_exist == false) unsaved_changes_exist = true;

	     }
	     else if(control_id == TXT_ENTRANCE_LP_PROTECTION_VALUE)
	     {
			sscanf((char*)str,"%f",&f_value); 
			Entrance_LP_Protection_Value = f_value;
		 	if(unsaved_changes_exist == false) unsaved_changes_exist = true;
	     }
	     else if(control_id == TXT_ENTRANCE_LP_PROTECT_RESTORE_VAL)
	     {
			sscanf((char*)str,"%f",&f_value); 
			Entrance_LP_Protection_Restore_Value = f_value;
		 	if(unsaved_changes_exist == false) unsaved_changes_exist = true;

	     }
	}
/*******************************************SCREEN_ENTRANCE_CALI【入口压力校准】*******************************************/
	else if(screen_id == SCREEN_ENTRANCE_CALI)
	{
		 float f_value;
	     if (control_id == TXT_ENTRANCE_PRESSURE_CALI_BIAS)
	     {
			sscanf((char*)str,"%f",&f_value); 
			Entrance_Pressure_Bias = f_value;
			SetBias(f_value, IND_CH_ENTRANCE_PRESSURE);
			printf("Entrance_Pressure_Bias = %f\n",Entrance_Pressure_Bias);

		 	if(unsaved_changes_exist == false) unsaved_changes_exist = true;
	     }
	     if (control_id == TXT_ENTRANCE_PRESSURE_CALI_COEFF)
	     {
			sscanf((char*)str,"%f",&f_value); 
			EntrancePressureCaliCoeff = f_value;

			printf("EntrancePressureCaliCoeff = %f\n",EntrancePressureCaliCoeff);

		 	if(unsaved_changes_exist == false) unsaved_changes_exist = true;
	     }
	}
/*******************************************SCREEN_PUMP_SWITCH_CONDTION【切泵条件】*******************************************/
	else if(screen_id == SCREEN_PUMP_SWITCH_CONDTION)
	{
		 float f_value;
		 int i_value;  
		  

	
	     if((control_id == 1) || (control_id == 2) || (control_id == 4) || (control_id == 7) || (control_id == 8) || (control_id == 9))
	     {
		 	sscanf((char*)str,"%d",&i_value); 
			
			Pump_Switch_Condtion[VarIndex_SCREEN_PUMP_SWITCH_CONDTION[control_id]] = i_value / 256;
			Pump_Switch_Condtion[VarIndex_SCREEN_PUMP_SWITCH_CONDTION[control_id] + 1] = i_value % 256;

		 	if(unsaved_changes_exist == false) unsaved_changes_exist = true;

	     }
	     else if((control_id == 3) || (control_id == 5) || (control_id == 6))
	     {
		 	sscanf((char*)str,"%d",&i_value); 
			Pump_Switch_Condtion[VarIndex_SCREEN_PUMP_SWITCH_CONDTION[control_id]] = i_value;

		 	if(unsaved_changes_exist == false) unsaved_changes_exist = true;

	     }
	     else if (control_id == TXT_PUMP_CANCEL_FREQ)
	     {
		 	sscanf((char*)str,"%d",&i_value); 
			PumpCancelFreq = i_value;
		 	if(unsaved_changes_exist == false) unsaved_changes_exist = true;
	     }
		 //20210911
	     else if (control_id == TXT_PUMP_MAX_FREQ)
	     {
		 	char buff[5];
		 	sscanf((char*)str,"%f",&f_value); 
			PumpMaxFreq = f_value;
			if (PumpMaxFreq > MaxFreqOut)
			{
			 	PumpMaxFreq = MaxFreqOut; //default
				
				sprintf(buff, "%.1f", PumpMaxFreq);
				SetTextValue(SCREEN_PUMP_SWITCH_CONDTION, TXT_PUMP_MAX_FREQ, (uchar*)buff);
			}
		 	if (unsaved_changes_exist == false) unsaved_changes_exist = true;
	     }
 		 else if(control_id == 10)
	     {
		 	sscanf((char*)str,"%f",&f_value); 
			Pump_Switch_Condtion[15] = (int)(f_value * 100);

		 	if(unsaved_changes_exist == false) unsaved_changes_exist = true;

	     }
	}	
/*******************************************SCREEN_SLEEP_SETTING【休眠设置】*******************************************/	
	else if(screen_id == SCREEN_SLEEP_SETTING)
	{
		 float f_value;
		 int i_value;  
		  

	
	     if((control_id == 1) || (control_id == 4))
	     {
		 	sscanf((char*)str,"%d",&i_value); 			
			Sleep_Setting[VarIndex_SCREEN_SLEEP_SETTING[control_id]] = i_value / 256;
			Sleep_Setting[VarIndex_SCREEN_SLEEP_SETTING[control_id] + 1] = i_value % 256;
		 	if(unsaved_changes_exist == false) unsaved_changes_exist = true;

	     }
	     else if((control_id == 3) || (control_id == 6))
	     {
		 	sscanf((char*)str,"%f",&f_value); 
			Sleep_Setting[VarIndex_SCREEN_SLEEP_SETTING[control_id]] = (int)(f_value * 100);

		 	if(unsaved_changes_exist == false) unsaved_changes_exist = true;

	     }
	     else
		 {
		 	sscanf((char*)str,"%d",&i_value); 
			Sleep_Setting[VarIndex_SCREEN_SLEEP_SETTING[control_id]] = i_value;

		 	if(unsaved_changes_exist == false) unsaved_changes_exist = true;

	     }		 
	}
/*******************************************SCREEN_POWER_UP_SETTING【上电运行】*******************************************/
	else if(screen_id == SCREEN_POWER_UP_SETTING)
	{
		 float f_value;
		 uint16 i_value;   


	     if(control_id == 10) 
	     {
		 	sscanf((char*)str, "%f", &f_value); 

			FcFreqOutBias = f_value ;

		 	if(unsaved_changes_exist == false) unsaved_changes_exist = true;

	     }
  		 else if(control_id == 11)
	     {
		 	sscanf((char*)str,"%d",&i_value); 
			Power_Up_Setting[8] = i_value / 256 ;
			Power_Up_Setting[11] = i_value % 256;
		 	if(unsaved_changes_exist == false) unsaved_changes_exist = true;

	     }
	}
/*******************************************SCREEN_VALVE_CONTROL【阀门控制】*******************************************/
	else if(screen_id == SCREEN_VALVE_CONTROL)
	{	
		 float f_value;
		 int i_value;   

	     if((control_id >= 3) && (control_id <= 4))				//23 45 
	     {
		 	  sscanf((char*)str,"%d",&i_value); 
			  Valve_Control[control_id * 2 - 4] = i_value / 256;
			  Valve_Control[control_id * 2 - 3] = i_value % 256;
			 
		 	if(unsaved_changes_exist == false) unsaved_changes_exist = true;



	     }
	     else if((control_id >= 5) && (control_id <= 6))				//6 7
	     {
		 	  sscanf((char*)str,"%f",&f_value); 
			  printf("f_value = %f\n",f_value);

			  Valve_Control[control_id + 1] = (int)(f_value * 100.0 + 0.1);
	
			  printf("Valve_Control[control_id + 1]  = %d\n",Valve_Control[control_id + 1] );

 
		 	if(unsaved_changes_exist == false) unsaved_changes_exist = true;

	
	     }	
	}  
/*******************************************SCREEN_PID_SETTING【PID设置】****************uint8 TargetLockerTiming;***************************/
	else if(screen_id == SCREEN_PID_SETTING)
	{
		int i_value; 
		float f_value;
		uint16* pid_arr[3] = {&pFactor, &iFactor, &dFactor};
		if(control_id == 2) 
		{
			sscanf((char*)str,"%f",&f_value); 
			PID_Setting[1] = (int)(f_value * 10);
			if(unsaved_changes_exist == false) unsaved_changes_exist = true;

		} 
		else if((control_id > 2) && (control_id <= 5))	  //PID factors
		{
			sscanf((char*)str,"%d",&i_value); 
			*(pid_arr[control_id - 3]) = i_value;
			printf("pid factors: %d,  %d, %d\n", pFactor, iFactor, dFactor);

			if(unsaved_changes_exist == false) unsaved_changes_exist = true;
		}
		else if(control_id == TXT_TARGET_LOCKER_TIMING)	  
		{
			sscanf((char*)str, "%d", &i_value); 
			TargetLockerTiming = i_value;
			printf("TargetLockerTiming: %d\n", TargetLockerTiming);
			if(unsaved_changes_exist == false) unsaved_changes_exist = true;
		}
		else if(control_id == TXT_TARGET_LOCKER_DELTA)	  
		{
			sscanf((char*)str, "%f", &f_value); 
			TargetLockerDelta = f_value;
			printf("TargetLockerDelta: %f\n", TargetLockerDelta);
			if(unsaved_changes_exist == false) unsaved_changes_exist = true;
		}
		else if(control_id == TXT_SPEED_UP_FACTOR)	  	 //20210712
		{
			sscanf((char*)str, "%f", &f_value); 
			SpeedUpFactor = f_value;
			printf("SpeedUpFactor: %.1f\n", SpeedUpFactor);
			if(unsaved_changes_exist == false) unsaved_changes_exist = true;
		}
	}
/*******************************************SCREEN_MANUAL_CTR_2【手动控制-2】*******************************************/
	if(screen_id == SCREEN_MANUAL_CTR_2)
    {
		float frequency;
		char buff[5];
		if(control_id == TXT_RUNNING_FREQUENCY)
		{
		    sscanf((char*)str,"%f",&frequency); 
		    if (frequency > MaxFreqOut) frequency = (float)MaxFreqOut;

		    TELL_FC_CONTROLLER_FREQ_CHANGE_IN_MANUAL_MODE

		   
			printf("********************************frequency = %0.1f\n",frequency);
			
			manual_freq_setting = frequency;
			
			sprintf(buff, "%.1f", frequency);
			
			SetTextValue(SCREEN_MANUAL_CTR_2, TXT_RUNNING_FREQUENCY, (uchar*)buff);

		}		  			 	  
	}
/*******************************************SCREEN_MANUAL_CTR_AVF【手动控制-AVF】*******************************************/
	if(screen_id == SCREEN_MANUAL_CTR_AVF)
    {
		float frequency;
		char buff[5];
		if(control_id == TXT_RUNNING_FREQUENCY)
		{
		    sscanf((char*)str,"%f",&frequency); 
		    if (frequency > MaxFreqOut) frequency = (float)MaxFreqOut;

			// TELL_FC_CONTROLLER_FREQ_CHANGE_IN_MANUAL_MODE  //removed 20220513
			
			/*
			 * added 20220513
			 * Update frequency of pump0-5
			 * */
			if (manual_avf_button_state[0] == ON) UpdatePumpFreq(frequency, 0);
			if (manual_avf_button_state[1] == ON) UpdatePumpFreq(frequency, 1);
			if (manual_avf_button_state[2] == ON) UpdatePumpFreq(frequency, 2);
			if (manual_avf_button_state[3] == ON) UpdatePumpFreq(frequency, 3);
			if (manual_avf_button_state[4] == ON) UpdatePumpFreq(frequency, 4);
			if (manual_avf_button_state[5] == ON) UpdatePumpFreq(frequency, 5);
		   
		    printf("********************************frequency = %0.1f\n",frequency);

		    manual_freq_setting = frequency;

		    sprintf(buff, "%.1f", frequency);

		    SetTextValue(SCREEN_MANUAL_CTR_AVF, TXT_RUNNING_FREQUENCY, (uchar*)buff);

		}		  			 	  
	}
/*******************************************SCREEN_ENTRANCE_SENSOR0【入口传感器0】*******************************************/
	else if(screen_id == SCREEN_ENTRANCE_SENSOR0)
	{
		 int i_value;   

	     if(control_id == 5)
	     {
		 	sscanf((char*)str, "%d", &i_value); 
		//	Entrance_sensor0[4] = i_value;
		printf("i_value = %d\n",i_value);

			WL_Bias = i_value;
			SetBias((float)i_value, IND_CH_WATER_LEVEL);
		 	if(unsaved_changes_exist == false) unsaved_changes_exist = true;

	     }
  		 else if(control_id == 4)
	     {
		 	sscanf((char*)str,"%d",&i_value); 
	
			Entrance_sensor0[2] = i_value / 256;
			Entrance_sensor0[3] = i_value % 256;

		 	if(unsaved_changes_exist == false) unsaved_changes_exist = true;

	     }
		 else
	     {
		 	sscanf((char*)str,"%d",&i_value); 
	
			Entrance_sensor0[control_id * 2 - 6] = i_value / 256;	   
			Entrance_sensor0[control_id * 2 - 5] = i_value % 256;

		 	if(unsaved_changes_exist == false) unsaved_changes_exist = true;

	     }
	}
/*******************************************SCREEN_ENTRANCE_SENSOR2【入口传感器2】*******************************************/
	else if(screen_id == SCREEN_ENTRANCE_SENSOR2)
	{
		 float f_value;

		 sscanf((char*)str, "%f", &f_value); 
	     if(control_id == 3)
	     {
		 	
		
			Entrance_sensor2[1] = (int)(f_value * 100) / 256;
			Entrance_sensor2[2] = (int)(f_value * 100) % 256;

		 	if(unsaved_changes_exist == false) unsaved_changes_exist = true;

	     }
  		 else if(control_id == 4)
	     {
		 
			Entrance_sensor2[3] = (int)(f_value * 100.0 + 0.1);

		 	if(unsaved_changes_exist == false) unsaved_changes_exist = true;
	     }

	}
/*******************************************SCREEN_VALVE_CONTROL1【阀门控制1】*******************************************/
	else if(screen_id == SCREEN_VALVE_CONTROL1)
	{	
		 float f_value;
		 int i_value;   

	     if((control_id == 1) || (control_id == 5))				//23 45 
	     {
			sscanf((char*)str,"%f",&f_value); 
			Valve_Control1[control_id - 1] = (int)(f_value * 100.0 + 0.1);

		 	if(unsaved_changes_exist == false) unsaved_changes_exist = true;
	     }
	     else 
	     {
			sscanf((char*)str,"%d",&i_value); 
			Valve_Control1[control_id - 1] = (uint16)i_value;

		 	if(unsaved_changes_exist == false) unsaved_changes_exist = true;

	
	     }	
	}  
/*******************************************SCREEN_SUPPLIER_INFO_3【厂家信息】*******************************************/
	else if(screen_id == SCREEN_SUPPLIER_INFO_3)
	{	
		
	     if(control_id == TXT_PHONE_NO)				
	     {

			 if(strlen((char*)str) == 0)   //empty number
			 {
			 	SavePhoneNbr("123456789abcde");
				SetTextValue(SCREEN_MAIN_1,  26 , " ");	  
				delay_ms(100);
				SetTextValue(SCREEN_MAIN_1, TXT_PHONE_NUMBER_MAIN, "");
				delay_ms(100);
				SetTextValue(SCREEN_MAIN_1, TXT_QR_CODE_MAIN, "");

			 }
			 else if(strlen((char*)str) <= 13)	//eligible input
			 {
				SavePhoneNbr(str);
			 	strcpy(PhoneNumber, (char*)str);
				SetTextValue(SCREEN_MAIN_1,  26 , "服务热线");	 	
				delay_ms(100);
				SetTextValue(SCREEN_MAIN_1, TXT_PHONE_NUMBER_MAIN, str);
				delay_ms(100);
				SetTextValue(SCREEN_MAIN_1, TXT_QR_CODE_MAIN, str);
			 }
			 else	  //out of range, discard
			 {
				 SetTextValue(SCREEN_SUPPLIER_INFO_3, TXT_PHONE_NO , (uchar*)PhoneNumber);	  
			 }
	     }		//
	     else if(control_id == TXT_COMPANY_NAME)				
	     {

			 if(strlen((char*)str) == 0)   //empty number
			 {
			 	SaveCompanyName(" ");
				SetTextValue(SCREEN_MAIN_1,  9 , " ");	  

			 }
			 else if(strlen((char*)str) <= 40)	//eligible input
			 {
			 	printf("%s", (char*)str);
				SaveCompanyName(str);
				SetTextValue(SCREEN_MAIN_1,  9 , str);	  
			 }
			 else	  //out of range, discard
			 {
			 	SetTextValue(SCREEN_SUPPLIER_INFO_3, TXT_COMPANY_NAME, (uchar*)CompanyName);	  
			 }
	     }	

	} 
/*******************************************SCREEN_DEVICE_ADDRESS【本机地址】*******************************************/
	else if(screen_id == SCREEN_DEVICE_ADDRESS)
	{
 		 int i_value;   

	     if(control_id == 1)			//23 45 
	     {
			sscanf((char*)str,"%d",&i_value); 
			DeviceAddress = i_value; 
		 	if(unsaved_changes_exist == false) unsaved_changes_exist = true;
	     }
	} 
/*******************************************SCREEN_PUMP_TEM_CONTROL【泵温监控】*******************************************/
/*temp_limit[4] = {60.0, 60.0, 60.0, 60.0};
float temp_bias[4] = {0.0, 0.0, 0.0, 0.0}; */
	else if(screen_id == SCREEN_PUMP_TEM_CONTROL)
	{	
		 float f_value;  
		 sscanf((char*)str, "%f", &f_value); 

		 if(control_id % 3 == 2)	//limit
		 {
			 temp_limit[control_id / 3] = f_value;
		 }
		 else if(control_id % 3 == 0)	//bias
		 {
			 temp_bias[control_id / 3 - 1] = f_value;
			 phy_quantities_bias[control_id / 3 - 1 + 3] = f_value; 
		 }
		 else if (control_id == 4)	//temp_restore
		 {
			 temp_restore = f_value;
		 }
		  
		 if(unsaved_changes_exist == false) unsaved_changes_exist = true;

	}
/*******************************************SCREEN_TRAIL_DATE_SETTING【试用期设置3】*******************************************/	
	else if(screen_id == SCREEN_TRAIL_DATE_SETTING)
	{	
		 int i_value;  
		 sscanf((char*)str, "%d", &i_value); 

	     if(control_id == TXT_YEAR)				
	     {
			StopDate.year = i_value;
	     }	
		 else if(control_id == TXT_MONTH)				
	     {
			StopDate.month = i_value;
	     }	
		 else if(control_id == TXT_DAY)				
	     {
			StopDate.day = i_value;
	     }	
		 if(unsaved_changes_exist == false) unsaved_changes_exist = true;

	}
#ifdef DBG_FLASH_IMAGE	
	else if(screen_id == 37)
	{	
		 int i_value; 
		 uint8 write_buff[2]; 
		 sscanf((char*)str, "%d", &i_value); 
		 if(control_id == 49)				
	     {
			flash_index = i_value;

	     }	
		 if(control_id == 51)				
	     {
			flash_cell_value0 = i_value;
	     }	
	     else if(control_id == 55)				
	     {
			write_buff[0] = flash_cell_value0;
			write_buff[1] = i_value;
		
			STM32_FLASH_Write(STM32_FLASH_SAVE_ADDR_USER_SETTINGS + flash_index, (u16*)write_buff,1);
	     }	
	}
#endif
/*******************************************SCREEN_RTC_SETTING【系统时钟设置】*******************************************/
			                                                                                    
}                                                                                

/*!                                                                              
*  \brief  进度条控件通知                                                       
*  \details  调用GetControlValue时，执行此函数                                  
*  \param screen_id 画面ID                                                      
*  \param control_id 控件ID                                                     
*  \param value 值                                                              
*/                                                                              
void NotifyProgress(uint16 screen_id, uint16 control_id, uint32 value)           
{  

}                                                                                

/*!                                                                              
*  \brief  滑动条控件通知                                                       
*  \details  当滑动条改变(或调用GetControlValue)时，执行此函数                  
*  \param screen_id 画面ID                                                      
*  \param control_id 控件ID                                                     
*  \param value 值                                                              
*/                                                                              
void NotifySlider(uint16 screen_id, uint16 control_id, uint32 value)             
{                                                             
   
}

/*! 
*  \brief  仪表控件通知
*  \details  调用GetControlValue时，执行此函数
*  \param screen_id 画面ID
*  \param control_id 控件ID
*  \param value 值
*/
void NotifyMeter(uint16 screen_id, uint16 control_id, uint32 value)
{
    //TODO: 添加用户代码
}

/*! 
*  \brief  菜单控件通知
*  \details  当菜单项按下或松开时，执行此函数
*  \param screen_id 画面ID
*  \param control_id 控件ID
*  \param item 菜单项索引
*  \param state 按钮状态：0松开，1按下
*/
void NotifyMenu(uint16 screen_id, uint16 control_id, uint8 item, uint8 state)
{
    //TODO: 添加用户代码
}

/*! 
*  \brief  选择控件通知
*  \details  当选择控件变化时，执行此函数
*  \param screen_id 画面ID
*  \param control_id 控件ID
*  \param item 当前选项
*/
void NotifySelector(uint16 screen_id, uint16 control_id, uint8  item)
{
    if(screen_id == 20&&control_id ==8)                                //获取当前选择控件的值
    {
       if(item == 0)       printf("0-1Mpa is selected\n");
       if(item == 1)       printf("0-1.6Mpa is selected\n");
       if(item == 2)       printf("0-2.5Mpa is selected\n");

    } 
 

}

/*! 
*  \brief  定时器超时通知处理
*  \param screen_id 画面ID
*  \param control_id 控件ID
*/
void NotifyTimer(uint16 screen_id, uint16 control_id)
{
    if(screen_id==8&&control_id == 7)
    {
        SetBuzzer(100);
    } 
}

/*! 
*  \brief  读取用户FLASH状态返回
*  \param status 0失败，1成功
*  \param _data 返回数据
*  \param length 数据长度
*/
void NotifyReadFlash(uint8 status,uint8 *_data,uint16 length)
{
//#define DBG_NOTIFYREADFLASH
//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_NOTIFYREADFLASH

    printf("\n\n\n Entrance: void NotifyReadFlash(uint8 status,uint8 *_data,uint16 length)...............\n\n");
#endif  
//--------------------------------------------------------------------------------------------------

#ifdef DBG_NOTIFYREADFLASH
	printf("Get the response from from HMI for reading CMD \n");
#endif 

	IWDG_FEED
	if(status)
	{	
		if(FactoryInfoCheckPassed==0)	  //At startup, BKG data is to be checked: load FactoryInfoLocked and check checksum
		{
			  
#ifdef DBG_NOTIFYREADFLASH
			 printf("Factory checking...\n");
#endif
			   
			 //check……...
			 if(_data[0] != 0x5a)	   // not initialized yet, need to reset to factory settings 
			 {
			 	
#ifdef DBG_NOTIFYREADFLASH
				printf("kw in flash is %d instead of expected 0x5a, check failed…reset flash to factory\n", _data[0]);
#endif 

				FactorySettings(0x5a);	  //send cmd: write user init-data(to be read back in NotifyWriteFlash) into hmi flash

			 }
			 else//check ok, jump to next phase
			 {
			 	UseBKG_SettingsInFlash(_data);

			 	FactoryInfoCheckPassed = 1;
				LoadAllSettingsInFlash();

			 }

		}
		else  //result of normal settings
		{
			
#ifdef DBG_NOTIFYREADFLASH
			printf("no need to check, normal reading\n");
#endif

			UseSettingsInFlash(_data, length);	
		}

	}
#ifdef DBG_NOTIFYREADFLASH
	else  printf("read flash data:error\n\n");
#endif

#ifdef DBG_NOTIFYREADFLASH
    printf("\n\n\n Exit: void NotifyReadFlash(uint8 status,uint8 *_data,uint16 length)...............\n\n");
#endif  

	
}

/*! 
*  \brief  写用户FLASH状态返回
*  \param status 0失败，1成功
*/
void NotifyWriteFlash(uint8 status)
{
    //TODO: 添加用户代码
	if(status)
	{
   		printf("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++write flash ok:\n\n");
		if(FactoryInfoCheckPassed==0)	  //kw not checked yet
		{
			printf("write factory info ok\n");
			printf("read factory setting again\n");
			FactoryCheck();				  //send cmd: read bkg data(just written into hmi flash) for checking
		}								   
	}
	else
   		printf("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++write flash error:\n\n");
}

uint8 TrialTimeOut(MY_DATE rtc, MY_DATE target)
{
	if(rtc.year > target.year) return TRUE;
	if(rtc.year < target.year) return FALSE;

	if(rtc.month > target.month) return TRUE;
	if(rtc.month < target.month) return FALSE;

	if(rtc.day > target.day) return TRUE;
	return FALSE;
	
}

#ifdef MY_CLOCK
void RefeshClock(void)
{
	SetMeterValue(0, 2, now_min);
	delay_ms(100);
	SetMeterValue(0, 3, (now_hour % 12) * 5 + now_min / 12);
	delay_ms(100);
	SetMeterValue(0, 9, now_sec);

}
#endif

void UpdateRTC(void)
{
	RTC_Date.year = 2000 + now_year;   //20xx
	RTC_Date.month = now_month;
    RTC_Date.day = now_day;

}

/******NotifyReadRTC***********************************/
/*! 
*  \brief  读取RTC时间，注意返回的是BCD码
*  \param year 年（BCD）
*  \param month 月（BCD）
*  \param week 星期（BCD）
*  \param day 日（BCD）
*  \param hour 时（BCD）
*  \param minute 分（BCD）
*  \param second 秒（BCD）
*/
void NotifyReadRTC(uint8 year,uint8 month,uint8 week,uint8 day,uint8 hour,uint8 minute,uint8 second)
{
    int years,months,weeks,days,hours,minutes;
//*****************************UNIT TEST HEADER*****************************************************

//--------------------------------------------------------------------------------------------------

       
    sec    =(0xff & (second>>4))*10 +(0xf & second);                                    //BCD码转十进制
    years   =(0xff & (year>>4))*10 +(0xf & year);                                      
    months  =(0xff & (month>>4))*10 +(0xf & month);                                     
    weeks   =(0xff & (week>>4))*10 +(0xf & week);                                      
    days    =(0xff & (day>>4))*10 +(0xf & day);                                      
    hours   =(0xff & (hour>>4))*10 +(0xf & hour);                                       
    minutes =(0xff & (minute>>4))*10 +(0xf & minute);  
	now_year = years;
	now_month = months;
	now_day = days;
	now_hour = hours;
    now_min =  minutes; 
	now_sec = sec; 


    /* ------------------------------------------------------
	                      sun  mon	tue  wes   thu  fri sat
	  convert weekday:   [ 0,   1,   2,   3,   4,   5,   6] 
	  	        	-->  [ 6,   0,   1,   2,   3,   4,   5]
       --------------------------------------------------------*/
	if(weeks == 0)
	{
		now_weekday = 6;  	//Sunday
	}
	else
	{
		now_weekday = weeks - 1;   
	}

#ifdef    DBG_RTC	
	printf("\ntime is now: %02d:%02d:%02d, weekday=%d\n",now_hour,  now_min,  now_sec, now_weekday);
#endif
	PumpMileageMonitor(); //20210805

    if(now_hour != OldHour)
	{
		printf("RTC hour changed\n");

		if((now_hour == 0) && (OldHour == 23))		   //new day  
		{
			new_day_starts = 1;
			ExpireDateManager();

#ifdef DBG_FUNC_INFO
            if(func_dbg_info) printf("\nTo PC-API:new day starts: %d-%d-%d: 星期%d\n", now_year, now_month, now_day, now_weekday + 1);
#endif	  
		}
		if(now_hour == (OldHour + 1) % 24)			   //new hour 
		{
			printf("\nnew hour starts\n");
			/*
			if(MileageEnabled == TRUE)
			{
				HourMileage ++;
				UpdateMileage();
			    WriteMileageToFlash();
		
				if(HourMileage == MileageLimit)	   //Balance exhausted
				{		
					MileageStatus = MILEAGE_STATUS_RED;
					ReturnScreenId = current_screen_id;
					ShowMessage(WARNING_BALANCE_1H);   //
				}
				if(HourMileage + 24 == MileageLimit)	  //Balance is 24 hrs
				{
					ReturnScreenId = current_screen_id;
					ShowMessage(WARNING_BALANCE_24H);
				}
			}  */

			if(nHoursCounter > 0)
			{
				nHoursCounter--;
				if(nHoursCounter == 0)			 
				{
				 	printf("n hours elapsed\n");
				}
			}
#ifdef FATIGE_CLOCK
//			if(NewMinuteEvent == OFF) SetMeterValue(1, 28, (36 - (int)(RunningCounter / 60.0 + 0.5)) * 10);
#endif
		}
		OldHour = now_hour; 
	}  
	
	 if(now_min != OldMinute)     //new minute
	 {	
	 	/*New minute comes and there might be an updating of the target pressure: entrance/exit of TC window*/
	    printf("ShowTargetPressure(LoadTargetPressure());\n");

		if ((RemoteMode == TRUE) && (RemoteTargetEnable == TRUE)) 
		{
			Tartget_Pressure = RemoteTargetPressure;//Remote Mode
		}
		else
		{
			Tartget_Pressure = LoadTargetPressure(); //Local Mode
		}

 	/*	if ((!RemoteMode) || (RemoteTargetEnable == FALSE)) Tartget_Pressure = LoadTargetPressure(); //Local Mode
		else Tartget_Pressure = RemoteTargetPressure;	*/											 //Remote Mode

		ShowTargetPressure(Tartget_Pressure);

	  	OldMinute = now_min; 

#ifndef DBG_USE_SECOND_AS_MINUTE
		NewMinuteEvent = ON;
		printf("NEW MINUTE!\n");
#endif
	 }  

#ifdef DBG_USE_SECOND_AS_MINUTE
	NewMinuteEvent = ON;
	printf("NEW MINUTE!\n");
#endif

#ifdef MY_CLOCK
	RefeshClock();
#endif 

#ifdef DBG_FUNC_INFO
    if(func_dbg_info) printf("\n\n\n Exit: void NotifyReadRTC(uint8 year,uint8 month,uint8 week,uint8 day,uint8 hour,uint8 minute,uint8 second)...............\n\n");
#endif  
     
}

#ifdef MY_CLOCK
	void set_clock()
	{
		uint8 rtc[7] = { 34, 59, 18, 17, 0, 10, 38};		
		
		WriteRTC(rtc);
	 }
#endif


void PowerUpAutoRunManager(void)
{
	if(PowerUpAutoRunTimer > 0)
	{
		PowerUpAutoRunTimer--;
		printf("PowerUpAutoRunTimer = %d\n", PowerUpAutoRunTimer);
		if(PowerUpAutoRunTimer == 0)
		{
			if(ErrorCounter == 0)
			{
			/*	PIDParament_Init();
				start_mod_1_2();
				start_task_scheduler();
				PumpErrStatus = PUMP_RUNNING;

				RefreshButtons();	 */




				PIDParament_Init();
						
				start_task_scheduler();
				RefreshButtons();		 
				start_mod_1_2();
				PumpErrStatus = PUMP_RUNNING;
			}
		}
	}
}

#ifdef USE_CURVES
uint8 ManualFcRunning(void)
{
	return FC_is_on;
}

#endif

#ifdef USE_MODBUS
void ApiCmdExecutor(void)	//
{

	if(ApiCmd == API_CMD_START) 
	{
		ApiCmd = 'x';
		if(SysRunning == FALSE)
		{
			ReturnModBusCMD();
#ifdef DBG_EVENT_AS_ERROR
			EventRecorder("收到运行指令");
#endif
	
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
	}
	else if(ApiCmd == API_CMD_STOP)
	{
		ApiCmd = 'x';
		if(SysRunning == TRUE)
		{
		
			ReturnModBusCMD();
#ifdef DBG_EVENT_AS_ERROR
			EventRecorder("收到停止指令");
#endif
			SysRunning = FALSE;
			Sys_Switches[IND_SYSTEM] = OFF;
		
			Start_Stopper(STOP_TYPE_SYSTEM);
			EscapeSleeper();
#ifdef MY_CLOCK
	    	set_clock();
#endif
		}


	} 

}
#endif
#ifdef DBG_USE_UART1_RECEIVING
void ApiCmdExecutor(void)	//
{

	if(ApiCmd == API_CMD_START)
	{
		//	NotifyButton(SCREEN_MAIN_1, BTN_START_AUTO_SCREEN_MAIN, ON);
		ApiCmd = 'x';
	
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
	else if(ApiCmd == API_CMD_STOP)
	{
		ApiCmd = 'x';
		SysRunning = FALSE;
		Sys_Switches[IND_SYSTEM] = OFF;
		
		Start_Stopper(STOP_TYPE_SYSTEM);
		EscapeSleeper();
#ifdef MY_CLOCK
	    set_clock();
#endif


	} 
}
#endif
/***********************CODER FINDER************************************

KEYWORDS FOR ENTRANCE

***SCREEN_START  
                
***SCREEN_MAIN_1                 
	***SCREEN_MANUAL_CTR_2     
	      
	***SCREEN_USER_SETTINGS_2     
		***SCREEN_PUMP_GROUPING       
		***SCREEN_OUTLET_SENSOR  
		***SCREEN_OUTLET_CALI
		***SCREEN_ENTRANCE_CALI 
		***SCREEN_ENTRANCE_SENSOR  
		***SCREEN_ENTRANCE_SENSOR0       
		***SCREEN_ENTRANCE_SENSOR2
		***SCREEN_PUMP_SWITCH_CONDTION 	  
		***SCREEN_SLEEP_SETTING 
		***SCREEN_POWER_UP_SETTING

	***SCREEN_SYS_SETTINGS_2  
		***SCREEN_SCREEN_SETTING
		***SCREEN_PID_SETTING  		    //20210712
	       
	***SCREEN_FAILURE_INQUIRY_2      
	      
	***SCREEN_TIME_CTR_3   
	
	          
***SCREEN_TIME_SETTING_3         
***SCREEN_PIN_SETTING_3          
       
***SCREEN_SUPPLIER_INFO_3        
***SCREEN_PIN_CONTROL            
***SCREEN_PIN_CONTROL_FACTORY    
***SCREEN_BALANCE_WARNING        
***SCREEN_USER_PIN_MANAGER       

         
       
  
          
       
***SCREEN_VALVE_CONTROL          
***SCREEN_PID_SETTING            
***SCREEN_CONTROL_CURVES         
       
***SCREEN_VALVE_CONTROL1         
***SCREEN_STOP_CONFIRM1          
***SCREEN_RTC_SETTING            
***SCREEN_PASSWORD_SELECT        
***SCREEN_TRAIL_DATE_SETTING     
***SCREEN_FUNC_DBG_SWITCHES      
***SCREEN_PUMP_TEM_CONTROL       
***SCREEN_DEVICE_ADDRESS         
***SCREEN_MANUAL_CTR_AVF   

***NotifyReadRTC      



*/