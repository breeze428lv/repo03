/****************************************************************************************
**                            深圳英捷思科技有限公司
**                             http://www.
**-----------------------------------文件信息--------------------------------------------
** MCU: STM32F103VET6(High-density device)
**       Flash memory density: 512 Kbytes
**
** 文件名称:   main.c
** Version:    
** 修改时间:   2019-11-24 13:13
** 文件说明:   
**
--------------------------------------------------------------------------------------*/
#include <math.h>
#include "hmi_driver.h"
#include "cmd_queue.h"
#include "cmd_process.h"
#include "stdio.h"
#include "stdlib.h"
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
#include "measurement.h"
#include "settings.h"
#include "time_control.h"
#include "security.h"
#include "pump_running.h"
#include "ui_controls.h"
#include "UserInterface.h"
#include "api_comm.h"

//--------------------Macors------------------------------------------
#define TIME_100MS 10   
#define TICK_BIAS  0 	 //for fine tune of timing

#define POWER_UP_AUTORUN_DISABLE PowerUpAutoRunTimer = 0;




//--------------------Debugging---------------------------------------

#ifdef DBG_FUNC_FOOTPRINT
	 uint8 dbg_func_depth = 0;
	 uint8 dbg_indent_ind;
#endif	

#ifdef DBG_FUNC_LEN
typedef struct{
				uint32 start;
				uint32 end;
				uint16 count;
			    uint32 sum;
} PROFILE;

PROFILE profile;

#endif

#ifdef DBG_FUNC_INFO
	 uint8  func_dbg_info;
#endif

//--------------------Variables---------------------------------

TYPE_FATIGUE_PUMP_CANCEL FatiguePumpCancel = {0, 255};

extern TYPE_PUMP_ICONS PumpIcons[];

uint8 fast_mode = FALSE;						   //To control exeution period of Pump_Manager(): 100ms|1s

uint16 RunningCounter = 1;						   //Counter for continous running time in minute, deicated for fatigue switching

uint16 t_counter, i_counter;                       //100毫秒(10个单位)
 
volatile uint32  timer_tick_count = 0;             //定时器节拍

uint8 SysRunning;								   //System is running, sleeping is included.

uint8 WarmUpDone = FALSE;						   //Flag of warm up. After 2 measurements from power-up, WarmUpDone = TRUE

extern uint8 Pump_Status;

extern uint16 ReturnScreenId;

extern uint8 Pump_Switch_Condtion[];

extern float Tartget_Pressure;

extern float OutletRealTimePressure;

bool TargetLockConfirmed = false;
uint8 TargetLockCounter = 0;

//Timing

uint16 tick_counter_100ms = 0;					  // Counter for 100ms timer

uint8 tick_counter_error_scan = 2;				  // Counter for error_scan  2s-1s-1s-1s...

uint8 TickPidCounter = 0;						  // Counter for pid.  1s

uint16 PowerUpAutoRunTimer;						  // PowerUpAutoRun.  1s

#ifdef DUAL_WATER_SOURCE
uint16 TankPeriodCounter;

uint16 TankOnCounter;
#endif

extern uint8 pump_mode;

uint8 Manual_Setting;

uint8 SoftStopping;

#ifdef USE_CURVES
extern uint8 up_crossed;	
#endif

extern uint8 PumpGroupMode;

extern uint8 MileageEnabled;

uint8 Error_Channel_Disconnected;

extern uint8 Error_Switch_Status;

extern float DefaultTargetPressure;

extern uint8 error_occured;

uint8  cmd_buffer[CMD_MAX_SIZE];    										  //指令缓存
static uint8 update_en = 0;                                                    

static uint8 FirstInLoop  = TRUE;

extern uint8 new_day_starts;

extern uint8 schedule_loaded;

extern uint8 SettingsLoaded;

extern uint8 WaterSourceMode;

#ifdef DUAL_WATER_SOURCE
extern uint8 Valve_Control1[];
#endif


#ifdef USE_PWM
uint8 DutyCycle = 0;
#endif

#ifdef USE_MODBUS
uint8 ModBusTrafficTimer = MODBUS_RED_LIGHT_DURATON;
#endif

#ifdef DBG_TARGET_LOCKER
#define ShowTagetLockerInfo SetTextValue(SCREEN_MAIN_1, 75, "TargetLocked"); SetControlVisiable(SCREEN_MAIN_1, 76, VISIBLE);	   
#define HideTagetLockerInfo SetTextValue(SCREEN_MAIN_1, 75, ""); SetControlVisiable(SCREEN_MAIN_1, 76, INVISIBLE);
#endif

extern uint8 pump_on_off_tbl[];
extern uint8 focused_pump_index;

void WarningTagManager(uint8 cmd)
{
	static uint8 cnt = 0;
	if (cmd == 1) //set cnt
	{
		cnt = 2;
		return;
	}   
	//monitor
	if (cnt)
	{
	    cnt--;
		if (cnt == 0)	 //time out
		{
			SetTextValue(SCREEN_MAIN_1, 9, "");
		}
	}
}


#ifdef DBG_VIRTUAL_ERR 

	extern uint8 VirtualErr[];
	#define TOGGLE(x)       x = 1 - x;

	void virtual_err_generator(void)
	{
	    static uint8 counter = 0;
		static uint8 action_counter = 1;
		counter++;
		if (counter >= 10) 	counter = 0;

	    if (action_counter == counter)
		{
			TOGGLE(VirtualErr[0]) 

			/*int rand(void): return a random number in range:0~RAND_MAX（no less than 32767）*/
			action_counter = (action_counter + (rand() % 8)) % 10;
		}
	}
#endif

/************************************************************************                                                                               
*  Entrance of Main program                                                           
*************************************************************************/   
                                                                              
int main()                                                                          
{                                                                                     
    uint32 timer_tick_last_update = 0;                                               //上一次更新的时间
	uint32 timer_tick_last_update_1s = 0;                                               //上一次更新的时间
	uint32 timer_tick_last_update_xms = 0;    
    qsize  size = 0; 
	//////////////////////DBG
	uint16 ccrv = 0;
	uint8 freq = 0;
	uint8 dir = 1;

	uint8 iii = 0;
    char buff[5];
	////////////////////////// 

#ifdef DBG_TARGET_LOCKER
	static bool TargetLocked = true; 
#endif


#ifdef USE_PWM			
    static uint8 duty_inc = TRUE;
#endif                                                              

    //配置时钟                                                                    
    Set_System_Clocks();                                                                                                                                  
	                                                       
    //配置时钟节拍                                                            
    systicket_init();    
	                                                           
    //串口初始化  
#ifdef PRINT_TO_UART			  //Uart1 for dbg-msg printer
    USART1_Config(115200);
#else                                                                                       
    USART1_Config(9600);
#endif

	USART2_Config(9600);		  //Uart2 for HMI communication

	//Interrupt setting for Uart1/Uart2
	NVIC_Configuration1();
	NVIC_Configuration2();

#ifdef USE_PWM				   //currently unused 
	Timerx_Init(9,719);	//100us 
#endif

#ifdef USE_MODBUS
	Timerx_Init(99, 479);	   //Timer3: Frequency(period): 48M/((479 + 1) * (99 + 1)) = 1000Hz = 1ms 	   
#endif

	RCC_Configuration();

	GPIO_Configuration();

	TIM3_PWM_Configuration();

	Status_GPIO_Config();

	RS485_Initialization();

    //清空串口接收缓冲区      	                                                    
    queue_reset();                                                                  

	if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)
	{
	    printf("/* 独立看门狗复位 */\n");
		debugger("IWDG RESET....................",ON);
	
		/* 清除标志 */
		RCC_ClearFlag();
	}
#ifdef USE_IWDG
	IWDG_Config(IWDG_Prescaler_256 ,625 * 4);	//256*625*4/40K = 16(S)		   20200327
#endif

    //Wait for initialization of the screen, no less than 300ms       	                              
    delay_ms(300);        
    
	IWDG_FEED

    Adc_Configuration();	  
	Dac_Config();
	DMA_Configuration();

	Relay_GPIO_Config();

	update_en = 1;
  	
	schedule_loaded = 0;

#ifdef  DBG_POWER_UP_INFO
	#if(AUTHORITY_ADMIN)
		printf("AUTHORITY_ADMIN \n");
	#else
		printf("AUTHORITY_USER \n");
	#endif
	printf("*********************************PROGRAMMED THROUGH ST-LINK PROGRAMMER*****************************\n");
#endif
	
	error_occured = 0;

	#ifdef  USE_FLASH
	    FactoryCheck();
	#endif

	/*Initialize entrance pressure check*/	
	reset_pump_entrance_pressure_check_startup_monitor();


	//TO BE REMOVED FOR FORMAL USE!!!
	/*********************************UI Tester***********************************************/
//#define UI_TESTER
#ifdef UI_TESTER
	while (1)
	{
	#if 1
			   iii++;
			  if (iii % 4 == 0) //stop
			   {
			   		SetControlVisiable(SCREEN_MAIN_1,PumpIcons[(iii / 4) % 6].icon_pump_fc, INVISIBLE);
					SetControlVisiable(SCREEN_MAIN_1,PumpIcons[(iii / 4) % 6].icon_pump_pf, INVISIBLE);
					SetControlVisiable(SCREEN_MAIN_1,PumpIcons[(iii / 4) % 6].icon_pump_mask, INVISIBLE);
					AnimationStop(SCREEN_MAIN_1,    PumpIcons[(iii / 4) % 6].icon_pump_flow);
					SetTextValue(SCREEN_MAIN_1, PumpIcons[(iii / 4) % 6].txt_pump_mode, "");
				//	AnimationStop(SCREEN_MAIN_1,    67);
			   }  
		       else if (iii % 4 == 1) //pf
			   {
			   		SetControlVisiable(SCREEN_MAIN_1,PumpIcons[(iii / 4) % 6].icon_pump_fc, INVISIBLE);
					SetControlVisiable(SCREEN_MAIN_1,PumpIcons[(iii / 4) % 6].icon_pump_pf, VISIBLE);
					SetControlVisiable(SCREEN_MAIN_1,PumpIcons[(iii / 4) % 6].icon_pump_mask, INVISIBLE);
					AnimationStart(SCREEN_MAIN_1,    PumpIcons[(iii / 4) % 6].icon_pump_flow);
				    SetTextValue(SCREEN_MAIN_1, PumpIcons[(iii / 4) % 5].txt_pump_mode, "工频");
			   }	 
			   else if (iii % 4 == 2) //fc
			   {
			   		SetControlVisiable(SCREEN_MAIN_1,PumpIcons[(iii / 4) % 6].icon_pump_fc, VISIBLE);
					SetControlVisiable(SCREEN_MAIN_1,PumpIcons[(iii / 4) % 6].icon_pump_pf, INVISIBLE);
					SetControlVisiable(SCREEN_MAIN_1,PumpIcons[(iii / 4) % 6].icon_pump_mask, INVISIBLE);
				//	AnimationStart(SCREEN_MAIN_1,    PumpIcons[(iii / 4) % 5].icon_pump_flow);
					SetTextValue(SCREEN_MAIN_1, PumpIcons[(iii / 4) % 6].txt_pump_mode, "变频");
				    SetTextValue(SCREEN_MAIN_1, PumpIcons[(iii / 4) % 6].icon_freq, "25.9");
			   }
			    else if (iii % 4 == 3) //vanish
			   {
			   		SetControlVisiable(SCREEN_MAIN_1,PumpIcons[(iii / 4) % 6].icon_pump_fc, INVISIBLE);
					SetControlVisiable(SCREEN_MAIN_1,PumpIcons[(iii / 4) % 6].icon_pump_pf, INVISIBLE);
					SetControlVisiable(SCREEN_MAIN_1,PumpIcons[(iii / 4) % 6].icon_pump_mask, VISIBLE);
			   }   	
			   delay_ms(1000);	  
			   IWDG_FEED
	#endif	


	}
	//result:
	/*   	 1      2     3     4     5     6
	   pf	 -1	   -1				 -1
	   fc	 -1	   -1			-2	 -1	     
	*/						     
#endif  
//------Main Loop-------                
    while(1)                                                                      
    {  
   		// CMD reader 
#ifdef  DBG_FUNC_FLOW
 
		printf("CMD reader \n");
#endif 
                                                                     
        size = queue_find_cmd(cmd_buffer,CMD_MAX_SIZE);                              //从缓冲区中获取一条指令    
		     
   		/*When HMI is powered it issues a soft reset command(0x07) to MCU,
		  then starts the data exchange with the latter*/
        if(size > 0 && cmd_buffer[1] != 0x07)                                        //接收到指令 ，及判断是否为开机提示
        {                                                                           
            ProcessMessage((PCTRL_MSG)cmd_buffer, size);                             //指令处理  
        }                                                                           
        else if(size > 0 && cmd_buffer[1] == 0x07)                                   //如果为指令0x07就软重置STM32  
        {                                                                          
            __disable_fault_irq();      // 关闭总中断			                                     
            NVIC_SystemReset();        // Initiate a system reset request.                                                                                                                                  
        }                                                                            
   
		if(FirstInLoop == TRUE)	//excuted only once	at the beginning , do some initializations. At this stage HMI is ready for communication
		{
			FirstInLoop = FALSE;
	
			InitVisibility();

			RefreshErrorInfo(0, ADD_ERROR_NO);
			
			SetTextValue(SCREEN_MAIN_1,TXT_PM_PRESSURE, "--"); 	
			
				
			
			load_err_record();	
			
		}
				///////////////////////
	//	delay_ms(3000);	
			
		/***********************Every xxms***********************************************/
		/*
        if(timer_tick_count - timer_tick_last_update_xms>=5)   //30ms
        {
			timer_tick_last_update_xms = timer_tick_count; 
			////////////////////////
     	 	ModBusCmdExtractor(); 
		
		}	*/											  
		
		/***********************Every 100ms***********************************************/
        if(update_en&&timer_tick_count-timer_tick_last_update>=TIME_100MS) 
        {
           
            timer_tick_last_update = timer_tick_count;   

            UpdateUI();

			tick_counter_100ms ++;
	
			take_cmd_pump();  

#ifdef USE_PWM
			if(duty_inc)
			{
            	DutyCycle = DutyCycle + 1;	 
				if(DutyCycle == 10) duty_inc = FALSE;
			}
			else
			{
            	DutyCycle = DutyCycle - 1;	 
				if(DutyCycle == 0) duty_inc = TRUE;
			}
#endif		


#ifdef DBG_FUNC_LEN
			profile.start = timer_tick_count; 
#endif

			if(OutletRealTimePressure != Tartget_Pressure) //PressureInTargetRange() == FALSE)
			{
		    	pid_calc(); 

#ifdef DBG_TARGET_LOCKER
				if(TargetLocked) 
				{
					 TargetLocked = false; 
					 HideTagetLockerInfo
				}
#endif 
			}

#ifdef DBG_FUNC_LEN
			profile.end = timer_tick_count; 
			profile.sum = profile.end - profile.start; 
			printf("TimeLen of pid_calc(): %dms\n", profile.sum * 10);
#endif
			
			if(fast_mode)  Pump_Manager();	 


////////////////////////////DBG
#if 0				    
					freq = freq + dir * 2 - 1;

					if ((freq % 50) == 0) dir = 1 - dir;




				//	if (freq < 50) freq++;
					UpdatePumpFreq((float)freq, 0);
					UpdatePumpFreq((float)freq, 1);
			    	UpdatePumpFreq((float)freq, 2);
					UpdatePumpFreq((float)freq, 3);
					UpdatePumpFreq((float)freq, 4);
					UpdatePumpFreq((float)freq, 5);
#endif


				    FatigueCancelMonitorAvf();	


////////////////////////////////////			
			
			/***********************Every second******************************************/
			
			if(timer_tick_count - timer_tick_last_update_1s >= TIME_100MS * 10) 	 
	        { 	          
				tick_counter_100ms = 0;
				timer_tick_last_update_1s = timer_tick_count;
				

		 		if(SettingsLoaded == TRUE)//(WarmUpDone == TRUE)
  					RealtimeMeasurement();  
				
#ifdef DBG_USE_UART1_RECEIVING
				ApiCmdExecutor();
#endif
#ifdef USE_MODBUS
				ApiCmdExecutor();
#endif 	 		 
#ifdef DBG_LABVIEW
                ApiLabViewFeeder();
#endif

		
				pump_pressure_check_startup_monitor();
				pump_entrance_pressure_check_startup_monitor();
				PowerUpAutoRunManager();

			//	if(OutletRealTimePressure == Tartget_Pressure)//PressureInTargetRange() == TRUE)
				if(TargetLockConfirmed)
				{
					printf("TargetLockConfirmed, FC_Retreator();\n");
					FC_Retreator();

#ifdef DBG_TARGET_LOCKER
					if(!TargetLocked) 
					{
						 TargetLocked = true; 
						 ShowTagetLockerInfo
					}
#endif

				}

#ifdef DBG_USE_FUEL_MONITOR
				FuelMonitor();
#endif
				/*************************DUAL_WATER_SOURCE only: Tank switch****TO BE MOVED TO 1 HOUR ROUTINE*********************************/
#ifdef DUAL_WATER_SOURCE
				if(WaterSourceMode == TANK_NONNEGTIVE_MODE)
				{

					printf("TankPeriodCounter = %d\n",TankPeriodCounter);
					printf("TankOnCounter = %d\n",TankOnCounter);


					if(TankPeriodCounter > 0)
					{
						TankPeriodCounter--;
						if(TankPeriodCounter == 0)		//New Tank-switch period starts, switch to tank water supply
						{
							ValveSwitchToTank = ON;
							TankPeriodCounter = Valve_Control1[2];		
							TankOnCounter = Valve_Control1[3];
						}
					}
					if(TankOnCounter > 0)			//Switch to Non-negative water supply
					{
						TankOnCounter--;
						if(TankOnCounter == 0)
						{
							ValveSwitchToTank = OFF;
						}
					}
				}
#endif
			
				/*************************for pump switch *****************************************************/
			 	
				FatigePumpSwitcher();
			 
			
				if(tick_counter_error_scan > 0) 
				{
					tick_counter_error_scan--;
				}

				/***********************Every 1s, for Status Scanning******************************************/
				else                 
				{
				    tick_counter_error_scan = 1;  

#ifndef DBG_DISABLE_WARNINGS
			 	    if(WarmUpDone) Status_Scan();	
#endif		

				} 
				/***********************Every ns, for pid******************************************/    
				
				if(TickPidCounter > 0) 
				{
					TickPidCounter--;
					if(TickPidCounter == 0) 
					{
						TickPidCounter = 1; 
						
					    if(fast_mode == FALSE)  Pump_Manager();	 
						
					}

				}
				WarningTagManager(0);
#ifndef USE_PWM
	#ifdef DBG_VIRTUAL_ERR
				virtual_err_generator();
	#else
			  	HeartBeating(); 
	#endif
#endif

#ifdef DBG_TIMING_CONSULTANT
 	            TimingConsultant(CMD_COUNTING);
#endif
			//	printf("%d\n", rand()%10);



//each pump displays pf-fc-none-stop in turn
//	    pf      fc
//1     -1		-1
//2		-1      -1
//3		-1      -1
//4		-1	    -1
//5
#if 0	 //DISABLED, FOR DBG ONLY
			   iii++;
			  if (iii % 4 == 0) //stop
			   {
			   		SetControlVisiable(SCREEN_MAIN_1,PumpIcons[(iii / 4) % 5].icon_pump_fc, INVISIBLE);
					SetControlVisiable(SCREEN_MAIN_1,PumpIcons[(iii / 4) % 5].icon_pump_pf, INVISIBLE);
					SetControlVisiable(SCREEN_MAIN_1,PumpIcons[(iii / 4) % 5].icon_pump_mask, INVISIBLE);
					AnimationStop(SCREEN_MAIN_1,    PumpIcons[(iii / 4) % 5].icon_pump_flow);
					SetTextValue(SCREEN_MAIN_1, PumpIcons[(iii / 4) % 5].txt_pump_mode, "");
				//	AnimationStop(SCREEN_MAIN_1,    67);
			   }  
		       else if (iii % 4 == 1) //pf
			   {
			   		SetControlVisiable(SCREEN_MAIN_1,PumpIcons[(iii / 4) % 5].icon_pump_fc, INVISIBLE);
					SetControlVisiable(SCREEN_MAIN_1,PumpIcons[(iii / 4) % 5].icon_pump_pf, VISIBLE);
					SetControlVisiable(SCREEN_MAIN_1,PumpIcons[(iii / 4) % 5].icon_pump_mask, INVISIBLE);
					AnimationStart(SCREEN_MAIN_1,    PumpIcons[(iii / 4) % 5].icon_pump_flow);
				    SetTextValue(SCREEN_MAIN_1, PumpIcons[(iii / 4) % 5].txt_pump_mode, "工频");
			   }	 
			   else if (iii % 4 == 2) //fc
			   {
			   		SetControlVisiable(SCREEN_MAIN_1,PumpIcons[(iii / 4) % 5].icon_pump_fc, VISIBLE);
					SetControlVisiable(SCREEN_MAIN_1,PumpIcons[(iii / 4) % 5].icon_pump_pf, INVISIBLE);
					SetControlVisiable(SCREEN_MAIN_1,PumpIcons[(iii / 4) % 5].icon_pump_mask, INVISIBLE);
				//	AnimationStart(SCREEN_MAIN_1,    PumpIcons[(iii / 4) % 5].icon_pump_flow);
					SetTextValue(SCREEN_MAIN_1, PumpIcons[(iii / 4) % 5].txt_pump_mode, "变频");
			   }
			 /*  else if (iii % 4 == 3) //vanish
			   {
			   		SetControlVisiable(SCREEN_MAIN_1,PumpIcons[(iii / 4) % 5].icon_pump_fc, INVISIBLE);
					SetControlVisiable(SCREEN_MAIN_1,PumpIcons[(iii / 4) % 5].icon_pump_pf, INVISIBLE);
					SetControlVisiable(SCREEN_MAIN_1,PumpIcons[(iii / 4) % 5].icon_pump_mask, VISIBLE);
			   }   		 */
#endif	
#if 0
				   iii++;

			       PUMP4_FC = ((iii % 4) == 0); 
				   PUMP4_PF = ((iii % 4) == 1); 
				   PUMP5_PF = ((iii % 4) == 2); 
				   PUMP5_FC = ((iii % 4) == 3); 

				   if ((iii % 4) == 3)
				   {
				   		delay_ms(200);
						PUMP5_FC = OFF;
				   }
				  


#endif		   			  
///////////////////////////////////////
				ReadRTC();
		
				if(new_day_starts)
				{
					new_day_starts = 0;
					ScheduleUpdate();

				}
    			
			 	task_scheduler();

				UpdateCommData();	  //20210618
	
				IWDG_FEED
		     
			}															  
        }
    }  
}
