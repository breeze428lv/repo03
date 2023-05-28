/************************************版权申明********************************************
**                            深圳英捷思科技有限公司
**                             
**  Core methodology:
**	The fc is running as an independent flexible pressure source, when a pump is set to fc mode, 
**  it is connected to fc and outputs soft pressure under control of pid algorithm
**     
**	The pumps act as pressure buckets.	
**	We can regard the pump team as a whole. A pf-bucket contains a pressure block
**	while fc-bucket is loaded with a part of it, working in a fine-tune manner.

**	fc and pf form a super-fc putting all the pf-input and the only fc-input together,
**	when fc-overflow occurs, the super-fc is still running smoothly, with newly created
**	pressure block being reserved and the next bucket is selected as the new 
**	fc-bucket(mod1), or transfered to a new bucket by emptying fc-bucket
**	and filling up the 1st empty bucket in the waiting queue (mod2)

** ---------------------------------- Key Variables -----------------------------------
   ******** This is a useful checklist for both coding and debugging ********

			amount_of_running_pumps
			focused_pump_index
			pump_on_off_tbl[6]

			pump_usablilty_tbl[6]			
			pump_enable_tbl[6]
		
			pump_running_mode_tbl[6] 
 			pump_error_tbl[6]


			Tartget_Pressure
--------------------------------------------------------------------------------------*/
#include "hmi_driver.h"

#include "cmd_queue.h"
#include "cmd_process.h"
#include "stdio.h"
#include "stdbool.h"
#include "hw_config.h"
#include "ulitity.h"
#include "string.h"
#include "stdlib.h"
#include "stm32f10x.h"
#include "usart.h"
#include "relays.h"
#include "delay.h"
#include "adc.h"
#include "hmi_user_uart.h"
#include "hw_config.h"
#include "macros.h"
#include "measurement.h"
#include "settings.h"
#include "pump_sleeping.h"
#include "error_handle.h"
#include "pump_running.h"
#include "time_control.h"
#include "security.h"
#include "UserInterface.h"
#include "ui_controls.h"
#include "api_comm.h"

#define     u8              unsigned char
#define     u16             int
#define     MODEL_P         1
#define     MODEL_PI        2
#define     MODEL_PID       3

#define     Pid_Kp   pid.Kp
#define     Pid_Ki   pid.Kp * pid.T / pid.Ti
#define     Pid_Kd   pid.Kp * pid.Td / pid.T 


#define RED_COLOR                   0xf800
#define BLUE_COLOR                  0x001f

#define    WIN1                        0
#define    WIN2                        1

#define    OVER_ROOF                               OutletRealTimePressure > PressureRoof
#define    UNDER_FLOOR                             OutletRealTimePressure < PressureFloor
#define    OVER_FLOOR                              OutletRealTimePressure >= PressureFloor
#define    UNDER_ROOF                              OutletRealTimePressure <= PressureRoof

#define    EXIT_WIND1                              win1_counter == 0
#define    EXIT_WIND2                              win2_counter == 0


#define    WIN1_MOVE_A_STEP                        if(win1_counter > 0) win1_counter--;
#define    WIN2_MOVE_A_STEP                        if(win2_counter > 0) win2_counter--;

#define    FC_EVENT_NONE                           0
#define    FC_EVENT_FULL                           1
#define    FC_EVENT_EMPTY                          2


#define    TRAFFIC_INQUIRY_ID_PUMP_MANAGER		   0

#define    FC_IS_STILL_EMPTY  (pid.curr >= pid.set)     //		20210623

#define    FC_FLOOR_DELTA					       1

#define    FREEZE_RESET                            2

//Traffic Light 
#define    OUT_OF_GREY_ZONE_HALF_SLEEP_N_MAIN_PUMP  (pump_waking_up_from_half_phase1 == FALSE) && (pump_waking_up_from_half_phase2 == FALSE)\
													 && (pump_waking_up == FALSE) //20200408
#define    PID_GREEN_LIGHT             (SysRunning == TRUE) && (Pump_Status != ALL_SLEEP) && (AllPumpsDead == FALSE) && \
										OUT_OF_GREY_ZONE_HALF_SLEEP_N_MAIN_PUMP

#define    FC_MANAGER_GREEN_LIGHT      (SoftStopping == FALSE) && (pump_waking_up_from_half_phase1 == FALSE)&&\
										 (pump_waking_up_from_half_phase2 == FALSE) && (pump_waking_up == FALSE)\
										 &&(AllPumpsDead == FALSE)
								
									
#define    PID_CALC_GREEN_LIGHT         !((AvfMode == OFF) && (PumpGroupMode == 2) && (pump_usablilty_tbl[0] == FALSE) && (Pump_Status == MAIN_PUMP_ON))	//20210624
#define    RUNNING_GREEN_LIGHT         (pump_startup_done == TRUE) && (pump_switch_phase == 0) && (pump_fatige_switch_phase == 0) &&\
									   (pump_switch_phase_mod2 == 0)	 //2020-4-1
								

#define    NOT_STARTING                (pump_startup_done == TRUE)
#define    NOT_ADDING                  (pump_switch_phase == 0) && (pump_switch_phase_mod2 == 0) && (adding_pump_confirming == FALSE)
#define    NOT_CANCELING               (canceling_pump == FALSE)
#define    FATIGE_SWITCH_GREEN_LIGHT   NOT_ADDING && NOT_CANCELING && NOT_STARTING 	//20200319
									
#define    FATIGE_EXCHANGE             pump_fatige_switch_exch_mode == TRUE

#ifdef DBG_FUNC_INFO
	 extern uint8  func_dbg_info;
#endif

#ifdef DBG_FUNC_INFO
	 uint8 FuncSwitches[8] = {0, 0, 0, 0, 0, 0, 0, 0};
#endif

#ifdef DBG_SHOW_NAMES
	 const char* algo_names[2] = {
									  "PID",
	 							      "LINEAR"
	 };
#endif


#ifdef DBG_FUNC_FOOTPRINT
	 extern uint8 dbg_func_depth;
	 extern uint8 dbg_indent_ind;
#endif	

#ifdef DBG_PUMP_MANAGER
	 float voltage_out;
#endif

#define CANCEL_FREQ_UNACTIIVATED    0				  //20210710
#define CANCEL_FREQ_ACTIIVATED      1
uint8 status_for_cancel = CANCEL_FREQ_UNACTIIVATED;

float PumpMaxFreq;	//20210911

char DbgBuff[100];

float TargetLockerDelta;

uint16 pFactor, iFactor, dFactor;

extern uint8 Use60HzAsMaxFreq;
extern uint8 RemoteTargetEnable;

extern uint8 MaxFreqOut;

extern TYPE_FATIGUE_PUMP_CANCEL FatiguePumpCancel;

extern uint8 AvfMode;

extern float RemoteTargetPressure;
extern bool RemoteMode;

extern float FcFreqOutBias;
extern uint16 sleep_chk_timer;
extern uint8 NewMinuteEvent;
extern uint8 fast_mode;
extern uint16 RunningCounter;

extern uint8 AllPumpsDead;

//pump sleeping
extern uint8 pump_waking_up;

extern uint8 pump_waking_up_from_half_phase1;

extern uint8 pump_waking_up_from_half_phase2;

extern uint8 pump_half_waking_up;

extern uint8 pump_falling_asleep;

//Pump running
uint8 pump_switch_phase = 0;
uint8 pump_fatige_switch_phase = 0;
uint8 pump_fatige_switch_exch_mode;

uint8 pump_switch_phase_mod2 = 0;

uint8 pump_startup_done = FALSE;
uint8 old_pump_index;
uint8 add_now;
uint8 cancel_now;
uint8 adding_pump_confirming = FALSE;
uint8 adding_pump_executing = FALSE;
uint8 canceling_pump = FALSE;

uint8 SparePumpMode = 0;



uint8 AlgoType;

float SpeedUpFactor = 2.0;//20210712

/*
	Tartget_Pressure:
	---- Local Mode:
			--- IN TC-WINDOW
				Day.Task[i].TargetPressure
			--- DAY OFF
				0.0 MPa
			--- TC-IDLE GAP: 
				Default
				 	--- IO control
					--- Menu
						Set by:
							--- TOUCH SCREEN
							--- MODBUS
	--- Remote Mode:
			Remote Target Pressure
*/
float Tartget_Pressure;

uint16 stop_cnt;
int stop_pointer;
uint8 PumpPointer;
uint8 rec_focused_pump_index;


uint8 PumpGroupMode = 1;
uint16 Win1_Counter_Setting = 30;
uint16 Win2_Counter_Setting = 20;

#ifdef USE_PUMP_ADDING_WAITING_UINIT
	uint32 freq_up_counter_gap = 0;
	uint32 freq_up_counter = 0; //20210415
#endif

uint32 win1_counter = 0;
uint32 win2_counter = 0;
uint8 active_win;

uint8 amount_of_running_pumps = 0;
uint8 Index_Pf_Queue_Head;
uint8 FC_Pump_Index;
uint8 index_pf_queue_head;

uint8 focused_pump_index = 0; //Points at the most recently activated pump.
							  // normal running
							  // --mode1: the currently working fc pump
							  // --mode2: if there are pf pumps running, it is the latest added pf pump.
							  //          otherwise if pump1 is the only running one, it is pump1
							  //sleeping  
							  // --shallow: small pump
							  // --deep:    --

uint8 win1_counter_started;

uint8 f_switch_new_fc_pump_index;


extern uint8 SleepingEnableMode;

extern uint8 ErrorCounter;

extern uint8 Pump_Status; 

extern uint8 Sys_Switches[];
extern uint8 Valve_Control1[];

extern uint8 Pump_Switch_Condtion[];
extern uint8 Max_Pump_Nbr;

float PressureRoof = 1.5;
float PressureFloor = 1.0;

uint8 FC_Event;

extern uint8 TickPidCounter;

extern uint8 WaterSourceMode;

#ifdef DUAL_WATER_SOURCE
extern uint16 TankPeriodCounter;

extern uint16 TankOnCounter;
#endif

extern uint8 SysRunning;

extern float phy_quantities[];
extern float OutletRealTimePressure;
extern float DefaultTargetPressure;
extern uint8 PID_Setting[];
extern bool AllPumpsOn;
extern uint8 SoftStopping;

extern uint8 SysStandby;

uint8 Demo = OFF;

//Fatige switching
bool fc_pump_only;


/*-------------------------------------------------------------------------------------------

   This is a usablilty table for 4 pumps (pump0 - pump3)
   pump i is usable if: 
   		1. it is selected by user(pump_enable_tbl[i] == 1)
		2. it is error-free

   The number of usable is no more than	 Max_Pump_Nbr;
   when a group of pumps are started, this table is set according to 
   Max_Pump_Nbr and pump_enable_tbl[]
   once pump_enable_tbl[i] is set to 1, it can be turned on by pump-running process
   (put in use while is not necessarily running at the moment)

--------------------------------------------------------------------------------------------*/
uint8 pump_usablilty_tbl[6] = {0, 0, 0, 0, 0, 0};   //pump_usablilty_tbl[i] == 0 : pump i has an error or disabled by user

uint8 pump_error_tbl[6] = {0, 0, 0, 0, 0, 0}; 

uint8 pump_enable_tbl[6] = {1, 1, 1, 1, 1, 0};
uint8 pump_on_off_tbl[6] = {OFF, OFF, OFF, OFF, OFF, OFF};
uint8 pump_running_mode_tbl[6] = {MOD_PF, MOD_PF, MOD_PF, MOD_PF, MOD_PF, MOD_PF};


char* mod_strings[3] = {
							"工频",
							"变频",
							""

};

char* cmd_strings[3] = {
							"",
							"SET_TEXT",
							"SET_PROGRESS"

};

#define CMD_BUFF_LEN                 30
uint16 pump_cmd_buffer[CMD_BUFF_LEN][4] = {
											{CMD_NUL, 0,0,0},
											{CMD_NUL, 0,0,0},
											{CMD_NUL, 0,0,0},
											{CMD_NUL, 0,0,0},
											{CMD_NUL, 0,0,0},
											{CMD_NUL, 0,0,0},
											{CMD_NUL, 0,0,0},
											{CMD_NUL, 0,0,0},
											{CMD_NUL, 0,0,0},
											{CMD_NUL, 0,0,0},
											{CMD_NUL, 0,0,0},
											{CMD_NUL, 0,0,0},
											{CMD_NUL, 0,0,0},
											{CMD_NUL, 0,0,0},
											{CMD_NUL, 0,0,0},
											{CMD_NUL, 0,0,0},
											{CMD_NUL, 0,0,0},
											{CMD_NUL, 0,0,0},
											{CMD_NUL, 0,0,0},
											{CMD_NUL, 0,0,0},
											{CMD_NUL, 0,0,0},
											{CMD_NUL, 0,0,0},
											{CMD_NUL, 0,0,0},
											{CMD_NUL, 0,0,0},
											{CMD_NUL, 0,0,0},
											{CMD_NUL, 0,0,0},
											{CMD_NUL, 0,0,0},
											{CMD_NUL, 0,0,0},
											{CMD_NUL, 0,0,0},
											{CMD_NUL, 0,0,0},						  
};




uint8 cmd_head_index = 0;
uint8 cmd_tail_index = 0;
 
typedef struct
{
    u8 choose_model;             //使用哪个模式调节
    
    float curr;                  //当前值
    float set;                   //设定值
    
 
    float En;                    //当前时刻
    float En_1;                  //前一时刻
    float En_2;                  //前二时刻
        
    float Kp;                    //比例系数
    float T;                     //采样周期---控制周期，每隔T控制器输出一次PID运算结果
    u16   Tdata;                 //判断PID周期到没到
    float Ti;                    //积分时间常数
    float Td;                    //微分时间常数
    
    float Dout;                  //增量PID计算本次应该输出的增量值--本次计算的结果
    float OUT0;                  //一个维持的输出，防止失控
    
    float curr_output;       
	float prev_output;      
    u16 max_output;              // 
 
}PID;
 
extern volatile  uint32 timer_tick_count; 
uint32  timer_tick_count_rec; 
uint32  f_switch_timer_tick_count_rec; 

uint32  timer_tick_count_rec_mod2; 

uint32  timer_tick_count_rec_spep;

#define    SPARE_PUMP_MODE_NORMAL              0
#define    SPARE_PUMP_MODE_ERROR               1
int new_pump_ind;
uint8 waiting_for_green_light = FALSE;

uint8 err_pump_ind;

uint8 PumpRunningPausing;

uint8 SinglePumpErrPausing;


extern uint8  cmd_buffer[];
extern u8 STATUS;
extern PID pid;
void PIDParament_Init(void);  /*增量式PID初始化*/
void pid_calc(void);                  /*pid计算 并输出*/
 
uint16 FuelMeter[6] = {20, 20, 20, 20, 20, 20};
 
PID pid;

/*Enable the relevant Macro if you want highlight a variable in debugging information*/
#ifdef DBG_PUMP_MANAGER	

//#define  HIGHLIGHT_INDEX0 
#define  HIGHLIGHT_INDEX1 
//#define  HIGHLIGHT_INDEX2              
void show_on_off(void)
{
 	uint8 i;
	printf("\n\n");
#ifdef HIGHLIGHT_INDEX0
	printf("****************************************");
#endif
	printf("amount_of_running_pumps     = %d\n",amount_of_running_pumps);		//0
#ifdef HIGHLIGHT_INDEX1
	printf("****************************************");
#endif
	printf("focused_pump_index          = %d\n",focused_pump_index);			//1
#ifdef HIGHLIGHT_INDEX2
	printf("****************************************");
#endif
	printf("index_pf_queue_head         = %d\n",index_pf_queue_head);			//2

	printf("on/off        : [ ");
	for(i = 0; i < 4; i++)
	{
		printf("%d  ", pump_on_off_tbl[i]);										//3
	}
	printf("]\n\n");
	printf("working mode  : [ ");
	for(i = 0; i < 4; i++)
	{
		printf("%d  ", pump_running_mode_tbl[i]);								//4
	}
	printf("]\n\n");

	printf("pump_usablilty: [ ");
	for(i = 0; i < 4; i++)
	{
		printf("%d  ", pump_usablilty_tbl[i]);									//5
	}
	printf("]\n\n");
	
	printf("pump_error_tbl: [ ");
	for(i = 0; i < 4; i++)
	{
		printf("%d  ", pump_error_tbl[i]);									    //6
	}
	printf("]\n\n");
}

void OSD_DbgInfo(void)
{
	sprintf(DbgBuff, "amount_of_running_pumps = %d",amount_of_running_pumps);
    debugger(DbgBuff, DBG_DATA);
	sprintf(DbgBuff, "focused_pump_index          = %d\n",focused_pump_index);
    debugger(DbgBuff, DBG_DATA);
	sprintf(DbgBuff, "on_off:  [%d %d %d %d] ", pump_on_off_tbl[0], pump_on_off_tbl[1], pump_on_off_tbl[2], pump_on_off_tbl[3]);	
    debugger(DbgBuff, DBG_DATA);
	sprintf(DbgBuff, "working mode:  [%d %d %d %d] ",  pump_running_mode_tbl[0],  pump_running_mode_tbl[1],  pump_running_mode_tbl[2],  pump_running_mode_tbl[3]);	
    debugger(DbgBuff, DBG_DATA);

	sprintf(DbgBuff, "usablilty:  [%d %d %d %d] ", pump_usablilty_tbl[0], pump_usablilty_tbl[1], pump_usablilty_tbl[2], pump_usablilty_tbl[3]);	
    debugger(DbgBuff, DBG_DATA);
	sprintf(DbgBuff, "error:  [%d %d %d %d] ", pump_error_tbl[0], pump_error_tbl[1], pump_error_tbl[2], pump_error_tbl[3]);	
    debugger(DbgBuff, DBG_DATA);
}
#endif

#ifdef TRAFFIC_LIGHT
void traffic_light(uint8 color)
{
	//UNUSED
}
#endif

#ifdef DBG_USE_FUEL_MONITOR
/**********************************************************************************************
 * Name：FuelMonitor()
 * Brief：Monitor the running time(reading of fuel meter) of the pump team, 
 *        switch to a new one if a pump has been running for specified time
 *  
 * Para：    	
 * Return ： 
 * 
 * Caller(s):  main.c/main() 
 **********************************************************************************************/
void FuelMonitor()
{
	 uint8 i;
	 for(i = 0; i < 4; i++)
	 {
		 if(pump_on_off_tbl[i] == ON)
		 {
		  	if(FuelMeter[i] > 0) 
			{
				FuelMeter[i]--;
				if(FuelMeter[i] == 0) 		 //time out, need a rest and switch to another pump
				{
					printf("Pump%d needs a rest!!!\n\n", i);
					use_new_pump(i);
				}
			}
		 }
		 else
		 {
		    if(FuelMeter[i] < 100) FuelMeter[i]++;
		 }			
	 }
}
#endif

/*pid.curr_output--->Freq.*/
float LoadFrequency(void)
{
	return pid.curr_output/300.0;
}

/**********************************************************************************************
 * Name： save_pump_pointer()
 * Brief：Save the index of current fc pump in floating fc mode
 *  
 * Para：    	
 * Return ： 
 * 
 * Caller(s):  main.c/main() 
 **********************************************************************************************/
/*************Variable Description******************
*												   *
*        PID_Setting[0]: X * 10 + Y                *
*        X: PumpPointer                            *
*        Y: SELF_DEF_PID_PAR                       *
*                                                  *
***************************************************/

void save_pump_pointer()
{
	 PID_Setting[0] = PumpPointer * 10 + PID_Setting[0] % 10;
	 SaveSettingsToFlash(SCREEN_PID_SETTING);	
}

void execute_cmd(uint8 cmd, uint16 screen_id,  uint8 control_id, uint16 value)
{
	char buff[10];
	switch (cmd)
	{
		case CMD_SET_TEXT:
			if(value != TXT_VALUE_FOR_NUL)
				sprintf(buff, "%d", value);
			else	  //"--"
				sprintf(buff, "--");
			SetTextValue(screen_id, control_id, (uchar*)buff);
			
			break;

		case CMD_SET_BUTTON:
			SetButtonValue(screen_id, control_id, value);
			
			break;
			
		case CMD_SET_PROGRESS:		
			SetProgressValue(screen_id, control_id, value);
			break;

		case CMD_SET_FG_COLOR:		
			
			SetControlForeColor(screen_id, control_id, value);
			break;
			
		case CMD_SET_VISIBILITY:
			SetControlVisiable(screen_id, control_id, value);
			break;

		case CMD_SET_USABILITY:
			SetControlEnable(screen_id, control_id, value);
			break;

			
		case CMD_API_READ_FREQUENCY:
			AnswerApiReadFrequency();
			break;	

		case CMD_API_READ_ERRORS:
			AnswerApiReadErrors();
			break;	//	AnswerApiReadOutlet(void);

		case CMD_API_READ_OUTLET_PRESSURE:
			AnswerApiReadOutlet();
			break;	
				
		case CMD_API_READ_ENTRANCE:
			 AnswerApiReadEntrance();
			break;
#ifdef  USE_NEW_API_PROTOCOL
		case CMD_API_READ_PUMPS:
			AnswerApiReadPumps();
			break;
#else					
		case CMD_API_READ_PUMP1:
			AnswerApiReadPump(0);
			break;	
				
		case CMD_API_READ_PUMP2:
			AnswerApiReadPump(1);
			break;		

		case CMD_API_READ_PUMP3:
			AnswerApiReadPump(2);
			break;		

		case CMD_API_READ_SMALL_PUMP:
			AnswerApiReadPump(3);
			break;		
#endif
		case CMD_API_READ_TARGET_PRESSURE:
			AnswerApiReadTargetPressure();
			break;		

		case CMD_API_READ_DEFAULT_PRESSURE:
		    AnswerApiReadDefaultPressure();		  //
			break;	

		case CMD_API_READ_PUMP1_TEMP:
		    AnswerApiReadPump1Temp();		  //
			break;	

		case CMD_API_READ_PUMP2_TEMP:
		    AnswerApiReadPump2Temp();		  //
			break;	

		case CMD_API_READ_PUMP3_TEMP:
		    AnswerApiReadPump3Temp();		  //
			break;	

		case CMD_API_READ_SMALL_PUMP_TEMP:
		    AnswerApiReadSmallPump();		  //
			break;	
				
		case CMD_API_WRITE_DEFAULT_PRESSURE:
			AnswerApiWriteDefaultPressure(screen_id, control_id);   // screen_id  control_id	: default target pressure
			break;	
				
		case CMD_API_SET_PRESSURE:
			sprintf(buff, "%0.2f", (float)(value / 1000.0));
			SetTextValue(screen_id, control_id, (uchar*)buff);
			break;

		case CMD_API_SET_NON_NEGATIVE:
			AnswerApiSetNonNeagtive();
		
			break;

		case CMD_API_SET_SMALL_PUMP_ON:		
			AnswerApiSetSmallPumpOn();
			break;

		case CMD_API_SET_FACTORY_VALUES:
			AnswerApiSetFactoryValues();
			break;
			
		default:
			
			break;		
	}
} 

#if 0	//DISABLED
void push_cmd_pump(uint8 cmd, uint16 screen_id, uint8 control_id, uint16 value)
{
	
	pump_cmd_buffer[cmd_tail_index][0] = cmd;
	pump_cmd_buffer[cmd_tail_index][1] = screen_id;
	pump_cmd_buffer[cmd_tail_index][2] = control_id;
	pump_cmd_buffer[cmd_tail_index][3] = value;		

	cmd_tail_index++;

#ifdef DBG_BUFFER_WATCHDOG
	printf("..............................[%d, %d, %d, %d] ",
									cmd, screen_id, control_id, value);
	if(cmd_tail_index > CMD_BUFF_LEN - 1)	printf("pump_cmd_buffer overflow!!!\n\n");
	else   printf("pump_cmd_buffer safe\n\n");
#endif

}

void take_cmd_pump()
{
	if(cmd_head_index == cmd_tail_index)//empty buffer
	{
#ifdef DBG_BUFFER_WATCHDOG
     	printf("cmd buffer is empty!!!\n");
#endif
	}		
	else
	{
		execute_cmd(pump_cmd_buffer[cmd_head_index][0], pump_cmd_buffer[cmd_head_index][1],	pump_cmd_buffer[cmd_head_index][2],pump_cmd_buffer[cmd_head_index][3]);
		cmd_head_index++;
		if(cmd_head_index == cmd_tail_index) //empty now
		{
			cmd_head_index = 0;
			cmd_tail_index = 0;	
		}
	}
}
#else
void push_cmd_pump(uint8 cmd, uint16 screen_id, uint8 control_id, uint16 value)
{
#ifdef DBG_BUFFER_WATCHDOG
	printf("TRY TO PUSH: [%d, %d, %d, %d] \n",
									cmd, screen_id, control_id, value);
#endif

	
	if((cmd_tail_index + 1) % CMD_BUFF_LEN == cmd_head_index)
	{
#ifdef DBG_BUFFER_WATCHDOG	
	printf("but pump_cmd_buffer is full!! skip the cmd\n\n");
#endif		
	}
	else
	{
	
		pump_cmd_buffer[cmd_tail_index][0] = cmd;
		pump_cmd_buffer[cmd_tail_index][1] = screen_id;
		pump_cmd_buffer[cmd_tail_index][2] = control_id;
		pump_cmd_buffer[cmd_tail_index][3] = value;		
	
		cmd_tail_index = (cmd_tail_index + 1) % CMD_BUFF_LEN;
#ifdef DBG_BUFFER_WATCHDOG	
	printf("pump_cmd_buffer is not full!! successfully push the cmd\n\n");
#endif	
	}	

}
//Every 100ms
void take_cmd_pump()
{
	if(cmd_head_index == cmd_tail_index)//empty buffer
	{
#ifdef DBG_BUFFER_WATCHDOG
     	printf("cmd buffer is empty!!!\n");
#endif
	}		
	else
	{
		execute_cmd(pump_cmd_buffer[cmd_head_index][0], pump_cmd_buffer[cmd_head_index][1],	pump_cmd_buffer[cmd_head_index][2],\
														pump_cmd_buffer[cmd_head_index][3]);
		cmd_head_index = (cmd_head_index + 1) % CMD_BUFF_LEN;
#ifdef DBG_BUFFER_WATCHDOG
     	printf("cmd buffer is not empty, a cmd successfully executed\n");
		printf("【%d  %d  %d  %d】\n", pump_cmd_buffer[cmd_head_index][0], pump_cmd_buffer[cmd_head_index][1],	pump_cmd_buffer[cmd_head_index][2],\
														pump_cmd_buffer[cmd_head_index][3]);
#endif
	}
}
#endif



#define  DAC_FREQ_RATIO      63.64
#define  FREQ_SHRINK_FACTOR  (50.0 / MaxFreqOut)

#define METHOD1	   //old method: dac-out range 0-2.5V

/********************************************************************************************************
	Output pid.curr_output to DAC, which outputs an analog voltage to control FC for focused pump
*********************************************************************************************************/
void Control_FC()
{
	uint16 dac_value;
	float  frequency;
	frequency = pid.curr_output/300;

	if (AvfMode) 
	{
		UpdatePumpFreq(frequency, focused_pump_index);
		return;
	}
	frequency += FcFreqOutBias;		  //frequency compensation	 0.05v -- 1hz
	if(frequency < 0) frequency = 0;

	
	
#ifdef METHOD1
	//dac_value = (int)(frequency * 63.0);	// 			 50hz-3150-2.5v
	dac_value = (int)(frequency * DAC_FREQ_RATIO * FREQ_SHRINK_FACTOR);	// 			 50hz-3181-2.5v 20210622
#endif



#ifdef DBG_DAC
    printf("frequency = %0.2f,  dac_value = %d, voltage = %0.2fv\n\n", frequency, dac_value, dac_value * 3.3 /4095);     
#endif

 	DAC_SetChannel1Data(DAC_Align_12b_R,dac_value);	
#ifdef DBG_PUMP_MANAGER
	voltage_out = pid.curr_output / 15000.0 * 2.5;
#endif
}

void UpdateDac(float freq, uint8 ch)
{
	uint16 dac_value;
	float  frequency;
    
	frequency = freq;

 	frequency += FcFreqOutBias;		  //frequency compensation	 0.05v -- 1hz
 	if(frequency < 0) frequency = 0;
	
#ifdef METHOD1
	dac_value = (int)(frequency * DAC_FREQ_RATIO * FREQ_SHRINK_FACTOR);	// 			 50hz-3128-2.5v
#endif

	if (ch == 0)
 		DAC_SetChannel1Data(DAC_Align_12b_R,dac_value);	
	else
		DAC_SetChannel2Data(DAC_Align_12b_R,dac_value);	

}

//#define    PWM_VAL_CORRECTION_FACTOR     1.1  //Ver-1.1.8   old：1.0101		//9.9-->10.0
//#define    PWM_VAL_CORRECTION_FACTOR     0.93  //Ver-1.1.9   
#define    PWM_VAL_CORRECTION_FACTOR     0.8783  //Ver-1.2.0 	 20210703
#define    IS_DAC_CH(n)                  (n < 2)
#define    CCR_VALUE(x)                  (int)((665 * 2.5 / 3.3 * x / 50.0) * FREQ_SHRINK_FACTOR * PWM_VAL_CORRECTION_FACTOR + 0.5)

/**
  * @brief  Update the frequency of a pump channel(AVF only)
  * @param  freq: specified frequency
  *         n:    channel nbr(0~5)		 
  * @retval  None
  */
void UpdatePumpFreq(float freq, uint8 n)
{
	if (IS_DAC_CH(n))	  //Dedicated DAC channel
		UpdateDac(freq, n);
	else			      //PWM-DAC channel
	{
		UpdatePWM(CCR_VALUE(freq), n);	
	}
}

void Manual_FC(float frequency)
{
	uint16 dac_value;

#ifdef METHOD1
 	//dac_value = (int)(frequency * 2.0 * 31.28);	//3128->50hz ->2.5v
	dac_value = (int)(frequency * DAC_FREQ_RATIO * FREQ_SHRINK_FACTOR);	//3128->50hz ->2.5v
#endif
  
 	DAC_SetChannel1Data(DAC_Align_12b_R,dac_value);	
}

uint8 WakeUpPressureAlert(uint8 bias)
{
	//the pressure is too low, wake up
	 if(OutletRealTimePressure * 100 < pid.set - bias) return TRUE;
	 return FALSE;
}


uint8 SleepPressureConfirmed()
{
	if(OutletRealTimePressure >= Tartget_Pressure)		return TRUE;
	return FALSE;
}

uint8 GetCurrentFrequency(void)
{
	if(Pump_Status == SMALL_PUMP_ON) return (int)(pid.curr_output/300);
	return 	(int)(pid.curr_output/300) + (amount_of_running_pumps - 1) * 50;
}


void set_pid_factors(void)
{
//#define DBG_SET_PID_FACTORS          //Enabled for debugging only
//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_SET_PID_FACTORS

    printf("\n\n\n Entrance: void set_pid_factors(void)...............\n\n");
#endif  
//--------------------------------------------------------------------------------------------------


	if(PID_Setting[0] % 10 == TRUE)	  //Userdefined PID para
	{
	   pid.T = (float)(PID_Setting[1] / 10.0);
	   pid.Kp = (float)pFactor;
	   pid.Ti = (float)iFactor;
	   pid.Td = (float)dFactor;
	}
	else //Use Default Value
	{
	   pid.T = 0.1; 
 	   pid.Kp = 600.0;//updated from 300.0  20210416
	   pid.Ti = 10;//	updated from 2 20210416
	   pid.Td = 0;//1;
	}
#ifdef DBG_SET_PID_FACTORS
	printf("pid.T = %0.2f\n",pid.T);
	printf("pid.Kp = %0.2f\n",pid.Kp);
	printf("pid.Ti = %0.2f\n",pid.Ti);
	printf("pid.Td = %0.2f\n",pid.Td);
#endif
}
 
void UpdateTargetPressure(void)
{
	if ((!RemoteMode) || (RemoteTargetEnable == FALSE)) PidSetTargetPressure(LoadTargetPressure());		 //2020-7-16
	else  PidSetTargetPressure(RemoteTargetPressure);
#ifdef DBG_FUNC_INFO
	if(func_dbg_info) printf("\nTo PC-API:UpdateTargetPressure    TargetPressure = %0.2f\n", pid.set/100.0);
#endif
}

void PIDParament_Init()   
{
    pid.choose_model = MODEL_PID;
    
	if ((!RemoteMode) || (RemoteTargetEnable == FALSE)) pid.set = LoadTargetPressure() * 100.0;//100;            //用户设定值		 //2020-7-16
	else  pid.set = RemoteTargetPressure * 100.0;

	pid.T = 100;                //采样周期，定时器使用1ms，则最小执行PID的周期为330ms  
	set_pid_factors();	 // 20200327

    pid.OUT0 = 0;                //一个维持的输出
    pid.curr_output = Pump_Switch_Condtion[4] * 300;//0;	 FC_START_STOP_FRQENCY
	pid.prev_output = pid.curr_output;	 //0
 
	pid.En_1 = 9999.0;                //前一时刻
    pid.En_2 = 9999.0;                //前二时刻

	//pid.max_output = 300 * MaxFreqOut; //50hz | 60hz
	//20210911
	pid.max_output = (int)(300 * PumpMaxFreq);  
}


 /*增泵压力偏差：当达到增泵条件后，如果当前压力大于设定压力减设置值，则不进行增泵，
 此参数可有效减少水泵起停，有利管网稳定。*/

char in_target_zone()
{
//                                 增泵压力偏差
	if(pid.curr >= pid.set - Pump_Switch_Condtion[15]) //&& (pid.curr <= pid.set + 3.0)) 
	{
		return TRUE;
	}
	{
		return FALSE;
	}

}
extern TYPE_PUMP_ICONS PumpIcons[];
void update_pump_icon(uint8 ind, uint8 mode)	//???pump id is not correct for pump5 pump6
{
	switch(mode)
	{	  //PumpIcons[ind].icon_pump_

	    case MOD_PF:
			push_cmd_pump(CMD_SET_VISIBILITY, SCREEN_MAIN_1, PumpIcons[ind].icon_pump_pf, VISIBLE);
			push_cmd_pump(CMD_SET_VISIBILITY, SCREEN_MAIN_1, PumpIcons[ind].icon_pump_fc, INVISIBLE);
	   
	      break;
	      
	    case MOD_FC:
			push_cmd_pump(CMD_SET_VISIBILITY, SCREEN_MAIN_1, PumpIcons[ind].icon_pump_pf, INVISIBLE);
			push_cmd_pump(CMD_SET_VISIBILITY, SCREEN_MAIN_1, PumpIcons[ind].icon_pump_fc, VISIBLE);
	     
	      break;
	    case MOD_NUL:
			push_cmd_pump(CMD_SET_VISIBILITY, SCREEN_MAIN_1, PumpIcons[ind].icon_pump_pf, INVISIBLE);
			push_cmd_pump(CMD_SET_VISIBILITY, SCREEN_MAIN_1, PumpIcons[ind].icon_pump_fc, INVISIBLE);
		
			break;
	    default:
	      break;
    }
}


/**********************************************************************************************
 * Name：update_focused_pump_index()
 * Brief：scan from the pump next to current focused one, the 1st standby pump will be the next focused pump.
 * Para：  focused_pump_index, pump_usablilty_tbl, pump_on_off_tbl,    	
 * Return ：a new one if the searching succeeds, otherwise keep it unchanged 
 * 
 * Caller(s):   
 **********************************************************************************************/
void update_focused_pump_index(void)
{	      
	uint8 i;
//#define DBG_UPDATE_FOCUSED_PUMP_INDEX          //Enabled for debugging only
//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_UPDATE_FOCUSED_PUMP_INDEX

    printf("\n\n\n Entrance: void update_focused_pump_index(void)...............\n\n");
#endif  
//--------------------------------------------------------------------------------------------------

	i = (focused_pump_index + 1) % 5;	//20210518	  
	while(i != focused_pump_index)	
	{
#ifdef DBG_UPDATE_FOCUSED_PUMP_INDEX
		printf("i = %d\n",i);
		printf("pump_usablilty_tbl[i] = %d\n", pump_usablilty_tbl[i]);
		printf("pump_on_off_tbl[i] = %d\n", pump_on_off_tbl[i]);
#endif

		if(pump_usablilty_tbl[i] == TRUE)
		{
			if(pump_on_off_tbl[i] == OFF)	 
			{
			    // a normal pump is selected				
			    focused_pump_index = i;
				rec_focused_pump_index = focused_pump_index;		//record the index of current pump
#ifdef DBG_UPDATE_FOCUSED_PUMP_INDEX
				printf("rec_focused_pump_index = %d\n",rec_focused_pump_index);
#endif

				break;				
			}
		}

		i = (i + 1) % 5;	//20210518
	}
}

void Set_FC_StartValue(void)
{
	pid.curr_output = Pump_Switch_Condtion[4] * 300;// - FcFreqOutBias * 300;
	Control_FC();
}

void Set_FC_Zero(void)
{
	FC_SWITCH = OFF;
    pid.curr_output = 0;
	Control_FC();
	SetTextValue(SCREEN_MAIN_1, TXT_CURRENT_FREQUENCY, "0.0");

}

extern TYPE_PUMP_ICONS PumpIcons[];
void show_frequency_for_pump(uint8 index)
{
	char buff[5];

	if (index > 5) return;		//20210624

    //frequency
	if(pid.curr_output/300.0 < 0)  sprintf(buff, "0.0");
	else sprintf(buff, "%.1f", pid.curr_output/300.0);
	if (AvfMode == TRUE)
	{
		SetTextValue(SCREEN_MAIN_1, PumpIcons[index].icon_freq, (uchar*)buff);	  //???
	}
	else
	{
		SetTextValue(SCREEN_MAIN_1, TXT_CURRENT_FREQUENCY, (uchar*)buff);
	}

}

void show_frequency(void)
{
    if ((PumpGroupMode == 1) || (AvfMode == TRUE)) show_frequency_for_pump(focused_pump_index);
	else if(PumpGroupMode == 2)
	{
		if(focused_pump_index != 5) show_frequency_for_pump(0);
		else show_frequency_for_pump(5);
	}
}

extern TYPE_PUMP_ICONS PumpIcons[];
void switch_to_fc(uint8 index)
{
	control_pump(index, MOD_FC, ON);

	//purple
	SetControlVisiable(SCREEN_MAIN_1,PumpIcons[index].icon_pump_fc, VISIBLE);
	SetControlVisiable(SCREEN_MAIN_1,PumpIcons[index].icon_pump_pf, INVISIBLE);

	SHOW_VF_MODE_TXT(index)
	START_WATERFLOW(index)	

	pump_on_off_tbl[index] = ON;		   
	//update running mode
	pump_running_mode_tbl[index] = MOD_FC;

}


void switch_to_pf(uint8 index)
{
	control_pump(index, MOD_PF, ON);

	//blue

	SetControlVisiable(SCREEN_MAIN_1,PumpIcons[index].icon_pump_fc, INVISIBLE);
	SetControlVisiable(SCREEN_MAIN_1,PumpIcons[index].icon_pump_pf, VISIBLE);

	SHOW_PF_MODE_TXT(index)
	START_WATERFLOW(index)	
	//on/off
	pump_on_off_tbl[index] = ON;
			   
	//running mode
	pump_running_mode_tbl[index] = MOD_PF;

}


void pause(uint8 index)
{
	control_pump(index, MOD_PF, OFF);

	//Update Pump icon to OFF
	SetControlVisiable(SCREEN_MAIN_1,PumpIcons[index].icon_pump_fc, INVISIBLE);
	SetControlVisiable(SCREEN_MAIN_1,PumpIcons[index].icon_pump_pf, INVISIBLE);

	CLR_MODE_TXT(index)
	STOP_WATERFLOW(index)

	pump_on_off_tbl[index] = OFF;
	pump_running_mode_tbl[index] = MOD_PF; 
	if(amount_of_running_pumps > 0) amount_of_running_pumps--;

}

void start_up_server(void)
{	 
	 if(pump_startup_done == FALSE)
	 {
		if((timer_tick_count - timer_tick_count_rec) * 10 >= (Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14]))
		{
#ifdef DBG_FUNC_INFO
			printf("FC ON, GREEN1 ON\n");

#endif
			if (AvfMode) switch_to_fc(focused_pump_index);
			else FC_SWITCH = ON;

			Pump_Status = MAIN_PUMP_ON;
		 
		 	RESET_FATIGE_SWITCHING_TIMER

#ifdef DBG_TIMING_CONSULTANT 
 	        TimingConsultant(CMD_RESET_TIME);
#endif		    
			
		  	pump_startup_done = TRUE;
			fast_mode = FALSE;
#ifdef USE_PUMP_ADDING_WAITING_UINIT
			freq_up_counter = timer_tick_count;	  //20210415
#endif
			RefreshErrorInfo(0, ADD_ERROR_NO);	//0229
			

		}
	 }
}

uint8 the_left(uint8 a, uint8 b)
{
	uint8 i;
	for(i = 0; i < 3; i++)
	{
		if((i != a) && (i != b) && (pump_on_off_tbl[i] == ON)) return i;
	}
	return 9;
}

#define NBR_OF_PUMPS      6
/***********************************************************************
 * Scan the pump team from left to right in a circualr manner, 
 * starting from the next of ind to find a running pf pump and return its 
 * index.
 * return 9 for failed search
 ***********************************************************************/
uint8 find_next_pf_head(uint8 ind)
{
	uint8 i;
	int result = 9;
	for(i = (ind + 1)% NBR_OF_PUMPS; i != ind; i = (i + 1) % NBR_OF_PUMPS)		 //20210622
    {
    	if((pump_usablilty_tbl[i] == ON) && (pump_on_off_tbl[i] == ON) && (pump_running_mode_tbl[i] == MOD_PF))
		{
			return i;
		}
    }	
	return result;

}

void escape_pump_adding_loop(void)
{
	pump_switch_phase = 0;
	fast_mode = FALSE;	
}

void StartAnFcPump(void)
{
	/*Core Operation*/
	// Set freq.
	Set_FC_StartValue();
	// Switch foucused pump	to FC mode
	control_pump(focused_pump_index, MOD_FC, ON);		//20210625

	/*Skin Operation*/
	//Show fc pump icon	of purple color
	SetControlVisiable(SCREEN_MAIN_1,PumpIcons[focused_pump_index].icon_pump_fc, VISIBLE);
	SetControlVisiable(SCREEN_MAIN_1,PumpIcons[focused_pump_index].icon_pump_pf, INVISIBLE);
	SHOW_VF_MODE_TXT(focused_pump_index)
	START_WATERFLOW(focused_pump_index)
	show_frequency();

	//Update Variables
	pump_on_off_tbl[focused_pump_index] = ON;
	amount_of_running_pumps++;

}

#define NO_NEW_PUMP_AVAILABLE  	 (find_target_pump(old_pump_index) < 0) || (Pump_Status == SMALL_PUMP_ON)
/*	
    Pump-i    fc...........off..........pf	         pf  		   pf
    Pump-j    off	       off	        off..........fc			   fc
    fc        on...........off          off          off...........on
    fc-freq   50...........0	        0............freq0		   freq0
	--------------------------------------------------------------------------
		      [1]-~~~->A-->[2]-~~~->B-->[4]-~~~->C-->[6]-~~~->D-->Exit 
						|						   |	   
		   				-->[3]-~~~->C-->[5]-~~~->B-^
	--------------------------------------------------------------------------
    i          	...........off          off..........pf	         
    j  		   	           off..........fc	         fc  
    fc        	...........off          off          off   
    fc-freq    		       0............freq0  		 freq0	  

		
			  A:  (fc-off, fc=0)               switch off fc source	  AFC: ---
			  B:  (i:fc->pf)			       pump-i        		  AFC: ---
			  C:  (j:pf->fc, fc=freq0)		   pump-j				  AFC: (j: fc=freq0)
			  D:  (fc-on)		               switch on fc source	  AFC: (j: fc-on)
*/
void AddPumpSwitchManager(void)
{
	int ind;
	static uint16 delta = 0;
	static uint8 new_fc_pump_is_on = FALSE;

//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_FUNC_INFO
    func_dbg_info = OFF;
    if(func_dbg_info) printf("\n\n\n Entrance: void AddPumpSwitchManager(void)...............\n\n");
#endif  
//--------------------------------------------------------------------------------------------------


	switch(pump_switch_phase)
	{
	   case 1:			
			if((timer_tick_count - timer_tick_count_rec) * 10 >= (Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14]))
			{
 				if (AvfMode)	//AVF	  
				{
					ind = find_target_pump(old_pump_index);

					if(ind >= 10) 				 //not full, find a new backup
					{
						printf("not full, find a new backup: pump%d, set 20hz\n", ind - 9);
						focused_pump_index = ind - 10;

						Set_FC_StartValue();
						show_frequency();
					
						
						UNBLOCK_FATIGUE_SWITCH	   //relief any blocked pump  
						pump_switch_phase = 6;
					}
					else
					{
						pump_switch_phase = 0;
						fast_mode = FALSE;
					}					 
				}
				else 	//non-avf
				{
#ifdef DBG_FUNC_INFO
   	
	                if(func_dbg_info)
					{
						printf("                                       End of case1: Set_FC_Zero();\nFCi OFF. pause(disconnect) the current fc pump\n");
						printf("To PC-API:pump_switch_phase1");
						printf("\n");
						show_on_off();
					}

#endif
					new_fc_pump_is_on = FALSE;
					Set_FC_Zero();								   //
					if(pump_on_off_tbl[old_pump_index] == ON)
					{
					    pause(old_pump_index);	  //FCi OFF. pause(disconnect) the current fc pump
						pump_on_off_tbl[old_pump_index] = OFF;
		
					}
				
					if(NO_NEW_PUMP_AVAILABLE)	//the current pump is the only usable one. go all-dead
					{
						AllPumpsDead = TRUE;
						escape_pump_adding_loop();

#ifdef DBG_PUMP_MANAGER
						printf("the current pump is the only usable one. go AllPumpsDead, Exit \n");
#endif
					}

					else
					{
					
						  //FC_PAUSE_TIME_AFTER_ADDING_PUMP        //FC_TO_PF_DELAY
						if(Pump_Switch_Condtion[8] * 1000 > Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14])
						{
							delta = Pump_Switch_Condtion[8] * 1000 - (Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14]);
					    	pump_switch_phase = 2;
	#ifdef DBG_STATE_MACHINE
				SetControlBackColor(SCREEN_MAIN_1, 62, 0xffff);	   //1
			    SetControlBackColor(SCREEN_MAIN_1, 66, 0x07e0);	   //2
	#endif
		
						}
						else
						{
							delta = Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14] - Pump_Switch_Condtion[8] * 1000;
							pump_switch_phase = 3;
	#ifdef DBG_STATE_MACHINE
				SetControlBackColor(SCREEN_MAIN_1, 62, 0xffff);	   //1
			    SetControlBackColor(SCREEN_MAIN_1, 63, 0x07e0);	   //3
	#endif
		
						}
	#ifdef DBG_FUNC_INFO
						if(func_dbg_info)
						{
							printf("变频转工频延时: %d ms\n", Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14]);
							printf("增泵后变频暂停,延时: %d s\n", Pump_Switch_Condtion[8]);
						}
	#endif
						timer_tick_count_rec = timer_tick_count; 
					}
				}
			} 
	   		break;

	   case 2:		
			if((timer_tick_count - timer_tick_count_rec) * 10 >= (Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14]))
			{
				//pf mode ON	Switch当前泵到工频
#ifdef DBG_FUNC_INFO
				if(func_dbg_info)
				{
					printf("                                       End of case2: 启动当前泵%d到工频\n", old_pump_index);
					printf("To PC-API:pump_switch_phase2");
					printf("\n");

					show_on_off();
				}
#endif
				if(pump_usablilty_tbl[old_pump_index] == TRUE) 
				{
					if(find_target_pump(old_pump_index) >= 10)
					{
						switch_to_pf(old_pump_index);
						amount_of_running_pumps++;//20200221
					}
					else	//no pump found at all ,exit
					{
					   pump_switch_phase = 0;
					   fast_mode = FALSE;
					   break;
					}
				}
#ifdef DBG_FUNC_INFO
				if(func_dbg_info)
				{
					printf("泵%d故障，操作取消\n", focused_pump_index);
 					show_on_off();
					printf("----------------amount_of_running_pumps = %d\n",amount_of_running_pumps);
					
					Set_FC_Zero();		   //????
				}
#endif

			    pump_switch_phase = 4;
#ifdef DBG_STATE_MACHINE
				SetControlBackColor(SCREEN_MAIN_1, 66, 0xffff);
			    SetControlBackColor(SCREEN_MAIN_1, 67, 0x07e0);
#endif
				timer_tick_count_rec = timer_tick_count; 
			} 

	   		break;

	   case 3:
	   case 4:
					
	    	if((timer_tick_count - timer_tick_count_rec) * 10 >= ((pump_switch_phase == 3)? Pump_Switch_Condtion[8] * 1000 : delta))
			{
				printf("                                       End of case %d:\n", pump_switch_phase);
				printf("To PC-API:pump_switch_phase%d", pump_switch_phase);
		    	printf("\n");
				
				//Find target pump for fc mode							
				ind = find_target_pump(old_pump_index);
				
				/*  i: old pump index, pump i will be switched to pf
				    j: new pump index, pump j will be switched to fc
				 If pump i is about to switch to pf from fc in case 5(path A: 1-[3]-5-6) or has done it already 
				 in case 2(path B: 1-2-[4]-6), only find that all other pumps unusable(unselected or break down)
				 (fail to find j, while pump j is supposed to be the new fc pump), pump i will retreat to 
				 fc mode(let: j = i, pump_switch_phase = 4 to drive this state machine into path B and 
				 shortcut it to phase6 where fc source is switched on at the exit) */

				//all pumps but the only pf pump(whether on already or waiting for it) can work, switch pf pump to fc
				if((ind < 0) && (pump_usablilty_tbl[old_pump_index] == ON) && (pump_running_mode_tbl[old_pump_index] == MOD_PF))
				{
					ind = old_pump_index;
					pump_switch_phase = 4;	 // will skip to phase 6: pump_running_mode_tbl[old_pump_index] = MOD_FC
					printf("all pumps but the only pf pump can work, switch pf pump to fc\n");
				}
				
				//switch the target to fc if it is available
				if(ind < 0)	   // fail to find one, all pumps can not work
				{
					printf("no pf or free pump available at all, all pumps can not work\n");
					AllPumpsDead = TRUE;
					escape_pump_adding_loop();

#ifdef DBG_PUMP_MANAGER
					printf("fc error occurs in phase3, AllPumpsDead, Exit \n");
#endif

				}
				else	 // find a pump and switch it to fc(either a free one or an overriden pf one),   
				{
					if(ind >= 10) 				 //not full, find a new backup
					{
						printf("not full, find a new backup: pump%d\n", ind - 9);
						focused_pump_index = ind - 10;
						rec_focused_pump_index = focused_pump_index;		//record the index of current pump
					}
					else  //full, override a pf pump
					{
						printf("full, override a pf: pump%d\n", ind + 1);
					    focused_pump_index = ind;
						pause(focused_pump_index);
						rec_focused_pump_index = focused_pump_index;		//record the index of current pump
						
	
					}
				
#ifdef DBG_FUNC_INFO
					if(func_dbg_info)
					{
						printf("启动新泵到变频  \n");
					}
#endif

					//switch fc to the new pump
					switch_to_fc(focused_pump_index);
					amount_of_running_pumps++; 
					new_fc_pump_is_on = TRUE;
					Set_FC_StartValue();	
					show_frequency();		

#ifdef DBG_FUNC_INFO
					if(func_dbg_info)
					{
						show_on_off();
						printf("----------------amount_of_running_pumps = %d\n",amount_of_running_pumps);
					}
#endif

				    pump_switch_phase += 2;
#ifdef DBG_STATE_MACHINE
					if(pump_switch_phase == 5)
					{
						SetControlBackColor(SCREEN_MAIN_1, 63, 0xffff);
					    SetControlBackColor(SCREEN_MAIN_1, 64, 0x07e0);
					}
					else
					{
						SetControlBackColor(SCREEN_MAIN_1, 67, 0xffff);
					    SetControlBackColor(SCREEN_MAIN_1, 65, 0x07e0);
					}

#endif
					timer_tick_count_rec = timer_tick_count; 
				}
			} 
	   		break;

	   case 5:		
			if((timer_tick_count - timer_tick_count_rec) * 10 >= delta)
			{
				//pf mode ON	启动当前泵到工频
#ifdef DBG_FUNC_INFO
				if(func_dbg_info)
				{
					printf("                                       End of case5: 启动当前泵%d到工频 BLUE1 ON\n", old_pump_index + 1);
					printf("To PC-API:pump_switch_phase5");
					printf("\n");
					show_on_off();
				}
#endif

				if(pump_usablilty_tbl[old_pump_index] == TRUE) 
				{
					if((find_target_pump(old_pump_index) >= 10) || (new_fc_pump_is_on == TRUE))//20200304
					{											  //new_fc_pump_is_on == TRUE: Path A: 1->>3->>5->>6, 
					                                              //new fc pump is found already, it is safe to switch on old pump
																  //in pf mode now. Otherwise, it is path B: 1->>2->>4->>6
						switch_to_pf(old_pump_index);
						amount_of_running_pumps++;//20200221
					}
					else	//no pump found at all ,exit
					{

						printf("new fc pump not found! exit...");
					   pump_switch_phase = 0;
					   fast_mode = FALSE;
					   break;
					}
				}
	     		else
				{
#ifdef DBG_FUNC_INFO
					if(func_dbg_info)
					{
						printf("当前泵突发故障，操作取消\n");
					}
#endif

				}	

#ifdef DBG_FUNC_INFO
				if(func_dbg_info)
				{
					show_on_off();
					printf("----------------amount_of_running_pumps = %d\n",amount_of_running_pumps);
				}
#endif



			    pump_switch_phase = 6;
#ifdef DBG_STATE_MACHINE
				SetControlBackColor(SCREEN_MAIN_1, 64, 0xffff);
			    SetControlBackColor(SCREEN_MAIN_1, 65, 0x07e0);
#endif
#ifdef DBG_FUNC_INFO
				if(func_dbg_info)
				{
					printf("变频转工频延时: %d ms\n", Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14]);
				}
#endif
				timer_tick_count_rec = timer_tick_count; 
			} 
	   		break;
	   case 6:
	   				
			if((timer_tick_count - timer_tick_count_rec) * 10 >= (Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14]))
			{
				if (AvfMode)	//AVF	 
				{
					switch_to_fc(focused_pump_index);
					amount_of_running_pumps++; 
					printf("Sart FC-%d\n",focused_pump_index);


	#ifdef USE_PUMP_ADDING_WAITING_UINIT
					freq_up_counter = timer_tick_count;	  //20210415
	#endif
				}
				else
				{
	#ifdef DBG_FUNC_INFO
					if(func_dbg_info)
					{
						printf("                                       End of case6: FC ON, GREEN1 ON\n");
						printf("To PC-API:pump_switch_phase6");
						printf("\n");
						printf("----------------amount_of_running_pumps = %d\n",amount_of_running_pumps);
	
					}
	#endif
				   	if(pump_usablilty_tbl[focused_pump_index] == TRUE)
					{
						FC_SWITCH = ON;
	#ifdef USE_PUMP_ADDING_WAITING_UINIT
						freq_up_counter = timer_tick_count;	  //20210415
	#endif
					}
				}

			    pump_switch_phase = 0;

				status_for_cancel = CANCEL_FREQ_UNACTIIVATED;//20210710

#ifdef DBG_STATE_MACHINE
				SetControlBackColor(SCREEN_MAIN_1, 65, 0xffff);
#endif
				fast_mode = FALSE;
				adding_pump_executing = FALSE;
#ifdef TRAFFIC_LIGHT
    			traffic_light(GREEN);
#endif
#ifdef DBG_PUMP_MANAGER
				debugger("AddPumpSwitchManager: Exit", ON);
				OSD_DbgInfo();
#endif
			} 
	   		break;
		default:
			break;

	 }
#ifdef DBG_FUNC_INFO
    if(func_dbg_info) printf("\n\n\n Exit: void AddPumpSwitchManager(void)...............\n\n");
#endif  

}	   



void AddPumpSwitchManagerMode2(void)
{

//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_FUNC_INFO
    func_dbg_info = OFF;
    if(func_dbg_info) printf("\n\n\n Entrance: void AddPumpSwitchManagerMode2(void)...............\n\n");
#endif  
//--------------------------------------------------------------------------------------------------

	switch(pump_switch_phase_mod2)
	{
	   case 1:				
			if((timer_tick_count - timer_tick_count_rec_mod2) * 10 >= (Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14]))
			{

				if(pump_on_off_tbl[0] == ON)
				{
				    pause(0);	  //FC0 OFF. pause(disconnect) the current fc pump
					Set_FC_Zero(); 
				}
#ifdef DBG_PUMP_MANAGER
				printf("FC0 OFF. pause(disconnect) the current fc pump1\n");
				show_on_off();
#endif				

							
#ifdef DBG_FUNC_INFO
				if(func_dbg_info)
				{
					printf("增泵后变频暂停,延时: %d s\n", Pump_Switch_Condtion[8]);
				}
#endif					 
				pump_switch_phase_mod2 = 2;
				timer_tick_count_rec_mod2 = timer_tick_count; 
#ifdef DBG_STATE_MACHINE
				SetControlBackColor(SCREEN_MAIN_1, 62, 0xffff);	   //1
			    SetControlBackColor(SCREEN_MAIN_1, 66, 0x07e0);	   //2
#endif				
			} 
	   		break;

	   case 2:			
			if((timer_tick_count - timer_tick_count_rec_mod2) * 10 >= Pump_Switch_Condtion[8] * 1000)
			{

#ifdef DBG_FUNC_INFO
				if(func_dbg_info)
				{
					printf("启动泵1到FC \n");
				}
#endif
#ifdef DBG_PUMP_MANAGER
				
				show_on_off();
#endif
				
				if(pump_usablilty_tbl[0] == TRUE) 
				{
				   Set_FC_StartValue();
				   show_frequency();

				   switch_to_fc(0);
				   
				   amount_of_running_pumps++;

				}

#ifdef DBG_PUMP_MANAGER
				show_on_off();
				printf("----------------amount_of_running_pumps = %d\n",amount_of_running_pumps);
#endif

			    pump_switch_phase_mod2 = 3;
				timer_tick_count_rec_mod2 = timer_tick_count; 
#ifdef DBG_STATE_MACHINE
				SetControlBackColor(SCREEN_MAIN_1, 66, 0xffff);	   //
			    SetControlBackColor(SCREEN_MAIN_1, 63, 0x07e0);	   //2 -> 3
#endif
			} 

	   		break;

	   case 3:				
			if((timer_tick_count - timer_tick_count_rec_mod2) * 10 >= Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14])
			{

#ifdef DBG_FUNC_INFO
				if(func_dbg_info)
				{
					printf("FC ON\n");
				}
#endif
			   	if(pump_usablilty_tbl[0] == TRUE)
				{
					FC_SWITCH = ON;
#ifdef USE_PUMP_ADDING_WAITING_UINIT
					freq_up_counter = timer_tick_count;	  //20210415
#endif
				}
			    pump_switch_phase_mod2 = 0;
				status_for_cancel = CANCEL_FREQ_UNACTIIVATED;//20210710
#ifdef DBG_STATE_MACHINE
				SetControlBackColor(SCREEN_MAIN_1, 63, 0xffff);	   //0
#endif
				fast_mode = FALSE;
				adding_pump_executing = FALSE;
			} 
	   		break;
		default:
			break;

	 }
#ifdef DBG_FUNC_INFO
    if(func_dbg_info) printf("\n\n\n Exit: void AddPumpSwitchManagerMode2(void)...............\n\n");
#endif  

}

/**************************************************************************
* In fatige switching process(Mode1, only 2 of the selected 3 pumps 
* are permitted to run), suppose the current running pattern is
* [off  pf  fc], pump2 is switched off and  pump1 will be selected
* as the new fc pump while pump3 will work in pf. To prevent pump2 
* from being wrongly selected as fc pump, this "freeze" funtion is 
* designed as a shield. as soon as pump2 is off,it is called to freeze 
* pump2, which will be set free at the exit of switching process.
* example: freeze_pf_pump(1, ON)  freezes pump2
*          freeze_pf_pump(0, OFF) set free the previously frozen pump(not neccesarily pump1)
***************************************************************************/
void freeze_pf_pump(uint8 ind, uint8 cmd)
{
	static uint8 the_ind = 0;
	static bool frozen = false;
	if(cmd == ON)
	{
		the_ind = ind;
		pump_usablilty_tbl[the_ind] = FALSE;
		frozen = true;
	}
	else if(frozen)
	{
		pump_usablilty_tbl[the_ind] = TRUE;
		frozen = false;
	}
}



#ifdef DBG_EVENT_AS_ERROR
uint8 who_is_pf(void)
{
   uint8 i;
   for(i = 0; i < 3; i++)
   {
	   if((pump_on_off_tbl[i] == ON) && (pump_running_mode_tbl[i] == MOD_PF)) return i + 1;
   }
   return 0;
}
#endif
/*

	 Currently a PF pump has been stopped already, the FC pump(FCi) is paused(case 1) and then
	 started in PF(case 2), a new spare pump j is found and started in FC(case 4)

						 Pump-h           Pump-i            Pump-j
  before switch	     	   PF               FC               OFF
  						 >>>>>>>---------RIGHT SHIFT-------->>>>>>				                             
  after switch 			   OFF              PF               FC


        h---PF--->i---FC--->j---OFF--->(h)

*/	   
void FatigeSwitchManager(void)
{

	int ind;
	static uint16 delta = 0;
	static uint8 new_fc_pump_is_on = FALSE;

#ifdef DBG_EVENT_AS_ERROR
	char buff[50];
#endif

//#define DBG_FATIGESWITCHMANAGER          //Enabled for debugging only
//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_FATIGESWITCHMANAGER
    printf("\n\n\n Entrance: void FatigeSwitchManager(void)...............\n\n");
#endif  
//--------------------------------------------------------------------------------------------------

	switch(pump_fatige_switch_phase)
	{
	   case 1:
	   		printf("case 1\n");				
			if((timer_tick_count - f_switch_timer_tick_count_rec) * 10 >= (Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14]))
			{			
#ifdef DBG_PUMP_MANAGER
				printf("Set_FC_Zero();\nFCi OFF. pause(disconnect) the current fc pump\n");
				show_on_off();
#endif
				new_fc_pump_is_on = FALSE;
				Set_FC_Zero();								   //0 freq. --> FC
				if(pump_on_off_tbl[old_pump_index] == ON)	   //FCi is ON
				{
				    pause(old_pump_index);	  //FCi OFF. pause(disconnect) the current fc pump
					pump_on_off_tbl[old_pump_index] = OFF;
	
				}
			
				if(NO_NEW_PUMP_AVAILABLE)	//the current pump is the only usable one. go all-dead
				{
					AllPumpsDead = TRUE;
					escape_pump_adding_loop();

#ifdef DBG_PUMP_MANAGER
					printf("the current pump is the only usable one. go AllPumpsDead, Exit \n");
#endif
				}

				else 
				{
					if(!fc_pump_only)	 //normal adding process if there is one pf pump running
					{
					
						 //FC_PAUSE_TIME_AFTER_ADDING_PUMP        //FC_TO_PF_DELAY
						if(Pump_Switch_Condtion[8] * 1000 > Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14])
						{
							delta = Pump_Switch_Condtion[8] * 1000 - (Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14]);
					    	pump_fatige_switch_phase = 2;
		
						}
						else
						{
							delta = Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14] - Pump_Switch_Condtion[8] * 1000;
							pump_fatige_switch_phase = 3;
		
						}
	#ifdef DBG_FUNC_INFO
						if(func_dbg_info)
						{
							printf("变频转工频延时: %d ms\n", Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14]);
							printf("增泵后变频暂停,延时: %d s\n", Pump_Switch_Condtion[8]);
						}
	#endif
					
					}
					else	  //only fc pump, no need to convert pump i into pf. jump to phase7
					{
						pump_fatige_switch_phase = 7;
					}
					f_switch_timer_tick_count_rec = timer_tick_count; 
				}
				
			} 
	   		break;

	   case 2:
			//FCi-->PFi		
			if((timer_tick_count - f_switch_timer_tick_count_rec) * 10 >= (Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14]))
			{
				//pf mode ON	启动当前泵到工频
#ifdef DBG_FATIGESWITCHMANAGER
				printf("启动当前泵%d到工频\n", old_pump_index);
#endif
#ifdef DBG_FATIGESWITCHMANAGER
				show_on_off();
#endif
				if(pump_usablilty_tbl[old_pump_index] == TRUE) 
				{
					if(find_target_pump(old_pump_index) >= 10)
					{
						switch_to_pf(old_pump_index);
						amount_of_running_pumps++;//20200221
					}
					else	//no pump found at all ,exit
					{
					   pump_fatige_switch_phase = 0;
					   fast_mode = FALSE;

					   break;
					}
				}
#ifdef DBG_FATIGESWITCHMANAGER
				else
				{
					printf("泵%d故障，操作取消\n", focused_pump_index);	
				}
#endif
#ifdef DBG_FATIGESWITCHMANAGER
				show_on_off();
				printf("----------------amount_of_running_pumps = %d\n",amount_of_running_pumps);
#endif

			    pump_fatige_switch_phase = 4;
				f_switch_timer_tick_count_rec = timer_tick_count; 
			} 

	   		break;

	   case 3:
	   case 4:
			printf("case %d\n", pump_fatige_switch_phase);				
			if((timer_tick_count - f_switch_timer_tick_count_rec) * 10 >= ((pump_switch_phase == 3)? Pump_Switch_Condtion[8] * 1000 : delta))
			{

				ind = find_target_pump(old_pump_index);

				if(ind < 0)
				{
					printf("no pf or free pump available at all, retrieve\n");
					AllPumpsDead = TRUE;
					escape_pump_adding_loop();

#ifdef DBG_FATIGESWITCHMANAGER
					printf("fc error occurs in phase3, AllPumpsDead, Exit \n");
#endif

				}
				else
				{
					if(ind >= 10) 				 //not full, find a new backup
					{
						printf("not full, find a new backup: pump%d\n", ind - 9);
						focused_pump_index = ind - 10;
					}
					else  //full, override a pf pump
					{
						printf("full, override a pf: pump%d\n", ind + 1);
					    focused_pump_index = ind;
						pause(focused_pump_index);
						
	
					}
				
#ifdef DBG_FATIGESWITCHMANAGER
					printf("启动新泵到变频  \n");				
#endif

					//switch fc to the new pump
					switch_to_fc(focused_pump_index);
					amount_of_running_pumps++;
					new_fc_pump_is_on = TRUE;
					Set_FC_StartValue();	
					show_frequency();		

#ifdef DBG_FATIGESWITCHMANAGER
					show_on_off();
					printf("----------------amount_of_running_pumps = %d\n",amount_of_running_pumps);
#endif


				    pump_fatige_switch_phase += 2;
					f_switch_timer_tick_count_rec = timer_tick_count; 
				}
			} 
	   		break;

	   case 5:
	 
			printf("case 5\n");				
			if((timer_tick_count - f_switch_timer_tick_count_rec) * 10 >= delta)
			{
				//pf mode ON	启动当前泵到工频
#ifdef DBG_FATIGESWITCHMANAGER
				printf("启动当前泵%d到工频 BLUE1 ON\n", old_pump_index + 1);
#endif
#ifdef DBG_FATIGESWITCHMANAGER
			    show_on_off();
#endif
				if(pump_usablilty_tbl[old_pump_index] == TRUE) 
				{
					if((find_target_pump(old_pump_index) >= 10) || (new_fc_pump_is_on == TRUE))//20200304
					{											  //new_fc_pump_is_on == TRUE: Path A: 1->>3->>5->>6, 
					                                              //new fc pump is found already, it is safe to switch on old pump
																  //in pf mode now. Otherwise, it is path B: 1->>2->>4->>6
						switch_to_pf(old_pump_index);
						amount_of_running_pumps++;//20200221
					}
					else	//no pump found at all ,exit
					{

					   printf("new fc pump not found! exit...");
					   pump_fatige_switch_phase = 0;
					   fast_mode = FALSE;
					   break;
					}
				}
	     		else
				{
#ifdef DBG_FATIGESWITCHMANAGER
					printf("当前泵突发故障，操作取消\n");
#endif
				}	
#ifdef DBG_FATIGESWITCHMANAGER
				show_on_off();
				printf("----------------amount_of_running_pumps = %d\n",amount_of_running_pumps);
#endif

			    pump_fatige_switch_phase = 6;

#ifdef DBG_FATIGESWITCHMANAGER
				printf("变频转工频延时: %d ms\n", Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14]);
#endif
				f_switch_timer_tick_count_rec = timer_tick_count; 
			} 
	   		break;
	   case 6:
	   	
			printf("case 6\n");			
			if((timer_tick_count - f_switch_timer_tick_count_rec) * 10 >= (Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14]))
			{
#ifdef DBG_FATIGESWITCHMANAGER
				printf("FC ON, GREEN1 ON\n");			
#endif
			   	if(pump_usablilty_tbl[focused_pump_index] == TRUE)
				{
					FC_SWITCH = ON;
				}
#ifdef DBG_FATIGESWITCHMANAGER
				printf("----------------amount_of_running_pumps = %d\n",amount_of_running_pumps);
#endif
		

			    pump_fatige_switch_phase = 0;
				fast_mode = FALSE;
				adding_pump_executing = FALSE;

				freeze_pf_pump(0, OFF);

				RESET_FATIGE_SWITCHING_TIMER		

#ifdef TRAFFIC_LIGHT
    			traffic_light(DIM);
#endif

#ifdef DBG_EVENT_AS_ERROR
				sprintf(buff,"倒泵完成-变频泵:%d, 工频泵%d"	, focused_pump_index + 1, who_is_pf());
				EventRecorder((uint8*)buff);
#endif

#ifdef DBG_PUMP_MANAGER
				debugger("AddPumpSwitchManager: Exit", ON);
				OSD_DbgInfo();
#endif
			} 
	   		break;

	   case 7:
			printf("case %d\n", pump_switch_phase);				
			if((timer_tick_count - f_switch_timer_tick_count_rec) * 10 >= Pump_Switch_Condtion[8] * 1000)
			{
				ind = find_target_pump(old_pump_index);

				if(ind < 0)
				{
					printf("no pf or free pump available at all, retrieve\n");
					AllPumpsDead = TRUE;
					escape_pump_adding_loop();

#ifdef DBG_FATIGESWITCHMANAGER
					printf("fc error occurs in phase3, AllPumpsDead, Exit \n");
#endif

				}
				else
				{
					if(ind >= 10) 				 //not full, find a new backup
					{
						printf("not full, find a new backup: pump%d\n", ind - 9);
						focused_pump_index = ind - 10;
					}
					else  //full, override a pf pump
					{
						printf("full, override a pf: pump%d\n", ind + 1);
					    focused_pump_index = ind;
						pause(focused_pump_index);	
					}
				
#ifdef DBG_FATIGESWITCHMANAGER
						printf("启动新泵到变频  \n");
#endif

					//switch fc to the new pump
					switch_to_fc(focused_pump_index);
					amount_of_running_pumps++;//20200221
					new_fc_pump_is_on = TRUE;
					Set_FC_StartValue();	
					show_frequency();		

#ifdef DBG_FATIGESWITCHMANAGER
					show_on_off();
					printf("----------------amount_of_running_pumps = %d\n",amount_of_running_pumps);
#endif
				    pump_fatige_switch_phase = 6;
					f_switch_timer_tick_count_rec = timer_tick_count; 
				}
			} 

	   		break;


		default:
			break;
	 }
#ifdef DBG_FATIGESWITCHMANAGER
    printf("\n\n\n Exit: void FatigeSwitchManager(void)...............\n\n");
#endif   

}	

 /**************************************************************************
  * Ideally This is the user-specified Max_Pump_Nbr.
  *	In case some errors make one or more pumps 
  * unusable, the max number must be no more than number of the workable
  * pumps, ie. min(Spec. max, Usable max)
  **************************************************************************/
uint8 get_max_nbr(void)
{


   uint8 i, nbr = 0;
//#define DBG_GET_MAX_NBR          //Enabled for debugging only
//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_GET_MAX_NBR 

    printf("\n\n\n Entrance: get_max_nbr(void)...............\n\n");
#endif  
//--------------------------------------------------------------------------------------------------
   for(i = 0; i < 5; i++)
   {
	   if(pump_usablilty_tbl[i]	== TRUE) nbr++;
   }
   if(nbr < Max_Pump_Nbr)
   { 
#ifdef DBG_GET_MAX_NBR 
		printf("nbr = %d\n",nbr);
#endif  	
		return nbr;
   }
#ifdef DBG_GET_MAX_NBR 
		printf("Max_Pump_Nbr = %d\n",Max_Pump_Nbr);
#endif 
   return Max_Pump_Nbr;

}

/***********************************************************
  Mode 1: the full fc bucket converts itself into a pf bucket and 
          find the next empty bucket to receive the new fc energy
		  (work in fc mode) 
  Mode 2: find the next empty bucket and pour the energy in the full
          fc bucket	into it(run it in pf mode)

***********************************************************/
void  load_new_bucket()
{
#ifdef DBG_FUNC_INFO
	 func_dbg_info = OFF;
#endif
#ifdef DBG_UNIT_TEST	
	 if(func_dbg_info) printf("\n\n\n	Entrance: 【load_new_bucket()】................\n\n");
#endif  			 
	//load the pressure-block
   	if ((PumpGroupMode == 1) || (AvfMode == TRUE))   //moving fc: set the current pump to pf(reserve the pressure block)
	{

	   if(amount_of_running_pumps < get_max_nbr())//Max_Pump_Nbr)//Not full yet
	   {
	        printf("load_new_bucket()\n");

			old_pump_index = focused_pump_index;

			// get new pump 
			if(amount_of_running_pumps == 1)//currently this is the only pf pump
			{
				index_pf_queue_head = old_pump_index;
			//dbg only
#ifdef DBG_PUMP_MANAGER
				show_on_off();
#endif
		    }		   

#ifdef DBG_FUNC_INFO
			if(func_dbg_info)
			{
				printf("Frequency is frozen,K1/ GREEN1 OFF\n");
			}
#endif
			//Frequency is frozen
			FC_SWITCH = OFF;

			pump_switch_phase = 1;

#ifdef DBG_STATE_MACHINE
		    SetControlBackColor(SCREEN_MAIN_1, 62, 0x07e0);
#endif

			fast_mode = TRUE;
#ifdef DBG_FUNC_INFO
			if(func_dbg_info)
			{
				printf("变频转工频延时: %d ms\n", Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14]);
			}
#endif
			timer_tick_count_rec = timer_tick_count; 

		}
		else   //All pumps are already running now
		{
			//convert the currently running fc pump into pf mode
			control_pump(focused_pump_index, MOD_PF, ON);

			// update the displaying controls
			update_pump_icon(focused_pump_index, MOD_PF);

			pump_running_mode_tbl[focused_pump_index] = MOD_PF;

			AllPumpsOn = true;

		}

		
	}	
	else if(PumpGroupMode == 2)//  fixed-FC :set the next pump to pf (reserve the pressure block)
	{	
	   /*Here are 2 threads running:  thread A for pump1, implemented through a state machine
	     AddPumpSwitchManagerMode2(), where pump1 is running along several states devided by 
		  a time gap. fc off - pump1 off - pump1 on, and the fc will be switched on at the exit of the machine 
		  thread B turns on a new pump in pf mode and starts A if only A is not running, 
		  which means B can not interrupt A.	   
	   */	
	   if(amount_of_running_pumps < get_max_nbr())//if(focused_pump_index < 3)
	   {
			//Thread B for new pump: Switch off FC and turn on the new pump in pf
			FC_SWITCH = OFF;

			focused_pump_index = find_target_pump(focused_pump_index);	 //20210623
			if (focused_pump_index < 10) return;
			
		
		   	focused_pump_index = focused_pump_index - 10;
			switch_to_pf(focused_pump_index);
	 		amount_of_running_pumps++;

#ifdef DBG_PUMP_MANAGER
			printf("Pump-adding confirmed, FC_SWITCH = OFF, switch_to_new pf\n");
			show_on_off();
#endif

			//Thread A for pump1 operation if ready
			if(pump_switch_phase_mod2 == 0)
			{
				printf("Enter thread A for pump1 operation\n");
				pump_switch_phase_mod2 = 1;

#ifdef DBG_STATE_MACHINE
				SetControlBackColor(SCREEN_MAIN_1, 62, 0x07e0);	   //1
#endif
				fast_mode = TRUE;
#ifdef DBG_FUNC_INFO
				if(func_dbg_info)
				{
					printf("变频转工频延时: %d ms\n", Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14]);
				}
#endif
				timer_tick_count_rec_mod2 = timer_tick_count; 
			}
			else
			{
				printf("Thread A is running, do not interrupt it\n");
			}
		}
		else
		{
			AllPumpsOn = true;
		}
	}

#ifdef DBG_UNIT_TEST	
	 if(func_dbg_info) printf("\n\n\n	Exit: 【load_new_bucket()】................\n\n");
#endif
}



void empty_fc_bucket()
{
	pid.curr_output = Pump_Switch_Condtion[4] * 300;
}

/***********************************************************
  The fc bucket is full and find a new pump to either receive fc energy
  or pour the fc energy to it.

***********************************************************/
 void use_new_bucket()
 {

 	//---------------------at the moment fc-bucket is full

	 //---------------------switch to new fc bucket if needed
	if(amount_of_running_pumps < get_max_nbr())//Max_Pump_Nbr)//2020303
	{
		printf(" amount_of_running_pumps < Max_Pump_Nbr;\n");
		if(find_target_pump(focused_pump_index) >= 10)
		{
			printf("free pump exists\n");
			if(PumpGroupMode == 2)
			{
				printf("PumpGroupMode == 2, do not reset pid-output here\n");
			}
			else
			{
		 		empty_fc_bucket();
			}
		 	load_new_bucket();
		}
		else
		{
#ifdef DBG_PUMP_MANAGER
			show_on_off();
#endif
			printf("amount_of_running_pumps < Max_Pump_Nbr  but NO free pump exists!!!\n");
		}
	}
}

 /*Set pid target pressure and display it on main screen*/
 void PidSetTargetPressure(float target_pressure)
 {
 	  char buff[5];
	  Tartget_Pressure = target_pressure;

	  pid.set = target_pressure * 100.0;

#ifdef DBG_PID
	  printf("target_pressure = %f\n",target_pressure);
#endif

	  sprintf(buff, "%.2f", target_pressure);
	  SetTextValue(SCREEN_MAIN_1, TXT_TARGET_PRESSURE, (uchar*)buff);
}

#define STEP_LEN_CORRECTION_FACTOR	       1.0      //updated from 2.0	 20210415
float get_step_up_length()
{
	if(Tartget_Pressure < OutletRealTimePressure)	 //decrease
	{
   	 	return -300.0 * (MaxFreqOut - Pump_Switch_Condtion[4]) / (Pump_Switch_Condtion[2] * 256 + Pump_Switch_Condtion[3]) * PID_Setting[1] / 10.0 * STEP_LEN_CORRECTION_FACTOR;
	}
   	else 											 //increase
	{
		//                 --frequency gap--                                     --duration--					   --execution period--
		return 300.0 * (MaxFreqOut - Pump_Switch_Condtion[4]) / (Pump_Switch_Condtion[0] * 256 + Pump_Switch_Condtion[1]) * PID_Setting[1] / 10.0 * STEP_LEN_CORRECTION_FACTOR;
	}

}
#ifdef USE_CURVES
uint8 GetFrequencyDataForCurve(void)
{
 	return (int)(pid.curr_output/300.0 * 2);
}
#endif



extern uint8 Sleep_Setting[];
extern uint8 PumpCancelFreq;
uint8 get_pump_cancel_freq(void)		//20210710
{
	// if (status_for_cancel == CANCEL_FREQ_UNACTIIVATED) return 0;
	 if (amount_of_running_pumps <= 1) return Sleep_Setting[2];
	 return PumpCancelFreq;
}



#define PID_ALIVE   pump_switch_phase_mod2 == 0
 /*************************************************************************
 	This function calculates the new output value and put it to frequency converter.
	2 types of events will occur: FC_EVENT_FULL/FC_EVENT_EMPTY 
	in case the output overflows or underflow respectively.

	The details include:
	--select output according to AlgoType: pid or linear
	--the controls(progress bar and current frequency) are refreshed.
	--middle variables like  	pid.En_2, pid.En_1, pid.prev_output are updated.

	Execution period: 1s
 
 **************************************************************************/   	
void pid_calc_core(void)  
{

	float dkp, dki, dkd, step_len;
	float tp, ti, td;
	char buff[5];
	uint8 cancel_freq;
	





//#define DBG_PID_CALC_CORE
//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_PID_CALC_CORE
        printf("\n\n\n Entrance: void pid_calc_core(void) ...............\n\n");
#endif  
//--------------------------------------------------------------------------------------------------

	if(PID_ALIVE)
	{
//-----------------PID CAL------------------------------------	
		pid.curr = OutletRealTimePressure * 100.0;		 	 

#ifdef DBG_PID_CALC_CORE
		printf("pid.curr out = %.0f\n",pid.curr);
		printf("pid.set = %.0f\n",pid.set);			
#endif

		pid.En = pid.set - pid.curr;  //本次误差   (100MPa)
		// ------------------------------calculate pid output--------------------------
		if(pid.En_2 == 9999.0)
		{
			dkp = 0;   //本次偏差与上次偏差之差
			dki = 0;
			dkd = 0;
			printf("starup pid \n");
		}
		else
		{		
			dkp = pid.En - pid.En_1;   //本次偏差与上次偏差之差
			dki = pid.En;
			dkd = pid.En - 2 * pid.En_1 + pid.En_2;
		}
#ifdef DBG_PID_CALC_CORE
		printf("										pid.set      pid.curr    pid.En       pid.En1      pid.En2       dkp      dki      dkd\n");
		printf("										%0.2f        %0.2f       %0.2f        %0.2f        %0.2f         %0.2f    %0.2f    %0.2f\n\n\n", \
														pid.set, pid.curr, pid.En, pid.En_1, pid.En_2, dkp, dki, dkd);
#endif
		
		tp = Pid_Kp * dkp;         //比例
		
		ti = Pid_Ki * dki;      //积分
		
		td = Pid_Kd * dkd;        //微分
		
		
		switch(pid.choose_model)
		{
			case MODEL_P:     pid.Dout= tp;                  //  printf("使用P运算\r\n") ;
			 					break;
			
			case MODEL_PI:    pid.Dout= tp + ti;              //  printf("使用PI运算\r\n") ;
			 					break;
			     
			case MODEL_PID:   pid.Dout= tp + ti + td;       // printf("使用PID运算\r\n") ;
								 break;
		} 

		//-------------------------------select output according to AlgoType--------------------------
		if(AlgoType == ALGO_LINEAR)
		{
			step_len = get_step_up_length();

			pid.curr_output += step_len;
		}
		else
		{  
			pid.curr_output += pid.Dout;  //current output
 			if ((Tartget_Pressure < OutletRealTimePressure) && (pid.Dout < 0))  pid.curr_output += (SpeedUpFactor - 1.0) * pid.Dout;   //Speed up of freq. decreasing   //20210712
		}
		//-------------------------------refresh algo：prepare for next cycle ----------------------------------------------
		select_algo();	   //pid out changed, there might be a change of algo as well.

		/* Range check: freq. must be in range of min~max */
		if(pid.curr_output >= pid.max_output) pid.curr_output = pid.max_output;          //overflow, fc-pressure bucket is full
		//else if(pid.curr_output < 0) pid.curr_output = 0;//20210710 20220623
 		else if(pid.curr_output < Pump_Switch_Condtion[4] * 300) \
 		        pid.curr_output = Pump_Switch_Condtion[4] * 300;//20200327 20220623

	}	

#ifdef DBG_PID_CALC_CORE
		printf("PID OUT:\t%.0f\r\n\n\n", pid.curr_output);
#endif	

	if(pid.curr_output >= PumpCancelFreq * 300)  	 //20210710	  up-across  PumpCancelFreq
	{
		 if (status_for_cancel == CANCEL_FREQ_UNACTIIVATED) status_for_cancel = CANCEL_FREQ_ACTIIVATED;
	} 
	cancel_freq = get_pump_cancel_freq();
	//----------------------------FC_EVENT_FULL------------------------------------------------
	if((pid.curr_output >= pid.max_output) && (pid.curr < pid.set))           //overflow
	{
		{
			Control_FC();

#ifdef DBG_PID_CALC_CORE		
			if(adding_pump_confirming == FALSE) printf("----------------------------1st FC OVERFLOW!!!\n");		
#endif
	
			if(Pump_Status == MAIN_PUMP_ON) FC_Event = FC_EVENT_FULL;				   //create an event to start the timer for add-pump operation.
		}
	}
	
	/*	管网压力超过设定压力值后，变频泵将降速运行，频率降到启停频率后还是超过设定值，
		则第1台运行的工频泵停止运行，其余的工频泵也依此方式停止，最后停止的是变频泵。*/
	//---------------------------FC_EVENT_EMPTY------------------------------------------------
//	else if((pid.curr_output <= PumpCancelFreq * 300) && (pid.curr > pid.set))	  //

	/* Undeflow is triggered each time freq. out is euql to or less than cancel-threshold*/
    else if((pid.curr_output <= (cancel_freq * 300)) && (pid.curr > pid.set))	//20210710
	{


     	//20220319 let frequency floating instead of stick to cancel_freq.
	   // if (status_for_cancel == CANCEL_FREQ_ACTIIVATED) pid.curr_output = cancel_freq * 300 - FC_FLOOR_DELTA;		   //20210710
	    if (pid.curr_output < 0) pid.curr_output = 0;
		
		Control_FC();

#ifdef DBG_PID_CALC_CORE		
		printf("FC UNDERFLOW!!!\n");
#endif

		/* Not In Canceling yet, Issue a new EMPTY event for pump canceling */
		if (canceling_pump == FALSE) //20220319
		{
		    printf("\nNot In Canceling yet, Issue a new EMPTY event for pump canceling\n");
		    FC_Event = FC_EVENT_EMPTY;
		} 
		            
	}
	else
	{	  
		Control_FC();
	}
	//-------------------------------Display:frequency/progress bar------------------------------------------------
	if ((PumpGroupMode == 1) || (AvfMode == TRUE))	//floating fc
	{
	    //frequency
		show_frequency();
	/*	if(pid.curr_output/300.0 < 0)  sprintf(buff, "0.0");
		else sprintf(buff, "%.1f", pid.curr_output/300.0);
		SetTextValue(SCREEN_MAIN_1, TXT_CURRENT_FREQUENCY, (uchar*)buff);	  */
			 
	}
	else if(PumpGroupMode == 2)	  //fixed fc
	{

	   	if(PID_ALIVE)
	   	{
			//frequency
			show_frequency();
		}
	}
	if(PID_ALIVE)
	{
		//-----------------------------------medium variables----------------------------
		pid.En_2 = pid.En_1;
		pid.En_1 = pid.En;
		pid.prev_output = pid.curr_output;
	}

//---------------------------------------------------------------------------------------------
#ifdef PID_CALC_CORE
    printf("\n\n\n Exit: void pid_calc_core(void) ...............\n\n");
#endif  

}	 

//Every 100ms
void pid_calc(void) 
{	 //PID_Setting[1]
	static uint8 pid_counter = 1;
	if((PID_CALC_GREEN_LIGHT) && (RUNNING_GREEN_LIGHT) && (PID_GREEN_LIGHT)) 
	{	   
		pid_counter--;
		if(pid_counter == 0)
		{
			pid_counter = PID_Setting[1];	  //PID sampling period: T

			pid_calc_core();
		}
	}
}

/******************************************************************
  Control a pump: let it work in fc or pf mode or turn it off.
  Para: i--------pump number, 0-3(pump1-pump4)
        mode-----FC/PF
		status---ON/OFF
  Caller(s): load_new_bucket()/start_new_pump()/cancel_pump()/ stop_a_pump()/switch_to_new_pump()/
             Start_Stopper()/recover_from_standby()/start_mod_1_2()
*******************************************************************/
void control_pump(uint8 i, uint8 mode, uint8 status)	   //???
{
	switch(i)
	{
	      case 0:
				if(mode == MOD_PF)
				{
				 	PUMP1_FC = OFF;
					PUMP1_PF = status;
				}
				else
				{
					PUMP1_FC = status;
					PUMP1_PF = OFF;
				}
			  
	          break;

	      case  1:
				if(mode == MOD_PF)
				{
					PUMP2_FC = OFF;
					PUMP2_PF = status;
				}
				else
				{
				    PUMP2_FC = status;
					PUMP2_PF = OFF;
				}
	          break;
	      case  2:
				if(mode == MOD_PF)
				{
					PUMP3_FC = OFF;
					PUMP3_PF = status;
				}
				else
				{
					PUMP3_FC = status;
					PUMP3_PF = OFF;
				}
	          break;
	      case  3:
				if(mode == MOD_PF)
				{
					PUMP4_FC = OFF;
					PUMP4_PF = status;
				}
				else
				{
					PUMP4_FC = status;
					PUMP4_PF = OFF;
				}
	          break;
	      case  4:
				if(mode == MOD_PF)
				{
					PUMP5_FC = OFF;
					PUMP5_PF = status;
				}
				else
				{
					PUMP5_FC = status;
					PUMP5_PF = OFF;
				}
	          break;
	      case  5:
				if(mode == MOD_PF)
				{
					AUX_PUMP_FC = OFF;
				}
				else
				{
					AUX_PUMP_FC = status;
				}

	      default:
	          break;
	}

}
/**********************************************************************************************
 * Name：Manual_Control_Pump
 * Brief： Manually control the pumps
 *         Limitaions: 1. Each pump is working in either FC or PF mode.
 *	                   2. There is always at most 1 pump running in FC mode.(FC source is connected to it)
 * Para:   i: pump index(0~5)
 * 		   mode:PF/FC       
 *         state:ON/OFF
 * Caller(s):  main.c/NotifyButton-SCREEN_MANUAL_CTR_2
 **********************************************************************************************/
void Manual_Control_Pump(uint8 i, uint8 mode, uint8 status)
{

	switch(i)
	{
	      case 0:
				if(mode == MOD_PF)	   //PF
				{
					PUMP1_PF = status;
					if(status == ON) PUMP1_FC = OFF;
				}
				else		 //FC
				{
					PUMP1_FC = status;
					if(status == ON) PUMP1_PF = OFF;
				}
			  
	          break;

	      case  1:
				if(mode == MOD_PF)	   //PF
				{
					PUMP2_PF = status;
					if(status == ON) PUMP2_FC = OFF;
				}
				else		 //FC
				{
					PUMP2_FC = status;
					if(status == ON) PUMP2_PF = OFF;
				}
			  
	          break;
	      case  2:
				if(mode == MOD_PF)	   //PF
				{
					PUMP3_PF = status;
					if(status == ON) PUMP3_FC = OFF;
				}
				else		 //FC
				{
					PUMP3_FC = status;
					if(status == ON) PUMP3_PF = OFF;
				}
			  
	          break;
	      case  3:
				if(mode == MOD_PF)	   //PF
				{
					PUMP4_PF = status;
					if(status == ON) PUMP4_FC = OFF;
				}
				else		 //FC
				{
					PUMP4_FC = status;
					if(status == ON) PUMP4_PF = OFF;
				}
			  
	          break;
	      case  4:
				if(mode == MOD_PF)	   //PF
				{
					PUMP5_PF = status;
					if(status == ON) PUMP5_FC = OFF;
				}
				else		 //FC
				{
					PUMP5_FC = status;
					if(status == ON) PUMP5_PF = OFF;
				}
			  
	          break;
	      case  5:
		  		if(mode == MOD_PF)	   //PF
				{
					if(status == ON) AUX_PUMP_FC = OFF;
				}
				else		 //FC
				{
					AUX_PUMP_FC = status;
				//	if(status == ON) PUMP3_FC = OFF;
				}

			  	break;

	      default:
	          break;
	}
}

/**********************************************************************************************
 * Name：start_new_pump
 * Brief： For Mode2 only--Find the next idle and usable pump and start it with PF mode
 * Caller(s):  pf_manager
 **********************************************************************************************/
void start_new_pump()
{
	 uint8 i = 0;
	 while(i < 4)
	 {
	 	if(pump_usablilty_tbl[i] == 1)		   //find usable pump only
		{
			if(pump_on_off_tbl[i] == OFF)	//idle pump
			{
				control_pump(i, MOD_PF, ON);
				pump_on_off_tbl[i] = ON;		  //new status :on

				update_pump_icon(i, MOD_PF);
				break;
			} 
		}
		i++;
	 }
}


/**********************************************************************************************
 * Name：stop_a_pump()
 * Brief： stop the pump with index number i
 * Caller(s): stop_all_pumps_immediately() 
 **********************************************************************************************/
void stop_a_pump(uint8 i)
{

#ifdef DBG_PUMP_MANAGER
	printf("stop pump %d\n", i);
#endif
	control_pump(i, MOD_PF, OFF);	   // stop the pump

	pump_on_off_tbl[i] = OFF;		   // update on/off table

	if (AvfMode)	//AVF	 
	{
		UpdatePumpFreq(0, i);	   //freq<--0
		SetTextValue(SCREEN_MAIN_1, PumpIcons[i].icon_freq, "--");	   //freq: --
		SetControlVisiable(SCREEN_MAIN_1,PumpIcons[i].icon_pump_fc, INVISIBLE);
		SetControlVisiable(SCREEN_MAIN_1,PumpIcons[i].icon_pump_pf, INVISIBLE);	//stop icon
	}
	else
	{
		update_pump_icon(i, MOD_NUL);	   // set the relevent pump icon to STOP mode 
	}
	CLR_MODE_TXT(i)
	STOP_WATERFLOW(i)
 
}

/**********************************************************************************************
 * Name：stop_all_pumps_immediately()
 * Brief： Scan from left to right to find the next running pump and stop it
 * Caller(s):  f_manager
 **********************************************************************************************/
void stop_all_pumps_immediately()
{
	uint8 i;
#ifdef DBG_PUMP_MANAGER

   	printf("stop_all_pumps_immediately() \n");
   	printf("pump_enable_tbl: ");
	display(pump_enable_tbl, 4);
	printf("\n pump_usablilty_tbl: ");
	display(pump_usablilty_tbl, 4);
	printf("\n pump_on_off_tbl: ");
	display(pump_on_off_tbl, 4);

	Set_FC_Zero();

#endif
	 for(i = 0; i < 6; i++)
	 {
	 	if(pump_usablilty_tbl[i] == 1)
		{
			if(pump_on_off_tbl[i] == ON)
			{
				stop_a_pump(i);
			} 
		}
	 }
	 RefreshButtons();
}

/**********************************************************************************************
 * Name：find_the_pump_to_be_canceled()
 * Brief： find_the_pump_to_be_canceled
 *         If there is only an fc pump running, just return it. 
 *         otherwise return  index_pf_queue_head
 *
 * Caller(s):  cancel_a_pump() 
 **********************************************************************************************/
uint8 find_the_pump_to_be_canceled()
{
	printf("find_the_pump_to_be_canceled():amount_of_running_pumps = %d\n",amount_of_running_pumps);
#ifdef DBG_PUMP_MANAGER
			show_on_off();
#endif

	if(amount_of_running_pumps == 1)  //FC 
	{		
	   if ((PumpGroupMode == 1) || (AvfMode == TRUE))
	   {
	   //to be changed later
		   return 9;//focused_pump_index;				  //there is only an fc pump
	   }
	   else if(PumpGroupMode == 2)
	   {
	   	   if(pump_usablilty_tbl[0] == FALSE)	 return find_next_pf_head(NBR_OF_PUMPS - 1);	//20210622
	   	   return 0;
	   } 
	}
	else if(amount_of_running_pumps > 1)			 //pf pumps exist
	{
	    return find_next_pf_head(NBR_OF_PUMPS - 1);	   //20200301 20210622
	}
	return 0; //logically impossible
}

#define NBR_OF_USABLE_PUMPS (pump_usablilty_tbl[0] + pump_usablilty_tbl[1] + pump_usablilty_tbl[2] +\
					                                         pump_usablilty_tbl[3] + pump_usablilty_tbl[4])
/*
    Find the pump to be replaced by a spare one.
	Scan from left to right for a running pump other than the focused one
	Start point: the pump next to the focused one
*/
uint8 find_the_pump_to_be_canceled_avf_mode()
{
	uint8 i;
	printf("find_the_pump_to_be_canceled_avf_mode():amount_of_running_pumps = %d\n",amount_of_running_pumps);
#ifdef DBG_PUMP_MANAGER
			show_on_off();
#endif
	if (amount_of_running_pumps >= NBR_OF_USABLE_PUMPS) return 255;
   	// if (Max_Pump_Nbr == 1)  return focused_pump_index;	  //20210625

	for(i = (focused_pump_index + 1) % 5; i != focused_pump_index; i = (i + 1) % 5)
    {
    	if (pump_on_off_tbl[i] == ON)  return i;
    }	
	return 255; //not found
}

/*
    Find the new_focused_pump.
	Scan from left to right for a running pump other than the focused one
	Start point: the pump next to the focused one
*/
uint8 find_the_new_focused_pump_avf_mode()
{
	uint8 i;

#ifdef DBG_PUMP_MANAGER
			show_on_off();
#endif
//	if (amount_of_running_pumps >= NBR_OF_USABLE_PUMPS) return 255;		 //error!	   removed 20210623
	for(i = (focused_pump_index + 1) % 5; i != focused_pump_index; i = (i + 1) % 5)
    {
    	if (pump_on_off_tbl[i] == ON)  return i;
    }	
	return 255; //not found
}
/**********************************************************************************************
 * Name：update_index_pf_queue_head()
 * Brief： update the index of pf_queue_head when a running pf-pump is stopped
 *  	   scan the pf queue, starting from the next of the current index_pf_queue_head
 *         condition: 	(pump_usablilty_tbl[i] == 1)&&
 *			            (pump_on_off_tbl[i] == ON) &&
 *						(pump_running_mode_tbl[i] == MOD_PF))	 
 *
 * Caller(s):  cancel_a_pump() 
 **********************************************************************************************/
void update_index_pf_queue_head()
{
	uint8 i;

	i = (index_pf_queue_head + 1) % NBR_OF_PUMPS;
	while(1)
	{
		if(pump_usablilty_tbl[i] == 1)
		{
			if((pump_on_off_tbl[i] == ON) &&(pump_running_mode_tbl[i] == MOD_PF))	 
			{
			    index_pf_queue_head = i;
			//dbg only
#ifdef DBG_PUMP_MANAGER
//			    show_on_off();
#endif
				break;
			}
		}
		i = (i + 1) % NBR_OF_PUMPS;	
	}
}
/**********************************************************************************************
 * Name：cancel_a_pump()
 * Brief： find the target pump and cancel it.
 *		  --- In fixed-fc mode, in case pump1 have broken down, while the target pump is the only running one, 
 *            fall asleep immedietely
 *          
 *  
 * Para：    	
 * Return ： 
 * 
 * Caller(s): fc_manager  
 **********************************************************************************************/

void cancel_a_pump()
{
	uint8 i;

//#define DBG_CANCEL_A_PUMP          //Enabled for debugging only
//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_CANCEL_A_PUMP

    printf("\n\n\n Entrance: void cancel_a_pump()...............\n\n");
#endif  
//--------------------------------------------------------------------------------------------------
 

    // avf mode 
	if (AvfMode == TRUE)
	{
#ifdef DBG_CANCEL_A_PUMP
		printf("----------------is avf_mode\n");
#endif
		i = focused_pump_index;

		//pump off
		control_pump(i, MOD_PF, OFF);

		//Update variables
		pump_on_off_tbl[i] = OFF;
		pump_running_mode_tbl[i] = MOD_PF;

		//Frequency: 0
		UpdatePumpFreq(0, i);

		//Update UI controls
		SetTextValue(SCREEN_MAIN_1, PumpIcons[i].icon_freq, "--");
		SetControlVisiable(SCREEN_MAIN_1,PumpIcons[i].icon_pump_fc, INVISIBLE);
		SetControlVisiable(SCREEN_MAIN_1,PumpIcons[i].icon_pump_pf, INVISIBLE);

		CLR_MODE_TXT(i)
		STOP_WATERFLOW(i)

		i = find_the_new_focused_pump_avf_mode();
		printf("----------------find_the_new_focused_pump_avf_mode(); i = %d\n",i);
		if (i < 5) 
		{
			focused_pump_index = i;
			pid.curr_output = pid.max_output;

#ifdef DBG_CANCEL_A_PUMP			
			printf("----------------pid.curr_output = %d\n",pid.curr_output);
#endif
		}

		//decrease amount_of_running_pumps
		if(amount_of_running_pumps > 0)
		{
			amount_of_running_pumps--;
#ifdef DBG_CANCEL_A_PUMP
			printf("----------------amount_of_running_pumps = %d\n",amount_of_running_pumps);
#endif
		}
		else
		{
#ifdef DBG_CANCEL_A_PUMP
		    printf("all pumps are off now! Bedtime...\n");
#endif
		}


		if(AllPumpsOn == true)	AllPumpsOn = false;

		return;
	}

   // not avf mode 
	i = find_the_pump_to_be_canceled();


#ifdef DBG_CANCEL_A_PUMP
	printf("=====================================================_pump %d_to_be_canceled\n",i);
#endif

	control_pump(i, MOD_PF, OFF);
	pump_on_off_tbl[i] = OFF;

	SetControlVisiable(SCREEN_MAIN_1,PumpIcons[i].icon_pump_fc, INVISIBLE);	   //20210622
	SetControlVisiable(SCREEN_MAIN_1,PumpIcons[i].icon_pump_pf, INVISIBLE);

	CLR_MODE_TXT(i)
	STOP_WATERFLOW(i)

	//decrease amount_of_running_pumps
	if(amount_of_running_pumps > 0)
	{
		amount_of_running_pumps--;

#ifdef DBG_CANCEL_A_PUMP
		printf("----------------amount_of_running_pumps = %d\n",amount_of_running_pumps);
#endif
	}
	else
	{
#ifdef DBG_CANCEL_A_PUMP
	    printf("all pumps are off now! Bedtime...\n");
#endif
	}

	if(PumpGroupMode == 2)		//fixed fc mode
	{
		focused_pump_index = find_the_pump_to_be_canceled();
		if(focused_pump_index == 9) focused_pump_index = 0;

	}

	if((PumpGroupMode == 2) && (pump_usablilty_tbl[0] == FALSE) && (amount_of_running_pumps == 0))
	{
		focused_pump_index = i;
		ShortCutToSleep();
	}
	else
	{

		//	update_index_pf_queue_head 
		if(amount_of_running_pumps > 1)		   //pf queue is not empty
		{
			update_index_pf_queue_head();	   //get new index_pf_queue_head
		}
		else if(amount_of_running_pumps == 1)	//fc only
		{
			
		}
		else if	(amount_of_running_pumps == 0) //none is left
		{
			if ((PumpGroupMode == 1) || (AvfMode == TRUE))
			{
				pump_on_off_tbl[focused_pump_index] = OFF;			
			}
	
			focused_pump_index = 0;
	
#ifdef DBG_CANCEL_A_PUMP
			printf("index_pf_queue_head = %d\n",index_pf_queue_head);
#endif
		}
	}
	if(AllPumpsOn == true)	AllPumpsOn = false;

#ifdef DBG_CANCEL_A_PUMP
	debugger("cancel_a_pump(): Exit", ON);
#endif

#ifdef DBG_CANCEL_A_PUMP
    printf("\n\n\n Exit: void cancel_a_pump()...............\n\n");
#endif  
 

}

void reload_fc_bucket()
{
	Set_FC_StartValue();
	show_frequency();
}

char get_win_number()
{
	 if(win1_counter > 0) return  WIN1;
	 return WIN2;
}

void reset_window(uint8 window)
{
    if(window == WIN1)
	{
		 win1_counter = Win1_Counter_Setting; 

#ifdef DBG_PUMP_MANAGER
		 printf("reset_window\n");
#endif

	}
	else
	{ 
		 win2_counter = Win2_Counter_Setting; 
	} 	 
}


void switch_to_new_pump(uint8 destination, uint8 source)
{
	//switch the pump
	control_pump(source, MOD_PF, OFF);	 //source off
	control_pump(destination, pump_running_mode_tbl[source], ON);	  //destination on

	//update variables
	//-----pump_usablilty_tbl
	pump_usablilty_tbl[destination] = ON;  
	pump_usablilty_tbl[source] = OFF; 
	//-----running mode
	pump_running_mode_tbl[destination] = pump_running_mode_tbl[source];
	pump_running_mode_tbl[source] = MOD_NUL;
	//-----on/off
	pump_on_off_tbl[destination] = ON;
	pump_on_off_tbl[source] = OFF;
	//-----pf_queue_head/focus_index
	if(pump_running_mode_tbl[destination] == MOD_PF)
	{
 	   if(index_pf_queue_head == source) update_index_pf_queue_head();
	}
	else	  //fc
	{
	   focused_pump_index = destination;
	}

	//update icons
	//----pump icons

	update_pump_icon(source, MOD_NUL);
	update_pump_icon(destination, pump_running_mode_tbl[destination]);

}

int find_spare()
{
	uint8 i, max_fuel = 0;
	int index = -1;
 	for(i = 0; i < 4; i++)
    {
		//       OFF now                     enabled                      error-free    
    	if((pump_on_off_tbl[i] == OFF) && (pump_enable_tbl[i] == ON) && (pump_error_tbl[i] == FALSE))
    	{
    		if(FuelMeter[i] > max_fuel) 
    		{
    			index = i;
    			max_fuel = FuelMeter[i];
			}
		}
    }	
	return index;
}

void use_new_pump(uint8 old_index)
{
	int new_index;
	new_index = find_spare();
	if(new_index >= 0)
	{
		printf("Start:switch_to_new_pump: pump%d\n", new_index);
		switch_to_new_pump(new_index, old_index);
		printf("End:switch_to_new_pump: pump%d\n", new_index);
	}
	else
	{
		printf("No spare pump found!\n");
	}
	
}

#ifdef DUAL_WATER_SOURCE
void WaterSourceTankSwitching(uint8 ON_OFF)
{
	if(WaterSourceMode == TANK_NONNEGTIVE_MODE)
	{
		if(ON_OFF == ON)
		{
			TankPeriodCounter = Valve_Control1[2];		
			TankOnCounter = Valve_Control1[3];
		}
		else
		{
			TankPeriodCounter = 0;		
			TankOnCounter = 0;
		}
	}
}
#endif

float get_gap(float a, float b)
{
	 if(a > b) return a - b;
	 return b - a;
}
#ifdef   USE_NARROW_PID_ZONE
	#define  PID_ZONE_SIZE                       0.2 //updated from 0.5	   20210415
#else
	#define  PID_ZONE_SIZE                       0.5 
#endif


void select_algo()
{

	if(get_gap(Tartget_Pressure, OutletRealTimePressure)/Tartget_Pressure < PID_ZONE_SIZE)	//approach target, use pid
	{
	   AlgoType = ALGO_PID;
	   
	}
	else
	{
	   AlgoType = ALGO_LINEAR;
	}
	TickPidCounter = 1;

#ifdef DBG_FUNC_INFO
	if(func_dbg_info)
	{	
		printf("get_gap(Tartget_Pressure, OutletRealTimePressure)/Tartget_Pressure = %0.1f\n\n",
				get_gap(Tartget_Pressure, OutletRealTimePressure)/Tartget_Pressure);
		printf("Tartget_Pressure = %0.1f, OutletRealTimePressure = %0.1f\n\n", Tartget_Pressure, OutletRealTimePressure);
		printf("AlgoType: %d\n\n", AlgoType);
		printf("AlgoType Name: %s\n\n", algo_names[AlgoType]);	
	}
#endif
}


void clear_tables(void)
{
	uint8 i;
	for(i = 0; i < 4; i++)
	{
	   pump_running_mode_tbl[i] = MOD_PF;
	   pump_on_off_tbl[i] = OFF;
	}

}

void pump_running_initializaition(void)
{
	select_algo();
	pump_startup_done = FALSE;

	adding_pump_confirming = FALSE;
	canceling_pump = FALSE;

	pump_switch_phase = 0; 
	pump_switch_phase_mod2 = 0; 
	pump_fatige_switch_phase = 0;

	win1_counter = 0;
	win2_counter = 0;
	amount_of_running_pumps = 0;

	clear_tables();

	add_now = FALSE;
    cancel_now = FALSE;
	fast_mode = FALSE;

	sleep_chk_timer = 0;
	status_for_cancel = CANCEL_FREQ_UNACTIIVATED;	   //20210710

}


//20210805
void SetFocus(void)
{
	int i;
 	i = FindLeastUsedPump();
		
	if (i >= 0)
	{
		focused_pump_index = i;
		printf("Start the least used pump%d\n", i + 1);
	}	   
	else	    //almost  impossible            
	{
	    focused_pump_index = PumpPointer;
	}
}


 /*********************************************************
 	 Start up the pumps in mode 1 or 2
 **********************************************************/ 
void start_mod_1_2(void)
{
 
//#define DBG_START_MOD_1_2
//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_START_MOD_1_2
    printf("\n\n\n Entrance: void start_mod_1_2(void)...............\n\n");
#endif  
//--------------------------------------------------------------------------------------------------
 

	//initialization
	pump_running_initializaition();

	/******************Load Valve control settings for the WaterSourceMode**********************/
#ifdef DUAL_WATER_SOURCE
	WaterSourceTankSwitching(ON);
#endif

	if ((PumpGroupMode == 1) || (AvfMode == TRUE))	  //moving fc
	{

#ifdef DBG_START_MOD_1_2
		if (AvfMode)  printf("Starting: This is AvfMode!\n");
#endif

	   //to be changed later
	   index_pf_queue_head = 0;
	 
#ifdef DBG_START_MOD_1_2
			show_on_off();
#endif
	  
	   	//focused_pump_index = PumpPointer;
	   //20210805
	   SetFocus();

#ifdef DBG_START_MOD_1_2
		printf("focused_pump_index = %d\n",focused_pump_index);
		show_on_off();
#endif


	   //if this pump is not usable anymore(has an error or disabled by user), find next
	   if(pump_usablilty_tbl[focused_pump_index] == 0)
	   {
		  update_focused_pump_index();
		  printf("The recoded pump is not workable, find another one:focused_pump_index = %d\n",focused_pump_index);
	   }	   
	}
	else if(PumpGroupMode == 2) //fixed fc
	{
	   index_pf_queue_head = 1;
	
	   focused_pump_index = 0;
	
	} 
   	// Start FC and the 1st Pump
	if((OutletRealTimePressure < Tartget_Pressure) || (SleepingEnableMode == SLEEP_MODE_ALWAYS_RUNNING)\
												   || (SleepingEnableMode == SLEEP_MODE_SLEEP_DISABLE))
	{

#ifdef DBG_START_MOD_1_2
		printf("启动当前泵到FC, RED1 ON\n");
#endif
		// Switch foucused pump	to FC mode
		control_pump(focused_pump_index, MOD_FC, ON);

#ifdef DBG_START_MOD_1_2
		printf("启动FC, Frequency: START/STOP\n");
#endif
	    //Start FC at Start/stop frequency
		if (AvfMode) 
		{	
			UpdatePumpFreq(Pump_Switch_Condtion[4], focused_pump_index);
		}
		else	  //???
		{
			Set_FC_StartValue();

		}
		//Show fc pump icon	of purple color
		SetControlVisiable(SCREEN_MAIN_1,PumpIcons[focused_pump_index].icon_pump_fc, VISIBLE);
		SetControlVisiable(SCREEN_MAIN_1,PumpIcons[focused_pump_index].icon_pump_pf, INVISIBLE);
		SHOW_VF_MODE_TXT(focused_pump_index)
		START_WATERFLOW(focused_pump_index)	

		show_frequency();



		//on-off 
		pump_on_off_tbl[focused_pump_index] = ON;



		// mode-fc
		pump_running_mode_tbl[focused_pump_index] = MOD_FC;
		//amount
		amount_of_running_pumps = 1;
		
		if(PumpGroupMode == 2)
		{
			//focused_pump_index = (PumpPointer + 3) % 4;	  // point at the previous of the 1st PF pump.
			index_pf_queue_head = PumpPointer;
			printf("---------------------------focused_pump_index = %d\n",focused_pump_index);
			//dbg only
#ifdef DBG_START_MOD_1_2
			show_on_off();
#endif		
		}
	
#ifdef DBG_START_MOD_1_2
		printf("amount_of_running_pumps = %d\n",amount_of_running_pumps);
		printf("\n\n\n...............Exit of 【void start_mod_1_2(void)】................\n\n");
#endif 	 

		pump_startup_done = FALSE;
		

#ifdef DBG_START_MOD_1_2
		printf("变频转工频延时: %d ms\n", Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14]);
#endif	
		fast_mode = TRUE;
		timer_tick_count_rec = timer_tick_count; 

	}
	else  //real-time pressure is higher than target, sleep.
	{
		pump_startup_done = TRUE;
		pump_switch_phase = 0;
		pump_switch_phase_mod2 = 0;
		pump_fatige_switch_phase = 0;
		SysStandby = TRUE;
		Pump_Status = ALL_SLEEP;
		FallAsleep();


#ifdef DBG_START_MOD_1_2
   		printf("amount_of_running_pumps = %d\n",amount_of_running_pumps);
		printf("\n\n\n...............Exit of 【void start_mod_1_2(void)】................\n\n");
#endif 
 
	}
}


void execute_pump_adding(void)
{
//#define DBG_EXECUTE_PUMP_ADDING
//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_EXECUTE_PUMP_ADDING
    printf("\n\n\n Entrance: void execute_pump_adding(void)...............\n\n");
#endif  
//--------------------------------------------------------------------------------------------------


	adding_pump_executing = TRUE;
	use_new_bucket();
	adding_pump_confirming = FALSE;

#ifdef DBG_EXECUTE_PUMP_ADDING
	printf("To PC-API: Add-pump confirmed\n");
	debugger("Add-pump confirmed\n", ON);

	printf("use_new_bucket();\n");	
    printf("\nExit: 【execute_pump_adding(void)】................\n\n");
#endif
}

/**********************************************************************************************
 * Name：void pump_adding_monitor(void)
 * Brief：monitor the adding-confirming window for a delayed pump adding
 *        or immediately add a pump if cancel-now is ON.
 *        in the 1st case, a time out event causes an action of pump adding
 *  
 * Para：  None
 * Running condtions: RUNNING_GREEN_LIGHT         
 *                    ---  (pump_startup_done == TRUE)     && 
 *                         (pump_switch_phase == 0)        && 
 *                         (pump_fatige_switch_phase == 0) &&
 *                         (pump_switch_phase_mod2 == 0)	
 *        
 * Output：
 * 
 * Caller(s):  
 **********************************************************************************************/ 
void pump_adding_monitor()
{

//#define DBG_PUMP_ADDING_MONITOR
//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_PUMP_ADDING_MONITOR

    printf("\n\n\n Entrance: void pump_adding_monitor()...............\n\n");
#endif  
//--------------------------------------------------------------------------------------------------
#ifdef USE_PUMP_ADDING_WAITING_UINIT
	 if((adding_pump_confirming == TRUE) && \
	     ((timer_tick_count - win1_counter) * 10 >= \
		 1000* ((Pump_Switch_Condtion[9] * 256 + Pump_Switch_Condtion[10])) + \
		    10 * freq_up_counter_gap))// 20210415
#else
	 if((adding_pump_confirming == TRUE) && \
	     ((timer_tick_count - win1_counter) * 10 >= \
		 1000* ((Pump_Switch_Condtion[9] * 256 + Pump_Switch_Condtion[10])) + \
			0))// 20210415
#endif
	 {		 
		 {
		 	printf("Pump-adding waiting time out!!, check the pressure\n");

			if((in_target_zone() == FALSE) && (pid.curr_output >= pid.max_output))//20200304
			{
				
#ifdef DBG_PUMP_ADDING_MONITOR
				printf("pid.out=%0.1f, inspection window time-out, adding_pump = %d, Add-pump confirmed\n", pid.curr_output, adding_pump_confirming);
#endif
				adding_pump_confirming = FALSE;//20200304
				execute_pump_adding();

#ifdef TRAFFIC_LIGHT
    			traffic_light(RED);
#endif

			}	
			else	 //fail to add pump, exit
			{
				adding_pump_confirming = FALSE;

#ifdef DBG_PUMP_ADDING_MONITOR
				printf("Add-pump canceled\n");
#endif
				
#ifdef TRAFFIC_LIGHT
    			traffic_light(GREEN);
#endif
			
			}	
		 }
	 }
	 else if(add_now == TRUE)
	 {
	 	execute_pump_adding();
	    add_now = FALSE;

	 }
#ifdef DBG_PUMP_ADDING_MONITOR
	 printf("\n\n\n   Exit: 【pump_adding_monitor()】................\n\n");
#endif
}

void execute_pump_canceling(void)
{
#define DBG_EXECUTE_PUMP_CANCELING
//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_EXECUTE_PUMP_CANCELING

    printf("\n\n\n Entrance: void execute_pump_canceling(void)...............\n\n");
#endif  
//--------------------------------------------------------------------------------------------------



	canceling_pump = FALSE;
	printf("Cancel-pump confirmed\n");
	
	if (AvfMode == OFF)			 //20210510
	{
		if(pump_usablilty_tbl[0] == TRUE)
		{				
	 		reload_fc_bucket();
		}
	}
	cancel_a_pump();

#ifdef DBG_EXECUTE_PUMP_CANCELING
	printf("--------------cancel_a_pump() done!\n");
    printf("\n\n\n Exit: void execute_pump_canceling(void)...............\n\n");
#endif  


}


uint8 get_floor(void)
{
	if((PumpGroupMode == 2) && (pump_usablilty_tbl[0] == FALSE)) return 0;	//There is only one pf pump running, cancel it anyway
	return 1;  // otherwise after all cancelings are done, at least the fc pump should be running alone.
}

/**********************************************************************************************
 * Name：void pump_canceling_monitor(void)
 * Brief：monitor the canceling-confirming window for a delayed pump canceling
 *        or immediately cancel a pump if cancel-now is ON.
 *        in the 1st case, a time out event causes an action of pump canceling
 *  
 * Para：  None
 * Running condtions: RUNNING_GREEN_LIGHT         
 *                    ---  (pump_startup_done == TRUE)     && 
 *                         (pump_switch_phase == 0)        && 
 *                         (pump_fatige_switch_phase == 0) &&
 *                         (pump_switch_phase_mod2 == 0)	
 *        
 * Output：
 * 
 * Caller(s):  
 **********************************************************************************************/ 

void pump_canceling_monitor()
{
	 if(win2_counter > 0)		   //time out check	for canceling-pump
	 {
		 win2_counter--;
		 if(win2_counter == 0)		  //pump-canceling inspection window time out 
		 {
		 	if(FC_IS_STILL_EMPTY)	  //Outlet pressure is still higher than target
			{
				printf("amount_of_running_pumps = %d\n",amount_of_running_pumps);
				if(amount_of_running_pumps > 1)
				{
					execute_pump_canceling();
#ifdef TRAFFIC_LIGHT
    				traffic_light(GREEN);
#endif
				}
				else	 //fc is empty and the relevant pump is the only running one, pause the pump with system still running
				{					
#ifdef TRAFFIC_LIGHT
    				traffic_light(GREEN);
#endif			
					canceling_pump = FALSE;
				}	
			}
			else   //pressure restored and cancel the action
			{
				canceling_pump = FALSE;
				
				printf("Cancel-pump canceled\n");
#ifdef TRAFFIC_LIGHT
    			traffic_light(GREEN);
#endif
			
			}						
		 }
		 //20220319
		 else //In canceling check-window, not time-out
		 {
		    //up-cross cancel frequency, exit canceling check-window
		    if(pid.curr_output > (get_pump_cancel_freq()) * 300)
		    {
		        win2_counter = 0;
		        canceling_pump = FALSE;
		        printf("\nup-cross cancel frequency, exit canceling check-window\n");
		    }
		 }
	 }
	 else if(cancel_now == TRUE)
	 {
	 	if(amount_of_running_pumps > get_floor())
		{
			printf("Cancel-pump right away\n");
	 		execute_pump_canceling();
#ifdef TRAFFIC_LIGHT
    		traffic_light(GREEN);
#endif
		}
		else
		{
				
			canceling_pump = FALSE;
#ifdef TRAFFIC_LIGHT
    		traffic_light(GREEN);
#endif

		}
	    cancel_now = FALSE;
	 }
}
 
 /**********************************************************************************************
 * Name：void fc_full_empty_radar(void)
 * Brief：Detect FC full/empty events and start a timer for confirming window for pump adding/canceling.
 *        In case all specified pumps are already running, start low pressure check monitor.
 *  
 * Para：  None
 * Running condtions: RUNNING_GREEN_LIGHT         
 *                    ---  (pump_startup_done == TRUE)     && 
 *                         (pump_switch_phase == 0)        && 
 *                         (pump_fatige_switch_phase == 0) &&
 *                         (pump_switch_phase_mod2 == 0)	
 *        
 * Output：
 * 
 * Caller(s):  
 **********************************************************************************************/ 
void fc_full_empty_radar(void)
{

	 //FC is full and start the confirmation timer for pump-adding
	 u32 delta;

//#define DBG_FC_FULL_EMPTY_RADAR          //Enabled for debugging only
//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_FC_FULL_EMPTY_RADAR
    printf("\n\n\n Entrance: void fc_full_empty_radar(void)...............\n\n");
#endif  
//--------------------------------------------------------------------------------------------------

#ifdef DBG_FUNC_FOOTPRINT
		FUNC_GATE_DECORATOR_HEAD
#endif 

	 
	 if(adding_pump_confirming == FALSE)
	 {
		 
		 if(FC_Event == FC_EVENT_FULL)
		 {
			FC_Event = FC_EVENT_NONE; 

#ifdef DBG_FC_FULL_EMPTY_RADAR
			printf("amount_of_running_pumps = %d\n",amount_of_running_pumps);
#endif

			if(amount_of_running_pumps < get_max_nbr())
			{
				if(in_target_zone() == FALSE)
				{
#ifdef USE_PUMP_ADDING_WAITING_UINIT
				    delta = timer_tick_count - freq_up_counter;
				    freq_up_counter_gap = (delta < 100 * (Pump_Switch_Condtion[0] * 256 + Pump_Switch_Condtion[1]))? \
					                      ((Pump_Switch_Condtion[0] * 256 + Pump_Switch_Condtion[1]) * 100 - delta):0;

	#ifdef DBG_FC_FULL_EMPTY_RADAR					
					printf("50hz now!\n");
					printf("freq_up_counter_gap = %d, extra %ds will be added\n",freq_up_counter_gap, freq_up_counter_gap/100);
	#endif
#endif
					win1_counter = timer_tick_count; 
					if( 1 * (Pump_Switch_Condtion[9] * 256 + Pump_Switch_Condtion[10]) == 0) add_now = TRUE;
	
					adding_pump_confirming = TRUE;

#ifdef DBG_TIMING_CONSULTANT 
 	        		printf("************************************************Recommended Fatigue Switching Gap: %d\n", TimingConsultant(CMD_CHK_TIME));
					 //35S
#endif		
		
#ifdef TRAFFIC_LIGHT
    				traffic_light(YELLOW);
#endif		
	
				}
			}
			else if (amount_of_running_pumps >= Max_Pump_Nbr)	//full already
			{
				if(!AllPumpsOn)		  //this is a new full signal
				{
					AllPumpsOn = true;
			
#ifdef DBG_FC_FULL_EMPTY_RADAR
			
				    printf(" ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~AllPumpsOn = true;\n");		
				    sprintf(DbgBuff, "AllPumpsOn = true, amount_of_running_pumps = %d", amount_of_running_pumps);
			        //debugger(DbgBuff, ON);
#endif
	
					reset_pump_pressure_check_startup_monitor();
				
				}
			}
		 }
	 }
	 else		 //Pump-adding Inspection is already running 
	 {
		 if(FC_Event == FC_EVENT_FULL)
		 {
		 	FC_Event = FC_EVENT_NONE; 
#ifdef DBG_FC_FULL_EMPTY_RADAR	
	 	    printf("Pump-adding Inspection is already running, Overflow evnet ignored\n");
#endif
		 }
	 }
	 
	 if(canceling_pump == FALSE)		   // not in cancelling phase, ready for new EMPTY evnt
	 {
		 //pump1 breaks down under fixed fc mode
		 if((PumpGroupMode == 2) && (pump_usablilty_tbl[0] == FALSE) && (AvfMode == FALSE)) //20210624
		 {
			 printf("      pump1 breaks down under fixed fc mode\n\n");
			 if(OutletRealTimePressure > Tartget_Pressure)
 			 {	
				cancel_now = TRUE;
				canceling_pump = TRUE;
				printf("      OutletRealTimePressure is higher than Tartget_Pressure\n\n");
			 }
		 }
		 else
		 {
			 /*
			 	FC is empty and start confirmation timer for pump-canceling
				Confirmation timer is checked in  pump_canceling_monitor() and 
				cancel a pump at time-out(win2_counter == 0)
			 */
			 if(FC_Event == FC_EVENT_EMPTY)
			 {
				FC_Event = FC_EVENT_NONE; 
				win2_counter = 1 * (Pump_Switch_Condtion[11] * 256 + Pump_Switch_Condtion[12]);//Win2_Counter_Setting;
				if(win2_counter == 0) cancel_now = TRUE;
				canceling_pump = TRUE;
#ifdef TRAFFIC_LIGHT
    			traffic_light(YELLOW);
#endif	
			 }
		 }
	 }
	 else	  //Pump-canceling Inspection is already running
	 {
		 if(FC_Event == FC_EVENT_EMPTY)
		 {
		 	FC_Event = FC_EVENT_NONE; 
		 	printf("Pump-canceling Inspection is already running, Underflow evnet ignored\n");
		 }
	 }

#ifdef DBG_FUNC_FOOTPRINT	
	FUNC_GATE_DECORATOR_TAIL	
#endif	
#ifdef DBG_FC_FULL_EMPTY_RADAR 
    printf("Exit: 【void fc_full_empty_radar()】................\n\n");
#endif
  

}

#ifdef DBG_UNIT_TEST
const char* variable_names[3] = 
{
						"pump_startup_done"       ,
	   				    "pump_switch_phase"       ,
						"pump_fatige_switch_phase",
};	
void running_green_light_error_report(void)
{
	uint8 i;
    uint8* arr[3] = {
	   					&pump_startup_done,	 
	   				    &pump_switch_phase,		 
				  	    &pump_fatige_switch_phase,     
				    };
	uint8 val[3] = {
						1,
						0,
						0,
					};
	printf("RUNNING_GREEN_LIGHT FAILED, fc_full_empty_radar() IS SKIPPED\n");
	for(i = 0; i < 3; i++)
	{
		 if(*(arr[i]) != val[i])
		 printf("         %s = %d\n", variable_names[i], *(arr[i]));	
	}

}
#endif

void fc_manager(void)
{

//#define DBG_FC_MANAGER
//*****************************UNIT TEST HEADER*****************************************************
		printf("\n\n\n");
#ifdef DBG_FUNC_FOOTPRINT
		FUNC_GATE_DECORATOR_HEAD
#endif 
 //		printf("Entrance: void fc_manager(void)...............\n\n");

//--------------------------------------------------------------------------------------------------


     start_up_server();

     if(RUNNING_GREEN_LIGHT) 
	 {
	    fc_full_empty_radar();
	 }
#ifdef DBG_UNIT_TEST
	 else
	 {
	 	if(func_dbg_info) running_green_light_error_report();
	 }
#endif
	
     if(RUNNING_GREEN_LIGHT) 
	 {
	 	pump_adding_monitor();
	 }

	 AddPumpSwitchManager();

	 AddPumpSwitchManagerMode2();
	 

	 FatigeSwitchManager();

     if(RUNNING_GREEN_LIGHT) 
	 {
	 	pump_canceling_monitor();
	 }

#ifdef DBG_FC_MANAGER
     show_on_off();
#endif  


	printf("\n\n\n");
#ifdef DBG_FUNC_FOOTPRINT		
	FUNC_GATE_DECORATOR_TAIL
#endif
//	printf("Exit: void fc_manager(void)...............\n\n"); 

}

/**********************************************************************************************
 * Name：int next_running_pump(uint8 ind)
 * Brief：find the next running pump and return the index (0~3)
 * Para： 
 *
 * Return ：-1: nothing found 
 * Caller(s): Start_Stopper()
 **********************************************************************************************/
int next_running_pump(uint8 ind)
{
	if(ind > 5) return -1;
	while(pump_on_off_tbl[ind] == OFF)
	{
	   ind++;
	   if(ind > 5) return -1;	
	}			
	return ind;

}

/**********************************************************************************************
 * Name：void end_of_stopper_watchdog()
 * Brief：Monitor the stopping procedure and make a final stop as soon as no running pumps are left.
 * 
 * Caller(s): stop_manager() | Start_Stopper()
 **********************************************************************************************/
void end_of_stopper_watchdog(void)
{
	if(amount_of_running_pumps == 0)
	{
		stop_cnt = 0;
		Set_FC_Zero();
		SoftStopping = FALSE;
		SysStandby = FALSE;

		//frequency display:--
		SetTextValue(SCREEN_MAIN_1, TXT_CURRENT_FREQUENCY, "--");
 		delay_ms(100);
		RefreshButtons();

		printf("----------------amount_of_running_pumps = %d\n",amount_of_running_pumps);

	}
}


/**********************************************************************************************
 * Name：stop_manager()
 * Brief：Check stop_cnt(every 1s) and stop the next running pump if it reaches 0.
 *  	  The core logic of stopping a team of running pumps is doing it in turn, with a
 *        time gap(here controled by Pump_Switch_Condtion[7]) in between. Thus this function 
 *        plays a role of gap monitor, once the gap has elapsed the next pump is stopped 
 *        right away while new gap is set for next phase.
 * Para： 
 * Core variables: stop_cnt-- the counter for gap measuring.	
 * Return ： 
 * Timing: Every 1s
 * Caller(s): Pump_Manager() 
 **********************************************************************************************/
void stop_manager()
{	
	if(stop_cnt > 0)
	{
		stop_cnt--;
		if(stop_cnt == 0)		 //time out
		{
			stop_pointer = next_running_pump(stop_pointer + 1);	
			
			if(stop_pointer >= 0)		//find one
			{
				stop_cnt = Pump_Switch_Condtion[7] * 1; // new time gap is set (in 0.1s)

				stop_a_pump(stop_pointer);

				if(amount_of_running_pumps > 0) amount_of_running_pumps--;
			    printf("amount_of_running_pumps = %d\n",amount_of_running_pumps);
				end_of_stopper_watchdog();
		
		
			}
		}			
	}
}

void stop_small_pump(void)
{

	stop_a_pump(IND_SMALL_PUMP);

	stop_cnt = 0;
	Set_FC_Zero();
	FC_SWITCH = OFF;
	SoftStopping = FALSE;
	SysStandby = FALSE;
	
	//frequency display:--
	SetTextValue(SCREEN_MAIN_1, TXT_CURRENT_FREQUENCY, "--");


}

/**********************************************************************************************
 * Name：Start_Stopper()
 * Brief：Kick off the stopping of all running pumps. see the description in func. stop_manager() 	
 * Return ： 
 * Timing: Every 100 ms
 * Caller(s): Status.c/WaterLevelMonitor(), main.c/NotifyButton-SCREEN_MAIN_1
 **********************************************************************************************/
void Start_Stopper(uint8 stop_type)
{

#ifdef DBG_PUMP_MANAGER
	show_on_off();
	printf("---------Start_Stopper-------amount_of_running_pumps = %d\n",amount_of_running_pumps);
#endif

#ifdef DUAL_WATER_SOURCE
   //Shut off the water gate from entrance of the tank
	WaterSourceTankSwitching(OFF);
#endif

	fast_mode = FALSE;
	//fc off
	FC_SWITCH = OFF;
	//reset fc
	Set_FC_Zero();
	sleep_chk_timer = 0;
	pump_fatige_switch_phase = 0;


	//Data protection 
	if(rec_focused_pump_index != PumpPointer) //pump pointer has been changed, save it
	{
		PumpPointer = rec_focused_pump_index;
		save_pump_pointer();
	}

#ifdef  DBG_PUMP_MANAGER
	printf("Pump_Switch_Condtion[7] = %d\n",Pump_Switch_Condtion[7]);
#endif

	pump_waking_up_from_half_phase1 = FALSE;
	pump_waking_up_from_half_phase2 = FALSE;

	//Small pump
	if(Pump_Status == SMALL_PUMP_ON)
	{
		stop_small_pump();
		amount_of_running_pumps = 0;
		end_of_stopper_watchdog();
	}
	else  //main pump(s) running
	{
		   //no delay between pumps	, stop simutaneusly
		if((Pump_Switch_Condtion[7] == 0) || (Pump_Status == ALL_SLEEP))
		{
			stop_all_pumps_immediately();
			Set_FC_Zero();
			SoftStopping = FALSE;
			amount_of_running_pumps = 0;
			
			SysStandby = FALSE;
			
			//frequency display:--
			SetTextValue(SCREEN_MAIN_1, TXT_CURRENT_FREQUENCY, "--");
		}
		else  //stop in turn
		{
			stop_pointer = next_running_pump(0);
		
			if(stop_pointer >= 0)	 //a running pump found
			{
				SoftStopping = TRUE;

				RefreshButtons();
	
				stop_cnt = Pump_Switch_Condtion[7] * 1;   //time gap between pumps(in second)

				empty_fc_bucket();
	
				stop_a_pump(stop_pointer);
	
				amount_of_running_pumps--;
				printf("amount_of_running_pumps = %d\n",amount_of_running_pumps);
				end_of_stopper_watchdog();
			
			}
		}
	}
	if(stop_type == STOP_TYPE_SYSTEM)
	{
	
		RefreshErrorInfo(0, ADD_ERROR_NO);
	}
	AllPumpsOn = false;
}


/****************Every 1s *********************/		
void Pump_Manager()
{
//*****************************UNIT TEST HEADER*****************************************************
		
		printf("\n\n\n");
#ifdef DBG_FUNC_FOOTPRINT
		FUNC_GATE_DECORATOR_HEAD
#endif 
 //		printf("Entrance: void Pump_Manager()...............\n\n");
 
//--------------------------------------------------------------------------------------------------

	if(RUNNING_GREEN_LIGHT) SleepManager();
	
	if(PID_GREEN_LIGHT)
	{
		if(PumpGroupMode == 3)
		{

		}
	    else
		{
		 	if(FC_MANAGER_GREEN_LIGHT) fc_manager();
			
		}		
	}
	else if(SoftStopping == TRUE)
	{
		stop_manager();
	}

		printf("\n\n\n");
#ifdef DBG_FUNC_FOOTPRINT		
		FUNC_GATE_DECORATOR_TAIL
#endif

}



uint8 get_fc_pump_index()
{
	if ((PumpGroupMode == 1) || (AvfMode == TRUE))  return focused_pump_index;
	return 0;
}

uint8 IsSmallPump()
{
   if(get_fc_pump_index() == 5) return TRUE;
   return FALSE;

}
/*
return -1: 	nothing found
return >=10: spare found
return 0-9: pf found

*/
int find_target_pump(uint8 ind)
{
	uint8 i;
	int result = -1;
	int fatigue_blocked = -1;
	if((Pump_Status == SMALL_PUMP_ON)) return -1;  //small pump is wrong, no backup available

 	for(i = (ind + 1) % 5; i != ind; i = (i + 1) % 5)
    {
    	if(pump_usablilty_tbl[i] == ON)
		{
			if(i != ind)
			{
				if(pump_on_off_tbl[i] == OFF)
				{ 
					if (BLOCKED_BY_FATIGUE_SWITCH(i) == FALSE) return  10 + i; //non-overriding selection: find a free pump.
					fatigue_blocked = 10 + i;
				}
				else if(result == -1) result = i;  //record the 1st ON-pump only.(in case searching for OFF-pump fails, 
												//return this for 'overriding selection')					
			}
		}
    }
	if (fatigue_blocked >= 10) result = fatigue_blocked;
	return result;	
}

void jump_to_new_fc_pump(uint8 ind)
{
	pump_fatige_switch_exch_mode = FALSE;

	pump_fatige_switch_phase = 1;
	f_switch_new_fc_pump_index = ind;
	fast_mode = TRUE;


}
extern TYPE_FATIGUE_PUMP_CANCEL FatiguePumpCancel;
void FatigueCancelMonitorAvf(void)
{
	char buff[5];
	if ((pump_on_off_tbl[FatiguePumpCancel.pump_ind]) && (focused_pump_index != FatiguePumpCancel.pump_ind))
	{
		if (FatiguePumpCancel.freq)
		{
			FatiguePumpCancel.freq--;
			UpdatePumpFreq(FatiguePumpCancel.freq, FatiguePumpCancel.pump_ind);
			sprintf(buff, "%.1f", (float)(FatiguePumpCancel.freq));
			SetTextValue(SCREEN_MAIN_1, PumpIcons[FatiguePumpCancel.pump_ind].icon_freq, (uchar*)buff);
			if (FatiguePumpCancel.freq == 0)
			{
				stop_a_pump(FatiguePumpCancel.pump_ind); 
				if(amount_of_running_pumps > 0) amount_of_running_pumps--;

			}
		}
	}
}

int find_a_spare_pump(void)
{
	uint8 i;
	for(i = (focused_pump_index + 1) % 5; i != focused_pump_index; i = (i + 1) % 5)
    {
    	if ((pump_usablilty_tbl[i] == ON) && (pump_on_off_tbl[i] == OFF))
		{
			return i;
		}
    }
	return -1;
}
//#define USE_SWITCH_BUFFER
void start_fatige_switching_avf(void)
{
	int result;
	uint8 ind;

	printf("---------------------------开始倒泵(AVF MODE):");

	result = find_the_pump_to_be_canceled_avf_mode();
	printf("Stop Pump%d\n",result + 1);

	if (Max_Pump_Nbr == 1)		//the only pump is running
	{
		result = find_a_spare_pump();
		if (result >= 0)
		{
			stop_a_pump(focused_pump_index); 

			focused_pump_index = result;
			StartAnFcPump();
			
		}
	}  	  //20210625
	else
	{

		if (result <= 4)  //is a main pump: 0~4
		{
#ifdef USE_SWITCH_BUFFER
			START_FATIGUE_CANCEL_AVF(result);
#else
			stop_a_pump(result); 
			if(amount_of_running_pumps > 0) amount_of_running_pumps--;
#endif		
		}
		else 
		{
			 printf("No spare pump found! Fatigue swtich is cancelled\n");
		}
	}

	//Set interval for next switching
	RESET_FATIGE_SWITCHING_TIMER

}


int find_a_running_pf_pump(uint8 skip_pump1)
{
	static uint8 ind = 3;
	uint8 i;
	ind = (ind + 1) % 5; //right-shift	
							
	for (i = (ind + 1) % 5; i != ind; (i = i + 1) % 5)								
	{	
							
		if (pump_on_off_tbl[i] == ON) 
		{
			if (skip_pump1 == FALSE) return i;	
			if (i != 0) return i;
		}													
	}	
	
	return -1;							
}

#define IS_FIXED_FC_MODE               1
void start_fatige_switching(void)
{
	int result, temp_ind1, temp_ind2;
	uint8 ind;

      printf("开始倒泵\n");

#ifdef DBG_EVENT_AS_ERROR
	  EventRecorder("开始倒泵");
#endif

#ifdef TRAFFIC_LIGHT
    traffic_light(GREEN);
#endif
	if (AvfMode == TRUE) 
	{
		 start_fatige_switching_avf();
		 return;
	}
	
	//result = find_target_pump(focused_pump_index);	 //find a spare pump

	if(PumpGroupMode == 2)		// mode2: fixed fc
	{
		if(get_max_nbr() > 1)	 // more than one pump usable: pf pump is permitted
		{
			temp_ind1 = find_a_running_pf_pump(IS_FIXED_FC_MODE);
	
			if (temp_ind1 >= 0)		//valid pump to pause
			{
				temp_ind2 = find_target_pump(temp_ind1);
			
				if (temp_ind2 >= 10)   //valid pump to start
				{
					temp_ind2 = temp_ind2 - 10;	
					pause(temp_ind1);
					switch_to_pf(temp_ind2);	
					focused_pump_index = temp_ind2;			
					amount_of_running_pumps++;
					printf("                         switch pump: [%d ---> %d]\n\n", temp_ind1, temp_ind2);

				}
			}
				
#if 0 //old code
				/*
				if(focused_pump_index == 0)		  //pump1: fixed-fc mode, can not be paused
				{
					focused_pump_index = 3 - result;		  //find a running pf-pump to stop 
				}

				if(pump_on_off_tbl[focused_pump_index] == ON) pause(focused_pump_index);

				 //???what if only fc-pump is running
			    switch_to_pf(result);
				focused_pump_index = result;	 //point at the new pump( to be stopped next time)
				amount_of_running_pumps++;
				*/
#endif

			RESET_FATIGE_SWITCHING_TIMER
		
#ifdef DBG_PUMP_MANAGER
			show_on_off();
		    sprintf(DbgBuff, "start_fatige_switching");
	        debugger(DbgBuff, ON);
#endif
			
		}
	}
	else	//Mode1: floating fc	    
	/*
		Try to find a pare pump, then stop a pf pump if any, Switch off FC before enter 
		state machine flow where the spare pump will be switched on
		There might be only one fc pump running
	*/
	{
		result = find_target_pump(focused_pump_index);	 //find a spare pump
		if(result >= 10)   //spare pump found
		{
			pump_fatige_switch_exch_mode = FALSE;	

			f_switch_new_fc_pump_index = result % 10;
			old_pump_index = focused_pump_index;	 //record the fc index

			if(amount_of_running_pumps == 1) 
			{
				fc_pump_only = true;
			}
			else 
			{
				fc_pump_only = false;
				ind = find_the_pump_to_be_canceled();
				pause(ind);
				freeze_pf_pump(ind, ON);
			}
 		   
			FC_SWITCH = OFF;

			pump_fatige_switch_phase = 1;

			adding_pump_executing = TRUE;

			fast_mode = TRUE;

#ifdef DBG_FUNC_INFO
			if(func_dbg_info)
			{
				printf("FC_SWITCH = OFF; 变频转工频延时: %d ms\n", Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14]);
			}
#endif
			f_switch_timer_tick_count_rec = timer_tick_count; 
#ifdef DBG_PUMP_MANAGER
				show_on_off();
			    sprintf(DbgBuff, "start_fatige_switching");
		        debugger(DbgBuff, ON);
#endif

		}
	}

}

/**
  * @brief   			This function identifies entrance of fatigue swicthing and starts it.
  * @param   			None
  * @retval  			None
  * @details			When time-out occurs GREENLIGHT is checked, if it is ON, start switch immediately
  *          			Otherwise enter a waiting phase where GREENLIGHT is checked every second for switching
  * @execution_period:  1 second
  */

void FatigePumpSwitcher(void)
{
 
	if(Pump_Status == MAIN_PUMP_ON)
	{
		if(waiting_for_green_light)	   //need to check green light
		{						
			if(FATIGE_SWITCH_GREEN_LIGHT)
			{
			   	waiting_for_green_light = FALSE;
				if(SparePumpMode == SPARE_PUMP_MODE_NORMAL)
				{
					printf("SPARE_PUMP_MODE_NORMAL, start_fatige_switching();\n");
					printf("Finally Green light is on,  Go switch FC pump\n");	

					RESET_FATIGE_SWITCHING_TIMER			    	
					
					start_fatige_switching();
				}
				else
				{
					printf("SPARE_PUMP_MODE_ERROR, jump_to_new_fc_pump(new_pump_ind);;\n");
					jump_to_new_fc_pump(new_pump_ind);
					SparePumpMode = SPARE_PUMP_MODE_NORMAL;

				}				
			   	
			}	
			else
			{
				printf("Still Red light! Keep waiting...\n");
			}
		}	 
		else if(NewMinuteEvent)
		{
			if(RunningCounter > 0)
			{				
				RunningCounter--;
				printf("RunningCounter = %d\n",RunningCounter);


				if(RunningCounter == 0)	   //Time out! Prepare for pump switching
				{
					if(!(FATIGE_SWITCH_GREEN_LIGHT))	 //greenlight is not ON, enter waiting phase
					{
						waiting_for_green_light = TRUE;
						printf("Red light! waiting...\n");
					}
					else                                 //greenlight is ON, start switching
					{
						printf("1st Check: greenlight is ON! Go switch FC pump\n");
						start_fatige_switching();
					}
				}
			}
			NewMinuteEvent = FALSE;		
		}		  		
	}
}

void update_focused_pump(void)
{
	uint8 i;

 	for(i = 0; i < 5; i++)
    {
		if(pump_on_off_tbl[i] == ON) {focused_pump_index = i; return;}
	}
 	focused_pump_index = 0;
}

/**********************************************************************************************
 * Name：void UseSparePump(uint8 deffect_ind)
 * Brief：stop the defective pump and use a new one if needed
 * Details: 
 *       --Case 1: in FixedFc mode, the only fc pump defects, just stop it and FC source
 *       --Case 2: in FloatingFc mode
 *                  if defected pump is in pf mode, stop it and let pump-adding take action automatically
 *                  else defected pump is in fc mode, go to entrance of pump-adding where the defected will
 *                                                    be paused(and fail to work on pf due to the defection) and 
 *                                                    a new fc pump will be adopted 
 *       --Case 3: AVF mode: the focused pump defects, find a backup to start   
 *       --Case 4: AVF mode: unfocused pump defects, find a backup to start 
 * Para：    	
 * Return ： 
 * 
 * Caller(s):  main.c/main() 
 **********************************************************************************************/
void UseSparePump(uint8 deffect_ind)	
{
	 int ind;
#define DBG_USESPAREPUMP          //Enabled for debugging only
//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_USESPAREPUMP

    printf("\n\n\n Entrance: void UseSparePump(uint8 deffect_ind)...............\n\n");
#endif  
//--------------------------------------------------------------------------------------------------

	
 	 err_pump_ind = deffect_ind; 

	
	if (AvfMode == TRUE)
	{
		UpdatePumpFreq(0.0, err_pump_ind);
		SetTextValue(SCREEN_MAIN_1, PumpIcons[err_pump_ind].icon_freq, "--");

		stop_a_pump(err_pump_ind); 
		printf("err_pump_ind = %d\n",err_pump_ind);

		if(amount_of_running_pumps > 0) amount_of_running_pumps--;
		printf("amount_of_running_pumps = %d\n",amount_of_running_pumps);

		if (focused_pump_index != err_pump_ind) return; //not the focused pump which plays a role of locomotive, 
												        //no need to find a new one


		//Since the focused pump has error, a new one is needed or the whole train will lost its locomotive
		ind = find_target_pump(err_pump_ind);
		
		if(ind >= 10) 				 //not full, find a new backup
		{
			printf("find a new backup: pump%d, set 20hz\n", ind - 9);
			focused_pump_index = ind - 10;
			
			StartAnFcPump();
			UNBLOCK_FATIGUE_SWITCH	   //relief any blocked pump
		}
		else
		{
			printf("No pump is available\n");
		
			if (amount_of_running_pumps == 0) AllPumpsDead = TRUE;	   //20210624
			else 
			{
			   	update_focused_pump(); 
			}
			printf("AllPumpsDead = %d\n",AllPumpsDead);

		}

		return;

	}


	if(PumpGroupMode == 2)
	{
		
		if(deffect_ind == 0)
		{
			printf("               pump 1 error in FixFc Mode, Turn off FC and pump0\n");
			FC_SWITCH = OFF;

			Set_FC_Zero();
			show_frequency();	

		}
		pause(deffect_ind);
		if(amount_of_running_pumps == 0)  AllPumpsDead = TRUE;

#ifdef DBG_PUMP_MANAGER
			
		show_on_off();
#endif
	    return;
	}


	if(pump_running_mode_tbl[deffect_ind] == MOD_PF)
	{
		pause(deffect_ind);
		
    	return;

	}

	//for FC pump error
	old_pump_index = deffect_ind; 
	
	FC_SWITCH = OFF;	 

	if(pump_fatige_switch_phase == 0)	 //not in fatige switching
	{
	
		pump_switch_phase = 1;	  //add pump
	
		fast_mode = TRUE;
	
#ifdef DBG_FUNC_INFO
		if(func_dbg_info)
		{
			printf("变频转工频延时: %d ms\n", Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14]);
		}
#endif
	
		timer_tick_count_rec = timer_tick_count; 
	}
	else   //in fatige switching
	{
	
		pump_fatige_switch_phase = 1;	  //add pump
		freeze_pf_pump(0, OFF);    //set free the frozen pump if any.
	
		fast_mode = TRUE;
	
#ifdef DBG_FUNC_INFO
		if(func_dbg_info)
		{
			printf("变频转工频延时: %d ms\n", Pump_Switch_Condtion[13] * 256 + Pump_Switch_Condtion[14]);
		}
#endif
	
		f_switch_timer_tick_count_rec = timer_tick_count; 
	}

	printf("pause deffected pump%d\n", deffect_ind + 1);

}  


 
//#define TARGET_PRESSURE_DELTA	0.01  //0.03
uint8 PressureInTargetRange(void)
{
	return (OutletRealTimePressure >= Tartget_Pressure - TargetLockerDelta) && (OutletRealTimePressure <= Tartget_Pressure + TargetLockerDelta);
}


void FC_Retreator(void)
{
	char buff[10];
	if((Pump_Status == MAIN_PUMP_ON) || (Pump_Status == SMALL_PUMP_ON))
	{
		pid.curr = OutletRealTimePressure * 100.0;	
		//Decrease frequency 
		if(pid.curr_output > Pump_Switch_Condtion[4] * 300) pid.curr_output -= 15.0;//300.0 / 20;	 //decrease 0.05hz
		if(pid.curr_output <= Pump_Switch_Condtion[4] * 300) //lower than startup frequency
		{
	    	pid.curr_output = Pump_Switch_Condtion[4] * 300 - FC_FLOOR_DELTA;
			FC_Event = FC_EVENT_EMPTY;    
		}
		 
		//FC Output
		Control_FC();	

		//-------------------------------Display:frequency/progress bar------------------------------------------------		 
		show_frequency_for_pump(focused_pump_index);				//20210623
	}
}

void RefreshTarget(void)		 //20210707
{
	
	if ((RemoteMode == TRUE) && (RemoteTargetEnable == TRUE)) 	 //Remote Mode
	{
		Tartget_Pressure = RemoteTargetPressure;
	}
	else													     //Local Mode
	{
		Tartget_Pressure = LoadTargetPressure(); 
	}											  
	ShowTargetPressure(Tartget_Pressure);
	PidSetTargetPressure(Tartget_Pressure);
}

#ifdef DBG_TIMING_CONSULTANT	
 	uint16 TimingConsultant(uint8 cmd)
	{
		static uint16 _timer = 0;
		switch (cmd)
		{
			case CMD_COUNTING:
				_timer++;
				printf("TimingConsultant._timer = %d\n", _timer);
				break;
			case CMD_CHK_TIME:
				return _timer;
				break;
			case CMD_RESET_TIME:
				_timer = 0;
				break;
			default:
				break;
		}
	}
#endif

//20210805

float MileMeters[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
#define LOWER_LIMIT      -4320000.0	   //100days
void PumpMileageMonitor(void)
{
	 uint8 i;
	 for (i = 0; i < 5; i++)
	 {
		  if (pump_on_off_tbl[i])                MileMeters[i] += 1.0;
		  else if (MileMeters[i] > LOWER_LIMIT)   MileMeters[i] -= 0.5;	 
		  printf("~      %0.1f", MileMeters[i]);
	 }
	  printf("\n\n");
}

//20210805
#define MAX_MILEAGE      8640000.0
int FindLeastUsedPump(void)
{
	 uint8 i;
	 int j = -1;
	 float mileage = MAX_MILEAGE;
	 for (i = 0; i < 5; i++)
	 {
		 if(pump_usablilty_tbl[i] == ON)
		 {
			 if (MileMeters[i] < mileage)
			 {
				 mileage = MileMeters[i];
				 j = i;
			 }
		 }
	 }
	 return j;
}
	   