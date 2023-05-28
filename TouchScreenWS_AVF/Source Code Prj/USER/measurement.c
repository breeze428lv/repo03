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
#include "tables.h"
#include "error_handle.h"
#include "settings.h"
#include "pump_running.h"
#include "UserInterface.h"

//--------------------Debugging---------------------------------------	
#ifdef DBG_WDG_TEST
uint8 DogFood = ON; 
#endif

#ifdef USE_CURVES
	uint8 up_crossed = FALSE;	
	float max_curve_pressure, min_curve_pressure;
#endif

#ifdef DBG_FUNC_INFO
    extern uint8 func_dbg_info;
#endif  
#ifdef DBG_FUNC_LEN
	extern volatile  uint32 timer_tick_count; 
#endif

#define TARGET_CLOCKER_ENABLED        TargetLockerTiming > 0     

//--------------------Normal Variables---------------------------------
extern uint8 pump_enable_tbl[];
extern uint8 WarmUpDone;
extern float Tartget_Pressure;
extern uint8 task_index;
extern bool TargetLockConfirmed;
extern uint8 TargetLockCounter;
extern float temp_limit[];
extern uint8 TempMonitorEnable;
float pump_temp[6];

float real_outlet_pressure;

uint8 TargetLockerTiming;


float EntranceRealTimePressure;

float PM_pressure;
float PT_pressure;
float WaterLevel;

extern uint8 Outlet_Sensor_type;

extern uint8 Entrance_Sensor_type;

extern float manual_freq_setting;

extern uint8 now_year;
extern uint8 now_month;
extern uint8 now_day;

extern float Outlet_Sensor_Range;
extern float Entrance_Sensor_Range;

extern uint8 WaterSourceMode;

extern uint8 Entrance_sensor0[];

extern uint8 SysRunning;

extern uint16 AD_Value[N][M]; //用来存放ADC转换结果，也是DMA的目标地址

extern uint16 After_filter[M]; //用来存放求平均值之后的结果

uint8 channel_nbr = 1;

extern uint8 Pressure_Full_range;

extern int Pressure_Bias;

extern uint8 Low_Pressure_Protection_Selected;
extern uint8 Low_Pressure_Protection_Timing;

extern uint8 Manual_Setting;

float OutletRealTimePressure;
float OutletRealTimePressureForCali;
float EntranceRealTimePressureForCali;	

uint8 UseIoTargetPressure;								
float DefaultTargetPressureIO;								
						  
float phy_quantities[10] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
	
											 //p1_tmp  p2_tmp  p3_tmp  p4_tmp 
float phy_quantities_bias[10] = {0.0, 0.0, 0.0,  0.0,   0.0,    0.0,   0.0,  0.0, 0.0, 0.0} ;	
												 //3

typedef struct							
{							
       int         x1;							
       int         x2;							
       float       y1;							
       float       y2;							
							
}SENSOR_PAR;							
							
SENSOR_PAR  SensorParTable[3] = {							
					              {NTC_Vol1, NTC_Vol2, NTC_Temp1, NTC_Temp2},     //NTC							
					              {PM_Vol1 , PM_Vol2 , PM_Temp1 , PM_Temp2 },     //PM	(4-20mA) default						
					              {PT_Vol1 , PT_Vol2 , PT_Temp1 , PT_Temp2 }      //PT	(0-5V)						
							
};	

/*NTC data: temperature(°C) vs. voltage(V)*100*/
float temp_tbl[67][2] ={
						{0, 150},
						{5, 150},
						{10, 145},
						{15, 137.5},
						{20, 130},
						{25, 122.5},
						{30, 115},
						{38, 103},
						{40, 100},
						{46, 95},
						{51, 90},
						{58, 85},
						{60, 83},
						{65, 80},
						{72, 76},
						{75, 74.5},
						{78, 73},
						{82, 71},
						{89, 68},
						{93.5, 66},
						{101, 63},
						{106, 61},
						{111, 59},
						{115, 57.5},
						{119, 56},
						{125, 54},
						{130, 52},
						{135, 50.3},
						{139, 49},
						{145, 47},
						{148, 46},
						{154, 44},
						{162, 42},
						{168, 40},
						{170, 39},
						{178, 37},
						{180, 36.5},
						{184, 35},
						{191, 33},
						{195, 32},
						{200, 31},
						{208, 28},
						{210, 27},
						{214, 25},
						{220, 23.9},
						{225, 23},
						{230, 20},
						{238, 17},
						{240, 16},
						{245, 15},
						{253, 12},
						{258, 10},
						{263, 8},
						{265, 7.2},
						{272, 5},
						{278, 3},
						{280, 2.3},
						{283, 1},
						{290, -4},
						{294, -6},
						{301, -8},
						{304, -10},
						{306, -12},
						{315, -21},
						{320, -26},
						{325, -31},
						{330, -36},
						
};

#define DBG_USE_NEG
#ifdef 	DBG_USE_NEG
/*PM data: Pressure(Mpa) vs. voltage(V)*100*/
float pressure_tbl[34][2] ={
							{0, -0.1},
							{10, 0.01},
							{20, 0.12},
							{28.4, 0.19},
							{40, 0.29},
							{47.4, 0.37},
							{60, 0.5},
							{75.7, 0.64},
							{85.3, 0.74},
							{94.8, 0.83},
							{100, 0.9},
							{110, 1.0},
							{123.4, 1.1},
							{130, 1.18},
							{142, 1.28},
							{152, 1.37},
							{161, 1.46},
							{170, 1.55},
							{180, 1.63},
							{200, 1.8},
							{205, 1.85},
							{210, 1.88},
							{220, 1.95},
							{230, 2.05},
							{240, 2.15},
							{250, 2.25},
							{260, 2.35},
							{270, 2.45},
							{280, 2.55},
							{290, 2.65},
							{300, 2.75},
							{310, 2.85},
							{320, 2.95},
							{330, 3.05},

						
};


#else
/*PM data: Pressure(Mpa) vs. voltage(V)*100*/
float pressure_tbl[34][2] ={
							{0, 0},
							{10, 0.01},
							{20, 0.12},
							{28.4, 0.19},
							{40, 0.29},
							{47.4, 0.37},
							{60, 0.5},
							{75.7, 0.64},
							{85.3, 0.74},
							{94.8, 0.83},
							{100, 0.9},
							{110, 1.0},
							{123.4, 1.1},
							{130, 1.18},
							{142, 1.28},
							{152, 1.37},
							{161, 1.46},
							{170, 1.55},
							{180, 1.6},
							{205, 1.6},
							{200, 1.6},
							{210, 1.6},
							{220, 1.6},
							{230, 1.6},
							{240, 1.6},
							{250, 1.6},
							{260, 1.6},
							{270, 1.6},
							{280, 1.6},
							{290, 1.6},
							{300, 1.6},
							{310, 1.6},
							{320, 1.6},
							{330, 1.6},
						
};
#endif
/*PT data: Pressure(Mpa) vs. voltage(V)*100*/
#ifdef 	DBG_USE_NEG
float pt_pressure_tbl[34][2] ={
							{0, -0.44},
							{10, -0.35},
							{19.5, -0.26},
							{29.7, -0.17},
							{40, -0.08},
							{50, 0.01},
							{60, 0.1},
							{70, 0.22},
							{80, 0.32},
							{90, 0.4},
							{100, 0.53},
							{110, 0.62},
							{120, 0.7},
							{130, 0.75},
							{140, 0.84},
							{150, 0.95},
							{160, 1.05},
							{170, 1.1},
							{180, 1.2},
							{190, 1.32},
							{200, 1.4},
							{210, 1.47},
							{220, 1.57},
							{230, 1.66},
							{240, 1.75},
							{250, 1.84},
							{260, 1.93},
							{270, 2.02},
							{280, 2.11},
							{290, 2.2},
							{300, 2.29},
							{310, 2.38},
							{320, 2.47},
							{330, 2.56},

						
};
#else
float pt_pressure_tbl[34][2] ={
							{0, 0},
							{10, 0.0},
							{19.5, 0.0},
							{29.7, 0.0},
							{40, 0.0},
							{50, 0.0},
							{60, 0.1},
							{70, 0.22},
							{80, 0.32},
							{90, 0.4},
							{100, 0.53},
							{110, 0.62},
							{120, 0.7},
							{130, 0.75},
							{140, 0.84},
							{150, 0.95},
							{160, 1.05},
							{170, 1.1},
							{180, 1.2},
							{190, 1.32},
							{200, 1.4},
							{210, 1.47},
							{220, 1.57},
							{230, 1.6},
							{240, 1.6},
							{250, 1.6},
							{260, 1.6},
							{270, 1.6},
							{280, 1.6},
							{290, 1.6},
							{300, 1.6},
							{310, 1.6},
							{320, 1.6},
							{330, 1.6},
						
};
#endif

float pm_wl_tbl[34][2] ={
							{0, 0},		   
							{9, 20},		//0.09,20
							{20, 70},
							{30, 120},
							{40, 178},
							{50, 240},
							{60, 300},
							{70, 360},
							{80, 410},
							{90, 465},
							{100, 520},
							{110, 580},
							{120, 640},
							{130, 698},
							{140, 750},
							{150, 806},
							{160, 867},
							{170, 930},
							{180, 985},
							{190, 1000},
							{200, 1000},
							{210, 1000},
							{220, 1000},
							{230, 1000},
							{240, 1000},
							{250, 1000},
							{260, 1000},
							{270, 1000},
							{280, 1000},
							{290, 1000},
							{300, 1000},
							{310, 1000},
							{320, 1000},
							{330, 1000},
						
};

float pt_wl_tbl[34][2] ={
							{0, 0},		   
							{9, 0},		//0.09,20
							{20, 0},
							{30, 0},
							{40, 0},
							{50, 50},
							{60, 60},
							{70, 120},
							{80, 178},
							{90, 240},
							{100, 305},
							{110, 360},
							{120, 410},
							{130, 460},
							{140, 500},
							{150, 560},
							{160, 615},
							{170, 670},
							{180, 725},
							{190, 757},
							{200, 810},
							{210, 870},
							{220, 927},
							{230, 990},
							{240, 1000},
							{250, 1000},
							{260, 1000},
							{270, 1000},
							{280, 1000},
							{290, 1000},
							{300, 1000},
							{310, 1000},
							{320, 1000},
							{330, 1000},
						
};



void SetBias(float bias, uint8 i)
{
	phy_quantities_bias[i] = bias;
}
						
/*Read stub data for calculation of temperature for an input voltage*/
void temp_para_picker(int v, float* t1, float* t2, int* v1, int* v2)
{

	uint8 ind;

	if(v > 330) v = 320;
	if(v < 0) v = 0;

	ind = (int)(v/5);

	*v1 = temp_tbl[ind][0];
	*v2 = temp_tbl[ind + 1][0];
	*t1 = temp_tbl[ind][1];
	*t2 = temp_tbl[ind + 1][1];
	
}

/*Read stub data for calculation of pressure for an input voltage*/
void pressure_para_picker(int v, float* t1, float* t2, int* v1, int* v2, uint8 ss_type)
{
	uint8 ind;

	if(v > 330) v = 220;
	if(v < 0) v = 0;

	ind = (int)(v/10);
	if(ss_type == PM)
	{
		*v1 = pressure_tbl[ind][0];
		*v2 = pressure_tbl[ind + 1][0];
		*t1 = pressure_tbl[ind][1];
		*t2 = pressure_tbl[ind + 1][1];
	}
	else if(ss_type == PT)
	{
		*v1 = pt_pressure_tbl[ind][0];
		*v2 = pt_pressure_tbl[ind + 1][0];
		*t1 = pt_pressure_tbl[ind][1];
		*t2 = pt_pressure_tbl[ind + 1][1];
	}	
}


void wl_para_picker(int v, float* t1, float* t2, int* v1, int* v2, uint8 ss_type)
{
	uint8 ind;

	if(v > 330) v = 220;
	if(v < 0) v = 0;

	ind = (int)(v/10);
	if(ss_type == HM_PM)
	{
		*v1 = pm_wl_tbl[ind][0];
		*v2 = pm_wl_tbl[ind + 1][0];
		*t1 = pm_wl_tbl[ind][1];
		*t2 = pm_wl_tbl[ind + 1][1];
	}
	else if(ss_type == HM_PT)
	{
		*v1 = pt_wl_tbl[ind][0];
		*v2 = pt_wl_tbl[ind + 1][0];
		*t1 = pt_wl_tbl[ind][1];
		*t2 = pt_wl_tbl[ind + 1][1];
	}	
}
/*The range factor is proportional to Pressure_Full_range set by user*/
float range_factor(uint8 channel, int sensor_type)
{

   if(channel == 0)   return Outlet_Sensor_Range/1.6;
   else if(channel == 1)
   {
	  if((sensor_type == PM) || (sensor_type == PT)) return Entrance_Sensor_Range/1.6;
   }
   return (Entrance_sensor0[2] * 256 + Entrance_sensor0[3])/1000.0;

}


/***********************************************************************
*  Convert voltage into physical quantity: Temperature or Pressure.
*
*  Example:  238(2.38V)->17.0(°C)
************************************************************************/
float phy_quantity_from_volt(int sensor_type, int int_vol, uint8 channel)
{
	float phy_val, y1, y2;
	int x1, x2;
	float k = 1.0;
	if((sensor_type == PM) || (sensor_type == PT))//not HM
	{	
		pressure_para_picker(int_vol, &y1, &y2, &x1, &x2, sensor_type);
		k = range_factor(channel, sensor_type);
		phy_val = (x1 - int_vol)*(y1 - y2)/(x2 - x1) + y1;	
		phy_val = phy_val * k; 
			
	}
	else if((sensor_type == HM_PM) || (sensor_type == HM_PT))	//WL
	{	
		wl_para_picker(int_vol, &y1, &y2, &x1, &x2, sensor_type);
	    k = range_factor(channel, sensor_type);
		phy_val = (x1 - int_vol)*(y1 - y2)/(x2 - x1) + y1;	
		phy_val = phy_val * k; 

	}
	else if(sensor_type == NTC)
	{	
		temp_para_picker(int_vol, &y1, &y2, &x1, &x2);
		phy_val = (x1 - int_vol)*(y1 - y2)/(x2 - x1) + y1;	
	}	
#ifdef DBG_MEASURE
		printf("K=%f----------------phy_value =%f, \n", k, phy_val);
#endif		
	
	return phy_val;
}
/***********************************************************************
*  Convert float into string with specified number of decimal
*
*  Example: nrb_of_deicimal = 1,  25.83 --> "25.8" 
************************************************************************/
void data2string(float data, char* buff, int nrb_of_deicimal) 
{
	
	if(nrb_of_deicimal == 0)
	{
		sprintf(buff, "%d", (int)data);//convert a float number into a string and keep two decimals: 25.856->26
	}
	else if(nrb_of_deicimal == 1)
	{
		sprintf(buff, "%.1f", data);//convert a float number into a string and keep two decimals: 25.856->25.9
	}
	else if(nrb_of_deicimal == 2)
	{
		sprintf(buff, "%.2f", data);//convert a float number into a string and keep two decimals: 25.856->25.86
	}

}

/**********************************************************************************************
 * Name：GetVolt(uint16 advalue)
 * Brief：Convert output value from ADC(0~4095) into real voltage(0~3.3)
 * For convenience the voltage number is multiplied by 100  
 * Para：   null	
 * Return ：null
 * Example:  ADC output = 2048 -> result = 165.  that stands for "voltage = 1.65v"
 * Caller(s):  RealtimeMeasurement()
 **********************************************************************************************/
u16 GetVolt(uint16 advalue)
{
	return (uint16)(advalue * 330 / 4096); //求的结果扩大了100倍，方便下面求出小数
}


/**********************************************************************************************
 * Name： filter()
 * Brief：Calculate the average value of N ADC readings for each channel(in AD_Value[N][M]) and put 
 *        them in After_filter[]         
 * Para：   null	
 * Return ：null
 * Example:  N = 4, M = 3	   CH0    CH1    CH2
 *			AD_Value[N][M] = { 
 *			                   1000,  3000,   830,			 			
 *							   800,   3000,   810,	
 *							   1200,  3200,   780,	
 *							   1200,  2800,   820   }	
 *                            ----------------------
 *		    After_filter[] =  {1050,  3000,   810   }
*
 * Caller(s):  RealtimeMeasurement()
 **********************************************************************************************/
void filter(void)
{
	int sum = 0;
	u8 count, i;
	for(i = 0; i < M; i++)	 //for each channel	(column)
	{	
		for ( count=0; count < N; count++)	//for each reading(row)	of this channel
		{		
			sum += AD_Value[count][i];		
		}
		
		After_filter[i]=sum/N;		
		sum=0;
	}
}



/**********************************************************************************************
 * Name： update_variables
 * Brief：Put the physical value to the relevant location of array phy_quantities[]
 *           
 * Para：   phy_value	
 * Return ：null
 * Caller(s):   main.c/main()
 **********************************************************************************************/
void update_variables(int i, float phy_value)
{
 	phy_quantities[i] =  phy_value; 

#ifdef DBG_MEASURE
    printf("phy_quantities[%d] = %f\n", 	i, phy_quantities[i]);
#endif

}

uint8 get_ss_type(uint8 channel)
{
	if(channel == 0)//outlet
	{
	   if(Outlet_Sensor_type == OUTLET_SENSOER_TYPE_1) return PM;
	   if(Outlet_Sensor_type == OUTLET_SENSOER_TYPE_2) return PT;
	   return PM;
	}
	else if(channel == 1)//entrance
	{
		if(((Entrance_sensor0[0] & 0x0f) == ON) && (WaterSourceMode == TANK_MODE)) return WL_SWITCH;
		if((Entrance_sensor0[1] == OFF) && (WaterSourceMode == TANK_MODE)) return HM_PM;
		if((Entrance_Sensor_type == ENTRANCE_SENSOR_TYPE_1) && (WaterSourceMode == NONNEGTIVE_MODE)) return PM;
		if((Entrance_sensor0[1] == ON) && (WaterSourceMode == TANK_MODE)) return HM_PT;
		if((Entrance_Sensor_type == ENTRANCE_SENSOR_TYPE_2) && (WaterSourceMode == NONNEGTIVE_MODE)) return PT;
		return PM;
	}
	return NTC;
}

float get_bias(uint8 i)
{
	if(i == 1)		//entrance
	{
		if(WaterSourceMode == TANK_MODE) return phy_quantities_bias[IND_CH_WATER_LEVEL];   //WL
		if(WaterSourceMode == NONNEGTIVE_MODE) return phy_quantities_bias[IND_CH_ENTRANCE_PRESSURE];   //entrance pressure
	}
	if(i == 0) 	return phy_quantities_bias[IND_CH_OUTLET_PRESSURE];
	if(i > 1)	return phy_quantities_bias[i + 1];
	return 0;//In theory this is not reachable
}

//#define DBG_SHOW_ADC_BUFFER
#ifdef DBG_SHOW_ADC_BUFFER
void show_adc_buffer(void)
{
	uint8 i, j;
	printf("\n\n****AD_Value[i][j]****\n   Outlet      Entrance\n    ");	
	for(i = 0; i < N; i++)
	{
		for(j = 0; j < M; j++)
		{
			printf("%d          ", AD_Value[i][j]);	
		}
		printf("\n    ");
	}
}
#endif	

uint8 locker_watchdog(uint8 in_range)
{
	static counter = 0;
//	printf("~~~TargetLockCounter = %d\n", counter);
	if(in_range == TRUE) 
	{
		if(counter < TargetLockerTiming)
		{
			counter++;
			if(counter >= TargetLockerTiming)  return TRUE;
	
		}
	}
	else if(counter > 0)
	{
		counter = 0;
	}
	return FALSE;

}

void target_locker_manager(void)
{
	uint8 in_range;
	uint8 lock_event;

	real_outlet_pressure = OutletRealTimePressure;

	in_range = PressureInTargetRange();
	if(TARGET_CLOCKER_ENABLED)
	{
		lock_event = locker_watchdog(in_range);
	
		//TargetLockConfirmed 
		if(!TargetLockConfirmed)
		{
			if(lock_event == TRUE)
			{
				TargetLockConfirmed = true;
				printf("TargetLockConfirmed!\n");
			}
		}
		else if(in_range == FALSE)
		{
			TargetLockConfirmed = false;
			printf("Target UnLocked!\n");
		}
	}
	//OutletRealTimePressure
	if(in_range == TRUE) OutletRealTimePressure = Tartget_Pressure; 

}
extern uint8 AvfMode;
extern float OutletPressureCaliCoeff;	
extern float EntrancePressureCaliCoeff;
extern float DefaultTargetPressure;
/**********************************************************************************************
 * Name： RealtimeMeasurement()
 * Brief：Start ADC, read the result, convert it into physical value(temperature or pressure),
 *        update the relevant variable(stored in phy_quantities[]), and send to HMI for updating.
 *           
 * Para：   null	
 * Return ：null
 * Chart:   Measurement.ddd
 * Calling period: 1s
 * Caller(s):   main.c/main()
 **********************************************************************************************/

void RealtimeMeasurement()
{
	static uint8 measure_ready = FALSE;	   //Discard the 1st result
	char buff[5];
	uint8 ss_type, i;
	float phy_value;
	uint16 value[M];

#ifdef USE_CURVES
	char str[10];
	uint8 curve_data_byte[1];
	uint8 curve_target_data_byte[1];
	uint8 freq_data_byte[1];
#endif

#ifdef DBG_FUNC_LEN
	uint32 cnt = timer_tick_count;
#endif

//*****************************UNIT TEST HEADER*****************************************************
#ifdef DBG_FUNC_INFO
    func_dbg_info = OFF;
    if(func_dbg_info) printf("\n\n\nEntrance: void RealtimeMeasurement()...............\n\n");
#endif  
//--------------------------------------------------------------------------------------------------

	
 	ADC_SoftwareStartConvCmd(ADC1, ENABLE);	
	
	DMA_Cmd(DMA1_Channel1, ENABLE); //Start DMA channel
			
 	//wait for output of the most recent regular ADC, the result is put in AD_Value[N][M]
 	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));

#ifdef DBG_SHOW_ADC_BUFFER
	if(WarmUpDone) show_adc_buffer();
#endif

	filter();

	for(i = 0; i < M; i++)	  //for each of M channels
	{
		value[i] = GetVolt(After_filter[i]);
		if (i == M - 1)
		{
			phy_value = value[i] / 300.0 * Outlet_Sensor_Range;
			if (phy_value > Outlet_Sensor_Range) phy_value = Outlet_Sensor_Range;
		}
		else
		{

#ifdef DBG_FUNC_INFO
    		if(func_dbg_info) printf("  【Channel%d: %s】\n    voltage: %0.2fv\n", i, (i == 0)?"outlet":"entrance", value[i] / 100.0);
#endif 

			ss_type = get_ss_type(i);
#ifdef DBG_FUNC_INFO
	    	if(func_dbg_info) printf("    sensor_type =%d, %s\n", ss_type,\
			 (ss_type == PM)?"PM":(ss_type == PT)?"PT":(ss_type == HM_PT)?"HM_PT":"HM_PM");
#endif 

			phy_value = phy_quantity_from_volt(ss_type, value[i], i);
		    phy_value += get_bias(i);

#ifdef DBG_FUNC_INFO
	    	if(func_dbg_info) 
			{
				printf("    ---phy_value =%0.2f(%s)", phy_value,\
				                             ((WaterSourceMode == TANK_MODE) && (i == 1))?"mm":"Mpa");
				if(i == 1)	//entrance
				{
					if((WaterSourceMode == TANK_MODE) && (ss_type == WL_SWITCH)) printf(", FLOATING BALL, DISCARDED...");
					else if(Entrance_Sensor_type == ENTRANCE_SENSOR_TYPE_3) printf(", BTNE_JOINT_PM, DISCARDED...");
		
				}
				printf("\n\n\n");
			}
#endif 

			update_variables(i, phy_value);
		}

		if(i == 0)	 //	 OutletRealTimePressure
		{
			OutletRealTimePressure =  phy_value;

			OutletRealTimePressure *= OutletPressureCaliCoeff;
			OutletRealTimePressureForCali = OutletRealTimePressure;
			data2string(OutletRealTimePressure, buff, 2);
			SetTextValue(SCREEN_OUTLET_CALI, TXT_OUTLET_PRESSURE_CALI_REALTIME, (uchar*)buff); 

			if(OutletRealTimePressure > Outlet_Sensor_Range)  OutletRealTimePressure = Outlet_Sensor_Range;
			if(OutletRealTimePressure < 0)  OutletRealTimePressure = 0.0;


			target_locker_manager();

			data2string(real_outlet_pressure, buff, 2);

			if(Manual_Setting == TRUE)
			{
			   if (AvfMode) 
			   		SetTextValue(SCREEN_MANUAL_CTR_AVF, TXT_CURRENBT_PRESSURE, (uchar*)buff); 
			   else
			   		SetTextValue(SCREEN_MANUAL_CTR_2, TXT_CURRENBT_PRESSURE, (uchar*)buff); 
			}
			else
			{
				SetTextValue(SCREEN_MAIN_1,TXT_PM_PRESSURE, (uchar*)buff); 
			}
#ifdef USE_CURVES				
		    //add to curve
			if(SysRunning == TRUE)
			{
				curve_data_byte[0] = (int)(OutletRealTimePressure * 100);
				curve_target_data_byte[0] = (int)(Tartget_Pressure * 100);
				freq_data_byte[0] = GetFrequencyDataForCurve();	  

				delay_ms(100);
				GraphChannelDataAdd(SCREEN_CONTROL_CURVES, 1, 0,curve_data_byte, 1);
				delay_ms(100);
				
				GraphChannelDataAdd(SCREEN_CONTROL_CURVES, 1, 1,curve_target_data_byte, 1);
				delay_ms(100);
				
				
				GraphChannelDataAdd(SCREEN_CONTROL_CURVES, 1, 2, freq_data_byte, 1);


				delay_ms(100);

				if((OutletRealTimePressure > Tartget_Pressure) && (up_crossed == FALSE)) //up-cross occurs
				{
					up_crossed = TRUE;
					max_curve_pressure = OutletRealTimePressure;
					min_curve_pressure = Tartget_Pressure;
				}
				if(up_crossed == TRUE)
				{
					if(OutletRealTimePressure > max_curve_pressure)	max_curve_pressure = OutletRealTimePressure;
					if(OutletRealTimePressure < min_curve_pressure)	min_curve_pressure = OutletRealTimePressure;

					printf("max_curve_pressure = %.2f\n",max_curve_pressure);
					printf("min_curve_pressure = %.2f\n",min_curve_pressure);
				    printf("volatility = %.1f%%\n",(max_curve_pressure - min_curve_pressure)/Tartget_Pressure*100.0);

					data2string((max_curve_pressure - min_curve_pressure)/Tartget_Pressure * 100.0, str, 1);
					SetTextValue(SCREEN_CONTROL_CURVES, 23, (uchar*)str);
					delay_ms(100);

				}
				data2string(OutletRealTimePressure, str, 2);
				SetTextValue(SCREEN_CONTROL_CURVES, 12, (uchar*)str);

				delay_ms(100);
				data2string(Tartget_Pressure, str, 2);
				SetTextValue(SCREEN_CONTROL_CURVES, 13, (uchar*)str);

			}
			else if(ManualFcRunning() == TRUE)
			{
				curve_data_byte[0] = (int)(OutletRealTimePressure * 100);
			
				freq_data_byte[0] = (int)(manual_freq_setting * 2);

				delay_ms(100);
				GraphChannelDataAdd(SCREEN_CONTROL_CURVES, 1, 0,curve_data_byte, 1);
				delay_ms(100);
							
				GraphChannelDataAdd(SCREEN_CONTROL_CURVES, 1, 2, freq_data_byte, 1);

				delay_ms(100);			
			
			
			}	
#endif
		}
		else if(i == 1)	  //ADC10-入口压力/水箱液面
		{
			if(WaterSourceMode == TANK_MODE)
			{
				if(ss_type != WL_SWITCH)	//not floating ball
				{
					int max_wl = Entrance_sensor0[2] * 256 + Entrance_sensor0[3];
					WaterLevel = phy_value;	
					if(WaterLevel > max_wl)  WaterLevel = max_wl;
					if(WaterLevel < 0)	WaterLevel = 0;

					
					push_cmd_pump(CMD_SET_TEXT, SCREEN_MAIN_1, TXT_WATER_LEVEL, (int)WaterLevel);

					if (AvfMode) 
						push_cmd_pump(CMD_SET_TEXT, SCREEN_MANUAL_CTR_AVF, TXT_TANK_WATER_LEVEL, (int)WaterLevel);
					else
						push_cmd_pump(CMD_SET_TEXT, SCREEN_MANUAL_CTR_2, TXT_TANK_WATER_LEVEL, (int)WaterLevel);
			
					if((int)(phy_value * 95 /(Entrance_sensor0[6] * 256 + Entrance_sensor0[7]) > 95))
					{
						push_cmd_pump(CMD_SET_PROGRESS, SCREEN_MAIN_1, 55, 95);

					}
					else
					{
						push_cmd_pump(CMD_SET_PROGRESS, SCREEN_MAIN_1, 55, (int)(phy_value * 95 /(Entrance_sensor0[6] * 256 + Entrance_sensor0[7])));
					}                                                   

				}
				else	 //not floating ball
				{
					WaterLevel = 0.0;
				}
			}
			else	//non-negative	 Entrance_Sensor_Range
			{
				if(Entrance_Sensor_type == ENTRANCE_SENSOR_TYPE_3)// BTNE_JOINT_PM
				{
					EntranceRealTimePressure = 0.0;
				}
				else
				{
					EntranceRealTimePressure =  phy_value;

					EntranceRealTimePressure *= EntrancePressureCaliCoeff;
					EntranceRealTimePressureForCali = EntranceRealTimePressure;		
		
					data2string(EntranceRealTimePressureForCali, buff, 2);
					SetTextValue(SCREEN_ENTRANCE_CALI, TXT_ENTRANCE_PRESSURE_CALI_REALTIME, (uchar*)buff); 


					if(EntranceRealTimePressure > Entrance_Sensor_Range)  EntranceRealTimePressure = Entrance_Sensor_Range;
					if(EntranceRealTimePressure < 0)  EntranceRealTimePressure = 0.0;

					data2string(EntranceRealTimePressure, buff, 2);
					SetTextValue(SCREEN_MAIN_1, TXT_ENTRANCE_PRESSURE, (uchar*)buff);
					delay_ms(100);
					SetTextValue(SCREEN_MANUAL_CTR_2, TXT_MANUAL_ENTRANCE_PRESSURE, (uchar*)buff); 	
				}
			}		
		}
		else if(i == 2)		  //    pump temp
		{
			pump_temp[0] = phy_value;
			data2string(phy_value, buff, 1);
			SetTextValue(SCREEN_PUMP_TEM_CONTROL, 1, (uchar*)buff);

			if(TempMonitorEnable)
			{
				strcat(buff, "°C");
				SetTextValue(SCREEN_MAIN_1, 48, (uchar*)buff);
				if(phy_value > temp_limit[0])  SetControlForeColor(SCREEN_MAIN_1, 48, 0xf800);   
				else SetControlForeColor(SCREEN_MAIN_1, 48, 0xffffff); 
			}
			else
			{
				SetTextValue(SCREEN_MAIN_1, 48, "");
			}
		}
		else if(i == 3)	 //IO Target Pressure Setting	  20210702
		{
			if (UseIoTargetPressure)
			{
				DefaultTargetPressureIO = phy_value;

				DefaultTargetPressure = DefaultTargetPressureIO;  

				data2string(DefaultTargetPressureIO, buff, 2);	  //x.yy

				SetTextValue(SCREEN_TIME_CTR_3, TXT_TC_DEFAULT_PRESSURE, buff);

				PidSetTargetPressure(DefaultTargetPressure);		  //20210703
			}
		}
	}
	/*Power-up ---> 
	                【RealtimeMeasurement()】 ---> 
				                  	           measure_ready = TRUE --->
									                                     【RealtimeMeasurement()】 ---> 
													                                                 WarmUpDone = TRUE*/

	//Warm up Flag.  After 2 measurements from power-up, WarmUpDone = TRUE, then check expire date. 		 
	 if(measure_ready) 
	 {
	 	if(WarmUpDone == FALSE) 
		{
	
#ifdef DBG_RTC	
	        printf("\n        WarmUpDone, time is now: %d-%d-%d\n",now_year,  now_month,  now_day);
#endif
			ExpireDateManager();
			WarmUpDone = TRUE;
		    SetScreen(SCREEN_MAIN_1); 		  //20210604
		}
	 }
	 else 
	 {
	 	measure_ready = TRUE;
  	 }
	
#ifdef DBG_FUNC_INFO
     func_dbg_info = OFF;
     if(func_dbg_info) printf("\n\n\n Exit: void RealtimeMeasurement()...............\n\n");
#endif  
#ifdef DBG_FUNC_LEN
	 printf("LEN(RealtimeMeasurement()):%d\n", timer_tick_count - cnt);
#endif

#ifdef DBG_WDG_TEST
 	 if(DogFood == OFF)  
	 {
		 while(1);
	 }

#endif

}
