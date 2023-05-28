
/*--------------------------------------------------------
				MCU:          STM32F103VET
				Package Type: LQFP100
  -------------------------------------------------------*/
/*
	MODIFICATIONS:
	---nbr of errors definination
	---pump err
	---pump temperature
	---pump grouping
	---pump adding/cancelling


*/

//Obj File: DC_CPWS_7B-Ver-1.3.4.hex

/*------------------------------Path---------------------------------- 

D:\Embedded\Projects\YJS\CPWS\Projects\TouchScreenWS_AVF\Source Code Prj\OUTPUT

*/
  /*--------------------------------------------------------------------------------------------------------------------*                                                      
   * @RevisionHistory 
   *
   *         Version        	 Date Code	   		      Files               Remark
   *        --1.3.4              20220623            pump_running.c         modify bottom of pid freq.
   *        --1.3.3              20220513            UserInterface.c        Update frequency of pump0-5
   *        --1.3.2              20220513            UserInterface.c        Update frequency of pump0-5
   *	    --1.3.1              20220325
   *        --1.3.0              20220319
   *        --1.2.9              20210911
   *        --1.2.8              20210903
   *        --1.2.7              20210805
   *        --1.2.5              20210710
   *        --1.2.3              20210707
   *        --1.2.1              20210703
   *        --1.1.9              20210702
   *        --1.1.7              20210625 
   *        --1.1.1              20210624 
   *        --1.0.9              20210623 
   *        --1.0.8              20210623 
   *        --1.0.7              20210622  
   *        --1.0.6              20210618                                                                                                   *
   *        --1.0.5              20210604                                                                               *			   
   *--------------------------------------------------------------------------------------------------------------------*/

#ifndef __MACROS_H
	#define __MACROS_H

//	#define   ENGLISH_VERSION                   //If this macro is enabled, English	versioned UI will take effect

	#define   USE_PUMP_ADDING_WAITING_UINIT		//If this macro is defined an extra waiting time segment 
											    //is added after frequency reaches 50hz in case specified  
												//pump-adding time point is not reached yet.

	#define   USE_NARROW_PID_ZONE	        	//If this macro is defined the range of PID control is 20%, otherwise it is 50%

	#ifdef ENGLISH_VERSION
	   	#define   VERSION_NO              "Ver-1.0.0E"	//  "Ver-1.4.2E"
    #else
		#define   VERSION_NO              "Ver-1.3.4"
	#endif

	//Baudrate: 115200
//*********************************************************************************
//      #define     DBG_MASTER_SWITCH	//！！！Enabled for Debugging Info Output to USB-Comm. Must be disabled for final release
//---------------------------------------------------------------------------------
/*	#ifdef DBG_MASTER_SWITCH
		#define NBR_OF_MUNITES_IN_AN_HOUR	  1
	#else
		#define NBR_OF_MUNITES_IN_AN_HOUR	  60
	#endif*/

	#define   USE_BATCH_READ			  //20210618

	#ifdef     DBG_MASTER_SWITCH
	//	#define   DBG_TIMING_CONSULTANT  	  //DBG ONLY!!!
			#ifdef DBG_TIMING_CONSULTANT
		
				#define CMD_COUNTING       0
				#define CMD_CHK_TIME       1
				#define CMD_RESET_TIME     2
	
				#define DBG_USE_SECOND_AS_MINUTE
			#endif
	#endif

//	#define   DBG_VIRTUAL_ERR			  //DBG ONLY!!!

//	#define   DBG RESET_MSG

//	#define   DBG HYBRID_UART1_TXT           //output both dbg info and modbus data

//	#define   DBG_FUNC_FOOTPRINT
//     #define DBG_ARTIFICIAL_WDG			//man-made deadlock to provoke IWDG for a reset


//	#define   DBG_WDG_SHANDOW
//	#define   FIXED_DEVICE_ADDRESS

//	#define   USE_NEW_API_PROTOCOL

	
//	#pragma warning(disable: 181)

    #define    TRUE                                    1
    #define    FALSE                                   0

    #define    ON                                      1
    #define    OFF                                     0

    #define    BUTTON_STATUS_ERROR					   2
	#define    BUTTON_STATUS_NUL					   3
	#define    BUTTON_STATUS_MANUAL_DISABLED		   4
	#define    BUTTON_STATUS_MANUAL_ENABLED	    	   5
	#define    BUTTON_STATUS_LOCK_ALL				   6

	#define    CMD_USABILTY_SET                        0
	#define    CMD_USABILTY_RECOVER                    1

    #define    EFFECTIVE                               1
    #define    INACTIVE                                0
	#define    CONSUMED                                0

//-------------------------------------------------------------
    #define    AUTHORITY_ADMIN                         OFF               //ON: Factory version  OFF: User version


    #define    SELECTED                                1
    #define    UNSELECTED                              0

	#define    VISIBLE                                 1
	#define    INVISIBLE                               0
//--------------------------------------------------------------
typedef struct{
	unsigned char txt_pump_mode;
    unsigned char icon_pump_fc;
    unsigned char icon_pump_pf;
	unsigned char icon_pump_flow;
    unsigned char icon_pump_mask;
    unsigned char icon_temp_mask;
    unsigned char icon_freq;
    unsigned char icon_err;
   
     
} TYPE_PUMP_ICONS;

 //
typedef struct{
    unsigned char freq;
    unsigned char pump_ind;    
} TYPE_FATIGUE_PUMP_CANCEL;

#define  BLOCKED_BY_FATIGUE_SWITCH(i)  (FatiguePumpCancel.pump_ind == i)
#define  UNBLOCK_FATIGUE_SWITCH         FatiguePumpCancel.pump_ind = 255; FatiguePumpCancel.freq = 0; 
#define  START_FATIGUE_CANCEL_AVF(i)    FatiguePumpCancel.pump_ind = i; FatiguePumpCancel.freq = 50; 
//--------------------------------------------------------------
	 //FLASH ADDRESS: 0X8000000~0X807FFFF
	#define    USE_PIN_GENERATOR

	#define STM32_FLASH_SAVE_ADDR  				0X08070000		//设置FLASH 保存地址(必须为偶数，且其值要大于本代码所占用FLASH的大小+0X08000000)
	#define STM32_FLASH_SAVE_ADDR_TRIAL_DATE    0X08070100	
	#define STM32_FLASH_SAVE_ADDR_PW_SWITCH     0X08070200	
/*	
#ifdef    USE_PIN_GENERATOR
	#define STM32_FLASH_SAVE_ADDR_PW_ERR_IND    0X08070202		
#endif */
	#define STM32_FLASH_SAVE_ADDR_OUTLET_PRESSURE_BIAS     0X08070204	
	#define STM32_FLASH_SAVE_ADDR_ENTRANCE_PRESSURE_BIAS   0X08070206
	#define STM32_FLASH_SAVE_ADDR_WATER_LEVEL_BIAS         0X08070208	
	#define STM32_FLASH_SAVE_ADDR_ENTRANCE2_PRESSURE_BIAS  0X0807020A
	#define STM32_FLASH_SAVE_ADDR_FC_FRQ_OUT_BIAS          0X0807020C	
	#define STM32_FLASH_SAVE_ADDR_PID_FACTORS              0X0807020E
//	#define STM32_FLASH_SAVE_ADDR_WEBSITE                  0X08070214


//	#define STM32_FLASH_SAVE_ADDR_POWER_SAVING             0X08070214

	#define STM32_FLASH_SAVE_ADDR_POWER_SAVING             0X08071000 + 126

	#define STM32_FLASH_SAVE_ADDR_ALL_SETTINGS             0X08071000
	#define STM32_FLASH_SAVE_ADDR_USER_SETTINGS            0X08071000
	#define STM32_FLASH_USER_INIT_DATA_LEN                 178
	#define STM32_FLASH_SAVE_ADDR_USER_TC_INTERNAL         128
	#define STM32_FLASH_SAVE_ADDR_USER_TC                  0X08071000 + 128
	#define STM32_FLASH_USER_DATA_TC_LEN                   46

	#define STM32_FLASH_SAVE_ADDR_USER_SCREEN_PUMP_GROUPING_INTERNAL    0
	#define STM32_FLASH_USER_DATA_SCREEN_PUMP_GROUPING_LEN    8
	
	#define STM32_FLASH_SAVE_ADDR_USER_SCREEN_OUTLET_SENSOR_INTERNAL    8
	#define STM32_FLASH_USER_DATA_SCREEN_OUTLET_SENSOR_LEN    10
	
	#define STM32_FLASH_SAVE_ADDR_USER_SCREEN_ENTRANCE_SENSOR_INTERNAL    18
	#define STM32_FLASH_USER_DATA_SCREEN_ENTRANCE_SENSOR_LEN    10
	
	#define STM32_FLASH_SAVE_ADDR_USER_SCREEN_ENTRANCE_SENSOR0_INTERNAL    28
	#define STM32_FLASH_USER_DATA_SCREEN_ENTRANCE_SENSOR0_LEN    12
	
	#define STM32_FLASH_SAVE_ADDR_USER_SCREEN_ENTRANCE_SENSOR2_INTERNAL    40
	#define STM32_FLASH_USER_DATA_SCREEN_ENTRANCE_SENSOR2_LEN    4
	
	#define STM32_FLASH_SAVE_ADDR_USER_SCREEN_PUMP_SWITCH_CONDTION_INTERNAL    44
	#define STM32_FLASH_USER_DATA_SCREEN_PUMP_SWITCH_CONDTION_LEN    16
	
	#define STM32_FLASH_SAVE_ADDR_USER_SCREEN_SLEEP_SETTING_INTERNAL    60
	#define STM32_FLASH_USER_DATA_SCREEN_SLEEP_SETTING_LEN    10
	
	#define STM32_FLASH_SAVE_ADDR_USER_SCREEN_POWER_UP_SETTING_INTERNAL    70
	#define STM32_FLASH_USER_DATA_SCREEN_POWER_UP_SETTING_LEN    12
	
	#define STM32_FLASH_SAVE_ADDR_USER_SCREEN_VALVE_CONTROL_INTERNAL    82
	#define STM32_FLASH_USER_DATA_SCREEN_VALVE_CONTROL_LEN    8
	
	#define STM32_FLASH_SAVE_ADDR_USER_SCREEN_VALVE_CONTROL1_INTERNAL    90
	#define STM32_FLASH_USER_DATA_SCREEN_VALVE_CONTROL1_LEN    6
	
	#define STM32_FLASH_SAVE_ADDR_USER_SCREEN_PID_SETTING_INTERNAL    96
	#define STM32_FLASH_USER_DATA_SCREEN_PID_SETTING_LEN    8

	#define STM32_FLASH_SAVE_ADDR_USER_SCREEN_PHONE_NBR_INTERNAL    104
	#define STM32_FLASH_SAVE_ADDR_USER_SCREEN_EXPIRE_DATE_INTERNAL    118
	#define STM32_FLASH_SAVE_ADDR_USER_SCREEN_PW_SWITCH_INTERNAL    124
	#define STM32_FLASH_SAVE_ADDR_USER_SCREEN_POWER_SAVING_INTERNAL    126

	#define STM32_FLASH_SAVE_ADDR_BKG_DATA_START           0X08071000 + 174
	#define STM32_FLASH_SAVE_BKG_DATA_LEN                  12


	#define STM32_FLASH_SAVE_ADDR_PIN_USER           	   0X08071000 + 174
	#define STM32_FLASH_SAVE_ADDR_PIN_FACTORY        	   0X08071000 + 178

#ifdef    USE_PIN_GENERATOR
	#define STM32_FLASH_SAVE_ADDR_PW_ERR_IND               0X08071000 + 182		
#endif

	#define STM32_FLASH_SAVE_ADDR_INFO_LOCKED        	   0X08071000 + 184


	#define STM32_FLASH_SAVE_CHKSUM_ADDR                   STM32_FLASH_SAVE_ADDR_BKG_DATA_START

	#define STM32_FLASH_SAVE_ERROR_RECORD       	       0X08071000 + 186
																 
	#define STM32_FLASH_SAVE_TARGET_LOCKER       	       0X08071000 + 800

	#define STM32_FLASH_SAVE_PUMP_TEMP       	           0X08071000 + 802		 //18bytes

//	#define STM32_FLASH_SAVE_PWR_UP_AUTORUN_DELAY_H	       0X08071000 + 820

	#define STM32_FLASH_SAVE_COMPANY_NAME      	           0X08071000 + 850		 //42bytes

	#define STM32_FLASH_SAVE_REMOTE_TARGET     	           0X08071000 + 892		 //2bytes

	#define STM32_FLASH_SAVE_TARGET_LOCKER_DELTA	       0X08071000 + 894		 //1byte 

	#define STM32_FLASH_SAVE_DEVICE_ADDRESS	               0X08071000 + 896		 //1byte 

	#define STM32_FLASH_SAVE_USE_60HZ                      0X08071000 + 898		 //1byte  

	#define STM32_FLASH_SAVE_AVF_MODE                      0X08071000 + 900		 //1byte  

	#define STM32_FLASH_SAVE_PUMP_CANCEL_FREQ              0X08071000 + 902		 //1byte

	#define STM32_FLASH_SAVE_OUTLET_PRESSURE_CALI_COEFF    0X08071000 + 904		 //1byte   

	#define STM32_FLASH_SAVE_ENTRANCE_PRESSURE_CALI_COEFF  0X08071000 + 906		 //1byte 

	#define STM32_FLASH_SAVE_REMOTE_TARGET_ENABLE          0X08071000 + 908		 //1byte 

	#define STM32_FLASH_SAVE_USE_IO_TARGET_PRESSURE        0X08071000 + 910		 //1byte
	
	#define STM32_FLASH_SAVE_SPEED_UP_FACTOR               0X08071000 + 912		 //1byte 20210712 

	#define STM32_FLASH_SAVE_PUMP_MAX_FREQ    			   0X08071000 + 914		 //2bytes 20210911 


	//ICONS/TXT
	#define STOP_WATERFLOW(i)				AnimationStop(SCREEN_MAIN_1,    PumpIcons[i].icon_pump_flow);
	#define START_WATERFLOW(i)				AnimationStart(SCREEN_MAIN_1,    PumpIcons[i].icon_pump_flow);		

	#define CLR_MODE_TXT(i)		  		    SetTextValue(SCREEN_MAIN_1, PumpIcons[i].txt_pump_mode, "");
	#define SHOW_VF_MODE_TXT(i)		  		    SetTextValue(SCREEN_MAIN_1, PumpIcons[i].txt_pump_mode, "变频");
	#define SHOW_PF_MODE_TXT(i)		  		    SetTextValue(SCREEN_MAIN_1, PumpIcons[i].txt_pump_mode, "工频");

	//Password 

	#define TXT_CURRENT_PIN			4

	//DBG
	#define DBG_DATA    			2

	//POWER UP
	#define POWER_UP_AUTORUN_DISABLE PowerUpAutoRunTimer = 0;

	//UI_CONTROLS
	#define           UI_CMD_START				0
	#define           UI_CMD_STOP				1
	#define           UI_CMD_DIS_MANUAL			2
	#define           UI_CMD_EN_MANUAL			3
	#define           UI_CMD_EXPIRE				4
	#define           UI_CMD_NON_EXPIRE			5
	#define           UI_CMD_ERR				6
	#define           UI_CMD_REM_ERR			7

	//SCREEN_MAIN_1

	#define    WL_DEFAULT_PERCENTAGE                   67
	#define    SCREEN_MAIN_WATER_TANK                  55
	#define    SCREEN_MAIN_NN_PRESSURE_TANK            36

	#define    SCREEN_MAIN_TXT_COMPANY_NAME			   9

	#define    SCREEN_MAIN_WL_UNIT                     10
	#define    SCREEN_MAIN_ICON_CITY                   11

	#define    BTN_MANUAL_SETTING                      16
	#define    BTN_PARAMETER_SETTING                   17
	#define    BTN_SYSTEM_SETTING                      19
	#define    BTN_ERROR_CHECKING                      18
    #define    BTN_START_AUTO_SCREEN_MAIN              14
    #define    BTN_STOP_AUTO_SCREEN_MAIN               15

    #define    ICON_STOP_MASK                          33
    #define    ICON_START_MASK                         34
    #define    ICON_MANUAL_MASK                        35
    #define    ICON_PARA_SETTING_MASK                  37
    #define    ICON_SYS_SETTING_MASK                   38


    #define    TXT_RUNNING_STATUS                      30
    #define    TXT_ENTRANCE_PRESSURE                   20
    #define    TXT_WATER_LEVEL                         20
    #define    TXT_PM_PRESSURE                         21
	#define    TXT_TARGET_PRESSURE					   22
	#define    TXT_CURRENT_FREQUENCY				   24
    #define    ICON_NON_NEG                            36

    #define    ICON_TIMER_ON                           42
    #define    ICON_PUMP_RUNNING                       53
    #define    ICON_WATER_RUNNING                      43

    #define    ICON_MANUAL_RUNNING                     52
    #define    ICON_ERRORS_EXIST                       13

	#define    TXT_PHONE_NUMBER_MAIN                   54
	#define    TXT_QR_CODE_MAIN                        99

    //SCREEN_STOP_CONFIRM1
    #define    BTN_STOP_CONFIRM                        1
    #define    BTN_STOP_CANCEL                         2


    //SCREEN_MANUAL_CTR_2
  #define    BTN_HEATING_1                           1
    #define    BTN_HEATING_2                           2
    #define    BTN_HEATING_3                           3
    #define    BTN_PUMP_1                              5
    #define    BTN_PUMP_2                              6
    #define    BTN_ERROR                               7
    #define    BTN_COOLANT_HIGH                        8
    #define    BTN_COOLANT_LOW                         9

    #define    IND_CIRCULATING_PUMP                    4
    #define    IND_MAKEUP_PUMP                         5
    #define    NBR_OF_MANUAL_SCREEN_CHANNEL            9	  

    //SCREEN_MANUAL_CTR_2
	#define    MANUAL_ICON_CITY                        33
	#define    MANUAL_ICON_MPA                         34

    #define    TXT_TANK_WATER_LEVEL                    32
    #define    TXT_MANUAL_ENTRANCE_PRESSURE            32

    #define    TXT_CURRENBT_PRESSURE                   35
    #define    TXT_RUNNING_FREQUENCY                   36
    //
    //
    //
    #define    BTN_BACK_SCREEN_MANUAL_CTR              40

	//SCREEN_USER_PIN_MANAGER
	#define    BTN_PW_SWITCH                           1



    //SCREEN_USER_SETTINGS_2
    #define    BTN_TEMP_SETTING                        9
    #define    BTN_TIME_CTR                            5
    #define    BTN_VALVE_SETTING                       7
    #define    BTN_PUMP_SETTING                        8
    #define    BTN_BACK_USER_SETTINGS                  14

    //SCREEN_SYS_SETTINGS_2
    #define    BTN_TIME_SETTING                        5
    #define    BTN_PIN_SETTING                         6
    #define    BTN_RESET_TO_FACTORY                    9

	//PHONE NUMBER
    #define    TXT_PHONE_NO                            2
	#define    TXT_COMPANY_NAME                        5

    //SCREEN_VALVE_SETTING_3 
    #define    BTN_VALVE_AUTO_MODE                     13
    #define    BTN_VALVE_MANUAL_MODE                   15
    #define    BTN_BACK_SCREEN_VALVE_SETTING           14
    #define    TXT_TARGET_TEMP                         12
    #define    TXT_VALVE_TIME                          16
    #define    TXT_MANUAL_VALVE_OPENING                17
    #define    TARGET_HEATING_TEMPERATURE_ROOF         100
    #define    TARGET_HEATING_TEMPERATURE_FLOOR        20
    #define    TXT_VALVE_TIME_WINDOW_L                 6
    #define    TXT_VALVE_TIME_WINDOW_R                 7

    #define    TXT_VALVE_TIME_WINDOW_L_NIGHT           8
    #define    TXT_VALVE_TIME_WINDOW_R_NIGHT           9
    #define    TXT_TARGET_COOLANT_TEMPERATURE          1
    #define    TXT_COOLANTTEMPVALVETIMING              4
    #define    TXT_COOLANTTEMPVALVEOPENING             5
    #define    BTN_COOLANT_VALVE_AUTO_MODE             2
    #define    BTN_COOLANT_VALVE_MANUAL_MODE           3


    //SCREEN_TIME_CTR_3
	#define    BTN_REMOTE_TARGET_ENABLE                40	   //20210618
    #define    BTN_USE_IO_TARGET_PRESSURE              42	   //20210702
    #define    TXT_TC_DEFAULT_PRESSURE                 54
    #define    ICON_TC_INPUT_ERROR                     41
    #define    TXT_TC_REMOTE_PRESSURE                  1
    #define    BTN_MONDAY_ENABLED                      3
    #define    BTN_SUNDAY_ENABLED                      9
    #define    BTN_BACK_SCREEN_TIME_CTR                2
    #define    BTN_FIRST_TASK_ENABLED                  10
    #define    BTN_LAST_TASK_ENABLED                   15

    //SCREEN_SCREEN_SETTING
    #define    BTN_BL_CONTROL_ENABLE                   1

    #define    TXT_BL_ON_LEVEL                         3
    #define    TXT_BL_OFF_LEVEL                        4

    #define    TXT_BL_ON_TIME                          2
    #define    BTN_BACK_SCREEN_SCREEN_SETTING          14

    //SCREEN_TEMP_SETTING_3
    #define    TXT_RETURN_WATER_FLOOR                  0
    #define    TXT_OUTLET_TEMPERATURE_LIMIT            1
    #define    TXT_TANK_TEMPERATURE_LIMIT              2
    #define    TXT_TANK_TEMPERATURE_FLOOR              3
    #define    TXT_TANK_TEMPERATURE_SUPERLOW           4
    #define    TXT_INDOOR_TEMP_1                       17
    #define    TXT_INDOOR_TEMP_2                       18
    #define    TXT_OUTDOOR_TEMP                        19
    #define    TXT_COOLANT_TEMP                        20
    #define    TXT_INDOOR_TEMP_1_ROOF                  5
    #define    TXT_INDOOR_TEMP_2_ROOF                  6
    #define    TXT_OUTDOOR_TEMP_ROOF                   7
    #define    TXT_COOLANT_TEMP_ROOF                   8
    #define    TXT_INDOOR_TEMP_1_FLOOR                 9
    #define    TXT_INDOOR_TEMP_2_FLOOR                 10
    #define    TXT_OUTDOOR_TEMP_FLOOR                  11
    #define    TXT_COOLANT_TEMP_FLOOR                  12
    #define    BTN_INDOOR_TEMP_1_SELECTED              22
    #define    BTN_INDOOR_TEMP_2_SELECTED              14
    #define    BTN_OUTDOOR_TEMP_SELECTED               15
    #define    BTN_COOLANT_TEMP_SELECTED               16
    #define    BTN_BACK_SCREEN_TEMP_SETTING            21
    #define    BTN_TEMP_RANGE0_SELECTED                13
    #define    BTN_TEMP_RANGE1_SELECTED                32


    #define    IND_OUTLETTEMPERATURELIMIT              0
    #define    IND_OUTLETTEMPERATUREFLOOR              1
    #define    IND_TANKTEMPERATURELIMIT                2
    #define    IND_TANKTEMPERATUREFLOOR                3
    #define    IND_TANKTEMPERATURESUPERLOW             4
    #define    IND_INDOORTEMP_1_ROOF                   5
    #define    IND_INDOORTEMP_2_ROOF                   6
    #define    IND_OUTDOORTEMP_ROOF                    7
    #define    IND_COOLANTTEMP_ROOF                    8
    #define    IND_INDOORTEMP_1_FLOOR                  9
    #define    IND_INDOORTEMP_2_FLOOR                  10
    #define    IND_OUTDOORTEMP_FLOOR                   11
    #define    IND_COOLANTTEMP_FLOOR                   12
    #define    IND_INDOORTEMP_1_SELECTED               13
    #define    IND_INDOORTEMP_2_SELECTED               14
    #define    IND_OUTDOORTEMP_SELECTED                15
    #define    IND_COOLANTTEMP_SELECTED                16

    #define    IND_OUTLETTEMPERATURE_BIAS              17
	#define    IND_RETURNWATER_BIAS                    18
    #define    IND_TANKTEMPERATURE_BIAS                19
    #define    IND_INDOORTEMP_1_BIAS                   20
    #define    IND_INDOORTEMP_2_BIAS                   21
    #define    IND_OUTDOORTEMP_BIAS                    22
    #define    IND_COOLANTTEMP_BIAS                    23
    #define    IND_HEATINGTEMP_BIAS                    24
    #define    IND_TANK_TEMP_SUPERLOW_SELECTED         25
    #define    IND_TEMP_RANGE_NBR                      26



    #define    TXT_RETURN_WATER_BIAS                   23
    #define    TXT_OUTLET_TEMPERATURE_BIAS             24
    #define    TXT_TANK_TEMPERATURE_BIAS               25
    #define    TXT_INDOOR_TEMP_1_BIAS                  26
    #define    TXT_INDOOR_TEMP_2_BIAS                  27
    #define    TXT_OUTDOOR_TEMP_BIAS                   28
    #define    TXT_COOLANT_TEMP_BIAS                   29
    #define    TXT_HEATING_TEMP_BIAS                    30
    #define    BTN_TANK_TEMP_SUPERLOW_SELECTED         31

    //SCREEN_PRESSURE_SETTING_3
    #define    BTN_BACK_SCREEN_PRESSURE_SETTING        3
    #define    TXT_PRESSURE_FULL_RANGE                 6
    #define    TXT_PRESSURE_BIAS                       7
    #define    TXT_PRESSURE_FLOOR                      5
    #define    TXT_PRESSURE_ROOF                       4
    #define    BTN_LOW_PRESSURE_PROTECTION_SELECTED    1
    #define    TXT_LOW_PRESSURE_PROTECTION_TIMING      2
    #define    BTN_PT_SELECTED                         8



    //SCREEN_GROUP_SETTING_3
    #define    TXT_NBR_OF_GROUP                        1
    #define    BTN_BACK_SCREEN_GROUP_SETTING_3         14
	#define    ICON_BLUE_MASK                          15

    //SCREEN_PUMP_SETTING_3
    #define    BTN_CONTINUOUS_MODE                     13
    #define    BTN_DELAYED_MODE                        15
    #define    BTN_CIRCULATING_PUMP1                   19
    #define    BTN_CIRCULATING_PUMP2                   20
    #define    BTN_MAKEUP_PUMP                         21
    #define    BTN_BACK_SCREEN_PUMP_SETTING            14
    #define    TXT_DELAY_TIME                          16
    #define    CONTINUOUS_MODE                         0
    #define    DELAYED_MODE                            1
    #define    DEFAULT_DELAY_S                         10

    //SCREEN_MAIN_ANIMATION
    #define    BTN_START_STOP_SCREEN_MAIN_ANIMATION    2
    #define    ICON_TIMER_ON_ANM                       29


    //SCREEN_FAILURE_INQUIRY_2
    #define    BTN_CLEAR_ERROR_INFO                    12

    #define    DIN_ERROR                               0
    #define    DIN_NORMAL                              1
   
    #define    TXT_ERROR_FREE                          11

    //SCREEN_PUMP_GROUPING
 	#define    ICON_TITLE_NN                           7

    #define    BTN_TANK_MODE                           1
    #define    BTN_NONNEGTIVE_MODE                     2
    #define    BTN_TANK_NONNEGTIVE_MODE                3
    #define    BTN_PUMP1_SELECTED                      4
    #define    BTN_PUMP2_SELECTED                      5
    #define    BTN_PUMP3_SELECTED                      6

    #define    BTN_SMALL_PUMP_SELECTED                 7
    #define    BTN_FIXED_FC_PUMP_MODE                  9
    #define    TXT_MAX_PUMP_NBR                        11
    #define    BTN_BACK_SCREEN_PUMP_GROUPING           10
    #define    ICON_INPUT_ERROR1                       8
    #define    ICON_INPUT_ERROR2                       12
    #define    ICON_INPUT_ERROR3                       13

    #define    TANK_MODE                               0
    #define    NONNEGTIVE_MODE                         1
    #define    TANK_NONNEGTIVE_MODE                    2

	#define ONLY_SMALL_PUMP_SELECTED				               ((nbr == 1) && (pump_enable_tbl[5] == 1))
	#define NUMBER_OF_SELECTED_MAIN_PUMPS	                        (nbr - pump_enable_tbl[5])

    //SCREEN_OUTLET_SENSOR

    #define    BTN_OUTLET_SENSOR_RANGE1                1


    #define    BTN_OUTLET_SENSOR_RANGE2                2


    #define    BTN_OUTLET_LOW_PRESSURE_PROTECTION      3


    #define    BTN_OUTLET_HIGH_PRESSURE_PROTECTION     4


    #define    BTN_BACK_SCREEN_OUTLET_SENSOR           9


    #define    TXT_OUTLET_SENSOR_RANGE                 5


    #define    TXT_OUTLET_PRESSURE_BIAS                6


    #define    TXT_OUTLET_LP_PROTECTION_VALUE          7


    #define    TXT_OUTLET_HP_PROTECTION_VALUE          8


    #define    OUTLET_SENSOER_TYPE_1                   1
    #define    OUTLET_SENSOER_TYPE_2                   2

   //SCREEN_OUTLET_PRESSURE_CALI
     #define    TXT_OUTLET_PRESSURE_CALI_REALTIME      1

     #define    TXT_OUTLET_PRESSURE_CALI_BIAS          2
	 #define    BTN_OUTLET_ZERO_CALI                   5

     #define    TXT_OUTLET_PRESSURE_CALI_COEFF         3
	 #define    BTN_OUTLET_FULL_RANGE_CALI             6

   //SCREEN_ENTRANCE_PRESSURE_CALI
     #define    TXT_ENTRANCE_PRESSURE_CALI_REALTIME      1

     #define    TXT_ENTRANCE_PRESSURE_CALI_BIAS          2
	 #define    BTN_ENTRANCE_ZERO_CALI                   5

     #define    TXT_ENTRANCE_PRESSURE_CALI_COEFF         3
	 #define    BTN_ENTRANCE_FULL_RANGE_CALI             6
    //SCREEN_ENTRANCE_SENSOR 

    #define    BTN_ENTRANCE_SENSOR_INPUT1_SELECTED     1


    #define    BTN_ENTRANCE_SENSOR_INPUT2_SELECTED     2


    #define    BTNE_JOINT_PM_SELECTED                  3


    #define    BTN_BACK_SCREEN_ENTRANCE_SENSOR         4


    #define    TXT_ENTRANCE_SENSOR_RANGE               5


    #define    TXT_ENTRANCE_PRESSURE_BIAS              6


    #define    TXT_ENTRANCE_LP_PROTECTION_DELAY        8


    #define    TXT_ENTRANCE_LP_PROTECTION_VALUE        7


    #define    TXT_ENTRANCE_LP_PROTECT_RESTORE_VAL     9

    #define    ENTRANCE_SENSOR_TYPE_1                  0
    #define    ENTRANCE_SENSOR_TYPE_2                  1
    #define    ENTRANCE_SENSOR_TYPE_3                  2

    //SCREEN_ENTRANCE_SENSOR0 

    #define    BTN_BACK_SCREEN_ENTRANCE_SENSOR0        9


    #define    IND_WATER_LEVEL_INDICATOR_ON            0


    #define    IND_INPUT0_RANGE1                       1


    #define    IND_INPUT0_RANGE2                       2


    #define    IND_ENTRANCE0_SENSOR_RANGE              3


    #define    IND_WATER_LEVEL_BIAS                    4


    #define    IND_WATER_LEVLE_FULL_HEIGHT             5


    #define    IND_LOW_WL_PROTECTION_VALUE             7


    #define    IND_LOW_WL_PROTECT_RESTORE_VAL          8



    //SCREEN_ENTRANCE_SENSOR2

    #define    BTN_BACK_SCREEN_ENTRANCE_SENSOR2        5


    #define    IND_INPUT2_RANGE1                       0


    #define    IND_INPUT2_RANGE2                       1


    #define    IND_ENTRANCE_SENSOR_RANGE               2


    #define    IND_ENTRANCE2_PRESSURE_BIAS             3


    //SCREEN_VALVE_CONTROL1

    #define    BTN_BACK_SCREEN_VALVE_CONTROL1          6


    #define    IND_ENTRANCE_LP_SWITCH                  0


    #define    IND_LP_SWITCH_DELAY                     1


    #define    IND_TANK_SWITCH_TIME                    2


    #define    IND_TANK_WORK_TIME                      3

    #define    IND_ENTRANCE_RESTORE_P                  4

    //SCREEN_PUMP_SWITCH_CONDTION

    #define    BTN_BACK_SCREEN_PUMP_SWITCH_CONDTION    11


    #define    TXT_FC_SPEEDUP_TIME                     1


    #define    TXT_FC_SLOWDOWN_TIME                    2


    #define    TXT_FC_START_STOP_FRQENCY               3


    #define    TXT_FC_PUMP_SWITCH_TIME                 4


    #define    TXT_STOP_PUMP_DELAY                     5


    #define    TXT_FC_PAUSE_TIME_AFTER_ADDING_PUMP     6


    #define    TXT_PF_PUMP_ADD_DELAY                   7


    #define    TXT_PF_PUMP_DEC_DELAY                   8


    #define    TXT_FC_TO_PF_DELAY                      9


    #define    TXT_ADD_PUMP_PRESSURE_BIAS              10

	#define    TXT_PUMP_CANCEL_FREQ                    12

 	#define    TXT_PUMP_MAX_FREQ                       13   //20210911

    #define    IND_FC_SPEEDUP_TIME                     0


    #define    IND_FC_SLOWDOWN_TIME                    1


    #define    IND_FC_START_STOP_FRQENCY               2


    #define    IND_FC_PUMP_SWITCH_TIME                 3


    #define    IND_STOP_PUMP_DELAY                     4


    #define    IND_FC_PAUSE_TIME_AFTER_ADDING_PUMP     5


    #define    IND_PF_PUMP_ADD_DELAY                   6


    #define    IND_PF_PUMP_DEC_DELAY                   7


    #define    IND_FC_TO_PF_DELAY                      8


    #define    IND_ADD_PUMP_PRESSURE_BIAS              9

    //SCREEN_SLEEP_SETTING

    #define    BTN_BACK_SCREEN_SLEEP_SETTING           8


    #define    IND_SLEEP_JUDGE_DELAY                   0


    #define    IND_MAIN_PUMP_SLEEP_JUDGE_FREQ          1


    #define    IND_MAIN_PUMP_WAKE_UP_BIAS              2


    #define    IND_MAIN_PUMP_WAKE_UP_DELAY             3


    #define    IND_SMALL_PUMP_SLEEP_JUDGE_FREQ         4


    #define    IND_SMALL_PUMP_WAKE_UP_BIAS             5


    #define    IND_SMALL_PUMP_WAKE_UP_DELAY            6


	#define    SLEEP_MODE_SLEEP_ENABLE                 0

	#define    SLEEP_MODE_SLEEP_DISABLE                1

	#define    SLEEP_MODE_ALWAYS_RUNNING               2

    //SCREEN_POWER_UP_SETTING

    #define    BTN_BACK_SCREEN_POWER_UP_SETTING        14

	#define    BTN_USE_60HZ_AS_MAX_FREQUENCY           15

	#define    BTN_AVF_MODE                            16


    //SCREEN_VALVE_CONTROL

    #define    BTN_BACK_SCREEN_VALVE_CONTROL           7


    #define    IND_USE_ENTRANCE_VALVE                  0


    #define    IND_USE_DECOMPRESSION_VALVE             1


    #define    IND_WATER_LEVEL_FLOOR                   2


    #define    IND_WATER_LEVEL_CEILING                 3


    #define    IND_PRESSURE_CEILING                    4


    #define    IND_PRESSURE_FLOOR                      5

    //PID_SETTING

    #define    BTN_BACK_PID_SETTING                    6

    #define    BTN__SELF_DEF_PID_PAR                   1




    #define    IND_SELF_DEF_PID_PAR                    0


    #define    IND_SAMPLING_PERIOD                     1


    #define    IND_P_FACTOR                            2


    #define    IND_I_FACTOR                            3


    #define    IND_D_FACTOR                            4

	#define    TXT_TARGET_LOCKER_TIMING                7

	#define    TXT_SPEED_UP_FACTOR                     8   //20210712

	#define    TXT_TARGET_LOCKER_DELTA                 9

//SCREEN_TRAIL_DATE_SETTING
	#define    TXT_YEAR                                3
	#define    TXT_MONTH                               4
	#define    TXT_DAY                                 5

    //SCREEN_PUMP_TEM_CONTROL

    #define    BTN_BACK_SCREEN_PUMP_TEM_CONTROL        25


    #define    TXT_PUMP1_RT_TEMP                       1


    #define    TXT_PUMP1_LIMIT_TEMP                    2


    #define    TXT_PUMP1_TEMP_BIAS                     3


    #define    TXT_PUMP2_RT_TEMP                       4


    #define    TXT_PUMP2_LIMIT_TEMP                    5


    #define    TXT_PUMP2_TEMP_BIAS                     6


    #define    TXT_PUMP3_RT_TEMP                       7


    #define    TXT_PUMP3_LIMIT_TEMP                    8


    #define    TXT_PUMP3_TEMP_BIAS                     9


    #define    TXT_PUMP4_RT_TEMP                       10


    #define    TXT_PUMP4_LIMIT_TEMP                    11


    #define    TXT_PUMP4_TEMP_BIAS                     12



	//NTC
	#define    NTC_Vol1                                35//73
	#define    NTC_Vol2                                282//205
	#define    NTC_Temp1                               120//85
	#define    NTC_Temp2                               0//27

	//PM  0 - 1.6mpa
	#define    PM_Vol1                                 58            //0.58
	#define    PM_Vol2                                 225			  //2.25v
	#define    PM_Temp1                                0.11			  //mpa
	#define    PM_Temp2                                1.6			 //mpa
	//PT  0 - 1.6mpa
	#define    PT_Vol1                                 38			 //0.38
	#define    PT_Vol2                                 175			 //1.75
	#define    PT_Temp1                                0.28			 //mpa
	#define    PT_Temp2                                1.6			 //mpa

    //Coolant Monitor
    #define    IND_COOLANT_PROTECT_HIGH                9
    #define    IND_COOLANT_PROTECT_LOW                 8

    //ValveMaster
    #define    CH_VALVE_HEATING                        0
    #define    CH_VALVE_COOLANT                        1
 

	//Error Reporter
	#define    ADD_ERROR_YES                           1
	#define    ADD_ERROR_NO                            0


    //FLASH_SETTINGS

	#define    CHKSUM_ADDR                             152
    #define    CHKSUM_LEN                              1
    #define    PIN_USER_START_ADDR                     153
    #define    PIN_LEN                                 3
    #define    PIN_FACTORY_START_ADDR                  156
    #define    BKG_DATA_START_ADDR                     152
    #define    BKG_DATA_LEN                            13
    #define    FRG_DATA_LEN                            152
    #define    SCREEN_TEMP_SETTING_3_LEN               9
    #define    MILEAGE_START_ADDR                      161
    #define    MILEAGE_LEN                             3
    #define    MILEAGE_SET_START_ADDR                  159
    #define    MILEAGE_SET_LEN                         5
    #define    USER_INIT_DATA_LEN                      156
    #define    TOTAL_DATA_LEN                          165
    #define    MILEAGE_STATUS_GREEN                    0
    #define    MILEAGE_STATUS_RED                      1
    #define    FACTORY_INFO_LOCKED_START_ADDR          164
    #define    FACTORY_INFO_LOCKED_LEN                 1
    #define    SCREEN_TIME_CTR_3_START_ADDR            106


    //SCREEN_PIN_CONTROL
    #define    TXT_PIN_INPUT                           1

    //SCREEN_PIN_SETTING_3
    #define    TXT_NEW_PIN_INPUT                       1
    #define    TXT_USER_PIN_REMINDER                   2
	#define    BTN_CANCEL_PIN  						   14

    //PIN_SETTING
    //PIN input check result
    #define    PIN_INPUT                               0
    #define    PIN_REPEAT                              1
    #define    PIN_WRONG_LEN                           2
    #define    PIN_WRONG_CHAR                          3
    #define    PIN_NOT_EQUAL                           4
    #define    PIN_CONFIRMED                           5

    //PIN checking type
    #define    CHK_VALIDITY                            0
    #define    CHK_EQUALITY                            1

    #define    PIN_MODE_USER                           0
    #define    PIN_MODE_FACTORY                        1

    //SCREEN_USER_PIN_MANAGER
    #define    BTN_ENABLE_PIN                          1
    #define    BTN_CHANGE_PIN                          2
    #define    BTN_BACK_SCREEN_USER_PIN_MANAGER        3

// Valve control
 	#define CMD_VALVE_START							   1
	#define CMD_VALVE_NULL							   0


    //SCREEN_TIME_SETTING_3
    #define    TXT_MILEAGE_LIMIT                        2
    #define    TXT_MILEAGE                              5
    #define    BTN_MILEAGE_ENABLED                     15
    #define    BTN_CLEAR_MILEAGE                        10
    #define    BTN_BACK_SCREEN_TIME_SETTING_3          14
    #define    BTN_MODIFY_PIN                          9
     //SCREEN_BALANCE_WARNING
    #define    BTN_BACK_SCREEN_BALANCE_WARNING         14
    #define    MILEAGE_STATUS_GREEN                    0
    #define    MILEAGE_STATUS_RED                      1
    //Time control
    #define    PHASE_RUNNING_IN_WINDOW_STOPPED         0
    #define    PHASE_RUNNING_IN_WINDOW_STARTED         1
    #define    PHASE_END_OF_TASK                       99
    //Heater
    #define    HEAT_CMD_NULL                           0
    #define    HEAT_CMD_START                          1
    #define    HEAT_CMD_STOP                           2
    #define    HEAT_CMD_SINGLE_START                   3

    #define    HEAT_STAGE_STOPPED                      0
    #define    HEAT_STAGE_SOFT_STARTING                1
    #define    HEAT_STAGE_HEATERS_STARTING             2
    #define    HEAT_STAGE_RUNNING                      3
    #define    HEAT_STAGE_SOFT_STOPPING                4
    #define    HEAT_STAGE_HEATERS_STOPPING             5
    //EMERGENT STOP
    #define    EMERGENT_STOP_MODE_NONE                 0
    #define    EMERGENT_STOP_MODE_FRM_MANUAL           1
    #define    EMERGENT_STOP_MODE_FRM_AUTO             2

    //Warnings
    #define    WARNING_NBR                             17


    //MSG
    #define    WARNING_BALANCE_24H                     0
    #define    WARNING_BALANCE_1H                      1
    #define    INFO_FACTORY_SETTING_OK                 2
    #define    INFO_CANT_EDIT_IN_TC_MODE               3
    #define    INFO_CANT_ENTER_TC_MODE                 4
    #define    INFO_CANT_EDIT_IN_RUNNING               5
    #define    INFO_FATAL_FC_ERROR                     6


    //Heater
    #define    HEATING_TIME_GAP_S                      5
    #define    RED                                     0
    #define    GREEN                                   1
	#define    DIM                                     2
	#define    YELLOW                                  3

    #define    SW_HEATER                               0
    #define    SW_BUMP                                 1

    #define    TC_IDLE_PHASE                           (TaskIsRunning == TRUE) && (SysRunning == FALSE)

    #define    UPDATE_DISPLAY_OFF                      0
    #define    UPDATE_DISPLAY_ON                       1

    #define    VALVE_LEVEL_MAX                         20

	#define MOD_PF                          0
	#define MOD_FC                          1
	#define MOD_NUL                         2

    //MEASUREMENT
    #define    IND_CH_OUTLET_PRESSURE                  0
    #define    IND_CH_ENTRANCE_PRESSURE                1
    #define    IND_CH_WATER_LEVEL                      2

    //Working Mode
    #define    TANK_CONST_PRESSURE_MODE                0
    #define    NON_NEGATIVE_MODE                       1
    #define    TANK_NEGATIVE_MODE                      2

    //WaterL evel
    #define    EVENT_WL_NONE                           0
    #define    EVENT_WL_RESTORE                        1
    #define    EVENT_WL_UNDERFLOW                      2


    //Pump Status
    #define    PUMP_RUNNING                            1
    #define    LOW_WATER_LEVEL_PROTECTED               2
    #define    LOW_ENTRANCE_PRESSURE_PROTECTED         7


	#define    ALL_SLEEP     						   3
	#define    SMALL_PUMP_ON			               4
	#define    MAIN_PUMP_ON 					       5

	#define    SYS_STOP								   6
    //Status
    #define    IND_ERR_LP                              0
    #define    IND_ERR_HP                              1
    #define    IND_ERR_WL_L                            2
    #define    IND_ERR_ENTRANCE_LP                     3
    #define    IND_SYSTEM                              4
	#define    ALGO_PID								   0
	#define    ALGO_LINEAR							   1

	#define    STOP_TYPE_PUMP					       0
	#define    STOP_TYPE_SYSTEM					       1

	#define    IND_SMALL_PUMP				     	   5

	#define    STATUS_AUTO_RUN      	0
	#define    STATUS_WAITING       	1
	#define    STATUS_SHALLOW_SLEEP     2
	#define    STATUS_DEEP_SLEEP        3

	//cmd
	#define CMD_NUL           					0
	#define CMD_SET_TEXT           				1
	#define CMD_SET_PROGRESS            		2
	#define CMD_SET_FG_COLOR		    		3
	#define CMD_SET_VISIBILITY		    		4
	#define CMD_SET_BUTTON           			5
	#define CMD_SET_USABILITY           		6

	#define CMD_API_READ_BASE          		 	7
	#define CMD_API_READ_OUTLET_PRESSURE       	7		  //7+0
	#define CMD_API_READ_ENTRANCE        		8		  //7+1
	#define CMD_API_READ_FREQUENCY      		9		  //7+2
	#define CMD_API_READ_ERRORS         		10		  //7+3

#ifdef  USE_NEW_API_PROTOCOL
	#define CMD_API_READ_PUMPS            		12
#else
	#define CMD_API_READ_PUMP1          		12
	#define CMD_API_READ_PUMP2          		13
	#define CMD_API_READ_PUMP3          		14
	#define CMD_API_READ_PUMP4          		15
	#define CMD_API_READ_PUMP5          		16
	#define CMD_API_READ_SMALL_PUMP     		47 //15
#endif
	#define CMD_API_READ_TARGET_PRESSURE         17	 	//7+10
	#define CMD_API_READ_DEFAULT_PRESSURE        18	 	//7+11
	#define CMD_API_WRITE_DEFAULT_PRESSURE       21
	#define CMD_API_SET_PRESSURE                 22
	#define CMD_API_READ_SYSTEM_STATUS			 23
	#define CMD_API_SET_NON_NEGATIVE             23
	#define CMD_API_SET_SMALL_PUMP_ON            24
	#define CMD_API_SET_FACTORY_VALUES           25

    #define CMD_API_READ_PUMP_TEMP          	29	 //17 + 12
	#define CMD_API_READ_PUMP1_TEMP          	29
	#define CMD_API_READ_PUMP2_TEMP          	30
	#define CMD_API_READ_PUMP3_TEMP          	31
	#define CMD_API_READ_SMALL_PUMP_TEMP        32

	#define TXT_VALUE_FOR_NUL                    65000

 

	#define API_CMD_TYPE_READ           0x03
	#define API_CMD_TYPE_WRITE          0x06


	//Settings
	#define    P_FACTOR                 PID_Setting[2]
	#define    I_FACTOR                 PID_Setting[3] * 256 + PID_Setting[4]

	#define    VALVE_WATER_LEVEL_FLOOR        Valve_Control[2] * 256 + Valve_Control[3]
	#define    VALVE_WATER_LEVEL_CEILING      Valve_Control[4] * 256 + Valve_Control[5]
	#define    VALVE_PRESSURE_CEILING         (float)(Valve_Control[6] / 100.0)
	#define    VALVE_PRESSURE_FLOOR           (float)(Valve_Control[7] / 100.0)

	#define    FC_DEFECT_STOP                 Power_Up_Setting[0]
	#define    WATER_SHORTAGE_SC_EFFECTIVE    Power_Up_Setting[1]
 	#define    SAFETY_PROTECT_SC_EFFECTIVE    Power_Up_Setting[2]
 	#define    FC_DEFECT_SC_EFFECTIVE         Power_Up_Setting[3]
 	#define    PUMP1_DEFECT_SC_EFFECTIVE      Power_Up_Setting[4]
 	#define    PUMP2_DEFECT_SC_EFFECTIVE      Power_Up_Setting[5]
 	#define    PUMP3_DEFECT_SC_EFFECTIVE      Power_Up_Setting[6]
	#define    PUMP4_DEFECT_SC_EFFECTIVE      ((Power_Up_Setting[7] & 0x04)>>2)
	#define    PUMP5_DEFECT_SC_EFFECTIVE      ((Power_Up_Setting[7] & 0x02)>>1)
	#define    SMALL_PUMP_DEFECT_SC_EFFECTIVE  (Power_Up_Setting[7] & 0x01)


//	#define    SMALL_PUMP_DEFECT_SC_EFFECTIVE Power_Up_Setting[8]
/*	#define    FC_FRQ_OUT_BIAS_H              Power_Up_Setting[9]
	#define    FC_FRQ_OUT_BIAS_L              Power_Up_Setting[10]
	#define    PWR_UP_AUTORUN_DELAY           Power_Up_Setting[11]	 */




	//Error processing
	#define    PUMP_ERROR_PENDING      0
	#define    PUMP_ERROR_SOLVED       1																		//20200408
	#define    PATH_TO_MAIN_PUMP_ON_IS_BLOCKED     (PumpGroupMode == 2) && (pump_usablilty_tbl[0] == FALSE)// fixed fc pump1 is not usable

	//Fatigue swtiching
	
	#define    RESET_FATIGE_SWITCHING_TIMER RunningCounter = (Pump_Switch_Condtion[5] * 256 + Pump_Switch_Condtion[6]);//	  20210625

	//Mod bus comm.
	 #define   MODBUS_RED_LIGHT_DURATON	    200
	 #define   MODBUS_RED_LIGHT_ON    		ModBusTrafficTimer = MODBUS_RED_LIGHT_DURATON;
 	 #define   MODBUS_GREEN_LIGHT    		ModBusTrafficTimer == 0

//------------------------------------------------------------------
	#define    USE_ON_CHIP_FLASH
  	#define    USE_FLASH
	#define    USE_MODBUS
//	#define    CRC_CHK

//	#define    USE_DAC_BUFFER

 //	#define    USE_PWM
//  #define    DUAL_WATER_SOURCE

 	#define   USE_IWDG
	#define IWDG_FEED    IWDG_ReloadCounter();
//-------------------------Switches for debugging -------------------
//	#define DBG_FACTORY_SETTINGS		   //To be disabled before final release
//	#define    DBG_LABVIEW				  //To be disabled before final release


	#ifdef DBG_MASTER_SWITCH
	    #define    PRINT_TO_UART	  //Enable printing dbg info to uart1, disable MODBUS comm.
		#define    USE_DAC_BUFFER
		#define    DBG_FUNC_FOOTPRINT
		#ifdef DBG_FUNC_FOOTPRINT
	 		#define ADD_INDENT    for (dbg_indent_ind = 0; dbg_indent_ind < dbg_func_depth; dbg_indent_ind++){printf("     ");}
			#define FUNC_GATE_DECORATOR_HEAD   dbg_func_depth++; ADD_INDENT 
			#define FUNC_GATE_DECORATOR_TAIL    ADD_INDENT 	 if (dbg_func_depth) dbg_func_depth--;
		#endif	
	    
/*		#define    USE_CURVES	  //can slow down the reaction of the system, will be removed before final release
		#define    DBG_TARGET_LOCKER
	    #define    DBG_STATE_MACHINE
	  	#define    DBG_FLASH_IMAGE	
	 	#define    DBG_WDG_TEST
	 	#define    DBG_MANUAL_FSW		 */

	// 	#define    DBG_FUNC_LEN
	//	#define    DBG_POWER_UP_INFO
	//	#define    DBG_UNIT_TEST
		#define    DBG_FUNC_INFO
		#define    DBG_SHOW_NAMES
	// 	#define    DBG_FLASH
//	   #define    DBG_MEASURE			
//        #define    DBG_RTC			       
	//  #define    DBG_HEATING_TIMER
	//  #define    DBG_HEATING_STATUS
	//	#define    DBG_VALVE
	    #define DBG_PUMP_MANAGER
//	 	#define DBG_PUMP_PRESSURE_CHECK
	// 	#define DBG_WATER_LEVEL_CHECK
	//	#define DBG_PUMP_ENTRANCE_PRESSURE_CHECK
	
//		#define  MUTE
		#define  DBG_DELAYS
	
	//	#define DBG_WATER_LEVEL
	
	// 	#define DBG_SHOW_INFO_ON_SCREEN
	
	//	#define DBG_MEASURE_CH1
	
		#define DBG_UI_BT_MANAGER
	
//	 	#define DBG_BUFFER_WATCHDOG
	
		#define DBG_DISP_INFO
	#endif

//-----------TO BE DELETED LATER


//	#define DBG_DISABLE_WARNINGS
//	#define DBG_USE_FUEL_MONITOR


////////////////////////////////////for dbg through python
//	#define DBG_USE_UART1_RECEIVING
#ifdef DBG_USE_UART1_RECEIVING
	#define API_CMD_START        'a'
	#define API_CMD_STOP         'b'
	#define API_CMD_HUNGRY_DOG   'c'
#endif

#ifdef USE_MODBUS
	#define API_CMD_START        'a'
	#define API_CMD_STOP         'b'
	#define API_CMD_HUNGRY_DOG   'c'
#endif

////////////////////	for TEST ONLY, to be removed
//	#define DBG_USE_SCREEN_COPY
//	#define DBG_EVENT_AS_ERROR

//	#define MY_CLOCK
    #define FATIGE_CLOCK
	#define TRAFFIC_LIGHT


//---------MODIFICATION INSTRUCTIONS------------------

	/*
	 	Add a new setting item
			---UI control
			---Variable/range/default
			---In-Flash Address
			---Loading, refresh UI control & variable
			---User modification: save to flash
			---Reset to factory setting
	
	*/


 //----------Modification records
   /*
   	  add 2 pumps
			---UI control     
			---Variable/range/default
			---In-Flash Address
			---Loading, refresh UI control & variable
			---User modification: save to flash
			---Reset to factory setting
   
   
   */


	/************************DESKTOP SHORTCUTS********************************************
	//20210518
	 if (AvfMode)	//AVF
		stop_a_pump(FatiguePumpCancel.pump_ind); 
		if(amount_of_running_pumps > 0) amount_of_running_pumps--;


 				if (AvfMode)	//AVF	 
				{
					switch_to_fc(focused_pump_index);
					amount_of_running_pumps++; 
					printf("Sart FC-%d\n",focused_pump_index);
   						UpdatePumpFreq(Pump_Switch_Condtion[4], focused_pump_index);
						show_frequency();

	#ifdef USE_PUMP_ADDING_WAITING_UINIT
					freq_up_counter = timer_tick_count;	  //20210415
	#endif
				}

	Max_Pump_Nbr;	
	amount_of_running_pumps

	extern uint8 pump_enable_tbl[];
	extern uint8 pump_usablilty_tbl[];
	extern uint8 pump_on_off_tbl[]; 
	extern uint8 pump_running_mode_tbl[]; 

	pump_enable_tbl[4] 

	pump_usablilty_tbl[i]
	pump_on_off_tbl[i] 
	pump_running_mode_tbl[i] 
	pump_error_tbl[i]	
	printf("focused_pump_index          = %d\n",focused_pump_index);
#ifdef DBG_FUNC_INFO
	 if(func_dbg_info) printf("\n\n\n...............【UseSparePump(uint8 deffect_ind)】................\n\n");
#endif	
	

#ifdef DBG_FUNC_INFO
				if(func_dbg_info)
				{
					printf("Add-pump canceled\n");
				}
#endif	 

#ifdef DBG_FUNC_INFO
	 func_dbg_info = OFF;
#endif

printf("calling from error_handle.c/L0663\n");


#ifdef DBG_FUNC_INFO
	if(func_dbg_info) printf("\n.........\nTo PC-API:End of the task today\n");
#endif
p1 has an error but still can run

if(PumpGroupMode == 2) //fixed fc

	
	uint8 i;
	for(i = 0; i < 4; i++)
	{

	}

	ShowTargetPressure(float target)

	float tc_target_pressure[6];
float DefaultTargetPressure = 1.0;

	*/


									
 #endif

