/************************************版权申明********************************************
**                            深圳英捷思科技有限公司
**                            
**-----------------------------------文件信息--------------------------------------------
**
--------------------------------------------------------------------------------------*/
#include "hmi_driver.h"
#include "cmd_queue.h"
#include "cmd_process.h"
#include "stdio.h"
#include "hw_config.h"
#include "ulitity.h"
#include "string.h"
#include "stm32f10x.h"
#include "usart.h"
#include "relays.h"
#include "delay.h"
#include "adc.h"
#include "dac.h"
#include "error_handle.h"
#include "macros.h"
#include "settings.h"

uint8 MileageStatus;
extern uint8 FactoryInfoLocked;
extern uint16 HourMileage;
extern uint16 MileageLimit;
extern uint16 ReturnScreenId;

extern uint8 MileageEnabled;
extern char DbgBuff[];

extern uint8 now_year;
extern uint8 now_month;
extern uint8 now_day;
extern uint8 now_hour;
extern uint8 now_min;
extern uint8 now_sec;

extern uint8 UserPin[];
uint8 FactoryPin[3];

uint8 PinMode = PIN_MODE_USER;
uint8 Pin_Enabled;
uint8 Sys_Pin_Enabled;

char PinCopy[6];


int PinChkType = CHK_VALIDITY;


#ifdef ENGLISH_VERSION
char* UserPinReminder[6] = {
					           "Please enter new password(0-9)",
					           "Please re-enter",
					           "Less than 6 digits, please re-enter",
					           "Incorrect password, please re-enter",
					           "2 inputs are different, please re-enter",
							   "New password confirmed"

};
#else
char* UserPinReminder[6] = {
					           "请输入新密码(0-9)",
					           "请再次输入密码",
					           "不足6位，请重输",
					           "输入不正确，请重输",
					           "两次输入不一致，请重输",
							   "新密码生效"

};
#endif

#ifdef ENGLISH_VERSION
char* messages[7] = {
           "设备可用小时数不足24小时，请尽快和厂家联系",
           "设备可用小时数不足1小时，即将暂停使用",
           "Restored to factory settings,   please restart.",
           "请取消定时模式，再进行设置",
           "请先退出手动模式",
		   "请先退出运行模式",
		   "变频器故障，请重新开机"
};
#else
char* messages[7] = {
           "设备可用小时数不足24小时，请尽快和厂家联系",
           "设备可用小时数不足1小时，即将暂停使用",
           "设备已恢复出厂设置, 请重新上电",
           "请取消定时模式，再进行设置",
           "请先退出手动模式",
		   "请先退出运行模式",
		   "变频器故障，请重新开机"
};
#endif



/**********************************************************************************************
 * Name：pw_str2arr(char* str, uint8* arr)
 * Brief：convert a 3-char string into a 3-byte array
 *   
 * Para：   str -- a 3-char string
 *          arr -- a 3-byte array for the result
 *	      
 * Return  : result of check  
 * Example:  "147369"->{14, 73, 69}                    
 * Caller(s): PinMonitor()
 **********************************************************************************************/

void pw_str2arr(char* str, uint8* arr)
{
	int val, i;
	sscanf(str,"%ld",&val); 

	for(i = 2; i >= 0; i--) 
 	{
 		arr[i] = val%100;
		val/=100;
 	}	
}

/**********************************************************************************************
 * Name：valid_chk(char* str_in)
 * Brief：check the validity of current PIN input: should be 6 digits(0 ~ 9)
 *   
 * Para：   str_in-- input PIN
 *	      
 * Return  : result of check                       
						   	   	 PIN_REPEAT:      correct PIN                            
						   	     PIN_WRONG_LEN:   the length is not 6                         
						   	     PIN_WRONG_CHAR   includes charactor out of the range(0~9)                      
 * Caller(s):  PinCheck()
 **********************************************************************************************/
int valid_chk(char* str_in)
{
	int i = 0;
	char str[10];
	strcpy(str, str_in);
	strcat(str, "#");
	while(str[i] != '#')
	{
		if((str[i] >57 )||(str[i] < 48 )) return PIN_WRONG_CHAR;   //out of the range from '0' to '9'		
		i++;
	}

	if(i != 6) return PIN_WRONG_LEN;	 //the length is not 6 digits
	return PIN_REPEAT;				     // valid PIN input
}

/**********************************************************************************************
 * Name：equality_chk(char* pin_input)
 * Brief：check the current PIN input against PinCopy
 *   
 * Para：   pin_input-- input PIN
 *	      
 * Return  : result of check                       
						   	     PIN_NOT_EQUAL :  pin_input[] !=  PinCopy[]                       
						    	 PIN_CONFIRMED :  pin_input[] ==  PinCopy[]                         
 * Caller(s):  PinCheck()
 **********************************************************************************************/
int equality_chk(char* pin_input)
{
	if(strcmp(pin_input, PinCopy))	return PIN_NOT_EQUAL;
	return PIN_CONFIRMED;
} 

/**********************************************************************************************
 * Name：PinCheck(int chk_type, char* pin_input)
 * Brief：check the current PIN input according to the specified check type(chk_type)
 *   
 * Para：   pin_input-- input PIN
 *	        chk_type --	check type:	CHK_VALIDITY | CHK_EQUALITY
 * Return  : result of check:
						     	 PIN_INPUT                             
						    	 PIN_REPEAT                            
						   	     PIN_WRONG_LEN                         
						   	     PIN_WRONG_CHAR                         
						   	     PIN_NOT_EQUAL                          
						    	 PIN_CONFIRMED                         
 * Caller(s):  PinMonitor() 
 **********************************************************************************************/
int PinCheck(int chk_type, char* pin_input)
{
	int result;
	if(chk_type == CHK_VALIDITY)
	{
		result = valid_chk(pin_input);
	}
	else if(chk_type == CHK_EQUALITY)
	{
		result = equality_chk(pin_input);	
	}
	
	return result;
}
/**********************************************************************************************
 * Name：PinMonitor(char* str)
 * Brief：check the current PIN input and decide the next checking type and/or further actions
 *   
 * Para：   str-- input PIN	
 * Return  : null
 * Example: 1. input 2*8546, this is an invalid PIN, so go back to "check validity" at the next step.
 *          2. input 123456, it is valid , then switch to "check equality"
 *          3. input 123456 again, thus passes the equality check and get PIN_CONFIRMED, update the relevent 
 *             variables , save to FLASH and escapre the current screen.
 * Caller(s):  main.c/NotifyText()/SCREEN_PIN_SETTING_3   
 **********************************************************************************************/
void PinMonitor(char* str)
{
	int res;
	
	printf("%s\n",str);

	res = PinCheck(PinChkType, str);

 	printf("%s\n", UserPinReminder[res]);
	SetTextValue(SCREEN_PIN_SETTING_3, TXT_USER_PIN_REMINDER , (uchar*)UserPinReminder[res]); 
 	switch(res)
 	{
 		case PIN_CONFIRMED: 	
		    printf("PIN_CONFIRMED \n");
		    PinChkType = CHK_VALIDITY;

			if(PinMode == PIN_MODE_FACTORY)
				pw_str2arr(PinCopy, FactoryPin);
			else
		    	pw_str2arr(PinCopy, UserPin);
 	 	    printf("PinMode = %d\n",PinMode);

			printf("WritePinToFlash();\n");
			if(PinMode == PIN_MODE_FACTORY) 
			{
#ifdef  DBG_FLASH

				printf("FactPin[]:\n");	
	   	        display(FactoryPin,3);
#endif

				WriteFactoryPinToFlash();
				FactoryInfoLocked = 0xa5;
				SetScreen(SCREEN_SYS_SETTINGS_2);
			}
			else
			{ 
		    	WritePinToFlash();
				SetScreen(SCREEN_SYS_SETTINGS_2);
			}
		    printf("Go back to main sceen\n");
		
			SetTextValue(SCREEN_PIN_SETTING_3, TXT_USER_PIN_REMINDER , (uchar*)UserPinReminder[PIN_INPUT]); 

			break;
	    case PIN_REPEAT: 
		    PinChkType = CHK_EQUALITY;
		    strcpy(PinCopy, str);
 			break;
 		default:
 			PinChkType = CHK_VALIDITY;
 			break;
 		
	 }
}
/**********************************************************************************************
 * Name：ShowMessage(uint8 level)
 * Brief：Send cmd to hmi: give an alert according to variable level
 *   
 * Para：   level, the level deciding what to put on the warning screen for mileage	
 *          level = 0  "设备可用小时数不足24小时，请尽快和厂家联系"
 *          level = 1  "设备可用小时数不足1小时，即将暂停使用"
 * Example:  
 * Caller(s):  main.c/NotifyReadRTC |  main.c/NotifyButton/SCREEN_SYS_SETTINGS_2
 **********************************************************************************************/
void ShowMessage(uint8 level)
{
	SetTextValue(SCREEN_BALANCE_WARNING, 2, (uchar*)messages[level]);
	SetScreen(SCREEN_BALANCE_WARNING); 
	delay_ms(100);
}

/**********************************************************************************************
 * Name：PassStopperMonitor()
 * Brief：monitor the mileage, report the status whether the valid mileage is used out
 *   
 * Para：   null	
 * Return ：1  if there is less than one hour left
 *          0  there are at least 2 hours left 
 * Example:  
 * Caller(s):  main.c/NotifyButton()/SCREEN_TIME_CTR_3 | SCREEN_MAIN_1
 **********************************************************************************************/
uint8 PassStopperMonitor()
{
	if(MileageEnabled == FALSE) return 1;

	if(MileageStatus == MILEAGE_STATUS_RED)	
	{
		ShowMessage(WARNING_BALANCE_1H);
		return 0;
	}
	return 1;
}


uint8 dbg_info_nbr = 0;
void debugger_clean(void)
{
	uint8 i, j, jMax;			
	char buff[30];
	jMax = (dbg_info_nbr - 1) / 17 + 1;  
	
	for(j = 0; j < jMax; j++)	  //page nbr
	{
		sprintf(buff, "Page%d记录清除中...", j);
		SetTextValue(33, 28, (uchar*)buff);
		delay_ms(100);
		for(i = 1; i <= 17; i++)  //ind in page
		{
		   SetTextValue((j + 1) / 2 + j + 33, i, "");
		   delay_ms(100);
		}
		IWDG_FEED
	}
	SetTextValue(33, 28, "");
}

#define STR        3
void debugger(char* msg, uint8 cmd)
{
  	char buff[100];
	static uint8 page_nbr = 0;
	static uint8 dbg_info_ind = 1;
	if(cmd == OFF)
	{
	    debugger_clean();
		page_nbr = 0;
		dbg_info_ind = 1;
		dbg_info_nbr = 0;
	}
	else												   //0 1 2   +1 ->1 2 3  /2  0 1 1 +i -> 0 2 3 + 33-> 33 35 36
	{
		if(cmd == DBG_DATA)
		{
			sprintf(buff, "                             ---%s", msg);
		}
		else
		{
			sprintf(buff, "20%d-%02d-%02d %02d:%02d:%02d---%s",  now_year, now_month, now_day, now_hour, now_min, now_sec, msg);
		}

	 	SetTextValue((page_nbr + 1) / 2 + page_nbr + 33, dbg_info_ind, (uchar*)buff);

	 	delay_ms(100);
	
		if(dbg_info_nbr < 51) dbg_info_nbr++;
	
		if(dbg_info_ind < 17)
		{
		 	dbg_info_ind++;
		}
		else
		{
			page_nbr++;
			if(page_nbr > 2) page_nbr = 0;

		    dbg_info_ind = 1;
		}
	}
}



void ShowCurrentPassWord(uint8 type, uint8 visible)
{

	 char buff[30];
	 if(visible)
	 {
		 if(type == PIN_MODE_USER)
		 {
#ifdef ENGLISH_VERSION
			sprintf(buff, "Current password：%2d%2d%2d", UserPin[0], UserPin[1], UserPin[2]);
#else
		 	sprintf(buff, "当前参数密码：%2d%2d%2d", UserPin[0], UserPin[1], UserPin[2]);
#endif
		 }
		 else if(type == PIN_MODE_FACTORY)
		 {
#ifdef ENGLISH_VERSION
			sprintf(buff, "Current password：%2d%2d%2d", FactoryPin[0], FactoryPin[1], FactoryPin[2]);
#else
			sprintf(buff, "当前系统密码：%2d%2d%2d", FactoryPin[0], FactoryPin[1], FactoryPin[2]);
#endif
		 }
		 SetTextValue(SCREEN_USER_PIN_MANAGER, TXT_CURRENT_PIN, (uchar*)buff);
	 }
	 else
	 {
		 SetTextValue(SCREEN_USER_PIN_MANAGER, TXT_CURRENT_PIN, "");
	 }

}
