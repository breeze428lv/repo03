/************************************版权申明********************************************
**                             广州大彩光电科技有限公司
**                             http://www.gz-dc.com
**-----------------------------------文件信息--------------------------------------------
** 文件名称:   hmi_driver.c
** 修改时间:   2018-05-18
** 文件说明:   用户MCU串口驱动函数库
** 技术支持：  Tel: 020-82186683  Email: hmi@gz-dc.com Web:www.gz-dc.com
--------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------*/
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

#ifdef FIXED_DEVICE_ADDRESS
	#define SELF_ADDRESS        0x01
#else
	#define SELF_ADDRESS        DeviceAddress
#endif

#define MODBUS_DATA_LEN     15				//20210618
#define COMM_DATA_SEND_BUFF_LEN     34		//20210618

#define TX_8(P1) SEND_DATA((P1)&0xFF)                    //发送单个字节
#define TX_8N(P,N) SendNU8((uint8 *)P,N)                 //发送N个字节
#define TX_16(P1) TX_8((P1)>>8);TX_8(P1)                 //发送16位整数
#define TX_16N(P,N) SendNU16((uint16 *)P,N)              //发送N个16位整数
#define TX_32(P1) TX_16((P1)>>16);TX_16((P1)&0xFFFF)     //发送32位整数
uint8 ModBusReceiveBuff[8];

uint8 DeviceAddress;
extern volatile  uint32 timer_tick_count; 
uint32  api_comm_time_stamp;
uint8   ApiCommSysStatus;


static uint16 _crc16 = 0xffff;
extern uint16 SysStatusWord;
extern uint8 SysRunning;
extern uint8 Sys_Switches[];
extern uint16 PowerUpAutoRunTimer;
extern uint8 PumpErrStatus;
extern uint8 ApiCmd;

extern float pump_temp[];
extern uint8 pump_enable_tbl[];

extern bool AllPumpsOn;
extern char err_record[];
extern uint8 pump_error_tbl[];
extern uint8 pump_enable_tbl[];
extern uint8 pump_usablilty_tbl[];
extern uint8 pump_on_off_tbl[]; 
extern uint8 pump_running_mode_tbl[]; 

extern uint8 TrialExpired;
extern float OutletRealTimePressure;
extern float EntranceRealTimePressure;
extern float WaterLevel;
extern uint8 WaterSourceMode;

extern float DefaultTargetPressure;	
extern float Tartget_Pressure;

extern uint8 RemoteTargetEnable;
extern float RemoteTargetPressure;	 //20210618

uint8 CommDataSendBuff[COMM_DATA_SEND_BUFF_LEN];	   //20210618
typedef void(*FUNC)();								   //20210618


FUNC comm_data_func[MODBUS_DATA_LEN] = {		 //20210618
							update_comm_data0,							
							update_comm_data1,
							update_comm_data2,
							update_comm_data3,
							update_comm_data4,							
							update_comm_data5,
							update_comm_data6,
							update_comm_data7,
							update_comm_data8,							
							update_comm_data9,
							update_comm_data10,
							update_comm_data11,
							update_comm_data12,							
							update_comm_data13,
							update_comm_data14,

};


void RS485_Initialization(void)
{
	RS485_TX_En = OFF;
}

/*! 
*  \brief  检查数据是否符合CRC16校验
*  \param buffer 待校验的数据
*  \param n 数据长度，包含CRC16
*  \param pcrc 校验码
*/
static void AddCRC16(uint8 *buffer,uint16 n,uint16 *pcrc)
{
    uint16 i,j,carry_flag,a;

    for (i=0; i<n; i++)
    {
        *pcrc=*pcrc^buffer[i];
        for (j=0; j<8; j++)
        {
            a=*pcrc;
            carry_flag=a&0x0001;
            *pcrc=*pcrc>>1;
            if (carry_flag==1)
                *pcrc=*pcrc^0xa001;
        }
    }
}
/*! 
*  \brief  检查数据是否符合CRC16校验
*  \param buffer 待校验的数据，末尾存储CRC16
*  \param n 数据长度，包含CRC16
*  \return 校验通过返回1，否则返回0
*/
uint16 ModBusCheckCRC16(uint8 *buffer,uint16 n)
{
    uint16 crc0 = 0x0;
    uint16 crc1 = 0xffff;

    if(n>=2)
    {
        crc0 = ((buffer[n - 1] << 8) | buffer[n - 2]);
        AddCRC16(buffer, n - 2, &crc1);
    }

    return (crc0==crc1);
}

void  ModeBusSendChar(uchar t)
{
    USART_SendData(USART1,t);
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    while((USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET));//等待串口发送完毕
}


void ModBusSendData(uint8* data)
{
	uint8 i, c;
	_crc16 = 0xffff;  

	RS485_TX_En = ON;
//	delay_ms(20);
	for(i = 0; i < 5; i++)
	{
		c = data[i];
		AddCRC16(&c, 1, &_crc16);
		ModeBusSendChar(data[i]);
	}
	ModeBusSendChar(_crc16);
	ModeBusSendChar(_crc16 >> 8);
	RS485_TX_En = OFF;
}

void ModBusSendDataBlock(uint8* data, uint8 start_addr, uint8 end_addr)
{
	uint8 i, c;
	_crc16 = 0xffff;  

	RS485_TX_En = ON;

	for(i = 0; i < 3; i++)
	{
		c = data[i];
		AddCRC16(&c, 1, &_crc16);
		ModeBusSendChar(data[i]);
	}
	for(i = start_addr; i <= end_addr; i++)
	{
		c = data[i];
		AddCRC16(&c, 1, &_crc16);
		ModeBusSendChar(data[i]);
	}

	ModeBusSendChar(_crc16);
	ModeBusSendChar(_crc16 >> 8);
	RS485_TX_En = OFF;
}

void ReturnModBusCMD(void)
{
	uint8 i;
	uint8 data[8];

	RS485_TX_En = ON;
 	delay_ms(20);
	for(i = 0; i < 8; i++)
	{
		data[i] = ModBusReceiveBuff[i];
		ModeBusSendChar(data[i]);
	}
	RS485_TX_En = OFF;
	
//	ReturnModBusCMD();	   //20210112
}

uint16 get_frequency(void)					  //46.3
{
	float freq;
	freq = LoadFrequency();
	return (int)(freq * 10);

}

void AnswerApiReadOutlet(void)
{
	uint16 pressure = (int)(OutletRealTimePressure * 1000);
 
	uint8 buff[5] = {0, 0x03, 0x02, 0x00, 0x00};
	buff[0] = SELF_ADDRESS;
	buff[3] = (uint8)(pressure >> 8);
	buff[4] = (uint8)(pressure);

	ModBusSendData(buff);
	
}

void AnswerApiReadEntrance(void)
{
	uint16 data, type_bit_mask = 0;

	uint8 buff[5] = {0, 0x03, 0x02, 0x00, 0x00};
	buff[0] = SELF_ADDRESS;
 	if(WaterSourceMode == TANK_MODE)
	{
	    data = (int)WaterLevel;
	}
	else
	{
	 	data = (int)(EntranceRealTimePressure * 1000);
		type_bit_mask = 0x80;

	}

	buff[3] = (uint8)(data >> 8) + type_bit_mask;
	buff[4] = (uint8)(data);

	ModBusSendData(buff);

}

#ifdef  USE_NEW_API_PROTOCOL
uint8 pump_state(uint8 i)
{
	uint8 data;
 	if(pump_enable_tbl[i] == 0)
	{
	    data = 0;
	}
	else if(pump_error_tbl[i] == 1)
	{
	    data = 3;
	}
	else if(pump_on_off_tbl[i] == 0)
	{
		if(i == 3)	 //for small pump here is only standby mode
		{
			data = 1;
		}
		else
		{
		 	if(!AllPumpsOn) 
			{
			   	data = 1;
			}
			else
			{
			    data = 2;
			}
		}
	}
	else
	{
	 	if(pump_running_mode_tbl[i] == MOD_PF) 
		{
		   	data = 4;
		}
		else if(pump_running_mode_tbl[i] == MOD_FC) 
		{
		    data = 5;
		}
	}
	return data;

}
#else
uint8 pump_state(uint8 i)
{
	uint8 data;
 	if(pump_enable_tbl[i] == 0)
	{
	    data = 0;
	}
	else if(pump_error_tbl[i] == 1)
	{
	    data = 3;
	}
	else if(pump_on_off_tbl[i] == 0)
	{
		if(i == 3)	 //for small pump here is only standby mode
		{
			data = 1;
		}
		else
		{
		 	if(!AllPumpsOn) 
			{
			   	data = 1;
			}
			else
			{
			    data = 2;
			}
		}
	}
	else
	{
	 	if(pump_running_mode_tbl[i] == MOD_PF) 
		{
		   	data = 4;
		}
		else if(pump_running_mode_tbl[i] == MOD_FC) 
		{
		    data = 5;
		}
	}
	return data;

}
#endif

#ifdef  USE_NEW_API_PROTOCOL
void AnswerApiReadPumps(void)
{
	uint8 buff[5] = {0, 0x03, 0x02, 0x00, 0x00};
	buff[0] = SELF_ADDRESS;
	buff[3] = (pump_state(0)<<4) + pump_state(1);
	buff[4] = (pump_state(2)<<4) + pump_state(3);

	ModBusSendData(buff);
}
#else
void AnswerApiReadPump(uint8 i)
{
	uint8 buff[6] = {0, 0x03, 0x02, 0x05, 0x00, 0x00};
	buff[0] = SELF_ADDRESS;
	buff[3] = 0;
	buff[4] = pump_state(i);

	ModBusSendData(buff);
}
#endif
//DefaultTargetPressure;	 //LoadTargetPressure();
void AnswerApiReadTargetPressure(void)
{
	uint16 pressure = (int)(Tartget_Pressure * 1000);//(int)(LoadTargetPressure() * 1000);

	uint8 buff[5] = {0, 0x03, 0x02, 0x00, 0x00};
	buff[0] = SELF_ADDRESS;
	buff[3] = (uint8)(pressure >> 8);
	buff[4] = (uint8)(pressure);

	ModBusSendData(buff);

}

void AnswerApiReadDefaultPressure(void)
{
	uint16 pressure = (int)(DefaultTargetPressure * 1000);

 	uint8 buff[5] = {0, 0x03, 0x02, 0x00, 0x00};
	buff[0] = SELF_ADDRESS;
	buff[3] = (uint8)(pressure >> 8);
	buff[4] = (uint8)(pressure);

	ModBusSendData(buff);


}
//pump_temp[4]   123.5-> 0 1235
void AnswerApiReadPump1Temp(void)
{
	uint16 abs_temp;
	uint8 sig_bit;

 	uint8 buff[5] = {0, 0x03, 0x02, 0x00, 0x00};
	buff[0] = SELF_ADDRESS;
//	if(pump_enable_tbl[0] == TRUE)
	{	
		abs_temp = (pump_temp[0] < 0)? (int)(-pump_temp[0] * 10 + 0.5):(int)(pump_temp[0] * 10 + 0.5);
		sig_bit = (pump_temp[0] < 0)? 1:0;
	
		buff[3] = (uint8)(abs_temp >> 8) + sig_bit * 0x80;
		buff[4] = (uint8)(abs_temp);
	}
/*	else
	{
		buff[3] = 0xff;
		buff[4] = 0xff;

	}  */

	ModBusSendData(buff);

}
void AnswerApiReadPump2Temp(void)
{
	uint16 abs_temp;
	uint8 sig_bit;

 	uint8 buff[5] = {0, 0x03, 0x02, 0x00, 0x00};
	buff[0] = SELF_ADDRESS;
	if(pump_enable_tbl[1] == TRUE)
	{		
		abs_temp = (pump_temp[1] < 0)? (int)(-pump_temp[1] * 10):(int)(pump_temp[1] * 10);
		sig_bit = (pump_temp[1] < 0)? 1:0;
	
		buff[3] = (uint8)(abs_temp >> 8) + sig_bit * 0x80;
		buff[4] = (uint8)(abs_temp);
	}
	else
	{
		buff[3] = 0xff;
		buff[4] = 0xff;

	}
	ModBusSendData(buff);

}
void AnswerApiReadPump3Temp(void)
{
	uint16 abs_temp;
	uint8 sig_bit;
	
 	uint8 buff[5] = {0, 0x03, 0x02, 0x00, 0x00};
	buff[0] = SELF_ADDRESS;
	if(pump_enable_tbl[2] == TRUE)
	{
		abs_temp = (pump_temp[2] < 0)? (int)(-pump_temp[2] * 10):(int)(pump_temp[2] * 10);
		sig_bit = (pump_temp[2] < 0)? 1:0;
	
		buff[3] = (uint8)(abs_temp >> 8) + sig_bit * 0x80;
		buff[4] = (uint8)(abs_temp);
	}
	else
	{
		buff[3] = 0xff;
		buff[4] = 0xff;

	}	

	ModBusSendData(buff);

}
void AnswerApiReadSmallPump(void)
{
	uint16 abs_temp;
	uint8 sig_bit;

 	uint8 buff[5] = {0, 0x03, 0x02, 0x00, 0x00};
	buff[0] = SELF_ADDRESS;
	if(pump_enable_tbl[3] == TRUE)
	{	
		abs_temp = (pump_temp[3] < 0)? (int)(-pump_temp[3] * 10):(int)(pump_temp[3] * 10);
		sig_bit = (pump_temp[3] < 0)? 1:0;
	
		buff[3] = (uint8)(abs_temp >> 8) + sig_bit * 0x80;
		buff[4] = (uint8)(abs_temp);
	}
	else
	{
		buff[3] = 0xff;
		buff[4] = 0xff;

	}
	ModBusSendData(buff);
}

void AnswerApiReadFrequency(void)
{
	 uint16 the_freq;

	 uint8 buff[5] = {0, 0x03, 0x02, 0x00, 0x00};
	 buff[0] = SELF_ADDRESS;
	 the_freq = get_frequency();
	 buff[3] = (uint8)(the_freq >> 8);
	 buff[4] = (uint8)(the_freq);

	 ModBusSendData(buff);
}


void AnswerApiReadSystemStatus(void)	   //DC_CPWS-Ver-1.3.9
{
	 uint16 the_freq;

	 uint8 buff[5] = {0, 0x03, 0x02, 0x00, 0x00};
	 buff[0] = SELF_ADDRESS;
	 	
	 buff[4] = ApiCommSysStatus;

	 ModBusSendData(buff);
}
	
void AnswerApiReadErrors(void)
{
	 uint8 err[16];

	 uint8 buff[5] = {0, 0x03, 0x02, 0x00, 0x00};
	 uint8 i;
	 buff[0] = SELF_ADDRESS;
	 for(i = 0; i < 14; i++)
	 {
		 err[i] = err_record[i];
	 }
	 
	 err[8] = pump_error_tbl[0];
	 err[9] = pump_error_tbl[1];
	 err[10] = pump_error_tbl[2];
	 err[11] = pump_error_tbl[3];
	 err[14] = TrialExpired;
	
	 buff[3] = CompressData(err, 8);
	 buff[4] = CompressData(err + 8, 8);

	 ModBusSendData(buff);
}
/*[64 06 02 02 00 01 00 00]*/  //6 pumps done
void control_relays(uint8 addr, uint8 on_off)
{
	switch(addr)								
	{								
	    case 1:            //变频开关								
	        FC_SWITCH = on_off;								
	        break;								
	    case 2:            //泵1变频								
	        PUMP1_FC  = on_off;								
	        break;								
	    case 3:            //泵1工频								
	        PUMP1_PF = on_off;								
	        break;								
	    case 4:            //泵2变频								
	        PUMP2_FC  = on_off;								
	        break;								
	    case 5:            //泵2工频								
	        PUMP2_PF = on_off;								
	        break;								
	    case 6:            //泵3变频								
	        PUMP3_FC  = on_off;								
	        break;								
	    case 7:            //泵3工频								
	        PUMP3_PF = on_off;								
	        break;				
	    case 8:            //泵4变频								
	        PUMP4_FC  = on_off;								
	        break;								
	    case 9:            //泵4工频								
	        PUMP4_PF = on_off;								
	        break;								
	    case 10:            //泵5变频								
	        PUMP5_FC  = on_off;								
	        break;								
	    case 11:            //泵5工频								
	        PUMP5_PF = on_off;								
	        break;										
	    case 12:            //辅泵变频								
	        AUX_PUMP_FC = on_off;								
	        break;								
	    case 13:            //故障指示								
	        warning_out = on_off;								
	        break;								
	    case 14:            //进水阀								
	        ValveIntoTank = on_off;								
	        break;								
	    case 15:            //泄压阀								
	        ValveDecompression = on_off;								
	        break;								
	    default:															
	        break;								
	}								
	

}
extern uint8 UseIoTargetPressure;
extern float DefaultTargetPressureMenu;
void AnswerApiWriteDefaultPressure(uint16 data1, uint8 data2)
{

	float pressure = (data1 * 256 + data2) / 1000.0;

	ReturnModBusCMD();

	DefaultTargetPressureMenu = pressure;
	if (UseIoTargetPressure == FALSE)		  //20210703
	{
		DefaultTargetPressure = pressure; 
		push_cmd_pump(CMD_API_SET_PRESSURE, SCREEN_TIME_CTR_3 , TXT_TC_DEFAULT_PRESSURE, data1 * 256 + data2); 
	
		ScheduleUpdate();	   //update schedula data with DefaultTargetPressure	
		//SaveScheduleToFlash();	//20210630	
	}
	SaveScheduleToFlash();	//20210630	
}

void AnswerApiSetNonNeagtive(void)
{
	ReturnModBusCMD();
	WaterSourceMode = NONNEGTIVE_MODE;
    controls_update_SCREEN_PUMP_GROUPING();
	SaveSettingsToFlash(SCREEN_PUMP_GROUPING);	  //20210630
}

void AnswerApiSetSmallPumpOn(void)
{  
	ReturnModBusCMD();

    pump_enable_tbl[5] = ON;

	SetButtonValue(SCREEN_PUMP_GROUPING, 7, ON);	

	SetControlVisiable(SCREEN_MAIN_1, 43, OFF);

	SetControlVisiable(SCREEN_MAIN_1, 123, OFF);

	SaveSettingsToFlash(SCREEN_PUMP_GROUPING);		 //20210630
}
extern uint16 ReturnScreenId;
void AnswerApiSetFactoryValues(void)
{
	ReturnModBusCMD();
		
    FactorySettings(0x99);
}

#ifdef DBG_LABVIEW
void ApiLabViewFeeder(void)
{
	 uint8 buff[6] = {0x69, 0x69, 0x69, 0x69, 0x69, 0x69};
	 ModBusSendData(buff);
}
#endif

void SendCmdBack(void)
{
	uint8 i;
	uint8 data[8];

	RS485_TX_En = ON;
	delay_ms(20);
	for(i = 0; i < 8; i++)
	{
		data[i] = ModBusReceiveBuff[i];
		ModeBusSendChar(data[i]);
	}
	RS485_TX_En = OFF;
}

void SendCommData(uint8 start_addr, uint8 len)	   //20210618
{
	 CommDataSendBuff[0] = SELF_ADDRESS;
	 CommDataSendBuff[1] = 3;
	 CommDataSendBuff[2] = len * 2;
	 CommDataSendBuff[3] = 0;	   //unused
	 ModBusSendDataBlock(CommDataSendBuff, start_addr * 2 + 4, start_addr * 2 + 4 + len * 2 - 1);
}


#define IS_READ_PUMP_TEMP   (ModBusReceiveBuff[3] >= 12) && (ModBusReceiveBuff[3] <= 15)
void modbus_cmd_processor(void)
{

//	while(!MODBUS_GREEN_LIGHT);
//	 printf("SELF_ADDRESS=%d\n", SELF_ADDRESS);
//	 printf("ModBusReceiveBuff=%d,%d,%d\n", ModBusReceiveBuff[0],ModBusReceiveBuff[1],ModBusReceiveBuff[2]);
	IWDG_FEED
	if(ModBusReceiveBuff[0] == SELF_ADDRESS)  //my address	 01
	{
#ifdef  CRC_CHK
	 	if(!ModBusCheckCRC16(ModBusReceiveBuff, 8))
		{
		 	printf("CRC error!\n");
			return;
		}	 
#endif
    	if(ModBusReceiveBuff[1] == API_CMD_TYPE_WRITE)
		{
			
			if(ModBusReceiveBuff[2] == 0x01)
			{
				if(ModBusReceiveBuff[3] == 0x01)
				{
					printf("=============================MODBUS:START1\n");
					ApiCmd = API_CMD_START;
	
				}
				else if(ModBusReceiveBuff[3] == 0x02)
				{
					ApiCmd = API_CMD_STOP;
				}
				else if(ModBusReceiveBuff[3] == 0x03)	 //SET_NON_NEGATIVE:  01 06 01 03 00 00 00 00
				{
				//	AnswerApiSetNonNeagtive();
					push_cmd_pump(CMD_API_SET_NON_NEGATIVE, ModBusReceiveBuff[4], ModBusReceiveBuff[5], 0);
				}
				else if(ModBusReceiveBuff[3] == 0x04)
				{
				    //AnswerApiSetSmallPumpOn();
				  	push_cmd_pump(CMD_API_SET_SMALL_PUMP_ON, ModBusReceiveBuff[4], ModBusReceiveBuff[5], 0);
				}
				else if(ModBusReceiveBuff[3] == 0x0f)
				{
				//	AnswerApiSetFactoryValues();
					push_cmd_pump(CMD_API_SET_FACTORY_VALUES, ModBusReceiveBuff[4], ModBusReceiveBuff[5], 0);
				}

			}
			else if((ModBusReceiveBuff[2] == 0x00) && (ModBusReceiveBuff[3] == 0x0b))  
			{
				 
				 push_cmd_pump(CMD_API_WRITE_DEFAULT_PRESSURE, ModBusReceiveBuff[4], ModBusReceiveBuff[5], 0);
				// AnswerApiWriteDefaultPressure(ModBusReceiveBuff[4], ModBusReceiveBuff[5]);
			}
			else if(ModBusReceiveBuff[2] == 0x02)
			{
				 control_relays(ModBusReceiveBuff[3], ModBusReceiveBuff[5]);				 
			}
		}
		else if(ModBusReceiveBuff[1] == API_CMD_TYPE_READ)	 //Read
		{
#ifdef USE_BATCH_READ			 //20210618
		#ifdef DBG HYBRID_UART1_TXT 
			      printf("{ ");
		#endif	
			SendCommData((ModBusReceiveBuff[2] << 8) | ModBusReceiveBuff[3], (ModBusReceiveBuff[4] << 8) | ModBusReceiveBuff[5]);
		#ifdef DBG HYBRID_UART1_TXT 
			      printf(" }\n");
		#endif	
#else				
			if(IS_READ_PUMP_TEMP)
			{
				switch(CMD_API_READ_BASE + ModBusReceiveBuff[3] + 10)
				{
					case CMD_API_READ_PUMP_TEMP:
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
					default:
						break;	

				}
			    //push_cmd_pump(CMD_API_READ_BASE + ModBusReceiveBuff[3] + 10, 0, 0, 0);
			}
			else
			{	
				switch(CMD_API_READ_BASE + ModBusReceiveBuff[3])
				{
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
							
					case CMD_API_READ_PUMP4:
						AnswerApiReadPump(3);
						break;	
					case CMD_API_READ_PUMP5:
						AnswerApiReadPump(4);
						break;	
																	
					case CMD_API_READ_SMALL_PUMP:
						AnswerApiReadPump(5);
						break;		
#endif
			
					case CMD_API_READ_TARGET_PRESSURE:
						AnswerApiReadTargetPressure();
						break;		
			
					case CMD_API_READ_DEFAULT_PRESSURE:
					    AnswerApiReadDefaultPressure();		  //
						break;	
					case CMD_API_READ_SYSTEM_STATUS:		  //01 03 00 10 00 00 00 00	 //DC_CPWS-Ver-1.3.9
					    AnswerApiReadSystemStatus();		  //
						break;	

					default:
						
						break;		

				}				
					
			//	push_cmd_pump(CMD_API_READ_BASE + ModBusReceiveBuff[3], 0, 0, 0);
			}
#endif
		}


	}


}
//
void AnswerApiCmd(uint8 err)
{
	 uint16 the_freq;

	 uint8 buff[5] = {0, 0x03, 0x02, 0, 0};
	 buff[0] = SELF_ADDRESS;
	 buff[3] = err;
	 buff[4] = err;

	 ModBusSendData(buff);
}


void ModBusReceiver(uint8 data)
{
	static uint8 ind = 0;
    static uint8 in_cmd = 0;
	uint8 first_byte;

	first_byte = 0;
	if(in_cmd == 0) 
	{
		if(data == SELF_ADDRESS)  {in_cmd = 1; first_byte = 1;}
		else return;
	}
	if(ind < 8)
	{
	    if((first_byte == 0) && (timer_tick_count - api_comm_time_stamp > 500))//5000ms  time out within a cmd, the first byte of cmd does not apply
		{
		////////////////////
		 //	ModeBusSendChar(0xa5);
		//	ModeBusSendChar(0x5a);
			AnswerApiCmd(0xa5);

		    ind = 0;
			if(data != SELF_ADDRESS)
			{
			   in_cmd = 0;
			   return;
			}
		}
		ModBusReceiveBuff[ind] = data;
		api_comm_time_stamp = timer_tick_count; 
		ind++;
		if(ind == 8)
		{	////////////////////
		  //  ModeBusSendChar(0x66);
		//	ModeBusSendChar(ModBusReceiveBuff[2]);
		//	ModeBusSendChar(ModBusReceiveBuff[3]);

		 /////////////////////////
		//	ReturnModBusCMD();
			modbus_cmd_processor();
			ind = 0;
			in_cmd = 0;
		}
	}
	
}
void update_comm_data0(void)
{
 	uint16 data = (int)(OutletRealTimePressure * 1000);

    CommDataSendBuff[4] = (uint8)(data >> 8);
	CommDataSendBuff[5] = (uint8)(data);
}
void update_comm_data1(void)
{
	uint16 data, type_bit_mask = 0;

 	if(WaterSourceMode == TANK_MODE)
	{
	    data = (int)WaterLevel;
	}
	else
	{
	 	data = (int)(EntranceRealTimePressure * 1000);
		type_bit_mask = 0x80;
	}

    CommDataSendBuff[6] = (uint8)(data >> 8) + type_bit_mask;
	CommDataSendBuff[7] = (uint8)(data);

}
void update_comm_data2(void)
{
    uint16 data;
	data = get_frequency();

    CommDataSendBuff[8] = (uint8)(data >> 8);
	CommDataSendBuff[9] = (uint8)(data);
}
void update_comm_data3(void)			//???
{
	 uint8 err[16];
	 uint8 i;

	 for(i = 0; i < 14; i++)
	 {
		 err[i] = err_record[i];
	 }
	 
	 err[8]  = pump_error_tbl[0];
	 err[9]  = pump_error_tbl[1];
	 err[10] = pump_error_tbl[2];
	 err[11] = pump_error_tbl[3];
	 err[12] = pump_error_tbl[4];
	 err[13] = pump_error_tbl[5];
	 err[14] = TrialExpired;
	
	 CommDataSendBuff[10] = CompressData(err, 8);
	 CommDataSendBuff[11] = CompressData(err + 8, 8);
}

void update_comm_data4(void)
{
	 CommDataSendBuff[12] = 0;
	 CommDataSendBuff[13] = pump_state(0);
}

void update_comm_data5(void)
{
	 CommDataSendBuff[14] = 0;
	 CommDataSendBuff[15] = pump_state(1);
}
void update_comm_data6(void)
{
	 CommDataSendBuff[16] = 0;
	 CommDataSendBuff[17] = pump_state(2);
}
void update_comm_data7(void)
{
	 CommDataSendBuff[18] = 0;
	 CommDataSendBuff[19] = pump_state(3);
}
void update_comm_data8(void)
{
	 CommDataSendBuff[20] = 0;
	 CommDataSendBuff[21] = pump_state(4);
}
void update_comm_data9(void)
{
	 CommDataSendBuff[22] = 0;
	 CommDataSendBuff[23] = pump_state(5);
}

void update_comm_data10(void)	  //20210618
{
 	uint16 data = (int)(RemoteTargetPressure * 1000);
    CommDataSendBuff[24] = (uint8)(data >> 8);
	CommDataSendBuff[25] = (uint8)(data);
}

void update_comm_data11(void)
{
 	uint16 data = (int)(Tartget_Pressure * 1000);
    CommDataSendBuff[26] = (uint8)(data >> 8);
	CommDataSendBuff[27] = (uint8)(data);
}
void update_comm_data12(void)
{
 	uint16 data = (int)(DefaultTargetPressure * 1000);
    CommDataSendBuff[28] = (uint8)(data >> 8);
	CommDataSendBuff[29] = (uint8)(data);
}

void update_comm_pump_temp(uint8 i)
{
	uint16 abs_temp;
	uint8 sig_bit;

	if(pump_enable_tbl[i] == TRUE)
	{	
		abs_temp = (pump_temp[i] < 0)? (int)(-pump_temp[i] * 10 + 0.5):(int)(pump_temp[i] * 10 + 0.5);
		sig_bit = (pump_temp[i] < 0)? 1:0;
	
		CommDataSendBuff[30 + i * 2] = (uint8)(abs_temp >> 8) + sig_bit * 0x80;
		CommDataSendBuff[31 + i * 2] = (uint8)(abs_temp);
	}
	else
	{
		CommDataSendBuff[30 + i * 2] = 0xff;
		CommDataSendBuff[31 + i * 2] = 0xff;
	}

}

void update_comm_data13(void)
{
	update_comm_pump_temp(0);
}



void update_comm_data14(void)
{
	CommDataSendBuff[32] = 0;
	CommDataSendBuff[33] = ApiCommSysStatus;
}


void UpdateCommData(void)
{
	uint8 i;
	for (i = 0; i < MODBUS_DATA_LEN; i++)		//20210618
	{
		 comm_data_func[i]();
	}
}
