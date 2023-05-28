

#ifndef __ERROR_HANDLE_H
#define __ERROR_HANDLE_H	 



#define StatusCH2    GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_4)		 //D4	  SAFETY PROTECTION
#define StatusCH3    GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_5)		 //D5	  FC ERROR
#define StatusCH4    GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_10)		 //D10	  PUMP1
#define StatusCH7    GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12)		 //C12	  PUMP4
#define StatusCH8    GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_6)		 //D6     REMOTE CONTROL		 CODE PAD CH-R8 
#define StatusCH9    GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_7)		 //D7     NO RUNNING 
#define StatusCH5    GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_10)		 //C10	  PUMP2
#define StatusCH6    GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_11)		 //C11	  PUMP3
#define StatusCH1    GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2)		 //D2	  LACK OF WATER
#define StatusCH10   GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_8)	     //D8	  PUMP5
#define StatusCH11   GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_9)		 //D9	  PUMP6

void Status_GPIO_Config(void);//IO≥ı ºªØ

void Status_Scan(void);

char ErrorOccured(unsigned char channel);

void ClearErrorMsg(void);

void clear_error(void);

uint8 WaterLevelMonitor(void);

void TryToRestoreSysRunning(void);

void RefreshErrorInfo(uint8 ind, uint8 on_off);

uint8 ErrorsExist(void);

void IWDG_Config(uint8_t prv ,uint16_t rlv);

void show_an_err_record(uint8* err);

uint8 switch_value(void);

#ifdef DBG_EVENT_AS_ERROR
	void EventRecorder(uint8* event_str);
	void EventRecorderWithoutTimeStamp(uint8* event_str);
#endif


#endif
