#ifndef __RELAYS_H
#define	__RELAYS_H

#include "stm32f10x.h"
#include "system.h"


#define dout0       PEout(13)			            //DOUT0	CH1
#define FC_SWITCH   PBout(15)		           		//DOUT1	CH2
#define PUMP1_FC    PBout(14)		                //DOUT2  CH3
#define PUMP1_PF    PBout(13)		                //DOUT3	CH4
#define PUMP2_FC    PBout(12)		                //DOUT4	CH5
#define PUMP2_PF    PBout(11)		                //DOUT5  CH6
#define PUMP3_FC    PBout(10)		                //DOUT6  CH7






//#define ch12 PEout(10)			//DOUT10 CH11
#define RS485_TX_En         PEout(0)			//
#define PUMP4_FC            PEout(1)	
#define PUMP4_PF            PEout(2)
#define PUMP5_PF            PEout(3)	                
#define ValveSwitchToTank   PEout(7)			//DOUT12
#define ValveDecompression  PEout(9)			//DOUT11  
#define ValveIntoTank       PEout(10)		 	//DOUT10 
#define warning_out         PEout(11)			   //DOUT9  CH10
#define AUX_PUMP_FC         PEout(12)	   //DOUT8  CH9

#define PUMP5_FC            PEout(14)
#define PUMP3_PF            PEout(15)	   //DOUT7  CH8
				

#define led_run             PAout(11)

//#define led_run             PAout(0)

void Relay_GPIO_Config(void);   /* LED ¶Ë¿Ú³õÊ¼»¯ */

#endif /* __RELAYS_H */
