#ifndef __ADC_H
#define __ADC_H	
#include "stm32f10x.h"
 

/*
#define ADC_CH0  0 //通道0
#define ADC_CH1  1 //通道1
#define ADC_CH2  2 //通道2
#define ADC_CH3  3 //通道3	   
*/
#define N 50           // 每通道采50次
#define M 4            // 4个通道
/*
#define N 50//10//50 //每通道采50次
#define M 10//为10个通道		*/

void Adc_GPIO_Config(void); 
void Adc_Configuration(void);
void DMA_Configuration(void);


 
#endif 
