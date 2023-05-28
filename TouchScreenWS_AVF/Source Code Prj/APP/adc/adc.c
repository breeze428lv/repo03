

#include "adc.h"
#include "hw_config.h"
#include "delay.h"
#include "hmi_user_uart.h"


void Adc_GPIO_Config(void)
{
  	GPIO_InitTypeDef GPIO_InitStructure;
	//使能GPIOA和ADC1通道时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC1, ENABLE );	  
 
	//将PA0设置为模拟输入                      
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	//将GPIO设置为模拟输入
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
}

/*
void  Adc_Config(void)
{ 	
	ADC_InitTypeDef ADC_InitStructure; 
	Adc_GPIO_Config();	
	
   //72M/6=12,ADC最大时间不能超过14M
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);  
	
	//将外设 ADC1 的全部寄存器重设为默认值
	ADC_DeInit(ADC1); 
	
   //ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	
	
	//模数转换工作在单通道模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
		
	//模数转换工作在单次转换模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	
    //ADC转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	
	
	//ADC数据右对齐
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	
	
	//顺序进行规则转换的ADC通道的数目
	ADC_InitStructure.ADC_NbrOfChannel = 1;	
	
	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器 
	ADC_Init(ADC1, &ADC_InitStructure);	
 
    //使能指定的ADC1
	ADC_Cmd(ADC1, ENABLE);	
	
	//重置指定的ADC1的校准寄存器
	ADC_ResetCalibration(ADC1);	
	
	// Gets the selected ADC reset calibration registers status.
	while(ADC_GetResetCalibrationStatus(ADC1));
	
	//开始指定ADC1的校准
	ADC_StartCalibration(ADC1);	
		
   //获取指定ADC1的校准程序,设置状态则等待
	while(ADC_GetCalibrationStatus(ADC1));	
		
   //使能指定的ADC1的软件转换启动功能
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		
}	
			*/
 void Adc_Configuration(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	
	ADC_DeInit(ADC1); //将外设 ADC1 的全部寄存器重设为缺省值
		
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; //ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode =ENABLE; //模数转换工作在扫描模式
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //模数转换工作在连续转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //外部触发转换关闭
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = M; //顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure); //根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器


	//设置指定ADC的规则组通道，设置它们的转化顺序和采样时间
	//ADC1,ADC通道x,规则采样顺序值为y,采样时间为239.5周期

//	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_239Cycles5 );
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_239Cycles5 );

/*
 	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_239Cycles5 );	//PA6  ch0
 	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 2, ADC_SampleTime_239Cycles5 );	//PA7  ch1
 	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 3, ADC_SampleTime_239Cycles5 );	//PB0  ch2
    ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 4, ADC_SampleTime_239Cycles5 );   //PB1  ch3	 */



 	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_239Cycles5 );	 //PC4	ch0   出口压力变送器
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 2, ADC_SampleTime_239Cycles5 );   //PC5	ch1	  水箱液面/入口压力变送器
 	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 3, ADC_SampleTime_239Cycles5 ); 	 //PC0  ch2	  System Temperature	  
 // ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 4, ADC_SampleTime_239Cycles5 );   //PC1  ch3   
 	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 4, ADC_SampleTime_239Cycles5 );   //PC2  ch4   -Target Pressure Setting  20210702
 /*	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 6, ADC_SampleTime_239Cycles5 );	 //PC3  ch5   pump4 temperature	 
 	ADC_RegularChannelConfig(ADC1, ADC_Channel_0 , 8, ADC_SampleTime_239Cycles5 );   //PA0	ch6   pump5 temperature
  	ADC_RegularChannelConfig(ADC1, ADC_Channel_1 , 7, ADC_SampleTime_239Cycles5 );   //PA1	ch7   pump6 temperature

	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 7, ADC_SampleTime_239Cycles5 );
 	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 8, ADC_SampleTime_239Cycles5 );
 	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 9, ADC_SampleTime_239Cycles5 );			 
 	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 10, ADC_SampleTime_239Cycles5 );	*/

	// 开启ADC的DMA支持（要实现DMA功能，还需独立配置DMA通道等参数）
	ADC_DMACmd(ADC1, ENABLE);

	ADC_Cmd(ADC1, ENABLE); //使能指定的ADC1
	
	ADC_ResetCalibration(ADC1); //复位指定的ADC1的校准寄存器
	
	while(ADC_GetResetCalibrationStatus(ADC1)); //获取ADC1复位校准寄存器的状态,设置状态则等待
	
	
	ADC_StartCalibration(ADC1); //开始指定ADC1的校准状态
	
	while(ADC_GetCalibrationStatus(ADC1)); //获取指定ADC1的校准程序,设置状态则等待


}


/*

void Get_Adc_Average(void)
{
	
	float ADC_ConvertedValue; 
    float ADC_ConvertedValueLocal; 
    float temp; 

	//ADC1,ADC通道3,规则采样顺序值为1,采样时间为239.5周期
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_239Cycles5 );	  		
		    
    //使能指定的ADC1的软件转换启动功能
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);			

	//等待转换结束
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));

	//返回最近一次ADC1规则组的转换结果
	ADC_ConvertedValue=ADC_GetConversionValue(ADC1);		
//	printf("\r\n ADC_CH0_VAL = 0x%04X \r\n", ADC_ConvertedValue); 	
	temp=(float)ADC_ConvertedValue*(3.3/4096);
	ADC_ConvertedValueLocal=temp;
//	printf("\r\n ADC_CH0_VOL = %f V \r\n",ADC_ConvertedValueLocal); 	

}	
	   */

/*
#include "adc.h"
#include "hw_config.h"
#include "delay.h"


#include "stm32f10x.h" //这个头文件包括STM32F10x所有外围寄存器、位、内存映射的定义
#include "eval.h" //头文件（包括串口、按键、LED的函数声明）
#include "SysTickDelay.h"
#include "UART_INTERFACE.h"
#include <stdio.h>

#define N 50 //每通道采50次
#define M 12 //为12个通道

vu16 AD_Value[N][M]; //用来存放ADC转换结果，也是DMA的目标地址
vu16 After_filter[M]; //用来存放求平均值之后的结果
int i;



void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //因为USART1管脚是以复用的形式接到GPIO口上的，所以使用复用推挽式输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);



	//PA0/1/2 作为模拟通道输入引脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //模拟输入引脚
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//PB0/1 作为模拟通道输入引脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //模拟输入引脚
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//PC0/1/2/3/4/5 作为模拟通道输入引脚
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //模拟输入引脚
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}



void RCC_Configuration(void)
{
	ErrorStatus HSEStartUpStatus;
	
	RCC_DeInit(); //RCC 系统复位
	RCC_HSEConfig(RCC_HSE_ON); //开启HSE
	HSEStartUpStatus = RCC_WaitForHSEStartUp(); //等待HSE准备好
	if(HSEStartUpStatus == SUCCESS)
	{
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable); //Enable Prefetch Buffer
		FLASH_SetLatency(FLASH_Latency_2); //Set 2 Latency cycles
		RCC_HCLKConfig(RCC_SYSCLK_Div1); //AHB clock = SYSCLK
		RCC_PCLK2Config(RCC_HCLK_Div1); //APB2 clock = HCLK
		RCC_PCLK1Config(RCC_HCLK_Div2); //APB1 clock = HCLK/2
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_6); //PLLCLK = 12MHz * 6 = 72 MHz
		RCC_PLLCmd(ENABLE); //Enable PLL

		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET); //Wait till PLL is ready
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); //Select PLL as system clock source
	
		while(RCC_GetSYSCLKSource() != 0x08); //Wait till PLL is used as system clock source		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB
		| RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC1 | RCC_APB2Periph_AFIO |RCC_APB2Periph_USART1, ENABLE ); //使能ADC1通道时钟，各个管脚时钟
		
		RCC_ADCCLKConfig(RCC_PCLK2_Div6); //72M/6=12,ADC最大时间不能超过14M
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //使能DMA传输
		
	}
}

void ADC1_Configuration(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	
	ADC_DeInit(ADC1); //将外设 ADC1 的全部寄存器重设为缺省值
		
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; //ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode =ENABLE; //模数转换工作在扫描模式
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //模数转换工作在连续转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //外部触发转换关闭
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = M; //顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure); //根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器


	//设置指定ADC的规则组通道，设置它们的转化顺序和采样时间
	//ADC1,ADC通道x,规则采样顺序值为y,采样时间为239.5周期
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 5, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 6, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 7, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 8, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 9, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 10, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 11, ADC_SampleTime_239Cycles5 );
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 12, ADC_SampleTime_239Cycles5 );

	// 开启ADC的DMA支持（要实现DMA功能，还需独立配置DMA通道等参数）
	ADC_DMACmd(ADC1, ENABLE);

	ADC_Cmd(ADC1, ENABLE); //使能指定的ADC1
	
	ADC_ResetCalibration(ADC1); //复位指定的ADC1的校准寄存器
	
	while(ADC_GetResetCalibrationStatu
	
	s(ADC1)); //获取ADC1复位校准寄存器的状态,设置状态则等待
	
	
	ADC_StartCalibration(ADC1); //开始指定ADC1的校准状态
	
	while(ADC_GetCalibrationStatus(ADC1)); //获取指定ADC1的校准程序,设置状态则等待


}


void DMA_Configuration(void)
{

	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(DMA1_Channel1); //将DMA的通道1寄存器重设为缺省值
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR; //DMA外设ADC基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&AD_Value; //DMA内存基地址

	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; //内存作为数据传输的目的地
	DMA_InitStructure.DMA_BufferSize = N*M; //DMA通道的DMA缓存的大小

	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //数据宽度为16位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //数据宽度为16位

	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; //工作在循环缓存模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; //DMA通道 x拥有高优先级
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; //DMA通道x没有设置为内存到内存传输

	DMA_Init(DMA1_Channel1, &DMA_InitStructure); //根据DMA_InitStruct中指定的参数初始化DMA的通道

}


//配置所有外设
void Init_All_Periph(void)
{

	RCC_Configuration();
	
	GPIO_Configuration();
	
	ADC1_Configuration();
	
	DMA_Configuration();
	
	//USART1_Configuration();
	USART_Configuration(9600);


}



u16 GetVolt(u16 advalue)
{
	return (u16)(advalue * 330 / 4096); //求的结果扩大了100倍，方便下面求出小数
}

void filter(void)
{
	int sum = 0;
	u8 count;
	for(i=0;i<12;i++)	
	{	
		for ( count=0;count<N;count++)		
		{
		
			sum += AD_Value[count][i];
		
		}
		
		After_filter[i]=sum/N;		
		sum=0;
	}
}

int main(void)
{

	u16 value[M];
	
	init_All_Periph();
	SysTick_Initaize();
	
	
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	DMA_Cmd(DMA1_Channel1, ENABLE); //启动DMA通道
	while(1)
	{
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);//等待传输完成否则第一位数据容易丢失
		
		filter();
		for(i=0;i<12;i++)
		{
			value[i]= GetVolt(After_filter[i]);
			
			printf("value[%d]:\t%d.%dv\n",i,value[i]/100,value[i]0) ;
			delay_ms(100);
		}
	}

}							*/
