

#include "adc.h"
#include "hw_config.h"
#include "delay.h"
#include "hmi_user_uart.h"


void Adc_GPIO_Config(void)
{
  	GPIO_InitTypeDef GPIO_InitStructure;
	//ʹ��GPIOA��ADC1ͨ��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC1, ENABLE );	  
 
	//��PA0����Ϊģ������                      
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	//��GPIO����Ϊģ������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
}

/*
void  Adc_Config(void)
{ 	
	ADC_InitTypeDef ADC_InitStructure; 
	Adc_GPIO_Config();	
	
   //72M/6=12,ADC���ʱ�䲻�ܳ���14M
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);  
	
	//������ ADC1 ��ȫ���Ĵ�������ΪĬ��ֵ
	ADC_DeInit(ADC1); 
	
   //ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	
	
	//ģ��ת�������ڵ�ͨ��ģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
		
	//ģ��ת�������ڵ���ת��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	
    //ADCת��������������ⲿ��������
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	
	
	//ADC�����Ҷ���
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	
	
	//˳����й���ת����ADCͨ������Ŀ
	ADC_InitStructure.ADC_NbrOfChannel = 1;	
	
	//����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ��� 
	ADC_Init(ADC1, &ADC_InitStructure);	
 
    //ʹ��ָ����ADC1
	ADC_Cmd(ADC1, ENABLE);	
	
	//����ָ����ADC1��У׼�Ĵ���
	ADC_ResetCalibration(ADC1);	
	
	// Gets the selected ADC reset calibration registers status.
	while(ADC_GetResetCalibrationStatus(ADC1));
	
	//��ʼָ��ADC1��У׼
	ADC_StartCalibration(ADC1);	
		
   //��ȡָ��ADC1��У׼����,����״̬��ȴ�
	while(ADC_GetCalibrationStatus(ADC1));	
		
   //ʹ��ָ����ADC1�����ת����������
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		
}	
			*/
 void Adc_Configuration(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	
	ADC_DeInit(ADC1); //������ ADC1 ��ȫ���Ĵ�������Ϊȱʡֵ
		
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; //ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode =ENABLE; //ģ��ת��������ɨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //ģ��ת������������ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //�ⲿ����ת���ر�
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = M; //˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitStructure); //����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���


	//����ָ��ADC�Ĺ�����ͨ�����������ǵ�ת��˳��Ͳ���ʱ��
	//ADC1,ADCͨ��x,�������˳��ֵΪy,����ʱ��Ϊ239.5����

//	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_239Cycles5 );
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_239Cycles5 );

/*
 	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_239Cycles5 );	//PA6  ch0
 	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 2, ADC_SampleTime_239Cycles5 );	//PA7  ch1
 	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 3, ADC_SampleTime_239Cycles5 );	//PB0  ch2
    ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 4, ADC_SampleTime_239Cycles5 );   //PB1  ch3	 */



 	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_239Cycles5 );	 //PC4	ch0   ����ѹ��������
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 2, ADC_SampleTime_239Cycles5 );   //PC5	ch1	  ˮ��Һ��/���ѹ��������
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

	// ����ADC��DMA֧�֣�Ҫʵ��DMA���ܣ������������DMAͨ���Ȳ�����
	ADC_DMACmd(ADC1, ENABLE);

	ADC_Cmd(ADC1, ENABLE); //ʹ��ָ����ADC1
	
	ADC_ResetCalibration(ADC1); //��λָ����ADC1��У׼�Ĵ���
	
	while(ADC_GetResetCalibrationStatus(ADC1)); //��ȡADC1��λУ׼�Ĵ�����״̬,����״̬��ȴ�
	
	
	ADC_StartCalibration(ADC1); //��ʼָ��ADC1��У׼״̬
	
	while(ADC_GetCalibrationStatus(ADC1)); //��ȡָ��ADC1��У׼����,����״̬��ȴ�


}


/*

void Get_Adc_Average(void)
{
	
	float ADC_ConvertedValue; 
    float ADC_ConvertedValueLocal; 
    float temp; 

	//ADC1,ADCͨ��3,�������˳��ֵΪ1,����ʱ��Ϊ239.5����
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_239Cycles5 );	  		
		    
    //ʹ��ָ����ADC1�����ת����������
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);			

	//�ȴ�ת������
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));

	//�������һ��ADC1�������ת�����
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


#include "stm32f10x.h" //���ͷ�ļ�����STM32F10x������Χ�Ĵ�����λ���ڴ�ӳ��Ķ���
#include "eval.h" //ͷ�ļ����������ڡ�������LED�ĺ���������
#include "SysTickDelay.h"
#include "UART_INTERFACE.h"
#include <stdio.h>

#define N 50 //ÿͨ����50��
#define M 12 //Ϊ12��ͨ��

vu16 AD_Value[N][M]; //�������ADCת�������Ҳ��DMA��Ŀ���ַ
vu16 After_filter[M]; //���������ƽ��ֵ֮��Ľ��
int i;



void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //��ΪUSART1�ܽ����Ը��õ���ʽ�ӵ�GPIO���ϵģ�����ʹ�ø�������ʽ���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);



	//PA0/1/2 ��Ϊģ��ͨ����������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //ģ����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//PB0/1 ��Ϊģ��ͨ����������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //ģ����������
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//PC0/1/2/3/4/5 ��Ϊģ��ͨ����������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //ģ����������
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}



void RCC_Configuration(void)
{
	ErrorStatus HSEStartUpStatus;
	
	RCC_DeInit(); //RCC ϵͳ��λ
	RCC_HSEConfig(RCC_HSE_ON); //����HSE
	HSEStartUpStatus = RCC_WaitForHSEStartUp(); //�ȴ�HSE׼����
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
		| RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC1 | RCC_APB2Periph_AFIO |RCC_APB2Periph_USART1, ENABLE ); //ʹ��ADC1ͨ��ʱ�ӣ������ܽ�ʱ��
		
		RCC_ADCCLKConfig(RCC_PCLK2_Div6); //72M/6=12,ADC���ʱ�䲻�ܳ���14M
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //ʹ��DMA����
		
	}
}

void ADC1_Configuration(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	
	ADC_DeInit(ADC1); //������ ADC1 ��ȫ���Ĵ�������Ϊȱʡֵ
		
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; //ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode =ENABLE; //ģ��ת��������ɨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //ģ��ת������������ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //�ⲿ����ת���ر�
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; //ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = M; //˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitStructure); //����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���


	//����ָ��ADC�Ĺ�����ͨ�����������ǵ�ת��˳��Ͳ���ʱ��
	//ADC1,ADCͨ��x,�������˳��ֵΪy,����ʱ��Ϊ239.5����
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

	// ����ADC��DMA֧�֣�Ҫʵ��DMA���ܣ������������DMAͨ���Ȳ�����
	ADC_DMACmd(ADC1, ENABLE);

	ADC_Cmd(ADC1, ENABLE); //ʹ��ָ����ADC1
	
	ADC_ResetCalibration(ADC1); //��λָ����ADC1��У׼�Ĵ���
	
	while(ADC_GetResetCalibrationStatu
	
	s(ADC1)); //��ȡADC1��λУ׼�Ĵ�����״̬,����״̬��ȴ�
	
	
	ADC_StartCalibration(ADC1); //��ʼָ��ADC1��У׼״̬
	
	while(ADC_GetCalibrationStatus(ADC1)); //��ȡָ��ADC1��У׼����,����״̬��ȴ�


}


void DMA_Configuration(void)
{

	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(DMA1_Channel1); //��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR; //DMA����ADC����ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&AD_Value; //DMA�ڴ����ַ

	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; //�ڴ���Ϊ���ݴ����Ŀ�ĵ�
	DMA_InitStructure.DMA_BufferSize = N*M; //DMAͨ����DMA����Ĵ�С

	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //�����ַ�Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //�ڴ��ַ�Ĵ�������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //���ݿ��Ϊ16λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //���ݿ��Ϊ16λ

	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; //������ѭ������ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; //DMAͨ�� xӵ�и����ȼ�
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; //DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��

	DMA_Init(DMA1_Channel1, &DMA_InitStructure); //����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��

}


//������������
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
	return (u16)(advalue * 330 / 4096); //��Ľ��������100���������������С��
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
	DMA_Cmd(DMA1_Channel1, ENABLE); //����DMAͨ��
	while(1)
	{
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);//�ȴ�������ɷ����һλ�������׶�ʧ
		
		filter();
		for(i=0;i<12;i++)
		{
			value[i]= GetVolt(After_filter[i]);
			
			printf("value[%d]:\t%d.%dv\n",i,value[i]/100,value[i]0) ;
			delay_ms(100);
		}
	}

}							*/
