/******************** (C) COPYRIGHT 2018 STMicroelectronics ********************
* File Name          : hw_config.c
* Author             : MCD Application Team
* Version            : V3.1.1
* Date               : 21/08/2018
* Description        : Hardware Configuration & Setup
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "hw_config.h"
#include "hmi_user_uart.h"
#include "adc.h"

uint16 AD_Value[N][M]; //用来存放ADC转换结果，也是DMA的目标地址
uint16 After_filter[M]; //用来存放求平均值之后的结果



ErrorStatus HSEStartUpStatus;
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

USART_InitTypeDef USART_InitStructure;



/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : Set_System
* Description    : Configures Main system clocks & power
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_System_Clocks(void)
{


    // SYSCLK, HCLK, PCLK2 and PCLK1 configuration 
    // RCC system reset(for debug purpose) 
    RCC_DeInit();
    //使用内部的晶振
    RCC_HSICmd(ENABLE);
    while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET)
    {
    }
    if(1)
    {

        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);	     	//Enable FLASH_PrefetchBuffer
        FLASH_SetLatency(FLASH_Latency_2);					        	//FLASH_Latency:2

        RCC_HCLKConfig(RCC_SYSCLK_Div1);                                 //AHB时钟	   48M

        RCC_PCLK2Config(RCC_HCLK_Div1);                                  //APB2时钟

        RCC_PCLK1Config(RCC_HCLK_Div2);                                  //ABP1时钟	   24M

        //设置 PLL 时钟源及倍频系数
        RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_12);            //4M*12 =48M


        //使能或者失能 PLL,这个参数可以取：ENABLE或者DISABLE
        RCC_PLLCmd(ENABLE);//如果PLL被用于系统时钟,那么它不能被失能


        //等待指定的 RCC 标志位设置成功 等待PLL初始化成功
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
        {
        }

        //设置系统时钟（SYSCLK） 设置PLL为系统时钟源
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

        //等待PLL成功用作于系统时钟的时钟源
        //  0x00：HSI 作为系统时钟
        //  0x04：HSE作为系统时钟
        //  0x08：PLL作为系统时钟
        while(RCC_GetSYSCLKSource() != 0x08)
        {
        }
    }
}

void RCC_Configuration(void)
{

	//Enable clock for GPIOA. GPIOB, ADC1, USART1, AFIO 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD
                          |RCC_APB2Periph_ADC1 |RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE ); 
	//Enable clock for USART2, Timer3
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC|RCC_APB1Periph_USART2 | RCC_APB1Periph_TIM3, ENABLE);
	
	//Enable clock for DMA1
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); //72M/6=12,ADC最大时间不能超过14M
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //使能DMA传输
		
}

void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

    //USART1---------------------------	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //因为USART1管脚是以复用的形式接到GPIO口上的，所以使用复用推挽式输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//USART2----------------------------
  		//USART2 Tx (PA.2)） 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
  
		//USART1 Rx (PA.3)   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

    //ADC---------------
		//PA1 作为模拟通道输入引脚	AIN1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //模拟输入引脚
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		//PC0/1/2 作为模拟通道输入引脚AIN10/AIN11/AIN12
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; //模拟输入引脚
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//DAC---------------
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;				 // 端口配置
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; 		 //模拟输入
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_4)	;//PA.4 输出高

 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;				 // 端口配置
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; 		  
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_5)	;//PA.5 输出高	
		
	//PWM TIM3-CH1~CH4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;		  //A6/A7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;	 	  //B0/B1
	GPIO_Init(GPIOB, &GPIO_InitStructure);

}

/*---------------------------------------------------------------------
     TIM3 Configuration: generate 4 PWM signals with 4 different duty cycles:
     The TIM3CLK frequency is set to SystemCoreClock (Hz), to get TIM3 counter
     clock at 24 MHz the Prescaler is computed as following:
      - Prescaler = (TIM3CLK / TIM3 counter clock) - 1
     SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density
     and Connectivity line devices and to 24 MHz for Low-Density Value line and
     Medium-Density Value line devices

     The TIM3 is running at 36 KHz: TIM3 Frequency = TIM3 counter clock/(ARR + 1)
                                                   = 24 MHz / 666 = 36 KHz
     TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
     TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 = 37.5%
     TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR)* 100 = 25%
     TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR)* 100 = 12.5%
   ----------------------------------------------------------------------- */
void TIM3_PWM_Configuration(void)
{
	 /* Private typedef -----------------------------------------------------------*/
	 /* Private define ------------------------------------------------------------*/
	 /* Private macro -------------------------------------------------------------*/
	 /* Private variables ---------------------------------------------------------*/
	 TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	 TIM_OCInitTypeDef  TIM_OCInitStructure;
	 uint16_t CCR1_Val = 0;
	 uint16_t CCR2_Val = 0;
	 uint16_t CCR3_Val = 0;
	 uint16_t CCR4_Val = 0;
	 uint16_t PrescalerValue = 0;
	
//	printf("SystemCoreClock = %d\n",SystemCoreClock);

	/* Compute the prescaler value */
	PrescalerValue = (uint16_t) (SystemCoreClock / 24000000) - 1;
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 665;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	/* PWM1 Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	/* PWM1 Mode configuration: Channel2 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR2_Val;
	
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	/* PWM1 Mode configuration: Channel3 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR3_Val;
	
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	/* PWM1 Mode configuration: Channel4 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = CCR4_Val;
	
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	
	/* TIM3 enable counter */
	TIM_Cmd(TIM3, ENABLE);

}

/**
  * @brief  None
  * @param  None
  * @retval  None
  */
void UpdatePWM(uint16 ccr_val, uint8 n)
{
	switch(n)								
	{								
	    case   2:								
			TIM3->CCR1 = ccr_val;							
	        break;								
	    case   3:								
	        TIM3->CCR2 = ccr_val;									
	        break;								
	    case   4:								
	        TIM3->CCR3 = ccr_val;									
	        break;								
	    case   5:								
	        TIM3->CCR4 = ccr_val;									
	        break;								
	 				
	    default:								
	        break;								
	}								

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

	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; //工作在环形缓存模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; //DMA通道 x拥有高优先级
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; //DMA通道x没有设置为内存到内存传输

	DMA_Init(DMA1_Channel1, &DMA_InitStructure); //根据DMA_InitStruct中指定的参数初始化DMA的通道

}

/*******************************************************************************
* Function Name  : USB_Interrupts_Config
* Description    : Configures the USB interrupts
* Input          : None.
* Return         : None.
*******************************************************************************/

void Interrupts_Config1(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    //Enable USART Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
}			
						
void Interrupts_Config2(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    // Enable USART Interrupt 
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
}

void Interrupts_Config(void)
{
	Interrupts_Config1();
	Interrupts_Config2();
	
}

//配置所有外设
void Init_All_Hardware(void)
{
	Set_System_Clocks();

	RCC_Configuration();
	
	GPIO_Configuration();
	
	Adc_Configuration();
	
	DMA_Configuration();
	
	//USART1_Configuration();
//	USART_Configuration(9600);


}
/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
