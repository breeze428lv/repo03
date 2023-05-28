/******************** **************************

**********************************************************************************/
#include "usart.h"
#include "misc.h"
#include "macros.h"
/*
 * 函数名：USART1_Config
 * 描述  ：USART1 GPIO 配置,工作模式配置。
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */
 
void USART1_Config(unsigned int bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	// 配置串口1 （USART1） 时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
	//串口GPIO端口配置 
    // 配置串口1 （USART1 Tx (PA.09)） 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
  
	// 配置串口1 USART1 Rx (PA.10)   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//串口1模式（USART1 mode）配置
	USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启中断
	
	USART_Cmd(USART1, ENABLE); //使能串口 
}


//Usart1 NVIC 配置
void NVIC_Configuration1(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	// Configure the NVIC Preemption Priority Bits *  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
	// 使能串口1中断 *
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;

	//IRQ通道使能
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器USART1
	NVIC_Init(&NVIC_InitStructure);
}
		  

 void USART2_Config(unsigned int bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	// 配置串口2 （USART2） 时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	//串口GPIO端口配置 
  // 配置串口2 （USART2 Tx (PA.2)） 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
  
	// 配置串口2 USART1 Rx (PA.3)   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//串口2模式（USART1 mode）配置
	USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启中断
	
	USART_Cmd(USART2, ENABLE); //使能串口 
}


//Usart2 NVIC 配置
void NVIC_Configuration2(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	// Configure the NVIC Preemption Priority Bits *  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
	// 使能串口2中断 *
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	//IRQ通道使能
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器USART1
	NVIC_Init(&NVIC_InitStructure);
}


#ifdef USE_MODBUS
//通用定时器中断初始化
//这里时钟选择为APB1的2倍，而APB1为24M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器3
 void Timerx_Init(u16 arr,u16 psc)
 {
     	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
 
    	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
 
     	TIM_TimeBaseStructure.TIM_Period = arr + 1; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值 
     	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
     	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
     	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
     	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
      
     	TIM_ITConfig(  //使能或者失能指定的TIM中断
				         TIM3, //TIM2
				         TIM_IT_Update  |  //TIM 中断源
				         TIM_IT_Trigger,   //TIM 触发中断源 
				         ENABLE  //管
        );
      
     	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
     	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
     	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
     	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
     	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
 
     	TIM_Cmd(TIM3, ENABLE);  //使能TIMx外设
                              
}


#endif

/*
 * 函数名：fputc
 * 描述  ：重定向c库函数printf到USART1
 * 输入  ：无
 * 输出  ：无
 * 调用  ：由printf调用
 */
int fputc(int ch, FILE *f)
{
#ifdef PRINT_TO_UART
	/* 将Printf内容发往串口 */
	USART_SendData(USART1, (unsigned char) ch);
	while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);
#endif	
	return (ch);
}
