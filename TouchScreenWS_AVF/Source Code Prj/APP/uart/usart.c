/******************** **************************

**********************************************************************************/
#include "usart.h"
#include "misc.h"
#include "macros.h"
/*
 * ��������USART1_Config
 * ����  ��USART1 GPIO ����,����ģʽ���á�
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
 */
 
void USART1_Config(unsigned int bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	// ���ô���1 ��USART1�� ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
	//����GPIO�˿����� 
    // ���ô���1 ��USART1 Tx (PA.09)�� 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
  
	// ���ô���1 USART1 Rx (PA.10)   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//����1ģʽ��USART1 mode������
	USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�����ж�
	
	USART_Cmd(USART1, ENABLE); //ʹ�ܴ��� 
}


//Usart1 NVIC ����
void NVIC_Configuration1(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	// Configure the NVIC Preemption Priority Bits *  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
	// ʹ�ܴ���1�ж� *
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;

	//IRQͨ��ʹ��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���USART1
	NVIC_Init(&NVIC_InitStructure);
}
		  

 void USART2_Config(unsigned int bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	// ���ô���2 ��USART2�� ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	//����GPIO�˿����� 
  // ���ô���2 ��USART2 Tx (PA.2)�� 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
  
	// ���ô���2 USART1 Rx (PA.3)   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//����2ģʽ��USART1 mode������
	USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�����ж�
	
	USART_Cmd(USART2, ENABLE); //ʹ�ܴ��� 
}


//Usart2 NVIC ����
void NVIC_Configuration2(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	// Configure the NVIC Preemption Priority Bits *  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
	// ʹ�ܴ���2�ж� *
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	//IRQͨ��ʹ��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���USART1
	NVIC_Init(&NVIC_InitStructure);
}


#ifdef USE_MODBUS
//ͨ�ö�ʱ���жϳ�ʼ��
//����ʱ��ѡ��ΪAPB1��2������APB1Ϊ24M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//����ʹ�õ��Ƕ�ʱ��3
 void Timerx_Init(u16 arr,u16 psc)
 {
     	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
 
    	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
 
     	TIM_TimeBaseStructure.TIM_Period = arr + 1; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ 
     	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��  
     	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
     	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
     	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
      
     	TIM_ITConfig(  //ʹ�ܻ���ʧ��ָ����TIM�ж�
				         TIM3, //TIM2
				         TIM_IT_Update  |  //TIM �ж�Դ
				         TIM_IT_Trigger,   //TIM �����ж�Դ 
				         ENABLE  //��
        );
      
     	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
     	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
     	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
     	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
     	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
 
     	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIMx����
                              
}


#endif

/*
 * ��������fputc
 * ����  ���ض���c�⺯��printf��USART1
 * ����  ����
 * ���  ����
 * ����  ����printf����
 */
int fputc(int ch, FILE *f)
{
#ifdef PRINT_TO_UART
	/* ��Printf���ݷ������� */
	USART_SendData(USART1, (unsigned char) ch);
	while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);
#endif	
	return (ch);
}
