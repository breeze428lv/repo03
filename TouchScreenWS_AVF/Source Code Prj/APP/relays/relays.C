/********************   **************************
 
**********************************************************************************/
#include "relays.h"

/*
 * 函数名：Relay_GPIO_Config
 * 描述  ：配置LED用到的I/O口
 * 输入  ：无
 * 输出  ：无
 */
void Relay_GPIO_Config(void)
{	

	//定义一个GPIO_InitTypeDef类型的结构体
	 GPIO_InitTypeDef GPIO_InitStructure;

	//PORT-A

	//开启GPIOA的外设时钟
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	//选择要控制的GPIOA引脚															   
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;	
//	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;	

	//设置引脚模式为通用推挽输出
  	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

	//设置引脚速率为50MHz   
  	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

	//调用库函数，初始化GPIOA
     GPIO_Init(GPIOA, &GPIO_InitStructure);


	//PORT-B

	//开启GPIOB的外设时钟
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	//选择要控制的GPIOB引脚															   
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;	

	//设置引脚模式为通用推挽输出
  	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

	//设置引脚速率为50MHz   
  	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

	//调用库函数，初始化GPIOB
     GPIO_Init(GPIOB, &GPIO_InitStructure);


	//PORT-E
	//开启GPIOE的外设时钟
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

	//选择要控制的GPIOE引脚															   
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1  |  GPIO_Pin_2  | GPIO_Pin_3 | GPIO_Pin_7  | GPIO_Pin_8|\
	 							   GPIO_Pin_9 | GPIO_Pin_10 |  GPIO_Pin_11 |GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;	

	//设置引脚模式为通用推挽输出
  	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

	//设置引脚速率为50MHz   
  	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

	//调用库函数，初始化GPIOF
     GPIO_Init(GPIOE, &GPIO_InitStructure);






	 
    
	
}



