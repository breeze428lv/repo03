#ifndef __USART_H
#define	__USART_H

#include "stm32f10x.h"
#include "macros.h"
#include <stdio.h>

void USART1_Config(unsigned int bound);
void USART2_Config(unsigned int bound);
void NVIC_Configuration1(void);
void NVIC_Configuration2(void);
int fputc(int ch, FILE *f);

#ifdef USE_PWM
void Timerx_Init(u16 arr,u16 psc);
#endif

#ifdef USE_MODBUS
void Timerx_Init(u16 arr,u16 psc);
#endif

#endif /* __USART1_H */
