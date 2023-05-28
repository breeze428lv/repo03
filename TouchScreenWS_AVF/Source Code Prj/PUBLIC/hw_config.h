/*! 
*  \file hmi_driver.h
*  \brief ϵͳʱ�����ã��ж�����
*  \version 1.0
*  \date 2012-2018
*  \copyright ���ݴ�ʹ��Ƽ����޹�˾
*/
#ifndef _HW_CONFIG_H
#define _HW_CONFIG_H

void Set_System_Clocks(void);
void Interrupts_Config(void);
void RCC_Configuration(void);
void GPIO_Configuration(void);
void TIM3_PWM_Configuration(void);
void UpdatePWM( unsigned short int ccr_val, unsigned char n);
void UpdatePumpFreq(float freq, unsigned char n);




#endif
