
/********************  **************************

**********************************************************************************/
#include "macros.h"

#ifndef __MANUAL_CONTROL_H
#define __MANUAL_CONTROL_H

void BtnResponseManualCtr(uint16 control_id, uint8  state);

void dout1(uint8 state);
void dout2(uint8 state);
void dout3(uint8 state);
void dout4(uint8 state);
void dout5(uint8 state);
void dout6(uint8 state);
void dout7(uint8 state);
void dout8(uint8 state);
void dout9(uint8 state);
void dout10(uint8 state);
void dout11(uint8 state);
void dout12(uint8 state);
void dout13(uint8 state);
void dout14(uint8 state);
void HeartBeating(void);

#ifdef USE_PWM
void PWM_manager(void);
#endif


#endif
