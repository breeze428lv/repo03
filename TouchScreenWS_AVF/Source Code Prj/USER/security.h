
/**********************************************************************************/


#ifndef __SECURITY_H
#define __SECURITY_H

void UpdateMileageStatus(void);
uint8 PassStopperMonitor(void);
void ShowMessage(uint8 level);
void PinMonitor(char* str);
void debugger(char* msg, uint8 cmd);
void ShowCurrentPassWord(uint8 type, uint8 visible);
#endif
