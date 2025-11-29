#ifndef __SOLENOID_H__
#define __SOLENOID_H__

#include "../common/common.h"

STD_HighLow Solenoid_GetState(void);
void Solenoid_OnOff(STD_OnOff on_off_state);
void Solenoid_Init(void);

#endif /* __SOLENOID_H__ */