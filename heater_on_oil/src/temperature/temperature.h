#ifndef __TEMPERATURE_H__
#define __TEMPERATURE_H__

#include "../common/common.h"

float Temperature_GetAcc(void);
uint16_t Temperature_GetRef(void);
void Temperature_Init(void);
void Temperature_Cyclic(void);

#endif /* __TEMPERATURE_H__ */