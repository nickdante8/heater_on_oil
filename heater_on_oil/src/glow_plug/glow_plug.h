#ifndef __GLOW_PLUG_H__
#define __GLOW_PLUG_H__

#include "../common/common.h"

uint16_t GlwoPlug_GetCurrentPwm(void);
void GlowPlug_SetPwm(uint16_t u16_pwm);
void GlowPlug_OnOff(STD_OnOff on_off_state);

void GlowPlug_Init(void);

#endif /* __GLOW_PLUG_H__ */