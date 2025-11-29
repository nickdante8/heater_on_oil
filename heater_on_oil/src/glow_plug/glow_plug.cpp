#include "glow_plug.h"

#define GLOW_PLUG_PIN   5

#define GLOW_PLUG_POWER_OFF     ((uint8_t)0U)
#define GLOW_PLUG_POWER_ON_MAX  ((uint8_t)255U)

typedef struct glow_plug
{
    uint16_t u16_temp;
    uint8_t u8_pwm;
} ts_glow_plug;

static ts_glow_plug glow_plug_inst;

uint16_t GlwoPlug_GetCurrentPwm(void)
{
    return glow_plug_inst.u8_pwm;
}

void GlowPlug_SetPwm(uint16_t u16_pwm)
{
    if (u16_pwm > GLOW_PLUG_POWER_ON_MAX)
    {
        u16_pwm = GLOW_PLUG_POWER_ON_MAX;
    }

    analogWrite(GLOW_PLUG_PIN, u16_pwm);
}

void GlowPlug_OnOff(STD_OnOff on_off_state)
{
    if (on_off_state == STD_ON)
    {
        analogWrite(GLOW_PLUG_PIN, (int)GLOW_PLUG_POWER_ON_MAX);
    }
    else
    {
        analogWrite(GLOW_PLUG_PIN, (int)GLOW_PLUG_POWER_OFF);
    }
}

void GlowPlug_Init(void)
{
    pinMode(GLOW_PLUG_PIN, OUTPUT);
    GlowPlug_OnOff(STD_OFF);
}
