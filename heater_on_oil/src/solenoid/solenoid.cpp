#include "../common/common.h"

#define SOLENOID_L298M_ENA   3
#define SOLENOID_L298M_IN1   2
#define SOLENOID_L298M_IN2   4

#define SOLENOID_POWER_OFF     ((uint8_t)0U)
#define SOLENOID_POWER_ON_MAX  ((uint8_t)255U)

void Solenoid_OnOff(STD_OnOff on_off_state)
{
    if (on_off_state == STD_ON)
    {
        analogWrite(SOLENOID_L298M_ENA, (int)SOLENOID_POWER_ON_MAX);
    }
    else
    {
        analogWrite(SOLENOID_L298M_ENA, (int)SOLENOID_POWER_OFF);
    }
}

void Solenoid_Init(void)
{
    /* Pin configuration */
    pinMode(SOLENOID_L298M_ENA, OUTPUT);
    pinMode(SOLENOID_L298M_IN1, OUTPUT);
    pinMode(SOLENOID_L298M_IN2, OUTPUT);
    /* Set solenoid direction and turn it off */
    digitalWrite(SOLENOID_L298M_IN1, HIGH);
    digitalWrite(SOLENOID_L298M_IN2, LOW);
    analogWrite(SOLENOID_L298M_ENA, 0);
}
