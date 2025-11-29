#include "../common/common.h"
#include "OneWire.h"
#include "DallasTemperature.h"

#define T0_KNOB   PIN_A0
// #define T1_KNOB   PIN_A1
#define T0_KNOB_MAX_ADC     ((uint16_t)1023)
#define TEMPERATURE_REF_MAX ((int8_t)100)
#define TEMPERATURE_REF_MIN ((int8_t)-20)

#define DS18B20_PIN 8
#define TEMPERATURE_REQUEST_TIME_PERIOD ((uint16_t)1000U)

/* Private variables */
static OneWire oneWire(DS18B20_PIN);
static DallasTemperature sensors(&oneWire);

static uint16_t u16_cyclic_time = 0U;

float Temperature_GetAcc(void)
{
    return sensors.getTempCByIndex(0);
}

int16_t Temperature_GetRef(void)
{
    uint16_t lu16_adc = 0U;
    int16_t li16_temp = 0;
    
    /* Read ADC value of current temperature */
    lu16_adc = analogRead(T0_KNOB);

    /* Convert adc to desired temperature */
    li16_temp = ((lu16_adc * TEMPERATURE_REF_MAX) / T0_KNOB_MAX_ADC) + TEMPERATURE_REF_MIN;

    return lu16_adc;
}

void Temperature_Init(void)
{
    u16_cyclic_time = TEMPERATURE_REQUEST_TIME_PERIOD;
    /* Select temperature sensro from bus */
    sensors.begin();
}

void Temperature_Cyclic(void)
{
    if (u16_cyclic_time > 0U)
    {
        u16_cyclic_time--;
    }
    else
    {
        /* Request temperature */
        sensors.requestTemperatures();

        /* Update variables */
        u16_cyclic_time = TEMPERATURE_REQUEST_TIME_PERIOD;
    }
}
