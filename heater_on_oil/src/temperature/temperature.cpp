#include "../common/common.h"
#include "OneWire.h"
#include "DallasTemperature.h"

#define T0_KNOB   PIN_A0
// #define T1_KNOB   PIN_A1
#define T0_KNOB_MAX_ADC     ((uint16_t)1023U)
#define TEMPERATURE_REF_MAX ((uint8_t)100U)

#define DS18B20_PIN 8
#define TEMPERATURE_REQUEST_TIME_PERIOD ((uint16_t)1000U)

/* Private variables */
static OneWire oneWire(DS18B20_PIN);
static DallasTemperature sensors(&oneWire);

static uint16_t u16_cyclic_time = 0U;
static float gf_acc_temp;

float Temperature_GetAcc(void)
{
    return gf_acc_temp;
}

uint8_t Temperature_GetRef(void)
{
    uint16_t lu16_adc = 0U;
    uint8_t lu08_temp = 0;
    
    /* Read ADC value of current temperature */
    lu16_adc = analogRead(T0_KNOB);

    /* Convert adc to desired temperature */
    lu08_temp = (uint8_t)(((uint32_t)lu16_adc * TEMPERATURE_REF_MAX) / T0_KNOB_MAX_ADC);

    return lu08_temp;
}

uint16_t Temperature_GetRawRef(void)
{
    uint16_t lu16_adc = 0U;

    /* Read ADC value of current temperature */
    lu16_adc = analogRead(T0_KNOB);

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
        gf_acc_temp = sensors.getTempCByIndex(0);

        /* Update variables */
        u16_cyclic_time = TEMPERATURE_REQUEST_TIME_PERIOD;
    }
}
