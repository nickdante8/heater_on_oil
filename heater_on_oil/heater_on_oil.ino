#include "stdint.h"
#include "src/appl/appl.h"
#include "src/temperature/temperature.h"

/* Setup and configuration */
void setup() {
  Appl_Init();
}

/* Main loop */
void loop() {
    static volatile uint32_t prevTime = 0U;
    static volatile uint32_t currTime = 0U;

    /* Get current time */
    currTime = millis();

    /* Check if 1ms has passed */
    if (currTime != prevTime)
    {
        /* Update tick variables */
        prevTime = currTime;
        /* Cyclic temperature update */
        Temperature_Cyclic();
        /* Call cyclical functions */
        Appl_Cyclic();
    }
}
