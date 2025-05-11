#include "stdint.h"
#include "DS18B20.h"

/* Global defines */
#define T0_KNOB   PIN_A0
#define T1_KNOB   PIN_A1
#define L298M_ENA   3
#define L298M_IN1   2
#define L298M_IN2   4
#define DS18B20_PIN 5

#define PWM_MAX_VALUE   ((uint8_t)255U)

#define TERMINAL_BUFFER_SIZE  ((uint16_t)1024U)
#define TERMINAL_INTERFACE    Serial

#define APPL_CYCLIC_TIME_PERIOD ((uint16_t)1000U)

#define APPL_STATE_ANALOG                   '0'
#define APPL_STATE_DS18B20                  '1'
#define APPL_STATE_SOLENOID_CTRL_FULL_ON    '2'
#define APPL_STATE_SOLENOID_CTRL_FULL_OFF   '3'
#define APPL_STATE_SOLENOID_CTRL_RISE_FALL  '4'
#define APPL_STATE_PRINT_MENU               '5'
#define APPL_STATE_IDLE_STATE               '6'

/* Data type */
typedef struct appl_type
{
  uint16_t u16_applc_cyclic_time;
  uint8_t u08_pwm_power;
  uint8_t u08_temp_id;
  char    ch_menuState;
  char    ch_data_stream[TERMINAL_BUFFER_SIZE];
  union
  {
    struct
    {
      uint8_t b0_printOnce  :1;
      uint8_t bx_reserved   :7;
    } f;
    uint8_t u08_flags;
  } status;
} ts_appl_type;

/* Global variables */
/* Knob values */
static uint16_t gu16_t0_knob = 0U;
static uint16_t gu16_t1_knob = 0U;

/* Temperature sensor */
DS18B20 ds(DS18B20_PIN);

/* Data stream sent to terminal */
static ts_appl_type appl_inst;

/* Setup and configuration */
void setup() {
  /* Temperature sensor addresses */
  uint8_t lu8_ds18b20_address[] = {40, 250, 31, 218, 4, 0, 0, 52};
  /* Variable set */
  appl_inst.u16_applc_cyclic_time = APPL_CYCLIC_TIME_PERIOD;
  appl_inst.u08_pwm_power = PWM_MAX_VALUE;
  appl_inst.ch_menuState = APPL_STATE_PRINT_MENU;
  appl_inst.status = { 1 };
  /* Pin configuration */
  pinMode(L298M_ENA, OUTPUT);
  pinMode(L298M_IN1, OUTPUT);
  pinMode(L298M_IN2, OUTPUT);
  /* Set solenoid direction and turn it off */
  digitalWrite(L298M_IN1, HIGH);
  digitalWrite(L298M_IN2, LOW);
  analogWrite(L298M_ENA, 0);
  /* Configure Uart to see output results */
  TERMINAL_INTERFACE.begin(115200);
  /* Select temperature sensro from bus */
  appl_inst.u08_temp_id = ds.select(lu8_ds18b20_address);
  if (appl_inst.u08_temp_id)
  {
    TERMINAL_INTERFACE.print("Temperature device found with address ");
    TERMINAL_INTERFACE.println(appl_inst.u08_temp_id);
  }
  else
  {
      TERMINAL_INTERFACE.print("Temperature device not found!");
  }
}

void Appl_Cyclic(void)
{
  /* Read menu input mode */
  if (TERMINAL_INTERFACE.available())
  {
    TERMINAL_INTERFACE.readBytes(&appl_inst.ch_menuState, 1);
  }

  /* Application's state machine */
  switch(appl_inst.ch_menuState)
  {
    case APPL_STATE_ANALOG: /* Analog valuse */
    {
      /* Decrement application cyclic time preiod */
      if (appl_inst.u16_applc_cyclic_time > 0U)
      {
        appl_inst.u16_applc_cyclic_time--;
      }
      else
      {
        /* Reset cyclic timer value */
        appl_inst.u16_applc_cyclic_time = APPL_CYCLIC_TIME_PERIOD;
        /* Read analog values of the knobs */
        gu16_t0_knob = analogRead(T0_KNOB);
        gu16_t1_knob = analogRead(T1_KNOB);
        /* Print result and visualize it */
        sprintf(appl_inst.ch_data_stream, "%4.1d %4.1d\n\r", gu16_t0_knob, gu16_t1_knob);
        /* Print buffer */
        TERMINAL_INTERFACE.write(appl_inst.ch_data_stream);
      }
      break;
    }

    case APPL_STATE_DS18B20: /* Temperatur sensor data */
    {
      /* Decrement application cyclic time preiod */
      if (appl_inst.u16_applc_cyclic_time > 0U)
      {
        appl_inst.u16_applc_cyclic_time--;
      }
      else
      {
        /* Reset cyclic timer value */
        appl_inst.u16_applc_cyclic_time = APPL_CYCLIC_TIME_PERIOD;
        /* Get temperature */
        TERMINAL_INTERFACE.println(ds.getTempC());
      }
      break;
    }

    case APPL_STATE_SOLENOID_CTRL_FULL_ON: /* Solenoid valve control - full power */
    {
      /* Set full pwm power */
      appl_inst.u08_pwm_power = PWM_MAX_VALUE;
      analogWrite(L298M_ENA, appl_inst.u08_pwm_power);
      /* Go to idle state */
      appl_inst.ch_menuState = APPL_STATE_IDLE_STATE;
      break;
    }

    case APPL_STATE_SOLENOID_CTRL_FULL_OFF: /* Solenoid valve control - off */
    {
      /* Set full pwm power */
      appl_inst.u08_pwm_power = 0U;
      analogWrite(L298M_ENA, appl_inst.u08_pwm_power);
      /* Go to idle state */
      appl_inst.ch_menuState = APPL_STATE_IDLE_STATE;
      break;
    }

    case APPL_STATE_SOLENOID_CTRL_RISE_FALL: /* Solenoid valve control - increase/decrease */
    {
      /* Set driver direction and PWM */
      digitalWrite(L298M_IN1, HIGH);
      digitalWrite(L298M_IN2, LOW);
      /* Decrement application cyclic time preiod */
      if (appl_inst.u16_applc_cyclic_time > 0U)
      {
        appl_inst.u16_applc_cyclic_time--;
      }
      else
      {
        /* Reset cyclic timer value */
        appl_inst.u16_applc_cyclic_time = APPL_CYCLIC_TIME_PERIOD;
        /* PWM power update */
        appl_inst.u08_pwm_power -= 10U;
        analogWrite(L298M_ENA, appl_inst.u08_pwm_power);
      }
      break;
    }

    case APPL_STATE_PRINT_MENU: /* Print menu again */
    {
      /* Print menu */
      TERMINAL_INTERFACE.write("Menu:\n\r");
      TERMINAL_INTERFACE.write(" 0 - T0 and T1 knob value\n\r");
      TERMINAL_INTERFACE.write(" 1 - Temperature sensor value\n\r");
      TERMINAL_INTERFACE.write(" 2 - Solenoid valve control - full power\n\r");
      TERMINAL_INTERFACE.write(" 3 - Solenoid valve control - off\n\r");
      TERMINAL_INTERFACE.write(" 4 - Solenoid valve control - increase/decrease\n\r");
      TERMINAL_INTERFACE.write(" 5 - Print this menu\n\r");
      TERMINAL_INTERFACE.write("   - ");
      /* Reset terminal print timer */
      appl_inst.u16_applc_cyclic_time = APPL_CYCLIC_TIME_PERIOD;
      /* Go to idle state */
      appl_inst.ch_menuState = APPL_STATE_IDLE_STATE;
      break;
    }

    case APPL_STATE_IDLE_STATE:
    {
      /* Idle state. Do nothing */
    }

    default:
    {
      /* Do nothing */
      break;
    }
  }
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
    /* Call cyclical functions */
    Appl_Cyclic();
  }
}
