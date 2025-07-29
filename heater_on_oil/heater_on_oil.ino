#include "stdint.h"
#include "OneWire.h"
#include "DallasTemperature.h"

/* Global defines */
#define T0_KNOB   PIN_A0
#define T1_KNOB   PIN_A1
#define L298M_ENA   3
#define L298M_IN1   2
#define L298M_IN2   4
#define GLOW_PLUG   5
#define DS18B20_PIN 8

#define PWM_MAX_VALUE                       ((uint8_t)255U)

#define TERMINAL_BUFFER_SIZE                ((uint16_t)1024U)
#define TERMINAL_INTERFACE                  Serial

#define APPL_CYCLIC_TIME_PERIOD             ((uint16_t)1000U)

#define APPL_READ_INPUT_CARIAGE_RETURN_SIZE ((uint8_t)2U)
#define APPL_STATE_PRINT_MENU                   '0'
#define APPL_STATE_ANALOG                       '1'
#define APPL_STATE_DS18B20                      '2'
#define APPL_STATE_SOLENOID_CTRL_FULL_ON        '3'
#define APPL_STATE_SOLENOID_CTRL_FULL_OFF       '4'
#define APPL_STATE_SOLENOID_CTRL_RISE_FALL      '5'
#define APPL_STATE_SOLENOID_KNOB_CONTROL        '6'
#define APPL_STATE_TIME_CONVERT_FACTOR          '7'
#define APPL_STATE_SOLENOID_KNOB_GLOW_PLG_CTRL  '8'
#define APPL_STATE_IDLE_STATE                   '9'

// Setup a oneWire instance to communicate with any OneWire device
OneWire oneWire(DS18B20_PIN);

// Pass oneWire reference to DallasTemperature library
DallasTemperature sensors(&oneWire);


/* Data type */
typedef struct appl_type
{
  uint16_t u16_cyclic_time;
  uint8_t u08_pwm_power;
  uint8_t u08_temp_id;
  char    ch_menuState[APPL_READ_INPUT_CARIAGE_RETURN_SIZE];
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
static uint16_t gu16_t0_knob_adc = 0U;
static uint16_t gu16_t1_knob_adc = 0U;
static uint32_t gu32_t0_knob_ref = 0U;
static uint32_t gu32_t1_knob_ref = 0U;
static uint32_t gu32_t0_knob = 0U;
static uint32_t gu32_t1_knob = 0U;
static float gf_t0_factor = 1.0F;
static float gf_t1_factor = 1.0F;

/* Temperature sensor */
// DS18B20 ds(DS18B20_PIN);

/* Data stream sent to terminal */
static ts_appl_type appl_inst;

/* Setup and configuration */
void setup() {
  /* Temperature sensor addresses */
  uint8_t lu8_ds18b20_address[] = {40, 250, 31, 218, 4, 0, 0, 52};
  /* Variable set */
  appl_inst.u16_cyclic_time = APPL_CYCLIC_TIME_PERIOD;
  appl_inst.u08_pwm_power = PWM_MAX_VALUE;
  appl_inst.ch_menuState[0U] = APPL_STATE_PRINT_MENU;
  appl_inst.status = { 1 };
  /* Pin configuration */
  pinMode(L298M_ENA, OUTPUT);
  pinMode(L298M_IN1, OUTPUT);
  pinMode(L298M_IN2, OUTPUT);
  pinMode(GLOW_PLUG, OUTPUT);
  analogWrite(GLOW_PLUG, 0);
  /* Set solenoid direction and turn it off */
  digitalWrite(L298M_IN1, HIGH);
  digitalWrite(L298M_IN2, LOW);
  analogWrite(L298M_ENA, 0);
  /* Configure Uart to see output results */
  TERMINAL_INTERFACE.begin(115200);
  /* Select temperature sensro from bus */
  sensors.begin();

  /* Analog reference */
  analogReference(EXTERNAL);
}

void ADC_Update(void)
{
  /* Get values and save them */
  gu16_t0_knob_adc = analogRead(T0_KNOB);
  gu16_t1_knob_adc = analogRead(T1_KNOB);
}

void Solenoid_Pwr_Update(uint16_t u16_pwr)
{
  if (u16_pwr > PWM_MAX_VALUE)
  {
    u16_pwr = PWM_MAX_VALUE;
  }

  /* Set full pwm power */
  appl_inst.u08_pwm_power = u16_pwr;
  analogWrite(L298M_ENA, appl_inst.u08_pwm_power);
}

void T0_T1_Knob_Update(void)
{
  /* Update open/close */
  if (gu32_t0_knob > 0U)
  {
    gu32_t0_knob--;
  }
  else
  {
    gu32_t0_knob = gu32_t0_knob_ref;
    Solenoid_Pwr_Update(PWM_MAX_VALUE);
    TERMINAL_INTERFACE.println("Open");
  }

  /* Update open/close */
  if (gu32_t1_knob > 0U)
  {
    gu32_t1_knob--;
  }
  else
  {
    gu32_t1_knob = gu32_t1_knob_ref;
    Solenoid_Pwr_Update(0U);
    TERMINAL_INTERFACE.println("Close");
  }
}

void Appl_Cyclic(void)
{
  /* Read menu input mode */
  if (TERMINAL_INTERFACE.available())
  {
    TERMINAL_INTERFACE.readBytes(appl_inst.ch_menuState, APPL_READ_INPUT_CARIAGE_RETURN_SIZE);
  }

  /* Application's state machine */
  switch(appl_inst.ch_menuState[0U])
  {
    case APPL_STATE_ANALOG: /* Analog valuse */
    {
      /* Decrement application cyclic time preiod */
      if (appl_inst.u16_cyclic_time > 0U)
      {
        appl_inst.u16_cyclic_time--;
      }
      else
      {
        /* Reset cyclic timer value */
        appl_inst.u16_cyclic_time = APPL_CYCLIC_TIME_PERIOD;
        /* Print result and visualize it */
        sprintf(appl_inst.ch_data_stream, "%4.1d %4.1d\n\r", gu16_t0_knob_adc, gu16_t1_knob_adc);
        /* Print buffer */
        TERMINAL_INTERFACE.write(appl_inst.ch_data_stream);
      }
      break;
    }

    case APPL_STATE_DS18B20: /* Temperatur sensor data */
    {
      /* Decrement application cyclic time preiod */
      if (appl_inst.u16_cyclic_time > 0U)
      {
        appl_inst.u16_cyclic_time--;
      }
      else
      {
        /* Reset cyclic timer value */
        appl_inst.u16_cyclic_time = APPL_CYCLIC_TIME_PERIOD;
        /* Get temperature */
        sensors.requestTemperatures();
        TERMINAL_INTERFACE.println(sensors.getTempCByIndex(0));
      }
      break;
    }

    case APPL_STATE_SOLENOID_CTRL_FULL_ON: /* Solenoid valve control - full power */
    {
      /* Set full pwm power */
      Solenoid_Pwr_Update(PWM_MAX_VALUE);
      /* Go to idle state */
      appl_inst.ch_menuState[0U] = APPL_STATE_IDLE_STATE;
      break;
    }

    case APPL_STATE_SOLENOID_CTRL_FULL_OFF: /* Solenoid valve control - off */
    {
      /* Set full pwm power */
      Solenoid_Pwr_Update(0U);
      /* Go to idle state */
      appl_inst.ch_menuState[0U] = APPL_STATE_IDLE_STATE;
      break;
    }

    case APPL_STATE_SOLENOID_CTRL_RISE_FALL: /* Solenoid valve control - increase/decrease */
    {
      /* Set driver direction and PWM */
      digitalWrite(L298M_IN1, HIGH);
      digitalWrite(L298M_IN2, LOW);
      /* Decrement application cyclic time preiod */
      if (appl_inst.u16_cyclic_time > 0U)
      {
        appl_inst.u16_cyclic_time--;
      }
      else
      {
        /* Reset cyclic timer value */
        appl_inst.u16_cyclic_time = APPL_CYCLIC_TIME_PERIOD;
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
      TERMINAL_INTERFACE.write(" 0 - Print this menu\n\r");
      TERMINAL_INTERFACE.write(" 1 - T0 and T1 knob value\n\r");
      TERMINAL_INTERFACE.write(" 2 - Temperature sensor value\n\r");
      TERMINAL_INTERFACE.write(" 3 - Solenoid valve control - close\n\r");
      TERMINAL_INTERFACE.write(" 4 - Solenoid valve control - open\n\r");
      TERMINAL_INTERFACE.write(" 5 - Solenoid valve control - increase/decrease\n\r");
      TERMINAL_INTERFACE.write(" 6 - Solenoid knob control\n\r");
      TERMINAL_INTERFACE.write(" 7 - Time convert factor T0 and T1\n\r");
      TERMINAL_INTERFACE.write(" 8 - Solenoid knob glow plug control\n\r");
      TERMINAL_INTERFACE.write("   - ");
      /* Reset terminal print timer */
      appl_inst.u16_cyclic_time = APPL_CYCLIC_TIME_PERIOD;
      /* Go to idle state */
      appl_inst.ch_menuState[0U] = APPL_STATE_IDLE_STATE;
      break;
    }

    case APPL_STATE_SOLENOID_KNOB_CONTROL: /* Solenoid knob control */
    {
      /* Decrement application cyclic time preiod */
      if (appl_inst.u16_cyclic_time > 0U)
      {
        appl_inst.u16_cyclic_time--;
      }
      else
      {
        /* Reset cyclic timer value */
        appl_inst.u16_cyclic_time = APPL_CYCLIC_TIME_PERIOD;
        /* Print result and visualize it */
        sprintf(appl_inst.ch_data_stream, "%ld %ld\n\r", gu32_t0_knob_ref, gu32_t1_knob_ref);
        /* Print buffer */
        TERMINAL_INTERFACE.write(appl_inst.ch_data_stream);

        /* Knob update */
        T0_T1_Knob_Update();
      }
      break;
    }

    case APPL_STATE_TIME_CONVERT_FACTOR: /* Time convert factor */
    {
      /* Get t1 coeficient from terminal */
      TERMINAL_INTERFACE.println();
      TERMINAL_INTERFACE.print("Waiting input for t0 - ");
      while (TERMINAL_INTERFACE.available() == 0) {}
      if (TERMINAL_INTERFACE.available() > 0)
      {
        gf_t0_factor = TERMINAL_INTERFACE.parseFloat();
      }
      while (TERMINAL_INTERFACE.available() > 0)
      {
        TERMINAL_INTERFACE.read();
      }
      
      /* Get t1 coeficient from terminal */
      TERMINAL_INTERFACE.println();
      TERMINAL_INTERFACE.print("Waiting input for t1 - ");
      while (TERMINAL_INTERFACE.available() == 0) {}
      if (TERMINAL_INTERFACE.available() > 0)
      {
        gf_t1_factor = TERMINAL_INTERFACE.parseFloat();

        if ((gf_t0_factor != 0) && (gf_t1_factor != 0)
          && (gf_t0_factor >= 0.001) && (gf_t1_factor >= 0.001))
        {
          TERMINAL_INTERFACE.println();
          TERMINAL_INTERFACE.print("You entered the following factors: t0 - ");
          TERMINAL_INTERFACE.print(gf_t0_factor);
          TERMINAL_INTERFACE.print(" t1 - ");
          TERMINAL_INTERFACE.println(gf_t1_factor);
        }
        else
        {
          TERMINAL_INTERFACE.println("Bad t0 and t1 input. Try again.");
        }
      }
      while (TERMINAL_INTERFACE.available() > 0)
      {
        TERMINAL_INTERFACE.read();
      }

      /**/
      ADC_Update();
      /* Convert to time */
      gu32_t0_knob_ref = (uint32_t)gu16_t0_knob_adc * gf_t0_factor;
      gu32_t1_knob_ref = (uint32_t)gu16_t1_knob_adc * gf_t1_factor;

      /* Update to timer */
      gu32_t0_knob = gu32_t0_knob_ref;
      gu32_t1_knob = gu32_t1_knob_ref;
      /* Go to menu state */
      appl_inst.ch_menuState[0U] = APPL_STATE_PRINT_MENU;

      break;
    }

    case APPL_STATE_SOLENOID_KNOB_GLOW_PLG_CTRL:
    {
      uint8_t lu08_pwm_power = uint8_t((gu16_t0_knob_adc * (uint32_t)255u) / (uint16_t)1023U);

      /* Limit power */
      if (lu08_pwm_power > 255)
      {
        lu08_pwm_power = 255;
      }

      /* Print current applied power */
      if (appl_inst.u16_cyclic_time > 0U)
      {
        appl_inst.u16_cyclic_time--;
      }
      else
      {
        /* Reset cyclic timer value */
        appl_inst.u16_cyclic_time = APPL_CYCLIC_TIME_PERIOD;
        /* Construct string output */
        sprintf(appl_inst.ch_data_stream, "%4.1d %u\n\r", gu16_t0_knob_adc, lu08_pwm_power);
        /* Print buffer */
        TERMINAL_INTERFACE.write(appl_inst.ch_data_stream);
      }
      
      /* Write new PWM */
      analogWrite(GLOW_PLUG, lu08_pwm_power);
      break;
    }

    case APPL_STATE_IDLE_STATE:
    {
      /* Idle state. Do nothing */
      break;
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
    ADC_Update();
    Appl_Cyclic();
  }
}
