#include "appl.h"
#include "../temperature/temperature.h"
#include "../glow_plug/glow_plug.h"
#include "../solenoid/solenoid.h"

#ifdef ARDUINO_SAMD_NANO_33_IOT
#define VOLTAGE_REF AR_EXTERNAL
#else
#define VOLTAGE_REF EXTERNAL
#endif

/* Global defines */
#define PWM_MAX_VALUE                       ((uint8_t)255U)

#define TERMINAL_BUFFER_SIZE                ((uint16_t)1024U)
#define TERMINAL_INTERFACE                  Serial

#define APPL_CYCLIC_TIME_PERIOD             ((uint16_t)1000U)

#define APPL_READ_INPUT_CARIAGE_RETURN_SIZE ((uint8_t)2U)
#define APPL_STATE_PRINT_MENU               '0'
#define APPL_STATE_ACTIVATE                 '1'
#define APPL_STATE_DEZACTIVATE              '2'
#define APPL_STATE_SOLENOID_CTRL_FULL_ON    '3'
#define APPL_STATE_SOLENOID_CTRL_FULL_OFF   '4'
#define APPL_STATE_GLOW_PLUG_ON             '5'
#define APPL_STATE_GLOW_PLUG_OFF            '6'
#define APPL_STATE_IDLE_STATE               '7'

/* Data type */
typedef struct appl_type
{
  uint16_t u16_cyclic_time;
  uint8_t u08_pwm_power;
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

/* Data stream sent to terminal */
static ts_appl_type appl_inst;

void Appl_Plot(void)
{
  TERMINAL_INTERFACE.print("Tref");
  TERMINAL_INTERFACE.print(Temperature_GetRef());
  TERMINAL_INTERFACE.print("Tacc");
  TERMINAL_INTERFACE.print(Temperature_GetAcc());
  TERMINAL_INTERFACE.print("Solenoid");
  TERMINAL_INTERFACE.print(Temperature_GetAcc());
  TERMINAL_INTERFACE.print("PlugPower");
  TERMINAL_INTERFACE.print(GlwoPlug_GetCurrentPwm());
}

void Appl_FSTM(void)
{
  /* Read menu input mode */
  if (TERMINAL_INTERFACE.available())
  {
    TERMINAL_INTERFACE.readBytes(appl_inst.ch_menuState, APPL_READ_INPUT_CARIAGE_RETURN_SIZE);
  }

  /* Application's state machine */
  switch(appl_inst.ch_menuState[0U])
  {
    case APPL_STATE_PRINT_MENU: /* Print menu again */
    {
      /* Print menu */
      TERMINAL_INTERFACE.write("Menu:\n\r");
      TERMINAL_INTERFACE.write(" 0 - Print this menu\n\r");
      TERMINAL_INTERFACE.write(" 1 - Activate\n\r");
      TERMINAL_INTERFACE.write(" 2 - Dezactivate\n\r");
      TERMINAL_INTERFACE.write(" 3 - Solenoid valve control - close\n\r");
      TERMINAL_INTERFACE.write(" 4 - Solenoid valve control - open\n\r");
      TERMINAL_INTERFACE.write(" 5 - Glow Plug - on\n\r");
      TERMINAL_INTERFACE.write(" 6 - Glow Plug - off\n\r");
      TERMINAL_INTERFACE.write("   - ");
      /* Reset terminal print timer */
      appl_inst.u16_cyclic_time = APPL_CYCLIC_TIME_PERIOD;
      /* Go to idle state */
      appl_inst.ch_menuState[0U] = APPL_STATE_IDLE_STATE;
      break;
    }

    case APPL_STATE_ACTIVATE:
    {
      /* Activate and control the device */
      break;
    }

    case APPL_STATE_DEZACTIVATE:
    {
      GlowPlug_OnOff(STD_ON);
      Solenoid_OnOff(STD_OFF);
      /* Go to idle state */
      appl_inst.ch_menuState[0U] = APPL_STATE_IDLE_STATE;
      break;
    }

    case APPL_STATE_SOLENOID_CTRL_FULL_ON:
    {
      Solenoid_OnOff(STD_ON);
      /* Go to idle state */
      appl_inst.ch_menuState[0U] = APPL_STATE_IDLE_STATE;
      break;
    }

    case APPL_STATE_SOLENOID_CTRL_FULL_OFF:
    {
      Solenoid_OnOff(STD_OFF);
      /* Go to idle state */
      appl_inst.ch_menuState[0U] = APPL_STATE_IDLE_STATE;
      break;
    }

    case APPL_STATE_GLOW_PLUG_ON:
    {
      GlowPlug_OnOff(STD_ON);
      /* Go to idle state */
      appl_inst.ch_menuState[0U] = APPL_STATE_IDLE_STATE;
      break;
    }

    case APPL_STATE_GLOW_PLUG_OFF:
    {
      GlowPlug_OnOff(STD_OFF);
      /* Go to idle state */
      appl_inst.ch_menuState[0U] = APPL_STATE_IDLE_STATE;
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

void Appl_Init(void)
{
  /* Variable set */
  appl_inst.u16_cyclic_time = APPL_CYCLIC_TIME_PERIOD;
  appl_inst.u08_pwm_power = PWM_MAX_VALUE;
  appl_inst.ch_menuState[0U] = APPL_STATE_PRINT_MENU;
  appl_inst.status = { 1 };

  /* Configure Uart to see output results */
  TERMINAL_INTERFACE.begin(115200);

  /* Analog reference */
  analogReference(VOLTAGE_REF);
  /* Sensors */
  Temperature_Init();
  /* Actuators */
  GlowPlug_Init();
  /* Solenoid */
  Solenoid_Init();
}

void Appl_Cyclic(void)
{
  Appl_Plot();
  Appl_FSTM();
}