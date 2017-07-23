/*************************************************
** FILE : HW_Functions
** This file contains functions which are used
** to asses the internal state of the board.
** Ideally, we would have functions which can run
** diagnostics.
** All functions should be platform independent
**************************************************/


/*******************************************************************
** Includes ********************************************************
********************************************************************/


#include "./Include/Common_Config.h"

#if EXE_MODE==1 /* Emulator Mode */

#ifdef _IMU10736_
#include "./Include/IMU10736_Config.h"
#endif
#ifdef _IMU9250_
#include <SparkFunMPU9250-DMP.h>
#include "./Include/IMU9250_Config.h"
#endif

#include "./Include/DSP_Config.h"
#include "./Include/WISE_Config.h"
#include "./Include/Emulator_Config.h"
extern CAL_STATE_TYPE      g_calibration;
extern DCM_STATE_TYPE      g_dcm_state;
extern DSP_COMMON_TYPE     g_dsp;
extern SENSOR_STATE_TYPE   g_sensor_state;
extern CONTROL_STATE_TYPE  g_control_state;
extern WISE_STATE_TYPE     g_wise_state;
#endif  /* End Emulator Mode */


/*******************************************************************
** Functions *******************************************************
********************************************************************/



/*************************************************
** Init_Hardware
** This function sets the LED GPIO Pin and
** the com port baud rates.
*/
void Init_Hardware( void )
{
  #if EXE_MODE==0 /* IMU Mode */
  /* Initiate the LOG_PORT */
  LOG_PORT.begin(LOG_PORT_BAUD);
  
  LOG_PRINTLN("> Initializing Hardware");

  /* Set up LED pin (active-high, default to off) */
  pinMode(HW_LED_PIN, OUTPUT);
  digitalWrite(HW_LED_PIN, LOW);
  #endif

  g_control_state.timestamp      = 0;
  g_control_state.timestamp_old  = 0;
  g_control_state.G_Dt           = 0.0;

  g_control_state.g_BaudLock       = true;
  g_control_state.g_LedState       = false;
  g_control_state.g_LastBlinkTime  = 0;
} /* End Init_Hardware */

/*************************************************
** BLinkLED
** This function is used to communicate to the user
** that the board is indeed doing things
** TO DO: I plan to implement a blink code for debugging
*/
void Blink_LED( void )
{
  #if EXE_MODE==0 /* IMU Mode */
  /* We blink every UART_BLINK_RATE millisecods */
  if ( millis() > (g_control_state.g_LastBlinkTime + UART_BLINK_RATE) )
  {
    /* Log the current states to the debug port */
    Debug_LogOut();

    /* Display number of bytes available on comm port
    ** Com port is used for real-time communication with
    ** connected processor */
    LOG_PORT.println("> # Available on COMM_PORT: " + String(COMM_PORT.available()) );

    /* Toggle LED */
    LOG_PORT.println("> Blink ...");
    digitalWrite(HW_LED_PIN, g_control_state.g_LedState);
    g_control_state.g_LedState = !g_control_state.g_LedState;
    g_control_state.g_LastBlinkTime = millis();
  }
  #endif /* EXE_MODE */
} /* End Blink_LED */






