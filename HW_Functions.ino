
/*******************************************************************
** FILE: 
**   	HW_Functions
** DESCRIPTION:
**		The Hardware functions.
** 		This file contains functions which are used
** 		to asses the internal state of the board.
**		Ideally, we would have functions which can run
** 		diagnostics.
** 		All functions should be fairly independent of IMU hardware. 
**    These functions cannot be used in emulation mode.
********************************************************************/


/*******************************************************************
** Includes ********************************************************
********************************************************************/

#ifndef COMMON_CONFIG_H
	#include "../Include/Common_Config.h"
#endif
#if EXE_MODE==1 /* Emulator Mode */
	/* In emulatiom mode, "Emulator_Protos" is needed to 
	** use funcitons in other files.
	** NOTE: This header should contain the function 
	** 			 prototypes for all execution functions */
	#include "../Include/Emulator_Protos.h"
#endif  /* End Emulator Mode */

/* Only link if not in emulation mode 
** (i.e. in real-time execution mode) */
#if EXE_MODE==0

/*******************************************************************
** Functions *******************************************************
********************************************************************/


/*************************************************
** FUNCTION: Init_Hardware
** VARIABLES:
**		[IO]	CONTROL_TYPE 			*p_control
** RETURN:
**		NONE
** DESCRIPTION:
** 		This function sets the LED GPIO Pin and
** 		the com port baud rates.
*/
void Init_Hardware( CONTROL_TYPE	*p_control )
{
  /* Initiate the LOG_PORT */
  LOG_PORT.begin(LOG_PORT_BAUD);
  delay(2000);

  LOG_PRINTLN("> Initializing Hardware");

  /* Set up LED pin (active-high, default to off) */
  pinMode(HW_LED_PIN, OUTPUT);
	digitalWrite(HW_LED_PIN, LOW);

  p_control->BaudLock       = TRUE;
  p_control->LedState       = FALSE;
  p_control->LastBlinkTime  = 0;

} /* End Init_Hardware */

/*************************************************
** FUNCTION: BLinkLED
** VARIABLES:
**		[IO]	CONTROL_TYPE 			*p_control
** RETURN:
**		NONE
** DESCRIPTION:
** 		This function is used to communicate to the user
** 		that the board is indeed doing things
** 		TO DO: I plan to implement a blink code for debugging
*/
void Blink_LED( CONTROL_TYPE	*p_control )
{
	/* We blink every UART_BLINK_RATE millisecods */
	if ( millis() > (p_control->LastBlinkTime + UART_BLINK_RATE) )
	{
  	/* Toggle LED */
  	digitalWrite(HW_LED_PIN, p_control->LedState);
  	p_control->LedState = !p_control->LedState;
  	p_control->LastBlinkTime = millis();
	}
} /* End Blink_LED */


#endif  /* End EXE_MODE (Real-Time Execution) */

