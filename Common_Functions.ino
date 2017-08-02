
/*************************************************
** FILE: Common_Functions
** This file contains some MPU 9250 (HW specific)
** functions. Specifically, for initializing and
** reading the sensor registeres
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

/* Common_Init
** This function initializes variables and constants which
** are the same across all platforms and which are common
** across all agorithm variants
*/
void Common_Init( void )
{
  LOG_PRINT("> Initializing\n");

	/* Set default IO mode */
	g_control_state.output_mode = OUTPUT_MODE;
  g_calibration.output_mode   = CAL_OUTPUT_MODE;
  //g_calibration.calibrate_flag = 0;

  g_control_state.timestamp      = 0;
  g_control_state.timestamp_old  = 0;
  g_control_state.G_Dt           = 0.0;

  g_control_state.g_BaudLock       = true;
  g_control_state.g_LedState       = false;
  g_control_state.g_LastBlinkTime  = 0;
} /* End Common_Init*/



