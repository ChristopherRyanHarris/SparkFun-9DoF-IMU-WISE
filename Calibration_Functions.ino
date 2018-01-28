
/*******************************************************************
** FILE:
**   	Calibration_Functions
** DESCRIPTION:
** 		This file contains the functions which will aid in
**    calibrating the sensors.
** 		These functions are hardware independent.
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

/*******************************************************************
** Functions *******************************************************
********************************************************************/


/*************************************************
** FUNCTION: Calibration_Init
** VARIABLES:
**		[IO]	CONTROL_TYPE			*p_control
**		[IO]	CALIBRATION_TYPE	*p_calibration
** RETURN:
**		NONE
** DESCRIPTION:
** 		This function initializes the calibration state
** 		variables.
*/
void Calibration_Init ( CONTROL_TYPE			*p_control,
												CALIBRATION_TYPE 	*p_calibration )
{
  int i;

  LOG_PRINTLN("> Initializing Calibration");

	/* Set default calibration parameters */
	p_control->calibration_prms.output_mode = CAL_OUTPUT_MODE;

	p_control->calibration_prms.accel_min_x = ACCEL_X_MIN;
	p_control->calibration_prms.accel_max_x = ACCEL_X_MAX;
	p_control->calibration_prms.accel_min_y = ACCEL_Y_MIN;
	p_control->calibration_prms.accel_max_y = ACCEL_Y_MAX;
	p_control->calibration_prms.accel_min_z = ACCEL_Z_MIN;
	p_control->calibration_prms.accel_max_z = ACCEL_Z_MAX;

	p_control->calibration_prms.gyro_ave_offset_x = GYRO_AVERAGE_OFFSET_X;
	p_control->calibration_prms.gyro_ave_offset_y = GYRO_AVERAGE_OFFSET_Y;
	p_control->calibration_prms.gyro_ave_offset_z = GYRO_AVERAGE_OFFSET_Z;

	p_control->calibration_prms.magn_min_x = MAGN_X_MIN;
	p_control->calibration_prms.magn_max_x = MAGN_X_MAX;
	p_control->calibration_prms.magn_min_y = MAGN_Y_MIN;
	p_control->calibration_prms.magn_max_y = MAGN_Y_MAX;
	p_control->calibration_prms.magn_min_z = MAGN_Z_MIN;
	p_control->calibration_prms.magn_max_z = MAGN_Z_MAX;

	/* Initialize calibration state */
  for( i=0; i<3; i++ )
  {
    p_calibration->accel_total[i] = 0.0f;
    p_calibration->accel_max[i]   = -9999.0f;
    p_calibration->accel_min[i]   = 9999.0f;

    p_calibration->gyro_total[i]  = 0.0f;
    p_calibration->gyro_max[i]    = -9999.0f;
    p_calibration->gyro_min[i]    = 9999.0f;
  }
  p_calibration->N = 0;
} /* End Calibration_Init */


/*************************************************
** FUNCTION: Calibrate
** VARIABLES:
**		[I ]	CONTROL_TYPE			*p_control
**		[IO]	CALIBRATION_TYPE	*p_calibration
**		[I ]	SENSOR_STATE_TYPE	*p_sensor_state
** RETURN:
**		NONE
** DESCRIPTION:
** 		This function initializes the calibration state
** 		variables.
*/
void Calibrate ( CONTROL_TYPE				*p_control,
								 CALIBRATION_TYPE		*p_calibration,
								 SENSOR_STATE_TYPE	*p_sensor_state )
{
  int i;
  for( i=0; i<3; i++ )
  {
    p_calibration->accel_total[i] += p_sensor_state->accel[i];
    if( p_sensor_state->accel[i] > p_calibration->accel_max[i] ) { p_calibration->accel_max[i] = p_sensor_state->accel[i]; }
    if( p_sensor_state->accel[i] < p_calibration->accel_min[i] ) { p_calibration->accel_min[i] = p_sensor_state->accel[i]; }

    p_calibration->gyro_total[i] += p_sensor_state->gyro[i];
    if( p_sensor_state->gyro[i] > p_calibration->gyro_max[i] ) { p_calibration->gyro_max[i] = p_sensor_state->gyro[i]; }
    if( p_sensor_state->gyro[i] < p_calibration->gyro_min[i] ) { p_calibration->gyro_min[i] = p_sensor_state->gyro[i]; }
  }
  p_calibration->N++;

} /* End Calibrate */









