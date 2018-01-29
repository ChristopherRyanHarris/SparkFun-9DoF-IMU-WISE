
/*******************************************************************
** FILE:
**   	Logging_Functions
** DESCRIPTION:
** 		This file contains the logging functions.
**		These functions are used exclusively in debug mode
** 		and should not be included in the final firmware
**		implementation.
**		These functions are inteded for emulation
**		or debug modes only.
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
** FUNCTION: Debug_LogOut
** VARIABLES:
**		[I ]	CONTROL_TYPE			*p_control
**		[I ]	SENSOR_STATE_TYPE	*p_sensor_state
**		[I ]	WISE_STATE_TYPE		*p_wise_state
** RETURN:
**		NONE
** DESCRIPTION:
** 		This function just prints a standard string
** 		to the log_port serial port.
** 		It prints the rpy as well as the timestamp and
** 		and estimate of the sample rate
*/
void Debug_LogOut( CONTROL_TYPE				*p_control,
									 SENSOR_STATE_TYPE	*p_sensor_state,
									 WISE_STATE_TYPE		*p_wise_state )
{
  char LogBuffer[500];

  switch ( p_control->output_mode )
  {
    case 1:
    	sprintf(LogBuffer,"T:%09lu", p_control->timestamp );
      LOG_PRINT( LogBuffer );

      sprintf(LogBuffer,", DT:");LOG_PRINT( LogBuffer );
      FltToStr(p_control->G_Dt,4,LogBuffer);  LOG_PRINT( LogBuffer );
      sprintf(LogBuffer,", SR:");LOG_PRINT( LogBuffer );
      FltToStr((1/p_control->G_Dt),4,LogBuffer);  LOG_PRINT( LogBuffer );

      sprintf(LogBuffer,", R:");LOG_PRINT( LogBuffer );
      FltToStr(TO_DEG(p_sensor_state->roll),4,LogBuffer);  LOG_PRINT( LogBuffer );
      sprintf(LogBuffer,", P:");LOG_PRINT( LogBuffer );
      FltToStr(TO_DEG(p_sensor_state->pitch),4,LogBuffer); LOG_PRINT( LogBuffer );
      sprintf(LogBuffer,", Y:");LOG_PRINT( LogBuffer );
      FltToStr(TO_DEG(p_sensor_state->yaw),4,LogBuffer);   LOG_PRINT( LogBuffer );
      sprintf(LogBuffer,", A:%06d,%06d,%06d",
        (int)p_sensor_state->accel[0], (int)p_sensor_state->accel[1], (int)p_sensor_state->accel[2] );
      LOG_PRINT( LogBuffer );
      sprintf(LogBuffer,", G:%07d,%07d,%07d",
        (int)p_sensor_state->gyro[0],  (int)p_sensor_state->gyro[1],  (int)p_sensor_state->gyro[2] );
      LOG_PRINT( LogBuffer );

      LOG_PRINTLN(" ");
      break;

    case 2:
    	sprintf(LogBuffer,"%09lu", p_control->timestamp );
    	LOG_PRINT( LogBuffer );

      sprintf(LogBuffer,",%d,%d,%d",
	    	(int)p_sensor_state->accel[0],(int)p_sensor_state->accel[1],(int)p_sensor_state->accel[2] );
      LOG_PRINT( LogBuffer );
      sprintf(LogBuffer,",%d,%d,%d",
	    	(int)p_sensor_state->gyro[0],(int)p_sensor_state->gyro[1],(int)p_sensor_state->gyro[2] );
      LOG_PRINT( LogBuffer );

      LOG_PRINT(",");
      FltToStr(TO_DEG(p_sensor_state->roll),3,LogBuffer);
      LOG_PRINT( LogBuffer );
      LOG_PRINT(",");
      FltToStr(TO_DEG(p_sensor_state->pitch),3,LogBuffer);
      LOG_PRINT( LogBuffer );
      LOG_PRINT(",");
      FltToStr(TO_DEG(p_sensor_state->yaw),3,LogBuffer);
      LOG_PRINT( LogBuffer );

      LOG_PRINTLN(" ");
      break;
    default:
    	break;
  }
} /* End Debug_LogOut */


/*************************************************
** FUNCTION: Debug_LogOut
** VARIABLES:
**		[I ]	CONTROL_TYPE			*p_control
**		[I ]	SENSOR_STATE_TYPE	*p_sensor_state
**		[I ]	CALIBRATION_TYPE	*p_calibration
** RETURN:
**		NONE
** DESCRIPTION:
** Cal_LogOut
** This function just prints a standard string
** to the log_port serial port.
** It is designed to assist in the calibration
** of the sensor
*/
void Cal_LogOut( CONTROL_TYPE			 *p_control,
								 SENSOR_STATE_TYPE *p_sensor_state,
								 CALIBRATION_TYPE	 *p_calibration )
{
  char LogBuffer[500];

	sprintf(LogBuffer,"TIME: %09lu", p_control->timestamp );
  LOG_PRINT( LogBuffer );

  sprintf(LogBuffer,", DT: ");LOG_PRINT( LogBuffer );
  FltToStr(p_control->G_Dt,4,LogBuffer);  LOG_PRINT( LogBuffer );
  sprintf(LogBuffer,", SR: ");LOG_PRINT( LogBuffer );
  FltToStr((1/p_control->G_Dt),4,LogBuffer);  LOG_PRINT( LogBuffer );

  switch ( p_control->calibration_prms.output_mode )
  {
    case 0:
  		LOG_PRINT( ", accel (min/ave/max): " );

  		LOG_PRINT( "[a1](" );
  		FltToStr(p_calibration->accel_min[0],4,LogBuffer);
  		LOG_PRINT( LogBuffer );LOG_PRINT( "/" );
  		FltToStr(p_calibration->accel_total[0]/p_calibration->N,4,LogBuffer);
  		LOG_PRINT( LogBuffer );LOG_PRINT( "/" );
  		FltToStr(p_calibration->accel_max[0],4,LogBuffer);
  		LOG_PRINT( LogBuffer );LOG_PRINT( "), " );

  		LOG_PRINT( "[a2](" );
  		FltToStr(p_calibration->accel_min[1],4,LogBuffer);
  		LOG_PRINT( LogBuffer );LOG_PRINT( "/" );
  		FltToStr(p_calibration->accel_total[1]/p_calibration->N,4,LogBuffer);
  		LOG_PRINT( LogBuffer );LOG_PRINT( "/" );
  		FltToStr(p_calibration->accel_max[1],4,LogBuffer);
  		LOG_PRINT( LogBuffer );LOG_PRINT( "), " );

  		LOG_PRINT( "[a3](" );
  		FltToStr(p_calibration->accel_min[2],4,LogBuffer);
  		LOG_PRINT( LogBuffer );LOG_PRINT( "/" );
  		FltToStr(p_calibration->accel_total[2]/p_calibration->N,4,LogBuffer);
  		LOG_PRINT( LogBuffer );LOG_PRINT( "/" );
  		FltToStr(p_calibration->accel_max[2],4,LogBuffer);
  		LOG_PRINT( LogBuffer ); LOG_PRINT( ")" );

  		LOG_PRINTLN(" ");
  		break;
    case 1:
  		LOG_PRINT( ", gyro (ave/current): " );

  		LOG_PRINT( "[g1](" );
  		FltToStr(p_calibration->gyro_total[0]/p_calibration->N,4,LogBuffer);
  		LOG_PRINT( LogBuffer );LOG_PRINT( "/" );
  		FltToStr(p_sensor_state->gyro[0],4,LogBuffer);
  		LOG_PRINT( LogBuffer );LOG_PRINT( "), " );

  		LOG_PRINT( "[g2](" );
  		FltToStr(p_calibration->gyro_total[1]/p_calibration->N,4,LogBuffer);
  		LOG_PRINT( LogBuffer );LOG_PRINT( "/" );
  		FltToStr(p_sensor_state->gyro[1],4,LogBuffer);
  		LOG_PRINT( LogBuffer );LOG_PRINT( "), " );

  		LOG_PRINT( "[g3](" );
  		FltToStr(p_calibration->gyro_total[2]/p_calibration->N,4,LogBuffer);
  		LOG_PRINT( LogBuffer );LOG_PRINT( "/" );
  		FltToStr(p_sensor_state->gyro[2],4,LogBuffer);
  		LOG_PRINT( LogBuffer );LOG_PRINT( ")" );

  		LOG_PRINTLN(" ");
  		break;
  }
} /* End Cal_LogOut */




/*************************************************
** FUNCTION: FltToStr
** VARIABLES:
**    [I ]  float value
**    [I ]  int   precision
**    [IO]  char *StrBuffer
** RETURN:
**    NONE
** DESCRIPTION:
**    This function converts a floating point
**    number into a string. This is needed to
**    support logging floats in Arduino.
*/
void FltToStr( float value,
               int   precision,
               char *StrBuffer )
{
  switch ( precision )
  {
    case 0:
      sprintf(StrBuffer, "%d", (int)value);
      break;
    case 1:
      sprintf(StrBuffer, "%d.%01d", (int)value, ABS((int)(value*10)%10) );
      break;
    case 2:
      sprintf(StrBuffer, "%d.%02d", (int)value, ABS((int)(value*100)%100) );
      break;
    case 3:
      sprintf(StrBuffer, "%d.%03d", (int)value, ABS((int)(value*1000)%1000) );
      break;
    case 4:
      sprintf(StrBuffer, "%d.%04d", (int)value, ABS((int)(value*10000)%10000) );
      break;
    case 5:
      sprintf(StrBuffer, "%d.%05d", (int)value, ABS((int)(value*100000)%100000) );
      break;
  }
}


