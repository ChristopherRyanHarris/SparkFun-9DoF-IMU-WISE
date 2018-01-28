
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
  char fastlog[500];
  String t1,t2,t3;

  switch ( p_control->output_mode )
  {
    case 0:
    	sprintf(fastlog,"T:%09lu, DT:%.4f, SR:% 07.4f, R:% 09.4f, P:% 09.4f, Y:% 09.4f, A:% 05.0f,% 05.0f,% 05.0f, G:% 05.0f,% 05.0f,% 05.0f\n",
    		p_control->timestamp,p_control->G_Dt,(1/p_control->G_Dt),
	    	TO_DEG(p_sensor_state->roll),TO_DEG(p_sensor_state->pitch),TO_DEG(p_sensor_state->yaw),
	    	p_sensor_state->accel[0],p_sensor_state->accel[1],p_sensor_state->accel[2],
	    	p_sensor_state->gyro[0],p_sensor_state->gyro[1],p_sensor_state->gyro[2]);
      break;
    case 1:
    	sprintf(fastlog,"WISE: v:%.4f %.4f vave: %.4f %.4f Igait: %.4f Iave: %.4f I: %.4f\n",
    		p_wise_state->vel[0],p_wise_state->vel[1],p_wise_state->vel_ave[0],p_wise_state->vel_ave[1],
    		p_wise_state->Incline_gait,p_wise_state->Incline_ave,p_wise_state->Incline );
      break;
    case 2:
    	sprintf(fastlog,"WISE(v) (v/av/d/od/op): 1:%.4f/%.4f/%.4f/%.4f/%.4f 2:%.4f/%.4f/%.4f/%.4f/%.4f\n",
    		p_wise_state->vel[0],p_wise_state->vel_ave[0],p_wise_state->vel_delta[0],p_wise_state->omega_vd[0],p_wise_state->omega_vp[0],
    		p_wise_state->vel[1],p_wise_state->vel_ave[1],p_wise_state->vel_delta[1],p_wise_state->omega_vd[1],p_wise_state->omega_vp[1] );
      break;
    case 3:
    	sprintf(fastlog,"WISE(a) (a/aa/d/od/op): 1:%.4f/%.4f/%.4f/%.4f/%.4f 2:%.4f/%.4f/%.4f/%.4f/%.4f\n",
    		p_wise_state->accel[0],p_wise_state->accel_ave[0],p_wise_state->accel_delta[0],p_wise_state->omega_ad[0],p_wise_state->omega_ap[0],
    		p_wise_state->accel[1],p_wise_state->accel_ave[1],p_wise_state->accel_delta[1],p_wise_state->omega_ad[1],p_wise_state->omega_ap[1]);
      break;
    case 4:
    	sprintf(fastlog,"%09lu,%5d,%5d,%5d,%d,%d,%d,",
    		p_control->timestamp,
	    	(int)p_sensor_state->accel[0],(int)p_sensor_state->accel[1],(int)p_sensor_state->accel[2],
	    	(int)p_sensor_state->gyro[0],(int)p_sensor_state->gyro[1],(int)p_sensor_state->gyro[2] );
      LOG_PRINT( fastlog );

      FltToStr(TO_DEG(p_sensor_state->roll),fastlog);  LOG_PRINT( fastlog );LOG_PRINT(",");
      FltToStr(TO_DEG(p_sensor_state->pitch),fastlog); LOG_PRINT( fastlog );LOG_PRINT(",");
      FltToStr(TO_DEG(p_sensor_state->yaw),fastlog);   LOG_PRINT( fastlog );
      LOG_PRINTLN("");
      break;
    case 5:
    	//sprintf(fastlog,"G_ave: %.4f %.4f %.4f G_std: %.4f %.4f %.4f\n",
    	//	p_dcm_state->gyro_ave[0],p_dcm_state->gyro_ave[1],p_dcm_state->gyro_ave[2],
    	//	p_dcm_state->gyro_std[0],p_dcm_state->gyro_std[1],p_dcm_state->gyro_std[2]  );
    default:
    	break;
  }
} /* End Debug_LogOut */

/*************************************************
** Cal_LogOut
** This function just prints a standard string
** to the log_port serial port.
** It is designed to assist in the calibration
** of the sensor
*/
void Cal_LogOut(void)
{
  /*
  String imuLog = "";
  imuLog += "Time:" + String( p_control->timestamp ) + ", ";
  imuLog += "DT:" + String( p_control->G_Dt,3 ) + ", ";
  imuLog += "SR:" + String( (1/p_control->G_Dt) ) + ", ";

  switch ( p_calibration->output_mode )
  {
    case 0:
      imuLog += "accel (min/ave/max): ";
      imuLog += String(p_calibration->accel_min[0],3) + "/" + String(p_calibration->accel_total[0]/p_calibration->N,3) + "/" + String(p_calibration->accel_max[0],3) + ", ";
      imuLog += String(p_calibration->accel_min[1],3) + "/" + String(p_calibration->accel_total[1]/p_calibration->N,3) + "/" + String(p_calibration->accel_max[1],3) + ", ";
      imuLog += String(p_calibration->accel_min[2],3) + "/" + String(p_calibration->accel_total[2]/p_calibration->N,3) + "/" + String(p_calibration->accel_max[2],3);
      break;
    case 1:
      imuLog += "gyro (ave/current): ";
      imuLog += String(p_calibration->gyro_total[0]/p_calibration->N,3)  + "/" + String( p_sensor_state->gyro[0],3 ) + ", ";
      imuLog += String(p_calibration->gyro_total[1]/p_calibration->N,3)  + "/" + String( p_sensor_state->gyro[1],3 ) + ", ";
      imuLog += String(p_calibration->gyro_total[2]/p_calibration->N,3)  + "/" + String( p_sensor_state->gyro[2],3 );
      break;
  }
  imuLog += "\r\n";
  LOG_PRINT( imuLog );
  */
} /* End Cal_LogOut */




/*************************************************
** FUNCTION: FltToStr
** VARIABLES:
**    [I ]  float value
**    [IO]  char *StrBuffer
** RETURN:
**    NONE
** DESCRIPTION:
**    This function converts a floating point
**    number into a string. This is needed to 
**    support logging floats in Arduino.
*/
void FltToStr( float value,
               char *StrBuffer )
{
  sprintf(StrBuffer, "%4d.%03d", (int)value, abs((int)(value*1000)%1000) );
}


