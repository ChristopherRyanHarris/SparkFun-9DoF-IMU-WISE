
/*******************************************************************
** FILE:
**    Logging_Functions
** DESCRIPTION:
**    This file contains the logging functions.
**    These functions are used exclusively in debug mode
**    and should not be included in the final firmware
**    implementation.
**    These functions are inteded for emulation
**    or debug modes only.
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
  **       prototypes for all execution functions */
  #include "../Include/Emulator_Protos.h"
#endif  /* End Emulator Mode */

/*******************************************************************
** Functions *******************************************************
********************************************************************/


/*************************************************
** FUNCTION: Debug_LogOut
** VARIABLES:
**    [I ]  CONTROL_TYPE      *p_control
**    [I ]  SENSOR_STATE_TYPE *p_sensor_state
**    [I ]  WISE_STATE_TYPE   *p_wise_state
** RETURN:
**    NONE
** DESCRIPTION:
**    This function just prints a standard string
**    to the log_port serial port.
**    It prints the rpy as well as the timestamp and
**    and estimate of the sample rate
*/
void Debug_LogOut( CONTROL_TYPE       *p_control,
                   SENSOR_STATE_TYPE  *p_sensor_state,
                   GAPA_STATE_TYPE    *p_gapa_state,
                   WISE_STATE_TYPE    *p_wise_state )
{
  char LogBuffer[MAX_LINE_LEN];
  char tmpBuffer[MAX_LINE_LEN];

  switch ( p_control->output_mode )
  {
    case 1:
      sprintf( LogBuffer, "T:%09lu", p_control->timestamp );

      strcat( LogBuffer, ", DT:" );
      FltToStr( p_control->G_Dt, 4, tmpBuffer );
      strcat( LogBuffer, tmpBuffer );

      strcat( LogBuffer, ", SR:" );
      FltToStr( 1/p_control->G_Dt, 4, tmpBuffer );
      strcat( LogBuffer, tmpBuffer );

      strcat( LogBuffer, ", R:" );
      FltToStr( TO_DEG(p_sensor_state->roll), 4, tmpBuffer );
      strcat( LogBuffer, tmpBuffer );

      strcat( LogBuffer, ", P:" );
      FltToStr( TO_DEG(p_sensor_state->pitch), 4, tmpBuffer );
      strcat( LogBuffer, tmpBuffer );

      strcat( LogBuffer, ", Y:" );
      FltToStr( TO_DEG(p_sensor_state->yaw), 4, tmpBuffer );
      strcat( LogBuffer, tmpBuffer );

      sprintf(tmpBuffer,", A:%06d,%06d,%06d", (int)p_sensor_state->accel[0], (int)p_sensor_state->accel[1], (int)p_sensor_state->accel[2] );
      strcat( LogBuffer, tmpBuffer );

      sprintf(tmpBuffer,", G:%07d,%07d,%07d", (int)p_sensor_state->gyro[0],  (int)p_sensor_state->gyro[1],  (int)p_sensor_state->gyro[2] );
      strcat( LogBuffer, tmpBuffer );

      strcat( LogBuffer, ", PA(N):" );
      FltToStr( p_gapa_state->nu_normalized, 4, tmpBuffer );
      strcat( LogBuffer, tmpBuffer );
      break;

    case 2:
      sprintf( LogBuffer, "%09lu", p_control->timestamp );

      sprintf( tmpBuffer, ",%d,%d,%d", (int)p_sensor_state->accel[0], (int)p_sensor_state->accel[1], (int)p_sensor_state->accel[2] );
      strcat( LogBuffer, tmpBuffer );

      sprintf( tmpBuffer,",%d,%d,%d",  (int)p_sensor_state->gyro[0],  (int)p_sensor_state->gyro[1],  (int)p_sensor_state->gyro[2] );
      strcat( LogBuffer, tmpBuffer );

      strcat( LogBuffer, "," );
      FltToStr( TO_DEG(p_sensor_state->roll), 3, tmpBuffer);
      strcat( LogBuffer, tmpBuffer );

      strcat( LogBuffer, "," );
      FltToStr( TO_DEG(p_sensor_state->pitch), 3, tmpBuffer);
      strcat( LogBuffer, tmpBuffer );

      strcat( LogBuffer, "," );
      FltToStr( TO_DEG(p_sensor_state->yaw), 3, tmpBuffer);
      strcat( LogBuffer, tmpBuffer );
      break;

    default:
      break;
  }

  LOG_INFO(LogBuffer);
} /* End Debug_LogOut */


/*************************************************
** FUNCTION: Cal_LogOut
** VARIABLES:
**    [I ]  SENSOR_STATE_TYPE *p_sensor_state
**    [I ]  CALIBRATION_TYPE  *p_calibration
** RETURN:
**    NONE
** DESCRIPTION:
**    This function just prints a standard string
**    to the log_port serial port.
**    It is designed to assist in the calibration
**    of the sensor
*/
void Cal_LogOut( CONTROL_TYPE      *p_control,
                 SENSOR_STATE_TYPE *p_sensor_state,
                 CALIBRATION_TYPE  *p_calibration )
{
  char LogBuffer[MAX_LINE_LEN];
  char tmpBuffer[MAX_LINE_LEN];


  sprintf( LogBuffer, "TIME: %09lu", p_control->timestamp );

  strcat( LogBuffer, ", DT: " );
  FltToStr( p_control->G_Dt, 4, tmpBuffer );
  strcat( LogBuffer, tmpBuffer );

  strcat( LogBuffer, ", SR: " );
  FltToStr( 1/p_control->G_Dt, 4, tmpBuffer );
  strcat( LogBuffer, tmpBuffer );

  switch ( p_control->calibration_prms.output_mode )
  {
    case 0:
      strcat( LogBuffer, ", accel (min/ave/max): " );

      strcat( LogBuffer, "[a1](" );
      FltToStr( p_calibration->accel_min[0], 4, tmpBuffer );
      strcat( LogBuffer, tmpBuffer );

      strcat( LogBuffer, "/" );
      FltToStr( p_calibration->accel_total[0]/p_calibration->N, 4, tmpBuffer );
      strcat( LogBuffer, tmpBuffer );

      strcat( LogBuffer, "/" );
      FltToStr( p_calibration->accel_max[0], 4, tmpBuffer );
      strcat( LogBuffer, tmpBuffer );
      strcat( LogBuffer, "), " );


      strcat( LogBuffer, "[a2](" );
      FltToStr( p_calibration->accel_min[1], 4, tmpBuffer );
      strcat( LogBuffer, tmpBuffer );

      strcat( LogBuffer, "/" );
      FltToStr( p_calibration->accel_total[1]/p_calibration->N, 4, tmpBuffer );
      strcat( LogBuffer, tmpBuffer );

      strcat( LogBuffer, "/" );
      FltToStr( p_calibration->accel_max[1], 4, tmpBuffer );
      strcat( LogBuffer, tmpBuffer );
      strcat( LogBuffer, "), " );


      strcat( LogBuffer, "[a3](" );
      FltToStr( p_calibration->accel_min[2], 4, tmpBuffer) ;
      strcat( LogBuffer, tmpBuffer );

      strcat( LogBuffer, "/" );
      FltToStr( p_calibration->accel_total[2]/p_calibration->N, 4, tmpBuffer);
      strcat( LogBuffer, tmpBuffer );

      strcat( LogBuffer, "/" );
      FltToStr( p_calibration->accel_max[2], 4, tmpBuffer );
      strcat( LogBuffer, tmpBuffer );
      strcat( LogBuffer, ")" );
      break;

    case 1:
      strcat( LogBuffer, ", gyro (ave/current): " );

      strcat( LogBuffer, "[g1](" );
      FltToStr( p_calibration->gyro_total[0]/p_calibration->N, 4, tmpBuffer );
      strcat( LogBuffer, tmpBuffer );

      strcat( LogBuffer, "/" );
      FltToStr( p_sensor_state->gyro[0], 4, tmpBuffer );
      strcat( LogBuffer, tmpBuffer );
      strcat( LogBuffer, "), " );

      strcat( LogBuffer, "[g2](" );
      FltToStr( p_calibration->gyro_total[1]/p_calibration->N, 4, tmpBuffer);
      strcat( LogBuffer, tmpBuffer );

      strcat( LogBuffer, "/" );
      FltToStr( p_sensor_state->gyro[1], 4, tmpBuffer);
      strcat( LogBuffer, tmpBuffer );
      strcat( LogBuffer, "), " );

      strcat( LogBuffer, "[g3](" );
      FltToStr( p_calibration->gyro_total[2]/p_calibration->N, 4, tmpBuffer );
      strcat( LogBuffer, tmpBuffer );

      strcat( LogBuffer, "/" );
      FltToStr( p_sensor_state->gyro[2], 4, tmpBuffer );
      strcat( LogBuffer, tmpBuffer );
      strcat( LogBuffer, ")" );
      break;
  }

  LOG_INFO(LogBuffer);
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

/*************************************************
** FUNCTION: LogToFile
** VARIABLES:
**    [I ]  CONTROL_TYPE         *p_control
**    [I ]  OUTPUT_LOG_FILE_TYPE *log_file,
**    [I ]  char*                 message
** RETURN:
**    void
** DESCRIPTION:
**    This is a helper function.
**    It is used for logging to a file (SD or local).
**    This function will add data to the output buffers
**    and (when the buffer has reached the defined length)
**    write the buffer to the appropriate output file.
*/
void LogToFile( CONTROL_TYPE         *p_control,
                OUTPUT_LOG_FILE_TYPE *log_file,
                char*                 msg  )
{
  long int size_bytes;

  
  if( log_file->type==1 ) /* type 0:txt */
  {
    LOG_INFO( " HERE2 : size : %d", log_file->size );
  }
  
  /* If logging to file is disabled,
  ** send to default out (wither uart or stdout)
  ** Only works with txt logging */
  if( log_file->enabled==FALSE )
  {
    if( log_file->type==0 ) /* type 0:txt */
    {
      LOG_INFO_DEFAULT( msg );
    }
    return;
  }

  /* Add message to output buffer */
  if( log_file->type==0 ) /* type 0:txt */
  {
    log_file->LogBufferLen += strlen( msg );
    strcat( log_file->LogBuffer, msg );
  }
  else
  {
    LOG_INFO( " HERE3 : size : %d", log_file->size );
    memcpy( &log_file->LogBuffer[log_file->LogBufferLen], msg, log_file->size );
    log_file->LogBufferLen += log_file->size;
  }

  /* If buffer has reached designated size ...*/
  if( log_file->LogBufferLen > MAX_LOG_BUFFER_STORE )
  {
    /* If the filehandle is valid ...
    ** NOTE : File is kept open during execution
    **        First open is in common_init */
    if( log_file->LogFile_fh!=NULL )
    {
      /* Get current file size */
      size_bytes = FILE_SIZE_BYTES(log_file->LogFile_fh);

      /* If the filesize is larger than the designated
      ** max, close file and open next index */
      if( size_bytes>MAX_OUTPUT_FILE_SIZE )
      {
        if( log_file->LogFileIdx>LOG_FILE_MAX_IDX )
        {
          FILE_CLOSE( log_file->LogFile_fh );
          log_file->enabled = FALSE;
        }
        else
        {
          FILE_CLOSE( log_file->LogFile_fh );
          GetNextLogFileName( p_control, log_file );

          if( log_file->type==0 ) /* type 0:txt */
          {
            log_file->LogFile_fh = FILE_OPEN_WRITE( log_file->LogFileName );
          }
          else /* type 1:bin */
          {
            log_file->LogFile_fh = FILE_OPEN_WRITE_B( log_file->LogFileName );
          }
        }
      }
      if( log_file->LogFile_fh==NULL )
      {
        log_file->enabled        = FALSE;
        #if( EXE_MODE==0 ) /* 0 : IMU Mode */
          p_control->SDCardPresent = FALSE;
        #endif
      }
      else
      {
        if( log_file->type==0 ) /* type 0:txt */
        {
          FILE_PRINT_TO_FILE( log_file->LogFile_fh, log_file->LogBuffer );
          FILE_FLUSH( log_file->LogFile_fh );
        }
        else /* type 1:bin */
        {
          FILE_WRITE_TO_FILE( (log_file->LogBuffer), 1, log_file->LogBufferLen, log_file->LogFile_fh );
        }

        log_file->LogBuffer[0] = '\0';
        log_file->LogBufferLen = 0;
      }
    }
    else
    {
      log_file->enabled        = FALSE;
      #if( EXE_MODE==0 ) /* 0 : IMU Mode */
        p_control->SDCardPresent = FALSE;
      #endif
    } /* End if log_file->LogFile_fh!=NULL */
  } /* End if log_file->LogBufferLen > MAX_LOG_BUFFER_STORE */
} /* End LogToFile() */



/*************************************************
** FUNCTION: GetNextLogFileName
** VARIABLES:
**    [I ]  CONTROL_TYPE      *p_control
** RETURN:
**    void
** DESCRIPTION:
**    This is a helper function.
**    It is used for logging to an SD card.
**    This function creates a filename which does not
**    exist on the card, to which we will log our data.
*/
void GetNextLogFileName( CONTROL_TYPE          *p_control,
                         OUTPUT_LOG_FILE_TYPE  *log_file )
{
  int  i;

  for( i=log_file->LogFileIdx; i<LOG_FILE_MAX_IDX; i++ )
  {
    /* Construct a file with PREFIX[Index].SUFFIX */
    sprintf( log_file->LogFileName, "%s%i.%s", log_file->file_prefix, i, log_file->file_suffix );
    LOG_INFO( " > Trying File %s", log_file->LogFileName );

    /* If the file name doesn't exist, return it */
    //if( !SD.exists(log_file->LogFileName) )
    if( ! FILE_EXISTS_FLAG(log_file->LogFileName) )
    {
      LOG_INFO( " > File %s Available", log_file->LogFileName );
      log_file->LogFileIdx = i + 1;
      break;
    }
  }
} /* End GetNextLogF */
