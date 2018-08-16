
/*******************************************************************
** FILE: 
**    SparkFun-9DoF-IMU-WISE
** DESCRIPTION:
**    This is the calling executable for the real-time
**    Sparkfun IMU code.
**    To use, this code must be embedded into a compatible IMU and the 
**    appropriate header files must be referenced.
**    See the common header file for more specific information.
********************************************************************/


/*******************************************************************
** Includes ********************************************************
********************************************************************/

#include <SD.h>
#include "./Include/Common_Config.h"

#include <Wire.h>
#include <string.h>
#include <math.h>

#ifdef _IMU10736_
  #include "./Include/IMU10736_Config.h"
#endif

#ifdef _IMU9250_
  #include <SparkFunMPU9250-DMP.h>
  #include "./Include/IMU9250_Config.h"
#endif


/*******************************************************************
** Globals *********************************************************
********************************************************************/

#ifdef _IMU9250_
  MPU9250_DMP imu; 
#endif

/* FES Structure
** This is specific to the FES test */
FES_TEST_TYPE     g_fes_state;

/* Control Structure
** This will contain all the various settings */
CONTROL_TYPE        g_control;

/* Input data structure
** This will contain the input data
** In emulation mode, this is read from a binary file
** In real-time mode, this will be from the sensors. */
SENSOR_STATE_TYPE   g_sensor_state;

/* Calibration Structure
** This structure is used to aid in calibrating the sensor
** This is not used in normal processing */
CALIBRATION_TYPE  g_calibration;


/* DSP state
** The Digital Signal Processing algorithms
** are filters which are applied to the individual
** data feeds. This structure holds the state variables
** for the algorithm.
** NOTE: At the moment, this is a very simple FIR and IIR
**       filter set. Could easily be expanded. */
DSP_STATE_TYPE   g_dsp;


/* DCM variables
** The Directional cosine matrix is one
** method of determining the orientation of the
** IMU. This matrix holds the state information of
** the DCM algorithm. */
DCM_STATE_TYPE      g_dcm_state;


/* GaPA state
** The Gait Phase Angle estimator is an algorithm
** (or set of algorithms) which will determine the
** current gait phase angle of the user. This
** structure holds the state variables of the algorithm */
GAPA_STATE_TYPE   g_gapa_state;


/* WISE state
** The Walking Incline and Speed Estimator is
** an algorithm (or set of algorithms) which
** estimates the walking speed and the incline
** of motion of the user. This structure holds
** the state variables for the algorithm. */
WISE_STATE_TYPE   g_wise_state;


/*******************************************************************
** START ***********************************************************
********************************************************************/


/*************************************************
** FUNCTION: setup
** VARIABLES:
**    NONE
** RETURN:
**    NONE
** DESCRIPTION:
**    This function contains the setup functions 
**    including the initialization of the hardware
**    and the initialization of the serial ports
*/
void setup( void )
{
  bool ret;
  CONTROL_TYPE *p_control = &g_control;
  
  /* Initialize the hardware */
  Init_Hardware( &g_control );
  
  /* Initialize the control structure */
  Common_Init( &g_control, &g_sensor_state );
  
  /* Initialize the IMU sensors*/
  ret = Init_IMU( &g_control, &g_sensor_state );
  if ( ret==0 ) 
  {
    LOG_INFO( "ERROR : Setup : Cant Connect to IMU" );
    while(1){}
  }
  
  /* Write the meta header */
  Meta_LogOut( &g_control, &g_sensor_state, &g_gapa_state, &g_wise_state, &g_fes_state );
  
  /* Set the initial roll/pitch/yaw from 
  ** initial accel/gyro */
  
  /* Read all active sensors */
  Read_Sensors( &g_control, &g_sensor_state );
  
  /* Initialize Freq. Filter */
  if( g_control.DSP_on==1 ){ DSP_Filter_Init( &g_control, &g_dsp ); }

  /* Initialize calibration parameters */
  if( g_control.calibration_on==1 ){ Calibration_Init( &g_control, &g_calibration ); }

  /* Initialize the Directional Cosine Matrix algorithm parameters */
  if( g_control.DCM_on==1 ){ DCM_Init( &g_control, &g_dcm_state, &g_sensor_state ); }

  /* Initialize GaPA parameters */
  if( g_control.GaPA_on==1 ){ GaPA_Init( &g_control, &g_gapa_state ); }

  /* Initialize Walking Incline and Speed Estimator */
  if( g_control.WISE_on==1 ){ WISE_Init( &g_control, &g_sensor_state, &g_wise_state ); }
    
  
  /* Initialize the FES state 
  ** Set both relays low (open) */
  FES_Init( &g_control, &g_fes_state );
  
  LOG_INFO( "> IMU Setup Done" );
  
} /* End setup */


/*************************************************
** FUNCTION: loop
** VARIABLES:
**    NONE
** RETURN:
**    NONE
** DESCRIPTION:
**    In the case of IMU real-time execution, 
**    this is the main loop for the executable.
**    It loops while there is power.    
*/
void loop( void )
{
  
  /* Update sensor readings */
  Read_Sensors( &g_control, &g_sensor_state );
  
  /* Update the timestamp */
  Update_Time( &g_control );
  
  /* If in calibration mode,
  ** call calibration function */
  if( g_control.calibration_on==1 ){ Calibrate( &g_control, &g_calibration, &g_sensor_state ); }

  /* Apply Freq Filter to Input */
  if( g_control.DSP_on==1 )
  {
    if( g_control.dsp_prms.IIR_on==1 ){ FIR_Filter( &g_control, &g_dsp, &g_sensor_state ); }
    if( g_control.dsp_prms.IIR_on==1 ){ IIR_Filter( &g_control, &g_dsp, &g_sensor_state ); }
    DSP_Shift( &g_control, &g_dsp );
  }

  /* Apply the DCM Filter */
  if( g_control.DCM_on==1 ){ DCM_Filter( &g_control, &g_dcm_state, &g_sensor_state ); } /* cost: 70 sam/s */

  /* Estimate the Gait Phase Angle */
  if( g_control.GaPA_on==1 ){ GaPA_Update( &g_control, &g_sensor_state, &g_gapa_state ); } /* cost: 20 sam/s */

  /* Estimate Walking Speed and Incline */
  if( g_control.WISE_on==1 ){ if( (g_sensor_state.gyro.mag_mave<g_control.gapa_prms.min_gyro) ){ WISE_Update(&g_control, &g_sensor_state, &g_wise_state ); } }
  
  /* ***************** **
  ** FES Test Specific **
  ** ***************** */
  FES_Update ( &g_control, &g_fes_state, &g_sensor_state, &g_dcm_state, &g_gapa_state, &g_wise_state ); /* cost: 45 sam/s */
    
  
  /* Read/Respond to command */
  if( SERIAL_AVAILABLE>0 ){ f_RespondToInput( &g_control, &g_sensor_state, &g_calibration, SERIAL_AVAILABLE ); }

  /* We blink every LOG_INFO_RATE millisecods */
  if ( micros()>(g_control.LastLogTime+DATA_LOG_RATE) )
  {
    /* Log the current states to the debug port */
    /* Debug_LogOut sends values to Arduino Serial Monitor */
    /* Data_LogOut sends values to SD Card (if available) */
    Debug_LogOut( &g_control, &g_sensor_state, &g_gapa_state, &g_wise_state, &g_fes_state ); /* sd vs uart : cost ~100 sam/s */
    //Data_LogOut( &g_control, &g_sensor_state, &g_gapa_state, &g_wise_state, &g_fes_state ); /* cost: ~20 sam/s */
    
    g_control.LastLogTime = micros();

    /* Display number of bytes available on comm port
    ** Com port is used for real-time communication with
    ** connected processor */
    //LOG_PORT.println("> # Available on SERIAL_PORT: " + String(SERIAL_PORT.available()) );
  }

  /* Blink LED 
  ** TO DO: It would be nice to have a blink code
  **        to communicate during operation */
  Blink_LED( &g_control );
} /* End loop */
