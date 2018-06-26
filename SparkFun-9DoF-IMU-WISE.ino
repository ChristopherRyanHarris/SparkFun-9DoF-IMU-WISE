
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
FES_TEST_TYPE g_fes_state;

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
    LOG_PRINT( "ERROR : Setup : Cant Connect to IMU" );
    while(1){}
  }
  
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
  g_fes_state.relays_on 						   = FALSE;
  g_fes_state.relay_start_micros      = 0;
  g_fes_state.relay_start_iteration   = 0;
  g_fes_state.foot_sensor_1_val_ave   = 0;
  g_fes_state.foot_sensor_2_val_ave   = 0;
  g_fes_state.foot_sensor_1_volts_ave = 0;
  g_fes_state.foot_sensor_2_volts_ave = 0;
	RELAY_1_SET_LOW;
	RELAY_2_SET_LOW;
  
  
  UART_LOG( "> IMU Setup Done" );
  
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
  if( g_control.DCM_on==1 ){ DCM_Filter( &g_control, &g_dcm_state, &g_sensor_state ); }

  /* Estimate the Gait Phase Angle */
  if( g_control.GaPA_on==1 ){ GaPA_Update( &g_control, &g_sensor_state, &g_gapa_state ); }

  /* Estimate Walking Speed and Incline */
  if( g_control.WISE_on==1 )
  {
    if( (g_sensor_state.gyro_mAve<g_control.gapa_prms.min_gyro) )
    {
      WISE_Update(&g_control, &g_sensor_state, &g_wise_state );
    }
  }
  
  /* ***************** **
  ** FES Test Specific **
  ** ***************** */
  /* Read foot switch values */
  g_fes_state.foot_sensor_1_val       = FOOT_SENSOR_1_VAL;
  g_fes_state.foot_sensor_2_val       = FOOT_SENSOR_2_VAL;
  g_fes_state.foot_sensor_1_val_ave   = Windowed_Mean( g_fes_state.foot_sensor_1_val_ave, (float)g_fes_state.foot_sensor_1_val, g_control.SampleNumber, FOOT_SENSOR_1_VAL_ALPHA );
  g_fes_state.foot_sensor_2_val_ave   = Windowed_Mean( g_fes_state.foot_sensor_2_val_ave, (float)g_fes_state.foot_sensor_2_val, g_control.SampleNumber, FOOT_SENSOR_2_VAL_ALPHA );
  g_fes_state.foot_sensor_1_volts     = FOOT_SENSOR_1_VOLTS;
  g_fes_state.foot_sensor_2_volts     = FOOT_SENSOR_2_VOLTS;
  g_fes_state.foot_sensor_1_volts_ave = Windowed_Mean( g_fes_state.foot_sensor_1_volts_ave, g_fes_state.foot_sensor_1_volts, g_control.SampleNumber, FOOT_SENSOR_1_VOLTS_ALPHA );
  g_fes_state.foot_sensor_2_volts_ave = Windowed_Mean( g_fes_state.foot_sensor_2_volts_ave, g_fes_state.foot_sensor_2_volts, g_control.SampleNumber, FOOT_SENSOR_2_VOLTS_ALPHA );
  /* At given event, trigger relay */
	if( PHASE_ANGLE_SWITCH_ON_EVENT )
	{
		/* Relay : On (High), update state variables */
		if( g_fes_state.relays_on==FALSE )
		{
			/* Update current relay state */
			g_fes_state.relays_on = TRUE;
			
			/* Set both relays on */
			RELAY_1_SET_HIGH;
			RELAY_2_SET_HIGH;
			
			/* Record relay on start time variable */
			g_fes_state.relay_start_micros = micros();
			/* Record relay on start iteration variable */
			g_fes_state.relay_start_iteration = g_control.SampleNumber;
		}
		/* Update time up counter */
		g_fes_state.relay_on_micros     = g_fes_state.relay_start_micros - micros();
		/* Update iterations up counter */
		g_fes_state.relay_on_iterations = g_fes_state.relay_start_iteration - g_control.SampleNumber;
	}
	else
	{
		/* Relay : Off (Low), reset state variables */
		if( g_fes_state.relays_on==TRUE )
		{
			/* Update current relay state */
			g_fes_state.relays_on = FALSE;
			
			/* Set both relays off */
			RELAY_1_SET_LOW;
			RELAY_2_SET_LOW;
			
			/* NOTE:
			** By maxing the counter, we ensure we only 
			** trigger the relay on at the desired moment 
			** See the header file for more info */
			
			/* Reset recorded start time variable */
			g_fes_state.relay_start_micros    = -1;
			/* Reset recorded start iteration variable */
			g_fes_state.relay_start_iteration = -1;
			
			/* Reset time up counter */
			g_fes_state.relay_on_micros       = -1;
			/* Reset iterations up counter */
			g_fes_state.relay_on_iterations   = -1;
		}
	}
  		
  		
  /* Read/Respond to command */
  if( SERIAL_AVAILABLE>0 )
  {
    f_RespondToInput( &g_control, &g_sensor_state, &g_calibration, SERIAL_AVAILABLE );  
  }

  /* We blink every UART_LOG_RATE millisecods */
  if ( micros()>(g_control.LastLogTime+DATA_LOG_RATE) )
  {
    /* Log the current states to the debug port */
    Debug_LogOut( &g_control, &g_sensor_state, &g_gapa_state, &g_wise_state );
    
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






