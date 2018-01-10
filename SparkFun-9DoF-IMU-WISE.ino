
/*************************************************
** FILE: SparkFun-9DoF-IMU-WISE
** This is the calling executable for the real-time
** Sparkfun IMU. This specific implementation is 
** designed for the WISE (Walking Incline and Speed
** Estimator) algorithm variant. To use, this code 
** must be embedded into a compatible IMU and the 
** appropriate header files must be referenced.
*/


/*******************************************************************
** Includes ********************************************************
********************************************************************/

#include "./Include/Common_Config.h"

//#include <stdint.h>
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

#include "./Include/DSP_Config.h"
#include "./Include/WISE_Config.h"
#include "./Include/Math.h"


/*******************************************************************
** Globals *********************************************************
********************************************************************/

#ifdef _IMU9250_
MPU9250_DMP imu; 
#endif

/* DCM variables */
CAL_STATE_TYPE      g_calibration;
DCM_STATE_TYPE      g_dcm_state;
SENSOR_STATE_TYPE   g_sensor_state;
CONTROL_STATE_TYPE  g_control_state;
WISE_STATE_TYPE     g_wise_state;
GAPA_STATE_TYPE     g_gapa_state;
DSP_COMMON_TYPE     g_dsp;

/*******************************************************************
** START ***********************************************************
********************************************************************/


/*************************************************
** Setup Function 
** This function contains the setup functions 
** including the initialization of the hardware
** and the initialization of the serial ports
*/
void setup()
{
  Common_Init();
  
  /* Initialize the hardware */
  Init_Hardware();

  /* Initialize the IMU */
  if ( !Init_IMU() ) 
  {
    LOG_PRINTLN("Error connecting to IMU");
    while(1){ }
  }
  delay(2000);
  
  /* Set the initial roll/pitch/yaw from 
  ** initial accel/gyro */
  Read_Sensors();
  Reset_Sensor_Fusion(); 
  
  #if( CALIBRATE_MODE==1 )
  Calibration_Init();
  #endif

  DCM_Init();
  GaPA_Init();

  #if( DSP_ON==1 )
  DSP_Filter_Init();
  #endif
  
  #if( WISE_ON==1 )
  WISE_Init();
  #endif
  
  LOG_PRINTLN("> IMU Setup Done");
  
} /* End setup */

// Main loop
void loop()
{ 
  /* Update sensor readings */
  Read_Sensors();
  
//  /* Apply Freq Filter to Input */
//  #if( DSP_ON==1 )
//  FIR_Filter();
//  IIR_Filter();
//  DSP_Shift();
//  #endif

//  #if( CALIBRATE_MODE==1 )
//  Calibrate();
//  #endif
  
  /* Apply the DCM Filter */
  Update_Time();
  DCM_Filter();
  GaPA_Update();
  
  /* Estimate Walking Speed and Incline */
//  #if( WISE_ON==1 )
//  if( ((g_dcm_state.gyro_std[0]+g_dcm_state.gyro_std[1]+g_dcm_state.gyro_std[2])/3 > MOVE_MIN_GYRO_STD) )
//	{
//		WISE_Update();
//	}
//	#endif
    
  /* Read/Respond to command */
//  if( COMM_PORT.available()>0 ) { f_RespondToInput( COMM_PORT.available() );  }

  /* We blink every UART_LOG_RATE millisecods */
//  if ( micros()>(g_control_state.g_LastLogTime+UART_LOG_RATE) )
//  {
  	/* Log the current states to the debug port */
    Debug_LogOut();
    
//    g_control_state.g_LastLogTime = micros();
//
//    /* Display number of bytes available on comm port
//    ** Com port is used for real-time communication with
//    ** connected processor */
//    //LOG_PORT.println("> # Available on COMM_PORT: " + String(COMM_PORT.available()) );
//  }

  /* Blink LED 
  ** TO DO: It would be nice to have a blink code
  **        to communicate during operation */
  Blink_LED();
} /* End loop */








