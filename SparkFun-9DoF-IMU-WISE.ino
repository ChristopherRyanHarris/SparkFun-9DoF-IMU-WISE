
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
  /* Initialize the hardware */
  Init_Hardware();

  /* Initialize the IMU */
  if ( !Init_IMU() ) 
  {
    LOG_PRINTLN("Error connecting to IMU");
    while(1){ }  // Loop forever if we fail to connect
  }
  LOG_PRINTLN("> IMU Initialized");
  delay(2000);
  
  /* Set the initial roll/pitch/yaw from 
  ** initial accel/gyro */
  Read_Sensors();
  Reset_Sensor_Fusion(); 
  
  if( CALIBRATE_MODE ) { Calibration_Init(); }

  DCM_Init();
  WISE_Init();
  //DSP_Filter_Init();
  
  LOG_PRINTLN("> IMU Setup Done");
} /* End setup */

// Main loop
void loop()
{ 
  /* Update sensor readings */
  Read_Sensors();
  
  /* Apply Freq Filter to Input */
  //FIR_Filter();
  //IIR_Filter();
  //DSP_Shift();

  //if( CALIBRATE_MODE ) { Calibrate(); }
  
  /* Apply the DCM Filter */
  Update_Time();
  DCM_Filter();
  //Reset_Sensor_Fusion(); 
  
  /* Estimate Walking Speed and Incline */
  WISE_Update();

  /* Read/Respond to command */
  if( COMM_PORT.available() > 0 ) { f_RespondToInput( COMM_PORT.available() );  }

  /* Blink LED 
  ** TO DO: It would be nice to have a blink code
  **        to communicate during operation */
  Blink_LED();
} /* End loop */








