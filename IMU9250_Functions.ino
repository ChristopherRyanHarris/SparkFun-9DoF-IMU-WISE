
/*******************************************************************
** FILE: 
**   	IMU9250_Functions
** DESCRIPTION:
** 		This file contains some MPU 9250 (HW specific)
** 		functions. Specifically, for initializing and 
** 		reading the sensor registeres
**		These functions cannot be used in emulation mode.
**		Furhter, these functions can only be used for the IMU9250
**		platform.
********************************************************************/


/*************************************************
** Notes on orientation for the 9250 IMU
**   Terms:
**     Fore:       (Front) Edge of the USB port
**     Aft:        (Rear) Edge oposite of the USB port
**     Starboard:  (Right) Edge oposite of PWR switch
**     Port:       (Left) Edge with PWR switch
**     Zenith:     (Up) face with USB port
**     Nadir:      (Down) face oposite USB port
**   Contrary to the silk, the axis are positioned as follows:
**     +x is Fore,       -x is Aft
**     +y is Starboard,  -y is Port
**     +z is Zenith,     -z is Nadir
**   This means, placing the board on a flat surface with the
**   face without the USB port (Nadir) down will result in an acceleration
**   of about -2000 (1xg) for accel[2] (z) since the acceleration
**   from gravity with be acting along -z.
**   Accel: [ x  y  z ]
**   Gyro:  [-x -y -z ] (RH rule)
**************************************************/


/*******************************************************************
** Includes ********************************************************
********************************************************************/

#ifndef COMMON_CONFIG_H
	#include "../Include/Common_Config.h"
#endif

/* Only link if using IMU9250 */
#ifdef _IMU9250_

/*******************************************************************
** Functions *******************************************************
********************************************************************/


/*************************************************
** FUNCTION: Init_IMU
** VARIABLES:
**		[I ]	CONTROL_TYPE 			*p_control
**    [IO]  SENSOR_STATE_TYPE *p_sensor_state
** RETURN:
**		BOOL	1:Successful initialization
**					0:Failure
** DESCRIPTION: 
** 		This function set the IMU parameters
** 		This includes things like the SR and the 
** 		internal LPF corner. 
*/
bool Init_IMU( CONTROL_TYPE	      *p_control,
               SENSOR_STATE_TYPE  *p_sensor_state )
{
	unsigned char activate_sensors = 0;
	
	LOG_PRINTLN("> Initializing IMU9250");
	
  /* Set up MPU-9250 interrupt input (active-low) */
  pinMode(MPU9250_INT_PIN, INPUT_PULLUP);
  
  /* imu.begin() should return 0 on success. Will initialize
  ** I2C bus, and reset MPU-9250 to defaults */
  if (imu.begin() != INV_SUCCESS) { return FALSE; }

  /* Initiate accel and gyro sensors only */
  
  /* Turn on or off MPU-9250 sensors. Any of the following defines can be combined: \
  ** 		INV_XYZ_GYRO, INV_X_GYRO, INV_Y_GYRO, or INV_Z_GYRO, 
  **		INV_XYZ_ACCEL, or INV_XYZ_COMPASS
	** Input: Combination of enabled sensors. 
	** Unless specified a sensor will be disabled.
	** Output: INV_SUCCESS (0) on success, otherwise error */
	#if ACCEL_ON==1
		activate_sensors = activate_sensors | INV_XYZ_ACCEL;
	#endif
	#if GYRO_ON==1
		activate_sensors = activate_sensors | INV_XYZ_GYRO;
	#endif
	#if MAGN_ON==1
		activate_sensors = activate_sensors | INV_XYZ_COMPASS;
	#endif
  imu.setSensors( activate_sensors );
  imu.setSensors( INV_XYZ_ACCEL | INV_XYZ_GYRO );

  /* Configure sensors: */
  imu.setGyroFSR( IMU_GYRO_FSR );
  imu.setAccelFSR( IMU_ACCEL_FSR );
  imu.setLPF( IMU_AG_LPF );
  imu.setSampleRate( IMU_AG_SAMPLE_RATE );
  
  return TRUE;
} /* End Init_IMU */

/*************************************************
** FUNCTION: Read_Sensors
** VARIABLES:
**		[I ]	CONTROL_TYPE 			*p_control
**		[IO]	SENSOR_STATE_TYPE *p_sensor_state
** RETURN:
**		NONE
** DESCRIPTION: 
** 		This function reads the sensor registers and
** 		assigns them to the global input vectors
*/
void Read_Sensors( CONTROL_TYPE				*p_control,
								   SENSOR_STATE_TYPE	*p_sensor_state )
{
  int i;
  
  /* Read the Accelerometer */
  #if ACCEL_ON==1
  	imu.updateAccel();
  	p_sensor_state->accel[0] = (float)imu.ax;
  	p_sensor_state->accel[1] = (float)imu.ay;
  	p_sensor_state->accel[2] = (float)imu.az;
  #endif 
  
 	/* Read the Gyroscope */
  #if GYRO_ON==1
  	imu.updateGyro();
  	p_sensor_state->gyro[0] = (float)imu.gx;
  	p_sensor_state->gyro[1] = (float)imu.gy;
  	p_sensor_state->gyro[2] = (float)imu.gz;
  #endif 
  
  /* Read the Magnometer */
  #if MAGN_ON==1
  #endif
  
} /* End Read_Sensors */

#endif /* End _IMU9250_ */




