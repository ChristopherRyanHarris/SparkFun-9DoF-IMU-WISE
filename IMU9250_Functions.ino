
/*************************************************
** FILE: MPU9250_Functions
** This file contains some MPU 9250 (HW specific)
** functions. Specifically, for initializing and 
** reading the sensor registeres
**************************************************/

#ifdef _IMU9250_

/*******************************************************************
** Functions *******************************************************
********************************************************************/


/*************************************************
** Read_Sensors 
** This function reads the sensor registers and
** assigns them to the global input vectors
*/
void Read_Sensors()
{
  int i;
  
  /* Set the initial accel and gyro vectors */
  imu.updateAccel();
  g_sensor_state.accel[0] = imu.ax;
  g_sensor_state.accel[1] = imu.ay;
  g_sensor_state.accel[2] = imu.az;
  imu.updateGyro();
  g_sensor_state.gyro[0] = imu.gx;
  g_sensor_state.gyro[1] = imu.gy;
  g_sensor_state.gyro[2] = imu.gz;
  
}


/*************************************************
** Init_IMU
** This function set the IMU parameters
** This includes things like the SR and the 
** internal LPF corner. 
*/
bool Init_IMU(void)
{
	
	LOG_PRINTLN("> Initializing IMU9250");
	
  /* Set up MPU-9250 interrupt input (active-low) */
  pinMode(MPU9250_INT_PIN, INPUT_PULLUP);
  
  /* imu.begin() should return 0 on success. Will initialize
  ** I2C bus, and reset MPU-9250 to defaults */
  if (imu.begin() != INV_SUCCESS) { return false; }

  /* Initiate accel and gyro sensors only */
  imu.setSensors(INV_XYZ_ACCEL | INV_XYZ_GYRO);

  /* Configure sensors: */
  imu.setGyroFSR( IMU_GYRO_FSR );
  imu.setAccelFSR( IMU_ACCEL_FSR );
  imu.setLPF( IMU_AG_LPF );
  imu.setSampleRate( IMU_AG_SAMPLE_RATE );
  return true; // Return success
}

#endif /* #ifdef _IMU9250_ */




