
/*******************************************************************
** FILE: 
**    IMU10736_Functions
** DESCRIPTION:
**    This file contains some IMU 10736 (HW specific)
**    functions. Specifically, for initializing and
**    reading the sensor registeres.
**    These functions cannot be used in emulation mode.
**    Furhter, these functions can only be used for the IMU10736 
**    platform.
********************************************************************/


/*************************************************
** NOTES on orientation for the 10736 IMU
**   Terms:
**     Fore:       (Front) Edge oposite of the power port
**     Aft:        (Rear) Edge of the power port
**     Starboard:  (Right) Edge with reset switch
**     Port:       (Left) Edge oposite of reset switch
**     Zenith:     (Up) Clean face of board
**     Nadir:      (Down) Populated face of board
**   Contrary to the silk, the axis are positioned as follows:
**     +x is Fore,       -x is Aft
**     +y is Starboard,  -y is Port
**     +z is Zenith,     -z is Nadir
**   This means, placing the board on a flat surface with the
**   unpopulated side (Zenith) down will result in an acceleration
**   of about -256 (1xg) for accel[2] (z) since the acceleration
**   from gravity with be acting along -z.
**************************************************/


/*******************************************************************
** Includes ********************************************************
********************************************************************/

#ifndef COMMON_CONFIG_H
  #include "../Include/Common_Config.h"
#endif

/* Only link if using IMU10736 */
#ifdef _IMU10736_

/*******************************************************************
** Functions *******************************************************
********************************************************************/


/*************************************************
** FUNCTION: Init_IMU
** VARIABLES:
**    [I ]  CONTROL_TYPE      *p_control
**    [IO]  SENSOR_STATE_TYPE *p_sensor_state
** RETURN:
**    BOOL  1:Successful initialization
**          0:Failure
** DESCRIPTION: 
**    This function initiates I2C communicatino with
**    the gyro/magn/accel sensors. Further, it initializes
**    the sensors (setting sampling rate, data format, etc.)
*/
bool Init_IMU( CONTROL_TYPE       *p_control,
               SENSOR_STATE_TYPE  *p_sensor_state )
{
  LOG_INFO( "> Initializing IMU10736" );
  
  /* Initialize sensors */
  delay(20);
  I2C_Init( p_control );
  
  /* Initialize Gyroscope */
  #if GYRO_ON==1
    Gyro_Init( p_control );
  #endif
  
  /* Initialize Accelerometer */
  #if ACCEL_ON==1
    Accel_Init( p_control );
  #endif
  
  /* Initialize Magnometer */
  #if MAGN_ON==1
    Magn_Init( p_control );
  #endif

  return TRUE;
} /* End Init_IMU */


/*************************************************
** FUNCTION: Read_Sensors
** VARIABLES:
**    [I ]  CONTROL_TYPE      *p_control
**    [IO]  SENSOR_STATE_TYPE *p_sensor_state
** RETURN:
**    NONE
** DESCRIPTION: 
**    This function calls the read functions
**    which read the data registers from the
**    sensors.
*/
void Read_Sensors( CONTROL_TYPE       *p_control,
                   SENSOR_STATE_TYPE  *p_sensor_state )
{
  /* Read Gyroscope */
  #if GYRO_ON==1
    Read_Gyro( p_control, p_sensor_state );
  #endif
  
  /* Read Accelerometer */
  #if ACCEL_ON==1
    Read_Accel( p_control, p_sensor_state );
  #endif
  
  /* Read Magnometer */
  #if MAGN_ON==1
    Read_Magn( p_control, p_sensor_state );
  #endif
} /* End Read_Sensors */


/*************************************************
** FUNCTION: I2C_Init
** VARIABLES:
**    [I ]  CONTROL_TYPE      *p_control
** RETURN:
**    NONE
** DESCRIPTION: 
**    This function initiates I2C communication
**    with the gyro/magn/accel. This board only
**    has the one wire port available, so here we
**    simply initate wire.
*/
void I2C_Init( CONTROL_TYPE *p_control )
{ 
  Wire.begin(); 
} /* End I2C_Init */


/*************************************************
** FUNCTION: Accel_Init
** VARIABLES:
**    [I ]  CONTROL_TYPE      *p_control
** RETURN:
**    NONE
** DESCRIPTION: 
**    This function initializes the accelerometer
*/
void Accel_Init( CONTROL_TYPE *p_control )
{
  /* Set measurement mode */
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(ACCEL_POWER);
  WIRE_SEND(0b00001000);   // Measurement mode
  Wire.endTransmission();
  delay(5);

  /* Set data resolution */
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(ACCEL_FORMAT);
  WIRE_SEND(0b00001000);    // Set to full resolution
  Wire.endTransmission();
  delay(5);

  /* Set sample rate
  ** Accepts 0x06-0x0F
  ** 0x06:6.25  0x07:12.5  0x08:25   0x09:50    0x0A:100
  ** 0x0B:200   0x0C:400   0x0D:800  0x0E:1600  0x0F:3200
  ** Hz  */
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(ACCEL_RATE);
  //WIRE_SEND(0x0F);
  WIRE_SEND(0x0E);
  Wire.endTransmission();
  delay(5);
} /* End Accel_Init */


/*************************************************
** FUNCTION: Read_Accel
** VARIABLES:
**    [I ]  CONTROL_TYPE      *p_control
**    [IO]  SENSOR_STATE_TYPE *p_sensor_state
** RETURN:
**    NONE
** DESCRIPTION: 
**    This function reads x/y/z data from the
**    accelerometer
*/
void Read_Accel( CONTROL_TYPE       *p_control,
                 SENSOR_STATE_TYPE  *p_sensor_state )
{
  int i = 0;
  uint8_t buff[6];

  /* Send address to read from */
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(ACCEL_DATA);
  Wire.endTransmission();

  /* Read 6 bytes */
  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.requestFrom(ACCEL_ADDRESS, 6);
  while(Wire.available())
  {
    buff[i] = WIRE_RECEIVE();
    i++;
    if ( i>6 )
    {
      LOG_INFO( "ERROR : Reading Accelerometer : Buffer Overflow" );
      return;
    }
  }
  Wire.endTransmission();

  /* Unpack Data */
  if (i == 6)
  {
    /* No multiply by -1 for coordinate system transformation here, because of double negation:
    ** We want the gravity vector, which is negated acceleration vector. */
    p_sensor_state->accel[0] = (int16_t)((((uint16_t) buff[3]) << 8) | buff[2]);  // X axis (internal sensor y axis)
    p_sensor_state->accel[1] = (int16_t)((((uint16_t) buff[1]) << 8) | buff[0]);  // Y axis (internal sensor x axis)
    p_sensor_state->accel[2] = (int16_t)((((uint16_t) buff[5]) << 8) | buff[4]);  // Z axis (internal sensor z axis)
  }
  else
  {
    LOG_INFO( "ERROR : Reading Accelerometer : Lost Bytes" );
  }
} /* End Read_Accel */


/*************************************************
** FUNCTION: Magn_Init
** VARIABLES:
**    [I ]  CONTROL_TYPE      *p_control
** RETURN:
**    NONE
** DESCRIPTION: 
**    This function initializes the magnetometer
*/
void Magn_Init( CONTROL_TYPE *p_control )
{
  /* Set continuous sampling mode */
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(MAGN_MODE);
  WIRE_SEND(0x00);
  Wire.endTransmission();
  delay(5);

  /* Set sample rate
  ** Offset 2 bits ( xxx###xx )
  ** Accepts 000-110
  ** 000:0.75  001:1.5  010:3   011:7.5
  ** 100:15    101:30   110:75
  ** Hz  */
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(MAGN_CONFIG_A);
  WIRE_SEND(0b00011000); /* 75 Hz */
  Wire.endTransmission();
  delay(5);
  
} /* End Magn_Init */


/*************************************************
** FUNCTION: Read_Magn
** VARIABLES:
**    [I ]  CONTROL_TYPE      *p_control
**    [IO]  SENSOR_STATE_TYPE *p_sensor_state
** RETURN:
**    NONE
** DESCRIPTION: 
**    This function reads x/y/z data from the
**    magnetometer.
*/
void Read_Magn( CONTROL_TYPE      *p_control,
                SENSOR_STATE_TYPE *p_sensor_state )
{
  int i = 0;
  uint8_t buff[6];

  /* Send address to read from */
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(MAGN_DATA_MSBX);
  Wire.endTransmission();

  /* Read 6 bytes */
  Wire.beginTransmission(MAGN_ADDRESS);
  Wire.requestFrom(MAGN_ADDRESS, 6);
  while(Wire.available())
  {
    buff[i] = WIRE_RECEIVE();
    i++;
    if ( i>6 )
    {
      LOG_INFO( "ERROR : Reading Magnetometer : Buffer Overflow" );
      return;
    }
  }
  Wire.endTransmission();

  /* Unpack data */
  if ( i==6 )
  {
    /* 9DOF Razor IMU SEN-10736 using HMC5883L magnetometer
    ** Data 2 byte width, MSB byte first then LSB; 
    ** Y and Z reversed: X, Z, Y */
    
    /* X axis (internal sensor -y axis) */
    p_sensor_state->mag[0] = -1 * (int16_t)(((((uint16_t) buff[4]) << 8) | buff[5]));  
    /* Y axis (internal sensor -x axis) */
    p_sensor_state->mag[1] = -1 * (int16_t)(((((uint16_t) buff[0]) << 8) | buff[1]));  
    /* Z axis (internal sensor -z axis) */
    p_sensor_state->mag[2] = -1 * (int16_t)(((((uint16_t) buff[2]) << 8) | buff[3]));  
  }
  else
  {
    LOG_INFO( "ERROR : Reading Magnetometer : Lost Bytes" );
  }
} /* End Read_Magn */


/*************************************************
** FUNCTION: Gyro_Init
** VARIABLES:
**    [I ]  CONTROL_TYPE      *p_control
** RETURN:
**    NONE
** DESCRIPTION: 
**    This function initializes the gyroscope
*/
void Gyro_Init( CONTROL_TYPE *p_control )
{
  /* Power up reset defaults */
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(GYRO_POWER);
  WIRE_SEND(0x80);
  Wire.endTransmission();
  delay(5);

  /* Full-scale range of the gyro sensors
  ** Set LP filter bandwidth B0-B2
  **  NOTE:The LPF determines Finternal!
  **  DLPF_CFG:LPF BW(Hz): Finternal(Hz)
  **  000:256:8  001:188:1  010:98:1  011:42:1
  **  100:20:1   101:10:1   110:5:1
  ** FS_SEL: Full scale range, B3-B4, only accepts 11 */
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(GYRO_DLPF);
  //WIRE_SEND(0x1B);  // DLPF_CFG = 3:LPF 42Hz,Fi 1kHz , FS_SEL = 3:+-2000deg/sec
  WIRE_SEND(0x03);  // DLPF_CFG = 3:LPF 42Hz,Fi 1kHz , FS_SEL = 3:+-2000deg/sec
  Wire.endTransmission();
  delay(5);

  /* Set sample rato divider
  ** Fsample = Finternal / (divider+1), where Finternal is either 1kHz or 8kHz
  ** 8 bit field (0-255) */
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(GYRO_RATE);
  //WIRE_SEND(0x0A);  //  SMPLRT_DIV = 10 (90Hz w/ Fi=1kHz)
  WIRE_SEND(0x00);  //  SMPLRT_DIV = 0 (1000Hz w/ Fi=1kHz)
  Wire.endTransmission();
  delay(5);

  /* Set clock to PLL with z gyro reference */
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(GYRO_POWER);
  //WIRE_SEND(0x00); /* Internal occilator */
  WIRE_SEND(0x01); /* Gyro x ref */
  Wire.endTransmission();
  delay(5);
} /* End Gyro_Init */


/*************************************************
** FUNCTION: Read_Gyro
** VARIABLES:
**    [I ]  CONTROL_TYPE      *p_control 
**    [IO]  SENSOR_STATE_TYPE *p_sensor_state
** RETURN:
**    NONE
** DESCRIPTION: 
**    This function reads x/y/z data from the
**    gyroscope.
*/
void Read_Gyro( CONTROL_TYPE      *p_control, 
                SENSOR_STATE_TYPE *p_sensor_state )
{
  int i = 0;
  uint8_t buff[7];

  /* Send address to read from */
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(GYRO_DATA);
  Wire.endTransmission();

  /* Read 6 bytes */
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.requestFrom(GYRO_ADDRESS, 6);
  while ( Wire.available() )
  {
    buff[i] = WIRE_RECEIVE();
    i++;
    if ( i>6 )
    {
      LOG_INFO( "ERROR : Reading Gyroscope : Buffer Overflow" );
      return;
    }
  }
  Wire.endTransmission();

  /* Unpack Data */
  if ( i==6 )
  {
    /* X axis (internal sensor -y axis) */
    p_sensor_state->gyro[0] = -1 * (int16_t)(((((uint16_t) buff[2]) << 8) | buff[3]));   
    /* Y axis (internal sensor -x axis) */
    p_sensor_state->gyro[1] = -1 * (int16_t)(((((uint16_t) buff[0]) << 8) | buff[1]));    
    /* Z axis (internal sensor -z axis) */
    p_sensor_state->gyro[2] = -1 * (int16_t)(((((uint16_t) buff[4]) << 8) | buff[5]));    
  }
  else
  {
    LOG_INFO( "ERROR : Reading Gyroscope : Lost Bytes" );
  }
} /* End Read_Gyro */


#endif /* End _IMU10736_ */









