
/*******************************************************************
** FILE:
**    IMU9250_Config.h
** DESCRIPTION:
**    This file contains all header information specific to the
**    9250 platform
********************************************************************/
#ifndef IMU9250_CONFIG_H
#define IMU9250_CONFIG_H

#ifndef MATH_H
  #include "../Include/Math.h"
#endif

/******************************************************************
** User defined
** General Settings
** Here we define the general settings for the system
** These can be changed to suit the users needs
******************************************************************/

/* TO DO : Move this somehow 
** Count the number of arguments */
#define PP_NARG(...) \
         PP_NARG_(__VA_ARGS__,PP_RSEQ_N())
#define PP_NARG_(...) \
         PP_ARG_N(__VA_ARGS__)
#define PP_ARG_N( \
          _1, _2, _3, _4, _5, _6, _7, _8, _9,_10, \
         _11,_12,_13,_14,_15,_16,_17,_18,_19,_20, \
         _21,_22,_23,_24,_25,_26,_27,_28,_29,_30, \
         _31,_32,_33,_34,_35,_36,_37,_38,_39,_40, \
         _41,_42,_43,_44,_45,_46,_47,_48,_49,_50, \
         _51,_52,_53,_54,_55,_56,_57,_58,_59,_60, \
         _61,_62,_63,N,...) N
#define PP_RSEQ_N() \
         63,62,61,60,                   \
         59,58,57,56,55,54,53,52,51,50, \
         49,48,47,46,45,44,43,42,41,40, \
         39,38,37,36,35,34,33,32,31,30, \
         29,28,27,26,25,24,23,22,21,20, \
         19,18,17,16,15,14,13,12,11,10, \
         9,8,7,6,5,4,3,2,1,0


/* DCM parameters
*******************************************************************/

/*
** Notes on orientation for the 9250 IMU
**   Terms:
**     Fore:       (Front) Edge of the USB port
**     Aft:        (Rear) Edge opposite of the USB port
**     Starboard:  (Right) Edge opposite of PWR switch
**     Port:       (Left) Edge with PWR switch
**     Zenith:     (Up) face with USB port
**     Nadir:      (Down) face opposite USB port
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
*/

/* Define pitch orientation convention
** Range: -90:90
** With PITCH_ROT_CONV==1 :
** PITCH_O:1 - Pitch orientation #1. Angle x-axis w/ Horizontal Plane  +Rot:Aft-Down    0:Nadir0/Zenith down. +90:Aft down   -90:Fore down
** PITCH_O:2 - Pitch orientation #2. Angle y-axis w/ Horizontal Plane  +Rot:Port-Down   0:Fore/Aft down       +90:Port down  -90:Starboard down
** PITCH_O:3 - Pitch orientation #3. Angle z-axis w/ Horizontal Plane  +Rot:Nadir-Down  0:Fore/Aft down       +90:Nadir down -90:Zenith down */
#define PITCH_O  1

/* Pitch rotation convention
** This sets the sign of rotation for pitch
**   1 :+Towards +axis
**  -1 :+Towards -axis */
#define PITCH_ROT_CONV -1

/* NOTE:
** Since Pitch is -90:+90, there is no "PITCH_ZREF"
** as this would have the exact role as flipping
** "PITCH_ROT_CONV" */

/* Roll rotation convention
** This sets the sign of rotation for roll
**   1 :+Towards +axis
**  -1 :+Towards -axis */
#define ROLL_ROT_CONV 1

/* Roll rotation reference direction
** This will reverse the direction of the "0" reference for roll
**   1 :"0" is in direction of +axis (Fore)
**  -1 :"0" is in direction of -axis (Aft) */
#define ROLL_ZREF 1





/******************************************************************
** User dependent
** DON'T TOUCH!
** These are created from user variables
*******************************************************************/

/* Define roll orientation convention
** We should only be using 3,4,5 orientations, but they are all available for hacking.
** Range: -180:180
** With ROLL_ROT_CONV==1  ROLL_ZREF==1
** ROLL_O:1 - Roll orientation #1. Rotation around z-axis (Nadir-Zenith) +Rot:Aft-Port    0:Port down     +-180:stbd down
** ROLL_O:2 - Roll orientation #2. Rotation around y-axis (Port-Stbd)    +Rot:Aft-Nadir   0:Nadir down    +-180:Zenith down
** ROLL_O:3 - Roll orientation #3. Rotation around x-axis (Fore-Aft)     +Rot:Port-Nadir  0:Nadir down    +-180:Zenith down
** ROLL_O:4 - Roll orientation #4. Rotation around z-axis (Nadir-Zenith) +Rot:Aft-Port    0:Aft down      +-180:Fore down
** ROLL_O:5 - Roll orientation #5. Rotation around y-axis (Port-Stbd)    +Rot:Aft-Nadir   0:Aft down      +-180:Fore down
** ROLL_O:6 - Roll orientation #6. Rotation around x-axis (Fore-Aft)     +Rot:Port-Nadir  0:Port down     +-180:Stbd down
** WARNING: ROLL should be determined from pitch orientation, not set manually */
#if    PITCH_O==1  /* Rotation around y-axis  +Rot:Aft-Down    0:Nadir0/Zenith down. +90:Aft down -90:Fore down */
  #define ROLL_O 3
#elif  PITCH_O==2  /* Rotation around z-axis  +Rot:Port-Down   0:Fore/Aft down       +90:Port down -90:Starboard down */
  /* This is offset 90, need to find another option */
  #define ROLL_O 5
#elif PITCH_O==3   /* Rotation around y-axis  +Rot:Nadir-Down  0:Fore/Aft down       +90:Nadir down -90:Zenith down */
  /* This is offset 90, need to find another option */
  #define ROLL_O 4
#endif



/******************************************************************
** HW specific
** SparkFun "9DOF Razor IMU" version 9250
*******************************************************************/

/* 9250
*******************************************************************/
#define MPU9250_INT_PIN 4
#define MPU9250_INT_ACTIVE LOW




/* Sampling resolution
*******************************************************************/
/* Set the system sampling rate */
//#define TIME_SR         200.0f    /* Warning: depends on sensor settings! */
#define TIME_SR         99999.0f    /* Warning: depends on sensor settings! */

/* Resolution of system time
** Used to set delta T - see Update_Time */
//#define TIME_RESOLUTION 1000.0f
#define TIME_RESOLUTION 1000000.0f /* units/s */

/* TIME_RESOLUTION should match TIME_FUPDATE !! */
//#define TIME_FUPDATE    millis()
#define TIME_FUPDATE    micros()


/* Board LED
*******************************************************************/
#define HW_LED_PIN 13


/* Accelerometer I2C addresses (Register Map)
******************************************************************/
#define IMU_AG_SAMPLE_RATE 10000 // Accel/gyro sample rate Must be between 4Hz and 1kHz
#define IMU_ACCEL_FSR      16 // Accel full-scale range (2, 4, 8, or 16)
#define IMU_AG_LPF         5 // Accel/Gyro LPF corner frequency (5, 10, 20, 42, 98, or 188 Hz)


/* Magnetometer I2C addresses (Register Map)
******************************************************************/


/* Gyroscope addresses I2C addresses (Register Map)
******************************************************************/
#define IMU_GYRO_FSR       2000 // Gyro full-scale range (250, 500, 1000, or 2000)



/******************************************************************
** Sensor Calibration
*******************************************************************/

/* "1G reference" used for DCM filter and accelerometer calibration */
#define GRAVITY 2000.0f

/* Calibration Macros
******************************************************************/

/* Movement Detection Thresholds
******************************************************************/

/* Accelerometer Calibration
******************************************************************/
#define ACCEL_X_MIN ((float) -2000)
#define ACCEL_X_MAX ((float) 2000)
#define ACCEL_Y_MIN ((float) -2000)
#define ACCEL_Y_MAX ((float) 2000)
#define ACCEL_Z_MIN ((float) -2000)
#define ACCEL_Z_MAX ((float) 2000)
#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_GAIN (GRAVITY/(ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_GAIN (GRAVITY/(ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_GAIN (GRAVITY/(ACCEL_Z_MAX - ACCEL_Z_OFFSET))
#define ACCEL_X_SCALED(x) ( (x - ACCEL_X_OFFSET - (0))*ACCEL_X_GAIN )
#define ACCEL_Y_SCALED(x) ( (x - ACCEL_Y_OFFSET - (0))*ACCEL_Y_GAIN )
#define ACCEL_Z_SCALED(x) ( (x - ACCEL_Z_OFFSET)*ACCEL_Z_GAIN )

////#define ACCEL_GAIN 0.0134
////#define ACCEL_GAIN 0.0151
//#define ACCEL_GAIN 0
//#define ACCEL_X_SCALED(x) (x * ACCEL_GAIN)
//#define ACCEL_Y_SCALED(x) (x * ACCEL_GAIN)
//#define ACCEL_Z_SCALED(x) (x * ACCEL_GAIN)


/* Magnetometer Calibration
******************************************************************/
#define MAGN_X_MIN ((float) -600)
#define MAGN_X_MAX ((float) 600)
#define MAGN_Y_MIN ((float) -600)
#define MAGN_Y_MAX ((float) 600)
#define MAGN_Z_MIN ((float) -600)
#define MAGN_Z_MAX ((float) 600)
#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))

/* Gyroscope Calibration
******************************************************************/
// Gain for gyroscope (ITG-3200)
#define GYRO_GAIN 0.06957 // Same gain on all axes
//#define GYRO_GAIN 0.001214
#define GYRO_AVERAGE_OFFSET_X ((float) 0.0)
#define GYRO_AVERAGE_OFFSET_Y ((float) 0.0)
#define GYRO_AVERAGE_OFFSET_Z ((float) 0.0)
#define GYRO_SCALED_RAD(x) (x * TO_RAD(GYRO_GAIN))
#define GYRO_X_SCALED(x) ((x-GYRO_AVERAGE_OFFSET_X) * TO_RAD(GYRO_GAIN))
#define GYRO_Y_SCALED(x) ((x-GYRO_AVERAGE_OFFSET_Y) * TO_RAD(GYRO_GAIN))
#define GYRO_Z_SCALED(x) ((x-GYRO_AVERAGE_OFFSET_Z) * TO_RAD(GYRO_GAIN))






#endif /* End IMU9250_CONFIG_H */











