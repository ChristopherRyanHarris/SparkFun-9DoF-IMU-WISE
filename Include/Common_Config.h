
/******************************************************************
** FILE: IMU_Common.h
** Header containing IMU definitions which are platform agnostic.
** Definitions in this file should be independent of IMU version.
******************************************************************/

/* 0: IMU
** 1: Emulator */
#define EXE_MODE 0


#ifndef IMU_COMMON_H
#define IMU_COMMON_H

#if EXE_MODE /* Emulator mode */
#include <stdbool.h>
#include <inttypes.h>
#include <stdio.h>
#else
//#define _IMU10736_ /* Using IMU10736 */
#define _IMU9250_ /* Using IMU9250 */
#endif

#define DEBUG 1

/* IO params */
#define OUTPUT_MODE 0
#define NUM_COM_MODES 5

/* Calibration params  */
#define CALIBRATE_MODE  0
#define CAL_OUTPUT_MODE 0
#define NUM_CALCOM_MODES 2

/*******************************************************************
** Tyedefs *********************************************************
********************************************************************/

/*
** TYPE: CAL_STATE_TYPE
** This type is used to hold
** data useful for calibration */
typedef struct
{
  int output_mode;

  float accel_total[3];
  float accel_max[3];
  float accel_min[3];

  float gyro_total[3];
  float gyro_max[3];
  float gyro_min[3];

  float N;
} CAL_STATE_TYPE;

/*
** TYPE: DCM_STATE_TYPE
** This type is used to hold the DCM
** specific arrays and variables */
typedef struct
{
  float Omega_P[3];
  float Omega_I[3];
  float DCM_Matrix[3][3];
} DCM_STATE_TYPE;

/*
** TYPE: SENSOR_STATE_TYPE
** This type is used to hold the sensor
** variables */
typedef struct
{
  float yaw;
  float pitch;
  float roll;

  float mag[3]; /* not used */

  /* Accel x:Fore y:Port z:Zenith */
  float accel[3];
  float gyro[3];
} SENSOR_STATE_TYPE;

/*
** TYPE: CONTROL_STATE_TYPE
** This type is used to hold all control
** variables. */
typedef struct
{
  int output_mode;// = 3;

  unsigned long timestamp;
  unsigned long timestamp_old;
  float G_Dt;

  /* Serial communication globals */
  bool g_BaudLock; /* Used to set baud rate */
  uint32_t  g_LastLogTime; /* Sets the UART LOG Rate */

  /* LED state globals */
  bool      g_LedState; /* Used to set LED state */
  uint32_t  g_LastBlinkTime; /* Used to set LED state */
  

} CONTROL_STATE_TYPE;

/*
** TYPE: RESPONSE_TYPE
** Used to store temporary resonse data
** for responding to request from master */
typedef struct
{
  uint16_t       Packet_nBytes;  /* Length of entire packet, minus this variable, in bytes */
  uint16_t       PacketType;     /* Type code of packet */
  uint16_t       Buffer_nBytes;  /* Length of data buffer in bytes (0-50) */
  unsigned char  Buffer[50];     /* Data buffer */
  unsigned char  CheckSum;       /* CheckSum of data buffer only */
} RESPONSE_TYPE;


#endif // IMU_COMMON_H
