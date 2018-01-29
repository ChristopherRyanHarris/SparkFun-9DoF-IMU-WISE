
/*******************************************************************
** FILE:
**   	DCM_Config.h
** DESCRIPTION:
** 		Header file for the Directional Cosine Matrix (DCM) "filter"
** 		and orientation algorithms.
** 		This file should contain only definitions specific to the
** 		DCM algorithms
********************************************************************/
#ifndef DCM_CONFIG_H
#define DCM_CONFIG_H


/* DCM parameters
*******************************************************************/

/* DCM gain */
//#define Kp_ROLLPITCH 0.0f
//#define Kp_ROLLPITCH 0.1f
//#define Kp_ROLLPITCH 0.0002f
#define Kp_ROLLPITCH 0.002f

//#define Ki_ROLLPITCH 0.0f
//#define Ki_ROLLPITCH 0.00005f
//#define Ki_ROLLPITCH 0.00006f
//#define Ki_ROLLPITCH 0.00001f /*1E-5*/
#define Ki_ROLLPITCH 0.000001f /*1E-6*/
//#define Ki_ROLLPITCH 0.0000001f /*1E-7*/

#define Kp_YAW 0.0f
//#define Kp_YAW 1.2f
//#define Kp_YAW 1.5f

#define Ki_YAW 0.0f
//#define Ki_YAW 0.00002f
//#define Ki_YAW 0.00005f

/*******************************************************************
** Typedefs *********************************************************
********************************************************************/

/*
** TYPE: DCM_STATE_TYPE
** This type is used to hold the DCM
** specific arrays and variables */
typedef struct
{
  float Omega_P[3];
  float Omega_I[3];
  float DCM_Matrix[3][3];

  float gyro_ave[3];
  float gyro_var[3];
  float gyro_std[3];

  float std_time;

  long int SampleNumber;
} DCM_STATE_TYPE;


/*
** TYPE: DCM_PRMS_TYPE
** This type is used to hold the DCM
** execution parameters. */
typedef struct
{
  float Kp_RollPitch;
  float Ki_RollPitch;

  float Kp_Yaw;
  float Ki_Yaw;

  int PitchOrientation;
  int PitchRotationConv;

  int RollOrientation;
  int RollRotationConv;
  int RollRotationRef;

} DCM_PRMS_TYPE;



#endif /* End DCM_CONFIG_H */
