
/******************************************************************
** FILE: WISE_Common.h
** Header for the Walking Speed and Incline Estimator (WISE)
** This file should contain only definitions specific to the
** WISE algorithms
******************************************************************/

#ifndef WISE_COMMON_H
#define WISE_COMMON_H

#define WISE_ON 0

//#define WISE_GAIN_AD 0.025f
#define WISE_GAIN_AD 0.05f
//#define WISE_GAIN_AP 0.00005f
#define WISE_GAIN_AP 0.0f

//#define WISE_GAIN_VD 0.25f
#define WISE_GAIN_VD 0.3f
//#define WISE_GAIN_VP 1.0f
//#define WISE_GAIN_VP 0.03f
#define WISE_GAIN_VP 0.0f

#define WISE_CORRECTION 1.8
//#define WISE_CORRECTION 5.0
//#define WISE_CORRECTION 3.720962
//#define WISE_CORRECTION 1.0

#define WISE_MINCOUNT 50

/*******************************************************************
** Tyedefs
********************************************************************/


/* 
** TYPE: WISE_GATE_TYPE
** This holds the state variables
** for a gait cycle at toe-off
*/
typedef struct
{
	float vel[3];
	float vel_total[3];
	float drift[3];
	long int Time;
	float Nsamples;
} WISE_GATE_TYPE;


/*
** TYPE: WISE_STATE_TYPE
** This holds all the state variables
** for the walking speed and estimation code
*/
typedef struct
{
  bool swing_state; // 0:down 1:up
  bool toe_off;
  int minCount;

	WISE_GATE_TYPE GaitStart;
	WISE_GATE_TYPE GaitEnd;
	
	WISE_GATE_TYPE CrossingP;

  float pitch_mem;
  float pitch_delta;
  float pitch_delta_total;

  /* [a_x, a_y, a_z]
  ** Leg acceleration vector
  ** wrt leg coordinate reference frame */
  float accel[3];
  float accel_total[3];
  float accel_ave[3];

  float accel_delta[3];
  float omega_ad[3];
  float omega_ap[3];

  /* [v_x, v_y, v_Z]
  ** Leg velocity vector
  ** wrt leg coordinate reference frame */
  float vel[3];
  float vel_total[3];
  float vel_ave[3];

  float vel_delta[3];
  float omega_vd[3];
  float omega_vp[3];
  
  
  /* [r_x, r_y, r_Z]
  ** Integral of gyro data
  ** Used to determine gait toe-off (max point)
  ** Can also be used to fine heel strike (min point) */
  float gyr[3];
  float rot[3];
  float rot_total[3];
  float rot_ave[3];
  float rot_delta[3];
  
  float dist[3];
  
  float Incline;
  float Incline_ave;
  float Incline_gait;

  float pe[3];
  float pave;

	float Time;
  float Nsamples;
  float Ncycles;
} WISE_STATE_TYPE;


#endif // WISE_COMMON_H


