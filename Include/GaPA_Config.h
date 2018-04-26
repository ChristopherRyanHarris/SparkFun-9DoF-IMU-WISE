
/*******************************************************************
** FILE:
**   	GaPA_Config.h
** DESCRIPTION:
** 		Header for the Gait Phase Angle estimator (GaPA)
** 		This file should contain only definitions specific to the
**		GaPA algorithms
********************************************************************/
#ifndef GAPA_CONFIG_H
#define GAPA_CONFIG_H


/*******************************************************************
** Defines
********************************************************************/

//#define GAPA_Kp_PHI 0.01
#define GAPA_Kp_PHI 0.001
#define GAPA_Ki_PHI 0

//#define GAPA_Kp_phi 0.01
#define GAPA_Kp_phi 0.0001
//#define GAPA_Ki_phi 0.01
#define GAPA_Ki_phi 0.02

//#define GAPA_PHImw_ALPHA 0.01
#define GAPA_PHImw_ALPHA 0.006
//#define GAPA_phimw_ALPHA 0.01
#define GAPA_phimw_ALPHA 0.006

#define GAPA_MIN_GYRO (500)
#define GAPA_GAIT_END_THRESH 1.5708

#define GAPA_DEFAULT_Z_phi 0.5f
#define GAPA_DEFAULT_Z_PHI 1.0f


/*******************************************************************
** Tyedefs
********************************************************************/

/*
** TYPE: GAPA_STATE_TYPE
** This holds the state variables
** for the full Gait Phase Angle estimation code.
*/
typedef struct
{
	int version; /* Phase portrait version (PHI/PHV) */

	int iteration;

	float phi; /* "thigh angle wrt vertical down (i.e. Pitch) */
	float phi_max, phi_min; /* max/min pitch in previous gait */
	float phi_max_next, phi_min_next; /* max/min pitch in this gait (for use in next cycle) */
	float PHI; /* Time integral of the thigh angle (i.e. Integral of the Pitch) */
	float PHI_max, PHI_min; /* max/min pitch integral in previous gait */
	float PHI_max_next, PHI_min_next; /* max/min swing distance in this gait (for use in next cycle) */

	float gamma; /* "right shift variable" */
	float GAMMA; /* "left shift variable" */
	float z_phi; /* the "scale factor" */
	float z_PHI; /* the "scale factor" */

	float PErr_phi, IErr_phi;
	float PErr_PHI, IErr_PHI;

	float phi_mw, PHI_mw;
	float phin, PHIn;

	float prev_phi[3], prev_PHI[3];

	float nu;	/* The Phase Angle */
	float nu_prev;	/* The previous Phase Angle */
	float nu_normalized;	/* The phase angle on the region [0,1] */
	
	/* Boolean to mark the end of a gait cycle */
	bool Gait_End;
	
} GAPA_STATE_TYPE;


typedef struct
{
	int phase_method;

	float Kp_PHI;
	float Ki_PHI;
	float Kp_phi;
	float Ki_phi;

	float PHImw_alpha;
	float phimw_alpha;

	float default_z_phi;
	float default_z_PHI;

	float min_gyro;
	float gait_end_threshold;
} GAPA_PERMS_TYPE;

/*
** TYPE: ERNIE_STATE_TYPE
** This holds the state variables for the so-called "ERNIE" algorithm
** The "ERNIE" algorithm is a method of computing a fast moving phase
** variable. We will use this algorithm for the first few strides, or
** until the ernie phase matches the unified phase. The ERNIE
** algorithm is essentially a Piece-Wise algorithm of computing the
** phase variable */

#endif /* End GAPA_CONFIG_H */

