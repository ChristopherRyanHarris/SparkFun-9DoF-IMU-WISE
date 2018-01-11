
/******************************************************************
** FILE: GaPA_Config.h
** Header for the Gait Phase Angle estimator (GaPA)
** This file should contain only definitions specific to the
** GaPA algorithms
******************************************************************/

#ifndef GAPA_H
#define GAPA_H


/*******************************************************************
** Defines
********************************************************************/

#define GAPA_ON 1

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
	
	float phi; /* "thigh angle wrt vertical down (i.e. Pitch) */ 
	float phi_max, phi_min; /* max/min pitch in previous gait */
	float phi_max_next, phi_min_next; /* max/min pitch in this gait (for use in next cycle) */
	float PHI; /* Time integral of the thigh angle (i.e. Integral of the Pitch) */ 
	float PHI_max, PHI_min; /* max/min pitch integral in previous gait */ 
	float PHI_max_next, PHI_min_next; /* max/min swing distance in this gait (for use in next cycle) */
	
	float gamma; /* "right shift variable" */
	float GAMMA; /* "left shift variable" */
	float z; /* the "scale factor" */
	
	float nu;	/* The Phase Angle */
	float nu_prev;	/* The previous Phase Angle */
} GAPA_STATE_TYPE;


/* 
** TYPE: ERNIE_STATE_TYPE
** This holds the state variables for the so-called "ERNIE" algorithm
** The "ERNIE" algorithm is a method of computing a fast moving phase 
** variable. We will use this algorithm for the first few strides, or 
** until the ernie phase matches the unified phase. The ERNIE 
** algorithm is essentially a Piece-Wise algorithm of computing the 
** phase variable */
typedef struct
{
	float 
}
#endif // GAPA_H

