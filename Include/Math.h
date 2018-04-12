
/*******************************************************************
** FILE:
**   	Math.h
** DESCRIPTION:
** 		This header file is intended to hold common mathematical
**		macros and commonly used constants.
********************************************************************/
#ifndef MATH_H
#define MATH_H


/*******************************************************************
** Defines *********************************************************
********************************************************************/

/* General Defines */

#define FALSE 0
#define TRUE 1


/* Constants */

#define GTOMPS2 (9.80665)
#define MPSTOMPH (2.23694)

#define PI (3.14159265359)
#define TWOPI (6.28318530718)

/* Macros */

#define TO_RAD(x) (x * 0.01745329252)  // deg to rad: *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // rad to deg: *180/pi


#if EXE_MODE==1 /* Emulator mode */
	#define FCONSTRAIN(x,m,M) (fmin(fmax((x),m),M))
#else
	#define FCONSTRAIN constrain
#endif /* EXE_MODE */

//#define SIGN(x) ( (0<x)-(x<0)+(x==0) )
#define SIGN(x) ( (0<(x)) ? (-1) : (1) )
#define FABS(x)	( ( (x)>=0) ? (x) : -(x) )
#define ABS(x)	( ( (x)>=0) ? (x) : -(x) )

#define MAX( a, b ) ( ( (a) > (b) ) ? (a) : (b) )
#define MIN( a, b ) ( ( (a) < (b) ) ? (a) : (b) )


#endif /* End MATH_H */
