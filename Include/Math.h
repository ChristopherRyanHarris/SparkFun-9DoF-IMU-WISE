
/*******************************************************************
** FILE: 
**   	Math.h
** DESCRIPTION: 
** 		This header file is inteded to hold common mathmatical
**		macros and commonly used constants.
********************************************************************/


#ifndef MATH_H
#define MATH_H

//#ifndef COMMON_CONFIG_H
//	#include "../Include/Common_Config.h"
//#endif

#define FALSE 0
#define TRUE 1

#define GTOMPS2 (9.80665)
#define MPSTOMPH (2.23694)

#define PI (3.14159265359)
#define TO_RAD(x) (x * 0.01745329252)  // deg to rad: *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // rad to deg: *180/pi

#if EXE_MODE==1 /* Emulator mode */
	#define FCONSTRAIN(x,m,M) (fmin(fmax(x,m),M))
#else
	#define FCONSTRAIN constrain
#endif /* EXE_MODE */

#define SIGN(x) ( (0<x)-(x<0)+(x==0) )
#define FABS(x)	( (x>=0) ? x : (x*-1) )

#define MAX( a, b ) ( ( a > b) ? a : b )
#define MIN( a, b ) ( ( a < b) ? a : b )


#endif /* End MATH_H */
