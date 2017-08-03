/******************************************************************
** FILE: Math.h
** Header containing Math defines and platform conversions.
******************************************************************/


#ifndef MATH_H
#define MATH_H

#include "Common_Config.h"

#define FALSE 0
#define TRUE 1
#define GTOMPS2 (9.80665)
#define MPSTOMPH (2.23694)

#if EXE_MODE /* Emulator mode */
#define FCONSTRAIN(x,m,M) (fmin(fmax(x,m),M))
#else
#define FCONSTRAIN constrain
#endif /* EXE_MODE */

#define SIGN(x) ( (0<x)-(x<0)+(x==0) )

#endif /* End MATH_H */
