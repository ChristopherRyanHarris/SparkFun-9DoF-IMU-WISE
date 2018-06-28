
/*******************************************************************
** FILE:
**    Math.h
** DESCRIPTION:
**    This header file is intended to hold common mathematical
**    macros and commonly used constants.
********************************************************************/
#ifndef MATH_H
#define MATH_H


/*******************************************************************
** Defines *********************************************************
********************************************************************/

/* General Defines */

#define FALSE 0
#define TRUE 1

/* Data sizes */
#define POW_2_0               1
#define POW_2_1               2
#define POW_2_2               4
#define POW_2_3               8
#define POW_2_4              16
#define POW_2_5              32
#define POW_2_6              64
#define POW_2_7             128
#define POW_2_8             256
#define POW_2_9             512
#define POW_2_10           1024 /* 1KB */
#define POW_2_11           2048
#define POW_2_12           4096
#define POW_2_13           8192
#define POW_2_14          16384
#define POW_2_15          32768
#define POW_2_16          65536
#define POW_2_17         131072
#define POW_2_18         262144
#define POW_2_19         524288
#define POW_2_20        1048576 /* 1MB */
#define POW_2_21        2097152
#define POW_2_22        4194304
#define POW_2_23        8388608
#define POW_2_24       16777216
#define POW_2_25       33554432
#define POW_2_26       67108864
#define POW_2_27      134217728
#define POW_2_28      268435456
#define POW_2_29      536870912
#define POW_2_30     1073741824 /* 1GB */


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
#define FABS(x) ( ( (x)>=0) ? (x) : -(x) )
#define ABS(x)  ( ( (x)>=0) ? (x) : -(x) )

#define MAX( a, b ) ( ( (a) > (b) ) ? (a) : (b) )
#define MIN( a, b ) ( ( (a) < (b) ) ? (a) : (b) )


#endif /* End MATH_H */
