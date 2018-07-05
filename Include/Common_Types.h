
/*******************************************************************
** FILE:
**    Common_Types.h
** DESCRIPTION:
**    Header containing common types used across several functions.
**    None of these types should be specific to any algorithm. 
**    Rather, they should be very generic data containers.
********************************************************************/
#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H


/*******************************************************************
** Defines *********************************************************
********************************************************************/

/* Precision Limits */
#define INT_8_MIN   (-128)
#define INT_8_MAX   (128)
#define UINT_8_MAX  (255)

#define INT_16_MIN  (-32768)
#define INT_16_MAX  (32767)
#define UINT_16_MAX (65535)

#define INT_32_MIN  (-2147483648)
#define INT_32_MAX  (2147483648)
#define UINT_32_MAX (4294967295)

#define INT_64_MIN  (-9223372036854775808)
#define INT_64_MAX  (9223372036854775808)
#define UINT_64_MAX (18446744073709551615)

#define FLOAT_MIN   (1.175494E-37)
#define FLOAT_MAX   (3.402823E+37)

/*******************************************************************
** Typedefs *********************************************************
********************************************************************/

/*
** TYPE: SAMPLE_DATA_1D_TYPE
** Used to store a 1x1 (scalar) sample type 
** as well as any associated statistics or metrics.
** This approach allows us to keep as much relevant 
** information together as possible (and is a bit 
** easier to read in my opinion) */
typedef struct
{
  /* Raw data from sensor */
  float val[3]; /* unscaled, 3 sample history */ 
 
  /* calibration terms */
  float dc_offset;
  float scale;
  
  /* Statistics/Metrics */
  float val_min, val_max;
  float val_M2;
  float val_mave_alpha; /* alpha:Moving average decay term */
  float val_ave;  /* Total average of val (no decay) */
  float val_mave; /* Moving average of mag */
  float val_sam_var; 
  float val_pop_var;
  
  /* Other */
  float val_sum[3];  /* Full accumulation : sum[t]=sum[t-1]+x[t] */
  float val_diff[3]; /* 2-point difference : x[t]-x[t-1] */  
} SAMPLE_DATA_1D_TYPE;

/*
** TYPE: SAMPLE_DATA_2D_TYPE
** Used to store a 1x3 (vector) sample type (accel, gyro or magn) 
** as well as any associated statistics or metrics.
** This approach allows us to keep as much relevant 
** information together as possible (and is a bit 
** easier to read in my opinion) */
typedef struct
{
  /* Raw data from sensor */
  float val[3*3]; /* unscaled, 3 sample history */ 
  float mag[3];   /* scaled : (mag_raw-dc_offset)*scale */
  
  /* calibration terms */
  float dc_offset[3];
  float scale;
  
  /* Statistics/Metrics */
  float mag_min, mag_max;
  float mag_M2;
  float mag_mave_alpha; /* alpha:Moving average decay term */
  float mag_ave;  /* Total average of mag (no decay) */
  float mag_mave; /* Moving average of mag */
  float mag_sam_var; 
  float mag_pop_var;
  
  /* Other */
  float mag_sum[3];  /* Full accumulation : sum[t]=sum[t-1]+x[t] */
  float mag_diff[3]; /* 2-point difference : x[t]-x[t-1] */  
} SAMPLE_DATA_2D_TYPE;


#endif /* End COMMON_TYPES_H */
