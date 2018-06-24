
/*******************************************************************
** FILE:
**    Communication_Config.h
** DESCRIPTION:
**
********************************************************************/
#ifndef CALIBRATION_CONFIG_H
#define CALIBRATION_CONFIG_H


/*******************************************************************
** Typedefs *********************************************************
********************************************************************/

/*
** TYPE: CALIBRATION_TYPE
** This type is used to hold
** data useful for calibration */
typedef struct
{
  float accel_total[3];
  float accel_max[3];
  float accel_min[3];

  float gyro_total[3];
  float gyro_max[3];
  float gyro_min[3];

  float N;
} CALIBRATION_TYPE;

/* TYPE: CALIBRATION_PRMS_TYPE
** This type is used to hold the calibration
** control parameters. */
typedef struct
{
  int output_mode;

  float accel_min_x;
  float accel_max_x;
  float accel_min_y;
  float accel_max_y;
  float accel_min_z;
  float accel_max_z;

  float magn_min_x;
  float magn_max_x;
  float magn_min_y;
  float magn_max_y;
  float magn_min_z;
  float magn_max_z;

  float gyro_ave_offset_x;
  float gyro_ave_offset_y;
  float gyro_ave_offset_z;
} CALIBRATION_PRMS_TYPE;



#endif /* End CALIBRATION_CONFIG_H */
