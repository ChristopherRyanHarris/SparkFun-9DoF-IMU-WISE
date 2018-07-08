
/*******************************************************************
** FILE:
**    GaPA_Config.h
** DESCRIPTION:
**    Header for the Gait Phase Angle estimator (GaPA)
**    This file should contain only definitions specific to the
**    GaPA algorithms
********************************************************************/
#ifndef GAPA_CONFIG_H
#define GAPA_CONFIG_H


/*******************************************************************
** Defines
********************************************************************/

//#define GAPA_Kp_PHI 0.01
//#define GAPA_Kp_PHI 0.001
#define GAPA_Kp_PHI 0.016 /* 1/sr */   
//#define GAPA_Ki_PHI 0
#define GAPA_Ki_PHI 0.00002

#define GAPA_Kp_phi 1
//#define GAPA_Kp_phi 0.01
//#define GAPA_Kp_phi 0.0001
//#define GAPA_Ki_phi 0.01
//#define GAPA_Ki_phi 0.02
#define GAPA_Ki_phi 0.005

//#define GAPA_PHImw_ALPHA 0.01
//#define GAPA_PHImw_ALPHA 0.006
#define GAPA_PHImw_ALPHA 0.1

//#define GAPA_phimw_ALPHA 0.01
//#define GAPA_phimw_ALPHA 0.006
#define GAPA_phimw_ALPHA 0.1

#define GAPA_MIN_GYRO 200.0f
#define GAPA_GAIT_END_THRESH 1.5708

#define GAPA_DEFAULT_Z_phi 0.5
#define GAPA_DEFAULT_Z_PHI 0.15


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
  
  SAMPLE_DATA_1D_TYPE phi;
  float phin;
  float PErr_phi, IErr_phi;
  
  SAMPLE_DATA_1D_TYPE PHI;
  float PHIn;
  float PErr_PHI, IErr_PHI;

  float gamma; /* "right shift variable" */
  float GAMMA; /* "left shift variable" */
  
  float z_phi; /* the phi axis scale factor */
  float z_PHI; /* the PHI axis scale factor */
  
  SAMPLE_DATA_1D_TYPE nu;
  SAMPLE_DATA_1D_TYPE nu_norm; /* The phase angle on the region [0,1] */
  
  /* Boolean to mark the end of a gait cycle */
  bool Gait_End;
  
} GAPA_STATE_TYPE;


typedef struct
{
  int phase_method;

  float Kp_phi;
  float Ki_phi;
  float Kp_PHI;
  float Ki_PHI;

  float phi_mave_alpha;
  float PHI_mave_alpha;

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

