
/*******************************************************************
** FILE:
**    GaPA_Functions
** DESCRIPTION:
**    This file contains all functions spacific to the
**    Gait Phase Angle estimation (GaPA) capabilities
********************************************************************/


/*******************************************************************
** Includes ********************************************************
********************************************************************/

#ifndef COMMON_CONFIG_H
  #include "../Include/Common_Config.h"
#endif
#if EXE_MODE==1 /* Emulator Mode */
  /* In emulatiom mode, "Emulator_Protos" is needed to
  ** use funcitons in other files.
  ** NOTE: This header should contain the function
  **       prototypes for all execution functions */
  #include "../Include/Emulator_Protos.h"
#endif  /* End Emulator Mode */

/*******************************************************************
** Functions *******************************************************
********************************************************************/


/*****************************************************************
** FUNCTION: GaPA_Init
** VARIABLES:
**    [I ]  CONTROL_TYPE      *p_control
**    [IO]  GAPA_STATE_TYPE   *p_gapa_state
** RETURN:
**    NONE
** DESCRIPTION:
**    This function initializes the GaPA state
**    variables.
*/
void GaPA_Init( CONTROL_TYPE      *p_control,
                GAPA_STATE_TYPE   *p_gapa_state )
{
  char tmpBuffer[MAX_LINE_LEN];
  
  LOG_INFO( "> Initializing GaPA Parameters" );

  /*
  ** Initialize GaPA control parameters
  */

  p_control->gapa_prms.phase_method       = 1;
  p_control->gapa_prms.Kp_PHI             = GAPA_Kp_PHI;
  p_control->gapa_prms.Ki_PHI             = GAPA_Ki_PHI;
  p_control->gapa_prms.Kp_phi             = GAPA_Kp_phi;
  p_control->gapa_prms.Ki_phi             = GAPA_Ki_phi;
  p_control->gapa_prms.PHI_mave_alpha     = GAPA_PHImw_ALPHA;
  p_control->gapa_prms.phi_mave_alpha     = GAPA_phimw_ALPHA;
  p_control->gapa_prms.min_gyro           = GAPA_MIN_GYRO;
  p_control->gapa_prms.gait_end_threshold = GAPA_GAIT_END_THRESH;
  p_control->gapa_prms.default_z_phi      = GAPA_DEFAULT_Z_phi;
  p_control->gapa_prms.default_z_PHI      = GAPA_DEFAULT_Z_PHI;
  
    
  /*
  ** Initialize GaPA state parameters
  */

  /* The version of the phase portrait to use
  **  1:PHI
  **  2:PHV
  ** NOTE: Only PHI implemented at this time */
  p_gapa_state->version   = 1;

  p_gapa_state->iteration = 0;

  Init_Stats_1D( p_control, &p_gapa_state->phi );
  p_gapa_state->phi.val_mave_alpha = p_control->gapa_prms.phi_mave_alpha;
  p_gapa_state->z_phi    = p_control->gapa_prms.default_z_phi;
  p_gapa_state->gamma    = 0.0f;
  p_gapa_state->PErr_phi = 0.0f;
  p_gapa_state->IErr_phi = 0.0f;
  
  Init_Stats_1D( p_control, &p_gapa_state->PHI );
  p_gapa_state->PHI.val_mave_alpha = p_control->gapa_prms.PHI_mave_alpha;
  p_gapa_state->z_PHI    = p_control->gapa_prms.default_z_PHI;
  p_gapa_state->GAMMA    = 0.0f;
  p_gapa_state->PErr_PHI = 0.0f;
  p_gapa_state->IErr_PHI = 0.0f;

  Init_Stats_1D( p_control, &p_gapa_state->nu );
  p_gapa_state->nu.val[0]         = 0.0f;
  p_gapa_state->nu.val_mave_alpha = 0.9f; /* Weight towards new value */
  
  Init_Stats_1D( p_control, &p_gapa_state->nu_norm );
  p_gapa_state->nu_norm.val[0]         = 0.5f;
  p_gapa_state->nu_norm.val_mave_alpha = 0.9f; /* Weight towards new value */
  
  /* LOG */
  FltToStr( GAPA_Kp_PHI, 8, tmpBuffer );
  LOG_INFO( "GAPA_Kp_PHI          : %s", tmpBuffer );
  FltToStr( GAPA_Ki_PHI, 8, tmpBuffer );
  LOG_INFO( "GAPA_Ki_PHI          : %s", tmpBuffer );
  FltToStr( GAPA_Kp_phi, 8, tmpBuffer );
  LOG_INFO( "GAPA_Kp_phi          : %s", tmpBuffer );
  FltToStr( GAPA_Ki_phi, 8, tmpBuffer );
  LOG_INFO( "GAPA_Ki_phi          : %s", tmpBuffer );
  FltToStr( GAPA_PHImw_ALPHA, 8, tmpBuffer );
  LOG_INFO( "GAPA_PHImw_ALPHA     : %s", tmpBuffer );
  FltToStr( GAPA_phimw_ALPHA, 8, tmpBuffer );
  LOG_INFO( "GAPA_phimw_ALPHA     : %s", tmpBuffer );
  FltToStr( GAPA_MIN_GYRO, 1, tmpBuffer );
  LOG_INFO( "GAPA_MIN_GYRO        : %s", tmpBuffer );
  FltToStr( GAPA_GAIT_END_THRESH, 8, tmpBuffer );
  LOG_INFO( "GAPA_GAIT_END_THRESH : %s", tmpBuffer );
  FltToStr( GAPA_DEFAULT_Z_phi, 8, tmpBuffer );
  LOG_INFO( "GAPA_DEFAULT_Z_phi   : %s", tmpBuffer );
  FltToStr( GAPA_DEFAULT_Z_PHI, 8, tmpBuffer );
  LOG_INFO( "GAPA_DEFAULT_Z_PHI   : %s", tmpBuffer );
  
}/* End GaPA_Init */


/*****************************************************************
** FUNCTION: GaPA_Reset
** VARIABLES:
**    [I ]  CONTROL_TYPE      *p_control
**    [IO]  GAPA_STATE_TYPE   *p_gapa_state
** RETURN:
**    NONE
** DESCRIPTION:
**    The GaPA state variables must be reset
*/
void GaPA_Reset( CONTROL_TYPE     *p_control,
                 GAPA_STATE_TYPE  *p_gapa_state )
{
  p_gapa_state->gamma        = 0.0f;
  p_gapa_state->GAMMA        = 0.0f;
  p_gapa_state->z_phi        = p_control->gapa_prms.default_z_phi;
  p_gapa_state->z_PHI        = p_control->gapa_prms.default_z_PHI;
}/* End GaPA_Reset */



/*****************************************************************
** FUNCTION: GaPA_Update
** VARIABLES:
**    [I ]  CONTROL_TYPE      *p_control
**    [I ]  SENSOR_STATE_TYPE *p_sensor_state
**    [IO]  GAPA_STATE_TYPE   *p_gapa_state
** RETURN:
**    NONE
** DESCRIPTION:
**    This function runs the Gait Phase Angle (GaPA) estimator
**    This code will execute the code necessary to
**    solve for the phasae variable "nu"
**    where,
**      nu = atan2( -z*(PHI+GAMMA), -(phi+gamma) )
*/
void GaPA_Update( CONTROL_TYPE      *p_control,
                  SENSOR_STATE_TYPE *p_sensor_state,
                  GAPA_STATE_TYPE   *p_gapa_state )
{
  /* Initialize temp variables */
  int i;
  float tmpf;

  float R;
  float p1[3], p2[3], p3[3];
  float center[2];

  float leftParam, rightParam;

  /* Itaration Count */
  p_gapa_state->iteration++;
  
  /* Set our phase variables phi and PHI
  ** NOTE: version [1,2]:
  **  1: PHI
  **     phi: The thigh angle (pitch)
  **     PHI: The integral of the thigh angle (swing distance)
  **  2: PHV - NOT IMPLEMENTED!
  **     phi: The thigh angular velocity (swing angular velocity)
  **     PHI: The thigh angle (pitch)
  */
  p_gapa_state->version=1;
  switch( p_gapa_state->version )
  {
    case 1 : /* PHI */
      tmpf =  p_sensor_state->pitch.val[0] - p_gapa_state->PErr_phi - p_gapa_state->IErr_phi;
      Update_Stats_1D( p_control, &p_gapa_state->phi, tmpf );
      
      tmpf += p_gapa_state->phi.val[0]*p_control->G_Dt - p_gapa_state->PErr_PHI - p_gapa_state->IErr_PHI;
      Update_Stats_1D( p_control, &p_gapa_state->PHI, tmpf );
      break;
    case 2 : /* PHV */
      //p_gapa_state->phi = (p_sensor_state->pitch.val[0] - p_sensor_state->prev_pitch)*p_control->G_Dt;
      //p_gapa_state->PHI = p_sensor_state->pitch.val[0];
      break;
    default:
      // TO DO: Need to add some catch statements
      break;
  } /* End version switch */
  
  /* Compute phase variable feedback error */
  p_gapa_state->PErr_phi =  p_gapa_state->phi.val_mave * p_control->gapa_prms.Kp_phi;
  p_gapa_state->IErr_phi += p_gapa_state->phi.val_mave * p_control->gapa_prms.Ki_phi;
  p_gapa_state->PErr_PHI =  p_gapa_state->PHI.val_mave * p_control->gapa_prms.Kp_PHI;
  p_gapa_state->IErr_PHI += p_gapa_state->PHI.val_mave * p_control->gapa_prms.Ki_PHI;
  
  /* Scale by z */
  if(p_gapa_state->z_phi==0){ p_gapa_state->z_phi=p_control->gapa_prms.default_z_phi; }
  //p_gapa_state->phin = (p_gapa_state->phi.val[0]/p_gapa_state->z_phi);
  p_gapa_state->phin = (p_gapa_state->phi.val_mave/p_gapa_state->z_phi);
  
  if(p_gapa_state->z_PHI==0){ p_gapa_state->z_PHI=p_control->gapa_prms.default_z_PHI; }
  //p_gapa_state->PHIn = (p_gapa_state->PHI.val[0]/p_gapa_state->z_PHI);
  p_gapa_state->PHIn = (p_gapa_state->PHI.val_mave/p_gapa_state->z_PHI);

  /* Normalize to 1 for ease of computation and visualization */
  R = sqrt( p_gapa_state->phin*p_gapa_state->phin + p_gapa_state->PHIn*p_gapa_state->PHIn );
  p_gapa_state->phin = p_gapa_state->phin/R;
  p_gapa_state->PHIn = p_gapa_state->PHIn/R;

  /* We can only get a phase angle ofter 3 iterations */
  if( p_control->SampleNumber<10 ){ return; }

  /* Get the shift variables by determining the phase portrait center */
  p1[0] = p_gapa_state->phi.val[1]; p1[1] = p_gapa_state->PHI.val[1];
  p2[0] = p_gapa_state->phi.val[2]; p2[1] = p_gapa_state->PHI.val[2];
  p3[0] = p_gapa_state->phi.val[0]; p3[1] = p_gapa_state->PHI.val[0];
  calc_circle_center( p1, p2, p3, &center[0] );
  p_gapa_state->gamma = -center[0];
  p_gapa_state->GAMMA = -center[1];

  /* Get the phase angle */
  leftParam  = -1 * (p_gapa_state->PHIn+p_gapa_state->GAMMA);
  rightParam = -1 * (p_gapa_state->phin+p_gapa_state->gamma);
  //leftParam  = -1 * (p_gapa_state->PHI.val[0]+p_gapa_state->GAMMA);
  //rightParam = -1 * (p_gapa_state->phi.val[0]+p_gapa_state->gamma);
  tmpf = f_atan2( leftParam, rightParam );
  Update_Stats_1D( p_control, &p_gapa_state->nu, tmpf );
  
  /* Get the normalized phase angle (reported) 
  ** Computed only if there is motion detected */
  if( p_sensor_state->gyro.mag_mave>=p_control->gapa_prms.min_gyro )
  {
    tmpf = (p_gapa_state->nu.val[0]+PI)/(TWOPI);
  }
  else
  {
    tmpf = 0.5f;
  }
  Update_Stats_1D( p_control, &p_gapa_state->nu_norm, tmpf );
  
  /* Reset the normalization term at the start of each gait cycle
  ** The normalization term(i.e. the scaling factors "z") is the maximum
  ** absolute value from the previous iteration. I am using it to ensure the 
  ** magnitude of both of the phase variables is the same and smoothish */
  if( (fabs(p_gapa_state->nu.val[0]-p_gapa_state->nu.val[1])>p_control->gapa_prms.gait_end_threshold ) &&
      (p_sensor_state->gyro.mag_mave>=p_control->gapa_prms.min_gyro) )
  {
    /* Update phi "z" scaling parameter */
    p_gapa_state->z_phi = p_gapa_state->phi.val_max;
    if(p_gapa_state->z_phi==0){ p_gapa_state->z_phi=p_control->gapa_prms.default_z_phi; }
    p_gapa_state->phi.val_max = FABS( p_gapa_state->phi.val[0] );
    
    /* Update PHI "z" scaling parameter */
    p_gapa_state->z_PHI = p_gapa_state->PHI.val_max;
    if(p_gapa_state->z_PHI==0){ p_gapa_state->z_PHI=p_control->gapa_prms.default_z_PHI; }
    p_gapa_state->PHI.val_max = FABS( p_gapa_state->PHI.val[0] );
      
    /* Mark the end of gain flag
    ** This will indicate when we believe we have completed a cycle. */
    p_gapa_state->Gait_End = TRUE;
  }
  
  /* No Motion detected (subject assumed stopped)
  ** Reset phase variables to prepare for motion */
  if( (p_sensor_state->gyro.mag_mave<p_control->gapa_prms.min_gyro) )
  {
    p_gapa_state->z_phi         = p_gapa_state->phi.val_max;
    //p_gapa_state->z_phi         = p_control->gapa_prms.default_z_phi;
    p_gapa_state->phi.val[0]    = 0.0;
    p_gapa_state->phin          = 0.0;
    p_gapa_state->phi.val_max   = 0.0;
        
    p_gapa_state->z_PHI         = p_gapa_state->PHI.val_max;
    //p_gapa_state->z_PHI         = p_control->gapa_prms.default_z_PHI;
    p_gapa_state->PHI.val[0]    = 0.0;
    p_gapa_state->PHIn          = 0.0;
    p_gapa_state->PHI.val_max   = 0.0;
        
    p_gapa_state->nu_norm.val[0] = 0.0;
  }

}/* End GaPA_Update */

