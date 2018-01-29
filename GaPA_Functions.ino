
/*******************************************************************
** FILE:
**   	GaPA_Functions
** DESCRIPTION:
** 		This file contains all functions spacific to the
** 		Gait Phase Angle estimation (GaPA) capabilities
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
	** 			 prototypes for all execution functions */
	#include "../Include/Emulator_Protos.h"
#endif  /* End Emulator Mode */

/*******************************************************************
** Functions *******************************************************
********************************************************************/


/*****************************************************************
** FUNCTION: GaPA_Init
** VARIABLES:
**		[I ]	CONTROL_TYPE			*p_control
**		[IO]	GAPA_STATE_TYPE		*p_gapa_state
** RETURN:
**		NONE
** DESCRIPTION:
** 		This function initializes the GaPA state
** 		variables.
*/
void GaPA_Init( CONTROL_TYPE			*p_control,
								GAPA_STATE_TYPE		*p_gapa_state )
{
  LOG_PRINTLN("> Initializing GaPA Parameters");

	/*
	** Initialize GaPA control parameters
	*/

	p_control->gapa_prms.phase_method 			= 1;
	p_control->gapa_prms.Kp_PHI 						= GAPA_Kp_PHI;
	p_control->gapa_prms.Ki_PHI 						= GAPA_Ki_PHI;
	p_control->gapa_prms.Kp_phi 						= GAPA_Kp_phi;
	p_control->gapa_prms.Ki_phi 						= GAPA_Ki_phi;
	p_control->gapa_prms.PHImw_alpha 				= GAPA_PHImw_ALPHA;
	p_control->gapa_prms.phimw_alpha 				= GAPA_phimw_ALPHA;
	p_control->gapa_prms.min_gyro 				  = GAPA_MIN_GYRO;
	p_control->gapa_prms.gait_end_threshold = GAPA_GAIT_END_THRESH;

	/*
	** Initialize GaPA state parameters
	*/

	/* The version of the phase portrait to use
	**  1:PHI
	**  2:PHV
	** NOTE: Only PHI implemented at this time */
	p_gapa_state->version      = 1;

	p_gapa_state->iteration    = 0;

	p_gapa_state->phi_max      = 0.0f;
	p_gapa_state->phi_min      = 0.0f;
	p_gapa_state->PHI_max      = 0.0f;
	p_gapa_state->PHI_min      = 0.0f;
	p_gapa_state->phi_max_next = 0.0f;
	p_gapa_state->phi_min_next = 0.0f;
	p_gapa_state->PHI_max_next = 0.0f;
	p_gapa_state->PHI_min_next = 0.0f;
	p_gapa_state->gamma        = 0.0f;
	p_gapa_state->GAMMA        = 0.0f;
	p_gapa_state->z_phi        = 1.0f;
	p_gapa_state->z_PHI        = 1.0f;
	p_gapa_state->phi_mw       = 0.0f;
	p_gapa_state->PHI_mw       = 0.0f;
	p_gapa_state->PErr_phi     = 0.0f;
	p_gapa_state->IErr_phi     = 0.0f;
	p_gapa_state->PErr_PHI     = 0.0f;
	p_gapa_state->IErr_PHI     = 0.0f;
	p_gapa_state->nu           = 0.0f;
}/* End GaPA_Init */


/*****************************************************************
** FUNCTION: GaPA_Reset
** VARIABLES:
**		[I ]	CONTROL_TYPE			*p_control
**		[IO]	GAPA_STATE_TYPE		*p_gapa_state
** RETURN:
**		NONE
** DESCRIPTION:
** 		The GaPA state variables must be reset
*/
void GaPA_Reset( CONTROL_TYPE			*p_control,
								 GAPA_STATE_TYPE	*p_gapa_state )
{
	p_gapa_state->phi_max_next = 0.0f;
	p_gapa_state->phi_min_next = 0.0f;
	p_gapa_state->PHI_max_next = 0.0f;
	p_gapa_state->PHI_min_next = 0.0f;
	p_gapa_state->gamma        = 0.0f;
	p_gapa_state->GAMMA        = 0.0f;
	p_gapa_state->z_phi        = 0.0f;
	p_gapa_state->z_PHI        = 0.0f;
}/* End GaPA_Reset */



/*****************************************************************
** FUNCTION: GaPA_Update
** VARIABLES:
**		[I ]	CONTROL_TYPE			*p_control
**		[I ]	SENSOR_STATE_TYPE *p_sensor_state
**		[IO]	GAPA_STATE_TYPE		*p_gapa_state
** RETURN:
**		NONE
** DESCRIPTION:
** 		This function runs the Gait Phase Angle (GaPA) estimator
** 		This code will execute the code necessary to
** 		solve for the phasae variable "nu"
** 		where,
**   		nu = atan2( -z*(PHI+GAMMA), -(phi+gamma) )
*/
void GaPA_Update( CONTROL_TYPE			*p_control,
									SENSOR_STATE_TYPE *p_sensor_state,
								 	GAPA_STATE_TYPE		*p_gapa_state )
{
	/* Initialize temp variables */
	int i;

	float R;
	float p1[3], p2[3], p3[3];
	float center[2];

	float leftParam, rightParam;

	/* Itaration Count */
	p_gapa_state->iteration++;

	/* Store previous nu, phi and PHI
	** NOTE: On the first cycle, this is meaningless since
	**       both nu and nu_prev are zero */
	p_gapa_state->nu_prev = p_gapa_state->nu;
	for( i=2;i>0; i-- ){ p_gapa_state->prev_phi[i-1]=p_gapa_state->prev_phi[i]; }
	for( i=2;i>0; i-- ){ p_gapa_state->prev_PHI[i-1]=p_gapa_state->prev_PHI[i]; }
	p_gapa_state->prev_phi[2] = p_gapa_state->phin;
	p_gapa_state->prev_PHI[2] = p_gapa_state->PHIn;


	/* Set our phase variables phi and PHI
	** NOTE: version [1,2]:
	** 	1: PHI
	**		 phi: The thigh angle (pitch)
	**		 PHI: The integral of the thigh angle (swing distance)
	**  2: PHV - NOT IMPLEMENTED!
	**		 phi: The thigh angular velocity (swing angular velocity)
	**		 PHI: The thigh angle (pitch)
	*/
	p_gapa_state->version=1;
	switch( p_gapa_state->version )
	{
		case 1 : /* PHI */
			p_gapa_state->phi =  p_sensor_state->pitch - p_gapa_state->PErr_phi - p_gapa_state->IErr_phi;
			p_gapa_state->PHI += p_gapa_state->phi*p_control->G_Dt - p_gapa_state->PErr_PHI - p_gapa_state->IErr_PHI;
			break;
		case 2 : /* PHV */
			//p_gapa_state->phi = (p_sensor_state->pitch - p_sensor_state->prev_pitch)*p_control->G_Dt;
			//p_gapa_state->PHI = p_sensor_state->pitch;
			break;
		default :
			// Need to add some catch statements
			break;
	} /* End version switch */


	/* Compute the windowed moving average of each of the
	** phase variables */
	p_gapa_state->phi_mw = Windowed_Mean( p_gapa_state->phi_mw, p_gapa_state->phi, p_gapa_state->iteration, p_control->gapa_prms.phimw_alpha );
	p_gapa_state->PHI_mw = Windowed_Mean( p_gapa_state->PHI_mw, p_gapa_state->PHI, p_gapa_state->iteration, p_control->gapa_prms.PHImw_alpha );

	/* Compute phase variable feedback error */
	p_gapa_state->PErr_phi =  p_gapa_state->phi_mw * p_control->gapa_prms.Kp_phi;
	p_gapa_state->IErr_phi += p_gapa_state->phi_mw * p_control->gapa_prms.Ki_phi;
	p_gapa_state->PErr_PHI =  p_gapa_state->PHI_mw * p_control->gapa_prms.Kp_PHI;
	p_gapa_state->IErr_PHI += p_gapa_state->PHI_mw * p_control->gapa_prms.Ki_PHI;

	/* Record phase variable min and max */
	p_gapa_state->phi_max = MAX( p_gapa_state->phi_max, p_gapa_state->phi );
	p_gapa_state->PHI_max = MAX( p_gapa_state->PHI_max, p_gapa_state->PHI );

	/* Scale by z */
	if(p_gapa_state->z_phi==0){p_gapa_state->z_phi=1;}
	if(p_gapa_state->z_PHI==0){p_gapa_state->z_PHI=1;}
	p_gapa_state->phin = p_gapa_state->phi/p_gapa_state->z_phi;
	p_gapa_state->PHIn = p_gapa_state->PHI/p_gapa_state->z_PHI;

	/* Normalize to 1 */
	R = sqrt( p_gapa_state->phin*p_gapa_state->phin + p_gapa_state->PHIn*p_gapa_state->PHIn );
	p_gapa_state->phin = p_gapa_state->phin/R;
	p_gapa_state->PHIn = p_gapa_state->PHIn/R;

	/* We can only get a phase angle ofter 3 iterations */
	if( p_gapa_state->iteration<10 )
	{
		return;
	}

	/* Get the shift variables by determining the phase portrait center */
	p1[0] = p_gapa_state->prev_phi[1]; p1[1] = p_gapa_state->prev_PHI[1];
	p2[0] = p_gapa_state->prev_phi[2]; p2[1] = p_gapa_state->prev_PHI[2];
	p3[0] = p_gapa_state->phin; p3[1] = p_gapa_state->PHIn;
	calc_circle_center( p1, p2, p3, &center[0] );
	p_gapa_state->gamma = -center[0];
	p_gapa_state->GAMMA = -center[1];

	/* Get the input to the atan2 calc */
	leftParam  = -1 * (p_gapa_state->PHIn+p_gapa_state->GAMMA);
	rightParam = -1 * (p_gapa_state->phin+p_gapa_state->gamma);

	/* Get the phase angle */
	p_gapa_state->nu = f_atan2( leftParam, rightParam );

	/* Detect the end of gait */
	if( fabs(p_gapa_state->nu-p_gapa_state->nu_prev) > p_control->gapa_prms.gait_end_threshold )
	{
		p_gapa_state->z_phi = p_gapa_state->phi_max;
		if(p_gapa_state->z_phi==0){p_gapa_state->z_phi=1;}
		p_gapa_state->z_PHI = p_gapa_state->PHI_max;
		if(p_gapa_state->z_PHI==0){p_gapa_state->z_PHI=1;}

		p_gapa_state->phi_max = fabs( p_gapa_state->phi );
		p_gapa_state->PHI_max = fabs( p_gapa_state->PHI );
	}

	/* If no motion, reset phase variables */

}/* End GaPA_Update */

/*****************************************************************
** FUNCTION: TrackPhiVariables
** VARIABLES:
**		[IO]	GAPA_STATE_TYPE		*p_gapa_state
** RETURN:
**		NONE
** DESCRIPTION:
** 		Keep track of the min/max of the phase variables phi and PHI.
** 		These will be used to scale the phase portrait to force it to
** 		Be close to a constant radius.
*/
void TrackPhiVariables( GAPA_STATE_TYPE* p_gapa_state )
{
	/* Update phi min/max */
	if( p_gapa_state->phi > p_gapa_state->phi_max_next )
	{
		p_gapa_state->phi_max_next = p_gapa_state->phi;
	}
	else if( p_gapa_state->phi < p_gapa_state->phi_min_next )
	{
		p_gapa_state->phi_min_next = p_gapa_state->phi;
	}

	/* Update PHI min/max */
	if( p_gapa_state->PHI > p_gapa_state->PHI_max_next )
	{
		p_gapa_state->PHI_max_next = p_gapa_state->PHI;
	}
	else if( p_gapa_state->PHI < p_gapa_state->PHI_min_next )
	{
		p_gapa_state->PHI_min_next = p_gapa_state->PHI;
	}
} /* End TrackPhiVariables */


/*****************************************************************
** FUNCTION: calc_SftPrmLeft
** VARIABLES:
**		[IO]	float	*GAMMA
**		[I ]	float	PHI_max
**		[I ]	float	PHI_min
** RETURN:
**		NONE
** DESCRIPTION:
** 		Calculate the left side shift parameter "GAMMA"
** 		where,
**   		GAMMA = -( (PHI_max+PHI_min)/2 )
*/
void calc_SftPrmLeft( float* GAMMA, float PHI_max, float PHI_min )
{
	(*GAMMA) = -( (PHI_max+PHI_min)*0.5 );
}/* End calc_SftPrmLeft */


/*****************************************************************
** FUNCTION: calc_SftPrmRight
** VARIABLES:
**		[IO]	float	*gamma
**		[I ]	float	phi_max
**		[I ]	float	phi_min
** RETURN:
**		NONE
** DESCRIPTION:
** 		Calculate the right side shift parameter "gamma"
** 		where,
**   		gamma = -( (phi_max+phi_min)/2 )
*/
void calc_SftPrmRight( float* gamma, float phi_max, float phi_min )
{
	(*gamma) = -( (phi_max+phi_min)*0.5 );
}/* End calc_SftPrmRight */


/*****************************************************************
** FUNCTION: calc_ScaleFactor
** VARIABLES:
**		[IO]	float	*z
**		[I ]	float	phi_max
**		[I ]	float	phi_min
**		[I ]	float	PHI_max
**		[I ]	float	PHI_min
** RETURN:
**		NONE
** DESCRIPTION:
** 		Calculate the shift parameter "z"
** 		where,
**   		z = abs(phi_max-phi_min)/abs(PHI_max-PHI_min)
*/
void calc_ScaleFactor( float *z, float phi_max, float phi_min, float PHI_max, float PHI_min )
{
    *z = fabs( (phi_max-phi_min)/(PHI_max-PHI_min) );
}/* End calc_ScaleFactor */


/*****************************************************************
** FUNCTION: calc_PhaseAngle
** VARIABLES:
**		[IO]	float	*nu
**		[I ]	float	z
**		[I ]	float	PHI
**		[I ]	float	GAMMA
**		[I ]	float	phi
**		[I ]	float	gamma
** RETURN:
**		NONE
** DESCRIPTION:
** 		Calculate the phase angle "nu"
** 		where,
**   		nu = atan2( -z*(PHI+GAMMA), -(phi+gamma) )
*/
void calc_PhaseAngle( float* nu, float z, float PHI, float GAMMA, float phi, float gamma )
{
	(*nu) = f_atan2( (-z*(PHI+GAMMA)) , (-phi+gamma) );
}/* End calc_PhaseAngle */



