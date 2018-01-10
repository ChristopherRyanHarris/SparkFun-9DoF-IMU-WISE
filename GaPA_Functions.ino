/*************************************************
** FILE: GaPa_Functions
** This file contains all functions spacific to the
** Gait Phase Angle estimation (GaPA) capabilities
**************************************************/


/*******************************************************************
** Includes ********************************************************
********************************************************************/

#include "../Include/Common_Config.h"

#if EXE_MODE /* Emulator mode */
#include <math.h>
#include <string.h>

#ifdef _IMU10736_
#include "../Include/IMU10736_Config.h"
#endif
#ifdef _IMU9250_
#include <SparkFunMPU9250-DMP.h>
#include "../Include/IMU9250_Config.h"
#endif

#include "../Include/Math.h"
#include "../Include/GaPA_Config.h"
#include "../Include/DCM_Config.h"
#include "../Include/Emulator_Config.h"

extern CAL_STATE_TYPE      g_calibration;
extern DCM_STATE_TYPE      g_dcm_state;
extern DSP_COMMON_TYPE     g_dsp;
extern SENSOR_STATE_TYPE   g_sensor_state;
extern CONTROL_STATE_TYPE  g_control_state;
extern GAPA_STATE_TYPE     g_gapa_state;
#endif /* End Emulator mode */

/*******************************************************************
** Functions *******************************************************
********************************************************************/




/*****************************************************************
** Function: GaPA_Init
** This function initializes the GaPA state
** variables.
*/
void GaPA_Init( void )
{
	/* The version of the phase portrait to use
	**  1:PHI
	**  2:PHV
	** NOTE: Only PHI implemented at this time */
	g_gapa_state.version      = 1;

	g_gapa_state.iteration    = 0;

	g_gapa_state.phi_max      = 0.0f;
	g_gapa_state.phi_min      = 0.0f;
	g_gapa_state.PHI_max      = 0.0f;
	g_gapa_state.PHI_min      = 0.0f;
	g_gapa_state.phi_max_next = 0.0f;
	g_gapa_state.phi_min_next = 0.0f;
	g_gapa_state.PHI_max_next = 0.0f;
	g_gapa_state.PHI_min_next = 0.0f;
	g_gapa_state.gamma        = 0.0f;
	g_gapa_state.GAMMA        = 0.0f;
	g_gapa_state.z_phi        = 1.0f;
	g_gapa_state.z_PHI        = 1.0f;
	g_gapa_state.phi_mw       = 0.0f;
	g_gapa_state.PHI_mw       = 0.0f;
	g_gapa_state.PErr_phi     = 0.0f;
	g_gapa_state.IErr_phi     = 0.0f;
	g_gapa_state.PErr_PHI     = 0.0f;
	g_gapa_state.IErr_PHI     = 0.0f;
	g_gapa_state.nu           = 0.0f;
}/* End GaPA_Init */


/*****************************************************************
** Function: GaPA_Reset
** The GaPA state variables must be reset
*/
void GaPA_Reset( void )
{
	g_gapa_state.phi_max_next = 0.0f;
	g_gapa_state.phi_min_next = 0.0f;
	g_gapa_state.PHI_max_next = 0.0f;
	g_gapa_state.PHI_min_next = 0.0f;
	g_gapa_state.gamma        = 0.0f;
	g_gapa_state.GAMMA        = 0.0f;
	g_gapa_state.z_phi        = 0.0f;
	g_gapa_state.z_PHI        = 0.0f;
}/* End GaPA_Reset */



/*****************************************************************
** Function: GaPA_Update
** This function runs the Gait Phase Angle (GaPA) estimator
** This code will execute the code necessary to
** solve for the phasae variable "nu"
** where,
**   nu = atan2( -z*(PHI+GAMMA), -(phi+gamma) )
*/
void GaPA_Update( void )
{
	/* Initialize temp variables */
	int i;

	float R;
	float p1[3], p2[3], p3[3];
	float center[2];

	float leftParam, rightParam;


	/* Itaration Count */
	g_gapa_state.iteration++;

	/* Store previous nu, phi and PHI
	** NOTE: On the first cycle, this is meaningless since
	**       both nu and nu_prev are zero */
	g_gapa_state.nu_prev = g_gapa_state.nu;
	for( i=2;i>0; i-- ){ g_gapa_state.prev_phi[i-1]=g_gapa_state.prev_phi[i]; }
	for( i=2;i>0; i-- ){ g_gapa_state.prev_PHI[i-1]=g_gapa_state.prev_PHI[i]; }
	g_gapa_state.prev_phi[2] = g_gapa_state.phin;
	g_gapa_state.prev_PHI[2] = g_gapa_state.PHIn;


	/* Set our phase variables phi and PHI
	** NOTE: version [1,2]:
	** 	1: PHI
	**		 phi: The thigh angle (pitch)
	**		 PHI: The integral of the thigh angle (swing distance)
	**  2: PHV - NOT IMPLEMENTED!
	**		 phi: The thigh angular velocity (swing angular velocity)
	**		 PHI: The thigh angle (pitch)
	*/
	g_gapa_state.version=1;
	switch( g_gapa_state.version )
	{
		case 1 : /* PHI */
			g_gapa_state.phi =  g_sensor_state.pitch - g_gapa_state.PErr_phi - g_gapa_state.IErr_phi;
			g_gapa_state.PHI += g_gapa_state.phi*g_control_state.G_Dt - g_gapa_state.PErr_PHI - g_gapa_state.IErr_PHI;
			break;
		case 2 : /* PHV */
			//g_gapa_state.phi = (g_sensor_state.pitch - g_sensor_state.prev_pitch)*g_control_state.G_Dt;
			//g_gapa.state.PHI = g_sensor_state.pitch;
			break;
		default :
			// Need to add some catch statements
			break;
	} /* End version switch */


	/* Compute the windowed moving average of each of the
	** phase variables */
	g_gapa_state.phi_mw = Windowed_Mean( g_gapa_state.phi_mw, g_gapa_state.phi, g_gapa_state.iteration, (float)0.01 );
	g_gapa_state.PHI_mw = Windowed_Mean( g_gapa_state.PHI_mw, g_gapa_state.PHI, g_gapa_state.iteration, (float)0.01 );

	/* Compute phase variable feedback error */
	g_gapa_state.PErr_phi =  g_gapa_state.phi_mw * 0.01;
	g_gapa_state.IErr_phi += g_gapa_state.phi_mw * 0.01;
	g_gapa_state.PErr_PHI =  g_gapa_state.PHI_mw * 0.01;
	g_gapa_state.IErr_PHI += g_gapa_state.PHI_mw * 0;

	/* Record phase variable min and max */
	g_gapa_state.phi_max = MAX( g_gapa_state.phi_max, g_gapa_state.phi );
	g_gapa_state.PHI_max = MAX( g_gapa_state.PHI_max, g_gapa_state.PHI );

	/* Scale by z */
	if(g_gapa_state.z_phi==0){g_gapa_state.z_phi=1;}
	if(g_gapa_state.z_PHI==0){g_gapa_state.z_PHI=1;}
	g_gapa_state.phin = g_gapa_state.phi/g_gapa_state.z_phi;
	g_gapa_state.PHIn = g_gapa_state.PHI/g_gapa_state.z_PHI;

	/* Normalize to 1 */
	R = sqrt( g_gapa_state.phin*g_gapa_state.phin + g_gapa_state.PHIn*g_gapa_state.PHIn );
	g_gapa_state.phin = g_gapa_state.phin/R;
	g_gapa_state.PHIn = g_gapa_state.PHIn/R;

	/* We can only get a phase angle ofter 3 iterations */
	if( g_gapa_state.iteration<10 )
	{
		return;
	}

	/* Get the shift variables by determining the phase portrait center */
	p1[0] = g_gapa_state.prev_phi[1]; p1[1] = g_gapa_state.prev_PHI[1];
	p2[0] = g_gapa_state.prev_phi[2]; p2[1] = g_gapa_state.prev_PHI[2];
	p3[0] = g_gapa_state.phin; p3[1] = g_gapa_state.PHIn;
	calc_circle_center( p1, p2, p3, &center[0] );
	g_gapa_state.gamma = -center[0];
	g_gapa_state.GAMMA = -center[1];

	/* Get the input to the atan2 calc */
	leftParam  = -1 * (g_gapa_state.PHIn+g_gapa_state.GAMMA);
	rightParam = -1 * (g_gapa_state.phin+g_gapa_state.gamma);

	/* Get the phase angle */
	g_gapa_state.nu = f_atan2( leftParam, rightParam );

	/* Detect the end of gait */
	if( fabs(g_gapa_state.nu-g_gapa_state.nu_prev) > 0.25*PI )
	{
		g_gapa_state.z_phi = g_gapa_state.phi_max;
		if(g_gapa_state.z_phi==0){g_gapa_state.z_phi=1;}
		g_gapa_state.z_PHI = g_gapa_state.PHI_max;
		if(g_gapa_state.z_PHI==0){g_gapa_state.z_PHI=1;}

		g_gapa_state.phi_max = fabs( g_gapa_state.phi );
		g_gapa_state.PHI_max = fabs( g_gapa_state.PHI );
	}

	/* If no motion, reset phase variables */

}/* End GaPA_Update */

/*****************************************************************
** Function: TrackPhiVariables
** Keep track of the min/max of the phase variables phi and PHI.
** These will be used to scale the phase portrait to force it to
** Be close to a constant radius.
*/
void TrackPhiVariables( GAPA_STATE_TYPE* l_gapa_state )
{
	/* Update phi min/max */
	if( l_gapa_state->phi > l_gapa_state->phi_max_next )
	{
		l_gapa_state->phi_max_next = l_gapa_state->phi;
	}
	else if( l_gapa_state->phi < l_gapa_state->phi_min_next )
	{
		l_gapa_state->phi_min_next = l_gapa_state->phi;
	}

	/* Update PHI min/max */
	if( l_gapa_state->PHI > l_gapa_state->PHI_max_next )
	{
		l_gapa_state->PHI_max_next = l_gapa_state->PHI;
	}
	else if( l_gapa_state->PHI < l_gapa_state->PHI_min_next )
	{
		l_gapa_state->PHI_min_next = l_gapa_state->PHI;
	}
} /* End TrackPhiVariables */


/*****************************************************************
** Function: calc_SftPrmLeft
** Calculate the left side shift parameter "GAMMA"
** where,
**   GAMMA = -( (PHI_max+PHI_min)/2 )
*/
void calc_SftPrmLeft( float* GAMMA, float PHI_max, float PHI_min )
{
	(*GAMMA) = -( (g_gapa_state.PHI_max+PHI_min)*0.5 );
}/* End calc_SftPrmLeft */


/*****************************************************************
** Function: calc_SftPrmRight
** Calculate the right side shift parameter "gamma"
** where,
**   gamma = -( (phi_max+phi_min)/2 )
*/
void calc_SftPrmRight( float* gamma, float phi_max, float phi_min )
{
	(*gamma) = -( (phi_max+phi_min)*0.5 );
}/* End calc_SftPrmRight */


/*****************************************************************
** Function: calc_ScaleFactor
** Calculate the shift parameter "z"
** where,
**   z = abs(phi_max-phi_min)/abs(PHI_max-PHI_min)
*/
void calc_ScaleFactor( float *z, float phi_max, float phi_min, float PHI_max, float PHI_min )
{
    *z = fabs( (g_gapa_state.phi_max-phi_min)/(g_gapa_state.PHI_max-PHI_min) );
}/* End calc_ScaleFactor */


/*****************************************************************
** Function: calc_PhaseAngle
** Calculate the phase angle "nu"
** where,
**   nu = atan2( -z*(PHI+GAMMA), -(phi+gamma) )
*/
void calc_PhaseAngle( float* nu, float z, float PHI, float GAMMA, float phi, float gamma )
{
	(*nu) = f_atan2( (-z*(PHI+GAMMA)) , (-phi+gamma) );
}/* End calc_PhaseAngle */



