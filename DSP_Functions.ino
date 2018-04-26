
/*******************************************************************
** FILE:
**   	DSP_Functions
** DESCRIPTION:
**		The Digital Signal Processing (DSP) functions.
** 		This file contains the functions specific to
** 		our low pass filter functions.
** 		We are supplying support for several flavors,
** 		but in the general case, we will be applying a
** 		simple filters on each of the inputs.
** 		A short FIR LP filter and a IIR filter on
** 		each of the input accelerations
** 		A short FIR HP filter on the gyro
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


/*************************************************
** FUNCTION: DSP_Filter_Init
** VARIABLES:
**		[IO]	CONTROL_TYPE		*p_control
**		[IO]	DSP_STATE_TYPE	*p_dsp_state
** RETURN:
**		NONE
** DESCRIPTION:
** 		This function initializes the state memory
*/
void DSP_Filter_Init ( CONTROL_TYPE			*p_control,
											 DSP_STATE_TYPE		*p_dsp_state )
{
	int i,j;

  float IIR_coeffs_La[NTAPS] = IIR_LPF_a;
	float IIR_coeffs_Lb[NTAPS] = IIR_LPF_b;
	float IIR_coeffs_Ha[NTAPS] = IIR_HPF_a;
	float IIR_coeffs_Hb[NTAPS] = IIR_HPF_b;
	float FIR_coeffs_L[NTAPS]  = FIR_LPF;
	float FIR_coeffs_H[NTAPS]  = FIR_HPF;

  LOG_PRINTLN("> Initializing DSP Filter");

  /*
  ** Initialize DSP control parameters
  */

	p_control->dsp_prms.n_taps = NTAPS;
	p_control->dsp_prms.FIR_on = DSP_FIR_ON;
	p_control->dsp_prms.IIR_on = DSP_IIR_ON;

	/*
	** Initialize DSP state parameters
	*/

	memcpy(&p_dsp_state->IIR_coeffs_La[0],&IIR_coeffs_La[0],NTAPS*sizeof(float));
	memcpy(&p_dsp_state->IIR_coeffs_Lb[0],&IIR_coeffs_Lb[0],NTAPS*sizeof(float));
	memcpy(&p_dsp_state->IIR_coeffs_Ha[0],&IIR_coeffs_Ha[0],NTAPS*sizeof(float));
	memcpy(&p_dsp_state->IIR_coeffs_Hb[0],&IIR_coeffs_Hb[0],NTAPS*sizeof(float));
	memcpy(&p_dsp_state->FIR_coeffs_L[0],&FIR_coeffs_L[0],NTAPS*sizeof(float));
	memcpy(&p_dsp_state->FIR_coeffs_H[0],&FIR_coeffs_H[0],NTAPS*sizeof(float));

	for( i=0;i<3;i++ )
	{
		for( j=0;j<NTAPS;j++ )
		{
			p_dsp_state->accel_mem[i][j] = 0.0f;
			p_dsp_state->gyro_mem[i][j] = 0.0f;
		}
	}
} /* End DSP_Filter_Init */


/*************************************************
** FUNCTION: DSP_Update
** VARIABLES:
**		[I ]	CONTROL_TYPE			*p_control
**		[IO]	DSP_STATE_TYPE		*p_dsp_state
**		[I ]	SENSOR_STATE_TYPE *p_sensor_state
** RETURN:
**		NONE
** DESCRIPTION:
** 		This function updates the necessary
** 		DSP state variabls. This includes
** 		the memory arrays
*/
void DSP_Update ( CONTROL_TYPE			*p_control,
									DSP_STATE_TYPE		*p_dsp_state,
									SENSOR_STATE_TYPE *p_sensor_state )
{
	int i;
	/* log new inputs */
	for( i=0;i<3;i++ )
	{
		p_dsp_state->accel_mem[i][0] = p_sensor_state->accel[i];
		p_dsp_state->gyro_mem[i][0]  = p_sensor_state->gyro[i];
	}
} /* DSP_Update */


/*************************************************
** FUNCTION: DSP_Shift
** VARIABLES:
**		[I ]	CONTROL_TYPE			*p_control
**		[IO]	DSP_STATE_TYPE		*p_dsp_state
** RETURN:
**		NONE
** DESCRIPTION:
** 		This function shifts the memory
** 		arrays in preparation for the next
** 		iteration
*/
void DSP_Shift ( CONTROL_TYPE				*p_control,
								 DSP_STATE_TYPE		*p_dsp_state )
{
	int i,j;
	/* Shift sesor values */
	for( i=0;i<3;i++)
	{
		for( j=0;j<(NTAPS-1);j++)
		{
			p_dsp_state->accel_mem[i][j+1] = p_dsp_state->accel_mem[i][j];
			p_dsp_state->gyro_mem[i][j+1]  = p_dsp_state->gyro_mem[i][j];
		}
	}
} /* End DSP_Shift */


/*************************************************
** FUNCTION: IIR_Filter
** VARIABLES:
**		[I ]	CONTROL_TYPE			*p_control
**		[IO]	DSP_STATE_TYPE		*p_dsp_state
**		[IO]	SENSOR_STATE_TYPE *p_sensor_state
** RETURN:
**		NONE
** DESCRIPTION:
** 		This function applies a IIR filter
** 		to the input sensor data.
** 		Equation:
**			y[n] = (1/a[0]) * ( b[0]*x[n] + ... + b[P]*x[n-P] - a[1]*y[n-1] - ... - a[Q]*y[n-Q] )
*/
void IIR_Filter ( CONTROL_TYPE				*p_control,
								  DSP_STATE_TYPE			*p_dsp_state,
									SENSOR_STATE_TYPE 	*p_sensor_state )
{
	/* TO DO: Add functionality for additional modes */
	int i,j;
	float temp;

	DSP_Update( p_control, p_dsp_state, p_sensor_state );

	/* Accel - LPF */
	for( j=0;j<3;j++ )
	{
		temp = 0.0f;
		for( i=0;i<NTAPS;i++ ) { temp = temp + p_dsp_state->IIR_coeffs_Lb[i]*p_dsp_state->accel_mem[j][i]; }
		for( i=1;i<NTAPS;i++ ) { temp = temp - p_dsp_state->IIR_coeffs_La[i]*p_dsp_state->accel_mem[j][i]; }
		p_sensor_state->accel[j] = (1/p_dsp_state->IIR_coeffs_La[0])*temp;
	}
	/* Gyro - HPF */
	for( j=0;j<3;j++ )
	{
		temp = 0.0f;
		for( i=0;i<NTAPS;i++ ) { temp = temp + p_dsp_state->IIR_coeffs_Hb[i]*p_dsp_state->accel_mem[j][i]; }
		for( i=1;i<NTAPS;i++ ) { temp = temp - p_dsp_state->IIR_coeffs_Ha[i]*p_dsp_state->accel_mem[j][i]; }
		p_sensor_state->gyro[j] = (1/p_dsp_state->IIR_coeffs_Ha[0])*temp;
	}
} /* End IIR_Filter */


/*************************************************
** FUNCTION: FIR_Filter
** VARIABLES:
**		[I ]	CONTROL_TYPE			*p_control
**		[IO]	DSP_STATE_TYPE		*p_dsp_state
**		[IO]	SENSOR_STATE_TYPE *p_sensor_state
** RETURN:
**		NONE
** DESCRIPTION:
** 		This function applies a FIR filter
** 		to the input sensor data.
** 		Equation:
**			y[n] = b[0]*x[n] + b[1]*x[n-1] + ... + b[N]*x[n-N]
*/
void FIR_Filter ( CONTROL_TYPE				*p_control,
								  DSP_STATE_TYPE			*p_dsp_state,
									SENSOR_STATE_TYPE 	*p_sensor_state )
{
	/* TO DO: Add functionality for additional modes */
	int i,j;
	float temp;

	DSP_Update( p_control, p_dsp_state, p_sensor_state );

	/* Accel - LPF */
	for( j=0;j<3;j++ )
	{
		temp = 0.0f;
		for( i=0;i<NTAPS;i++ ) { temp = temp + p_dsp_state->FIR_coeffs_L[i]*p_dsp_state->accel_mem[j][i]; }
		p_sensor_state->accel[j] = temp;
	}
	/* Gyro - HPF */
	for( j=0;j<3;j++ )
	{
		temp = 0.0f;
		for( i=0;i<NTAPS;i++ ) { temp = temp + p_dsp_state->FIR_coeffs_H[i]*p_dsp_state->accel_mem[j][i]; }
		p_sensor_state->gyro[j] = temp;
	}
} /* End FIR_Filter */




