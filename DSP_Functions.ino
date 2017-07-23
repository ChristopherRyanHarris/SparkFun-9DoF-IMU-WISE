/*************************************************
** FILE: DSP_Functions
** This file contains the functions specific to
** our low pass filter functions.
** We are supplying support for several flavors,
** but in the general case, we will be applying a
** simple filters on each of the inputs.
** A short FIR LP filter (4 taps) and a IIR filter on
** each of the input accelerations
** A short FIR HP filter (4 taps) on the gyro
**************************************************/


/*******************************************************************
** Includes ********************************************************
********************************************************************/


#include "./Include/Common_Config.h"

#if EXE_MODE==1 /* Emulator Mode */
#include <string.h>

#ifdef _IMU10736_
#include "./Include/IMU10736_Config.h"
#endif
#ifdef _IMU9250_
#include <SparkFunMPU9250-DMP.h>
#include "./Include/IMU9250_Config.h"
#endif

#include "./Include/DSP_Config.h"
#include "./Include/WISE_Config.h"
#include "./Include/Emulator_Config.h"
extern CAL_STATE_TYPE      g_calibration;
extern DCM_STATE_TYPE      g_dcm_state;
extern DSP_COMMON_TYPE     g_dsp;
extern SENSOR_STATE_TYPE   g_sensor_state;
extern CONTROL_STATE_TYPE  g_control_state;
extern WISE_STATE_TYPE     g_wise_state;
#endif  /* End Emulator Mode */


/*******************************************************************
** Functions *******************************************************
********************************************************************/





/*************************************************
** Function: DSP_Filter_Init
** This function initializes the state memory 
*/
void DSP_Filter_Init ( void )
{
	int i,j;

  float IIR_coeffs_La[NTAPS] = IIR_LPF_a;
	float IIR_coeffs_Lb[NTAPS] = IIR_LPF_b;
	float IIR_coeffs_Ha[NTAPS] = IIR_HPF_a;
	float IIR_coeffs_Hb[NTAPS] = IIR_HPF_b;
	float FIR_coeffs_L[NTAPS]  = FIR_LPF;
	float FIR_coeffs_H[NTAPS]  = FIR_HPF;

	memcpy(&g_dsp.IIR_coeffs_La[0],&IIR_coeffs_La[0],NTAPS*sizeof(float));
	memcpy(&g_dsp.IIR_coeffs_Lb[0],&IIR_coeffs_Lb[0],NTAPS*sizeof(float));
	memcpy(&g_dsp.IIR_coeffs_Ha[0],&IIR_coeffs_Ha[0],NTAPS*sizeof(float));
	memcpy(&g_dsp.IIR_coeffs_Hb[0],&IIR_coeffs_Hb[0],NTAPS*sizeof(float));
	memcpy(&g_dsp.FIR_coeffs_L[0],&FIR_coeffs_L[0],NTAPS*sizeof(float));
	memcpy(&g_dsp.FIR_coeffs_H[0],&FIR_coeffs_H[0],NTAPS*sizeof(float));

	for( i=0;i<3;i++ )
	{
		for( j=0;j<NTAPS;j++ )
		{
			g_dsp.accel_mem[i][j] = 0.0f;
			g_dsp.gyro_mem[i][j] = 0.0f;
		}
	}
} /* End DSP_Filter_Init */


/*************************************************
** Function: DSP_Update
** This function updates the necessary
** DSP state variabls. This includes
** the memory arrays 
*/
void DSP_Update ( void )
{
	int i;
	/* log new inputs */
	for( i=0;i<3;i++ )
	{
		g_dsp.accel_mem[i][0] = g_sensor_state.accel[i];
		g_dsp.gyro_mem[i][0] = g_sensor_state.gyro[i];
	}
} /* DSP_Update */


/*************************************************
** Function: DSP_Shift
** This function shifts the memory
** arrays in preparation for the next
** iteration 
*/
void DSP_Shift ( void )
{
	int i,j;
	/* Shift sesor values */
	for( i=0;i<3;i++)
	{
		for( j=0;j<(NTAPS-1);j++)
		{
			g_dsp.accel_mem[i][j+1] = g_dsp.accel_mem[i][j];
			g_dsp.gyro_mem[i][j+1] = g_dsp.gyro_mem[i][j];
		}
	}
} /* End DSP_Shift */


/*************************************************
** Function IIR_Filter
** This function applies a IIR filter
** to the input sensor data.
** Equation:
**	y[n] = (1/a[0]) * ( b[0]*x[n] + ... + b[P]*x[n-P] - a[1]*y[n-1] - ... - a[Q]*y[n-Q] ) 
*/
void IIR_Filter ( void )
{
	/* TO DO: Add functionality for additional modes */
	int i,j;
	float temp;

	DSP_Update();

	/* Accel - LPF */
	for( j=0;j<3;j++ )
	{
		temp = 0.0f;
		for( i=0;i<NTAPS;i++ ) { temp = temp + g_dsp.IIR_coeffs_Lb[i]*g_dsp.accel_mem[j][i]; }
		for( i=1;i<NTAPS;i++ ) { temp = temp - g_dsp.IIR_coeffs_La[i]*g_dsp.accel_mem[j][i]; }
		g_sensor_state.accel[j] = (1/g_dsp.IIR_coeffs_La[0])*temp;
	}
	/* Gyro - HPF */
	for( j=0;j<3;j++ )
	{
		temp = 0.0f;
		for( i=0;i<NTAPS;i++ ) { temp = temp + g_dsp.IIR_coeffs_Hb[i]*g_dsp.accel_mem[j][i]; }
		for( i=1;i<NTAPS;i++ ) { temp = temp - g_dsp.IIR_coeffs_Ha[i]*g_dsp.accel_mem[j][i]; }
		g_sensor_state.gyro[j] = (1/g_dsp.IIR_coeffs_Ha[0])*temp;
	}
} /* End IIR_Filter */


/*************************************************
** Function FIR_Filter
** This function applies a FIR filter
** to the input sensor data.
** Equation:
**	y[n] = b[0]*x[n] + b[1]*x[n-1] + ... + b[N]*x[n-N] 
*/
void FIR_Filter ( void )
{
	/* TO DO: Add functionality for additional modes */
	int i,j;
	float temp;

	DSP_Update();

	/* Accel - LPF */
	for( j=0;j<3;j++ )
	{
		temp = 0.0f;
		for( i=0;i<NTAPS;i++ ) { temp = temp + g_dsp.FIR_coeffs_L[i]*g_dsp.accel_mem[j][i]; }
		g_sensor_state.accel[j] = temp;
	}
	/* Gyro - HPF */
	for( j=0;j<3;j++ )
	{
		temp = 0.0f;
		for( i=0;i<NTAPS;i++ ) { temp = temp + g_dsp.FIR_coeffs_H[i]*g_dsp.accel_mem[j][i]; }
		g_sensor_state.gyro[j] = temp;
	}
} /* End FIR_Filter */






