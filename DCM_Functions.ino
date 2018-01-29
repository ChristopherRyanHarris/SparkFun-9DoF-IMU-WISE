
/*******************************************************************
** FILE:
**   	DCM_Functions
** DESCRIPTION:
**		The Directional Cosine Matrix filtering algorithm.
** 		This file contains the DCM filter functions
** 		These functions take the accel/magn/gyro data
** 		and (via the DCM filter) output filtered
** 		Euler angles.
********************************************************************/


/*******************************************************************
** Includes ********************************************************
********************************************************************/

#ifndef COMMON_CONFIG_H
	#include "../Include/Common_Config.h"
#endif
#if EXE_MODE==1 /* Emulator Mode */
	/* In emulation mode, "Emulator_Protos" is needed to
	** use functions in other files.
	** NOTE: This header should contain the function
	** 			 prototypes for all execution functions */
	#include "../Include/Emulator_Protos.h"
#endif  /* End Emulator Mode */

/*******************************************************************
** Functions *******************************************************
********************************************************************/


/*************************************************
** FUNCTION: DCM_Init
** VARIABLES:
**		[IO]	CONTROL_TYPE			*p_control
**		[IO]	DCM_STATE_TYPE		*p_dcm_state
**		[IO]	SENSOR_STATE_TYPE	*p_sensor_state
** RETURN:
**		NONE
** DESCRIPTION:
** 		Initialize DCM variables
*/
void DCM_Init( CONTROL_TYPE				*p_control,
							 DCM_STATE_TYPE 		*p_dcm_state,
							 SENSOR_STATE_TYPE	*p_sensor_state )
{
  int i;

  LOG_PRINTLN("> Initializing DCM");

	/*
	** Initialize DCM control parameters
	*/

	p_control->dcm_prms.Kp_RollPitch 			= Kp_ROLLPITCH;
	p_control->dcm_prms.Ki_RollPitch 			= Ki_ROLLPITCH;
	p_control->dcm_prms.Kp_Yaw       			= Kp_YAW;
	p_control->dcm_prms.Ki_Yaw       			= Ki_YAW;
	p_control->dcm_prms.PitchOrientation  = PITCH_O;
	p_control->dcm_prms.PitchRotationConv = PITCH_ROT_CONV;
	p_control->dcm_prms.RollOrientation   = ROLL_O;
	p_control->dcm_prms.RollRotationConv  = ROLL_ROT_CONV;
	p_control->dcm_prms.RollRotationRef   = ROLL_ZREF;

	/*
	** Initialize DCM state parameters
	*/

  for(i=0;i<3;i++) p_dcm_state->Omega_I[i] = 0.0f;
  for(i=0;i<3;i++) p_dcm_state->Omega_P[i] = 0.0f;
  p_dcm_state->SampleNumber=0;

  Reset_Sensor_Fusion( p_control, p_dcm_state, p_sensor_state );

  for(i=0;i<3;i++) p_sensor_state->gyro_ave[i] = p_sensor_state->gyro[i];
  for(i=0;i<3;i++) p_sensor_state->gyro_var[i] = 0.0f;
  for(i=0;i<3;i++) p_sensor_state->gyro_std[i] = 0.0f;
  p_sensor_state->std_time=0;
} /* End DCM_Init */


/*************************************************
** Reset_Sensor_Fusion
** FUNCTION: Reset_Sensor_Fusion
** VARIABLES:
**		[I ]	CONTROL_TYPE			*p_control
**		[IO]	DCM_STATE_TYPE		*p_dcm_state
**		[IO]	SENSOR_STATE_TYPE	*p_sensor_state
** RETURN:
**		NONE
** DESCRIPTION:
** 		Read every sensor and record a time stamp.
** 		Init DCM with unfiltered orientation
** 		I.e. set initial roll/pitch from initial guess
*       	 and initialize the DCM arrays.
*/
void Reset_Sensor_Fusion( CONTROL_TYPE			*p_control,
													DCM_STATE_TYPE		*p_dcm_state,
													SENSOR_STATE_TYPE	*p_sensor_state )
{
	/* Initialize roll/pitch/yaw from accelerations */
  Set_Sensor_Fusion( p_control, p_sensor_state );

  /* Init rotation matrix */
  Init_Rotation_Matrix( p_control, p_dcm_state, p_sensor_state );
} /* End Reset_Sensor_Fusion */


/*************************************************
** FUNCTION: Set_Sensor_Fusion
** VARIABLES:
**		[I ]	CONTROL_TYPE			*p_control
**		[IO]	SENSOR_STATE_TYPE	*p_sensor_state
** RETURN:
**		NONE
** DESCRIPTION:
** 		Similar to Reset_Sensor_Fusion, except we do not
** 		re-initialize the DCM arrays. This is used to
** 		reset the roll/pitch values without touching the
** 		DCM matrix state.
*/
void Set_Sensor_Fusion( CONTROL_TYPE			*p_control,
												SENSOR_STATE_TYPE	*p_sensor_state )
{
  float temp1[3];
  float temp2[3];
  float xAxis[] = {1.0f, 0.0f, 0.0f};

  /* GET PITCH
  ** Using y-z-plane-component/x-component of gravity vector */
  p_sensor_state->pitch = -1 * f_atan2( p_sensor_state->accel[0], sqrt(p_sensor_state->accel[1]*p_sensor_state->accel[1] + p_sensor_state->accel[2]*p_sensor_state->accel[2]) );

  /* GET ROLL
  ** Compensate pitch of gravity vector */
  Vector_Cross_Product( p_sensor_state->accel, xAxis, temp1 );
  Vector_Cross_Product( xAxis, temp1, temp2 );

  /* Normally using x-z-plane-component/y-component of compensated gravity vector
  ** 	roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
  ** Since we compensated for pitch, x-z-plane-component equals z-component: */
  p_sensor_state->roll = f_atan2(temp2[1], temp2[2]);

  /* GET YAW */
  p_sensor_state->yaw = 0;
} /* End Set_Sensor_Fusion */


/*************************************************
** FUNCTION: Init_Rotation_Matrix
** VARIABLES:
**		[I ]	CONTROL_TYPE			*p_control
**		[IO]	DCM_STATE_TYPE		*p_dcm_state
**		[I ]	SENSOR_STATE_TYPE	*p_sensor_state
** RETURN:
**		NONE
** DESCRIPTION:
** 		Initialize the DCM rotation matrix using
** 		Euler angles
*/
void Init_Rotation_Matrix( CONTROL_TYPE				*p_control,
													 DCM_STATE_TYPE			*p_dcm_state,
													 SENSOR_STATE_TYPE	*p_sensor_state )
{
	float	*m		= &(p_dcm_state->DCM_Matrix[0][0]);
	float roll 	= p_sensor_state->roll;
	float pitch = p_sensor_state->pitch;
	float yaw 	= p_sensor_state->yaw;

  float c1 = cos(roll);
  float s1 = sin(roll);
  float c2 = cos(pitch);
  float s2 = sin(pitch);
  float c3 = cos(yaw);
  float s3 = sin(yaw);

  /* Euler angles, right-handed, intrinsic, XYZ convention
  ** (which means: rotate around body axes Z, Y', X'')  */
  m[0*3 + 0] = c2 * c3;
  m[0*3 + 1] = c3 * s1 * s2 - c1 * s3;
  m[0*3 + 2] = s1 * s3 + c1 * c3 * s2;

  m[1*3 + 0] = c2 * s3;
  m[1*3 + 1] = c1 * c3 + s1 * s2 * s3;
  m[1*3 + 2] = c1 * s2 * s3 - c3 * s1;

  m[2*3 + 0] = -s2;
  m[2*3 + 1] = c2 * s1;
  m[2*3 + 2] = c1 * c2;
} /* End Init_Rotation_Matrix */


/******************************************************************
** FUNCTION: DCM_Filter
** VARIABLES:
**		[I ]	CONTROL_TYPE			*p_control
**		[IO]	DCM_STATE_TYPE		*p_dcm_state
**		[IO]	SENSOR_STATE_TYPE	*p_sensor_state
** RETURN:
**		NONE
** DESCRIPTION:
** There are 4 parts to the DCM filter
**   1. Matrix_Update    - Update the DCM
**   2. Normalize        - Normalize the DCM
**   3. Drift_Correction - Correct for drift in orientation
**   4. Get_Euler_Angles - Extract Euler angles from DCM
*/
void DCM_Filter( CONTROL_TYPE				*p_control,
								 DCM_STATE_TYPE			*p_dcm_state,
								 SENSOR_STATE_TYPE	*p_sensor_state )
{
  int i;

  float temp;
  float error = 0;
  float renorm = 0;

  float TempM[3][3];
  float TempM2[3][3];

  float Accel_Vector[3];
  float Accel_magnitude;
  float Accel_weight;

  float Omega_Vector[3];
  float ErrorGain[3];
  float errorRollPitch[3];
  float errorYaw[3];

  /* Clear Rolling Std/Average after set time
  ** NOTE: This should be moved elsewhere */
  if( p_control->WISE_on==1 )
  {
    p_sensor_state->std_time+=(p_control->G_Dt*TIME_RESOLUTION);
  	if( p_sensor_state->std_time>MOVE_RESET_RATE )
 		{
	  	for(i=0;i<3;i++) p_sensor_state->gyro_ave[i] = p_sensor_state->gyro[i];
	  	for(i=0;i<3;i++) p_sensor_state->gyro_var[i] = 0.0f;
	  	for(i=0;i<3;i++) p_sensor_state->gyro_std[i] = 0.0f;
	  	p_sensor_state->std_time=0;
	  	p_dcm_state->SampleNumber=0;
 		}

  	/* Update Rolling Std */
  	p_dcm_state->SampleNumber++;
  	for( i=0;i<3;i++)
		{
  		temp = Rolling_Mean( p_dcm_state->SampleNumber, p_sensor_state->gyro_ave[i], p_sensor_state->gyro[i] );
  		p_sensor_state->gyro_var[i] = Rolling_Variance( p_sensor_state->gyro_ave[i], temp, p_sensor_state->gyro[i], p_sensor_state->gyro_var[i] );
  		p_sensor_state->gyro_ave[i] = temp;
  		p_sensor_state->gyro_std[i] = p_sensor_state->gyro_var[i]/p_dcm_state->SampleNumber;
  	}
  }


  /******************************************************************
  ** DCM 1. Update the Direction Cosine Matrix
  ** We set the DCM matrix for this iteration.
  ** We update the states assuming the IMU is
  ** traveling along the direction described by the
  ** previous iterations DCM orientation.
  ** Apply the feedback gains from the last iteration
  ** in order to account for any drift.
  ******************************************************************/

  /* Convert the acceleration values
  ** Note: Values read from sensor are fixed point */
  Accel_Vector[0] = ( p_sensor_state->accel[0] );
  Accel_Vector[1] = ( p_sensor_state->accel[1] );
  Accel_Vector[2] = ( p_sensor_state->accel[2] );

  /* Apply prop and int gain to rotation
  ** Need to convert the Gyro values to radians
  **    Note: Values read from sensor are fixed point */
  Omega_Vector[0] = GYRO_X_SCALED( p_sensor_state->gyro[0] ) + p_dcm_state->Omega_I[0] + p_dcm_state->Omega_P[0];
  Omega_Vector[1] = GYRO_Y_SCALED( p_sensor_state->gyro[1] ) + p_dcm_state->Omega_I[1] + p_dcm_state->Omega_P[1];
  Omega_Vector[2] = GYRO_Z_SCALED( p_sensor_state->gyro[2] ) + p_dcm_state->Omega_I[2] + p_dcm_state->Omega_P[2];

  /* Update the state matrix
  ** We are essentially applying a rotation
  ** from the new gyro data. This is an estimate
  ** of the current orientation */
  TempM2[0][0] = p_control->G_Dt * (p_dcm_state->DCM_Matrix[0][1]*Omega_Vector[2] - p_dcm_state->DCM_Matrix[0][2]*Omega_Vector[1]) + p_dcm_state->DCM_Matrix[0][0];
  TempM2[0][1] = p_control->G_Dt * (p_dcm_state->DCM_Matrix[0][2]*Omega_Vector[0] - p_dcm_state->DCM_Matrix[0][0]*Omega_Vector[2]) + p_dcm_state->DCM_Matrix[0][1];
  TempM2[0][2] = p_control->G_Dt * (p_dcm_state->DCM_Matrix[0][0]*Omega_Vector[1] - p_dcm_state->DCM_Matrix[0][1]*Omega_Vector[0]) + p_dcm_state->DCM_Matrix[0][2];
  TempM2[1][0] = p_control->G_Dt * (p_dcm_state->DCM_Matrix[1][1]*Omega_Vector[2] - p_dcm_state->DCM_Matrix[1][2]*Omega_Vector[1]) + p_dcm_state->DCM_Matrix[1][0];
  TempM2[1][1] = p_control->G_Dt * (p_dcm_state->DCM_Matrix[1][2]*Omega_Vector[0] - p_dcm_state->DCM_Matrix[1][0]*Omega_Vector[2]) + p_dcm_state->DCM_Matrix[1][1];
  TempM2[1][2] = p_control->G_Dt * (p_dcm_state->DCM_Matrix[1][0]*Omega_Vector[1] - p_dcm_state->DCM_Matrix[1][1]*Omega_Vector[0]) + p_dcm_state->DCM_Matrix[1][2];

  /******************************************************************
  ** DCM 2. Normalize DCM
  ** We must normalize the DCM matrix
  ** After each update in order to keep
  ** each vector in the DCM orthogonal
  ******************************************************************/

  /* Determine vector overlap
  ** Each vector should be orthogonal */
  error = -Vector_Dot_Product(&TempM2[0][0],&TempM2[1][0]) * 0.5;

  /* temp = V .* e */
  Vector_Scale( &TempM2[1][0], error, &TempM[0][0] );
  Vector_Scale( &TempM2[0][0], error, &TempM[1][0] );

  /* temp = temp .* DCM[0][:] */
  Vector_Add( &TempM[0][0], &TempM2[0][0], &TempM[0][0] );
  Vector_Add( &TempM[1][0], &TempM2[1][0], &TempM[1][0] );

  /* Force orthogonality */
  Vector_Cross_Product( &TempM[0][0], &TempM[1][0], &TempM[2][0] );

  /* Normalize each vector
  ** DCM[i][:] = temp ./ ( 0.5*(3 - sum(temp.^2)) )
  ** Note that the sum of the DCM vectors should have length of 1 */

  renorm = .5 *(3 - Vector_Dot_Product(&TempM[0][0],&TempM[0][0]));
  Vector_Scale( &TempM[0][0], renorm, &p_dcm_state->DCM_Matrix[0][0] );

  renorm = .5 *(3 - Vector_Dot_Product(&TempM[1][0],&TempM[1][0]));
  Vector_Scale( &TempM[1][0], renorm, &p_dcm_state->DCM_Matrix[1][0] );

  renorm = .5 *(3 - Vector_Dot_Product(&TempM[2][0],&TempM[2][0]));
  Vector_Scale( &TempM[2][0], renorm, &p_dcm_state->DCM_Matrix[2][0] );



  /******************************************************************
  ** DCM 3. Drift_Correction Accel_magnitude
  ** Drift correction basically looks at the difference in
  ** the orientation described by the DCM matrix and the
  ** orientation described by the current acceleration vector.
  ** Essentially, the acceleration is the input and the DCM
  ** matrix is the current state.
  ** NOTE: We are applying a drift correction by adjusting the
  **       proportional and integral feedback. So, this will not
  **       have an effect until the next iteration!
  ******************************************************************/

  /* Roll and Pitch
  ** Calculate the magnitude of the accelerometer vector
  ** Scale to gravity */
  Accel_magnitude = sqrt( Vector_Dot_Product( &Accel_Vector[0], &Accel_Vector[0] ) ) / p_control->sensor_prms.gravity; //GRAVITY;

  /* Dynamic weighting of accelerometer info (reliability filter)
  ** Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0) */
  Accel_weight = FCONSTRAIN( 1.0-2.0*FABS(1-Accel_magnitude), 0.0, 1.0 ) ;

  /* Adjust the ground of reference
  ** errorRP = accel x DCM[2][:]
  ** The error is essentially the amount that the DCM
  ** vector, the vector which describes our current orientation
  ** in space, and the current acceleration vector differ. The accel
  ** vector is naturally very noisy, but it is our input for each cycle
  ** and serves as our state estimate. Therefore, we scale the error
  ** by a integral and proportional gain in each cycle */
  Vector_Cross_Product( &Accel_Vector[0], &p_dcm_state->DCM_Matrix[2][0], &errorRollPitch[0] );

  Vector_Scale( &errorRollPitch[0], Kp_ROLLPITCH*Accel_weight, &p_dcm_state->Omega_P[0] );
  Vector_Scale( &errorRollPitch[0], Ki_ROLLPITCH*Accel_weight, &ErrorGain[0] );
  Vector_Add( p_dcm_state->Omega_I, ErrorGain, p_dcm_state->Omega_I );

  /* Note:
  ** Roll and pitch have been lumped here, to simplify the math
  ** since the orientation is only described by a 3-dim point.
  ** We could split the roll/pitch gains (since the roll is
  ** described by DCM[2][1-2] and pitch is described by DCM[2,0])
  ** however, we would have to be clever about how we apply the
  ** acceleration vector */

  /* YAW
  ** We make the gyro YAW drift correction based on compass magnetic heading
  ** The yaw is an estimate, and will drift towards some equilibrium as time
  ** progresses. However, presuming the drift is not constant towards any
  ** particular direction, this should be ok */

  Vector_Scale( &p_dcm_state->DCM_Matrix[2][0], p_dcm_state->DCM_Matrix[0][0], errorYaw ); /* Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position. */

  /* Update the proportional and integral gains per yaw error */
  Vector_Scale( &errorYaw[0], p_control->dcm_prms.Kp_Yaw, &ErrorGain[0] ); /* proportional of YAW. */
  //Vector_Add( p_dcm_state->Omega_P, ErrorGain, p_dcm_state->Omega_P ); /* Adding  Proportional. */

  Vector_Scale( &errorYaw[0], p_control->dcm_prms.Ki_Yaw, &ErrorGain[0] ); /* Adding Integrator */
  //Vector_Add( p_dcm_state->Omega_I, ErrorGain, p_dcm_state->Omega_I ); /* Adding integrator to the Omega_I */

  /******************************************************************
  ** DCM 4. Extract Euler Angles from DCM
  ** Calculate the Euler angles from the orientation
  ** described by the DCM matrix.
  ** DCM[2][:] is essentially a vector describing the
  ** orientation of the IMU in space.
  ******************************************************************/

  /* Pitch Conventions (set in config):
  ** Range: -90:90
  ** With PITCH_ROT_CONV==1 :
  ** PITCH_O:1 - Pitch orientation #1. Angle x-axis w/ Horizontal Plane  +Rot:Aft-Down    0:Nadir0/Zenith down. +90:Aft down   -90:Fore down
  ** PITCH_O:2 - Pitch orientation #2. Angle y-axis w/ Horizontal Plane  +Rot:Port-Down   0:Fore/Aft down       +90:Port down  -90:Starboard down
  ** PITCH_O:3 - Pitch orientation #3. Angle z-axis w/ Horizontal Plane  +Rot:Nadir-Down  0:Fore/Aft down       +90:Nadir down -90:Zenith down */
  switch ( p_control->dcm_prms.PitchOrientation )
  {
    case 1 :
      //p_sensor_state->pitch = -PITCH_ROT_CONV*f_asin( p_dcm_state->DCM_Matrix[2][0] );
      p_sensor_state->pitch = -p_control->dcm_prms.PitchRotationConv*f_asin( p_dcm_state->DCM_Matrix[2][0] );
      break;
    case 2 :
      //p_sensor_state->pitch = -PITCH_ROT_CONV*f_asin( p_dcm_state->DCM_Matrix[2][1] );
      p_sensor_state->pitch = -p_control->dcm_prms.PitchRotationConv*f_asin( p_dcm_state->DCM_Matrix[2][1] );
      break;
    case 3 :
      //p_sensor_state->pitch = -PITCH_ROT_CONV*f_asin( p_dcm_state->DCM_Matrix[2][2] );
      p_sensor_state->pitch = -p_control->dcm_prms.PitchRotationConv*f_asin( p_dcm_state->DCM_Matrix[2][2] );
      break;
  }

  /* Define roll orientation convention (set in config):
  ** We should only be using 3,4,5 orientations, but they are all available for hacking.
  ** Range: -180:180
  ** With ROLL_ROT_CONV==1  ROLL_ZREF==1
  ** ROLL_O:1 - Roll orientation #1. Rotation around z-axis (Nadir-Zenith) +Rot:Aft-Port    0:Port down     +-180:stbd down
  ** ROLL_O:2 - Roll orientation #2. Rotation around y-axis (Port-Stbd)    +Rot:Aft-Nadir   0:Nadir down    +-180:Zenith down
  ** ROLL_O:3 - Roll orientation #3. Rotation around x-axis (Fore-Aft)     +Rot:Port-Nadir  0:Nadir down    +-180:Zenith down
  ** ROLL_O:4 - Roll orientation #4. Rotation around z-axis (Nadir-Zenith) +Rot:Aft-Port    0:Aft down      +-180:Fore down
  ** ROLL_O:5 - Roll orientation #5. Rotation around y-axis (Port-Stbd)    +Rot:Aft-Nadir   0:Aft down      +-180:Fore down
  ** ROLL_O:6 - Roll orientation #6. Rotation around x-axis (Fore-Aft)     +Rot:Port-Nadir  0:Port down     +-180:Stbd down
  ** WARNING: ROLL should be determined from pitch orientation, not set manually */
  switch ( p_control->dcm_prms.RollOrientation )
  {
    case 1 :
      p_sensor_state->roll = -p_control->dcm_prms.RollRotationConv*f_atan2( p_dcm_state->DCM_Matrix[2][0], -p_control->dcm_prms.RollRotationRef*p_dcm_state->DCM_Matrix[2][1] );
      break;
    case 2 :
      p_sensor_state->roll = -p_control->dcm_prms.RollRotationConv*f_atan2( p_dcm_state->DCM_Matrix[2][0], -p_control->dcm_prms.RollRotationRef*p_dcm_state->DCM_Matrix[2][2] );
      break;
    case 3 :
      p_sensor_state->roll = -p_control->dcm_prms.RollRotationConv*f_atan2( p_dcm_state->DCM_Matrix[2][1], -p_control->dcm_prms.RollRotationRef*p_dcm_state->DCM_Matrix[2][2] );
      break;
    case 4 :
      p_sensor_state->roll =  p_control->dcm_prms.RollRotationConv*f_atan2( p_dcm_state->DCM_Matrix[2][1], -p_control->dcm_prms.RollRotationRef*p_dcm_state->DCM_Matrix[2][0] );
      break;
    case 5 :
      p_sensor_state->roll =  p_control->dcm_prms.RollRotationConv*f_atan2( p_dcm_state->DCM_Matrix[2][2], -p_control->dcm_prms.RollRotationRef*p_dcm_state->DCM_Matrix[2][0] );
      break;
    case 6 :
      p_sensor_state->roll =  p_control->dcm_prms.RollRotationConv*f_atan2( p_dcm_state->DCM_Matrix[2][2], -p_control->dcm_prms.RollRotationRef*p_dcm_state->DCM_Matrix[2][1] );
      break;
  }

  p_sensor_state->yaw   =  f_atan2( p_dcm_state->DCM_Matrix[1][0], p_dcm_state->DCM_Matrix[0][0] ); // A faster atan2
} /* End DCM_Filter */










