
/*******************************************************************
** FILE:
**   	WISE_Functions
** DESCRIPTION:
** 		This file contains all functions which are
** 		specific to the speed and walking incline
** 		estimation capabilities
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
** FUNCTION: WISE_Init
** VARIABLES:
**		[I ]	CONTROL_TYPE			*p_control
**		[I ]	SENSOR_STATE_TYPE	*p_sensor_state
**		[IO]	WISE_STATE_TYPE		*p_wise_state
** RETURN:
**		NONE
** DESCRIPTION:
** 		This function initializes the WISE state
** 		variables.
*/
void WISE_Init ( CONTROL_TYPE				*p_control,
								 SENSOR_STATE_TYPE	*p_sensor_state,
								 WISE_STATE_TYPE		*p_wise_state )
{
  int i;

  LOG_PRINTLN("> Initializing WISE");

  /*
  ** Initialize WISE control parameters
  */

	p_control->wise_prms.gain_ad 		= WISE_GAIN_AD;
	p_control->wise_prms.gain_ap 		= WISE_GAIN_AP;
	p_control->wise_prms.gain_vd 		= WISE_GAIN_VD;
	p_control->wise_prms.gain_vp	  = WISE_GAIN_VP;
	p_control->wise_prms.correction = WISE_CORRECTION;
	p_control->wise_prms.mini_count = WISE_MINCOUNT;


	/*
	** Initialize WISE state
	*/

  p_wise_state->swing_state = FALSE; /* Bool */
  p_wise_state->toe_off     = FALSE; /* Bool */
  p_wise_state->minCount    = WISE_MINCOUNT;

  p_wise_state->Nsamples = 1.0f;
  p_wise_state->Ncycles  = 0.0f;

  p_wise_state->pitch_mem         = p_sensor_state->pitch;
  p_wise_state->pitch_delta       = 0.0f;
  p_wise_state->pitch_delta_total = 0.0f;

  p_wise_state->GaitStart.vel[0]       = 999;
  p_wise_state->GaitStart.vel[1]       = 999;
  p_wise_state->GaitStart.vel_total[0] = 999;
  p_wise_state->GaitStart.vel_total[1] = 999;
  p_wise_state->GaitStart.Nsamples     = 999;
  p_wise_state->GaitStart.drift[0]     = 0.0f;
  p_wise_state->GaitStart.drift[1]     = 0.0f;
  p_wise_state->GaitStart.drift[2]     = 0.0f;

  p_wise_state->GaitEnd.vel[0]       = 999;
  p_wise_state->GaitEnd.vel[1]       = 999;
  p_wise_state->GaitEnd.vel_total[0] = 999;
  p_wise_state->GaitEnd.vel_total[1] = 999;
  p_wise_state->GaitEnd.Nsamples     = 999;
  p_wise_state->GaitEnd.drift[0]     = 0.0f;
  p_wise_state->GaitEnd.drift[1]     = 0.0f;
  p_wise_state->GaitEnd.drift[2]     = 0.0f;

  p_wise_state->CrossingP.vel[0] = 999;

  for( i=0;i<3;i++ )
  {
    /* Initialize WISE Acceleration state vector */
    p_wise_state->accel[i]       =  0.0f;
    p_wise_state->accel_ave[i]   =  1.0f;
    p_wise_state->accel_total[i] =  0.0f;
    p_wise_state->accel_delta[i] =  0.0f;

    /* Initialize WISE Velocity state vector */
    p_wise_state->vel[i]         = 0.0f;
    p_wise_state->vel_ave[i]     = 1.0f;
    //p_wise_state->vel_total[i]   = 0.0f;
    p_wise_state->vel_delta[i]   =  0.0f;

    /* Initialize WISE rotation state vector */
    p_wise_state->gyr[i]       = 0.0f;
    p_wise_state->rot[i]       = 0.0f;
    p_wise_state->rot_ave[i]   = 1.0f;
    p_wise_state->rot_total[i] = 0.0f;
    p_wise_state->rot_delta[i] = 0.0f;

    p_wise_state->CrossingP.vel[0] = 0.0f;
    p_wise_state->CrossingP.vel[1] = 0.0f;

    p_wise_state->vel_delta[i] = 0.0f;
    p_wise_state->omega_vd[i]  = 0.0f;
    p_wise_state->omega_vp[i]  = 0.0f;

    p_wise_state->dist[i]      = 0.0f;
  }
  p_wise_state->Nsamples++;
} /* End WISE_Init*/




/*****************************************************************
** FUNCTION: WISE_Update
** VARIABLES:
**		[I ]	CONTROL_TYPE			*p_control
**		[I ]	SENSOR_STATE_TYPE	*p_sensor_state
**		[IO]	WISE_STATE_TYPE		*p_wise_state
** RETURN:
**		NONE
** DESCRIPTION:
** 		This code executes the speed and
** 		walking incline estimation state update
*/
void WISE_Update ( CONTROL_TYPE				*p_control,
								   SENSOR_STATE_TYPE	*p_sensor_state,
									 WISE_STATE_TYPE		*p_wise_state )
{
  p_wise_state->Nsamples++;
  p_wise_state->pitch_delta = p_sensor_state->pitch - p_wise_state->pitch_mem;
  p_wise_state->pitch_delta_total += p_wise_state->pitch_delta;

  p_wise_state->swing_state = 1;

  /* Map acceleration to normal/tangent */
  Map_Accel_2D( p_control, p_sensor_state, p_wise_state );

  /* Integrate accel to get vel */
  Integrate_Accel_2D( p_control, p_sensor_state, p_wise_state );

  /* Velocity Adjustment */
  Adjust_Velocity( p_control, p_sensor_state, p_wise_state );

  /* Get distance traveled and compute incline */
  Adjust_Incline( p_control, p_sensor_state, p_wise_state );

  /* Reset at toeoff */
  if( p_wise_state->toe_off==TRUE ) { WISE_Reset( p_control, p_wise_state ); }

  p_wise_state->pitch_mem = p_sensor_state->pitch;
} /* End WISE_Update */




/*****************************************************************
** FUNCTION: WISE_Reset
** VARIABLES:
**		[I ]	CONTROL_TYPE			*p_control
**		[IO]	WISE_STATE_TYPE		*p_wise_state
** RETURN:
**		NONE
** DESCRIPTION:
** 		This function resets the WISE
** 		state variables. In particular,
** 		the integrated variables.
*/
void WISE_Reset ( CONTROL_TYPE			*p_control,
									WISE_STATE_TYPE		*p_wise_state )
{
  int i;

  p_wise_state->toe_off  = FALSE;
  p_wise_state->Nsamples = 1.0f;

  for( i=0; i<=2; i++)
  {
    /* Initialize WISE Acceleration state vector */
    p_wise_state->accel[i]       =  0.0f;
    p_wise_state->accel_ave[i]   =  1.0f;
    p_wise_state->accel_total[i] =  0.0f;
    p_wise_state->accel_delta[i] =  0.0f;

    /* Initialize WISE Velocity state vector */
    p_wise_state->vel[i]       = 0.0f;
    p_wise_state->vel_total[i] = 0.0f;
    p_wise_state->vel_delta[i] = 0.0f;
    p_wise_state->omega_vd[i]  = 0.0f;
    p_wise_state->omega_vp[i]  = 0.0f;

    /* Initialize WISE Rotation state vector */
    p_wise_state->gyr[i]       = 0.0f;
    p_wise_state->rot[i]       = 0.0f;
    p_wise_state->rot_ave[i]   = 0.0f;
    p_wise_state->rot_total[i] = 0.0f;
    p_wise_state->rot_delta[i] = 0.0f;

    p_wise_state->dist[i]      = 0.0f;
  }
} /* End WISE_Reset */




/*****************************************************************
** FUNCTION: Map_Accel_2D
** VARIABLES:
**		[I ]	CONTROL_TYPE			*p_control
**		[IO]	WISE_STATE_TYPE		*p_wise_state
** RETURN:
**		NONE
** DESCRIPTION:
** 		This function maps a_t and a_n to a_x and a_y
** 		using the filtered pitch assuming 2D motion.
** 		This Does not account for roll.
** 		NOTE: There may be a better way of extracting
**       		this from the mid-filter DCM
*/
void Map_Accel_2D ( CONTROL_TYPE				*p_control,
								    SENSOR_STATE_TYPE		*p_sensor_state,
									  WISE_STATE_TYPE			*p_wise_state )
{

  /*
  ** Notes on orientation for the 10736 IMU
  **   Terms:
  **     Fore:       (Front) Edge oposite of the power port
  **     Aft:        (Rear) Edge of the power port
  **     Starboard:  (Right) Edge with reset switch
  **     Port:       (Left) Edge oposite of reset switch
  **     Zenith:     (Up) Clean face of board
  **     Nadir:      (Down) Populated face of board
  **   Contrary to the silk, the axis are positioned as follows:
  **     +x is Fore,       -x is Aft
  **     +y is Starboard,  -y is Port
  **     +z is Zenith,     -z is Nadir
  */

	/*
	** Notes on orientation for the 9250 IMU
	**   Terms:
	**     Fore:       (Front) Edge of the USB port
	**     Aft:        (Rear) Edge oposite of the USB port
	**     Starboard:  (Right) Edge oposite of PWR switch
	**     Port:       (Left) Edge with PWR switch
	**     Zenith:     (Up) face with USB port
	**     Nadir:      (Down) face oposite USB port
	**   Contrary to the silk, the axis are positioned as follows:
	**     +x is Fore,       -x is Aft
	**     +y is Starboard,  -y is Port
	**     +z is Zenith,     -z is Nadir
	**   This means, placing the board on a flat surface with the
	**   face without the USB port (Nadir) down will result in an acceleration
	**   of about -2000 (1xg) for accel[2] (z) since the acceleration
	**   from gravity with be acting along -z.
	*/

  /* Accel x:Fore y:Port z:Zenith
  ** Note: IMU coordinate ref. frame definced in IMU#_Config.h
  **       Rotation will need to be accounted for
  */
  float Ax, Az, R;

  switch( PITCH_O )
  {
  	case 1: /* P: 0:Nadir0/Zenith down  +90:Aft down   -90:Fore down */
		  Ax = p_sensor_state->accel[0]; /* Movement: +x */
		  Az = p_sensor_state->accel[2]; /* Gravity:  -z */
		  R  = p_sensor_state->gyro[1];  /* Rotation: -y */
		  break;
		case 2: /* P: 0:Fore/Aft down       +90:Port down  -90:Starboard down */
		  Ax = p_sensor_state->accel[1]; /* Movement: +y (stbd) */
		  Az = p_sensor_state->accel[0]; /* Gravity:  -x (fwd) */
		  R  = p_sensor_state->gyro[2];  /* Rotation: -z */
			break;
		case 3: /* P: 0:Fore/Aft down       +90:Nadir down -90:Zenith down */
		  Ax = p_sensor_state->accel[2]; /* Movement: +x */
		  Az = p_sensor_state->accel[0]; /* Gravity:  -z */
		  R  = p_sensor_state->gyro[1];  /* Rotation: -y */
			break;
	}
  p_wise_state->accel_delta[0] = Ax;
  p_wise_state->accel_delta[1] = Az;
  p_wise_state->gyr[0]         = R;

  /**********************************
  ** Tangent Part *******************
  **********************************/

  /* Calc wrt world coordinate system */
  p_wise_state->accel[0]   = (Ax*cos(p_sensor_state->pitch) - Az*sin(p_sensor_state->pitch)) * GTOMPS2/GRAVITY * MPSTOMPH * WISE_CORRECTION;

  /* Feedback */
  p_wise_state->accel_delta[0] = p_wise_state->accel[0] - p_wise_state->accel_delta[0];
  p_wise_state->omega_ad[0]    = p_wise_state->accel_delta[0]*WISE_GAIN_AD;
  p_wise_state->omega_ap[0]    = p_wise_state->accel[0]*WISE_GAIN_AP;

  /* Get average */
  p_wise_state->accel_total[0] += p_wise_state->accel[0];
  p_wise_state->accel_ave[0]    = p_wise_state->accel_total[0]/p_wise_state->Nsamples;

  /*********************************
  ** Normal Part *******************
  **********************************/

  /* Calc Ay wrt world coordinate system */
  p_wise_state->accel[1]  = -(Ax*sin(p_sensor_state->pitch) - Az*cos(p_sensor_state->pitch)  - GRAVITY) * GTOMPS2/GRAVITY * MPSTOMPH;// * (1/WISE_CORRECTION);

  /* Feedback */
  p_wise_state->accel_delta[1] = p_wise_state->accel[1] - p_wise_state->accel_delta[1];
  p_wise_state->omega_ad[1]    = p_wise_state->accel_delta[1]*WISE_GAIN_AD;
  p_wise_state->omega_ap[1]    = p_wise_state->accel[1]*WISE_GAIN_AP;

  /* Get average */
  p_wise_state->accel_total[1] += p_wise_state->accel[1];
  p_wise_state->accel_ave[1]    = p_wise_state->accel_total[1]/p_wise_state->Nsamples;

} /* End Map_Accel_2D */




/*****************************************************************
** FUNCTION: Integrate_Accel_2D
** VARIABLES:
**		[I ]	CONTROL_TYPE			*p_control
**		[I ]	SENSOR_STATE_TYPE	*p_sensor_state
**		[IO]	WISE_STATE_TYPE		*p_wise_state
** RETURN:
**		NONE
** DESCRIPTION:
**		Integrate acceleration (wrt leg ref coordinates)
** 		to get velocity (wrt leg ref coordinates)
** 		Assumes 2D motion
*/
void Integrate_Accel_2D ( CONTROL_TYPE				*p_control,
								   				SENSOR_STATE_TYPE		*p_sensor_state,
									 				WISE_STATE_TYPE			*p_wise_state )
{
  int i;
  for( i=0; i<=2; i++)
  {
    p_wise_state->vel_delta[i] = p_wise_state->vel[i];

    //p_wise_state->vel[i]  = p_wise_state->vel[i] + p_wise_state->accel_ave[i]*p_control->G_Dt;
    p_wise_state->vel[i]  = p_wise_state->vel[i] + p_wise_state->accel[i]*p_control->G_Dt;

    //p_wise_state->vel[i] -= (p_wise_state->omega_vd[i] + p_wise_state->omega_vp[i]);
    //p_wise_state->vel_delta[i] = p_wise_state->vel[i] - p_wise_state->vel_delta[i];
    //p_wise_state->omega_vd[i]  = p_wise_state->vel_delta[i]*WISE_GAIN_VD;
    //p_wise_state->omega_vp[i]  = p_wise_state->vel[i]*WISE_GAIN_VP;
    if( (p_wise_state->Ncycles>=1) )
    {
	    //p_wise_state->omega_vp[i]  = (p_wise_state->GaitStart.drift[i]*0.5*p_wise_state->Nsamples*p_wise_state->Nsamples)*WISE_GAIN_VP;
	    p_wise_state->omega_vp[i]  = (p_wise_state->GaitStart.drift[i])*WISE_GAIN_VP;
	    //p_wise_state->vel[i]      -= (p_wise_state->omega_vp[i]);
		}

    p_wise_state->vel_total[i] += p_wise_state->vel[i];
    //p_wise_state->vel_ave[i]  = p_wise_state->vel_total[i]/p_wise_state->Nsample;
  }


  /*********************************
  ** Rotational Part ***************
  **********************************/
  p_wise_state->rot[0] = p_wise_state->rot[i] + p_wise_state->gyr[i]*p_control->G_Dt;


} /* End Integrate_Accel_2D */




/*****************************************************************
** FUNCTION: Adjust_Velocity
** VARIABLES:
**		[I ]	CONTROL_TYPE			*p_control
**		[I ]	SENSOR_STATE_TYPE	*p_sensor_state
**		[IO]	WISE_STATE_TYPE		*p_wise_state
** RETURN:
**		NONE
** DESCRIPTION:
** 		This function adjusts for velocity drift.
** 		We detect toe-off events, from there we
** 		can determine an estimated drift over the previous
** 		gait cycle. Then we can adjust the average velocity.
*/
void Adjust_Velocity( CONTROL_TYPE				*p_control,
								   		SENSOR_STATE_TYPE		*p_sensor_state,
									 		WISE_STATE_TYPE			*p_wise_state )
{
  float NGaitSamples = 0.0f;

  /* Part 0: Set gait start for first cycle */
  if( (p_wise_state->Ncycles==0) & (p_wise_state->Nsamples==1) )
  {
    p_wise_state->GaitStart.vel[0]       = p_wise_state->vel[0];
    p_wise_state->GaitStart.vel[1]       = p_wise_state->vel[1];
    p_wise_state->GaitStart.vel_total[0] = p_wise_state->vel_total[0];
    p_wise_state->GaitStart.vel_total[1] = p_wise_state->vel_total[1];
    p_wise_state->GaitStart.Time         = p_control->timestamp;
    p_wise_state->GaitStart.Nsamples     = p_wise_state->Nsamples;

    p_wise_state->CrossingP.vel[0]       = p_wise_state->rot[0];
    p_wise_state->CrossingP.vel[1]       = p_wise_state->rot[0];
  }


  /* In Testing */
  if( p_wise_state->rot[0]>p_wise_state->CrossingP.vel[0] )
	{
		/* Record rotational maximum */
		p_wise_state->CrossingP.vel[0] = p_wise_state->rot[0];

		/* Record velocity minimum
		** GaitStart[0] = { vel tangent at minima }
		** GaitStart[1] = { vel normal at minima  } */
		p_wise_state->GaitEnd.Time         = p_control->timestamp;
		p_wise_state->GaitEnd.vel[0]       = p_wise_state->vel[0];
		p_wise_state->GaitEnd.vel[1]       = p_wise_state->vel[1];
		p_wise_state->GaitEnd.vel_total[0] = p_wise_state->vel_total[0];
		p_wise_state->GaitEnd.vel_total[1] = p_wise_state->vel_total[1];
		p_wise_state->GaitEnd.Nsamples     = p_wise_state->Nsamples;
	}

  /* Part I : At toe-off, we must correct velocity measured
  ** We must wait a few full gait cycles to get
  ** a good slope estimate.
  ** We must be within, at a minimum, the second full
  ** gait cycle.
  ** GaitStart[0] is initialized to 999
  ** GaitStart[0] is reset within the first full gait cycle */
  if ( (p_wise_state->Nsamples-p_wise_state->GaitEnd.Nsamples)>(p_wise_state->minCount) )
  {
  	//fprintf(stdout,"DEBUG - Toe off! S:%f vel[0]:%f vel[1]:%f\n",p_wise_state->GaitEnd.Nsamples,p_wise_state->GaitEnd.vel[0],p_wise_state->GaitEnd.vel[1]);

    /* We must be within, at a minimum, the second
    ** gait cycle. This garuntees we are calculating the
    ** slope from toe-off to toe-off
    ** GaitStart[1] is initialized to -1
    ** GaitStart[1] is reset within the first full gait cycle */
    if( p_wise_state->Ncycles>=1 )
    {
    	/* Get number of samples in this gait */
    	NGaitSamples = p_wise_state->GaitEnd.Nsamples - p_wise_state->GaitStart.Nsamples - 1;

      /* We correct for the drift by approximating
      ** the slope in velocity over the gait cycle.
      ** We can then subtract the accumulated drift
      ** over the cycle and the dc bias.
      ** GaitStart = {vel0_i, vel1_i}
      ** This assumes we reset N at each toeoff */
      p_wise_state->GaitEnd.drift[0] = ( p_wise_state->GaitEnd.vel[0]-p_wise_state->GaitStart.vel[0] )/NGaitSamples;
      p_wise_state->vel_ave[0]   = ( (p_wise_state->GaitStart.vel_total[0]) - (p_wise_state->GaitEnd.drift[0]*0.5*NGaitSamples*NGaitSamples) - (p_wise_state->GaitStart.vel[0]*NGaitSamples) ) * (1/NGaitSamples);

      p_wise_state->GaitEnd.drift[1] = ( p_wise_state->GaitEnd.vel[1]-p_wise_state->GaitStart.vel[1] )/NGaitSamples;
      p_wise_state->vel_ave[1]   = ( (p_wise_state->GaitStart.vel_total[1]) - (p_wise_state->GaitEnd.drift[1]*0.5*NGaitSamples*NGaitSamples) - (p_wise_state->GaitStart.vel[1]*NGaitSamples) ) * (1/NGaitSamples);
    }

    /* Reset saved minima and increment cycle counter */
    p_wise_state->Ncycles++;

    memcpy( &(p_wise_state->GaitStart), &(p_wise_state->GaitEnd), sizeof(WISE_GATE_TYPE) );
    p_wise_state->GaitStart.Nsamples   = 1.0f;

    p_wise_state->GaitEnd.vel[0]       = (999);
    p_wise_state->GaitEnd.vel[1]       = (999);
    p_wise_state->GaitEnd.vel_total[0] = (999);
    p_wise_state->GaitEnd.vel_total[1] = (999);
    p_wise_state->GaitEnd.Nsamples     = (999);

    p_wise_state->CrossingP.vel[0] = p_wise_state->rot[0];

    /* Reset gait parameters
    ** for next cycle */
    p_wise_state->toe_off = TRUE; /* Signal reset */
    //WISE_Reset( p_control, p_wise_state );
  }

//  /* Part II : Record minima
//  ** Here we are attempting to locate the local minima
//  ** If this velocit is lower than the previous minima ... */
//  if ( p_wise_state->vel[0] < p_wise_state->GaitEnd.vel[0] )
//  {
//    /* If this sample is more than the minimal required */
//    if ( (p_wise_state->Nsamples-p_wise_state->GaitEnd.Nsamples ) > (p_wise_state->minCount) )
//    { } /* This shouldn't happen */
//    else
//    {
//      /* GaitStart[0] = { vel tangent at minima }
//      ** GaitStart[1] = { vel normal at minima  } */
//    	p_wise_state->GaitEnd.Time         = p_control->timestamp;
//      p_wise_state->GaitEnd.vel[0]       = p_wise_state->vel[0];
//      p_wise_state->GaitEnd.vel[1]       = p_wise_state->vel[1];
//      p_wise_state->GaitEnd.vel_total[0] = p_wise_state->vel_total[0];
//      p_wise_state->GaitEnd.vel_total[1] = p_wise_state->vel_total[1];
//      p_wise_state->GaitEnd.Nsamples     = p_wise_state->Nsamples;
//    }
//  }

//  /* Part III : Update the min count threshold
//  ** Once we have warmed up, we can try to
//  ** estimate how far apart each gait is */
//  if ( (p_wise_state->Ncycles>3) & (p_wise_state->Nsamples==1) )
//  {
//    p_wise_state->minCount = ceil( ((NGaitSamples*0.4)+p_wise_state->minCount)*0.5 );
//  }

} /* End Adjust_Velocity */


/*****************************************************************
** FUNCTION: Adjust_Incline
** VARIABLES:
**		[I ]	CONTROL_TYPE			*p_control
**		[I ]	SENSOR_STATE_TYPE	*p_sensor_state
**		[IO]	WISE_STATE_TYPE		*p_wise_state
** RETURN:
**		NONE
** DESCRIPTION:
** 		From our latest velocity estimate (from Adjust_Velocity)``
** 		we can determine our distance traveled along each dimension.
** 		Distance can be computed as dist += vel*dt
** 		Incline can be computed as (dy/dx)*100
*/
void Adjust_Incline( CONTROL_TYPE				*p_control,
								   	 SENSOR_STATE_TYPE	*p_sensor_state,
									 	 WISE_STATE_TYPE		*p_wise_state )
{
	float tempi;
	float tempx=0,tempy=0;

	/* Compute distance traveled in each direction */
	p_wise_state->dist[0] += p_wise_state->vel[0]*(p_control->G_Dt);
	p_wise_state->dist[1] += p_wise_state->vel[1]*(p_control->G_Dt);


	/* Compute incline estimate */
	tempi = (p_wise_state->dist[1]/p_wise_state->dist[0])*100;
	p_wise_state->Incline_ave = Rolling_Mean( p_wise_state->Nsamples, p_wise_state->Incline_ave, tempi );

	/* Compute an average incline estimate using the final velocity estimate */
	if( (p_wise_state->Ncycles>3) )
	{
		tempx = p_wise_state->vel_ave[0]*(p_control->timestamp-p_wise_state->GaitStart.Time)/TIME_RESOLUTION;
		tempx = p_wise_state->vel_ave[1]*(p_control->timestamp-p_wise_state->GaitStart.Time)/TIME_RESOLUTION;
		p_wise_state->Incline_gait = (tempy/tempx)*100;
	}

} /* End Get_WISE */







/*****************************************************************
** Function: Estimate_Error
** FUNCTION: Estimate_Error
** VARIABLES:
**		[I ]	CONTROL_TYPE			*p_control
**		[I ]	SENSOR_STATE_TYPE	*p_sensor_state
**		[IO]	WISE_STATE_TYPE		*p_wise_state
** RETURN:
**		NONE
** DESCRIPTION:
** 		This function is inteded to estimate the
** 		error in the intitial velocity estimates.
** 		There are several ways we can estimate the
** 		error in the velocity:
**  		1. We assume there is a relationship between the
**     		 velocity error and the pitch delta. I.e. at
**     		 small theta_dot, there is a high probability
**     		 of error in the estimate
**  		2. We can take the value of the velocity feedback
**     		 intergation term as an approximation of the error.
**  		3. We can take the difference between the final velocity and
**     		 the average velocity. This assumes that the
**     		 velocity should be a constant. Therefore, a large
**     		 difference may indicate that the velocity has error. */
void Estimate_Error( 	CONTROL_TYPE				*p_control,
								   		SENSOR_STATE_TYPE		*p_sensor_state,
									 		WISE_STATE_TYPE			*p_wise_state )
{
//  int i;
//  float pave,pe1,pe2,pe3;
//  float m1_pho, m1_ave_pitch_delta;
//  float m2_ave_feedback;
//  float m3_ave_vel, m3_delta;
//
//  /* Method 1
//  ** Estimate error from relationship to pitch delta
//  ** m1_pho: Strength of relationship */
//  m1_pho = 0.01f;
//  //m1_ave_pitch_delta = abs(p_wise_state->pitch_delta_total/p_wise_state->Nsample);
//  m1_ave_pitch_delta = fabs(p_wise_state->pitch_delta);
//  //p_wise_state->pe[0] = m1_ave_pitch_delta;
//  p_wise_state->pe[0] = exp(-m1_ave_pitch_delta/m1_pho);
//
//  /* Method 2:
//  ** Estimate error based on velocity feedback values */
//  m2_ave_feedback = 0.0f;
//  for( i=0;i<1;i++ ) { m2_ave_feedback += (p_wise_state->omega_vd[i]/p_wise_state->vel[i]); }
//  //for( i=0;i<2;i++ ) { m2_ave_feedback += abs(p_wise_state->omega_vd[i]/p_wise_state->vel[i]); }
//  p_wise_state->pe[1] = m2_ave_feedback;
//  //p_wise_state->pe[1] = min( abs(m2_ave_feedback/2), 1);
//
//  /* Method 3:
//  ** Estimate error based on the difference between
//  ** the average velocity and the final velocity */
//  m3_delta = 0.0f;
//  for( i=0;i<2;i++ )
//  {
//    m3_ave_vel = (p_wise_state->vel_total[i]/p_wise_state->Nsample);
//    m3_delta += fabs((m3_ave_vel - p_wise_state->vel[i])/m3_ave_vel);
//  }
//  //p_wise_state->pe[2] = m3_delta;
//  p_wise_state->pe[2] = fmin(m3_delta/2,1);
//
//  /* Get final error estimate
//  ** Final estimate can be the avereage p */
//  p_wise_state->pave = (p_wise_state->pe[0] + p_wise_state->pe[1] + p_wise_state->pe[2])/3;
//
//  /*
//  if ( millis() > (p_control->g_LastBlinkTime + UART_BLINK_RATE) )
//  {
//    imuLog = "\t\t err est (p1/p2/p3/pave): ";
//    imuLog += String( pe1,7 ) + "/" + String( pe2,7 ) + "/" + String( pe3,7 ) + "/" +String( pave,7 );
//    imuLog += "\r\n\n";
//    LOG_PRINT( imuLog );
//  } */
} /* End Estimate_Error */

