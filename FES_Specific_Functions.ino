
/*******************************************************************
** FILE:
**    FES_Specific_Functions
** DESCRIPTION:
**    Relay specific functions / Foot pressure sensor Functions
**    This file was built specific for the FES test.
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
** FUNCTION: FES_Init
** VARIABLES:
**    [I ]  CONTROL_TYPE      *p_control
**    [IO]  FES_TEST_TYPE     *p_fes_state
** RETURN:
**    NONE
** DESCRIPTION:
**    This function initializes the FES specific variables
*/
void FES_Init ( CONTROL_TYPE *p_control, FES_TEST_TYPE *p_fes_state )
{  
  /* Relays are Normally-Open. 
  ** Initialize low to set relay closed */
  
  SET_RELAY_1_OUTPUT;
	RELAY_1_SET_LOW;
  //pinMode( RELAY_1_PIN, OUTPUT );
  //digitalWrite( RELAY_1_PIN,LOW );
	
  SET_RELAY_2_OUTPUT;
	RELAY_2_SET_LOW;
  //pinMode( RELAY_1_PIN, OUTPUT );
  //digitalWrite( RELAY_1_PIN,LOW );
  
  /* Foot sensros are read from a voltage split */
  pinMode( FOOT_SENSOR_1_PIN, INPUT );
  pinMode( FOOT_SENSOR_2_PIN, INPUT );
  pinMode( FOOT_SENSOR_3_PIN, INPUT );      //add 7-23-2018 Rebecca
  pinMode( FOOT_SENSOR_4_PIN, INPUT );      //add 7-23-2018 Rebecca
  SET_FOOT_SENSOR_1_OUTPUT;
  SET_FOOT_SENSOR_2_OUTPUT;
  SET_FOOT_SENSOR_3_OUTPUT;                 //add 7-23-2018 Rebecca
  SET_FOOT_SENSOR_4_OUTPUT;                 //add 7-23-2018 Rebecca
  
  p_fes_state->relays_on 						   = FALSE;
  p_fes_state->relay_start_micros      = 0;
  p_fes_state->relay_start_iteration   = 0;
  p_fes_state->foot_sensor_1_val_ave   = 0;
  p_fes_state->foot_sensor_2_val_ave   = 0;
  p_fes_state->foot_sensor_3_val_ave   = 0;   //add 7-23-2018 Rebecca
  p_fes_state->foot_sensor_4_val_ave   = 0;   //add 7-23-2018 Rebecca
  p_fes_state->foot_sensor_1_volts_ave = 0;
  p_fes_state->foot_sensor_2_volts_ave = 0;
  p_fes_state->foot_sensor_3_volts_ave = 0;   //add 7-23-2018 Rebecca
  p_fes_state->foot_sensor_4_volts_ave = 0;   //add 7-23-2018 Rebecca
}

/*****************************************************************
** FUNCTION: FES_Update
** VARIABLES:
**    [I ]  CONTROL_TYPE      *p_control
**    [IO]  FES_TEST_TYPE     *p_fes_state
** RETURN:
**    NONE
** DESCRIPTION:
**    This function updates the FES specific variables
*/
void FES_Update ( 
  CONTROL_TYPE        *p_control, 
  FES_TEST_TYPE       *p_fes_state,
  SENSOR_STATE_TYPE   *p_sensor_state,
  DCM_STATE_TYPE      *p_dcm_state,
  GAPA_STATE_TYPE     *p_gapa_state,
  WISE_STATE_TYPE     *p_wise_state )
{
  /* Read foot switch values */
  p_fes_state->foot_sensor_1_val       = FOOT_SENSOR_1_VAL;
  p_fes_state->foot_sensor_2_val       = FOOT_SENSOR_2_VAL;
  p_fes_state->foot_sensor_3_val       = FOOT_SENSOR_3_VAL;   //add 7-23-2018 Rebecca
  p_fes_state->foot_sensor_4_val       = FOOT_SENSOR_4_VAL;   //add 7-23-2018 Rebecca
  p_fes_state->foot_sensor_1_val_ave   = Windowed_Mean( p_fes_state->foot_sensor_1_val_ave, (float)p_fes_state->foot_sensor_1_val, p_control->SampleNumber, FOOT_SENSOR_1_VAL_ALPHA );
  p_fes_state->foot_sensor_2_val_ave   = Windowed_Mean( p_fes_state->foot_sensor_2_val_ave, (float)p_fes_state->foot_sensor_2_val, p_control->SampleNumber, FOOT_SENSOR_2_VAL_ALPHA );
  p_fes_state->foot_sensor_3_val_ave   = Windowed_Mean( p_fes_state->foot_sensor_3_val_ave, (float)p_fes_state->foot_sensor_3_val, p_control->SampleNumber, FOOT_SENSOR_3_VAL_ALPHA );    //add 7-23-2018 Rebecca
  p_fes_state->foot_sensor_4_val_ave   = Windowed_Mean( p_fes_state->foot_sensor_4_val_ave, (float)p_fes_state->foot_sensor_4_val, p_control->SampleNumber, FOOT_SENSOR_4_VAL_ALPHA );    //add 7-23-2018 Rebecca
  p_fes_state->foot_sensor_1_volts     = FOOT_SENSOR_1_VOLTS;
  p_fes_state->foot_sensor_2_volts     = FOOT_SENSOR_2_VOLTS;
  p_fes_state->foot_sensor_3_volts     = FOOT_SENSOR_3_VOLTS;    //add 7-23-2018 Rebecca
  p_fes_state->foot_sensor_4_volts     = FOOT_SENSOR_4_VOLTS;    //add 7-23-2018 Rebecca
  p_fes_state->foot_sensor_1_volts_ave = Windowed_Mean( p_fes_state->foot_sensor_1_volts_ave, p_fes_state->foot_sensor_1_volts, p_control->SampleNumber, FOOT_SENSOR_1_VOLTS_ALPHA );
  p_fes_state->foot_sensor_2_volts_ave = Windowed_Mean( p_fes_state->foot_sensor_2_volts_ave, p_fes_state->foot_sensor_2_volts, p_control->SampleNumber, FOOT_SENSOR_2_VOLTS_ALPHA );
  p_fes_state->foot_sensor_3_volts_ave = Windowed_Mean( p_fes_state->foot_sensor_3_volts_ave, p_fes_state->foot_sensor_3_volts, p_control->SampleNumber, FOOT_SENSOR_3_VOLTS_ALPHA );    //add 7-23-2018 Rebecca
  p_fes_state->foot_sensor_4_volts_ave = Windowed_Mean( p_fes_state->foot_sensor_4_volts_ave, p_fes_state->foot_sensor_4_volts, p_control->SampleNumber, FOOT_SENSOR_4_VOLTS_ALPHA );    //add 7-23-2018 Rebecca
  
  /* At given event, trigger relay */
	if( PHASE_ANGLE_SWITCH_ON_EVENT )
	{
		/* Relay : On (High), update state variables */
		if( p_fes_state->relays_on==FALSE )
		{
		  LOG_INFO( "SWITCH RELAY 1 ON");
			/* Update current relay state */
			p_fes_state->relays_on = TRUE;
			
			/* Set both relays on */
			RELAY_1_SET_HIGH;
			RELAY_2_SET_LOW;
			
			/* Record relay on start time variable */
			p_fes_state->relay_start_micros = micros();
			/* Record relay on start iteration variable */
			p_fes_state->relay_start_iteration = p_control->SampleNumber;
		}
		/* Update time up counter */
		p_fes_state->relay_on_micros     = p_fes_state->relay_start_micros - micros();
		/* Update iterations up counter */
		p_fes_state->relay_on_iterations = p_fes_state->relay_start_iteration - p_control->SampleNumber;
	}
  else if( PHASE_ANGLE_SWITCH_ON_EVENT_DF )
  {
   /* Relay : On (High), update state variables */
     if( p_fes_state->relays_on==FALSE )
     {
       LOG_INFO( "SWITCH RELAY 2 ON");
       /* Update current relay state */
       p_fes_state->relays_on = TRUE;
       
       /* Set both relays on */
       RELAY_1_SET_LOW;
       RELAY_2_SET_HIGH;
       
       /* Record relay on start time variable */
       p_fes_state->relay_start_micros = micros();
       /* Record relay on start iteration variable */
       p_fes_state->relay_start_iteration = p_control->SampleNumber;
     }
     /* Update time up counter */
     p_fes_state->relay_on_micros     = p_fes_state->relay_start_micros - micros();
     /* Update iterations up counter */
     p_fes_state->relay_on_iterations = p_fes_state->relay_start_iteration - p_control->SampleNumber;
  }
	else
	{
		/* Relay : Off (Low), reset state variables */
		if( p_fes_state->relays_on==TRUE )
		{
		  LOG_INFO( "SWITCH RELAY OFF");
			/* Update current relay state */
			p_fes_state->relays_on = FALSE;
			
			/* Set both relays off */
			RELAY_1_SET_LOW;
			RELAY_2_SET_LOW;
			
			/* NOTE:
			** By maxing the counter, we ensure we only 
			** trigger the relay on at the desired moment 
			** See the header file for more info */
			
			/* Reset recorded start time variable */
			p_fes_state->relay_start_micros    = -1;
			/* Reset recorded start iteration variable */
			p_fes_state->relay_start_iteration = -1;
			
			/* Reset time up counter */
			p_fes_state->relay_on_micros       = -1;
			/* Reset iterations up counter */
			p_fes_state->relay_on_iterations   = -1;
		}
	}
} /* End FES_Update() */
