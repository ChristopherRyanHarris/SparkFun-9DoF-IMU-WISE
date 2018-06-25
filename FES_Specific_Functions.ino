
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
** RETURN:
**    NONE
** DESCRIPTION:
**    This function initializes the FES specific variables
*/
void FES_Init ( CONTROL_TYPE *p_control )
{  
  /* Relays are Normally-Open. 
  ** Initialize low to set relay closed */
  
  pinMode( RELAY_1_PIN, OUTPUT );
  digitalWrite( RELAY_1_PIN,LOW );
  
  pinMode( RELAY_1_PIN, OUTPUT );
  digitalWrite( RELAY_1_PIN,LOW );
  
  /* Foot sensros are read from a voltage split */
  pinMode( FOOT_SENSOR_1_PIN, INPUT );
  pinMode( FOOT_SENSOR_2_PIN, INPUT );
}