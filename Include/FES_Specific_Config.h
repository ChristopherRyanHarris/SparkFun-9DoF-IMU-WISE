
/*******************************************************************
** FILE:
**    FES_Specific_Config.h
** DESCRIPTION:
**    This header was built specific for the FES test.
********************************************************************/
#ifndef FES_SPECIFIC_CONFIG_H
#define FES_SPECIFIC_CONFIG_H


/* Choose a method by which we will toggle the relays */
#define RELAY_SWITCH_METHOD 1


#if RELAY_SWITCH_METHOD==1
  /* Method 1: On from [angle_start->angle_end]
  ** Phase angle offset is used to ensure the phase_start > phase_end .... TO DO: add mode to ensure 0:1*/
  #define PHASE_ANGLE_SWITCH_ON_VAL (0.0)
  #define PHASE_ANGLE_SWITCH_OFF_VAL (0.1)
  #define PHASE_ANGLE_OFFSET (0)
  #define PHASE_ANGLE_SWITCH_ON_EVENT ( (fmod((g_gapa_state.nu+PHASE_ANGLE_OFFSET),1)>PHASE_ANGLE_SWITCH_ON_VAL) && (fmod((g_gapa_state.nu+PHASE_ANGLE_OFFSET),1)<PHASE_ANGLE_SWITCH_ON_VAL) )

#elif RELAY_SWITCH_METHOD==2
  /* Method 2: On at angle_start and hold for N iterations*/
  #define PHASE_ANGLE_SWITCH_ON_VAL (0.0)
  #define PHASE_ANGLE_OFFSET (0)
  #define SWITCH_HOLD_ITERATIONS (1)
  #define PHASE_ANGLE_SWITCH_ON_EVENT ( ((fmod((g_gapa_state.nu_prev+PHASE_ANGLE_OFFSET),1)<PHASE_ANGLE_SWITCH_ON_VAL)&&(fmod((g_gapa_state.nu+PHASE_ANGLE_OFFSET),1)>PHASE_ANGLE_SWITCH_ON_VAL))||(g_fes_test.relay_on_iterations<SWITCH_HOLD_ITERATIONS) )

#elif RELAY_SWITCH_METHOD==3
  /* Method 3: On at angle_start and hold for N micros */
  #define PHASE_ANGLE_SWITCH_ON_VAL (0.0)
  #define PHASE_ANGLE_OFFSET (0)
  #define SWITCH_HOLD_MICROS (1)
  #define PHASE_ANGLE_SWITCH_ON_EVENT ( ((fmod((g_gapa_state.nu_prev+PHASE_ANGLE_OFFSET),1)<PHASE_ANGLE_SWITCH_ON_VAL)&&(fmod((g_gapa_state.nu+PHASE_ANGLE_OFFSET),1)>PHASE_ANGLE_SWITCH_ON_VAL))||(g_fes_test.relay_on_micros<SWITCH_HOLD_MICROS) )

#elif RELAY_SWITCH_METHOD==4
  /* Method 4: On at foot sensors 1 and 2 raw values above threshold. Hold for N iterations */
  #define FOOT_SENSOR_1_THRESHOLD_VAL (0.0)
  #define FOOT_SENSOR_2_THRESHOLD_VAL (0.0)
  #define SWITCH_HOLD_ITERATIONS (1)
  #define PHASE_ANGLE_SWITCH_ON_EVENT ( ((g_fes_state.foot_sensor_1_val>FOOT_SENSOR_1_THRESHOLD_VAL)&&(g_fes_state.foot_sensor_2_val>FOOT_SENSOR_2_THRESHOLD_VAL))||(g_fes_test.relay_on_iterations<SWITCH_HOLD_ITERATIONS) )

#elif RELAY_SWITCH_METHOD==5
  /* Method 4: On at foot sensors 1 and 2 raw values above threshold. Hold for N micros */
  #define FOOT_SENSOR_1_THRESHOLD_VAL (0.0)
  #define FOOT_SENSOR_2_THRESHOLD_VAL (0.0)
  #define SWITCH_HOLD_MICROS (1)
  #define PHASE_ANGLE_SWITCH_ON_EVENT ( ((g_fes_state.foot_sensor_1_val>FOOT_SENSOR_1_THRESHOLD_VAL)&&(g_fes_state.foot_sensor_2_val>FOOT_SENSOR_2_THRESHOLD_VAL))||(g_fes_test.relay_on_micros<SWITCH_HOLD_MICROS) )

#elif RELAY_SWITCH_METHOD==6
  /* Method 4: On at foot sensors 1 and 2 ave values above threshold. Hold for N iterations */
  #define FOOT_SENSOR_1_THRESHOLD_VAL (0.0)
  #define FOOT_SENSOR_2_THRESHOLD_VAL (0.0)
  #define SWITCH_HOLD_ITERATIONS (1)
  #define PHASE_ANGLE_SWITCH_ON_EVENT ( ((g_fes_state.foot_sensor_1_val_ave>FOOT_SENSOR_1_THRESHOLD_VAL)&&(g_fes_state.foot_sensor_2_val_ave>FOOT_SENSOR_2_THRESHOLD_VAL))||(g_fes_test.relay_on_iterations<SWITCH_HOLD_ITERATIONS) )

#elif RELAY_SWITCH_METHOD==7
  /* Method 4: On at foot sensors 1 and 2 ave values above threshold. Hold for N micros */
  #define FOOT_SENSOR_1_THRESHOLD_VAL (0.0)
  #define FOOT_SENSOR_2_THRESHOLD_VAL (0.0)
  #define SWITCH_HOLD_MICROS (1)
  #define PHASE_ANGLE_SWITCH_ON_EVENT ( ((g_fes_state.foot_sensor_1_val_ave>FOOT_SENSOR_1_THRESHOLD_VAL)&&(g_fes_state.foot_sensor_2_val_ave>FOOT_SENSOR_2_THRESHOLD_VAL))||(g_fes_test.relay_on_micros<SWITCH_HOLD_MICROS) )
  
#endif


/*******************************************************************
** Typedefs
********************************************************************/

typedef struct
{
  /* Relay variables */
  bool relays_on;
  
  unsigned long int relay_start_micros;
  unsigned long int relay_on_micros;
  
  unsigned long int relay_start_iteration;
  unsigned long int relay_on_iterations;
  
  /* Foot sensor variables */
  unsigned int foot_sensor_1_val;
  unsigned int foot_sensor_2_val;
  float foot_sensor_1_val_ave;
  float foot_sensor_2_val_ave;
  
  float foot_sensor_1_volts;
  float foot_sensor_2_volts;
  float foot_sensor_1_volts_ave;
  float foot_sensor_2_volts_ave;
  
} FES_TEST_TYPE;



/*******************************************************************
** Relays
********************************************************************/

#define RELAY_1_PIN 8 /* digital pin */
#define RELAY_2_PIN 9 /* digital pin */ 

/* Read Current Relay State 
** Returns High or low 
** Relays are normally open, so a low 
** state would set the relays open */
#define READ_RELAY_1_STATE digitalRead(RELAY_1_PIN)
#define READ_RELAY_2_STATE digitalRead(RELAY_2_PIN)

/* Set Relay State
** No Return
** Relays are normally open, so 
** setting state low state would 
** set the relays open, and High would be closed */
#define RELAY_1_SET_HIGH digitalWrite( RELAY_1_PIN, HIGH )
#define RELAY_1_SET_LOW  digitalWrite( RELAY_1_PIN, LOW )
#define RELAY_2_SET_HIGH digitalWrite( RELAY_2_PIN, HIGH )
#define RELAY_2_SET_LOW  digitalWrite( RELAY_2_PIN, LOW )



/*******************************************************************
** Foot sensor
********************************************************************/

/* Analog Foot sensor in GPIO pin */
#define FOOT_SENSOR_1_PIN A1
#define FOOT_SENSOR_2_PIN A2

/* Analog Read val to decimal */
#define ANALOG_MAX_VAL (1023)

/* Supplied Voltage */
#define FOOT_SENSOR_VOLTAGE_IN (3.3f)

/* Side A voltage splitting resistor */
#define FOOT_SENSOR_1_R (1000000.0f) //Ohms
/* Side B voltage splitting resistor */
#define FOOT_SENSOR_2_R (1000000.0f) //Ohms


/* Read Foot Sensor raw value  [0-ANALOG_MAX_VAL] 
** Returns : int */
#define FOOT_SENSOR_1_VAL (analogRead(FOOT_SENSOR_1_PIN))
#define FOOT_SENSOR_2_VAL (analogRead(FOOT_SENSOR_2_PIN))

/* Read Foot Sensor as voltage [0-FOOT_SENSRO_VOLTAGE_IN]
** Returns : double */
#define FOOT_SENSOR_1_VOLTS (((double)FOOT_SENSOR_1_VAL/(double)ANALOG_MAX_VAL)*(double)FOOT_SENSOR_VOLTAGE_IN)
#define FOOT_SENSOR_2_VOLTS (((double)FOOT_SENSOR_2_VAL/(double)ANALOG_MAX_VAL)*(double)FOOT_SENSOR_VOLTAGE_IN)

/* Read Foot Sensor as resistance [0-inf] 
** Returns : double */
#define FOOT_SENSOR_1_R ((((double)FOOT_SENSOR_VOLTAGE_IN/(double)FOOT_SENSOR_1_VOLTS)-1)*(double)FOOT_SENSOR_1_R)
#define FOOT_SENSOR_2_R ((((double)FOOT_SENSOR_VOLTAGE_IN/(double)FOOT_SENSOR_2_VOLTS)-1)*(double)FOOT_SENSOR_2_R)


/* For Moving Averages, define alpha */
#define FOOT_SENSOR_1_VAL_ALPHA (0.4f)
#define FOOT_SENSOR_2_VAL_ALPHA (0.4f)
#define FOOT_SENSOR_1_VOLTS_ALPHA (0.4f)
#define FOOT_SENSOR_2_VOLTS_ALPHA (0.4f)

#endif /* End FES_SPECIFIC_CONFIG_H */


