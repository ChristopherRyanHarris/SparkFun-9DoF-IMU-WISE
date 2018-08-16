
/*******************************************************************
** FILE:
**    FES_Specific_Config.h
** DESCRIPTION:
**    This header was built specific for the FES test.
********************************************************************/
#ifndef FES_SPECIFIC_CONFIG_H
#define FES_SPECIFIC_CONFIG_H


/* Choose a method by which we will toggle the relays */
/* Rebecca - FES study uses method 1 and method 6 */
#define RELAY_SWITCH_METHOD 6


/* Define thresholds for IMU based relay control methods */
#define PHASE_ANGLE_SWITCH_ON_VAL (0.58)
#define PHASE_ANGLE_SWITCH_OFF_VAL (0.69)
#define PHASE_ANGLE_SWITCH_ON_VAL_DF (0.7)
#define PHASE_ANGLE_SWITCH_OFF_VAL_DF (1.0)

/* Define thresholds for foot sensor based relay control methods */
#define FOOT_SENSOR_1_THRESHOLD_VOLTS (2.2f)
#define FOOT_SENSOR_2_THRESHOLD_VOLTS (2.2f)
#define FOOT_SENSOR_3_THRESHOLD_VOLTS (2.2f)
#define FOOT_SENSOR_4_THRESHOLD_VOLTS (2.2f)


#if RELAY_SWITCH_METHOD==1
  /* Method 1: On from [angle_start->angle_end]
  ** Phase angle offset is used to ensure the phase_start > phase_end .... TO DO: add mode to ensure 0:1*/
  #define PHASE_ANGLE_OFFSET (0)
  #define NO_STIM (( (fmod((p_gapa_state->nu_norm.val[1]+PHASE_ANGLE_OFFSET),1)>PHASE_ANGLE_SWITCH_OFF_VAL_DF) && (fmod((p_gapa_state->nu_norm.val[0]+PHASE_ANGLE_OFFSET),1)<PHASE_ANGLE_SWITCH_ON_VAL) ) || ( (fmod((p_gapa_state->nu_norm.val[1]+PHASE_ANGLE_OFFSET),1)>PHASE_ANGLE_SWITCH_OFF_VAL) && (fmod((p_gapa_state->nu_norm.val[0]+PHASE_ANGLE_OFFSET),1)<PHASE_ANGLE_SWITCH_ON_VAL_DF) ))
  #define PHASE_ANGLE_SWITCH_ON_EVENT ( (fmod((p_gapa_state->nu_norm.val[1]+PHASE_ANGLE_OFFSET),1)<PHASE_ANGLE_SWITCH_OFF_VAL) && (fmod((p_gapa_state->nu_norm.val[0]+PHASE_ANGLE_OFFSET),1)>PHASE_ANGLE_SWITCH_ON_VAL) )
  #define PHASE_ANGLE_SWITCH_ON_EVENT_DF ( (fmod((p_gapa_state->nu_norm.val[1]+PHASE_ANGLE_OFFSET),1)<PHASE_ANGLE_SWITCH_OFF_VAL_DF) && (fmod((p_gapa_state->nu_norm.val[0]+PHASE_ANGLE_OFFSET),1)>PHASE_ANGLE_SWITCH_ON_VAL_DF) )


#elif RELAY_SWITCH_METHOD==2
  /* Method 2: On at angle_start and hold for N iterations*/
  #define PHASE_ANGLE_OFFSET (0)
  #define SWITCH_HOLD_ITERATIONS (1)
  #define PHASE_ANGLE_SWITCH_ON_EVENT ( ((fmod((p_gapa_state->nu_norm.val[1]+PHASE_ANGLE_OFFSET),1)<PHASE_ANGLE_SWITCH_OFF_VAL)&&(fmod((p_gapa_state->nu_norm.val[0]+PHASE_ANGLE_OFFSET),1)>PHASE_ANGLE_SWITCH_ON_VAL))||(p_fes_test.relay_on_iterations<SWITCH_HOLD_ITERATIONS) )
  #define PHASE_ANGLE_SWITCH_ON_EVENT_DF ( ((fmod((p_gapa_state->nu_norm.val[1]+PHASE_ANGLE_OFFSET),1)<PHASE_ANGLE_SWITCH_OFF_VAL_DF)&&(fmod((p_gapa_state->nu_norm.val[0]+PHASE_ANGLE_OFFSET),1)>PHASE_ANGLE_SWITCH_ON_VAL_DF))||(p_fes_test.relay_on_iterations<SWITCH_HOLD_ITERATIONS) )

#elif RELAY_SWITCH_METHOD==3
  /* Method 3: On at angle_start and hold for N micros */
  #define PHASE_ANGLE_OFFSET (0)
  #define SWITCH_HOLD_MICROS (1)
  #define PHASE_ANGLE_SWITCH_ON_EVENT ( ((fmod((p_gapa_state->nu_norm.val[1]+PHASE_ANGLE_OFFSET),1)<PHASE_ANGLE_SWITCH_OFF_VAL)&&(fmod((p_gapa_state->nu_norm.val[0]+PHASE_ANGLE_OFFSET),1)>PHASE_ANGLE_SWITCH_ON_VAL))||(p_fes_test.relay_on_micros<SWITCH_HOLD_MICROS) )
  #define PHASE_ANGLE_SWITCH_ON_EVENT_DF ( ((fmod((p_gapa_state->nu_norm.val[1]+PHASE_ANGLE_OFFSET),1)<PHASE_ANGLE_SWITCH_OFF_VAL_DF)&&(fmod((p_gapa_state->nu_norm.val[0]+PHASE_ANGLE_OFFSET),1)>PHASE_ANGLE_SWITCH_ON_VAL_DF))||(p_fes_test.relay_on_micros<SWITCH_HOLD_MICROS) )

#elif RELAY_SWITCH_METHOD==4
  /* Method 4: On at foot sensors 1 and 2 AVE values below threshold. Hold for N iterations */
  #define SWITCH_HOLD_ITERATIONS (1)
  #define PHASE_ANGLE_SWITCH_ON_EVENT ( ((p_fes_state->foot_sensor_1_val_ave>FOOT_SENSOR_1_THRESHOLD_VAL)&&(p_fes_state->foot_sensor_2_val_ave>FOOT_SENSOR_2_THRESHOLD_VAL))||(p_fes_test.relay_on_iterations<SWITCH_HOLD_ITERATIONS) )

#elif RELAY_SWITCH_METHOD==5
  /* Method 5: On at foot sensors 1 and 2 AVE values below threshold. Hold for N micros */
  #define SWITCH_HOLD_MICROS (1)
  #define PHASE_ANGLE_SWITCH_ON_EVENT ( ((p_fes_state->foot_sensor_1_val_ave>FOOT_SENSOR_1_THRESHOLD_VAL)&&(p_fes_state->foot_sensor_2_val_ave>FOOT_SENSOR_2_THRESHOLD_VAL))||(p_fes_test.relay_on_micros<SWITCH_HOLD_MICROS) )
  
#elif RELAY_SWITCH_METHOD==6
  /* Method 8: On at foot sensors 1 and 2 AVE volts below threshold. Hold until 1 AVE volts is above threshold*/
  /* Dorsiflexor Stim. (DF) = Hold until 1 and 2 AVE volts are both below threshold */
  /* Plantarflexor Stim. (PF) = Hold until 1 AVE volts above threshold and 2 AVE volts still below threshold */
  /* No Stim. = Both 1 and 2 AVE volts are above threshold */
  #define SWITCH_HOLD_ITERATIONS (1)
  #define NO_STIM ((p_fes_state->foot_sensor_1_volts_ave<FOOT_SENSOR_1_THRESHOLD_VOLTS)&&(p_fes_state->foot_sensor_2_volts_ave<FOOT_SENSOR_2_THRESHOLD_VOLTS))
  #define PHASE_ANGLE_SWITCH_ON_EVENT ((p_fes_state->foot_sensor_1_volts_ave>FOOT_SENSOR_1_THRESHOLD_VOLTS)&&(p_fes_state->foot_sensor_2_volts_ave<FOOT_SENSOR_2_THRESHOLD_VOLTS))
  #define PHASE_ANGLE_SWITCH_ON_EVENT_DF ((p_fes_state->foot_sensor_1_volts_ave>FOOT_SENSOR_1_THRESHOLD_VOLTS)&&(p_fes_state->foot_sensor_2_volts_ave>FOOT_SENSOR_2_THRESHOLD_VOLTS))

#elif RELAY_SWITCH_METHOD==7
  /* Method 9: On at foot sensors 1 and 2 AVE volts below threshold. Hold until 1 AVE volts is above threshold*/
  /* Dorsiflexor Stim. (DF) = Hold until 1 and 2 AVE volts are both below threshold */
  /* Plantarflexor Stim. (PF) = Hold until 1 AVE volts above threshold and 2 AVE volts still below threshold */
  /* No Stim. = Both 1 and 2 AVE volts are above threshold */
  #define SWITCH_HOLD_MICROS (1)
  #define NO_STIM ((p_fes_state->foot_sensor_1_volts_ave<FOOT_SENSOR_1_THRESHOLD_VOLTS)&&(p_fes_state->foot_sensor_2_volts_ave<FOOT_SENSOR_2_THRESHOLD_VOLTS))
  #define PHASE_ANGLE_SWITCH_ON_EVENT ((p_fes_state->foot_sensor_1_volts_ave>FOOT_SENSOR_1_THRESHOLD_VOLTS)&&(p_fes_state->foot_sensor_2_volts_ave<FOOT_SENSOR_2_THRESHOLD_VOLTS))
  #define PHASE_ANGLE_SWITCH_ON_EVENT_DF ((p_fes_state->foot_sensor_1_volts_ave>FOOT_SENSOR_1_THRESHOLD_VOLTS)&&(p_fes_state->foot_sensor_2_volts_ave>FOOT_SENSOR_2_THRESHOLD_VOLTS))

#endif


/*******************************************************************
** Typedefs
********************************************************************/

typedef struct
{
  /* Relay variables */
  bool relay1_on;
  bool relay2_on;
  
  unsigned long int relay1_start_micros;
  unsigned long int relay1_on_micros;
  unsigned long int relay2_start_micros;
  unsigned long int relay2_on_micros;
  
  unsigned long int relay1_start_iteration;
  unsigned long int relay1_on_iterations;
  unsigned long int relay2_start_iteration;
  unsigned long int relay2_on_iterations;
  
  /* Foot sensor variables */
  unsigned int foot_sensor_1_val;
  unsigned int foot_sensor_2_val;
  unsigned int foot_sensor_3_val;
  unsigned int foot_sensor_4_val;
  float foot_sensor_1_val_ave;
  float foot_sensor_2_val_ave;
  float foot_sensor_3_val_ave;
  float foot_sensor_4_val_ave;
  
  float foot_sensor_1_volts;
  float foot_sensor_2_volts;
  float foot_sensor_3_volts;
  float foot_sensor_4_volts;
  float foot_sensor_1_volts_ave;
  float foot_sensor_2_volts_ave;
  float foot_sensor_3_volts_ave;
  float foot_sensor_4_volts_ave;
  
} FES_TEST_TYPE;



/*******************************************************************
** Relays
********************************************************************/

#define RELAY_1_PIN 8 /* digital pin */
#define RELAY_2_PIN 9 /* digital pin */ 

/* Set pins as output */
#define SET_RELAY_1_OUTPUT pinMode(RELAY_1_PIN,OUTPUT)
#define SET_RELAY_2_OUTPUT pinMode(RELAY_2_PIN,OUTPUT)
/* Set pins as INPUT ... not used */
//#define SET_RELAY_1_OUTPUT pinMode(RELAY_1_PIN,INPUT)
//#define SET_RELAY_2_OUTPUT pinMode(RELAY_2_PIN,INPUT)

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
** Foot Sensor LED Monitor
********************************************************************/
/*
#define LED_H_PIN 10 // digital pin
#define LED_T_PIN 12 // digital pin
*/
/* Set LED pins to monitor Foot Switches 1 & 2*/
/*
#define SET_LED_H_OUTPUT pinMode(LED_H_PIN,OUTPUT)
#define SET_LED_T_OUTPUT pinMode(LED_T_PIN,OUTPUT)
        */
/* Set pins as INPUT ... not used */
//#define SET_LED_H_OUTPUT pinMode(LED_H_PIN,INPUT)
//#define SET_LED_T_OUTPUT pinMode(LED_T_PIN,INPUT)

/* Set LED FootSwitch Monitor State */
/*
#define LED_H_SET_HIGH digitalWrite( LED_H_PIN, HIGH )
#define LED_H_SET_LOW  digitalWrite( LED_H_PIN, LOW )
#define LED_T_SET_HIGH digitalWrite( LED_T_PIN, HIGH )
#define LED_T_SET_LOW  digitalWrite( LED_T_PIN, LOW )

#define LED_H_ON ((p_fes_state->foot_sensor_3_volts_ave>FOOT_SENSOR_3_THRESHOLD_VOLTS))
#define LED_T_ON ((p_fes_state->foot_sensor_4_volts_ave>FOOT_SENSOR_4_THRESHOLD_VOLTS))
*/

/*******************************************************************
** Foot Sensors
********************************************************************/

/* Analog Foot sensor in GPIO pin */
#define FOOT_SENSOR_1_PIN A1    //IMU side, heel
#define FOOT_SENSOR_2_PIN A2    //IMU side, toe
#define FOOT_SENSOR_3_PIN A3    //Contralateral side, heel
#define FOOT_SENSOR_4_PIN A4    //Contralateral side, toe

/* Analog Read val to decimal */
#define ANALOG_MAX_VAL (1023)

/* Supplied Voltage */
#define FOOT_SENSOR_VOLTAGE_IN (3.3f)

/* Side A voltage splitting resistor */
#define FOOT_SENSOR_1_R (100.0f) //Ohms ;  Heel A (IMU side)
#define FOOT_SENSOR_2_R (100.0f) //Ohms ;  Toe  A (IMU side)
/* Side B voltage splitting resistor */
#define FOOT_SENSOR_3_R (100.0f) //Ohms ;  Heel B
#define FOOT_SENSOR_4_R (100.0f) //Ohms ;  Toe  B


/* Set pins as INPUT ... not used */
#define SET_FOOT_SENSOR_1_OUTPUT pinMode(FOOT_SENSOR_1_PIN,INPUT)
#define SET_FOOT_SENSOR_2_OUTPUT pinMode(FOOT_SENSOR_2_PIN,INPUT)
#define SET_FOOT_SENSOR_3_OUTPUT pinMode(FOOT_SENSOR_3_PIN,INPUT)
#define SET_FOOT_SENSOR_4_OUTPUT pinMode(FOOT_SENSOR_4_PIN,INPUT)

/* Read Foot Sensor raw value  [0-ANALOG_MAX_VAL] 
** Returns : int */
#define FOOT_SENSOR_1_VAL (analogRead(FOOT_SENSOR_1_PIN))
#define FOOT_SENSOR_2_VAL (analogRead(FOOT_SENSOR_2_PIN))
#define FOOT_SENSOR_3_VAL (analogRead(FOOT_SENSOR_3_PIN))
#define FOOT_SENSOR_4_VAL (analogRead(FOOT_SENSOR_4_PIN))

/* Read Foot Sensor as voltage [0-FOOT_SENSRO_VOLTAGE_IN]
** Returns : double */
#define FOOT_SENSOR_1_VOLTS (((double)FOOT_SENSOR_1_VAL/(double)ANALOG_MAX_VAL)*(double)FOOT_SENSOR_VOLTAGE_IN)
#define FOOT_SENSOR_2_VOLTS (((double)FOOT_SENSOR_2_VAL/(double)ANALOG_MAX_VAL)*(double)FOOT_SENSOR_VOLTAGE_IN)
#define FOOT_SENSOR_3_VOLTS (((double)FOOT_SENSOR_3_VAL/(double)ANALOG_MAX_VAL)*(double)FOOT_SENSOR_VOLTAGE_IN)
#define FOOT_SENSOR_4_VOLTS (((double)FOOT_SENSOR_4_VAL/(double)ANALOG_MAX_VAL)*(double)FOOT_SENSOR_VOLTAGE_IN)

/* Read Foot Sensor as resistance [0-inf] 
** Returns : double */
#define FOOT_SENSOR_1_R ((((double)FOOT_SENSOR_VOLTAGE_IN/(double)FOOT_SENSOR_1_VOLTS)-1)*(double)FOOT_SENSOR_1_R)
#define FOOT_SENSOR_2_R ((((double)FOOT_SENSOR_VOLTAGE_IN/(double)FOOT_SENSOR_2_VOLTS)-1)*(double)FOOT_SENSOR_2_R)
#define FOOT_SENSOR_3_R ((((double)FOOT_SENSOR_VOLTAGE_IN/(double)FOOT_SENSOR_3_VOLTS)-1)*(double)FOOT_SENSOR_3_R)
#define FOOT_SENSOR_4_R ((((double)FOOT_SENSOR_VOLTAGE_IN/(double)FOOT_SENSOR_4_VOLTS)-1)*(double)FOOT_SENSOR_4_R)

/* For Moving Averages, define alpha */
#define FOOT_SENSOR_1_VAL_ALPHA (0.4f)
#define FOOT_SENSOR_2_VAL_ALPHA (0.4f)
#define FOOT_SENSOR_3_VAL_ALPHA (0.4f)
#define FOOT_SENSOR_4_VAL_ALPHA (0.4f)

#define FOOT_SENSOR_1_VOLTS_ALPHA (0.4f)
#define FOOT_SENSOR_2_VOLTS_ALPHA (0.4f)
#define FOOT_SENSOR_3_VOLTS_ALPHA (0.4f)
#define FOOT_SENSOR_4_VOLTS_ALPHA (0.4f)

#endif /* End FES_SPECIFIC_CONFIG_H */