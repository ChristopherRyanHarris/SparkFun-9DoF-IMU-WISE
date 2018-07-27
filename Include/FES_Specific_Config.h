
/*******************************************************************
** FILE:
**    FES_Specific_Config.h
** DESCRIPTION:
**    This header was built specific for the FES test.
********************************************************************/
#ifndef FES_SPECIFIC_CONFIG_H
#define FES_SPECIFIC_CONFIG_H


/* Choose a method by which we will toggle the relays */
#define RELAY_SWITCH_METHOD 9


#if RELAY_SWITCH_METHOD==1
  /* Method 1: On from [angle_start->angle_end]
  ** Phase angle offset is used to ensure the phase_start > phase_end .... TO DO: add mode to ensure 0:1*/
  #define PHASE_ANGLE_SWITCH_ON_VAL (0.5)
  #define PHASE_ANGLE_SWITCH_OFF_VAL (0.6)
  #define PHASE_ANGLE_OFFSET (0)
  #define PHASE_ANGLE_SWITCH_ON_EVENT ( (fmod((p_gapa_state->nu_norm.val[1]+PHASE_ANGLE_OFFSET),1)<PHASE_ANGLE_SWITCH_ON_VAL) && (fmod((p_gapa_state->nu_norm.val[0]+PHASE_ANGLE_OFFSET),1)>PHASE_ANGLE_SWITCH_ON_VAL) )
  #define PHASE_ANGLE_SWITCH_ON_VAL_DF (0.7)
  #define PHASE_ANGLE_SWITCH_OFF_VAL_DF (0.9)
  #define PHASE_ANGLE_SWITCH_ON_EVENT_DF ( (fmod((p_gapa_state->nu_norm.val[1]+PHASE_ANGLE_OFFSET),1)<PHASE_ANGLE_SWITCH_ON_VAL_DF) && (fmod((p_gapa_state->nu_norm.val[0]+PHASE_ANGLE_OFFSET),1)>PHASE_ANGLE_SWITCH_ON_VAL_DF) )


#elif RELAY_SWITCH_METHOD==2
  /* Method 2: On at angle_start and hold for N iterations*/
  #define PHASE_ANGLE_SWITCH_ON_VAL (0.0)
  #define PHASE_ANGLE_OFFSET (0)
  #define SWITCH_HOLD_ITERATIONS (1)
  #define PHASE_ANGLE_SWITCH_ON_EVENT ( ((fmod((p_gapa_state->nu_norm.val[1]+PHASE_ANGLE_OFFSET),1)<PHASE_ANGLE_SWITCH_ON_VAL)&&(fmod((p_gapa_state->nu_norm.val[0]+PHASE_ANGLE_OFFSET),1)>PHASE_ANGLE_SWITCH_ON_VAL))||(p_fes_test.relay_on_iterations<SWITCH_HOLD_ITERATIONS) )

#elif RELAY_SWITCH_METHOD==3
  /* Method 3: On at angle_start and hold for N micros */
  #define PHASE_ANGLE_SWITCH_ON_VAL (0.0)
  #define PHASE_ANGLE_OFFSET (0)
  #define SWITCH_HOLD_MICROS (1)
  #define PHASE_ANGLE_SWITCH_ON_EVENT ( ((fmod((p_gapa_state->nu_norm.val[1]+PHASE_ANGLE_OFFSET),1)<PHASE_ANGLE_SWITCH_ON_VAL)&&(fmod((p_gapa_state->nu_norm.val[0]+PHASE_ANGLE_OFFSET),1)>PHASE_ANGLE_SWITCH_ON_VAL))||(p_fes_test.relay_on_micros<SWITCH_HOLD_MICROS) )

#elif RELAY_SWITCH_METHOD==4
  /* Method 4: On at foot sensors 1 and 2 AVE values above threshold. Hold for N iterations */
  #define FOOT_SENSOR_1_THRESHOLD_VAL (300.0f)
  #define FOOT_SENSOR_2_THRESHOLD_VAL (300.0f)
  #define SWITCH_HOLD_ITERATIONS (1)
  #define PHASE_ANGLE_SWITCH_ON_EVENT ( ((p_fes_state->foot_sensor_1_val_ave>FOOT_SENSOR_1_THRESHOLD_VAL)&&(p_fes_state->foot_sensor_2_val_ave>FOOT_SENSOR_2_THRESHOLD_VAL))||(p_fes_test.relay_on_iterations<SWITCH_HOLD_ITERATIONS) )

#elif RELAY_SWITCH_METHOD==5
  /* Method 5: On at foot sensors 1 and 2 AVE values above threshold. Hold for N micros */
  #define FOOT_SENSOR_1_THRESHOLD_VAL (300.0f)
  #define FOOT_SENSOR_2_THRESHOLD_VAL (300.0f)
  #define SWITCH_HOLD_MICROS (1)
  #define PHASE_ANGLE_SWITCH_ON_EVENT ( ((p_fes_state->foot_sensor_1_val_ave>FOOT_SENSOR_1_THRESHOLD_VAL)&&(p_fes_state->foot_sensor_2_val_ave>FOOT_SENSOR_2_THRESHOLD_VAL))||(p_fes_test.relay_on_micros<SWITCH_HOLD_MICROS) )
  
#elif RELAY_SWITCH_METHOD==6
  /* Method 6: On at foot sensors 1 and 2 AVE volts above threshold. Hold for N iterations */
  #define FOOT_SENSOR_1_THRESHOLD_VOLTS (300.0f)
  #define FOOT_SENSOR_2_THRESHOLD_VOLTS (300.0f)
  #define SWITCH_HOLD_ITERATIONS (1)
  #define PHASE_ANGLE_SWITCH_ON_EVENT ( ((p_fes_state->foot_sensor_1_volts_ave>FOOT_SENSOR_1_THRESHOLD_VOLTS)&&(p_fes_state->foot_sensor_2_volts_ave>FOOT_SENSOR_2_THRESHOLD_VOLTS))||(p_fes_test.relay_on_iterations<SWITCH_HOLD_ITERATIONS) )

#elif RELAY_SWITCH_METHOD==7
  /* Method 7: On at foot sensors 1 and 2 AVE volts above threshold. Hold for N micros */
  #define FOOT_SENSOR_1_THRESHOLD_VOLTS (300.0f)
  #define FOOT_SENSOR_2_THRESHOLD_VOLTS (300.0f)
  #define SWITCH_HOLD_MICROS (1)
  #define PHASE_ANGLE_SWITCH_ON_EVENT ( ((p_fes_state->foot_sensor_1_volts_ave>FOOT_SENSOR_1_THRESHOLD_VOLTS)&&(p_fes_state->foot_sensor_2_volts_ave>FOOT_SENSOR_2_THRESHOLD_VOLTS))||(p_fes_test.relay_on_micros<SWITCH_HOLD_MICROS) )

#elif RELAY_SWITCH_METHOD==8
  /* Method 8: On at foot sensors 1 and 2 AVE volts above threshold. Hold until 1 AVE volts is below threshold*/
  /* DF = Hold until 1 and 2 AVE volts are below threshold */
  #define FOOT_SENSOR_1_THRESHOLD_VOLTS (300.0f)
  #define FOOT_SENSOR_2_THRESHOLD_VOLTS (300.0f)
  #define SWITCH_HOLD_ITERATIONS (1)
  #define PHASE_ANGLE_SWITCH_ON_EVENT ((p_fes_state->foot_sensor_1_volts_ave<FOOT_SENSOR_1_THRESHOLD_VOLTS)&&(p_fes_state->foot_sensor_2_volts_ave>FOOT_SENSOR_2_THRESHOLD_VOLTS))
  #define PHASE_ANGLE_SWITCH_ON_EVENT_DF ((p_fes_state->foot_sensor_1_volts_ave<FOOT_SENSOR_1_THRESHOLD_VOLTS)&&(p_fes_state->foot_sensor_2_volts_ave<FOOT_SENSOR_2_THRESHOLD_VOLTS))

#elif RELAY_SWITCH_METHOD==9
  /* Method 9: On at foot sensors 1 and 2 AVE volts above threshold. Hold until 1 AVE volts is below threshold*/
  /* DF = Hold until 1 and 2 AVE volts are below threshold */
  #define FOOT_SENSOR_1_THRESHOLD_VOLTS (300.0f)
  #define FOOT_SENSOR_2_THRESHOLD_VOLTS (300.0f)
  #define SWITCH_HOLD_MICROS (1)
  #define PHASE_ANGLE_SWITCH_ON_EVENT ((p_fes_state->foot_sensor_1_volts_ave<FOOT_SENSOR_1_THRESHOLD_VOLTS)&&(p_fes_state->foot_sensor_2_volts_ave>FOOT_SENSOR_2_THRESHOLD_VOLTS))
  #define PHASE_ANGLE_SWITCH_ON_EVENT_DF ((p_fes_state->foot_sensor_1_volts_ave<FOOT_SENSOR_1_THRESHOLD_VOLTS)&&(p_fes_state->foot_sensor_2_volts_ave<FOOT_SENSOR_2_THRESHOLD_VOLTS))

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
  unsigned int foot_sensor_3_val;   /*add 7/23/2018 Rebecca*/
  unsigned int foot_sensor_4_val;   /*add 7/23/2018 Rebecca*/
  float foot_sensor_1_val_ave;
  float foot_sensor_2_val_ave;
  float foot_sensor_3_val_ave;      /*add 7/23/2018 Rebecca*/
  float foot_sensor_4_val_ave;      /*add 7/23/2018 Rebecca*/
  
  float foot_sensor_1_volts;
  float foot_sensor_2_volts;
  float foot_sensor_3_volts;        /*add 7/23/2018 Rebecca*/
  float foot_sensor_4_volts;        /*add 7/23/2018 Rebecca*/
  float foot_sensor_1_volts_ave;
  float foot_sensor_2_volts_ave;
  float foot_sensor_3_volts_ave;    /*add 7/23/2018 Rebecca*/
  float foot_sensor_4_volts_ave;    /*add 7/23/2018 Rebecca*/
  
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
#define SET_RELAY_1_OUTPUT pinMode(RELAY_1_PIN,INPUT)
#define SET_RELAY_2_OUTPUT pinMode(RELAY_2_PIN,INPUT)

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
#define LED_DF_PIN 10 /* digital pin */ 
#define LED_PF_PIN 12 /* digital pin */ 

/* Set LED pins to monitor Foot Switches 1 & 2*/
#define SET_LED_DF_OUTPUT pinMode(LED_DF_PIN,OUTPUT)
#define SET_LED_PF_OUTPUT pinMode(LED_PF_PIN,OUTPUT)
/* Set pins as INPUT ... not used */
#define SET_LED_DF_OUTPUT pinMode(LED_DF_PIN,INPUT)
#define SET_LED_PF_OUTPUT pinMode(LED_PF_PIN,INPUT)

/* Set LED FootSwitch Monitor State
** set the relays open, and High would be closed */
#define LED_DF_SET_HIGH digitalWrite( LED_DF_PIN, HIGH )
#define LED_DF_SET_LOW  digitalWrite( LED_DF_PIN, LOW )
#define LED_PF_SET_HIGH digitalWrite( LED_PF_PIN, HIGH )
#define LED_PF_SET_LOW  digitalWrite( LED_PF_PIN, LOW )
        

/* Analog Foot sensor in GPIO pin */
#define FOOT_SENSOR_1_PIN A1
#define FOOT_SENSOR_2_PIN A2
#define FOOT_SENSOR_3_PIN A3    //add 7-23-2018 Rebecca, heel B
#define FOOT_SENSOR_4_PIN A4    //add 7-23-2018 Rebecca, toe B

/* Analog Read val to decimal */
#define ANALOG_MAX_VAL (1023)

/* Supplied Voltage */
#define FOOT_SENSOR_VOLTAGE_IN (3.3f)

/* Side A voltage splitting resistor */
#define FOOT_SENSOR_1_R (1000.0f) //Ohms
#define FOOT_SENSOR_2_R (1000.0f) //Ohms
/* Side B voltage splitting resistor */
#define FOOT_SENSOR_3_R (1000.0f) //Ohms
#define FOOT_SENSOR_4_R (1000.0f) //Ohms


/* Set pins as INPUT ... not used */
#define SET_FOOT_SENSOR_1_OUTPUT pinMode(FOOT_SENSOR_1_PIN,INPUT)
#define SET_FOOT_SENSOR_2_OUTPUT pinMode(FOOT_SENSOR_2_PIN,INPUT)
#define SET_FOOT_SENSOR_3_OUTPUT pinMode(FOOT_SENSOR_3_PIN,INPUT)   //add 7-23-2018 Rebecca
#define SET_FOOT_SENSOR_4_OUTPUT pinMode(FOOT_SENSOR_4_PIN,INPUT)   //add 7-23-2018 Rebecca

/* Read Foot Sensor raw value  [0-ANALOG_MAX_VAL] 
** Returns : int */
#define FOOT_SENSOR_1_VAL (analogRead(FOOT_SENSOR_1_PIN))
#define FOOT_SENSOR_2_VAL (analogRead(FOOT_SENSOR_2_PIN))
#define FOOT_SENSOR_3_VAL (analogRead(FOOT_SENSOR_3_PIN))   //add 7-23-2018 Rebecca
#define FOOT_SENSOR_4_VAL (analogRead(FOOT_SENSOR_4_PIN))   //add 7-23-2018 Rebecca

/* Read Foot Sensor as voltage [0-FOOT_SENSRO_VOLTAGE_IN]
** Returns : double */
#define FOOT_SENSOR_1_VOLTS (((double)FOOT_SENSOR_1_VAL/(double)ANALOG_MAX_VAL)*(double)FOOT_SENSOR_VOLTAGE_IN)
#define FOOT_SENSOR_2_VOLTS (((double)FOOT_SENSOR_2_VAL/(double)ANALOG_MAX_VAL)*(double)FOOT_SENSOR_VOLTAGE_IN)
#define FOOT_SENSOR_3_VOLTS (((double)FOOT_SENSOR_3_VAL/(double)ANALOG_MAX_VAL)*(double)FOOT_SENSOR_VOLTAGE_IN) //add 7-23-2018 Rebecca
#define FOOT_SENSOR_4_VOLTS (((double)FOOT_SENSOR_4_VAL/(double)ANALOG_MAX_VAL)*(double)FOOT_SENSOR_VOLTAGE_IN) //add 7-23-2018 Rebecca

/* Read Foot Sensor as resistance [0-inf] 
** Returns : double */
#define FOOT_SENSOR_1_R ((((double)FOOT_SENSOR_VOLTAGE_IN/(double)FOOT_SENSOR_1_VOLTS)-1)*(double)FOOT_SENSOR_1_R)
#define FOOT_SENSOR_2_R ((((double)FOOT_SENSOR_VOLTAGE_IN/(double)FOOT_SENSOR_2_VOLTS)-1)*(double)FOOT_SENSOR_2_R)
#define FOOT_SENSOR_3_R ((((double)FOOT_SENSOR_VOLTAGE_IN/(double)FOOT_SENSOR_3_VOLTS)-1)*(double)FOOT_SENSOR_3_R)  //add 7-23-2018 Rebecca
#define FOOT_SENSOR_4_R ((((double)FOOT_SENSOR_VOLTAGE_IN/(double)FOOT_SENSOR_4_VOLTS)-1)*(double)FOOT_SENSOR_4_R)  //add 7-23-2018 Rebecca

/* For Moving Averages, define alpha */
#define FOOT_SENSOR_1_VAL_ALPHA (0.4f)
#define FOOT_SENSOR_2_VAL_ALPHA (0.4f)
#define FOOT_SENSOR_3_VAL_ALPHA (0.4f)      //add 7-23-2018 Rebecca
#define FOOT_SENSOR_4_VAL_ALPHA (0.4f)      //add 7-23-2018 Rebecca

#define FOOT_SENSOR_1_VOLTS_ALPHA (0.4f)
#define FOOT_SENSOR_2_VOLTS_ALPHA (0.4f)
#define FOOT_SENSOR_3_VOLTS_ALPHA (0.4f)    //add 7-23-2018 Rebecca
#define FOOT_SENSOR_4_VOLTS_ALPHA (0.4f)    //add 7-23-2018 Rebecca

#endif /* End FES_SPECIFIC_CONFIG_H */