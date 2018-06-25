
/*******************************************************************
** FILE:
**    FES_Specific_Config.h
** DESCRIPTION:
**    This header was built specific for the FES test.
********************************************************************/
#ifndef FES_SPECIFIC_CONFIG_H
#define FES_SPECIFIC_CONFIG_H



/*******************************************************************
** Relays
********************************************************************/

#define RELAY_1_PIN 8 /* digital pin */
#define RELAY_2_PIN 9 /* digital pin */ 

/* Read Current Relay State 
** Returns High or low 
** Relays are normally open, so a low 
** state would set the relays open */
#define RELAY_1_STATE digitalRead(RELAY_1_PIN)
#define RELAY_2_STATE digitalRead(RELAY_2_PIN)

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



#endif /* End FES_SPECIFIC_CONFIG_H */


