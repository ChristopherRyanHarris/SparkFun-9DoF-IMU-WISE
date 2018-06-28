
/*******************************************************************
** FILE:
**    Communication_Config.h
** DESCRIPTION:
**
********************************************************************/
#ifndef CCOMMUNICATION_CONFIG_H
#define CCOMMUNICATION_CONFIG_H


/*******************************************************************
** Typedefs ********************************************************
********************************************************************/

/*
** TYPE: COMMUNICATION_PACKET_TYPE
** Used to store temporary response data
** for responding to request from master */
typedef struct
{
  uint16_t       Packet_nBytes;  /* Length of entire packet, minus this variable, in bytes */
  uint16_t       PacketType;     /* Type code of packet */
  uint16_t       Buffer_nBytes;  /* Length of data buffer in bytes (0-50) */
  unsigned char  Buffer[50];     /* Data buffer */
  unsigned char  CheckSum;       /* CheckSum of data buffer only */
} COMMUNICATION_PACKET_TYPE;

enum
{
	/* Block A : 
	** Used for setting parameters.
	** Items such as toggling the output format and resetting states
	** would be assigned to these.
	*/
	INPUT_A0 = 0xA0, 
	INPUT_A1,  INPUT_A2,  INPUT_A3,  INPUT_A4,  INPUT_A5, 
	INPUT_A6,  INPUT_A7,  INPUT_A8,  INPUT_A9,  INPUT_AA,
	INPUT_AB,  INPUT_AC,  INPUT_AD,  INPUT_AE,  INPUT_AF,
	
	/* Block B : 
	** Used for formatted data requests 
	*/
	INPUT_B0  = 0xB0,
	INPUT_B1,  INPUT_B2,  INPUT_B3,  INPUT_B4,  INPUT_B5, 
	INPUT_B6,  INPUT_B7,  INPUT_B8,  INPUT_B9,  INPUT_BA,
	INPUT_BB,  INPUT_BC,  INPUT_BD,  INPUT_BE,  INPUT_BF,
	
	/* Block D : 
	** Used for debug
	*/
	INPUT_D0  = 0xD0,
	INPUT_D1,  INPUT_D2,  INPUT_D3,  INPUT_D4,  INPUT_D5, 
	INPUT_D6,  INPUT_D7,  INPUT_D8,  INPUT_D9,  INPUT_DA,
	INPUT_DB,  INPUT_DC,  INPUT_DD,  INPUT_DE,  INPUT_DF,

} INPUT_MAPPINGS_ENUM; /* Setting parameters */


/*******************************************************************
** Macros **********************************************************
********************************************************************/

#if EXE_MODE==0 /* 0 : IMU Mode */

  /* I2C Macros I2C addresses
  ******************************************************************/
  #define WIRE_SEND(b) Wire.write( (byte) b )
  #define WIRE_RECEIVE() Wire.read()

#endif


#endif /* CCOMMUNICATION_CONFIG_H */
