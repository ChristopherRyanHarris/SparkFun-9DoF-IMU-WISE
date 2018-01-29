
/*******************************************************************
** FILE:
**   	Communication_Config.h
** DESCRIPTION:
**
********************************************************************/
#ifndef CCOMMUNICATION_CONFIG_H
#define CCOMMUNICATION_CONFIG_H


/*******************************************************************
** Typedefs *********************************************************
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


#endif /* CCOMMUNICATION_CONFIG_H */
