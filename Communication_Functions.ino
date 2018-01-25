
/*******************************************************************
** FILE:
**   	Communication_Functions
** DESCRIPTION:
** 		This file contains all the serial communication functions
**		and protocols for use in the real-time execution code. These
**		functions are intended for the final executable.
**		These functions cannot be used in emulation mode.
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


/*************************************************
** FUNCTION: f_RespondToInput
** VARIABLES:
**		[I ]	CONTROL_TYPE			*p_control
**		[I ]	SENSOR_STATE_TYPE	*p_sensor_state
**		[I ]	WISE_STATE_TYPE		*p_wise_state
**		[I ]	int								nBytes
** RETURN:
**		NONE
** DESCRIPTION:
** 		We have recieved a request
** 		The request will be a single character (one Byte)
** 		which will coorespond to a given type of data being
** requested by the master
*/
void f_RespondToInput( CONTROL_TYPE 			*p_control,
											 SENSOR_STATE_TYPE 	*p_sensor_state,
											 CALIBRATION_TYPE		*p_calibration,
                       int nBytesIn )
{
  int i;
  unsigned char RequestByte;


  /* Debug Logging */
  char fastlog[500];


  /* uint16_t Packet_nBytes;
  ** uint16_t nBytes;
  ** uint8_t Buffer[50];
  ** uint8_t CheckSum; */
  COMMUNICATION_PACKET_TYPE Response;

  /* Some Log Output (usb) */

  sprintf(fastlog,"> Recieved %i Bytes",nBytesIn); LOG_PRINTLN( fastlog );

  /* We must read the request and respond appropriately
  ** If there is more than one request, we will respond
  ** in a FIFO fashion
  ** Each request is a single byte code character
  ** A response can be variable length and depends on the
  ** data being requested */
  for( i=0; i<nBytesIn; i++)
  {
    /* Read request from the master */
    RequestByte = COMM_READ;

    /* Some Log outputs (usb) */
  sprintf(fastlog,"> Request Code (HEX): %x",RequestByte); LOG_PRINTLN( fastlog );

    /* Respond the the request appropriately
    ** The packet architecture allows for mulitple
    ** requestes and responses via FIFO */
    switch( RequestByte )
    {
      /* TO DO:
      **   Need to add cases for things like re-locking and
      **   other error codes for rhobustness */

      case 0xB1: /* 0xB# : Debug */
        /* Packet type 11
        ** Debug test byte
        ** Data buffer
        **   1 x 16 bit integer
        **   Ints are signed */

        /* Some Log outputs (usb) */
  			sprintf(fastlog,"\tRecieved Debug Request: %x",RequestByte); LOG_PRINTLN( fastlog );
        Response.PacketType     = 11;
        Response.Buffer_nBytes  = sizeof(uint8_t)*2*1;
        Response.Packet_nBytes  = sizeof(uint16_t)*2 + sizeof(uint8_t)*(1 + Response.Buffer_nBytes);
        f_WriteIToPacket( &Response.Buffer[0], 0xB1 );
        Response.CheckSum       = f_CheckSum( &Response.Buffer[0], Response.Buffer_nBytes );
        f_SendPacket( Response );
        break;

      case 0xB2:
        /* Packet type 12
        ** Debug test 32 bit float
        ** Data buffer
        **   1 x 32 bit float
        **   Float is sent bit for bit */
  			sprintf(fastlog,"\tRecieved Debug Request: %x",RequestByte); LOG_PRINTLN( fastlog );
        Response.PacketType     = 12;
        Response.Buffer_nBytes  = sizeof(uint8_t)*4*1;
        Response.Packet_nBytes  = sizeof(uint16_t)*2 + sizeof(uint8_t)*(1 + Response.Buffer_nBytes);
        f_WriteFToPacket_s32( &Response.Buffer[0], -2.0 );
        Response.CheckSum       = f_CheckSum( &Response.Buffer[0], Response.Buffer_nBytes );
        f_SendPacket( Response );
        break;

      case 0xA1:
        /* Packet type 1
        ** Roll pitch yaw data
        ** Data buffer:
        **    3 x 16 bit fixed point floats
        **    Each element is shifted 7 bits
        **    floats are signed */

        /* Some Log outputs (usb) */
  			sprintf(fastlog,"\tRecieved Roll Pitch request ... Case : %d",RequestByte); LOG_PRINTLN( fastlog );
        Response.PacketType     = 1;
        Response.Buffer_nBytes  = sizeof(uint8_t)*2*3;
        Response.Packet_nBytes  = sizeof(uint16_t)*2 + sizeof(uint8_t)*(1 + Response.Buffer_nBytes);
        f_WriteFToPacket_u16( &Response.Buffer[sizeof(uint16_t)*0], TO_DEG(p_sensor_state->roll) );
        f_WriteFToPacket_u16( &Response.Buffer[sizeof(uint16_t)*1], TO_DEG(p_sensor_state->pitch) );
        f_WriteFToPacket_u16( &Response.Buffer[sizeof(uint16_t)*2], TO_DEG(p_sensor_state->yaw) );
        Response.CheckSum       = f_CheckSum( &Response.Buffer[0], Response.Buffer_nBytes );
        f_SendPacket( Response );
        break;

      case 0xA2:
        /* Packet type 2
        ** Roll pitch yaw data
        ** Data buffer:
        **    3 x 32 bit floats
        **    floats are packed bit for bit */
        /* Some Log outputs (usb) */
  			sprintf(fastlog,"\tRecieved Roll Pitch request ... Case : %d",RequestByte); LOG_PRINTLN( fastlog );
        Response.PacketType     = 2;
        Response.Buffer_nBytes  = sizeof(uint8_t)*4*3;
        Response.Packet_nBytes  = sizeof(uint16_t)*2 + sizeof(uint8_t)*(1 + Response.Buffer_nBytes);
        f_WriteFToPacket_s32( &Response.Buffer[sizeof(uint32_t)*0], TO_DEG(p_sensor_state->roll) );
        f_WriteFToPacket_s32( &Response.Buffer[sizeof(uint32_t)*1], TO_DEG(p_sensor_state->pitch) );
        f_WriteFToPacket_s32( &Response.Buffer[sizeof(uint32_t)*2], TO_DEG(p_sensor_state->yaw) );
        Response.CheckSum       = f_CheckSum( &Response.Buffer[0], Response.Buffer_nBytes );
        f_SendPacket( Response );
        break;

      case 0x62:
        /* DEBUG - Toggle Output
        ** Toggles calibration output mode
        ** Used to switch between gyro and accel
        ** calibration output
        ** 0:Accel (min/ave/max) in text
        ** 1:Gyro  (current/ave) in text */
  			sprintf(fastlog,"\t> Recieved Output Toggle Request ... Case : %d",RequestByte); LOG_PRINTLN( fastlog );
        if( p_control->calibration_on==1 ) { p_control->calibration_prms.output_mode = (p_control->calibration_prms.output_mode+1)%NUM_CALCOM_MODES; }
        else { p_control->output_mode = (p_control->output_mode+1)%NUM_COM_MODES; }
        break;

      case 0x63:
        /* DEBUG - Reset Calibration Variables
        ** Resets all calibration states */
  			sprintf(fastlog,"\t> Recieved Calibration Reset Request ... Case : %d",RequestByte); LOG_PRINTLN( fastlog );
        Calibration_Init( p_control, p_calibration );
        break;

      case 0x64:
        /* WISE - Reset WISE state variables
        ** Simulate heel strike */
  			sprintf(fastlog,"\t> Recieved WISE Reset Request ... Case : %d",RequestByte); LOG_PRINTLN( fastlog );
        //WISE_Reset( p_control, p_wise_state );
        break;

      default:
        LOG_PRINTLN("\t ERROR: I don't understand the request!");
  			sprintf(fastlog,"\t> Unidentified Request Code (DEC): %d",RequestByte); LOG_PRINTLN( fastlog );
        break;
    }

    /* Reset the input buffer */
    RequestByte = 0;
  }
} /* End f_RespondToInput */


/*************************************************
** FUNCTION: f_SendPacket
** VARIABLES:
**		[I ]	COMMUNICATION_PACKET_TYPE Response
** RETURN:
**		NONE
** DESCRIPTION: 
** 		This code builds the contiguous byte array
** 		from the defined "Response" packet then sends
** 		the data as a singly stream over the UART line
*/
void f_SendPacket( COMMUNICATION_PACKET_TYPE Response )
{
  uint8_t Packet[100];
  int ret;
  int i;

  /* Debug Logging */
	char fastlog[500];

  /* Initialize the array */
  for( ret=0; ret<100; ret++) Packet[ret] = 0;

  /* Build the transmit packet */
  f_WriteIToPacket( &Packet[0], Response.Packet_nBytes );
  f_WriteIToPacket( &Packet[2], Response.PacketType );
  f_WriteIToPacket( &Packet[4], Response.Buffer_nBytes );

  for( i=0; i<Response.Buffer_nBytes; i++ ) Packet[6+i] = Response.Buffer[i];
  Packet[6+Response.Buffer_nBytes] = Response.CheckSum;

  for( i=0; i<Response.Packet_nBytes+2; i++ )
  {
		sprintf(fastlog,"%x",Packet[i]); LOG_PRINT( fastlog );
    LOG_PRINT(" , ");
    COMM_WRITE(&Packet[i],1);
  }
  LOG_PRINTLN(" ");
} /* End f_SendPacket */


/*************************************************
** FUNCTION: f_WriteIToPacket
** VARIABLES:
**		[IO]	uint8_t			*Packet
**		[I ]	uint16_t		InputBuffer
** RETURN:
**		NONE
** DESCRIPTION:  
** 		This is a helper function which copies an 2 byte integer
** 		into an array of single bytes
*/
void f_WriteIToPacket( uint8_t *Packet, uint16_t InputBuffer )
{
  int i;
  int nBytes = sizeof(uint16_t);

  for( i=0; i<nBytes; i++ )
  {
    Packet[i] = (uint8_t)(InputBuffer >> ((nBytes-1-i)*8));
  }
} /* End f_WriteIToPacket */

/*************************************************
** FUNCTION: f_WriteFToPacket_u16
** VARIABLES:
**		[IO]	unsigned char	*Packet
**		[I ]	float					Input
** RETURN:
**		NONE
** DESCRIPTION:  
** 		This is a helper function which copies an 4 byte float
** 		into an array of single bytes
** 		Because the master (the C200) uses half precision floats,
** 		we cannot send the full 4 byte float.
** 		Instead, we:
**    	1) Convert the 4 byte float into a 2 byte unsigned integer
**    	2) Pack the 2 byte integer into an array of single bytes
** 		When the data is recieved by the master, it is converted to
** 		a 2 byte float.
** 		This function assumes that we are sending signed floats.
** 		If we can assume that the data is unsigned, we would be able
** 		to shift an additional bit. This could be implemented as another
** 		packet "Type"
*/
void f_WriteFToPacket_u16( unsigned char *Packet, float Input )
{
  uint16_t hpFloat = 0;
  int nBytes = 2;
  int i;

  /* Convert 4 byte float into a 2 byte unsigned int */
  hpFloat = (uint16_t)(Input * pow(2,7) + 0.5);

  /* Pack the 2 byte int into a byte array */
  for( i=0; i<nBytes; i++ )
  {
    Packet[i] = (uint8_t)( hpFloat >> ((nBytes-1-i)*8) );
  }
} /* End f_WriteFToPacket_u16 */

/*************************************************
** FUNCTION: f_WriteFToPacket_s32
** VARIABLES:
**		[IO]	unsigned char	*Packet
**		[I ]	float					Input
** RETURN:
**		NONE
** DESCRIPTION:   
** 		This function writes a float to the packet
** 		in a bit for bit fashion. Ie. we pack the float
**		as it is stored in memory
*/
void f_WriteFToPacket_s32( unsigned char *Packet, float Input )
{
  unsigned char *p_byte;
  int nBytes = 4;
  int i;

  p_byte =(unsigned char *)&Input;

  /* Pack the 32 bytes int into a byte array */
  for( i=0; i<nBytes; i++ ) { Packet[i] = ( p_byte[nBytes-1-i] ); }

} /* End f_WriteFToPacket_s32 */


/*************************************************
** FUNCTION: f_Handshake
** VARIABLES:
**		[I ]	CONTROL_TYPE	*p_control
** RETURN:
**		NONE
** DESCRIPTION: 
** 		The Handshake code waits for the TI board to
** 		initiate the handshake. Handshake is initiated
** 		by recieving any character(s). Once Initiated,
** 		we must send the baud lock character "a" or "A"
** 		and then wait for the confirmation charaacter
*/
void f_Handshake( CONTROL_TYPE *p_control )
{
  /* ASCII 'A' :: DEC:65 HEX:0x41 */
  /* ASCII 'a' :: DEC:97 HEX:0x61 */
  const char BaudLockChar = 'a';
  /* 0xAA in Binary: 1010 1010 */
  uint8_t ConfirmChar = 0xAA;
  /* 0x7E in Binary 0111 1110 */
  uint8_t FailChar = 0xAB;

  /* Rx/Tx variabels */
  uint8_t IncomingByte;
  uint8_t JunkByte;
  int     nBytesIn;

  /* Debug Logging */
  char fastlog[500];

  sprintf(fastlog,"> Using BaudLockChar (int):%i",BaudLockChar); LOG_PRINTLN( fastlog );
  sprintf(fastlog,"> Using ConfirmChar (int):%i",ConfirmChar); LOG_PRINTLN( fastlog );
  sprintf(fastlog,"> Using FailChar (int):%i",FailChar); LOG_PRINTLN( fastlog );

  /* We continue to attempt a handshake
  ** until there is a lock */
  while( p_control->BaudLock==FALSE )
  {
    /* Some Log Output (usb) */
    LOG_PRINTLN( "> Beginning Handshake" );

    /* Wait for initiation
    ** Master can send any character(s)
    ** NOTE: The first data sent from the master
    **       is assumed to be garbage */
    while( COMM_AVAILABLE==0 ) {}

    LOG_PRINTLN( "> Recieved Initiazation" );

    /* Clear the input buffer
    ** Since the data sent from the master will be garbage,
    ** we need to clear the buffer to ensure further reads
    ** are not corrupted.
    ** We delay a few ms to ensure all garbage is in FIFO buffer
    ** This allows for a proper clear */
    delay( 5 );
    nBytesIn = COMM_AVAILABLE;

  	sprintf(fastlog,"> Clearing %d characters from buffer",nBytesIn); LOG_PRINTLN( fastlog );
    while( nBytesIn-- > 0 ) { JunkByte = COMM_READ; }

    /* Once handshake is initated by the master,
    ** we send the lock character
    ** The master should respond with the confirmation
    ** character */
    // Serial.print to Tx pin
    COMM_PRINT( BaudLockChar );
  	sprintf(fastlog,"> BaudLockChar \"%c\" sent",BaudLockChar); LOG_PRINTLN( fastlog );

    /* We delay a few ms to allow the
    ** master to detect and answer the handshake */
    delay( 5 );

    /* Read incomming characters
    ** If confirmation character is detected,
    ** toggle baud lock variable */
    while( COMM_AVAILABLE==0 ) {}
    nBytesIn = COMM_AVAILABLE; /* nBytes should == 1 */
    if( nBytesIn>0 ) { IncomingByte = COMM_READ; }

    /* Some Log Output (usb) */
  	sprintf(fastlog,"> Recieved %d Bytes",nBytesIn); LOG_PRINTLN( fastlog );
  	sprintf(fastlog,"  Character (int): %d",IncomingByte); LOG_PRINTLN( fastlog );

    /* If confirmation character detected, Baud is locked
    ** Reply with confimation character to end handshake
    ** If confirmation character is not detected, lock failed
    ** We then send an error character */
    if( IncomingByte==ConfirmChar )
    {
      /* Baud lock successful */
      LOG_PRINTLN("> Baud Lock Successful");

      /* Toggle Boud lock */
      p_control->BaudLock = TRUE;

      /* Reply with confimation char to
      ** complete the handshake with the master */
      COMM_PRINT( ConfirmChar );
      LOG_PRINTLN("> Confirmation Character Sent");
    }
    else
    {
      /* Baud Lock failed */
      LOG_PRINTLN("> Baud Lock Fail");

      /* Clear input buffer */
      delay( 5 );
      nBytesIn = COMM_AVAILABLE;

  		sprintf(fastlog,"> Clearing %d characters from buffer",nBytesIn); LOG_PRINTLN( fastlog );
      while( nBytesIn-- > 0 ) { JunkByte = COMM_READ; }

      /* Reply with Error char
      ** If the Baud lock truely failed, then
      ** the master will likely not understand
      ** this response at all. This is more of
      ** a symbolic check */
      //COMM_PRINT( FailChar );
      //COMM_PRINT( IncomingByte );
      LOG_PRINTLN("> Fail Character Sent");
    }
    /* Reset Input Buffer */
    IncomingByte = 0;
  }
} /* End f_Handshake */


/*************************************************
** FUNCTION: f_CheckSum
** VARIABLES:
**		[I ]	unsigned char	*p_Buffer
**		[I ]	uint16_t			nBytes
** RETURN:
**		uint8_t		checksum
** DESCRIPTION: 
** 		This function gets a simple checksum for
** 		the response packet. This checksum simply summs
** 		the response buffer (modulated to keep within 1 byte)
** 		This allows for proper data transmission
*/
uint8_t f_CheckSum( unsigned char *p_Buffer, uint16_t nBytes )
{
  int i;
  uint8_t checksum = 0;

  /* Our check sum is a simmple summation */
  for( i=0; i<nBytes; i++) { checksum += p_Buffer[i]; }

  return( checksum & 0xFF );
} /* End f_CheckSum */




