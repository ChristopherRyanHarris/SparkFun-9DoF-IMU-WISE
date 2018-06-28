
/*******************************************************************
** FILE:
**    Communication_Functions
** DESCRIPTION:
**    This file contains all the serial communication functions
**    and protocols for use in the real-time execution code. These
**    functions are intended for the final executable.
**    These functions cannot be used in emulation mode.
********************************************************************/


/*******************************************************************
** Includes ********************************************************
********************************************************************/

#ifndef COMMON_CONFIG_H
  #include "../Include/Common_Config.h"
#endif
#if EXE_MODE==1 /* Emulator Mode */
  /* In emulation mode, "Emulator_Protos" is needed to
  ** use functions in other files.
  ** NOTE: This header should contain the function
  **       prototypes for all execution functions */
  #include "../Include/Emulator_Protos.h"
#endif  /* End Emulator Mode */

/*******************************************************************
** Functions *******************************************************
********************************************************************/


/*************************************************
** FUNCTION: f_RespondToInput
** VARIABLES:
**    [I ]  CONTROL_TYPE      *p_control
**    [I ]  SENSOR_STATE_TYPE *p_sensor_state
**    [I ]  WISE_STATE_TYPE   *p_wise_state
**    [I ]  int               nBytes
** RETURN:
**    NONE
** DESCRIPTION:
**    We have received a request
**    The request will be a single character (one Byte)
**    which will correspond to a given type of data being
** requested by the master
*/
void f_RespondToInput( CONTROL_TYPE       *p_control,
                       SENSOR_STATE_TYPE  *p_sensor_state,
                       CALIBRATION_TYPE   *p_calibration,
                       int nBytesIn )
{
  int i;
  unsigned char RequestByte;
  
  /* uint16_t Packet_nBytes;
  ** uint16_t nBytes;
  ** uint8_t Buffer[50];
  ** uint8_t CheckSum; */
  COMMUNICATION_PACKET_TYPE Response;

  /* Some Log Output (usb) */

  LOG_INFO( "> Recieved %i Bytes", nBytesIn );

  /* We must read the request and respond appropriately
  ** If there is more than one request, we will respond
  ** in a FIFO fashion
  ** Each request is a single byte code character
  ** A response can be variable length and depends on the
  ** data being requested */
  for( i=0; i<nBytesIn; i++)
  {
    /* Read request from the master */
    RequestByte = SERIAL_READ;

    /* Some Log outputs (usb) */
    LOG_INFO( "> Request Code (HEX): %x", RequestByte );

    /* Respond the the request appropriately
    ** The packet architecture allows for multiple
    ** requests and responses via FIFO */
    switch( RequestByte )
    {
      /* TO DO:
      **   Need to add cases for things like re-locking and
      **   other error codes for robustness */

      case 0xB1: /* 0xB# : Debug */
        /* Packet type 11
        ** Debug test byte
        ** Data buffer
        **   1 x 16 bit integer
        **   Ints are signed */

        /* Some Log outputs (usb) */
        LOG_INFO( "\tReceived Debug Request: %x", RequestByte );
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
        LOG_INFO( "\tReceived Debug Request: %x", RequestByte );
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
        LOG_INFO( "\tReceived Roll Pitch request ... Case : %d", RequestByte );
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
        LOG_INFO( "\tReceived Roll Pitch request ... Case : %d", RequestByte );
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
        LOG_INFO( "\t> Received Output Toggle Request ... Case : %d", RequestByte );
        if( p_control->calibration_on==1 ) { p_control->calibration_prms.output_mode = (p_control->calibration_prms.output_mode+1)%NUM_CALCOM_MODES; }
        else { p_control->output_mode = (p_control->output_mode+1)%NUM_COM_MODES; }
        break;

      case 0x63:
        /* DEBUG - Reset Calibration Variables
        ** Resets all calibration states */
        LOG_INFO( "\t> Received Calibration Reset Request ... Case : %d", RequestByte );
        Calibration_Init( p_control, p_calibration );
        break;

      case 0x64:
        /* WISE - Reset WISE state variables
        ** Simulate heel strike */
        LOG_INFO( "\t> Received WISE Reset Request ... Case : %d", RequestByte );
        //WISE_Reset( p_control, p_wise_state );
        break;

      default:
        LOG_INFO( "\t ERROR: I don't understand the request!" );
        LOG_INFO( "\t> Unidentified Request Code (DEC): %d", RequestByte );
        break;
    }

    /* Reset the input buffer */
    RequestByte = 0;
  }
} /* End f_RespondToInput */


/*************************************************
** FUNCTION: f_SendPacket
** VARIABLES:
**    [I ]  COMMUNICATION_PACKET_TYPE Response
** RETURN:
**    NONE
** DESCRIPTION:
**    This code builds the contiguous byte array
**    from the defined "Response" packet then sends
**    the data as a singly stream over the UART line
*/
void f_SendPacket( COMMUNICATION_PACKET_TYPE Response )
{
  uint8_t Packet[100];
  int ret;
  int i;
  
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
    SERIAL_WRITE(&Packet[i],1);
  }
} /* End f_SendPacket */


/*************************************************
** FUNCTION: f_WriteIToPacket
** VARIABLES:
**    [IO]  uint8_t     *Packet
**    [I ]  uint16_t    InputBuffer
** RETURN:
**    NONE
** DESCRIPTION:
**    This is a helper function which copies an 2 byte integer
**    into an array of single bytes
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
**    [IO]  unsigned char *Packet
**    [I ]  float         Input
** RETURN:
**    NONE
** DESCRIPTION:
**    This is a helper function which copies an 4 byte float
**    into an array of single bytes
**    Because the master (the C200) uses half precision floats,
**    we cannot send the full 4 byte float.
**    Instead, we:
**      1) Convert the 4 byte float into a 2 byte unsigned integer
**      2) Pack the 2 byte integer into an array of single bytes
**    When the data is received by the master, it is converted to
**    a 2 byte float.
**    This function assumes that we are sending signed floats.
**    If we can assume that the data is unsigned, we would be able
**    to shift an additional bit. This could be implemented as another
**    packet "Type"
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
**    [IO]  unsigned char *Packet
**    [I ]  float         Input
** RETURN:
**    NONE
** DESCRIPTION:
**    This function writes a float to the packet
**    in a bit for bit fashion. Ie. we pack the float
**    as it is stored in memory
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
**    [I ]  CONTROL_TYPE  *p_control
** RETURN:
**    NONE
** DESCRIPTION:
**    The Handshake code waits for the TI board to
**    initiate the handshake. Handshake is initiated
**    by receiving any character(s). Once Initiated,
**    we must send the baud lock character "a" or "A"
**    and then wait for the confirmation character
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

  /* Rx/Tx variables */
  uint8_t IncomingByte;
  uint8_t JunkByte;
  int     nBytesIn;
  
  LOG_INFO( "> Using BaudLockChar (int):%i", BaudLockChar );
  LOG_INFO( "> Using ConfirmChar (int):%i",  ConfirmChar );
  LOG_INFO( "> Using FailChar (int):%i",     FailChar );

  /* We continue to attempt a handshake
  ** until there is a lock */
  while( p_control->BaudLock==FALSE )
  {
    /* Some Log Output (usb) */
    LOG_INFO( "> Beginning Handshake" );

    /* Wait for initiation
    ** Master can send any character(s)
    ** NOTE: The first data sent from the master
    **       is assumed to be garbage */
    while( SERIAL_AVAILABLE==0 ) {}

    LOG_INFO( "> Received Initialization" );

    /* Clear the input buffer
    ** Since the data sent from the master will be garbage,
    ** we need to clear the buffer to ensure further reads
    ** are not corrupted.
    ** We delay a few ms to ensure all garbage is in FIFO buffer
    ** This allows for a proper clear */
    delay( 5 );
    nBytesIn = SERIAL_AVAILABLE;

    LOG_INFO( "> Clearing %d characters from buffer", nBytesIn );
    while( nBytesIn-- > 0 ) { JunkByte = SERIAL_READ; }

    /* Once handshake is initiated by the master,
    ** we send the lock character
    ** The master should respond with the confirmation
    ** character */
    // Serial.print to Tx pin
    SERIAL_PRINT( BaudLockChar );
    LOG_INFO( "> BaudLockChar \"%c\" sent", BaudLockChar );

    /* We delay a few ms to allow the
    ** master to detect and answer the handshake */
    delay( 5 );

    /* Read incoming characters
    ** If confirmation character is detected,
    ** toggle baud lock variable */
    while( SERIAL_AVAILABLE==0 ) {}
    nBytesIn = SERIAL_AVAILABLE; /* nBytes should == 1 */
    if( nBytesIn>0 ) { IncomingByte = SERIAL_READ; }

    /* Some Log Output (usb) */
    LOG_INFO( "> Recieved %d Bytes", nBytesIn );
    LOG_INFO( "  Character (int): %d", IncomingByte );

    /* If confirmation character detected, Baud is locked
    ** Reply with confirmation character to end handshake
    ** If confirmation character is not detected, lock failed
    ** We then send an error character */
    if( IncomingByte==ConfirmChar )
    {
      /* Baud lock successful */
      LOG_INFO( "> Baud Lock Successful" );

      /* Toggle Baud lock */
      p_control->BaudLock = TRUE;

      /* Reply with confirmation char to
      ** complete the handshake with the master */
      SERIAL_PRINT( ConfirmChar );
      LOG_INFO( "> Confirmation Character Sent" );
    }
    else
    {
      /* Baud Lock failed */
      LOG_INFO( "> Baud Lock Fail");

      /* Clear input buffer */
      delay( 5 );
      nBytesIn = SERIAL_AVAILABLE;

      LOG_INFO( "> Clearing %d characters from buffer", nBytesIn );
      while( nBytesIn-- > 0 ) { JunkByte = SERIAL_READ; }

      /* Reply with Error char
      ** If the Baud lock truly failed, then
      ** the master will likely not understand
      ** this response at all. This is more of
      ** a symbolic check */
      //SERIAL_PRINT( FailChar );
      //SERIAL_PRINT( IncomingByte );
      LOG_INFO( "> Fail Character Sent" );
    }
    /* Reset Input Buffer */
    IncomingByte = 0;
  }
} /* End f_Handshake */


/*************************************************
** FUNCTION: f_CheckSum
** VARIABLES:
**    [I ]  unsigned char *p_Buffer
**    [I ]  uint16_t      nBytes
** RETURN:
**    uint8_t   checksum
** DESCRIPTION:
**    This function gets a simple checksum for
**    the response packet. This checksum simply sums
**    the response buffer (modulated to keep within 1 byte)
**    This allows for proper data transmission
*/
uint8_t f_CheckSum( unsigned char *p_Buffer, uint16_t nBytes )
{
  int i;
  uint8_t checksum = 0;

  /* Our check sum is a simple summation */
  for( i=0; i<nBytes; i++) { checksum += p_Buffer[i]; }

  return( checksum & 0xFF );
} /* End f_CheckSum */




