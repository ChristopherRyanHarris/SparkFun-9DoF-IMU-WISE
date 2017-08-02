/*************************************************
** FILE: Com_Functions
** This file contains the helper functions which
** allow us to communicate with either the user
** (via LOG_PORT) or another processor (via COM_PORT)
** It handles data packing, command interpretation,
** and other such protocol-level operations.
**************************************************/


/*******************************************************************
** Includes ********************************************************
********************************************************************/


#if EXE_MODE==1 /* Emulator Mode */
#include "./Include/Emulator_Config.h"
extern CAL_STATE_TYPE      g_calibration;
extern DCM_STATE_TYPE      g_dcm_state;
extern DSP_COMMON_TYPE     g_dsp;
extern SENSOR_STATE_TYPE   g_sensor_state;
extern CONTROL_STATE_TYPE  g_control_state;
extern WISE_STATE_TYPE     g_wise_state;
#endif /* End Emulator Mode */


/*******************************************************************
** Functions *******************************************************
********************************************************************/


/*************************************************
** Debug_LogOut
** This function just prints a standard string
** to the log_port serial port.
** It prints the rpy as well as the timestamp and
** and estimate of the sample rate
*/
void Debug_LogOut( void )
{
  String imuLog = ""; 
  char fastlog[500];

  switch ( g_control_state.output_mode )
  {
    case 0:
    	sprintf(fastlog,"T:%d, DT:%.4f, SR:%.4f, R:%.4f, P:%.4f, Y:%.4f, A:%.0f,%.0f,%.0f, G:%.0f,%.0f,%.0f\n",
    		g_control_state.timestamp,g_control_state.G_Dt,(1/g_control_state.G_Dt),
	    	TO_DEG(g_sensor_state.roll),TO_DEG(g_sensor_state.pitch),TO_DEG(g_sensor_state.yaw),
	    	g_sensor_state.accel[0],g_sensor_state.accel[1],g_sensor_state.accel[2],
	    	g_sensor_state.gyro[0],g_sensor_state.gyro[1],g_sensor_state.gyro[2]);
      break;
    case 1:
    	sprintf(fastlog,"WISE_v (v/av/d/od/op): 1:%.4f/%.4f/%.4f/%.4f 2:%.4f/%.4f/%.4f/%.4f\n",
    		g_wise_state.vel[0],g_wise_state.vel_ave[0],g_wise_state.vel_delta[0],g_wise_state.omega_vd[0],g_wise_state.omega_vp[0],
    		g_wise_state.vel[1],g_wise_state.vel_ave[1],g_wise_state.vel_delta[1],g_wise_state.omega_vd[1],g_wise_state.omega_vp[1] );
      break;
    case 2:
    	sprintf(fastlog,"WISE_a (a/aa/d/od/op): 1:%.4f/%.4f/%.4f/%.4f 2:%.4f/%.4f/%.4f/%.4f\n",
    		g_wise_state.accel[0],g_wise_state.accel_ave[0],g_wise_state.accel_delta[0],g_wise_state.omega_ad[0],g_wise_state.omega_ap[0],
    		g_wise_state.accel[1],g_wise_state.accel_ave[1],g_wise_state.accel_delta[1],g_wise_state.omega_ad[1],g_wise_state.omega_ap[1]);
      break;
    case 3:
    	sprintf(fastlog,"%d,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.4f,%.4f,%.4f\n",
    		g_control_state.timestamp,
	    	g_sensor_state.accel[0],g_sensor_state.accel[1],g_sensor_state.accel[2],
	    	g_sensor_state.gyro[0],g_sensor_state.gyro[1],g_sensor_state.gyro[2],
	    	TO_DEG(g_sensor_state.yaw),TO_DEG(g_sensor_state.pitch),TO_DEG(g_sensor_state.roll) );
	    LOG_PRINT( fastlog ); 
      break;
    case 4:
    	sprintf(fastlog,"\terr est (p1/p2/p3/pave): %.4f,%.4f,%.4f,%.4f\n",
    		g_wise_state.pe[0],g_wise_state.pe[1],g_wise_state.pe[2],g_wise_state.pave );
    default:
    	break;
  }
  LOG_PRINT( fastlog ); 
} /* End Debug_LogOut */

/*************************************************
** Cal_LogOut
** This function just prints a standard string
** to the log_port serial port.
** It is designed to assist in the calibration
** of the sensor
*/
void Cal_LogOut(void)
{
  String imuLog = ""; 
  imuLog += "Time:" + String( g_control_state.timestamp ) + ", "; 
  imuLog += "DT:" + String( g_control_state.G_Dt,3 ) + ", ";
  imuLog += "SR:" + String( (1/g_control_state.G_Dt) ) + ", "; 

  switch ( g_calibration.output_mode )
  {
    case 0:
      imuLog += "accel (min/ave/max): ";
      imuLog += String(g_calibration.accel_min[0],3) + "/" + String(g_calibration.accel_total[0]/g_calibration.N,3) + "/" + String(g_calibration.accel_max[0],3) + ", ";
      imuLog += String(g_calibration.accel_min[1],3) + "/" + String(g_calibration.accel_total[1]/g_calibration.N,3) + "/" + String(g_calibration.accel_max[1],3) + ", ";
      imuLog += String(g_calibration.accel_min[2],3) + "/" + String(g_calibration.accel_total[2]/g_calibration.N,3) + "/" + String(g_calibration.accel_max[2],3);
      break;
    case 1:
      imuLog += "gyro (ave/current): ";
      imuLog += String(g_calibration.gyro_total[0]/g_calibration.N,3)  + "/" + String( g_sensor_state.gyro[0],3 ) + ", ";
      imuLog += String(g_calibration.gyro_total[1]/g_calibration.N,3)  + "/" + String( g_sensor_state.gyro[1],3 ) + ", ";
      imuLog += String(g_calibration.gyro_total[2]/g_calibration.N,3)  + "/" + String( g_sensor_state.gyro[2],3 );
      break;
  }
  imuLog += "\r\n";
  LOG_PRINT( imuLog );
} /* End Cal_LogOut */

/*************************************************
** f_RespondToInput
** We have recieved a request
** The request will be a single character (one Byte)
** which will coorespond to a given type of data being
** requested by the master 
*/
void f_RespondToInput( int nBytesIn )
{
  int i;
  unsigned char RequestByte;
  unsigned char Garbage;

  /* uint16_t Packet_nBytes;
  ** uint16_t nBytes;
  ** uint8_t Buffer[50];
  ** uint8_t CheckSum; */
  RESPONSE_TYPE Response;

  /* Some Log Output (usb) */
  LOG_PRINTLN("> Recieved " + String(nBytesIn) + " Bytes");

  /* We must read the request and respond appropriately
  ** If there is more than one request, we will respond
  ** in a FIFO fashion
  ** Each request is a single byte code character
  ** A response can be variable length and depends on the
  ** data being requested */
  for( i=0; i<nBytesIn; i++)
  {
    /* Read request from the master */
    RequestByte = COMM_PORT.read();

    /* Some Log outputs (usb) */
    LOG_PRINT("> Request Code (HEX): ");
    LOG_PRINTLN(RequestByte, HEX);

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
        LOG_PRINT("\tRecieved Debug Request: ");
        LOG_PRINTLN(RequestByte, HEX);
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
        LOG_PRINT("\tRecieved Debug Request: ");
        LOG_PRINTLN(RequestByte, HEX);
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
        LOG_PRINT("\tRecieved Roll Pitch request ... Case : ");
        LOG_PRINTLN(RequestByte, DEC);
        Response.PacketType     = 1;
        Response.Buffer_nBytes  = sizeof(uint8_t)*2*3;
        Response.Packet_nBytes  = sizeof(uint16_t)*2 + sizeof(uint8_t)*(1 + Response.Buffer_nBytes);
        f_WriteFToPacket_u16( &Response.Buffer[sizeof(uint16_t)*0], TO_DEG(g_sensor_state.roll) );
        f_WriteFToPacket_u16( &Response.Buffer[sizeof(uint16_t)*1], TO_DEG(g_sensor_state.pitch) );
        f_WriteFToPacket_u16( &Response.Buffer[sizeof(uint16_t)*2], TO_DEG(g_sensor_state.yaw) );
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
        LOG_PRINT("\tRecieved Roll Pitch request ... Case : ");
        LOG_PRINTLN(RequestByte, DEC);
        Response.PacketType     = 2;
        Response.Buffer_nBytes  = sizeof(uint8_t)*4*3;
        Response.Packet_nBytes  = sizeof(uint16_t)*2 + sizeof(uint8_t)*(1 + Response.Buffer_nBytes);
        f_WriteFToPacket_s32( &Response.Buffer[sizeof(uint32_t)*0], TO_DEG(g_sensor_state.roll) );
        f_WriteFToPacket_s32( &Response.Buffer[sizeof(uint32_t)*1], TO_DEG(g_sensor_state.pitch) );
        f_WriteFToPacket_s32( &Response.Buffer[sizeof(uint32_t)*2], TO_DEG(g_sensor_state.yaw) );
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
        LOG_PRINT("\t> Recieved Output Toggle Request ... Case : ");
        LOG_PRINTLN(RequestByte, DEC);
        if( CALIBRATE_MODE ) { g_calibration.output_mode = (g_calibration.output_mode+1)%NUM_CALCOM_MODES; }
        else { g_control_state.output_mode = (g_control_state.output_mode+1)%NUM_COM_MODES; }
        break;

      case 0x63:
        /* DEBUG - Reset Calibration Variables
        ** Resets all calibration states */
        LOG_PRINT("\t> Recieved Calibration Reset Request ... Case : ");
        LOG_PRINTLN(RequestByte, DEC);
        Calibration_Init();
        break;

      case 0x64:
        /* WISE - Reset WISE state variables
        ** Simulate heel strike */
        LOG_PRINT("\t> Recieved WISE Reset Request ... Case : ");
        LOG_PRINTLN(RequestByte, DEC);
        WISE_Reset();
        break;

      default:
        LOG_PRINTLN("\t ERROR: I don't understand the request!");
        LOG_PRINT("\t Unidentified Request Code (DEC): ");
        LOG_PRINTLN(RequestByte, DEC);
        break;
    }

    /* Reset the input buffer */
    RequestByte = 0;
  }
} /* End f_RespondToInput */


/*************************************************
** f_SendPacket
** This code builds the contiguous byte array
** from the defined "Response" packet then sends
** the data as a singly stream over the UART line
*/
void f_SendPacket( RESPONSE_TYPE Response )
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
    LOG_PRINT(Packet[i],HEX);
    LOG_PRINT(" , ");
    COMM_PORT.write(&Packet[i],1);
  }
  LOG_PRINTLN();
} /* End f_SendPacket */

/*************************************************
** f_WriteIToPacket
** This is a helper function which copies an 2 byte integer
** into an array of single bytes
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
** f_WriteFToPacket_u16
** This is a helper function which copies an 4 byte float
** into an array of single bytes
** Because the master (the C200) uses half precision floats,
** we cannot send the full 4 byte float.
** Instead, we:
**    1) Convert the 4 byte float into a 2 byte unsigned integer
**    2) Pack the 2 byte integer into an array of single bytes
** When the data is recieved by the master, it is converted to
** a 2 byte float.
** This function assumes that we are sending signed floats.
** If we can assume that the data is unsigned, we would be able
** to shift an additional bit. This could be implemented as another
** packet "Type"
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
** f_WriteFToPacket_s32
** This function writes a float to the packet
** in a bit for bit fashion. Ie. we pack the float
** as it is stored in memory
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
** f_Handshake
** The Handshake code waits for the TI board to
** initiate the handshake. Handshake is initiated
** by recieving any character(s). Once Initiated,
** we must send the baud lock character "a" or "A"
** and then wait for the confirmation charaacter
*/
void f_Handshake( void )
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
  int     nBytesIn;

  LOG_PRINT("> Using BaudLockChar (int):");
  LOG_PRINTLN(BaudLockChar,DEC);
  LOG_PRINT("> Using ConfirmChar (int):");
  LOG_PRINTLN(ConfirmChar,DEC);
  LOG_PRINT("> Using FailChar (int):");
  LOG_PRINTLN(FailChar,DEC);

  /* We continue to attempt a handshake
  ** until there is a lock */
  while( g_control_state.g_BaudLock==false )
  {
    /* Some Log Output (usb) */
    LOG_PRINTLN("> Beginning Handshake");

    /* Wait for initiation
    ** Master can send any character(s)
    ** NOTE: The first data sent from the master
    **       is assumed to be garbage */
    while( Serial.available()==0 ) {}

    LOG_PRINTLN("> Recieved Initiazation");

    /* Clear the input buffer
    ** Since the data sent from the master will be garbage,
    ** we need to clear the buffer to ensure further reads
    ** are not corrupted.
    ** We delay a few ms to ensure all garbage is in FIFO buffer
    ** This allows for a proper clear */
    delay( 5 );
    nBytesIn = Serial.available();
    LOG_PRINTLN("> Clearing " + String(nBytesIn) + " characters from buffer");
    while( nBytesIn-- > 0 ) { Serial.read(); }

    /* Once handshake is initated by the master,
    ** we send the lock character
    ** The master should respond with the confirmation
    ** character */
    Serial.print( BaudLockChar ); // Serial.print to Tx pin
    LOG_PRINTLN("> BaudLockChar \"" + String(BaudLockChar) + "\" sent");

    /* We delay a few ms to allow the
    ** master to detect and answer the handshake */
    delay( 5 );

    /* Read incomming characters
    ** If confirmation character is detected,
    ** toggle baud lock variable */
    while( Serial.available()==0 ) {}
    nBytesIn = Serial.available(); /* nBytes should == 1 */
    if( nBytesIn>0 ) { IncomingByte = Serial.read(); }

    /* Some Log Output (usb) */
    LOG_PRINTLN("> Recieved " + String(nBytesIn) + " Bytes");
    LOG_PRINT("  Character (int): ");
    LOG_PRINTLN(IncomingByte, DEC);

    /* If confirmation character detected, Baud is locked
    ** Reply with confimation character to end handshake
    ** If confirmation character is not detected, lock failed
    ** We then send an error character */
    if( IncomingByte==ConfirmChar )
    {
      /* Baud lock successful */
      LOG_PRINTLN("> Baud Lock Successful");

      /* Toggle Boud lock */
      g_control_state.g_BaudLock = true;

      /* Reply with confimation char to
      ** complete the handshake with the master */
      Serial.print( ConfirmChar );
      LOG_PRINTLN("> Confirmation Character Sent");
    }
    else
    {
      /* Baud Lock failed */
      LOG_PRINTLN("> Baud Lock Fail");

      /* Clear input buffer */
      delay( 5 );
      nBytesIn = Serial.available();
      LOG_PRINTLN("> Clearing " + String(nBytesIn) + " characters from buffer");
      while( nBytesIn-- > 0 ) { Serial.read(); }

      /* Reply with Error char
      ** If the Baud lock truely failed, then
      ** the master will likely not understand
      ** this response at all. This is more of
      ** a symbolic check */
      //Serial.print( FailChar );
      //Serial.print( IncomingByte );
      LOG_PRINTLN("> Fail Character Sent");
    }
    /* Reset Input Buffer */
    IncomingByte = 0;
  }
} /* End f_Handshake */


/*************************************************
** f_CheckSum
** This function gets a simple checksum for
** the response packet. This checksum simply summs
** the response buffer (modulated to keep within 1 byte)
** This allows for proper data transmission
*/
uint8_t f_CheckSum( unsigned char *p_Buffer, uint16_t nBytes )
{
  int i;
  uint8_t checksum = 0;

  /* Our check sum is a simmple summation */
  for( i=0; i<nBytes; i++) { checksum += p_Buffer[i]; }

  return( checksum & 0xFF );
} /* End f_CheckSum */





