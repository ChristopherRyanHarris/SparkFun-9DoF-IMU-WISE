
/******************************************************************
** Communication Parameters
*******************************************************************/

/* TO DO : Move this somehow */
#define ENABLE_SD_LOGGING 1

/* Serial Port Configuration */
//#define UART_PORT_BAUD 115200
#define UART_PORT_BAUD 250000

//#define SERIAL_PORT_BAUD 9600
#define SERIAL_PORT_BAUD 250000

/* Data LOG period (us) */
#define DATA_LOG_RATE 1

/* The LED can be used for external debugging */
#define LED_BLINK_RATE 300

#define MAX_LINE_LEN 500
#define MAX_BUFFER_LEN ONE_KILOBYTE 




/******************************************************************
** SD Logging Parameters
*******************************************************************/


#define SD_PIN 38

#define LOG_FILE_INDEX_MAX 999      /* Max number of "log.txt" files */
#define LOG_FILE_PREFIX "log"       /* Prefix name for log files */
#define LOG_FILE_SUFFIX "txt"       /* Suffix name for log files */
#define SD_MAX_FILE_SIZE ONE_GIGABYTE /* 1 GB max file size */


/******************************************************************
** Communication Macros
*******************************************************************/

#if EXE_MODE==0 /* 0 : IMU Mode */
  
  /* The debug port is simply the serial terminal */
  #define UART_PORT SERIAL_PORT_USBVIRTUAL
  
  /* Print [func,line] */
  #define LOG_TAG \
  do{ \
    char _buffer_log_tag_db[MAX_LINE_LEN]; \
    sprintf(_buffer_log_tag_db,"[%s:%d] ",__FUNCTION__,__LINE__); \
    UART_PORT.print(_buffer_log_tag_db); \
    UART_PORT.flush(); \
  }while(0)
  
  /* Print [long_func, line] */
  #define LOG_TAG_LONG \
  do{ \
    char _buffer_log_tag_lg_db[MAX_LINE_LEN]; \
    sprintf(_buffer_log_tag_lg_db,"[%s:%d] ",__PRETTY_FUNCTION__,__LINE__); \
    UART_PORT.print(_buffer_log_tag_lg_db); \
    UART_PORT.flush(); \
  }while(0)
  
  /* Force log to serial (UART) port w/ DB tag */
  #define UART_LOG(...) \
  do{ \
    if(DEBUG) \
    { \
      char _buffer_uart_log_db[MAX_LINE_LEN]; \
      LOG_TAG; \ 
      sprintf(_buffer_uart_log_db,__VA_ARGS__); \
      UART_PORT.println(_buffer_uart_log_db); \
      UART_PORT.flush(); \
    } \
  }while(0)
  
  /* Force log to serial (UART) port w/o DB tag */
  #define UART_LOG_SHORT(...) \
  do{ \
    if(DEBUG) \
    { \
      char _buffer_uart_log_short_db[MAX_LINE_LEN]; \
      sprintf(_buffer_uart_log_short_db,__VA_ARGS__); \
      UART_PORT.println(_buffer_uart_log_short_db); \
      UART_PORT.flush(); \
    } \
  }while(0)
  
  /* In the SEN-14001 platform, we have the 
  ** ability to log to an sd card (if present) */
  #if (ENABLE_SD_LOGGING==TRUE) /* Using SD Card */ 
    
    /* Print message without forced newline */
    #define LOG_PRINT(...) \
    do{ \
      char _buffer_dat[MAX_LINE_LEN]; \
      char _buffer_log_pnt_db[MAX_LINE_LEN]; \
      sprintf(_buffer_log_pnt_db,__VA_ARGS__); \
      sprintf( _buffer_dat, "[%s:%d] %s", __FUNCTION__,__LINE__, _buffer_log_pnt_db ); \
      if(p_control->SDCardPresent==TRUE) \
      { \
        p_control->LogBufferLen += strlen(_buffer_dat); \
        strcat( p_control->LogBuffer, _buffer_dat ); \
        strcat( p_control->LogBuffer, "\n" ); \
        if(p_control->LogBufferLen>MAX_BUFFER_LEN) \
        { \
          UART_LOG( "<LOG_PRINT::Buffer len above thresh>" ); \
          if(p_control->LogFile_fh!=NULL) \
          { \
            UART_LOG( "<LOG_PRINT::Valid fh>" ); \
            if(p_control->LogFile_fh.size()>SD_MAX_FILE_SIZE) \
            { \
              UART_LOG( "<LOG_PRINT::File reached max size, closing %s ... Getting new file name>", p_control->LogFileName ); \
              p_control->LogFile_fh.close(); \
              GetNextLogFileName(p_control); \
              p_control->LogFile_fh=SD.open( p_control->LogFileName, FILE_WRITE ); \
            } \
            if(p_control->LogFile_fh==NULL) \
            { \
              UART_LOG( "<LOG_PRINT::ERROR: Can't open file %s ... Disabling SD logging>", p_control->LogFileName ); \
              p_control->SDCardPresent = FALSE; \
            } \
            else \
            { \
              UART_LOG( "<LOG_PRINT::Logging to SD file %s>", p_control->LogFileName ); \
              p_control->LogFile_fh.print(p_control->LogBuffer); \
              p_control->LogFile_fh.flush(); \
              p_control->LogBuffer[0] = '\0'; \
              p_control->LogBufferLen = 0; \
            } \
          } \
          else \
          { \
            UART_LOG( "<LOG_PRINT::ERROR: Can't open file %s ... Disabling SD logging>", p_control->LogFileName ); \
            p_control->SDCardPresent = FALSE; \
          } \
        } \
      } \
      UART_LOG( _buffer_dat ); \
    }while(0)

  #else /* SD_LOGGING mode not enabled */
    
    #define LOG_PRINT(...) \
    do{ \
      char _buffer[MAX_LINE_LEN]; \
      sprintf(_buffer,__VA_ARGS__); \
      UART_PORT.println(_buffer); \
    }while(0)
    
  #endif /* End SD_LOGGING */
  
  #define SERIAL_PORT SERIAL_PORT_USBVIRTUAL
  #define SERIAL_PRINT SERIAL_PORT.print
  #define SERIAL_WRITE SERIAL_PORT.write
  #define SERIAL_AVAILABLE SERIAL_PORT.available()
  #define SERIAL_READ SERIAL_PORT.read()
  
#else /* EXE_MODE==1 , c executable code */

#endif







/* I2C Macros I2C addresses
******************************************************************/
#define WIRE_SEND(b) Wire.write((byte) b)
#define WIRE_RECEIVE() Wire.read()





