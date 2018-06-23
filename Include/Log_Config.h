

/* Communication Parameters
*******************************************************************/
/* Serial Port Configuration */
//#define LOG_PORT_BAUD 115200
#define LOG_PORT_BAUD 250000
//#define COMM_PORT_BAUD 9600
#define COMM_PORT_BAUD 250000

/* Data LOG period (us) */
#define DATA_LOG_RATE 1

/* The LED can be used for external debugging */
#define UART_BLINK_RATE 300


#if EXE_MODE==0 /* IMU Mode */
  
  /* The debug port is simply the serial terminal */
  #define DB_PORT SERIAL_PORT_USBVIRTUAL
  
  /* debug log macros */
  #define LOG_DB_TAG \
  do{ \
    char _buffer_db[100]; \
    sprintf(_buffer_db,"[%s:%d] ",__FUNCTION__,__LINE__); \
    DB_PORT.print(_buffer_db); \
  }while(0)
  
  #define LOG_DB_TAG_LONG \
  do{ \
    char _buffer_db[100]; \
    sprintf(_buffer_db,"[%s:%d] ",__PRETTY_FUNCTION__,__LINE__); \
    DB_PORT.print(_buffer_db); \
  }while(0)
  
  #define DB_LOG \
  do{ \
    if(DEBUG_MODE) \
    { \
      char _buffer_db[100]; \
      LOG_DB_TAG;
      sprintf(_buffer_db,__VA_ARGS__); \
      DB_PORT.print(_buffer_db); \
    } \
  }while(0)
  
  #define DB_LOG_SHORT \
  do{ \
    if(DEBUG_MODE) \
    { \
      char _buffer_db[100]; \
      sprintf(_buffer_db,__VA_ARGS__); \
      DB_PORT.print(_buffer_db); \
    } \
  }while(0)
  
  #if ENABLE_SD_LOGGING==1
    
    #define LOG_PRINTLN(...) { char _buffer[100];sprintf(_buffer, __VA_ARGS__);LOG_PORT.println(_buffer);}
    
    #define LOG_PRINT(...) { \
      char _buffer_dat[100]; \
      sprintf(_buffer_dat,__VA_ARGS__); \
      DB_LOG(_buffer_dat); \
      if(p_control->SDCardPresent==TRUE) \
      { \
        p_control->LogBufferLen += strlen(_buffer_dat); \
        sprintf(p_control->LogBuffer,"%s%s",p_control->LogBuffer,_buffer_dat); \
        if(p_control->LogBufferLen>512) \
        { \
          sprintf(_buffer_db,"<Buffer len above thresh>"); \
          DB_LOG(_buffer_db); \
          p_control->LogFile_fh=SD.open( p_control->LogFileName, FILE_WRITE ); \
          if(p_control->LogFile_fh!=NULL) \
          { \
            sprintf(_buffer_db,"<Valid fh>"); \
            LOG_PORT.print(_buffer_db); \
            if(p_control->LogFile_fh.size()>SD_MAX_FILE_SIZE) \
            { \
              sprintf(_buffer_db,"<File reached max size, closing %s ... Getting new file name>",p_control->LogFileName); \
              LOG_PORT.print(_buffer_db); \
              p_control->LogFile_fh.close(); \
              GetNextLogFileName(p_control); \
              p_control->LogFile_fh=SD.open( p_control->LogFileName, FILE_WRITE ); \
            } \
            if(p_control->LogFile_fh==NULL) \
            { \
              sprintf(_buffer_db,"<ERROR: Can't open file %s ... Disabling SD logging>",p_control->LogFileName); \
              LOG_PORT.print(_buffer_db); \
              p_control->SDCardPresent = FALSE; \
            } \
            else \
            { \
              sprintf(_buffer_db,"<Logging to SD file %s>\n  >> Data being logged to SD : \n",p_control->LogFileName); \
              LOG_PORT.print(_buffer_db); \
              LOG_PORT.print(p_control->LogBuffer); \
              p_control->LogFile_fh.print(p_control->LogBuffer); \
              p_control->LogFile_fh.flush(); \
              p_control->LogFile_fh.close(); \
              sprintf(p_control->LogBuffer,'\0'); \
              p_control->LogBufferLen = 0; \
            } \
          } \
          else \
          { \
            sprintf(_buffer_db,"<ERROR: Can't open file %s ... Disabling SD logging>",p_control->LogFileName); \
            LOG_PORT.print(_buffer_db); \
            p_control->SDCardPresent = FALSE; \
          } \
        } \
      } \
    }
//        if(p_control->LogFile_fh!=NULL) \
//        { \
//          if(p_control->LogFile_fh.size()>SD_MAX_FILE_SIZE) \
//          { \
//            p_control->LogFile_fh.close(); \
//            GetNextLogFileName(p_control); \
//            p_control->LogFile_fh=SD.open( p_control->LogFileName, FILE_WRITE ); \
//          } \
//          SD_FH_VAR.print(_buffer); \
//          SD_FH_VAR.close(); \
//          p_control->LogBufferLen = 0; \
//        } \
//      } \
    
//    #define LOG_PRINTLN(...) { \
//      char _buffer[100]; \
//      char _buffern[100]; \
//      sprintf(_buffer,__VA_ARGS__); \
//      sprintf(_buffer,"%s\n",_buffer); \
//      LOG_PORT.print(_buffer); \
//      SD_FH_VAR=SD.open( SD_FN_VAR, FILE_WRITE ); \
//      if(SD_FH_VAR!=NULL) \
//      { \
//        if(SD_FH_VAR.size()>SD_MAX_FILE_SIZE) \
//        { \
//          sprintf(_buffern," <Closing Log File:%s>\n",p_control->LogFileName); \
//          LOG_PORT.print(_buffern); \
//          SD_FH_VAR.print(_buffern); \
//          SD_FH_VAR.close(); \
//          GetNextLogFileName(p_control); \
//          SD_FH_VAR=SD.open( p_control->LogFileName, FILE_WRITE ); \
//          sprintf(_buffern," <Opening Log File:%s>\n",p_control->LogFileName); \
//          LOG_PORT.print(_buffern); \
//          SD_FH_VAR.print(_buffern); \
//        } \
//        SD_FH_VAR.print(_buffer); \
//        SD_FH_VAR.close(); \
//      } \
//    }
  #else
    //#define LOG_PORT if(DEBUG)Serial
    #define LOG_DB_LN {char _buffer[100];sprintf(_buffer,"[%s:%d] : ", __FILE__, __LINE__);LOG_PORT.print(_buffer);}
    
    //#define LOG_PRINTLN LOG_PORT.println
    
    //#define LOG_PRINT LOG_PORT.print
    #define LOG_PRINT(...) { char _buffer[100];sprintf(_buffer,__VA_ARGS__);LOG_PORT.print(_buffer);}
    #define LOG_PRINTLN(...) { char _buffer[100];sprintf(_buffer, __VA_ARGS__);LOG_PORT.println(_buffer);}
  #endif
  
  #define COMM_PORT SERIAL_PORT_USBVIRTUAL
  #define COMM_PRINT COMM_PORT.print
  #define COMM_WRITE COMM_PORT.write
  #define COMM_AVAILABLE COMM_PORT.available()
  #define COMM_READ COMM_PORT.read()
#endif

/******************************************************************
** SD Logging Parameters
*******************************************************************/


#define SD_PIN 38

#define LOG_FILE_INDEX_MAX 999   /* Max number of "log.txt" files */
#define LOG_FILE_PREFIX "log"    /* Prefix name for log files */
#define LOG_FILE_SUFFIX "txt"    /* Suffix name for log files */
#define SD_MAX_FILE_SIZE 1048576 /* 1 MB max file size */