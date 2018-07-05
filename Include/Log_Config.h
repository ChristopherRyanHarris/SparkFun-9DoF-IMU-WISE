
/*******************************************************************
** FILE:
**    Log_Config.h
** DESCRIPTION:
**    The header contains all the logging functions, regardless
**    of platform. Note that the "communiction" functions typically
**    refer to the binary messaging type functions.
********************************************************************/

#ifndef LOGGING_CONFIG_H
#define LOGGING_CONFIG_H





/******************************************************************
** Logging Parameters
*******************************************************************/

#define ENABLE_SD_LOGGING 0
#define ENABLE_C_FILE_LOGGING 0 /* emulation mode */

#if( ENABLE_SD_LOGGING==1 )
  #ifndef _IMU9250_
    #error "SD logging is only available on the SEN-14001 (MPU-9250)! Set ENABLE_SD_LOGGING=0"
  #endif
  #if( EXE_MODE==1 ) /* Emulation Mode */
    #error "SD logging is only available on the SEN-14001 (MPU-9250)! Set ENABLE_SD_LOGGING=0. Use ENABLE_C_FILE_LOGGING instead"
  #endif 
#endif
#if( ENABLE_C_FILE_LOGGING==1 )
  #if( EXE_MODE==0 ) /* IMU Mode */
    #error "Local file logging (separate from SD logging) is not available in IMU Mode! Set ENABLE_C_FILE_LOGGING=0"
  #endif
#endif



/* Serial Port Configuration */
//#define UART_PORT_BAUD 115200
#define UART_PORT_BAUD 250000

//#define SERIAL_PORT_BAUD 9600
#define SERIAL_PORT_BAUD 250000

/* Data LOG period (us) */
#define DATA_LOG_RATE 1

/* The LED can be used for external debugging */
#define LED_BLINK_RATE 300

#define SHORT_LINE_LEN 50
#define MED_LINE_LEN 100
#define MAX_LINE_LEN 500

#define MAX_LOG_BUFFER_LEN POW_2_10

/* Data/Meta log packet version */
#define DATA_PACKET_VERSION 1


/******************************************************************
** SD/File Logging Parameters
*******************************************************************/
#define SD_PIN 38

#define LOG_FILE_MAX_IDX 999  /* Max number of "log.txt" files */

#define LOG_INFO_FILE_PREFIX "info_" /* Prefix name for log info files */
#define LOG_INFO_FILE_SUFFIX "txt"       /* Suffix name for log info files */

#define LOG_DATA_FILE_PREFIX "data_" /* Prefix name for log data files */
#define LOG_DATA_FILE_SUFFIX "bin"       /* Suffix name for log data files */

#define MAX_LOG_BUFFER_SIZE  POW_2_10 /* 1KB */
//#define MAX_LOG_BUFFER_STORE POW_2_9
#define MAX_LOG_BUFFER_STORE 1
#define MAX_OUTPUT_FILE_SIZE POW_2_30 /* 1 GB max file size */

#if( EXE_MODE==0 ) 
  /* 0 : IMU Mode */
  #define FILE_TYPE File
#else
  #define FILE_TYPE FILE*
#endif

/******************************************************************
** Logging Types
*******************************************************************/

/* Define the meta packet type */
typedef struct
{
	float version_id;
	float header_length;
	float collection_id;
	float collection_date;
	float platform_used;

	float data_quality;
	float collection_subject_id;
	float imu_position;
	float collection_env;
	float multiple_speed_flag;
	float speed;
	float mult_incline_flag;
	float incline_pct;

	float time_scale;
	float sample_rate_flag;
	float sample_rate_ave;
	float sample_rate_std;
	float number_of_samples;
	float number_of_elements_per_sample;
	float orientation_PO;
	float orientation_PRC;
	float orientation_RRC;
	float orientation_ZR;
}	LOG_META_PACKET_V1_TYPE;

/* Define meta packet type to use */
#if DATA_PACKET_VERSION==1
	#define META_PACKET_TYPE LOG_META_PACKET_V1_TYPE
#else
	#error "Meta Packet Is not Defined!"
#endif


typedef struct
{
	/* Common Log controls */
	FILE_TYPE LogFile_fh;
	int       LogFileIdx;
  char      LogFileName[MAX_LINE_LEN];
  int       LogBufferLen;
  char      LogBuffer[MAX_LOG_BUFFER_SIZE];

  char      file_suffix[SHORT_LINE_LEN];
  char      file_prefix[SHORT_LINE_LEN];
  
  /* type: 0:txt 1:bin */
  int       type;
  int       size;
  
  bool      enabled;
} OUTPUT_LOG_FILE_TYPE;

/******************************************************************
** Logging Macros
*******************************************************************/

#if( EXE_MODE==0 ) 
  /* 0 : IMU Mode 
  ** Writing/Reading from SD */
  #define FILE_EXISTS_FLAG(_fn)            (SD.exists(_fn))
  #define FILE_SIZE_BYTES(_fh)             (_fh.size())
  #define FILE_CLOSE(_fh)                  do{ if(_fh!=NULL){_fh.close();} }while(0)
  #define FILE_OPEN_WRITE(_fn)             (SD.open(_fn,FILE_WRITE))
  #define FILE_OPEN_WRITE_B(_fn)           (SD.open(_fn,FILE_WRITE))
  #define FILE_FLUSH(_fh)                  (_fh.flush())
  #define FILE_PRINT_TO_FILE(_fh,_s)       (_fh.print(_s))
  #define FILE_WRITE_TO_FILE(_b,_m,_n,_fh) (_fh.write(_b,(_m*_n)))
#else
  /* 1 : Emulation Mode 
  ** Writing/Reading from local file */
  #define FILE_EXISTS_FLAG(_fn)            (fopen(_fn,"r")!=NULL)
  #define FILE_SIZE_BYTES(_fh)             ftell(_fh)
  #define FILE_CLOSE(_fh)                  do{ if(_fh!=NULL){fclose(_fh);} }while(0)
  #define FILE_FLUSH(_fh)                  (fflush(_fh))
  #define FILE_OPEN_WRITE(_fn)             (fopen(_fn,"w"))
  #define FILE_OPEN_WRITE_B(_fn)           (fopen(_fn,"wb"))
  #define FILE_PRINT_TO_FILE(_fh,_s)       (fprintf(_fh,_s))
  #define FILE_WRITE_TO_FILE(_b,_m,_n,_fh) (fwrite(_b,_m,_n,_fh))
#endif


#define LOG_INFO_FAIL_STRING "<ERROR::FATAL FAIL>"
#define LOG_INFO_WARNING_STRING "<WARNING>"

#define DATA_READ_ASSERT( _n, _nmemb ) \
do{	\
	if( (_n)!=(_nmemb) ) \
	{ \
		LOG_INFO_FAIL("Read Failed! Expected:%d Read:%d",_nmemb,_n ); \
	} \
}while(0)

#define DATA_WRITE_ASSERT( _n, _nmemb ) \
do{	\
	if( (_n)!=(_nmemb) ) \
	{ \
		LOG_INFO_FAIL("Write Failed! Expected:%d Wrote:%d",_nmemb,_n ); \
	} \
}while(0)

#define DATA_READ( _addr, _sizeBytes, _nmemb, _fh ) \
do{ \
	int _c = fread( _addr, _sizeBytes, _nmemb, _fh ); \
  DATA_READ_ASSERT( _c, _nmemb ); \
}while(0)




#if EXE_MODE==0 /* 0 : IMU Mode */
  /* The debug port is simply the serial terminal */
  #define UART_PORT SERIAL_PORT_USBVIRTUAL

  /* Serial I/O */
  #define SERIAL_PORT  SERIAL_PORT_USBVIRTUAL
  #define SERIAL_PRINT SERIAL_PORT.print
  #define SERIAL_WRITE SERIAL_PORT.write
  #define SERIAL_READ  SERIAL_PORT.read()
  #define SERIAL_AVAILABLE SERIAL_PORT.available()

  /*
  ** Helpers & Simple Log calls
  */
  
  /* Print [func,line] */
  #define LOG_TAG_UART \
  do{ \
    char _buffer_log_tag_db[MAX_LINE_LEN]; \
    sprintf(_buffer_log_tag_db,"[%s:%d] ",__FUNCTION__,__LINE__); \
    UART_PORT.print(_buffer_log_tag_db); \
  }while(0)

  /* Print [long_func, line] */
  #define LOG_TAG_UART_LONG \
  do{ \
    char _buffer_log_tag_lg_db[MAX_LINE_LEN]; \
    sprintf(_buffer_log_tag_lg_db,"[%s:%d] ",__PRETTY_FUNCTION__,__LINE__); \
    UART_PORT.print(_buffer_log_tag_lg_db); \
  }while(0)

  /*
  ** Logging to SD
  */
  
  /* Record binary data */
  #define LOG_DATA_SD(_p,_n) \
  do{ \
    char _log_data_buff[MAX_LINE_LEN]; \
    memcpy( _log_data_buff, _p, _n ); \
    p_control->log_data_file.size = _n; \
    LogToFile( p_control, &p_control->log_data_file, _log_data_buff ); \
  }while(0)
  
  /* Print message to SD file with forced newline */
  #define LOG_INFO_SD(...) \
  do{ \
    char _buffer_dat[MAX_LINE_LEN]; \
    char _buffer_log_pnt_db[MAX_LINE_LEN]; \
    sprintf(_buffer_log_pnt_db,__VA_ARGS__); \
    sprintf( _buffer_dat, "[%s:%d] %s\n", __FUNCTION__,__LINE__, _buffer_log_pnt_db ); \
    LogToFile( p_control, &p_control->log_info_file, _buffer_dat ); \
  }while(0)

  /* Print failure message to SD file with forced newline */
  #define LOG_INFO_FAIL_SD(...) \
  do{ \
    char _buffer_dat[MAX_LINE_LEN]; \
    char _buffer_log_pnt_db[MAX_LINE_LEN]; \
    sprintf(_buffer_log_pnt_db,__VA_ARGS__); \
    sprintf( _buffer_dat, "[%s:%d] %s %s\n", __FUNCTION__,__LINE__, LOG_INFO_FAIL_STRING, _buffer_log_pnt_db ); \
    LogToFile( p_control, &p_control->log_info_file, _buffer_dat ); \
  }while(0)

  /* Print warning message to SD file with forced newline */
  #define LOG_INFO_WARNING_SD(...) \
  do{ \
    char _buffer_dat[MAX_LINE_LEN]; \
    char _buffer_log_pnt_db[MAX_LINE_LEN]; \
    sprintf(_buffer_log_pnt_db,__VA_ARGS__); \
    sprintf( _buffer_dat, "[%s:%d] %s %s\n", __FUNCTION__,__LINE__, LOG_INFO_WARNING_STRING, _buffer_log_pnt_db ); \
    LogToFile( p_control, &p_control->log_info_file, _buffer_dat ); \
  }while(0)
  
  /*
  ** Logging to UART 
  */
  
  /* Print contents to UART */
  #define LOG_UART_SIMPLE(...) UART_PORT.print(__VA_ARGS__)
  
  /* Print message to uart with forced newline */
  #define LOG_INFO_UART(...) \
  do{ \
    char _buffer[MAX_LINE_LEN]; \
    sprintf(_buffer,__VA_ARGS__); \
    LOG_TAG_UART; \
    UART_PORT.println(_buffer); \
    UART_PORT.flush(); \
  }while(0)

  /* Print failure message to uart with forced newline */
  #define LOG_INFO_FAIL_UART(...) \
  do{ \
    char _buffer[MAX_LINE_LEN]; \
    sprintf(_buffer,LOG_INFO_FAIL_STRING); \
    sprintf(_buffer," "); \
    sprintf(_buffer,__VA_ARGS__); \
    LOG_TAG_UART; \
    UART_PORT.println(_buffer); \
    UART_PORT.flush(); \
  }while(0)
  
  /* Print warning message to uart with forced newline */
  #define LOG_INFO_WARNING_UART(...) \
  do{ \
    char _buffer[MAX_LINE_LEN]; \
    sprintf(_buffer,LOG_INFO_WARNING_STRING); \
    sprintf(_buffer," "); \
    sprintf(_buffer,__VA_ARGS__); \
    LOG_TAG_UART; \
    UART_PORT.println(_buffer); \
    UART_PORT.flush(); \
  }while(0)

  /*
  ** Set logging calls from mode dependent functions
  */
  
  /* In the SEN-14001 platform, we have the
  ** ability to log to an sd card (if present) */
  #if( ENABLE_SD_LOGGING==TRUE ) /* Using SD Card */
    #define LOG_DATA          LOG_DATA_SD
    #define LOG_INFO          LOG_INFO_SD
    #define LOG_INFO_FAIL     LOG_INFO_FAIL_SD
    #define LOG_INFO_WARNING  LOG_INFO_WARNING_SD
  #else /* SD_LOGGING mode not enabled */
    #define LOG_DATA(...)     /* Do Nothing */
    #define LOG_INFO          LOG_INFO_UART
    #define LOG_INFO_FAIL     LOG_INFO_FAIL_UART
    #define LOG_INFO_WARNING  LOG_INFO_WARNING_UART
  
  #endif /* End SD_LOGGING Block */
  
  #define LOG_INFO_DEFAULT(...) LOG_UART_SIMPLE(__VA_ARGS__)
  

#else /* EXE_MODE==1 , c executable code */

  /* Mock Serial I/O */
  #define SERIAL_PRINT(...) /* Do Nothing */
  #define SERIAL_WRITE(...) /* Do Nothing */
  #define SERIAL_READ 0  /* Do Nothing */
  #define SERIAL_AVAILABLE 0 /* Do Nothing */

  /*
  ** Helpers
  */
  
  /*
  ** Logging to file
  */
  
  /* Record binary data */
  #define LOG_DATA_FILE(_p,_n) \
  do{ \
    char _log_data_buff[MAX_LINE_LEN]; \
    memcpy( _log_data_buff, _p, _n ); \
    p_control->log_data_file.size = _n; \
    LogToFile( p_control, &p_control->log_data_file, _log_data_buff ); \
  }while(0)
  
  /* Print message to file with forced newline */
  #define LOG_INFO_FILE(...) \
  do{ \
    char _buffer_dat[MAX_LINE_LEN]; \
    char _buffer_log_pnt_db[MAX_LINE_LEN]; \
    sprintf(_buffer_log_pnt_db,__VA_ARGS__); \
    sprintf( _buffer_dat, "[%s:%d] %s\n", __FUNCTION__,__LINE__, _buffer_log_pnt_db ); \
    LogToFile( p_control, &p_control->log_info_file, _buffer_dat ); \
  }while(0)

  /* Print failure message to file with forced newline */
  #define LOG_INFO_FAIL_FILE(...) \
  do{ \
    char _buffer_dat[MAX_LINE_LEN]; \
    char _buffer_log_pnt_db[MAX_LINE_LEN]; \
    sprintf(_buffer_log_pnt_db,__VA_ARGS__); \
    sprintf( _buffer_dat, "[%s:%d] %s %s\n", __FUNCTION__,__LINE__, LOG_INFO_FAIL_STRING, _buffer_log_pnt_db ); \
    LogToFile( p_control, &p_control->log_info_file, _buffer_dat ); \
  }while(0)

  /* Print warning message to file with forced newline */
  #define LOG_INFO_WARNING_FILE(...) \
  do{ \
    char _buffer_dat[MAX_LINE_LEN]; \
    char _buffer_log_pnt_db[MAX_LINE_LEN]; \
    sprintf(_buffer_log_pnt_db,__VA_ARGS__); \
    sprintf( _buffer_dat, "[%s:%d] %s %s\n", __FUNCTION__,__LINE__, LOG_INFO_WARNING_STRING, _buffer_log_pnt_db ); \
    LogToFile( p_control, &p_control->log_info_file, _buffer_dat ); \
  }while(0)

  /* 
  ** Logging to stdout 
  */
  
  /* Print contents to stdout */
  #define LOG_STDOUT_SIMPLE(...) fprintf( stdout, __VA_ARGS__ )

  /* Print message to file with forced newline */
  #define LOG_INFO_STDOUT(...) \
  do{ \
    char _buffer_log_pnt_db[MAX_LINE_LEN]; \
    sprintf(_buffer_log_pnt_db,__VA_ARGS__); \
    fprintf( stdout, "[%s:%d] %s\n", __FUNCTION__,__LINE__, _buffer_log_pnt_db ); \
  }while(0)

  /* Print failure message to file with forced newline */
  #define LOG_INFO_FAIL_STDOUT(...) \
  do{ \
    char _buffer_log_pnt_db[MAX_LINE_LEN]; \
    sprintf(_buffer_log_pnt_db,__VA_ARGS__); \
    fprintf( stdout, "[%s:%d] %s %s\n", __FUNCTION__,__LINE__, LOG_INFO_FAIL_STRING, _buffer_log_pnt_db ); \
  }while(0)

  /* Print warning message to file with forced newline */
  #define LOG_INFO_WARNING_STDOUT(...) \
  do{ \
    char _buffer_log_pnt_db[MAX_LINE_LEN]; \
    sprintf(_buffer_log_pnt_db,__VA_ARGS__); \
    fprintf( stdout, "[%s:%d] %s %s\n", __FUNCTION__,__LINE__, LOG_INFO_WARNING_STRING, _buffer_log_pnt_db ); \
  }while(0)
  
  /*
  ** Set logging calls from mode dependent functions
  */
  
  #if( ENABLE_C_FILE_LOGGING==1 )
    #define LOG_DATA         LOG_DATA_FILE
    #define LOG_INFO         LOG_INFO_FILE
    #define LOG_INFO_FAIL    LOG_INFO_FAIL_FILE
    #define LOG_INFO_WARNING LOG_INFO_WARNING_FILE
  #else /* Log to stdout */
    #define LOG_DATA(...)    /* Do Nothing */
    #define LOG_INFO         LOG_INFO_STDOUT
    #define LOG_INFO_FAIL    LOG_INFO_FAIL_STDOUT
    #define LOG_INFO_WARNING LOG_INFO_WARNING_STDOUT
  #endif /* End if( ENABLE_C_FILE_LOGGING==1 ) */
  
  #define LOG_INFO_DEFAULT(...) LOG_STDOUT_SIMPLE(__VA_ARGS__)
  
#endif /* End EXE_MODE==1 , c executable code */



#endif /* End LOG_CONFIG_H */



