

#ifndef _USER_TYPES_H
#define _USER_TYPES_H

#include <stdint.h>
#include "qp_port.h"
#include "config_sys.h"
#include "at45db.h"
#include "epsilon.h"
#include "rf10.h"
#include "tl10.h"


#define HEAD_PACK_FWU    0x62
#define HEAD_PACK_CONF   0x63
#define HEAD_PACK_AUTH   0x64
#define HEAD_PACK_ONLINE 0x65 
#define HEAD_PACK_ARCH   0x66

#define EVT_ID       0x80
#define WARNING_DATA_ID 29

#define CCR_ENABLE_Set          ((uint32_t)0x00000001)
#define CCR_ENABLE_Reset        ((uint32_t)0xFFFFFFFE)

typedef enum
{
	TRACKER_DEV_NAME=1,
	SECURITY_DEV_NAME,
	FUEL_METER_DEV_NAME,
}XtrackProtocolDeviceName;


typedef enum
{
	IO_OFF,//0
	IO_DIN,//1
	IO_DOUT,//2
	IO_AIN,//3
	IO_UNUSED,//4
	IO_ACCUM_COUNT,//5
	IO_RPM,//6
	IO_KMPH,//7
	IO_FREQ,//8
	IO_IDENTIFIER=11,
	IO_FUEL_SENSOR=12,
	IO_FUEL_AND_TEMPERATURE_SENSOR=13,
	
	
}IoModeType;

		 typedef	enum 
{
	no_nmea,
	no_fix,
	fix,
}NavigationStateType;

typedef enum
{
	SysEvtExternalPowerOff=1,
	SysEvtAccumDischarged,
	SysEvtIntervention,
	SysEvtIncorrectApn,
	SysEvtJammingGsm,
	SysEvtJammingGps,
	SysEvtLostFix,
	SysEvtFirstFix,
	SysEvtGpsAntOff,
	SysEvtGpsAntShort,
	SysEvtStopMove,
	SysEvtStartMove,
	SysEvtCorner,
}SysEvtEnum;

typedef enum
{
GetCmd=1,	
}ConfigCmdEnum;

typedef enum
{
	cmd_set,
	cmd_get,
}CommandType;


/*typedef struct
{
	unsigned fram:1;
}SendErrorSmsFlagsType;*/

typedef __packed struct
{
	char *fram;
	char *flash;
	char *system;
	char *restart;
}MsgPtrsType;


#define PACKET_HEAD_SIZE 4
#define FLASH_PAGE_ITEM_SIZE (DF_PAGE_SIZE-sizeof(uint32_t)-sizeof(uint8_t))

#undef  NULL
#define NULL          ((void *) 0)


#define NOP()         __NOP()
#define ToDigit(x)   (x-0x30)

#define CONCAT(FIRST, SECOND) FIRST ## SECOND
#define TO_STR(xx_)  #xx_
#define RED_STR(str_)     TO_STR(\x1b[31;1m##str_)
#define GREEN_STR(str_)   TO_STR(\x1b[32;1m##str_)
#define YELLOW_STR(str_)  TO_STR(\x1b[33;1m##str_)
#define BLUE_STR(str_)    TO_STR(\x1b[34;1m##str_)
#define MAGENTA_STR(str_) TO_STR(\x1b[35;1m##str_)
#define CYAN_STR(str_)    TO_STR(\x1b[36;1m##str_)
#define WHITE_STR(str_)   TO_STR(\x1b[37;1m##str_)

//typedef   void(*pf)(void);

enum PublicSignals
{
   TIC_100ms_SIG = Q_USER_SIG,//4
   MAX_PUB_SIG,               //5
};


#ifdef Q_SPY
 enum AppRecords
 {              
  QS_MODEM = QS_USER,//000
	QS_SIM900,         //001
	QS_MODEM_PARSER,   //002
	QS_SMS_PARSER,     //003
	QS_NAV_RECEIVER,   //004
	QS_NAV_FILTER,     //005
	QS_CONTROL,        //006
	QS_DISK,           //007
	QS_IO,             //008
 };
 
#endif

 
enum Prio
{
	NAV_FILTER_PRIO = 1,
	FDISK_PRIO,//2
	IO_PRIO,//3
  NAV_RECEIVER_PRIO,//4
	MODEM_PRIO,//5
	CONTROL_PRIO,//6
	MODEM_PARSER_PRIO,//7 
};

enum IrqPrio
{
	RS485_TIM_IRQ_PRIO = 0, //HIGH PRIO 
	RS485_UART_IRQ_PRIO,
	GPS_UART_IRQ_PRIO,
};

	 
	 typedef __packed struct 
   {
		 unsigned unused:19;
		 //-----------------------------
   unsigned restart_modem:1;
	 unsigned reboot_system:1;
	 unsigned save_config:1;
		 //--------------------------
	 unsigned get_sms_balance:1;
	 unsigned get_sms_config_gprs:1;
	 unsigned get_sms_config_nav:1;
	 unsigned get_sms_config_rs485:1;
	 unsigned get_sms_config_pack:1;
	 unsigned get_sms_ver:1;
	 unsigned get_sms_status:1;
	 unsigned get_sms_modem:1;
	 unsigned get_sms_imei:1;
	 unsigned break_upd:1;
   }FlagsType;
	 
	 typedef int (*cmd_handler)(char* data_ptr,FlagsType* flag_ptr); 
	 
	 typedef struct
{
	char *cmd_name;
	CommandType cmd_type;
	cmd_handler handler;
}ConfigCommandType;
	  	 
typedef __packed struct
{
	uint16_t count;
	uint16_t head;
	uint16_t tail;
	uint32_t crc;
}FlashPagesRingBufType;

 typedef __packed struct 
   {
   unsigned current_sim:1;
   unsigned move:1;
	 unsigned power_int:1;
   unsigned accel:1;
   unsigned gsm_reg:1;
   unsigned gps_fix:1;
	 unsigned gps_ant:1;
   unsigned angle:1;
   }StatusType;
	 
	 typedef __packed struct
	 {
		 unsigned internal:12; 
		 unsigned external:12; 
	 }VoltageType;
	 
	 typedef __packed union
   {
   uint8_t SatGsm;
   __packed struct
      {
		  unsigned  gsmq:4;
      unsigned sat_used:4;
      }sat_gsm;
   }SatGsmType;

  
	 typedef __packed struct 
   {
		uint32_t time;//4
   int32_t lat_int;//4
   int32_t lon_int;  //4
   uint8_t speed; //1
	 int16_t alt; //2	 
	 uint16_t angle; //2		 
   SatGsmType sg;//1
   uint8_t accel;  //1 
   VoltageType voltage;  //3 
   StatusType status; //1		 
   } QGpsDataType;//size =23 bytes

   typedef __packed struct 
	 {
		uint8_t any_data[SENSOR_DATA_SIZE]; 
	 }QSensorDataType;//7
	 
	  typedef __packed struct 
   {
   uint8_t id;		 //1
  __packed struct
      {
		  unsigned  data_size:3;//L
      unsigned data_id:5;//H
      }descriptor;//       1
   QSensorDataType data; //7
   }QSensorType;         //9
	 
		 
	typedef __packed struct
{
  QGpsDataType gps;                                  //21
  uint8_t sensors_count;	                           //1
	QSensorType sensors_array[MAX_SENSORS_IN_MESSAGE];//9*5=45
}QPointType;                                        //67

	typedef struct
{
   QEvt super;
   void *ptr;
	union
	{
	 uint32_t u32_data;
	 uint32_t size;
	FlagsType flag;
	}data;
} DataPtrEvt;//payload size=8

typedef struct
{
   QEvt super;
	QSensorType sensor;//9
} SensorEvt;//total size=10

typedef struct ModemEvtTag
{
    QEvent super;                                    
} SmallEvt;

	 
 //-----------------------------------------------------------
	  typedef __packed struct //FlashPage
   {
   uint32_t crc;//4
	 uint8_t points_count;//1	 
   uint8_t points[FLASH_PAGE_ITEM_SIZE];
   }FlashPageType; //528
	 
	
	 
	
typedef union
   {
   int32_t word;
   uint8_t byte[4]; 
   }WordByteType;
	 
 
	 typedef struct
   {
   char tf_num[MAX_PHONE_STR_LEN];
   char buf[MAX_SMS_SIZE];
   }SmsType;
	 
	typedef struct
   {
   QEvt super;
   void *sms_ptr;
   }SmsMessageType;

typedef union
   {
   int16_t word;
   uint8_t byte[2]; 
   }DoubleByteType;

	 
	 typedef struct
   {
   QEvt super;
   uint8_t gsmq;          
   }GsmqMessageType;

	 
	 typedef enum
{
point_not_stored,	
point_stop,
point_move_distance,
point_move_time,
point_move_angle,	
point_lost_fix,
}PointStoreEnum;
 

typedef struct 
   {
   uint32_t crc;
   uint32_t len;
   uint32_t start_addr;
   uint8_t position;         
   }FwType;

typedef struct
   {
   uint8_t *ptr;
   uint16_t size;
   }DataBufType;
	 
typedef __packed struct
   {
		 uint8_t save;
   char ip[MAX_SERVER_STR_LEN];//32
   uint16_t port; //2
	 uint16_t crc16;
   }ConfigServerType;//34
	 	 
	 typedef __packed struct
   {
		 uint8_t save;
   uint16_t time;//2
	 uint16_t dist; //2
	 uint8_t angle; //1
	 uint8_t filter_on;//1
		uint16_t crc16;//2
   }ConfigNavFilterType;
	 
	  typedef __packed struct
   {
		 uint8_t save;
   uint16_t time;
	 uint16_t dist;
   uint8_t prio_filter;		 
   uint16_t crc16;		 
   }ConfigPacketSendType;

typedef __packed struct
   {
		 uint8_t save;
	 char ussd_balance[USSD_STR_LEN];//7
   char apn   [MAX_GPRS_STR_LEN];//32
   char uname   [MAX_GPRS_STR_LEN];//32
   char psw  [MAX_GPRS_STR_LEN];//32
		 uint16_t crc16;
   }ConfigGprsType;//103
	 
	  typedef __packed struct
   {
		 uint8_t save;
   uint16_t regular_restart_timeout;
	 uint16_t fw_ver_new;
	 uint8_t protocol;	
   uint8_t send_reset_sms;		 
	 char user_phone[MAX_PHONE_STR_LEN]; 
   uint16_t crc16;
   }ConfigSecurType;
	 
	    typedef __packed struct
   {
		 uint8_t save;
		 uint8_t updTryCounter;
	   uint8_t updTryTimeout;
		 uint16_t regular_restart_counter;
		__packed struct
		 {
			 uint32_t restart;
       uint32_t nmea_err; 			 
		 }evt_counter;
		  uint16_t crc16;
	 }LogType;//4
	 
	 
	  typedef __packed struct
   {
		 uint8_t save;
		  char gps_name[MAX_GPS_NAME_STR_LEN]; 	
   uint16_t crc16;
   }ConfigHardwareType;
	 
	   typedef __packed struct
   {
		 uint8_t save;
		uint16_t deltaN; 
   uint16_t crc16;
   }ConfigRS485Type;
	 
	 typedef __packed struct
{
	uint8_t save;
	IoModeType mode[MAX_FIN_COUNT];
	uint16_t crc16;
}ConfigFinType;

 typedef __packed struct
{
	uint8_t save;
	uint8_t out_state;
	uint16_t crc16;
}ConfigDoutType;

   typedef __packed struct
   {
		 uint8_t save;
		uint8_t sim_num;
		 uint16_t crc16;
	 }ConfigSimType;
	 
	  typedef __packed struct
   {
		 ConfigGprsType gprs[2]; 
		 ConfigServerType server[2];
		 ConfigSecurType secur;
		 ConfigPacketSendType packet_send;
		 ConfigHardwareType hardware;
		 LogType log;
		 ConfigRS485Type rs485;
		 ConfigFinType fin;
		 ConfigDoutType dout;
		 ConfigSimType sim;
   }ConfigType;
	 
	typedef struct
		 {
			uint8_t view_gps; 
			uint8_t view_glo;
	    uint8_t used; 
	    uint8_t signal_max;
		 }SatType;
	
typedef  struct  
   {
		 uint32_t time;
   double latitude;
   double longitude;
   float altitude;
   float course;
   float speed;
   float pdop;
   float hdop;
   float vdop;
	 uint8_t accel;
   SatType sat;
   StatusType status;
	 unsigned pos_mode:2;  
   unsigned prio:2;		 
	 unsigned move:1; 
   }GpsDataExtendedType;
	 
	 
	  typedef __packed struct
   {
   uint32_t time;
	 uint32_t program_start_addr;
	 uint16_t point_store_timer;
	 PointStoreEnum point_store;
   uint8_t gsmq;
	 RF10DataType rf10_data;
	 TL10DataType tl10_data;
	 uint16_t NavSignalAndSat;
	 uint8_t nav_signal_average;	 
	 MsgPtrsType msg_ptr;
	 NavigationStateType nav_state;
	 uint16_t fuel_level_saved[MAX_RS485_SENSORS_COUNT];
  __packed struct 
      {
      unsigned unused:2;
		  unsigned din1:1;
			unsigned din2:1;
      unsigned power_ext:1;
      unsigned real_time:1;
      unsigned time_is_set:1;
		  unsigned fwu_processing:1;
      }flag; 
			
			__packed struct
			{
				float float_int;
				float float_ext;
			}voltage;
			__packed struct
			{
				uint32_t acc;
				uint16_t moment;
			}counter[2];
			
   }SysInfoType;
	 
	 
	 
	 
	  typedef struct 
      {
      unsigned unused:31;
      unsigned nav_filter:1;
      }WatchFlagsType; 
	 
	 
typedef __packed struct
   {
  QSensorDataType rs485[MAX_RS485_SENSORS_COUNT];
 
   }IoDataType;
	 

	 typedef __packed struct
{
__packed	struct
	{
	  uint16_t total;
		uint16_t used;
		uint16_t to_send;
	}size;//48
	
	__packed	struct
	{
	 	uint16_t count; 
		uint16_t to_send;
	}point;//32
	
	uint16_t tail;//2
	uint16_t head;//2
	uint8_t buf[FRAM_RINGBUF_SIZE];//3072
	uint8_t dummy;
	uint32_t crc;//4
}FramPointsRingBufType;//3072+88=3160

	 
	 typedef __packed struct
	 {
		 FramPointsRingBufType		points_buf; //3160
		 FlashPagesRingBufType    pages_buf;  //80
		 LogType log;
		 ConfigHardwareType conf_hardware;
		 ConfigNavFilterType conf_nav_filter;
		 ConfigPacketSendType conf_packet_send;
		 ConfigSecurType conf_secur; 
		 ConfigGprsType conf_gprs[2];                                   
		 ConfigServerType conf_server[2];
		 ConfigRS485SensorType conf_rs485_sensor[MAX_RS485_SENSORS_COUNT];//5*6=30 
     ConfigRS485Type conf_rs485;
     ConfigFinType conf_fin;
     ConfigDoutType conf_dout;	
     ConfigSimType conf_sim;		 
	 }FramMemoryType;	//3584		
	 
	 
typedef struct 
{
	uint8_t	prn;						
	uint8_t	signal_quality;				
	uint8_t	azimuth;					
	uint8_t	elevation;				
}SatInfoType;
 
	
typedef __packed struct
   {
   uint8_t dev_name;//1
   uint8_t dev_id[15];
   uint8_t sim_id[18];
   uint32_t password;//4
   }QLoginType;//38
  
	
	 typedef __packed struct
   {
   uint8_t id;
   uint16_t len;
   uint16_t crc;
   }XTDevPacketHeadType;//5
	 
	  typedef __packed struct
   {
   XTDevPacketHeadType head; 
    uint8_t payload[];
   }XTDevPacketType;
	 
 
typedef __packed struct
{	 
uint16_t pack_index;
uint16_t ver;
uint16_t max_pack_index;	
uint32_t size;	
uint32_t crc;	
}XTDevFwDescriptorPayloadType;

typedef __packed struct
{	 
uint16_t pack_index;
uint16_t ver;
 uint8_t fw_data[];
}XTDevFwDataType;


	  
//-------TELTONIKA START-------------------------	
	 
	 typedef __packed struct
   {
	 uint8_t head;
   uint8_t size;
   char imei[15];
   }PacketFM2200AuthType;
	 
	   	 typedef __packed struct
   {
		 uint32_t lon;
		 uint32_t lat;
		 int16_t alt;
		 uint16_t angle;
		 uint8_t sat;
		 uint16_t speed;
   }FM2200GpsElementType;
	 
	    	 typedef __packed struct
   {
		 uint8_t EventIoId;
		 uint8_t ioCountTotal;
   }FM2200IoElementType;
	 
	  typedef __packed struct
   {
		 uint64_t time;
		 uint8_t prio;
		 FM2200GpsElementType gpsElement;
		 FM2200IoElementType ioElement;
   }FM2200AvlDataType;

	  typedef __packed struct
   {
		  uint8_t codecId;
		 uint8_t dataCount;
		 FM2200AvlDataType data;
   }FM2200AvlDataArrayType;
	 
	 
typedef __packed struct
   {
		 uint32_t zero;
	 uint32_t len;
   FM2200AvlDataArrayType AvlDataArray;		 
   }FM2200AvlPacketType;
	 
//-------TELTONIKA END---------------	 
	 
typedef struct
   {
   QPointType *buf_ptr;
   uint8_t count;
   uint8_t size;
   uint8_t tail;
   uint8_t head;
   uint8_t send_count;
   }PointsRingBufType;
	 
	 typedef enum
    {
    FW_OK,
		FW_END, 
		FW_SIZE_ERROR,
		FW_FLASH_ERASE_PAGE_ERROR, 
    FW_FLASH_PROGRAM_ERROR, 
    FW_FLASH_TIMEOUT_ERROR,  
    FW_FLASH_WRP_ERROR, 
    FW_FLASH_BUSY_ERROR, 
//  FW_TEST_OK, 			
    }FlashWriteResultType;

#define CONFIG_PAGE_FREE_SPACE (DF_PAGE_SIZE-sizeof(ConfigGprsType)*2-sizeof(ConfigServerType)*2)
		
typedef __packed struct 
   {
	  ConfigGprsType gprs[2]; 
		 ConfigServerType server[2];
   uint8_t points[CONFIG_PAGE_FREE_SPACE];
   }ConfigPageType; //528
		
		
extern WatchFlagsType watch_flag;
extern SysInfoType SysInfo;	 
extern const uint8_t FwVer[];
	 
	 
#endif// _USER_TYPES_H

