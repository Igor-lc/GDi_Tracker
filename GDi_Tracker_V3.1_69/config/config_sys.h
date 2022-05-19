

#ifndef _CONFIG_SYS_H
#define _CONFIG_SYS_H    

#define WATCH_DOG_ENABLE

#ifdef PCB_V2_1
#define PCB  PCB_V2_1
#define PCB_VER   0x0201//1.1
#endif

#if  defined PCB_V2_1
//OK
#else
#error PCB VERSION NOT DEFINED !!!
#endif	

#define MAIN_SERVER                   0
#define SERVICE_SERVER                1
#define SERVER_ANSWER_OK              1
#define SERVER_ANSWER_CRC_ERROR       3
#define SERVER_ANSWER_FORMAT_ERROR    4

//#define MAX_DOUT_COUNT             2
#define MAX_FIN_COUNT              2
#define MAX_RS485_SENSORS_COUNT    6
#define SENSOR_DATA_SIZE           7
#define MAX_RS485_SENSOR_TYPE_LEN  10
#define MAX_SENSORS_IN_MESSAGE     5
#define MAX_SMS_SIZE              161
#define MAX_SMS_PROCESSING        50
#define MAX_DEVNAME_STR_LEN       10
#define USSD_STR_LEN              7

//-----------------NAVIGATION---------------------
#define NAVIGATION_FILTER_DEFAULT     1
#define STORE_POINT_TIME_DEFAULT     300  
#define STORE_POINT_DIST_DEFAULT     300  //300 meters
#define STORE_POINT_ANGLE_DEFAULT    5    //5 degree
//----------------PACKET SEND----------------------------
#define MAX_PACKET_SEND_PRIO       3
#define PACKET_SEND_PRIO_FILTER_DEFAULT MAX_PACKET_SEND_PRIO //if (==0) ALL PRIO>0 WILL be sent immediately
#define PACKET_SEND_TIME_DEFAULT     30 // 1 sec
#define PACKET_SEND_DIST_DEFAULT     100 //100 meters
//-------------------------------------------------------

#define REGULAR_RESTART_TIMEOUT_MIN        5     //min
#define REGULAR_RESTART_TIMEOUT_MAX       (60*24)//min
#define REGULAR_RESTART_TIMEOUT_DEFAULT   (60*3) //min

#define DOUT_STATE_DEFAULT 0
#define CONFIG_UPDATE_TIMEOUT     30//TRY EVERY 30 min
#define POINTS_POOL_SIZE          5
#define MODEM_PARSER_TICK_PERIOD  2//2*5=10 ms
#define MODEM_RING_BUF_SIZE         2000
#define PARSER_MESSAGE_BUF_SIZE   400
#define PACKET_RX_BUF_SIZE  1500
#define MAX_FWU_PACKET_SIZE 1441
#define MAX_PHONE_STR_LEN   18
#define MAX_SERVER_STR_LEN 	32
#define MAX_GPRS_STR_LEN 	  32
#define MAX_APN_STR_LEN 	  32
#define MAX_DEVICE_NAME_LEN 20
#define MODEM_COMMAND_TX_BUF_SIZE 100
#define FRAM_RINGBUF_SIZE      0xC00//3072
//---------------------------------------------------
#define MAX_GPS_NAME_STR_LEN   11
//----------------------------------------------------------------------------------
#define USER_PHONE    ""
#define OLEG_PHONE    "\"+380506674904\""
#define IGOR_PHONE    "\"+380506674904\""
#define CONF_CURRENT_SIM_DEFAULT	1
//Gprs
#define GPRS1_USSD_BALANCE_DEFAULT	"*101#"
#define GPRS1_APN_DEFAULT   	"GPS"
//#define GPRS1_USSD_BALANCE_DEFAULT	"*100#"
//#define GPRS1_APN_DEFAULT   	"3g.utel.ua"
#define GPRS1_UNAME_DEFAULT 	""
#define GPRS1_PSW_DEFAULT     ""

#define GPRS2_USSD_BALANCE_DEFAULT	"*111#"
#define GPRS2_APN_DEFAULT   	"www.kyivstar.net"
#define GPRS2_UNAME_DEFAULT 	""
#define GPRS2_PSW_DEFAULT     ""

#define ORANGE_SERVER_PORT    20210
#define ORANGE_SERVER_IP     "193.193.165.166"
#define VIALON_SERVER_PORT    20129
#define VIALON_SERVER_IP      "77.123.137.100"
#define MY_SERVER_PORT        3333
#define MY_SERVER_IP          "178.150.20.13"
#define MTS_SERVER_PORT        1005
#define MTS_SERVER_IP          "178.218.169.57"

#ifdef OLEG
#define MAIN_SERVER_IP_DEFAULT      MY_SERVER_IP
#define MAIN_SERVER_PORT_DEFAULT    MY_SERVER_PORT
#else
#define MAIN_SERVER_IP_DEFAULT      MTS_SERVER_IP
#define MAIN_SERVER_PORT_DEFAULT    MTS_SERVER_PORT
#endif

#define SERVICE_SERVER_IP_DEFAULT   MY_SERVER_IP
#define SERVICE_SERVER_PORT_DEFAULT MY_SERVER_PORT

#define PROTOCOL_NTRACK        0
#define PROTOCOL_FMXXXX        1
//#define PROTOCOL_DEFAULT     PROTOCOL_NTRACK
#define PROTOCOL_DEFAULT       PROTOCOL_FMXXXX
#define SMS_PASSWORD_DEF       7777
#define RS485_ON_DEFAULT   1
#define DELTA_N_DEFAULT    10
#define DELTA_N_MIN        1
#define DELTA_N_MAX        100
#define RESET_SMS_DEFAULT  0
#define MODEM_STATE_HISTORY_SIZE 20

#define FWU_TRY_COUNT_MAX  5

//#define  APN_MTS      "www.umc.ua"
//#define  APN_UTEL     "3g.utel.ua"
//#define CONF_GPRS_APN   "www.kyivstar.net"
//#define CONF_GPRS_APN   "www.ab.kyivstar.net"

//#define DEBUG_NMEA_PARSER
#define DEBUG_MODEM	
#define DEBUG_MODEM_PARSER	
//#define DEBUG_SMS_PARSER	
//#define DEBUG_SIM900	
#define DEBUG_NAV_FILTER
//#define DEBUG_CONTROL
//#define DEBUG_DISK
//#define DEBUG_IO	

#endif

