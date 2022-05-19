

#include "ParserSms.h"
#include "user_types.h"
#include <string.h>
#include <ctype.h>
#include "modem.h"
#include "crc16.h"
#include <stdio.h>
#include <stdlib.h>
#include "io.h"
#include "Control.h"
#include "stm32f10x.h"

static uint8_t Config_obj;

const ConfigGprsType config_gprs_default[2]=
{
   {
      0,//save
      GPRS1_USSD_BALANCE_DEFAULT,
      GPRS1_APN_DEFAULT,
      GPRS1_UNAME_DEFAULT,
      GPRS1_PSW_DEFAULT,
      0//crc
   },
   {
      0,//save
      GPRS2_USSD_BALANCE_DEFAULT,
      GPRS2_APN_DEFAULT,
      GPRS2_UNAME_DEFAULT,
      GPRS2_PSW_DEFAULT,
      0//crc
   }
};

const ConfigServerType config_server_default[2]=
{
   {
      0,//save
      MAIN_SERVER_IP_DEFAULT,
      MAIN_SERVER_PORT_DEFAULT ,
      0//crc
   },
   {
      0,//save
      SERVICE_SERVER_IP_DEFAULT,
      SERVICE_SERVER_PORT_DEFAULT ,
      0//crc
   }
};

ConfigGprsType *gprs_ptr;
ConfigNavFilterType conf_nav_filter;
ConfigRS485SensorType conf_rs485_sensor[MAX_RS485_SENSORS_COUNT];
ConfigType conf;


static int Set1FinMode(char* data_ptr,FlagsType *flag_ptr);
static int Set2FinMode(char* data_ptr,FlagsType *flag_ptr);
static int SetGprsApn1(char* data_ptr,FlagsType *flag_ptr);
static int SetGprsUname1(char* data_ptr,FlagsType *flag_ptr);
static int SetGprsPsw1(char* data_ptr,FlagsType *flag_ptr);
static int SetGprsApn2(char* data_ptr,FlagsType *flag_ptr);
static int SetGprsUname2(char* data_ptr,FlagsType *flag_ptr);
static int SetGprsPsw2(char* data_ptr,FlagsType *flag_ptr);
static int SetMainIp(char* data_ptr,FlagsType *flag_ptr);
static int SetMainPort(char* data_ptr,FlagsType *flag_ptr);
static int SetServiceIp(char* data_ptr,FlagsType *flag_ptr);
static int SetServicePort(char* data_ptr,FlagsType *flag_ptr);
static int Set1RS485Dev(char* data_ptr,FlagsType *flag_ptr);
static int Set2RS485Dev(char* data_ptr,FlagsType *flag_ptr);
static int Set3RS485Dev(char* data_ptr,FlagsType *flag_ptr);
static int Set4RS485Dev(char* data_ptr,FlagsType *flag_ptr);
static int Set5RS485Dev(char* data_ptr,FlagsType *flag_ptr);
static int Set6RS485Dev(char* data_ptr,FlagsType *flag_ptr);
static int Set1RS485Addr(char* data_ptr,FlagsType *flag_ptr);
static int Set2RS485Addr(char* data_ptr,FlagsType *flag_ptr);
static int Set3RS485Addr(char* data_ptr,FlagsType *flag_ptr);
static int Set4RS485Addr(char* data_ptr,FlagsType *flag_ptr);
static int Set5RS485Addr(char* data_ptr,FlagsType *flag_ptr);
static int Set6RS485Addr(char* data_ptr,FlagsType *flag_ptr);
static int Set1RS485Out(char* data_ptr,FlagsType *flag_ptr);
static int Set2RS485Out(char* data_ptr,FlagsType *flag_ptr);
static int Set3RS485Out(char* data_ptr,FlagsType *flag_ptr);
static int Set4RS485Out(char* data_ptr,FlagsType *flag_ptr);
static int Set5RS485Out(char* data_ptr,FlagsType *flag_ptr);
static int Set6RS485Out(char* data_ptr,FlagsType *flag_ptr);
static int SetPointStoreTime(char* data_ptr,FlagsType *flag_ptr);
static int SetPointStoreAngle(char* data_ptr,FlagsType *flag_ptr);
static int SetPointStoreDist(char* data_ptr,FlagsType *flag_ptr);
static int SetPacketSendTime(char* data_ptr,FlagsType *flag_ptr);
static int SetPacketSendDist(char* data_ptr,FlagsType *flag_ptr);
static int SetPacketPrioFilter(char* data_ptr,FlagsType *flag_ptr);
static int SetUssdBalance1(char* data_ptr,FlagsType *flag_ptr);
static int SetUssdBalance2(char* data_ptr,FlagsType *flag_ptr);
static int GetBalance(char* data_ptr,FlagsType *flag_ptr);
static int GetGprs(char* data_ptr,FlagsType *flag_ptr);
static int SetNavFilter(char* data_ptr,FlagsType *flag_ptr);
static int GetNav(char* data_ptr,FlagsType *flag_ptr);
static int GetRS485(char* data_ptr,FlagsType *flag_ptr);
static int GetPack(char* data_ptr,FlagsType *flag_ptr);
static int GetImei(char* data_ptr,FlagsType *flag_ptr);
static int Config(char* data_ptr,FlagsType *flag_ptr);
static int Reboot(char* data_ptr,FlagsType *flag_ptr);
static int SetProtocol(char* data_ptr,FlagsType *flag_ptr);
static int SetSim(char* data_ptr,FlagsType *flag_ptr);
static int SetFwVerNew(char* data_ptr,FlagsType *flag_ptr);
static int GetVer(char* data_ptr,FlagsType *flag_ptr);
static int GetStatus(char* data_ptr,FlagsType *flag_ptr);
static int GetModem(char* data_ptr,FlagsType *flag_ptr);
static int Rrt(char* data_ptr,FlagsType *flag_ptr);
static int DutDeltaN(char* data_ptr,FlagsType *flag_ptr);
static char* StrToUper(char *str);
static int CheckRS485DeviceName(uint8_t name);
static int CheckFinMode(uint8_t mode);
//static int DutLastN(char* data_ptr,FlagsType *flag_ptr);
static int Start(char* data_ptr,FlagsType *flag_ptr);
static int Stop(char* data_ptr,FlagsType *flag_ptr);


ConfigCommandType sms_cmd_tbl[]=
{
   //--------------GET--------------------------------
   {"RS485",cmd_get,GetRS485},//
   {"NAV",cmd_get,GetNav},
   {"PACK",cmd_get,GetPack},
   {"VER",cmd_get,GetVer},
   {"GPRS",cmd_get,GetGprs},
   {"BAL",cmd_get,GetBalance},
   {"STATUS",cmd_get,GetStatus},
   {"MODEM",cmd_get,GetModem},
   {"IMEI",cmd_get,GetImei},
   //--------------GPRS1---------------------------
   {"APN1=",cmd_set,SetGprsApn1},
   {"UNAME1=",cmd_set,SetGprsUname1},
   {"PSW1=",cmd_set,SetGprsPsw1},
   {"B1=",cmd_set,SetUssdBalance1},
   //--------------GPRS2---------------------------
   {"APN2=",cmd_set,SetGprsApn2},
   {"UNAME2=",cmd_set,SetGprsUname2},
   {"PSW2=",cmd_set,SetGprsPsw2},
   {"B2=",cmd_set,SetUssdBalance2},
   //---------MAIN SERVER----------
   {"IP=",cmd_set,SetMainIp},
   {"PORT=",cmd_set,SetMainPort},
   //---------SERVICE SERVER-------
   {"SIP=",cmd_set,SetServiceIp},
   {"SPORT=",cmd_set,SetServicePort},
   //---------SET NAVIGATION-------------------
   {"STORE TIME=",cmd_set,SetPointStoreTime},
   {"STORE DIST=",cmd_set,SetPointStoreDist},
   {"STORE ANGLE=",cmd_set,SetPointStoreAngle},
   {"NAV FILTER=",cmd_set,SetNavFilter},// value is ON or OFF
   //-----------SEND PACKET
   {"SEND TIME=",cmd_set,SetPacketSendTime},
   {"SEND DIST=",cmd_set,SetPacketSendDist},
	 {"SEND PRIO FILTER=",cmd_set,SetPacketPrioFilter},

   {"FW VER NEW=",cmd_set,SetFwVerNew},
   {"CONFIG=",cmd_set,Config},
	 {"RRT=",cmd_set,Rrt},      
	 {"REBOOT=",cmd_set,Reboot},//value is 1 only
   {"PROTOCOL=",cmd_set,SetProtocol},//FM or NT
   {"SIM=",cmd_set,SetSim}, //1 or 2
   //--------------------------------------	 
//	 {"SMS RESET=",cmd_set,SmsReset},
//	 {"SMS FRAM ERR=",cmd_set,SmsFramError},
	 //---------------------------------------
	  {"START",cmd_set,Start},
		{"STOP",cmd_set,Stop},
	 //--------------------------------------------
	 {"1FIN MODE=",cmd_set,Set1FinMode},//correct values is 0,1,6,
	 {"2FIN MODE=",cmd_set,Set2FinMode},//correct values is 0,1,6,
   //---------SET RS485 SENSORS-------------------
	 //{"DUT LASTN=",cmd_set,DutLastN},
	 {"DUT DELTAN=",cmd_set,DutDeltaN},
   {"1RS485DEV=",cmd_set,Set1RS485Dev},
   {"2RS485DEV=",cmd_set,Set2RS485Dev},
   {"3RS485DEV=",cmd_set,Set3RS485Dev},
   {"4RS485DEV=",cmd_set,Set4RS485Dev},
	 {"5RS485DEV=",cmd_set,Set5RS485Dev},
	 {"6RS485DEV=",cmd_set,Set6RS485Dev},
	 
	 {"1RS485ADDR=",cmd_set,Set1RS485Addr},
	 {"2RS485ADDR=",cmd_set,Set2RS485Addr},
	 {"3RS485ADDR=",cmd_set,Set3RS485Addr},
	 {"4RS485ADDR=",cmd_set,Set4RS485Addr},
	 {"5RS485ADDR=",cmd_set,Set5RS485Addr},
	 {"6RS485ADDR=",cmd_set,Set6RS485Addr},
	 
   {"1RS485OUT=",cmd_set,Set1RS485Out},
   {"2RS485OUT=",cmd_set,Set2RS485Out},
   {"3RS485OUT=",cmd_set,Set3RS485Out},
   {"4RS485OUT=",cmd_set,Set4RS485Out},
	 {"5RS485OUT=",cmd_set,Set5RS485Out},
   {"6RS485OUT=",cmd_set,Set6RS485Out},
   {NULL,cmd_get,NULL},//NULL is end of table
};


static char* OnOffIntToStr(uint8_t val)
   {
   return(val==0)?"OFF":"ON";
   }

static int32_t OnOffStrToInt(char* val)
   {
		 int32_t ret;
		 val=StrToUper(val);
   if (0==strcmp(val,"ON"))ret= 1;
   else if (0==strcmp(val,"OFF"))ret= 0;
   else ret = -1;
	 return ret;
   }

int SetProtocol(char* data_ptr,FlagsType *flag_ptr)
   {
   char *protocol=StrToUper(data_ptr);
   if (0==strcmp(protocol,"FM"))
      {
      parser_sms_answer.flag.restart_modem=1;
      conf.secur.protocol=PROTOCOL_FMXXXX;
      conf.secur.save=1;
      return 1;
      }
   else if (0==strcmp(protocol,"NT"))
      {
      parser_sms_answer.flag.restart_modem=1;
      conf.secur.protocol=PROTOCOL_NTRACK;
      conf.secur.save=1;
      return 1;
      }
			else return 0;
   }

int SetSim(char* data_ptr,FlagsType *flag_ptr)
   {
   uint8_t temp=(uint8_t)atoi(data_ptr);
	 if (temp==1)conf.sim.sim_num=temp;//sim1
	 else if (temp==2 )conf.sim.sim_num=temp;//sim2
		 else conf.sim.sim_num=0;//auto mode
      
      parser_sms_answer.flag.restart_modem=1;
      conf.secur.save=1;
		  return 1;
   }
	 
	
	 
	 

//------------MAIN SERVER--------------------------------
int SetMainIp(char* data_ptr,FlagsType *flag_ptr)
   {
   strlcpy(conf.server[MAIN_SERVER].ip, data_ptr,MAX_APN_STR_LEN);
   parser_sms_answer.flag.restart_modem=1;
   conf.server[MAIN_SERVER].save=1;
   return 1;
   }

int SetMainPort(char* data_ptr,FlagsType *flag_ptr)
   {
   uint32_t port=atoi(data_ptr);
   if (port<=UINT16_MAX )
      {
      conf.server[MAIN_SERVER].port=(uint16_t)port;
      parser_sms_answer.flag.restart_modem=1;
      conf.server[MAIN_SERVER].save=1;
      return 1;
      }
			return 0;
   }
//--------------SERVICE SERVER----------------------
int SetServiceIp(char* data_ptr,FlagsType *flag_ptr)
   {
   strlcpy(conf.server[SERVICE_SERVER].ip, data_ptr,MAX_APN_STR_LEN);
   parser_sms_answer.flag.restart_modem=1;
   conf.server[SERVICE_SERVER].save=1;
   return 1;
   }

int SetServicePort(char* data_ptr,FlagsType *flag_ptr)
   {
   uint32_t port=atoi(data_ptr);
   if (port<=UINT16_MAX )
      {
      conf.server[SERVICE_SERVER].port=(uint16_t)port;
      parser_sms_answer.flag.restart_modem=1;
      conf.server[SERVICE_SERVER].save=1;
      return 1;
      }
			return 0;
   }
	 
	 
	 //------------FIN--------------------------------------------------
int Set1FinMode(char* data_ptr,FlagsType *flag_ptr)
   {
   uint8_t mode=(uint8_t)atoi(data_ptr);
   if (CheckFinMode(mode))
      {
      conf.fin.mode[0]=(IoModeType)mode;
      conf.fin.save=1;
      return 1;
      }
		else return 0;
   }
	 
	 int Set2FinMode(char* data_ptr,FlagsType *flag_ptr)
   {
   uint8_t mode=(uint8_t)atoi(data_ptr);
   if (CheckFinMode(mode))
      {
      conf.fin.mode[1]=(IoModeType)mode;
      conf.fin.save=1;
      return 1;
      }
		else return 0;
   }
	 
//---------DOUT--------------------------------------
int Start(char* data_ptr,FlagsType *flag_ptr)
   {
      conf.dout.out_state=0;
      conf.dout.save=1;
			DOUT1_OFF();
      return 1;
   }
	 
	 int Stop(char* data_ptr,FlagsType *flag_ptr)
   {
      conf.dout.out_state=1;
      conf.dout.save=1;
			DOUT1_ON();
      return 1;
   }
	 
	 


//------------RS485--------------------------------------------------
int Set1RS485Dev(char* data_ptr,FlagsType *flag_ptr)
   {
   uint8_t device=(uint8_t)atoi(data_ptr);
   if (CheckRS485DeviceName(device))
      {
      conf_rs485_sensor[0].device=device;
      conf_rs485_sensor[0].save=1;
			//conf.log_rs485_data[0].data=0;
			//conf.log_rs485_data[0].save=1;
      return 1;
      }
			return 0;
   }

int Set2RS485Dev(char* data_ptr,FlagsType *flag_ptr)
   {
   uint8_t device=(uint8_t)atoi(data_ptr);
   if (CheckRS485DeviceName(device))
      {
      conf_rs485_sensor[1].device=device;
      conf_rs485_sensor[1].save=1;
			//conf.log_rs485_data[1].data=0;
			//conf.log_rs485_data[1].save=1;
      return 1;
      }
			return 0;
   }
	 
int Set3RS485Dev(char* data_ptr,FlagsType *flag_ptr)
   {
   uint8_t device=(uint8_t)atoi(data_ptr);
   if (CheckRS485DeviceName(device))
      {
      conf_rs485_sensor[2].device=device;
      conf_rs485_sensor[2].save=1;
		  //conf.log_rs485_data[2].data=0;
			//conf.log_rs485_data[2].save=1;
      return 1;
      }
			return 0;
   }

int Set4RS485Dev(char* data_ptr,FlagsType *flag_ptr)
   {
   uint8_t device=(uint8_t)atoi(data_ptr);
   if (CheckRS485DeviceName(device))
      {
      conf_rs485_sensor[3].device=device;
      conf_rs485_sensor[3].save=1;
			//conf.log_rs485_data[3].data=0;
			//conf.log_rs485_data[3].save=1;
      return 1;
      }
			return 0;
   }  

	 
	 int Set5RS485Dev(char* data_ptr,FlagsType *flag_ptr)
   {
   uint8_t device=(uint8_t)atoi(data_ptr);
   if (CheckRS485DeviceName(device))
      {
      conf_rs485_sensor[4].device=device;
      conf_rs485_sensor[4].save=1;
			//conf.log_rs485_data[3].data=0;
			//conf.log_rs485_data[3].save=1;
      return 1;
      }
			return 0;
   }  
	 
	 int Set6RS485Dev(char* data_ptr,FlagsType *flag_ptr)
   {
   uint8_t device=(uint8_t)atoi(data_ptr);
   if (CheckRS485DeviceName(device))
      {
      conf_rs485_sensor[5].device=device;
      conf_rs485_sensor[5].save=1;
			//conf.log_rs485_data[3].data=0;
			//conf.log_rs485_data[3].save=1;
      return 1;
      }
			return 0;
   }  
	 
	 
	 /*****************************************************************/
	 int Set1RS485Addr(char* data_ptr,FlagsType *flag_ptr)
   {
      uint8_t addr=(uint8_t)atoi(data_ptr);
      conf_rs485_sensor[0].addr=addr;
      conf_rs485_sensor[0].save=1;
      return 1;
   }
	 
	  int Set2RS485Addr(char* data_ptr,FlagsType *flag_ptr)
   {
      uint8_t addr=(uint8_t)atoi(data_ptr);
      conf_rs485_sensor[1].addr=addr;
      conf_rs485_sensor[1].save=1;
      return 1;
   }
	 
	  int Set3RS485Addr(char* data_ptr,FlagsType *flag_ptr)
   {
      uint8_t addr=(uint8_t)atoi(data_ptr);
      conf_rs485_sensor[2].addr=addr;
      conf_rs485_sensor[2].save=1;
      return 1;
   }
	 
	  int Set4RS485Addr(char* data_ptr,FlagsType *flag_ptr)
   {
      uint8_t addr=(uint8_t)atoi(data_ptr);
      conf_rs485_sensor[3].addr=addr;
      conf_rs485_sensor[3].save=1;
      return 1;
   }
	 
	  int Set5RS485Addr(char* data_ptr,FlagsType *flag_ptr)
   {
      uint8_t addr=(uint8_t)atoi(data_ptr);
      conf_rs485_sensor[4].addr=addr;
      conf_rs485_sensor[4].save=1;
      return 1;
   }
	 
	  int Set6RS485Addr(char* data_ptr,FlagsType *flag_ptr)
   {
      uint8_t addr=(uint8_t)atoi(data_ptr);
      conf_rs485_sensor[5].addr=addr;
      conf_rs485_sensor[5].save=1;
      return 1;
   }
	 
	 
//-----------------------------------------------------------
int Set1RS485Out(char* data_ptr,FlagsType *flag_ptr)
   {
   int32_t temp=OnOffStrToInt(data_ptr);
   if (temp!=-1)
      {
      conf_rs485_sensor[0].out_state= temp;
      conf_rs485_sensor[0].save=1;
      return 1;
      }
			return 0;
   }

int Set2RS485Out(char* data_ptr,FlagsType *flag_ptr)
   {
   int32_t temp=OnOffStrToInt(data_ptr);
   if (temp!=-1)
      {
      conf_rs485_sensor[1].out_state= temp;
      conf_rs485_sensor[1].save=1;
      return 1;
      }
			return 0;
   }

int Set3RS485Out(char* data_ptr,FlagsType *flag_ptr)
   {
   int32_t temp=OnOffStrToInt(data_ptr);
   if (temp!=-1)
      {
      conf_rs485_sensor[2].out_state= temp;
      conf_rs485_sensor[2].save=1;
      return 1;
      }
			return 0;
   }

int Set4RS485Out(char* data_ptr,FlagsType *flag_ptr)
   {
   int32_t temp=OnOffStrToInt(data_ptr);
   if (temp!=-1)
      {
      conf_rs485_sensor[3].out_state= temp;
      conf_rs485_sensor[3].save=1;
      return 1;
      }
			 return 0;
   }
	 
	 int Set5RS485Out(char* data_ptr,FlagsType *flag_ptr)
   {
   int32_t temp=OnOffStrToInt(data_ptr);
   if (temp!=-1)
      {
      conf_rs485_sensor[4].out_state= temp;
      conf_rs485_sensor[4].save=1;
      return 1;
      }
			 return 0;
   }
	 
	  int Set6RS485Out(char* data_ptr,FlagsType *flag_ptr)
   {
   int32_t temp=OnOffStrToInt(data_ptr);
   if (temp!=-1)
      {
      conf_rs485_sensor[5].out_state= temp;
      conf_rs485_sensor[5].save=1;
      return 1;
      }
			 return 0;
   }

	 
	  int DutDeltaN(char* data_ptr,FlagsType *flag_ptr)
   {
   int32_t temp=atoi(data_ptr);
   if (temp<=DELTA_N_MAX&&temp>=DELTA_N_MIN)
      {
      conf.rs485.deltaN=temp;
      conf.rs485.save=1;
      return 1;
      }
			 return 0;
   }
	 
	/* int DutLastN(char* data_ptr,FlagsType *flag_ptr)
   {
   int32_t temp=OnOffStrToInt(data_ptr);
    if (temp!=-1)
      {
      //conf.rs485.send_last_n=temp;
      conf.rs485.save=1;
      return 1;
      }
		else return 0;
   }*/
	 
	

int SetFwVerNew(char* data_ptr,FlagsType *flag_ptr)
   {
   uint32_t fw_ver_new=atoi(data_ptr);
   if (fw_ver_new<=UINT16_MAX)
      {
      conf.secur.fw_ver_new=(uint16_t)fw_ver_new;
      conf.secur.save=1;
      return 1;
      }
		return 0;
   }

//----------NAV FILTER------------------------------------------------------	 
int SetPointStoreAngle(char* data_ptr,FlagsType *flag_ptr)
   {
   uint32_t angle=atoi(data_ptr);
   if (angle<=180)
      {
      conf_nav_filter.angle=angle;
      conf_nav_filter.save=1;
      return 1;
      }
			 return 0;
   }

int SetPointStoreTime(char* data_ptr,FlagsType *flag_ptr)
   {
   uint32_t time=atoi(data_ptr);
   if (time<=UINT16_MAX )
      {
      conf_nav_filter.time=time;
      conf_nav_filter.save=1;
      return 1;
      }
			return 0;
   }

int SetPointStoreDist(char* data_ptr,FlagsType *flag_ptr)
   {
   uint32_t dist=atoi(data_ptr);
   if (dist<=UINT16_MAX )
      {
      conf_nav_filter.dist=dist;
      conf_nav_filter.save=1;
      return 1;
      }
			return 0;
   }

int SetNavFilter(char* data_ptr,FlagsType *flag_ptr)
   {
   int32_t temp=OnOffStrToInt(data_ptr);
   if (temp!=-1)
      {
      conf_nav_filter.filter_on=temp;
      conf_nav_filter.save=1;
      return 1;
      }
			return 0;
   }
//----------------SEND PACKET----------------------------------
int SetPacketSendTime(char* data_ptr,FlagsType *flag_ptr)
   {
   uint32_t time=atoi(data_ptr);
   if (time<=UINT16_MAX )
      {
      conf.packet_send.time=time;
      conf.packet_send.save=1;
      return 1;
      }
			return 0;
   }

int SetPacketSendDist(char* data_ptr,FlagsType *flag_ptr)
   {
   uint32_t dist=atoi(data_ptr);
   if (dist<=UINT16_MAX )
      {
      conf.packet_send.dist=dist;
      conf.packet_send.save=1;
      return 1;
      }
			return 0;
   }
	 
	 int SetPacketPrioFilter(char* data_ptr,FlagsType *flag_ptr)
   {
   uint32_t prio=atoi(data_ptr);
   if (prio<=MAX_PACKET_SEND_PRIO )
      {
      conf.packet_send.prio_filter=prio;
      conf.packet_send.save=1;
      return 1;
      }
			return 0;
   }
//--------------GPRS1-------------------------------------------
int SetUssdBalance1(char* data_ptr,FlagsType *flag_ptr)
   {
   strncpy(conf.gprs[0].ussd_balance, data_ptr,USSD_STR_LEN);
   conf.gprs[0].save=1;
   return 1;
   }

int SetGprsApn1(char* data_ptr,FlagsType *flag_ptr)
   {
		 char temp[50];
		 sprintf(temp,"SetGprsApn1 command=%s",data_ptr);
		  QS_BEGIN(QS_SMS_PARSER, &Config_obj)                                 
   QS_STR(temp);                              
   QS_END()
   strlcpy(conf.gprs[0].apn, data_ptr,MAX_APN_STR_LEN);
   conf.gprs[0].save=1;
   parser_sms_answer.flag.restart_modem=1;
   return 1;
   }

int SetGprsUname1(char* data_ptr,FlagsType *flag_ptr)
   {
   strlcpy(conf.gprs[0].uname, data_ptr,MAX_APN_STR_LEN);
   conf.gprs[0].save=1;
   parser_sms_answer.flag.restart_modem=1;
   return 1;
   }
	 
int SetGprsPsw1(char* data_ptr,FlagsType *flag_ptr)
   {
   strlcpy(conf.gprs[0].psw, data_ptr,MAX_APN_STR_LEN);
   conf.gprs[0].save=1;
   parser_sms_answer.flag.restart_modem=1;
   return 1;
   }
//-----------------GPRS2--------------------------------------	 
int SetUssdBalance2(char* data_ptr,FlagsType *flag_ptr)
   {
   strncpy(conf.gprs[1].ussd_balance, data_ptr,USSD_STR_LEN);
   conf.gprs[1].save=1;
   return 1;
   }

int SetGprsApn2(char* data_ptr,FlagsType *flag_ptr)
   {
   strlcpy(conf.gprs[1].apn, data_ptr,MAX_APN_STR_LEN);
   conf.gprs[1].save=1;
   parser_sms_answer.flag.restart_modem=1;
   return 1;
   }

int SetGprsUname2(char* data_ptr,FlagsType *flag_ptr)
   {
   strlcpy(conf.gprs[1].uname, data_ptr,MAX_APN_STR_LEN);
   conf.gprs[1].save=1;
   parser_sms_answer.flag.restart_modem=1;
   return 1;
   }

int SetGprsPsw2(char* data_ptr,FlagsType *flag_ptr)
   {
   strlcpy(conf.gprs[1].psw, data_ptr,MAX_APN_STR_LEN);
   conf.gprs[1].save=1;
   parser_sms_answer.flag.restart_modem=1;
   return 1;
   }
//----------------------------------------------------------
int Config(char* data_ptr,FlagsType *flag_ptr)
   {
		 static QEvt const UpdBreakEvt={CONTROL_UPDATE_BREAK_SIG,0U,0U};
   int temp=atoi(data_ptr);
		 flag_ptr->break_upd=0;
		 char temp_str[50];
		 sprintf(temp_str,"CONFIG=%u",temp);
		  QS_BEGIN(QS_SMS_PARSER, &Config_obj)                                 
   QS_STR(temp_str);                              
   QS_END()
   if (temp>=0 && temp<=FWU_TRY_COUNT_MAX)
      {
			if(temp==0)
			{
				flag_ptr->break_upd=1;
				QACTIVE_POST(AO_Control, &UpdBreakEvt, &Config_obj); 
			}
      conf.log.updTryCounter=(uint8_t)temp;
      conf.log.updTryTimeout=CONFIG_UPDATE_TIMEOUT;
      conf.log.save=1;
				
      return 1;
      }
			return 0;
   }
	 
	 
	 int Rrt(char* data_ptr,FlagsType *flag_ptr)
   {
   int temp=atoi(data_ptr);
		 if(temp>REGULAR_RESTART_TIMEOUT_MAX)temp=REGULAR_RESTART_TIMEOUT_MAX;
		 else  if (temp<REGULAR_RESTART_TIMEOUT_MIN)temp=REGULAR_RESTART_TIMEOUT_MIN;
      conf.secur.regular_restart_timeout= temp;
      conf.secur.save=1;
      return 1;
   }
	 
	 int Reboot(char* data_ptr,FlagsType *flag_ptr)
   {
		int temp=atoi(data_ptr);
   if (temp==1)
	 {
		 flag_ptr->reboot_system=1; 
		 		 return 1;
	 }
	 else return 0;
   }




int GetGprs(char* data_ptr,FlagsType *flag_ptr)
   {
     flag_ptr->get_sms_config_gprs=1;
		 		 return 1;
   }

int GetNav(char* data_ptr,FlagsType *flag_ptr)
   {
   flag_ptr->get_sms_config_nav=1;
		 		  return 1;
   }
	 
int GetPack(char* data_ptr,FlagsType *flag_ptr)
   {
   flag_ptr->get_sms_config_pack=1;
		 		  return 1;
   }

int GetVer(char* data_ptr,FlagsType *flag_ptr)
   {
   flag_ptr->get_sms_ver=1;
		 		  return 1;
   }

int GetRS485(char* data_ptr,FlagsType *flag_ptr)
   {
    flag_ptr->get_sms_config_rs485=1;
				  return 1;
   }
	 
	 int GetBalance(char* data_ptr,FlagsType *flag_ptr)
   {
   flag_ptr->get_sms_balance=1;
		 		  return 1;
   }
	 
	 int GetStatus(char* data_ptr,FlagsType *flag_ptr)
   {
   flag_ptr->get_sms_status=1;
		 		  return 1;
   }
	 
	  int GetModem(char* data_ptr,FlagsType *flag_ptr)
   {
   flag_ptr->get_sms_modem=1;
		 		  return 1;
   }
	 
	   int GetImei(char* data_ptr,FlagsType *flag_ptr)
   {
   flag_ptr->get_sms_imei=1;
		 		  return 1;
   }


uint32_t ConfigPacketMaker(uint8_t* pack_ptr)
   {
   char temp[50];
   uint32_t pack_size=0;
   char* payload_ptr=(char*)((XTDevPacketType*)pack_ptr)->payload;
   *payload_ptr='\0';
   strcat(payload_ptr,"{\r\n"); 
   //----------------------------------------------------------------------------------------
   if (conf.secur.protocol==PROTOCOL_FMXXXX)strcat(payload_ptr,"PROTOCOL=FM;\r\n");
   else strcat(payload_ptr,"PROTOCOL=NT;\r\n");
   sprintf(temp,"FW VER=%u_%u;\r\n",FwVer[0],FwVer[1]);
   strcat(payload_ptr,temp);
   sprintf(temp,"FW VER NEW=%u_%u;\r\n",(uint8_t)(conf.secur.fw_ver_new>>8),(uint8_t)conf.secur.fw_ver_new);
   strcat(payload_ptr,temp); 
   sprintf(temp,"IP=%s;\r\n",conf.server[MAIN_SERVER].ip);
   strcat(payload_ptr,temp);
   sprintf(temp,"SIP=%s;\r\n",conf.server[SERVICE_SERVER].ip);
   strcat(payload_ptr,temp);
   sprintf(temp,"PORT=%u;\r\n",conf.server[MAIN_SERVER].port);
   strcat(payload_ptr,temp);
   sprintf(temp,"SPORT=%u;\r\n",conf.server[SERVICE_SERVER].port);
   strcat(payload_ptr,temp);
   sprintf(temp,"APN1=%s;\r\n",conf.gprs[0].apn); 
   strcat(payload_ptr,temp);
   sprintf(temp,"APN2=%s;\r\n",conf.gprs[1].apn); 
   strcat(payload_ptr,temp);
   sprintf(temp,"B1=%s;\r\n",conf.gprs[0].ussd_balance);      
   strcat(payload_ptr,temp);
   sprintf(temp,"B2=%s;\r\n",conf.gprs[1].ussd_balance);      
   strcat(payload_ptr,temp);
   sprintf(temp,"STORE TIME=%u;\r\n",conf_nav_filter.time); 
   strcat(payload_ptr,temp);
   sprintf(temp,"STORE DIST=%u;\r\n",conf_nav_filter.dist);
   strcat(payload_ptr,temp);     
   sprintf(temp,"STORE ANGLE=%u;\r\n",conf_nav_filter.angle);     
   strcat(payload_ptr,temp);
   sprintf(temp,"SEND TIME=%u;\r\n", conf.packet_send.time);
   strcat(payload_ptr,temp);
   sprintf(temp,"SEND DIST=%u;\r\n", conf.packet_send.dist);
   strcat(payload_ptr,temp);
   sprintf(temp,"NAV FILTER=%s;\r\n", OnOffIntToStr(conf_nav_filter.filter_on));      
   strcat(payload_ptr,temp);
   //---------------------RS485---------------------------------------------------------------------------------------------------------------
   for (uint32_t i=0;i<MAX_RS485_SENSORS_COUNT; i++)
      {
      sprintf(temp,"%uRS485D=%u;\r\n",i+1,conf_rs485_sensor[i].device);
      strcat(payload_ptr,temp);
//      sprintf(temp,"%uRS485A=%u;\r\n",i+1,conf_rs485_sensor[i].addr);
  //    strcat(payload_ptr,temp);
      if (conf_rs485_sensor[i].device==RS485_DEVICE_MR9)
         {
         sprintf(temp,"MR9 RELE=%s;\r\n",OnOffIntToStr(conf_rs485_sensor[i].out_state));
         strcat(payload_ptr,temp);
         }
      }
   strcat(payload_ptr,temp);
   //----------END----------------------------------------------------------------------------------------------------------------------------------
   strcat(payload_ptr,"}");
   uint32_t payload_len=strlen(payload_ptr);
   pack_size=((uint32_t)payload_ptr+payload_len)-(uint32_t)pack_ptr;
   ((XTDevPacketType*)pack_ptr)->head.id=HEAD_PACK_CONF;  
   ((XTDevPacketType*)pack_ptr)->head.len=pack_size;
   ((XTDevPacketType*)pack_ptr)->head.crc=0;
   ((XTDevPacketType*)pack_ptr)->head.crc=MakeCRC16(pack_ptr,pack_size);    
   return pack_size;
   }



char* StrToUper(char *str)
   {
   char *str_ptr=str;
   while (*str_ptr!='\0')
      {
      if (islower(*str_ptr))
         {
         *str_ptr=toupper(*str_ptr);
         }
      str_ptr++;
      }
   return str;
   }

int CheckRS485DeviceName(uint8_t name)
   {
   for (int32_t device_name=RS485_NODEVICE;device_name<DEVICES_TABLE_END;device_name++)
      {
      if (device_name==name)return 1;
      }
   return 0;
   }
	 
	 int CheckFinMode(uint8_t mode)
   {
    if (mode==IO_RPM||mode==IO_DIN || mode==IO_OFF)return 1;
    else return 0;
   }
	 
	/* int IpIsCorrect(char *ptr)
{
	#define MAX_IP_FIELD_LEN 3
	static enum
	{
		find_first_char_field_1,
		find_comma_field_1,
		find_first_char_field_2,
		find_comma_field_2,
		find_first_char_field_3,
		find_comma_field_3,
		find_first_char_field_4,
		find_last_char_field_4,
	}state=find_first_char_field_1;
	uint8_t field_len=1;
	for(uint8_t i=0;i<MAX_SERVER_STR_LEN;i++)
	{
		switch(state)
		{
			case find_first_char_field_1:
			{
				if(1==isdigit(ptr[i]))
					{
						state=find_comma_field_1;
				  }
					else return 0;
			}
			break;
			case find_comma_field_1:
			{
				if(1==isdigit(ptr[i]))
				  {
					if(++field_len>MAX_IP_FIELD_LEN)return 0;
				  }
				else if('.'==ptr[i])
			         	{
						     state=find_first_char_field_2;
				        }
							else return 0;
			}
			break;
			case find_first_char_field_2:
			{
				if(1==isdigit(ptr[i]))
					{
					  field_len=1;
						state=find_comma_field_2;
				  }
					else return 0;
			}
			break;
			case find_comma_field_2:
			{
				if(1==isdigit(ptr[i]))
				  {
					if(++field_len>MAX_IP_FIELD_LEN)return 0;
				  }
				else if('.'==ptr[i])
			         	{
						     state=find_first_char_field_3;
				        }
							else return 0;
			}
			break;
			case find_first_char_field_3:
			{
				if(1==isdigit(ptr[i]))
					{
					  field_len=1;
						state=find_comma_field_3;
				  }
					else return 0;
			}
			break;
				case find_comma_field_3:
			{
				if(1==isdigit(ptr[i]))
				  {
					if(++field_len>MAX_IP_FIELD_LEN)return 0;
				  }
				else if('.'==ptr[i])
			         	{
						     state=find_first_char_field_4;
				        }
							else return 0;
			}
			break;
				case find_first_char_field_4:
			{
				if(1==isdigit(ptr[i]))
					{
					  field_len=1;
						state=find_last_char_field_4;
				  }
					else return 0;
			}
			break;
				case find_last_char_field_4:
			{
				if(1==isdigit(ptr[i]))
					{
					 	if(++field_len>MAX_IP_FIELD_LEN)return 0;
				  }
					else if(0==ptr[i])return 1;
			}
			break;
		}
	}
	return 0;
}*/









