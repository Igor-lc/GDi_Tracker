
#ifndef _MODEM_H
#define _MODEM_H

#include "qp_port.h"
#include <stdint.h>
#include "user_types.h"
#include "ParserSms.h"

#define AUTH_PACK_MAX_SIZE  60
#define GSMQ_TIMEOUT    50//5 sec

#define POWER_ON_START_UP_ERROR_TIMEOUT      300//30sec 
#define POWER_ON_START_UP_FATAL_ERROR_TIMEOUT (POWER_ON_START_UP_ERROR_TIMEOUT*3)
//-----------------------------------------------------------------------
#define OPEN_GSM_ERROR_TIMEOUT               50//5sec SIM_ERR_HANDLED
#define OPEN_GSM_FATAL_ERROR_TIMEOUT        (OPEN_GSM_ERROR_TIMEOUT)
//-----------------------------------------------------------------------
#define OPEN_GPRS_ERROR_TIMEOUT              50//5sec 
#define OPEN_GPRS_FATAL_ERROR_TIMEOUT        (OPEN_GPRS_ERROR_TIMEOUT)
//---------------------------------------------------------------------
#define OPEN_CONNECTION_ERROR_TIMEOUT        50//5sec 
#define OPEN_CONNECTION_FATAL_ERROR_TIMEOUT  1200//120 sec
//---------------------------------------------------------------------
#define AUTHORIZATION_ERROR_TIMEOUT          50//20sec 
#define AUTHORIZATION_FATAL_ERROR_TIMEOUT    1200//600 sec
//---------------------------------------------------------------------
#define SEND_PACKET_ERROR_TIMEOUT            80//8sec 
#define SEND_PACKET_FATAL_ERROR_TIMEOUT      1200//600 sec

#define SIM_ERROR_TIMEOUT             50//5sec SIM_ERR_HANDLED
#define SIM_FATAL_ERROR_TIMEOUT      (SIM_ERROR_TIMEOUT*2)
#define SIM_LOCK_TIMEOUT              50//5sec SIM_ERR_HANDLED
#define SIM_LOCK_FATAL_ERROR_TIMEOUT (SIM_LOCK_TIMEOUT*2)

#define MODEM_TASK_EVENT_QUEUE_DEPTH  10


typedef	enum 
{
	gsm_modem_off,
	gsm_registration,
	gsm_registered_in_home,
	gsm_registered_in_roaming,
	gsm_reg_error,
	gsm_sim_error,
	gsm_sim_lock_error,
}GsmStateType;

typedef	enum 
{
	gprs_off,
	gprs_opening,
	gprs_is_open,
	gprs_open_error,
}GprsStateType;

typedef	enum 
{
	server_not_connected,
	server_connecting,
	server_authorized_and_connected,
	server_send_success,
	server_connect_error,
	server_authorization_error,
}ServerStateType;


typedef struct 
   {
   uint8_t power_on_start_up_error;
   uint8_t open_gsm_error;
   uint8_t open_gprs_error;
   uint8_t open_connection_error;
   uint8_t authorization_error;
   uint8_t send_packet_error;
	 uint8_t sim_error;
	 uint8_t sim_lock_error;
   }ErrorCounterType;
	 
	 typedef struct 
   {
	 uint16_t power_on_start_up;	
   uint16_t open_gsm;		 
	 uint16_t open_gprs;
	 uint16_t open_connection;
	 uint16_t authorization;
	 uint16_t send_packet;
	 uint16_t sim;
	 uint16_t sim_lock;
   }FatalErrorTimerType;


#ifdef DEBUG_MODEM
void OutDebugModem( char const *dbg_msg);
void OutDebugModemSprintf1( char const *str,uint32_t val);
void OutDebugModemSprintf2( char const *str1,char const *str2,uint32_t val1,uint32_t val2);
#else
#define OutDebugModem(x) __nop()
#define OutDebugModemSprintf1(x,y) __nop()
#define OutDebugModemSprintf2(x1,x2,x3,x4) __nop()
#endif

extern ProcessSmsAnswerType parser_sms_answer;
extern QActive* const AO_Modem; 
extern void Modem_ctor(void);
extern GsmStateType GsmState;
extern GprsStateType GprsState;
extern ServerStateType ServerState;

#endif






