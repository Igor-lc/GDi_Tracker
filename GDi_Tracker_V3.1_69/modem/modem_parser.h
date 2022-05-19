

#ifndef _MODEM_PARSER
#define _MODEM_PARSER

#include "qp_port.h"
//#include "main.h"
#include "modem_signals.h"


enum ModemParserSignals
{
	MODEM_PARSER_TIC_SIG=MAX_PUB_SIG,//5
	MODEM_PARSER_WAITING_IP_SEND_SIG,
//  MODEM_PARSER_SMS_PTR_SIG,   	
};

typedef struct
{
	const char *msg;
	const QSignal sig; 
	uint8_t len;
}ModemMsgType;



extern void Mparser_ctor(void);
extern QActive* const AO_ModemParser; 

#endif


