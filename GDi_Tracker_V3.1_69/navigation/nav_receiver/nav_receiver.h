

#ifndef _GPS_PARSER_H
#define _GPS_PARSER_H

#include "qp_port.h"
#include "user_types.h"

#ifdef DEBUG_NAVREC
#define NAVREC_STATE_HISTORY_SIZE 10
#define NAVREC_PARSER_ID_HISTORY_SIZE 100
#define DEBUG_VTG
#define DEBUG_RMC 
//#define DEBUG_GGA 
//#define DEBUG_GSA 
//#define DEBUG_GSV
//#define DEBUG_GLL
#define DEBUG_PMTK010
#define DEBUG_PSTMTG
#define DEBUG_GPTXT
#endif

#define GPS_BUF_SIZE                100

enum GpsParserSignals
{
	NAV_BUF_SIG=MAX_PUB_SIG,
	NAV_PMTK010_SIG,
	NAV_SIM68_SIG,
	NAV_TIMEOUT_SIG,
	
};


typedef enum
    {
    comma_field,
    end_field,
    error_field,
    ok_field,   
    }FieldEnumType;
		
		
#ifdef DEBUG_NAVREC		
extern void OutDebugNavReceiver( char const *str);
extern void OutDebugNavReceiverSprintf1( char const *str,uint32_t val);
#else
#define OutDebugNavReceiver(x) __nop()
#define OutDebugNavReceiverSprintf1( x1,x2)  __nop()
#endif

extern void NavReceiver_ctor(void);
extern QActive * const AO_NavReceiver;

#endif
