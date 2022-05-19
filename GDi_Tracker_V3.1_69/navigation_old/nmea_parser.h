

#ifndef _GPS_PARSER_H
#define _GPS_PARSER_H

#include "qp_port.h"
#include "user_types.h"

#define GPS_BUF_SIZE                300

#define NMEA_SEQUENCE_MAX_LENGTH  100
#define ADDRESS_FIELD_MAX_LENGTH  8
#define MAXSATINVIEW              15		// maximum sat in view
#define MAXFIELD	                25		// maximum field length

typedef enum
{
	gprmc_time_field=1,
	gprmc_status_field,
	gprmc_lat_field,
	gprmc_ns_field,
	gprmc_lon_field,
	gprmc_ew_field,
	gprmc_speed_field,
	gprmc_course_field,
	gprmc_date_field,
	gprmc_mv_field,
}GprmcFieldsEnum;

enum GpsParserSignals
{
	NAV_BUF_SIG____unused=MAX_PUB_SIG,
//	NAV_RESET_SIG,
	
};


typedef enum
    {
    comma_field,
    end_field,
    error_field,
    ok_field,   
    }FieldEnumType;
		



#ifdef DEBUG_NMEA_PARSER
extern void OutDebugNmeaParser( char const *str);
extern void OutDebugGpsParserSprintf( char const *str,uint32_t val);
#else
#define OutDebugNmeaParser(x) __nop()
#define OutDebugGpsParserSprintf(x,y) __nop()
#endif


extern void GpsParser_ctor(void);
extern QActive * const AO_GpsParser;

#endif
