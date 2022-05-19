
#ifndef _GXRMC_H
#define _GXRMC_H

#include "nmea_parser.h"
#include <stdint.h>

typedef struct 
{
	uint32_t      unix_time;
	char          warn;
	double        latitude;
	double        longitude;
	float         speed;
	float         course;
	//unsigned long date;
	float        magvar;
}nmeap_rmc_t;

extern NmeaParserIdEnum nmeap_gxrmc(NmeaParserType *context,nmeap_sentence_t *sentence);

#endif
