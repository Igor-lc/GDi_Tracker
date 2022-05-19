
#ifndef _GPGGA_H
#define _GPGGA_H

#include <stdint.h>
#include "nmea_parser.h"

typedef struct 
{
	double        latitude;
	double        longitude;
	float         altitude;
	unsigned long time;
	uint8_t       sat_used;
	uint8_t       fix_mode;
	float         hdop;
	float         geoid;
}nmeap_gga_t;

extern NmeaParserIdEnum nmeap_gpgga(NmeaParserType *context,nmeap_sentence_t *sentence);

#endif
