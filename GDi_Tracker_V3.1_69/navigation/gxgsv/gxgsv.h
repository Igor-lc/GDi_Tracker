
#ifndef _GXGSV_H
#define _GXGSV_H

#include "nmea_parser.h"
#include <stdint.h>

typedef struct 
{
	uint8_t sat_total;
}nmeap_gsv_t;

extern NmeaParserIdEnum nmeap_gxgsv(NmeaParserType *context,nmeap_sentence_t *sentence);

#endif
