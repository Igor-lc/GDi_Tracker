
#ifndef _GPVTG_H
#define _GPVTG_H

#include "nmea_parser.h"
#include <stdint.h>

typedef struct 
{
	float         speed_kmh;
}nmeap_vtg_t;

extern NmeaParserIdEnum nmeap_gpvtg(NmeaParserType *context,nmeap_sentence_t *sentence);

#endif
