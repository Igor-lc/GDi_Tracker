
#ifndef _PMTK010_H
#define _PMTK010_H

#include "nmea_parser.h"
#include <stdint.h>

#define PMTK010_STARTUP_MESSAGE 1

typedef struct 
{
	uint8_t  message;
}nmeap_pmtk010_t;

extern NmeaParserIdEnum nmeap_pmtk010(NmeaParserType *context,nmeap_sentence_t *sentence);

#endif
