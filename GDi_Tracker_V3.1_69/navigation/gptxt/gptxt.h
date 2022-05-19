
#ifndef _GPTXT_H
#define _GPTXT_H

#include "nmea_parser.h"
#include <stdint.h>

//#define PMTK010_STARTUP_MESSAGE 1

typedef struct 
{
	char  *txt;
}nmeap_gptxt_t;

extern NmeaParserIdEnum nmeap_gptxt(NmeaParserType *context,nmeap_sentence_t *sentence);

#endif
