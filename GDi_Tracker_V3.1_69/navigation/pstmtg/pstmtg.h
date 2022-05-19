
#ifndef _PSTMTG_H
#define _PSTMTG_H

#include "nmea_parser.h"
#include <stdint.h>

typedef struct 
{
	uint8_t dummy;
}nmeap_pstmtg_t;

extern NmeaParserIdEnum nmeap_pstmtg(NmeaParserType *context,nmeap_sentence_t *sentence);

#endif
