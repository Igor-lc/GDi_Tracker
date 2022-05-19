
#ifndef _GXGLL_H
#define _GXGLL_H

#include "nmea_parser.h"

typedef struct 
{
	double        latitude;
	double        longitude;
	char          ew;
	char          ns;
	unsigned long time;
	char          valid;
	char          pos_mode;//N-no fix,A-autonomous GNSS fix,D-differential GNSS fix
}nmeap_gll_t;

extern NmeaParserIdEnum nmeap_gxgll(NmeaParserType *context,nmeap_sentence_t *sentence);

#endif
