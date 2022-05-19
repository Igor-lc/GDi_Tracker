
#ifndef _GXGSA_H
#define _GXGSA_H

#include "nmea_parser.h"
#include <stdint.h>

typedef struct 
{
	char          mode;//A-auto,M-manual switch 2D/3D
	uint8_t       fix_status;//1-nofix,2-2D,3-3D
	uint8_t sat_used_channel[12];
	float pdop;
	float hdop;
	float vdop;
}nmeap_gsa_t;

extern NmeaParserIdEnum nmeap_gxgsa(NmeaParserType *context,nmeap_sentence_t *sentence);

#endif
