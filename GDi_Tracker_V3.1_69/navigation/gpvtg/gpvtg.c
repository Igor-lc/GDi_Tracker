

#include <stdlib.h>
#include "gpvtg.h"
#include "tokens.h"
#include "nmea_parser.h"
#include <stdio.h>
#include <time.h>
#include <string.h>


 NmeaParserIdEnum nmeap_gpvtg(NmeaParserType *context,nmeap_sentence_t *sentence)
   {
			 nmeap_vtg_t *vtg = (nmeap_vtg_t *)sentence->data;
			 if (vtg != 0)
      {		
      vtg->speed_kmh = atof(context->token[7]);
      }
   if (sentence->callout != 0)
      {
      (*sentence->callout)(context,vtg,context->user_data);
      }
			return NMEAP_GPVTG;
   }
	 
