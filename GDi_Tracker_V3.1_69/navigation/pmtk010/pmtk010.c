

#include <stdlib.h>
#include "pmtk010.h"
#include "tokens.h"
#include "nmea_parser.h"
#include <stdio.h>
#include "config_sys.h"
#include "nav_receiver.h"
#include <time.h>
#include <string.h>


 NmeaParserIdEnum nmeap_pmtk010(NmeaParserType *context,nmeap_sentence_t *sentence)
   {
			 nmeap_pmtk010_t *pmtk010 = (nmeap_pmtk010_t *)sentence->data;
			 if (pmtk010 != 0)
      {		
      pmtk010->message    = atoi(context->token[1]);
      }
			
   if (sentence->callout != 0)
      {
      (*sentence->callout)(context,pmtk010,context->user_data);
      }

   return NMEAP_PMTK010;
   }
	 
