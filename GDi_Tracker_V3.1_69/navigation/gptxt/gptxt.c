

#include <stdlib.h>
#include "gptxt.h"
#include "tokens.h"
#include "nmea_parser.h"
#include <stdio.h>
#include "config_sys.h"
#include "nav_receiver.h"
#include <time.h>
#include <string.h>


 NmeaParserIdEnum nmeap_gptxt(NmeaParserType *context,nmeap_sentence_t *sentence)
   {
			 nmeap_gptxt_t *gptxt = (nmeap_gptxt_t *)sentence->data;
			 if (gptxt != 0)
      {		
      gptxt->txt    = context->token[1];
      }
			
   if (sentence->callout != 0)
      {
      (*sentence->callout)(context,gptxt,context->user_data);
      }

   return NMEAP_GPTXT;
   }
	 
