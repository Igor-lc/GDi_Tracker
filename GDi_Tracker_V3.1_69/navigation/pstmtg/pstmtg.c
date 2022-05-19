

#include <stdlib.h>
#include "pstmtg.h"
#include "tokens.h"
#include "nmea_parser.h"
#include <time.h>


 NmeaParserIdEnum nmeap_pstmtg(NmeaParserType *context,nmeap_sentence_t *sentence)
   {
			 nmeap_pstmtg_t *pstmtg = (nmeap_pstmtg_t *)sentence->data;
   if (sentence->callout != 0)
      {
      (*sentence->callout)(context,pstmtg,context->user_data);
      }
			return NMEAP_PSTMTG;
   }
	 
