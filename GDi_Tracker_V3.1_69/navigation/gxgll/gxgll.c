

#include <stdlib.h>
#include "gxgll.h"
#include "tokens.h"
#include "nmea_parser.h"
#include <stdio.h>
#include "config_sys.h"
#include "nav_receiver.h"


 NmeaParserIdEnum nmeap_gxgll(NmeaParserType *context,nmeap_sentence_t *sentence)
   {
   /* get pointer to sentence data */
   nmeap_gll_t *gll = (nmeap_gll_t *)sentence->data;

   /* if there is a data element, use it */
   if (gll != 0)
      {
      gll->latitude  = nmeap_latitude(context->token[1],context->token[2]);
      gll->longitude = nmeap_longitude(context->token[3],context->token[4]);
		  gll->time       = atoi(context->token[5]);
			gll->valid      = *context->token[6];
			gll->pos_mode   = *context->token[7];
      }

#ifdef DEBUG_GxGLL  
char print_buf[100];			
   /* print raw input string */
   sprintf(print_buf,"%s",context->debug_input);
OutDebugNavReceiver(print_buf);
   /* print some validation data */
   sprintf(print_buf,"%s==%s %02x==%02x",context->input_name,sentence->name,context->icks,context->ccks);
OutDebugNavReceiver(print_buf);
   /* print the tokens */
   for (int i=0;i<context->tokens;i++)
      {
      sprintf(print_buf,"%d:%s",i,context->token[i]);
				OutDebugNavReceiver(print_buf);
      }
#endif   
			  /* if the sentence has a callout, call it */
   if (sentence->callout != 0)
      {
      (*sentence->callout)(context,gll,context->user_data);
      }

   return NMEAP_GxGLL;
   }
	 
