

#include <stdlib.h>
#include "gxgsv.h"
#include "tokens.h"
#include "nmea_parser.h"
#include <stdio.h>
#include "config_sys.h"
#include "nav_receiver.h"


 NmeaParserIdEnum nmeap_gxgsv(NmeaParserType *context,nmeap_sentence_t *sentence)
   {
   nmeap_gsv_t *gsv_ptr = (nmeap_gsv_t *)sentence->data;
   if (gsv_ptr != 0)
      {
				gsv_ptr->sat_total=atoi(context->token[3]);
      }

#ifdef DEBUG_GxGSV 		
char print_buf[100];			
   /* print raw input string */
   sprintf(print_buf,"%s",context->debug_input);
OutDebugNavReceiver(print_buf);
   /* print some validation data */
   sprintf(print_buf,"%s==%s %02x==%02x %s %s %s",context->sentence_name,sentence->name,context->in_crc,context->calc_crc,context->token[1],context->token[2],context->token[3]);
OutDebugNavReceiver(print_buf);
   /* print the tokens */
  /* for (int i=0;i<context->tokens;i++)
      {
      sprintf(print_buf,"%d:%s",i,context->token[i]);
				OutDebugNavReceiver(print_buf);
      }*/
#endif   
			  /* if the sentence has a callout, call it */
   if (sentence->callout != 0)
      {
      (*sentence->callout)(context,gsv_ptr,context->user_data);
      }

   return NMEAP_GxGSV;
   }
	 
