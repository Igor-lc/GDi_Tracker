

#include <stdlib.h>
#include "gxgsa.h"
#include "tokens.h"
#include "nmea_parser.h"
#include <stdio.h>
#include "config_sys.h"
#include "nav_receiver.h"


 NmeaParserIdEnum nmeap_gxgsa(NmeaParserType *context,nmeap_sentence_t *sentence)
   {
   /* get pointer to sentence data */
   nmeap_gsa_t *gsa_ptr = (nmeap_gsa_t *)sentence->data;

   /* if there is a data element, use it */
   if (gsa_ptr != 0)
      {
       gsa_ptr->mode          = *context->token[1];
			 gsa_ptr->fix_status = atoi(context->token[2]);
			 for(int i=0;i<12;i++) gsa_ptr->sat_used_channel[i] = atoi(context->token[i+3]);
       gsa_ptr->pdop     = atof(context->token[15]);
       gsa_ptr->hdop      = atoi(context->token[16]);
       gsa_ptr->vdop     = atof(context->token[17]);
      }

#ifdef DEBUG_GxGSA  
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
      (*sentence->callout)(context,gsa_ptr,context->user_data);
      }

   return NMEAP_GxGSA;
   }
	 
