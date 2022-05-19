

#include <stdlib.h>
#include "gxrmc.h"
#include "tokens.h"
#include "nmea_parser.h"
#include <stdio.h>
#include "config_sys.h"
#include "nav_receiver.h"
#include <time.h>
#include <string.h>


 NmeaParserIdEnum nmeap_gxrmc(NmeaParserType *context,nmeap_sentence_t *sentence)
   {
			 nmeap_rmc_t *rmc = (nmeap_rmc_t *)sentence->data;
			 if (rmc != 0)
      {		
			rmc->unix_time= nmeap_utime(context->token[1],context->token[9]);
      rmc->warn       = *context->token[2];
      rmc->latitude  = nmeap_latitude(context->token[3],context->token[4]);
      rmc->longitude = nmeap_longitude(context->token[5],context->token[6]);
      rmc->speed      = atof(context->token[7]);
      rmc->course     = atof(context->token[8]);
     // rmc->date       = atoi(context->token[9]);
      rmc->magvar     = atof(context->token[10]);
      }

#ifdef DEBUG_GxRMC  
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
      (*sentence->callout)(context,rmc,context->user_data);
      }

   return NMEAP_GxRMC;
   }
	 
