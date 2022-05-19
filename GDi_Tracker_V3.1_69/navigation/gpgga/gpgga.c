

#include "nmea_parser.h"
#include <stdlib.h>
#include <stdio.h>
#include "tokens.h"
#include "gpgga.h"
#include "config_sys.h"
#include "nav_receiver.h"

NmeaParserIdEnum nmeap_gpgga(NmeaParserType *context,nmeap_sentence_t *sentence)
{
    /* get pointer to sentence data */
    nmeap_gga_t *gga = (nmeap_gga_t *)sentence->data;
    
    /* if there is a data element, extract data from the tokens */
	if (gga != 0)
		{
		gga->latitude  = nmeap_latitude(context->token[2],context->token[3]);
		gga->longitude = nmeap_longitude(context->token[4],context->token[5]);
		gga->altitude  = nmeap_altitude(context->token[9],context->token[10]);
		gga->time       = atoi(context->token[1]);
		gga->sat_used = atoi(context->token[7]);
		gga->fix_mode    = atoi(context->token[6]);
		gga->hdop       = atof(context->token[8]);
		gga->geoid      = nmeap_altitude(context->token[11],context->token[12]);
	}

#ifdef DEBUG_GPGGA  
char print_buf[100];	  
    /* print raw input string */
    sprintf(print_buf,"%s",context->debug_input);
    OutDebugNavReceiver(print_buf);
    /* print some validation data */
    sprintf(print_buf,"%s==%s %02x==%02x",context->input_name,sentence->name,context->icks,context->ccks);
    OutDebugNavReceiver(print_buf);
    /* print the tokens */
    for(int i=0;i<context->tokens;i++) 
	     {
        sprintf(print_buf,"%d:%s",i,context->token[i]);
				 OutDebugNavReceiver(print_buf);
       }
#endif   

    /* if the sentence has a callout, call it */
    if (sentence->callout != 0)
			{
        (*sentence->callout)(context,gga,context->user_data);
    }
    
    return NMEAP_GPGGA;
}




