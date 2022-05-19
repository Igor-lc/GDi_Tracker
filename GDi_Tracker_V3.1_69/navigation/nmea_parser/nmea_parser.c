

#include "nmea_parser.h"
#include "nav_receiver.h"
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>


	//NmeaParserIdEnum nmeap_parse(NmeaParserType *context,char ch);
	static NmeaParserIdEnum nmeap_process(NmeaParserType *context);
	static int nmeap_tokenize(NmeaParserType *context);

void nmeap_init(NmeaParserType *context,void *user_data)
   {
   assert(context != 0);
   memset(context,0,sizeof(*context));
   context->user_data = user_data;    
   }
	 
	 
int nmeap_addParser(NmeaParserType *context,const char *sentence_name,nmeap_sentence_parser_t sentence_parser,nmeap_callout_t sentence_callout,void *sentence_data)
   {
   nmeap_sentence_t *s = 0;
   assert(context != 0); //runtime error  
   if (context->sentence_count >= NMEAP_MAX_SENTENCES)// sentence capacity overflow 
      {
      return -1;
      }
   s = &context->sentence[context->sentence_count]; // point at next empty sentence buffer
   context->sentence_count++; //advance sentence data count
   memset(s,0,sizeof(*s)); // clear the sentence data
   strncpy(s->name,sentence_name,NMEAP_MAX_SENTENCE_NAME_LENGTH); //name
   s->parser = sentence_parser; //parser
   s->callout = sentence_callout; //callout
   s->data    = sentence_data; //data
   return 0;
   }
	 
	  
	 
	 NmeaParserIdEnum nmeap_parseBuffer(NmeaParserType *context,const char *buffer,int *length)
   {
   int  i;
   NmeaParserIdEnum  id=NMEAP_UNKNOWN_ID;
   int  rem;
   int  tlen;

   tlen   = *length;
   rem    = *length;
   /* for each byte in the buffer */
   for (i=0;i<tlen;i++)
      {
      /* decrement remaining byte count */
      rem--;
      /* parse the byte */
      id= nmeap_parse(context,buffer[i]);
      if (id != NMEAP_UNKNOWN_ID)
         {
         /* message found or error */
         break;
         }
      }

   /* return remaining byte count */
   *length = rem;
   return id;
   }
	 
	 NmeaParserIdEnum nmeap_parse(NmeaParserType *context,char ch)
   {
		  #define OFFSET 2
   NmeaParserIdEnum id =NMEAP_UNKNOWN_ID;
   /* check for input buffer overrun first to avoid duplicating code in the
   individual states
   */
   if (context->input_count >= (sizeof(context->input)-1))
      {
      /* input buffer overrun, restart state machine */
      context->state = LOOKING_FOR_SOF;
      /* reset input count */
      context->input_count = 0;
      }

   /* store the byte */
   context->input[context->input_count] = ch;

   /* next buffer position */
   context->input_count++;

   /* run it through the lexical scanner */
   switch (context->state)
      {
      case LOOKING_FOR_SOF:
         if (ch == '$')
            {
            /*look for id */
            context->state = LOOKING_FOR_SENTENCE_ID;
            context->calc_crc        = 0;
            context->in_crc        = 0;
            }
         else
            {
            context->err_header++;
           // context->state = LOOKING_FOR_SOF;
            context->input_count = 0;
            }
         break;
     // case LOOKING_FOR_SENTENCE_ID:
         //allow numbers even though it isn't usually done 
         //a proprietary id might have a numeral 
        /* if (isalnum(ch))
            {
            context->sentence_name[context->input_count - 2] = ch;
            context->calc_crc ^= ch;
            if (context->input_count >= 6)
               {
               context->state = LOOKING_FOR_CR_OR_CRC_INDICATOR;
               }
            }
         else
            {
            context->err_id++;
            context->state = LOOKING_FOR_SOF;
            context->input_count = 0;
            }
         break;*/
			 case LOOKING_FOR_SENTENCE_ID:
         /* allow numbers even though it isn't usually done */
         /* a proprietary id might have a numeral */
         context->input_s_name[context->input_count - OFFSET] = ch;
         context->calc_crc ^= ch;
						 if (isalnum (ch))
						 {
							 if (context->input_count >NMEAP_MAX_SENTENCE_NAME_LENGTH+1)
                  {
                  // context->state = LOOKING_FOR_CR_OR_CRC_INDICATOR;
										context->err_id++;
                    context->state = LOOKING_FOR_SOF;
                    context->input_count = 0;
								   // DEBUG_NMEA_PARSER_SPRINTF("ERR FINDED:: %s\r\n",context->input_s_name);
                  }
						}
						 else if(( ch==',')&& context->input_count >=NMEAP_MIN_SENTENCE_NAME_LENGTH+OFFSET && context->input_count <=NMEAP_MAX_SENTENCE_NAME_LENGTH+OFFSET)
						 {
                context->state = LOOKING_FOR_CR_OR_CRC_INDICATOR;
							   context->input_s_name[context->input_count - OFFSET] = '\0';//cut comma
							 char temp[50];
							   sprintf(temp,"OK FINDED:: %s\r\n",context->input_s_name);
						 }
						else
							{
							  context->err_id++;
                context->state = LOOKING_FOR_SOF;
               context->input_count = 0;
						 }
         break;
      case LOOKING_FOR_CR_OR_CRC_INDICATOR:
         if (ch == '*')
            {
            /* this sentence has a checksum */
            context->state = LOOKING_FOR_FIRST_CRC_CHAR;
            }
         else if (ch == '\r')
            {
            /* carriage return, no checksum, force a match */
            context->in_crc = 0;
            context->calc_crc = 0;
            context->state = LOOKING_FOR_LF;
            }
         else
            {
            /* continue accumulating data */
            /* checksum */
            context->calc_crc ^= ch;
            }
         break;
      case LOOKING_FOR_FIRST_CRC_CHAR:
         /* must be upper case hex digit */
         if (isxdigit(ch) && (ch <= 'F'))
            {
            /* got first checksum byte */
            context->state = LOOKING_FOR_SECOND_CRC_CHAR;
            context->in_crc = HEXTOBIN(ch) << 4;
            }
         else
            {
            /* input error, restart */
            context->err_crc++;
            context->state = LOOKING_FOR_SOF;
            context->input_count = 0;
            }
         break;
      case LOOKING_FOR_SECOND_CRC_CHAR:
         /* must be upper case hex digit */
         if (isxdigit(ch) && (ch <= 'F'))
            {
            /* got second checksum byte */
            context->state = LOOKING_FOR_CR;
            context->in_crc += HEXTOBIN(ch);
            }
         else
            {
            /* input error, restart */
            context->err_crc++;
            context->state = LOOKING_FOR_SOF;
            context->input_count = 0;
            }
         break;
      case LOOKING_FOR_CR:
         if (ch == '\r')
            {
            /* carriage return */
            context->state = LOOKING_FOR_LF;
            }
         else
            {
            /* input error, restart */
            context->err_cr++;
            context->state = LOOKING_FOR_SOF;
            context->input_count = 0;
            }
         break;
      case LOOKING_FOR_LF:
         if (ch == '\n')
            {
            /* linefeed, line complete */

            /* delimit buffer */
            context->input[context->input_count] = 0;

            /* if the checksums match, process the sentence */
            if (context->calc_crc == context->in_crc)
               {
               /* process */  
               id = nmeap_process(context);

               /* count good messages */
               context->msgs++;
               }
            else
               {
               /* count checksum errors */
               context->err_crc++;
               }

            /* restart next time */
            context->state = LOOKING_FOR_SOF;
            context->input_count = 0;
            }
         else
            {
            /* input error, restart */
            context->err_lf++;
            context->state = LOOKING_FOR_SOF;
            context->input_count = 0;
            }
         break;
      default:
         context->err_unk++;
         context->state = LOOKING_FOR_SOF;
         break;
      }
   return id;
   }
	 
	 
	 NmeaParserIdEnum nmeap_process(NmeaParserType *context)
   {
  NmeaParserIdEnum ID=NMEAP_UNKNOWN_ID;
		 int slen;
   nmeap_sentence_t *s;

   /* copy the input to a debug buffer */
   /* remove debug_input when everything is working. */
   strncpy(context->debug_input,context->input,sizeof(context->debug_input));

   /* tokenize the input */
   context->tokens = nmeap_tokenize(context);

   /* try to find a matching sentence parser */
   /* this search is O(n). it has a lot of potential for optimization, at the expense of complexity, if you have a lot of sentences */
   /* binary search instead of linear (have to keep sentences in sorted order) O(NlogN) */
   /* OR, when sentences are added, create a TRIE structure to find the names with a constant time search O(5) */
   for (int i=0;i<context->sentence_count;i++)
      {
      s = &context->sentence[i];
      assert(s != 0);
     // if (strncmp(context->input_s_name,s->name,5) == 0)
				  slen=strlen(context->input_s_name);
      if (strncmp(context->input_s_name,s->name,slen) == 0)
         {
         /* found a match, call its parser */
					 // OutDebugNavReceiver(context->input_name);
         ID = (*context->sentence[i].parser)(context,s);
         if (ID > NMEAP_UNKNOWN_ID)
            {
							if(ID==NMEAP_GxGLL)
							{
								__nop();
							}
            break;
            }
         }
      }

   return ID;
   }
	 
	 int nmeap_tokenize(NmeaParserType *context)
   {
   char *s;
   int   tokens;
   int   state;

   /* first token is header. assume it is there */
   tokens = 0;
   s = context->input;
   context->token[tokens] = s;

   /* get rest of tokens */
   tokens = 1;
   state = 0;
   while ((*s != 0)&&(tokens < NMEAP_MAX_TOKENS))
      {
      switch (state)
         {
         case 0:
            /* looking for end of a token */
            if (*s == ',')
               {
               /* delimit at the comma */
               *s    = 0;
               /* new token */
               state = 1;
               }
            break;
         case 1:
            /* start of next token, might be another comma */
            context->token[tokens++] = s;
            if (*s == ',')
               {
               /* delimit at the comma */
               *s    = 0;
               }
            else
               {
               /* not a comma */
               state = 0;
               }
            break;
         default:
            state = 0;
            break;
         }

      // next character
      s++;
      }
   return tokens;
   }






	 

 
	 
