

#include "ParserSms.h"
#include "user_types.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include "config.h"
#include "modem.h"
#include "LedIndicator.h"

static const char *SkipEmptySpaces(const char *str_ptr);
static const char *next_cmd_ptr; 

#ifdef DEBUG_SMS_PARSER  
static uint8_t SMS_PARSER;
static void OutDebugParserSms( char const *dbg_msg);
#else
#define OutDebugParserSms(x) __nop()
#define OutDebugParserSmsSprintf1(x1,x2) __nop()
#endif	 
	 

void ParserSms(ProcessSmsAnswerType* ptr)
   {
   uint8_t cmd_len;
	 *(uint32_t*)&ptr->flag=(uint32_t)0;	 		 
		 
   if (NULL==(next_cmd_ptr= SkipEmptySpaces(ptr->sms_ptr)))
      {
      ptr->answer=PROCESS_SMS_FORMAT_ERROR;
      return ;
      }
   else if (NULL==(next_cmd_ptr= FindPassword(next_cmd_ptr,&cmd_len)))
      {
      ptr->answer=PROCESS_SMS_PASSWORD_ERROR;
      return ;
      }
   while (NULL!=next_cmd_ptr)
      {
      next_cmd_ptr=ProcessCommand(next_cmd_ptr,&cmd_len,&ptr->flag);
      }
   ptr->answer=PROCESS_SMS_OK;      
   }

const char *SkipEmptySpaces(const char *str_ptr)
   {
   const char *ptr = str_ptr;
   while ((*ptr!='\0')&&(*ptr==' '))
      {
      ptr++;
      if ((uint32_t)(ptr - str_ptr) >= MAX_SMS_SIZE)return NULL;
      }
   return ptr; 
   }

const char *CheckPassword(const char *str_ptr,uint8_t *cmd_len)
   {
   const char *end_ptr,*start_ptr;
   char temp_buf[5];
   end_ptr = strchr(str_ptr, ';');//set end_ptr to end of password
   if (end_ptr == NULL)return NULL;
   if ((end_ptr-str_ptr)!=4)return NULL;//password len error
   strncpy(temp_buf, str_ptr, 4);
   temp_buf[4] = '\0';
   uint16_t password= atoi(temp_buf);
   //if (conf.secur.sms_password!= atoi(temp_buf))return NULL;
	 if (SMS_PASSWORD_DEF!= atoi(temp_buf))return NULL;
   start_ptr=end_ptr+1;
   end_ptr = strchr(start_ptr, ';');//set end_ptr to end of next command
   if (end_ptr == NULL)return NULL;//end of next command not finded
   *cmd_len=(uint8_t)(end_ptr-start_ptr);
   return start_ptr; 
   }


const char *FindPassword(const char *str_ptr,uint8_t *cmd_len)
   {
   const char *pasw_ptr,*start_ptr,*end_ptr;
   char temp_buf[7];
   //sprintf(temp_buf,"%u;",conf.secur.sms_password);
	 sprintf(temp_buf,"%u;",SMS_PASSWORD_DEF);
   pasw_ptr = strstr(str_ptr,temp_buf);
   if (pasw_ptr == NULL)return NULL;
   start_ptr=pasw_ptr+5;
   end_ptr = strchr(start_ptr, ';');//set end_ptr to end of next command
   if (end_ptr == NULL)return NULL;//end of next command not finded
   *cmd_len=(uint8_t)(end_ptr-start_ptr);
   return start_ptr; 
   }



const char *ProcessCommand(const char *start_ptr,uint8_t *cmd_len,FlagsType* flag_ptr)
   {
   static char string_data_buf[50];
   const char *end_ptr;
   char *char_ptr;
   uint8_t table_index;

   if (*cmd_len>=MIN_CMD_LEN)
      {
//----------------------TOUPPER------------------------------------------------
      char_ptr=(char*)start_ptr;
      while ((*char_ptr!=';')&&(*char_ptr!='='))
         {
         *char_ptr=toupper(*char_ptr);
         char_ptr++;
         }
			if (0==strncmp((const char*)start_ptr,"FW VER",6))
         {
           __nop();                      
         }
//----------------------------------------------------------------------------
      table_index=0;
      while (sms_cmd_tbl[table_index].cmd_name!=NULL)
         {
         char *table_char_ptr=sms_cmd_tbl[table_index].cmd_name;
         char_ptr=(char*)start_ptr;
         while (*char_ptr==*table_char_ptr)
            {
            if ((*(table_char_ptr+1)=='\0') && (*(char_ptr+1)==';'||*char_ptr=='='))
               {
               char sms_print_buf[50];
               sprintf(sms_print_buf,"Finded COMMAND is \"%s\"\r\n",sms_cmd_tbl[table_index].cmd_name);
               OutDebugParserSms(sms_print_buf);
               switch (sms_cmd_tbl[table_index].cmd_type)
                  {
                  case cmd_set:
                     {
                        char_ptr++;
                        char *end_ptr;
                        if ((end_ptr = strchr(char_ptr, ';')) == NULL)goto PROCESS_NEXT_CMD;
                        strncpy(string_data_buf, char_ptr, end_ptr - char_ptr);
                        string_data_buf[end_ptr - char_ptr] = '\0';
                        if(1==sms_cmd_tbl[table_index].handler(string_data_buf,flag_ptr))
												{
													flag_ptr->save_config=1;
													ind_flag.sms_command_complete=1;
												}
											  goto PROCESS_NEXT_CMD;
                     }
										case cmd_get:
                     {
												sms_cmd_tbl[table_index].handler(string_data_buf,flag_ptr);
											  ind_flag.sms_command_complete=1;
                        goto PROCESS_NEXT_CMD;
                     }
                  }
               }
            else if ((*table_char_ptr=='\0')||(*char_ptr==';')||(*char_ptr=='='))break;//my be next?
            else
               {
               char_ptr++;
               table_char_ptr++;
               }
            }
         table_index++;
         }    
      }
   PROCESS_NEXT_CMD:
   end_ptr=start_ptr+*cmd_len;//set end_ptr to end of this command
   start_ptr=end_ptr+1;//set start_ptr to start of next command
   end_ptr = strchr(start_ptr, ';');//set end_ptr to end of next command
   if (end_ptr == NULL)return NULL;//end of next command not finded
   *cmd_len=(uint8_t)(end_ptr-start_ptr);
   return start_ptr; 
   }

/*void MakeAnswerSms(ProcessSmsAnswerType* ptr,uint32_t buf_size)
   {
	// OutDebugParserSms("MAKE ANSWER SMS");
	 char temp[MAX_SMS_SIZE/2];
	 for(uint32_t i=0;i<buf_size;i++)ptr->sms_ptr[i]=0;	 
   sprintf(temp,"PORT=%u;",config.main_server.port);
	 if(strlen(temp)<buf_size-strlen(ptr->sms_ptr))strcat(ptr->sms_ptr,temp);
   sprintf(temp,"IP=%s;",config.main_server.ip);  
	 if(strlen(temp)<buf_size-strlen(ptr->sms_ptr))strcat(ptr->sms_ptr,temp);
   sprintf(temp,"APN=%s;",config.gprs.apn);
	 if(strlen(temp)<buf_size-strlen(ptr->sms_ptr))strcat(ptr->sms_ptr,temp); 
   sprintf(temp,"NAV FILTER=%u;",config.nav_filter); 
	 if(strlen(temp)<buf_size-strlen(ptr->sms_ptr))strcat(ptr->sms_ptr,temp); 
	 sprintf(temp,"STORE TIME=%u;",config.pointStore.time); 
		if(strlen(temp)<buf_size-strlen(ptr->sms_ptr))strcat(ptr->sms_ptr,temp); 
		sprintf(temp,"STORE ANGLE=%u;",config.pointStore.angle); 
		if(strlen(temp)<buf_size-strlen(ptr->sms_ptr))strcat(ptr->sms_ptr,temp); 
		sprintf(temp,"STORE DIST=%u;",config.pointStore.distance); 
		if(strlen(temp)<buf_size-strlen(ptr->sms_ptr))strcat(ptr->sms_ptr,temp);
    sprintf(temp,"SEND TIME=%u;",config.packetsend.time); 
		if(strlen(temp)<buf_size-strlen(ptr->sms_ptr))strcat(ptr->sms_ptr,temp); 	
    sprintf(temp,"SEND DIST=%u;",config.packetsend.distance); 
		if(strlen(temp)<buf_size-strlen(ptr->sms_ptr))strcat(ptr->sms_ptr,temp); 
    if(config.rs485.id1!=0)
	    {
       sprintf(temp,"1S485ID=%u;",config.rs485.id1); 
		   if(strlen(temp)<buf_size-strlen(ptr->sms_ptr))strcat(ptr->sms_ptr,temp);
	    }
     if(config.rs485.id2!=0)
	    {
       sprintf(temp,"2S485ID=%u;",config.rs485.id2); 
		   if(strlen(temp)<buf_size-strlen(ptr->sms_ptr))strcat(ptr->sms_ptr,temp);
	    }	
     if(config.rs485.id3!=0)
	    {
       sprintf(temp,"3S485ID=%u;",config.rs485.id3); 
		   if(strlen(temp)<buf_size-strlen(ptr->sms_ptr))strcat(ptr->sms_ptr,temp);
	    }	
     if(config.rs485.id4!=0)
	    {
       sprintf(temp,"4S485ID=%u;",config.rs485.id4); 
		   if(strlen(temp)<buf_size-strlen(ptr->sms_ptr))strcat(ptr->sms_ptr,temp);
	    }		
 //OutDebugParserSmsSprintf1(ptr->sms_buf,(uint32_t)ptr->sms_buf);			
   }*/
	 
#ifdef DEBUG_SMS_PARSER  
void OutDebugParserSms( char const *dbg_msg)
   {
   QS_BEGIN(QS_SMS_PARSER, &SMS_PARSER)                                 
   QS_STR(dbg_msg);                              
   QS_END()
   } 
	 
	 void OutDebugParserSmsSprintf1( char const *str,uint32_t val)
   {
   QS_BEGIN(QS_SMS_PARSER, &SMS_PARSER)                                  
   QS_STR(str); 
   QS_U32(4, val);       
   QS_END()
   }
#endif	 
	 

	





