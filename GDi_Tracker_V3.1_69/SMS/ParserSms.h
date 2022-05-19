
#ifndef _PARSER_SMS
#define _PARSER_SMS

#include <stdint.h>
#include "user_types.h"
#include <RTL.H>


#define MIN_CMD_LEN 3

typedef enum
{
	PROCESS_SMS_FORMAT_ERROR,
	PROCESS_SMS_PASSWORD_ERROR,
	PROCESS_SMS_OK,
	
}ProcessSmsAnswerEnumType;

typedef struct
{
	ProcessSmsAnswerEnumType answer;
  FlagsType flag;
	char* sms_ptr;
}ProcessSmsAnswerType;


extern void ParserSms(ProcessSmsAnswerType* ptr);
extern const char *FindPassword(const char *str_ptr,uint8_t *cmd_len);
extern const char *ProcessCommand(const char *start_ptr,uint8_t *cmd_len,FlagsType* flag_ptr);


#endif




