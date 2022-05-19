
#include "stm32f10x.h"
#include "qp_port.h"
#include "modem_parser.h"
#include "modem.h"
#include "Control.h"
#include "ModemRingBuf.h"
#include "modem_signals.h"
#include <ctype.h>
#include <stdlib.h>

Q_DEFINE_THIS_MODULE("modem_parser.c")

#define STATIC_BUF_SIZE 50

ModemMsgType msg_tbl[]=
{
   {"OK\r\n",MODEM_OK_SIG},
//   {"NO CARRIER\r\n",MODEM_NO_CARRIER_SIG},
   {"ERROR\r\n",MODEM_ERROR_SIG},
   {"CLOSED\r\n",MODEM_CLOSED_SIG},
   {"SEND OK\r\n",MODEM_SEND_OK_SIG},
   {"SHUT OK\r\n",MODEM_SHUT_OK_SIG},
   {"CLOSE OK\r\n",MODEM_CLOSE_OK_SIG},
   {"DEACT OK\r\n",MODEM_DEACT_OK_SIG},
   {"+CMGF: 0\r\n",MODEM_PDU_MODE_SIG},
   {"+CMGF: 1\r\n",MODEM_TEXT_MODE_SIG},
   {"+CGATT: 1\r\n",MODEM_GPRS_ATTACHED_SIG},
   {"+CGATT: 0\r\n",MODEM_GPRS_NOT_ATTACHED_SIG},
   {"CONNECT OK\r\n",MODEM_CONNECT_OK_SIG},
   {"CONNECT FAIL\r\n",MODEM_CONNECT_FAIL_SIG},
   {"Call Ready\r\n",MODEM_CALL_READY_SIG},
	 {"SMS Ready\r\n",MODEM_SMS_READY_SIG},
   {"NORMAL POWER DOWN\r\n",MODEM_NORMAL_POWER_DOWN_SIG},
   {"+IPR: 115200\r\n",MODEM_BAUDRATE_115200_SIG},
   {"+CREG: 0,0\r\n",MODEM_CREG_NOT_REGISTERED_NO_OPERATOR_SEARCHING_SIG}, 
   {"+CREG: 0,1\r\n",MODEM_CREG_REGISTERED_HOME_NETWORK_SIG},              
   {"+CREG: 0,2\r\n",MODEM_CREG_NOT_REGISTERED_BUT_OPERATOR_SEARCHING_SIG},
   {"+CREG: 0,3\r\n",MODEM_CREG_REGISTRATION_DENIED_SIG},                  
   {"+CREG: 0,4\r\n",MODEM_CREG_UNKNOWN_ANSWER_SIG},                       
   {"+CREG: 0,5\r\n",MODEM_CREG_REGISTERED_ROAMING_SIG},                   
   {"+CMS ERROR: 604\r\n",MODEM_CMS_ERROR_604_SIG},
   {"STATE: TCP CLOSED\r\n",MODEM_STATE_TCP_CLOSED_SIG},
   {"STATE: TCP CLOSING\r\n",MODEM_STATE_TCP_CLOSING_SIG},
   {"STATE: IP START\r\n",MODEM_STATE_IP_START_SIG},
   {"STATE: IP CONFIG\r\n",MODEM_STATE_IP_CONFIG_SIG},
   {"STATE: IP INITIAL\r\n",MODEM_STATE_IP_INITIAL_SIG},
   {"STATE: PDP DEACT\r\n",MODEM_STATE_PDP_DEACT_SIG},
   {"STATE: IP STATUS\r\n",MODEM_STATE_IP_STATUS_SIG},
   {"STATE: TCP CONNECTING\r\n",MODEM_STATE_TCP_CONNECTING_SIG},
   {"STATE: CONNECT OK\r\n",MODEM_STATE_CONNECT_OK_SIG},
   {"+PDP: DEACT\r\n",MODEM_PDP_DEACT_SIG},
   {"ALREADY CONNECT\r\n",MODEM_ALREADY_CONNECT_SIG},
   {"+CPIN: NOT INSERTED\r\n",MODEM_SIM_NOT_INSERTED_SIG},
   {"+CPIN: NOT READY\r\n",MODEM_SIM_NOT_READY_SIG},
   {"+CPIN: READY\r\n",MODEM_SIM_READY_SIG},
   {"+CPIN: SIM PIN\r\n",MODEM_SIM_PIN_SIG},
   {"+CPIN: SIM PUK\r\n",MODEM_SIM_PUK_SIG},
   {"+CPIN: PH_SIM PIN\r\n",MODEM_PH_SIM_PIN_SIG},
   {"+CPIN: PH_SIM PUK\r\n",MODEM_PH_SIM_PUK_SIG},
   {"+CPIN: SIM PIN2\r\n",MODEM_SIM_PIN2_SIG},
   {"+CPIN: SIM PUK2\r\n",MODEM_SIM_PUK2_SIG},

   {NULL,0},//NULL is end of table
};


typedef struct
   {
   QActive super;
	 SmsType sms; 
   uint32_t message_buf_index;
   uint32_t server_answer_len; 
   uint8_t sms_size;
   uint8_t max_msg_table_len; 
   uint8_t min_msg_table_len; 		 
   uint16_t temp_index; 
   uint16_t tick_counter;		 
   QTimeEvt timeEvt;    
   } MPAR;

	
static QEvt const SmsIsDeadEvt = { MODEM_SMS_IS_READ_SIG, 0U, 0U};
static QEvt const NewSmsEvt = { MODEM_NEW_SMS_RECEIVED_SIG, 0U, 0U};
static QEvt const LocalIpEvt = { MODEM_LOCAL_IP_SIG, 0U, 0U};
static QEvt const DataReceivedEvt = { MODEM_TCP_DATA_RECEIVED_SIG, 0U, 0U};
static QEvt const IpSendEvt = { MODEM_IPSEND_SIG, 0U, 0U};
#ifdef DEBUG_MODEM_PARSER
static void OutDebugModemParser( char const *dbg_msg);
static void OutDebugModemParserSprintf( char const *str,uint32_t val);
#else
   #define OutDebugModemParser(x) __nop()
   #define OutDebugModemParserSprintf(x1,x2) __nop()
#endif

//char dbg_sms[]={"\r\n+CMGR: \"REC UNREAD\",\"+380930158409\",\"\",\"16/09/16,09:43:04+12\"\r\n7777;gprs;\r\n"};
static const char CRLFstr[]={"\r\n"};  
static uint8_t packet_rx_buf[PACKET_RX_BUF_SIZE]; 
uint8_t modem_ring_buf[MODEM_RING_BUF_SIZE];
static uint8_t message_buf[PARSER_MESSAGE_BUF_SIZE]; 
static uint8_t static_buf[STATIC_BUF_SIZE];
MPAR mpar; 
QActive * const AO_ModemParser = &mpar.super; 
static void MsgTable_init(MPAR* me,ModemMsgType* tbl_ptr);
static uint8_t CalcCharInString(const char *str,char ch);
static uint8_t CalcDigitInString(const char *str);
static int CheckStringIsDigit(char *str); 
static QState Mparser_initial(MPAR * const me, QEvt const * const e);
static QState Mparser_search_start_of_message(MPAR * const me, QEvt const * const e);
static QState Mparser_search_LF(MPAR * const me, QEvt const * const e);
static QState Mparser_copy_unread_sms_to_buf(MPAR * const me, QEvt const * const e);
static QState Mparser_copy_packet_to_buf(MPAR * const me, QEvt const * const e);

void Mparser_ctor(void)
   {
   MPAR *me = &mpar;
   QActive_ctor(&me->super, Q_STATE_CAST(&Mparser_initial));
   }


QState Mparser_initial(MPAR * const me, QEvt const * const e)
   {
   (void) e; 

   QS_SIG_DICTIONARY(MODEM_PARSER_TIC_SIG, me);//private signal
   QS_OBJ_DICTIONARY(&mpar); 
   QS_FUN_DICTIONARY(&Mparser_initial);
   QS_FUN_DICTIONARY(&Mparser_search_LF);
   QS_FUN_DICTIONARY(&Mparser_copy_packet_to_buf);
   QS_FUN_DICTIONARY(&Mparser_copy_unread_sms_to_buf);
   QS_FUN_DICTIONARY(&Mparser_search_start_of_message);

   //QS_SIG_DICTIONARY(TIC_100ms_SIG,(void *)0); // global signal	  
   //QS_FILTER_SM_OBJ(&modem_parser);
   QTimeEvt_ctor(&me->timeEvt, MODEM_PARSER_TIC_SIG);       /* private time event ctor */
   QTimeEvt_postEvery(&me->timeEvt,(QActive *)me, MODEM_PARSER_TICK_PERIOD );
   ModemRingBuf_init(modem_ring_buf,MODEM_RING_BUF_SIZE);
   MsgTable_init(me,msg_tbl);
	 //for(int i=0;i<sizeof(dbg_sms);i++) ModemRingBuf_push(dbg_sms[i]);
		 
   return Q_TRAN(&Mparser_search_LF);
   }

QState Mparser_search_start_of_message(MPAR * const me, QEvt const * const e)
   {
   char ch;
   static uint8_t index;
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            index=0;
         }
         return Q_HANDLED();
      case MODEM_PARSER_TIC_SIG:
         {
            while (ModemRingBuf_count())
               {
               ch=ModemRingBuf_pop();
               switch (index)
                  {
                  case 0: if (ch==CRLFstr[0])index=1;
                     break;
                  case 1: if (ch==CRLFstr[1])return  Q_TRAN(&Mparser_search_LF);
                     break;
                  }
               }
         }
				  return Q_HANDLED();
			 case Q_EXIT_SIG:  return Q_HANDLED();
      }
   return Q_SUPER(&QHsm_top);
   }
	 
	 


QState Mparser_search_LF(MPAR * const me, QEvt const * const e)
   {
   QState ret= Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            me-> message_buf_index=0;
         }
         return ret;
      case MODEM_PARSER_WAITING_IP_SEND_SIG: return  Q_TRAN(&Mparser_search_start_of_message);
      case MODEM_PARSER_TIC_SIG:
         {
            PARSE:
            if (ModemRingBuf_count())
               {
               while (ModemRingBuf_count())
                  {
                  char ch;
                  message_buf[me->message_buf_index++]= ch=ModemRingBuf_pop();
                  if (ch=='\n')
                     {
                     //me->mbipsend_flag=0;
                     message_buf[me->message_buf_index]='\0';
                     uint8_t table_index=0,message_len;
                     char *char_ptr;
//-----------------------------------------------------------------------------

                     if (me->message_buf_index<=me->max_msg_table_len && me->message_buf_index>=me->min_msg_table_len)
                        while (msg_tbl[table_index].msg!=NULL)
                           {
                           const char *table_char_ptr=msg_tbl[table_index].msg;
                           char_ptr=(char*)message_buf;
                           message_len=0;
                           while (*char_ptr==*table_char_ptr)
                              {
                              message_len++;
                              if ((*(table_char_ptr+1)=='\0') && (*(char_ptr+1)=='\0'))//finded !
                                 {
                                 SmallEvt *pe = Q_NEW(SmallEvt, msg_tbl[table_index].sig);
                                 QACTIVE_POST(AO_Modem, &pe->super, me);
                                 me-> message_buf_index=0;
                                 goto PARSE;
                                 }
                              else
                                 {
                                 char_ptr++;
                                 table_char_ptr++;
                                 }
                              }
                           table_index++;
                           }
                     switch (me-> message_buf_index)
                        {
                        case 9:
                           {
                              if ((3==CalcCharInString((const char*)message_buf,'.'))&&(4==CalcDigitInString((const char*)message_buf)))
                                 {
                                 QACTIVE_POST(AO_Modem, &LocalIpEvt, me);
                                 }
                              else goto DEFAULT;
                           }
                           break;
                        case 10:
                           {
                              if ((3==CalcCharInString((const char*)message_buf,'.'))&&(5==CalcDigitInString((const char*)message_buf)))
                                 {
                                  QACTIVE_POST(AO_Modem, &LocalIpEvt, me);
                                 }
                              else goto DEFAULT;
                           }
                           break;
                        case 11:
                           {
                              if ((3==CalcCharInString((const char*)message_buf,'.'))&&(6==CalcDigitInString((const char*)message_buf)))
                                 {
                                  QACTIVE_POST(AO_Modem, &LocalIpEvt, me);
                                 }
                              else goto DEFAULT;
                           }
                           break;
                        case 12:
                           {
                              if ((3==CalcCharInString((const char*)message_buf,'.'))&&(7==CalcDigitInString((const char*)message_buf)))
                                 {
                                  QACTIVE_POST(AO_Modem, &LocalIpEvt, me);
                                 }
                              else goto DEFAULT;
                           }
                           break;
                        case 13:
                           {
                              if ((3==CalcCharInString((const char*)message_buf,'.'))&&(8==CalcDigitInString((const char*)message_buf)))
                                 {
                                 QACTIVE_POST(AO_Modem, &LocalIpEvt, me);
                                 }
                              else goto DEFAULT;
                           }
                           break;
                        case 14:
                           {
                              if ((3==CalcCharInString((const char*)message_buf,'.'))&&(9==CalcDigitInString((const char*)message_buf)))
                                 {
                                 QACTIVE_POST(AO_Modem, &LocalIpEvt, me);
                                 }
                              else goto DEFAULT;
                           }
                           break;
                        case 15:
                           {
                              if (0==strncmp((const char*)message_buf,"+CMTI: \"SM\",",12))
                                 {
                                 QACTIVE_POST(AO_Modem, &NewSmsEvt, me);
                                 }
                              else if ((3==CalcCharInString((const char*)message_buf,'.'))&&(10==CalcDigitInString((const char*)message_buf)))
                                 {
                                 QACTIVE_POST(AO_Modem, &LocalIpEvt, me);
                                 }
                              else goto DEFAULT;
                           }
                           break;
                        case 16:
                           {   
                              if ((3==CalcCharInString((const char*)message_buf,'.'))&&(11==CalcDigitInString((const char*)message_buf)))
                                 {
                                  QACTIVE_POST(AO_Modem, &LocalIpEvt, me);
                                 }
                              else goto DEFAULT;
                           }
                           break;
                        case 17:
												case 20:
                           {
                              //uint8_t index=15;
                              Q_ASSERT(me-> message_buf_index-2<STATIC_BUF_SIZE);
                              //  memcpy(static_buf,message_buf,15); 
                              //static_buf[15]='\0'; 
                              //strncpy((char*)static_buf,(char*)message_buf,me-> message_buf_index-2);
														  memcpy(static_buf,message_buf,me-> message_buf_index-2);
														  static_buf[me-> message_buf_index-2]='\0'; 
                              if (1==CheckStringIsDigit((char*)static_buf))
                                 {
                                 DataPtrEvt *pe = Q_NEW(DataPtrEvt,MODEM_IMEI_IMSI_SIG);
                                 pe->ptr=static_buf;
                                 QACTIVE_POST(AO_Modem,(QEvt*)pe, me);
                                 }
                              else if ((3==CalcCharInString((const char*)message_buf,'.'))&&(12==CalcDigitInString((const char*)message_buf)))
                                 {
                                  QACTIVE_POST(AO_Modem, &LocalIpEvt, me);
                                 }
                              else goto DEFAULT;
                           }
                           break;
                           DEFAULT:
                        default:
                           {
                              char *str_ptr,*comma_ptr;
                              uint32_t temp_buf_index;
															 if (NULL!=(str_ptr=strstr((const char*)message_buf,"+CMT: \"")))
                                 {
                                 QACTIVE_POST(AO_Modem, &NewSmsEvt, me);
                                 }
                              else if (NULL!=(str_ptr=strstr((const char*)message_buf,"+CSQ:")))
                                 {
                                 char temp_buf[25];
                                 memset(temp_buf,0,sizeof(temp_buf));
                                 temp_buf_index=0;
                                 str_ptr+=5;
                                 while ((temp_buf_index<sizeof(temp_buf))&&(*str_ptr!=','))
                                    {
                                    temp_buf[temp_buf_index++]=*str_ptr++;
                                    }
                                 GsmqMessageType *pe= Q_NEW(GsmqMessageType,MODEM_GSMQ_SIG);
                                 pe->gsmq =atoi(temp_buf);
                                 QACTIVE_POST(AO_Modem,(QEvt*)pe, me);
                                 }
                              else if (NULL!=(str_ptr=strstr((const char*)message_buf,"+CMGR:")))
                                 {
                                 if (NULL!=(str_ptr=strstr((const char*)message_buf,"REC READ")))
                                    {
                                    QACTIVE_POST(AO_Modem, &SmsIsDeadEvt, me);
                                    }
                                 else if (NULL!=(str_ptr=strstr((const char*)message_buf,"REC UNREAD")))
                                    {
																			OutDebugModemParser("REC UNREAD");
                                    uint32_t comma_index=me->message_buf_index;
                                    do
                                       {
                                       if (message_buf[--comma_index]==',')break;
                                       }while (comma_index);
                                    if (message_buf[comma_index]==',')//last comma is finded
                                       {
                                       char temp_buf[25];
                                       strncpy(temp_buf,(const char*)&message_buf[comma_index+1],me->message_buf_index-(comma_index+1));
                                       temp_buf[me->message_buf_index-(comma_index+1)]='\0';
                                      // me->sms_size=atoi(temp_buf);
																				if (NULL!=(str_ptr = strchr(temp_buf, '+')))
																				{
																					 me->sms_size=atoi(str_ptr+1);
																				}
                                       if (me->sms_size<=160 && me->sms_size>0)
                                          {
                                          if (NULL!=(str_ptr=strchr((const char*)message_buf,',')))
                                             {
                                             str_ptr++;
                                             if (NULL!=(comma_ptr = strchr(str_ptr, ',')))
                                                {
                                                strncpy(me->sms.tf_num, str_ptr, comma_ptr - str_ptr);
                                                me->sms.tf_num[comma_ptr - str_ptr] = '\0';
                                                return Q_TRAN(&Mparser_copy_unread_sms_to_buf);
                                                }
                                             }
                                          }
                                       }
                                    }
                                 }
																  else if (NULL!=(str_ptr=strstr((const char*)message_buf,"+CLIP:")))
																	{
																		char* end_ptr;
                                    if (NULL!=(str_ptr=strchr((const char*)message_buf,'"')))
                                        {
                                           if (NULL!=(end_ptr=strchr(str_ptr+1,'"')))
                                              {
																								end_ptr++;
                                                static char tf_buf[MAX_PHONE_STR_LEN];
																								strncpy(tf_buf, str_ptr, end_ptr - str_ptr);
																								 DataPtrEvt *pe = Q_NEW(DataPtrEvt, MODEM_RING_SIG);
                                                 pe->ptr=tf_buf;
                                                 QACTIVE_POST(AO_Modem, &pe->super, me);
                                              }	 
                                        }	 
																	}
                              else if ((NULL!=(str_ptr=strstr((const char*)message_buf,"+CIPRXGET:1")))||(NULL!=(str_ptr=strstr((const char*)message_buf,"+CIPRXGET: 1"))))
                                 {
                                 QACTIVE_POST(AO_Modem, &DataReceivedEvt, me);
                                 }
                              else if (NULL!=(str_ptr=strstr((const char*)message_buf,"+CIPRXGET:2")))
                                 {
                                 char temp_buf[25];
                                 memset(temp_buf,0,sizeof(temp_buf));
                                 temp_buf_index=0;
                                 str_ptr+=12;
                                 while ((temp_buf_index<sizeof(temp_buf))&&(*str_ptr!=','))
                                    {
                                    temp_buf[temp_buf_index++]=*str_ptr++;
                                    }
                                 me->server_answer_len=atoi(temp_buf);
                                 return Q_TRAN(&Mparser_copy_packet_to_buf);
                                 }
															else if (NULL!=(str_ptr=strstr((const char*)message_buf,"+CIPRXGET: 2")))
                                 {
                                 char temp_buf[25];
                                 memset(temp_buf,0,sizeof(temp_buf));
                                 temp_buf_index=0;
                                 str_ptr+=13;
                                 while ((temp_buf_index<sizeof(temp_buf))&&(*str_ptr!=','))
                                    {
                                    temp_buf[temp_buf_index++]=*str_ptr++;
                                    }
                                 me->server_answer_len=atoi(temp_buf);
                                 return Q_TRAN(&Mparser_copy_packet_to_buf);
                                 }
                              else if (NULL!=(str_ptr=strstr((const char*)message_buf,"+CUSD: 0")))
                                 {
																	 str_ptr+=9;
																	  strlcpy(me->sms.buf,str_ptr,MAX_SMS_SIZE);
																	  DataPtrEvt *pe = Q_NEW(DataPtrEvt, MODEM_USSD_SIG);
                                    pe->ptr=(void*)&me->sms;
                                    QACTIVE_POST(AO_Modem, &pe->super, me);
                                 }
                           }
                           break;
                        }
                     me-> message_buf_index=0;//reset parser											 
//***********************************************************************************************************	
                     }
                  else if (me->message_buf_index==2)
                     {
                     if (0==strncmp((char*)message_buf,"> ",2))
                        {
                        QACTIVE_POST(AO_Modem, &IpSendEvt, me);
                        me-> message_buf_index=0;
                        }
                     }
                  else if (me->message_buf_index==PARSER_MESSAGE_BUF_SIZE)
                     {
                     me-> message_buf_index=0;
                     }
                  else if (ch=='\0')
                     {
                     me-> message_buf_index=0;
                     }
                  }
               }
            else if (me->message_buf_index!=0)//unexpected gup between chars
               {
               me-> message_buf_index=0;
               }
         }
         return ret;
      case Q_EXIT_SIG: return ret;
      }
   return Q_SUPER(&QHsm_top);
   }


/*QState Mparser_waiting_ipsend(ModemParser * const me, QEvt const * const e)
   {
      char ch;
     static uint8_t index;
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            index=0;
         }
         return Q_HANDLED();
      case MODEM_PARSER_TIC_SIG:
         {
          while (ModemRingBuf_count())
               {
                  ch=ModemRingBuf_pop();
                 switch(index)
                 {
                   case 0: if (ch==IPSENDstr[index])index++;
                           else  return  Q_TRAN(&Mparser_search_LF);	
                           break;
                   case 1: if (ch==IPSENDstr[index])index++;
                           else  return  Q_TRAN(&Mparser_search_LF);	
                           break;
                   case 2: if (ch==IPSENDstr[index])index++;
                           else  return  Q_TRAN(&Mparser_search_LF);	
                           break;
                   case 3: if (ch==IPSENDstr[index])
                              {
                                SmallEvt *pe = Q_NEW(SmallEvt, MODEM_IPSEND_SIG);
                                QACTIVE_POST(AO_Modem, &pe->super, me);
                              }
                           return  Q_TRAN(&Mparser_search_LF);	
                 }
             }
         }
      }
   return Q_SUPER(&QHsm_top);
   }*/




QState Mparser_copy_unread_sms_to_buf(MPAR * const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            me-> temp_index=0;
					  me->tick_counter=0;
					uint8_t temp=ModemRingBuf_count();
					 OutDebugModemParserSprintf( "LEN=",temp);
         }  
         return Q_HANDLED();
      case MODEM_PARSER_TIC_SIG://10 ms
         {
            while (ModemRingBuf_count()&&me->temp_index<MAX_SMS_SIZE-1)
               {
								me->tick_counter=0;
                me->sms.buf[me->temp_index]=ModemRingBuf_pop();
								if (me->sms.buf[me->temp_index]=='\r')
                  {
                  me->sms.buf[me->temp_index]='\0';
									 	me->sms_size=strlen(me->sms.buf);
                  DataPtrEvt *pe = Q_NEW(DataPtrEvt, MODEM_SMS_IS_UNREAD_SIG);
                  pe->ptr=(void*)&me->sms;
										OutDebugModemParser("RECEIVED SMS IS ");
									OutDebugModemParser(me->sms.buf);
                  QACTIVE_POST(AO_Modem, &pe->super, me);
                  return  Q_TRAN(&Mparser_search_LF);
                  }
									else me->temp_index++;
               }
           if (me->temp_index!=0 || me->temp_index>=MAX_SMS_SIZE-1)//unexpected gup between chars
						//if (++me->tick_counter==5)//50 ms
               {
								 OutDebugModemParserSprintf("BIG GUP ERROR! received=",me->temp_index);
								 me->sms.buf[me->temp_index]='\0';
								 OutDebugModemParser(me->sms.buf);
               return  Q_TRAN(&Mparser_search_LF);
               }
         }
         return Q_HANDLED();
      case Q_EXIT_SIG:  return Q_HANDLED();
      }
   return Q_SUPER(&QHsm_top);
   }

QState Mparser_copy_packet_to_buf(MPAR * const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            me->temp_index=0;
					 me->tick_counter=0;
         }
         return Q_HANDLED();
      case MODEM_PARSER_TIC_SIG:
         {
            while (ModemRingBuf_count())
               {
								me->tick_counter=0;
               packet_rx_buf[me->temp_index++]=ModemRingBuf_pop();
               if (me->temp_index==me->server_answer_len)
                  {
                  DataPtrEvt *pe = Q_NEW(DataPtrEvt, MODEM_SERVER_DATA_SIG);
                  pe->ptr=packet_rx_buf;
                  pe->data.size=me->server_answer_len;
                  QACTIVE_POST(AO_Modem, &pe->super, me);
                  return  Q_TRAN(&Mparser_search_LF);
                  }
               }
            if (++me->tick_counter==100)//10*100=1000 ms MAX GAP
               {
								OutDebugModemParserSprintf("BIG GUP ERROR! received=",me->temp_index);
               return  Q_TRAN(&Mparser_search_LF);
               }
         }
         return Q_HANDLED();
      case Q_EXIT_SIG:  return Q_HANDLED();
      }
   return Q_SUPER(&QHsm_top);
   }

uint8_t CalcCharInString(const char *str,char ch)
   {
   uint8_t result=0;
   while (*str)
      {
      if (ch==(*str++))result++;
      }
   return result;
   }

uint8_t CalcDigitInString(const char *str)
   {
   uint8_t result=0;
   while (*str)
      {
      if (isdigit(*str++))result++;
      }
   return result;
   }

int CheckStringIsDigit(char *str)
   {
   while (*str)
      {
      if (!isdigit(*str++))return 0;
      }
   return 1;
   }


void MsgTable_init(MPAR* me,ModemMsgType* tbl_ptr)
   {
   uint8_t index=0;
		me->max_msg_table_len=0;
		me->min_msg_table_len=0xFF;

   while (msg_tbl[index].msg!=NULL)
      {
      msg_tbl[index].len=strlen(msg_tbl[index].msg);
      if (me->max_msg_table_len<msg_tbl[index].len)me->max_msg_table_len=msg_tbl[index].len;
			if (me->min_msg_table_len>msg_tbl[index].len)me->min_msg_table_len=msg_tbl[index].len;
      index++;
      }
   }

#ifdef DEBUG_MODEM_PARSER	 
void OutDebugModemParser( char const *dbg_msg)
   {
   QS_BEGIN(QS_MODEM_PARSER, AO_ModemParser)                                 
   QS_STR(dbg_msg);                              
   QS_END()
   }  

 void OutDebugModemParserSprintf( char const *str,uint32_t val)
   {
   QS_BEGIN(QS_MODEM_PARSER, AO_ModemParser)                                  
   QS_STR(str); 
   QS_U32(4, val);  		 
   QS_END()
   }	 

#endif



