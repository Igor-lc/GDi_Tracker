
#include "stm32f10x.h"
#include "qp_port.h"
#include "modem.h"
#include "modem_signals.h"
#include "modem_sim900.h"
#include "ParserSms.h"
#include "config.h"
#include <stdio.h>
#include "Control.h"
#include "Disk.h"
#include <string.h>
#include "modem_parser.h"
#include "bsp.h"
#include "crc16.h"
#include "ByteStream.h"
#include "LedIndicator.h"
#define _DEBUG_SMS


typedef struct
   {
   QActive super;
   struct 
      {
      unsigned get_ready:1;            
      unsigned packet:1;
      unsigned check_gsmq:1;
      //----------------------------------------
      unsigned fatal_error:1;
      unsigned sms_new:1;
		  unsigned sms_send:1;
      unsigned upd:1;
      unsigned off:1;
      }flag;
   struct
      {
      uint16_t gsmq;
      int16_t tick; 
      uint8_t resend;
      }counter;
   char command_buf[50];
   int done; 
	 int sim_error;
	 FatalErrorTimerType fatal_error_timer;
   ErrorCounterType error_counter;
   DataBufType data_packet;
   DataBufType config_packet;
   QStateHandler ret_ptr;
   SmsType* input_sms_ptr;    
   char sms_tf[MAX_PHONE_STR_LEN];
   char ring_tf[MAX_PHONE_STR_LEN];
   uint8_t sms_index;
   uint32_t error_timeout;
   QTimeEvt TimeEvt;
   SIM900 msm;
   } MOD;

static uint8_t auth_pack_buf[AUTH_PACK_MAX_SIZE];
char send_sms[170]; 
static QSignal debug_sig;
static MOD modem; 
GsmStateType GsmState=gsm_modem_off;
GprsStateType GprsState=gprs_off;
ServerStateType ServerState=server_not_connected;
QActive * const AO_Modem = &modem.super; 
ProcessSmsAnswerType parser_sms_answer;

static QEvt const ControlModemIsOffEvt ={ CONTROL_MODEM_IS_OFF_SIG, 0U, 0U}; 
static QEvt const ControlUpdPacketLevelErrorEvt = { CONTROL_UPD_PACKET_LEVEL_ERROR_SIG, 0U, 0U};   
static QEvt const ControlUpdTransportLevelErrorEvt = { CONTROL_UPD_TRANSPORT_LEVEL_ERROR_SIG, 0U, 0U};   
static QEvt const ControlUpdateConnectOkEvt = { CONTROL_UPDATE_CONNECT_OK_SIG, 0U, 0U};
static QEvt const ParserWaitingIpSendEvt = { MODEM_PARSER_WAITING_IP_SEND_SIG, 0U, 0U};
static QEvt const SendPacketOkEvt = { CONTROL_SEND_PACKET_OK_SIG, 0U, 0U};
static QEvt const SendPacketErrorEvt = { CONTROL_SEND_PACKET_ERROR_SIG, 0U, 0U};
static QEvt const ModemReadyEvt = { CONTROL_MODEM_READY_SIG, 0U, 0U};
static QEvt const BadPacketEvt = { CONTROL_BAD_PACKET_SIG, 0U, 0U};
static uint32_t  XTDeviceAuthPacketMaker(MOD *me,uint8_t *ptr);
static uint32_t FM2200AuthPacketMaker(MOD *me,uint8_t *ptr);
static void SendMsmSignal(MOD *me, QSignal sig);
static void MakeGprsSms(char* ptr,uint32_t buf_size);
static void MakeNavSms(char* ptr,uint32_t size);
static void MakeRS485Sms(char* ptr,uint32_t size);
static void MakeVerSms(char* ptr,uint32_t size);
static void MakePackSms(char* ptr,uint32_t size);
static void MakeStatusSms(MOD *me,char* ptr,uint32_t size);
static void MakeModemSms(char* ptr,uint32_t size);
static void MakeImeiSms(MOD* me,char* ptr,uint32_t size);
static void ClearErrorCounters(MOD *me);
static void ModemUart_off(void);
static void ModemUart_on(void);
static void SelectSim(MOD * const me);
uint8_t modem_state_history[MODEM_STATE_HISTORY_SIZE]; 
static void ClearStateHistory(void);
static void AddStateHistory(uint8_t state);
static QState Modem_initial(MOD * const me, QEvt const * const e);

 
static QState Modem_top_1(MOD * const me, QEvt const * const e);//1
static QState Modem_power_on_start_up_2(MOD * const me, QEvt const * const e);//2 simerr
static QState Modem_power_on_start_up_error_3(MOD * const me, QEvt const * const e);//3 simerr
static QState Modem_restart_4(MOD * const me, QEvt const * const e);          //24
static QState Modem_hard_reset_5(MOD * const me, QEvt const * const e);//5 
static QState Modem_gsm_open_6(MOD * const me, QEvt const * const e); //6                    
static QState Modem_gsm_is_open_7(MOD * const me, QEvt const * const e);//7 
static QState Modem_gsm_open_error_8(MOD * const me, QEvt const * const e);//8
static QState Modem_open_gprs_9(MOD * const me, QEvt const * const e);//9
static QState Modem_close_gprs_10(MOD * const me, QEvt const * const e);//10
static QState Modem_gprs_is_open_state_11(MOD * const me, QEvt const * const e);    //11                
static QState Modem_open_gprs_error_12(MOD * const me, QEvt const * const e);       //12    
static QState Modem_tcp_is_open_state_13(MOD * const me, QEvt const * const e) ;    //13
static QState Modem_check_if_connected_14(MOD * const me, QEvt const * const e);    //14
static QState Modem_open_connection_15(MOD * const me, QEvt const * const e) ;      //15             
static QState Modem_close_connection_16(MOD * const me, QEvt const * const e) ;     //16
static QState Modem_open_connection_error_17(MOD * const me, QEvt const * const e); //17
static QState Modem_authorization_18(MOD * const me, QEvt const * const e);         //18       
static QState Modem_authorization_error_19(MOD * const me, QEvt const * const e) ;  //19
static QState Modem_sim_error_20(MOD * const me, QEvt const * const e);        //20 simerr
static QState Modem_sim_lock_error_21(MOD * const me, QEvt const * const e);   //21 
static QState Modem_send_packet_22(MOD * const me, QEvt const * const e);      //22
static QState Modem_send_packet_error_23(MOD * const me, QEvt const * const e);//23    
//---------------SMS--------------------------------------------------------------------------------
static QState Modem_sms_super_30(MOD * const me, QEvt const * const e);//30  OK
static QState Modem_sms_read_next_31(MOD * const me, QEvt const * const e); //31 OK
static QState Modem_sms_waiting_read_ok_32(MOD * const me, QEvt const * const e);//32 OK
static QState Modem_sms_waiting_unread_ok_33(MOD * const me, QEvt const * const e);//33 OK
static QState Modem_sms_delete_34(MOD * const me, QEvt const * const e) ;//34 OK
static QState ModemSmsRequestToSendAnswer_35(MOD * const me, QEvt const * const e) ;//35 OK
static QState ModemSmsSendAnsw36(MOD * const me, QEvt const * const e);//36 OK
static QState ModemSmsProcSmsFlags37(MOD * const me, QEvt const * const e) ;//37
static QState Modem_sms_check_balance_38(MOD * const me, QEvt const * const e) ;//38 OK
static QState Modem_sms_final_39(MOD * const me, QEvt const * const e) ;//39 OK
//--------------UPDATE----------------------------------------------------------------
static QState Modem_upd_super_40(MOD * const me, QEvt const * const e); 
static QState Modem_upd_open_conn_41(MOD * const me, QEvt const * const e) ;
static QState Modem_upd_auth_42(MOD * const me, QEvt const * const e) ;
static QState Modem_upd_send_packet_43(MOD * const me, QEvt const * const e) ;
static QState Modem_upd_ready_44(MOD * const me, QEvt const * const e) ;
static QState Modem_off_45(MOD * const me, QEvt const * const e);
static QState Modem_upd_get_remain_fw_data_46(MOD * const me, QEvt const * const e) ;
static QState Modem_upd_connect_error_47(MOD * const me, QEvt const * const e) ;
static QState ModemSmsRequestToSendServ48(MOD * const me, QEvt const * const e);
static QState ModemSmsSendServ49(MOD * const me, QEvt const * const e);
static QState ModemSmsDeleteAll50(MOD * const me, QEvt const * const e); 
//---------------FATAL ERROR------------------------------------------------------------------------
static QState Modem_power_on_start_up_fatal_error_60(MOD * const me, QEvt const * const e);//60
static QState Modem_gsm_open_fatal_error_61(MOD * const me, QEvt const * const e); //61
static QState Modem_open_gprs_fatal_error_62(MOD * const me, QEvt const * const e); //62
static QState Modem_open_connection_fatal_error_63(MOD * const me, QEvt const * const e);//63
static QState Modem_authorization_fatal_error_64(MOD * const me, QEvt const * const e);//64
static QState Modem_send_packet_fatal_error_65(MOD * const me, QEvt const * const e); //65
static QState Modem_sim_fatal_error_66(MOD * const me, QEvt const * const e);//66
static QState Modem_sim_lock_fatal_error_67(MOD * const me, QEvt const * const e);//67

void Modem_ctor(void)
   {
   MOD *me = &modem;
   ModemSM_ctor(&me->msm);
   QActive_ctor(&me->super, Q_STATE_CAST(&Modem_initial));
   QTimeEvt_ctor(&me->TimeEvt,  MODEM_TIMEOUT_SIG);
   }


QState Modem_initial(MOD * const me, QEvt const * const e)
   {
   (void) e; 

   ClearErrorCounters(me);
   me->flag.sms_new=0;
	 me->flag.sms_send=0;
   me->flag.get_ready=0;
   me->flag.packet=0;
   me->flag.fatal_error=0;   
   me->flag.off=0;
   me->flag.upd=0;  
   me->sim_error=0;		 
   SysInfo.gsmq=0;
   me->data_packet.size=0; 
   me->data_packet.ptr=0;  
   QS_OBJ_DICTIONARY(&modem);
   void *sim900=&modem.msm;
   QS_OBJ_DICTIONARY(sim900);   
   QS_FUN_DICTIONARY(&QHsm_top);
   QS_FUN_DICTIONARY(&Modem_initial);
   QS_FUN_DICTIONARY(&Modem_power_on_start_up_2);
   QS_FUN_DICTIONARY(&Modem_power_on_start_up_error_3);
   QS_FUN_DICTIONARY(&Modem_power_on_start_up_fatal_error_60);
   QS_FUN_DICTIONARY(&Modem_hard_reset_5);
   QS_FUN_DICTIONARY(&Modem_gsm_open_6);
   QS_FUN_DICTIONARY(&Modem_gsm_is_open_7);
   QS_FUN_DICTIONARY(&Modem_gsm_open_error_8);
   QS_FUN_DICTIONARY(&Modem_open_gprs_9);
   QS_FUN_DICTIONARY(&Modem_close_gprs_10);
   QS_FUN_DICTIONARY(&Modem_gprs_is_open_state_11);
   QS_FUN_DICTIONARY(&Modem_open_gprs_error_12);
   QS_FUN_DICTIONARY(&Modem_tcp_is_open_state_13);
   QS_FUN_DICTIONARY(&Modem_check_if_connected_14);
   QS_FUN_DICTIONARY(&Modem_open_connection_15);
   QS_FUN_DICTIONARY(&Modem_close_connection_16);
   QS_FUN_DICTIONARY(&Modem_open_connection_error_17);
   QS_FUN_DICTIONARY(&Modem_authorization_18);
   QS_FUN_DICTIONARY(&Modem_authorization_error_19);
   QS_FUN_DICTIONARY(&Modem_sim_error_20);
   QS_FUN_DICTIONARY(&Modem_sim_lock_error_21);
   QS_FUN_DICTIONARY(&Modem_send_packet_22);
   QS_FUN_DICTIONARY(&Modem_send_packet_error_23);
   QS_FUN_DICTIONARY(&Modem_restart_4);
   QS_FUN_DICTIONARY(&Modem_gsm_open_fatal_error_61);
   QS_FUN_DICTIONARY(&Modem_open_gprs_fatal_error_62);
   QS_FUN_DICTIONARY(&Modem_open_connection_fatal_error_63);
   QS_FUN_DICTIONARY(&Modem_authorization_fatal_error_64);
   QS_FUN_DICTIONARY(&Modem_send_packet_fatal_error_65); 
   QS_FUN_DICTIONARY(&Modem_sms_super_30);
   QS_FUN_DICTIONARY(&Modem_sms_read_next_31);
   QS_FUN_DICTIONARY(&Modem_sms_waiting_read_ok_32);
   QS_FUN_DICTIONARY(&Modem_sms_waiting_unread_ok_33);
   QS_FUN_DICTIONARY(&Modem_sms_delete_34);
   QS_FUN_DICTIONARY(&ModemSmsRequestToSendAnswer_35);
   QS_FUN_DICTIONARY(&ModemSmsSendAnsw36);
   QS_FUN_DICTIONARY(&ModemSmsProcSmsFlags37);
   QS_FUN_DICTIONARY(&Modem_sms_check_balance_38);
   QS_FUN_DICTIONARY(&Modem_sms_final_39);
   //---------------------------------------------------
   QS_FUN_DICTIONARY(&Modem_upd_super_40);
   QS_FUN_DICTIONARY(&Modem_upd_open_conn_41);
   QS_FUN_DICTIONARY(&Modem_upd_auth_42);
   QS_FUN_DICTIONARY(&Modem_upd_send_packet_43);
   QS_FUN_DICTIONARY(&Modem_upd_ready_44);
	 QS_FUN_DICTIONARY(&Modem_off_45);
   QS_FUN_DICTIONARY(&Modem_upd_get_remain_fw_data_46);
   QS_FUN_DICTIONARY(&Modem_upd_connect_error_47);
	 QS_FUN_DICTIONARY(&ModemSmsRequestToSendServ48);
	 QS_FUN_DICTIONARY(&ModemSmsSendServ49);
	 QS_FUN_DICTIONARY(&ModemSmsDeleteAll50);


   QS_SIG_DICTIONARY(MSM_ANSWER_SIG, me);
   QS_SIG_DICTIONARY(TIC_100ms_SIG,(void *)0); // global signal	
   //QS_FILTER_SM_OBJ(&modem);
   ModemSM_init(&me->msm);  
   QActive_subscribe(&me->super, TIC_100ms_SIG);
   ClearStateHistory();
   return Q_TRAN(&Modem_top_1);
 //  return Q_TRAN(&Modem_off_45);
   }

QState Modem_top_1(MOD * const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case 0: break;//start sig ignored
      case Q_ENTRY_SIG: 
         {
            AddStateHistory(1);
         }
         break;
      case Q_INIT_SIG: 
				return Q_TRAN(&Modem_power_on_start_up_2);
			
			case MODEM_OFF_SIG: me->flag.off=1; return Q_TRAN(&Modem_restart_4);
      case TIC_100ms_SIG:
         {
            if (me->flag.check_gsmq==1)
               {
               if (me->counter.gsmq==0)
                  {
                  ModemSendCommand("AT+CSQ\r\n");
                  me->counter.gsmq= GSMQ_TIMEOUT;
                  }
               else me->counter.gsmq--;
               }
            else me->counter.gsmq=0;
         }
         break;
      case MODEM_DATA_PACKET_SIG:
         {
            DataPtrEvt *pe=(DataPtrEvt*)e;
            me->data_packet.ptr=pe->ptr;
            me->data_packet.size=pe->data.size;
            me->flag.packet=1;
         }
         break;
      case MODEM_UPD_START_SIG:
         {
            me->flag.upd=1;
         }
         break;
      case MODEM_REQUEST_TO_SEND_SIG:
         {
            me->flag.get_ready=1;
         }
         break;
      case MODEM_OK_SIG:
         {
            OutDebugModem("UNH OK_SIG");
         }
         break;
      case MODEM_NEW_SMS_RECEIVED_SIG:
         {
            OutDebugModem("MODEM::top-NEW_SMS_RECEIVED_SIG");
            ind_flag.new_sms_received=1;
            me->flag.sms_new=1;
         }
         break;
				 case MODEM_SEND_SMS_SIG:
				 {
					 OutDebugModem("MODEM::top-MODEM_SEND_SMS_SIG");
            me->flag.sms_send=1;
				 }
        break;
      case MODEM_GSMQ_SIG:
         {
            SysInfo.gsmq=((GsmqMessageType*)e)->gsmq;
            char temp_print[15];
            sprintf(temp_print,"GSMQ=%u",SysInfo.gsmq);
            OutDebugModem(temp_print);
         }
         break;
      case MODEM_RING_SIG:
         {
            // DataPtrEvt *pe=(DataPtrEvt*)e;
            // strcpy(me->ring_tf,pe->ptr);
         }
         break;
      default:
         {
            debug_sig=e->sig;
            (void)debug_sig;
         }
         break;
      }
   return Q_SUPER(&QHsm_top);
   }


//---------UPDATE------------------------------------------------------------------- 
QState Modem_upd_super_40(MOD * const me, QEvt const * const e) 
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(40);
            me->flag.packet=0;
         }
         return Q_HANDLED() ;
      case Q_INIT_SIG: return Q_TRAN(&Modem_upd_open_conn_41);
				case MODEM_NEW_SMS_RECEIVED_SIG:
         {
            OutDebugModem("MODEM::upd_super-NEW_SMS_RECEIVED_SIG");
            ind_flag.new_sms_received=1;
            me->flag.sms_new=1;
					 QACTIVE_POST(AO_Control, &ControlUpdTransportLevelErrorEvt, me);
					  me->ret_ptr=(QStateHandler)&Modem_upd_super_40;
         }
         return Q_TRAN(&Modem_sms_super_30);
      case MODEM_UPD_FINAL_SIG: 
         {
            me->flag.upd=0;
         }       
         return Q_TRAN(&Modem_close_connection_16);
				 
      case MODEM_OFF_SIG: me->flag.off=1;
      case MODEM_UPD_RESTART_SIG:  return Q_TRAN(&Modem_restart_4);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG:  return Q_TRAN(&Modem_sim_error_20); 
      case Q_EXIT_SIG: return Q_HANDLED() ;
      }
   return Q_SUPER(&Modem_top_1);
   }



QState Modem_upd_open_conn_41(MOD * const me, QEvt const * const e) 
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         { 
            AddStateHistory(41);
            ServerState=server_connecting;
            GprsState=gprs_is_open;
            QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,BSP_TICKS_PER_SEC*70); //70 sec guard timeout
            sprintf(me->command_buf,"AT+CIPSTART=\"TCP\",\"%s\",\"%u\"\r\n",conf.server[SERVICE_SERVER].ip ,conf.server[SERVICE_SERVER].port);     
            me->done=ModemSendCommand(me->command_buf);
            OutDebugModem(me->command_buf); 
         }
         return Q_HANDLED() ;
      case MODEM_CONNECT_OK_SIG:
         {
            OutDebugModem("CONNECT_OK_SIG");         
         }
         return Q_TRAN(&Modem_upd_auth_42);
      case MODEM_TIMEOUT_SIG:
         {
            OutDebugModem("TIMEOUT_SIG");  
         }
         return Q_TRAN(&Modem_upd_connect_error_47);
      case MODEM_PDP_DEACT_SIG: 
         {
            OutDebugModem("PDP_DEACT_SIG");  
         }
         return Q_TRAN(&Modem_upd_connect_error_47);
      case MODEM_CLOSED_SIG: 
         {
            OutDebugModem("CLOSED_SIG");  
         }
         return Q_TRAN(&Modem_upd_connect_error_47);
      case MODEM_CONNECT_FAIL_SIG:
         {
            OutDebugModem("CONNECT_FAIL_SIG");  
         }
         return Q_TRAN(&Modem_upd_connect_error_47);
      case MODEM_ALREADY_CONNECT_SIG: 
         {
            OutDebugModem("ALREADY_CONNECT_SIG");  
         }
         return Q_TRAN(&Modem_upd_connect_error_47);
      case MODEM_ERROR_SIG: 
         {
            OutDebugModem("ERROR_SIG");  
         }
         return Q_TRAN(&Modem_upd_connect_error_47);
      case Q_EXIT_SIG:
         {
            QTimeEvt_disarm(&me->TimeEvt);
         }
         return Q_HANDLED();
      }
   return Q_SUPER(&Modem_upd_super_40);
   }


QState Modem_upd_auth_42(MOD * const me, QEvt const * const e) 
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(42);
            me->msm.send_packet.size=  XTDeviceAuthPacketMaker(me,auth_pack_buf);
            me->msm.send_packet.ptr=auth_pack_buf;
            SendMsmSignal(me, MSM_SEND_PACKET_SIG);
            me->error_counter.open_connection_error=0;
            GprsState=gprs_is_open;
         }
         return Q_HANDLED() ;
      case MODEM_IPSEND_SIG:
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG:
      case MODEM_OK_SIG: 
      case MODEM_ERROR_SIG:
      case MODEM_CLOSED_SIG:
      case MODEM_SEND_OK_SIG:
      case MODEM_TCP_DATA_RECEIVED_SIG:
      case MODEM_SERVER_DATA_SIG:
         {
            ModemSM_dispatch(&me->msm, e);
         }
         return Q_HANDLED();
      case MSM_ANSWER_SIG:
         switch (((MsmAnswerType*)e)->answer)
            {
            case ANSWER_OK: if (me->msm.server_data.size==1&&*me->msm.server_data.ptr==1)
                  {
                  ServerState=server_authorized_and_connected;
                  OutDebugModem("UPD AUTH OK"); 
                  QACTIVE_POST(AO_Control, &ControlUpdateConnectOkEvt, me);                                    
                  return Q_TRAN(&Modem_upd_ready_44); 
                  }
            default:
               {
                  OutDebugModem("UPD AUTH ERROR");
                  return Q_TRAN(&Modem_upd_connect_error_47);
               }
            }
      case Q_EXIT_SIG: return Q_HANDLED() ;
      case TIC_100ms_SIG:
         {
            ModemSM_dispatch(&me->msm, e); 
         }
         break;
      }
   return Q_SUPER(&Modem_upd_super_40);
   }


QState Modem_upd_send_packet_43(MOD * const me, QEvt const * const e) 
   {
   static enum
      {
     // flush,
      send,       
      }state;
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(43);
            //SendMsmSignal(me, MSM_GET_TCP_DATA_SIG);
            me->error_counter.open_connection_error=0;
            GprsState=gprs_is_open;
           // state=flush;
						me->msm.send_packet.ptr=me->config_packet.ptr;
            me->msm.send_packet.size= me->config_packet.size;
            SendMsmSignal(me, MSM_SEND_PACKET_SIG);
            state=send;  
         }
         return Q_HANDLED() ;
      case MODEM_IPSEND_SIG:
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG:
      case MODEM_OK_SIG: 
      case MODEM_ERROR_SIG:
      case MODEM_CLOSED_SIG:
      case MODEM_SEND_OK_SIG:
      case MODEM_TCP_DATA_RECEIVED_SIG:
      case MODEM_SERVER_DATA_SIG:
         {
            ModemSM_dispatch(&me->msm, e);
         }
         return Q_HANDLED();
      case MSM_ANSWER_SIG:
         switch (state)
            {
            /*case flush: switch (((MsmAnswerType*)e)->answer)
                  {
                  case ANSWER_ERROR:
									{
										 OutDebugModem("GET TCP DATA ERROR");
										 me->msm.send_packet.ptr=me->config_packet.ptr;
                        me->msm.send_packet.size= me->config_packet.size;
                        SendMsmSignal(me, MSM_SEND_PACKET_SIG);
                        state=send;  
									}
									 return Q_HANDLED(); 
                  case ANSWER_OK:
                     {
											 OutDebugModem("GET TCP DATA OK");
                        me->msm.send_packet.ptr=me->config_packet.ptr;
                        me->msm.send_packet.size= me->config_packet.size;
                        SendMsmSignal(me, MSM_SEND_PACKET_SIG);
                        state=send;          
                     }
										  return Q_HANDLED(); 
                  case ANSWER_SIM_ERROR:
									case ANSWER_SIM_LOCK:
                     {
											  OutDebugModem("SIM LOCK or ERROR");
                        QACTIVE_POST(AO_Control, &ControlUpdPacketLevelErrorEvt, me);
                        return Q_TRAN(&Modem_upd_ready_44);
                     }
                  }*/
            case send: switch (((MsmAnswerType*)e)->answer)
                  {
                  case ANSWER_OK:
                     {
                        DataPtrEvt *pe = Q_NEW(DataPtrEvt, CONTROL_TCP_DATA_SIG);
                        pe->ptr=me->msm.server_data.ptr;
                        pe->data.size=me->msm.server_data.size;
                        QACTIVE_POST(AO_Control, &pe->super, me);
                        OutDebugModem("RECEIVE TCP DATA OK");  
                        return Q_TRAN(&Modem_upd_ready_44);                
                     }
                  default:
                     {
                        OutDebugModem("RECEIVE TCP DATA ERROR");
                        QACTIVE_POST(AO_Control, &ControlUpdPacketLevelErrorEvt, me);
                        return Q_TRAN(&Modem_upd_ready_44);
                     }
                  }
            } 
      case Q_EXIT_SIG: return Q_HANDLED() ;
      case TIC_100ms_SIG:
         {
            ModemSM_dispatch(&me->msm, e); 
         }
         break;
      }
   return Q_SUPER(&Modem_upd_super_40);
   }


QState Modem_upd_ready_44(MOD * const me, QEvt const * const e) 
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         { 
            AddStateHistory(44);      
         }
         return Q_HANDLED() ;
      case MODEM_UPD_GET_REMAIN_FW_DATA_SIG:  return Q_TRAN(&Modem_upd_get_remain_fw_data_46);
      case MODEM_UPD_SEND_PACKET_SIG:
         {
            OutDebugModem("UPD_SEND_PACKET_SIG");
            DataPtrEvt *pe=(DataPtrEvt*)e;
            me->config_packet.ptr=pe->ptr;
            me->config_packet.size=pe->data.size;
         }
         return Q_TRAN(&Modem_upd_send_packet_43);
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return Q_SUPER(&Modem_upd_super_40);
   }


QState Modem_upd_get_remain_fw_data_46(MOD * const me, QEvt const * const e) 
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         { 
            AddStateHistory(46); 
            SendMsmSignal(me, MSM_GET_TCP_DATA_SIG);
         }
         return Q_HANDLED() ;
      case MODEM_OK_SIG:
      case MODEM_ERROR_SIG:
      case MODEM_CLOSED_SIG: 
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG:
      case MODEM_SERVER_DATA_SIG:
         {
            ModemSM_dispatch(&me->msm, e);
         }
         return Q_HANDLED(); 
      case MSM_ANSWER_SIG:
         switch (((MsmAnswerType*)e)->answer)
            {
            case ANSWER_OK:
               {
                  DataPtrEvt *pe = Q_NEW(DataPtrEvt, CONTROL_TCP_DATA_SIG);
                  pe->ptr=me->msm.server_data.ptr;
                  pe->data.size=me->msm.server_data.size;
                  QACTIVE_POST(AO_Control, &pe->super, me);
                  OutDebugModem("MODEM_UPD:: GET REMAIN FW DATA OK");  
                  return Q_TRAN(&Modem_upd_ready_44);                
               }
            default:
               {
                  OutDebugModem("MODEM_UPD:: GET REMAIN FW DATA ERROR");  
                  QACTIVE_POST(AO_Control, &ControlUpdPacketLevelErrorEvt, me);
                  return Q_TRAN(&Modem_upd_ready_44);
               }
            }
      case Q_EXIT_SIG: return Q_HANDLED();
      case TIC_100ms_SIG:
         {
            ModemSM_dispatch(&me->msm, e); 
         }
         break;
      }
   return Q_SUPER(&Modem_upd_super_40);
   }

QState Modem_upd_connect_error_47(MOD * const me, QEvt const * const e) 
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         { 
            AddStateHistory(47);
            QACTIVE_POST(AO_Control, &ControlUpdTransportLevelErrorEvt, me);       
         }
         return Q_HANDLED() ;
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return Q_SUPER(&Modem_upd_super_40);
   }

//------------------------------------------------------------------------------------	 

QState Modem_off_45(MOD * const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG: 
         {
					  AddStateHistory(45);
					  ModemUart_off();
            ModemTxPin_deinit();
            ModemRxPin_deinit();
            POWERKEY_DOWN();
            MODEM_POWER_OFF();
            QACTIVE_POST(AO_Control, &ControlModemIsOffEvt, me);  
         }
         return Q_HANDLED();
      case TIC_100ms_SIG: 
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG:  
      case MODEM_OK_SIG: 
      case MODEM_ERROR_SIG: 
			return Q_HANDLED();
      case Q_EXIT_SIG: 
			{
				ModemUart_on();
			}
				return Q_HANDLED();
      }
   return Q_SUPER(&QHsm_top);
   }
 __used AnswerEnum debug_answer;
	 
QState Modem_power_on_start_up_2(MOD * const me, QEvt const * const e)
   {
   static enum
      {
      delay,
      start
      }state;
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(2);
            state=delay;
            me->counter.tick=0;
            GsmState=gsm_modem_off;
            GprsState=gprs_off;
            ServerState=server_not_connected;
            me->msm.server_ptr=&conf.server[MAIN_SERVER];
					 SelectSim(me);
         }
         return  Q_HANDLED();
      case MODEM_BAUDRATE_115200_SIG:
      case MODEM_OK_SIG:
      case MODEM_IMEI_IMSI_SIG:
      case MODEM_CALL_READY_SIG:
			case MODEM_SMS_READY_SIG:
      case MODEM_ERROR_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG:
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_READY_SIG:
      case MODEM_SIM_PIN_SIG:
      case MODEM_SIM_PUK_SIG:
      case MODEM_PH_SIM_PIN_SIG:
      case MODEM_PH_SIM_PUK_SIG:
      case MODEM_SIM_PIN2_SIG:
      case MODEM_SIM_PUK2_SIG:
         {
            ModemSM_dispatch(&me->msm, e);
         }
         return  Q_HANDLED();
      case  MSM_ANSWER_SIG:
         {
					  debug_answer=((MsmAnswerType*)e)->answer;
            //AnswerEnum answer=((MsmAnswerType*)e)->answer;
            OutDebugModemSprintf1("MSM_ANSWER_SIG, answer=",debug_answer);
            switch (debug_answer)
               {
               case ANSWER_OK:
                  NOP();
                  return Q_TRAN(&Modem_gsm_open_6);
               case ANSWER_SIM_LOCK: 
                  NOP();
                  return Q_TRAN(&Modem_sim_lock_error_21);
               case ANSWER_SIM_ERROR:
                  NOP();
                  return Q_TRAN(&Modem_sim_error_20);
               default:
                  NOP();
                  return Q_TRAN(&Modem_power_on_start_up_error_3);
               }
         }
      case Q_EXIT_SIG: return Q_HANDLED();
      case TIC_100ms_SIG:
         {
            switch (state)
               {
               case delay:
                  if (++ me->counter.tick==10)
                     {
                     SendMsmSignal(me, MSM_POWER_ON_START_UP_SIG);
                     state=start; 
                     }
                  break;
               case start:
                  ModemSM_dispatch(&me->msm, e);
                  break;
               }
         }
         break;
      }
   return Q_SUPER(&Modem_top_1);
   }


QState Modem_power_on_start_up_error_3(MOD * const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case  Q_ENTRY_SIG:
         {
            AddStateHistory(3);
					  ModemTxPin_deinit();
            ModemRxPin_deinit();
            POWERKEY_DOWN();
            MODEM_POWER_OFF(); 
            ++me->error_counter.power_on_start_up_error;
            me->counter.tick =0;
            GprsState=gprs_off;
            ServerState=server_not_connected;
					   OutDebugModemSprintf1("POWER_ON_START_UP_ERRORS count=", me->error_counter.power_on_start_up_error);
         }
         return Q_HANDLED();
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG:  return Q_TRAN(&Modem_sim_error_20);  
      case MODEM_OK_SIG: return Q_HANDLED();
      case MODEM_ERROR_SIG: return Q_HANDLED();
      case Q_EXIT_SIG: return Q_HANDLED();
      case TIC_100ms_SIG:
         {
            if (me->error_counter.power_on_start_up_error>3) 
						{
							me->fatal_error_timer.power_on_start_up=0;
							return Q_TRAN(&Modem_power_on_start_up_fatal_error_60);
						}
           else if (++me->counter.tick==POWER_ON_START_UP_ERROR_TIMEOUT)
               {
								  return Q_TRAN(&Modem_power_on_start_up_2);   
              // return Q_TRAN(&Modem_hard_reset_5);  
               }
					 else if(me->counter.tick%50==0)
					 {
						 OutDebugModemSprintf2("POWER_ON_START_UP_ERROR timeout=","count=",POWER_ON_START_UP_ERROR_TIMEOUT-me->counter.tick,me->error_counter.sim_error);
					 }
         }
         break;
      }
   return Q_SUPER(&Modem_top_1);
   }

QState Modem_power_on_start_up_fatal_error_60(MOD * const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         { 
            AddStateHistory(60);
            me->counter.tick=0;
            me->flag.check_gsmq=0;
            SysInfo.gsmq=0; 
            ClearErrorCounters(me);              
            if (me->flag.packet)
               {
               me->flag.packet=0;
               me->flag.fatal_error=1;      
               QACTIVE_POST(AO_Control, &SendPacketErrorEvt, me);                
               }
						if(me->flag.upd)
							 {
								 QACTIVE_POST(AO_Control, &ControlUpdTransportLevelErrorEvt, me); 
							 }
            GsmState=gsm_modem_off;
         }
         return Q_HANDLED() ;
      case TIC_100ms_SIG:
         {
            if (++me->fatal_error_timer.power_on_start_up==POWER_ON_START_UP_FATAL_ERROR_TIMEOUT)
               {
               return Q_TRAN(&Modem_power_on_start_up_2);                     
               }
            else if (me->fatal_error_timer.power_on_start_up%100==0)
               {
               OutDebugModemSprintf1("POWER_ON_START_UP_FATAL_ERROR_TIMEOUT remain=",POWER_ON_START_UP_FATAL_ERROR_TIMEOUT-me->fatal_error_timer.power_on_start_up);
               }
         }
         break;
      }
   return Q_SUPER(&Modem_top_1);
   }


QState Modem_hard_reset_5(MOD * const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(5);
            SendMsmSignal(me, MSM_HARD_RESET_SIG);
            me->counter.tick =0;
            GsmState=gsm_modem_off;
            GprsState=gprs_off;
            ServerState=server_not_connected;
         }
         return  Q_HANDLED();
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG:
         {
            ModemSM_dispatch(&me->msm, e);         
         }
         return  Q_HANDLED();
      case  MSM_ANSWER_SIG: switch (((MsmAnswerType*)e)->answer)
            {
            case ANSWER_SIM_ERROR: return Q_TRAN(&Modem_sim_error_20);
            default: return Q_TRAN(&Modem_power_on_start_up_2);
            }
      case MODEM_OK_SIG: return  Q_HANDLED();
      case MODEM_ERROR_SIG: return  Q_HANDLED();
      case Q_EXIT_SIG: return  Q_HANDLED();
      case TIC_100ms_SIG:
         {
            ModemSM_dispatch(&me->msm, e);         
         }
         break; 
      }
   return Q_SUPER(&Modem_top_1);
   }


QState Modem_gsm_open_6(MOD * const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case  Q_ENTRY_SIG:
         {
            AddStateHistory(6);
            SendMsmSignal(me, MSM_OPEN_GSM_SIG);
            me->counter.tick =0;
            GsmState=gsm_registration;
            GprsState=gprs_off;
            ServerState=server_not_connected;
         }
         return  Q_HANDLED();
      case MODEM_CREG_NOT_REGISTERED_NO_OPERATOR_SEARCHING_SIG:
      case MODEM_CREG_REGISTERED_HOME_NETWORK_SIG:
      case MODEM_CREG_NOT_REGISTERED_BUT_OPERATOR_SEARCHING_SIG:
      case MODEM_CREG_REGISTRATION_DENIED_SIG:
      case MODEM_CREG_UNKNOWN_ANSWER_SIG:
      case MODEM_CREG_REGISTERED_ROAMING_SIG:
      case MODEM_OK_SIG:
      case MODEM_ERROR_SIG:
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG:
         {
            ModemSM_dispatch(&me->msm, e);
         }
         return  Q_HANDLED();
      case  MSM_ANSWER_SIG:
         switch (((MsmAnswerType*)e)->answer)
            {
            case ANSWER_OK:
               OutDebugModem("OPEN GSM OK");
               if (me->flag.upd==1)return Q_TRAN(&Modem_gsm_is_open_7);
               else
                  {
                  me->ret_ptr=(QStateHandler)&Modem_gsm_is_open_7;
                  return Q_TRAN(&Modem_sms_super_30);
                  }                       
            case ANSWER_SIM_ERROR:
               OutDebugModem("SIM ERROR");
               return Q_TRAN(&Modem_sim_error_20);
            default: 
               OutDebugModem("OPEN GSM ERROR");
               me->ret_ptr=(QStateHandler)&Modem_gsm_open_error_8;
               return Q_TRAN(&Modem_sms_super_30);            
            }
      case Q_EXIT_SIG:  return Q_HANDLED();
      case TIC_100ms_SIG:
         {
            ModemSM_dispatch(&me->msm, e);   
         }
         break;
      }
   return Q_SUPER(&Modem_top_1);
   }

QState Modem_gsm_open_error_8(MOD * const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case  Q_ENTRY_SIG:
         {
            AddStateHistory(8);
            me->ret_ptr=(QStateHandler)&Modem_gsm_open_error_8;
            me->counter.tick =0;
            ++me->error_counter.open_gsm_error;
            GprsState=gprs_off;
            GsmState=gsm_reg_error; 
            ServerState=server_not_connected;          
            me->flag.check_gsmq=1;
         }
         return Q_HANDLED();
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG:  return Q_TRAN(&Modem_sim_error_20);  
      case MODEM_OK_SIG: 
      case MODEM_ERROR_SIG: return Q_HANDLED();
      case Q_EXIT_SIG:
         {
            me->flag.check_gsmq=0;
         }
         return Q_HANDLED();
      case TIC_100ms_SIG:
         {
            if ( me->flag.sms_new==1|| me->flag.sms_send==1)
               {
               --me->error_counter.open_gsm_error;  
               return Q_TRAN(&Modem_sms_super_30);  
               }
            else if (me->error_counter.open_gsm_error>3)
						{
							me->fatal_error_timer.open_gsm=0;
							return  Q_TRAN(&Modem_gsm_open_fatal_error_61);
						}
            else if (++me->counter.tick==OPEN_GSM_ERROR_TIMEOUT)
               {
               return  Q_TRAN(&Modem_gsm_open_6);
               }
            else  me->flag.check_gsmq=1;
         }
         break;
      }
   return Q_SUPER(&Modem_top_1);
   }

QState Modem_gsm_open_fatal_error_61(MOD * const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         { 
            AddStateHistory(61);
            me->counter.tick=0;
            ModemTxPin_deinit();
            ModemRxPin_deinit();
            POWERKEY_DOWN();
            MODEM_POWER_OFF(); 
            me->flag.check_gsmq=0;
            SysInfo.gsmq=0; 
            ClearErrorCounters(me);              
            if (me->flag.packet)
               {
               me->flag.packet=0;
               me->flag.fatal_error=1;      
               QACTIVE_POST(AO_Control, &SendPacketErrorEvt, me);    
               }
						if(me->flag.upd)
							 {
								 QACTIVE_POST(AO_Control, &ControlUpdTransportLevelErrorEvt, me); 
							 }
            GsmState=gsm_reg_error;
            GprsState=gprs_off;
						me->sim_error=1;
         }
         return Q_HANDLED();
      case TIC_100ms_SIG:
         {
            if (++me->fatal_error_timer.open_gsm==OPEN_GSM_FATAL_ERROR_TIMEOUT)
               {
               return Q_TRAN(&Modem_power_on_start_up_2);
               }
            else if (me->fatal_error_timer.open_gsm%100==0)
               {
               OutDebugModemSprintf1("OPEN_GSM_FATAL_ERROR_TIMEOUT remain=",OPEN_GSM_FATAL_ERROR_TIMEOUT-me->fatal_error_timer.open_gsm);
               }
         }
         break;
      }
   return Q_SUPER(&Modem_top_1);
   }

QState Modem_gsm_is_open_7(MOD * const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case  Q_ENTRY_SIG:
         {
            AddStateHistory(7);
            me->ret_ptr=(QStateHandler)&Modem_gsm_is_open_7;
            me->flag.check_gsmq=1;
            ServerState=server_not_connected;
         }
         return Q_HANDLED();
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG:  return Q_TRAN(&Modem_sim_error_20);          
      case MODEM_OK_SIG: 
      case MODEM_ERROR_SIG: return Q_HANDLED();
      case Q_EXIT_SIG:
         {
            me->flag.check_gsmq=0;
         }
         return Q_HANDLED();
      case TIC_100ms_SIG:
         {
            if ( me->flag.sms_send||me->flag.sms_new)return Q_TRAN(&Modem_sms_super_30);
            else if ((me->flag.packet==1)||(me->flag.fatal_error==1)||(me->flag.get_ready==1)||(me->flag.upd==1))
               {
               return Q_TRAN(&Modem_open_gprs_9);
               }
         }
         break;
      }
   return Q_SUPER(&Modem_top_1);
   }

QState Modem_open_gprs_9(MOD * const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(9);
            SendMsmSignal(me, MSM_OPEN_GPRS_SIG);
            me->counter.tick =0;
            GprsState=gprs_opening;
            ServerState=server_not_connected;
         }
         return Q_HANDLED();
			case MODEM_GPRS_ATTACHED_SIG:
      case MODEM_STATE_IP_STATUS_SIG:
      case MODEM_STATE_TCP_CLOSED_SIG:
      case MODEM_STATE_IP_INITIAL_SIG:
      case MODEM_STATE_PDP_DEACT_SIG:
      case MODEM_LOCAL_IP_SIG:
      case MODEM_OK_SIG:
      case MODEM_ERROR_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG:
      case MODEM_SIM_NOT_READY_SIG:
         {
            ModemSM_dispatch(&me->msm, e);
         }
         return Q_HANDLED();
      case  MSM_ANSWER_SIG: 
         switch (((MsmAnswerType*)e)->answer)
            {
            case ANSWER_OK: return Q_TRAN(&Modem_gprs_is_open_state_11);
            case ANSWER_SIM_ERROR: return Q_TRAN(&Modem_sim_error_20);
            default: return Q_TRAN(&Modem_open_gprs_error_12);
            }
      case Q_EXIT_SIG: return Q_HANDLED();
      case TIC_100ms_SIG:
         {
            ModemSM_dispatch(&me->msm, e); 
         }
         break;
      }
   return Q_SUPER(&Modem_top_1);
   }

QState Modem_close_gprs_10(MOD * const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(10);
            me->done=ModemSendCommand("AT+CIPSHUT\r\n");
            me->counter.tick =0;
         }
         return Q_HANDLED();
      case MODEM_OK_SIG:  return Q_HANDLED();
      case MODEM_SHUT_OK_SIG:
         {
            OutDebugModem("CLOSE GPRS OK");  
         }
         return Q_TRAN(&Modem_open_gprs_9);      
      case MODEM_ERROR_SIG: return Q_TRAN(&Modem_hard_reset_5);   
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG:  return Q_TRAN(&Modem_sim_error_20);  
      case TIC_100ms_SIG:
         {
            if (++ me->counter.tick ==100)
               {
               return Q_TRAN(&Modem_hard_reset_5);    
               }
            if (me->done==0)me->done=ModemSendCommand("AT+CIPSHUT\r\n");
         }
         break;
      case Q_EXIT_SIG:
         {
            GprsState=gprs_off;
            ServerState=server_not_connected;
         }
         return Q_HANDLED();
      }
   return Q_SUPER(&Modem_top_1);
   }



QState Modem_gprs_is_open_state_11(MOD * const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(11);
            me->ret_ptr=(QStateHandler)&Modem_gprs_is_open_state_11;
            me->counter.tick =0;
            me->error_counter.open_gprs_error=0;
            me->flag.check_gsmq=1;
            GprsState=gprs_is_open;
         }
         return Q_HANDLED();
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG:  return Q_TRAN(&Modem_sim_error_20);             
      case MODEM_OK_SIG: return Q_HANDLED() ;
      case MODEM_ERROR_SIG: return Q_HANDLED() ;
      case Q_EXIT_SIG:
         {
            me->flag.check_gsmq=0;
         }
         return Q_HANDLED() ;
      case TIC_100ms_SIG:
         {
            if ( me->flag.sms_send|| me->flag.sms_new)return Q_TRAN(&Modem_sms_super_30);
//            else if (me->flag.restart==1)return Q_TRAN(&Modem_restart_24);
            else if (me->flag.upd==1)return Q_TRAN(&Modem_upd_super_40);
            else if ((me->flag.packet==1)||(me->flag.fatal_error==1)||(me->flag.get_ready==1))
               {
               return Q_TRAN(&Modem_check_if_connected_14);
               }
         }
         break;
      }
   return Q_SUPER(&Modem_top_1);
   }


QState Modem_open_gprs_error_12(MOD * const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(12);
            ServerState=server_not_connected;
            GprsState=gprs_open_error;
            me->ret_ptr=(QStateHandler)&Modem_open_gprs_error_12;
            me->counter.tick =0;
            ++me->error_counter.open_gprs_error;       
         }
         return Q_HANDLED();
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG:  return Q_TRAN(&Modem_sim_error_20);  
      case MODEM_OK_SIG: return Q_HANDLED() ;
      case MODEM_ERROR_SIG: return Q_HANDLED() ;
      case Q_EXIT_SIG:
         {
            me->flag.check_gsmq=0;
         }
         return Q_HANDLED() ;
      case TIC_100ms_SIG:
         {
            if ( me->flag.sms_send||me->flag.sms_new)
               {
               --me->error_counter.open_gprs_error; 
               return Q_TRAN(&Modem_sms_super_30);  
               }
            else if (++me->counter.tick==OPEN_GPRS_ERROR_TIMEOUT)
               {
               switch (me->error_counter.open_gprs_error)
                  {
                  case 1: return Q_TRAN(&Modem_gsm_open_6); 
                  case 2: return Q_TRAN(&Modem_hard_reset_5); 
                  case 3: 
                  case 4: 
                  case 5:
                  case 6:
                  case 7:
                  case 8:                       
                  case 9: return Q_TRAN(&Modem_gsm_open_6); 
                  case 10: me->fatal_error_timer.open_gprs=0;
                     return Q_TRAN(&Modem_open_gprs_fatal_error_62);  
                  }
               }
            else  me->flag.check_gsmq=1;
         }
         break;
      }
   return Q_SUPER(&Modem_top_1);
   }

QState Modem_open_gprs_fatal_error_62(MOD * const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         { 
            AddStateHistory(62);
            me->ret_ptr=(QStateHandler)&Modem_open_gprs_fatal_error_62;
            me->flag.check_gsmq=0;
            ClearErrorCounters(me);              
            if (me->flag.packet)
               {
               me->flag.packet=0;
               me->flag.fatal_error=1;      
               QACTIVE_POST(AO_Control, &SendPacketErrorEvt, me);    
               }
							 if(me->flag.upd)
							 {
								 QACTIVE_POST(AO_Control, &ControlUpdTransportLevelErrorEvt, me); 
							 }
            GprsState=gprs_off;
						me->sim_error=1;
         }
         return Q_HANDLED();
      case TIC_100ms_SIG:
         {
            if ( me->flag.sms_send||me->flag.sms_new)
               {
               return Q_TRAN(&Modem_sms_super_30);  
               }
            if (++me->fatal_error_timer.open_gprs==OPEN_GPRS_FATAL_ERROR_TIMEOUT)
               {
               return Q_TRAN(&Modem_hard_reset_5);                     
               }
            else if (me->fatal_error_timer.open_gprs%100==0)
               {
               OutDebugModemSprintf1("OPEN_GPRS_FATAL_ERROR_TIMEOUT remain=",OPEN_GPRS_FATAL_ERROR_TIMEOUT-me->fatal_error_timer.open_gprs);
               }
         }
         break;
      }
   return Q_SUPER(&Modem_top_1);
   }



QState Modem_check_if_connected_14(MOD * const me, QEvt const * const e) 
   {
   QState ret;
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(14);
            me->done=ModemSendCommand("AT+CIPSTATUS\r\n");
            GprsState=gprs_is_open;
            me->counter.tick =0;
         }
         return Q_HANDLED() ;
      case Q_EXIT_SIG: return Q_HANDLED() ;
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG:  return Q_TRAN(&Modem_sim_error_20); 
      case MODEM_STATE_TCP_CLOSING_SIG:
         {
            OutDebugModem("STATE_TCP_CLOSING_SIG");
            ret= Q_TRAN(&Modem_close_connection_16);
         }
         return ret ;
      case MODEM_STATE_TCP_CONNECTING_SIG:
         {
            OutDebugModem("STATE_TCP_CONNECTING_SIG");
            ret= Q_TRAN(&Modem_close_connection_16);
         }
         return ret;
      case MODEM_STATE_IP_CONFIG_SIG:
         {
            OutDebugModem("STATE_IP_CONFIG_SIG");
            ret= Q_TRAN(&Modem_open_gprs_9); 
         }
         return ret;
      case MODEM_STATE_PDP_DEACT_SIG:
         {
            OutDebugModem("STATE_PDP_DEACT_SIG");
            ret= Q_TRAN(&Modem_open_gprs_9);  
         }
         return ret;
      case MODEM_STATE_IP_START_SIG:
         {
            OutDebugModem("STATE_IP_START_SIG");
            ret= Q_TRAN(&Modem_open_gprs_9);   
         }
         return ret;
      case MODEM_STATE_CONNECT_OK_SIG:
         {
            OutDebugModem("STATE_CONNECT_OK_SIG");
            ret= Q_TRAN(&Modem_tcp_is_open_state_13);
         }
         return ret ;  
      case MODEM_STATE_IP_STATUS_SIG:
         {
            OutDebugModem("STATE_IP_STATUS_SIG");
            ret= Q_TRAN(&Modem_open_connection_15);
         }
         return ret ;  
      case MODEM_STATE_IP_INITIAL_SIG:
         {
            OutDebugModem("STATE_IP_INITIAL_SIG");
            ret= Q_TRAN(&Modem_open_gprs_9); 
         }
         return ret ;  
      case MODEM_STATE_TCP_CLOSED_SIG: 
         {
            OutDebugModem("STATE_TCP_CLOSED_SIG");
            ret= Q_TRAN(&Modem_open_connection_15);
         }
         return ret ;       
      case MODEM_CLOSE_OK_SIG:
         {
            OutDebugModem("CLOSE_OK_SIG");
            ret= Q_TRAN(&Modem_open_connection_15);
         }
         return ret;
      case MODEM_CLOSED_SIG: 
         {
            OutDebugModem("CLOSED_SIG");
            ret= Q_TRAN(&Modem_open_connection_15);
         }
         return ret ;
      case MODEM_ERROR_SIG: 
         {
            OutDebugModem("ERROR_SIG");
            ret= Q_TRAN(&Modem_close_connection_16);
         }
         return ret;
      case MODEM_OK_SIG: return Q_HANDLED() ;      
      case TIC_100ms_SIG:
         {
            if (++ me->counter.tick==100)
               {
               return Q_TRAN(&Modem_close_connection_16);
               }
            else if (me->done==0)me->done=ModemSendCommand("AT+CIPSTATUS\r\n");
         }
         break;
      }
   return Q_SUPER(&Modem_top_1);
   }



QState Modem_open_connection_15(MOD * const me, QEvt const * const e) 
   {
   QState ret;
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(15);
            ServerState=server_connecting;
            GprsState=gprs_is_open;
            QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,BSP_TICKS_PER_SEC*70); //70 sec delay
            me->flag.check_gsmq=0;
            sprintf(me->command_buf,"AT+CIPSTART=\"TCP\",\"%s\",\"%u\"\r\n",conf.server[MAIN_SERVER].ip ,conf.server[MAIN_SERVER].port); 
            me->done=ModemSendCommand(me->command_buf);
					  OutDebugModem(me->command_buf);
         }
         return Q_HANDLED();
      case MODEM_CONNECT_OK_SIG:
         {
            OutDebugModem("CONNECT_OK_SIG");
            /* if (me->flag.restart==1)ret= Q_TRAN(&Modem_restart_24);
             else*/
            ret= Q_TRAN(&Modem_authorization_18);          
         }
         return  ret; 
      case MODEM_PDP_DEACT_SIG: 
         {
            OutDebugModem("PDP_DEACT_SIG"); 
            ret= Q_TRAN(&Modem_open_connection_error_17);       
         }
         return ret;
      case MODEM_CLOSED_SIG: 
         {
            OutDebugModem("CLOSED_SIG"); 
            ret= Q_TRAN(&Modem_open_connection_error_17);           
         }
         return ret;
      case MODEM_CONNECT_FAIL_SIG:
         {
            OutDebugModem("CONNECT_FAIL_SIG");
            ret= Q_TRAN(&Modem_open_connection_error_17);         
         }
         return ret;
      case MODEM_ALREADY_CONNECT_SIG: 
         {
            OutDebugModem("ALREADY_CONNECT_SIG");
            ret= Q_TRAN(&Modem_open_connection_error_17);         
         }
         return ret;
      case MODEM_ERROR_SIG: 
         {
            OutDebugModem("ERROR_SIG"); 
            ret= Q_TRAN(&Modem_open_connection_error_17);          
         }
         return ret;
      case MODEM_TIMEOUT_SIG:
         {
            OutDebugModem("TIMEOUT_SIG"); 
            ret= Q_TRAN(&Modem_open_connection_error_17);              
         }
         return ret;
      case MODEM_OK_SIG: return Q_HANDLED() ;
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG:  return Q_TRAN(&Modem_sim_error_20); 
      case Q_EXIT_SIG:
         {
            QTimeEvt_disarm(&me->TimeEvt);
         }
         return Q_HANDLED();
      }
   return Q_SUPER(&Modem_top_1);
   }


QState Modem_close_connection_16(MOD * const me, QEvt const * const e) 
   {
   QState ret;
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(16);
            me->done=ModemSendCommand("AT+CIPCLOSE=1\r\n");
            me->counter.tick =0;
         }
         return Q_HANDLED() ;
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG:  return Q_TRAN(&Modem_sim_error_20);  
      case MODEM_CLOSE_OK_SIG: 
         {
            OutDebugModem("CLOSE_OK_SIG");
            ret=Q_TRAN(&Modem_open_gprs_9);           
         }
         return ret; 
      case MODEM_CLOSED_SIG: 
         {
            OutDebugModem("CLOSED_SIG");
            ret=Q_TRAN(&Modem_open_gprs_9);           
         }
         return ret; 
      case MODEM_OK_SIG:  return Q_HANDLED();
      case MODEM_ERROR_SIG: 
         {
            OutDebugModem("CLOSE CONNECTION ERROR!!!"); 
            ret=Q_TRAN(&Modem_open_gprs_9); 
         }
         return ret;
      case Q_EXIT_SIG:
         {
            ServerState=server_not_connected;
            ret= Q_HANDLED() ;
         }
         return ret;
      case TIC_100ms_SIG:
         {
            if (++me->counter.tick ==100)
               {
               OutDebugModem("CLOSE CONNECTION TIMEOUT ERROR!!!"); 
               return Q_TRAN(&Modem_open_gprs_9); 
               }
            else if (me->done==0) me->done=ModemSendCommand("AT+CIPCLOSE=1\r\n");
         }
         break;
      }
   return Q_SUPER(&Modem_top_1);
   }

QState Modem_restart_4(MOD * const me, QEvt const * const e) 
   {
   static enum
      {
      check_ip_status,
      close_connection,
      close_gprs,
      exit,
      }state;
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(4);
            if (me->flag.packet)
               {
               me->flag.packet=0;
               QACTIVE_POST(AO_Control, &SendPacketErrorEvt, me);   
               }
            me->done=ModemSendCommand("AT+CIPSTATUS\r\n");
            me->counter.tick =0;
            state=check_ip_status;
         }
         return Q_HANDLED() ;
      case Q_EXIT_SIG: return Q_HANDLED() ;
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG:  return Q_TRAN(&Modem_sim_error_20); 
      case MODEM_STATE_TCP_CLOSING_SIG:
         {
            switch (state)
               {
               case  check_ip_status:
                  {
                     me->done=ModemSendCommand("AT+CIPCLOSE=1\r\n");
                     me->counter.tick =0;
                     state=close_connection;
                  }
                  break;
               default: break;
               }
         }
         return Q_HANDLED() ;
      case MODEM_STATE_TCP_CONNECTING_SIG:
         {
            switch (state)
               {
               case  check_ip_status:
                  {
                     me->done=ModemSendCommand("AT+CIPCLOSE=1\r\n");
                     me->counter.tick =0;
                     state=close_connection;
                  }
                  break;
               default: break;
               }
         }
         return Q_HANDLED() ;
      case MODEM_STATE_IP_CONFIG_SIG:
         {
            ServerState=server_not_connected;
            switch (state)
               {
               case  check_ip_status:
                  {
                     me->done=ModemSendCommand("AT+CIPSHUT\r\n");
                     me->counter.tick =0;
                     state=close_gprs;
                  }
                  break;
               default: break;
               }
         }
         return Q_HANDLED() ;
      case MODEM_STATE_PDP_DEACT_SIG:
         {
            ServerState=server_not_connected;
            switch (state)
               {
               case  check_ip_status:
                  {
                     me->done=ModemSendCommand("AT+CIPSHUT\r\n");
                     me->counter.tick =0;
                     state=close_gprs;
                  }
                  break;
               default: break;
               }
         }
         return Q_HANDLED() ;
      case MODEM_STATE_IP_START_SIG:
         {
            ServerState=server_not_connected;
            switch (state)
               {
               case  check_ip_status:
                  {
                     me->done=ModemSendCommand("AT+CIPSHUT\r\n");
                     me->counter.tick =0;
                     state=close_gprs;
                  }
                  break;
               default: break;
               }
         }
         return Q_HANDLED() ;
      case MODEM_STATE_CONNECT_OK_SIG:
         {
            switch (state)
               {
               case  check_ip_status:
                  {
                     me->done=ModemSendCommand("AT+CIPCLOSE=1\r\n");
                     me->counter.tick =0;
                     state=close_connection;
                  }
                  break;
               default: break;
               }
         }
         return Q_HANDLED() ;
      case MODEM_STATE_IP_STATUS_SIG:
         {
            ServerState=server_not_connected;
            switch (state)
               {
               case  check_ip_status:
                  {
                     me->done=ModemSendCommand("AT+CIPSHUT\r\n");
                     me->counter.tick =0;
                     state=close_gprs;
                  }
                  break;
               default: break;
               }
         }
         return Q_HANDLED() ;
      case MODEM_STATE_IP_INITIAL_SIG:
         {
            ServerState=server_not_connected;
            switch (state)
               {
               case  check_ip_status:
                  {
                     me->done=ModemSendCommand("AT+CIPSHUT\r\n");
                     me->counter.tick =0;
                     state=close_gprs;
                  }
                  break;
               default: break;
               }
         }
         return Q_HANDLED() ;
      case MODEM_STATE_TCP_CLOSED_SIG: 
         {
            ServerState=server_not_connected;
            switch (state)
               {
               case  check_ip_status:
                  {
                     me->done=ModemSendCommand("AT+CIPSHUT\r\n");
                     me->counter.tick =0;
                     state=close_gprs;
                  }
                  break;
               default: break;
               }
         }
         return Q_HANDLED() ;       
      case MODEM_CLOSE_OK_SIG:
         {
            ServerState=server_not_connected;
            switch (state)
               {
               case  close_connection:
                  {
                     me->done=ModemSendCommand("AT+CIPSHUT\r\n");
                     me->counter.tick =0;
                     state=close_gprs;
                  }
                  break;
               default: break;
               }
         }
         return Q_HANDLED() ;
      case MODEM_CLOSED_SIG: 
         {
            ServerState=server_not_connected;
            switch (state)
               {
               case  check_ip_status:
                  {
                     me->done=ModemSendCommand("AT+CIPSHUT\r\n");
                     me->counter.tick =0;
                     state=close_gprs;
                  }
                  break;
               case  close_connection:
                  {
                     me->done=ModemSendCommand("AT+CIPSHUT\r\n");
                     me->counter.tick =0;
                     state=close_gprs;
                  }
                  break;
               default: break;
               }
         }
         return Q_HANDLED() ;
      case MODEM_SHUT_OK_SIG:
         {
            ServerState=server_not_connected;
            switch (state)
               {
               case  close_gprs:    state=exit;
                  break;
               default: break;
               }
         }
         return Q_HANDLED() ;
      case MODEM_ERROR_SIG: 
         {
            switch (state)
               {
               case  check_ip_status:
                  {
                     me->done=ModemSendCommand("AT+CIPCLOSE=1\r\n");
                     me->counter.tick =0;
                     state=close_connection;
                  }
                  break;
               case  close_connection:
                  {
                     me->done=ModemSendCommand("AT+CIPSHUT\r\n");
                     me->counter.tick =0;
                     state=close_gprs;
                  }
                  break;
               case  close_gprs: state=exit;
                  break;
									default: break;
               }
         }
         return Q_HANDLED() ;
      case MODEM_OK_SIG: return Q_HANDLED() ;      
      case TIC_100ms_SIG:
         {
            switch (state)
               {
               case  check_ip_status:
                  {
                     if (++me->counter.tick ==100)
                        {
                        me->done=ModemSendCommand("AT+CIPCLOSE=1\r\n");
                        state=close_connection;
                        }
                     else if (me->done==0) me->done=ModemSendCommand("AT+CIPSTATUS\r\n");
                  }
                  break;
               case  close_connection:
                  {
                     if (++me->counter.tick ==100)
                        {
                        me->done=ModemSendCommand("AT+CIPSHUT\r\n");
                        state=close_gprs;
                        }
                     else if (me->done==0)me->done=ModemSendCommand("AT+CIPCLOSE=1\r\n");
                  }
                  break;
               case  close_gprs:
                  {
                     if (++me->counter.tick ==100)state=exit;
                     else if (me->done==0) me->done=ModemSendCommand("AT+CIPSHUT\r\n");
                  }
                  break;
               case exit: if (me->flag.off==1) return Q_TRAN(&Modem_off_45);
                  else return Q_TRAN(&Modem_hard_reset_5);
               }
         }
         break;
      }
   return Q_SUPER(&Modem_top_1);
   }


QState Modem_open_connection_error_17(MOD * const me, QEvt const * const e) 
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(17);
            ServerState=server_connect_error;
            me->ret_ptr=(QStateHandler)&Modem_open_connection_error_17;
            me->counter.tick =0;
            ++me->error_counter.open_connection_error;
         }
         return Q_HANDLED() ;
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG:  return Q_TRAN(&Modem_sim_error_20);  
      case MODEM_OK_SIG: return Q_HANDLED() ;
      case MODEM_ERROR_SIG: return Q_HANDLED() ;
      case Q_EXIT_SIG:
         {
            me->flag.check_gsmq=0;
         }
         return Q_HANDLED() ;
      case TIC_100ms_SIG:
         {
            ++me->counter.tick;
            if ( me->flag.sms_send||me->flag.sms_new)
               {
               --me->error_counter.open_connection_error;
               return Q_TRAN(&Modem_sms_super_30);  
               }
            else if (me->counter.tick>=OPEN_CONNECTION_ERROR_TIMEOUT)
               switch (me->error_counter.open_connection_error)
                  {
                  case 1:
                  case 2:
                  case 3: return Q_TRAN(&Modem_close_connection_16);
                  case 4: return Q_TRAN(&Modem_close_gprs_10);
                  case 5: return Q_TRAN(&Modem_close_connection_16);
                  case 6: return Q_TRAN(&Modem_close_gprs_10);   									
                  default:
                     me->fatal_error_timer.open_connection=0;
                     return Q_TRAN(&Modem_open_connection_fatal_error_63);     
                  }
            else me->flag.check_gsmq=1;
         }
         break;
      }
   return Q_SUPER(&Modem_top_1);
   }


QState Modem_open_connection_fatal_error_63(MOD * const me, QEvt const * const e) 
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         { 

            AddStateHistory(63);
            me->ret_ptr=(QStateHandler)&Modem_open_connection_fatal_error_63;
           ClearErrorCounters(me);              
            if (me->flag.packet)
               {
               me->flag.fatal_error=1;     
               me->flag.packet=0;
               QACTIVE_POST(AO_Control, &SendPacketErrorEvt, me);   
               }
            ServerState=server_connect_error;
         }
         return Q_HANDLED() ;
      case TIC_100ms_SIG:
         {
           if ( me->flag.sms_send||me->flag.sms_new)
               {
               return Q_TRAN(&Modem_sms_super_30);  
               }
            if (++me->fatal_error_timer.open_connection==OPEN_CONNECTION_FATAL_ERROR_TIMEOUT)
               {
               return Q_TRAN(&Modem_hard_reset_5);   
               }
            else if (me->fatal_error_timer.open_connection%100==0)
               {
               OutDebugModemSprintf1("OPEN_CONNECTION_FATAL_ERROR_TIMEOUT remain=",OPEN_CONNECTION_FATAL_ERROR_TIMEOUT-me->fatal_error_timer.open_connection);
               }
         }
         break;
      }
   return Q_SUPER(&Modem_top_1);
   }


QState Modem_tcp_is_open_state_13(MOD * const me, QEvt const * const e) 
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(13);
            me->ret_ptr=(QStateHandler)&Modem_tcp_is_open_state_13;
            me->counter.tick =0;
            me->error_counter.authorization_error=0;
            me->flag.check_gsmq=1;
            GprsState=gprs_is_open;
            me->flag.get_ready=0;
            me->flag.fatal_error=0;
            QACTIVE_POST(AO_Control, &ModemReadyEvt, me);   
         }
         return Q_HANDLED();
				 case MODEM_REQUEST_TO_SEND_SIG:
				 {
					  QACTIVE_POST(AO_Control, &ModemReadyEvt, me);  
				 }
				  return Q_HANDLED();
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG:  return Q_TRAN(&Modem_sim_error_20);  
      case MODEM_OK_SIG: return Q_HANDLED() ;
      case MODEM_ERROR_SIG: return Q_HANDLED() ;
      case Q_EXIT_SIG:
         {
            me->flag.check_gsmq=0;
         }
         return Q_HANDLED() ;
      case TIC_100ms_SIG:
         {
            if ( me->flag.sms_send||me->flag.sms_new)return Q_TRAN(&Modem_sms_super_30);
//            if (me->flag.restart==1) return Q_TRAN(&Modem_restart_24);
            if (me->flag.packet==1)return Q_TRAN(&Modem_send_packet_22);
         }
         break;
      }
   return Q_SUPER(&Modem_top_1);
   }


QState Modem_authorization_18(MOD * const me, QEvt const * const e) 
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(18);
            if (conf.secur.protocol==PROTOCOL_FMXXXX)me->msm.send_packet.size=FM2200AuthPacketMaker(me,auth_pack_buf);
            else me->msm.send_packet.size= XTDeviceAuthPacketMaker(me,auth_pack_buf);
            me->msm.send_packet.ptr=auth_pack_buf;
            SendMsmSignal(me, MSM_SEND_PACKET_SIG);
            me->counter.tick =0;
            me->error_counter.open_connection_error=0;
            GprsState=gprs_is_open;
         }
         return Q_HANDLED() ;
      case MODEM_IPSEND_SIG:
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG:
      case MODEM_OK_SIG: 
      case MODEM_ERROR_SIG:
      case MODEM_CLOSED_SIG:
      case MODEM_SEND_OK_SIG:
      case MODEM_TCP_DATA_RECEIVED_SIG:
      case MODEM_SERVER_DATA_SIG:
         {
            ModemSM_dispatch(&me->msm, e);
         }
         return Q_HANDLED();
      case MSM_ANSWER_SIG:
         switch (((MsmAnswerType*)e)->answer)
            {
            case ANSWER_OK: 
               if (me->msm.server_data.size==1)
                  {
                  uint8_t answer=*(me->msm.server_data.ptr);
                  switch (answer)
                     {
                     case 1:
                        {
                           OutDebugModem("AUTHORIZATION OK"); 
                           ServerState=server_authorized_and_connected;
                           return Q_TRAN(&Modem_tcp_is_open_state_13); 
                        }
                     default:
                        {
                           OutDebugModem("AUTHORIZATION ERROR"); 
                           return Q_TRAN(&Modem_authorization_error_19); 
                        }
                     }
                  }
               else return Q_TRAN(&Modem_authorization_error_19);
            case ANSWER_SIM_ERROR: return Q_TRAN(&Modem_sim_error_20);
            default: return Q_TRAN(&Modem_authorization_error_19);
            }
      case Q_EXIT_SIG: return Q_HANDLED() ;
      case TIC_100ms_SIG:
         {
            ModemSM_dispatch(&me->msm, e); 
         }
         break;
      }
   return Q_SUPER(&Modem_top_1);
   }

QState Modem_authorization_error_19(MOD * const me, QEvt const * const e) 
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(19);
            me->ret_ptr=(QStateHandler)&Modem_authorization_error_19;
            me->counter.tick =0;
            ++me->error_counter.authorization_error;
            ServerState=server_authorization_error;
         }
         return Q_HANDLED() ;
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG:  return Q_TRAN(&Modem_sim_error_20);  
      case MODEM_OK_SIG: return Q_HANDLED() ;
      case MODEM_ERROR_SIG: return Q_HANDLED() ;
      case Q_EXIT_SIG:
         {
            me->flag.check_gsmq=0;
         }
         return Q_HANDLED() ;
      case TIC_100ms_SIG:
         {
            if ( me->flag.sms_send||me->flag.sms_new)
               {
               --me->error_counter.authorization_error;
               return Q_TRAN(&Modem_sms_super_30);  
               }
            else if (++me->counter.tick==AUTHORIZATION_ERROR_TIMEOUT)
               {
               switch (me->error_counter.authorization_error)
                  {
                  case 1:
                  case 2:
                  case 3:  return Q_TRAN(&Modem_close_connection_16);
                  case 4:  return Q_TRAN(&Modem_close_gprs_10);
									case 5:  return Q_TRAN(&Modem_close_connection_16);
                  case 6:  return Q_TRAN(&Modem_close_gprs_10);
                  default:
                     me->fatal_error_timer.authorization=0; 
                     return Q_TRAN(&Modem_authorization_fatal_error_64);
                  }
               }
            else me->flag.check_gsmq=1;
         }
         break;
      }
   return Q_SUPER(&Modem_top_1);
   }

QState Modem_authorization_fatal_error_64(MOD * const me, QEvt const * const e) 
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         { 
            AddStateHistory(64);
            me->ret_ptr=(QStateHandler)&Modem_authorization_fatal_error_64;
            ClearErrorCounters(me);              
            if (me->flag.packet)
               {
               me->flag.packet=0;
               QACTIVE_POST(AO_Control, &SendPacketErrorEvt, me);    
               }
							
         }
         return Q_HANDLED() ;   
      case TIC_100ms_SIG:
         {
            if ( me->flag.sms_send||me->flag.sms_new)
               {
               return Q_TRAN(&Modem_sms_super_30);  
               }
            if (++me->fatal_error_timer.authorization==AUTHORIZATION_FATAL_ERROR_TIMEOUT)
               {
               return Q_TRAN(&Modem_hard_reset_5);                    
               }
            else if (me->fatal_error_timer.authorization%100==0)
               {
               OutDebugModemSprintf1("AUTHORIZATION_FATAL_ERROR_TIMEOUT remain=",AUTHORIZATION_FATAL_ERROR_TIMEOUT-me->fatal_error_timer.authorization);
               }
         }
         break;
      }
   return Q_SUPER(&Modem_top_1);
   }


QState Modem_sim_error_20(MOD * const me, QEvt const * const e) 
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         { 
            AddStateHistory(20);
            me->counter.tick=0;
            me->flag.check_gsmq=0;
            SysInfo.gsmq=0;    
            ModemTxPin_deinit();
            ModemRxPin_deinit();
            POWERKEY_DOWN();
            MODEM_POWER_OFF(); 					 
						++me->error_counter.sim_error;
            GsmState=gsm_sim_error;
            GprsState=gprs_off;
            ServerState=server_not_connected;
					  me->sim_error=1;
         }
         return Q_HANDLED() ;
      case TIC_100ms_SIG:
         {
					  if (me->error_counter.sim_error>1)
						{
							me->fatal_error_timer.sim=0;
							 return Q_TRAN(&Modem_sim_fatal_error_66);  
						}
          else if (++me->counter.tick==SIM_ERROR_TIMEOUT)
               {
               return Q_TRAN(&Modem_power_on_start_up_2);                    
               }
            else if (me->counter.tick%100==0)
               {
               OutDebugModemSprintf2("SIM_ERROR timeout=","count=",SIM_ERROR_TIMEOUT-me->counter.tick,me->error_counter.sim_error);
               }
							 
         }
         break;
      }
   return Q_SUPER(&Modem_top_1);
   }
	 
	 
	 QState Modem_sim_fatal_error_66(MOD * const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         { 
            AddStateHistory(66);
            me->counter.tick=0;
            me->flag.check_gsmq=0;
            SysInfo.gsmq=0; 
            ClearErrorCounters(me);              
            if (me->flag.packet)
               {
               me->flag.packet=0;
               me->flag.fatal_error=1;      
               QACTIVE_POST(AO_Control, &SendPacketErrorEvt, me);                
               }
						if(me->flag.upd)
							 {
								 QACTIVE_POST(AO_Control, &ControlUpdTransportLevelErrorEvt, me); 
							 }
         }
         return Q_HANDLED() ;
      case TIC_100ms_SIG:
         {
            if (++me->fatal_error_timer.sim==SIM_FATAL_ERROR_TIMEOUT)
               {
                  return Q_TRAN(&Modem_power_on_start_up_2);                       
               }
            else if (me->fatal_error_timer.sim%100==0)
               {
               OutDebugModemSprintf1("SIM_FATAL_ERROR_TIMEOUT remain=",SIM_FATAL_ERROR_TIMEOUT-me->fatal_error_timer.sim);
               }
         }
         break;
      }
   return Q_SUPER(&Modem_top_1);
   }



QState Modem_sim_lock_error_21(MOD * const me, QEvt const * const e) 
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         { 
            AddStateHistory(21);
            me->flag.check_gsmq=0;
            SysInfo.gsmq=0;              
						me->counter.tick=0;
						++me->error_counter.sim_lock_error;
            GsmState=gsm_sim_lock_error;
            GprsState=gprs_off;
            ServerState=server_not_connected;
					 me->sim_error=1;
         }
         return Q_HANDLED() ;
      case TIC_100ms_SIG:
         {
					 if(me->error_counter.sim_lock_error>3)
					   {
						    me->fatal_error_timer.sim_lock=0;
							  return Q_TRAN(&Modem_sim_lock_fatal_error_67);
					   }
           else if (++me->counter.tick>=SIM_LOCK_TIMEOUT)
               {
               return Q_TRAN(&Modem_power_on_start_up_2);                    
               }
            else if (me->counter.tick%100==0)
               {
               OutDebugModemSprintf1("SIM_LOCK_TIMEOUT remain=",SIM_LOCK_TIMEOUT-me->counter.tick);
               }
         }
         break;
      }
   return Q_SUPER(&Modem_top_1);
   }
	 
	  QState Modem_sim_lock_fatal_error_67(MOD * const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         { 
            AddStateHistory(67);
            me->counter.tick=0;
            ModemTxPin_deinit();
            ModemRxPin_deinit();
            POWERKEY_DOWN();
            MODEM_POWER_OFF(); 
            me->flag.check_gsmq=0;
            SysInfo.gsmq=0; 
           ClearErrorCounters(me);              
            if (me->flag.packet)
               {
               me->flag.packet=0;
               me->flag.fatal_error=1;      
               QACTIVE_POST(AO_Control, &SendPacketErrorEvt, me);                
               }
						if(me->flag.upd)
							 {
								 QACTIVE_POST(AO_Control, &ControlUpdTransportLevelErrorEvt, me); 
							 }
         }
         return Q_HANDLED() ;
      case TIC_100ms_SIG:
         {
            if (++me->fatal_error_timer.sim_lock==SIM_LOCK_FATAL_ERROR_TIMEOUT)
               {
                  return Q_TRAN(&Modem_power_on_start_up_2);                       
               }
            else if (me->fatal_error_timer.sim_lock%100==0)
               {
               OutDebugModemSprintf1("SIM_FATAL_ERROR_TIMEOUT remain=",SIM_LOCK_FATAL_ERROR_TIMEOUT-me->fatal_error_timer.sim_lock);
               }
         }
         break;
      }
   return Q_SUPER(&Modem_top_1);
   }



QState Modem_send_packet_22(MOD * const me, QEvt const * const e) 
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(22);
            me->msm.send_packet.ptr=me->data_packet.ptr;
            me->msm.send_packet.size=me->data_packet.size;
            SendMsmSignal(me, MSM_SEND_PACKET_SIG);
            me->counter.tick =0;
            GprsState=gprs_is_open;
         }
         return Q_HANDLED() ;
      case MODEM_IPSEND_SIG:
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG:
      case MODEM_OK_SIG: 
      case MODEM_ERROR_SIG:
      case MODEM_CLOSED_SIG:
      case MODEM_SEND_OK_SIG:
      case MODEM_TCP_DATA_RECEIVED_SIG:
      case MODEM_SERVER_DATA_SIG:
         {
            ModemSM_dispatch(&me->msm, e);
         }
         return Q_HANDLED();
      case MSM_ANSWER_SIG:
         if (conf.secur.protocol==PROTOCOL_FMXXXX)
            {
            if (((MsmAnswerType*)e)->answer==ANSWER_OK)//Send OK
               {
               if (me->msm.server_data.size==4)
                  {
                  uint32_t answer;
                  BufToLittleEndianInt(me->msm.server_data.ptr,&answer);
                  OutDebugModemSprintf1("TELTONIKA SEND OK, packets received=",answer);
                  ind_flag.send_packet_succes=1;
                  me->counter.resend=0;
                  me->error_counter.send_packet_error=0;
                  me->flag.packet=0;
                  QACTIVE_POST(AO_Control, &SendPacketOkEvt, me);   
                  return Q_TRAN(&Modem_gprs_is_open_state_11);       
                  }
               else
                  {
                  OutDebugModem("ERROR!!! UNKNOWN SERVER ANSWER");
                  return Q_TRAN(&Modem_send_packet_error_23);   
                  }          
               }
            else
               {
               OutDebugModem("SEND PACKET ERROR");
               return Q_TRAN(&Modem_send_packet_error_23);  
               } 
            }
         else//.protocol==PROTOCOL_QUANTUM)
            {
            if (((MsmAnswerType*)e)->answer==ANSWER_OK)//Send OK
               {
               if (me->msm.server_data.size==1)
                  {
                  uint8_t answer=*(me->msm.server_data.ptr);
                  switch (answer)
                     {
                     case 1:
                        {
                           OutDebugModem("QUANTUM SEND OK"); 
                           ind_flag.send_packet_succes=1;
                           me->counter.resend=0;
                           me->error_counter.send_packet_error=0;
                           me->flag.packet=0;
                           QACTIVE_POST(AO_Control, &SendPacketOkEvt, me);   
                           return Q_TRAN(&Modem_gprs_is_open_state_11);       
                        }
                     default:
                        {
                           if (++me->counter.resend<5)
                              {
                              return Q_TRAN(&Modem_send_packet_22); 
                              }
                           else
                              {
                              me->counter.resend=0;
                              me->flag.packet=0;      
                              QACTIVE_POST(AO_Control, &BadPacketEvt, me);   
                              me->error_counter.send_packet_error=0;
                              return Q_TRAN(&Modem_gprs_is_open_state_11);                     
                              }
                        }
                     }    
                  }
               else
                  {
                  OutDebugModem("ERROR!!! UNKNOWN SERVER ANSWER");
                  return Q_TRAN(&Modem_send_packet_error_23);   
                  }          
               }
            else
               {
               OutDebugModem("SEND PACKET ERROR");
               return Q_TRAN(&Modem_send_packet_error_23);  
               } 
            } 
      case Q_EXIT_SIG: return Q_HANDLED() ;
      case TIC_100ms_SIG:
         {
            ModemSM_dispatch(&me->msm, e); 
         }
         break;
      }
   return Q_SUPER(&Modem_top_1);
   }



QState Modem_send_packet_error_23(MOD * const me, QEvt const * const e) 
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(23);
            me->ret_ptr=(QStateHandler)&Modem_send_packet_error_23;
            me->counter.tick =0;
            ++me->error_counter.send_packet_error;  
            me->flag.check_gsmq=1;   
         }
         return Q_HANDLED() ;
      case MODEM_OK_SIG:    return Q_HANDLED() ; 
      case MODEM_ERROR_SIG: return Q_HANDLED() ; 
      case Q_EXIT_SIG: 
         {
            me->flag.check_gsmq=0;   
         }       
         return Q_HANDLED() ;
      case TIC_100ms_SIG:
        if ( me->flag.sms_send||me->flag.sms_new)
            {
            --me->error_counter.send_packet_error;
            return Q_TRAN(&Modem_sms_super_30);  
            }
         switch (me->error_counter.send_packet_error)
            {
            case 1:
            case 2:
            case 3:  return Q_TRAN(&Modem_close_connection_16);
            case 4:  return Q_TRAN(&Modem_close_gprs_10);
            default:
               me->fatal_error_timer.send_packet=0;
               return Q_TRAN(&Modem_send_packet_fatal_error_65);
            }
      }
   return Q_SUPER(&Modem_top_1);
   }

QState Modem_send_packet_fatal_error_65(MOD * const me, QEvt const * const e) 
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         { 
            AddStateHistory(65);
            me->ret_ptr=(QStateHandler)&Modem_send_packet_fatal_error_65;
            ClearErrorCounters(me); 
            me->flag.fatal_error=1;              
            me->flag.packet=0;
            QACTIVE_POST(AO_Control, &SendPacketErrorEvt, me);
         }
         return Q_HANDLED() ;
      case MODEM_OK_SIG: return Q_HANDLED() ;
      case MODEM_ERROR_SIG: return Q_HANDLED() ;   
      case TIC_100ms_SIG: 
         {
            if ( me->flag.sms_send||me->flag.sms_new)
               {
               return Q_TRAN(&Modem_sms_super_30);  
               }
            if (++me->fatal_error_timer.send_packet==SEND_PACKET_FATAL_ERROR_TIMEOUT)
               {
               return Q_TRAN(&Modem_hard_reset_5);                
               }
            else if (me->fatal_error_timer.send_packet%100==0)
               {
               OutDebugModemSprintf1("SEND_PACKET_FATAL_ERROR_TIMEOUT remain=",SEND_PACKET_FATAL_ERROR_TIMEOUT-me->fatal_error_timer.send_packet);
               }
         }
         break;
      }
   return Q_SUPER(&Modem_top_1);
   }

//-------------------SMS---------------------------------------------------------------------

QState Modem_sms_final_39(MOD * const me, QEvt const * const e) 
   {
   static QEvt const SmsFinalEvt = { MODEM_SMS_FINAL_SIG, 0U, 0U};
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         { 
            AddStateHistory(39);
            QACTIVE_POST(AO_Modem, &SmsFinalEvt, me);    
         }
         return Q_HANDLED() ;
      }
   return Q_SUPER(&Modem_sms_super_30);
   }

QState Modem_sms_delete_34(MOD * const me, QEvt const * const e) 
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         { 
            AddStateHistory(34);
            sprintf(me->command_buf,"AT+CMGD=%u\r\n",me->sms_index);
            ModemSendCommand(me->command_buf);
            OutDebugModem(me->command_buf);
            me->error_timeout=60;//6 sec 
            QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,BSP_TICKS_PER_SEC*5); //5 sec delay
            me->done=0;          
         }
         return Q_HANDLED() ;
      case MODEM_OK_SIG: 
         {
            me->done=1;
         }
         return Q_TRAN(&Modem_sms_read_next_31);
      case MODEM_TIMEOUT_SIG: 
         {
            // OutDebugModem("DELETE SMS ERROR");
         }
         return Q_TRAN(&Modem_sms_read_next_31);
      case Q_EXIT_SIG:
         {
            QTimeEvt_disarm(&me->TimeEvt);
            if (me->done==1) OutDebugModem("DELETE SMS OK");
            else  OutDebugModem("DELETE SMS ERROR");
         }
         return Q_HANDLED() ;
      }
   return Q_SUPER(&Modem_sms_super_30);
   }

QState ModemSmsSendAnsw36(MOD * const me, QEvt const * const e) 
   {
   QState ret= Q_HANDLED() ;
   static enum
      {
      pause,
      push_data,
      waiting_ok,     
      }state;
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(36);
            me->error_timeout=10;//1 sec 
            QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,20); //100 ms delay
            OutDebugModem("PAUSE");
            state=pause;           
         }
         return Q_HANDLED() ;
      case MODEM_TIMEOUT_SIG: 
         {
            if (state==pause)
               {
               ModemSendCommand(send_sms); 
               QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,40); //40*5=200 ms delay
               OutDebugModem("PUSH DATA");
               state=push_data;             
               }
            else if (state==push_data)
               {
               char send[]={0x1A,0};
               ModemSendCommand(send); 
               OutDebugModem("WAITING OK");
               me->error_timeout=400;//40 sec guard timeout
               QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,BSP_TICKS_PER_SEC*35); //35 sec guard timeout
               state=waiting_ok;             
               }
            else if (state==waiting_ok)
               {
               OutDebugModem("SEND SMS ERROR");
               return Q_TRAN(&ModemSmsProcSmsFlags37);           
               }
         }
         return Q_HANDLED() ;
      case MODEM_OK_SIG:
         {
            if (state==waiting_ok)
               {
               OutDebugModem("SEND SMS OK");
               ind_flag.send_sms_succes=1; 
               ret= Q_TRAN(&ModemSmsProcSmsFlags37);
               }
            else
               {
               OutDebugModem("ERROR! UNEXPECTED OK SIG");
               ret= Q_HANDLED();
               }
         }
         return ret;
      case Q_EXIT_SIG:
         {
            QTimeEvt_disarm(&me->TimeEvt);
         }
         return Q_HANDLED();
      }
   return Q_SUPER(&Modem_sms_super_30);
   }
	 
	 QState ModemSmsSendServ49(MOD * const me, QEvt const * const e) 
   {
   QState ret= Q_HANDLED() ;
   static QEvt const errorSmsProcessedEvt = { CONTROL_ERROR_SMS_PROCESSED_SIG, 0U, 0U};
	 
   static enum
      {
      pause,
      push_data,
      waiting_ok,     
      }state;
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(49);
            me->error_timeout=10;//1 sec 
            QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,20); //100 ms delay
            OutDebugModem("PAUSE");
            state=pause;           
         }
         return Q_HANDLED() ;
      case MODEM_TIMEOUT_SIG: 
         {
            if (state==pause)
               {
               ModemSendCommand(send_sms); 
               QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,40); //40*5=200 ms delay
               OutDebugModem("PUSH DATA");
               state=push_data;             
               }
            else if (state==push_data)
               {
               char send[]={0x1A,0};
               ModemSendCommand(send); 
               OutDebugModem("WAITING OK");
               me->error_timeout=300;//30 sec guard timeout
               QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,BSP_TICKS_PER_SEC*25); //25 sec guard timeout
               state=waiting_ok;             
               }
            else if (state==waiting_ok)
               {
               OutDebugModem("SEND SMS ERROR");
               return Q_TRAN(&ModemSmsProcSmsFlags37);           
               }
         }
         return Q_HANDLED() ;
      case MODEM_OK_SIG:
         {
            if (state==waiting_ok)
               {
               OutDebugModem("SEND SMS OK");
               ind_flag.send_sms_succes=1; 
               ret= Q_TRAN(&ModemSmsProcSmsFlags37);
               }
            else
               {
               OutDebugModem("ERROR! UNEXPECTED OK SIG");
               ret= Q_HANDLED();
               }
         }
         return ret;
      case Q_EXIT_SIG:
         {
					 QACTIVE_POST(AO_Control, &errorSmsProcessedEvt, me); 
            QTimeEvt_disarm(&me->TimeEvt);
         }
         return Q_HANDLED();
      }
   return Q_SUPER(&Modem_sms_super_30);
   }

QState ModemSmsRequestToSendAnswer_35(MOD * const me, QEvt const * const e) 
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {    
            AddStateHistory(35);
            QACTIVE_POST(AO_ModemParser, &ParserWaitingIpSendEvt, me);  
            sprintf(me->command_buf,"AT+CMGS=%s\r\n",me->sms_tf); 
            ModemSendCommand(me->command_buf);      
            me->error_timeout=100;//10 sec		
            QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,BSP_TICKS_PER_SEC*5); //5 sec delay     					 
         }
         return Q_HANDLED() ;
      case MODEM_TIMEOUT_SIG:   
         {
            OutDebugModem("SEND SMS ERROR"); 
         }
         return Q_TRAN(&ModemSmsProcSmsFlags37);
      case MODEM_IPSEND_SIG: return Q_TRAN(&ModemSmsSendAnsw36); 
      case Q_EXIT_SIG:
         {
            QTimeEvt_disarm(&me->TimeEvt);
         }
         return Q_HANDLED();
      }
   return Q_SUPER(&Modem_sms_super_30);
   }
	 
	 QState ModemSmsRequestToSendServ48(MOD * const me, QEvt const * const e) 
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {    
            AddStateHistory(48);
            QACTIVE_POST(AO_ModemParser, &ParserWaitingIpSendEvt, me);  
            sprintf(me->command_buf,"AT+CMGS=%s\r\n",OLEG_PHONE); 
            ModemSendCommand(me->command_buf);      
            me->error_timeout=100;//10 sec		
            QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,BSP_TICKS_PER_SEC*5); //5 sec delay     					 
         }
         return Q_HANDLED() ;
      case MODEM_TIMEOUT_SIG:   
         {
            OutDebugModem("SEND SMS ERROR"); 
         }
         return Q_TRAN(&ModemSmsDeleteAll50);
      case MODEM_IPSEND_SIG: return Q_TRAN(&ModemSmsSendServ49); 
      case Q_EXIT_SIG:
         {
            QTimeEvt_disarm(&me->TimeEvt);
         }
         return Q_HANDLED();
      }
   return Q_SUPER(&Modem_sms_super_30);
   }

QState Modem_sms_check_balance_38(MOD * const me, QEvt const * const e) 
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {    
            AddStateHistory(38);
            strcpy(me->command_buf,"AT+CUSD=1,\""); 
            strcat(me->command_buf,gprs_ptr->ussd_balance);           
            strcat(me->command_buf,"\"\r\n"); 
            ModemSendCommand(me->command_buf);
            OutDebugModem(me->command_buf);
            me->error_timeout=30;//3 sec	
            QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,BSP_TICKS_PER_SEC*2); //2 sec delay   					 
            OutDebugModem("PROCESS USSD");         
         }
         return Q_HANDLED() ;
      case MODEM_TIMEOUT_SIG:   
         {
            OutDebugModem("PROCESS USSD ERROR");  
         }
         return Q_TRAN(&ModemSmsProcSmsFlags37);
      case MODEM_OK_SIG: 
         { 
            me->error_timeout=250;//25 sec	
            QTimeEvt_disarm(&me->TimeEvt);
            QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,BSP_TICKS_PER_SEC*20); //20 sec delay   						 
         }
         return Q_HANDLED() ;
      case MODEM_USSD_SIG:
         {                                                                  
            strcpy(send_sms,((SmsType*)((DataPtrEvt*)e)->ptr)->buf);           
         }
         return Q_TRAN(&ModemSmsRequestToSendAnswer_35);
      case Q_EXIT_SIG:
         {
            QTimeEvt_disarm(&me->TimeEvt);
         }
         return Q_HANDLED();
      }
   return Q_SUPER(&Modem_sms_super_30);
   }


QState ModemSmsProcSmsFlags37(MOD * const me, QEvt const * const e) 
   {
		  uint8_t sms_len;
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {    
            AddStateHistory(37);
            me->error_timeout=10; 
            QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1); 				 
         }
         return Q_HANDLED() ;
      case MODEM_TIMEOUT_SIG: 
         {
             if (parser_sms_answer.flag.save_config==1)
               {
               parser_sms_answer.flag.save_config=0;
               QACTIVE_POST(AO_Disk, &DiskSaveConfigEvt, me);
               me->error_timeout=10;//1 sec 
               QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,10);//50 ms
               }
            else if (parser_sms_answer.flag.get_sms_config_gprs==1)
               {
               parser_sms_answer.flag.get_sms_config_gprs=0;
               MakeGprsSms(send_sms,MAX_SMS_SIZE);
               sms_len=strlen(send_sms);
               if (sms_len>0)
                  {
                  OutDebugModemSprintf1(send_sms,sms_len);
                  return Q_TRAN(&ModemSmsRequestToSendAnswer_35);    
                  }
               else
                  {
                  OutDebugModem("ERROR! GPRS SMS IS EMPTY");
                  QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
                  }
               }
            else if (parser_sms_answer.flag.get_sms_config_nav==1)
               {
               parser_sms_answer.flag.get_sms_config_nav=0;
               MakeNavSms(send_sms,MAX_SMS_SIZE);
               sms_len=strlen(send_sms);
               if (sms_len>0)
                  {
                  OutDebugModemSprintf1(send_sms,sms_len);
                  return Q_TRAN(&ModemSmsRequestToSendAnswer_35);    
                  }
               else
                  {
                  OutDebugModem("ERROR! NAV SMS IS EMPTY");
                  QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
                  }
               }
						 else if (parser_sms_answer.flag.get_sms_config_pack==1)
               {
               parser_sms_answer.flag.get_sms_config_pack=0;
               MakePackSms(send_sms,MAX_SMS_SIZE);
               sms_len=strlen(send_sms);
               if (sms_len>0)
                  {
                  OutDebugModemSprintf1(send_sms,sms_len);
                  return Q_TRAN(&ModemSmsRequestToSendAnswer_35);    
                  }
               else
                  {
                  OutDebugModem("ERROR! PACK SMS IS EMPTY");
                  QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
                  }
               }
							  else if (parser_sms_answer.flag.get_sms_modem==1)
               {
               parser_sms_answer.flag.get_sms_modem=0;
               MakeModemSms(send_sms,MAX_SMS_SIZE);
               sms_len=strlen(send_sms);
               if (sms_len>0)
                  {
                  OutDebugModemSprintf1(send_sms,sms_len);
                  return Q_TRAN(&ModemSmsRequestToSendAnswer_35);    
                  }
               else
                  {
                  OutDebugModem("ERROR! MODEM SMS IS EMPTY");
                  QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
                  }
               }
            else if (parser_sms_answer.flag.get_sms_ver==1)
               {
               parser_sms_answer.flag.get_sms_ver=0;
               MakeVerSms(send_sms,MAX_SMS_SIZE);
               sms_len=strlen(send_sms);
               if (sms_len>0)
                  {
                  OutDebugModemSprintf1(send_sms,sms_len);
                  return Q_TRAN(&ModemSmsRequestToSendAnswer_35);    
                  }
               else
                  {
                  OutDebugModem("ERROR! VER SMS IS EMPTY");
                  QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
                  }
               }
							 else if (parser_sms_answer.flag.get_sms_status==1)
               {
               parser_sms_answer.flag.get_sms_status=0;
               MakeStatusSms(me,send_sms,MAX_SMS_SIZE);
               sms_len=strlen(send_sms);
               if (sms_len>0)
                  {
                  OutDebugModemSprintf1(send_sms,sms_len);
                  return Q_TRAN(&ModemSmsRequestToSendAnswer_35);    
                  }
               else
                  {
                  OutDebugModem("ERROR! VER SMS IS EMPTY");
                  QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
                  }
               }
            else if (parser_sms_answer.flag.get_sms_config_rs485==1)
               {
               parser_sms_answer.flag.get_sms_config_rs485=0;
               MakeRS485Sms(send_sms,MAX_SMS_SIZE);
               sms_len=strlen(send_sms);
               if (sms_len>0)
                  {
                  OutDebugModemSprintf1(send_sms,sms_len);
                  return Q_TRAN(&ModemSmsRequestToSendAnswer_35);    
                  }
               else
                  {
                  OutDebugModem("ERROR! SENS SMS IS EMPTY");
                  QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
                  }
               }
							  else if (parser_sms_answer.flag.get_sms_imei==1)
               {
               parser_sms_answer.flag.get_sms_imei=0;
							 MakeImeiSms(me,send_sms,MAX_SMS_SIZE);
               sms_len=strlen(send_sms);
               if (sms_len>0)
                  {
                  OutDebugModemSprintf1(send_sms,sms_len);
                  return Q_TRAN(&ModemSmsRequestToSendAnswer_35);    
                  }
               else
                  {
                  OutDebugModem("ERROR! IMEI SMS IS EMPTY");
                  QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
                  }
               }
							
            else if (parser_sms_answer.flag.get_sms_balance==1)
               {
               parser_sms_answer.flag.get_sms_balance=0;
               return Q_TRAN(&Modem_sms_check_balance_38);   
               }
						else if (parser_sms_answer.flag.reboot_system ==1)
               {
								 parser_sms_answer.flag.reboot_system=0;
                QACTIVE_POST(AO_Control, &CtrlRebootEvt, me);
                 return Q_TRAN(&Modem_sms_delete_34);							 
               }
            else //PROCESS FLAGS COMPLETE
               {
               return Q_TRAN(&Modem_sms_delete_34);
               }
         }
         return Q_HANDLED() ;
      case Q_EXIT_SIG:
         {
            QTimeEvt_disarm(&me->TimeEvt);
         }
         return Q_HANDLED();
      }
   return Q_SUPER(&Modem_sms_super_30);
   }

	 
	 QState ModemSmsDeleteAll50(MOD * const me, QEvt const * const e) 
   {
//		  uint8_t sms_len;
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {    
            AddStateHistory(50);
					  ModemSendCommand("AT+CMGDA=\"DEL ALL\"\r\n");
					 //ModemSendCommand("AT+CMGDA=?\r\n");
					// ModemSendCommand("AT+CMGD=?");
            me->error_timeout=300; //30 sec				 
         }
         return Q_HANDLED();
				 case MODEM_OK_SIG: 
				 case MODEM_ERROR_SIG: 
				 {
					 me->flag.sms_send=0;
				 }
				 return Q_TRAN(&Modem_sms_final_39);
      case Q_EXIT_SIG:
         {
         }
         return Q_HANDLED();
      }
   return Q_SUPER(&Modem_sms_super_30);
   }

QState Modem_sms_waiting_read_ok_32(MOD * const me, QEvt const * const e) 
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(32);
            me->error_timeout=50;//5 sec
            QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,BSP_TICKS_PER_SEC*4);//4 sec  	
         }
         return Q_HANDLED();
      case MODEM_TIMEOUT_SIG:
      case MODEM_OK_SIG: return Q_TRAN(&Modem_sms_delete_34); 
      case Q_EXIT_SIG:
         {
            QTimeEvt_disarm(&me->TimeEvt);
         }
         return Q_HANDLED();
      }
   return Q_SUPER(&Modem_sms_super_30);
   }

QState Modem_sms_waiting_unread_ok_33(MOD * const me, QEvt const * const e) 
   {
   QState ret=Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(33);
            me->error_timeout=50;//5 sec
            QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,BSP_TICKS_PER_SEC*4);//4 sec  	
         }
         return Q_HANDLED();
      case MODEM_TIMEOUT_SIG:
         {
            OutDebugModem("TIMEOUT ERROR");
         }
        return  Q_TRAN(&Modem_sms_read_next_31);
      case MODEM_OK_SIG: 
         {
            parser_sms_answer.sms_ptr=me->input_sms_ptr->buf;
					 __disable_irq();
            ParserSms(&parser_sms_answer);
					 __enable_irq();
            switch (parser_sms_answer.answer)
               {
               case PROCESS_SMS_OK:
                  {                                
                     OutDebugModem("PROCESS_SMS_OK");
                     ret= Q_TRAN(&ModemSmsProcSmsFlags37);
                  }
                  break;
               case PROCESS_SMS_PASSWORD_ERROR: 
                  {
                     OutDebugModem("PASSWORD_ERROR");
                     ret= Q_TRAN(&Modem_sms_read_next_31);
                  }
                  break;
               case PROCESS_SMS_FORMAT_ERROR: 
                  {
                     OutDebugModem("FORMAT_ERROR");
                     ret= Q_TRAN(&Modem_sms_read_next_31);
                  }
                  break;
               }
         }
         return ret; 
      case Q_EXIT_SIG:
         {
            QTimeEvt_disarm(&me->TimeEvt);
         }
         return Q_HANDLED();
      }
   return Q_SUPER(&Modem_sms_super_30);
   }


QState Modem_sms_read_next_31(MOD * const me, QEvt const * const e) 
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         { 
            AddStateHistory(31);
            me->sms_index++;
            me->error_timeout=50;
            QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);//5 ms 
         }
         return Q_HANDLED() ;
      case MODEM_TIMEOUT_SIG:
         if (me->sms_index<=MAX_SMS_PROCESSING)
            {
            sprintf(me->command_buf,"AT+CMGR=%u,0\r\n",me->sms_index);//read next,change status
            ModemSendCommand(me->command_buf);
            OutDebugModem(me->command_buf);
            return Q_HANDLED();
            }
         else
            {
            me->error_timeout=50;
            return Q_TRAN(&ModemSmsDeleteAll50);
            }
      case MODEM_OK_SIG:  return Q_TRAN(&Modem_sms_read_next_31); //nonexistent message
      case MODEM_SMS_IS_READ_SIG:
         {
            OutDebugModem("IS_READ");
         }
         return Q_TRAN(&Modem_sms_waiting_read_ok_32);
      case MODEM_SMS_IS_UNREAD_SIG:
         {
            OutDebugModem("IS_UNREAD");
            me->input_sms_ptr= ((DataPtrEvt*)e)->ptr;
            strcpy(me->sms_tf,me->input_sms_ptr->tf_num);
         }
         return Q_TRAN(&Modem_sms_waiting_unread_ok_33); 
      case Q_EXIT_SIG:
         {
            QTimeEvt_disarm(&me->TimeEvt);
         }
         return Q_HANDLED();         
      }
   return Q_SUPER(&Modem_sms_super_30);
   }


QState Modem_sms_super_30(MOD * const me, QEvt const * const e) 
   {
   QState ret;
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         { 
            AddStateHistory(30);
            me->error_timeout=10;
            me->sms_index=0;
         }
         return Q_HANDLED() ;
      case Q_INIT_SIG: 
			{
				//if(me->flag.sms_new)
				//{
					me->flag.sms_new=0;
					ret= Q_TRAN(&Modem_sms_read_next_31);
				//}
				//else ret= Q_TRAN(&ModemSmsDeleteAll50);
			}
			return ret;
      case MODEM_ERROR_SIG:
         {
            OutDebugModem("ERROR_SIG");
            if (parser_sms_answer.flag.restart_modem==1)
               {
               OutDebugModem("restart_modem==1");
               parser_sms_answer.flag.restart_modem=0;
               ret=Q_TRAN(&Modem_restart_4);
               }
            else if (me->flag.upd==1)
               {
               OutDebugModem("upd==1");
               ret=Q_TRAN(&Modem_restart_4);
               }
            else ret= Q_TRAN(me->ret_ptr);
         }
         return ret;
      case MODEM_SMS_FINAL_SIG : 
         {
            OutDebugModem("PROCESS_SMS_COMPLETE_SIG");
					 if(me->flag.sms_send)
				     {
					    me->flag.sms_send=0;
					    ret= Q_TRAN(&ModemSmsProcSmsFlags37);
				     }
            else if (parser_sms_answer.flag.restart_modem==1)
               {
               OutDebugModem("restart_modem==1");
               parser_sms_answer.flag.restart_modem=0;
               ret=Q_TRAN(&Modem_restart_4);
               }
            else if (me->flag.upd==1)
               {
               OutDebugModem("upd==1");
               ret=Q_TRAN(&Modem_restart_4);
               }
            else ret= Q_TRAN(me->ret_ptr);
         }
         return ret;
      case TIC_100ms_SIG: 
         {
            me->error_timeout--;
            if (me->error_timeout==0)
               {
               OutDebugModem("PROCESS SMS TIMEOUT ERROR");
               if (parser_sms_answer.flag.restart_modem==1)
                  {
                  OutDebugModem("restart_modem==1");
                  parser_sms_answer.flag.restart_modem=0;
                  return Q_TRAN(&Modem_restart_4);
                  }
               else if (me->flag.upd==1)
                  {
                  OutDebugModem("upd==1");
                  return Q_TRAN(&Modem_restart_4);
                  }
               else return Q_TRAN(me->ret_ptr);
               }
            else
               {
               if (me->error_timeout%10==0)
                  {
                  OutDebugModemSprintf1("PROCESS SMS TIMEOUT=",me->error_timeout);
                  }
               }
         }
         return Q_SUPER(&Modem_top_1);
      case Q_EXIT_SIG: return Q_HANDLED() ;
      }
   return Q_SUPER(&Modem_top_1);
   }

//----------SMS------------------------------------------------------------------
	 
	/*  void MakeSmsService(char* ptr,uint32_t size)
   {
   char temp[MAX_SMS_SIZE/2];
   for (uint32_t i=0;i<size;i++)ptr[i]=0;
   sprintf(temp,"SP=%u;",conf.server[SERVICE_SERVER].port);
   if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
   //-----------------------------------------------------
   sprintf(temp,"SIP=%s;",conf.server[SERVICE_SERVER].ip);  
   if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
   }*/

void MakeGprsSms(char* ptr,uint32_t size)
   {
   char temp[MAX_SMS_SIZE/2];
   for (uint32_t i=0;i<size;i++)ptr[i]=0;
   if (conf.secur.protocol==0)strcpy(temp,"QU;");
   else strcpy(temp,"FM;");
   if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
   sprintf(temp,"PORT=%u;",conf.server[MAIN_SERVER].port);
   if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
   //-----------------------------------------------------
   sprintf(temp,"IP=%s;",conf.server[MAIN_SERVER].ip);  
   if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
   //-------------------------------------------------------------
   sprintf(temp,"SIM=%u;",conf.sim.sim_num);
   if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
   sprintf(temp,"APN1=%s;",conf.gprs[0].apn);
   if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
   sprintf(temp,"APN2=%s;",conf.gprs[1].apn);
   if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
   sprintf(temp,"B1=%s;",conf.gprs[0].ussd_balance);
   if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
   sprintf(temp,"B2=%s;",conf.gprs[1].ussd_balance);
   if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
   }


void MakeNavSms(char* ptr,uint32_t size)
   {
   char temp[MAX_SMS_SIZE/2];
   for (uint32_t i=0;i<size;i++)ptr[i]=0;
   sprintf(temp,"NAV FILTER=%u;",conf_nav_filter.filter_on); 
   if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
   sprintf(temp,"STORE TIME=%u;",conf_nav_filter.time); 
   if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
   sprintf(temp,"STORE ANGLE=%u;",conf_nav_filter.angle); 
   if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
   sprintf(temp,"STORE DIST=%u;",conf_nav_filter.dist); 
   if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
   }
	 
	 void MakeStatusSms(MOD *me,char* ptr,uint32_t size)
   {
   char temp[MAX_SMS_SIZE/2];
   for (uint32_t i=0;i<size;i++)ptr[i]=0;
   sprintf(temp,"RRT=%u;",conf.secur.regular_restart_timeout);
   if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
   sprintf(temp,"RRC=%u;",conf.log.regular_restart_counter);
   if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
	 sprintf(temp,"RCOUNT=%u;",conf.log.evt_counter.restart);
   if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
	 sprintf(temp,"UTC=%u;",conf.log.updTryCounter);
   if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
	 sprintf(temp,"UTT=%u;",conf.log.updTryTimeout);
   if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
	 sprintf(temp,"NMEA_ERR=%u;",conf.log.evt_counter.nmea_err);
   if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
	 if(SysInfo.nav_state==fix) strcpy(temp,"NAV_STAT=FIX;");
	 else if(SysInfo.nav_state==no_fix) strcpy(temp,"NAV_STAT=NO FIX;");
	 else strcpy(temp,"NAV_STAT=NO NMEA;");
	 if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
		 //-------------------------------------------------
		strcpy(temp,me->msm.imei);
		if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
   }
	 
	 void MakeModemSms(char* ptr,uint32_t size)
   {
   char temp[MAX_SMS_SIZE/2],temp2[10];
   for (uint32_t i=0;i<size;i++)ptr[i]=0;
   
	 strcpy(temp,"MOD_HISTORY=");
		for (int32_t i=MODEM_STATE_HISTORY_SIZE-1;i>=0;i--)
		 {
			 sprintf(temp2,"%d;",modem_state_history[i]);
			 strcat(temp,temp2);
		 }
		 if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
		 //------------------------------------------------
   }
	 
	 void MakePackSms(char* ptr,uint32_t size)
   {
   char temp[MAX_SMS_SIZE/2];
   for (uint32_t i=0;i<size;i++)ptr[i]=0;
   sprintf(temp,"SEND TIME=%u;",conf.packet_send.time); 
   if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
   sprintf(temp,"SEND DIST=%u;",conf.packet_send.dist); 
   if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
	  sprintf(temp,"SEND PRIO FILTER=%u;",conf.packet_send.prio_filter); 
   if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
   }
	 
	  void MakeMsgSms(char* ptr,uint32_t size)
   {
		  char temp[MAX_SMS_SIZE/2];
		 for (uint32_t i=0;i<size;i++)ptr[i]=0;
		  if(SysInfo.msg_ptr.fram!=NULL)
			{
       strcpy(temp,SysInfo.msg_ptr.fram); 
		   strcat(temp,";");
				if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
		   SysInfo.msg_ptr.fram=NULL;
			}
			if(SysInfo.msg_ptr.flash!=NULL)
			{
       strcpy(temp,SysInfo.msg_ptr.flash); 
		   strcat(temp,";");
			 if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
		   SysInfo.msg_ptr.flash=NULL;
			}
			if(SysInfo.msg_ptr.system !=NULL)
			{
       strcpy(temp,SysInfo.msg_ptr.system); 
		   strcat(temp,";");
			 if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
		   SysInfo.msg_ptr.system=NULL;
			}
			if(SysInfo.msg_ptr.restart !=NULL)
			{
       strcpy(temp,SysInfo.msg_ptr.restart); 
		   strcat(temp,";");
			 if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
		   SysInfo.msg_ptr.restart=NULL;
			}
   }
	 
void MakeImeiSms(MOD* me,char* ptr,uint32_t size)
   {
		 for (uint32_t i=0;i<size;i++)ptr[i]=0;
		 sprintf(ptr,"IMEI=%s;",me->msm.imei);
   }

void MakeVerSms(char* ptr,uint32_t size)
   {
   char temp[MAX_SMS_SIZE/2];
   for (uint32_t i=0;i<size;i++)ptr[i]=0;
   sprintf(temp,"FW VER=%u_%u;\r\n",FwVer[0],FwVer[1]);
   if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
   sprintf(temp,"FW VER NEW=%u_%u;",(uint8_t)(conf.secur.fw_ver_new>>8),(uint8_t)conf.secur.fw_ver_new);
   if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
   sprintf(temp,"PCB_VER=%u_%u;",(uint8_t)(PCB_VER>>8),(uint8_t)PCB_VER);
   if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
   }

void MakeRS485Sms(char* ptr,uint32_t size)
   {
   char temp[MAX_SMS_SIZE/2];
   for (uint32_t i=0;i<size;i++)ptr[i]=0;
   for (uint32_t i=0;i<MAX_RS485_SENSORS_COUNT;i++)
      {
      sprintf(temp,"%uRS485D=%u;",i+1,conf_rs485_sensor[i].device); 
      if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
			sprintf(temp,"A=%d;",conf_rs485_sensor[i].addr);
      if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
      } 
			sprintf(temp,"DN=%u;",conf.rs485.deltaN);
      if (strlen(temp)<size-strlen(ptr))strcat(ptr,temp);
   }


void ClearErrorCounters(MOD *me)
   {
		 memset((uint8_t*)&me->error_counter,0,sizeof(ErrorCounterType));
   }

	
	 
void SendMsmSignal(MOD *me, QSignal sig)
   {
   QEvt evt;   
   evt.sig=sig;
   ModemSM_dispatch(&me->msm, &evt); 
   } 


uint32_t XTDeviceAuthPacketMaker(MOD *me,uint8_t *ptr)
   {
   uint32_t pack_size=sizeof(XTDevPacketHeadType)+sizeof(QLoginType);
   ((XTDevPacketType*)ptr)->head.id=HEAD_PACK_AUTH;  
   ((XTDevPacketType*)ptr)->head.crc=0;
   ((XTDevPacketType*)ptr)->head.len=pack_size; 
   ((QLoginType*)&((XTDevPacketType*)ptr)->payload)->dev_name=TRACKER_DEV_NAME;
   ((QLoginType*)&((XTDevPacketType*)ptr)->payload)->password=(uint32_t)0;
   memcpy((uint8_t*)&((QLoginType*)&((XTDevPacketType*)ptr)->payload)->dev_id,(uint8_t*)&me->msm.imei,sizeof(((QLoginType*)&((XTDevPacketType*)ptr)->payload)->dev_id));
   memcpy((uint8_t*)&((QLoginType*)&((XTDevPacketType*)ptr)->payload)->sim_id,(uint8_t*)&me->msm.imsi,sizeof(((QLoginType*)&((XTDevPacketType*)ptr)->payload)->sim_id));
   ((XTDevPacketType*)ptr)->head.crc=MakeCRC16((uint8_t*)ptr,pack_size);    
   return pack_size;
   }

uint32_t FM2200AuthPacketMaker(MOD *me,uint8_t *ptr)
   {
   ((PacketFM2200AuthType*)ptr)->head=0;
   ((PacketFM2200AuthType*)ptr)->size=15;
   strcpy(((PacketFM2200AuthType*)ptr)->imei,me->msm.imei);
   return sizeof(PacketFM2200AuthType);
   }
	 
	 void ModemUart_off(void)
	 {
		  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, DISABLE);
	 }
	 
	  void ModemUart_on(void)
	 {
		  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	 }

#ifdef DEBUG_MODEM

void OutDebugModem( char const *dbg_msg)
   {
   QS_BEGIN(QS_MODEM, AO_Modem)                                 
   QS_STR(dbg_msg);                              
   QS_END()
   }   

void OutDebugModemSprintf1( char const *str,uint32_t val)
   {
   QS_BEGIN(QS_MODEM, AO_Modem)                                  
   QS_STR(str); 
   QS_U32(4, val);       
   QS_END()
   }
	 
	 void OutDebugModemSprintf2( char const *str1,char const *str2,uint32_t val1,uint32_t val2)
   {
   QS_BEGIN(QS_MODEM, AO_Modem)                                  
   QS_STR(str1); 
   QS_U32(4, val1);
   QS_STR(str2); 
   QS_U32(4, val2);        
   QS_END()
   }

#endif

void AddStateHistory(uint8_t state)
   {
   for (uint32_t i=0;i<MODEM_STATE_HISTORY_SIZE-1;i++)
      {
      modem_state_history[i]=modem_state_history[i+1];
      }
   modem_state_history[MODEM_STATE_HISTORY_SIZE-1]=state;
   }

void ClearStateHistory(void)
   {
   for (uint32_t i=0;i<MODEM_STATE_HISTORY_SIZE;i++)
      {
      modem_state_history[i]=0;
      }
   }
	

 void SelectSim(MOD * const me)
	 {
		 if(me->sim_error)
		 {
		  if(conf.sim.sim_num==1)
			{
				conf.sim.sim_num=2;
				gprs_ptr=&conf.gprs[1];
				SELECT_SIM2();  
			}
		  else
			{
				conf.sim.sim_num=1;
				gprs_ptr=&conf.gprs[0];
				SELECT_SIM1();  
			}
			  me->sim_error=0;
				conf.sim.save=1;
			   QACTIVE_POST(AO_Disk,&DiskSaveConfigEvt, AO_Modem);
	   }
		 else
		 {
			 if(conf.sim.sim_num==1)
			{
				gprs_ptr=&conf.gprs[0];
				 SELECT_SIM1();  
			}
		  else
			{
				gprs_ptr=&conf.gprs[1];
				 SELECT_SIM2();  
			}
		 }
	 }	 
	 


