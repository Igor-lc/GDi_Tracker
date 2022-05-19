

#include "stm32f10x.h"
#include "stdint.h"
#include "stdio.h"
#include "modem_sim900.h"
#include "modem.h"
#include "modem_signals.h"
#include "ModemRingBuf.h"
#include "config.h"
#include "ByteStream.h"
#include "modem_parser.h"
#include "Disk.h"


	 
#ifdef DEBUG_SIM900	 
static void OutDebugSim900( char const *dbg_msg);
static void OutDebugSim900Sprintf( char const *str,uint32_t val);
#else
#define OutDebugSim900(x) __nop()
#define OutDebugSim900Sprintf(x,y) __nop()
#endif
static int modem_dma_busy_flag=0;
static MsmAnswerType answer_evt={MSM_ANSWER_SIG}; 
static QEvt const ParserWaitingIpSendEvt = { MODEM_PARSER_WAITING_IP_SEND_SIG, 0U, 0U};
static void Modem_hw_init (void);
static void ModemTxPin_init(void);
static void ModemRxPin_init(void);
static QState OkHandler(SIM900 *me);
static QState TickHandler(SIM900 *me);
static QState ErrorHandler(SIM900 *me,AnswerEnum answer);
static QState ModemSM_initial(SIM900 *me, QEvent const *e);
static QState ModemSM_idle(SIM900 *me, QEvent const *e);//1
static QState ModemSM_power_on(SIM900 *me, QEvent const *e);//2
static QState ModemSM_power_off(SIM900 *me, QEvent const *e);//3
static QState ModemSM_power_on_with_power_key(SIM900 *me, QEvent const *e);//4
static QState ModemSM_waiting_status_ok(SIM900 *me, QEvent const *e);//5
static QState ModemSM_send_at(SIM900 *me, QEvent const *e);//6
static QState ModemSM_fix_baudrate(SIM900 *me, QEvent const *e) ;//7
static QState ModemSM_check_baudrate(SIM900 *me, QEvent const *e) ;//8
static QState ModemSM_echo_off(SIM900 *me, QEvent const *e);//9
static QState ModemSM_sim_det_off(SIM900 *me, QEvent const *e);//10
static QState ModemSM_get_imei(SIM900 *me,QEvent const *e);//11
static QState ModemSM_get_imsi(SIM900 *me,QEvent const *e);//12
static QState ModemSM_waiting_call_ready(SIM900 *me,QEvent const *e);//13
static QState ModemSM_sms_set_text_mode(SIM900 *me,QEvent const *e);//14
static QState ModemSM_sms_set_ind_mode(SIM900 *me,QEvent const *e);//15
static QState ModemSM_sms_set_show_mode_1(SIM900 *me,QEvent const *e);//16
static QState ModemSM_pause(SIM900 *me, QEvent const *e); //17
static QState ModemSM_gsm_reg(SIM900 *me, QEvent const *e);//18
static QState ModemSM_open_gprs_check_ipstat(SIM900 *me,QEvent const *e);//19
static QState ModemSM_gprs_attach(SIM900 *me,QEvent const *e);//20
static QState ModemSM_add_iphead_mode(SIM900 *me,QEvent const *e);//21
static QState ModemSM_set_manual_getdata_mode(SIM900 *me,QEvent const *e);//22
static QState ModemSM_set_apn(SIM900 *me,QEvent const *e);//23
static QState ModemSM_start_task(SIM900 *me,QEvent const *e); //24
static QState ModemSM_bring_up_gprs_con(SIM900 *me,QEvent const *e);//25
static QState ModemSM_get_local_ip_address(SIM900 *me,QEvent const *e); //26
static QState ModemSM_packet_request_to_send(SIM900 *me,QEvent const *e);//27
static QState ModemSM_packet_send(SIM900 *me,QEvent const *e);//28
static QState ModemSM_waiting_tcp_data(SIM900 *me,QEvent const *e);//29 
static QState ModemSM_get_tcp_data(SIM900 *me,QEvent const *e);//30
static QState ModemSM_check_sim(SIM900 *me,QEvent const *e); //34
static QState ModemSM_set_te_character(SIM900 *me,QEvent const *e) ;//35
static QState ModemSM_aon_on(SIM900 *me,QEvent const *e) ;//36
static QState ModemSM_set_message_storage(SIM900 *me,QEvent const *e);
static QState ModemSM_waiting_sms_ready(SIM900 *me,QEvent const *e);
static QState ModemSM_waiting_gprs_ready(SIM900 *me,QEvent const *e) ;
//static QState ModemSM_get_ver(SIM900 *me,QEvent const *e) ;


//---------------------State Transition Tables ---------------------------------------
static StateType const power_on_start_up_stt[] = 
{  //handler,                                                     err, ok
   {(QStateHandler) & ModemSM_power_on,               ExitIfError, 0,  1},//ok
   {(QStateHandler) & ModemSM_power_on_with_power_key,ExitIfError, 0, 10},//ok
   {(QStateHandler) & ModemSM_waiting_status_ok,      ExitIfError, 20, 15},//ok
   {(QStateHandler) & ModemSM_send_at,                ExitIfError, 50, 0},//ok
   {(QStateHandler) & ModemSM_check_baudrate,         NextIfError, 20, 0},//ok
   {(QStateHandler) & ModemSM_fix_baudrate,           ExitIfError, 20, 0},//ok
   {(QStateHandler) & ModemSM_echo_off,               ExitIfError, 20, 0},//ok
   {(QStateHandler) & ModemSM_sim_det_off,            ExitIfError, 20, 0},//ok
   {(QStateHandler) & ModemSM_get_imei,               ExitIfError, 20, 0},//ok
	 //{(QStateHandler) & ModemSM_get_ver,                ExitIfError, 0, 10},//ok
   {(QStateHandler) & ModemSM_check_sim,              ExitIfError, 100, 0},//ok
   {(QStateHandler) & ModemSM_waiting_call_ready,     ExitIfError,300, 0},//ok
	 {(QStateHandler) & ModemSM_waiting_sms_ready,      ExitIfError,100, 0},//ok
   {(QStateHandler) & ModemSM_get_imsi,               ExitIfError, 20, 0},//ok
   {(QStateHandler) & ModemSM_set_te_character,       ExitIfError, 10, 0},//ok
   {(QStateHandler) & ModemSM_sms_set_text_mode,      ExitIfError, 10, 0},//ok
   {(QStateHandler) & ModemSM_sms_set_ind_mode,       ExitIfError, 10, 0},//ok
   {(QStateHandler) & ModemSM_set_message_storage,    ExitIfError, 10, 0},//ok
   {(QStateHandler) & ModemSM_aon_on,                 ExitIfError, 10, 0},
   {NULL,ExitIfError,0,0}  
};

static StateType const hard_reset_stt[] = 
{                                                                //err, ok
   {(QStateHandler) & ModemSM_power_off,              ExitIfError, 0,  20},
   {(QStateHandler) & ModemSM_pause,                  ExitIfError, 0, 200},
   {NULL,ExitIfError,0,0}  
};

static StateType const open_gsm_stt[] = 
{                                                               //err, ok
   {(QStateHandler) & ModemSM_gsm_reg,               ExitIfError, 600,  0},
   {NULL,ExitIfError,0,0}  
};

static StateType const open_gprs_stt[] = 
{                                                                     //err, ok
   {(QStateHandler) & ModemSM_open_gprs_check_ipstat,       ExitIfError, 50,  0},
   {(QStateHandler) & ModemSM_waiting_gprs_ready,           ExitIfError,100,  0},
	 {(QStateHandler) & ModemSM_gprs_attach,                  ExitIfError,300,  0},
   {(QStateHandler) & ModemSM_add_iphead_mode,              ExitIfError, 10,  0},
   {(QStateHandler) & ModemSM_set_manual_getdata_mode,      ExitIfError, 10,  0},
   {(QStateHandler) & ModemSM_set_apn,                      ExitIfError, 10,  0},
   {(QStateHandler) & ModemSM_start_task,                   ExitIfError, 10,  0},
   {(QStateHandler) & ModemSM_bring_up_gprs_con,            ExitIfError,600,  0},
   {(QStateHandler) & ModemSM_get_local_ip_address,         ExitIfError,100,  0},
   {NULL,ExitIfError,0,0}  
};


static StateType const send_packet_stt[] = 
{                                                                     //err, ok
   {(QStateHandler) & ModemSM_packet_request_to_send,       ExitIfError, 100,0},
   {(QStateHandler) & ModemSM_packet_send,                  ExitIfError, 100,0},
   {(QStateHandler) & ModemSM_waiting_tcp_data,             ExitIfError, 100,0},
   {(QStateHandler) & ModemSM_get_tcp_data,                 ExitIfError,  50,0},
   {NULL,                                                   ExitIfError,   0,0}  
};

static StateType const get_tcp_data_stt[] = 
{                                                                     //err, ok
	// {(QStateHandler) & ModemSM_waiting_tcp_data,             ExitIfError,  0,30},
   {(QStateHandler) & ModemSM_get_tcp_data,                 ExitIfError,  10,0},//1 sec err
   {NULL,                                                   ExitIfError,   0,0}  
};

//-------------------END OF STT------------------------------------------------------

void ModemSM_ctor(SIM900 *me) 
   {
   QFsm_ctor(&me->super, (QStateHandler)&ModemSM_initial);
   }

QState ModemSM_initial(SIM900 *me, QEvent const *e) 
   {
   (void)e;
   Modem_hw_init();
   QS_FUN_DICTIONARY(&ModemSM_initial);
   QS_FUN_DICTIONARY(&ModemSM_idle);
   QS_FUN_DICTIONARY(&ModemSM_power_on);
   QS_FUN_DICTIONARY(&ModemSM_power_on_with_power_key);
   QS_FUN_DICTIONARY(&ModemSM_waiting_status_ok);
   QS_FUN_DICTIONARY(&ModemSM_send_at);
   QS_FUN_DICTIONARY(&ModemSM_check_baudrate);
   QS_FUN_DICTIONARY(&ModemSM_fix_baudrate);
   QS_FUN_DICTIONARY(&ModemSM_echo_off);
   QS_FUN_DICTIONARY(&ModemSM_sim_det_off);
   QS_FUN_DICTIONARY(&ModemSM_get_imei);
   QS_FUN_DICTIONARY(&ModemSM_waiting_call_ready);
   QS_FUN_DICTIONARY(&ModemSM_get_imsi);
   QS_FUN_DICTIONARY(&ModemSM_sms_set_text_mode);
   QS_FUN_DICTIONARY(&ModemSM_sms_set_ind_mode);
   QS_FUN_DICTIONARY(&ModemSM_sms_set_show_mode_1);
   QS_FUN_DICTIONARY(&ModemSM_power_off);
   QS_FUN_DICTIONARY(&ModemSM_pause);
   QS_FUN_DICTIONARY(&ModemSM_gsm_reg);
   QS_FUN_DICTIONARY(&ModemSM_open_gprs_check_ipstat);
   QS_FUN_DICTIONARY(&ModemSM_gprs_attach);
   QS_FUN_DICTIONARY(&ModemSM_add_iphead_mode);
   QS_FUN_DICTIONARY(&ModemSM_set_manual_getdata_mode);
   QS_FUN_DICTIONARY(&ModemSM_set_apn);
   QS_FUN_DICTIONARY(&ModemSM_start_task);
   QS_FUN_DICTIONARY(&ModemSM_bring_up_gprs_con);
   QS_FUN_DICTIONARY(&ModemSM_get_local_ip_address);
   QS_FUN_DICTIONARY(&ModemSM_packet_request_to_send);
   QS_FUN_DICTIONARY(&ModemSM_packet_send);
   QS_FUN_DICTIONARY(&ModemSM_waiting_tcp_data);
   QS_FUN_DICTIONARY(&ModemSM_get_tcp_data);
	 QS_FUN_DICTIONARY(&ModemSM_check_sim);
	 QS_FUN_DICTIONARY(&ModemSM_set_te_character);
	 QS_FUN_DICTIONARY(&ModemSM_aon_on);

   QS_SIG_DICTIONARY(TIC_100ms_SIG, me);
   return Q_TRAN(&ModemSM_idle);
   }



QState ModemSM_idle(SIM900 *me, QEvent const *e) 
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG: return Q_HANDLED();
      case Q_EXIT_SIG:  return Q_HANDLED();
      case MSM_POWER_ON_START_UP_SIG:
         {
            me->sst_ptr=power_on_start_up_stt;
            return Q_TRAN(*me->sst_ptr->handler);
         }   
      case MSM_HARD_RESET_SIG:
         {
            me->sst_ptr=hard_reset_stt;
            return Q_TRAN(*me->sst_ptr->handler);
         }
      case MSM_OPEN_GSM_SIG:
         {
            me->sst_ptr=open_gsm_stt;
            return Q_TRAN(*me->sst_ptr->handler);
         }
      case MSM_OPEN_GPRS_SIG:
         {
            me->sst_ptr=open_gprs_stt;
            return Q_TRAN(*me->sst_ptr->handler);
         }
			case MSM_SEND_PACKET_SIG:
				 {
					  me->sst_ptr=send_packet_stt;
            return Q_TRAN(*me->sst_ptr->handler);
				 }
			case MSM_GET_TCP_DATA_SIG:
				 {
					  me->sst_ptr=get_tcp_data_stt;
            return Q_TRAN(*me->sst_ptr->handler);
				 }
      }
   return Q_IGNORED();
   }

QState ModemSM_power_on(SIM900 *me, QEvent const *e) 
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            me->flag.call_ready=0;
					 // SelectSim(me);
            POWERKEY_UP(); 						
            MODEM_POWER_ON(); 
            ModemTxPin_init();
            ModemRxPin_init();      
            me->tick_100ms_counter=0;
         }
         return Q_HANDLED(); 
      case MODEM_SIM_PIN_SIG:
      case MODEM_SIM_PUK_SIG:
      case MODEM_PH_SIM_PIN_SIG:
      case MODEM_PH_SIM_PUK_SIG:
      case MODEM_SIM_PIN2_SIG:
      case MODEM_SIM_PUK2_SIG: return ErrorHandler(me,ANSWER_SIM_LOCK);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);
      case TIC_100ms_SIG: return TickHandler(me); 
      case Q_EXIT_SIG: return Q_HANDLED();          
      }
   return Q_IGNORED();
   }

QState ModemSM_power_off(SIM900 *me, QEvent const *e) 
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            ModemTxPin_deinit();
            ModemRxPin_deinit();
            POWERKEY_DOWN();
            MODEM_POWER_OFF();
            me->tick_100ms_counter=0;
            SysInfo.gsmq=0;
         }
         return Q_HANDLED(); 
      case TIC_100ms_SIG: return TickHandler(me);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);         
      case Q_EXIT_SIG: return Q_HANDLED();         
      }
   return Q_IGNORED();
   }

QState ModemSM_pause(SIM900 *me, QEvent const *e) 
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            me->tick_100ms_counter=0;
         }
         return Q_HANDLED(); 
      case TIC_100ms_SIG: return TickHandler(me); 
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);
      case Q_EXIT_SIG: return Q_HANDLED();         
      }
   return Q_IGNORED();
   }



QState ModemSM_power_on_with_power_key(SIM900 *me, QEvent const *e) 
   {
   QState ret=Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         { 
            POWERKEY_DOWN();   
            me->tick_100ms_counter=0;
         }
         return ret;
      case TIC_100ms_SIG: return TickHandler(me); 
      case MODEM_SIM_PIN_SIG:
      case MODEM_SIM_PUK_SIG:
      case MODEM_PH_SIM_PIN_SIG:
      case MODEM_PH_SIM_PUK_SIG:
      case MODEM_SIM_PIN2_SIG:
      case MODEM_SIM_PUK2_SIG: return ErrorHandler(me,ANSWER_SIM_LOCK);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);
      case Q_EXIT_SIG:
         {
            POWERKEY_UP(); 
					 __nop();
         }
         return ret;
      }
   return Q_IGNORED();
   }

QState ModemSM_waiting_status_ok(SIM900 *me, QEvent const *e) 
   {
   QState ret=Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {   
            me->tick_100ms_counter=0;
         }
         return ret; 
      case TIC_100ms_SIG: 
			{
				if(GET_MODEM_STATUS())
				{
					return OkHandler(me);
				}
			}
				return TickHandler(me);
      case MODEM_SIM_PIN_SIG:
      case MODEM_SIM_PUK_SIG:
      case MODEM_PH_SIM_PIN_SIG:
      case MODEM_PH_SIM_PUK_SIG:
      case MODEM_SIM_PIN2_SIG:
      case MODEM_SIM_PUK2_SIG: return ErrorHandler(me,ANSWER_SIM_LOCK);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);        
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return Q_IGNORED();
   }

QState ModemSM_send_at(SIM900 *me, QEvent const *e) 
   {
   QState ret=Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {   
            ModemSendCommand("AT\r\n");
            me->tick_100ms_counter=0;
            me->counter=0;
         }
         return ret; 
      case TIC_100ms_SIG:
         {         
            ret= TickHandler(me); 
            if ((ret==Q_HANDLED())&&(++me->counter==10))
               {
               me->counter=0;
               ModemSendCommand("AT\r\n");
               }
         }
         return ret;
      case MODEM_OK_SIG: return OkHandler(me);
      case MODEM_SIM_PIN_SIG:
      case MODEM_SIM_PUK_SIG:
      case MODEM_PH_SIM_PIN_SIG:
      case MODEM_PH_SIM_PUK_SIG:
      case MODEM_SIM_PIN2_SIG:
      case MODEM_SIM_PUK2_SIG: return ErrorHandler(me,ANSWER_SIM_LOCK);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);      
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return Q_IGNORED();
   }

QState ModemSM_check_baudrate(SIM900 *me, QEvent const *e) 
   {
   QState ret=Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {   
            ModemSendCommand("AT+IPR?\r\n");
            me->tick_100ms_counter=0;
            me->counter=0;
            me->flag.next_ok=0;
         }
         return ret; 
      case TIC_100ms_SIG:
         {         
            ret= TickHandler(me); 
            if ((ret==Q_HANDLED())&&(++me->counter==5))
               {
               me->counter=0;
               ModemSendCommand("AT+IPR?\r\n");
               }
         }
         return ret;
      case MODEM_BAUDRATE_115200_SIG:
         {
            me->flag.next_ok=1;
         }
         return ret;
      case MODEM_OK_SIG:
         {
            if (me->flag.next_ok==1)ret=OkHandler(me);
         }
         return ret;
       case MODEM_SIM_PIN_SIG:
      case MODEM_SIM_PUK_SIG:
      case MODEM_PH_SIM_PIN_SIG:
      case MODEM_PH_SIM_PUK_SIG:
      case MODEM_SIM_PIN2_SIG:
      case MODEM_SIM_PUK2_SIG: return ErrorHandler(me,ANSWER_SIM_LOCK);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return Q_IGNORED();
   }

QState ModemSM_fix_baudrate(SIM900 *me, QEvent const *e) 
   {
   QState ret=Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {   
            ModemSendCommand("AT+IPR=115200\r\n");
            me->tick_100ms_counter=0;
            me->counter=0;
         }
         return ret; 
      case TIC_100ms_SIG:
         {         
            ret= TickHandler(me); 
            if ((ret==Q_HANDLED())&&(++me->counter==5))
               {
               me->counter=0;
               ModemSendCommand("AT+IPR=115200\r\n");
               }
         }
         return ret;
      case MODEM_OK_SIG: return OkHandler(me);
      case MODEM_SIM_PIN_SIG:
      case MODEM_SIM_PUK_SIG:
      case MODEM_PH_SIM_PIN_SIG:
      case MODEM_PH_SIM_PUK_SIG:
      case MODEM_SIM_PIN2_SIG:
      case MODEM_SIM_PUK2_SIG: return ErrorHandler(me,ANSWER_SIM_LOCK);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);      
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return Q_IGNORED();
   }


QState ModemSM_echo_off(SIM900 *me, QEvent const *e) 
   {
   QState ret=Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {   
            me->done=ModemSendCommand("ATE0\r\n");
            me->tick_100ms_counter=0;
         }
         return ret; 
      case TIC_100ms_SIG:
         {         
            if ((ret=TickHandler(me))==Q_HANDLED()&&me->done==0)
               {
               me->done=ModemSendCommand("ATE0\r\n");
               }
         }
         return ret;
      case MODEM_OK_SIG: return OkHandler(me);
      case MODEM_SIM_PIN_SIG:
      case MODEM_SIM_PUK_SIG:
      case MODEM_PH_SIM_PIN_SIG:
      case MODEM_PH_SIM_PUK_SIG:
      case MODEM_SIM_PIN2_SIG:
      case MODEM_SIM_PUK2_SIG: return ErrorHandler(me,ANSWER_SIM_LOCK);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);   
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return Q_IGNORED();
   }


QState ModemSM_sim_det_off(SIM900 *me, QEvent const *e) 
   {
   QState ret=Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {   
            me->done= ModemSendCommand("AT+CSDT?\r\n");
            me->tick_100ms_counter=0;
         }
         return ret; 
      case TIC_100ms_SIG:
         {         
            if ((ret=TickHandler(me))==Q_HANDLED()&&me->done==0)
               {
               me->done= ModemSendCommand("AT+CSDT?\r\n");
               }
         }
         return ret;
      case MODEM_OK_SIG: return OkHandler(me); 
      case MODEM_ERROR_SIG: return ErrorHandler(me,ANSWER_ERROR);
      case MODEM_SIM_PIN_SIG:
      case MODEM_SIM_PUK_SIG:
      case MODEM_PH_SIM_PIN_SIG:
      case MODEM_PH_SIM_PUK_SIG:
      case MODEM_SIM_PIN2_SIG:
      case MODEM_SIM_PUK2_SIG: return ErrorHandler(me,ANSWER_SIM_LOCK);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return Q_IGNORED();
   }


QState ModemSM_get_imei(SIM900 *me,QEvent const *e) 
   {
   QState ret=Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {   
            me->done=ModemSendCommand("AT+GSN\r\n"); 
            me->tick_100ms_counter=0;
            me->counter=0;
            me->flag.next_ok=0;
         }
         return ret; 
      case TIC_100ms_SIG:
         {         
            if ((ret=TickHandler(me))==Q_HANDLED()&&me->done==0)
               {
               me->done=ModemSendCommand("AT+GSN\r\n"); 
               }
         }
         return ret;
      case MODEM_IMEI_IMSI_SIG:
         {
          //  memcpy(me->imei,((ParserImeiImsiMessageType*)e)->ptr,16);
					  DataPtrEvt *pe =(DataPtrEvt*)e;
					   memcpy(me->imei,pe->ptr,16);
            char print_buf[25];
            sprintf(print_buf,"IMEI=%s",me->imei);
            OutDebugSim900(print_buf);            
            me->flag.next_ok=1;
         }
         return ret;
      case MODEM_OK_SIG:
         {
            if (me->flag.next_ok==1)ret=OkHandler(me);
         }
         return ret;
      case MODEM_ERROR_SIG: return ErrorHandler(me,ANSWER_ERROR);
      case MODEM_SIM_PIN_SIG:
      case MODEM_SIM_PUK_SIG:
      case MODEM_PH_SIM_PIN_SIG:
      case MODEM_PH_SIM_PUK_SIG:
      case MODEM_SIM_PIN2_SIG:
      case MODEM_SIM_PUK2_SIG: return ErrorHandler(me,ANSWER_SIM_LOCK);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return Q_IGNORED();
   }
	 
	 /*QState ModemSM_get_ver(SIM900 *me,QEvent const *e) 
   {
   QState ret=Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {   
            me->done=ModemSendCommand("AT+GMR\r\n"); 
            me->tick_100ms_counter=0;
         }
         return ret; 
				 
			case TIC_100ms_SIG: return TickHandler(me); 
     
      case MODEM_ERROR_SIG: return ErrorHandler(me,ANSWER_ERROR);
      case MODEM_SIM_PIN_SIG:
      case MODEM_SIM_PUK_SIG:
      case MODEM_PH_SIM_PIN_SIG:
      case MODEM_PH_SIM_PUK_SIG:
      case MODEM_SIM_PIN2_SIG:
      case MODEM_SIM_PUK2_SIG: return ErrorHandler(me,ANSWER_SIM_LOCK);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return Q_IGNORED();
   }
*/


QState ModemSM_waiting_call_ready(SIM900 *me,QEvent const *e) 
   {
   QState ret=Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {   
            me->tick_100ms_counter=0;
         }
         return ret; 
      case TIC_100ms_SIG: return TickHandler(me); 
      case MODEM_CALL_READY_SIG: return OkHandler(me);
      case MODEM_SIM_PIN_SIG:
      case MODEM_SIM_PUK_SIG:
      case MODEM_PH_SIM_PIN_SIG:
      case MODEM_PH_SIM_PUK_SIG:
      case MODEM_SIM_PIN2_SIG:
      case MODEM_SIM_PUK2_SIG: return ErrorHandler(me,ANSWER_SIM_LOCK);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return Q_IGNORED();
   }
	 
	 QState ModemSM_waiting_sms_ready(SIM900 *me,QEvent const *e) 
   {
   QState ret=Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {   
            me->tick_100ms_counter=0;
         }
         return ret; 
      case TIC_100ms_SIG: return TickHandler(me); 
      case MODEM_SMS_READY_SIG: return OkHandler(me);
      case MODEM_SIM_PIN_SIG:
      case MODEM_SIM_PUK_SIG:
      case MODEM_PH_SIM_PIN_SIG:
      case MODEM_PH_SIM_PUK_SIG:
      case MODEM_SIM_PIN2_SIG:
      case MODEM_SIM_PUK2_SIG: return ErrorHandler(me,ANSWER_SIM_LOCK);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return Q_IGNORED();
   }


QState ModemSM_get_imsi(SIM900 *me,QEvent const *e) 
   {
   QState ret=Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {   
            me->done= ModemSendCommand("AT+CIMI\r\n");
            me->tick_100ms_counter=0;
            me->counter=0;
            me->flag.next_ok=0;
         }
         return ret; 
      case TIC_100ms_SIG:
         {
            if ((ret=TickHandler(me))==Q_HANDLED()&&me->done==0)
               {
               me->done= ModemSendCommand("AT+CIMI\r\n");
               }
         }
         return ret;
      case MODEM_IMEI_IMSI_SIG:
         {
           // memcpy(me->imsi,((ParserImeiImsiMessageType*)e)->ptr,16);
					  DataPtrEvt *pe =(DataPtrEvt*)e;
					  memcpy(me->imsi,pe->ptr,19);
            char print_buf[30];
            sprintf(print_buf,"IMSI=%s",me->imsi);
            OutDebugSim900(print_buf);            
            me->flag.next_ok=1;
         }
         return ret;
      case MODEM_OK_SIG:
         {
            if (me->flag.next_ok==1)ret=OkHandler(me);
         }
         return ret;
      case MODEM_ERROR_SIG: return ErrorHandler(me,ANSWER_ERROR);
      case MODEM_SIM_PIN_SIG:
      case MODEM_SIM_PUK_SIG:
      case MODEM_PH_SIM_PIN_SIG:
      case MODEM_PH_SIM_PUK_SIG:
      case MODEM_SIM_PIN2_SIG:
      case MODEM_SIM_PUK2_SIG: return ErrorHandler(me,ANSWER_SIM_LOCK);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return Q_IGNORED();
   }
	 
	 QState ModemSM_check_sim(SIM900 *me,QEvent const *e) 
   {
   QState ret=Q_HANDLED();
		static AnswerEnum answer=ANSWER_ERROR;
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {   
            me->done= ModemSendCommand("AT+CPIN?\r\n");
            me->tick_100ms_counter=0;
            me->counter=0;
            me->flag.next_ok=0;
					  me->flag.exit_error=0;
         }
         return ret; 
      case TIC_100ms_SIG:
         {
					 ret=TickHandler(me);
            //if ((ret=TickHandler(me))==Q_HANDLED()&&me->done==0)
					 if (ret==Q_HANDLED())
               {
                ModemSendCommand("AT+CPIN?\r\n");
               }
         }
         return ret;
      case MODEM_SIM_READY_SIG:
         {    
            OutDebugSim900("MODEM_SIM_READY_SIG");					 
            me->flag.next_ok=1;
         }
         return ret;
			 case MODEM_SIM_PIN2_SIG:
			 case MODEM_SIM_PUK2_SIG:
			 case MODEM_PH_SIM_PUK_SIG:
			 case MODEM_PH_SIM_PIN_SIG:
			 case MODEM_SIM_PIN_SIG:
			 case MODEM_SIM_PUK_SIG:
					  {  
							 OutDebugSim900("ANSWER_SIM_LOCK");	
               answer=ANSWER_SIM_LOCK;							
               me->flag.exit_error=1;
           }
           return ret;
      case MODEM_OK_SIG:
         {
            if (me->flag.next_ok==1)
						{
							ret=OkHandler(me);
						}
					  else if (me->flag.exit_error==1)
						{
							ret= ErrorHandler(me,answer);
						}
         }
         return ret;
      case MODEM_ERROR_SIG: 
			{
				OutDebugSim900("ERROR_SIG");
			}
				return ErrorHandler(me,ANSWER_ERROR);
      case MODEM_SIM_NOT_READY_SIG: 
			{
				OutDebugSim900("SIM_NOT_READY_SIG");
			}
        return ErrorHandler(me,ANSWER_SIM_ERROR);				 
      case MODEM_SIM_NOT_INSERTED_SIG: 
					{
				OutDebugSim900("MODEM_SIM_NOT_INSERTED_SIG");
			    }
				return ErrorHandler(me,ANSWER_SIM_ERROR);
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return Q_IGNORED();
   }




QState ModemSM_sms_set_text_mode(SIM900 *me,QEvent const *e) 
   {
   QState ret=Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {   
            me->done=ModemSendCommand("AT+CMGF=1\r\n");
            me->tick_100ms_counter=0;
            me->counter=0;
         }
         return ret; 
      case TIC_100ms_SIG:
         {
            if ((ret=TickHandler(me))==Q_HANDLED()&&me->done==0)
               {
               me->done=ModemSendCommand("AT+CMGF=1\r\n");
               }
         }
         return ret;
      case MODEM_OK_SIG: return OkHandler(me); 
      case MODEM_ERROR_SIG: return ErrorHandler(me,ANSWER_ERROR);
     case MODEM_SIM_PIN_SIG:
      case MODEM_SIM_PUK_SIG:
      case MODEM_PH_SIM_PIN_SIG:
      case MODEM_PH_SIM_PUK_SIG:
      case MODEM_SIM_PIN2_SIG:
      case MODEM_SIM_PUK2_SIG: return ErrorHandler(me,ANSWER_SIM_LOCK);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return Q_IGNORED();
   }


QState ModemSM_sms_set_ind_mode(SIM900 *me,QEvent const *e) 
   {
   QState ret=Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {   
					 
					  // me->done=ModemSendCommand( "AT+CNMI=1,2,2,1,0\r\n");
            me->done=ModemSendCommand("AT+CNMI=2,1\r\n");
            me->tick_100ms_counter=0;
            me->counter=0;
         }
         return ret; 
      case TIC_100ms_SIG:
         {
            if ((ret=TickHandler(me))==Q_HANDLED()&&me->done==0)
               {
								  //me->done=ModemSendCommand( "AT+CNMI=1,2,2,1,0\r\n");
               me->done=ModemSendCommand("AT+CNMI=2,1\r\n");
               }
         }
         return ret;
      case MODEM_OK_SIG: return OkHandler(me); 
      case MODEM_ERROR_SIG: return ErrorHandler(me,ANSWER_ERROR);
      case MODEM_SIM_PIN_SIG:
      case MODEM_SIM_PUK_SIG:
      case MODEM_PH_SIM_PIN_SIG:
      case MODEM_PH_SIM_PUK_SIG:
      case MODEM_SIM_PIN2_SIG:
      case MODEM_SIM_PUK2_SIG: return ErrorHandler(me,ANSWER_SIM_LOCK);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return Q_IGNORED();
   }


QState ModemSM_sms_set_show_mode_1(SIM900 *me,QEvent const *e) 
   {
   QState ret=Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {   
            me->done=ModemSendCommand("AT+CSDH=1\r\n");
            me->tick_100ms_counter=0;
            me->counter=0;
         }
         return ret; 
      case TIC_100ms_SIG:
         {
            if ((ret=TickHandler(me))==Q_HANDLED()&&me->done==0)
               {
               me->done=ModemSendCommand("AT+CSDH=1\r\n");
               }
         }
         return ret;
      case MODEM_OK_SIG: return OkHandler(me); 
      case MODEM_ERROR_SIG: return ErrorHandler(me,ANSWER_ERROR);
      case MODEM_SIM_PIN_SIG:
      case MODEM_SIM_PUK_SIG:
      case MODEM_PH_SIM_PIN_SIG:
      case MODEM_PH_SIM_PUK_SIG:
      case MODEM_SIM_PIN2_SIG:
      case MODEM_SIM_PUK2_SIG: return ErrorHandler(me,ANSWER_SIM_LOCK);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return Q_IGNORED();
   }
	 
	 
	 QState ModemSM_set_te_character(SIM900 *me,QEvent const *e) 
   {
   QState ret=Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {   
            me->done=ModemSendCommand("AT+CSCS=\"IRA\"\r\n");
            me->tick_100ms_counter=0;
            me->counter=0;
         }
         return ret; 
      case TIC_100ms_SIG:
         {
            if ((ret=TickHandler(me))==Q_HANDLED()&&me->done==0)
               {
                me->done=ModemSendCommand("AT+CSCS=\"IRA\"\r\n");
               }
         }
         return ret;
      case MODEM_OK_SIG: return OkHandler(me); 
      case MODEM_ERROR_SIG: return ErrorHandler(me,ANSWER_ERROR);
      case MODEM_SIM_PIN_SIG:
      case MODEM_SIM_PUK_SIG:
      case MODEM_PH_SIM_PIN_SIG:
      case MODEM_PH_SIM_PUK_SIG:
      case MODEM_SIM_PIN2_SIG:
      case MODEM_SIM_PUK2_SIG: return ErrorHandler(me,ANSWER_SIM_LOCK);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return Q_IGNORED();
   }
	 
	  QState ModemSM_set_message_storage(SIM900 *me,QEvent const *e) 
   {
   QState ret=Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {   
             me->done=ModemSendCommand("AT+CPMS=\"SM\",\"SM\",\"SM\"\r\n");
            me->tick_100ms_counter=0;
            me->counter=0;
         }
         return ret; 
      case TIC_100ms_SIG:
         {
            if ((ret=TickHandler(me))==Q_HANDLED()&&me->done==0)
               {
                me->done=ModemSendCommand("AT+CPMS=\"SM\",\"SM\",\"SM\"\r\n");
               }
         }
         return ret;
      case MODEM_OK_SIG: return OkHandler(me); 
      case MODEM_ERROR_SIG: return ErrorHandler(me,ANSWER_ERROR);
     case MODEM_SIM_PIN_SIG:
      case MODEM_SIM_PUK_SIG:
      case MODEM_PH_SIM_PIN_SIG:
      case MODEM_PH_SIM_PUK_SIG:
      case MODEM_SIM_PIN2_SIG:
      case MODEM_SIM_PUK2_SIG: return ErrorHandler(me,ANSWER_SIM_LOCK);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return Q_IGNORED();
   }
	 
	
	 
	 
	 
	  QState ModemSM_aon_on(SIM900 *me,QEvent const *e) 
   {
   QState ret=Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {   
            me->done=ModemSendCommand("AT+CLIP=1\r\n");
            me->tick_100ms_counter=0;
            me->counter=0;
         }
         return ret; 
      case TIC_100ms_SIG:
         {
            if ((ret=TickHandler(me))==Q_HANDLED()&&me->done==0)
               {
                me->done=ModemSendCommand("AT+CLIP=1\r\n");
               }
         }
         return ret;
      case MODEM_OK_SIG: return OkHandler(me); 
      case MODEM_ERROR_SIG: return ErrorHandler(me,ANSWER_ERROR);
      case MODEM_SIM_PIN_SIG:
      case MODEM_SIM_PUK_SIG:
      case MODEM_PH_SIM_PIN_SIG:
      case MODEM_PH_SIM_PUK_SIG:
      case MODEM_SIM_PIN2_SIG:
      case MODEM_SIM_PUK2_SIG: return ErrorHandler(me,ANSWER_SIM_LOCK);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return Q_IGNORED();
   }
	 
	 

	
	 
	 

QState ModemSM_gsm_reg(SIM900 *me,QEvent const *e) 
   {
   static uint8_t reg00_counter,reg02_counter,reg03_counter,reg04_counter;
   QState ret=Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         { 
            reg00_counter=reg02_counter=reg03_counter=reg04_counter=0;
            me->tick_100ms_counter=0;
            me->counter=0;
            me->flag.roaming=0;
            me->flag.next_ok=0;
         }
         return ret;
      case TIC_100ms_SIG:
         {
            ret= TickHandler(me); 
            if ((ret==Q_HANDLED())&&(++me->counter==10))
               {
               me->counter=0;
               ModemSendCommand("AT+CREG?\r\n");
               }
         }
         return ret;
      case MODEM_CREG_NOT_REGISTERED_NO_OPERATOR_SEARCHING_SIG://CREG00
         {
					 GsmState=gsm_registration;
            if (++reg00_counter==5)//2x5=10 sec
               {
               ret= ErrorHandler(me,ANSWER_ERROR);
               }
         }
         return ret;
      case MODEM_CREG_REGISTERED_HOME_NETWORK_SIG://CREG01
         {
           GsmState=gsm_registered_in_home;
            me->flag.next_ok=1;
         }
         return ret ;
      case MODEM_CREG_NOT_REGISTERED_BUT_OPERATOR_SEARCHING_SIG://CREG02
         {
             GsmState=gsm_registration;
            if (++reg02_counter==30)//2x30=60 sec
               {
               ret= ErrorHandler(me,ANSWER_ERROR);
               }
         }
         return ret ;
      case MODEM_CREG_REGISTRATION_DENIED_SIG://CREG03
         {
             GsmState=gsm_registration;
            if (++reg03_counter==5)//2x5=10 sec
               {
               ret= ErrorHandler(me,ANSWER_ERROR);
               }
         }
         return ret ;
      case MODEM_CREG_UNKNOWN_ANSWER_SIG://CREG04
         {
             GsmState=gsm_registration;
            if (++reg04_counter==3)//3x2=6 sec
               {
               ret= ErrorHandler(me,ANSWER_ERROR);
               }
         }
         return ret;
      case MODEM_CREG_REGISTERED_ROAMING_SIG://CREG05
         {
            GsmState=gsm_registered_in_roaming;
            me->flag.roaming=1;
            me->flag.next_ok=1;
         }
         return ret ;
      case MODEM_OK_SIG:
         {
            if (me->flag.next_ok==1)ret=OkHandler(me);
         }
         return ret;
      case MODEM_ERROR_SIG: return ErrorHandler(me,ANSWER_ERROR);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);
      case Q_EXIT_SIG:  return Q_HANDLED();
      }
   return Q_IGNORED();
   } 

QState ModemSM_open_gprs_check_ipstat(SIM900 *me,QEvent const *e) 
   {
   QState ret=Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            me->tick_100ms_counter=0;
            me->flag.next_ok=0;
            me->flag.exit_ok=0;
            me->done= ModemSendCommand("AT+CIPSTATUS\r\n");
         }
         return ret ;
      case TIC_100ms_SIG:
         {
            if ((ret=TickHandler(me))==Q_HANDLED()&&me->done==0)
               {
               me->done= ModemSendCommand("AT+CIPSTATUS\r\n");
               }
         }
         return ret;
      case MODEM_STATE_TCP_CLOSED_SIG:
         {
            OutDebugSim900("STATE_TCP_CLOSED_SIG");
            answer_evt.answer=ANSWER_OK;
            QACTIVE_POST(AO_Modem, (QEvt*)&answer_evt, me);
            ret= Q_TRAN(&ModemSM_idle); 
         }
         return ret;
      case MODEM_STATE_IP_STATUS_SIG:
         {
            OutDebugSim900("STATE_IP_STATUS_SIG");
            answer_evt.answer=ANSWER_OK;
            QACTIVE_POST(AO_Modem, (QEvt*)&answer_evt, me);
            ret= Q_TRAN(&ModemSM_idle); 
         }
         return ret;
      case MODEM_STATE_IP_INITIAL_SIG:
         {
            OutDebugSim900("STATE_IP_INITIAL_SIG");
            ret=OkHandler(me);
         }
         return ret ;
      case MODEM_STATE_PDP_DEACT_SIG:
         {
            OutDebugSim900("STATE_PDP_DEACT_SIG");
            ret=OkHandler(me);
         }
         return ret ;
      case MODEM_OK_SIG:  return Q_HANDLED();
      case MODEM_ERROR_SIG: return ErrorHandler(me,ANSWER_ERROR);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return Q_IGNORED();
   }


QState ModemSM_waiting_gprs_ready(SIM900 *me,QEvent const *e) 
   {
   QState ret=Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            me->tick_100ms_counter=0;
					  me->counter=0;
            ModemSendCommand("AT+CGATT?\r\n");
         }
         return ret ;
      case TIC_100ms_SIG:
         {
            if ((ret=TickHandler(me))==Q_HANDLED() && ++me->counter==10)
               {
                me->counter=0;
								 ModemSendCommand("AT+CGATT?\r\n");
               }
							 
         }
         return ret;
      case MODEM_GPRS_ATTACHED_SIG:
         {
            OutDebugSim900("GPRS_ATTACHED_SIG");
            ret=OkHandler(me);
         }
         return ret ;
      case MODEM_OK_SIG: return Q_HANDLED() ;
      case MODEM_ERROR_SIG: return ErrorHandler(me,ANSWER_ERROR);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);
      case Q_EXIT_SIG: return Q_HANDLED() ;
      }
   return Q_IGNORED();
   }

QState ModemSM_gprs_attach(SIM900 *me,QEvent const *e)    
   {
   QState ret=Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            me->tick_100ms_counter=0;                              
            me->done=ModemSendCommand("AT+CGATT=1\r\n");
         }
         return ret;
      case TIC_100ms_SIG:
         {
            if ((ret=TickHandler(me))==Q_HANDLED()&&me->done==0)
               {
               me->done=ModemSendCommand("AT+CGATT=1\r\n");
               }
         }
         return ret;
      case MODEM_OK_SIG: return OkHandler(me);
      case MODEM_ERROR_SIG: return ErrorHandler(me,ANSWER_ERROR);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return Q_IGNORED();
   }

QState ModemSM_add_iphead_mode(SIM900 *me,QEvent const *e)    
   {
   QState ret=Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            me->tick_100ms_counter=0;                              
            me->done=ModemSendCommand("AT+CIPHEAD=1\r\n");
         }
         return ret;
      case TIC_100ms_SIG:
         {
            if ((ret=TickHandler(me))==Q_HANDLED()&&me->done==0)
               {
               me->done=ModemSendCommand("AT+CIPHEAD=1\r\n");
               }
         }
         return ret;
      case MODEM_OK_SIG: return OkHandler(me);
      case MODEM_ERROR_SIG: return ErrorHandler(me,ANSWER_ERROR);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return Q_IGNORED();
   }

QState ModemSM_set_manual_getdata_mode(SIM900 *me,QEvent const *e)    
   {
   QState ret=Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            me->tick_100ms_counter=0;                              
            me->done=ModemSendCommand("AT+CIPRXGET=1\r\n");
         }
         return ret;
      case TIC_100ms_SIG:
         {
            if ((ret=TickHandler(me))==Q_HANDLED()&&me->done==0)
               {
               me->done=ModemSendCommand("AT+CIPRXGET=1\r\n");
               }
         }
         return ret;
      case MODEM_OK_SIG: return OkHandler(me);
      case MODEM_ERROR_SIG:  return ErrorHandler(me,ANSWER_ERROR);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return Q_IGNORED();
   }

QState ModemSM_set_apn(SIM900 *me,QEvent const *e)    
   {
   QState ret=Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            me->tick_100ms_counter=0;  
            sprintf(me->command_buf,"AT+CIPCSGP=1,\"%s\",\"%s\",\"%s\"\r\n",gprs_ptr->apn,gprs_ptr->uname,gprs_ptr->psw);  
            me->done=ModemSendCommand(me->command_buf);
            OutDebugSim900(me->command_buf);
         }
         return ret;
      case TIC_100ms_SIG:
         {
            if ((ret=TickHandler(me))==Q_HANDLED()&&me->done==0)
               {
               me->done=ModemSendCommand(me->command_buf);
               }
         }
         return ret;
      case MODEM_OK_SIG: return OkHandler(me);
      case MODEM_ERROR_SIG: return ErrorHandler(me,ANSWER_ERROR);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return Q_IGNORED();
   }

QState ModemSM_start_task(SIM900 *me,QEvent const *e)    
   {
   QState ret=Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            me->tick_100ms_counter=0;                              
            me->done=ModemSendCommand("AT+CSTT\r\n");
         }
         return ret;
      case TIC_100ms_SIG:
         {
            if ((ret=TickHandler(me))==Q_HANDLED()&&me->done==0)
               {
               me->done=ModemSendCommand("AT+CSTT\r\n");
               }
         }
         return ret; 
      case MODEM_OK_SIG:  return OkHandler(me);
      case MODEM_ERROR_SIG: return ErrorHandler(me,ANSWER_ERROR);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return Q_IGNORED();
   }

QState ModemSM_bring_up_gprs_con(SIM900 *me,QEvent const *e)   
   {
   QState ret=Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            me->tick_100ms_counter=0;                              
            me->done=ModemSendCommand("AT+CIICR\r\n");
         }
         return ret;
      case TIC_100ms_SIG:
         {
            if ((ret=TickHandler(me))==Q_HANDLED()&&me->done==0)
               {
               me->done=ModemSendCommand("AT+CIICR\r\n");
               }
         }
         return ret; 
      case MODEM_OK_SIG: return OkHandler(me);
      case MODEM_ERROR_SIG: return ErrorHandler(me,ANSWER_ERROR);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);
      case Q_EXIT_SIG: return Q_HANDLED();   
      }
   return Q_IGNORED();
   }

QState ModemSM_get_local_ip_address(SIM900 *me,QEvent const *e)    
   {
   QState ret=Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            me->tick_100ms_counter=0;                              
            me->done=ModemSendCommand("AT+CIFSR\r\n");
            me->flag.next_ok=0;
         }
         return ret;
      case TIC_100ms_SIG:
         {
            if ((ret=TickHandler(me))==Q_HANDLED()&&me->done==0)
               {
               me->done=ModemSendCommand("AT+CIFSR\r\n");
               }
         }
         return ret; 
      case MODEM_LOCAL_IP_SIG:
         {
            ret=OkHandler(me);
         }
         return ret;
      case MODEM_OK_SIG: return Q_HANDLED();
      case MODEM_ERROR_SIG: return ErrorHandler(me,ANSWER_ERROR);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return Q_IGNORED();
   }


/*QState ModemSM_proper_open_connection(SIM900 *me,QEvent const *e) 
   {
  static int packet_ok=0;
   QState ret=Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            sprintf(me->command_buf,"AT+CIPSTART=\"TCP\",\"%s\",\"%u\"\r\n",me->server_ptr->ip ,me->server_ptr->port);               
            packet_ok= PacketFM2200AuthMaker(me,adp_buf,ADP_BUF_SIZE);
            me->tick_100ms_counter = 0;
         }
         return ret ;
      case TIC_100ms_SIG:
         {
            if (packet_ok)
						{
							ret= TickHandler(me);
						}
            else 
						{
							ret= ErrorHandler(me,ANSWER_ERROR);
						}
         }
         return ret;
      case MODEM_OK_SIG: return Q_HANDLED() ;
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);
      case Q_EXIT_SIG: return Q_HANDLED() ;
      }
   return Q_IGNORED();
   }*/

/*QState ModemSM_open_connection(SIM900 *me,QEvent const *e) 
   {
   QState ret= Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         { 
            me->tick_100ms_counter=0;
					  sprintf(me->command_buf,"AT+CIPSTART=\"TCP\",\"%s\",\"%u\"\r\n",me->server_ptr->ip ,me->server_ptr->port); 
            me->done=ModemSendCommand(me->command_buf);
            OutDebugSim900(me->command_buf); 
         }
         return Q_HANDLED();
      case TIC_100ms_SIG:
         {
					 if(me->tick_100ms_counter%10==0)OutDebugSim900Sprintf("sec_counter=",me->tick_100ms_counter/10);
           if ((ret=TickHandler(me))==Q_HANDLED()&&me->done==0)
               {
               me->done=ModemSendCommand(me->command_buf);
               }
         }
         return ret;
      case MODEM_PDP_DEACT_SIG: 
			{
				 OutDebugSim900("PDP_DEACT_SIG");  
			}
				return ErrorHandler(me,ANSWER_ERROR);
      case MODEM_CLOSED_SIG: 
			{
				 OutDebugSim900("CLOSED_SIG");  
			}
				return ErrorHandler(me,ANSWER_ERROR);
      case MODEM_CONNECT_FAIL_SIG:
			{
				 OutDebugSim900("CONNECT_FAIL_SIG");  
			}
				return ErrorHandler(me,ANSWER_ERROR);
      case MODEM_ALREADY_CONNECT_SIG: 
			{
				 OutDebugSim900("ALREADY_CONNECT_SIG");  
			}
				return ErrorHandler(me,ANSWER_ERROR);
      case MODEM_CONNECT_OK_SIG:
         {
            OutDebugSim900("CONNECT_OK_SIG");  
         }
         return  OkHandler(me);
      case MODEM_OK_SIG: return Q_HANDLED() ;
      case MODEM_ERROR_SIG: 
			{
				OutDebugSim900("ERROR_SIG");  
			}
				return ErrorHandler(me,ANSWER_ERROR);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);
      case Q_EXIT_SIG: return Q_HANDLED() ;
      }
   return Q_IGNORED();
   }*/


QState ModemSM_packet_request_to_send(SIM900 *me,QEvent const *e) 
   {
   QState ret= Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         { 
            me->tick_100ms_counter=0;
					  QACTIVE_POST(AO_ModemParser, &ParserWaitingIpSendEvt, me);		
            sprintf(me->command_buf, "AT+CIPSEND=%u\r\n",me->send_packet.size);
         }
         return Q_HANDLED() ;
      case TIC_100ms_SIG:
         {
					  if(me->tick_100ms_counter%10==0)OutDebugSim900Sprintf("sec_counter=",me->tick_100ms_counter/10);
            if ((ret=TickHandler(me))==Q_HANDLED()&&me->tick_100ms_counter==2)
               {
                me->done=ModemSendCommand(me->command_buf);
								 OutDebugSim900(me->command_buf);
               }
         }
         return ret;
      case MODEM_IPSEND_SIG:
         {
            OutDebugSim900("IPSEND_EVT\n\r");
            ret= OkHandler(me);
         }
         return ret;
      case MODEM_ERROR_SIG:
      case MODEM_CLOSED_SIG:           return ErrorHandler(me,ANSWER_ERROR);
      case MODEM_OK_SIG:               return Q_HANDLED() ;
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);
      case Q_EXIT_SIG:                 return Q_HANDLED() ;
      }
   return Q_IGNORED();
   }

uint8_t *debug_ptr2;
	 uint32_t debug_size;
QState ModemSM_packet_send(SIM900 *me,QEvent const *e) 
   {
   QState ret= Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         { 
            me->tick_100ms_counter=0;
					 debug_ptr2=me->send_packet.ptr;
					 debug_size=me->send_packet.size;
            me->done=ModemSendData(me->send_packet.ptr,me->send_packet.size);
         }
         return Q_HANDLED() ;
      case TIC_100ms_SIG:
         {
            if ((ret=TickHandler(me))==Q_HANDLED()&&me->done==0)
               {
               me->done=ModemSendData(me->send_packet.ptr,me->send_packet.size);
               }
         }
         return ret;
      case MODEM_SEND_OK_SIG:
         {
            OutDebugSim900("SEND_OK_EVT");
            ret=  OkHandler(me);
         }
         return  ret;
      case MODEM_ERROR_SIG:
      case MODEM_CLOSED_SIG:  return ErrorHandler(me,ANSWER_ERROR);
      case MODEM_OK_SIG: return Q_HANDLED() ;
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);
      case Q_EXIT_SIG:
			{
				NOP();
			}
				return Q_HANDLED() ;
      }
   return Q_IGNORED();
   }


QState ModemSM_waiting_tcp_data(SIM900 *me,QEvent const *e) 
   {
   QState ret= Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         { 
            me->tick_100ms_counter=0;
         }
         return Q_HANDLED() ;
      case TIC_100ms_SIG: return TickHandler(me);
      case MODEM_TCP_DATA_RECEIVED_SIG:
         {
            OutDebugSim900("TCP_DATA_RECEIVED_SIG");
            ret=  OkHandler(me);
         }
         return ret ;
      case MODEM_ERROR_SIG:
      case MODEM_CLOSED_SIG:  return ErrorHandler(me,ANSWER_ERROR);
      case MODEM_OK_SIG: return Q_HANDLED() ;
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);
      case Q_EXIT_SIG: return Q_HANDLED() ;
      }
   return Q_IGNORED();
   }


QState ModemSM_get_tcp_data(SIM900 *me,QEvent const *e) 
   {
   QState ret= Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         { 
            me->tick_100ms_counter=0;
            me->server_data.ptr =NULL;
            me->server_data.size=0;
					  sprintf(me->command_buf,"AT+CIPRXGET=2,%u\r\n",1460);
					  me->done= ModemSendCommand(me->command_buf);
         }
         return Q_HANDLED() ;
     /* case TIC_100ms_SIG:
         {
            if (++me->tick_100ms_counter==50)
               {
               answer_evt.answer=ANSWER_SERVER_NOT_ANSWER;
               QACTIVE_POST(AO_Modem,(QEvt*)&answer_evt, me);
               ret= Q_TRAN(&ModemSM_idle); 
               }
            else if (me->done==0) me->done= ModemSendCommand(me->command_buf);
         }
         return ret;*/
			case TIC_100ms_SIG: return TickHandler(me);
      case MODEM_SERVER_DATA_SIG:
         {
            DataPtrEvt *pe=(DataPtrEvt*)e;
            me->server_data.ptr =pe->ptr;
            me->server_data.size=pe->data.size;
					  OutDebugSim900Sprintf("SERVER_DATA_SIG size=",me->server_data.size);
         }
         return Q_HANDLED() ;
      case MODEM_OK_SIG:
         {
            if ((me->server_data.ptr !=NULL)&&( me->server_data.size!=0)) ret= OkHandler(me);
            else   ret= ErrorHandler(me,ANSWER_ERROR);
         }
         return ret;
      case MODEM_ERROR_SIG:
      case MODEM_CLOSED_SIG:  return ErrorHandler(me,ANSWER_ERROR);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);
      case Q_EXIT_SIG: return Q_HANDLED() ;
      }
   return Q_IGNORED();
   } 
	 
	 
	 QState ModemSM_check_unread_data(SIM900 *me,QEvent const *e) 
   {
   QState ret= Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         { 
            me->tick_100ms_counter=0;
            me->server_data.ptr =NULL;
            me->server_data.size=0;
					  me->done= ModemSendCommand("AT+CIPRXGET=4\r\n");
           // me->done= ModemSendCommand("AT+CIPRXGET=2,1400\r\n");
         }
         return Q_HANDLED() ;
     /* case TIC_100ms_SIG:
         {
            if (++me->tick_100ms_counter==50)
               {
               answer_evt.answer=ANSWER_SERVER_NOT_ANSWER;
               QACTIVE_POST(AO_Modem,(QEvt*)&answer_evt, me);
               ret= Q_TRAN(&ModemSM_idle); 
               }
            else if (me->done==0) me->done= ModemSendCommand(me->command_buf);
         }
         return ret;*/
			case TIC_100ms_SIG: return TickHandler(me);
      case MODEM_SERVER_DATA_SIG:
         {
            DataPtrEvt *pe=(DataPtrEvt*)e;
            me->server_data.ptr =pe->ptr;
            me->server_data.size=pe->data.size;
					  OutDebugSim900Sprintf("SERVER_DATA_SIG size=",me->server_data.size);
         }
         return Q_HANDLED() ;
      case MODEM_OK_SIG:
         {
            if ((me->server_data.ptr !=NULL)&&( me->server_data.size!=0)) ret= OkHandler(me);
            else   ret= ErrorHandler(me,ANSWER_ERROR);
         }
         return ret;
      case MODEM_ERROR_SIG:
      case MODEM_CLOSED_SIG:  return ErrorHandler(me,ANSWER_ERROR);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);
      case Q_EXIT_SIG: return Q_HANDLED() ;
      }
   return Q_IGNORED();
   } 
	 
	 
	 
/*	 QState ModemSM_check_server_data_answer(SIM900 *me,QEvent const *e) 
   {
		 QState ret=Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         { 
            me->tick_100ms_counter=0;
         }
         return Q_HANDLED() ;
      case TIC_100ms_SIG:
         {
					 {
						 if (me->server_data.size==4)
						 {
							 uint32_t answer;
							 BufToLittleEndianInt(me->server_data.ptr,&answer);
							 OutDebugSim900Sprintf("SERVER ANSWER OK, packets received=",answer);
							 ret=  OkHandler(me);
						 }
						 else
						 {
							 OutDebugSim900("ERROR!!! UNKNOWN SERVER ANSWER");
                ret= ErrorHandler(me,ANSWER_ANKNOWN_SERVER_ANSWER);
						 }
					 }
         }
         return ret ;
      case MODEM_OK_SIG: return Q_HANDLED() ;
			case MODEM_ERROR_SIG:
      case MODEM_CLOSED_SIG: return ErrorHandler(me,ANSWER_ERROR);
      }
    return Q_IGNORED();
   } */


/*QState ModemSM_check_auth_answer(SIM900 *me,QEvent const *e) 
   {
   QState ret= Q_HANDLED();
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         { 
            me->tick_100ms_counter=0;
         }
         return Q_HANDLED() ;
      case TIC_100ms_SIG:
         {
            if (me->server_data.size==1)
               {
               uint8_t answer=*(me->server_data.ptr);
               switch (answer)
                  {
                  case 1:
                     {
                        OutDebugSim900("Server answer \"AUTHORIZATION OK\"\r\n"); 
                        ret= OkHandler(me);  
                     }
                     break;
                  default:
                     {
                        answer_evt.serverAnswer=answer;
                        answer_evt.answer=ANSWER_SERVER_ERROR_ANSWER;
                        QACTIVE_POST(AO_Modem,(QEvt*)&answer_evt, me);
                        ret= Q_TRAN(&ModemSM_idle); 
                     }
                     break;
                  }
               }
            else ret= ErrorHandler(me,ANSWER_ERROR);
         }
         return ret ;
      case MODEM_OK_SIG: return Q_HANDLED() ;
      case MODEM_ERROR_SIG:
      case MODEM_CLOSED_SIG: return ErrorHandler(me,ANSWER_ERROR);
      case MODEM_SIM_NOT_READY_SIG:
      case MODEM_SIM_NOT_INSERTED_SIG: return ErrorHandler(me,ANSWER_SIM_ERROR);
      case Q_EXIT_SIG: return Q_HANDLED() ;
      }
   return Q_IGNORED();
   } */

	 
	/* QState ModemSM_proper_send_data_packet(SIM900 *me,QEvent const *e) 
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            me->send_packet.ptr= me->data_packet.ptr;
            me->send_packet.size=me->data_packet.size;
            me->tick_100ms_counter = 0;
         }
         return Q_HANDLED() ;
			case TIC_100ms_SIG: return TickHandler(me);
      case MODEM_OK_SIG: return Q_HANDLED() ;
      }
     return Q_IGNORED();
   }*/



int ModemSendCommand(const char *command)
   {
   static char modem_command_tx_buf[MODEM_COMMAND_TX_BUF_SIZE];
   static u8 i;
   if (!*command)return 0;
   i=0;
   if (modem_dma_busy_flag==1) return 0;
   while ((i<MODEM_COMMAND_TX_BUF_SIZE)&&(*command))modem_command_tx_buf[i++]=*command++;
   DMA1_Channel4->CCR &= CCR_ENABLE_Reset; //turn  DMA ch off
   DMA1_Channel4->CNDTR = i;
   DMA1_Channel4->CMAR =(u32)modem_command_tx_buf; 
   modem_dma_busy_flag=1;
   DMA1_Channel4->CCR |= CCR_ENABLE_Set; //turn  DMA ch  on
   return 1;
   }

int ModemSendData(const uint8_t *data_ptr,uint16_t const size)
   {
   if (data_ptr==0 || modem_dma_busy_flag==1) return 0;
   DMA1_Channel4->CCR &= CCR_ENABLE_Reset;
   DMA1_Channel4->CNDTR = size;
   DMA1_Channel4->CMAR =(u32)data_ptr; 
   modem_dma_busy_flag=1;
   DMA1_Channel4->CCR |= CCR_ENABLE_Set; 
   return 1;
   }  




void  DMA1_Channel4_IRQHandler(void)
   {
   DMA_ClearITPendingBit(DMA1_IT_TC4);
   modem_dma_busy_flag=0;
   }

void USART1_IRQHandler (void)
   {
		 if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
			{
       uint8_t data=USART1->DR;
       ModemRingBuf_push(data);  
	     //USART_ClearITPendingBit(USART1, USART_IT_RXNE);
     }
		 else
		 {
 			 __nop();
		 }
 }

void ModemTxPin_init(void)
   {
   GPIO_InitTypeDef GPIO_InitStructure;

   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Pin = MODEM_TX_PIN;
   GPIO_Init(MODEM_UART_GPIO , &GPIO_InitStructure);
		
   }

void ModemRxPin_init(void)
   {
   GPIO_InitTypeDef GPIO_InitStructure;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Pin = MODEM_RX_PIN;
   GPIO_Init(MODEM_UART_GPIO , &GPIO_InitStructure);
   }


void ModemTxPin_deinit(void)
   {
   GPIO_InitTypeDef GPIO_InitStructure;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Pin = MODEM_TX_PIN;
   GPIO_Init(MODEM_UART_GPIO , &GPIO_InitStructure);
   MODEM_UART_GPIO->BRR = MODEM_TX_PIN;//pull down
   }


void ModemRxPin_deinit(void)
   {
   GPIO_InitTypeDef GPIO_InitStructure;

   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Pin = MODEM_RX_PIN;
   GPIO_Init(MODEM_UART_GPIO , &GPIO_InitStructure);
   MODEM_UART_GPIO->BRR = MODEM_RX_PIN;//pull down
   }

QState OkHandler(SIM900 *me)
   {
   if (me->sst_ptr->error_tran==ExitIfError)//next if ok
      {
      ++me->sst_ptr;
      if (me->sst_ptr->handler==NULL)
         {
         answer_evt.answer=ANSWER_OK;
         QACTIVE_POST(AO_Modem, (QEvt*)&answer_evt, me);
         return Q_TRAN(&ModemSM_idle);  
         }
      else return Q_TRAN(*me->sst_ptr->handler);   
      }
   else//jump
      {
      me->sst_ptr+=2;
      if (me->sst_ptr->handler==NULL)
         {
         answer_evt.answer=ANSWER_OK;
         QACTIVE_POST(AO_Modem, (QEvt*)&answer_evt, me);
         return Q_TRAN(&ModemSM_idle);  
         }
      else return Q_TRAN(*me->sst_ptr->handler);   
      }
   }


QState ErrorHandler(SIM900 *me,AnswerEnum answer)
   {
   answer_evt.answer=answer;
   QACTIVE_POST(AO_Modem,(QEvt*)&answer_evt, me);
   return Q_TRAN(&ModemSM_idle);  
   } 

 QState TickHandler(SIM900 *me)
   {
   ++me->tick_100ms_counter; 
   if ((me->sst_ptr->timeout_err)&&(me->tick_100ms_counter== me->sst_ptr->timeout_err))
      {
      if (me->sst_ptr->error_tran==ExitIfError)//if just exit
         {
         answer_evt.answer=ANSWER_ERROR;
         QACTIVE_POST(AO_Modem,(QEvt*)&answer_evt, me);
				 OutDebugSim900("TIMEOUT ERROR");
         return Q_TRAN(&ModemSM_idle);  
         }
      else //if transition to next
         {
         if ((++me->sst_ptr)->handler==NULL)
            {
            answer_evt.answer=ANSWER_ERROR;
            QACTIVE_POST(AO_Modem,(QEvt*)&answer_evt, me);
            return Q_TRAN(&ModemSM_idle);  
            }
         else return Q_TRAN(*me->sst_ptr->handler);
         }
      }
   if ((me->sst_ptr->timeout_ok)&&(me->tick_100ms_counter== me->sst_ptr->timeout_ok))
      {
      if (me->sst_ptr->error_tran==ExitIfError)//next if ok
         {
         ++me->sst_ptr;
         if ((me->sst_ptr)->handler==NULL)
            {
            answer_evt.answer=ANSWER_OK;
            QACTIVE_POST(AO_Modem, (QEvt*)&answer_evt, me);
            return Q_TRAN(&ModemSM_idle);  
            }
         else return Q_TRAN(*me->sst_ptr->handler);
         }
      else//jump
         {
         me->sst_ptr+=2;
         if (me->sst_ptr->handler==NULL)
            {
            answer_evt.answer=ANSWER_OK;
            QACTIVE_POST(AO_Modem, (QEvt*)&answer_evt, me);
            return Q_TRAN(&ModemSM_idle);  
            }
         else return Q_TRAN(*me->sst_ptr->handler);  
         } 
      }
   return Q_HANDLED(); 
   } 




void Modem_hw_init (void)
   {
   GPIO_InitTypeDef GPIO_InitStructure;
   USART_InitTypeDef USART_InitStructure;
   DMA_InitTypeDef DMA_InitStructure;
   NVIC_InitTypeDef NVIC_InitStructure;
		 
		  GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);

	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Pin = MODEM_VCC_PIN;
   GPIO_Init(MODEM_VCC_GPIO , &GPIO_InitStructure);
   MODEM_POWER_OFF();

   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Pin = POWERKEY_PIN;
   GPIO_Init(POWERKEY_GPIO , &GPIO_InitStructure);
   POWERKEY_UP();
		 
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Pin = SIM_SELECT_PIN;
   GPIO_Init(SIM_SELECT_GPIO , &GPIO_InitStructure);
	 
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
   GPIO_InitStructure.GPIO_Pin = STATUS_PIN;
   GPIO_Init(STATUS_GPIO , &GPIO_InitStructure);

   USART_InitStructure.USART_BaudRate = 115200;
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   USART_InitStructure.USART_Parity = USART_Parity_No;
   USART_InitStructure.USART_HardwareFlowControl= USART_HardwareFlowControl_None;
   USART_InitStructure.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
   USART_Init(UART_MODEM, &USART_InitStructure);
   USART_DMACmd(UART_MODEM, USART_DMAReq_Tx, ENABLE);
   USART_Cmd(UART_MODEM, ENABLE);
	 

   NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; //канал
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //приоритет
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//приоритет субгруппы
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //включаем канал
   NVIC_Init(&NVIC_InitStructure); //инициализируем
	/*  NVIC_SetPriority (USART1_IRQn, 2); 
	 NVIC_EnableIRQ(USART1_IRQn);*/
   USART_ITConfig(UART_MODEM, USART_IT_RXNE, ENABLE);  //включаем нужное прерывание на прием

   DMA_DeInit(DMA1_Channel4);
   DMA_InitStructure.DMA_PeripheralBaseAddr = (unsigned int) &(UART_MODEM->DR);
   //DMA_InitStructure.DMA_MemoryBaseAddr = (u32) DMA4_Buffer;
   DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
   DMA_InitStructure.DMA_BufferSize = 0;
   DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
   DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
   DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
   DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
   DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
   DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
   DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
   DMA_Init(DMA1_Channel4, &DMA_InitStructure);
   USART_ClearFlag(UART_MODEM, USART_FLAG_RXNE | USART_FLAG_TC | USART_FLAG_TXE);
   USART_DMACmd(UART_MODEM, USART_DMAReq_Tx, ENABLE);

   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); // Configure one bit for preemption priority
   NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure); 
	/* NVIC_SetPriority (DMA1_Channel4_IRQn, 3); 
	 NVIC_EnableIRQ(DMA1_Channel4_IRQn);*/
   DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);  



   }
	 
	 
#ifdef DEBUG_SIM900	 
void OutDebugSim900( char const *dbg_msg)
   {
   QS_BEGIN(QS_SIM900, AO_Modem)                                 
   QS_STR(dbg_msg);                              
   QS_END()
   }
	 
void OutDebugSim900Sprintf( char const *str,uint32_t val)
   {
   QS_BEGIN(QS_SIM900, AO_Modem)                                  
   QS_STR(str); 
   QS_U32(4, val);  		 
   QS_END()
   }
#endif	 
	 
	

	 
	 






