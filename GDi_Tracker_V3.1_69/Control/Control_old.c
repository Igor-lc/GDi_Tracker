

#include "qp_port.h"
#include <stm32f30x.h>
#include "Control.h"
#include "at45db.h"
#include "fm25cl64.h"
#include "user_types.h"
#include <string.h>
#include <stdio.h>
#include "Disk.h"
#include "CRC32.h"
#include "CRC16.h"
#include "config.h"
#include "FramRingBuf.h"
#include "modem.h"
#include "modem_signals.h"
#include "ByteStream.h"
#include "loader_defs.h"
#include "stm32f30x_flash.h"
#include "loader_defs.h"

#define CONTROL_STATE_HISTORY_SIZE 10
typedef enum
   {
   upd_nothing_0,
   upd_config_start_1,
   upd_firmware_start_2,
   upd_firmware_continue_3,  
   }UpdStateType;

typedef struct
   {
   QActive super;
   QTimeEvt TimeEvt;
   struct
      {
      uint32_t start_addr;
      uint32_t size;
      uint32_t crc;
      uint16_t pack_index;
      uint16_t last_pack_index;
      uint16_t ver;
      }fw;
   struct 
      {
      uint16_t size;
      uint16_t crc;
      }pack;
   UpdStateType upd_state;
   unsigned send_enable :1;
   uint16_t upd_tick_counter;
   uint32_t conf_tick_counter;
   uint32_t sendInterval_counter; 
   uint8_t write_config_counter; 
   uint8_t reopen_connection_counter; 
   uint8_t bad_packet_counter; 
   uint8_t transport_err_counter;  
   uint16_t pack_remain_len;  
   } CONTROL;

static CONTROL control; 

//-------------PUBLIC------------------------------
const FramMemoryType *fm_ptr=NULL;
QActive * const AO_Control = &control.super;
//-------------PRIVATE------------------------------- 
static QEvt const DiskOffEvt=             {DISK_OFF_SIG,0U,0U};
static QEvt const ModemOffEvt=            {MODEM_OFF_SIG,0U,0U};	 
static QEvt const ModemUpdStartEvt =      { MODEM_UPD_START_SIG, 0U, 0U}; 
static QEvt const ModemUpdRestartEvt =    { MODEM_UPD_RESTART_SIG, 0U, 0U};
static QEvt const ModemUpdGetRemainFwDataEvt = { MODEM_UPD_GET_REMAIN_FW_DATA_SIG, 0U, 0U};
static QEvt const ModemUpdFinalEvt =           { MODEM_UPD_FINAL_SIG, 0U, 0U};   
static QEvt const ControlChangeSuperStateEvt = { CONTROL_CHANGE_SUPER_STATE_SIG, 0U, 0U};
static QEvt const MakeArchPacketEvt =     { DISK_MAKE_ARCH_PACK_SIG, 0U, 0U};
static QEvt const MakeFreshPacketEvt =    { DISK_MAKE_FRESH_PACK_SIG, 0U, 0U};
static QEvt const DeletePageEvt =         { DISK_DELETE_PAGE_SIG, 0U, 0U};
static QEvt const DeletePointsToSendEvt = { DISK_DELETE_POINTS_TO_SEND_SIG, 0U, 0U};
static QEvt const UnlockPointsToSendEvt = { DISK_UNLOCK_POINTS_TO_SEND_SIG, 0U, 0U};
static QEvt const DiskReadConfigEvt =     { DISK_READ_CONFIG_SIG, 0U, 0U};   


static uint8_t fwu_buf[MAX_FWU_PACKET_SIZE];
static bool CheckConfigUpdTimeout(void);
static bool DiskIsNotEmpty(void);
static int32_t CheckPageErase(uint32_t addr);
static uint32_t MakeFwReqPack(uint8_t *buf,uint16_t npack,uint16_t fw_ver);
static FlashWriteResultType FlashWrite(CONTROL* const me,uint8_t *data_buf,uint32_t data_size);
#ifdef DEBUG_CONTROL
uint8_t control_state_history[CONTROL_STATE_HISTORY_SIZE]; 
static void ClearStateHistory(void);
static void AddStateHistory(uint8_t state);
static void OutDebugControl( char const *dbg_msg);
static void OutDebugControlSprintf1( char const *str,uint32_t val);
static void OutDebugControlSprintf2( char const *str1,char const *str2,uint32_t val1,uint32_t val2);
#else
   #define AddStateHistory(x)    NOP()
   #define ClearStateHistory()	 NOP()
   #define OutDebugControl(x)  __nop()
   #define OutDebugControlSprintf1(x1,x2) __nop()
   #define OutDebugControlSprintf2(x1,x2,x3,x4) __nop()
#endif

static QState CtrlInitial1(CONTROL * const me, QEvt const * const e);
static QState CtrlWaitDiskReady2(CONTROL * const me, QEvt const * const e);
static QState CtrlPackSuper3(CONTROL * const me, QEvt const * const e);
static QState CtrlPackFramErr4(CONTROL * const me, QEvt const * const e);
static QState CtrlPackFlashErr5(CONTROL * const me, QEvt const * const e);
static QState CtrlPackFinal6(CONTROL * const me, QEvt const * const e);
static QState CtrlPackIdle7(CONTROL * const me, QEvt const * const e);
static QState CtrlPackModemBusy8(CONTROL * const me, QEvt const * const e);
static QState CtrlPackSendPage9(CONTROL * const me, QEvt const * const e);
static QState CtrlPackSendFresh10(CONTROL * const me, QEvt const * const e);
static QState CtrlFramErrSuper11(CONTROL * const me, QEvt const * const e);
//----------------------------------------------------------------------
static QState CtrlUpdSuper12(CONTROL* const me, QEvt const * const e);
static QState CtrlUpdOpenConnection13(CONTROL * const me, QEvt const * const e);
static QState CtrlUpdReOpenConnection14(CONTROL * const me, QEvt const * const e);
static QState CtrlUpdConf15(CONTROL * const me, QEvt const * const e);
static QState CtrlUpdWaitConfWrOk16(CONTROL * const me, QEvt const * const e);
static QState CtrlUpdFwStart17(CONTROL * const me, QEvt const * const e);
static QState CtrlUpdFwContinue18(CONTROL * const me, QEvt const * const e);
static QState CtrlUpdFinal19(CONTROL * const me, QEvt const * const e);
static QState CtrlSysRestart20(CONTROL * const me, QEvt const * const e);
//--------------------------------------------------------------------------------
static QState CtrlFlashErrSuper21(CONTROL * const me, QEvt const * const e);
static QState CtrlSysRebootSuper22(CONTROL * const me, QEvt const * const e);



void Control_ctor(void)
   {
   CONTROL *me = &control;
   QActive_ctor(&me->super, Q_STATE_CAST(&CtrlInitial1));
   QTimeEvt_ctor(&me->TimeEvt,  CONTROL_TIMEOUT_SIG);
   }

QState CtrlInitial1(CONTROL* const me, QEvt const * const e)
   {
		 AddStateHistory(1);
   QS_OBJ_DICTIONARY(&control);
   QS_FUN_DICTIONARY(&CtrlInitial1);
   QS_FUN_DICTIONARY(&CtrlWaitDiskReady2);
   QS_FUN_DICTIONARY(&CtrlPackSuper3);
   QS_FUN_DICTIONARY(&CtrlPackFramErr4);
   QS_FUN_DICTIONARY(&CtrlPackFlashErr5);
   QS_FUN_DICTIONARY(&CtrlPackFinal6); 
   QS_FUN_DICTIONARY(&CtrlPackIdle7); 
   QS_FUN_DICTIONARY(&CtrlPackModemBusy8); 
   QS_FUN_DICTIONARY(&CtrlPackSendPage9); 
   QS_FUN_DICTIONARY(&CtrlPackSendFresh10);
   QS_FUN_DICTIONARY(&CtrlFramErrSuper11);    
   QS_FUN_DICTIONARY(&CtrlUpdSuper12); 
   QS_FUN_DICTIONARY(&CtrlUpdOpenConnection13);
   QS_FUN_DICTIONARY(&CtrlUpdReOpenConnection14);  
   QS_FUN_DICTIONARY(&CtrlUpdConf15); 
   QS_FUN_DICTIONARY(&CtrlUpdWaitConfWrOk16);      
   QS_FUN_DICTIONARY(&CtrlUpdFwStart17);  
   QS_FUN_DICTIONARY(&CtrlUpdFwContinue18); 
   QS_FUN_DICTIONARY(&CtrlUpdFinal19); 
   QS_FUN_DICTIONARY(&CtrlSysRestart20);
   QS_FUN_DICTIONARY(&CtrlFlashErrSuper21); 
   QS_FUN_DICTIONARY(&CtrlSysRebootSuper22); 
   // QS_FILTER_SM_OBJ(&control);

   me->send_enable=true;
   me->conf_tick_counter=0;  
   QActive_subscribe(&me->super, TIC_100ms_SIG);
   ClearStateHistory();
   return Q_TRAN(&CtrlWaitDiskReady2);
   }
	 
	
QState CtrlWaitDiskReady2(CONTROL * const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG: 
         {
            AddStateHistory(2);
					 if(APP_0_START_ADDR==SysInfo.program_start_addr)      OutDebugControlSprintf1("PROGRAM START ADDR0=",SysInfo.program_start_addr);
					 else if(APP_1_START_ADDR==SysInfo.program_start_addr) OutDebugControlSprintf1("PROGRAM START ADDR1=",SysInfo.program_start_addr);
					 else OutDebugControl("UNKNOWN PROGRAM START ADDR");			 
         }
         return Q_HANDLED();
      case CONTROL_DISK_READY_SIG:  return Q_TRAN(&CtrlPackSuper3);
      case CONTROL_FLASH_ERROR_SIG:  return Q_TRAN(&CtrlFlashErrSuper21);
      case CONTROL_FRAM_ERROR_SIG:  return Q_TRAN(&CtrlFramErrSuper11);
      case Q_EXIT_SIG:  return Q_HANDLED();
      }
   return  Q_SUPER(&QHsm_top);
   }

//--------------------------UPDATE---------------------------------

QState CtrlUpdSuper12(CONTROL* const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(12);
            me->upd_tick_counter=0;
            me->send_enable=false;
         }
         return Q_HANDLED();
      case CONTROL_UPDATE_CONNECT_OK_SIG:
         {
            OutDebugControlSprintf1("CTRL12 READY_TO_UPDATE_SIG state=",control_state_history[CONTROL_STATE_HISTORY_SIZE-1]);
            OutDebugControlSprintf1("state-1=",control_state_history[CONTROL_STATE_HISTORY_SIZE-2]);
            OutDebugControlSprintf1("state-2=",control_state_history[CONTROL_STATE_HISTORY_SIZE-3]);
            OutDebugControlSprintf1("state-3=",control_state_history[CONTROL_STATE_HISTORY_SIZE-4]);
            OutDebugControlSprintf1("state-4=",control_state_history[CONTROL_STATE_HISTORY_SIZE-5]);
         }
         return Q_HANDLED();
      case Q_INIT_SIG: return Q_TRAN(&CtrlUpdOpenConnection13);
      case CONTROL_CHANGE_SUPER_STATE_SIG:  return Q_TRAN(&CtrlPackSuper3);
      case CONTROL_FLASH_ERROR_SIG: return Q_TRAN(&CtrlFlashErrSuper21); 
      case CONTROL_FRAM_ERROR_SIG: return Q_TRAN(&CtrlFramErrSuper11); 
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return  Q_SUPER(&QHsm_top);
   }

QState CtrlUpdOpenConnection13(CONTROL * const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(13);
            me->reopen_connection_counter=0;
            me->write_config_counter=0;
            QACTIVE_POST(AO_Modem,&ModemUpdStartEvt, me);
         }
         return Q_HANDLED();
      case CONTROL_UPDATE_CONNECT_OK_SIG:
         OutDebugControlSprintf1("CTRL13 UPDATE_CONNECT_OK_SIG upd_state=",me->upd_state);
         switch (me->upd_state)
            {
            case upd_config_start_1: return Q_TRAN(&CtrlUpdConf15); 
            case upd_firmware_start_2: return Q_TRAN(&CtrlUpdFwStart17);
            case upd_firmware_continue_3: return Q_TRAN(&CtrlUpdFwContinue18);              
            default: return Q_TRAN(&CtrlUpdFinal19); 
            }
      case CONTROL_UPD_TRANSPORT_LEVEL_ERROR_SIG: return Q_TRAN(&CtrlUpdReOpenConnection14); 
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return Q_SUPER(&CtrlUpdSuper12);
   }

QState CtrlUpdReOpenConnection14(CONTROL * const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(14);
            if (++me->reopen_connection_counter>3) 
						{
							if(conf.upd_setting.tryCounter>0)conf.upd_setting.tryCounter--;
							conf.upd_setting.tryTimeout=0;
							conf.upd_setting.save=1;
							 QACTIVE_POST(AO_Disk,&DiskSaveConfigEvt, me);//save config upd counters
							QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1); //final and try again later
						}
            else QACTIVE_POST(AO_Modem,&ModemUpdRestartEvt, me);
         }
         return Q_HANDLED();
      case CONTROL_UPDATE_CONNECT_OK_SIG:
         OutDebugControlSprintf1("CTRL14 UPDATE_CONNECT_OK_SIG upd_state=",me->upd_state);
         switch (me->upd_state)
            {
            case upd_config_start_1: return Q_TRAN(&CtrlUpdConf15); 
            case upd_firmware_start_2: return Q_TRAN(&CtrlUpdFwStart17); 
            case upd_firmware_continue_3: return Q_TRAN(&CtrlUpdFwContinue18);
            default: return Q_TRAN(&CtrlUpdFinal19); 
            }
      case CONTROL_TIMEOUT_SIG:  return Q_TRAN(&CtrlUpdFinal19); 
      case CONTROL_UPD_TRANSPORT_LEVEL_ERROR_SIG: return Q_TRAN(&CtrlUpdReOpenConnection14); 
      case Q_EXIT_SIG:
         {
            QTimeEvt_disarm(&me->TimeEvt);
         }
         return Q_HANDLED();
      }
   return Q_SUPER(&CtrlUpdSuper12);
   }


QState CtrlUpdConf15(CONTROL * const me, QEvt const * const e)
   {
   static enum
      {
      waiting_tcp_data,
      fw_update_start,
      update_final,
      update_restart,
      check_packet_len,
      check_packet_crc,
      parse_config,
      waiting_config_write_ok,
				config_update_end,
      }state;
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(15);
            DataPtrEvt *pe = Q_NEW(DataPtrEvt,MODEM_UPD_SEND_PACKET_SIG);
            pe->ptr=send_pack_buf;
            pe->data.size= ConfigPacketMaker(send_pack_buf);
            QACTIVE_POST(AO_Modem,&pe->super, me);
            state=waiting_tcp_data;
         }
         return Q_HANDLED();
      case CONTROL_TCP_DATA_SIG:
         {
            DataPtrEvt *pe = (DataPtrEvt*)e;
            uint8_t *pack_ptr=pe->ptr;
            uint16_t pack_size=pe->data.size;
            uint8_t pack_head=*(uint8_t*)pack_ptr;
            uint16_t pack_len=0;
            uint8_t cmd_len;
            const char *next_cmd_ptr;
            for (;;)
               {
               switch (state)
                  {
                  case waiting_tcp_data:
                     {
                        if (pack_size==1)
                           {
                           if (SERVER_ANSWER_OK==pack_head)////Config update success,
                              {
                              OutDebugControl("SERVER_ANSWER_OK");
                              state=config_update_end;
                              }
                           else if (SERVER_ANSWER_CRC_ERROR==pack_head)
                              {
                              OutDebugControl("SERVER_ANSWER_CRC_ERROR");
                              state=update_restart;
                              }
                           else if (SERVER_ANSWER_FORMAT_ERROR==pack_head)
                              {
                              OutDebugControl("SERVER_ANSWER_FORMAT_ERROR");
                              state=update_restart;
                              }
                           else
                              {
                              OutDebugControl("ANKNOWN SERVER ANSWER");
                              state=update_restart;
                              }
                           }
                        else
                           {
                           OutDebugControlSprintf2("UPD CONF RECEIVE DATA size=","head=",pe->data.size,pack_head);
                           if (pack_head==HEAD_PACK_CONF)
                              {
                              state=check_packet_len;
                              }
                           else
                              {
                              OutDebugControl("ANKNOWN PACKET");
                              state=update_restart;
                              }
                           }
                     }               
                     break; 
                  case check_packet_len:
                     {
                        pack_len=((QPacketType*)pe->ptr)->head.len;
                        if ((pack_len==pe->data.size)&&(pack_len>sizeof(((QPacketType*)pe->ptr)->head)))
                           {
                           state=check_packet_crc; 
                           }
                        else
                           {
                           OutDebugControl("LEN ERROR");
                           state=update_restart; 
                           }
                     }
                     break;
                  case check_packet_crc:
                     {
                        uint16_t pack_crc=((QPacketType*)pe->ptr)->head.crc;
                        ((QPacketType*)pe->ptr)->head.crc=(uint16_t)0;
                        uint16_t calc_crc= MakeCRC16(pack_ptr,pack_len);
                        if (pack_crc==calc_crc)
                           {
                           state=parse_config;
                           }
                        else
                           {
                           OutDebugControl("CRC ERROR");
                           state=update_restart; 
                           }                       
                     }
                     break;
                  case parse_config:
                     {
                        FlagsType flag={0};
                        flag.save_config=false;
                        next_cmd_ptr=(char*)((QPacketType*)pe->ptr)->payload;
												uint32_t payload_size,config_size;
												payload_size=pack_len-sizeof(QPacketHeadType);//40
												config_size=strlen(next_cmd_ptr)+1;
                        if (NULL==next_cmd_ptr) state=update_restart;
                        else if (config_size>payload_size)
                           {
                           OutDebugControl("CONFIG STRING ERROR!!!");
														//  OutDebugControlSprintf2("payload_size="," config_size=",payload_size,config_size);
                           state=update_restart; 
                           }
                        else if (NULL==(next_cmd_ptr= FindPassword(next_cmd_ptr,&cmd_len)))
                           {
                           OutDebugControl("PASSWORD_ERROR!!!");
                           state=update_restart; 
                           }
                        else
                           {
                           while (NULL!=next_cmd_ptr)
                              {
                              next_cmd_ptr=ProcessCommand(next_cmd_ptr,&cmd_len,&flag);
                              }
                           if (true==flag.save_config)
                              {
                             QACTIVE_POST(AO_Disk,&DiskSaveConfigEvt, me);
                              QACTIVE_POST(AO_Disk, &DiskReadConfigEvt, me);
                              state=waiting_config_write_ok;
                              }
													else  state=config_update_end;
                           }
                     }
                     break;
										 case config_update_end:
										 {
											  //if (config.fw_ver_new!=0 && config.fw_ver_new!=FW_VER)state=fw_update_start;
											 if (conf.secur.fw_ver_new!=0)state=fw_update_start;
                        else //Config update success, so finish it
                                 {
                                 conf.upd_setting.tryCounter=0;
																 conf.upd_setting.save=1;
                                 QACTIVE_POST(AO_Disk,&DiskSaveConfigEvt, me);
                                 OutDebugControl("SEND DiskWriteConfUpdSetEvt");
                                 state=update_final;
                                 }
										 }
										 break;
                  case fw_update_start: 
                     {
                        me->upd_state=upd_firmware_start_2;
                     }
                     return Q_TRAN(&CtrlUpdFwStart17);//  //config not changed,config.fwu==true
                  case update_final:  return Q_TRAN(&CtrlUpdFinal19); //config not changed,config.fwu==false
                  case update_restart: return Q_TRAN(&CtrlUpdReOpenConnection14);//any ERROR 
                  case waiting_config_write_ok: return Q_TRAN(&CtrlUpdWaitConfWrOk16); //config changed
                  }
               }            
         }
			case CONTROL_UPD_PACKET_LEVEL_ERROR_SIG:
      case CONTROL_UPD_TRANSPORT_LEVEL_ERROR_SIG:   return Q_TRAN(&CtrlUpdReOpenConnection14); 
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return Q_SUPER(&CtrlUpdSuper12);
   }


QState CtrlUpdFwStart17(CONTROL * const me, QEvt const * const e)
   {
   static enum
      {
      waiting_tcp_data,
      check_packet_len,
      check_packet_crc,
      process_fw_descriptor,
      reopen_connection,
		  fw_update_error_exit,
      }state;
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(17);
            DataPtrEvt *pe = Q_NEW(DataPtrEvt,MODEM_UPD_SEND_PACKET_SIG);
            me->transport_err_counter=0;
					  me->fw.pack_index=0;
					  me->fw.ver=conf.secur.fw_ver_new;
            if(APP_0_START_ADDR == SysInfo.program_start_addr) me->fw.ver|=0x8000;
            pe->data.size= MakeFwReqPack(send_pack_buf, me->fw.pack_index,me->fw.ver);
            pe->ptr=send_pack_buf;
            QACTIVE_POST(AO_Modem,&pe->super, me);
            state= waiting_tcp_data;                
         }
         return Q_HANDLED();
      case CONTROL_TCP_DATA_SIG:
         {
            uint8_t *payload_ptr=NULL;

            for (;;)
               {
               switch (state)
                  {
                  case waiting_tcp_data:
                     {
                        state=reopen_connection;
                        uint8_t *pack_ptr=(uint8_t*)((DataPtrEvt*)e)->ptr;
                        me->pack.size=((DataPtrEvt*)e)->data.size;
                        OutDebugControlSprintf2("PACKET size=","head=",me->pack.size,*pack_ptr);
                        if (((DataPtrEvt*)e)->data.size==1)
                           {
                           switch (*pack_ptr)
                              {
                              case SERVER_ANSWER_OK: OutDebugControl("SERVER_ANSWER_OK");
                                 break;
                              case SERVER_ANSWER_CRC_ERROR: OutDebugControl("SERVER_ANSWER_CRC_ERROR");
                                 break;
                              case SERVER_ANSWER_FORMAT_ERROR: OutDebugControl("SERVER_ANSWER_FORMAT_ERROR");
                                 break;
                              default: OutDebugControl("ANKNOWN SERVER ANSWER");
                                 break;  
                              }
                           }
                        else if (*pack_ptr!=HEAD_PACK_FWU)
                           {
                           OutDebugControl("ANKNOWN PACKET HEAD");
                           }
                        else if (me->pack.size>MAX_FWU_PACKET_SIZE ||me->pack.size <=sizeof(QPacketHeadType))
                           {
                           OutDebugControl("ERROR! PACKET TOO LARGE");
                           }
                        else if (me->pack.size <sizeof(QPacketHeadType))
                           {
                           OutDebugControl("ERROR! PACKET TOO LITTLE");
                           }
                        else
                           {
                           for (uint32_t i=0;i<me->pack.size;i++)fwu_buf[i]=*pack_ptr++;
                           state=check_packet_len;
                           }
                     }
                     break;
                  case check_packet_len:
                     {
                        if (((QPacketType*)fwu_buf)->head.len!=me->pack.size)
                           {
                           OutDebugControl("LEN ERROR");
                           state=reopen_connection;
                           }
                        else state=check_packet_crc;   
                     }
                     break;
                  case check_packet_crc:
                     {
                        state=reopen_connection;
                        uint16_t u16temp=((QPacketType*)fwu_buf)->head.crc;
                        ((QPacketType*)fwu_buf)->head.crc=(uint16_t)0;
                        if (u16temp!=MakeCRC16(fwu_buf,me->pack.size))
                           {
                           OutDebugControl("CRC ERROR");
                           }
                        else
                           {
                           payload_ptr= ((QPacketType*)fwu_buf)->payload;
														u16temp= ((QFwDescriptorType*)payload_ptr)->pack_index;
                           if (u16temp!=me->fw.pack_index)
                              {
                              OutDebugControl("pack_index ERROR");
                              }
                           else state=process_fw_descriptor;
                           }                      
                     }
                     break;
                  case process_fw_descriptor:
                     {
											 QFwDescriptorType *fwd_ptr=(QFwDescriptorType*)payload_ptr;
                        me->fw.size=fwd_ptr->size;
                        me->fw.crc=fwd_ptr->crc;
                        me->fw.last_pack_index =fwd_ptr->max_pack_index;
                        me->fw.ver=fwd_ptr->ver;
                        if (SysInfo.program_start_addr==APP_0_START_ADDR)
												{
												  if(me->fw.ver&0x8000)me->fw.start_addr=APP_1_START_ADDR;
													else
													{
														char temp_buf[50];
															OutDebugControl("SysInfo.program_start_addr==APP_0_START_ADDR");
														sprintf(temp_buf,"FW LOCATION ERROR me->fw.ver=0x%X",me->fw.ver);
														OutDebugControl(temp_buf);
														 state=reopen_connection;
													   break;
													}
												}
                        else if(SysInfo.program_start_addr==APP_1_START_ADDR)
												{
													 if(!(me->fw.ver&0x8000))me->fw.start_addr=APP_0_START_ADDR;
													 else
													  {
															char temp_buf[50];
															OutDebugControl("SysInfo.program_start_addr==APP_1_START_ADDR");
															sprintf(temp_buf,"FW LOCATION ERROR me->fw.ver=0x%X",me->fw.ver);
														OutDebugControl(temp_buf);
														 state=reopen_connection;
													   break;
													  }
												}
												else 
												{
													OutDebugControl("SysInfo.program_start_addr ERROR");
												  state=fw_update_error_exit;
													break;
												}
                        OutDebugControlSprintf2("pack_index=","max_pack_index=",me->fw.pack_index,me->fw.max_pack_index);
                        OutDebugControlSprintf2("fw_crc=","fw_size=",me->fw.crc,me->fw.size);
                        OutDebugControlSprintf2("fw_ver=","fw_addr=",me->fw.ver,me->fw.start_addr);
                        me->fw.pack_index++;
                        me->upd_state=upd_firmware_continue_3;
                        return Q_TRAN(&CtrlUpdFwContinue18); 
                     }
                  case reopen_connection: return Q_TRAN(&CtrlUpdReOpenConnection14);
									case fw_update_error_exit: return Q_TRAN(&CtrlUpdFinal19);
                  }
               }
         } 
				   
      case CONTROL_UPD_PACKET_LEVEL_ERROR_SIG:
      case CONTROL_UPD_TRANSPORT_LEVEL_ERROR_SIG: return Q_TRAN(&CtrlUpdReOpenConnection14);
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return Q_SUPER(&CtrlUpdSuper12);
   }

QState CtrlUpdFwContinue18(CONTROL * const me, QEvt const * const e)
   {

   static enum
      {
      waiting_packet,
      waiting_packet_remain,
      check_packet_len,
      check_packet_crc,
      process_fw_data,
      bad_packet,
      next_packet_request,
      remain_packet_request,
      fw_update_error_exit,
      write_boot_rec,
      fw_update_success_exit,
      }state;
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(18);
            DataPtrEvt *pe = Q_NEW(DataPtrEvt,MODEM_UPD_SEND_PACKET_SIG);
            me->transport_err_counter=0;
            pe->data.size= MakeFwReqPack(send_pack_buf, me->fw.pack_index,me->fw.ver);
            pe->ptr=send_pack_buf;
            QACTIVE_POST(AO_Modem,&pe->super, me);
            state=  waiting_packet; 
         }
         return Q_HANDLED();
      case CONTROL_TCP_DATA_SIG:
         {
            uint8_t *payload_ptr=NULL;

            for (;;)
               {
               switch (state)
                  {
                  case waiting_packet:
                     {
                        state=bad_packet;
                        uint8_t *pack_ptr=(uint8_t*)((DataPtrEvt*)e)->ptr;
                        me->pack.size=((DataPtrEvt*)e)->data.size;
                        OutDebugControlSprintf2("PACKET size=","head=",me->pack.size,*pack_ptr);
                        if (((DataPtrEvt*)e)->data.size==1)
                           {
                           switch (*pack_ptr)
                              {
                              case SERVER_ANSWER_OK: OutDebugControl("SERVER_ANSWER_OK");
                                 break;
                              case SERVER_ANSWER_CRC_ERROR: OutDebugControl("SERVER_ANSWER_CRC_ERROR");
                                 break;
                              case SERVER_ANSWER_FORMAT_ERROR: OutDebugControl("SERVER_ANSWER_FORMAT_ERROR");
                                 break;
                              default: OutDebugControl("ANKNOWN SERVER ANSWER");
                                 break;  
                              }
                           }
                        else if (*pack_ptr!=HEAD_PACK_FWU)
                           {
                           OutDebugControl("ANKNOWN PACKET HEAD");
                           }
                        else if (me->pack.size>MAX_FWU_PACKET_SIZE ||me->pack.size <=sizeof(QPacketHeadType))
                           {
                           OutDebugControl("ERROR! PACKET TOO LARGE");
                           }
                        else if (me->pack.size <sizeof(QPacketHeadType))
                           {
                           OutDebugControl("ERROR! PACKET TOO LITTLE");
                           }
                        else
                           {
                           for (uint32_t i=0;i<me->pack.size;i++)fwu_buf[i]=*pack_ptr++;
                           state=check_packet_len;
                           }
                     }
                     break;
                  case check_packet_len:
                     {
                        if (((QPacketType*)fwu_buf)->head.len!=me->pack.size)
                           {
                           OutDebugControl("LEN ERROR");
                           me->pack_remain_len=((QPacketType*)fwu_buf)->head.len-me->pack.size;
                           state=remain_packet_request;
                           }
                        else state=check_packet_crc;   
                     }
                     break;
                  case check_packet_crc:
                     {
                        state=bad_packet;
                        uint16_t u16temp=((QPacketType*)fwu_buf)->head.crc;
                        ((QPacketType*)fwu_buf)->head.crc=(uint16_t)0;
                        if (u16temp!=MakeCRC16(fwu_buf,me->pack.size))
                           {
                           OutDebugControl("CRC ERROR");
                           }
                        else
                           {
                           payload_ptr= ((QPacketType*)fwu_buf)->payload;
                           u16temp= *(uint16_t*)payload_ptr;
                           if (u16temp!=me->fw.pack_index)
                              {
                              OutDebugControl("pack_index ERROR");
                              }
                           else state=process_fw_data;
                           }                      
                     }
                     break;
                  case process_fw_data:
                     {
                        QFwDataType *fw_ptr=(QFwDataType*)payload_ptr;
											  uint16_t fw_ver=fw_ptr->ver;
                        uint16_t pack_index=fw_ptr->pack_index;
                        uint8_t *fw_data_ptr=fw_ptr->fw_data;
                        uint16_t fw_data_size=me->pack.size-(sizeof(QPacketHeadType)+sizeof(fw_ptr->pack_index)+sizeof(fw_ptr->ver));
											  OutDebugControlSprintf2("pack.size="," fw_data_size=",me->pack.size,fw_data_size);
											 if(me->fw.ver!=fw_ver)
											    {
												  OutDebugControl("FW VER ERROR");
												  state=bad_packet;
											   }
                        else if (pack_index!=me->fw.pack_index)
                           {
                           OutDebugControlSprintf2("PACK INDEX ERROR pack_index=","fw.pack_index",pack_index,me->fw.pack_index);
                           state=bad_packet;
                           }
                        else
                           {
                           FlashWriteResultType result= FlashWrite(me,fw_data_ptr,fw_data_size);
                           switch (result)
                              {
                              case FW_OK:
                                 {
                                    OutDebugControlSprintf1("FW FRITE OK pack_index=",me->fw.pack_index);
                                    state=next_packet_request; 
                                 }
                                 break;
                              case FW_END:
                                 {
                                    OutDebugControlSprintf1("FW FRITE END pack_index=",me->fw.pack_index);  
                                    uint32_t calc_crc=Crc32Eth((uint8_t*)me->fw.start_addr,me->fw.size);
                                    if (me->fw.crc==calc_crc)
                                       {
                                       OutDebugControl("FW CRC OK");
                                       state=write_boot_rec;
                                       }
                                    else
                                       {
																				 char temp_buf[50];
																				 sprintf(temp_buf,"FW CRC ERROR calc_crc=0x%X fw_crc=0x%X",calc_crc,me->fw.crc);
																				 OutDebugControl(temp_buf);
                                         state=fw_update_error_exit;  
                                       }
                                 }
                                 break;
                              case FW_SIZE_ERROR:
                                 {
                                    OutDebugControlSprintf1("FW SIZE ERROR pack_index=",me->fw.pack_index); 
                                    state=fw_update_error_exit;                               
                                 }
                                 break;
                              case FW_FLASH_PROGRAM_ERROR:
                                 {
                                    OutDebugControlSprintf1("FW FLASH PROGRAM ERROR pack_index=",me->fw.pack_index); 
                                    state=bad_packet;                                
                                 }
                                 break;
                              case FW_FLASH_TIMEOUT_ERROR:
                                 {
                                    OutDebugControlSprintf1("FW FLASH TIMEOUT ERROR pack_index=",me->fw.pack_index);
                                    state=bad_packet;
                                 }
                                 break;
                              case FW_FLASH_ERASE_PAGE_ERROR:
                                 {
                                    OutDebugControlSprintf1("FW FLASH ERASE PAGE ERROR pack_index=",me->fw.pack_index);
                                    state=bad_packet;
                                 }
                                 break;
                              case FW_FLASH_WRP_ERROR:
                                 {
                                    OutDebugControlSprintf1("FW FLASH WRP ERROR pack_index=",me->fw.pack_index);
                                    state=bad_packet;
                                 }
                                 break;
                              case FW_FLASH_BUSY_ERROR:
                                 {
                                    OutDebugControlSprintf1("FW FLASH BUSY ERROR pack_index=",me->fw.pack_index);
                                    state=bad_packet;
                                 }
                                 break;
                              }  
                           }                           
                     }
                     break;
                  case next_packet_request:
                     {
                        me->transport_err_counter=0;
                        me->bad_packet_counter=0;
                        if (++me->fw.pack_index>me->fw.last_pack_index)state=fw_update_error_exit;
                        else
                           {
                           DataPtrEvt *pe = Q_NEW(DataPtrEvt,MODEM_UPD_SEND_PACKET_SIG);
                           pe->data.size= MakeFwReqPack(send_pack_buf, me->fw.pack_index,me->fw.ver);
                           pe->ptr=send_pack_buf;
                           QACTIVE_POST(AO_Modem,&pe->super, me);
                           state=  waiting_packet; 
                           return Q_HANDLED();
                           }
                     }
                     break;
                  case bad_packet:
                     {
                        if (++me->bad_packet_counter>10)state=fw_update_error_exit;
                        else
                           {
                           DataPtrEvt *pe = Q_NEW(DataPtrEvt,MODEM_UPD_SEND_PACKET_SIG);
                           pe->data.size= MakeFwReqPack(send_pack_buf, me->fw.pack_index,me->fw.ver);
                           pe->ptr=send_pack_buf;
                           QACTIVE_POST(AO_Modem,&pe->super, me);
                           state= waiting_packet; 
                           return Q_HANDLED();
                           }
                     }
                     break;
                  case remain_packet_request:
                     {
                        QACTIVE_POST(AO_Modem,&ModemUpdGetRemainFwDataEvt, me);
                        state=  waiting_packet_remain;
                     }
                     return Q_HANDLED();
                  case  waiting_packet_remain:
                     {
                        if (me->pack_remain_len!=((DataPtrEvt*)e)->data.size)
                           {
                           state=  bad_packet;
                           }
                        else
                           {
                           uint8_t *pack_ptr=(uint8_t*)((DataPtrEvt*)e)->ptr;
                           for (uint32_t i=0;i<me->pack_remain_len;i++)fwu_buf[i+me->pack.size]=*pack_ptr++;
                           me->pack.size+=me->pack_remain_len;
                           state=check_packet_len;
                           }
                     }
                     break;
                  case write_boot_rec:
                     {
                        static BootRecType boot_rec;
                        BootRecType  *boot_rec_ptr=(BootRecType*)BOOT_REC_ADDRESS;
                        if (boot_rec_ptr->control_word==FACTORY_RECORD)
                           {
                           OutDebugControl("FACTORY_RECORD FINDED!!!");                       
                           boot_rec.fw0.start_addr =0;
                           boot_rec.fw0.size = 0;
                           boot_rec.fw0.crc = 0;
                           boot_rec.fw1.start_addr =me->fw.start_addr;
                           boot_rec.fw1.size = me->fw.size; 
                           boot_rec.fw1.crc = me->fw.crc;
                           boot_rec.fw_to_run = 1;       
                           }
                        else
                           {
                           if (APP_0_START_ADDR==SysInfo.program_start_addr)
                              {
                              boot_rec.fw0.start_addr =boot_rec_ptr->fw0.start_addr;
                              boot_rec.fw0.size = boot_rec_ptr->fw0.size;
                              boot_rec.fw0.crc = boot_rec_ptr->fw0.crc;
                              boot_rec.fw1.start_addr =me->fw.start_addr;
                              boot_rec.fw1.size = me->fw.size; 
                              boot_rec.fw1.crc = me->fw.crc;
                              boot_rec.fw_to_run = 1;                             
                              }
                           else if (APP_1_START_ADDR==SysInfo.program_start_addr)
                              {
                              boot_rec.fw1.start_addr =boot_rec_ptr->fw1.start_addr;
                              boot_rec.fw1.size = boot_rec_ptr->fw1.size;
                              boot_rec.fw1.crc = boot_rec_ptr->fw1.crc;
                              boot_rec.fw0.start_addr =me->fw.start_addr;
                              boot_rec.fw0.size = me->fw.size; 
                              boot_rec.fw0.crc = me->fw.crc;
                              boot_rec.fw_to_run = 0;                             
                              }
                           else
                              {
                              OutDebugControl("UNKNOWN APP START ADDRESS ERROR!!!"); 
                              state=fw_update_error_exit;
                              break;
                              }
                           }
                        boot_rec.control_word =(uint32_t)0;   
                        boot_rec.crc_wrong_flag = 0xFFFFFFFF;   
                        boot_rec.crc = Crc32Eth((uint8_t*)&boot_rec, ((uint32_t)&(boot_rec.crc)-(uint32_t)&boot_rec));
												char temp_buf[30];
												sprintf(temp_buf,"boot_rec.crc=0x%X",boot_rec.crc);
                        OutDebugControl(temp_buf);
                        uint16_t *p = (uint16_t*)&boot_rec;
                        uint32_t addr=BOOT_REC_ADDRESS;
                        FLASH_Status flash_status = FLASH_COMPLETE;
                        FLASH_Unlock();
                        FLASH_ErasePage(addr);
                        for (uint32_t i=0; (flash_status == FLASH_COMPLETE && i<(sizeof(BootRecType)/sizeof(uint16_t))); i++)
                           {
                           flash_status = FLASH_ProgramHalfWord(addr, p[i]);
                           addr += sizeof(uint16_t);
                           }
                        FLASH_Lock();
                        if (flash_status == FLASH_COMPLETE)
                           {
                            OutDebugControl("BOOT RECORD WRITE OK!!!"); 
														 state=fw_update_success_exit;
                           }
                        else
                           {
														 OutDebugControl("BOOT RECORD WRITE ERROR!!!"); 
                             state=fw_update_error_exit;
                           }   
                     }
                     break;
                  case fw_update_success_exit: return Q_TRAN(&CtrlSysRestart20); 
                  case fw_update_error_exit: return Q_TRAN(&CtrlUpdFinal19); 
                  }
               }
         } 
      case CONTROL_UPD_PACKET_LEVEL_ERROR_SIG:
         {
            if (++me->bad_packet_counter>10) return Q_TRAN(&CtrlUpdFinal19);
            switch (state)
               {
               case waiting_packet:
                  {
                     QACTIVE_POST(AO_Modem,&ModemUpdGetRemainFwDataEvt, me);
                     state=  waiting_packet_remain;
                  }
                  return Q_HANDLED();
               default: return Q_TRAN(&CtrlUpdFwContinue18);
               }
         }
      case CONTROL_UPD_TRANSPORT_LEVEL_ERROR_SIG: return Q_TRAN(&CtrlUpdReOpenConnection14);
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return Q_SUPER(&CtrlUpdSuper12);
   }


QState CtrlUpdWaitConfWrOk16(CONTROL * const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(16);
            ++me->write_config_counter;
         }       
         return Q_HANDLED();
      case  CONTROL_CONFIG_WRITE_OK_SIG: 
         if (me->write_config_counter<5)return Q_TRAN(&CtrlUpdConf15);
         else return Q_TRAN(&CtrlUpdFinal19); 
      }
   return Q_SUPER(&CtrlUpdSuper12);
   }


QState CtrlUpdFinal19(CONTROL * const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG: 
         {
            AddStateHistory(19);
            QACTIVE_POST(AO_Modem,&ModemUpdFinalEvt, me);
            QACTIVE_POST(AO_Control, &ControlChangeSuperStateEvt, me); 
         }
         return Q_HANDLED();
      }
   return Q_SUPER(&CtrlUpdSuper12);
   }
	 
	
//-------------------PACKET------------------------------------------------------
QState CtrlPackSuper3(CONTROL* const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(3);
            me->upd_tick_counter=0;
            me->send_enable=false;
         }
         return Q_HANDLED();
      case CONTROL_TIMED_SEND_SIG: 
         {
            me->send_enable=true;
         }
         return Q_HANDLED();
      case TIC_100ms_SIG:
         {  
            if (conf.upd_setting.tryCounter>0)//config update allowed 
               {
               if (conf.upd_setting.tryTimeout<CONFIG_UPDATE_TIMEOUT)//increment tryTimeout allowed
                  {
                  if (++me->conf_tick_counter>600)
                     {
                     me->conf_tick_counter=0;
                     conf.upd_setting.tryTimeout++;
										 conf.upd_setting.save=1;
                     QACTIVE_POST(AO_Disk,&DiskSaveConfigEvt, me);//save config upd counters
                     }
                  }
               }
            if (flash_disk_image.count!=0)
               {
               me->send_enable=true;
               }
            else if (++ me->upd_tick_counter==10)
               {
               me->upd_tick_counter=0;
               if (me->send_enable==false)
                  {
                  if (++me->sendInterval_counter>conf.packet_send.time)
                     {
                     me->send_enable=true;
                     }
                  }
               }
         }
         return Q_HANDLED();
			case Q_INIT_SIG:                      return Q_TRAN(&CtrlPackIdle7);
			case CONTROL_SYS_REBOOT_SIG:          return Q_TRAN(&CtrlSysRebootSuper22);
			case CONTROL_CHANGE_SUPER_STATE_SIG:  return Q_TRAN(&CtrlUpdSuper12);
      case CONTROL_FLASH_ERROR_SIG:         return Q_TRAN(&CtrlFlashErrSuper21);
      case CONTROL_FRAM_ERROR_SIG:          return Q_TRAN(&CtrlFramErrSuper11); 
      case Q_EXIT_SIG:                      return Q_HANDLED();
      }
   return  Q_SUPER(&QHsm_top);
   }

QState CtrlPackFinal6(CONTROL * const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG: 
         {
            AddStateHistory(6);
            QACTIVE_POST(AO_Control, &ControlChangeSuperStateEvt, me); 
         }
         return Q_HANDLED();
      }
   return Q_SUPER(&CtrlPackSuper3);
   }


QState CtrlPackIdle7(CONTROL * const me, QEvt const * const e)
   {
   QState ret;
   switch (e->sig)
      {
      case Q_ENTRY_SIG: 
         {
            AddStateHistory(7);
            QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1); 
         }
         return Q_HANDLED();
      case CONTROL_TIMEOUT_SIG:
      case TIC_100ms_SIG:
         { 
            if (true==CheckConfigUpdTimeout())
               {
               me->upd_state=upd_config_start_1;
               ret= Q_TRAN(&CtrlPackFinal6);  
               }
            else if ((me->send_enable==true)&&DiskIsNotEmpty())
               {
               ret= Q_TRAN(&CtrlPackModemBusy8);
               }
            else  ret= Q_SUPER(&CtrlPackSuper3);
         }
         return ret;
      case CONTROL_FLASH_ERROR_SIG:        return Q_TRAN(&CtrlPackFlashErr5); 
      case CONTROL_FRAM_ERROR_SIG:         return Q_TRAN(&CtrlPackFramErr4); 
      case Q_EXIT_SIG:
         {
            QTimeEvt_disarm(&me->TimeEvt);
         }
         return Q_HANDLED();
      }
   return Q_SUPER(&CtrlPackSuper3);
   }


QState CtrlPackModemBusy8(CONTROL * const me, QEvt const * const e)
   {
   QState ret;
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(8);
            SmallEvt *pe = Q_NEW(SmallEvt,MODEM_REQUEST_TO_SEND_SIG);
            QACTIVE_POST(AO_Modem, &pe->super, me);
         }
         return Q_HANDLED();
      case TIC_100ms_SIG:
         { 
            if (true==CheckConfigUpdTimeout())
               {
               me->upd_state=upd_config_start_1;
               ret= Q_TRAN(&CtrlPackFinal6);  
               }
            else  ret= Q_SUPER(&CtrlPackSuper3);
         }
         return ret;
      case CONTROL_MODEM_READY_SIG:
         {
            OutDebugControl("MODEM_READY_SIG");
            if (flash_disk_image.count>0)ret= Q_TRAN(&CtrlPackSendPage9);
            else if (FramRingBuf_size_used()>0)ret= Q_TRAN(&CtrlPackSendFresh10);
            else ret= Q_TRAN(&CtrlPackIdle7);  
         }
         return ret;
      case CONTROL_FLASH_ERROR_SIG:        return Q_TRAN(&CtrlPackFlashErr5); 
      case CONTROL_FRAM_ERROR_SIG:         return Q_TRAN(&CtrlPackFramErr4); 
      case Q_EXIT_SIG: return Q_HANDLED();
      default: ret= Q_SUPER(&CtrlPackSuper3);
      }
   return ret;
   }


QState CtrlPackSendPage9(CONTROL * const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(9);
            QACTIVE_POST(AO_Disk,&MakeArchPacketEvt, me);      
         }
         return Q_HANDLED();
      case CONTROL_BAD_PACKET_SIG:
         {
            OutDebugControl("BAD_PACKET_SIG");
            QACTIVE_POST(AO_Disk,&DeletePageEvt, me);       
         }
         return  Q_TRAN(&CtrlPackIdle7);
      case CONTROL_SEND_PACKET_OK_SIG:
         {
            OutDebugControl("SEND_PACKET_OK_SIG");
            me->send_enable=false;
            me->sendInterval_counter=0;  
            QACTIVE_POST(AO_Disk,&DeletePageEvt, me);            
         }
         return  Q_TRAN(&CtrlPackIdle7);
      case CONTROL_SEND_PACKET_ERROR_SIG:
         {
            OutDebugControl("SEND_PACKET_ERROR_SIG"); 
         }
         return  Q_TRAN(&CtrlPackIdle7); 
      case CONTROL_MAKE_PACKET_ERROR_SIG: 
         {
            OutDebugControl("MAKE_PACKET_ERROR_SIG"); 
         }
         return Q_TRAN(&CtrlPackIdle7); 
      case CONTROL_FLASH_ERROR_SIG:        return Q_TRAN(&CtrlPackFlashErr5); 
      case CONTROL_FRAM_ERROR_SIG:         return Q_TRAN(&CtrlPackFramErr4); 
      case Q_EXIT_SIG: return  Q_HANDLED();
      }
   return  Q_SUPER(&CtrlPackSuper3);
   }


QState CtrlPackSendFresh10(CONTROL* const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(10);
            QACTIVE_POST(AO_Disk,&MakeFreshPacketEvt, me);  
         }
         return Q_HANDLED();
      case CONTROL_BAD_PACKET_SIG:
         {
            OutDebugControl("BAD_PACKET_SIG");
            QACTIVE_POST(AO_Disk,&DeletePointsToSendEvt, me);    
         }
         return  Q_TRAN(&CtrlPackIdle7);
      case CONTROL_SEND_PACKET_OK_SIG:
         {
            OutDebugControl("SEND_PACKET_OK_SIG");
            me->send_enable=false;
            me->sendInterval_counter=0;  
            QACTIVE_POST(AO_Disk,&DeletePointsToSendEvt, me);        
         }
         return  Q_TRAN(&CtrlPackIdle7);
      case CONTROL_SEND_PACKET_ERROR_SIG:
         {
            OutDebugControl("SEND_PACKET_ERROR_SIG"); 
            QACTIVE_POST(AO_Disk,&UnlockPointsToSendEvt, me); 
         }
         return  Q_TRAN(&CtrlPackIdle7);
      case CONTROL_FLASH_ERROR_SIG:        return Q_TRAN(&CtrlPackFlashErr5); 
      case CONTROL_FRAM_ERROR_SIG:         return Q_TRAN(&CtrlPackFramErr4); 
      case Q_EXIT_SIG:  return Q_HANDLED();
      }
   return  Q_SUPER(&CtrlPackSuper3);
   }


QState CtrlPackFramErr4(CONTROL* const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(4);
           DataPtrEvt *pe = Q_NEW(DataPtrEvt,CONTROL_FRAM_ERROR_SIG);  
           QACTIVE_POST(AO_Control,&pe->super, me);
         }
         return Q_HANDLED();
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return  Q_SUPER(&CtrlPackSuper3);
   }


QState CtrlPackFlashErr5(CONTROL* const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(5);
            static QEvt const FlashErrorEvt = { CONTROL_FLASH_ERROR_SIG, 0U, 0U};
            QACTIVE_POST(AO_Control, &FlashErrorEvt, me);
         }
         return Q_HANDLED();
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return  Q_SUPER(&CtrlPackSuper3);
   }
	 
	 
	  QState CtrlSysRestart20(CONTROL * const me, QEvt const * const e)
   {
		 static uint8_t object_counter;
		  static uint16_t restart_counter;
   switch (e->sig)
      {
      case Q_ENTRY_SIG: 
         {
            AddStateHistory(20);
					  conf.upd_setting.tryCounter=10;
		        conf.upd_setting.tryTimeout=CONFIG_UPDATE_TIMEOUT;
					  conf.upd_setting.save=1;
					  QACTIVE_POST(AO_Disk, &DiskSaveConfigEvt, me); 
            QACTIVE_POST(AO_Modem,&ModemOffEvt, me);
            QACTIVE_POST(AO_Disk, &DiskOffEvt, me); 
					 object_counter=2;
					 restart_counter=600;
         }
         return Q_HANDLED();
				 case TIC_100ms_SIG:
				 {
					 if(-- restart_counter==0 ||object_counter==0)
					 {
						 NVIC_SystemReset();
					 }
				 }
				    return Q_HANDLED();
				 case CONTROL_MODEM_IS_OFF_SIG:
				 {
					  OutDebugControl("MODEM_IS_OFF_SIG");
					 if(object_counter) object_counter--;
				 }
				  return Q_HANDLED();
				  case CONTROL_DISK_IS_OFF_SIG:
				 {
					  OutDebugControl("DISK_IS_OFF_SIG");
					  if(object_counter) object_counter--;
				 }
				  return Q_HANDLED();
      }
   return Q_SUPER(&CtrlUpdSuper12);
   }
	 
	 
	  QState CtrlSysRebootSuper22(CONTROL * const me, QEvt const * const e)
   {
		 static uint8_t object_counter;
		  static uint16_t restart_counter;
   switch (e->sig)
      {
      case Q_ENTRY_SIG: 
         {
            AddStateHistory(22); 
            QACTIVE_POST(AO_Modem,&ModemOffEvt, me);
            QACTIVE_POST(AO_Disk, &DiskOffEvt, me); 
					 object_counter=2;
					 restart_counter=600;
         }
         return Q_HANDLED();
				 case TIC_100ms_SIG:
				 {
					 if(-- restart_counter==0 || object_counter==0)
					 {
						 NVIC_SystemReset();
					 }
				 }
				    return Q_HANDLED();
				 case CONTROL_MODEM_IS_OFF_SIG:
				 {
					  OutDebugControl("MODEM_IS_OFF_SIG");
					 if(object_counter) object_counter--;
				 }
				  return Q_HANDLED();
				  case CONTROL_DISK_IS_OFF_SIG:
				 {
					  OutDebugControl("DISK_IS_OFF_SIG");
					  if(object_counter) object_counter--;
				 }
				  return Q_HANDLED();
      }
   return  Q_SUPER(&QHsm_top);
   }



/*QState CtrlFramErrSuper11(CONTROL* const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(11);
         }
         return Q_HANDLED();
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return  Q_SUPER(&QHsm_top);
   }*/
	 
	  QState CtrlFramErrSuper11(CONTROL * const me, QEvt const * const e)
   {
		 static uint8_t object_counter;
		  static uint16_t restart_counter;
		 static enum
		 {
			 send_sms,
			 waiting_objects,
		 }state;
   switch (e->sig)
      {
      case Q_ENTRY_SIG: 
         {
            AddStateHistory(11); 
					 restart_counter=600;
					  DataPtrEvt *pe = Q_NEW(DataPtrEvt,MODEM_SEND_SMS_SIG); 
            QACTIVE_POST(AO_Modem,&pe->super, me);
					 state=send_sms;
         }
         return Q_HANDLED();
				/* case CONTROL_SYS_REBOOT_SIG: 
				 {
					 QACTIVE_POST(AO_Modem,&ModemOffEvt, me);
            QACTIVE_POST(AO_Disk, &DiskOffEvt, me); 
					 object_counter=2;
					 restart_counter=600; 
				 }
				  return Q_HANDLED();*/
				 case TIC_100ms_SIG:
				 {
					 switch(state)
					 {
						 case send_sms:
						 {
							if(--restart_counter==0)
							{
								QACTIVE_POST(AO_Modem,&ModemOffEvt, me); 
                restart_counter=600;
								object_counter=1;
								state=waiting_objects;
							}								
						 }
						 break;
						 case waiting_objects:
						 {
							 if(--restart_counter==0||object_counter==0)NVIC_SystemReset();
						 }
						 break;
					 }
				 }
				    return Q_HANDLED();
				 case CONTROL_MODEM_IS_OFF_SIG:
				 {
					  OutDebugControl("MODEM_IS_OFF_SIG");
					 if(object_counter) object_counter--;
				 }
				  return Q_HANDLED();
			/*case CONTROL_DISK_IS_OFF_SIG:
				 {
					  OutDebugControl("DISK_IS_OFF_SIG");
					  if(object_counter) object_counter--;
				 }
				  return Q_HANDLED();*/
      }
   return  Q_SUPER(&QHsm_top);
   }


	 
	 QState CtrlFlashErrSuper21(CONTROL* const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
            AddStateHistory(21);
         }
         return Q_HANDLED();
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return  Q_SUPER(&QHsm_top);
   }
	 
	 


bool DiskIsNotEmpty(void)
   {
   if (flash_disk_image.count>0)return true;
   if (FramRingBuf_size_used()>0)return true;
   return false;
   }

#ifdef DEBUG_CONTROL
void OutDebugControl( char const *dbg_msg)
   {
   QS_BEGIN(QS_CONTROL, AO_Control)                                 
   QS_STR(dbg_msg);                              
   QS_END()
   }

void OutDebugControlSprintf1( char const *str,uint32_t val)
   {
   QS_BEGIN(QS_CONTROL, AO_Control)                                  
   QS_STR(str); 
   QS_U32(4, val);       
   QS_END()
   }

void OutDebugControlSprintf2( char const *str1,char const *str2,uint32_t val1,uint32_t val2)
   {
   QS_BEGIN(QS_CONTROL, AO_Control)                                  
   QS_STR(str1); 
   QS_U32(4, val1);
   QS_STR(str2); 
   QS_U32(4, val2);        
   QS_END()
   }

void AddStateHistory(uint8_t state)
   {
   for (uint32_t i=0;i<CONTROL_STATE_HISTORY_SIZE-1;i++)
      {
      control_state_history[i]=control_state_history[i+1];
      }
   control_state_history[CONTROL_STATE_HISTORY_SIZE-1]=state;
   }

void ClearStateHistory(void)
   {
   for (uint32_t i=0;i<CONTROL_STATE_HISTORY_SIZE;i++)
      {
      control_state_history[i]=0;
      }
   }
#endif


/*	 void OutDebugPacketSprintf3( char const *str1,char const *str2,char const *str3,uint32_t val1,uint32_t val2,uint32_t val3)
   {
   QS_BEGIN(PACKET_DBG, AO_Packet)                                  
   QS_STR(str1); 
   QS_U32(4, val1);
   QS_STR(str2); 
   QS_U32(4, val2);
   QS_STR(str3); 
   QS_U32(4, val3);  		 
   QS_END()
   }*/


FlashWriteResultType FlashWrite(CONTROL* const me,uint8_t *data_buf,uint32_t data_size)
   {
   FlashWriteResultType result=FW_OK;
   FLASH_Status fs;
   int32_t i=0;
   u16 *pd ;
   static uint32_t write_addr;
   static uint32_t word_counter,page_counter,byte_counter;
   uint32_t len;
   static enum
      {
      start,
      write_next_data,                    
      }state=start;

   if (me->fw.pack_index==1)state=start;
   switch (state)
      {
      case start:
         {
            write_addr=me->fw.start_addr;
            len=data_size;
            i=0;
            byte_counter=0;
            word_counter=0;
            page_counter=0;
            QF_INT_DISABLE();
            FLASH_Unlock();
            fs=FLASH_ErasePage(write_addr);
            FLASH_Lock(); 
            QF_INT_ENABLE();          
            if (fs!=FLASH_COMPLETE)
               {
               OutDebugControl("FLASH_ErasePage ERROR!!!");
               switch (fs)
                  {
                  case FLASH_COMPLETE:break;
                  case FLASH_ERROR_PROGRAM: return FW_FLASH_PROGRAM_ERROR; 
                  case FLASH_TIMEOUT: return FW_FLASH_TIMEOUT_ERROR;
                  case FLASH_ERROR_WRP: return FW_FLASH_WRP_ERROR;
                  case FLASH_BUSY: return FW_FLASH_BUSY_ERROR;
                  }           
               }
            int32_t check_result=CheckPageErase(write_addr);
            if (-1!=check_result)
               {
               OutDebugControlSprintf1("CheckPageErase ERROR offset=",check_result);
               return FW_FLASH_ERASE_PAGE_ERROR;  
               }

            pd = (u16*)data_buf;
            QF_INT_DISABLE();
            FLASH_Unlock();
            while (fs == FLASH_COMPLETE && i < (len/sizeof(u16)))
               {
               fs = FLASH_ProgramHalfWord(write_addr, pd[i]);
               i++;
               word_counter++;
               byte_counter+=2;    
               write_addr += sizeof(u16);
               }
            FLASH_Lock(); 
            QF_INT_ENABLE();               
            switch (fs)
               {
               case FLASH_COMPLETE:
                  {
                     state=write_next_data;
                     page_counter++; 
                     result= FW_OK;   
                  }
                  break;
               case FLASH_ERROR_PROGRAM:
                  {
                     result= FW_FLASH_PROGRAM_ERROR; 
                     OutDebugControlSprintf1("FW_FLASH_PROGRAM_ERROR i=",i);
                  }
                  break;
               case FLASH_TIMEOUT:
                  {
                     result=FW_FLASH_TIMEOUT_ERROR;
                  }
                  break;
               case FLASH_ERROR_WRP:
                  {
                     result=FW_FLASH_WRP_ERROR;
                  }
                  break;
               case FLASH_BUSY:
                  {
                     result=FW_FLASH_BUSY_ERROR;
                  }
                  break;
               }
         }
         break;
      case write_next_data:
         {
            pd = (u16*)data_buf;
            len=data_size;
            i=0;
            fs = FLASH_COMPLETE;
            FLASH_Unlock();
            while (fs == FLASH_COMPLETE && i < (len/sizeof(u16)))
               {
               if (word_counter==FLASH_PAGE_SIZE/2)
                  {
                  word_counter=0;
                  fs=FLASH_ErasePage(write_addr);
                  if (fs!=FLASH_COMPLETE)OutDebugControl("FLASH_ErasePage ERROR!!!");
                  else
                     {
                     page_counter++;    
                     OutDebugControlSprintf1("FLASH_ErasePage OK num=",page_counter);
                     }                                                   
                  }
               fs = FLASH_ProgramHalfWord(write_addr, pd[i]);
               if (fs!=FLASH_COMPLETE)OutDebugControl("FLASH_ProgramHalfWord ERROR!!!");
               i++;
               word_counter++;
               byte_counter+=2;    
               write_addr += sizeof(u16);
               }
            FLASH_Lock(); 
            if (fs == FLASH_COMPLETE)
               {
               if (byte_counter==me->fw.size)result= FW_END;
               else if (byte_counter>me->fw.size)result= FW_SIZE_ERROR;
               else
                  {
                  OutDebugControlSprintf2("fw_len=", "bytes stored=",me->fw.size,byte_counter);
                  result= FW_OK;
                  }                                       
               }
            else
               {
               OutDebugControl("fs != FLASH_COMPLETE!!! ERROR!!!");
               result=FW_FLASH_PROGRAM_ERROR;
               }             
         }
         break;
      }
   return result;
   }

int32_t CheckPageErase(uint32_t addr)
   {
   for (uint32_t i=0;i<FLASH_PAGE_SIZE/4;i+=4)
      {
      if (*(uint32_t*)(addr+i)!=0xFFFFFFFF)return i;
      }
   return -1;
   }

uint32_t MakeFwReqPack(uint8_t *buf,uint16_t npack,uint16_t fw_ver)
   {
   uint8_t *ptr=buf;
   *ptr++=HEAD_PACK_FWU;
   *ptr++=(uint8_t)npack;
   *ptr++=(uint8_t)(npack>>8);
   *ptr++=(uint8_t)fw_ver;
   *ptr++=(uint8_t)(fw_ver>>8);
   return ptr-buf;
   }

bool CheckConfigUpdTimeout(void)
   {
   static uint8_t debug_counter=0;
   if (++debug_counter==50)
      {
      debug_counter=0;
      OutDebugControlSprintf2( "ConfigtryCounter=","ConfigtryTimeout=",conf_upd_setting.tryCounter,conf_upd_setting.tryTimeout);
      }
   return(conf.upd_setting.tryCounter>0 && conf.upd_setting.tryTimeout>=CONFIG_UPDATE_TIMEOUT);
   }








