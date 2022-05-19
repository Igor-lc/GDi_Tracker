

#include "stm32f30x.h"
#include "qp_port.h"
#include "memory_spi.h"
#include <stm32f30x.h>
#include "packet.h"
#include "at45db.h"
#include "fm25cl64.h"
#include "UserTypes.h"
#include "main.h"
#include <string.h>
#include "flash_disk.h"
#include "CRC32.h"
#include "CRC16.h"
#include "config.h"
#include "FramRingBuf.h"
#include "modem.h"
#include "modem_signals.h"
#include "ByteStream.h"


typedef struct
{
   QActive super;
	 QTimeEvt TimeEvt;
   PackFmFlagType flag; 
  WriteConfigMessageType *write_config_ptr; 
 QPointType *point_ptr;	
	uint16_t tick_counter;
	uint32_t sendInterval_counter; 
	 SendStatusType send_status; 
} PACK;

static PACK pack; 

//-------------PUBLIC------------------------------
const FramMemoryType *fm_ptr=NULL;
QActive * const AO_Packet = &pack.super;
//-------------PRIVATE-------------------------------
static uint8_t points_to_send_buf[POINTS_TO_SEND_BUF_SIZE]; 
static uint8_t pack_buf[PACKET_BUF_SIZE];
static FlashPageType flash_page_buf;
static FramPointsRingBufType points_ram_buf; 
static FlashPagesRingBufType flash_disk_image;//records ring buf image
static bool CopyRecordsRingBufImageToRam(FlashPagesRingBufType *fptr);
static bool GetPointsFromTailToWriteBuf(void);
static bool ConfigInit(ConfigType *config_ptr);
static bool DiskIsNotEmpty(void);
static bool ReadConfig(void);
static void WriteConfig(PACK *me);
static void WritePoint(PACK *me);
static int32_t DeletePageFromTail(void);
static int32_t PointsFmPacketMaker(uint8_t *buf_ptr,uint8_t *points_to_send_buf_ptr,uint16_t points_to_send_buf_size,uint8_t* points_to_send_count);
static uint32_t PageFmPacketMaker(uint8_t *buf_ptr,FlashPageType *page_ptr);
static void WriteConfigToFramAndSendRestart(RestartFlagsType restart);
static void SaveFatToFram(FlashPagesRingBufType *fptr);
static void OutDebugPacket( char const *dbg_msg);
static void OutDebugPacketSprintf1( char const *str,uint32_t val);
static void OutDebugPacketSprintf2( char const *str1,char const *str2,uint32_t val1,uint32_t val2);
static void OutDebugPacketSprintf3( char const *str1,char const *str2,char const *str3,uint32_t val1,uint32_t val2,uint32_t val3);
static QState Packet_initial(PACK * const me, QEvt const * const e);
static QState Packet_top(PACK * const me, QEvt const * const e);
static QState Packet_test_fram(PACK * const me, QEvt const * const e);
static QState Packet_test_flash(PACK * const me, QEvt const * const e);
static QState Packet_fram_error(PACK * const me, QEvt const * const e);
static QState Packet_flash_error(PACK * const me, QEvt const * const e);
static QState Packet_fram_and_flash_error(PACK * const me, QEvt const * const e);
static QState Packet_flash_disk_init(PACK * const me, QEvt const * const e);
static QState Packet_waiting(PACK * const me, QEvt const * const e);
static QState Packet_modem_busy(PACK * const me, QEvt const * const e);
static QState Packet_read_page(PACK * const me, QEvt const * const e);
static QState Packet_send_page(PACK * const me, QEvt const * const e);
static QState Packet_delete_bad_page(PACK * const me, QEvt const * const e);
static QState Packet_send_fresh_packet(PACK * const me, QEvt const * const e);

void Packet_ctor(void)
{
   PACK *me = &pack;
	 QTimeEvt_ctor(&me->TimeEvt,  PACKET_TIMEOUT_SIG);
   QActive_ctor(&me->super, Q_STATE_CAST(&Packet_initial));
}



QState Packet_initial(PACK * const me, QEvt const * const e)
{
	QS_OBJ_DICTIONARY(&pack);
  QS_FUN_DICTIONARY(&Packet_initial);
	QS_FUN_DICTIONARY(&Packet_test_fram);
  QS_FUN_DICTIONARY(&Packet_test_flash);
	QS_FUN_DICTIONARY(&Packet_fram_error);
	QS_FUN_DICTIONARY(&Packet_flash_error);
	QS_FUN_DICTIONARY(&Packet_fram_and_flash_error);
	QS_FUN_DICTIONARY(&Packet_flash_disk_init);
	QS_FUN_DICTIONARY(&Packet_waiting); 
  QS_FUN_DICTIONARY(&Packet_modem_busy); 
	QS_FUN_DICTIONARY(&Packet_read_page); 
	QS_FUN_DICTIONARY(&Packet_send_page); 
	QS_FUN_DICTIONARY(&Packet_send_fresh_packet); 
	QS_FUN_DICTIONARY(&Packet_delete_bad_page); 
	QS_FILTER_SM_OBJ(&pack);
  
	 MemorySpi_init();
   AT45DB_CS_DIS();
   FM25_CS_DIS();
   me->write_config_ptr=NULL;
   me->flag.timed_send=true;
   me->flag.spi_busy=false;
   me->point_ptr=NULL;
   SysInfo.flag.write_point=false;
	 QActive_subscribe(&me->super, TIC_100ms_SIG);
   return Q_TRAN(&Packet_top);
}

QState Packet_top(PACK * const me, QEvt const * const e)
{
	 // QState ret;
   switch (e->sig)
   {
      case Q_ENTRY_SIG: return Q_HANDLED();
		  case Q_INIT_SIG: return Q_TRAN(&Packet_test_fram);
			case TIC_100ms_SIG:
			{      
				if (flash_disk_image.count!=0)
						{
							me->flag.timed_send=true;
            }
				else if (++ me->tick_counter==10)
               {
               me->tick_counter=0;
               if (me->flag.timed_send==false)
                  {
                  if (++me->sendInterval_counter>config.packetsend.time)
                     {
                     me->flag.timed_send=true;
                     }
                  }
               }
			}
			 return Q_HANDLED();
			case PACK_POINT_SIG:
			{
				 if(((QPointMessageType*)e)->send_flag==1)
					     {
						    me->flag.timed_send=true;
					     }
					if (me->flag.spi_busy==false)
               {
               me->flag.fram_error= FramRingBuf_AddToHead(&((QPointMessageType*)e)->point);
               if (me->flag.fram_error==false)
                  {
										OutDebugPacketSprintf3("ADD POINT OK size:used="," toflash="," tosend=",FramRingBuf_size_used(),FramRingBuf_size_to_flash(),FramRingBuf_size_to_send());
                    OutDebugPacketSprintf3("timed_send="," sendInterval_counter="," config.packetsend.time=",me->flag.timed_send,me->sendInterval_counter,config.packetsend.time);
										if (FramRingBuf_size_to_flash()>DF_PAGE_SIZE)
                       {
												 me->flag.fram_error=GetPointsFromTailToWriteBuf();
												 if(me->flag.fram_error==false)
                           {
                            me->flag.spi_busy=true;
														SmallEvt *pe = Q_NEW(SmallEvt,FDISK_WRITE_TO_HEAD_SIG);
                            QACTIVE_POST(AO_Fdisk, &pe->super, me);
                           }
                       }
                  }
							 else
									{
                  OutDebugPacket("FramRingBuf_AddToHead ERROR");
                  }
               }
            else 
						   {
							  me->point_ptr=&((QPointMessageType*)e)->point;   
						   }							
			}
			 return Q_HANDLED();
			 case FD_ANSW_WRITE_OK_SIG:
         {
            me->flag.spi_busy=false;
            SaveFatToFram(&flash_disk_image);
            OutDebugPacket("FD_ANSW_WRITE_OK_SIG"); 
           	WritePoint(me);
				    WriteConfig(me);  
            if (me->point_ptr!=NULL)
               {
                me->flag.fram_error= FramRingBuf_AddToHead(me->point_ptr);
							  if (me->flag.fram_error==false)
									 {
										 OutDebugPacketSprintf3("ADD POINT OK size :used="," :toflash="," :tosend=",FramRingBuf_size_used(),FramRingBuf_size_to_flash(),FramRingBuf_size_to_send());
									 	 SmallEvt *pe = Q_NEW(SmallEvt,PACK_PAGE_WRITE_COMPLETE_SIG);
                     QACTIVE_POST(AO_Packet, &pe->super, me);
                   }
              else
                  {
                  OutDebugPacket("FramRingBuf_AddToHead ERROR!!!");
                  }
               }
         }
          return Q_HANDLED();
			 case FD_ANSW_WRITE_ERR_SIG:
         {
            me->flag.spi_busy=false;
					  me->flag.flash_error=true;
            OutDebugPacket("FD_ANSW_WRITE_ERR_SIG");  					 
         }
         return Q_HANDLED();
			 case PACK_WRITE_CONFIG_SIG:
         {
            OutDebugPacket("PACK_WRITE_CONFIG_SIG");
					  me->write_config_ptr=(void*)e;
            WriteConfig(me); 
         }
         return Q_HANDLED();
			case Q_EXIT_SIG: return Q_HANDLED();
			
   }
   return  Q_SUPER(&QHsm_top);
}



QState Packet_test_fram(PACK * const me, QEvt const * const e)
{
	 char test[FRAM_TEST_STRING_LEN]={"TEST FRAM OK!!!"};
  QState ret;
   switch (e->sig)
   {
      case Q_ENTRY_SIG:
      {
				 me->flag.fram_error=false;
				 QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
				ret=Q_HANDLED();
      }
       break;
			case PACKET_TIMEOUT_SIG:
			{
						char test_buf[FRAM_TEST_STRING_LEN];
            FM25_WriteArray(TEST_ADDR,(uint8_t*)test,sizeof(test));
            FM25_ReadArray(TEST_ADDR,(uint8_t*)test_buf,sizeof(test));
            if (0==strcmp(test_buf,test))
               {
               OutDebugPacket(test_buf); 
               }
            else
               {
               OutDebugPacket("TEST FRAM ERROR");
								 me->flag.fram_error=true;      
               } 	
            ret= Q_TRAN(&Packet_test_flash);							 
			}
			  break;
			 case Q_EXIT_SIG:
			 {
				 	QTimeEvt_disarm(&me->TimeEvt);
				 ret= Q_HANDLED();
			 }
			 break;
			 default: ret= Q_SUPER(&Packet_top);
   }
   return ret;
}


QState Packet_test_flash(PACK * const me, QEvt const * const e)
{
QState ret;
   switch (e->sig)
   {
      case Q_ENTRY_SIG:
         {
				 SmallEvt *pe = Q_NEW(SmallEvt,FDISK_TEST_SIG);
         QACTIVE_POST(AO_Fdisk, &pe->super, me);
				 ret= Q_HANDLED();
         }
       break;
			case FD_ANSW_TEST_OK_SIG:
			   {
					OutDebugPacket("FD_ANSW_TEST_OK_SIG");
          if(me->flag.fram_error==true)ret= Q_TRAN(&Packet_fram_error);	
          else ret= Q_TRAN(&Packet_flash_disk_init);						 
			   }
			 break;
				 case FD_ANSW_TEST_ERR_SIG:
			   {
					OutDebugPacket("FD_ANSW_TEST_ERR_SIG");
          if(me->flag.fram_error==true)ret= Q_TRAN(&Packet_fram_and_flash_error);	
          else ret= Q_TRAN(&Packet_flash_error);						 
			   }
			 break;
			  case Q_EXIT_SIG:
			 {
				 ret= Q_HANDLED();
			 }
			 break;
			  default: ret= Q_SUPER(&Packet_top);
   }
   return ret;
}

QState Packet_flash_disk_init(PACK * const me, QEvt const * const e)
{
	QState ret;
   switch (e->sig)
   {
      case Q_ENTRY_SIG:
         {
				   QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1); 
					 ret= Q_HANDLED();
         }
       break;
			case PACKET_TIMEOUT_SIG:
			{
						 if (false==CopyRecordsRingBufImageToRam(&flash_disk_image))
                 {
									 OutDebugPacket("FLASH DISK IS VIRGIN or FAT IS CORRUPTED!!!");
                    flash_disk_image.head=flash_disk_image.tail=flash_disk_image.count=0;
                    SaveFatToFram(&flash_disk_image);
									  if (CopyRecordsRingBufImageToRam(&flash_disk_image)==true)OutDebugPacket("FORMAT DISK OK");                                        
                    else
                           {
                           OutDebugPacket("FORMAT DISK FAILURE!!!");
                           return  Q_TRAN(&Packet_fram_error);	   
                           }
								 }	 
						 if (false== ConfigInit(&config))	OutDebugPacket("CONFIG DEFAULT STORED!!!");
             else  OutDebugPacket("READ CONFIG OK"); 
					   SysInfo.points_total=FM25CL64_ReadWord(POINTS_TOTAL_ADDR);
						 if (true== FramRingBuf_Init(&points_ram_buf,FRAM_RINGBUF_SIZE))
               {
               OutDebugPacketSprintf1("FramRingBuf_size_used=",FramRingBuf_size_used());
							 OutDebugPacketSprintf1("FramRingBuf_size_to_flash=",FramRingBuf_size_to_flash());
							 OutDebugPacketSprintf1("FramRingBuf_size_to_send=",FramRingBuf_size_to_send());
               }
             else OutDebugPacket("FRAM DISK IS VIRGIN or FAT IS CORRUPTED!!! FORMAT OK");
             ret=  Q_TRAN(&Packet_waiting);	 
			}
			  break;
			   case Q_EXIT_SIG:
			 {
				 	QTimeEvt_disarm(&me->TimeEvt);
				 ret= Q_HANDLED();
			 }
			 break;
			  default: ret= Q_SUPER(&Packet_top);
   }
   return ret;
}


QState Packet_waiting(PACK * const me, QEvt const * const e)
{ 
	QState ret;
   switch (e->sig)
   {
      case Q_ENTRY_SIG:
			   {
			   }
			    return Q_HANDLED();
			 case TIC_100ms_SIG:
         {
						if ((me->flag.timed_send==true)&&DiskIsNotEmpty())
               {
                 ret= Q_TRAN(&Packet_modem_busy);  
               }
						else  ret= Q_SUPER(&Packet_top);
         }
         break;  
			 case Q_EXIT_SIG:
			     {
				    ret= Q_HANDLED();
			     }
			 break;
			  default: ret= Q_SUPER(&Packet_top);
   }
   return ret;
}

QState Packet_modem_busy(PACK * const me, QEvt const * const e)
{ 
	QState ret;
   switch (e->sig)
   {
      case Q_ENTRY_SIG:
			   {
					  me->send_status=not_sending;
					  SmallEvt *pe = Q_NEW(SmallEvt,MODEM_REQUEST_TO_SEND_SIG);
            QACTIVE_POST(AO_Modem, &pe->super, me);
			   }
			    return Q_HANDLED();
				 case PACK_MODEM_READY_SIG:
         {
            OutDebugPacket("MODEM_READY_SIG");
               if (flash_disk_image.count>0)
                  {
                   ret= Q_TRAN(&Packet_read_page); 										
                  }
               else if (FramRingBuf_size_used()>0)
                  {
                 ret= Q_TRAN(&Packet_send_fresh_packet);    
                  }
               else
                  {
                 ret= Q_TRAN(&Packet_waiting);  
                  }
         }
         return ret;
			 case Q_EXIT_SIG:
			     {
				    ret= Q_HANDLED();
			     }
			 break;
			  default: ret= Q_SUPER(&Packet_top);
   }
   return ret;
}

QState Packet_read_page(PACK * const me, QEvt const * const e)
{
		QState ret;
   switch (e->sig)
   {
		 case Q_ENTRY_SIG:
		       {
						 QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1); 
						 ret= Q_HANDLED();
		       }
			   break;
	case PACKET_TIMEOUT_SIG://check flag.spi_busy
         {
					   if(me->flag.fram_error==true) ret=Q_TRAN(&Packet_fram_error); 
             else if(me->flag.flash_error==true) ret=Q_TRAN(&Packet_flash_error);    					 
						 else if(me->flag.spi_busy==false)
							  {
									me->flag.spi_busy=true;
								  SmallEvt *pe = Q_NEW(SmallEvt,FDISK_READ_FROM_TAIL_SIG);
                  QACTIVE_POST(AO_Fdisk, &pe->super, me);
									ret= Q_HANDLED();
							  }
							else 
							{
								QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
								ret= Q_HANDLED();
							}
         }
         break; 
				  case FD_ANSW_READ_OK_SIG:
         {
            OutDebugPacket("FD_ANSW_READ_OK_SIG"); 
						me->flag.spi_busy=false;
					  if(me->flag.fram_error==true) ret=Q_TRAN(&Packet_fram_error); 
             else if(me->flag.flash_error==true) ret=Q_TRAN(&Packet_flash_error);  
					   else ret=Q_TRAN(&Packet_send_page);    
         }
         return ret;
     case FD_ANSW_READ_ERR_SIG:
         {
            OutDebugPacket("FD_ANSW_READ_ERR_SIG"); 
						me->flag.spi_busy=false;
            ret=Q_TRAN(&Packet_flash_error);                        
         }
         return ret ;	
      case FD_ANSW_PAGE_IS_BAD_SIG:
         {
            OutDebugPacket("FD_ANSW_PAGE_IS_BAD_SIG"); 
					// 	me->flag.spi_busy=false;//keep spi busy
           ret=Q_TRAN(&Packet_delete_bad_page);                        
         }
         return ret ;				 
      case Q_EXIT_SIG:
		      {
						QTimeEvt_disarm(&me->TimeEvt);
			     ret= Q_HANDLED();
		      }
			     break;
		 default: ret= Q_SUPER(&Packet_top);
					 break;
   }
   return ret;
}


QState Packet_send_page(PACK * const me, QEvt const * const e)
{
		QState ret;
   switch (e->sig)
   {
		 case Q_ENTRY_SIG:
		        {
            PtrEvt *pe = Q_NEW(PtrEvt, MODEM_SEND_DATA_SIG);
            pe->ptr=pack_buf;
					  pe->size= PageFmPacketMaker(pack_buf,&flash_page_buf);
            QACTIVE_POST(AO_Modem, &pe->super, me);
            me->send_status=sending; 
					  ret= Q_HANDLED();
		        }
			   break;
		case PACK_SEND_OK_SIG:
         {
            OutDebugPacket("PACK_SEND_OK_SIG");
            me->flag.timed_send=false;
            me->sendInterval_counter=0;
            me->send_status=send_ok;              
            if (me->flag.spi_busy==false)
               {
               int32_t res= DeletePageFromTail();
               if (res==-1) OutDebugPacket("THE PAGE WAS NOT DELETED BECAUSE THE DISK IS EMPTY!!!");
               else OutDebugPacketSprintf2("PAGE DELETE OK num=", " stored count=",res,flash_disk_image.count);
               if ((me->flag.timed_send==true)&& DiskIsNotEmpty())
                  {
                  ret=Q_TRAN(&Packet_modem_busy);   
                  }
               else  ret=Q_TRAN(&Packet_waiting); 
               }
         }
         return ret;
				  case PACK_PAGE_WRITE_COMPLETE_SIG:
         {
            if (me->send_status==sending)return Q_HANDLED();
            else if (me->send_status==send_ok)
               {
               int32_t res= DeletePageFromTail();
                if (res==-1) OutDebugPacket("THE PAGE WAS NOT DELETED - DISK IS EMPTY!!!");
               else OutDebugPacketSprintf2("PAGE DELETE OK num=", " stored count=",res,flash_disk_image.count);
               if ((me->flag.timed_send==true)&& DiskIsNotEmpty())
                  {
                  ret=Q_TRAN(&Packet_modem_busy);    
                  }
               else  ret=Q_TRAN(&Packet_waiting); 
               }
         }
         return ret;
		case PACK_SEND_ERROR_SIG:
         {
            me->send_status=send_error;
            OutDebugPacket("PACK_SEND_ERROR_SIG"); 
             ret=Q_TRAN(&Packet_modem_busy);   
         }
         return ret;
     case Q_EXIT_SIG: ret= Q_HANDLED();
			     break;
		 default: ret= Q_SUPER(&Packet_top);
					 break;
   }
   return ret;
}


QState Packet_send_fresh_packet(PACK * const me, QEvt const * const e)
{
		QState ret;
   switch (e->sig)
   {
		 case Q_ENTRY_SIG:
		 {
			 me->send_status=not_sending;
			  QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
			 ret= Q_HANDLED();
		 }
			   break;
		  case PACKET_TIMEOUT_SIG:
			{
				 if(me->send_status==not_sending&&me->flag.spi_busy==false)
				 {
				uint8_t points_to_send_count;
				int32_t packet_len= PointsFmPacketMaker(pack_buf,points_to_send_buf,POINTS_TO_SEND_BUF_SIZE,&points_to_send_count);
            if (-1==packet_len)
               {
                 OutDebugPacket("PointsPacketMaker ERROR!!!");
                ret=Q_TRAN(&Packet_fram_error);  
               }
            else
               {
               OutDebugPacketSprintf1("PointsPacketMaker OK points in packet=",points_to_send_count); 
                PtrEvt *pe = Q_NEW(PtrEvt, MODEM_SEND_DATA_SIG);
                pe->ptr=pack_buf;
					      pe->size=packet_len;
               QACTIVE_POST(AO_Modem, &pe->super, me);
               me->send_status=sending;
							 ret= Q_HANDLED();
               }
				 }
			}
			break;
    case Q_EXIT_SIG:
		      {
						QTimeEvt_disarm(&me->TimeEvt);
			     ret= Q_HANDLED();
		      }
			     break;
		 default: ret= Q_SUPER(&Packet_top);
					 break;
   }
   return ret;
}


QState Packet_delete_bad_page(PACK * const me, QEvt const * const e)
{
		QState ret;
   switch (e->sig)
   {
		 case Q_ENTRY_SIG: 
		     {
			 	   QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
			     ret= Q_HANDLED();
		     }
			   break;
		 case PACKET_TIMEOUT_SIG:
				{
						  int32_t res= DeletePageFromTail();
              if (res==-1)  OutDebugPacket("THE PAGE WAS NOT DELETED - DISK IS EMPTY!!!");
							else OutDebugPacketSprintf2("PAGE DELETE OK num=", " stored count=",res,flash_disk_image.count);
              if ((me->flag.timed_send==true)&& DiskIsNotEmpty())
                  {
                  ret=Q_TRAN(&Packet_modem_busy);    
                  }
               else  ret=Q_TRAN(&Packet_waiting); 
				}
				break;
     case Q_EXIT_SIG:
		      {
						QTimeEvt_disarm(&me->TimeEvt);
			     ret= Q_HANDLED();
		      }
			     break;
		 default: ret= Q_SUPER(&Packet_top);
					 break;
   }
   return ret;
}


QState Packet_fram_error(PACK * const me, QEvt const * const e)
{
		QState ret;
   switch (e->sig)
   {
		 case Q_ENTRY_SIG: ret= Q_HANDLED();
			   break;
     case Q_EXIT_SIG: ret= Q_HANDLED();
			     break;
		 default: ret= Q_SUPER(&Packet_top);
					 break;
   }
   return ret;
}


QState Packet_flash_error(PACK * const me, QEvt const * const e)
{
  QState ret;
   switch (e->sig)
   {
		  case Q_ENTRY_SIG: ret= Q_HANDLED();
			 break;
     case Q_EXIT_SIG: ret= Q_HANDLED();
			 break;
			 default: ret= Q_SUPER(&Packet_top);
			 break; 
   }
   return ret;
}

QState Packet_fram_and_flash_error(PACK * const me, QEvt const * const e)
{
  QState ret;
   switch (e->sig)
   {
		  case Q_ENTRY_SIG: ret= Q_HANDLED();
			   break;
     case Q_EXIT_SIG: ret= Q_HANDLED();
			 break;
			default: ret= Q_SUPER(&Packet_top);
		 break;
   }
   return ret;
}


bool CopyRecordsRingBufImageToRam(FlashPagesRingBufType *fptr)
    {
    FM25_ReadArray(PAGES_BUF_ADDR,
			(uint8_t*)fptr,
			sizeof(FlashPagesRingBufType));
    return(fptr->crc==Crc32Eth((uint8_t*)fptr,sizeof(FlashPagesRingBufType)-sizeof(fptr->crc)))?true:false;
    }

void SaveFatToFram(FlashPagesRingBufType *fptr)
    {
    fptr->crc=Crc32Eth((uint8_t*)fptr,sizeof(FlashPagesRingBufType)-sizeof(fptr->crc));
    FM25_WriteArray(PAGES_BUF_ADDR,(uint8_t*)fptr,sizeof(FlashPagesRingBufType));
    }
		
		
	bool ConfigInit(ConfigType *config_ptr)
    {
    if (ReadConfig()==true)return true;
    else//write config default
        {
        //--------------GPRS-----------------------------------------------
        strlcpy(config_ptr->gprs.apn, CONF_GPRS_APN, MAX_GPRS_STR_LEN);
        strlcpy(config_ptr->gprs.login, CONF_GPRS_LOGIN, MAX_GPRS_STR_LEN);
        strlcpy(config_ptr->gprs.pass, CONF_GPRS_PASS, MAX_GPRS_STR_LEN);
        //--------------Server-----------------------------------------------------
        strlcpy(config_ptr->main_server.ip, MAIN_SERVER_IP, MAX_SERVER_STR_LEN);
        config_ptr->main_server.port = MAIN_SERVER_PORT;
				//---------------STORE POINT----------------------------------------------
					config_ptr->pointStore.time=STORE_POINT_TIME_DEFAULT;
					config_ptr->pointStore.distance=STORE_POINT_DISTANCE_DEFAULT;
					config_ptr->pointStore.angle=STORE_POINT_ANGLE_DEFAULT;
					//--------------PACKET SEND----------------------------------------------
					config_ptr->packetsend.time=PACKET_SEND_TIME_DEFAULT;
				  config_ptr->packetsend.distance =PACKET_SEND_DISTANCE_DEFAULT;
        //--------------485 SENSORS------------------------------------------------
					config_ptr->rs485.id1=0;
					config_ptr->rs485.id2=0;
					config_ptr->rs485.id3=0;
					config_ptr->rs485.id4=0;
				//-------------------------------------------------------------------
        config_ptr->sensors_count=SENSORS_COUNT_DEF;
        config_ptr->sms_password=SMS_PASSWORD_DEF; 
         FM25CL64_WriteWord(POINTS_TOTAL_ADDR,0);
        			 
					//------------DEVICE NAME--------------------------------------------
				///strlcpy(config_ptr->device_name,DEVICE_NAME_DEFAULT,MAX_DEVICE_NAME_LEN);
        //-----------------------------------------------------------------
        config_ptr->crc=Crc32Eth((uint8_t*)config_ptr,sizeof(ConfigType)-sizeof(config_ptr->crc));
        FM25_WriteArray(CONFIG_ADDR,(uint8_t*)config_ptr,sizeof(ConfigType));
        return false;
        }
    }
		
		
		bool ReadConfig(void)
    {
    FM25_ReadArray(CONFIG_ADDR,(uint8_t*)&config,sizeof(ConfigType));
    return(config.crc==Crc32Eth((uint8_t*)&config,sizeof(ConfigType)-sizeof(config.crc)))? true:false;
    }
		

void WriteConfig(PACK *me)
   {
		if(me->flag.spi_busy==true)return;
    if (me->write_config_ptr==NULL)return;
      WriteConfigToFramAndSendRestart(me->write_config_ptr->restart);
      OutDebugPacket("WRITE CONFIG OK");
      me->write_config_ptr=NULL;
   }
	 
	 
	 	void WriteConfigToFramAndSendRestart(RestartFlagsType restart)
    {
   // static Msg modem_restart_evt={MODEM_RESTART_EVT};
    config.crc=Crc32Eth((uint8_t*)&config,sizeof(ConfigType)-sizeof(config.crc));
    FM25_WriteArray(CONFIG_ADDR,(uint8_t*)&config,sizeof(ConfigType));
		if(restart.modem)
		  {			
     // os_mbx_send (modem_task_mailbox,&modem_restart_evt,0xFFFF);
		  }
    }
		
void WritePoint(PACK *me)
   {
		if(me->flag.spi_busy==true)return;
    if (me->point_ptr==NULL)return;
      me->flag.fram_error= FramRingBuf_AddToHead(me->point_ptr);
      me->point_ptr=NULL;  
   }
	 
	 bool DiskIsNotEmpty(void)
   {
   if (flash_disk_image.count>0)return true;
   if (FramRingBuf_size_used()>0)return true;
   return false;
   }
	 
	 bool GetPointsFromTailToWriteBuf(void)//1-run time 30 ms
   {
   uint16_t free_space=FLASH_PAGE_ITEM_SIZE;
   if (CopyFramPointsRingBufImageToRam())
      {
      FramRingBuf_GetFromTailToFlashBufFast((uint8_t*)&flash_page_buf.points,&free_space,&flash_page_buf.points_count);
      WriteFramPointsRingBufImageToFram();
      return true;
      }
   else return false;
   }
	 
	 int32_t DeletePageFromTail(void)
    {
			int32_t ret;
    if (flash_disk_image.count)
        {
        ret=flash_disk_image.tail;
        if (++flash_disk_image.tail==FLASH_DISK_SIZE)flash_disk_image.tail=0;
        flash_disk_image.count--;
        SaveFatToFram(&flash_disk_image);
        }
		 else ret=-1;
				return ret;
    }




void OutDebugPacket( char const *dbg_msg)
   {
   QS_BEGIN(PACKET_DBG, AO_Packet)                                 
   QS_STR(dbg_msg);                              
   QS_END()
   }
	 
	 void OutDebugPacketSprintf1( char const *str,uint32_t val)
   {
   QS_BEGIN(PACKET_DBG, AO_Packet)                                  
   QS_STR(str); 
   QS_U32(4, val);  		 
   QS_END()
   }
	 
	  void OutDebugPacketSprintf2( char const *str1,char const *str2,uint32_t val1,uint32_t val2)
   {
   QS_BEGIN(PACKET_DBG, AO_Packet)                                  
   QS_STR(str1); 
   QS_U32(4, val1);
   QS_STR(str2); 
   QS_U32(4, val2);  			 
   QS_END()
   }
	 
	 void OutDebugPacketSprintf3( char const *str1,char const *str2,char const *str3,uint32_t val1,uint32_t val2,uint32_t val3)
   {
   QS_BEGIN(PACKET_DBG, AO_Packet)                                  
   QS_STR(str1); 
   QS_U32(4, val1);
   QS_STR(str2); 
   QS_U32(4, val2);
   QS_STR(str3); 
   QS_U32(4, val3);  		 
   QS_END()
   }
	 
	 uint32_t PageFmPacketMaker(uint8_t *buf_ptr,FlashPageType *page_ptr)
   {
   uint64_t temp_long;
   uint16_t temp_short,read_len_total;
   uint8_t *write_ptr,*read_ptr,*avl_ptr; 
   read_len_total=0;
   ((FM2200AvlPacketType*)buf_ptr)->zero=(uint32_t)0;
   ((FM2200AvlPacketType*)buf_ptr)->AvlDataArray.codecId=8;
   ((FM2200AvlPacketType*)buf_ptr)->AvlDataArray.dataCount=page_ptr->points_count;
   write_ptr=(uint8_t*)&((FM2200AvlPacketType*)buf_ptr)->AvlDataArray.data.time;//set write ptr to first AVL 
   for (int32_t i=0;i<page_ptr->points_count;i++)
      {
      read_ptr=read_len_total+page_ptr->points;
      temp_long= ((QPointType*)read_ptr)->gps.time;
      temp_long*=1000;
      LongToBigEndianStream(write_ptr,temp_long);
      write_ptr+=8;//set write_ptr to prio
      *write_ptr=0;//write prio
      write_ptr++;//set to lon
      IntToBigEndianStream(write_ptr,((QPointType*)read_ptr)->gps.lon_int);
      write_ptr+=4;//set to lat
      IntToBigEndianStream(write_ptr,((QPointType*)read_ptr)->gps.lat_int);
      write_ptr+=4;//set to alt
      ShortToBigEndianStream(write_ptr,((QPointType*)read_ptr)->gps.alt);
      write_ptr+=2;//set to angle
      ShortToBigEndianStream(write_ptr,((QPointType*)read_ptr)->gps.angle);
      write_ptr+=2;//set to sat
      *write_ptr=((QPointType*)read_ptr)->gps.sat_gsmq>>4;
      write_ptr++;//set to speed
      temp_short=(uint16_t)((QPointType*)read_ptr)->gps.speed;
      ShortToBigEndianStream(write_ptr,temp_short);//write speed
      write_ptr+=2;//set to EVENT IO ID
      *write_ptr=0;
      write_ptr++;//set to N of total IO
      //total_io=((QPointType*)read_ptr)->sensors_count;
      *write_ptr=1;//write total io
      write_ptr++;//set to N of one byte IO
      *write_ptr=0;//write N of one byte IO
      write_ptr++;//set to N of two bytes IO
      *write_ptr=1;//write N of two bytes IO
      write_ptr++;//set to ID of two byte IO
      if (((QPointType*)read_ptr)->gps.status.power_int==1)
         {
         *write_ptr=67;//write ID of two byte IO//accum
         }
      else
         {
         *write_ptr=66;//write ID of two byte IO//ext power
         }
      write_ptr++;//set to value of two byte IO
      temp_short= ((QPointType*)read_ptr)->gps.voltage; 
      ShortToBigEndianStream(write_ptr,temp_short);//write value of two byte IO (voltage)
      write_ptr+=2;//set to N of four bytes IO								
      *write_ptr=0;//write N of four bytes IO	
      write_ptr++;//set to N of eight bytes IO
      *write_ptr=0;//write N of eight bytes IO
      write_ptr++;//set to next position	
      read_len_total+=sizeof(QGpsDataType)+sizeof(uint8_t)+sizeof(QSensorType)*((QPointType*)read_ptr)->sensors_count;          
      }
   *write_ptr++= page_ptr->points_count;//END OF AVL ARRAY	
   avl_ptr=&((FM2200AvlPacketType*)buf_ptr)->AvlDataArray.codecId;        
   temp_short=(uint16_t)(write_ptr-avl_ptr);
   IntToBigEndianStream((uint8_t*)&((FM2200AvlPacketType*)buf_ptr)->len,temp_short);
   uint32_t crc= MakeCRC16((uint8_t*)&((FM2200AvlPacketType*)buf_ptr)->AvlDataArray.codecId,temp_short);   
   IntToBigEndianStream(write_ptr,crc);
   write_ptr+=4;
   return (write_ptr-buf_ptr);
   }
	 
	 
	 int32_t PointsFmPacketMaker(uint8_t *buf_ptr,uint8_t *points_to_send_buf_ptr,uint16_t points_to_send_buf_size,uint8_t* points_to_send_count)
   {
   uint64_t temp_long;
   uint8_t *write_ptr,*read_ptr,*avl_ptr; 
   uint16_t free_space,read_len_total,temp_short;
   free_space=points_to_send_buf_size;
   read_len_total=0;
   if (-1== FramRingBuf_CopyPointsFromTailToSendBuf(points_to_send_buf_ptr,&free_space,points_to_send_count))
      {
      return -1;
      }
   else
      {
      ((FM2200AvlPacketType*)buf_ptr)->zero=0;
      ((FM2200AvlPacketType*)buf_ptr)->AvlDataArray.codecId=8;
      ((FM2200AvlPacketType*)buf_ptr)->AvlDataArray.dataCount=*points_to_send_count;
      write_ptr=(uint8_t*)&((FM2200AvlPacketType*)buf_ptr)->AvlDataArray.data.time;//set write ptr to first AVL 
      for (int32_t i=0;i<*points_to_send_count;i++)
         {
         read_ptr=points_to_send_buf_ptr+read_len_total;
         temp_long= ((QPointType*)read_ptr)->gps.time;
         temp_long*=1000;
         LongToBigEndianStream(write_ptr,temp_long);
         write_ptr+=8;//set write_ptr to prio
         *write_ptr=0;//write prio
         write_ptr++;//set to lon
         IntToBigEndianStream(write_ptr,((QPointType*)read_ptr)->gps.lon_int);
         write_ptr+=4;//set to lat
         IntToBigEndianStream(write_ptr,((QPointType*)read_ptr)->gps.lat_int);
         write_ptr+=4;//set to alt
         ShortToBigEndianStream(write_ptr,((QPointType*)read_ptr)->gps.alt);
         write_ptr+=2;//set to angle
         ShortToBigEndianStream(write_ptr,((QPointType*)read_ptr)->gps.angle);
         write_ptr+=2;//set to sat
         *write_ptr=((QPointType*)read_ptr)->gps.sat_gsmq>>4;
         write_ptr++;//set to speed
         temp_short=(uint16_t)((QPointType*)read_ptr)->gps.speed;
         ShortToBigEndianStream(write_ptr,temp_short);//write speed
         write_ptr+=2;//set to EVENT IO ID
         *write_ptr=0;
         write_ptr++;//set to N of total IO
         //total_io=((QPointType*)read_ptr)->sensors_count;
         *write_ptr=1;//write total io
         write_ptr++;//set to N of one byte IO
         *write_ptr=0;//write N of one byte IO
         write_ptr++;//set to N of two bytes IO
         *write_ptr=1;//write N of two bytes IO
         write_ptr++;//set to ID of two byte IO
         if (((QPointType*)read_ptr)->gps.status.power_int==1)
            {
            *write_ptr=67;//write ID of two byte IO//accum
            }
         else
            {
            *write_ptr=66;//write ID of two byte IO//ext power
            }
         write_ptr++;//set to value of two byte IO
         temp_short= ((QPointType*)read_ptr)->gps.voltage; 
         ShortToBigEndianStream(write_ptr,temp_short);//write value of two byte IO (voltage)
         write_ptr+=2;//set to N of four bytes IO								
         *write_ptr=0;//write N of four bytes IO	
         write_ptr++;//set to N of eight bytes IO
         *write_ptr=0;//write N of eight bytes IO
         write_ptr++;//set to next position	
         read_len_total+=sizeof(QGpsDataType)+sizeof(uint8_t)+sizeof(QSensorType)*((QPointType*)read_ptr)->sensors_count;          
         }
      *write_ptr++= *points_to_send_count;//END OF AVL ARRAY	
      avl_ptr=&((FM2200AvlPacketType*)buf_ptr)->AvlDataArray.codecId;        
      temp_short=(uint16_t)(write_ptr-avl_ptr);
      IntToBigEndianStream((uint8_t*)&((FM2200AvlPacketType*)buf_ptr)->len,temp_short);
      uint32_t crc= MakeCRC16((uint8_t*)&((FM2200AvlPacketType*)buf_ptr)->AvlDataArray.codecId,temp_short);   
      IntToBigEndianStream(write_ptr,crc);
      write_ptr+=4;
       return write_ptr-buf_ptr;
      }
   }




