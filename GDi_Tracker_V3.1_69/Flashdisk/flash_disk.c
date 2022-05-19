

#include "qp_port.h"
#include "flash_disk.h"
#include "at45db.h"
#include <string.h>
#include "UserTypes.h"
#include "CRC32.h"
#include "packet.h"

typedef struct
{
   QActive super;
	 QTimeEvt TimeEvt;
	uint16_t err_timer;
	uint8_t status;
	FlashPageType *rw_ptr;
   
} FDISK;


static FDISK fdisk; 
QActive * const AO_Fdisk = &fdisk.super; 


static FlashPageType   page_image;
static QState Fdisk_initial(FDISK * const me, QEvt const * const e);
static QState Fdisk_idle(FDISK * const me, QEvt const * const e);
static QState Fdisk_test(FDISK * const me, QEvt const * const e);



void Fdisk_ctor(void)
{
   FDISK *me = &fdisk;
	 QTimeEvt_ctor(&me->TimeEvt,  FDISK_TIMEOUT_SIG);
   QActive_ctor(&me->super, Q_STATE_CAST(&Fdisk_initial));
}


QState Fdisk_initial(FDISK * const me, QEvt const * const e)
{
	 QS_OBJ_DICTIONARY(&fdisk);
  QS_FUN_DICTIONARY(&Fdisk_initial);
	 QS_FUN_DICTIONARY(&Fdisk_idle);
  
	// QS_FILTER_SM_OBJ(&gparser);
   return Q_TRAN(&Fdisk_idle);
}


QState Fdisk_idle(FDISK * const me, QEvt const * const e)
{
   switch (e->sig)
   {
     case Q_ENTRY_SIG: return Q_HANDLED();
		 case FDISK_TEST_SIG: return Q_TRAN(&Fdisk_test);
	   case Q_EXIT_SIG: return Q_HANDLED();
   }
   return Q_SUPER(&QHsm_top);
}

QState Fdisk_test(FDISK * const me, QEvt const * const e)
{
	 static enum
        {
        write_page,//0
        compare_page,//1
        check_page,//2
        }state;
   switch (e->sig)
   {
    case Q_ENTRY_SIG:
		    {
			   QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
         me->err_timer=3;
         state=write_page;
		    }
			return Q_HANDLED();
		 case FDISK_TIMEOUT_SIG:
            {
                me->err_timer--;
                me->status= ReadDfStatus ();
                if (DF_READY(me->status)==true)
                    {
                    switch (state)
                        {
                        case write_page:
                            {
                                memcpy((uint8_t*)&page_image ,(uint8_t*)me->rw_ptr,DF_PAGE_SIZE);
                                page_image.crc= Crc32Eth((uint8_t*)&page_image.points_count ,sizeof(page_image)-sizeof(page_image.crc));
                                WritePageImageToBuf2 ((uint8_t*)&page_image);
                                WriteBuf2ToPageWithErase(FLASH_DISK_SIZE);
                                QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
                                me->err_timer=5;
                                state=compare_page;
                            }
                            break;
                        case compare_page:
                            {
                                Buf2ComparePage (FLASH_DISK_SIZE);
                                QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
                                me->err_timer=5;
                                state=check_page;
                            }
                            break;
                        case check_page:
                            {
															  SmallEvt *pe;
                                if (DF_COMPARE(me->status)==false) pe = Q_NEW(SmallEvt,FD_ANSW_TEST_ERR_SIG);
                                else pe = Q_NEW(SmallEvt,FD_ANSW_TEST_OK_SIG);
                                QACTIVE_POST(AO_Packet, &pe->super, me);    
                            }
                             return Q_TRAN(&Fdisk_idle);
                        }
                    }
                else if (me->err_timer==0)
                    {
											  SmallEvt *pe=Q_NEW(SmallEvt,FD_ANSW_TEST_ERR_SIG);
                        QACTIVE_POST(AO_Packet, &pe->super, me); 
											  return Q_TRAN(&Fdisk_idle);
                    }
                else  QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
            }
            return Q_HANDLED() ;
	  case Q_EXIT_SIG: return Q_HANDLED();
   }
   return Q_SUPER(&QHsm_top);
}









