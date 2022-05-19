

#include "stm32f10x.h"
#include "qp_port.h"
#include "Disk.h"
#include "at45db.h"
#include "fm25cl64.h"
#include <string.h>
#include "user_types.h"
#include "CRC32.h"
#include "CRC16.h"
#include "Control.h"
#include "config.h"
#include "FramRingBuf.h"
#include "memory_spi.h"
#include "ByteStream.h"
#include "modem_signals.h"
#include "modem.h"
#include "io.h"
#include "crc8.h"
#include "LedIndicator.h"
#include "crc16_teltonika.h"

#define WRITE 1
#define READ  0
#define EXTERNAL_POWER_VOLTAGE_ID   66
#define INTERNAL_POWER_VOLTAGE_ID   67
#define FUEL_LEVEL1_PARAM_ID   201
#define FUEL_LEVEL2_PARAM_ID   203
#define RF10_PARAM_ID          207
#define TL10_PARAM_ID          132
#define TELTONIKA_DIN1_ID      1
#define TELTONIKA_DIN2_ID      2
#define TELTONIKA_FIN1_ID      24
#define TELTONIKA_FIN2_ID      76
//#define TEST_DISK
#ifdef TEST_DISK
    #warning "TEST DISK"
#endif


typedef struct
    {
    QActive super;
    QTimeEvt TimeEvt;
    uint16_t err_timer;
    uint8_t status;
    QPointType* point_ptr;
    QEQueue requestQueue;    /* native QF queue for deferred request events */
    QEvt const *requestQSto[DISK_QUEUE_SIZE];      /* storage for deferred queue buffer */
    } DISK;


static DISK disk; 
QActive * const AO_Disk = &disk.super; 
uint8_t send_pack_buf[PACKET_BUF_SIZE];
int write_point_angle_evt_flag=0;
QEvt const DiskSaveConfigEvt =           { DISK_SAVE_CONFIG_SIG, 0U, 0U};
static QEvt const ControlDiskIsOffEvt = { CONTROL_DISK_IS_OFF_SIG, 0U, 0U};
//static QEvt const ControlConfigWriteOkEvt = { CONTROL_CONFIG_WRITE_OK_SIG, 0U, 0U};
//static QEvt const FlashErrorEvt = { CONTROL_FLASH_ERROR_SIG, 0U, 0U};
static QEvt const DiskReadyEvt =          { CONTROL_DISK_READY_SIG, 0U, 0U};
static QEvt const PrioSendEvt =      { CONTROL_PRIO_SEND_SIG, 0U, 0U};
static QEvt const MakePacketErrorEvt = { CONTROL_MAKE_PACKET_ERROR_SIG, 0U, 0U};
FlashPagesRingBufType flash_disk_image;//records ring buf image
FramPointsRingBufType points_ram_buf; 
static uint8_t points_to_send_buf[POINTS_TO_SEND_BUF_SIZE];
static FlashPageType page_image;
static uint32_t PageNTPacketMaker(uint8_t *buf_ptr,FlashPageType *page_ptr);
static uint32_t PageFMPacketMaker(uint8_t *buf_ptr,FlashPageType *page_ptr);
static int32_t PointsNTPacketMaker(uint8_t *buf_ptr,uint8_t *points_to_send_buf_ptr,uint16_t points_to_send_buf_size,uint8_t* points_to_send_count);
static int32_t PointsFMPacketMaker(uint8_t *buf_ptr,uint8_t *points_to_send_buf_ptr,uint16_t points_to_send_buf_size,uint8_t* points_to_send_count);
static int GetPointsFromTailToWritePageBuf(FlashPageType* page_image_ptr);
static int LoadFlashDiskImage(FlashPagesRingBufType *flash_disk_image_ptr);
static int ResetDiskImage(FlashPagesRingBufType *ptr);
static int PageImageCrcOk(FlashPageType *ptr);
static int ConfigReadWriteGprs(int write,uint8_t);
static int ConfigReadWriteServer(int write,uint8_t);
static int ConfigReadWriteNavFilter(int write,uint8_t);
static int ConfigReadWritePacketSend(int write,uint8_t);
static int LogReadWriteCounter(int write,uint8_t);
static int ConfigReadWriteRS485Sensor(int write,uint8_t);
static int ConfigReadWriteSecur(int write,uint8_t);
static int ConfigReadWriteHardware(int write,uint8_t unused);
static int ConfigReadWriteFin(int write,uint8_t unused);
static int ConfigReadWriteDout(int write,uint8_t index);
static void SaveFatToFram(FlashPagesRingBufType *fptr);
static uint32_t ConfigSave(void);
static uint8_t ConfigRead(void);

//---------------------------------------------------------------------------------
#ifdef DEBUG_DISK	 
static void OutDebugDisk( char const *dbg_msg);
static void OutDebugDiskSprintf1( char const *str,uint32_t val);
//static void OutDebugDiskSprintf2( char const *str1,char const *str2,uint32_t val1,uint32_t val2);
static void OutDebugDiskSprintf3( char const *str1,char const *str2,char const *str3,uint32_t val1,uint32_t val2,uint32_t val3);
static void OutDebugDiskSprintf4( char const *str1,char const *str2,char const *str3,char const *str4,uint32_t val1,uint32_t val2,uint32_t val3,uint32_t val4);
#else
    #define OutDebugDisk(x) __nop()
    #define OutDebugDiskSprintf1(x1,x2) __nop()
    #define OutDebugDiskSprintf3(x1,x2,x3,x4,x5,x6) __nop()
    #define OutDebugDiskSprintf4(x1,x2,x3,x4,x5,x6,x7,x8) __nop()
#endif
//-------------------------------------------------------------------------
static QState FdiskInitial0(DISK * const me, QEvt const * const e);
static QState FdiskTop1(DISK* const me, QEvt const * const e);
static QState FdiskInit2(DISK * const me, QEvt const * const e);
static QState FdiskIdle3(DISK * const me, QEvt const * const e);
static QState FdiskReadConfig5(DISK * const me, QEvt const * const e);
static QState FdiskWrite7(DISK * const me, QEvt const * const e);
static QState FdiskMakeArchPack8(DISK * const me, QEvt const * const e);
static QState FdiskMakeFreshPack9(DISK * const me, QEvt const * const e);
static QState FdiskDelPage10(DISK * const me, QEvt const * const e);
static QState FdiskDelToSend11(DISK * const me, QEvt const * const e);
static QState FdiskUnlToSend12(DISK * const me, QEvt const * const e);
static QState FdiskFramErr13(DISK * const me, QEvt const * const e);
static QState FdiskFlashErr14(DISK * const me, QEvt const * const e);
static QState FdiskSaveConfig15(DISK * const me, QEvt const * const e);
static QState FdiskOff16(DISK * const me, QEvt const * const e);
static QState FdiskFormatFlash(DISK * const me, QEvt const * const e);
static QState FdiskCheck(DISK * const me, QEvt const * const e);


void Fdisk_ctor(void)
{
    DISK *me = &disk;
    QEQueue_init(&me->requestQueue,me->requestQSto, Q_DIM(me->requestQSto));
    QTimeEvt_ctor(&me->TimeEvt,  DISK_TIMEOUT_SIG);
    QActive_ctor(&me->super, Q_STATE_CAST(&FdiskInitial0));
}


QState FdiskInitial0(DISK * const me, QEvt const * const e)
{
    QS_OBJ_DICTIONARY(&disk);
    QS_FUN_DICTIONARY(&FdiskInitial0);
    QS_FUN_DICTIONARY(&FdiskTop1);
    QS_FUN_DICTIONARY(&FdiskInit2);
    QS_FUN_DICTIONARY(&FdiskIdle3);
    QS_FUN_DICTIONARY(&FdiskReadConfig5);
    QS_FUN_DICTIONARY(&FdiskWrite7);
    QS_FUN_DICTIONARY(&FdiskMakeArchPack8);
    QS_FUN_DICTIONARY(&FdiskMakeFreshPack9);
    QS_FUN_DICTIONARY(&FdiskDelPage10);
    QS_FUN_DICTIONARY(&FdiskDelToSend11);
    QS_FUN_DICTIONARY(&FdiskUnlToSend12);
    QS_FUN_DICTIONARY(&FdiskFramErr13);
    QS_FUN_DICTIONARY(&FdiskFlashErr14);
    QS_FUN_DICTIONARY(&FdiskSaveConfig15);
    QS_FUN_DICTIONARY(&FdiskOff16);
    QS_FUN_DICTIONARY(&FdiskFormatFlash);

    //QS_FILTER_SM_OBJ(&disk);
    FM25_CS_init();
    AT45_CS_init();
    MemorySpi_init();
    return Q_TRAN(&FdiskTop1);
}

QState FdiskTop1(DISK* const me, QEvt const * const e)
{
    switch (e->sig)
        {
        case Q_ENTRY_SIG: return Q_HANDLED();
        case DISK_OFF_SIG:
            {
                if (QActive_defer((QActive *)me, &me->requestQueue, e)) OutDebugDisk("DISK_OFF_SIG deferred");
                else  OutDebugDisk("DISK_OFF_SIG ignored"); 
            }
            return Q_HANDLED();
        case DISK_POINT_SIG: 
            {
                if (QActive_defer((QActive *)me, &me->requestQueue, e)) OutDebugDisk("POINT_SIG deferred");
                else  OutDebugDisk("POINT_SIG ignored"); 
            }
            return Q_HANDLED();
        case DISK_MAKE_FRESH_PACK_SIG:
            {
                if (QActive_defer((QActive *)me, &me->requestQueue, e)) OutDebugDisk("MAKE_FRESH_PACK_SIG deferred");
                else  OutDebugDisk("MAKE_FRESH_PACK_SIG ignored");
            }
            return Q_HANDLED();
        case DISK_MAKE_ARCH_PACK_SIG:
            {
                if (QActive_defer((QActive *)me, &me->requestQueue, e)) OutDebugDisk("MAKE_ARCH_PACK_SIG deferred");
                else  OutDebugDisk("MAKE_ARCH_PACK_SIG ignored");
            }
            return Q_HANDLED();
        case DISK_DELETE_PAGE_SIG: 
            {
                if (QActive_defer((QActive *)me, &me->requestQueue, e)) OutDebugDisk("DELETE_PAGE_SIG deferred");
                else  OutDebugDisk("DELETE_PAGE_SIG ignored");
            }
            return Q_HANDLED();        
        case DISK_SAVE_CONFIG_SIG:
            {
                if (QActive_defer((QActive *)me, &me->requestQueue, e)) OutDebugDisk("SAVE_CONFIG_SIG deferred");
                else  OutDebugDisk("SAVE_CONFIG_SIG ignored");
            }
            return Q_HANDLED();
        case DISK_READ_CONFIG_SIG:
            {
                if (QActive_defer((QActive *)me, &me->requestQueue, e)) OutDebugDisk("READ_CONFIG_SIG deferred");
                else  OutDebugDisk("READ_CONFIG_SIG ignored");
            }
            return Q_HANDLED();
        case DISK_DELETE_POINTS_TO_SEND_SIG:
            {
                if (QActive_defer((QActive *)me, &me->requestQueue, e)) OutDebugDisk("DELETE_POINTS_TO_SEND_SIG deferred");
                else  OutDebugDisk("DELETE_POINTS_TO_SEND_SIG ignored");
            }
            return Q_HANDLED();
        case DISK_UNLOCK_POINTS_TO_SEND_SIG:
            {
                if (QActive_defer((QActive *)me, &me->requestQueue, e)) OutDebugDisk("UNLOCK_POINTS_TO_SEND_SIG deferred");
                else  OutDebugDisk("UNLOCK_POINTS_TO_SEND_SIG ignored");
            }
            return Q_HANDLED();
        case Q_INIT_SIG: return Q_TRAN(&FdiskInit2);
        case Q_EXIT_SIG: return Q_HANDLED();
        }
    return  Q_SUPER(&QHsm_top);
}

QState FdiskInit2(DISK * const me, QEvt const * const e)
{
    static enum
        {
        fram_ring_buf_init, 
        load_image,
        waiting_df_ready,
        }state;
    switch (e->sig)
        {
        case Q_ENTRY_SIG:
            {
                QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,10); 
                state=fram_ring_buf_init;
            }
            return Q_HANDLED();
        case DISK_TIMEOUT_SIG:
            {
                switch (state)
                    {
                    case fram_ring_buf_init:
                        {
                            /*ResetDiskImage(&flash_disk_image);
                            for(;;){}*/
                            uint8_t param_stored_default;
//                     SysInfo.points_total=FM25CL64_ReadWord(POINTS_TOTAL_ADDR);
                            int32_t result= FramRingBuf_Init(&points_ram_buf,FRAM_RINGBUF_SIZE);
                            if (result== 1)
                                {
                                OutDebugDisk("FRAM DISK IS VIRGIN or FAT IS CORRUPTED!!! FORMAT OK");
                                param_stored_default= ConfigRead();
                                if (0!= param_stored_default)OutDebugDiskSprintf1("param_stored_default=",param_stored_default);
                                else OutDebugDisk("READ CONFIG OK"); 
                                QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1); 
                                state=load_image;                         
                                }
                            else if (result== -1)
                                {
                                SysInfo.msg_ptr.fram="FramRingBuf_Init FATAL ERROR";
                                return Q_TRAN(&FdiskFramErr13);       
                                }
                            else //OK
                                {
                                param_stored_default= ConfigRead();
                                //SysInfo.err_msg_ptr.fram="FRAM ERROR TEST";
                                //return Q_TRAN(&FdiskFramErr13);       
                                if (0!= param_stored_default)OutDebugDiskSprintf1("param_stored_default=",param_stored_default);
                                else OutDebugDisk("READ CONFIG OK"); 
                                QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1); 
                                state=load_image;
                                }
                        }
                        break;
                    case load_image:
                        {
                            if (1==LoadFlashDiskImage(&flash_disk_image))
                                {
                                if (flash_disk_image.count>FLASH_DISK_SIZE) return Q_TRAN(&FdiskFormatFlash);
                                me->err_timer=3;
                                QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1); 
                                OutDebugDisk("DISK waiting_df_ready");
                                state=waiting_df_ready;
                                }
                            else    return Q_TRAN(&FdiskFormatFlash);
                        }
                        break;
                    case waiting_df_ready:
                        {
                            me->err_timer--;
                            OutDebugDiskSprintf1("err_timer=",me->err_timer);
                            me->status= ReadDfStatus ();
                            if (DF_READY(me->status)==1)
                                {
                                OutDebugDiskSprintf4("DISK INIT OK err_timer="," count="," head="," tail=",me->err_timer,flash_disk_image.count,flash_disk_image.head,flash_disk_image.tail);
                                FillBuf1 (0xFF);
                                QACTIVE_POST(AO_Control,&DiskReadyEvt, me);
                                return Q_TRAN(&FdiskIdle3);
                                }
                            else if (me->err_timer>0)QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
                            else
                                {
                                SysInfo.msg_ptr.flash="FdiskInit2 waiting_df_ready TIMEOUT ERROR";            
                                return Q_TRAN(&FdiskFlashErr14);
                                }
                        }
                        break;
                    }
            }
            return Q_HANDLED() ;
        case Q_EXIT_SIG: return Q_HANDLED();
        }
    return Q_SUPER(&FdiskTop1);
}


QState FdiskIdle3(DISK * const me, QEvt const * const e)
{
    switch (e->sig)
        {
        case Q_ENTRY_SIG:
            {
                if (QActive_recall((QActive *)me, &me->requestQueue)) OutDebugDisk("Request recalled");
                else  OutDebugDisk("No deferred requests");
            }
            return Q_HANDLED();
        case DISK_OFF_SIG:                   return Q_TRAN(&FdiskOff16);
        case DISK_UNLOCK_POINTS_TO_SEND_SIG: return Q_TRAN(&FdiskUnlToSend12);
        case DISK_DELETE_POINTS_TO_SEND_SIG: return Q_TRAN(&FdiskDelToSend11);
        case DISK_DELETE_PAGE_SIG:           return Q_TRAN(&FdiskDelPage10);
        case DISK_MAKE_ARCH_PACK_SIG:        return Q_TRAN(&FdiskMakeArchPack8);
        case DISK_MAKE_FRESH_PACK_SIG:       return Q_TRAN(&FdiskMakeFreshPack9);
        case DISK_READ_CONFIG_SIG:           return Q_TRAN(&FdiskReadConfig5);  
        case DISK_SAVE_CONFIG_SIG:           return Q_TRAN(&FdiskSaveConfig15);           
        case DISK_POINT_SIG: 
            {
                DataPtrEvt *pe=(DataPtrEvt*)e;
                me->point_ptr=pe->ptr;
                if (pe->data.u32_data>conf.packet_send.prio_filter)
                    {
                    QACTIVE_POST(AO_Control,&PrioSendEvt, me);
                    }
            }
            return Q_TRAN(&FdiskWrite7);
        case Q_EXIT_SIG:                     return Q_HANDLED();
        }
    return Q_SUPER(&FdiskTop1);
}


QState FdiskSaveConfig15(DISK * const me, QEvt const * const e)
{
    switch (e->sig)
        {
        case Q_ENTRY_SIG:
            {
                QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1); 
            }
            return Q_HANDLED();
        case DISK_TIMEOUT_SIG:
            {
                uint8_t param_saved= ConfigSave();
                OutDebugDiskSprintf1("DIISK:: param_saved=",param_saved);
            }
            return Q_TRAN(&FdiskIdle3);
        case Q_EXIT_SIG:  return Q_HANDLED();
        }
    return Q_SUPER(&FdiskTop1);
}


QState FdiskReadConfig5(DISK * const me, QEvt const * const e)
{
    switch (e->sig)
        {
        case Q_ENTRY_SIG:
            {
                QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1); 
            }
            return Q_HANDLED();
        case DISK_TIMEOUT_SIG:
            {
                uint8_t param_stored_default=ConfigRead();
                if (0!= param_stored_default)OutDebugDiskSprintf1("param_stored_default=",param_stored_default);
                else  OutDebugDisk("READ CONFIG OK");
               // QACTIVE_POST(AO_Control,&ControlConfigWriteOkEvt, me);
            }
            return Q_TRAN(&FdiskIdle3);
        case Q_EXIT_SIG:  return Q_HANDLED();
        }
    return Q_SUPER(&FdiskTop1);
}

QState FdiskWrite7(DISK * const me, QEvt const * const e)
{
    static enum
        {
        write_point,
        write_page,
        check_record,
        }state;
    switch (e->sig)
        {
        case Q_ENTRY_SIG:
            {
                QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
                me->err_timer=20;
                state=write_point;       
            }
            return Q_HANDLED();
        case DISK_TIMEOUT_SIG:
            {
                me->err_timer--;
                switch (state)
                    {
                    case write_point:
                        {
                            if (0==FramRingBuf_AddToHead(me->point_ptr))
                                {
                                SysInfo.msg_ptr.fram="FramRingBuf_AddToHead ERROR";   
                                return Q_TRAN(&FdiskFramErr13);
                                }
                            else
                                {
                                // SysInfo.points_total++;
                                if (me->point_ptr->gps.status.angle)write_point_angle_evt_flag=1;
                                else ind_flag.write_point=1;
                                OutDebugDiskSprintf1("FramRingBuf_size_to_flash=",FramRingBuf_size_to_flash());
                                if (FramRingBuf_size_to_flash()>FRAM_RINGBUF_SIZE)
                                    {
                                    if (1==FramRingBuf_Format()) return Q_TRAN(&FdiskIdle3);
                                    else  return Q_TRAN(&FdiskFramErr13);
                                    }
                                if (FramRingBuf_size_to_flash()>DF_PAGE_SIZE)
                                    {
                                    int result=GetPointsFromTailToWritePageBuf(&page_image);
                                    if (result==1)
                                        {
                                        page_image.crc= Crc32Eth((uint8_t*)&page_image.points_count ,sizeof(page_image)-sizeof(page_image.crc));
                                        WritePageImageToBuf2 ((uint8_t*)&page_image);
                                        WriteBuf2ToPageWithErase(flash_disk_image.head);
                                        QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
                                        state=write_page;
                                        }
                                    else
                                        {
                                        //me->fram_error_num=FRAM_ERR_2;
                                        SysInfo.msg_ptr.fram="GetPointsFromTailToWritePageBuf ERROR"; 
                                        return Q_TRAN(&FdiskFramErr13);
                                        }
                                    }
                                else  return Q_TRAN(&FdiskIdle3);
                                }
                        }
                        break;
                    case write_page:
                        {
                            me->status= ReadDfStatus ();
                            if (DF_READY(me->status)==1)
                                {
                                Buf2ComparePage (flash_disk_image.head);
                                QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
                                state=check_record;
                                }
                            else if (me->err_timer>0)QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
                            else
                                {
                                SysInfo.msg_ptr.flash="WRITE PAGE TIMEOUT ERROR";  
                                return Q_TRAN(&FdiskFlashErr14);
                                }
                        }
                        break;
                    case check_record:
                        {
                            me->status= ReadDfStatus ();
                            if (DF_READY(me->status)==1)
                                {
                                if (DF_COMPARE(me->status)==0)
                                    {
                                    SysInfo.msg_ptr.flash="WRITE PAGE COMPARE ERROR";   
                                    return Q_TRAN(&FdiskFlashErr14);
                                    }
                                else
                                    {
#ifdef DEBUG_DISK	 
                                    uint16_t write_page_num=flash_disk_image.head;
#endif
                                    if (++flash_disk_image.head==FLASH_DISK_SIZE)flash_disk_image.head=0;
                                    if (flash_disk_image.head==flash_disk_image.tail)//disk is full
                                        {
                                        if (++flash_disk_image.tail==FLASH_DISK_SIZE)flash_disk_image.tail=0;//getout one item
                                        }
                                    else  flash_disk_image.count++;
                                    SaveFatToFram(&flash_disk_image);
                                    OutDebugDiskSprintf3("WRITING OK err_timer="," write_page_num="," stored count=",me->err_timer, write_page_num,flash_disk_image.count); 
                                    return Q_TRAN(&FdiskIdle3); 
                                    }
                                }
                            else if (me->err_timer>0)QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
                            else
                                {
                                SysInfo.msg_ptr.flash="WRITE PAGE COMPARE TIMEOUT ERROR"; 
                                return Q_TRAN(&FdiskFlashErr14);
                                }
                        }
                        break;
                    }
            }
            return Q_HANDLED();
        case Q_EXIT_SIG:  return Q_HANDLED();
        }
    return Q_SUPER(&FdiskTop1);
}

QState FdiskMakeArchPack8(DISK * const me, QEvt const * const e)
{
    QState ret=Q_HANDLED();
    switch (e->sig)
        {
        case Q_ENTRY_SIG:
            {
                ReadPageDirect((uint8_t*)&page_image,flash_disk_image.tail);
                QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1); 
                me->err_timer=10;
            }
            return Q_HANDLED();
        case DISK_TIMEOUT_SIG:
            {
                me->err_timer--;
                me->status= ReadDfStatus ();
                if (DF_READY(me->status)==1)
                    {
                    if (PageImageCrcOk(&page_image)==1)
                        {
                        OutDebugDiskSprintf3("READ PAGE OK, err_timer="," page num="," stored count=",me->err_timer, flash_disk_image.tail,flash_disk_image.count);
                        DataPtrEvt *pe = Q_NEW(DataPtrEvt,MODEM_DATA_PACKET_SIG);
                        pe->ptr=send_pack_buf;
                        if (conf.secur.protocol==PROTOCOL_NTRACK) pe->data.size=PageNTPacketMaker(send_pack_buf,&page_image);
                        else  pe->data.size=PageFMPacketMaker(send_pack_buf,&page_image);                       
                        QACTIVE_POST(AO_Modem,&pe->super, me); 
                        ret= Q_TRAN(&FdiskIdle3);                             
                        }
                    else
                        {
                        OutDebugDiskSprintf1("CRC WRONG PAGE num=",flash_disk_image.tail);
                        QACTIVE_POST(AO_Control,&MakePacketErrorEvt, me);
                        ret= Q_TRAN(&FdiskCheck);
                        } 
                    }
                else if (me->err_timer>0)
                    {
                    QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
                    ret=Q_HANDLED();
                    }
                else
                    {
                    SysInfo.msg_ptr.flash="READ PAGE TIMEOUT ERROR";   
                    ret= Q_TRAN(&FdiskFlashErr14);
                    }
            }
            return ret;
        case Q_EXIT_SIG:  return Q_HANDLED();
        }
    return Q_SUPER(&FdiskTop1);
}

/* QState FdiskReadFlashConfig(DISK * const me, QEvt const * const e)
{
switch (e->sig)
  {
  case Q_ENTRY_SIG:
     {
        ReadPageDirect((uint8_t*)&page_image,FLASH_CONFIG_PAGE_NUM       );
        QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1); 
        me->err_timer=10;
     }
     return Q_HANDLED();
  case DISK_TIMEOUT_SIG:
     {
        me->err_timer--;
        me->status= ReadDfStatus ();
        if (DF_READY(me->status)==1)
           {
           if (PageImageCrcOk(&page_image)==1)
              {
              OutDebugDiskSprintf3("READ PAGE OK, err_timer="," page num="," stored count=",me->err_timer, flash_disk_image.tail,flash_disk_image.count);
              DataPtrEvt *pe = Q_NEW(DataPtrEvt,MODEM_DATA_PACKET_SIG);
              pe->ptr=send_pack_buf;
              if (conf.secur.protocol==PROTOCOL_NTRACK) pe->data.size=PageNTPacketMaker(send_pack_buf,&page_image);
              else  pe->data.size=PageFMPacketMaker(send_pack_buf,&page_image);                       
              QACTIVE_POST(AO_Modem,&pe->super, me);                          
              }
           else
              {
              OutDebugDiskSprintf1("CRC WRONG PAGE num=",flash_disk_image.tail);
              if (++flash_disk_image.tail==FLASH_DISK_SIZE)flash_disk_image.tail=0;
              flash_disk_image.count--;
              SaveFatToFram(&flash_disk_image);
              OutDebugDisk("BAD PAGE DELETED");
              QACTIVE_POST(AO_Control,&MakePacketErrorEvt, me);
              } 
           return Q_TRAN(&FdiskIdle3);
           }
        else if (me->err_timer>0)QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
        else
           {
           QACTIVE_POST(AO_Control,&FlashErrorEvt, me);
           return Q_TRAN(&FdiskIdle3);
           }
     }
     return Q_HANDLED();
  case Q_EXIT_SIG:  return Q_HANDLED();
  }
return Q_SUPER(&FdiskTop1);
}*/

QState FdiskMakeFreshPack9(DISK * const me, QEvt const * const e)
{
    switch (e->sig)
        {
        case Q_ENTRY_SIG:
            {
                QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1); 
            }
            return Q_HANDLED();
        case DISK_TIMEOUT_SIG:
            {
                int32_t packet_len;
                uint8_t points_to_send_count;
                if (conf.secur.protocol==PROTOCOL_NTRACK)packet_len=  PointsNTPacketMaker(send_pack_buf,points_to_send_buf,POINTS_TO_SEND_BUF_SIZE,&points_to_send_count);
                else packet_len= PointsFMPacketMaker(send_pack_buf,points_to_send_buf,POINTS_TO_SEND_BUF_SIZE,&points_to_send_count);
                if (-1==packet_len)
                    {
                    OutDebugDisk("PointsPacketMaker ERROR!!!");
                    //me->fram_error_num=FRAM_ERR_3;
                    SysInfo.msg_ptr.fram="PointsPacketMaker ERROR";
                    return Q_TRAN(&FdiskFramErr13);
                    }
                else
                    {
                    OutDebugDiskSprintf1("PointsPacketMaker OK points in packet=",points_to_send_count); 
                    DataPtrEvt *pe = Q_NEW(DataPtrEvt,MODEM_DATA_PACKET_SIG);
                    pe->ptr=send_pack_buf;
                    pe->data.size=packet_len;  
                    QACTIVE_POST(AO_Modem,&pe->super, me);    
                    }
            }
            return Q_TRAN(&FdiskIdle3); 
        case Q_EXIT_SIG:  return Q_HANDLED();
        }
    return Q_SUPER(&FdiskTop1);
}


QState FdiskDelPage10(DISK * const me, QEvt const * const e)
{
    static enum
        {
        erase,
        check,
        }state;
    switch (e->sig)
        {
        case Q_ENTRY_SIG:
            {
                QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
                ErasingPage (flash_disk_image.tail);
                me->err_timer=20;
                state=erase;
            }
            return Q_HANDLED();
        case DISK_TIMEOUT_SIG:
            {
                me->err_timer--;
                switch (state)
                    {
                    case erase:
                        {
                            me->status= ReadDfStatus ();
                            if (DF_READY(me->status)==1)
                                {
                                Buf1ComparePage (flash_disk_image.tail);
                                QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
                                state=check;
                                }
                            else if (me->err_timer>0)QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
                            else
                                {
                                SysInfo.msg_ptr.flash="DELETE PAGE TIMEOUT ERROR"; 
                                return Q_TRAN(&FdiskFlashErr14);
                                }
                        }
                        break;
                    case check:
                        {
                            me->status= ReadDfStatus ();
                            if (DF_READY(me->status)==1)
                                {
                                if (DF_COMPARE(me->status)==0)
                                    {
                                    SysInfo.msg_ptr.flash="DELETE PAGE CHECKING ERROR"; 
                                    return Q_TRAN(&FdiskFlashErr14);
                                    }
                                else
                                    {
#ifdef DEBUG_DISK	 
                                    uint16_t delete_page_num=flash_disk_image.tail;
#endif
                                    if (++flash_disk_image.tail==FLASH_DISK_SIZE)flash_disk_image.tail=0;
                                    flash_disk_image.count--;
                                    SaveFatToFram(&flash_disk_image);
                                    OutDebugDiskSprintf3("DELETE PAGE OK err_timer="," delete_page_num="," stored count=",me->err_timer,delete_page_num,flash_disk_image.count); 
                                    return Q_TRAN(&FdiskIdle3); 
                                    }
                                }
                            else if (me->err_timer>0)QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
                            else
                                {
                                SysInfo.msg_ptr.flash="DELETE PAGE CECKING TIMEOUT ERROR";  
                                return Q_TRAN(&FdiskFlashErr14);
                                }
                        }
                        break;
                    }
            }
            return Q_HANDLED();
        case Q_EXIT_SIG:  return Q_HANDLED();
        }
    return Q_SUPER(&FdiskTop1);
}

QState FdiskFormatFlash(DISK * const me, QEvt const * const e)
{
#define SECTOR_ERASE_ERR_TIMEOUT 1200//6 sec
    static enum
        {
        erasing_sector_0A,
        erasing_sector_0B,
        erasing_sectors_1_63,
        }state;
    static  uint8_t sector_counter;
    switch (e->sig)
        {
        case Q_ENTRY_SIG:
            {
                ind_flag.disk_format=1;
                FillBuf1 (0xFF);
                QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
                ErasingSector0A();
                me->err_timer=SECTOR_ERASE_ERR_TIMEOUT;
                state=erasing_sector_0A;
            }
            return Q_HANDLED();
        case DISK_TIMEOUT_SIG:
            {
                me->err_timer--;
                me->status= ReadDfStatus ();
                switch (state)
                    {
                    case erasing_sector_0A:
                        {
                            if (DF_READY(me->status)==1)
                                {
                                ErasingSector0B();
                                QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
                                me->err_timer=SECTOR_ERASE_ERR_TIMEOUT;
                                state=erasing_sector_0B;
                                }
                            else if (me->err_timer>0)QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
                            else
                                {
                                OutDebugDisk("erasing_sector_0A TIMEOUT ERROR");
                                SysInfo.msg_ptr.flash="FdiskFormatFlash::erasing_sector_0A TIMEOUT ERROR"; 
                                return Q_TRAN(&FdiskFlashErr14);
                                }
                        }
                        break;
                    case erasing_sector_0B:
                        {
                            if (DF_READY(me->status)==1)
                                {
                                sector_counter=1;
                                ErasingNonZeroSector(sector_counter);
                                QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
                                me->err_timer=SECTOR_ERASE_ERR_TIMEOUT;
                                state=erasing_sectors_1_63;
                                }
                            else if (me->err_timer>0)QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
                            else
                                {
                                OutDebugDisk("erasing_sector_0B TIMEOUT ERROR");
                                SysInfo.msg_ptr.flash="FdiskFormatFlash::erasing_sector_0B TIMEOUT ERROR"; 
                                return Q_TRAN(&FdiskFlashErr14);
                                }
                        }
                        break;
                    case erasing_sectors_1_63:
                        {
                            if (DF_READY(me->status)==1)
                                {
#ifdef DEBUG_DISK	
                                uint8_t num=sector_counter;
#endif
                                if (++sector_counter<=63)
                                    {
                                    ErasingNonZeroSector(sector_counter);
                                    OutDebugDiskSprintf1("ERASING SECTOR OK num=",num); 
                                    QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
                                    me->err_timer=SECTOR_ERASE_ERR_TIMEOUT;
                                    }
                                else if (1==ResetDiskImage(&flash_disk_image))
                                    {
                                    OutDebugDisk("DISK Format OK");
                                    QACTIVE_POST(AO_Control,&DiskReadyEvt, me);
                                    return Q_TRAN(&FdiskIdle3); 
                                    }
                                else
                                    {
                                    OutDebugDisk("ResetDiskImage ERROR");
                                    SysInfo.msg_ptr.fram="FdiskFormatFlash::ResetDiskImage ERROR";    
                                    return Q_TRAN(&FdiskFramErr13);
                                    }

                                }
                            else if (me->err_timer>0)QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
                            else
                                {
                                OutDebugDisk("ERASING PAGE TIMEOUT ERROR");
                                SysInfo.msg_ptr.flash="FdiskFormatFlash::ERASING PAGE TIMEOUT ERROR";  
                                return Q_TRAN(&FdiskFlashErr14);
                                }
                        }
                        break;
                    }
            }
            return Q_HANDLED();
        case Q_EXIT_SIG:  
            {
                ind_flag.disk_format=0;
            }
            return Q_HANDLED();
        }
    return Q_SUPER(&FdiskTop1);
}

QState FdiskCheck(DISK * const me, QEvt const * const e)
{
    static enum
        {
        read_page,
        delete_page,
        check,
        }state;
    switch (e->sig)
        {
        case Q_ENTRY_SIG:
            {
                state=read_page;
                QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1); 
            }
            return Q_HANDLED();
        case DISK_TIMEOUT_SIG:
            {  
                switch (state)
                    {
                    case read_page:
                        { 
                            if (flash_disk_image.count==0)return Q_TRAN(&FdiskIdle3);
                            ReadPageDirect((uint8_t*)&page_image,flash_disk_image.tail);
                            if (PageImageCrcOk(&page_image)==1)return Q_TRAN(&FdiskIdle3);
                            else
                                {
                                ErasingPage (flash_disk_image.tail);
                                me->err_timer=20;
                                state=delete_page;
                                QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1); 
                                }                                               
                        }
                        break;    
                    case delete_page:
                        {
                            me->err_timer--;
                            me->status= ReadDfStatus ();
                            if (DF_READY(me->status)==1)
                                {
                                Buf1ComparePage (flash_disk_image.tail);
                                me->err_timer=20;
                                QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
                                state=check;
                                }
                            else if (me->err_timer>0)QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
                            else
                                {
                                SysInfo.msg_ptr.flash="DELETE PAGE TIMEOUT ERROR"; 
                                return Q_TRAN(&FdiskFlashErr14);
                                }
                        }
                        break;
                    case check:
                        {
                            me->status= ReadDfStatus ();
                            if (DF_READY(me->status)==1)
                                {
                                if (DF_COMPARE(me->status)==0)
                                    {
                                    SysInfo.msg_ptr.flash="DELETE PAGE CHECKING ERROR"; 
                                    return Q_TRAN(&FdiskFlashErr14);
                                    }
                                else
                                    {
#ifdef DEBUG_DISK	 
                                    uint16_t delete_page_num=flash_disk_image.tail;
#endif
                                    if (++flash_disk_image.tail==FLASH_DISK_SIZE)flash_disk_image.tail=0;
                                    flash_disk_image.count--;
                                    SaveFatToFram(&flash_disk_image);
                                    OutDebugDiskSprintf3("DELETE PAGE OK err_timer="," delete_page_num="," stored count=",me->err_timer,delete_page_num,flash_disk_image.count); 
                                    QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
                                    state=read_page; 
                                    }
                                }
                            else if (me->err_timer>0)QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
                            else
                                {
                                SysInfo.msg_ptr.flash="DELETE PAGE CECKING TIMEOUT ERROR";  
                                return Q_TRAN(&FdiskFlashErr14);
                                }   
                        }
                        break;
                    }
            }
            return Q_HANDLED();
        case Q_EXIT_SIG:  return Q_HANDLED();
        }
    return Q_SUPER(&FdiskTop1);
}



QState FdiskDelToSend11(DISK * const me, QEvt const * const e)
{
    switch (e->sig)
        {
        case Q_ENTRY_SIG:
            {
                QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1); 
            }
            return Q_HANDLED();
        case DISK_TIMEOUT_SIG:
            {           
                if (-1==FramRingBuf_DeletePointsToSend())
                    {
                    OutDebugDisk("DeletePointsToSend ERROR!!!");
                    //me->fram_error_num=FRAM_ERR_4;
                    SysInfo.msg_ptr.fram="DeletePointsToSend ERROR";
                    return Q_TRAN(&FdiskFramErr13);
                    }
            }
            return Q_TRAN(&FdiskIdle3); 
        case Q_EXIT_SIG:  return Q_HANDLED();
        }
    return Q_SUPER(&FdiskTop1);
}

QState FdiskUnlToSend12(DISK * const me, QEvt const * const e)
{
    switch (e->sig)
        {
        case Q_ENTRY_SIG:
            {
                QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1); 
            }
            return Q_HANDLED();
        case DISK_TIMEOUT_SIG:
            {           
                if (-1==FramRingBuf_UnlockPointsToSend())
                    {
                    OutDebugDisk("UnlockPointsToSend ERROR!!!");
                    //me->fram_error_num=FRAM_ERR_5;
                    SysInfo.msg_ptr.fram="UnlockPointsToSend ERROR";
                    return Q_TRAN(&FdiskFramErr13);
                    }
            }
            return Q_TRAN(&FdiskIdle3); 
        case Q_EXIT_SIG:  return Q_HANDLED();
        }
    return Q_SUPER(&FdiskTop1);
}


QState FdiskFramErr13(DISK * const me, QEvt const * const e)
{
    static QEvt const ControlFramErrorEvt = { CONTROL_FRAM_ERROR_SIG, 0U, 0U};
    switch (e->sig)
        {
        case Q_ENTRY_SIG:
            {
                OutDebugDisk("DISK FRAM ERROR");
                FramRingBuf_Format(); 
                QACTIVE_POST(AO_Control,&ControlFramErrorEvt, me);
                QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1000);
            } 
            return Q_HANDLED(); 
        case DISK_TIMEOUT_SIG:   
            {
                OutDebugDisk("DISK FRAM ERROR");
                QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1000);
            }   
            return Q_HANDLED();
        case Q_EXIT_SIG:  
            {
                QTimeEvt_disarm(&me->TimeEvt);
            }
            return Q_HANDLED();
        }
    return Q_SUPER(&FdiskTop1);
}

QState FdiskFlashErr14(DISK * const me, QEvt const * const e)
{
    static QEvt const ControlFlashErrorEvt = { CONTROL_FLASH_ERROR_SIG, 0U, 0U};
    switch (e->sig)
        {
        case Q_ENTRY_SIG:
            {
                OutDebugDisk("DISK FLASH ERROR");
                QACTIVE_POST(AO_Control,&ControlFlashErrorEvt, me);
                QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1000);
            }
            return Q_HANDLED();  
        case DISK_TIMEOUT_SIG:
            {
                OutDebugDisk("DISK FLASH ERROR");
                QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1000);
            }   
            return Q_HANDLED();     
        case Q_EXIT_SIG:  return Q_HANDLED();
        }
    return Q_SUPER(&FdiskTop1);
}

QState FdiskOff16(DISK * const me, QEvt const * const e)
{
    switch (e->sig)
        {
        case Q_ENTRY_SIG:
            {
                QACTIVE_POST(AO_Control,&ControlDiskIsOffEvt, me);
            }
            return Q_HANDLED();      
        case Q_EXIT_SIG:  return Q_HANDLED();
        }
    return Q_SUPER(&FdiskTop1);
}


void SaveFatToFram(FlashPagesRingBufType *ptr)
{
    ptr->crc=Crc32Eth((uint8_t*)ptr,sizeof(FlashPagesRingBufType)-sizeof(ptr->crc));
    FM25_WriteArray(PAGES_BUF_ADDR,(uint8_t*)ptr,sizeof(FlashPagesRingBufType));
}

int ResetDiskImage(FlashPagesRingBufType *ptr)
{
    ptr->count=ptr->head=ptr->tail=0; 
    SaveFatToFram(ptr);
    return LoadFlashDiskImage(ptr);
}

int LoadFlashDiskImage(FlashPagesRingBufType *ptr)
{
    FM25_ReadArray(PAGES_BUF_ADDR,(uint8_t*)ptr,sizeof(FlashPagesRingBufType));
    return(ptr->crc==Crc32Eth((uint8_t*)ptr,sizeof(FlashPagesRingBufType)-sizeof(ptr->crc)))?1:0;
}



int GetPointsFromTailToWritePageBuf(FlashPageType* page_image_ptr)//1-run time 30 ms
{
    uint16_t free_space=FLASH_PAGE_ITEM_SIZE;
    if (CopyFramPointsRingBufImageToRam())
        {
        return FramRingBuf_GetFromTailToFlashBuf((uint8_t*)&page_image_ptr->points,&free_space,&page_image_ptr->points_count);
        }
    else return 0;
}

#ifdef DEBUG_DISK
void OutDebugDisk( char const *dbg_msg)
{
    QS_BEGIN(QS_DISK, AO_Disk)                                 
    QS_STR(dbg_msg);                              
    QS_END()
}

void OutDebugDiskSprintf1( char const *str,uint32_t val)
{
    QS_BEGIN(QS_DISK, AO_Disk)                                  
    QS_STR(str); 
    QS_U32(4, val);   
    QS_END()
}

/*void OutDebugDiskSprintf2( char const *str1,char const *str2,uint32_t val1,uint32_t val2)
   {
   QS_BEGIN(QS_DISK, AO_Disk)                                  
   QS_STR(str1); 
   QS_U32(4, val1);
   QS_STR(str2); 
   QS_U32(4, val2);  
   QS_END()
   }*/


void OutDebugDiskSprintf3( char const *str1,char const *str2,char const *str3,uint32_t val1,uint32_t val2,uint32_t val3)
{
    QS_BEGIN(QS_DISK, AO_Disk)                                  
    QS_STR(str1); 
    QS_U32(4, val1);
    QS_STR(str2); 
    QS_U32(4, val2);
    QS_STR(str3); 
    QS_U32(4, val3);      
    QS_END()
}

void OutDebugDiskSprintf4( char const *str1,char const *str2,char const *str3,char const *str4,uint32_t val1,uint32_t val2,uint32_t val3,uint32_t val4)
{
    QS_BEGIN(QS_DISK, AO_Disk)                                  
    QS_STR(str1); 
    QS_U32(4, val1);
    QS_STR(str2); 
    QS_U32(4, val2);
    QS_STR(str3); 
    QS_U32(4, val3); 
    QS_STR(str4); 
    QS_U32(4, val4);      
    QS_END()
}
#endif

int PageImageCrcOk(FlashPageType *ptr)
{
    uint32_t page_crc,calc_crc;
    page_crc=ptr->crc;
    calc_crc=Crc32Eth((uint8_t*)&ptr->points_count ,sizeof(FlashPageType)-sizeof(ptr->crc));
    return(calc_crc==page_crc)? 1 : 0;
}

uint32_t PageFMPacketMaker(uint8_t *buf_ptr,FlashPageType *page_ptr)
{
    //SatGsmType sg={0};
    uint64_t temp_long;
    uint16_t temp_short,read_offset;
    uint8_t *write_ptr,*read_ptr,*total_io_ptr;
    uint8_t *evt_id_ptr,data_id,evt_flag;
    uint8_t *Nbytes_ptr;  
    read_offset=0;
    ((FM2200AvlPacketType*)buf_ptr)->zero=(uint32_t)0;
    ((FM2200AvlPacketType*)buf_ptr)->AvlDataArray.codecId=8;
    ((FM2200AvlPacketType*)buf_ptr)->AvlDataArray.dataCount=page_ptr->points_count;
    write_ptr=(uint8_t*)&((FM2200AvlPacketType*)buf_ptr)->AvlDataArray.data.time;//set write ptr to first AVL 
    for (int32_t i=0;i<page_ptr->points_count;i++)
        {
        read_ptr=read_offset+page_ptr->points;
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
        // sg.SatGsm=((QPointType*)read_ptr)->gps.sat_gsmq;
        //*write_ptr=sg.sat_gsm.sat_used;//write sat used
        *write_ptr=((QPointType*)read_ptr)->gps.sg.sat_gsm.sat_used;
        write_ptr++;//set to speed
        temp_short=(uint16_t)((QPointType*)read_ptr)->gps.speed;
        ShortToBigEndianStream(write_ptr,temp_short);//write speed
        write_ptr+=2;//set to EVENT IO ID
        //--------------------------------------------------------------------
        evt_id_ptr=write_ptr;//save evt_id_ptr
        *write_ptr=0;//write EVENT IO ID
        write_ptr++;//set to N of total IO
        //------------------------------------
        *write_ptr=0;//write total io
        total_io_ptr=write_ptr;//save ptr to total_io;
        write_ptr++;//set to N of one byte IO
        //------------------1------------------
        *write_ptr=0;//write N of one byte IO
				 Nbytes_ptr=write_ptr;//save ptr to N of one bytes IO
        write_ptr++;//set to N of one bytes IO
				 for (uint32_t i=0;i<((QPointType*)read_ptr)->sensors_count;i++)
            {
            data_id=((QPointType*)read_ptr)->sensors_array[i].descriptor.data_id;
            evt_flag=((QPointType*)read_ptr)->sensors_array[i].id>>7;
						uint8_t	id=((QPointType*)read_ptr)->sensors_array[i].id;
						id&=~(1<<7);
            if (data_id==IO_DIN)
                {
                *write_ptr++=(id==DIN1)?TELTONIKA_DIN1_ID:TELTONIKA_DIN2_ID;
                 *write_ptr++=  *(uint8_t*)&((QPointType*)read_ptr)->sensors_array[i].data.any_data;
                ++(*total_io_ptr);//increment N of total IO
                ++(*Nbytes_ptr);//increment N of onebytes IO
                }
            }  				
        //--------------------2-----------------------------------
        *write_ptr=0;//write N of two bytes IO
        Nbytes_ptr=write_ptr;//save ptr to N of two bytes IO
        write_ptr++;//set ptr to next(ID of two bytes IO or N of four bytes IO)
        //-----------TWO BYTES DATA-------------------------------------------- 
        *write_ptr++=EXTERNAL_POWER_VOLTAGE_ID;
        ShortToBigEndianStream(write_ptr,10*((QPointType*)read_ptr)->gps.voltage.external );
        write_ptr+=sizeof(uint16_t);
        ++(*total_io_ptr);//increment N of total IO
        ++(*Nbytes_ptr);//increment N of two bytes IO
        //--------------------------------------------------
        *write_ptr++=INTERNAL_POWER_VOLTAGE_ID;
        ShortToBigEndianStream(write_ptr,10*((QPointType*)read_ptr)->gps.voltage.internal );
        write_ptr+=sizeof(uint16_t);
        ++(*total_io_ptr);//increment N of total IO
        ++(*Nbytes_ptr);//increment N of two bytes IO
        //--------------------------------------------------
        uint8_t fuel_sensors_counter=0;
        for (uint32_t i=0;i<((QPointType*)read_ptr)->sensors_count;i++)
            {
            data_id=((QPointType*)read_ptr)->sensors_array[i].descriptor.data_id;
            evt_flag=((QPointType*)read_ptr)->sensors_array[i].id>>7;
            uint8_t data_size=((QPointType*)read_ptr)->sensors_array[i].descriptor.data_size;
            if ((data_id==IO_FUEL_SENSOR || data_id==IO_FUEL_AND_TEMPERATURE_SENSOR)&& data_size==3)
                {
                fuel_sensors_counter++;
                if (fuel_sensors_counter==1)
                    {
                    if (evt_flag)*evt_id_ptr=FUEL_LEVEL1_PARAM_ID;
                    *write_ptr++=FUEL_LEVEL1_PARAM_ID;//write ID of eight bytes IO and set ptr to next 
                    }
                else if (fuel_sensors_counter==2)
                    {
                    if (evt_flag)*evt_id_ptr=FUEL_LEVEL2_PARAM_ID;
                    *write_ptr++=FUEL_LEVEL2_PARAM_ID;//write ID of eight bytes IO and set ptr to next 
                    }
                else continue;
                //*write_ptr++=((QPointType*)read_ptr)->sensors_array[i].data.any_data[1];//big endian format
                //*write_ptr++=((QPointType*)read_ptr)->sensors_array[i].data.any_data[0];
                ShortToBigEndianStream(write_ptr,*(uint16_t*)&((QPointType*)read_ptr)->sensors_array[i].data.any_data);
                write_ptr+=sizeof(uint16_t);
                ++(*total_io_ptr);//increment N of total IO
                ++(*Nbytes_ptr);//increment N of two bytes IO
                }
            /* else if (data_id==IO_ACCUM_COUNT&& data_size==2)//NMEA ERROR
{
  *write_ptr++=9;//analog input1
                 uint32_t counter=   *(uint32_t*)&((QPointType*)read_ptr)->sensors_array[i].data.any_data;
ShortToBigEndianStream(write_ptr,counter);
                 write_ptr+=sizeof(uint16_t);
++(*total_io_ptr);//increment N of total IO
++(*Nbytes_ptr);//increment N of fourbytes IO
} */
            }
						
					  for (uint32_t i=0;i<((QPointType*)read_ptr)->sensors_count;i++)
                {
                data_id=((QPointType*)read_ptr)->sensors_array[i].descriptor.data_id;
                evt_flag=((QPointType*)read_ptr)->sensors_array[i].id>>7;
								uint8_t	id=((QPointType*)read_ptr)->sensors_array[i].id;
								id&=~(1<<7);
                if (data_id==IO_RPM && id==FIN1)
                    {
                    *write_ptr++=TELTONIKA_FIN1_ID;
                    uint16_t rpm=   *(uint16_t*)&((QPointType*)read_ptr)->sensors_array[i].data.any_data;
                    ShortToBigEndianStream(write_ptr,rpm);
                    write_ptr+=sizeof(uint16_t);
                    ++(*total_io_ptr);//increment N of total IO
                    ++(*Nbytes_ptr);//increment N of fourbytes IO
                    }
                }		
        //-------------------------4---------------------------------					 
        *write_ptr=0;//write N of four bytes IO
        Nbytes_ptr=write_ptr;//save ptr to N of four bytes IO
        write_ptr++;//set ptr to next(ID of four bytes IO or N of eight bytes IO)
        //-----------FOUR BYTES DATA-------------------------------------------- 
        for (uint32_t i=0;i<((QPointType*)read_ptr)->sensors_count;i++)
            {
            data_id=((QPointType*)read_ptr)->sensors_array[i].descriptor.data_id;
            evt_flag=((QPointType*)read_ptr)->sensors_array[i].id>>7;
            uint8_t	id=((QPointType*)read_ptr)->sensors_array[i].id;
						id&=~(1<<7);
            if (data_id==IO_RPM && id==FIN2)
                {
                 *write_ptr++=TELTONIKA_FIN2_ID;
                uint32_t rpm=   *(uint16_t*)&((QPointType*)read_ptr)->sensors_array[i].data.any_data;
                IntToBigEndianStream(write_ptr,rpm);
                write_ptr+=sizeof(int);
                ++(*total_io_ptr);//increment N of total IO
                ++(*Nbytes_ptr);//increment N of fourbytes IO
                }
            }  						
//-------------------------8-----------------------------					 
        /* *write_ptr=0;//write N of eight bytes IO
         Nbytes_ptr=write_ptr;//save ptr to N of eight bytes IO
         write_ptr++;//set ptr to next or ID of eight bytes IO
         for (uint32_t i=0;i<((QPointType*)read_ptr)->sensors_count;i++)
            {
            data_size=((QPointType*)read_ptr)->sensors_array[i].descriptor.data_size;
            if (data_size>4 && data_size<=8)
               {
               data_id=((QPointType*)read_ptr)->sensors_array[i].descriptor.data_id;
               evt_flag=((QPointType*)read_ptr)->sensors_array[i].id>>7;
               if (data_id==IO_IDENTIFIER)
                  {
                  if (evt_flag)*evt_id_ptr=0x4E;
                  *write_ptr++=0x4E;//write ID of eight bytes IO and set ptr to next
                  for (uint32_t j=0;j<8-data_size;j++) *write_ptr++=0;//fill unused sensor data space
                  for (int32_t j=data_size-1;j>=0;j--) *write_ptr++=((QPointType*)read_ptr)->sensors_array[i].data.any_data[j]; 
                  ++(*total_io_ptr);//increment N of total IO
                  ++(*Nbytes_ptr);//increment N of eight bytes IO
                  }
               }
            }*/
        //------------------------------------------------------------------------------------		 
        *write_ptr=0;//write N of eight bytes IO
        Nbytes_ptr=write_ptr;//save ptr to N of eight bytes IO
        write_ptr++;//set ptr to next or ID of eight bytes IO
        for (uint32_t i=0;i<((QPointType*)read_ptr)->sensors_count;i++)
            {
            //memcpy(&dbg_sens,&((QPointType*)read_ptr)->sensors_array[i],sizeof(QSensorType));
            data_id=((QPointType*)read_ptr)->sensors_array[i].descriptor.data_id;
            uint8_t data_size=((QPointType*)read_ptr)->sensors_array[i].descriptor.data_size;
            evt_flag=((QPointType*)read_ptr)->sensors_array[i].id>>7;
            if (data_id==IO_IDENTIFIER)
                {
                if (data_size==RF10_CARD_ID_SIZE)
                    {
                    if (evt_flag)*evt_id_ptr=RF10_PARAM_ID;
                    ++(*total_io_ptr);//increment N of total IO
                    ++(*Nbytes_ptr);//increment N of eight bytes IO
                    *write_ptr++=RF10_PARAM_ID;//write ID of eight bytes IO and set ptr to next
                    *write_ptr++=0;//write byte 8 of eight bytes IO and set ptr to next
                    *write_ptr++=0;//write byte 7 of eight bytes IO and set ptr to next
                    *write_ptr++=0;//write byte 6 of eight bytes IO and set ptr to next
                    for (int32_t j=4;j>=0;j--)
                        {
                        *write_ptr++=((QPointType*)read_ptr)->sensors_array[i].data.any_data[j]; 
                        }
                    }
                else//TL10_DATA_SIZE
                    {
                    if (evt_flag)*evt_id_ptr=TL10_PARAM_ID;
                    ++(*total_io_ptr);//increment N of total IO
                    ++(*Nbytes_ptr);//increment N of eight bytes IO
                    *write_ptr++=TL10_PARAM_ID;//write ID of eight bytes IO and set ptr to next
                    *write_ptr++=0;//write byte 8 of eight bytes IO and set ptr to next
                    *write_ptr++=0;//write byte 7 of eight bytes IO and set ptr to next
                    *write_ptr++=0;//write byte 6 of eight bytes IO and set ptr to next
                    *write_ptr++=0;//write byte 5 of eight bytes IO and set ptr to next
                    *write_ptr++=0;//write byte 4 of eight bytes IO and set ptr to next
                    *write_ptr++=0;//write byte 3 of eight bytes IO and set ptr to next
                    for (int32_t j=1;j>=0;j--)
                        {
                        *write_ptr++=((QPointType*)read_ptr)->sensors_array[i].data.any_data[j]; 
                        }
                    }
                }
            }
        //--------------------------------------------------------------------------		 
        read_offset+=sizeof(QGpsDataType)+sizeof(uint8_t)+sizeof(QSensorType)*((QPointType*)read_ptr)->sensors_count;          
        }
    *write_ptr++= page_ptr->points_count;//END OF AVL ARRAY	
    //----------------------------------------------------------------------------
    uint8_t *temp_ptr=&((FM2200AvlPacketType*)buf_ptr)->AvlDataArray.codecId;        
    temp_short=(uint16_t)(write_ptr-temp_ptr);
    IntToBigEndianStream((uint8_t*)&((FM2200AvlPacketType*)buf_ptr)->len,temp_short);
    uint32_t crc= crc16_teltonika((uint8_t*)&((FM2200AvlPacketType*)buf_ptr)->AvlDataArray.codecId,temp_short);   
    IntToBigEndianStream(write_ptr,crc);
    write_ptr+=4;
    return(write_ptr-buf_ptr);
}



uint8_t debug_id;
uint8_t debug_din;
QSensorType dbg_sens[6];
uint8_t debug_data_arr[8];
int32_t PointsFMPacketMaker(uint8_t *buf_ptr,uint8_t *points_to_send_buf_ptr,uint16_t points_to_send_buf_size,uint8_t* points_to_send_count)
{
    uint8_t data_id,evt_flag,*evt_id_ptr;  
    uint64_t temp_long;
    uint8_t *write_ptr,*read_ptr, *total_io_ptr;  
    uint8_t *Nbytes_ptr;  
    uint16_t free_space,read_len_total;//temp_short;
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
            // sg.SatGsm=((QPointType*)read_ptr)->gps.sat_gsmq;
            //*write_ptr=sg.sat_gsm.sat_used;//write sat used
            *write_ptr=((QPointType*)read_ptr)->gps.sg.sat_gsm.sat_used;
            write_ptr++;//set to speed
            //temp_short=(uint16_t)((QPointType*)read_ptr)->gps.speed;
            ShortToBigEndianStream(write_ptr,((QPointType*)read_ptr)->gps.speed);//write speed
            write_ptr+=sizeof(short);//set to EVENT IO ID
            //--------------------------------------------------------------------------------
            evt_id_ptr=write_ptr;//save evt_id_ptr
            *write_ptr=0;//write EVENT IO ID
            write_ptr++;//set to N of total IO
            //------------------------------------
            *write_ptr=0;//write total io
            total_io_ptr=write_ptr;//save ptr to total_io;
            write_ptr++;//set to N of one byte IO
            //------------------1------------------
            *write_ptr=0;//write N of one byte IO
				    Nbytes_ptr=write_ptr;//save ptr to N of one bytes IO
            write_ptr++;//set to N of one bytes IO
				    for (uint32_t i=0;i<((QPointType*)read_ptr)->sensors_count;i++)
            {
            data_id=((QPointType*)read_ptr)->sensors_array[i].descriptor.data_id;
            evt_flag=((QPointType*)read_ptr)->sensors_array[i].id>>7;
            uint8_t	id=((QPointType*)read_ptr)->sensors_array[i].id;
						id&=~(1<<7);
            if (data_id==IO_DIN)
                {
                *write_ptr++=(id==DIN1)?TELTONIKA_DIN1_ID:TELTONIKA_DIN2_ID;
               debug_din=  *write_ptr++=  *(uint8_t*)&((QPointType*)read_ptr)->sensors_array[i].data.any_data;
                ++(*total_io_ptr);//increment N of total IO
                ++(*Nbytes_ptr);//increment N of onebytes IO
                }
            }   
            //--------------------2---------------------------------------------------------
            *write_ptr=0;//write N of two bytes IO
            Nbytes_ptr=write_ptr;//save ptr to N of two bytes IO
            write_ptr++;//set ptr to next(ID of two bytes IO or N of four bytes IO)
            //-----------TWO BYTES DATA-------------------------------------------- 
            *write_ptr++=EXTERNAL_POWER_VOLTAGE_ID;
            ShortToBigEndianStream(write_ptr,10*((QPointType*)read_ptr)->gps.voltage.external );
            write_ptr+=sizeof(uint16_t);
            ++(*total_io_ptr);//increment N of total IO
            ++(*Nbytes_ptr);//increment N of two bytes IO
            //--------------------------------------------------
            *write_ptr++=INTERNAL_POWER_VOLTAGE_ID;
            ShortToBigEndianStream(write_ptr,10*((QPointType*)read_ptr)->gps.voltage.internal );
            write_ptr+=sizeof(uint16_t);
            ++(*total_io_ptr);//increment N of total IO
            ++(*Nbytes_ptr);//increment N of two bytes IO
            //--------------------------------------------------
            uint8_t fuel_sensors_counter=0;
            uint8_t sensors_count=((QPointType*)read_ptr)->sensors_count;
            for (uint32_t i=0;i<sensors_count;i++)
                {
                memcpy(&dbg_sens[i],&((QPointType*)read_ptr)->sensors_array[i],sizeof(QSensorType));
                data_id=((QPointType*)read_ptr)->sensors_array[i].descriptor.data_id;
                evt_flag=((QPointType*)read_ptr)->sensors_array[i].id>>7;
                uint8_t data_size=((QPointType*)read_ptr)->sensors_array[i].descriptor.data_size;
                if ((data_id==IO_FUEL_SENSOR || data_id==IO_FUEL_AND_TEMPERATURE_SENSOR)&& data_size==3)
                    {
                    fuel_sensors_counter++;
                    if (fuel_sensors_counter==1)
                        {
                        if (evt_flag)*evt_id_ptr=FUEL_LEVEL1_PARAM_ID;
                        *write_ptr++=FUEL_LEVEL1_PARAM_ID;//write ID of eight bytes IO and set ptr to next 
                        }
                    else if (fuel_sensors_counter==2)
                        {
                        if (evt_flag)*evt_id_ptr=FUEL_LEVEL2_PARAM_ID;
                        *write_ptr++=FUEL_LEVEL2_PARAM_ID;//write ID of eight bytes IO and set ptr to next 
                        }
                    else continue;
                    // *write_ptr++=((QPointType*)read_ptr)->sensors_array[i].data.any_data[1];//big endian format
                    //*write_ptr++=((QPointType*)read_ptr)->sensors_array[i].data.any_data[0];
                    ShortToBigEndianStream(write_ptr,*(uint16_t*)&((QPointType*)read_ptr)->sensors_array[i].data.any_data);
                    write_ptr+=sizeof(uint16_t);
                    ++(*total_io_ptr);//increment N of total IO
                    ++(*Nbytes_ptr);//increment N of two bytes IO
                    }
                /*else if (data_id==IO_ACCUM_COUNT&& data_size==2)//NMEA ERROR
       {
          *write_ptr++=9;//analog input1
                         uint16_t nmea_err_counter=   *(uint16_t*)&((QPointType*)read_ptr)->sensors_array[i].data.any_data;
        ShortToBigEndianStream(write_ptr,nmea_err_counter);
                         write_ptr+=sizeof(uint16_t);
       ++(*total_io_ptr);//increment N of total IO
       ++(*Nbytes_ptr);//increment N of fourbytes IO
       }*/

                }     
            for (uint32_t i=0;i<sensors_count;i++)
                {
                data_id=((QPointType*)read_ptr)->sensors_array[i].descriptor.data_id;
                evt_flag=((QPointType*)read_ptr)->sensors_array[i].id>>7;
								uint8_t	id=((QPointType*)read_ptr)->sensors_array[i].id;
								id&=~(1<<7);
                if (data_id==IO_RPM && id==FIN1)
                    {
                    *write_ptr++=TELTONIKA_FIN1_ID;
                    uint16_t rpm=   *(uint16_t*)&((QPointType*)read_ptr)->sensors_array[i].data.any_data;
                    ShortToBigEndianStream(write_ptr,rpm);
                    write_ptr+=sizeof(uint16_t);
                    ++(*total_io_ptr);//increment N of total IO
                    ++(*Nbytes_ptr);//increment N of fourbytes IO
                    }
                }								
//-------------------------4------------------------------------------------------------
            *write_ptr=0;//write N of four bytes IO
            Nbytes_ptr=write_ptr;//save ptr to N of four bytes IO
            write_ptr++;//set ptr to next(ID of four bytes IO or N of eight bytes IO)
            for (uint32_t i=0;i<sensors_count;i++)
                {
                data_id=((QPointType*)read_ptr)->sensors_array[i].descriptor.data_id;
                evt_flag=((QPointType*)read_ptr)->sensors_array[i].id>>7;
								uint8_t	id=((QPointType*)read_ptr)->sensors_array[i].id;
								id&=~(1<<7);
                if (data_id==IO_RPM && id==FIN2)
                    {
                    *write_ptr++=TELTONIKA_FIN2_ID;
                    uint32_t rpm=   *(uint16_t*)&((QPointType*)read_ptr)->sensors_array[i].data.any_data;
                    IntToBigEndianStream(write_ptr,rpm);
                    write_ptr+=sizeof(int);
                    ++(*total_io_ptr);//increment N of total IO
                    ++(*Nbytes_ptr);//increment N of fourbytes IO
                    }
                }
//-------------------------8-----------------------------					 	
            *write_ptr=0;//write N of eight bytes IO
            Nbytes_ptr=write_ptr;//save ptr to N of eight bytes IO
            write_ptr++;//set ptr to next or ID of eight bytes IO
            for (uint32_t i=0;i<sensors_count;i++)
                {
                //memcpy(&dbg_sens,&((QPointType*)read_ptr)->sensors_array[i],sizeof(QSensorType));
                data_id=((QPointType*)read_ptr)->sensors_array[i].descriptor.data_id;
                uint8_t data_size=((QPointType*)read_ptr)->sensors_array[i].descriptor.data_size;
                evt_flag=((QPointType*)read_ptr)->sensors_array[i].id>>7;
                uint8_t* dbg_ptr;
                if (data_id==IO_IDENTIFIER)
                    {
                    if (data_size==TL10_DATA_SIZE)//TL10_DATA_SIZE
                        {
                        if (evt_flag)*evt_id_ptr=TL10_PARAM_ID;
                        ++(*total_io_ptr);//increment N of total IO
                        ++(*Nbytes_ptr);//increment N of eight bytes IO
                        *write_ptr++=TL10_PARAM_ID;//write ID of eight bytes IO and set ptr to next
                        dbg_ptr=write_ptr;
                        *write_ptr++=0;//write byte 8 of eight bytes IO and set ptr to next
                        *write_ptr++=0;//write byte 7 of eight bytes IO and set ptr to next
                        *write_ptr++=0;//write byte 6 of eight bytes IO and set ptr to next
                        *write_ptr++=0;//write byte 5 of eight bytes IO and set ptr to next
                        *write_ptr++=0;//write byte 4 of eight bytes IO and set ptr to next
                        *write_ptr++=0;//write byte 3 of eight bytes IO and set ptr to next
                        // *write_ptr++=0xFF;//write byte 2 of eight bytes IO and set ptr to next
                        //*write_ptr++=0xFF;//write byte 1 of eight bytes IO and set ptr to next
                        for (int32_t j=1;j>=0;j--)
                            {
                            debug_id=  *write_ptr++=((QPointType*)read_ptr)->sensors_array[i].data.any_data[j]; 
                            }
                        // for(int i=0;i<8;i++)debug_data_arr[i]=dbg_ptr[i];
                        }
                    else if (data_size==RF10_CARD_ID_SIZE)
                        {
                        if (evt_flag)*evt_id_ptr=RF10_PARAM_ID;
                        ++(*total_io_ptr);//increment N of total IO
                        ++(*Nbytes_ptr);//increment N of eight bytes IO
                        *write_ptr++=RF10_PARAM_ID;//write ID of eight bytes IO and set ptr to next
                        dbg_ptr=write_ptr;
                        *write_ptr++=0;//write byte 8 of eight bytes IO and set ptr to next
                        *write_ptr++=0;//write byte 7 of eight bytes IO and set ptr to next
                        *write_ptr++=0;//write byte 6 of eight bytes IO and set ptr to next
                        for (int32_t j=4;j>=0;j--)
                            {
                            *write_ptr++=((QPointType*)read_ptr)->sensors_array[i].data.any_data[j]; 
                            }
                        for (int i=0;i<8;i++)debug_data_arr[i]=dbg_ptr[i];
                        __nop();
                        }
                    }
                }
//----------------------------------------------------------------------------------------------------------
            read_len_total+=sizeof(QGpsDataType)+sizeof(uint8_t)+sizeof(QSensorType)*((QPointType*)read_ptr)->sensors_count;          
            }
        *write_ptr++= *points_to_send_count;//END OF AVL ARRAY	
        uint8_t *temp_ptr=&((FM2200AvlPacketType*)buf_ptr)->AvlDataArray.codecId;        
        uint32_t len=(uint32_t)(write_ptr-temp_ptr);
        IntToBigEndianStream((uint8_t*)&((FM2200AvlPacketType*)buf_ptr)->len,len);
        uint32_t crc= crc16_teltonika((uint8_t*)&((FM2200AvlPacketType*)buf_ptr)->AvlDataArray.codecId,len);   
        IntToBigEndianStream(write_ptr,crc);
        write_ptr+=4;
        return write_ptr-buf_ptr;
        }
}




int32_t PointsNTPacketMaker(uint8_t *buf_ptr,uint8_t *points_to_send_buf_ptr,uint16_t points_to_send_buf_size,uint8_t* points_to_send_count)
{
    uint8_t *write_ptr,*read_ptr,sensor_data_size; 
    uint16_t free_space,write_size,all_sensors_in_point_size;
    uint16_t pack_len;
    free_space=points_to_send_buf_size;
    if (-1== FramRingBuf_CopyPointsFromTailToSendBuf(points_to_send_buf_ptr,&free_space,points_to_send_count))
        {
        return -1;
        }
    else
        {
        ((XTDevPacketType*)buf_ptr)->head.id=HEAD_PACK_ONLINE; 
        ((XTDevPacketType*)buf_ptr)->head.crc=0;
        write_ptr=(uint8_t*)&((XTDevPacketType*)buf_ptr)->payload;//set ptr to points_count
        *write_ptr=*points_to_send_count;
        write_ptr++;//set ptr to first point	
        for (int32_t pointI=0,read_offset=0;pointI<*points_to_send_count;pointI++)
            {
            read_ptr=points_to_send_buf_ptr+read_offset;
            write_size=sizeof(QGpsDataType);
            memcpy(write_ptr,read_ptr,write_size);
            write_ptr+=write_size;//set write_ptr to sensors_count
            //--------------------------------------------------------------------------------
            *write_ptr = ((QPointType*)read_ptr)->sensors_count;
            //----------------------------------------------------------------------------------
            all_sensors_in_point_size=0;
            write_ptr++;//set write_ptr to sensors_array
            for (uint32_t sensorI=0;sensorI<((QPointType*)read_ptr)->sensors_count;sensorI++)
                {
                *write_ptr++=((QPointType*)read_ptr)->sensors_array[sensorI].id;
                *write_ptr++=*(uint8_t*)&((QPointType*)read_ptr)->sensors_array[sensorI].descriptor;
                sensor_data_size=((QPointType*)read_ptr)->sensors_array[sensorI].descriptor.data_size;
                all_sensors_in_point_size+=sensor_data_size+2;
                for (uint32_t byteI=0;byteI<sensor_data_size;byteI++)
                    {
                    *write_ptr++=((QPointType*)read_ptr)->sensors_array[sensorI].data.any_data[byteI];
                    } 
                }  
            read_offset+=sizeof(QGpsDataType)+sizeof(uint8_t)+sizeof(QSensorType)*((QPointType*)read_ptr)->sensors_count;      
            } 
        pack_len=write_ptr-buf_ptr;
        ((XTDevPacketType*)buf_ptr)->head.len=pack_len;
        ((XTDevPacketType*)buf_ptr)->head.crc =  MakeCRC16(buf_ptr,pack_len);
        return pack_len;
        }
}

uint32_t PageNTPacketMaker(uint8_t *buf_ptr,FlashPageType *page_ptr)
{
    uint8_t *write_ptr,*read_ptr,sensor_data_size;
    uint16_t write_size,all_sensors_in_point_size,sensors_in_point_count,pack_len;
    ((XTDevPacketType*)buf_ptr)->head.id=HEAD_PACK_ARCH; 
    ((XTDevPacketType*)buf_ptr)->head.crc=0;
    write_ptr=(uint8_t*)&((XTDevPacketType*)buf_ptr)->payload;//set ptr to points_count
    *write_ptr=page_ptr->points_count;
    write_ptr++;//set ptr to first point	
    for (int32_t pointI=0,read_offset=0;pointI<page_ptr->points_count;pointI++)
        {
        read_ptr=page_ptr->points+read_offset;
        write_size=sizeof(QGpsDataType);
        memcpy( write_ptr, read_ptr,write_size);//copy gps data
        write_ptr+=write_size;//set write_ptr to sensors_count
        //--------------------------------------------------------------------------------
        sensors_in_point_count=((QPointType*)read_ptr)->sensors_count;
        *write_ptr = sensors_in_point_count;
        //----------------------------------------------------------------------------------
        all_sensors_in_point_size=0;
        write_ptr++;//set write_ptr to sensors_array
        for (uint32_t sensorI=0;sensorI<sensors_in_point_count;sensorI++)
            {
            *write_ptr++=((QPointType*)read_ptr)->sensors_array[sensorI].id;
            *write_ptr++=*(uint8_t*)&((QPointType*)read_ptr)->sensors_array[sensorI].descriptor;
            sensor_data_size=((QPointType*)read_ptr)->sensors_array[sensorI].descriptor.data_size;
            all_sensors_in_point_size+=sensor_data_size+2;
            for (uint32_t byteI=0;byteI<sensor_data_size;byteI++)
                {
                *write_ptr++=((QPointType*)read_ptr)->sensors_array[sensorI].data.any_data[byteI];
                } 
            } 
        read_offset+=sizeof(QGpsDataType)+sizeof(uint8_t)+sizeof(QSensorType)*((QPointType*)read_ptr)->sensors_count;      
        } 
    pack_len=write_ptr-buf_ptr;
    ((XTDevPacketType*)buf_ptr)->head.len=pack_len;
    ((XTDevPacketType*)buf_ptr)->head.crc =  MakeCRC16(buf_ptr,pack_len);
    return pack_len;
}

//--------------CONFIG-------------------------------------------------------------------------------------	 
int ConfigReadWriteGprs(int write,uint8_t index)
{
    ConfigGprsType *ptr=&conf.gprs[index];
    int ret=1;
    uint16_t size=sizeof(ConfigGprsType);
    uint16_t addr=CONFIG_GPRS_ADDR+size*index;
    if (write)goto WRITE_L;
    FM25_ReadArray(addr,(uint8_t*)&conf.gprs[index],size);
    if (ptr->crc16==MakeCRC16((uint8_t*)ptr,size-sizeof(ptr->crc16)))return 1;
    else
        {
        ret=0;
        memcpy((uint8_t*)ptr,(uint8_t*)&config_gprs_default[index],size);
        WRITE_L:     
        ptr->crc16=MakeCRC16((uint8_t*)ptr,size-sizeof(ptr->crc16));
        FM25_WriteArray(addr,(uint8_t*)ptr,size);
        return ret;
        }
}


int ConfigReadWriteServer(int write,uint8_t index)
{
    ConfigServerType *ptr=&conf.server[index];
    int ret=1;
    uint16_t size=sizeof(ConfigServerType);
    uint16_t addr=CONFIG_SERVER_ADDR+size*index;
    if (write)goto WRITE_L;
    FM25_ReadArray(addr,(uint8_t*)ptr,size);
    if (ptr->crc16==MakeCRC16((uint8_t*)ptr,size-sizeof(ptr->crc16)))return 1;
    else
        {
        ret=0;
        memcpy((uint8_t*)ptr,(uint8_t*)&config_server_default[index],size);
        WRITE_L:     
        ptr->crc16=MakeCRC16((uint8_t*)ptr,size-sizeof(ptr->crc16));
        FM25_WriteArray(addr,(uint8_t*)ptr,size);
        return ret;
        }
}

int ConfigReadWriteNavFilter(int write,uint8_t unused)
{
    ConfigNavFilterType *ptr=&conf_nav_filter;
    int ret=1;
    uint16_t addr=CONFIG_NAV_FILTER_ADDR,size=sizeof(ConfigNavFilterType);
    if (write)goto WRITE_L;
    FM25_ReadArray(addr,(uint8_t*)ptr,size);
    if (ptr->crc16==MakeCRC16((uint8_t*)ptr,size-sizeof(ptr->crc16)))return 1;
    else
        {
        ret=0;
        ptr->filter_on=NAVIGATION_FILTER_DEFAULT;
        ptr->time=STORE_POINT_TIME_DEFAULT;
        ptr->dist=STORE_POINT_DIST_DEFAULT;
        ptr->angle=STORE_POINT_ANGLE_DEFAULT;
        WRITE_L:     
        ptr->crc16=MakeCRC16((uint8_t*)ptr,size-sizeof(ptr->crc16));
        FM25_WriteArray(addr,(uint8_t*)ptr,size);
        return ret;
        }
}

int ConfigReadWritePacketSend(int write,uint8_t unused)
{
    ConfigPacketSendType *ptr=&conf.packet_send;
    int ret=1;
    uint16_t addr=CONFIG_PACKET_SEND_ADDR,size=sizeof(ConfigPacketSendType);
    if (write)goto WRITE_L;
    FM25_ReadArray(addr,(uint8_t*)ptr,size);
    if (ptr->crc16==MakeCRC16((uint8_t*)ptr,size-sizeof(ptr->crc16)))return 1;
    else
        {
        ret=0;
        ptr->time=PACKET_SEND_TIME_DEFAULT;
        ptr->dist =PACKET_SEND_DIST_DEFAULT;
        ptr->prio_filter= PACKET_SEND_PRIO_FILTER_DEFAULT;
        WRITE_L:     
        ptr->crc16=MakeCRC16((uint8_t*)ptr,size-sizeof(ptr->crc16));
        FM25_WriteArray(addr,(uint8_t*)ptr,size);
        return ret;
        }
}

int ConfigReadWriteRS485Sensor(int write,uint8_t index)
{
    ConfigRS485SensorType *ptr=&conf_rs485_sensor[index];
    int ret=1;
    uint16_t size= sizeof(ConfigRS485SensorType),addr=CONFIG_RS485_SENSOR_ADDR+size*index;
    if (write)goto WRITE_L;
    FM25_ReadArray(addr,(uint8_t*)ptr,size);
    if (ptr->crc16==MakeCRC16((uint8_t*)ptr,size-sizeof(ptr->crc16)))return 1;
    else
        {
        ret=0;
        memset((uint8_t*)ptr,0,size);
        WRITE_L:     
        ptr->crc16=MakeCRC16((uint8_t*)ptr,size-sizeof(ptr->crc16));
        FM25_WriteArray(addr,(uint8_t*)ptr,size);
        return ret;
        }
}

int ConfigReadWriteRS485(int write,uint8_t unused)
{
    ConfigRS485Type *ptr=&conf.rs485;
    int ret=1;
    uint16_t addr=CONFIG_RS485_ADDR,size=  sizeof(ConfigRS485Type); 
    if (write)goto WRITE_L;
    FM25_ReadArray(addr,(uint8_t*)ptr,size);
    if (ptr->crc16==MakeCRC16((uint8_t*)ptr,size-sizeof(ptr->crc16)))return 1;
    else
        {
        ret=0;
        ptr->deltaN=DELTA_N_DEFAULT;
        WRITE_L:     
        ptr->crc16=MakeCRC16((uint8_t*)ptr,size-sizeof(ptr->crc16));
        FM25_WriteArray(addr,(uint8_t*)ptr,size);
        return ret;
        }
}

int ConfigReadWriteFin(int write,uint8_t unused)
{
    ConfigFinType *ptr=&conf.fin;
    int ret=1;
    uint16_t addr=CONFIG_FIN_ADDR,size=    sizeof(ConfigFinType); 
    if (write)goto WRITE_L;
    FM25_ReadArray(addr,(uint8_t*)ptr,size);
    if (ptr->crc16==MakeCRC16((uint8_t*)ptr,size-sizeof(ptr->crc16)))return 1;
    else
        {
        ret=0;
        for (int i=0;i<MAX_FIN_COUNT;i++)ptr->mode[i]=IO_OFF;
        WRITE_L:     
        ptr->crc16=MakeCRC16((uint8_t*)ptr,size-sizeof(ptr->crc16));
        FM25_WriteArray(addr,(uint8_t*)ptr,size);
        return ret;
        }
}


int ConfigReadWriteDout(int write,uint8_t unused)
{
    ConfigDoutType *ptr=&conf.dout;
    int ret=1;
    uint16_t addr=CONFIG_DOUT_ADDR,size=   sizeof(ConfigDoutType); 
    if (write)goto WRITE_L;
    FM25_ReadArray(addr,(uint8_t*)ptr,size);
    if (ptr->crc16==MakeCRC16((uint8_t*)ptr,size-sizeof(ptr->crc16)))
        {
        if (ptr->out_state==1)DOUT1_ON();
        else DOUT1_OFF();
        return 1;
        }
    else
        {
        ret=0;
        ptr->out_state=DOUT_STATE_DEFAULT;
        WRITE_L:     
        ptr->crc16=MakeCRC16((uint8_t*)ptr,size-sizeof(ptr->crc16));
        FM25_WriteArray(addr,(uint8_t*)ptr,size);

        if (ptr->out_state==1)DOUT1_ON();
        else DOUT1_OFF();

        return ret;
        }
}



int LogReadWriteCounter(int write,uint8_t unused)
{
    LogType *ptr=&conf.log;
    int ret=1;
    uint16_t addr=CONFIG_LOG_ADDR,size=sizeof(LogType);
    if (write)goto WRITE_L;
    FM25_ReadArray(addr,(uint8_t*)ptr,size);
    if (ptr->crc16==MakeCRC16((uint8_t*)ptr,size-sizeof(ptr->crc16)))return 1;
    else
        {
        ret=0;
        memset((uint8_t*)ptr,0,size); 
        WRITE_L:     
        ptr->crc16=MakeCRC16((uint8_t*)ptr,size-sizeof(ptr->crc16));
        FM25_WriteArray(addr,(uint8_t*)ptr,size);
        return ret;
        }
}

int ConfigReadWriteHardware(int write,uint8_t unused)
{
    ConfigHardwareType *ptr=&conf.hardware;
    int ret=1;
    uint16_t addr= CONFIG_HARDWARE_ADDR,size=sizeof(ConfigHardwareType);
    if (write)goto WRITE_L;
    FM25_ReadArray(addr,(uint8_t*)ptr,size);
    if (ptr->crc16==MakeCRC16((uint8_t*)ptr,size-sizeof(ptr->crc16)))return 1;
    else
        {
        ret=0;
        memset((uint8_t*)ptr,0,size);
        WRITE_L:     
        ptr->crc16=MakeCRC16((uint8_t*)ptr,size-sizeof(ptr->crc16));
        FM25_WriteArray(addr,(uint8_t*)ptr,size);
        return ret;
        }
}



int ConfigReadWriteSecur(int write,uint8_t unused)
{
    ConfigSecurType *ptr=&conf.secur;
    int ret=1;
    uint16_t addr=CONFIG_SECUR_ADDR,size=sizeof(ConfigSecurType);
    if (write)goto WRITE_L;
    FM25_ReadArray(addr,(uint8_t*)ptr,size);
    if (ptr->crc16==MakeCRC16((uint8_t*)ptr,size-sizeof(ptr->crc16)))
        {
        return 1;
        }
    else
        {
        ret=0;
        ptr->save=0;
        ptr->regular_restart_timeout=REGULAR_RESTART_TIMEOUT_DEFAULT;
        ptr->fw_ver_new=0;
        //ptr->sim=CONF_CURRENT_SIM_DEFAULT;
        ptr->protocol=PROTOCOL_DEFAULT; 
        ptr->send_reset_sms=     RESET_SMS_DEFAULT;;
        strlcpy(ptr->user_phone, USER_PHONE, MAX_PHONE_STR_LEN);
//      strlcpy(ptr->oleg_phone, OLEG_PHONE, MAX_PHONE_STR_LEN);
//      strlcpy(ptr->igor_phone, IGOR_PHONE, MAX_PHONE_STR_LEN);
        WRITE_L:     
        ptr->crc16=MakeCRC16((uint8_t*)ptr,size-sizeof(ptr->crc16));
        FM25_WriteArray(addr,(uint8_t*)ptr,size);
        return ret;
        }
}


int ConfigReadWriteSim(int write,uint8_t unused)
{
    ConfigSimType *ptr=&conf.sim;
    int ret=1;
    uint16_t addr=CONFIG_SIM_ADDR,size=sizeof(ConfigSimType);
    if (write)goto WRITE_L;
    FM25_ReadArray(addr,(uint8_t*)ptr,size);
    if (ptr->crc16==MakeCRC16((uint8_t*)ptr,size-sizeof(ptr->crc16)))
        {
        return 1;
        }
    else
        {
        ret=0;
        ptr->save=0;
        ptr->sim_num=CONF_CURRENT_SIM_DEFAULT;
        WRITE_L:     
        ptr->crc16=MakeCRC16((uint8_t*)ptr,size-sizeof(ptr->crc16));
        FM25_WriteArray(addr,(uint8_t*)ptr,size);
        return ret;
        }
}


uint32_t ConfigSave(void)
{
    uint32_t i;
    uint32_t wr_counter=0,dummy=0;
    if (conf.packet_send.save)
        {
        conf.packet_send.save=0;
        ConfigReadWritePacketSend(WRITE,dummy);
        wr_counter++;
        }
    if (conf.hardware.save)
        {
        conf.hardware.save=0;
        ConfigReadWriteHardware(WRITE,dummy);
        wr_counter++;
        }
    if (conf.fin.save)
        {
        conf.fin.save=0;
        ConfigReadWriteFin(WRITE,dummy);
        wr_counter++;
        }
    if (conf.dout.save)
        {
        conf.dout.save=0;
        ConfigReadWriteDout(WRITE,dummy);
        wr_counter++;
        }
    if (conf_nav_filter.save)
        {
        conf_nav_filter.save=0;
        ConfigReadWriteNavFilter(WRITE,dummy);
        wr_counter++;
        }
    if (conf.log.save)
        {
        conf.log.save=0;
        LogReadWriteCounter(WRITE,dummy);
        wr_counter++;
        }
    if (conf.secur.save)
        {
        conf.secur.save=0;
        ConfigReadWriteSecur(WRITE,dummy);
        wr_counter++;
        }
    if (conf.sim.save)
        {
        conf.sim.save=0;
        ConfigReadWriteSim(WRITE,dummy);
        wr_counter++;
        }
    if (conf.rs485.save)
        {
        conf.rs485.save=0;
        ConfigReadWriteRS485(WRITE,dummy);
        wr_counter++;
        }
    for (i=0;i<2;i++)
        {
        if (conf.gprs[i].save)
            {
            conf.gprs[i].save=0;
            ConfigReadWriteGprs(WRITE,i);
            wr_counter++;
            }
        }

    for (i=0;i<2;i++)
        {
        if (conf.server[i].save)
            {
            conf.server[i].save=0;
            ConfigReadWriteServer(WRITE,i);
            wr_counter++;
            }
        }
    for (i=0;i<MAX_RS485_SENSORS_COUNT;i++)
        {
        if (conf_rs485_sensor[i].save)
            {
            conf_rs485_sensor[i].save=0;
            ConfigReadWriteRS485Sensor(WRITE,i);
            wr_counter++;
            }
        }
    return wr_counter;
}

uint8_t ConfigRead(void)
{
    uint32_t i;
    uint8_t wr_counter=0,dummy=0;
    if (0==ConfigReadWritePacketSend(READ,dummy))wr_counter++;
    if (0==ConfigReadWriteNavFilter(READ,dummy))wr_counter++;
    if (0==LogReadWriteCounter(READ,dummy))wr_counter++;
    if (0==ConfigReadWriteSecur(READ,dummy))wr_counter++;
    if (0==ConfigReadWriteSim(READ,dummy))wr_counter++;
    if (0==ConfigReadWriteHardware(READ,dummy))wr_counter++;
    if (0==ConfigReadWriteRS485(READ,dummy))wr_counter++;
    if (0==ConfigReadWriteFin(READ,dummy))wr_counter++;
    if (0==ConfigReadWriteDout(READ,dummy))wr_counter++;
    for (i=0;i<2;i++)
        {
        if (0==ConfigReadWriteGprs(READ,i))wr_counter++;
        }
    for (i=0;i<2;i++)
        {
        if (0==ConfigReadWriteServer(READ,i))wr_counter++;
        }
    for (i=0;i<MAX_RS485_SENSORS_COUNT;i++)
        {
        if (0==ConfigReadWriteRS485Sensor(READ,i))wr_counter++;
        }
    return wr_counter;
}
/*
    const  uint8_t points_count=2;
QPointType test_point[points_count];   
int32_t PointsFMPacketMakerTest(uint8_t *buf_ptr,QPointType *rptr)
   {
        
   uint8_t data_id,evt_flag,*evt_id_ptr;  
   uint64_t temp_long;
   uint8_t *write_ptr, *total_io_ptr;  
   uint8_t *Nbytes_ptr;	

      ((FM2200AvlPacketType*)buf_ptr)->zero=0;
      ((FM2200AvlPacketType*)buf_ptr)->AvlDataArray.codecId=8;
      ((FM2200AvlPacketType*)buf_ptr)->AvlDataArray.dataCount=points_count;
      write_ptr=(uint8_t*)&((FM2200AvlPacketType*)buf_ptr)->AvlDataArray.data.time;//set write ptr to first AVL 
      for (int32_t i=0;i<points_count;i++)
         {
         rptr+=i;
         temp_long= rptr->gps.time;
         temp_long*=1000;
         LongToBigEndianStream(write_ptr,temp_long);
         write_ptr+=8;//set write_ptr to prio
         *write_ptr=0;//write prio
         write_ptr++;//set to lon
         IntToBigEndianStream(write_ptr,rptr->gps.lon_int);
         write_ptr+=4;//set to lat
         IntToBigEndianStream(write_ptr,rptr->gps.lat_int);
         write_ptr+=4;//set to alt
         ShortToBigEndianStream(write_ptr,rptr->gps.alt);
         write_ptr+=2;//set to angle
         ShortToBigEndianStream(write_ptr,rptr->gps.angle);
         write_ptr+=2;//set to sat
                 *write_ptr=rptr->gps.sg.sat_gsm.sat_used;
         write_ptr++;//set to speed
         ShortToBigEndianStream(write_ptr,rptr->gps.speed);//write speed
                     write_ptr+=sizeof(short);//set to EVENT IO ID
         //--------------------------------------------------------------------------------
         evt_id_ptr=write_ptr;//save evt_id_ptr
         *write_ptr=0;//write EVENT IO ID
                 write_ptr++;//set to N of total IO
         //------------------------------------
         *write_ptr=0;//write total io
         total_io_ptr=write_ptr;//save ptr to total_io;
                 write_ptr++;//set to N of one byte IO
         //------------------1------------------
         *write_ptr=0;//write N of one byte IO
                   write_ptr++;//set to N of two bytes IO
         //--------------------2---------------------------------------------------------
         *write_ptr=0;//write N of two bytes IO
         Nbytes_ptr=write_ptr;//save ptr to N of two bytes IO
         write_ptr++;//set ptr to next(ID of two bytes IO or N of four bytes IO)
                 //-----------TWO BYTES DATA-------------------------------------------- 
                 *write_ptr++=EXTERNAL_POWER_VOLTAGE_ID;
                      ShortToBigEndianStream(write_ptr,rptr->gps.voltage.external );
                        write_ptr+=sizeof(uint16_t);
            ++(*total_io_ptr);//increment N of total IO
            ++(*Nbytes_ptr);//increment N of two bytes IO
                        //--------------------------------------------------
                        *write_ptr++=INTERNAL_POWER_VOLTAGE_ID;
                      ShortToBigEndianStream(write_ptr,rptr->gps.voltage.internal );
                        write_ptr+=sizeof(uint16_t);
            ++(*total_io_ptr);//increment N of total IO
            ++(*Nbytes_ptr);//increment N of two bytes IO
                        //--------------------------------------------------
                 uint8_t fuel_sensors_counter=0;
                 uint8_t sensors_count=rptr->sensors_count;
         for (uint32_t i=0;i<sensors_count;i++)
            {
            data_id=rptr->sensors_array[i].descriptor.data_id;
            evt_flag=rptr->sensors_array[i].id>>7;
                        uint8_t data_size=rptr->sensors_array[i].descriptor.data_size;
            if ((data_id==IO_FUEL_SENSOR || data_id==IO_FUEL_AND_TEMPERATURE_SENSOR)&& data_size==3)
               {
                                 fuel_sensors_counter++;
                                 if(fuel_sensors_counter==1)
                                 {
                                     if (evt_flag)*evt_id_ptr=FUEL_LEVEL1_PARAM_ID;
                  *write_ptr++=FUEL_LEVEL1_PARAM_ID;//write ID of eight bytes IO and set ptr to next 
                                 }
                                 else if(fuel_sensors_counter==2)
                                 {
                                     if (evt_flag)*evt_id_ptr=FUEL_LEVEL2_PARAM_ID;
                  *write_ptr++=FUEL_LEVEL2_PARAM_ID;//write ID of eight bytes IO and set ptr to next 
                                 }
                                else continue;
              
                                 ShortToBigEndianStream(write_ptr,*(uint16_t*)&rptr->sensors_array[i].data.any_data);
                                 write_ptr+=sizeof(uint16_t);
               ++(*total_io_ptr);//increment N of total IO
               ++(*Nbytes_ptr);//increment N of two bytes IO
               }
                        
                            
                    }						
//-------------------------4------------------------------------------------------------
                         *write_ptr=0;//write N of four bytes IO
                   Nbytes_ptr=write_ptr;//save ptr to N of four bytes IO
             write_ptr++;//set ptr to next(ID of four bytes IO or N of eight bytes IO)
                        for (uint32_t i=0;i<sensors_count;i++)
            {
            data_id=rptr->sensors_array[i].descriptor.data_id;
            evt_flag=rptr->sensors_array[i].id>>7;
            if (data_id==IO_RPM)
               {
                 *write_ptr++=76;
                                 uint32_t rpm=   *(uint16_t*)&rptr->sensors_array[i].data.any_data;
                IntToBigEndianStream(write_ptr,rpm);
                             write_ptr+=sizeof(int);
               ++(*total_io_ptr);//increment N of total IO
               ++(*Nbytes_ptr);//increment N of fourbytes IO
               }         
                    }
//-------------------------8-----------------------------					 	
         *write_ptr=0;//write N of eight bytes IO
         Nbytes_ptr=write_ptr;//save ptr to N of eight bytes IO
         write_ptr++;//set ptr to next or ID of eight bytes IO
                     for (uint32_t i=0;i<sensors_count;i++)
            {
                            //memcpy(&dbg_sens,&((QPointType*)read_ptr)->sensors_array[i],sizeof(QSensorType));
                            data_id=rptr->sensors_array[i].descriptor.data_id;
                            uint8_t data_size=rptr->sensors_array[i].descriptor.data_size;
                            evt_flag=rptr->sensors_array[i].id>>7;
                            uint8_t* dbg_ptr;
             if (data_id==IO_IDENTIFIER)
               {
                                  if(data_size==TL10_DATA_SIZE)//TL10_DATA_SIZE
                                 {
                                      if(evt_flag)*evt_id_ptr=TL10_PARAM_ID;
                    ++(*total_io_ptr);//increment N of total IO
                    ++(*Nbytes_ptr);//increment N of eight bytes IO
                    *write_ptr++=TL10_PARAM_ID;//write ID of eight bytes IO and set ptr to next
                                     dbg_ptr=write_ptr;
                    *write_ptr++=0;//write byte 8 of eight bytes IO and set ptr to next
                    *write_ptr++=0;//write byte 7 of eight bytes IO and set ptr to next
                    *write_ptr++=0;//write byte 6 of eight bytes IO and set ptr to next
                                      *write_ptr++=0;//write byte 5 of eight bytes IO and set ptr to next
                    *write_ptr++=0;//write byte 4 of eight bytes IO and set ptr to next
                    *write_ptr++=0;//write byte 3 of eight bytes IO and set ptr to next
                                    
                   for (int32_t j=1;j>=0;j--)
                                       {
                                         debug_id=  *write_ptr++=rptr->sensors_array[i].data.any_data[j]; 
                                       }
                                            // for(int i=0;i<8;i++)debug_data_arr[i]=dbg_ptr[i];
                                 }
                                else if(data_size==RF10_CARD_ID_SIZE)
                                 {
                                      if(evt_flag)*evt_id_ptr=RF10_PARAM_ID;
                    ++(*total_io_ptr);//increment N of total IO
                    ++(*Nbytes_ptr);//increment N of eight bytes IO
                    *write_ptr++=RF10_PARAM_ID;//write ID of eight bytes IO and set ptr to next
                                     dbg_ptr=write_ptr;
                    *write_ptr++=0;//write byte 8 of eight bytes IO and set ptr to next
                    *write_ptr++=0;//write byte 7 of eight bytes IO and set ptr to next
                    *write_ptr++=0;//write byte 6 of eight bytes IO and set ptr to next
                    for (int32_t j=4;j>=0;j--)
                                       {
                                        *write_ptr++=rptr->sensors_array[i].data.any_data[j]; 
                                       }
                                            for(int i=0;i<8;i++)debug_data_arr[i]=dbg_ptr[i];
                                             __nop();
                                 }
               }
            }       
         }
      *write_ptr++= points_count;//END OF AVL ARRAY	
      uint8_t *temp_ptr=&((FM2200AvlPacketType*)buf_ptr)->AvlDataArray.codecId;        
      uint32_t len=(uint32_t)(write_ptr-temp_ptr);
      IntToBigEndianStream((uint8_t*)&((FM2200AvlPacketType*)buf_ptr)->len,len);
      uint32_t crc= MakeCRC16((uint8_t*)&((FM2200AvlPacketType*)buf_ptr)->AvlDataArray.codecId,len);   
      IntToBigEndianStream(write_ptr,crc);
      write_ptr+=4;
      return write_ptr-buf_ptr;
   }
*/
