

#include "modem.h"
#include "modem_parser.h"
#include "nmea_parser.h"
#include "nav_filter.h"
#include "Disk.h"
#include "Control.h"
#include "io.h"
#include "config.h"
#include "bsp.h"
#include "stm32f10x.h"
#include "LedIndicator.h"
#include "loader_defs.h"

#ifndef WATCH_DOG_ENABLE
#warning  WDT IS OFF
#endif

#ifdef _DEBUG_CPU_USAGE
volatile uint32_t cpu_counter=0,cpu_load_max=0;
#endif

const uint8_t FwVer[]  = {3,9}; // RO 

static QEvt const *DiskQueueSto[DISK_QUEUE_SIZE];
static QEvt const *ControlQueueSto[10];
static QEvt const *NavReceiverQueueSto[10];
static QEvt const *ModemQueueSto[10];
 QEvt const *ModemParserQueueSto[10];
static QEvt const *IoQueueSto[10];
static QEvt const *GpsFilterQueueSto[10];

static QF_MPOOL_EL(SmallEvt)    SmallPoolSto[20]; //small pool 
static QF_MPOOL_EL(SensorEvt)  MiddlePoolSto[20]; //midlle pool
static QSubscrList SubscrListSto[MAX_PUB_SIG];

SysInfoType SysInfo;
WatchFlagsType watch_flag;

void catch_handler(uint8_t exception_id)
{
//	panic(exception_id);
}

void HardFault_Handler(void)
{
QF_INT_DISABLE();
LED_SENSOR_ON();
LED_WRPOINT_SMS_ON();
LED_GPRS_ON();
LED_GSM_ON(); 
LED_GPS_ON();
LED_RS485_ON(); 	
	while(1);
}
void MemManage_Handler(void)
{
	QF_INT_DISABLE();
LED_SENSOR_ON();
LED_WRPOINT_SMS_ON();
LED_GPRS_ON();
LED_GSM_ON(); 
LED_GPS_ON();
LED_RS485_ON(); 	
	while(1);
}
void BusFault_Handler(void)
{
	QF_INT_DISABLE();
LED_SENSOR_ON();
LED_WRPOINT_SMS_ON();
LED_GPRS_ON();
LED_GSM_ON(); 
LED_GPS_ON();
LED_RS485_ON(); 	
	while(1);
}

void UsageFault_Handler(void)
{
	QF_INT_DISABLE();
LED_SENSOR_ON();
LED_WRPOINT_SMS_ON();
LED_GPRS_ON();
LED_GSM_ON(); 
LED_GPS_ON();
LED_RS485_ON(); 	
while(1);
}

void CauseOfReset(void)
{
   SysInfo.msg_ptr.fram=NULL;
	 SysInfo.msg_ptr.flash=NULL;
	 SysInfo.msg_ptr.system=NULL;
	
//	if(RCC->CSR&RCC_CSR_OBLRSTF)SysInfo.msg_ptr.restart="OBL RESET"; //OBL reset flag 
//	else if(RCC->CSR&RCC_CSR_PINRSTF)
//	{
//		RCC->CSR&=~RCC_CSR_PINRSTF;
//		SysInfo.msg_ptr.restart="PIN RESET";//PIN reset flag 
//	}
//	else if(RCC->CSR&RCC_CSR_PORRSTF)SysInfo.msg_ptr.restart="POR/PDR RESET";   //POR/PDR reset flag */
//	else if(RCC->CSR&RCC_CSR_SFTRSTF)SysInfo.msg_ptr.restart="SOFT RESET";  // Software Reset flag 
//  else if(RCC->CSR&RCC_CSR_IWDGRSTF)SysInfo.msg_ptr.restart="IWDG RESET";	//Independent Watchdog reset flag
//	else if(RCC->CSR&RCC_CSR_WWDGRSTF)SysInfo.msg_ptr.restart="WWDG RESET";// Window watchdog reset flag
//	else if(RCC->CSR&RCC_CSR_LPWRRSTF)SysInfo.msg_ptr.restart="LOW POWER RESET";// Low-Power reset flag 
//	else 
		SysInfo.msg_ptr.restart="UNKNOWN RESET";
  RCC->CSR|=RCC_CSR_RMVF;//remove reset flag
}

int main (void)
             {
  	SysInfo.program_start_addr=SCB->VTOR;	
	CauseOfReset();
	Io_ctor();
	Control_ctor();
  GpsParser_ctor();		 
  Modem_ctor();
	Mparser_ctor();
	NavFilter_ctor();	
	Fdisk_ctor();
	QF_init();
  Bsp_init();
	QS_OBJ_DICTIONARY(SmallPoolSto);
  QS_OBJ_DICTIONARY(IoQueueSto);	
  QF_poolInit(SmallPoolSto, sizeof(SmallPoolSto), sizeof(SmallPoolSto[0])); 
	QF_poolInit(MiddlePoolSto, sizeof(MiddlePoolSto), sizeof(MiddlePoolSto[0])); 
	QF_psInit(SubscrListSto, Q_DIM(SubscrListSto)); //init publish-subscribe
//-----------------------------------------------------------------------		 
  QActive_start(AO_Disk, (uint8_t)FDISK_PRIO, DiskQueueSto, Q_DIM(DiskQueueSto), (void *) 0, 0U, (QEvt *) 0);			 
  QActive_start(AO_Control, (uint8_t)CONTROL_PRIO, ControlQueueSto, Q_DIM(ControlQueueSto), (void *) 0, 0U, (QEvt *) 0);			 
	QActive_start(AO_NavFilter, (uint8_t)NAV_FILTER_PRIO, GpsFilterQueueSto, Q_DIM(GpsFilterQueueSto), (void *) 0, 0U, (QEvt *) 0);	 
  QActive_start(AO_GpsParser, (uint8_t)NAV_RECEIVER_PRIO, NavReceiverQueueSto, Q_DIM(NavReceiverQueueSto), (void *) 0, 0U, (QEvt *) 0);
	QActive_start(AO_ModemParser, (uint8_t)MODEM_PARSER_PRIO, ModemParserQueueSto, Q_DIM(ModemParserQueueSto), (void *) 0, 0U, (QEvt *) 0);
  QActive_start(AO_Modem, (uint8_t)MODEM_PRIO, ModemQueueSto, Q_DIM(ModemQueueSto), (void *) 0, 0U, (QEvt *) 0);
	QActive_start(AO_Io, (uint8_t)IO_PRIO, IoQueueSto, Q_DIM(IoQueueSto), (void *) 0, 0U, (QEvt *) 0);
	
	return QF_run();
  }




