

#ifndef _PACKET_H
#define _PACKET_H

#include "main.h"
#include "UserTypes.h"

#define PACKET_BUF_SIZE    1000
#define POINTS_TO_SEND_BUF_SIZE ((PACKET_BUF_SIZE/10)*9)
#define PAGES_BUF_ADDR    ((uint16_t)((uint32_t)&fm_ptr->pages_buf))
#define POINTS_BUF_ADDR   ((uint16_t)((uint32_t)&fm_ptr->points_buf))
#define POINTS_TOTAL_ADDR ((uint16_t)((uint32_t)&fm_ptr->points_total))
#define CONFIG_ADDR       ((uint16_t)((uint32_t)&fm_ptr->config))
#define TEST_ADDR         ((uint16_t)((uint32_t)&fm_ptr->fram_test))

	typedef	 struct 
        {
        unsigned unused:4;  
        unsigned spi_busy:1;					
        unsigned timed_send:1;
				unsigned fram_error:1;
			  unsigned flash_error:1;
        }PackFmFlagType; 
				
	typedef enum
        {
        not_sending,
        sending,
        send_ok,
				bad_packet,	
        send_error,
        fram_error,
        }SendStatusType;

typedef struct
	 {
		uint8_t fram_test[FRAM_TEST_STRING_LEN];
		 uint32_t points_total;
		 ConfigType config;
		 FramPointsRingBufType		points_buf; 
		 FlashPagesRingBufType    pages_buf;  
	 }FramMemoryType;				
				
enum PacketSignals
{
	PACKET_TIMEOUT_SIG=MAX_PUB_SIG,
	PACK_POINT_SIG,
	PACK_SEND_OK_SIG,
	PACK_SEND_ERROR_SIG,
	PACK_WRITE_CONFIG_SIG,
	PACK_PAGE_WRITE_COMPLETE_SIG,
	PACK_MODEM_READY_SIG,
	//-----------------------------
	FD_ANSW_READ_OK_SIG,
  FD_ANSW_WRITE_OK_SIG,
	FD_ANSW_TEST_OK_SIG,
	FD_ANSW_READ_ERR_SIG,
	FD_ANSW_WRITE_ERR_SIG,
	FD_ANSW_TEST_ERR_SIG,
	FD_ANSW_PAGE_IS_BAD_SIG,
	
};


extern const FramMemoryType *fm_ptr;
extern void Packet_ctor(void);
extern QActive * const AO_Packet;



#endif

