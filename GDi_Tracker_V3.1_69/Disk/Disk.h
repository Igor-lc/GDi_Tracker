

#ifndef _DISK_H
#define _DISK_H

#include "user_types.h"

#define DISK_QUEUE_SIZE         (POINTS_POOL_SIZE+10)
#define FLASH_DISK_SIZE         (0x2000-1)//unused page 8191 is CONFIG PAGE
#define FLASH_CONFIG_PAGE_NUM FLASH_DISK_SIZE
//typedef   bool(*conf_wr_fptr)(bool write,uint8_t offset);

typedef  enum FdiskSignals
{
	DISK_TIMEOUT_SIG=MAX_PUB_SIG,//ok
	DISK_MAKE_ARCH_PACK_SIG,
	DISK_MAKE_FRESH_PACK_SIG,//ok
	DISK_POINT_SIG,//ok
	DISK_DELETE_PAGE_SIG,//ok
	DISK_DELETE_POINTS_TO_SEND_SIG,//ok
	DISK_UNLOCK_POINTS_TO_SEND_SIG,//ok
	DISK_OFF_SIG,//ok
	DISK_READ_CONFIG_SIG,//ok
	DISK_SAVE_CONFIG_SIG,//ok
}FdiskSignalsType;

typedef enum
{
	FRAM_NO_ERROR,
	FRAM_ERR_1,
	FRAM_ERR_2,
	FRAM_ERR_3,
	FRAM_ERR_4,
	FRAM_ERR_5,
	FRAM_ERR_7,
	FRAM_ERR_8,
}FramErrorEnum;


typedef struct RequestEvtTag
{
    QEvt super;                                                     
} RequestEvt;

extern QEvt const DiskSaveConfigEvt;
extern int write_point_angle_evt_flag;
extern  uint8_t send_pack_buf[];
extern FlashPagesRingBufType flash_disk_image;//records ring buf image
extern void Fdisk_ctor(void);
extern QActive * const AO_Disk;


#endif


