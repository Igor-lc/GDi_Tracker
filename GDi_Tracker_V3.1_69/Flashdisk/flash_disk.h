

#ifndef _FLASH_DISK_H
#define _FLASH_DISK_H

#include "main.h"

#define FLASH_DISK_SIZE         (0x2000-1)//unused page 8191 is TEST PAGE

enum FdiskSignals
{
	FDISK_TIMEOUT_SIG=MAX_PUB_SIG,
	FDISK_TEST_SIG,
	FDISK_WRITE_TO_HEAD_SIG,
	FDISK_READ_FROM_TAIL_SIG,
	
};


extern void Fdisk_ctor(void);
extern QActive * const AO_Fdisk;


#endif


