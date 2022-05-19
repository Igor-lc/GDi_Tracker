
#ifndef MAIN_H
#define MAIN_H

#define false		0
#define true		!false

//#include "stm32f30x.h"
#include <stdint.h>
//#include <string.h>
//#include <alloca.h>
#include "loader_defs.h"
//#include "bsp.h"

typedef void (*TFunction)(void);

#define USER_APP_STACK_POINTER(x) 		(__USP((*((vu32*)x))))
#define MAIN_USER_APP(x)							(((TFunction)(*((vu32*)((u32)x+4))))())
#define NVIC_GENERATE_SYSTEM_RESET()	(SCB->AIRCR = (u32)0x05FA0000 | (u32)0x04)
#define NVIC_SET_VECTOR_TABLE(x)			(SCB->VTOR = ((u32)x & (u32)0x1FFFFF80))


uint8_t CheckFile(FirmwareDescriptorType *file);
void __USP(uint32_t p);

void WrTestLoaderRec(uint32_t addr);

#endif

