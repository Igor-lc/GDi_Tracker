

#ifndef _FM25CL64_H
#define _FM25CL64_H

#include "config_sys.h"
#ifndef _CONFIG_SYS_H
#error "config_sys.h" NOT INCLUDED!!!
#endif

#define FRAM_MEMORY_SIZE 0x2000//8192

#define WREN_OPCODE   0x06  // 0110b
#define WRDI_OPCODE   0x04  // 0100b
#define RDSR_OPCODE   0x05  // 0101b  
#define WRSR_OPCODE   0x01  // 0001b  
#define READ_OPCODE   0x03  // 0011b
#define WRITE_OPCODE  0x02  // 0110b

#define  FM25_SPI          MEMORY_SPI
#define  FM25_CS_GPIO      GPIOA
//#define  RCC_AHBPeriph_FM25_CS_GPIO   RCC_AHBPeriph_GPIOE
#define  FM25_CS_PIN      GPIO_Pin_3
#define  FM25_CS_DIS()    do{FM25_CS_GPIO->BSRR = FM25_CS_PIN;__nop();__nop();__nop();__nop();__nop();__nop();}while(0)
#define  FM25_CS_EN()     do{FM25_CS_GPIO->BRR =  FM25_CS_PIN;__nop();__nop();__nop();__nop();__nop();__nop();}while(0)


extern void FM25_CS_init(void);
extern void FM25CL64_WriteWord(unsigned int Address,uint32_t data);
extern uint32_t FM25CL64_ReadWord(uint16_t Address);
extern uint8_t FM25_ReadByte(uint16_t Address);
extern void FM25_WriteByte(uint16_t Address,uint8_t data);
void FM25_ReadArray(uint16_t Address,uint8_t *read_ptr,uint16_t size);
extern void FM25_WriteArray(uint16_t Address,uint8_t *write_ptr,uint16_t size);

#endif



