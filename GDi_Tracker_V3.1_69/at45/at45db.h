

#ifndef _AT45DB_H
#define _AT45DB_H

#include <stdint.h>

#include "config_sys.h"
#ifndef _CONFIG_SYS_H
#error "config_sys.h" NOT INCLUDED!!!
#endif

#define  AT45_SPI         MEMORY_SPI
#define  AT45DB_CS_GPIO   GPIOA
//#define  RCC_AHBPeriph_AT45DB_CS_GPIO   RCC_AHBPeriph_GPIOB
#define  AT45DB_CS_PIN    GPIO_Pin_4
#define  AT45DB_CS_DIS()  do{AT45DB_CS_GPIO->BSRR = AT45DB_CS_PIN;__nop();__nop();__nop();__nop();__nop();__nop();}while(0)
#define  AT45DB_CS_EN()   do{AT45DB_CS_GPIO->BRR =  AT45DB_CS_PIN;__nop();__nop();__nop();__nop();__nop();__nop();}while(0)
 

#define BlockErase               0x50  // Block erase
#define SectorErase              0x7C  // Sector erase
#define FlashPageRead            0xD2  // Main memory page read
#define FlashToBuf1Transfer      0x53  // Main memory page to buffer 1 transfer
#define Buf1Read                 0xD4  // Buffer 1 read
#define FlashToBuf2Transfer 		 0x55  // Main memory page to buffer 2 transfer
#define Buf2Read                 0xD6  // Buffer 2 read
#define StatusReg                0xD7  // Status register
#define PageErase                0x81  // Page erase
#define Buf1ToFlashWE            0x83  // Buffer 1 to main memory page program with built-in erase
#define Buf1Write                0x84  // Buffer 1 write
#define Buf2ToFlashWE   			   0x86  // Buffer 2 to main memory page program with built-in erase
#define Buf2Write                0x87  // Buffer 2 write
#define Buf1ToFlash              0x88  // Buffer 1 to main memory page program without built-in erase
#define Buf2ToFlash              0x89  // Buffer 2 to main memory page program without built-in erase
#define BUF1COMPAREPAGE          0x60
#define BUF2COMPAREPAGE          0x61
//------------------------------------------------------------------------------
#define BUF1                     0x01
#define BUF2                     0x02
#define DF_BUSYMASK              0x80
#define DF_COMPAREMASK           0x40
#define DF_PAGE_SIZE             528
#define DUMMY                    0xFF
#define DF_READY(_status)    ((_status & DF_BUSYMASK)?1:0)
#define DF_COMPARE(_status)  ((_status & DF_COMPAREMASK)?0:1)
//--------------------------------------------------------------------------
extern void AT45_CS_init(void);
extern uint8_t ReadDfStatus (void);
extern uint8_t GetPageBits(void);
//-----------READ-------------------------------------------
extern void ReadPageDirect(uint8_t * dest, uint16_t addr);
//extern void ReadPageToBuf1 (uint16_t PageAdr);
extern void ReadPageToBuf2 (uint16_t PageAdr);
//extern uint8_t ReadByteFromBuf1 (uint16_t addr);
extern uint8_t ReadByteFromBuf2 (uint16_t addr);
//extern void ReadNbytesFromBuf1(uint8_t * dest, uint16_t Nbytes, uint16_t addr);
extern void ReadNbytesFromBuf2(uint8_t * dest, uint16_t Nbytes, uint16_t addr);
//------------WRITE------------------------------
extern void FillBuf1 (uint8_t x);
//extern void WritePageImageToBuf1 (uint8_t *ptr);
extern void WritePageImageToBuf2 (uint8_t *ptr);
//extern void WriteBuf1ToPage (uint16_t addr_page);
extern void WriteBuf2ToPage (uint16_t addr_page);
//extern void WriteBuf1ToPageWithErase (uint16_t page_addr);
extern void WriteBuf2ToPageWithErase (uint16_t page_addr);
//------------ERASE--------------------------------------------------
extern void ErasingPage (uint16_t addr_page);
extern void ErasingBlock (uint16_t addr_block);
extern void ErasingSector0A (void);
extern void ErasingSector0B (void);
extern void ErasingNonZeroSector (uint16_t addr_sector);
//------------COMPARE------------------------------------
void Buf1ComparePage (uint16_t addr_page);
void Buf2ComparePage (uint16_t addr_page);

#endif



