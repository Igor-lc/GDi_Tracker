
#ifndef LOADER_DEFS_H
#define LOADER_DEFS_H

#include <stdint.h>

//#define PLACE_BOOT_REC_AT_END_OF_FLASH 
#define FLASH_BASE 0x08000000

#define FACTORY_RECORD                     0xA5A5A5A5
#define FLASH_SIZE											  (uint32_t)0x40000//256 kb 
#define FLASH_PAGE_SIZE										(uint32_t)0x800// 2 kb
#define FLASH_BOOTLDR_PAGES_COUNT					(uint32_t)4
#define BOOT_ARREA_SIZE                   (FLASH_PAGE_SIZE*FLASH_BOOTLDR_PAGES_COUNT)//0x2000

#ifdef PLACE_BOOT_REC_AT_END_OF_FLASH  

#define BOOT_REC_PAGES_COUNT     				 (uint32_t)1
#define BOOT_REC_ADDRESS                 (uint32_t)(FLASH_BASE + FLASH_SIZE-FLASH_PAGE_SIZE) 
#define APP_0_START_ADDR	               (FLASH_BASE + BOOT_ARREA_SIZE)//next page after BOOTLOADER
#define MAX_FIRMWARE_SIZE                ((FLASH_SIZE-FLASH_PAGE_SIZE*(FLASH_BOOTLDR_PAGES_COUNT+BOOT_REC_PAGES_COUNT)-FLASH_DRIVE_SIZE)/2)
#define APP_1_START_ADDR							   (FLASH_PROG_START_PHYSICAL_ADDRESS+MAX_FIRMWARE_SIZE+FLASH_PAGE_SIZE*(FLASH_BOOTLDR_PAGES_COUNT+BOOT_REC_PAGES_COUNT))
#else
#define APP_0_START_ADDR	               (FLASH_BASE + BOOT_ARREA_SIZE)//next page after BOOTLOADER
#define BOOT_REC_ADDRESS                 (uint32_t)(APP_0_START_ADDR-(FLASH_PAGE_SIZE*2)) 
#define MAX_FIRMWARE_SIZE                ((FLASH_SIZE-BOOT_ARREA_SIZE)/2)
#define APP_1_START_ADDR							   (FLASH_BASE+BOOT_ARREA_SIZE+MAX_FIRMWARE_SIZE)
	
#endif

typedef __packed struct
{
    uint32_t start_addr; //4   
    uint32_t size;       //4         
    uint32_t crc;        //4              
} FirmwareDescriptorType;//12

typedef __packed struct
{   
	  uint32_t control_word;      //4	
    FirmwareDescriptorType fw0;//12
    FirmwareDescriptorType fw1;//12
    uint32_t fw_to_run;         //4 
    uint32_t crc;               //4                
    uint32_t crc_wrong_flag;    //4     
} BootRecType; //40

typedef __packed struct
{
	BootRecType boot_rec;
	uint8_t dummy[FLASH_PAGE_SIZE-sizeof(BootRecType)];
}BootRecPageType;


#endif

