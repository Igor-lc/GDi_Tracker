

#ifndef _MEMORY_SPI_H
#define _MEMORY_SPI_H

#include <stdint.h>
#include "config_sys.h"

#ifndef _CONFIG_SYS_H
#error "config_sys.h" NOT INCLUDED!!!
#endif

#if PCB==PCB_V2_1

#define  MEMORY_SPI           SPI1
#define  MEMORY_SPI_GPIO  GPIOA
#define  MEMORY_SPI_MOSI_PIN    GPIO_Pin_7
#define  MEMORY_SPI_MISO_PIN    GPIO_Pin_6
#define  MEMORY_SPI_SCK_PIN     GPIO_Pin_5 

//#elif PCB==PCB_V2	



#else
#error PCB not defined!
#endif

extern void MemorySpi_init(void);


#endif




