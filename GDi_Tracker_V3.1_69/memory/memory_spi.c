

#include <stm32f10x.h>
#include "memory_spi.h"
#include "user_types.h"

void MemorySpi_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;

	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  GPIO_InitStructure.GPIO_Pin = MEMORY_SPI_MOSI_PIN;
  GPIO_Init(MEMORY_SPI_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = MEMORY_SPI_MISO_PIN;
  GPIO_Init(MEMORY_SPI_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = MEMORY_SPI_SCK_PIN;
  GPIO_Init(MEMORY_SPI_GPIO, &GPIO_InitStructure);
	
    /*SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    // SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; 
    // SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;  
    //SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;  //MODE3
   // SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //MODE3
     SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;  //MODE0
     SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; //MODE0
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    //SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
		 SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI2, &SPI_InitStructure);
    SPI_CalculateCRC(SPI2, DISABLE);
    SPI_Cmd(SPI2, ENABLE);*/
		
		 /* SPI configuration -------------------------------------------------------*/
  SPI_I2S_DeInit(MEMORY_SPI);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_Init(MEMORY_SPI, &SPI_InitStructure);

  SPI_Cmd(MEMORY_SPI, ENABLE);
}

	





















