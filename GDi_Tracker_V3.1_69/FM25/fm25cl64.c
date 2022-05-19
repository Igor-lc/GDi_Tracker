

#include <stm32f10x.h>
#include "fm25cl64.h"
#include "user_types.h"
#include "memory_spi.h"

static void FM25_WriteEnable(void);
static void FM25_WriteDisable(void);
static uint8_t FM25_ReadWriteByte(uint8_t data);

void FM25_CS_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = FM25_CS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(FM25_CS_GPIO , &GPIO_InitStructure);
	FM25_CS_DIS();
}

/*******************************************************************
-------------LOCAL SERVICE FUNCTIONS--------------------------------
*******************************************************************/

void FM25_WriteEnable(void)
{
    FM25_CS_EN();
    FM25_ReadWriteByte(WREN_OPCODE); //Write Enable OpCode
    FM25_CS_DIS();
}

void FM25_WriteDisable(void)
{
    FM25_CS_EN();
    FM25_ReadWriteByte(WRDI_OPCODE); //Write Enable OpCode
    FM25_CS_DIS();
}

uint8_t  FM25_ReadWriteByte( uint8_t data ) 
{
     while (SPI_I2S_GetFlagStatus(FM25_SPI, SPI_I2S_FLAG_TXE) == RESET);
    FM25_SPI->DR = data;
    while (SPI_I2S_GetFlagStatus(FM25_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    return FM25_SPI->DR;
}

void FM25_WriteEnableStatusReg( void )   
{
	 while (SPI_I2S_GetFlagStatus(FM25_SPI, SPI_I2S_FLAG_TXE) == RESET){};
	  FM25_CS_EN();
		  FM25_SPI->DR = WREN_OPCODE;
    while (SPI_I2S_GetFlagStatus(FM25_SPI, SPI_I2S_FLAG_RXNE) == RESET);
   volatile uint8_t temp=FM25_SPI->DR;
    FM25_CS_DIS();
}
/*******************************************************************
----------------READ FUNCTIONS-------------------------------------
*******************************************************************/

uint8_t FM25_ReadStatusRegister(void)
{
    uint8_t data;
	 while (SPI_I2S_GetFlagStatus(FM25_SPI, SPI_I2S_FLAG_TXE) == RESET){};
    FM25_CS_EN();
    FM25_SPI->DR= RDSR_OPCODE; 
    while (SPI_I2S_GetFlagStatus(FM25_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    volatile uint8_t temp=FM25_SPI->DR;
    FM25_SPI->DR=0;
    while (SPI_I2S_GetFlagStatus(FM25_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    data =FM25_SPI->DR;
    FM25_CS_DIS();
    return data;
}

uint8_t FM25_ReadByte(uint16_t Address)
{
    uint8_t data;
    FM25_CS_EN();
    FM25_ReadWriteByte(READ_OPCODE); 
    FM25_ReadWriteByte(Address>>8); 
    FM25_ReadWriteByte(Address&0x00ff); 
    data=FM25_ReadWriteByte(0); 
    FM25_CS_DIS();
    return data;
}

uint32_t FM25CL64_ReadWord(uint16_t Address)
{
    WordByteType word_byte_temp={0};
    FM25_CS_EN();
    FM25_ReadWriteByte(READ_OPCODE); 
    FM25_ReadWriteByte(Address>>8); 
    FM25_ReadWriteByte(Address&0x00ff); 
    word_byte_temp.byte[0] =FM25_ReadWriteByte(0); 
    word_byte_temp.byte[1] =FM25_ReadWriteByte(0); 
    word_byte_temp.byte[2] =FM25_ReadWriteByte(0); 
    word_byte_temp.byte[3] =FM25_ReadWriteByte(0); 
    FM25_CS_DIS();
    return word_byte_temp.word;
}
uint8_t debug_spi=0;
void FM25_ReadArray(uint16_t Address,uint8_t *read_ptr,uint16_t size)
{
    FM25_CS_EN();
    FM25_ReadWriteByte(READ_OPCODE); 
    FM25_ReadWriteByte(Address>>8); //high byte
    FM25_ReadWriteByte(Address&0X00ff);//low byte
    while (size--) 
		{
			debug_spi=FM25_ReadWriteByte(0); 
			*read_ptr++=debug_spi;
		}
    FM25_CS_DIS();
}

/*******************************************************************
----------------WRITE FUNCTIONS-------------------------------------
*******************************************************************/

void FM25_WriteStatusRegister(uint8_t status)
{
	 while (SPI_I2S_GetFlagStatus(FM25_SPI, SPI_I2S_FLAG_TXE) == RESET){};
    FM25_CS_EN();
     FM25_SPI->DR= WRSR_OPCODE; 
    while (SPI_I2S_GetFlagStatus(FM25_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    volatile uint8_t temp=FM25_SPI->DR;
    FM25_SPI->DR= status;
    while (SPI_I2S_GetFlagStatus(FM25_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    temp=FM25_SPI->DR;
    FM25_CS_DIS();
}


void FM25_WriteByte(uint16_t Address,uint8_t data)
{
    FM25_WriteEnable();
    FM25_CS_EN();
    FM25_ReadWriteByte(WRITE_OPCODE); 
    FM25_ReadWriteByte(Address>>8); 
    FM25_ReadWriteByte(Address&0X00ff); 
    FM25_ReadWriteByte(data); 
    FM25_CS_DIS();
    FM25_WriteDisable();
}

void FM25CL64_WriteWord(unsigned int Address,uint32_t data)
{
    WordByteType word_byte_temp; 
    word_byte_temp.word=data;
    FM25_WriteEnable();
    FM25_CS_EN();
    FM25_ReadWriteByte(WRITE_OPCODE); 
    FM25_ReadWriteByte(Address>>8); 
    FM25_ReadWriteByte(Address&0X00ff); 
    FM25_ReadWriteByte(word_byte_temp.byte[0]);
    FM25_ReadWriteByte(word_byte_temp.byte[1]); 
    FM25_ReadWriteByte(word_byte_temp.byte[2]);
    FM25_ReadWriteByte(word_byte_temp.byte[3]);   
    FM25_CS_DIS();
    FM25_WriteDisable();
}

void FM25_WriteArray(uint16_t Address,uint8_t *write_ptr,uint16_t size)
{
    FM25_WriteEnable();
    FM25_CS_EN();
    FM25_ReadWriteByte(WRITE_OPCODE); 
    FM25_ReadWriteByte(Address>>8); 
    FM25_ReadWriteByte(Address&0X00ff); 
    while (size--)FM25_ReadWriteByte(*write_ptr++); 
    FM25_CS_DIS();
    FM25_WriteDisable();
}


void FM25_FillPattern(uint8_t pattern)
{
    uint16_t addr=0;
    FM25_WriteEnable();
    FM25_CS_EN();
    FM25_ReadWriteByte(WRITE_OPCODE); 
    FM25_ReadWriteByte(addr>>8); 
    FM25_ReadWriteByte(addr&0X00ff); 
    for(addr=0;addr<FRAM_MEMORY_SIZE;addr++)FM25_ReadWriteByte(pattern); 
    FM25_CS_DIS();
    FM25_WriteDisable();
}











