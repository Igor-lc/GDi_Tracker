

#include <stm32f10x.h>
#include "memory_spi.h"
#include "at45db.h"
#include "user_types.h"




//Look-up table for these sizes ->          512k, 1M, 2M, 4M, 8M, 16M, 32M, 64M
static const uint8_t DF_pagebits[]  ={  9,  9,  9,  9,  9,  10,  10,  11};	//index of internal page address bits
static uint8_t PageBits=0;

uint8_t GetPageBits(void){return PageBits;}

void AT45_CS_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = AT45DB_CS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(AT45DB_CS_GPIO , &GPIO_InitStructure);
	 AT45DB_CS_DIS();
}

/************************************************************************
-------------------READ OPERATIONS--------------------------------------
*************************************************************************/

uint8_t  AT45DBSpiReadWriteByte(uint8_t data) 
{
	  while (SPI_I2S_GetFlagStatus(AT45_SPI, SPI_I2S_FLAG_TXE) == RESET){};
    AT45_SPI->DR =  data; 
    while (SPI_I2S_GetFlagStatus(AT45_SPI, SPI_I2S_FLAG_RXNE) == RESET);
    return AT45_SPI->DR;
}


unsigned char ReadDfStatus (void)
{
	unsigned char result,index_copy;
	 AT45DB_CS_EN();
	result =AT45DBSpiReadWriteByte(StatusReg);//send status register read op-code
	result =AT45DBSpiReadWriteByte(StatusReg);//send status register read op-code
	index_copy = ((result & 0x38) >> 3);	//get the size info from status register
	PageBits   = DF_pagebits[index_copy];	//get number of internal page address bits from look-up table
	 AT45DB_CS_DIS();
	return result;						         	//return the read status register value
}

uint8_t ReadByteFromBuf1 (uint16_t addr)//one byte
{
	  WordByteType wb;
    AT45DB_CS_EN();	
    wb.word=(uint32_t)addr;	
	  AT45DBSpiReadWriteByte(Buf1Read);	      
	  AT45DBSpiReadWriteByte(0x00);						//don't cares
		AT45DBSpiReadWriteByte(wb.byte[1]);
	  AT45DBSpiReadWriteByte(wb.byte[0]);
		AT45DBSpiReadWriteByte(0x00);						//don't cares
		uint8_t byte = AT45DBSpiReadWriteByte(DUMMY);		//read byte
	  AT45DB_CS_DIS();
	  return byte;
}


uint8_t ReadByteFromBuf2 (uint16_t addr)//one byte
{
	  WordByteType wb;
    AT45DB_CS_EN();	
    wb.word=(uint32_t)addr;	
	  AT45DBSpiReadWriteByte(Buf2Read);	      
		AT45DBSpiReadWriteByte(0x00);						//don't cares
		AT45DBSpiReadWriteByte(wb.byte[1]);
	  AT45DBSpiReadWriteByte(wb.byte[0]);
		AT45DBSpiReadWriteByte(0x00);						//don't cares
		uint8_t byte = AT45DBSpiReadWriteByte(DUMMY);		//read byte
	  AT45DB_CS_DIS();
	  return byte;
}

void ReadNbytesFromBuf1(uint8_t * ptr, uint16_t Nbytes, uint16_t addr)
{
	WordByteType wb;
   AT45DB_CS_EN();   
	 wb.word=(uint32_t)addr;
   AT45DBSpiReadWriteByte(Buf1Read);        
   AT45DBSpiReadWriteByte(0x00);						//don't cares
	 AT45DBSpiReadWriteByte(wb.byte[1]);
	 AT45DBSpiReadWriteByte(wb.byte[0]);
	 AT45DBSpiReadWriteByte(0x00);					  //don't cares
   while (Nbytes--)	*ptr++=AT45DBSpiReadWriteByte(DUMMY);
   AT45DB_CS_DIS();
}

void ReadNbytesFromBuf2(uint8_t * ptr, uint16_t Nbytes, uint16_t addr)
{
	WordByteType wb;
   AT45DB_CS_EN();   
	 wb.word=(uint32_t)addr;
   AT45DBSpiReadWriteByte(Buf2Read);      
   AT45DBSpiReadWriteByte(0x00);						//don't cares
	 AT45DBSpiReadWriteByte(wb.byte[1]);
	 AT45DBSpiReadWriteByte(wb.byte[0]);
	 AT45DBSpiReadWriteByte(0x00);						//don't cares
   while (Nbytes--)	*ptr++=AT45DBSpiReadWriteByte(DUMMY);
   AT45DB_CS_DIS();
}

void ReadPageToBuf1 (uint16_t addr_page)//528 bytes
{
	  WordByteType wb;
	  AT45DB_CS_EN();
	  wb.word=(uint32_t)addr_page;
	  wb.word<<=10;  
	  AT45DBSpiReadWriteByte(FlashToBuf1Transfer);	
		AT45DBSpiReadWriteByte(wb.byte[2]);
	  AT45DBSpiReadWriteByte(wb.byte[1]);
	  AT45DBSpiReadWriteByte(wb.byte[0]);
	  AT45DB_CS_DIS();
}


void ReadPageToBuf2 (uint16_t addr_page)//528 bytes
{
	  WordByteType wb;
	  AT45DB_CS_EN();
	  wb.word=(uint32_t)addr_page;
	  wb.word<<=10;  
	  AT45DBSpiReadWriteByte(FlashToBuf2Transfer);	
		AT45DBSpiReadWriteByte(wb.byte[2]);
	  AT45DBSpiReadWriteByte(wb.byte[1]);
	  AT45DBSpiReadWriteByte(wb.byte[0]);
	  AT45DB_CS_DIS();
}

void ReadPageDirect(uint8_t * ptr, uint16_t addr_page)
{
	 WordByteType wb;
	 AT45DB_CS_EN(); 
	 wb.word=(uint32_t)addr_page;
	 wb.word<<=10;  
   AT45DBSpiReadWriteByte(FlashPageRead);           
   AT45DBSpiReadWriteByte(wb.byte[2]);
	 AT45DBSpiReadWriteByte(wb.byte[1]);
	 AT45DBSpiReadWriteByte(wb.byte[0]);
	//--------------------------------------------
	AT45DBSpiReadWriteByte(0x00); //don't cares
	AT45DBSpiReadWriteByte(0x00); //don't cares
	AT45DBSpiReadWriteByte(0x00); //don't cares
	AT45DBSpiReadWriteByte(0x00); //don't cares
	//----------------------------------
	 uint16_t size=DF_PAGE_SIZE;
   while (size--)	*ptr++=AT45DBSpiReadWriteByte(DUMMY);	
   AT45DB_CS_DIS();
}

/************************************************************************
-------------------WRITE OPERATIONS--------------------------------------
*************************************************************************/

/*void WriteBuf1ToPageWithErase (uint16_t addr_page)
{
	  WordByteType wb;
	  AT45DB_CS_EN();
	  wb.word=(uint32_t)addr_page;
	  wb.word<<=10;
	  AT45DBSpiReadWriteByte(Buf1ToFlashWE);	
		AT45DBSpiReadWriteByte(wb.byte[2]);
	  AT45DBSpiReadWriteByte(wb.byte[1]);
	  AT45DBSpiReadWriteByte(wb.byte[0]);
	  AT45DB_CS_DIS();
}*/

void WriteBuf2ToPageWithErase (uint16_t addr_page)
{
	  WordByteType wb;
	  AT45DB_CS_EN();
	  wb.word=(uint32_t)addr_page;
	  wb.word<<=10;
	  AT45DBSpiReadWriteByte(Buf2ToFlashWE);	
		AT45DBSpiReadWriteByte(wb.byte[2]);
	  AT45DBSpiReadWriteByte(wb.byte[1]);
	  AT45DBSpiReadWriteByte(wb.byte[0]);
	  AT45DB_CS_DIS();
}

/*void WritePageImageToBuf1 (uint8_t *ptr)//load new page image (528 bytes) from RAM to buf
{
	  AT45DB_CS_EN();
    AT45DBSpiReadWriteByte(Buf1Write);	//buffer 1 write op-code
		AT45DBSpiReadWriteByte(0);	//don't cares
		AT45DBSpiReadWriteByte(0);//upper part of internal buffer address
		AT45DBSpiReadWriteByte(0);	//lower part of internal buffer address
		uint16_t size=DF_PAGE_SIZE;
    while (size--)	AT45DBSpiReadWriteByte(*ptr++);	
    AT45DB_CS_DIS();	
}*/

void FillBuf1 (uint8_t x)
{
	  AT45DB_CS_EN();
    AT45DBSpiReadWriteByte(Buf1Write);	//buffer 1 write op-code
		AT45DBSpiReadWriteByte(0);	//don't cares
		AT45DBSpiReadWriteByte(0);//upper part of internal buffer address
		AT45DBSpiReadWriteByte(0);	//lower part of internal buffer address
		uint16_t size=DF_PAGE_SIZE;
    while (size--)	AT45DBSpiReadWriteByte(x);	
    AT45DB_CS_DIS();	
}


void WritePageImageToBuf2 (uint8_t *ptr)//load new page image (528 bytes) from RAM to buf
{
	  AT45DB_CS_EN();
    AT45DBSpiReadWriteByte(Buf2Write);	//buffer 1 write op-code
		AT45DBSpiReadWriteByte(0);	//don't cares
		AT45DBSpiReadWriteByte(0);//upper part of internal buffer address
		AT45DBSpiReadWriteByte(0);	//lower part of internal buffer address
		uint16_t size=DF_PAGE_SIZE;
    while (size--)	AT45DBSpiReadWriteByte(*ptr++);
    AT45DB_CS_DIS();	
}

/*void WriteStrToBuf1 (uint16_t size,uint16_t addr,uint8_t *ptr)
{
	  WordByteType wb;
	  AT45DB_CS_EN();
	  wb.word=(uint32_t)addr;
    AT45DBSpiReadWriteByte(Buf1Write);	//buffer 1 write op-code
		AT45DBSpiReadWriteByte(0);	//don't cares
		AT45DBSpiReadWriteByte(wb.byte[1]);
	  AT45DBSpiReadWriteByte(wb.byte[0]);
	  while (size--)AT45DBSpiReadWriteByte(*ptr++);
    AT45DB_CS_DIS();	
}*/

/*void WriteBuf1ToPage (uint16_t addr_page)//528 bytes
{
	  WordByteType wb;
	  AT45DB_CS_EN();
	  wb.word=(uint32_t)addr_page;
	  wb.word<<=10;
	  AT45DBSpiReadWriteByte(Buf1ToFlash );		
		AT45DBSpiReadWriteByte(wb.byte[2]);
	  AT45DBSpiReadWriteByte(wb.byte[1]);
	  AT45DBSpiReadWriteByte(wb.byte[0]);
	  AT45DB_CS_DIS();
}*/


void WriteBuf2ToPage (uint16_t addr_page)//528 bytes
{
  	WordByteType wb;
	  AT45DB_CS_EN();
	  wb.word=(uint32_t)addr_page;
	  wb.word<<=10;
	  AT45DBSpiReadWriteByte(Buf2ToFlash );		
		AT45DBSpiReadWriteByte(wb.byte[2]);
	  AT45DBSpiReadWriteByte(wb.byte[1]);
	  AT45DBSpiReadWriteByte(wb.byte[0]);
	  AT45DB_CS_DIS();
}


/************************************************************************
-------------------COMPARE OPERATIONS--------------------------------------
*************************************************************************/

void Buf1ComparePage (uint16_t addr_page)//528 bytes
{
	  WordByteType wb;
	  AT45DB_CS_EN();
	  wb.word=(uint32_t)addr_page;
	  wb.word<<=10;
	  AT45DBSpiReadWriteByte(BUF1COMPAREPAGE);		
		AT45DBSpiReadWriteByte(wb.byte[2]);
	  AT45DBSpiReadWriteByte(wb.byte[1]);
	  AT45DBSpiReadWriteByte(wb.byte[0]);
	  AT45DB_CS_DIS();
}

void Buf2ComparePage (uint16_t addr_page)//528 bytes
{
	  WordByteType wb;
	  AT45DB_CS_EN();
	  wb.word=(uint32_t)addr_page;
	  wb.word<<=10;
	  AT45DBSpiReadWriteByte(BUF2COMPAREPAGE);		
		AT45DBSpiReadWriteByte(wb.byte[2]);
	  AT45DBSpiReadWriteByte(wb.byte[1]);
	  AT45DBSpiReadWriteByte(wb.byte[0]);
	  AT45DB_CS_DIS();
}


/************************************************************************
-------------------ERASING OPERATIONS--------------------------------------
*************************************************************************/
void ErasingPage (uint16_t addr_page)//528 bytes mode
{
	WordByteType wb;
	AT45DB_CS_EN();
	wb.word=(uint32_t)addr_page;
	wb.word<<=10;
  AT45DBSpiReadWriteByte(PageErase); 
	AT45DBSpiReadWriteByte(wb.byte[2]);
	AT45DBSpiReadWriteByte(wb.byte[1]);
	AT45DBSpiReadWriteByte(wb.byte[0]);
	AT45DB_CS_DIS();
} 


void ErasingBlock (uint16_t addr_block)
{
	WordByteType wb;
	AT45DB_CS_EN();
	wb.word=(uint32_t)addr_block;
	wb.word<<=13;
  AT45DBSpiReadWriteByte(BlockErase); 
  AT45DBSpiReadWriteByte(wb.byte[2]);
	AT45DBSpiReadWriteByte(wb.byte[1]);
	AT45DBSpiReadWriteByte(wb.byte[0]);
	AT45DB_CS_DIS();
} 

void ErasingSector0A (void)
{
	AT45DB_CS_EN();
  AT45DBSpiReadWriteByte(SectorErase); 
  AT45DBSpiReadWriteByte(0);
	AT45DBSpiReadWriteByte(0);
	AT45DBSpiReadWriteByte(0);
	AT45DB_CS_DIS();
} 

void ErasingSector0B (void)
{
WordByteType wb;
	AT45DB_CS_EN();
	wb.word=(uint32_t)1;
	wb.word<<=13;
  AT45DBSpiReadWriteByte(SectorErase); 
  AT45DBSpiReadWriteByte(wb.byte[2]);
	AT45DBSpiReadWriteByte(wb.byte[1]);
	AT45DBSpiReadWriteByte(wb.byte[0]);
	AT45DB_CS_DIS();
} 

void ErasingNonZeroSector (uint16_t addr_sector)
{
WordByteType wb;
	if(addr_sector==0)return;
	AT45DB_CS_EN();
	wb.word=(uint32_t)addr_sector;
	wb.word<<=17;
  AT45DBSpiReadWriteByte(SectorErase); 
  AT45DBSpiReadWriteByte(wb.byte[2]);
	AT45DBSpiReadWriteByte(wb.byte[1]);
	AT45DBSpiReadWriteByte(wb.byte[0]);
	AT45DB_CS_DIS();
} 




