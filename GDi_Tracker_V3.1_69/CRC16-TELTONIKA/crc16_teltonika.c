#include <stdint.h>


uint16_t crc16_teltonika(uint8_t *pData,uint32_t size)
{
	uint16_t result=0,val;
	for(uint32_t i=0;i<size;i++)
	{
		val=(uint16_t)*(pData+i);
		result^=val;
		for(uint32_t j=0;j<8;j++)
		{
			result=result&0x0001?(result>>1)^0xA001:result>>1;
		}
	}
	
	return result;
}
