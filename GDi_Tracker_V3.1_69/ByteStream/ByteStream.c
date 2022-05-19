
#include "ByteStream.h"


void ShortToBigEndianStream(uint8_t *buf,uint16_t sh)
{
	*buf++=*(1+(uint8_t *)(&sh));
	*buf=*((uint8_t *)&sh);
}

  void IntToBigEndianStream(uint8_t *buf,uint32_t in)
{
	for(int i=3;i>=0;i--)*buf++=*(i+(uint8_t *)(&in));
}

void LongToBigEndianStream(uint8_t *buf,uint64_t ln)
{
	for(int i=7;i>=0;i--)*buf++=*(i+(uint8_t *)(&ln));
}

void BufToLittleEndianInt(uint8_t *buf,uint32_t *in)
{
	for(int i=3;i>=0;i--)*(i+(uint8_t *)in)=*buf++;
}




