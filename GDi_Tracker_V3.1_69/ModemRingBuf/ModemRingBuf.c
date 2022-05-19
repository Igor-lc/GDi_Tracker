

#include "ModemRingBuf.h"

 static uint8_t* buf;
  uint16_t count,size,tail,head;
 static uint8_t ring_buf_pop(void);//get item from tail
        
void ModemRingBuf_init(uint8_t *buf_ptr,uint16_t Size)
{
     buf = buf_ptr;
     memset( buf, 0, Size );
     size = Size ;
     count=0;
     tail=0;
     head=0;
}

void ring_buf_clear(void)
{
  count=tail=head=0;
}

/*BOOL ring_buf_write(const uint8_t* data, const uint_fast16_t cnt)
{
if( cnt > (size - count) )return __FALSE;
for(uint_fast8_t i = 0; i < cnt; i++)ring_buf_push(*(data++));
return __TRUE;
}*/

/*void ring_buf_read(uint8_t* const data, const uint_fast16_t cnt)
{
    uint_fast16_t nItems = cnt <= count ? cnt : count;
    for(uint_fast8_t i = 0; i < nItems; i++)data[i] = ring_buf_pop();
}*/


 


uint8_t ModemRingBuf_pop(void)
{
static uint8_t byte;
	__disable_irq();
    if(count) byte= ring_buf_pop();
    else byte= buf[tail];
	 __enable_irq();
	return byte;
}

void ModemRingBuf_push(const uint8_t item)//add item to head
{
    buf[head] = item;
    head++;
    count++;
	if(count>size)
	{
 		__nop();
	}
    if(head == size)head = 0;
}

uint8_t ring_buf_pop(void)//get item from tail
{
 uint8_t item;
    item = buf[tail];
    count--;
    tail++;
    if(tail == size)tail = 0;
    return item;
}

 uint8_t ModemRingBuf_count(void)
{
    return count;
}


