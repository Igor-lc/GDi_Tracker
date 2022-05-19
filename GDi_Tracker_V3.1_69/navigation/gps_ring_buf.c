


#include "gps_ring_buf.h"

 static uint8_t* buf;
 static uint16_t count,size,tail,head;
 static uint8_t ring_buf_pop(void);
static void ring_buf_push(const uint8_t item);
        
void GpsRingBuf_init(uint8_t *buf_ptr,uint16_t Size)
{
     buf = buf_ptr;
     memset( buf, 0, Size );
     size = Size ;
     count=0;
     tail=0;
     head=0;
}


int GpsRingBuf_push(const uint8_t item)
{
    if(count == size)
	{
	return 0;
	}
	else
	{
    ring_buf_push(item);
    return 1;
	}
}

uint8_t ring_buf_pop_byte(void)
{
static uint8_t byte;
	__disable_irq();
    if(count) byte= ring_buf_pop();
    else byte= buf[tail];
	 __enable_irq();
	return byte;
}

void ring_buf_push(const uint8_t item)//add item to head
{
    buf[head] = item;
    head++;
    count++;
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

 uint8_t GpsRingBuf_count(void)
{
    return count;
}


