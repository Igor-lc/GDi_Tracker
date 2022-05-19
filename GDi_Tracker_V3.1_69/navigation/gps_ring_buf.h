

#include <stdint.h>
#include <string.h>


#ifndef _RING_BUF_H
#define _RING_BUF_H

extern void GpsRingBuf_init(uint8_t *buf_ptr,uint16_t Size);
extern uint8_t GpsRingBuf_count(void); 
extern int GpsRingBuf_push(const uint8_t item);
extern uint8_t  ring_buf_pop_byte(void);
   
#endif



