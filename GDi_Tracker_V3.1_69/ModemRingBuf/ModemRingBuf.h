
#include <stdint.h>
#include <string.h>
#include <RTL.h>

#ifndef _RING_BUF_H
#define _RING_BUF_H

extern void ring_buf_clear(void);
extern void ModemRingBuf_init(uint8_t *buf_ptr,uint16_t Size);
extern void ModemRingBuf_push(const uint8_t item);
extern uint8_t ModemRingBuf_pop(void);
extern uint8_t ModemRingBuf_count(void);   
#endif

