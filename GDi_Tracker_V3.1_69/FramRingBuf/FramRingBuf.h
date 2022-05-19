
#ifndef _FRAM_RING_BUF
#define _FRAM_RING_BUF

#include <stdint.h>
#include "user_types.h"

extern uint16_t FramRingBuf_size_used(void);
extern uint16_t FramRingBuf_size_to_send(void);
extern uint16_t FramRingBuf_size_to_flash(void);
extern int FramRingBuf_AddToHead(QPointType *point_ptr);
extern int FramRingBuf_GetFromHead(QPointType *point_ptr);
extern int32_t FramRingBuf_Init(FramPointsRingBufType *ptr,uint16_t size);	
extern int32_t FramRingBuf_CopyPoint(uint8_t *point_ptr,uint8_t *point_size);
extern int32_t FramRingBuf_CopyPointsFromTailToSendBuf(uint8_t *dest_ptr,uint16_t *dest_free_space_ptr,uint8_t *points_count);
//extern int FramRingBuf_GetToFlashBuf(uint8_t *dest_ptr,uint16_t *dest_free_space_ptr,uint8_t *Npoints);
extern int32_t FramRingBuf_UnlockPointsToSend(void);
extern int32_t FramRingBuf_DeletePointsToSend(void);
extern int FramRingBuf_Format(void);
//extern void FramRingBuf_GetFromHeadFast(QPointType *point_ptr);
//extern void FramRingBuf_AddToHeadFast(QPointType *point_ptr);
extern int CopyFramPointsRingBufImageToRam(void);
extern int FramRingBuf_GetFromTailToFlashBuf(uint8_t *dest_ptr,uint16_t *dest_free_space_ptr,uint8_t *Npoints);
extern int WriteFramPointsRingBufImageToFram(void);


#endif

