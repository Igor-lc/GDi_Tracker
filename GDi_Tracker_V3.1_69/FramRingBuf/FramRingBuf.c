

#include "crc32.h"
#include "fm25cl64.h"
#include "FramRingBuf.h"
#include "Control.h"
#include "disk.h"

static FramPointsRingBufType *rb_ptr;
static FramPointsRingBufType control_buf;

int32_t FramRingBuf_Init(FramPointsRingBufType *ptr,uint16_t size)
{
   rb_ptr=ptr;
 if (CopyFramPointsRingBufImageToRam()) return 0;
   else
      {
      rb_ptr->tail=rb_ptr->head=0;
      rb_ptr->size.to_send=0;
      rb_ptr->size.used=0;
			rb_ptr->point.count=0;
			rb_ptr->point.to_send=0;
      rb_ptr->size.total=size;
      if(0== WriteFramPointsRingBufImageToFram())return -1;
			if (CopyFramPointsRingBufImageToRam())return 1;
		  else return -1;
      }
}


int FramRingBuf_Format(void)
{
      rb_ptr->tail=rb_ptr->head=0;
      rb_ptr->size.to_send=0;
      rb_ptr->size.used=0;
			rb_ptr->point.count=0;
			rb_ptr->point.to_send=0;
    return  WriteFramPointsRingBufImageToFram();	
}


int FramRingBuf_AddToHead(QPointType *point_ptr)
{
   if (CopyFramPointsRingBufImageToRam())
      {
      uint8_t *read_ptr=(uint8_t*)point_ptr;
      uint8_t point_size=sizeof(point_ptr->gps)+sizeof(point_ptr->sensors_count)+sizeof(QSensorType)*point_ptr->sensors_count;
//---------------write sise--------------------------------				
      *(rb_ptr->buf + rb_ptr->head)=point_size;//write size
      if (++rb_ptr->head==rb_ptr->size.total)rb_ptr->head=0;
//---------------write item--------------------------------				
      for (uint8_t i=0;i<point_size;i++)//write item
         {
         *(rb_ptr->buf + rb_ptr->head)=*read_ptr++;
         if (++rb_ptr->head==rb_ptr->size.total)rb_ptr->head=0;
         }
//---------------write sise--------------------------------
		 *(rb_ptr->buf + rb_ptr->head)=point_size;//write size
      if (++rb_ptr->head==rb_ptr->size.total)rb_ptr->head=0;
//------------------------------------------------------------
				  rb_ptr->size.used+=(point_size+2);
				 rb_ptr->point.count++;
//-------------------------------------------------------------				 
      WriteFramPointsRingBufImageToFram();
//			FM25CL64_WriteWord(POINTS_TOTAL_ADDR,SysInfo.points_total);
      return 1;
      }
   else return 0;
}


/*void FramRingBuf_AddToHeadFast(QPointType *point_ptr)
{
      uint8_t *read_ptr=(uint8_t*)point_ptr;
      uint8_t point_size=sizeof(point_ptr->gps)+sizeof(point_ptr->sensors_count)+sizeof(QSensorType)*point_ptr->sensors_count;
//---------------write sise--------------------------------				
      *(rb_ptr->buf + rb_ptr->head)=point_size;//write size
      if (++rb_ptr->head==rb_ptr->size.total)rb_ptr->head=0;
//---------------write item--------------------------------				
      for (uint8_t i=0;i<point_size;i++)//write item
         {
         *(rb_ptr->buf + rb_ptr->head)=*read_ptr++;
         if (++rb_ptr->head==rb_ptr->size.total)rb_ptr->head=0;
         }
//---------------write sise--------------------------------
		 *(rb_ptr->buf + rb_ptr->head)=point_size;//write size
      if (++rb_ptr->head==rb_ptr->size.total)rb_ptr->head=0;
//------------------------------------------------------------
				  rb_ptr->size.used+=(point_size+2);
				 rb_ptr->point.count++;
}*/

int FramRingBuf_GetFromTail(QPointType *point_ptr)
{
	uint8_t point_size;
   if (CopyFramPointsRingBufImageToRam())
      {
      uint8_t *write_ptr=(uint8_t*)point_ptr;
//---------------read sise--------------------------------				
      point_size=*(rb_ptr->buf + rb_ptr->tail);
      if (++rb_ptr->tail==rb_ptr->size.total)rb_ptr->tail=0;
//-------------read item-----------------------------------				
      for (uint8_t i=0;i<point_size;i++)
         {
         *write_ptr++=*(rb_ptr->buf + rb_ptr->tail);
         if (++rb_ptr->tail==rb_ptr->size.total)rb_ptr->tail=0;
         }
//---------------read sise--------------------------------
			if (++rb_ptr->tail==rb_ptr->size.total)rb_ptr->tail=0;
				 //-------------------------------------------------------
				 rb_ptr->size.used-=(point_size+2);
				 rb_ptr->point.count--;
//-----------------------------------------------------				 
      WriteFramPointsRingBufImageToFram();
      return 1;
      }
   else return 0;
}

int FramRingBuf_GetFromHead(QPointType *point_ptr)
{
	uint8_t point_size, *write_ptr;
	uint16_t temp_ptr;
    if (CopyFramPointsRingBufImageToRam())
      {
      write_ptr=(uint8_t*)point_ptr;
			temp_ptr=rb_ptr->head;//set temp ptr to head
//---------------set temp ptr to size and read size--------------------------------	
      if (temp_ptr==0)temp_ptr=rb_ptr->size.total-1;
			else temp_ptr--;				
      point_size=*(rb_ptr->buf + temp_ptr);
//-------------set temp ptr to point-----------------------------------				
      for (uint8_t i=0;i<point_size;i++)
         {
           if (temp_ptr==0)temp_ptr=rb_ptr->size.total-1;
			     else temp_ptr--;	
         }
//---------------read point--------------------------------
			for (uint8_t i=0;i<point_size;i++)
         {
         *write_ptr++=*(rb_ptr->buf + temp_ptr);
           if (++temp_ptr==rb_ptr->size.total)temp_ptr=0;
         }
				 //---------------set head to new position--------------------------------
			for (uint8_t i=0;i<point_size+2;i++)
         {
          if (rb_ptr->head==0)rb_ptr->head=rb_ptr->size.total-1;
			    else rb_ptr->head--;	
         }
				 //-----------------------------------------------------
				 rb_ptr->size.used-=(point_size+2);
				 rb_ptr->point.count--;
//-----------------------------------------------------				 
      WriteFramPointsRingBufImageToFram();
      return 1;
      }
   else return 0;
}

/*void FramRingBuf_GetFromHeadFast(QPointType *point_ptr)
{
	uint8_t point_size, *write_ptr;
	uint16_t temp_ptr;
      write_ptr=(uint8_t*)point_ptr;
			temp_ptr=rb_ptr->head;//set temp ptr to head
//---------------set temp ptr to size and read size--------------------------------	
      if (temp_ptr==0)temp_ptr=rb_ptr->size.total-1;
			else temp_ptr--;				
      point_size=*(rb_ptr->buf + temp_ptr);
//-------------set temp ptr to point-----------------------------------				
      for (uint8_t i=0;i<point_size;i++)
         {
           if (temp_ptr==0)temp_ptr=rb_ptr->size.total-1;
			     else temp_ptr--;	
         }
//---------------read point--------------------------------
			for (uint8_t i=0;i<point_size;i++)
         {
         *write_ptr++=*(rb_ptr->buf + temp_ptr);
           if (++temp_ptr==rb_ptr->size.total)temp_ptr=0;
         }
				 //---------------set head to new position--------------------------------
			for (uint8_t i=0;i<point_size+2;i++)
         {
          if (rb_ptr->head==0)rb_ptr->head=rb_ptr->size.total-1;
			    else rb_ptr->head--;	
         }
				 //-----------------------------------------------------
				 rb_ptr->size.used-=(point_size+2);
				 rb_ptr->point.count--;
//--------------------------------------------------------------				  
}*/


int32_t FramRingBuf_CopyPointsFromTailToSendBuf(uint8_t *dest_ptr,uint16_t *dest_free_space_ptr,uint8_t *points_to_send_count)
{
   uint8_t point_size;
   uint16_t temp_tail;
   if (!CopyFramPointsRingBufImageToRam())return -1;
   else 
			{
      temp_tail=rb_ptr->tail;
      while ((*dest_free_space_ptr>=(point_size=*(rb_ptr->buf + temp_tail)))&&(rb_ptr->point.to_send<rb_ptr->point.count))
         {
						if (++temp_tail==rb_ptr->size.total)temp_tail=0;//skip size
						//-----------copy point--------------------------
         for (uint8_t i=0;i<point_size;i++)
            {
            *dest_ptr++=*(rb_ptr->buf + temp_tail);
            if (++temp_tail==rb_ptr->size.total)temp_tail=0;
            (*dest_free_space_ptr)--; 
            }
						//-------skip size----------------------------
						if (++temp_tail==rb_ptr->size.total)temp_tail=0;
						//-------------------------------------
						rb_ptr->size.to_send+=(point_size+2);
             rb_ptr->point.to_send++;
         }
				 *points_to_send_count=rb_ptr->point.to_send;
        if(0== WriteFramPointsRingBufImageToFram())return -1;
        else return rb_ptr->size.used;
			 }
}



int32_t FramRingBuf_CopyPoint(uint8_t *point_ptr,uint8_t *point_size)
{
   uint16_t temp_tail;
   if (!CopyFramPointsRingBufImageToRam())return -1;
    else 
			{
       temp_tail=rb_ptr->tail;
			 *point_size=*(rb_ptr->buf + temp_tail);
			 if (++temp_tail==rb_ptr->size.total)temp_tail=0;//skip size
						//-----------copy point--------------------------
         for (uint8_t i=0;i<*point_size;i++)
            {
            *point_ptr++=*(rb_ptr->buf + temp_tail);
            if (++temp_tail==rb_ptr->size.total)temp_tail=0;
            }
						rb_ptr->size.to_send+=(*point_size+2);
             rb_ptr->point.to_send++;
			 }
		 if(0==WriteFramPointsRingBufImageToFram())return -1;
     else return rb_ptr->size.used;
}

int FramRingBuf_GetFromTailToFlashBuf(uint8_t *dest_ptr,uint16_t *dest_free_space_ptr,uint8_t *Npoints)
{
   uint8_t point_size,points_to_buf;
	uint16_t dest_free_space;
    if (CopyFramPointsRingBufImageToRam())
      {
				dest_free_space=*dest_free_space_ptr;
				points_to_buf=0;
				while ((dest_free_space>=(point_size=*(rb_ptr->buf + rb_ptr->tail)))&&(rb_ptr->size.used-rb_ptr->size.to_send>0))
         {
         if (++rb_ptr->tail==rb_ptr->size.total)rb_ptr->tail=0;
         for (uint8_t i=0;i<point_size;i++)
            {
            *dest_ptr++=*(rb_ptr->buf + rb_ptr->tail);
            if (++rb_ptr->tail==rb_ptr->size.total)rb_ptr->tail=0;
            dest_free_space--; 
            }
						//-------skip size----------------------------
						if (++rb_ptr->tail==rb_ptr->size.total)rb_ptr->tail=0;
						//-------------------------------------
						rb_ptr->size.used-=(point_size+2);
						rb_ptr->point.count--;
            points_to_buf++;
         }
				*dest_free_space_ptr= dest_free_space;
				 *Npoints=points_to_buf;
      return WriteFramPointsRingBufImageToFram();
      }
   return 0;
}

/*void FramRingBuf_GetFromTailToFlashBufFast(uint8_t *dest_ptr,uint16_t *dest_free_space_ptr,uint8_t *Npoints)
{
   uint8_t point_size,points_to_buf=0;
	uint16_t dest_free_space;
	dest_free_space=*dest_free_space_ptr;
    
				while ((dest_free_space>=(point_size=*(rb_ptr->buf + rb_ptr->tail)))&&(rb_ptr->size.used-rb_ptr->size.to_send>0))
         {
         if (++rb_ptr->tail==rb_ptr->size.total)rb_ptr->tail=0;
         for (uint8_t i=0;i<point_size;i++)
            {
            *dest_ptr++=*(rb_ptr->buf + rb_ptr->tail);
            if (++rb_ptr->tail==rb_ptr->size.total)rb_ptr->tail=0;
            dest_free_space--; 
            }
						//-------skip size----------------------------
						if (++rb_ptr->tail==rb_ptr->size.total)rb_ptr->tail=0;
						//-------------------------------------
						rb_ptr->size.used-=(point_size+2);
						rb_ptr->point.count--;
            points_to_buf++;
         }
				*dest_free_space_ptr= dest_free_space;
				*Npoints=points_to_buf;     
}*/

int32_t FramRingBuf_UnlockPointsToSend(void)
{
	  FM25_ReadArray(POINTS_BUF_ADDR,(uint8_t*)rb_ptr,sizeof(FramPointsRingBufType));
	 if (!CopyFramPointsRingBufImageToRam())return -1;
   rb_ptr->point.to_send=0;
	  rb_ptr->size.to_send=0;
	 if(0==WriteFramPointsRingBufImageToFram())return -1;
	 return rb_ptr->size.used;
}

int32_t FramRingBuf_DeletePointsToSend(void)
{
	 uint8_t point_size;
	 if (!CopyFramPointsRingBufImageToRam())return -1;
  else while(rb_ptr->point.to_send>0)
	 {
//---------------read sise--------------------------------				
      point_size=*(rb_ptr->buf + rb_ptr->tail);
//-------------read item-----------------------------------				
      for (uint8_t i=0;i<point_size+2;i++)
         {
         if (++rb_ptr->tail==rb_ptr->size.total)rb_ptr->tail=0;
         }
//-----------------------------------------------------	
				 rb_ptr->size.used-=(point_size+2);
				 rb_ptr->size.to_send-=(point_size+2);
          rb_ptr->point.count--;	
         rb_ptr->point.to_send--;				 
	 }
	if(0==WriteFramPointsRingBufImageToFram())return -1;
	else  return rb_ptr->size.used;
}

int CopyFramPointsRingBufImageToRam(void)
{
	uint8_t try_counter=3;
	int result;
	do{
 FM25_ReadArray(POINTS_BUF_ADDR,(uint8_t*)rb_ptr,sizeof(FramPointsRingBufType));
		 result=(rb_ptr->crc==Crc32Eth((uint8_t*)rb_ptr,sizeof(FramPointsRingBufType)-sizeof(rb_ptr->crc)));
	}
   while(result==0&&try_counter-->0);
	return result;
}

int WriteFramPointsRingBufImageToFram(void)
{
	uint8_t try_counter=3;
	int result;
	do
		{
	  rb_ptr->crc=Crc32Eth((uint8_t*)rb_ptr,sizeof(FramPointsRingBufType)-sizeof(rb_ptr->crc));
    FM25_WriteArray(POINTS_BUF_ADDR,(uint8_t*)rb_ptr,sizeof(FramPointsRingBufType));
			
			FM25_ReadArray(POINTS_BUF_ADDR,(uint8_t*)&control_buf,sizeof(FramPointsRingBufType));
		 result=(control_buf.crc==Crc32Eth((uint8_t*)&control_buf,sizeof(FramPointsRingBufType)-sizeof(control_buf.crc)));
	  }
		 while(result==0&&try_counter-->0);
	   return result;
}

uint16_t FramRingBuf_size_used(void)
{
   return rb_ptr->size.used;
}


uint16_t FramRingBuf_size_to_send(void)
{
   return rb_ptr->size.to_send;
}

uint16_t FramRingBuf_size_to_flash(void)
{
   return rb_ptr->size.used-rb_ptr->size.to_send;
}






