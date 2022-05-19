


#include "main.h"

__asm u32 revbit(u32 data)
 {
   rbit r0, r0
   bx lr
 }


u32 CalcCRC32(u8 *buffer,u32 size)
 {
   u32 i, j;
   u32 ui32;
   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC,ENABLE);

   CRC->CR=1;

   __asm("NOP");
	 __asm("NOP");
	 __asm("NOP");//delay for hardware ready

   i = size >> 2;

   while(i--)
   {
     ui32=*buffer;
     buffer ++;
     ui32=revbit(ui32);//reverse the bit order of input data
     CRC->DR=ui32;
   }

   ui32=CRC->DR;

   ui32=revbit(ui32);//reverse the bit order of output data

   i = size;

   while(i--)
   {
     ui32 ^= (u32)*buffer++;

     for(j=0; j<8; j++)
       if (ui32 & 1)
         ui32 = (ui32 >> 1) ^ 0xEDB88320;
       else
         ui32 >>= 1;
   }

   ui32^=0xffffffff;//xor with 0xffffffff
   return ui32;//now the output is compatible with windows/winzip/winrar
 }

 
 
 
