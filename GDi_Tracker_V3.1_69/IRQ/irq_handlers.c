
#include "stm32f10x.h"
#include "public_messages.h"
#include "ring_buf.h"
#include "io_task.h"

void USART3_IRQHandler (void)
{
char ch;
   ch=USART3->SR;
   ch=USART3->DR;                
   USART_ClearITPendingBit(USART3, USART_IT_RXNE);
   ring_buf_push_byte(ch);   
}









