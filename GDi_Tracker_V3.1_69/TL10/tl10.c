

#include "tl10.h"
#include "crc8.h"
#include "user_types.h"
#include <string.h>
#include "io.h"


int TL10ReadId_tx_handler(TxRxHandlerParamType* ptr)
   {
	//	delay+=100;
   uint8_t cmd_len,i=0;
   ptr->rx_size=TL10_READ_ID_CMD_ANSWER_LEN;	 
   ptr->rx_delay=	TL10_RX_DELAY;	 
   ptr->tx_buf_ptr[i++]=TL10_TX_CMD_HEAD;
   ptr->tx_buf_ptr[i++]=ptr->addr;
   ptr->tx_buf_ptr[i++]=TL10_READ_ID_CMD; 
   cmd_len=i;
   ptr->tx_buf_ptr[i++]=CalcCrc8(ptr->tx_buf_ptr,cmd_len);
   ptr->txf(i);
		 return 1;
   }
	 
	 int TL10ReadId_rx_handler(TxRxHandlerParamType* ptr)
   {
		 static uint8_t err_counter=0;
   uint8_t crc=CalcCrc8(ptr->rx_buf_ptr,TL10_READ_ID_CMD_ANSWER_LEN-1);
   if (crc==ptr->rx_buf_ptr[TL10_READ_ID_CMD_ANSWER_LEN-1])
      {
        //memcpy((uint8_t*)&IoData.rs485[ptr->current_sensor],3+ptr->rx_buf_ptr,TL10_DATA_SIZE); 
        memcpy((uint8_t*)&SysInfo.tl10_data.id,3+ptr->rx_buf_ptr,TL10_DATA_SIZE);				
  			err_counter=0;
      return 1; 
      }
   else
      {
      if(++err_counter==10) memset((uint8_t*)&SysInfo.tl10_data.id,0,TL10_DATA_SIZE);
      return 0;   
      }
   }
	 
	 
	  //--------------------WRITE ADDR------------------------------------------	 
	 int TL10WriteAddr_tx_handler(TxRxHandlerParamType* ptr)
   {
   uint8_t cmd_len,i=0;  
	 ptr->rx_size=TL10_WRITE_NET_ADDR_CMD_ANSWER_LEN;	
	 ptr->rx_delay=	TL10_RX_DELAY;	
   ptr->tx_buf_ptr[i++]=TL10_TX_CMD_HEAD;
   ptr->tx_buf_ptr[i++]=0xFF;
   ptr->tx_buf_ptr[i++]=TL10_WRITE_NET_ADDR_CMD;
   ptr->tx_buf_ptr[i++]=ptr->addr; 		 
   cmd_len=i;
   ptr->tx_buf_ptr[i++]=CalcCrc8(ptr->tx_buf_ptr,cmd_len);
   ptr->txf(i);
		 return 1;
   }
	 
	  int TL10WriteAddr_rx_handler(TxRxHandlerParamType* ptr)
   {
   uint8_t crc=CalcCrc8(ptr->rx_buf_ptr,EPSILON_WRITE_NET_ADDR_CMD_ANSWER_LEN-1);
   if (crc==ptr->rx_buf_ptr[TL10_WRITE_NET_ADDR_CMD_ANSWER_LEN-1])
      {
//				SysInfo.io_data.epsilon.addr=*(1+ptr->rx_buf_ptr);
      if(ptr->addr==*(1+ptr->rx_buf_ptr))return 1; 
      }
   return 0; 
   }
	 
	 
	  Rs485CmdType TL10_cmd_table[]=
	 {
		{TL10ReadId_tx_handler,TL10ReadId_rx_handler},
	 };

