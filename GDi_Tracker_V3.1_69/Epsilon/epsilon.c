

#include <stdint.h>
#include <stdlib.h>
#include "rs485.h"
#include "epsilon.h"
#include "user_types.h"
#include <string.h>
#include "io.h"
#include "crc8.h"

//----------------READ DATA---------------------------------------------------------------
int EpsilonReadData_tx_handler(TxRxHandlerParamType* ptr)
   {
   uint8_t cmd_len,i=0;
   ptr->rx_size=EPSILON_READ_DATA_CMD_ANSWER_LEN;	 
   ptr->rx_delay=	EPSILON_RX_DELAY;	 
   ptr->tx_buf_ptr[i++]=EPSILON_TX_CMD_HEAD;
   ptr->tx_buf_ptr[i++]=ptr->addr;
   ptr->tx_buf_ptr[i++]=EPSILON_READ_DATA_CMD; 
   cmd_len=i;
   ptr->tx_buf_ptr[i++]=CalcCrc8(ptr->tx_buf_ptr,cmd_len);
   ptr->txf(i);
		 return 1;
   }
	 
	 int EpsilonReadData_rx_handler(TxRxHandlerParamType* ptr)
   {
		 static uint8_t err_counter=0;
   uint8_t crc=CalcCrc8(ptr->rx_buf_ptr,EPSILON_READ_DATA_CMD_ANSWER_LEN-1);
   if (crc==ptr->rx_buf_ptr[EPSILON_READ_DATA_CMD_ANSWER_LEN-1])
      {
//         SysInfo.io_data.epsilon.addr=*(1+ptr->rx_buf_ptr);
        memcpy((uint8_t*)&IoData.rs485[ptr->current_sensor],3+ptr->rx_buf_ptr,EPSILON_DATA_SIZE);
					//fuel_level_old=((EpsilonDataType*)&IoData.rs485[ptr->current_sensor])->ndata;
  			err_counter=0;
      return 1; 
      }
   else
      {
      if(++err_counter==5) memset((uint8_t*)&IoData.rs485[ptr->current_sensor],0,SENSOR_DATA_SIZE);
      return 0;   
      }
   }
	 
	 //--------------------WRITE ADDR------------------------------------------	 
	 int EpsilonWriteAddr_tx_handler(TxRxHandlerParamType* ptr)
   {
   uint8_t cmd_len,i=0;  
	 ptr->rx_size=EPSILON_WRITE_NET_ADDR_CMD_ANSWER_LEN;	
	 ptr->rx_delay=	EPSILON_RX_DELAY;	
   ptr->tx_buf_ptr[i++]=EPSILON_TX_CMD_HEAD;
   ptr->tx_buf_ptr[i++]=0xFF;
   ptr->tx_buf_ptr[i++]=EPSILON_WRITE_NET_ADDR_CMD;
   ptr->tx_buf_ptr[i++]=ptr->addr; 		 
   cmd_len=i;
   ptr->tx_buf_ptr[i++]=CalcCrc8(ptr->tx_buf_ptr,cmd_len);
   ptr->txf(i);
		 return 1;
   }
	 
	 int EpsilonWriteAddr_rx_handler(TxRxHandlerParamType* ptr)
   {
   uint8_t crc=CalcCrc8(ptr->rx_buf_ptr,EPSILON_WRITE_NET_ADDR_CMD_ANSWER_LEN-1);
   if (crc==ptr->rx_buf_ptr[EPSILON_WRITE_NET_ADDR_CMD_ANSWER_LEN-1])
      {
//				SysInfo.io_data.epsilon.addr=*(1+ptr->rx_buf_ptr);
      if(ptr->addr==*(1+ptr->rx_buf_ptr))return 1; 
      }
   return 0; 
   }
	 
	  Rs485CmdType Epsilon_cmd_table[]=
	 {
		{EpsilonReadData_tx_handler,EpsilonReadData_rx_handler},
    {EpsilonWriteAddr_tx_handler,EpsilonWriteAddr_rx_handler},		
		
	 };
	 
