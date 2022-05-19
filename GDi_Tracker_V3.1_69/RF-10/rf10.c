
#include <stdint.h>
#include "rs485.h"
#include "rf10.h"
#include <string.h>
#include "io.h"
#include "crc8.h"

//----------------READ CARD ID---------------------------------------------------------------
int RF10ReadCardId_tx_handler(TxRxHandlerParamType* ptr)
   {
   uint8_t cmd_len,i=0; 
	 ptr->rx_size=RF10_READ_CARD_ID_CMD_ANSWER_LEN;
   ptr->rx_delay=RF10_RX_DELAY;			 
   ptr->tx_buf_ptr[i++]=RF10_TX_CMD_HEAD;
   ptr->tx_buf_ptr[i++]=ptr->addr;
   ptr->tx_buf_ptr[i++]=RF10_READ_CARD_ID_CMD; 
   cmd_len=i;
   ptr->tx_buf_ptr[i++]=CalcCrc8(ptr->tx_buf_ptr,cmd_len);
   ptr->txf(i);
		 return 1;
   }

int RF10ReadCardId_rx_handler(TxRxHandlerParamType* ptr)
   {
   uint8_t crc=CalcCrc8(ptr->rx_buf_ptr,RF10_READ_CARD_ID_CMD_ANSWER_LEN-1);
		 uint8_t addr;
   if (crc==ptr->rx_buf_ptr[RF10_READ_CARD_ID_CMD_ANSWER_LEN-1])
      {
				addr=*(1+ptr->rx_buf_ptr);
      SysInfo.rf10_data.addr=addr;
      if (0!=memcmp(3+ptr->rx_buf_ptr,SysInfo.rf10_data.card_id,RF10_CARD_ID_SIZE))//new card id 
         {
         memcpy(SysInfo.rf10_data.card_id,3+ptr->rx_buf_ptr,RF10_CARD_ID_SIZE);
				 //memcpy((uint8_t*)&io_data.rs485[ptr->current_sensor].data,0,SENSOR_DATA_SIZE);
         SensorEvt *pe = Q_NEW(SensorEvt,ptr->sig);
				 pe->sensor.id=EVT_ID|(RS485_SENSOR_GROUP_ID+addr);
         pe->sensor.descriptor.data_id =IO_IDENTIFIER ;
				 pe->sensor.descriptor.data_size=RF10_CARD_ID_SIZE;
				 memcpy(pe->sensor.data.any_data,3+ptr->rx_buf_ptr,RF10_CARD_ID_SIZE);
         QACTIVE_POST(ptr->receiver, &pe->super,ptr->sender);
         }
      return 1; 
      }
   else
      {
			//memset((uint8_t*)&io_data.rs485[ptr->current_sensor].data,0,SENSOR_DATA_SIZE);
      return 0; 
      }
   }
//----------------RELE--------------------------------------------------------------------------
	 int RF10Rele_tx_handler(TxRxHandlerParamType* ptr)
   {
   uint8_t cmd_len,i=0; 
   ptr->rx_size=RF10_RELE_CMD_ANSWER_LEN;		 
	 ptr->rx_delay=	RF10_RX_DELAY;	
   ptr->tx_buf_ptr[i++]=RF10_TX_CMD_HEAD;
   ptr->tx_buf_ptr[i++]=ptr->addr;
   ptr->tx_buf_ptr[i++]=RF10_RELE_CMD; 
	 ptr->tx_buf_ptr[i++]=ptr->cmd_par;
   cmd_len=i;
   ptr->tx_buf_ptr[i++]=CalcCrc8(ptr->tx_buf_ptr,cmd_len);
   ptr->txf(i);
		return 1; 
   }
	 
	 
	 int RF10Rele_rx_handler(TxRxHandlerParamType* ptr)
   {
   uint8_t crc=CalcCrc8(ptr->rx_buf_ptr,RF10_RELE_CMD_ANSWER_LEN-1);
   if (crc==ptr->rx_buf_ptr[RF10_RELE_CMD_ANSWER_LEN-1])
	 {
		  if(0==*(3+ptr->rx_buf_ptr))return 1; 
	 }
   return 0; 
   } 
	 
//--------------------WRITE ADDR------------------------------------------	 
	 int RF10WriteAddr_tx_handler(TxRxHandlerParamType* ptr)
   {
   uint8_t cmd_len,i=0;
	 ptr->rx_size=RF10_WRITE_NET_ADDR_CMD_ANSWER_LEN;		
   ptr->rx_delay=	RF10_RX_DELAY;			 
   ptr->tx_buf_ptr[i++]=RF10_TX_CMD_HEAD;
   ptr->tx_buf_ptr[i++]=0xFF;
   ptr->tx_buf_ptr[i++]=RF10_WRITE_NET_ADDR_CMD;
   ptr->tx_buf_ptr[i++]=ptr->addr; 		 
   cmd_len=i;
   ptr->tx_buf_ptr[i++]=CalcCrc8(ptr->tx_buf_ptr,cmd_len);
   ptr->txf(i);
		 return 1;
   }
	 
	 int RF10WriteAddr_rx_handler(TxRxHandlerParamType* ptr)
   {
   uint8_t crc=CalcCrc8(ptr->rx_buf_ptr,RF10_WRITE_NET_ADDR_CMD_ANSWER_LEN-1);
   if (crc==ptr->rx_buf_ptr[RF10_WRITE_NET_ADDR_CMD_ANSWER_LEN-1])
      {
				SysInfo.rf10_data.addr=*(1+ptr->rx_buf_ptr);
      if(ptr->addr==*(1+ptr->rx_buf_ptr))return 1; 
      }
   return 0; 
   }
	  
	 Rs485CmdType RF10_cmd_table[]=
	 {
		{RF10ReadCardId_tx_handler,RF10ReadCardId_rx_handler}, 
		{RF10Rele_tx_handler,RF10Rele_rx_handler},
		{RF10WriteAddr_tx_handler,RF10WriteAddr_rx_handler},
	 };
	 
	
	 
/*static void MR9ReadCardId_answer_handler(IoEvtParamType* ptr)
{
 if (0!=memcmp(ptr->new_data_ptr,ptr->old_data_ptr,CARD_ID_SIZE))//new card id 
 {
   memcpy(((MR9DataType*)ptr->old_data_ptr)->card_id,3+(uint8_t*)ptr->new_data_ptr,CARD_ID_SIZE);
   ((MR9DataType*)ptr->old_data_ptr)->addr=*(1+(uint8_t*)ptr->new_data_ptr);
   DataPtrEvt *pe = Q_NEW(DataPtrEvt, ptr->sig);
   pe->data.u32_data=MR9_SENSOR;
   QACTIVE_POST(ptr->receiver, &pe->super,ptr->sender);
 }	 
}*/

/*const uint8_t read_card_id_cmd[]= {0x33, 0x01, 0x06};
const Rs485CmdType MR9_ReadCardIdCmd=
{
  read_card_id_cmd,
  sizeof(read_card_id_cmd),
  READ_CARD_ID_CMD_ANSWER_LEN,
  MR9ReadCardId_answer_handler,
  NULL,
};*/

/*const uint8_t write_net_addr_cmd[]= {0x33, 0x01, 0x03,RS485_NET_ADDR_DEF};
const Rs485CmdType MR9_WriteNetAddrCmd=
{
  write_net_addr_cmd,
  sizeof(write_net_addr_cmd),
  WRITE_NET_ADDR_CMD_ANSWER_LEN,
  NULL,
};

const uint8_t actuator_on_cmd[]= {0x33, 0x01, 0x09,0x13};
const Rs485CmdType MR9_ActuatorOnCmd=
{
  actuator_on_cmd,
  sizeof(actuator_on_cmd),
  ACTUATOR_CMD_ANSWER_LEN,
  NULL,
};

const uint8_t actuator_off_cmd[]= {0x33, 0x01, 0x09,0x12};
const Rs485CmdType MR9_ActuatorOffCmd=
{
  actuator_off_cmd,
  sizeof(actuator_off_cmd),
  ACTUATOR_CMD_ANSWER_LEN,
  NULL,
};*/


/*const Rs485CmdType* MR9_cmd_table[]=
{
  &MR9_ReadCardIdCmd,  //0
  &MR9_WriteNetAddrCmd,//1
  &MR9_ActuatorOnCmd,  //2
  &MR9_ActuatorOffCmd, //3
  NULL,
};*/



