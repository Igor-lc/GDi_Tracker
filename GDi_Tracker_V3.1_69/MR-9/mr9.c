
#include <stdint.h>
#include "rs485.h"
#include "mr9.h"
#include "UserTypes.h"
#include <string.h>
#include "io.h"
#include "crc8.h"

//----------------READ CARD ID---------------------------------------------------------------
bool MR9ReadCardId_tx_handler(TxRxHandlerParamType* ptr)
   {
   uint8_t cmd_len,i=0; 
	 ptr->rx_size=MR9_READ_CARD_ID_CMD_ANSWER_LEN;
   ptr->rx_delay=MR9_RX_DELAY;			 
   ptr->tx_buf_ptr[i++]=MR9_TX_CMD_HEAD;
   ptr->tx_buf_ptr[i++]=ptr->addr;
   ptr->tx_buf_ptr[i++]=MR9_READ_CARD_ID_CMD; 
   cmd_len=i;
   ptr->tx_buf_ptr[i++]=CalcCrc8(ptr->tx_buf_ptr,cmd_len);
   ptr->txf(i);
		 return true;
   }

bool MR9ReadCardId_rx_handler(TxRxHandlerParamType* ptr)
   {
   uint8_t crc=CalcCrc8(ptr->rx_buf_ptr,MR9_READ_CARD_ID_CMD_ANSWER_LEN-1);
		 uint8_t addr;
   if (crc==ptr->rx_buf_ptr[MR9_READ_CARD_ID_CMD_ANSWER_LEN-1])
      {
				addr=*(1+ptr->rx_buf_ptr);
      SysInfo.io_data.mr9.addr=addr;
      if (0!=memcmp(3+ptr->rx_buf_ptr,SysInfo.io_data.mr9.card_id,MR9_CARD_ID_SIZE))//new card id 
         {
         //memcpy(SysInfo.io_data.mr9.card_id,3+ptr->rx_buf_ptr,MR9_CARD_ID_SIZE);
				//	memcpy((uint8_t*)&io_data.rs485[ptr->current_sensor].data,0,SENSOR_DATA_SIZE);
         SensorEvt *pe = Q_NEW(SensorEvt,ptr->sig);
				 pe->sensor.id=0x80|(RS485_SENSOR_GROUP_ID+addr);
         pe->sensor.descriptor.data_id =MR9_CARDID_DATA_ID;
				 pe->sensor.descriptor.data_size=MR9_CARD_ID_SIZE;
				 memcpy(pe->sensor.data.any_data,3+ptr->rx_buf_ptr,MR9_CARD_ID_SIZE);
         QACTIVE_POST(ptr->receiver, &pe->super,ptr->sender);
         }
      return true; 
      }
   else
      {
			//memset((uint8_t*)&io_data.rs485[ptr->current_sensor].data,0,SENSOR_DATA_SIZE);
      return false; 
      }
   }
//----------------RELE--------------------------------------------------------------------------
	 bool MR9Rele_tx_handler(TxRxHandlerParamType* ptr)
   {
   uint8_t cmd_len,i=0; 
   ptr->rx_size=MR9_RELE_CMD_ANSWER_LEN;		 
	 ptr->rx_delay=	MR9_RX_DELAY;	
   ptr->tx_buf_ptr[i++]=MR9_TX_CMD_HEAD;
   ptr->tx_buf_ptr[i++]=ptr->addr;
   ptr->tx_buf_ptr[i++]=MR9_RELE_CMD; 
	 ptr->tx_buf_ptr[i++]=ptr->cmd_par;
   cmd_len=i;
   ptr->tx_buf_ptr[i++]=CalcCrc8(ptr->tx_buf_ptr,cmd_len);
   ptr->txf(i);
		return true; 
   }
	 
	 
	 bool MR9Rele_rx_handler(TxRxHandlerParamType* ptr)
   {
   uint8_t crc=CalcCrc8(ptr->rx_buf_ptr,MR9_RELE_CMD_ANSWER_LEN-1);
   if (crc==ptr->rx_buf_ptr[MR9_RELE_CMD_ANSWER_LEN-1])
	 {
		  if(0==*(3+ptr->rx_buf_ptr))return true; 
	 }
   return false; 
   } 
	 
//--------------------WRITE ADDR------------------------------------------	 
	 bool MR9WriteAddr_tx_handler(TxRxHandlerParamType* ptr)
   {
   uint8_t cmd_len,i=0;
	 ptr->rx_size=MR9_WRITE_NET_ADDR_CMD_ANSWER_LEN;		
   ptr->rx_delay=	MR9_RX_DELAY;			 
   ptr->tx_buf_ptr[i++]=MR9_TX_CMD_HEAD;
   ptr->tx_buf_ptr[i++]=0xFF;
   ptr->tx_buf_ptr[i++]=MR9_WRITE_NET_ADDR_CMD;
   ptr->tx_buf_ptr[i++]=ptr->addr; 		 
   cmd_len=i;
   ptr->tx_buf_ptr[i++]=CalcCrc8(ptr->tx_buf_ptr,cmd_len);
   ptr->txf(i);
		 return true;
   }
	 
	 bool MR9WriteAddr_rx_handler(TxRxHandlerParamType* ptr)
   {
   uint8_t crc=CalcCrc8(ptr->rx_buf_ptr,MR9_WRITE_NET_ADDR_CMD_ANSWER_LEN-1);
   if (crc==ptr->rx_buf_ptr[MR9_WRITE_NET_ADDR_CMD_ANSWER_LEN-1])
      {
				SysInfo.io_data.mr9.addr=*(1+ptr->rx_buf_ptr);
      if(ptr->addr==*(1+ptr->rx_buf_ptr))return true; 
      }
   return false; 
   }
	  
	 Rs485CmdType MR9_cmd_table[]=
	 {
		{MR9ReadCardId_tx_handler,MR9ReadCardId_rx_handler}, 
		{MR9Rele_tx_handler,MR9Rele_rx_handler},
		{MR9WriteAddr_tx_handler,MR9WriteAddr_rx_handler},
	 };
	 
	/*txrxhp MR9_cmd_table[]=
{
	//-----------------------------------------
  (txrxhp)MR9ReadCardId_tx_handler,  //0
	(txrxhp)MR9ReadCardId_rx_handler,  //1
//-----------------------------------------------
	 (txrxhp)MR9Rele_tx_handler,  //2
	 (txrxhp)MR9Rele_rx_handler,  //3
	//-----------------------------------------------
	 (txrxhp)MR9WriteAddr_tx_handler,  //4
	 (txrxhp)MR9WriteAddr_rx_handler,  //5
	//---------------------------------------------
};*/
	 
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



