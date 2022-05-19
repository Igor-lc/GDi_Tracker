

#ifndef RS485_H
#define RS485_H

#include <stdint.h>
#include "qp_port.h"

#define  RS485_ANSWER_ERROR_TIMEOUT 10//10*5=50 ms
#define  RS485_RX_BUF_SIZE         50
#define  RS485_TX_BUF_SIZE         50



typedef  enum 
{
	RS485_NODEVICE=0,
	RS485_DEVICE_EPSILON,//1
	RS485_DEVICE_MR9,//2
	RS485_DEVICE_RF10,//3
	RS485_DEVICE_TL10,//4
	//--------------------------
	DEVICES_TABLE_END,
}RS485DevicesEnum;

typedef __packed struct
{
	uint8_t save;
	uint8_t device;
	uint8_t out_state;
	uint8_t addr;
	uint16_t crc16;
}ConfigRS485SensorType;


typedef	enum 
{
	rs485_off,
rs485_sensors_error,
	rs485_sensors_ok,
}RS485StateType;

typedef struct TxRxHandlerParamType TxRxHandlerParamType;
typedef   int(*txrxhp)(TxRxHandlerParamType* ptr);
typedef   void(*txfp)(uint8_t cmd_len);

struct TxRxHandlerParamType
{
	//once init
  uint8_t* rx_buf_ptr;
	uint8_t* tx_buf_ptr;
	QActive* sender;
  QActive* receiver;
	txfp txf;
	QSignal sig;	
	//dinamic init
	uint16_t rx_delay;
	uint8_t addr;
	uint8_t cmd_par;
	uint8_t rx_size;
	uint8_t current_sensor;
} ;

	 
	 typedef struct
	 {
		txrxhp tx_cmd;
    txrxhp rx_cmd;		 
	 }Rs485CmdType;

#endif



