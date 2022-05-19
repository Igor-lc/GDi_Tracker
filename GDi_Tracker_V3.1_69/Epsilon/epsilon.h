
#ifndef EPSILON_H
#define EPSILON_H

#include "rs485.h"


#define EPSILON_TX_CMD_HEAD                0x31
#define EPSILON_RX_CMD_HEAD                0x3E
#define EPSILON_RX_DELAY                   1000
//------EPSILON_READ_DATA_CMD--------------------------
#define EPSILON_READ_DATA_CMD               0x06
#define EPSILON_READ_DATA_CMD_ANSWER_LEN    9
#define EPSILON_DATA_SIZE                   5
//------EPSILON_WRITE_NET_ADDR_CMD------------------------------------------
#define EPSILON_WRITE_NET_ADDR_CMD             0x56
#define EPSILON_WRITE_NET_ADDR_CMD_ANSWER_LEN  5

typedef __packed struct
{
	int8_t tdata;
	uint16_t ndata;
	uint16_t cdata;
}EpsilonDataType;


typedef enum 
{
 EPSILON_READ_DATA_CMD_NUM ,
 EPSILON_WRITE_NET_ADDR_CMD_NUM,
}EpsilonCommandEnum;


extern Rs485CmdType Epsilon_cmd_table[];

#endif


