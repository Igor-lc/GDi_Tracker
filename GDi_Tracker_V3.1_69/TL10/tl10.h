
#pragma once

#include "rs485.h"


#define TL10_TX_CMD_HEAD                0x31
#define TL10_RX_CMD_HEAD                0x3E
#define TL10_RX_DELAY                   1000
//------TL10_READ_ID_CMD--------------------------
#define TL10_READ_ID_CMD                0x20
#define TL10_READ_ID_CMD_ANSWER_LEN       6
#define TL10_DATA_SIZE                    2
//---------------------------------------------
#define TL10_WRITE_NET_ADDR_CMD             0x56
#define TL10_WRITE_NET_ADDR_CMD_ANSWER_LEN  5
#define TL10_ADDR                           10

typedef __packed struct
{
	uint16_t id;
}TL10DataType;


typedef enum 
{
 TL10_READ_ID_CMD_NUM ,
}TL10CommandEnum;


extern Rs485CmdType TL10_cmd_table[];




