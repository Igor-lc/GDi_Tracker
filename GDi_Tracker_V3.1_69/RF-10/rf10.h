
#pragma once

#include "rs485.h"

#define RF10_TX_CMD_HEAD                    0x33
#define RF10_RX_CMD_HEAD                    0x23
#define RF10_RX_DELAY                       620
//-----RF10_READ_CARD_ID_CMD--------------------------
#define RF10_READ_CARD_ID_CMD_ANSWER_LEN    9
#define RF10_READ_CARD_ID_CMD               0x06
#define RF10_CARD_ID_SIZE                   5
//-----RF10_WRITE_NET_ADDR_CMD---------------------
#define RF10_WRITE_NET_ADDR_CMD_ANSWER_LEN  5
#define RF10_WRITE_NET_ADDR_CMD             0x03
//-------RF10_RELE_CMD------------------------------
#define RF10_RELE_CMD_ANSWER_LEN            5
#define RF10_RELE_CMD                       0x09
#define RF10_RELE_ON_UC                     0x13
#define RF10_RELE_OFF_UC                    0x12
//--------------------------------------------------
#define	RF10_RELE_DATA_ID    2



typedef __packed struct
{
	uint8_t card_id[RF10_CARD_ID_SIZE];
	uint8_t addr;
}RF10DataType;


typedef enum 
{
 RF10_READ_CARD_ID_CMD_NUM ,
RF10_RELE_CMD_NUM,          	
RF10_WRITE_NET_ADDR_CMD_NUM,
}RF10_CommandEnum;


extern Rs485CmdType RF10_cmd_table[];



