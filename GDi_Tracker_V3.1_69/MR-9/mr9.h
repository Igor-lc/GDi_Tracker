
#ifndef MR9_H
#define MR9_H

#include "rs485.h"

#define MR9_TX_CMD_HEAD                    0x33
#define MR9_RX_DELAY                       620
//-----MR9_READ_CARD_ID_CMD--------------------------
#define MR9_READ_CARD_ID_CMD_ANSWER_LEN    9
#define MR9_READ_CARD_ID_CMD               0x06
#define MR9_CARD_ID_SIZE                   5
//-----MR9_WRITE_NET_ADDR_CMD---------------------
#define MR9_WRITE_NET_ADDR_CMD_ANSWER_LEN  5
#define MR9_WRITE_NET_ADDR_CMD             0x03
//-------MR9_RELE_CMD------------------------------
#define MR9_RELE_CMD_ANSWER_LEN            5
#define MR9_RELE_CMD                       0x09
#define MR9_RELE_ON_UC                     0x13
#define MR9_RELE_OFF_UC                    0x12
//--------------------------------------------------
#define	MR9_CARDID_DATA_ID  11
#define	MR9_RELE_DATA_ID    2



typedef __packed struct
{
	uint8_t card_id[MR9_CARD_ID_SIZE];
	uint8_t addr;
}MR9DataType;


typedef enum 
{
 MR9_READ_CARD_ID_COMMAND ,
MR9_RELE_COMMAND          ,	
MR9_WRITE_NET_ADDR_COMMAND,
}MR9_CommandEnum;


extern Rs485CmdType MR9_cmd_table[];

#endif


