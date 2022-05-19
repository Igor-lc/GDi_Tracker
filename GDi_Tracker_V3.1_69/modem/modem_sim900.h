

#ifndef SIM900_H
#define SIM900_H

#include "qp_port.h"
#include "user_types.h"
#include "ledindicator.h"

#define IMEI_BUF_SIZE 16
#define IMSI_BUF_SIZE 19

typedef enum
{
ExitIfError,
NextIfError	
}ErrorTransitionEnum;

typedef struct
{
	QStateHandler handler;
	ErrorTransitionEnum error_tran;
	uint16_t timeout_err;
	uint16_t timeout_ok;
}StateType;

typedef struct sim900Tag 
{       
  QFsm super;
  StateType const *sst_ptr;
	ConfigServerType *server_ptr;
	DataBufType send_packet;
  DataBufType server_data;
	uint8_t done;
struct 
    {
		unsigned unused:3;
    unsigned call_ready:1;
		unsigned roaming:1;
	  unsigned next_ok:1;
		unsigned exit_error:1;
		unsigned exit_ok:1;
    }flag;	
	uint16_t 	tick_100ms_counter;
	uint16_t counter;
	char imei[IMEI_BUF_SIZE];
	char imsi[IMSI_BUF_SIZE];
	char command_buf[100]; 
} SIM900;

#define UART_MODEM USART1
#define MODEM_UART_GPIO GPIOB

#define MODEM_TX_PIN   GPIO_Pin_6
#define MODEM_RX_PIN   GPIO_Pin_7

//---------------------------------------
//#define MODEM_VCC_GPIO GPIOA
//#define MODEM_VCC_PIN  GPIO_Pin_12
#define MODEM_VCC_GPIO GPIOB
#define MODEM_VCC_PIN  GPIO_Pin_4//lsmsck

//#define MODEM_POWER_OFF()   do{MODEM_VCC_GPIO->BRR = MODEM_VCC_PIN; LED_GPS_OFF();}while(0)
//#define MODEM_POWER_ON()    do{MODEM_VCC_GPIO->BSRR = MODEM_VCC_PIN;LED_GPS_ON();}while(0)
#define MODEM_POWER_OFF()   MODEM_VCC_GPIO->BRR = MODEM_VCC_PIN
#define MODEM_POWER_ON()    MODEM_VCC_GPIO->BSRR = MODEM_VCC_PIN
//-------------------------------------------------------------
#define POWERKEY_GPIO GPIOB
#define POWERKEY_PIN  GPIO_Pin_8
#define POWERKEY_DOWN()	  POWERKEY_GPIO->BSRR = POWERKEY_PIN
#define POWERKEY_UP()	    POWERKEY_GPIO->BRR = POWERKEY_PIN

#define SIM_SELECT_GPIO GPIOB
#define SIM_SELECT_PIN  GPIO_Pin_9
#define SELECT_SIM2()	  SIM_SELECT_GPIO->BSRR = SIM_SELECT_PIN
#define SELECT_SIM1()	  SIM_SELECT_GPIO->BRR =  SIM_SELECT_PIN


#define STATUS_GPIO GPIOB
#define STATUS_PIN  GPIO_Pin_5
#define GET_MODEM_STATUS()  ((STATUS_GPIO->IDR & STATUS_PIN)?1:0)

extern void ModemTxPin_deinit(void);
extern void ModemRxPin_deinit(void);
extern int ModemSendCommand(const char *command);
extern int ModemSendData(const uint8_t *data_ptr,uint16_t const size);
extern void ModemSM_ctor(SIM900 *me); 
#define ModemSM_init(me_)           QFsm_init    ((QFsm *)(me_), (QEvent *)0)
#define ModemSM_dispatch(me_, e_)   QFsm_dispatch((QFsm *)(me_), e_)

#endif

