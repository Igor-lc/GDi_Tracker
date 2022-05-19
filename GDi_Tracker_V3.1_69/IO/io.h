

#ifndef IO_H
#define IO_H

#include "user_types.h"
#include "config_sys.h"
#include "rs485.h"

//#define RS485_UART_DMA_MODE

#define IO_AD_VREF		 (float)3.0
#define IO_AD_INTEGR_K (float)0.08
#define IO_BAT_DIV_K   (float)((100.0+100.0)*1.341/100.0)
#define IO_POW_DIV_K	 (float)((15.0+1.0)/1.0)

typedef  enum IoSignals
{
	IO_TIMEOUT_SIG=MAX_PUB_SIG,
	IO_RX_COMPLETE_SIG,
	IO_ON_SIG,
}IoSignalsType;




/*
typedef enum
{
	PULSE_COUNTER_DATA_ID=5,
	IDENTIFIER_DATA_ID=11,
	FUEL_DATA_ID=12,
	FUEL_AND_TEMPERATURE_DATA_ID=13,
}DataIdEnum;*/

typedef  enum IoSensors
{
	
	DIN1=1,//
	DIN2,//
	DIN3,//
	DIN4,//4
	AIN1,//
	AIN2,
	AIN3,
	AIN4,//8
	DOUT1,
	DOUT2,
	DOUT3,
	DOUT4,
	FIN1=13,
	FIN2,
	FIN3,
	FIN4,
	RS232_1,//17
	RS232_2,//18
	RS485_SENSOR_GROUP_ID=20,
}IoSensorIdEnum;




typedef struct
{
   uint8_t sensor_id;
	 uint8_t sensor_cmd;
	 uint8_t cmd_par;
} IoCmdType;

typedef __packed struct
{
   uint16_t fuel;
	 int8_t temperature;
} FuelSensorDataType;



typedef struct
{
   QEvt super;
   IoCmdType cmd;
} IoCmdMessType;


#define RS485_TIM TIM7
//#define RS485_TMR_START_STOP(arr)		((RS485_TIM->ARR = (u16)arr), (RS485_TIM->CR1 |= (arr)?(1):(0)))
#define RS485_TMR_START(arr)		    do{RS485_TIM->ARR = (uint16_t)arr; RS485_TIM->CR1 |= 1;}while(0)
#define RS485_TIM_IRQn TIM7_IRQn
#define RCC_APB1Periph_RS485_TIM RCC_APB1Periph_TIM7

#define DIN_TIM TIM1
#define RCC_APB2Periph_DIN_TIM RCC_APB2Periph_TIM1


#if PCB==PCB_V2_1

#define UART485_IRQn                UART5_IRQn
#define UART485_IRQHandler          UART5_IRQHandler
#define UART485                     UART5
#define UART485_GPIO_AF             GPIO_AF_7

//#define  RE_GPIO  GPIOA
//#define  RE_PIN   GPIO_Pin_12
//#define  TX_EN()  RE_GPIO->BSRR = RE_PIN
//#define  RX_EN()  RE_GPIO->BRR =  RE_PIN

#define  RS485_RX_GPIO   GPIOD
#define  RS485_RX_PIN    GPIO_Pin_2
#define  RS485_RX_PIN_GPIO_PinSource GPIO_PinSource9

//#define  RS485_TX_GPIO   GPIOC
//#define  RS485_TX_PIN    GPIO_Pin_12
//#define  RS485_TX_PIN_GPIO_PinSource GPIO_PinSource8

#define  TX_PIN_EN()  RS485_TX_GPIO->BSRR = RS485_TX_PIN
#define  TX_PIN_DIS() RS485_TX_GPIO->BRR =  RS485_TX_PIN

#define  ADC_GPIO   GPIOC
#define  VPOW_ADC_PIN GPIO_Pin_1
#define  VBAT_ADC_PIN GPIO_Pin_0
//----------DOUT---------------------------------------
#define  DOUT1_GPIO  GPIOA
#define  DOUT1_PIN   GPIO_Pin_15
#define  DOUT1_OFF() DOUT1_GPIO->BRR =  DOUT1_PIN
#define  DOUT1_ON()  DOUT1_GPIO->BSRR = DOUT1_PIN
//---------FIN-----------------------------------
#define  UIN1_GPIO   GPIOC
#define  UIN1_PIN    GPIO_Pin_8

#define  UIN2_GPIO   GPIOC
#define  UIN2_PIN    GPIO_Pin_7
#else
#error PCB not defined!
#endif

extern RS485StateType RS485State;
extern QActive* const AO_Io;
extern void Io_ctor(void);
extern IoDataType IoData;	
#endif



