

#ifndef _LED_INDICATOR_H
#define _LED_INDICATOR_H

#include "stm32f10x.h"
#include "user_types.h"

#if PCB==PCB_V2_1

//#define  LED_SENSOR_GPIO     GPIOB
//#define  LED_SENSOR_PIN      GPIO_Pin_1
#define  LED_SENSOR_ON()     __nop()
#define  LED_SENSOR_OFF()    __nop()

#define  LED_WRPOINT_SMS_GPIO     GPIOA//ok
#define  LED_WRPOINT_SMS_PIN      GPIO_Pin_2
#define  LED_WRPOINT_SMS_ON()     LED_WRPOINT_SMS_GPIO->BSRR = LED_WRPOINT_SMS_PIN
#define  LED_WRPOINT_SMS_OFF()    LED_WRPOINT_SMS_GPIO->BRR =  LED_WRPOINT_SMS_PIN

#define  LED_GPRS_GPIO    GPIOA//ok
#define  LED_GPRS_PIN     GPIO_Pin_0
#define  LED_GPRS_ON()    LED_GPRS_GPIO->BSRR = LED_GPRS_PIN
#define  LED_GPRS_OFF()   LED_GPRS_GPIO->BRR =  LED_GPRS_PIN

#define  LED_GSM_GPIO    GPIOC//ok
#define  LED_GSM_PIN     GPIO_Pin_3
#define  LED_GSM_ON()    LED_GSM_GPIO->BSRR = LED_GSM_PIN
#define  LED_GSM_OFF()   LED_GSM_GPIO->BRR =  LED_GSM_PIN

#define  LED_GPS_GPIO     GPIOC//ok
#define  LED_GPS_PIN      GPIO_Pin_2
#define  LED_GPS_ON()     LED_GPS_GPIO->BSRR = LED_GPS_PIN
#define  LED_GPS_OFF()    LED_GPS_GPIO->BRR =  LED_GPS_PIN
#define  LED_GPS_TOGGLE() LED_GPS_GPIO->ODR ^= LED_GPS_PIN

#define  LED_SERVER_GPIO     GPIOA//ok
#define  LED_SERVER_PIN      GPIO_Pin_1
#define  LED_SERVER_ON()     LED_SERVER_GPIO->BSRR = LED_SERVER_PIN
#define  LED_SERVER_OFF()    LED_SERVER_GPIO->BRR =  LED_SERVER_PIN

#define  LED_RS485_ON()    __nop()
#define  LED_RS485_OFF()    __nop()

//#elif PCB==PCB_V2
//#elif PCB==PCB_V1_1

#else
#error PCB not defined!
#endif  
//-----------------------------------------------------
typedef struct
{
unsigned send_packet_succes:1;
unsigned send_sms_succes:1;
unsigned new_sms_received:1;
unsigned sms_command_complete:1;
unsigned disk_format:1;
unsigned write_point:1;
}IndFlagsType;

extern IndFlagsType ind_flag;
extern void LedIndicator(void);	
extern void LedIndicator_init(void);	

#endif




