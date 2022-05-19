

#ifndef _BSP_H
#define _BSP_H

#define  LED_GPRS_GPIO    GPIOA//ok
#define  LED_GPRS_PIN     GPIO_Pin_0
#define  LED_GPRS_ON()    LED_GPRS_GPIO->BSRR = LED_GPRS_PIN
#define  LED_GPRS_OFF()   LED_GPRS_GPIO->BRR =  LED_GPRS_PIN

#define  LED_GSM_GPIO    GPIOC//ok
#define  LED_GSM_PIN     GPIO_Pin_2
#define  LED_GSM_ON()    LED_GSM_GPIO->BSRR = LED_GSM_PIN
#define  LED_GSM_OFF()   LED_GSM_GPIO->BRR =  LED_GSM_PIN

#define  LED_GPS_GPIO     GPIOC//ok
#define  LED_GPS_PIN      GPIO_Pin_3
#define  LED_GPS_ON()     LED_GPS_GPIO->BSRR = LED_GPS_PIN
#define  LED_GPS_OFF()    LED_GPS_GPIO->BRR =  LED_GPS_PIN
#define  LED_GPS_TOGGLE() LED_GPS_GPIO->ODR ^= LED_GPS_PIN

#define  LED_WRPOINT_SMS_GPIO     GPIOA//ok
#define  LED_WRPOINT_SMS_PIN      GPIO_Pin_2
#define  LED_WRPOINT_SMS_ON()     LED_WRPOINT_SMS_GPIO->BSRR = LED_WRPOINT_SMS_PIN
#define  LED_WRPOINT_SMS_OFF()    LED_WRPOINT_SMS_GPIO->BRR =  LED_WRPOINT_SMS_PIN

#define  LED_SERVER_GPIO     GPIOA//ok
#define  LED_SERVER_PIN      GPIO_Pin_1
#define  LED_SERVER_ON()     LED_SERVER_GPIO->BSRR = LED_SERVER_PIN
#define  LED_SERVER_OFF()    LED_SERVER_GPIO->BRR =  LED_SERVER_PIN

extern void Bsp_init(void);
extern void Bsp_de_init(void);

#endif



