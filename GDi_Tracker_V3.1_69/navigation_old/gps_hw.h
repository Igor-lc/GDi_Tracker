


#ifndef _GPS_HW_H
#define _GPS_HW_H

#define GPS_UART UART4
#define GPS_UART_GPIO  GPIOC
#define GPS_UART_TX_Pin              GPIO_Pin_10
#define GPS_UART_RX_Pin              GPIO_Pin_11

#define GPS_POWER_GPIO_PORT         GPIOA
#define GPS_POWER_PIN               GPIO_Pin_11
#define GPS_POWER_ON()              GPS_POWER_GPIO_PORT->BSRR = GPS_POWER_PIN
#define GPS_POWER_OFF()             GPS_POWER_GPIO_PORT->BRR = GPS_POWER_PIN

/*
#define GPS_NRESET_PIN            GPIO_Pin_2
#define GPS_NRESET_GPIO_PORT      GPIOF
#define GPS_NRESET_HIGH()         GPS_NRESET_GPIO_PORT->BSRR = GPS_NRESET_PIN
#define GPS_NRESET_LOW()          GPS_NRESET_GPIO_PORT->BRR = GPS_NRESET_PIN
*/


#endif
