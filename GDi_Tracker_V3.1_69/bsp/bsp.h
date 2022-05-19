

#ifndef _BSP_H
#define _BSP_H
#include <stdint.h>
#include <RTL.h>
#include "config_sys.h"
#ifndef  _CONFIG_SYS_H
#error "config_sys.h" NOT INCLUDED!!!
#endif


#define BSP_TICKS_PER_SEC    200U//5 ms
#define BSP_1_SEC_TIMEOUT    BSP_TICKS_PER_SEC
#define BSP_5_SEC_TIMEOUT    (BSP_TICKS_PER_SEC*5)
#define NVIC_GENERATE_SYSTEM_RESET()	(SCB->AIRCR = (u32)0x05FA0000 | (u32)0x04)

#ifdef _DEBUG
#define PANIC_TIMEOUT  0
#else
#define PANIC_TIMEOUT  100000000
#endif

#define UART_QSPY  UART5
#define GPIO_UART_QSPY GPIOC
#define GPIO_Pin_UART_QSPY_TX GPIO_Pin_12

#define  NET_LIGHT_GPIO     GPIOC
#define  NET_LIGHT_PIN      GPIO_Pin_9

#define  RS232_GPIO     GPIOA
#define  RS232_RX_PIN      GPIO_Pin_9
#define  RS232_TX_PIN      GPIO_Pin_10

#define  LSM303_INT_GPIO     GPIOC
#define  LSM303_INT1_PIN     GPIO_Pin_7
#define  LSM303_INT2_PIN     GPIO_Pin_6

#define  LED_GSM2_GPIO     GPIOC
#define  LED_GSM2_PIN      GPIO_Pin_13

extern void Bsp_init(void);
#endif


