

#include "bsp.h"
#include "stm32f10x_conf.h"

static void LedIndicator_init(void);

void Bsp_init(void)
{

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC , ENABLE);
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD , ENABLE);
 LedIndicator_init();
}

void Bsp_de_init(void)
{
//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, DISABLE);
//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, DISABLE);
}


void LedIndicator_init(void)
   {
    GPIO_InitTypeDef GPIO_InitStructure;
 
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

   GPIO_InitStructure.GPIO_Pin = LED_GPS_PIN; 
   GPIO_Init(LED_GPS_GPIO , &GPIO_InitStructure);

   GPIO_InitStructure.GPIO_Pin = LED_GSM_PIN;
   GPIO_Init(LED_GSM_GPIO , &GPIO_InitStructure);

   GPIO_InitStructure.GPIO_Pin = LED_GPRS_PIN;
   GPIO_Init(LED_GPRS_GPIO , &GPIO_InitStructure);

   GPIO_InitStructure.GPIO_Pin = LED_WRPOINT_SMS_PIN;
   GPIO_Init(LED_WRPOINT_SMS_GPIO , &GPIO_InitStructure);
		 
   GPIO_InitStructure.GPIO_Pin = LED_SERVER_PIN;
   GPIO_Init(LED_SERVER_GPIO , &GPIO_InitStructure);
   LED_GPS_OFF();
   LED_GSM_OFF();
   LED_GPRS_OFF();
   LED_WRPOINT_SMS_OFF();
	 LED_SERVER_OFF();
   }






