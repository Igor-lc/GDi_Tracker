
#include "qp_port.h"
 #include "LedIndicator.h"
	#include "bsp.h"
	#include <string.h>
	#include "io.h"
 
 
 Q_DEFINE_THIS_FILE

enum ISR_Priorities
{ /* ISR priorities starting from the highest urgency */
   EXTI0_PRIO, /* highest priority */
   SYSTICK_PRIO,
};

static void Clock_init(void);
static void Dout_init(void);
static void NetLight_init(void);
static void RS232_init(void);
static void LSM303_init(void);
static void LedGsm2_init(void);

#ifdef Q_SPY
QSTimeCtr QS_tickTime_;
QSTimeCtr QS_tickPeriod_;
static uint8_t l_SysTick_Handler;

#define QS_BUF_SIZE   (2*1024)
#define QS_BAUD_RATE  115200
#endif

#ifdef  WATCH_DOG_ENABLE
static void WatchDog_init(void);
#endif


void Bsp_init(void)
{
   SystemInit();
   Clock_init();
	 SysInfo.time=0;
	 SysInfo.flag.real_time=0; 
	 SysInfo.flag.time_is_set=0; 
   Dout_init();
	 NetLight_init();
	 RS232_init();
	 LSM303_init();
	 LedGsm2_init();
   LedIndicator_init();
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST, ENABLE);
	
	
   if (QS_INIT((void *)0) == 0)
      {
         Q_ERROR();
      }
#ifdef	WATCH_DOG_ENABLE
WatchDog_init();
#endif
			//FLASH_ITConfig(FLASH_IT_ERROR | FLASH_IT_EOP, ENABLE);
			FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	
}


 #ifdef WATCH_DOG_ENABLE
void WatchDog_init(void)
{
	 RCC->CSR |= (1<<24);
  IWDG->KR  = 0x5555;                                           // enable write to PR, RLR
	//IWDG_SetPrescaler(IWDG_Prescaler_64); IWDG->RLR = 0xFFF;       //6553 ms WDT period
  IWDG_SetPrescaler(IWDG_Prescaler_256); IWDG->RLR = 0xFFF;       //26214 ms WDT period
  //IWDG->PR  = __IWDG_PR;                                        // Init prescaler
  //IWDG->RLR = __IWDG_RLR;                                       // Init RLR
  IWDG->KR  = 0xAAAA;                                           // Reload the watchdog
  IWDG->KR  = 0xCCCC;         
}
#endif

void  Clock_init(void)
{
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO , ENABLE);
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC , ENABLE);
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD , ENABLE);
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 , ENABLE);//MEMORY
	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);//MODEM
 RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);//GPS  
 RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);//DEBUG
 RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);//
 RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
 RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
 RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
	
}



void Dout_init(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;
  
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

   GPIO_InitStructure.GPIO_Pin = DOUT1_PIN;
   GPIO_Init(DOUT1_GPIO , &GPIO_InitStructure);
}

void NetLight_init(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;
  
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

   GPIO_InitStructure.GPIO_Pin = NET_LIGHT_PIN;
   GPIO_Init(NET_LIGHT_GPIO , &GPIO_InitStructure);
}

void RS232_init(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;
  
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

   GPIO_InitStructure.GPIO_Pin = RS232_RX_PIN|RS232_TX_PIN;
   GPIO_Init(RS232_GPIO , &GPIO_InitStructure);
}

void LSM303_init(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;
  
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

   GPIO_InitStructure.GPIO_Pin = LSM303_INT1_PIN|LSM303_INT2_PIN;
   GPIO_Init(LSM303_INT_GPIO , &GPIO_InitStructure);
}

void LedGsm2_init(void)
{
   GPIO_InitTypeDef GPIO_InitStructure;
  
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

   GPIO_InitStructure.GPIO_Pin = LED_GSM2_PIN;
   GPIO_Init(LED_GSM2_GPIO , &GPIO_InitStructure);
}




/*void panic(uint8_t exception_id)
{
	uint8_t exc=exception_id;
uint32_t panic_timeout=PANIC_TIMEOUT;
if(panic_timeout==0)while(1);
else	while(panic_timeout--){exc=exc;}
NVIC_GENERATE_SYSTEM_RESET();	
}*/

uint32_t tick_counter=0;
void SysTick_Handler(void)//every 5 ms
{
	tick_counter++;
	static QEvt const tickEvt = { TIC_100ms_SIG, 0U, 0U };
   static uint8_t tic_counter_100ms = 0,counter_10sec=0;
   QK_ISR_ENTRY(); // inform QK-nano about ISR entry
#ifdef Q_SPY
   uint32_t dummy = SysTick->CTRL; /* clear NVIC_ST_CTRL_COUNT flag */
   (void) dummy;
   QS_tickTime_ += QS_tickPeriod_; /* account for the clock rollover */
#endif
   if (++tic_counter_100ms == 20)//every 100 ms
      {
         tic_counter_100ms = 0;
				 QF_PUBLISH(&tickEvt, &l_SysTick_Handler);
				 LedIndicator();
				if(++counter_10sec==50)//every 5 sec
				{
					counter_10sec=0;
					watch_flag.unused=0;
					if(SysInfo.flag.fwu_processing)watch_flag.nav_filter=0;//disable this flag if fw updating
					if(0==*(uint32_t*)&watch_flag)IWDG->KR = ((uint16_t)0xAAAA);
					*(uint32_t*)&watch_flag=UINT32_MAX;
					watch_flag.unused=0;
				}
      }
   QF_TICK(&l_SysTick_Handler);
   QK_ISR_EXIT(); // inform QK-nano about ISR exit
}

void QK_onIdle(void)
{
  /* QF_INT_DISABLE();
	NOP();
   QF_INT_ENABLE();*/

#ifdef Q_SPY

      if ((UART_QSPY->SR & USART_FLAG_TXE) != 0)// is TXE empty? 
      { 
         uint16_t b;
         QF_INT_DISABLE();
         b = QS_getByte();
         QF_INT_ENABLE();

         if (b != QS_EOD)// not End-Of-Data?
            { 
               UART_QSPY->DR = (b & 0xFF); /* put into the DR register */
            }
      }

#elif defined NDEBUG
   __WFI(); /* wait for interrupt */
#endif
}

void QF_onCleanup(void) {} 

void QF_onStartup(void)
{
   //NVIC_InitTypeDef nvic_init;
   RCC_ClocksTypeDef RCC_Clocks;
	 uint32_t tick;

   /* Set up and enable the SysTick timer.  It will be used as a reference
    * for delay loops in the interrupt handlers.  The SysTick timer period
    * will be set up for BSP_TICKS_PER_SEC.
    */
   /* SysTick end of count event each 5  ms */
   RCC_GetClocksFreq(&RCC_Clocks);
	tick=RCC_Clocks.HCLK_Frequency / BSP_TICKS_PER_SEC;
   SysTick_Config(tick);
   // SysTick_Config(SystemFrequency_SysClk / BSP_TICKS_PER_SEC);

   NVIC_SetPriority(SysTick_IRQn, SYSTICK_PRIO);
   //  NVIC_SetPriority(EXTI0_IRQn,   EXTI0_PRIO);
}



#ifdef Q_SPY

uint8_t QS_onStartup(void const *arg)
{
   RCC_ClocksTypeDef RCC_Clocks;
   GPIO_InitTypeDef GPIO_InitStructure;
   static uint8_t qsBuf[QS_BUF_SIZE]; /* buffer for Quantum Spy */
   QS_initBuf(qsBuf, sizeof(qsBuf));

   
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_UART_QSPY_TX;
   GPIO_Init(GPIO_UART_QSPY , &GPIO_InitStructure);


   USART_InitTypeDef usart_init;
   usart_init.USART_BaudRate = QS_BAUD_RATE;
   usart_init.USART_WordLength = USART_WordLength_8b;
   usart_init.USART_StopBits = USART_StopBits_1;
   usart_init.USART_Parity = USART_Parity_No;
   usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
   usart_init.USART_Mode = USART_Mode_Tx;
   USART_Init(UART_QSPY, &usart_init);

   USART_ClockInitTypeDef usart_clk_init;
   usart_clk_init.USART_Clock = USART_Clock_Disable;
   usart_clk_init.USART_CPOL = USART_CPOL_Low;
   usart_clk_init.USART_CPHA = USART_CPHA_2Edge;
   usart_clk_init.USART_LastBit = USART_LastBit_Disable;
   USART_ClockInit(UART_QSPY, &usart_clk_init);

   USART_Cmd(UART_QSPY, ENABLE); /* enable USART3 */

   RCC_GetClocksFreq(&RCC_Clocks);
   QS_tickPeriod_ = (QSTimeCtr) (RCC_Clocks.HCLK_Frequency  / BSP_TICKS_PER_SEC);
   QS_tickTime_ = QS_tickPeriod_; /* to start the timestamp at zero */
                                            
    QS_FILTER_ON(QS_ALL_RECORDS);

   //QS_FILTER_OFF(QS_QEP_STATE_ENTRY);
    //QS_FILTER_OFF(QS_QEP_STATE_EXIT);
    QS_FILTER_OFF(QS_QEP_STATE_INIT);
    QS_FILTER_OFF(QS_QEP_INIT_TRAN);
    QS_FILTER_OFF(QS_QEP_INTERN_TRAN);
    QS_FILTER_OFF(QS_QEP_TRAN);
    QS_FILTER_OFF(QS_QEP_IGNORED);

    QS_FILTER_OFF(QS_QF_ACTIVE_ADD);
    QS_FILTER_OFF(QS_QF_ACTIVE_REMOVE);
    QS_FILTER_OFF(QS_QF_ACTIVE_SUBSCRIBE);
    QS_FILTER_OFF(QS_QF_ACTIVE_UNSUBSCRIBE);
    QS_FILTER_OFF(QS_QF_ACTIVE_POST_FIFO);
    QS_FILTER_OFF(QS_QF_ACTIVE_POST_LIFO);
    QS_FILTER_OFF(QS_QF_ACTIVE_GET);
    QS_FILTER_OFF(QS_QF_ACTIVE_GET_LAST);
    QS_FILTER_OFF(QS_QF_EQUEUE_INIT);
    QS_FILTER_OFF(QS_QF_EQUEUE_POST_FIFO);
    QS_FILTER_OFF(QS_QF_EQUEUE_POST_LIFO);
    QS_FILTER_OFF(QS_QF_EQUEUE_GET);
    QS_FILTER_OFF(QS_QF_EQUEUE_GET_LAST);
    QS_FILTER_OFF(QS_QF_MPOOL_INIT);
    QS_FILTER_OFF(QS_QF_MPOOL_GET);
    QS_FILTER_OFF(QS_QF_MPOOL_PUT);
    QS_FILTER_OFF(QS_QF_PUBLISH);
    QS_FILTER_OFF(QS_QF_NEW);
    QS_FILTER_OFF(QS_QF_GC_ATTEMPT);
    QS_FILTER_OFF(QS_QF_GC);
    QS_FILTER_OFF(QS_QF_TICK);
    QS_FILTER_OFF(QS_QF_TIMEEVT_ARM);
    QS_FILTER_OFF(QS_QF_TIMEEVT_AUTO_DISARM);
    QS_FILTER_OFF(QS_QF_TIMEEVT_DISARM_ATTEMPT);
    QS_FILTER_OFF(QS_QF_TIMEEVT_DISARM);
    QS_FILTER_OFF(QS_QF_TIMEEVT_REARM);
    QS_FILTER_OFF(QS_QF_TIMEEVT_POST);
    QS_FILTER_OFF(QS_QF_CRIT_ENTRY);
    QS_FILTER_OFF(QS_QF_CRIT_EXIT);
    QS_FILTER_OFF(QS_QF_ISR_ENTRY);
    QS_FILTER_OFF(QS_QF_ISR_EXIT);

    QS_FILTER_OFF(QS_QK_MUTEX_LOCK);
    QS_FILTER_OFF(QS_QK_MUTEX_UNLOCK);
    QS_FILTER_OFF(QS_QK_SCHEDULE);
		 QS_FILTER_OFF(QS_QEP_DISPATCH);
		
	 
	                                            
   return (uint8_t) 1; /* return success */
}
/*..........................................................................*/
void QS_onCleanup(void)
{
}

QSTimeCtr QS_onGetTime(void)
{ /* invoked with interrupts locked */
   if ((SysTick->CTRL & 0x00010000) == 0)
      { /* COUNT no set? */
         return QS_tickTime_ - (QSTimeCtr) SysTick->VAL;
      }
   else
      { /* the rollover occured, but the SysTick_ISR did not run yet */
         return QS_tickTime_ + QS_tickPeriod_ - (QSTimeCtr) SysTick->VAL;
      }
}

void QS_onFlush(void)
{
   uint16_t b;
   while ((b = QS_getByte()) != QS_EOD)
      { /* while not End-Of-Data... */
         while ((UART_QSPY->SR & USART_FLAG_TXE) == 0)
            { /* while TXE not empty */
            }
						UART_QSPY->DR =b & (uint16_t)0x01FF;
      }
}

#endif

int line_num;
void Q_onAssert(char const * const file, int line)
{
	char file_name[100];
	
  // (void) file; /* avoid compiler warning */
  // (void) line; /* avoid compiler warning */
	strcpy(file_name,file);
	line_num=line;
	(void) line_num;
   QF_INT_DISABLE();
   /* make sure that all interrupts are disabled */
         for (;;)
      { /* NOTE: replace the loop with reset for final version */
      }
}



	


	






 











