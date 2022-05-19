

#include "stm32f10x.h"
#include "qp_port.h"
#include "rs485.h"
#include "io.h"
#include "config.h"
#include <string.h>
#include "crc8.h"
#include "bsp.h"
#include "nav_filter.h"
#include <stdlib.h>
#include <stdio.h>
#include "TL10.h"

typedef struct
    {
    QActive super;
    QTimeEvt TimeEvt;
    Rs485CmdType* cmd_ptr;
    QEQueue requestQueue;    
    QEvt const *requestQSto[POINTS_POOL_SIZE*2];    
    uint8_t addr;
    uint8_t sensor_answer;
    IoCmdType cmd;   
    }IO;    

IoDataType IoData;   

#define PM_SMOOTH_BUF_SIZE 4
		
#ifdef DEBUG_IO	 
static void OutDebugIo( char const *dbg_msg);
static void OutDebugIoSprintf1( char const *str,uint32_t val);
#else
    #define OutDebugIo(x) __nop()  
    #define OutDebugIoSprintf1(x1,x2) __nop()
#endif
//----single instance static object and public object ptr
static IO io;
QActive* const AO_Io = &io.super;
RS485StateType RS485State;
__IO uint16_t ADC1ConvertedVoltage = 0;
//----------------------------------------------------------
#ifndef RS485_UART_DMA_MODE		 
volatile uint32_t TxIndex;
volatile uint32_t RxIndex;
volatile uint32_t fin_counter[2];
static uint8_t tx_size;  
static __IO uint32_t ADCDualConvertedValue;
static enum 
    {
    stop_mode,
    tx_mode,
    rx_mode,
    }uart_mode=stop_mode;
#endif

uint8_t polling_sensor;
static void RS485_tx(uint8_t cmd_len);  
static QEvt const RxCompleteEvt = { IO_RX_COMPLETE_SIG, 0U, 0U};
static TxRxHandlerParamType hpar; 
static uint16_t PerMinSmooth1(uint16_t perx);
static uint16_t PerMinSmooth2(uint16_t perx);
static uint8_t rs485_tx_buf[RS485_TX_BUF_SIZE];
uint8_t rs485_rx_buf[RS485_RX_BUF_SIZE];
static void RS485TMR_init(void);
static void DINTMR_init(void);
static void ADC_init(void);
static void UIN_init(void);
static void IO_RS485_init (void);
static float ADresToVolts(double);
static void InputHandler1(void);
static void InputHandler2(void);
static QState IO_initial(IO * const me, QEvt const * const e);
static QState IO_top(IO* const me, QEvt const * const e);
static QState IO_idle(IO * const me, QEvt const * const e);
static QState IO_RS485_polling(IO * const me, QEvt const * const e);
static QState IO_process_RS485_command(IO * const me, QEvt const * const e);


void Io_ctor(void)
{
    IO *me = &io;
    hpar.rx_buf_ptr=rs485_rx_buf;
    hpar.tx_buf_ptr=rs485_tx_buf;
    hpar.sender=(QActive*)&io;
    hpar.receiver=AO_NavFilter;
    hpar.txf=RS485_tx;
    hpar.sig=NAV_FILTER_EVT_SIG;  
    QEQueue_init(&me->requestQueue,me->requestQSto, Q_DIM(me->requestQSto));
    QTimeEvt_ctor(&me->TimeEvt,  IO_TIMEOUT_SIG);
	  //QTimeEvt_ctor(&me->AdcStartEvt,  IO_START_ADC_SIG);
    QActive_ctor(&me->super, Q_STATE_CAST(&IO_initial));
}


QState IO_initial(IO * const me, QEvt const * const e)
{
    QS_OBJ_DICTIONARY(&io);
    QS_FUN_DICTIONARY(&IO_initial);
    QS_FUN_DICTIONARY(&IO_top);
    QS_FUN_DICTIONARY(&IO_idle);
    QS_FUN_DICTIONARY(&IO_RS485_polling);
   //QS_FUN_DICTIONARY(&IO_off);
    QS_FUN_DICTIONARY(&IO_process_RS485_command);
    //QS_FILTER_SM_OBJ(&io);
    //IO_RS485_init();
    //RS485TMR_init();
	  DINTMR_init();
    ADC_init();
	  UIN_init(); 
    QActive_subscribe(&me->super, TIC_100ms_SIG);
    return Q_TRAN(&IO_top);
}


QState IO_top(IO* const me, QEvt const * const e)
{
    switch (e->sig)
        {
        case Q_ENTRY_SIG:
            {
                polling_sensor=0;
                me->sensor_answer=0;
                RS485State=rs485_off;
							 fin_counter[0]=fin_counter[1]=0;
							  //QTimeEvt_postIn(&me->AdcStartEvt, (QActive *)me,1);
            }
            return Q_HANDLED();
				case TIC_100ms_SIG:
						{
//						float voltage = ADresToVolts(ADC3->DR)*IO_BAT_DIV_K;
//						SysInfo.voltage.float_int += (voltage - SysInfo.voltage.float_int)*IO_AD_INTEGR_K;
//            voltage = ADresToVolts(ADC2->DR)*IO_POW_DIV_K;
//						SysInfo.voltage.float_ext += (voltage - SysInfo.voltage.float_ext)*IO_AD_INTEGR_K;
							
							
							/*uint16_t result= VExternalSmooth(ADC2->DR);
							float voltage=ADresToVolts(result)*IO_POW_DIV_K;
							SysInfo.voltage.float_ext=
							result= VInternalSmooth(ADC3->DR);
							SysInfo.voltage.float_int=ADresToVolts(result)*IO_BAT_DIV_K;*/
							
							
							
							InputHandler1();
							InputHandler2();
							
							
							//ADC_StartConversion(ADC2);   
		          //ADC_StartConversion(ADC3); 
							//QTimeEvt_postIn(&me->AdcStartEvt, (QActive *)me,5);
						}
						 return Q_HANDLED();
        case Q_INIT_SIG: return Q_TRAN(&IO_idle);
        case Q_EXIT_SIG: return Q_HANDLED();
        }
    return  Q_SUPER(&QHsm_top);
}


uint8_t sensors_in_config;
uint8_t debug_dev;
QState IO_idle(IO * const me, QEvt const * const e)
{
    QState ret=Q_HANDLED();
    static enum
        {
        start,
        polling,
        }sub_state=start;
    switch (e->sig)
        {
        case Q_ENTRY_SIG:
            {
              //  QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);**********************************************
							
							
							
							
							
                /*me->polling_addr=0;
      if (QActive_recall((QActive *)me, &me->requestQueue)) OutDebugIo("Request recalled");
      else
         {
         OutDebugIo("No deferred requests");
         for (;polling_sensor<MAX_RS485_SENSORS_COUNT; polling_sensor++)
            {
             if (conf_rs485_sensor[polling_sensor].device!=RS485_NODEVICE)
                               {
                                    QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);//next sensor polling
                                   return Q_HANDLED();
                               }
            }
         if (polling_sensor==MAX_RS485_SENSORS_COUNT)
            {
                                  if(me->sensor_answer==0)RS485State=rs485_sensors_error;
                                  else RS485State=rs485_sensors_ok;
              QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,BSP_TICKS_PER_SEC);//pause 1 sec
            }
         else  QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);//next sensor polling
         }*/
            }
            return Q_HANDLED();
        case IO_TIMEOUT_SIG: 
            {
							 return ret;
                 switch (sub_state)
                    {
                    case start:
                        {
													 debug_dev=RS485_NODEVICE;
													  sensors_in_config=0;
                            for (int i=0;i<MAX_RS485_SENSORS_COUNT;i++)
                                {
																debug_dev=conf_rs485_sensor[i].device;
                                if (debug_dev>RS485_NODEVICE&&debug_dev<DEVICES_TABLE_END)
																   {
																	  sensors_in_config++;
																   }
                                }
                            if (sensors_in_config>0)
                                {
                                sub_state=polling;
                                polling_sensor=0;
                                me->sensor_answer=0;
															  debug_dev=conf_rs485_sensor[polling_sensor].device;
                                 if (debug_dev>RS485_NODEVICE&&debug_dev<DEVICES_TABLE_END)
                                    {
                                    me->addr=conf_rs485_sensor[polling_sensor].addr;
                                    ret=Q_TRAN(&IO_RS485_polling);
                                    }
                                else QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
                                }
                            else//any devices not finded in config
                                {
                                RS485State=rs485_off;
                                QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,BSP_TICKS_PER_SEC*5);//pause 1 sec
                                }
                        }
                        break;
                    case polling:
                        {
                            ++polling_sensor;
													 debug_dev=conf_rs485_sensor[polling_sensor].device;
                            if (polling_sensor==MAX_RS485_SENSORS_COUNT)
                                {
                                RS485State=me->sensor_answer?rs485_sensors_ok:rs485_sensors_error;
                                sub_state=start;
                                QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,BSP_TICKS_PER_SEC);//pause 1 sec
                                }
                            else  if (debug_dev>RS485_NODEVICE&&debug_dev<DEVICES_TABLE_END)
                                {
                                 me->addr=conf_rs485_sensor[polling_sensor].addr;
                                ret=Q_TRAN(&IO_RS485_polling);
                                }
                            else QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);
                        }
                        break;
                    }
            }
            return ret;
        case Q_EXIT_SIG: 
            {
                QTimeEvt_disarm(&me->TimeEvt);
            }
            return Q_HANDLED();
        }
    return Q_SUPER(&IO_top);
}



QState IO_RS485_polling(IO * const me, QEvt const * const e)
{
	static uint8_t TL10_err_counter=0;
    QState ret= Q_HANDLED();
    switch (e->sig)
        {
        case Q_ENTRY_SIG:
            {
                OutDebugIoSprintf1("polling_sensor=",polling_sensor);
                OutDebugIoSprintf1("polling_addr=",me->addr);
                OutDebugIoSprintf1("current_device=",conf_rs485_sensor[polling_sensor].device);
                switch (conf_rs485_sensor[polling_sensor].device)
                    {
                    case RS485_DEVICE_EPSILON: 
										{
                        me->cmd_ptr=Epsilon_cmd_table;
                        me->cmd_ptr+=EPSILON_READ_DATA_CMD_NUM;
												hpar.addr=me->addr;
                        hpar.current_sensor=polling_sensor;
                        me->cmd_ptr->tx_cmd(&hpar); 
                        OutDebugIoSprintf1("EPSILON RXDELAY=",hpar.rx_delay); 
										}											
                        break;
										 case RS485_DEVICE_RF10: 
										{
                        me->cmd_ptr=RF10_cmd_table;
                        me->cmd_ptr+=RF10_READ_CARD_ID_CMD_NUM;
                        hpar.addr=me->addr;
                        hpar.current_sensor=polling_sensor;
                        me->cmd_ptr->tx_cmd(&hpar); 
                        OutDebugIoSprintf1("RF10 RXDELAY=",hpar.rx_delay); 
										}											
                        break;
										 case RS485_DEVICE_TL10:
										{
                        me->cmd_ptr=TL10_cmd_table;
                        me->cmd_ptr+=TL10_READ_ID_CMD_NUM;
                        hpar.addr=me->addr;
                        hpar.current_sensor=polling_sensor;
                        me->cmd_ptr->tx_cmd(&hpar); 
                        OutDebugIoSprintf1("TL10 RXDELAY=",hpar.rx_delay); 
										}											
                        break;
                    default: 
                        break;
                    }
                QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,RS485_ANSWER_ERROR_TIMEOUT); 
            }
            return Q_HANDLED();
        case IO_RX_COMPLETE_SIG:
            {
                char index='0';
                for (int i=0;i<hpar.rx_size;i++,index++)
                    {
                    OutDebugIoSprintf1(&index,rs485_rx_buf[i]);
                    }     
                if (1==me->cmd_ptr->rx_cmd(&hpar))
                    {
                    OutDebugIo("CRC OK");
                    switch (conf_rs485_sensor[polling_sensor].device)
                        {
                        case RS485_DEVICE_MR9:
                            {
															 OutDebugIo("MR9 READ OK"); 
                                ret=  Q_TRAN(&IO_idle);
                            }               
                            break;	
											case RS485_DEVICE_RF10:
                            {
															 OutDebugIo("RF10 READ OK");
                                ret=  Q_TRAN(&IO_idle);
                            }               
                            break;
                       case RS485_DEVICE_TL10:
                            {
															OutDebugIo("TL10 READ OK");
															TL10_err_counter=0;
                              ret=  Q_TRAN(&IO_idle);
                            }               
                            break;															
                        case RS485_DEVICE_EPSILON:
                            me->sensor_answer=1;
                            OutDebugIo("EPSILON READ OK");
                            int delta=SysInfo.fuel_level_saved[polling_sensor]-((EpsilonDataType*)&IoData.rs485[polling_sensor])->ndata;
                            if (abs(delta)>conf.rs485.deltaN)
                                {
                                SensorEvt *pe = Q_NEW(SensorEvt,NAV_FILTER_EVT_SIG);
                                pe->sensor.id=RS485_SENSOR_GROUP_ID+polling_sensor;
                                pe->sensor.descriptor.data_size=2;
                                pe->sensor.descriptor.data_id=IO_FUEL_SENSOR;
                                ((FuelSensorDataType*)&pe->sensor.data.any_data)->fuel=((EpsilonDataType*)&IoData.rs485[polling_sensor])->ndata;
                                QACTIVE_POST(AO_NavFilter, &pe->super,AO_Io);
                                }
                            // if (SysInfo.io_data.epsilon.addr==config.rs485[me->current_sensor].addr)
                            {
                                OutDebugIoSprintf1("Tdata=",((EpsilonDataType*)&IoData.rs485[polling_sensor])->tdata);
                                OutDebugIoSprintf1("Ndata=",((EpsilonDataType*)&IoData.rs485[polling_sensor])->ndata);
                                OutDebugIoSprintf1("Cdata=",((EpsilonDataType*)&IoData.rs485[polling_sensor])->cdata);
                                /* for (int i=0;i<hpar.rx_size;i++,index++)
                                   {
                                    OutDebugIoSprintf1(&index,*(uint8_t*)(i+&IoData.rs485[polling_sensor]));
                                 }*/
                                //RS485State=rs485_sensors_ok;	
                                ret=  Q_TRAN(&IO_idle);
                            }
                            // else  ret=  Q_TRAN(&IO_RS485_write_addr);                   
                            break;
                        }
                    }
                else
                    {
                      OutDebugIo("CRC ERROR");
                      ret=  Q_TRAN(&IO_idle); 
                    }
            }
            return ret;
        case IO_TIMEOUT_SIG:
            {
                OutDebugIo("IO_TIMEOUT_ERROR");
                  memset((uint8_t*)&IoData.rs485[polling_sensor],0,SENSOR_DATA_SIZE);
							uint8_t device=conf_rs485_sensor[polling_sensor].device;
							if (device==RS485_DEVICE_TL10)
							{
								if(++TL10_err_counter==10)
								{
							   memset((uint8_t*)&SysInfo.tl10_data.id,0,TL10_DATA_SIZE);
								}
							}
							ret=  Q_TRAN(&IO_idle); 
                //if (config.rs485[me->current_sensor].id==MR9_SENSOR)memset(SysInfo.io_data.mr9.card_id,0,MR9_CARD_ID_SIZE);
						
            }
            return ret;
        case Q_EXIT_SIG: 
            {
                QTimeEvt_disarm(&me->TimeEvt);
            }
            return Q_HANDLED();
        }
    return Q_SUPER(&IO_top);
}



QState IO_process_RS485_command(IO * const me, QEvt const * const e)
{
    switch (e->sig)
        {
        case Q_ENTRY_SIG:
            {
                for (uint32_t i=0;i<MAX_RS485_SENSORS_COUNT; i++)
                    {
                    if (conf_rs485_sensor[i].device==me->cmd.sensor_id)
                        {
                        switch (me->cmd.sensor_id)
                            {
                            /*case MR9_SENSOR_CARDID:
                               me->cmd_ptr=MR9_cmd_table;
                               me->cmd_ptr+=me->cmd.sensor_cmd;
                               hpar.addr=config.rs485[i].addr;
                               me->cmd_ptr->tx_cmd(&hpar);  
                               break;*/
                            }
                        break;
                        }
                    }
                QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,RS485_ANSWER_ERROR_TIMEOUT); 
            }
            return Q_HANDLED();
        case IO_RX_COMPLETE_SIG:
            {
                /*  char index='0';
                   for (int i=0;i<rx_size;i++,index++)
                      {
                      OutDebugIoSprintf1(&index,rs485_rx_buf[i]);
                      }  */           
                if (1==me->cmd_ptr->rx_cmd(&hpar)) OutDebugIo("CMD EXEC OK");
                else OutDebugIo("CMD EXEC ERROR"); 
            }
            return  Q_TRAN(&IO_idle);
        case IO_TIMEOUT_SIG:
            {
                /*   char index='0';
        for (int i=0;i<rx_size;i++,index++)
           {
           OutDebugIoSprintf1(&index,rs485_rx_buf[i]);
           }  
                   index='0';
        for (int i=0;i<tx_size;i++,index++)
           {
           OutDebugIoSprintf1(&index,rs485_tx_buf[i]);
           } */
                OutDebugIo("IO_TIMEOUT_ERROR");
            }
            return Q_TRAN(&IO_idle);//end of cmd_table
        case Q_EXIT_SIG: 
            {
                QTimeEvt_disarm(&me->TimeEvt);
            }
            return Q_HANDLED();
        }
    return Q_SUPER(&IO_top);
}






#ifdef RS485_UART_DMA_MODE	

void IO_RS485_init (void)//RS485_UART_DMA_MODE	
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  =  GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
    GPIO_InitStructure.GPIO_Pin = RS485_RX_PIN;
    GPIO_Init(RS485_RX_GPIO , &GPIO_InitStructure);
	
    GPIO_InitStructure.GPIO_Pin = RS485_TX_PIN;
    GPIO_Init(RS485_TX_GPIO , &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Pin = RE_PIN;
    GPIO_Init(RE_GPIO, &GPIO_InitStructure);


    USART_InitStructure.USART_BaudRate = 19200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl= USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
    USART_Init(UART485, &USART_InitStructure);
    USART_Cmd(UART485, ENABLE);


    DMA_DeInit(DMA1_Channel2);//TX
    DMA_InitStructure.DMA_PeripheralBaseAddr = (unsigned int) &(UART485->TDR);
//  DMA_InitStructure.DMA_MemoryBaseAddr = (u32) rs485_tx_buf;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = 0;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel2, &DMA_InitStructure);

    DMA_DeInit(DMA1_Channel3);//RX
    DMA_InitStructure.DMA_PeripheralBaseAddr = (unsigned int) &(UART485->RDR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)0;
    //DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = 0;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel3, &DMA_InitStructure);
    USART_ClearFlag(UART485, USART_FLAG_RXNE | USART_FLAG_TC | USART_FLAG_TXE);
    USART_DMACmd(UART485, USART_DMAReq_Tx|USART_DMAReq_Rx, ENABLE);
    //DMA_Cmd(DMA1_Channel6, ENABLE);


    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel3_IRQn;//RX
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure); 
    DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE); 

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;//TX
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure); 
    DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE); 
}


void RS485_tx(uint8_t cmd_len)
{
    TX_EN();
    for (uint8_t i=0;i<RS485_RX_BUF_SIZE;i++)rs485_rx_buf[i]=0;
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;
    DMA1_Channel2->CNDTR = cmd_len;
    DMA1_Channel2->CMAR =(u32)rs485_tx_buf; 
    DMA1_Channel2->CCR |= DMA_CCR_EN; 
}

void RS485_rx_on(void)
{
    RX_EN();
    DMA1_Channel3->CCR &= ~DMA_CCR_EN;
    DMA1_Channel3->CMAR =(u32)rs485_rx_buf; 
    DMA1_Channel3->CNDTR = hpar.rx_size;
    DMA1_Channel3->CCR |= DMA_CCR_EN; 
}


void TIM7_IRQHandler(void)
{
    TIM7->SR = 0;    
    RS485_rx_on();  
}

void  DMA1_Channel2_IRQHandler(void)//tx complete
{
    QK_ISR_ENTRY(); 
    DMA1_Channel2->CCR &= ~DMA_CCR_EN; 
    DMA_ClearITPendingBit(DMA1_IT_TC2);
    RS485_TMR_START( hpar.rx_delay); 
    QK_ISR_EXIT(); 
}

void  DMA1_Channel3_IRQHandler(void)//rx complete
{
    static uint8_t RS485_IRQHandler;
    QK_ISR_ENTRY(); 
    DMA1_Channel3->CCR &= ~DMA_CCR_EN; 
    DMA_ClearITPendingBit(DMA1_IT_TC3);
    QACTIVE_POST(AO_Io, &RxCompleteEvt,&RS485_IRQHandler);
    QK_ISR_EXIT(); 
}

#else	 

void IO_RS485_init (void)//RS485_UART_IRQ_MODE	
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = RS485_RX_PIN;
    GPIO_Init(RS485_RX_GPIO , &GPIO_InitStructure);
	
	 // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   // GPIO_InitStructure.GPIO_Pin = RS485_TX_PIN;
    //GPIO_Init(RS485_TX_GPIO , &GPIO_InitStructure);
	
   // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   // GPIO_InitStructure.GPIO_Pin = RE_PIN;
   // GPIO_Init(RE_GPIO, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 19200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl= USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
    USART_Init(UART485, &USART_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = UART485_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&NVIC_InitStructure); 
     NVIC_SetPriority (UART485_IRQn, 1); 
     NVIC_EnableIRQ(UART485_IRQn);

    USART_Cmd(UART485, ENABLE);
    USART_ITConfig(UART485, USART_IT_RXNE, ENABLE);  
    USART_ITConfig(UART485, USART_IT_TXE, ENABLE);  
}


void RS485_tx(uint8_t cmd_len)
{
   // TX_EN();
    uart_mode = tx_mode;
    TxIndex=0;
    tx_size=cmd_len;
    USART_ITConfig(UART485, USART_IT_TC, ENABLE);  
}  

void TIM7_IRQHandler(void)
{
    TIM7->SR = 0;
//    RX_EN();       
    RxIndex=0;     
}

void UART485_IRQHandler (void)
{
    static uint8_t RS485_IRQHandler;
    QK_ISR_ENTRY(); 

    if (USART_GetITStatus(UART485, USART_IT_TC) == SET)
        {
        USART_ClearITPendingBit(UART485, USART_IT_TC);
        if (uart_mode == tx_mode)
            {
            UART485->DR = (rs485_tx_buf[TxIndex++] & (uint16_t)0x01FF);
            if (TxIndex == tx_size)
                {
                USART_ITConfig(UART485, USART_IT_TC, DISABLE);
                uint8_t dummy =(uint8_t)(UART485->DR & (uint16_t)0x01FF);
                uart_mode = rx_mode;  
                RS485_TMR_START( hpar.rx_delay);   
                // RS485_TMR_START(50); //50 us OK	
                //RS485_TMR_START(40); //40 us OK	
                // RS485_TMR_START(30); //30 us OK	
                // RS485_TMR_START(20); //20 us OK tested ok	
                //  RS485_TMR_START(10); //10 us OK
                // RS485_TMR_START(2);//2 us ERROR 								
                }
            }
        else
            {
            USART_ITConfig(UART485, USART_IT_TC, DISABLE);
            }
        }
    if (USART_GetITStatus(UART485, USART_IT_RXNE) == SET)
        {
        if (uart_mode == rx_mode)
            {
            rs485_rx_buf[RxIndex++] = (uint8_t)(UART485->DR & (uint16_t)0x01FF);
            if (RxIndex==hpar.rx_size)
                {
                RxIndex=0;
                QACTIVE_POST(AO_Io, &RxCompleteEvt,&RS485_IRQHandler);
                uart_mode=stop_mode;        
                }
            }
        else
            {
            uint8_t dummy =(uint8_t)(UART485->DR & (uint16_t)0x01FF);//just clear irq flag
            }
        }
    QK_ISR_EXIT();   
}

#endif

#ifdef DEBUG_IO	 
void OutDebugIo( char const *dbg_msg)
{
    QS_BEGIN(QS_IO, AO_Io)                                 
    QS_STR(dbg_msg);                              
    QS_END()
}   

void OutDebugIoSprintf1( char const *str,uint32_t val)
{
    QS_BEGIN(QS_IO, AO_Io)                                  
    QS_STR(str); 
    QS_U32(4, val);       
    QS_END()
}
#endif

float ADresToVolts(double result)
{
    return(((result * IO_AD_VREF)/4096.0));
}

void RS485TMR_init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_RS485_TIM, ENABLE);
    TIM_TimeBaseInitTypeDef tmr;
    TIM_TimeBaseStructInit(&tmr);
    tmr.TIM_Prescaler  = 72;// 1us TMR CLOCK
    tmr.TIM_Period = 0;
    TIM_TimeBaseInit(RS485_TIM, &tmr);
    TIM_SelectOnePulseMode(RS485_TIM, TIM_OPMode_Single);
    TIM_ITConfig(RS485_TIM, TIM_IT_Update, ENABLE);
    NVIC_SetPriority (RS485_TIM_IRQn, 0); 
    NVIC_EnableIRQ(RS485_TIM_IRQn);
}



void DINTMR_init(void)
{
     RCC_APB2PeriphClockCmd(RCC_APB2Periph_DIN_TIM, ENABLE);
    TIM_TimeBaseInitTypeDef tmr;
    TIM_TimeBaseStructInit(&tmr);
    tmr.TIM_Prescaler  = 72;// 1us TMR CLOCK
	  tmr.TIM_CounterMode = TIM_CounterMode_Up;
    tmr.TIM_Period = 10000;//10 ms
    TIM_TimeBaseInit(DIN_TIM, &tmr);
	  TIM_Cmd(DIN_TIM, ENABLE);
    TIM_ITConfig(DIN_TIM, TIM_IT_Update, ENABLE);
    NVIC_SetPriority (TIM1_UP_IRQn, 10); 
    NVIC_EnableIRQ(TIM1_UP_IRQn);
}

void ADC_init(void)
{
    GPIO_InitTypeDef       GPIO_InitStructure;
   /* ADC_InitTypeDef       ADC_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    
    RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div2); 
    RCC_ADCCLKConfig(RCC_ADC34PLLCLK_Div2); 
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);          
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC34, ENABLE);*/
    
    GPIO_InitStructure.GPIO_Pin = VPOW_ADC_PIN|VBAT_ADC_PIN ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(ADC_GPIO, &GPIO_InitStructure);

   /* ADC_StructInit(&ADC_InitStructure);

   
    ADC_VoltageRegulatorCmd(ADC2, ENABLE); //Calibration procedure  
    ADC_VoltageRegulatorCmd(ADC3, ENABLE);

    
    volatile int delay=360;// Insert delay equal to 10 µs 
    while (delay)delay--;
    ADC_SelectCalibrationMode(ADC2, ADC_CalibrationMode_Single);
    ADC_SelectCalibrationMode(ADC3, ADC_CalibrationMode_Single);
    ADC_StartCalibration(ADC2);
    ADC_StartCalibration(ADC3);
    while (ADC_GetCalibrationStatus(ADC2) != RESET )
        {
        };
    while (ADC_GetCalibrationStatus(ADC3) != RESET )
        {
        };

    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;                                                                    
    ADC_CommonInitStructure.ADC_Clock = ADC_Clock_AsynClkMode;                    
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;             
    ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_OneShot;                  
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;          

    ADC_CommonInit(ADC2, &ADC_CommonInitStructure);
    ADC_CommonInit(ADC3, &ADC_CommonInitStructure);

    ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; 
    ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;         
    ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;   
    ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;  
    ADC_InitStructure.ADC_NbrOfRegChannel = 1;
    ADC_Init(ADC2, &ADC_InitStructure);   
    ADC_Init(ADC3, &ADC_InitStructure);         

    ADC_RegularChannelConfig(ADC2, ADC_Channel_12, 1, ADC_SampleTime_7Cycles5);
    ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_7Cycles5);
    ADC_Cmd(ADC2, ENABLE);
    ADC_Cmd(ADC3, ENABLE);
    while (!ADC_GetFlagStatus(ADC2, ADC_FLAG_RDY));
    while (!ADC_GetFlagStatus(ADC3, ADC_FLAG_RDY));
		ADC_StartConversion(ADC2);   
		ADC_StartConversion(ADC3); 
 
    while(ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC) == RESET){};
		while(ADC_GetFlagStatus(ADC3, ADC_FLAG_EOC) == RESET){};
    SysInfo.voltage.float_int = ADresToVolts(ADC3->DR)*IO_BAT_DIV_K;
    SysInfo.voltage.float_ext= ADresToVolts(ADC2->DR)*IO_POW_DIV_K;			*/
}




void UIN_init(void)
{
   GPIO_InitTypeDef       GPIO_InitStructure;
	EXTI_InitTypeDef        EXTI_InitStructure;
	NVIC_InitTypeDef        NVIC_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Pin = UIN2_PIN;//7
  GPIO_Init(UIN2_GPIO, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin = UIN1_PIN;//8
  GPIO_Init(UIN1_GPIO, &GPIO_InitStructure);
  
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource7);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource8);
	
  EXTI_ClearFlag(EXTI_Line7);
  EXTI_InitStructure.EXTI_Line = EXTI_Line7;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
	
	EXTI_ClearFlag(EXTI_Line8);
  EXTI_InitStructure.EXTI_Line = EXTI_Line8;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;  
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
//	
  // Enable and set Button EXTI Interrupt to the lowest priority 
  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);	
}

	
	

void EXTI9_5_IRQHandler(void)
{
	if((EXTI_GetITStatus(EXTI_Line8) != RESET))
  {
    fin_counter[0]++;
   EXTI_ClearITPendingBit(EXTI_Line8);
  }
	else if((EXTI_GetITStatus(EXTI_Line7) != RESET))
  {
    fin_counter[1]++;
   EXTI_ClearITPendingBit(EXTI_Line7);
  }
}


uint16_t PerMinSmooth1(uint16_t perx)
{
	uint16_t ret;
	static enum
	{
		fill_buf,
		work,
	}state=fill_buf;
	static uint8_t i=0;
	static uint16_t buf[PM_SMOOTH_BUF_SIZE];
	if(state==fill_buf)
	{
		buf[i++]=perx;
		if(i==PM_SMOOTH_BUF_SIZE)
		{
			float sum=0;
			for(int k=0;k<PM_SMOOTH_BUF_SIZE;k++)sum+=buf[k];
			sum=sum/4;//
			state=work;
			i=0;
			ret= sum;
		}
		else ret= perx;
	}
	else//work
	{
		buf[i]=perx;
		if(++i==PM_SMOOTH_BUF_SIZE)i=0;
		uint32_t sum=0;
		for(int k=0;k<PM_SMOOTH_BUF_SIZE;k++)sum+=buf[k];
		sum=sum/4.0;
		ret= sum;
	}
	return ret;
}

uint16_t PerMinSmooth2(uint16_t perx)
{
	uint16_t ret;
	static enum
	{
		fill_buf,
		work,
	}state=fill_buf;
	static uint8_t i=0;
	static uint16_t buf[PM_SMOOTH_BUF_SIZE];
	if(state==fill_buf)
	{
		buf[i++]=perx;
		if(i==PM_SMOOTH_BUF_SIZE)
		{
			float sum=0;
			for(int k=0;k<PM_SMOOTH_BUF_SIZE;k++)sum+=buf[k];
			sum=sum/4;//
			state=work;
			i=0;
			ret= sum;
		}
		else ret= perx;
	}
	else//work
	{
		buf[i]=perx;
		if(++i==PM_SMOOTH_BUF_SIZE)i=0;
		uint32_t sum=0;
		for(int k=0;k<PM_SMOOTH_BUF_SIZE;k++)sum+=buf[k];
		sum=sum/4.0;
		ret= sum;
	}
	return ret;
}




void TIM1_UP_IRQHandler(void)
{
	static uint8_t deb_counter1,deb_counter2;
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	   if (conf.fin.mode[0]==IO_DIN)
	      {
		     uint8_t din1_state=(UIN1_GPIO->IDR & UIN1_PIN)?0:1;
	       if(SysInfo.flag.din1!=din1_state)
	          {
		         if(++deb_counter1==5)
		            {
			           deb_counter1=0;
			           SysInfo.flag.din1=din1_state;
								 SensorEvt *pe = Q_NEW(SensorEvt,NAV_FILTER_EVT_SIG);
                 pe->sensor.id=DIN1;
                 pe->sensor.descriptor.data_size=1;
                 pe->sensor.descriptor.data_id=IO_DIN;
                 *pe->sensor.data.any_data=SysInfo.flag.din1;
                 QACTIVE_POST(AO_NavFilter, &pe->super,AO_Io);
		            }
	          }
	        else deb_counter1=0;
        }
				
				 if (conf.fin.mode[1]==IO_DIN)
	      {
		     uint8_t din2_state=(UIN2_GPIO->IDR & UIN2_PIN)?0:1;
	       if(SysInfo.flag.din2!=din2_state)
	          {
		         if(++deb_counter2==5)
		            {
			           deb_counter2=0;
			           SysInfo.flag.din2=din2_state;
								 SensorEvt *pe = Q_NEW(SensorEvt,NAV_FILTER_EVT_SIG);
                 pe->sensor.id=DIN2;
                 pe->sensor.descriptor.data_size=1;
                 pe->sensor.descriptor.data_id=IO_DIN;
                 *pe->sensor.data.any_data=SysInfo.flag.din2;
                 QACTIVE_POST(AO_NavFilter, &pe->super,AO_Io);
		            }
	          }
	        else deb_counter2=0;
        }
}

void InputHandler1(void)
{
	static uint8_t tick_counter=0;
	switch(conf.fin.mode[0])
	{
		case IO_RPM:
		{
			if(++tick_counter==10)
			{
				tick_counter=0;
			char temp[50];
			SysInfo.counter[0].moment=(float)60.0* PerMinSmooth1(fin_counter[0]);
		  fin_counter[0]=0;
			sprintf(temp,"Vint=%5.2f Vext=%5.2f ADC2->DR=%u",SysInfo.voltage.float_int,SysInfo.voltage.float_ext,ADC2->DR);
			OutDebugIo(temp);
			sprintf(temp,"counter%u moment=%u",0,SysInfo.counter[0].moment);
			OutDebugIo(temp);
			}
		}
		break;
		case IO_DIN:
		{
						
		}
		break;
		default:
			break;
	}
}


void InputHandler2(void)
{
	static uint8_t tick_counter=0;
	switch(conf.fin.mode[1])
	{
		case IO_RPM:
		{
			if(++tick_counter==10)
			{
				tick_counter=0;
			char temp[50];
			SysInfo.counter[1].moment=(float)60.0* PerMinSmooth2(fin_counter[1]);
		  fin_counter[1]=0;
			sprintf(temp,"Vint=%5.2f Vext=%5.2f ADC2->DR=%u",SysInfo.voltage.float_int,SysInfo.voltage.float_ext,ADC2->DR);
			OutDebugIo(temp);
			sprintf(temp,"counter%u moment=%u",1,SysInfo.counter[1].moment);
			OutDebugIo(temp);
			}
		}
		break;
		case IO_DIN:
		{
						
		}
		break;
		default:
			break;
	}
}


/*void DAC_init(void)
{
  DAC_InitTypeDef    DAC_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
  
   GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
  
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bits2_0;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
  DAC_Init(DAC_Channel_1, &DAC_InitStructure);
  
  DAC_Cmd(DAC_Channel_1, ENABLE);
	DAC_SetChannel1Data(DAC_Align_12b_R, 0xFFF/2);
}*/


 
/*void COMP1_2_3_IRQHandler(void)
{
    EXTI_ClearITPendingBit(EXTI_Line21);//COMP1 output EXTI
}*/


/*uint16_t VExternalSmooth(uint16_t ad)
{
	uint16_t ret;
	static enum
	{
		fill_buf,
		work,
	}state=fill_buf;
	static uint8_t i=0;
	static uint16_t buf[SMOOTH_BUF_SIZE];
	if(state==fill_buf)
	{
		buf[i++]=ad;
		if(i==SMOOTH_BUF_SIZE)
		{
			uint32_t sum=0;
			for(int k=0;k<SMOOTH_BUF_SIZE;k++)sum+=buf[k];
			sum=sum>>4;//div 16
			state=work;
			i=0;
			ret= sum;
		}
		else ret= ad;
	}
	else//work
	{
		buf[i++]=ad;
		if(i==SMOOTH_BUF_SIZE)i=0;
		uint32_t sum=0;
			for(int k=0;k<SMOOTH_BUF_SIZE;k++)sum+=buf[k];
			sum=sum>>4;//div 16
		ret= sum;
	}
	return ret;
}


uint16_t VInternalSmooth(uint16_t ad)
{
	uint16_t ret;
	static enum
	{
		fill_buf,
		work,
	}state=fill_buf;
	static uint8_t i=0;
	static uint16_t buf[SMOOTH_BUF_SIZE];
	if(state==fill_buf)
	{
		buf[i++]=ad;
		if(i==SMOOTH_BUF_SIZE)
		{
			uint32_t sum=0;
			for(int k=0;k<SMOOTH_BUF_SIZE;k++)sum+=buf[k];
			sum=sum>>4;//div 16
			state=work;
			i=0;
			ret= sum;
		}
		else ret= ad;
	}
	else//work
	{
		buf[i]=ad;
		if(++i==SMOOTH_BUF_SIZE)i=0;
		uint32_t sum=0;
		for(int k=0;k<SMOOTH_BUF_SIZE;k++)sum+=buf[k];
		sum=sum>>4;//div 16
		ret= sum;
	}
	return ret;
}
*/

