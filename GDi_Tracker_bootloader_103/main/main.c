
#include "main.h"
#include "stm32f10x_conf.h"
#include "bsp.h"
#include <stdint.h>
#include "loader_defs.h"

// #define WR_TEST_RECORD

uint32_t CalcCrc32(uint8_t *buf,uint32_t len);
static u8 CheckFw(FirmwareDescriptorType *fw_ptr);
void  Delay(uint32_t delay);
#ifndef PLACE_BOOT_REC_AT_END_OF_FLASH 
//const BootRecType factory_boot_rec __attribute__((at(BOOT_REC_ADDRESS))) = {FACTORY_RECORD}; // RO 
const BootRecPageType  factory_boot_rec __attribute__((at(BOOT_REC_ADDRESS))) = {FACTORY_RECORD}; // RO 
const uint32_t  device_id __attribute__((at(BOOT_REC_ADDRESS-sizeof(uint32_t)))) = {0xFFFFFFFF}; // RO 
#endif
BootRecType *boot_rec_ptr = (BootRecType*)BOOT_REC_ADDRESS;
//uint32_t app_start_addr = 0;
//uint32_t size=0;
//volatile uint32_t debug_led_counter;

//====================================================================================
int main(void)
   {
		 uint32_t app_start_addr = 0;
   enum
      {
      check_control_word,
      check_boot_record,
      select_app,
      run_app,
      boot_rec_error,
			fw_error,
      }state=check_control_word;
   //RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);
   Bsp_init();
//   uint32_t del=0xFFFFFF;
#ifdef WR_TEST_RECORD
   WrTestLoaderRec(LAST_PAGE_ADDR);
#endif

   for (;;)
      {
      switch (state)
         {
         case check_control_word:
            {
               if (boot_rec_ptr->control_word ==FACTORY_RECORD)//it is Factory Fw, start without CRC control
                  {
										for(int i=0;i<2;i++)
										{
                     LED_GSM_ON();
											Delay(0xFFFFF/4);
											LED_GSM_OFF();
											Delay(0xFFFFF/4);
										}
                  app_start_addr= APP_0_START_ADDR;
                  state=run_app;
                  }
               else state=check_boot_record;
            }
            break;
         case check_boot_record:
            {
               uint32_t boot_rec_size=(uint32_t)&(boot_rec_ptr->crc) - (uint32_t)boot_rec_ptr;
               if (CalcCrc32((u8*)boot_rec_ptr, boot_rec_size) == boot_rec_ptr->crc)state=select_app;
               else state=boot_rec_error;
            }
            break;
         case select_app:
            {
              if(boot_rec_ptr->fw_to_run==0)
							{
								if(true==CheckFw(&boot_rec_ptr->fw0))
								{
									for(int i=0;i<3;i++)
										{
                      LED_GSM_ON();
											Delay(0xFFFFF/4);
											LED_GSM_OFF();
											Delay(0xFFFFF/4);
										}
									app_start_addr=boot_rec_ptr->fw0.start_addr;// APP_0_START_ADDR;
									state=run_app;//main app selected
								}
							else if(true==CheckFw(&boot_rec_ptr->fw1))
								{
									for(int i=0;i<5;i++)
										{
                      LED_GPRS_ON();
											Delay(0xFFFFF/4);
											LED_GPRS_OFF();
											Delay(0xFFFFF/4);
										}
									app_start_addr=boot_rec_ptr->fw1.start_addr;// APP_1_START_ADDR;
									state=run_app;//RESERVED APP
								}
								else state=fw_error;
							}
							
							else 
							{
								if(true==CheckFw(&boot_rec_ptr->fw1))
								{
									 for(int i=0;i<3;i++)
										{
                      LED_GPRS_ON();
											Delay(0xFFFFF/4);
											LED_GPRS_OFF();
											Delay(0xFFFFF/4);
										}
									app_start_addr=boot_rec_ptr->fw1.start_addr;// APP_1_START_ADDR
									state=run_app;//MAIN APP
								}
							else if(true==CheckFw(&boot_rec_ptr->fw0))
								{
									for(int i=0;i<5;i++)
										{
                      LED_GSM_ON();
											Delay(0xFFFFF/4);
											LED_GSM_OFF();
											Delay(0xFFFFF/4);
										}
									app_start_addr=boot_rec_ptr->fw0.start_addr;// APP_0_START_ADDR
									state=run_app;//RESERVED APP
								}
								else state=fw_error;
							}
            }
            break;
         case run_app:
            {
							 LED_GPS_ON();
               LED_GSM_ON();
               LED_GPRS_ON();
               LED_WRPOINT_SMS_ON();
               LED_SERVER_ON();
							 	Delay(0xFFFFF/4);
               LED_GPS_OFF();
               LED_GSM_OFF();
               LED_GPRS_OFF();
               LED_WRPOINT_SMS_OFF();
               LED_SERVER_OFF();
               NVIC_SET_VECTOR_TABLE(app_start_addr);
               USER_APP_STACK_POINTER(app_start_addr);
               MAIN_USER_APP(app_start_addr); 
            }
            break;
         case boot_rec_error:
            {
               for (;;)
                  {
                    LED_GPS_OFF();
                    LED_GSM_OFF();
                    LED_GPRS_OFF();
                    LED_WRPOINT_SMS_OFF();
                    LED_SERVER_OFF();
                    Delay(0xFFFFF);
                    LED_WRPOINT_SMS_ON();
										Delay(0xFFFFF);
                  }
            }
						case fw_error:
						{
							 for (;;)
                  {
                    LED_GPS_OFF();
                    LED_GSM_OFF();
                    LED_GPRS_OFF();
                    LED_WRPOINT_SMS_OFF();
                    LED_SERVER_OFF();
                    Delay(0xFFFFF);
                    LED_SERVER_ON();
										Delay(0xFFFFF);
                  }
						}
         }
      }
   //Bsp_de_init();
//   START:
 //  Delay(500000);
//   NVIC_GENERATE_SYSTEM_RESET();
   }

void  Delay(uint32_t delay)
   {
   for (uint32_t i=0;i<delay;i++);
   }
//=============================================================================
u8 CheckFw(FirmwareDescriptorType *fw_ptr)
   {
   if (fw_ptr->size > MAX_FIRMWARE_SIZE)return false;   
   else return(CalcCrc32((u8*)(fw_ptr->start_addr), fw_ptr->size) == fw_ptr->crc);
   }
//==============================================================================
#ifdef WR_TEST_RECORD
void WrTestLoaderRec(u32 addr)
   {
   BootRecType lr;
   FLASH_Status flash_status = FLASH_COMPLETE;

   //CRC_ResetDR(); // Описание программы 0
   lr.file0.start_addr =  APP_0_START_ADDR;
   lr.file0.file_len = 100124;
   lr.file0.crc = CalcCrc32((u8*)lr.file0.start_addr, lr.file0.file_len/sizeof(u8));

   //CRC_ResetDR(); // Описание программы 1
   lr.file1.start_addr = APP_1_START_ADDR;
   lr.file1.file_len = 100124;
   lr.file1.crc = CalcCrc32((u8*)lr.file1.start_addr, lr.file1.file_len/sizeof(u8));

   lr.file_num_to_run = 1;          // Номер программы для запуска
   lr.crc_wrong_flag = U16_MAX;     // Сбрасываем флаг негодной crc для указанной программы
   //CRC_ResetDR();                 // CRC для полей file и file_num_to_run
   //lr.crc = CRC_CalcBlockCRC((u32*)&lr, ((u32)&(lr.crc)-(u32)&lr)/sizeof(u32));
   lr.crc =CalcCrc32((u8*)&lr, ((u32)&(lr.crc)-(u32)&lr)/sizeof(u8));

   // Сохраняем запись
   FLASH_Unlock();
   u16 *p = (u16*)&lr;
   FLASH_ErasePage(addr);
   for (u32 i=0; (flash_status == FLASH_COMPLETE && i<(sizeof(BootRecType)/sizeof(u16))); i++)
      {
      flash_status = FLASH_ProgramHalfWord(addr, p[i]);
      addr += sizeof(u16);
      }
   FLASH_Lock();
   }
#endif

uint32_t CalcCrc32(uint8_t *buf,uint32_t len)
   {
   unsigned int crc_table[] =
   {
      0x4DBDF21C, 0x500AE278, 0x76D3D2D4, 0x6B64C2B0,
      0x3B61B38C, 0x26D6A3E8, 0x000F9344, 0x1DB88320,
      0xA005713C, 0xBDB26158, 0x9B6B51F4, 0x86DC4190,
      0xD6D930AC, 0xCB6E20C8, 0xEDB71064, 0xF0000000
   };
   u32 i, crc=0;
   for (i=0; i<len; i++)
      {
      crc = (crc >> 4) ^ crc_table[(crc ^ (buf[i] >> 0)) & 0x0F];
      crc = (crc >> 4) ^ crc_table[(crc ^ (buf[i] >> 4)) & 0x0F];
      }
   return crc;
   }
//==============================================================================
