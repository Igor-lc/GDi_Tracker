

#include "io.h"
#include "modem.h"
#include "disk.h"
#include "LedIndicator.h"
#include "user_types.h"
#include "config.h"
#include "nav_filter.h"

extern volatile int calc;
extern volatile uint32_t debug_counter;
extern volatile uint8_t gps_mbx,modem_mbx;
IndFlagsType ind_flag={0,0,0,0,0,0};

static uint8_t common_led_state=0,gsm_common_led_state=0,led_on_off;
#ifdef TEST
static void TestLedHandler(void);
#endif
static void NavigationLedHandler(void);
static void GsmLedHandler(void);
static void GprsLedHandler(void);
static void ServerLedHandler(void);
static void WPointSmsLedHandler(void);
static void RS485LedHandler(void);




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
	 
	 LED_GPS_ON();
   LED_GSM_ON();
   LED_GPRS_ON();
   LED_WRPOINT_SMS_ON();
   LED_SERVER_ON();
   }

void LedIndicator(void)//every 100 ms
   {
   static uint8_t counter_1000ms=0;

   if (++counter_1000ms==10)
      {
      counter_1000ms=0;
      common_led_state^=1;
      if (common_led_state==1)gsm_common_led_state^=1;
      }
   led_on_off^=1;
			#ifdef TEST
			 TestLedHandler();
			#else
	 NavigationLedHandler();
   WPointSmsLedHandler();
   GsmLedHandler();
   GprsLedHandler();
   ServerLedHandler();
	 RS485LedHandler();
			#endif
   }
	

	#ifdef TEST	 
	 void TestLedHandler(void)
	 {
		static uint32_t counter=0;
		 counter++;
		 switch(counter)
		 {
			 case 10:   LED_GPS_ON();
			 break;
			  case 20:   LED_GSM_ON();
			 break;
			  case 30:   LED_GPRS_ON();
			 break;
			  case 40:   LED_SERVER_ON();
			 break;
			  case 50:   LED_WRPOINT_SMS_ON();
			 break;
			  case 60:   LED_SENSOR_ON();
			 break;
			  case 70:   LED_RS485_ON();
			 break;
			  case 80:  
				{
					counter=0;
					LED_GPS_OFF();
					LED_GSM_OFF();
					LED_GPRS_OFF();
					LED_SERVER_OFF();
					LED_WRPOINT_SMS_OFF();
					LED_SENSOR_OFF();
					LED_RS485_OFF();
				}
			 break;
		 }
	 }

	#endif
	 
void NavigationLedHandler(void)
   {
   switch (SysInfo.nav_state)
      {
      case no_nmea:
         {
            LED_GPS_OFF();
         }
         break;     
      case no_fix:
         {
            if (led_on_off==1) LED_GPS_ON();
					  else LED_GPS_OFF();
         }
         break; 
      case fix:
         {
            if (common_led_state)LED_GPS_ON();
            else LED_GPS_OFF();
         }
         break;                                 
      }
   }
	 
	 void RS485LedHandler(void)
   {
   switch (RS485State)
      {
      case rs485_off:
         {
            LED_RS485_OFF();
         }
         break;     
      case rs485_sensors_error:
         {
            if (led_on_off==1) LED_RS485_ON();
            else  LED_RS485_OFF();
         }
         break; 
      case rs485_sensors_ok:
         {
            if (common_led_state)LED_RS485_ON();
            else LED_RS485_OFF();
         }
         break;                                 
      }
   }
	 
	  /*void RS485LedHandler(void)
   {
		 static uint8_t counter;
   switch (RS485State)
      {
      case rs485_off:
         {
      if (write_point_angle_evt_flag==1)
         {
         write_point_angle_evt_flag=0;
				 LED_RS485_ON();
         counter=1;//0.1 sec
         RS485State=rs485_write_point_angle;
         }
         }
         break;     
     
      case rs485_write_point_angle:
         {
					  LED_RS485_ON();
            if (counter==0)
						{
							LED_RS485_OFF();
							RS485State=rs485_off;
						}
						else counter--;
         }
         break;                                
      }
   }*/

void GsmLedHandler(void)
   {
   static GsmStateType gsm_led_state=gsm_modem_off;
   static uint8_t counter,entry_flag;

   if (gsm_led_state!=GsmState)
      {
      gsm_led_state=GsmState;
      entry_flag=1;
      }

   switch (GsmState)
      {
      case gsm_modem_off:
         {
            LED_GSM_OFF();
         }
         break;     
      case gsm_registration:
         {
            if (led_on_off==1) LED_GSM_ON();
            else  LED_GSM_OFF();
         }
         break; 
      case gsm_registered_in_home:
         {
            if (common_led_state)LED_GSM_ON();
            else LED_GSM_OFF();
         }
         break;
      case gsm_registered_in_roaming:
         {
            static uint8_t local_common_led_state=0,gsm_led_on_off;
            if (local_common_led_state!=common_led_state)
               {
               if (common_led_state==1)gsm_led_on_off^=1;
               local_common_led_state=common_led_state;
               }
            if (common_led_state==1)
               {
               if (gsm_led_on_off==1)LED_GSM_ON();
               else LED_GSM_OFF(); 
               }
            else if (common_led_state==0)  LED_GSM_OFF();
         }
         break;
      case gsm_reg_error:
         {
            if (entry_flag==1)
               {
               entry_flag=0;
               counter=0;
               LED_GSM_OFF();
               }
            switch (counter++)
               {
               case 0:
               case 1:
               case 2:
               case 3:
               case 4: 
               case 5:
               case 6:
               case 7: 
               case 8:
               case 9: 
               case 10:
               case 11:
               case 12:
               case 13:
               case 14: 
               case 15:
               case 16:
               case 17:  
               case 18:
               case 19:
								 break;
							 case 20:
               case 21: LED_GSM_ON();
                  break;
							 case 22:
							 case 23: LED_GSM_OFF();
                  break;
							 case 24:
							 case 25: LED_GSM_ON();
                  break;
							 case 26:
							 case 27: LED_GSM_OFF();
                  break;
							 case 28:
							 case 29: LED_GSM_ON();
                  break;
							 case 30:
               case 31: LED_GSM_OFF();
                  counter=0;
                  break;   
               }
         }
         break;
      case gsm_sim_error:
         {
            if (entry_flag==1)
               {
               entry_flag=0;
               counter=0;
               LED_GSM_OFF();
               }
            switch (counter++)
               {
               case 0:
               case 1:
               case 2:
               case 3:
               case 4:
               case 5:
               case 6:
               case 7:
               case 8:
               case 9:
               case 10:
               case 11:
               case 12:
               case 13:
               case 14:
               case 15:
               case 16:
               case 17:
               case 18:
               case 19: break;
							 case 20:
               case 21:  LED_GSM_ON();
                  break;
							 case 22:
               case 23: LED_GSM_OFF();
							  if (gprs_ptr==&conf.gprs[0])counter=0;
                  break;
							 case 24:
               case 25: LED_GSM_ON();
                  break;
							 case 26:
               case 27: LED_GSM_OFF();
                  counter=0;
                  break;   
               }
         }
         break;
      case gsm_sim_lock_error:
         {
            if (entry_flag==1)
               {
               entry_flag=0;
               counter=0;
               LED_GSM_ON();
               }
         }
         break;
      }
   }

void GprsLedHandler(void)
   {
   static uint8_t counter,entry_flag;
   static GprsStateType  gprs_led_state=gprs_off;
   if (gprs_led_state!=GprsState)
      {
      gprs_led_state=GprsState;
      entry_flag=1;
      }
   switch (gprs_led_state)
      {
      case gprs_off:
         {
            LED_GPRS_OFF();
         }
         break;     
      case gprs_is_open:
         {
            if (common_led_state)  LED_GPRS_ON();
            else  LED_GPRS_OFF();
         }
         break; 
      case gprs_opening:
         {
            if (led_on_off==1) LED_GPRS_ON();
            else  LED_GPRS_OFF();
         }
         break; 
      case gprs_open_error:
         {
            if (entry_flag==1)
               {
               entry_flag=0;
               counter=0;
               LED_GPRS_OFF();
               }
            switch (counter++)
               {
               case 0:
               case 1:
               case 2:
               case 3:
               case 4: 
               case 5:
               case 6:
               case 7:
               case 8: 
               case 9:   
               case 10:
               case 11:
               case 12:
               case 13:
               case 14: 
               case 15:
               case 16:
               case 17:
               case 18:  
               case 19: break;
               case 20: LED_GPRS_ON();
                  break;
               case 21: LED_GPRS_OFF();
                  counter=0;
                  break;   
               }
         }
         break;      
      }
   }

void ServerLedHandler(void)
   {
   static uint8_t counter,lock=0,entry_flag;
   static ServerStateType  server_led_state=server_not_connected;

   if (lock==0)
      {
      if (ind_flag.send_packet_succes ==1)
         {
         ind_flag.send_packet_succes=0;
         counter=20;//2 sec
         server_led_state=server_send_success;
         lock=1;
         }
      else
         {
         if (server_led_state!=ServerState)
            {
            server_led_state=ServerState;
            entry_flag=1;
            }
         }
      }
   switch (server_led_state)
      {
      case server_not_connected:
         {
            LED_SERVER_OFF();
         }
         break;     
      case server_authorized_and_connected:
         {
            if (common_led_state)  LED_SERVER_ON();
            else  LED_SERVER_OFF();
         }
         break; 
      case server_connecting:
         {
             LED_SERVER_ON();
					  //if (led_on_off==1) LED_SERVER_ON();
           // else  LED_SERVER_OFF();
         }
         break;
      case server_send_success:
         {
            if (led_on_off==1) LED_SERVER_ON();
            else  LED_SERVER_OFF();
            if (--counter==0)lock=0;
         }
         break;
      case server_connect_error:
         {
            if (entry_flag==1)
               {
               entry_flag=0;
               counter=0;
               LED_SERVER_OFF();
               }
            switch (counter++)
               {
               case 0:
               case 1:
               case 2:
               case 3:
               case 4:
               case 5:
               case 6:
               case 7:
               case 8:
               case 9:
               case 10:
               case 11:
               case 12:
               case 13:
               case 14:
               case 15:
               case 16:
               case 17:
               case 18:
               case 19: break;
               case 20: LED_SERVER_ON();
                  break;
               case 21: LED_SERVER_OFF();
                  counter=0;
                  break;   
               }
         }
         break;
      case server_authorization_error:
         {
            if (entry_flag==1)
               {
               entry_flag=0;
               counter=0;
               LED_SERVER_OFF();
               }
            switch (counter++)
               {
               case 0:
               case 1:
               case 2:
               case 3:
               case 4:
               case 5:
               case 6:
               case 7:
               case 8:
               case 9:
               case 10:
               case 11:
               case 12:
               case 13:
               case 14:
               case 15:
               case 16:
               case 17:
               case 18:
               case 19:  break;
               case 20: LED_SERVER_ON(); break;
               case 21: LED_SERVER_OFF(); break;
               case 22: LED_SERVER_ON(); break;
               case 23: LED_SERVER_OFF();
                  counter=0;
                  break;   
               }
         }
         break;            
      }
   }

void WPointSmsLedHandler(void)
   {
   typedef enum
      {
      led_off,
      sms_send,
      sms_received,
      sms_command_complete,				
      write_point, 
			disk_format,
      }WPointSmsLedStateType; 
   static uint8_t counter,entry_flag,lock_state=0;
   static WPointSmsLedStateType  led_state=led_off;
   if (lock_state==0)
      {
      if (ind_flag.send_sms_succes==1)
         {
         ind_flag.send_sms_succes=0;
         led_state=sms_send;
         entry_flag=1;
         lock_state=1;
         }
      else if (ind_flag.write_point==1)
         {
         ind_flag.write_point=0;
         led_state=write_point;
         entry_flag=1;
         lock_state=1;
         LED_WRPOINT_SMS_OFF();
					 counter=10;//led off delay
         }
      else if (ind_flag.new_sms_received==1)
         {
         ind_flag.new_sms_received=0;
         led_state=sms_received;
         entry_flag=1;
         lock_state=1;
         }
			else if (ind_flag.sms_command_complete==1)
         {
         ind_flag.sms_command_complete=0;
         led_state=sms_command_complete;
         entry_flag=1;
         lock_state=1;
         }
			else if (ind_flag.disk_format ==1)
         {
         led_state=disk_format;
         }
      else led_state=led_off;
      }

   switch (led_state)
      {
      case led_off:
         {
            LED_WRPOINT_SMS_OFF();
         }
         break;
      case sms_send:
         {
            if (entry_flag==1)
               {
               entry_flag=0;
               counter=0;
               LED_WRPOINT_SMS_OFF();
               }
            switch (counter++)
               {
               case 0:
               case 1:
               case 2:
               case 3:
               case 4:
               case 5:
               case 6:
               case 7:
               case 8:
               case 9:
               case 10:
               case 11:
               case 12:
               case 13:
               case 14:
               case 15:
               case 16:
               case 17:
               case 18:
               case 19: break;
               case 20: 
               case 21: LED_WRPOINT_SMS_ON();
                         break; 
               case 22:
               case 23: LED_WRPOINT_SMS_OFF();
                        break;   
               case 24: 
               case 25: LED_WRPOINT_SMS_ON();
                        break;   
               case 26: 
               case 27: LED_WRPOINT_SMS_OFF();
							          break;
							 case 28: 
               case 29: LED_WRPOINT_SMS_ON();
                        break;   
               case 30: 
               case 31: LED_WRPOINT_SMS_OFF();
							          break;
							 case 32: 
               case 33: LED_WRPOINT_SMS_ON();
                        break;   
               case 34: 
               case 35: LED_WRPOINT_SMS_OFF();
                  counter=0;
                  lock_state=0;
                  break;             
               } 
         }
         break;    
      case sms_received:
         {
            if (entry_flag==1)
               {
               entry_flag=0;
               counter=0;
               LED_WRPOINT_SMS_OFF();
               }
							 
            switch (counter++)
               {
               case 0:
               case 1:
               case 2:
               case 3:
               case 4:
               case 5:
               case 6:
               case 7:
               case 8:
               case 9:
               case 10:
               case 11:
               case 12:
               case 13:
               case 14:
               case 15:
               case 16:
               case 17:
               case 18:
               case 19: break;
               case 20: LED_WRPOINT_SMS_ON();
                  break;
               case 21: LED_WRPOINT_SMS_OFF();
                  break; 
               case 22: LED_WRPOINT_SMS_ON();
                  break;
               case 23: LED_WRPOINT_SMS_OFF();
                  break;   
               case 24: LED_WRPOINT_SMS_ON();
                  break;
							 case 25: LED_WRPOINT_SMS_OFF();
                  break;   
               case 26: LED_WRPOINT_SMS_ON();
                  break;
               case 27: LED_WRPOINT_SMS_OFF();
                  break;   
               case 28: LED_WRPOINT_SMS_ON();
                  break;
               case 29: LED_WRPOINT_SMS_OFF();
                  counter=0;
                  lock_state=0;
                  break;             
               }
         }
         break;  
      case write_point:
         {
            if (entry_flag==1)
               {
               if (--counter==0)
                  {
                  entry_flag=0;
                  LED_WRPOINT_SMS_ON();
										 counter=3;//led on delay
                  }
               }
            else
               {
               if (--counter==0)
							   {
								  LED_WRPOINT_SMS_OFF();
								 lock_state=0;
							   }
               }
         }
         break;   
				  case sms_command_complete:
         {
            if (entry_flag==1)
               {
               if (--counter==0)
                  {
                  entry_flag=0;
                  LED_WRPOINT_SMS_ON();
										 counter=30;//led on delay
                  }
               }
            else
               {
               if (--counter==0)
							   {
								  LED_WRPOINT_SMS_OFF();
								 lock_state=0;
							   }
               }
         }
         break;   
case disk_format:
{
  LED_WRPOINT_SMS_ON();
}
break;
      }
   }











