

#include "stm32f30x.h"
#include "qp_port.h"
#include "LedIndicator.h"
#include "nav_receiver.h"
#include "nav_filter.h"
#include "nmea_parser.h"
#include <ctype.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "gxrmc.h" 
#include "gpgga.h"
#include "gpvtg.h"
#include "gxgsa.h"
#include "gxgsv.h"
#include "gxgll.h"
#include "pmtk010.h"
#include "pstmtg.h"
#include "gptxt.h"
#include "gps_hw.h"
#include <assert.h>
#include "my_time.h"
#include "config.h"
#include "Disk.h"
#include "Control.h"
#include "bsp.h"

#ifndef _CONFIG_SYS_H
   #error "ConfigSys.h" NOT INCLUDED !!!
#endif

typedef enum
{
	unknown_mode,
	sim68_mode,
	l7x_mode,
}NavReceiverModeEnum;

typedef struct
   {
   QActive super;
   uint8_t nav_buf_counter;
   uint8_t reset_counter;
		  QTimeEvt TimeEvt;
   } NAVRECEIVER;

//------------------GLOBAL-----------------------------------------------------
//------------------PRIVATE--------------------------------------------------------
static NavReceiverModeEnum receiver_mode;
static QEvt const navFilterResetEvt ={ NAV_FILTER_RESET_SIG, 0U, 0U}; 
 NmeaParserType parser;   
static nmeap_gga_t     gpgga; 
static nmeap_vtg_t     gpvtg; 
static nmeap_rmc_t     gprmc;
static nmeap_rmc_t     gnrmc;
static nmeap_gsa_t     gpgsa; 
static nmeap_gsa_t     gngsa; 
static nmeap_gsv_t     gpgsv; 
static nmeap_gsv_t     gngsv; 
static nmeap_gll_t     gpgll;
static nmeap_gll_t     gngll;
static nmeap_pmtk010_t pmtk010;
static nmeap_pstmtg_t  pstmtg;
static nmeap_gptxt_t   gptxt;
static int  user_data;
static char gps_buf_1[GPS_BUF_SIZE];
static char gps_buf_2[GPS_BUF_SIZE];
static char *current_gps_buf_ptr=gps_buf_1;
static GpsDataExtendedType nav_data1;
//static GpsDataExtendedType nav_data2;
static GpsDataExtendedType *nav_data_ptr;
static void NavReceiver_gpio_init (void);
static void NavReceiver_uart_init (uint32_t baud);
static bool SIM68ResetStateMachine(void);
static void gpvtg_cb(NmeaParserType *context,void *data,void *user_data);
static void gxrmc_cb(NmeaParserType *context,void *data,void *user_data);
static void gpgga_cb(NmeaParserType *context,void *data,void *user_data);
static void gxgsa_cb(NmeaParserType *context,void *data,void *user_data);
static void gxgsv_cb(NmeaParserType *context,void *data,void *user_data);
static void gxgll_cb(NmeaParserType *context,void *data,void *user_data);
static void pmtk010_cb(NmeaParserType *context,void *data,void *user_data);
static void pstmtg_cb(NmeaParserType *context,void *data,void *user_data);
static void gptxt_cb(NmeaParserType *context,void *data,void *user_data);

#ifdef DEBUG_NAVREC	
uint8_t navrec_state_history[NAVREC_STATE_HISTORY_SIZE]; 
uint8_t navrec_parser_id_history[NAVREC_PARSER_ID_HISTORY_SIZE]; 
static void ClearStateHistory(void);
static void AddStateHistory(uint8_t state);
static void ClearParserIdHistory(void);
static void print_rmc_data(nmeap_rmc_t *rmc,NmeaParserType *context);
#ifdef DEBUG_GGA
static void print_gga(nmeap_gga_t *gga,NmeaParserType *context);
#endif
#ifdef DEBUG_GSA 
static void print_gsa(nmeap_gsa_t *gsa,NmeaParserType *context);
#endif
#ifdef DEBUG_GSV
static void print_gsv(nmeap_gsv_t *gsv,NmeaParserType *context);
#endif
#ifdef DEBUG_GLL	
static void print_gll(nmeap_gll_t *gll,NmeaParserType *context);
#endif
static void print_pmtk010(nmeap_pmtk010_t *pmtk010,NmeaParserType *context);
#else
#define AddStateHistory(x)      NOP()
#define ClearStateHistory()	    NOP()
#define AddParserIdHistory(x)   NOP()
#define ClearParserIdHistory()	NOP()
#endif
static void process_rmc_data(nmeap_rmc_t *rmc);
static void process_gga(nmeap_gga_t *gga);
static void process_gsa(nmeap_gsa_t *gsa);
static void SendNavPacket(void);
static QState NavReceiver_initial1(NAVRECEIVER * const me, QEvt const * const e);
static QState NavReceiver_L7x_reset2(NAVRECEIVER * const me, QEvt const * const e);
static QState NavReceiver_L7x_start3(NAVRECEIVER * const me, QEvt const * const e);
static QState NavReceiver_L7x_work4(NAVRECEIVER * const me, QEvt const * const e);
static QState NavReceiver_SIM68_reset5(NAVRECEIVER * const me, QEvt const * const e);
static QState NavReceiver_SIM68_start6(NAVRECEIVER * const me, QEvt const * const e);
static QState NavReceiver_SIM68_work7(NAVRECEIVER * const me, QEvt const * const e);

static NAVRECEIVER navreceiver; 
QActive * const AO_NavReceiver = &navreceiver.super; 


void NavReceiver_ctor(void)
   {
   NAVRECEIVER *me = &navreceiver;
	 QTimeEvt_ctor(&me->TimeEvt,  NAV_TIMEOUT_SIG);
   QActive_ctor(&me->super, Q_STATE_CAST(&NavReceiver_initial1));
   }

QState NavReceiver_initial1(NAVRECEIVER * const me, QEvt const * const e)
   {
   int status;
		 AddStateHistory(1);
   QS_OBJ_DICTIONARY(&navreceiver);
		QS_FUN_DICTIONARY(&NavReceiver_initial1);
		QS_FUN_DICTIONARY(&NavReceiver_L7x_reset2);
		QS_FUN_DICTIONARY(&NavReceiver_L7x_start3);
		QS_FUN_DICTIONARY(&NavReceiver_L7x_work4);
		QS_FUN_DICTIONARY(&NavReceiver_SIM68_reset5);
   QS_FUN_DICTIONARY(&NavReceiver_SIM68_start6);
   QS_FUN_DICTIONARY(&NavReceiver_SIM68_work7);
   //QS_SIG_DICTIONARY(TIC_100ms_SIG, me); 
   //QS_FILTER_SM_OBJ(&gparser);
  // QActive_subscribe(&me->super, TIC_100ms_SIG);
   nmeap_init(&parser,(void *)&user_data);
	 status = nmeap_addParser(&parser,"GPTXT",nmeap_gptxt,gptxt_cb,&gptxt); 
   status = nmeap_addParser(&parser,"GPRMC",nmeap_gxrmc,gxrmc_cb,&gprmc);
   status = nmeap_addParser(&parser,"GNRMC",nmeap_gxrmc,gxrmc_cb,&gnrmc);
	 status = nmeap_addParser(&parser,"GPVTG",nmeap_gpvtg,gpvtg_cb,&gpvtg);
   status = nmeap_addParser(&parser,"GPGGA",nmeap_gpgga,gpgga_cb,&gpgga);
   status = nmeap_addParser(&parser,"GPGSA",nmeap_gxgsa,gxgsa_cb,&gpgsa);
   status = nmeap_addParser(&parser,"GNGSA",nmeap_gxgsa,gxgsa_cb,&gngsa);
   status = nmeap_addParser(&parser,"GPGSV",nmeap_gxgsv,gxgsv_cb,&gpgsv);
   status = nmeap_addParser(&parser,"GNGSV",nmeap_gxgsv,gxgsv_cb,&gngsv);
   status = nmeap_addParser(&parser,"GPGLL",nmeap_gxgll,gxgll_cb,&gpgll);
   status = nmeap_addParser(&parser,"GNGLL",nmeap_gxgll,gxgll_cb,&gngll);
   status = nmeap_addParser(&parser,"PMTK010",nmeap_pmtk010,pmtk010_cb,&pmtk010);
   status = nmeap_addParser(&parser,"PSTMTG",nmeap_pstmtg,pstmtg_cb,&pstmtg);
   assert(status == 0); 	 
   nav_data_ptr=&nav_data1;
   NavReceiver_gpio_init(); 
   me->reset_counter=0;
	 receiver_mode=unknown_mode;
	  ClearStateHistory();
		 ClearParserIdHistory();
		
   return Q_TRAN(&NavReceiver_SIM68_reset5);
   }

//-------------------L7X-----------------------------------------------------------------
QState NavReceiver_L7x_reset2(NAVRECEIVER * const me, QEvt const * const e)
{
   switch (e->sig)
   {
      case Q_ENTRY_SIG:
      {
				AddStateHistory(2);
        GPS_NRESET_LOW();	
        NavReceiver_uart_init(9600);
        me->reset_counter++;
       QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1); 				
      }
        return Q_HANDLED();
      case NAV_TIMEOUT_SIG:
      {
          GPS_NRESET_HIGH();	
      }
        return Q_TRAN(&NavReceiver_L7x_start3);	
       case Q_EXIT_SIG: return Q_HANDLED();
   }
   return Q_SUPER(&QHsm_top);
}

QState NavReceiver_L7x_start3(NAVRECEIVER * const me, QEvt const * const e)
{
   switch (e->sig)
   {
      case Q_ENTRY_SIG:
      {
				AddStateHistory(3);
       QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,BSP_5_SEC_TIMEOUT);				
      }
        return Q_HANDLED();
      case NAV_TIMEOUT_SIG: return Q_TRAN(&NavReceiver_SIM68_reset5);	
      case NAV_PMTK010_SIG:
      {
        uint8_t message=((DataPtrEvt*)e)->data.u32_data;
        if(message==PMTK010_STARTUP_MESSAGE)
            {
							receiver_mode=l7x_mode;
             OutDebugNavReceiver("PMTK010_STARTUP_MESSAGE !!!");
             if (0!=strcmp(conf_hardware.gps_name,"L7X"))
                 {
                   OutDebugNavReceiver("conf_hardware.gps_name IS NOT L7X !!!");
                   strlcpy(conf_hardware.gps_name, "L7X",MAX_GPS_NAME_STR_LEN);
                   conf_hardware.save=1;
                   QACTIVE_POST(AO_Disk,&DiskSaveConfigEvt, me);
                 }
             else
                {
                 OutDebugNavReceiver("conf_hardware.gps_name IS L7X");
                }
          return Q_TRAN(&NavReceiver_L7x_work4);	
        }
      }
     return Q_HANDLED();
      case NAV_BUF_SIG:
      {
         char *buf_ptr=((DataPtrEvt*)e)->ptr;
          for (int i = 0; i < GPS_BUF_SIZE ; i++)nmeap_parse(&parser,buf_ptr[i]);
      }
       return Q_HANDLED();
       case Q_EXIT_SIG: 
			 {
				  QTimeEvt_disarm(&me->TimeEvt);
			 }
				 return Q_HANDLED();
   }
   return Q_SUPER(&QHsm_top);
}


QState NavReceiver_L7x_work4(NAVRECEIVER * const me, QEvt const * const e)
{
   switch (e->sig)
   {
      case Q_ENTRY_SIG:
      {
				AddStateHistory(4);
        me->reset_counter=0;
        me->nav_buf_counter=0;
				// QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,BSP_5_SEC_TIMEOUT);		
      }
        return Q_HANDLED();
      case NAV_TIMEOUT_SIG:
      {
       /* if(me->nav_buf_counter<3)
        {
          me->reset_counter=0;
          return Q_TRAN(&NavReceiver_L7x_reset2);	
        }
				else 
				{
					  me->nav_buf_counter=0;
					 QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,BSP_5_SEC_TIMEOUT);
				}*/
      }
       return Q_HANDLED();
      case NAV_BUF_SIG:
      {
          me->nav_buf_counter++;
				 LED_GPS_TOGGLE();
          // char *buf_ptr=((DataPtrEvt*)e)->ptr;
         // for (int i = 0; i < GPS_BUF_SIZE ; i++)AddParserIdHistory(nmeap_parse(&parser,buf_ptr[i]));
      }
      return Q_HANDLED();
       case Q_EXIT_SIG: return Q_HANDLED();
   }
   return Q_SUPER(&QHsm_top);
}
//-------------------SIM68-----------------------------------------------------------------------------
QState NavReceiver_SIM68_reset5(NAVRECEIVER * const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
					 AddStateHistory(5);
					  QACTIVE_POST(AO_NavReceiver, &navFilterResetEvt, me);  
            NavReceiver_uart_init(115200);
            me->reset_counter++;
            QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);					 
         }
         return Q_HANDLED();
      case NAV_TIMEOUT_SIG:
         {
            if (true== SIM68ResetStateMachine())
               {
               GPS_NRESET_HIGH();  
               return Q_TRAN(&NavReceiver_SIM68_start6);            
               }
						else  QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,20);		
         }
         return Q_HANDLED();
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return Q_SUPER(&QHsm_top);
   }

QState NavReceiver_SIM68_start6(NAVRECEIVER * const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
					 AddStateHistory(6);
           QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,BSP_5_SEC_TIMEOUT);		    
         }
         return Q_HANDLED();
      case NAV_TIMEOUT_SIG:  return Q_TRAN(&NavReceiver_L7x_reset2);  
      case NAV_SIM68_SIG:
         {
					     receiver_mode=sim68_mode;
               OutDebugNavReceiver("NAV_SIM68_SIG");
               if (0!=strcmp(conf_hardware.gps_name,"SIM68"))
                  {
                  OutDebugNavReceiver("conf_hardware.gps_name IS NOT SIM68 !!!");
                  strlcpy(conf_hardware.gps_name, "SIM68",MAX_GPS_NAME_STR_LEN);
                  conf_hardware.save=1;
                  QACTIVE_POST(AO_Disk,&DiskSaveConfigEvt, me);
                  }
               else
                  {
                  OutDebugNavReceiver("conf_hardware.gps_name IS SIM68");
                  }
         }
         return Q_TRAN(&NavReceiver_SIM68_work7); 
      case NAV_BUF_SIG:
         {
            char *buf_ptr=((DataPtrEvt*)e)->ptr;
            for (int i = 0; i < GPS_BUF_SIZE ; i++)nmeap_parse(&parser,buf_ptr[i]);
         }
         return Q_HANDLED();
      case Q_EXIT_SIG: 
			 {
				  QTimeEvt_disarm(&me->TimeEvt);
			 }
				 return Q_HANDLED();
      }
   return Q_SUPER(&QHsm_top);
   }

QState NavReceiver_SIM68_work7(NAVRECEIVER * const me, QEvt const * const e)
   {
   switch (e->sig)
      {
      case Q_ENTRY_SIG:
         {
					 AddStateHistory(7);
            me->reset_counter=0;
            me->nav_buf_counter=0;
					 	 QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,BSP_5_SEC_TIMEOUT);	
         }
         return Q_HANDLED();
      case NAV_TIMEOUT_SIG:
      {
        if(me->nav_buf_counter<3)
        {
          me->reset_counter=0;
          return Q_TRAN(&NavReceiver_SIM68_reset5);	
        }
				else 
				{
					  me->nav_buf_counter=0;
					 QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,BSP_5_SEC_TIMEOUT);
				}
      }
       return Q_HANDLED();
      case NAV_BUF_SIG:
         {
             me->nav_buf_counter++;
            char *buf_ptr=((DataPtrEvt*)e)->ptr;
            for (int i = 0; i < GPS_BUF_SIZE ; i++)nmeap_parse(&parser,buf_ptr[i]);
         }
         return Q_HANDLED();
      case Q_EXIT_SIG: return Q_HANDLED();
      }
   return Q_SUPER(&QHsm_top);
   }


bool SIM68ResetStateMachine(void)
   {
   static uint8_t power_off_counter;
   static enum
      {
      start,
      power_off,
      stdbyn_up,
      power_on,
      exit,
      } state=start;
   bool ret=false;
   switch (state)
      {
      case start:
         {
            GPS_POWER_OFF();
            GPS_NRESET_LOW();
            GPS_STDBYN_LOW();
            power_off_counter=0;
            state = power_off;
         }
         break;
      case power_off:
         {
            if (++power_off_counter==50)
               {
               GPS_STDBYN_HIGH();
               state = stdbyn_up;
               }
         }
         break;
      case stdbyn_up:
         {
            GPS_POWER_ON();
            state = power_on;
         }
         break;
      case power_on:
         {
            GPS_NRESET_HIGH();
            state = exit;
         }
         break;
      case exit:
         {
            state = start;
            ret=true;
         }
         break;
      }
   return ret;
   }
//-------------------------------------------------------------------------------------------
#ifdef DEBUG_NAVREC
void print_rmc_data(nmeap_rmc_t *rmc,NmeaParserType *context)
   {
   char buf[200];
   struct tm  *ptm;
   ptm=mygmtime (rmc->unix_time);
   if (ptm==NULL) OutDebugNavReceiver("CONVERT TIME ERROR !!!!");
   else
      {
      sprintf (buf,"DATE/TIME: %s", asctime (ptm) );
      OutDebugNavReceiver(buf);
      } 
   sprintf (buf,"---------ERRORS:: crc=%lu cr=%lu lf=%lu hdr=%lu id=%lu ovr=%lu unk=%lu----------",
            context->err_crc,
            context->err_cr,
            context->err_lf,
            context->err_header,
            context->err_id,
            context->err_ovr,
            context->err_unk
           );
   OutDebugNavReceiver(buf);
   sprintf(buf,"%s warn=%c lat=%.6f lon=%.6f",
           context->input_s_name,
           rmc->warn,
           rmc->latitude,
           rmc->longitude
          );
   OutDebugNavReceiver(buf);
   sprintf(buf,"%s speed=%.2f course=%.2f magvar=%.2f",
           context->input_s_name,
           rmc->speed,
           rmc->course,
           rmc->magvar
          );
   OutDebugNavReceiver(buf);
   }
#ifdef DEBUG_GGA
void print_gga(nmeap_gga_t *gga,NmeaParserType *context)
   {
   char buf[200];
   sprintf(buf,"%s lat=%.6f lon=%.6f alt=%.0f time=%lu",
           context->input_s_name,
           gga->latitude  ,
           gga->longitude, 
           gga->altitude , 
           gga->time     
          );
   OutDebugNavReceiver(buf);
   sprintf(buf,"%s sat_used=%d fix_mode=%d hdop=%.2f geoid=%.2f",
           context->input_s_name,
           gga->sat_used,
           gga->fix_mode   ,
           gga->hdop      ,
           gga->geoid     
          );
   OutDebugNavReceiver(buf);
   }
#endif 
	 
#ifdef DEBUG_GSA 
void print_gsa(nmeap_gsa_t *gsa,NmeaParserType *context)
   {
   char buf[200];
   char temp[50];
   uint8_t ch;
   sprintf(buf,"%s mode=%c fix stat=%u",context->input_s_name,gsa->mode,gsa->fix_status);
   OutDebugNavReceiver(buf);
   buf[0]='\0';//clear buf
   strcat(buf,context->input_s_name);
   for (int i=0;i<6;i++)
      {
      ch=i+1;
      sprintf(temp," ch%u=%u",ch,gsa->sat_used_channel[i]);
      strcat(buf,temp);
      }
   OutDebugNavReceiver(buf);
   buf[0]='\0';//clear buf
   strcat(buf,context->input_s_name);
   for (int i=6;i<12;i++)
      {
      ch=i+1;
      sprintf(temp," ch%u=%u",ch,gsa->sat_used_channel[i]);
      strcat(buf,temp);
      }
   OutDebugNavReceiver(buf);
   buf[0]='\0';//clear buf
   sprintf(buf,"%s hdop=%.2f pdop=%.2f vdop=%.2f",context->input_s_name,gsa->hdop,gsa->pdop,gsa->vdop);
   OutDebugNavReceiver(buf);  
   }
#endif
	 
#ifdef DEBUG_GSV
void print_gsv(nmeap_gsv_t *gsv,NmeaParserType *context)
   {
   char buf[100];
   sprintf(buf,"%s sat_total=%u",context->input_s_name,gsv->sat_total);
   OutDebugNavReceiver(buf);
   }
#endif	 
void print_vtg(nmeap_vtg_t *vtg,NmeaParserType *context)
   {
   char buf[50];
   sprintf(buf,"%s speed_kmh=%0.2f",context->input_s_name,vtg->speed_kmh);
   OutDebugNavReceiver(buf);
   }
#ifdef DEBUG_GLL	
void print_gll(nmeap_gll_t *gll,NmeaParserType *context)
   {
   char buf[200];
   sprintf(buf,"%s lat=%.6f lon=%.6f time=%lu valid=%c pos_mode=%c",
           context->input_s_name,
           gll->latitude,
           gll->longitude,
           gll->time,
           gll->valid,
           gll->pos_mode
          );
   OutDebugNavReceiver(buf);
   }
#endif
void print_pmtk010(nmeap_pmtk010_t *pmtk010,NmeaParserType *context)
   {
   char buf[50];
   sprintf(buf,"%s message=%u",context->input_s_name,pmtk010->message);
   OutDebugNavReceiver(buf);
   }

void print_gptxt(nmeap_gptxt_t *gptxt,NmeaParserType *context)
   {
   char buf[100];
   sprintf(buf,"%s txt=%s",context->input_s_name,gptxt->txt);
   OutDebugNavReceiver(buf);
   }
	 
	 void print_pstmtg(nmeap_pstmtg_t *pstmtg,NmeaParserType *context)
   {
		 char buf[100];
   sprintf(buf,"%s:: ....\r\n",context->input_s_name);
		 OutDebugNavReceiver(buf);
   }
	 
#endif

//---------------RMC-------------------------------------------------------------------------
void process_rmc_data(nmeap_rmc_t *rmc)
   {
   nav_data_ptr->latitude=rmc->latitude;
   nav_data_ptr->longitude=rmc->longitude;
   nav_data_ptr->course= rmc->course;
   nav_data_ptr->time= rmc->unix_time;
   nav_data_ptr->speed= rmc->speed;
   }

void gxrmc_cb(NmeaParserType *context,void *data,void *user_data)
   {
   nmeap_rmc_t *rmc_ptr = (nmeap_rmc_t *)data;
   process_rmc_data(rmc_ptr);
#ifdef DEBUG_RMC	
   print_rmc_data(rmc_ptr,context);
#endif
   } 

//-------------------GGA------------------------------------------------------------------
void process_gga(nmeap_gga_t *gga)
   {
   nav_data_ptr->altitude=gga->altitude;
   nav_data_ptr->sat_used=gga->sat_used;
   }

void gpgga_cb(NmeaParserType *context,void *data,void *user_data)
   {
   nmeap_gga_t *gga_ptr = (nmeap_gga_t *)data;
   process_gga(gga_ptr);
#ifdef DEBUG_GGA	
   print_gga(gga_ptr,context);
#endif
   } 
//----------GSA-----------------------------------------
void process_gsa(nmeap_gsa_t *gsa)
   {
   nav_data_ptr->hdop=gsa->hdop;
   nav_data_ptr->vdop=gsa->vdop; 
   nav_data_ptr->pdop=gsa->pdop;   
   nav_data_ptr->pos_mode=gsa->fix_status;      
   }

void gxgsa_cb(NmeaParserType *context,void *data,void *user_data)
   {
   nmeap_gsa_t *gsa_ptr = (nmeap_gsa_t *)data;
   process_gsa(gsa_ptr);
#ifdef DEBUG_GSA	
   print_gsa(gsa_ptr,context);
#endif
   } 
//----------------GSV------------------------------------------------------------------------
void gxgsv_cb(NmeaParserType *context,void *data,void *user_data)
   {
#ifdef DEBUG_GSV
   nmeap_gsv_t *gsv_ptr = (nmeap_gsv_t *)data;
   print_gsv(gsv_ptr,context);
#endif
   } 
//----------------GPVTG------------------------------------------------
void gpvtg_cb(NmeaParserType *context,void *data,void *user_data)
   {
#ifdef DEBUG_VTG	
   nmeap_vtg_t *vtg_ptr = (nmeap_vtg_t *)data;
   print_vtg(vtg_ptr,context);
#endif
   } 
//----------------GLL--------------------------------------------------
void gxgll_cb(NmeaParserType *context,void *data,void *user_data)
   {
		if(receiver_mode==l7x_mode)
		{
			 SendNavPacket();
		}
#ifdef DEBUG_GLL	
   nmeap_gll_t *gll_ptr = (nmeap_gll_t *)data;
   print_gll(gll_ptr,context);
#endif
   } 
//-------------------PMTK010----------------------------------------------
void pmtk010_cb(NmeaParserType *context,void *data,void *user_data)
   {
   nmeap_pmtk010_t *pmtk010_ptr = (nmeap_pmtk010_t *)data;
   DataPtrEvt *pe = Q_NEW(DataPtrEvt,NAV_PMTK010_SIG);
   pe->data.u32_data =pmtk010_ptr->message;
   QACTIVE_POST(AO_NavReceiver, &pe->super, AO_NavReceiver);
#ifdef DEBUG_PMTK010	
   print_pmtk010(pmtk010_ptr,context);
#endif
   } 
	 //-------------PSTMTG----------------------------------------------	  
void pstmtg_cb(NmeaParserType *context,void *data,void *user_data)
   {
	 SendNavPacket();
		#ifdef DEBUG_PSTMTG	
		  nmeap_pstmtg_t *pstmtg_ptr = (nmeap_pstmtg_t *)data;
     print_pstmtg(pstmtg_ptr,context);
		 #endif	
   } 

	 
//----------------GPTXT---------------------------------------------------
void gptxt_cb(NmeaParserType *context,void *data,void *user_data)
   {
   nmeap_gptxt_t *gptxt_ptr = (nmeap_gptxt_t *)data;
   if (NULL!=strstr(gptxt_ptr->txt,"SIM68"))
      {
      DataPtrEvt *pe = Q_NEW(DataPtrEvt,NAV_SIM68_SIG);
      pe->ptr=gptxt_ptr->txt;
      QACTIVE_POST(AO_NavReceiver, &pe->super, AO_NavReceiver);
      }
#ifdef DEBUG_GPTXT	
   print_gptxt(gptxt_ptr,context);
#endif
   } 
	 
//---------------------------------------------------------------
void SendNavPacket(void)
   {
  // DataPtrEvt *pe = Q_NEW(DataPtrEvt,NAV_FILTER_NAV_DATA_SIG);
  // pe->ptr=nav_data_ptr;
  // QACTIVE_POST(AO_NavFilter, &pe->super, AO_NavReceiver);
  // if (nav_data_ptr==&nav_data1)nav_data_ptr=&nav_data2;
  // else nav_data_ptr=&nav_data1;
  // memset((uint8_t*)nav_data_ptr,0,sizeof(GpsDataExtendedType));
   }

void NavReceiver_uart_init (uint32_t baud)
   {
   USART_InitTypeDef USART_InitStructure;
   DMA_InitTypeDef DMA_InitStructure;
   NVIC_InitTypeDef NVIC_InitStructure;

   USART_InitStructure.USART_BaudRate = baud;
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   USART_InitStructure.USART_Parity = USART_Parity_No;
   USART_InitStructure.USART_HardwareFlowControl= USART_HardwareFlowControl_None;
   USART_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
   USART_Init(GPS_UART, &USART_InitStructure);
   USART_Cmd(GPS_UART,ENABLE);
   //----------DMA---------------------------------------------
   DMA_DeInit(DMA1_Channel6);
   DMA_InitStructure.DMA_PeripheralBaseAddr = (unsigned int) &(GPS_UART->RDR);
   DMA_InitStructure.DMA_MemoryBaseAddr = (u32)current_gps_buf_ptr;
//DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
   DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
   DMA_InitStructure.DMA_BufferSize = GPS_BUF_SIZE;
   DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
   DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
   DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
   DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
   DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
   DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
   DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
   DMA_Init(DMA1_Channel6, &DMA_InitStructure);
   USART_ClearFlag(GPS_UART, USART_FLAG_RXNE | USART_FLAG_TC | USART_FLAG_TXE);
   USART_DMACmd(GPS_UART,USART_DMAReq_Rx, ENABLE);
   DMA_Cmd(DMA1_Channel6, ENABLE);

//-----------------IRQ------------------------------------
   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);// NVIC_IRQChannelSubPriority always zero
   NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = GPS_UART_IRQ_PRIO;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure); 
   /* NVIC_SetPriority (DMA1_Channel6_IRQn, 4); 
    NVIC_EnableIRQ(DMA1_Channel6_IRQn);*/
   DMA_ITConfig(DMA1_Channel6, DMA_IT_TC, ENABLE);
   }


void NavReceiver_gpio_init (void)
   {
   GPIO_InitTypeDef GPIO_InitStructure;

   GPIO_InitStructure.GPIO_Pin = GPS_POWER_PIN;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPS_POWER_GPIO_PORT, &GPIO_InitStructure);

   GPIO_InitStructure.GPIO_Pin = GPS_STDBYN_PIN;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPS_STDBYN_GPIO_PORT, &GPIO_InitStructure);

   GPIO_InitStructure.GPIO_Pin = GPS_NRESET_PIN;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPS_NRESET_GPIO_PORT, &GPIO_InitStructure);

   GPS_POWER_OFF();
   GPS_STDBYN_HIGH();
   GPS_NRESET_LOW();

//-------USART3 RX PIN-----------------------------------	
   GPIO_PinAFConfig(GPS_UART_GPIO, GPS_UART_RX_PinSource11, GPIO_AF_7);
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Pin = GPS_UART_RX_Pin;
   GPIO_Init(GPS_UART_GPIO , &GPIO_InitStructure);
//-------USART3 TX PIN-----------------------------------			
   GPIO_PinAFConfig(GPS_UART_GPIO, GPS_UART_TX_PinSource11, GPIO_AF_7);
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Pin = GPS_UART_TX_Pin;
   GPIO_Init(GPS_UART_GPIO , &GPIO_InitStructure);
   }

void  DMA1_Channel6_IRQHandler(void)
   {
   static uint8_t Gps_IRQHandler;
   static DataPtrEvt nav_buf_evt = {NAV_BUF_SIG, 0U, 0U};
   QK_ISR_ENTRY(); 
   DMA1_Channel6->CCR &= ~DMA_CCR_EN; //turn  DMA ch off
   nav_buf_evt.ptr=current_gps_buf_ptr; 
   QACTIVE_POST(AO_NavReceiver, &nav_buf_evt.super,&Gps_IRQHandler);
   if (current_gps_buf_ptr==gps_buf_1)current_gps_buf_ptr=gps_buf_2;
   else current_gps_buf_ptr=gps_buf_1;
   DMA1_Channel6->CMAR =(u32)current_gps_buf_ptr; 
   DMA1_Channel6->CNDTR = GPS_BUF_SIZE;
   DMA1_Channel6->CCR |= DMA_CCR_EN; //turn  DMA ch  on
   DMA_ClearITPendingBit(DMA1_IT_TC6); 
   QK_ISR_EXIT(); 
   }


#ifdef DEBUG_NAVREC	
void OutDebugNavReceiver( char const *str)
   {
   QS_BEGIN(QS_NAV_RECEIVER, AO_NavReceiver)                                  
   QS_STR(str);        
   QS_END()
   }


void OutDebugNavReceiverSprintf1( char const *str,uint32_t val)
   {
   QS_BEGIN(QS_NAV_RECEIVER, AO_NavReceiver)                                  
   QS_STR(str); 
   QS_U32(4, val);       
   QS_END()
   }
	 
	 void ClearStateHistory(void)
   {
   for (uint32_t i=0;i<NAVREC_STATE_HISTORY_SIZE;i++)
      {
      navrec_state_history[i]=0;
      }
   }
	 
	 void AddStateHistory(uint8_t state)
   {
   for (uint32_t i=0;i<NAVREC_STATE_HISTORY_SIZE-1;i++)
      {
      navrec_state_history[i]=navrec_state_history[i+1];
      }
   navrec_state_history[NAVREC_STATE_HISTORY_SIZE-1]=state;
   }
	 
	 void ClearParserIdHistory(void)
   {
   for (uint32_t i=0;i<NAVREC_PARSER_ID_HISTORY_SIZE;i++)
      {
      navrec_parser_id_history[i]=0;
      }
   }
	 
/*	  void AddParserIdHistory(uint8_t id)
   {
   for (uint32_t i=0;i<NAVREC_PARSER_ID_HISTORY_SIZE-1;i++)
      {
      navrec_parser_id_history[i]=navrec_parser_id_history[i+1];
      }
   navrec_parser_id_history[NAVREC_PARSER_ID_HISTORY_SIZE-1]=id;
   }*/
#endif







