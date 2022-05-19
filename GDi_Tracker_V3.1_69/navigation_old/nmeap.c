

#include "stm32f30x.h"
#include "nmeap.h"
#include "gpgga.h"
#include "gprmc.h"
#include "qp_port.h"
#include "UserTypes.h"
#include "gps_hw.h"


typedef struct
{
   QActive super;
	//uint8_t nmea_counter; 
} NMEAP;



static nmeap_context_t nmea;	   /* parser context */
static nmeap_gga_t     gga;		   /* this is where the data from GGA messages will show up */
static nmeap_rmc_t     rmc;		   /* this is where the data from RMC messages will show up */
static int             user_data; /* user can pass in anything. typically it will be a pointer to some user data */
int  status;
static QState Nmeap_initial(NMEAP * const me, QEvt const * const e);
static QState Nmeap_power_on_start(NMEAP * const me, QEvt const * const e);


static NMEAP nmeap; 
QActive * const AO_Nmeap = &nmeap.super; 


void Nmeap_ctor(void)
{
   NMEAP *me = &nmeap;
   QActive_ctor(&me->super, Q_STATE_CAST(&Nmeap_initial));
}


QState Nmeap_initial(NMEAP * const me, QEvt const * const e)
{
//   GpsParser_hwinit();	
	 QS_OBJ_DICTIONARY(&nmeap);
   QS_FUN_DICTIONARY(&Nmeap_initial);
  // QS_FUN_DICTIONARY(&Gparser_work);
	// QS_FILTER_SM_OBJ(&gparser);
   QActive_subscribe(&me->super, TIC_100ms_SIG);
   return Q_TRAN(&Nmeap_power_on_start);
}

QState Nmeap_power_on_start(NMEAP * const me, QEvt const * const e)
{
	static uint8_t reset_counter;
   switch (e->sig)
   {
      case Q_ENTRY_SIG:
			{
				   GPS_NRESET_LOW();	
						GPS_POWER_ON();	
            reset_counter=0;				
			}
				return Q_HANDLED();
			case TIC_100ms_SIG:
			{
				if(++reset_counter==10)
				{
					GPS_NRESET_HIGH();	
					//return Q_TRAN(&Gparser_work);	
				}
			}
			 return Q_HANDLED();
			 case Q_EXIT_SIG: return Q_HANDLED();
   }
   return Q_SUPER(&QHsm_top);
}
 






