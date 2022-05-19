
#include "user_types.h"


#define STOP_SPEED_TRESHOLD 1
#define MOVE_SPEED_TRESHOLD 3
#define SHORT_STOP_TIME  60
#define HDOP_KOEF    3.0f
#define DOWBLE_HDOP_KOEF (HDOP_KOEF*3.0f)
#define HDOP_MAX    10.0f
#define HDOP_MAX_HALF (HDOP_MAX/2)
#define DHDOP_MIN   0.5f
#define PI        3.14159265358979323846

enum NavFilterSignals
{
	NAV_FILTER_NAV_DATA_SIG=MAX_PUB_SIG,
	NAV_FILTER_EVT_SIG,
	NAV_FILTER_RESET_SIG,
};

typedef struct
   {
   double x;
   double y;
   double z;
   }gps_decart_t;

typedef struct
   {
   gps_decart_t  dec;
   GpsDataExtendedType   gps;
   }gps_filter_t;
	 
	 /* typedef	enum 
{
	no_nmea,
	no_fix,
	fix,
}NavigationStateType;*/
	 
#ifdef DEBUG_NAV_FILTER
extern void OutDebugNavFilter( char const *str);
extern void OutDebugNavFilterSprintf( char const *str,uint32_t val);
#else
#define OutDebugNavFilter(x) __nop()
#define OutDebugNavFilterSprintf(x,y) __nop()
#endif
	 
	  
extern QActive * const AO_NavFilter;
extern void NavFilter_ctor(void);
//extern NavigationStateType NavigationState;



