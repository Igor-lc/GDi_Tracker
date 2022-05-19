


#include "qp_port.h"
#include "nmea_parser.h"
#include "nav_filter.h"
#include <math.h>
#include <time.h>
#include <stdio.h>
#include "config.h"
#include "Disk.h"
#include "tl10.h"
#include "io.h"
#include <string.h>


typedef struct
    {
    QActive super;
    float last_course;
    float distance_total;
    time_t last_send_time;
    time_t stop_time;
#ifdef _VAR_SENS_SIZE	 
    void* free_mess_ptr;
#else
    QPointType* free_mess_ptr;
#endif
    QSensorType sensor;
    uint32_t debug_packet_counter;
    } NAVFILTER;

static struct
    {
    gps_filter_t cur;
    gps_filter_t prev;
    gps_filter_t stop;
    gps_filter_t ffix;
    }point;

static struct
    {
    float stop;
    float old;
    }hdop;


//----single instance static object and public object ptr
static NAVFILTER nav_filter; 
QActive* const AO_NavFilter = &nav_filter.super;
//NavigationStateType NavigationState=no_nmea;
//----------------------------------------------------
#ifdef _VAR_SENS_SIZE	
static MaxPointSizeType points_pool[POINTS_POOL_SIZE];    
#else
static QPointType points_pool[POINTS_POOL_SIZE];  
#endif
static int TimeValid(uint32_t time);
static gps_decart_t GpsToDec(GpsDataExtendedType *data);
static int SendPointOrEvent(NAVFILTER * const me,GpsDataExtendedType* point_ptr);
static float DeltaCource(float first, float second);
static double CalcDistance(gps_decart_t *first, gps_decart_t *second);
static void SaveCurrentPoint( QEvt const * const e);     
static QState Gfilter_initial(NAVFILTER * const me, QEvt const * const e);
static QState Gfilter_waiting_fix(NAVFILTER * const me, QEvt const * const e);
static QState Gfilter_waiting_hdop(NAVFILTER * const me, QEvt const * const e);
static QState Gfilter_move(NAVFILTER * const me, QEvt const * const e);
static QState Gfilter_short_stop(NAVFILTER * const me, QEvt const * const e);
static QState Gfilter_long_stop(NAVFILTER * const me, QEvt const * const e);
static QState Gfilter_off(NAVFILTER * const me, QEvt const * const e);

void NavFilter_ctor(void)
{
    NAVFILTER *me = &nav_filter;
//	 QTimeEvt_ctor(&me->TimeEvt,  NAV_FILTER_TIMEOUT_SIG);
    QActive_ctor(&me->super, Q_STATE_CAST(&Gfilter_initial));
}

QState Gfilter_initial(NAVFILTER * const me, QEvt const * const e)
{
    QS_OBJ_DICTIONARY(&nav_filter);
    QS_FUN_DICTIONARY(&Gfilter_initial);
    QS_FUN_DICTIONARY(&Gfilter_waiting_fix);
    QS_FUN_DICTIONARY(&Gfilter_waiting_hdop);
    QS_FUN_DICTIONARY(&Gfilter_move);
    QS_FUN_DICTIONARY(&Gfilter_short_stop);
    QS_FUN_DICTIONARY(&Gfilter_long_stop);
//	 QS_FUN_DICTIONARY(&Gfilter_test_disk);

    //QS_FILTER_SM_OBJ(&gfilter);
    me->free_mess_ptr=&points_pool[0];
//	AccelConfig();
    SysInfo.nav_state=no_nmea;
    return Q_TRAN(&Gfilter_waiting_fix);
    //  return Q_TRAN(&Gfilter_test_disk);
}

/* QState Gfilter_test_disk(NAVFILTER * const me, QEvt const * const e)
{
switch (e->sig)
  {
  case Q_ENTRY_SIG:
     {
                 me->debug_packet_counter=0;
        NavigationState=no_fix;
                    QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,1);//5 sec 
     }
     return Q_HANDLED();
  case NAV_FILTER_TIMEOUT_SIG:
     {
                   OutDebugNavFilterSprintf("SEND DEBUG PACKET num=",++me->debug_packet_counter);
                   SendPointDebug(me,&point.cur.gps);
        QTimeEvt_postIn(&me->TimeEvt, (QActive *)me,200);
     }
     return Q_HANDLED();
  case Q_EXIT_SIG: return Q_HANDLED();
  }
return Q_SUPER(&QHsm_top);
}*/


QState Gfilter_waiting_fix(NAVFILTER * const me, QEvt const * const e)
{
    QState ret= Q_HANDLED();
    switch (e->sig)
        {
        case Q_ENTRY_SIG:
            {
                SysInfo.nav_state=no_fix;
            }
            return Q_HANDLED();
        case NAV_FILTER_NAV_DATA_SIG:
            {
                watch_flag.nav_filter=0;
                SaveCurrentPoint(e);
                struct tm * timeinfo = localtime ( &point.cur.gps.time );
                if (0==TimeValid(point.cur.gps.time))
                    {
                    OutDebugNavFilter("Gfilter_waiting_fix-TIME NOT VALID");
                    }
                /* else if (point.cur.gps.status.gps_fix==0)
                    {
                    OutDebugNavFilter("Gfilter_waiting_fix-DATA NOT VALID");  
                    }*/
                else if (point.cur.gps.pos_mode < 2)
                    {
                    OutDebugNavFilterSprintf("Gfilter_waiting_fix-NO FIX, pos_mode=",point.cur.gps.pos_mode);
                    }
                else//fix ok
                    {
                    OutDebugNavFilterSprintf("Gfilter_waiting_fix-FIX OK, pos_mode=",point.cur.gps.pos_mode);
                    if (conf_nav_filter.filter_on)ret= Q_TRAN(&Gfilter_waiting_hdop);
                    else ret= Q_TRAN(&Gfilter_off);
                    } 
            }
            return ret;
        case NAV_FILTER_RESET_SIG: return Q_TRAN(&Gfilter_waiting_fix);
        case Q_EXIT_SIG: return Q_HANDLED();
        }
    return Q_SUPER(&QHsm_top);
}

QState Gfilter_waiting_hdop(NAVFILTER * const me, QEvt const * const e)
{
    static float dhdop_new, dhdop2, dhdop3;
    switch (e->sig)
        {
        case Q_ENTRY_SIG:
            {
                hdop.old= dhdop_new=dhdop2=dhdop3=HDOP_MAX;
                SysInfo.nav_state=no_fix;
            }
            return Q_HANDLED();
        case NAV_FILTER_EVT_SIG:
            {
                if (1==TimeValid(point.cur.gps.time))
                    {
                    me->sensor=((SensorEvt*)e)->sensor;//copy sensor struct
                    point.cur.gps.status.angle=0;          
                    int result=SendPointOrEvent(me,&point.cur.gps);
                    OutDebugNavFilterSprintf("SendPointOrEvent result=",result);
                    }
            }
            return Q_HANDLED();
        case NAV_FILTER_NAV_DATA_SIG:
            {
                watch_flag.nav_filter=0;
                SaveCurrentPoint(e);
                struct tm * timeinfo = localtime ( &point.cur.gps.time );
                if (0==TimeValid(point.cur.gps.time))
                    {
                    OutDebugNavFilter("Gfilter_waiting_hdop-TIME NOT VALID");
                    return Q_TRAN(&Gfilter_waiting_fix);
                    }
                /*else if (point.cur.gps.status.gps_fix==0)
                   {
                   OutDebugNavFilter("Gfilter_waiting_hdop-DATA NOT VALID"); 
                   return Q_TRAN(&Gfilter_waiting_fix);
                   }*/
                else if (point.cur.gps.pos_mode < 2)
                    {
                    OutDebugNavFilterSprintf("Gfilter_waiting_hdop NO FIX, pos_mode=",point.cur.gps.pos_mode);
                    return Q_TRAN(&Gfilter_waiting_fix);
                    }
                else//fix ok
                    {
                    dhdop_new= hdop.old-point.cur.gps.hdop;
                    hdop.old=point.cur.gps.hdop;
                    if (dhdop_new<=DHDOP_MIN&&dhdop_new<=dhdop2&&dhdop2<=dhdop3&&point.cur.gps.hdop<HDOP_MAX_HALF)
                        {
                        point.cur.gps.status.move = 1;
                        point.cur.gps.prio=1;
                        point.cur.gps.status.angle=0;
                        //macke and send event
                        me->sensor.id=EVT_ID;
                        me->sensor.descriptor.data_id=WARNING_DATA_ID;
                        me->sensor.descriptor.data_size=1;
                        me->sensor.data.any_data[0]=SysEvtFirstFix;
                        int result=SendPointOrEvent(me,&point.cur.gps);
                        OutDebugNavFilterSprintf("SendPointOrEvent result=",result);
                        SysInfo.point_store =point_move_distance;
                        char time_str[40];
                        sprintf( time_str,"%s", asctime (timeinfo) );
                        OutDebugNavFilter(time_str);
                        OutDebugNavFilter("Gfilter_waiting_hdop-EVT FIRST FIX");
                        return Q_TRAN(&Gfilter_move);
                        }
                    else
                        {
                        dhdop3=dhdop2;
                        dhdop2=dhdop_new;
                        }
                    }
            }
            return Q_HANDLED();
        case NAV_FILTER_RESET_SIG: return Q_TRAN(&Gfilter_waiting_fix);
        case Q_EXIT_SIG: return Q_HANDLED();
        }
    return Q_SUPER(&QHsm_top);
}

QState Gfilter_off(NAVFILTER * const me, QEvt const * const e)
{
    switch (e->sig)
        {
        case Q_ENTRY_SIG:
            {
                point.prev=point.cur;
                me->last_course=point.cur.gps.course;
                me->distance_total = 0;
                //me->last_time = point.cur.gps.time;
            }
            return Q_HANDLED();
        case NAV_FILTER_EVT_SIG:
            {
                if (1==TimeValid(point.cur.gps.time))
                    {
                    me->sensor=((SensorEvt*)e)->sensor;
                    point.cur.gps.status.angle=0;
                    int result=SendPointOrEvent(me,&point.cur.gps);
                    OutDebugNavFilterSprintf("SendPointOrEvent result=",result);
                    }
            }
            return Q_HANDLED();
        case NAV_FILTER_NAV_DATA_SIG:
            {
                watch_flag.nav_filter=0;
                char time_str[40];
                SaveCurrentPoint(e);
                struct tm * timeinfo = localtime ( &point.cur.gps.time );
                sprintf( time_str,"%s", asctime (timeinfo) );
                //if (1==TimeValid(point.cur.gps.time) && point.cur.gps.status.gps_fix==1 && point.cur.gps.pos_mode >= 2)
                if (1==TimeValid(point.cur.gps.time) && point.cur.gps.pos_mode >= 2)
                    {
                    me->last_course=point.cur.gps.course;
                    //me->last_time = point.cur.gps.time;
                    point.cur.gps.status.move = 1;
                    point.cur.gps.prio=0;
                    point.cur.gps.status.angle=0;
                    int result=SendPointOrEvent(me,&point.cur.gps);
                    OutDebugNavFilterSprintf("SendPointOrEvent result=",result);
                    SysInfo.point_store =point_move_time;
									  SysInfo.nav_state=fix;
                    point.prev=point.cur;
                    OutDebugNavFilter("Gfilter_off-POINT");
                    if (conf_nav_filter.filter_on)return Q_TRAN(&Gfilter_waiting_hdop);
                    }
                else//no fix 
                    {
                    me->distance_total = 0;
                    point.prev.gps.status.move = 1;
                    point.prev.gps.prio=1;
                    point.cur.gps.status.angle=0;
                    me->sensor.id=EVT_ID;
                    me->sensor.descriptor.data_id=WARNING_DATA_ID;
                    me->sensor.descriptor.data_size=1;
                    me->sensor.data.any_data[0]=SysEvtLostFix;
                    int result=SendPointOrEvent(me,&point.prev.gps);
                    OutDebugNavFilterSprintf("SendPointOrEvent result=",result);
                    SysInfo.point_store =point_lost_fix;
                    OutDebugNavFilter(time_str);
                    OutDebugNavFilter("Gfilter_off-POINT LOST FIX");
                    return Q_TRAN(&Gfilter_waiting_fix);
                    }    
            }
            return Q_HANDLED();
        case NAV_FILTER_RESET_SIG: return Q_TRAN(&Gfilter_waiting_fix);
        case Q_EXIT_SIG: return Q_HANDLED();
        }
    return Q_SUPER(&QHsm_top);
}


QState Gfilter_move(NAVFILTER * const me, QEvt const * const e)
{
    QState ret= Q_HANDLED();
    switch (e->sig)
        {
        case Q_ENTRY_SIG:
            {
                point.prev=point.cur;
                me->last_course=point.cur.gps.course;
                me->distance_total = 0;
                //me->last_time = point.cur.gps.time;
                SysInfo.nav_state=fix;
            }
            return Q_HANDLED();
        case NAV_FILTER_EVT_SIG:
            {
                if (1==TimeValid(point.cur.gps.time))
                    {
                    me->sensor=((SensorEvt*)e)->sensor;
                    point.cur.gps.status.angle=0;
                    int result=SendPointOrEvent(me,&point.cur.gps);
                    OutDebugNavFilterSprintf("SendPointOrEvent result=",result);
                    }
            }
            return Q_HANDLED();
        case NAV_FILTER_NAV_DATA_SIG:
            {
                watch_flag.nav_filter=0;
                if (conf_nav_filter.filter_on==0)return Q_TRAN(&Gfilter_off);
                char time_str[40];
                SaveCurrentPoint(e);
                struct tm * timeinfo = localtime ( &point.cur.gps.time );
                sprintf( time_str,"%s", asctime (timeinfo) );
                // if (1==TimeValid(point.cur.gps.time) && point.cur.gps.status.gps_fix==1 && point.cur.gps.pos_mode >= 2)
                if (1==TimeValid(point.cur.gps.time) && point.cur.gps.pos_mode >=2)
                    {
                    if (point.cur.gps.speed < STOP_SPEED_TRESHOLD)
                        {
                        point.cur.gps.status.move = 0;
                        point.cur.gps.prio=1;
                        point.cur.gps.status.angle=0;

                        me->sensor.id=EVT_ID;
                        me->sensor.descriptor.data_id=WARNING_DATA_ID;
                        me->sensor.descriptor.data_size=1;
                        me->sensor.data.any_data[0]=SysEvtStopMove;
                        int result=SendPointOrEvent(me,&point.cur.gps);
                        OutDebugNavFilterSprintf("SendPointOrEvent result=",result);
                        SysInfo.point_store =point_stop;
                        OutDebugNavFilter(time_str);
                        OutDebugNavFilter("Gfilter_move-POINT STOP");
                        return Q_TRAN(&Gfilter_short_stop);
                        }
                    float tmp_distanse = CalcDistance(&point.prev.dec, &point.cur.dec);
                    float delta_course = DeltaCource(me->last_course, point.cur.gps.course);
                    me->distance_total  += tmp_distanse;
                    if (fabs(delta_course) > conf_nav_filter.angle)
                        {
                        me->distance_total = 0;
                        me->last_course=point.cur.gps.course;
//                  me->last_time = point.cur.gps.time;
                        point.cur.gps.status.move = 1;
                        point.cur.gps.prio=0;
                        point.cur.gps.status.angle=1;
                        me->sensor.id=EVT_ID;
                        me->sensor.descriptor.data_id=WARNING_DATA_ID;
                        me->sensor.descriptor.data_size=1;
                        me->sensor.data.any_data[0]=SysEvtCorner;
                        int result=SendPointOrEvent(me,&point.cur.gps);
                        OutDebugNavFilterSprintf("SendPointOrEvent result=",result);
                        SysInfo.point_store =point_move_angle;
                        point.prev=point.cur;
                        OutDebugNavFilter(time_str);
                        OutDebugNavFilter("Gfilter_move-POINT ANGLE");
                        }
                    else if (me->distance_total > conf_nav_filter.dist)
                        {
                        me->distance_total = 0;
                        me->last_course=point.cur.gps.course;
                        // me->last_time = point.cur.gps.time;
                        point.cur.gps.status.move = 1;
                        point.cur.gps.prio=0;
                        point.cur.gps.status.angle=0;
                        int result=SendPointOrEvent(me,&point.cur.gps);
                        OutDebugNavFilterSprintf("SendPointOrEvent result=",result);
                        SysInfo.point_store =point_move_distance;
                        point.prev=point.cur;
                        OutDebugNavFilter(time_str);
                        OutDebugNavFilter("Gfilter_move-POINT DISTANCE");
                        }

                    else if (point.cur.gps.time - me->last_send_time > conf_nav_filter.time && conf_nav_filter.time!=0)
                        {
                        me->distance_total = 0;
                        me->last_course=point.cur.gps.course;
                        // me->last_time = point.cur.gps.time;
                        point.cur.gps.status.move = 1;
                        point.cur.gps.prio=0;
                        point.cur.gps.status.angle=0;
                        int result=SendPointOrEvent(me,&point.cur.gps);
                        OutDebugNavFilterSprintf("SendPointOrEvent result=",result);
                        SysInfo.point_store =point_move_time;
                        point.prev=point.cur;
                        OutDebugNavFilter(time_str);
                        OutDebugNavFilter("Gfilter_move-POINT TIME");
                        }
                    }
                else
                    {
                    me->distance_total = 0;
                    point.prev.gps.status.move = 1;
                    point.prev.gps.prio=1;
                    point.cur.gps.status.angle=0;
                    me->sensor.id=EVT_ID;
                    me->sensor.descriptor.data_id=WARNING_DATA_ID;
                    me->sensor.descriptor.data_size=1;
                    me->sensor.data.any_data[0]=SysEvtLostFix;
                    int result=SendPointOrEvent(me,&point.prev.gps);
                    OutDebugNavFilterSprintf("SendPointOrEvent result=",result);
                    SysInfo.point_store =point_lost_fix;
                    OutDebugNavFilter(time_str);
                    OutDebugNavFilter("Gfilter_move-POINT LOST FIX");
                    ret= Q_TRAN(&Gfilter_waiting_fix);
                    }    
            }
            return ret;
        case NAV_FILTER_RESET_SIG: return Q_TRAN(&Gfilter_waiting_fix);
        case Q_EXIT_SIG: return Q_HANDLED();
        }
    return Q_SUPER(&QHsm_top);
}

QState Gfilter_short_stop(NAVFILTER * const me, QEvt const * const e)
{
    QState ret= Q_HANDLED();
    switch (e->sig)
        {
        case Q_ENTRY_SIG:
            {
                point.stop=point.cur;
                point.stop.gps.status.move = 0;
                me->distance_total = 0;
                me->stop_time = point.cur.gps.time;
                SysInfo.nav_state=fix;
            }
            return Q_HANDLED();
        case NAV_FILTER_EVT_SIG:
            {
                if (1==TimeValid(point.cur.gps.time))
                    {
                    me->sensor=((SensorEvt*)e)->sensor;
                    point.cur.gps.status.angle=0;
                    int result=SendPointOrEvent(me,&point.cur.gps);
                    OutDebugNavFilterSprintf("SendPointOrEvent result=",result);
                    }
            }
            return Q_HANDLED();
        case NAV_FILTER_NAV_DATA_SIG:
            {
                watch_flag.nav_filter=0;
                if (conf_nav_filter.filter_on==0)return Q_TRAN(&Gfilter_off);
                char time_str[40];
                SaveCurrentPoint(e);
                struct tm * timeinfo = localtime ( &point.cur.gps.time );
                sprintf( time_str,"%s", asctime (timeinfo) );
                // if (1==TimeValid(point.cur.gps.time) && point.cur.gps.status.gps_fix==1 && point.cur.gps.pos_mode >= 2)
                if (1==TimeValid(point.cur.gps.time) && point.cur.gps.pos_mode >=2)
                    {
                    float tmp_distanse = CalcDistance(&point.stop.dec, &point.cur.dec);
                    //if ((tmp_distanse > point.stop.gps.hdop * HDOP_KOEF + point.cur.gps.hdop * HDOP_KOEF) && (point.cur.gps.speed > MOVE_SPEED_TRESHOLD) && (point.cur.gps.move))
                    if ((tmp_distanse > point.stop.gps.hdop * HDOP_KOEF + point.cur.gps.hdop * HDOP_KOEF) && (point.cur.gps.speed > MOVE_SPEED_TRESHOLD))
                        {
                        point.cur.gps.status.move = 1;
                        point.cur.gps.prio=1;
                        point.cur.gps.status.angle=0;
                        me->sensor.id=EVT_ID;
                        me->sensor.descriptor.data_id=WARNING_DATA_ID;
                        me->sensor.descriptor.data_size=1;
                        me->sensor.data.any_data[0]=SysEvtStartMove;
                        int result=SendPointOrEvent(me,&point.cur.gps);
                        OutDebugNavFilterSprintf("SendPointOrEvent result=",result);
                        SysInfo.point_store =point_move_distance;
                        OutDebugNavFilter(time_str);
                        OutDebugNavFilter("Gfilter_short_stop-POINT STOP -> MOVE");
                        return Q_TRAN(&Gfilter_move);
                        }
                    if (point.cur.gps.time - me->stop_time>SHORT_STOP_TIME)
                        {
                        return Q_TRAN(&Gfilter_long_stop);
                        }
                    OutDebugNavFilterSprintf("conf_nav_filter.time=",conf_nav_filter.time);
                    OutDebugNavFilterSprintf("delta_time=",point.cur.gps.time - me->last_send_time);
                    if (point.cur.gps.time - me->last_send_time > conf_nav_filter.time && conf_nav_filter.time!=0)
                        {
                        point.stop.gps.time = point.cur.gps.time;
                        point.stop.gps.sat.used = point.cur.gps.sat.used;
                        point.stop.gps.pdop = point.cur.gps.pdop;
                        point.stop.gps.hdop = point.cur.gps.hdop;
                        point.stop.gps.vdop = point.cur.gps.vdop;
                        point.stop.gps.pos_mode = point.cur.gps.pos_mode;
                        OutDebugNavFilter(time_str);
                        OutDebugNavFilter("Gfilter_short_stop-POINT TIME");
                        //me->last_time = point.cur.gps.time;
                        point.stop.gps.prio=0;
                        point.cur.gps.status.angle=0;
                        int result=SendPointOrEvent(me,&point.stop.gps);
                        OutDebugNavFilterSprintf("SendPointOrEvent result=",result);
                        SysInfo.point_store =point_stop;
                        point.prev=point.cur;
                        }
                    }
                else
                    {
                    point.prev.gps.status.move = 0;
                    point.prev.gps.prio=1;
                    point.cur.gps.status.angle=0;
                    me->sensor.id=EVT_ID;
                    me->sensor.descriptor.data_id=WARNING_DATA_ID;
                    me->sensor.descriptor.data_size=1;
                    me->sensor.data.any_data[0]=SysEvtLostFix;
                    int result=SendPointOrEvent(me,&point.prev.gps);
                    OutDebugNavFilterSprintf("SendPointOrEvent result=",result);
                    SysInfo.point_store =point_lost_fix;
                    OutDebugNavFilter(time_str);
                    OutDebugNavFilter("Gfilter_short_stop-POINT LOST FIX");
                    ret= Q_TRAN(&Gfilter_waiting_fix);
                    }  
            }
            return ret;
        case NAV_FILTER_RESET_SIG: return Q_TRAN(&Gfilter_waiting_fix);
        case Q_EXIT_SIG: return Q_HANDLED();
        }
    return Q_SUPER(&QHsm_top);
}


QState Gfilter_long_stop(NAVFILTER * const me, QEvt const * const e)
{
    QState ret= Q_HANDLED();
    switch (e->sig)
        {
        case Q_ENTRY_SIG: return Q_HANDLED();
        case NAV_FILTER_EVT_SIG:
            {
                if (1==TimeValid(point.cur.gps.time))
                    {
                    me->sensor=((SensorEvt*)e)->sensor;
                    point.cur.gps.status.angle=0;
                    int result=SendPointOrEvent(me,&point.cur.gps);
                    OutDebugNavFilterSprintf("SendPointOrEvent result=",result);
                    }
            }
            return Q_HANDLED();
        case NAV_FILTER_NAV_DATA_SIG:
            {
                watch_flag.nav_filter=0;
                if (conf_nav_filter.filter_on==0)return Q_TRAN(&Gfilter_off);
                char time_str[40];
                SaveCurrentPoint(e);
                struct tm * timeinfo = localtime ( &point.cur.gps.time );
                sprintf( time_str,"%s", asctime (timeinfo) );
                if (1==TimeValid(point.cur.gps.time) && point.cur.gps.pos_mode >= 2)
                    {
                    float tmp_distanse = CalcDistance(&point.stop.dec, &point.cur.dec);
                    //if ((tmp_distanse > point.stop.gps.hdop * DOWBLE_HDOP_KOEF + point.cur.gps.hdop * DOWBLE_HDOP_KOEF) && (point.cur.gps.speed > MOVE_SPEED_TRESHOLD) && (point.cur.gps.move))
                    if ((tmp_distanse > point.stop.gps.hdop * DOWBLE_HDOP_KOEF + point.cur.gps.hdop * DOWBLE_HDOP_KOEF) && (point.cur.gps.speed > MOVE_SPEED_TRESHOLD))
                        {
                        point.cur.gps.status.move = 1;
                        point.cur.gps.prio=1;
                        point.cur.gps.status.angle=0;
                        me->sensor.id=EVT_ID;
                        me->sensor.descriptor.data_id=WARNING_DATA_ID;
                        me->sensor.descriptor.data_size=1;
                        me->sensor.data.any_data[0]=SysEvtStartMove;
                        int result=SendPointOrEvent(me,&point.cur.gps);
                        OutDebugNavFilterSprintf("SendPointOrEvent result=",result);
                        SysInfo.point_store =point_move_distance;
                        OutDebugNavFilter(time_str);
                        OutDebugNavFilter("Gfilter_long_stop-POINT STOP -> MOVE");
                        return Q_TRAN(&Gfilter_move);
                        }
                    OutDebugNavFilterSprintf("conf_nav_filter.time=",conf_nav_filter.time);
                    OutDebugNavFilterSprintf("delta_time=",point.cur.gps.time - me->last_send_time);
                    if (point.cur.gps.time - me->last_send_time > conf_nav_filter.time && conf_nav_filter.time!=0)
                        {
                        point.stop.gps.time = point.cur.gps.time;
                        point.stop.gps.sat.used = point.cur.gps.sat.used;
                        point.stop.gps.pdop = point.cur.gps.pdop;
                        point.stop.gps.hdop = point.cur.gps.hdop;
                        point.stop.gps.vdop = point.cur.gps.vdop;
                        point.stop.gps.pos_mode = point.cur.gps.pos_mode;
                        OutDebugNavFilter(time_str);
                        OutDebugNavFilter("Gfilter_long_stop-POINT TIME");
                        // me->last_time = point.cur.gps.time;
                        point.stop.gps.prio=0;
                        point.cur.gps.status.angle=0;
                        int result=SendPointOrEvent(me,&point.stop.gps);
                        OutDebugNavFilterSprintf("SendPointOrEvent result=",result);
                        SysInfo.point_store =point_stop;
                        point.prev=point.cur;
                        }
                    }
                else
                    {
                    me->distance_total = 0;
                    point.prev.gps.status.move = 0;
                    point.prev.gps.prio=1;
                    point.cur.gps.status.angle=0;
                    me->sensor.id=EVT_ID;
                    me->sensor.descriptor.data_id=WARNING_DATA_ID;
                    me->sensor.descriptor.data_size=1;
                    me->sensor.data.any_data[0]=SysEvtLostFix;
                    int result=SendPointOrEvent(me,&point.prev.gps);
                    OutDebugNavFilterSprintf("SendPointOrEvent result=",result);
                    SysInfo.point_store =point_lost_fix;
                    OutDebugNavFilter(time_str);
                    OutDebugNavFilter("Gfilter_long_stop-POINT LOST FIX");
                    ret= Q_TRAN(&Gfilter_waiting_fix);
                    }  
            }
            return ret;
        case NAV_FILTER_RESET_SIG: return Q_TRAN(&Gfilter_waiting_fix);
        case Q_EXIT_SIG: return Q_HANDLED();
        }
    return Q_SUPER(&QHsm_top);
}


double DegrToRad(double val)
{
    return((PI*val)/180.0);
}

gps_decart_t GpsToDec(GpsDataExtendedType *data)
{
    double phi, lam, h = 0, temp;
    double a = 6378137.00, b = 6356752.31;
    gps_decart_t tmp_dec;

    phi = DegrToRad(data->latitude);
    lam = DegrToRad(data->longitude);
    temp = a / (sqrt(1.0 - ((pow(a, 2) - pow(b, 2)) / pow(a, 2)) * pow(sin(phi), 2)));
    tmp_dec.x = (temp + h) * cos(phi) * cos(lam);  // x
    tmp_dec.y = (temp + h) * cos(phi) * sin(lam);  // y
    tmp_dec.z = (temp * (1 - ((pow(a, 2) - pow(b, 2)) / (pow(a, 2)))) + h) * sin(phi);  //z
    return tmp_dec;
}

int TimeValid(uint32_t time)
{
    if (time == 0xffffffff || time < 1356998400)return 0; // < 2013
    else return 1; 
}

double CalcDistance(gps_decart_t *first, gps_decart_t *second)
{
    return sqrt(pow(first->x - second->x, 2) + pow(first->y - second->y, 2) + pow(first->z - second->z, 2));
}

float DeltaCource(float first, float second)
{
    float dc = second - first;
    if (fabs(dc) > 270)
        {
        if (dc > 0)dc -= 360;
        else dc += 360;
        }
    return dc;
}

void SaveCurrentPoint( QEvt const * const e)
{
    DataPtrEvt *pe=(DataPtrEvt*)e;
    point.cur.gps=*(GpsDataExtendedType*)pe->ptr;
    // point.cur.gps.stored=0;
    point.cur.dec = GpsToDec(&point.cur.gps);
}





uint8_t debug_sensor_id;
uint8_t debug_sensor_id2;
uint8_t event_debug_card_id[RF10_CARD_ID_SIZE];
uint8_t not_event_debug_card_id[RF10_CARD_ID_SIZE];
QSensorType nav_dbg_sens[6];

int SendPointOrEvent(NAVFILTER * const me,GpsDataExtendedType* point_ptr)
{
    if (me->last_send_time==point_ptr->time) return 0;
    me->last_send_time=point_ptr->time;
    me->free_mess_ptr->gps.time=point_ptr->time;
    me->free_mess_ptr->gps.lat_int=(int32_t)(point_ptr->latitude*10000000.0);
    me->free_mess_ptr->gps.lon_int=(int32_t)(point_ptr->longitude*10000000.0);
    uint16_t temp=(uint16_t)point_ptr->speed;
    if (temp<255)me->free_mess_ptr->gps.speed= temp;
    else me->free_mess_ptr->gps.speed= 255;
    me->free_mess_ptr->gps.alt=(int16_t)point_ptr->altitude;
    me->free_mess_ptr->gps.angle=(int16_t)point_ptr->course;
    me->free_mess_ptr->gps.sg.sat_gsm.gsmq=(SysInfo.gsmq==99)?0:SysInfo.gsmq/2;
    me->free_mess_ptr->gps.sg.sat_gsm.sat_used=point_ptr->sat.used;
    me->free_mess_ptr->gps.accel=0;
    me->free_mess_ptr->gps.voltage.external=(uint16_t)(SysInfo.voltage.float_ext*100);
    me->free_mess_ptr->gps.voltage.internal=(uint16_t)(SysInfo.voltage.float_int*100);
//----------------------------------------------------------------------------------------------------
    debug_sensor_id=me->sensor.id;
    uint8_t sensorI=0;
   /* int its_event= (me->sensor.id&EVT_ID)?1:0;
    if (its_event)//it's event 
        {
        if (debug_sensor_id==0x95)
            {
            for (uint32_t i=0;i<RF10_CARD_ID_SIZE;i++)
                {
                event_debug_card_id[i]=me->sensor.data.any_data[i];;
                }
            }
        memcpy(&me->free_mess_ptr->sensors_array[sensorI],&me->sensor,sizeof(me->sensor));//copy event sensor to point
        me->sensor.id=0;//clear evt flag
        sensorI++;
        }*/
    for (uint32_t i=0;i<MAX_RS485_SENSORS_COUNT;i++)
        {
        if (conf_rs485_sensor[i].device==RS485_DEVICE_EPSILON)
            {
            // uint8_t id=RS485_SENSOR_GROUP_ID+i;
            // if(id== me->free_mess_ptr->sensors_array[0].id)continue;//this sensor whas been saved as event, skip and process next
            me->free_mess_ptr->sensors_array[sensorI].id=RS485_SENSOR_GROUP_ID+i;
            me->free_mess_ptr->sensors_array[sensorI].descriptor.data_size=3;
            me->free_mess_ptr->sensors_array[sensorI].descriptor.data_id=IO_FUEL_AND_TEMPERATURE_SENSOR;
            ((FuelSensorDataType*)&me->free_mess_ptr->sensors_array[sensorI].data.any_data)->temperature=((EpsilonDataType*)&IoData.rs485[i])->tdata;
            ((FuelSensorDataType*)&me->free_mess_ptr->sensors_array[sensorI].data.any_data)->fuel=((EpsilonDataType*)&IoData.rs485[i])->ndata;
            SysInfo.fuel_level_saved[sensorI]=((EpsilonDataType*)&IoData.rs485[i])->ndata;
            sensorI++;//inc sensors count
             if (sensorI==MAX_SENSORS_IN_MESSAGE)break;//end of free space in sensors array
            }
        } 
    if (sensorI<MAX_SENSORS_IN_MESSAGE)
        {
        for (uint32_t i=0;i<MAX_RS485_SENSORS_COUNT;i++)
            {
            if (conf_rs485_sensor[i].device==RS485_DEVICE_RF10)
                {
                /*if (its_event)
                    {
                    uint8_t sensor_id=debug_sensor_id2=(RS485_SENSOR_GROUP_ID+SysInfo.rf10_data.addr)|EVT_ID;
                    if (sensor_id== me->free_mess_ptr->sensors_array[0].id)
                        {
                        for (uint32_t i=0;i<RF10_CARD_ID_SIZE;i++)
                            {
                           	debug_card_id[i]=me->free_mess_ptr->sensors_array[0].data.any_data[i];
                            }
                        continue;//this sensor whas been saved as event, skip and process next
                        }
                    }*/
                me->free_mess_ptr->sensors_array[sensorI].id=RS485_SENSOR_GROUP_ID+i;
                me->free_mess_ptr->sensors_array[sensorI].descriptor.data_size=RF10_CARD_ID_SIZE;
                me->free_mess_ptr->sensors_array[sensorI].descriptor.data_id=IO_IDENTIFIER;
                memcpy(me->free_mess_ptr->sensors_array[sensorI].data.any_data,SysInfo.rf10_data.card_id,RF10_CARD_ID_SIZE);
                for (uint32_t k=0;k<RF10_CARD_ID_SIZE;k++)
                    {
                    not_event_debug_card_id[k]=me->free_mess_ptr->sensors_array[sensorI].data.any_data[k];;
                    }
                sensorI++;//inc sensors count
                if (sensorI==MAX_SENSORS_IN_MESSAGE)break;//end of free space in sensors array
                }
            } 
        }
				if (sensorI<MAX_SENSORS_IN_MESSAGE)
        {
        for (uint32_t i=0;i<MAX_RS485_SENSORS_COUNT;i++)
            {
            if (conf_rs485_sensor[i].device==RS485_DEVICE_TL10)
                {
                me->free_mess_ptr->sensors_array[sensorI].id=RS485_SENSOR_GROUP_ID+i;
                me->free_mess_ptr->sensors_array[sensorI].descriptor.data_size=TL10_DATA_SIZE;
                me->free_mess_ptr->sensors_array[sensorI].descriptor.data_id=IO_IDENTIFIER;
                memcpy(me->free_mess_ptr->sensors_array[sensorI].data.any_data,(uint8_t*)&SysInfo.tl10_data.id,TL10_DATA_SIZE);
                for (uint32_t k=0;k<TL10_DATA_SIZE;k++)
                    {
                    not_event_debug_card_id[k]=me->free_mess_ptr->sensors_array[sensorI].data.any_data[k];;
                    }
                sensorI++;//inc sensors count
                if (sensorI==MAX_SENSORS_IN_MESSAGE)break;//end of free space in sensors array
                }
            } 
        }
    if (sensorI<MAX_SENSORS_IN_MESSAGE)
        {
        if (conf.fin.mode[0]==IO_RPM)
            {
            me->free_mess_ptr->sensors_array[sensorI].id=FIN1;
            me->free_mess_ptr->sensors_array[sensorI].descriptor.data_id=IO_RPM;
            me->free_mess_ptr->sensors_array[sensorI].descriptor.data_size=2;
            *(uint16_t*)&me->free_mess_ptr->sensors_array[sensorI].data.any_data=SysInfo.counter[0].moment;
            sensorI++;
            }
				else if (conf.fin.mode[0]==IO_DIN)
				 {
					 me->free_mess_ptr->sensors_array[sensorI].id=DIN1;
            me->free_mess_ptr->sensors_array[sensorI].descriptor.data_id=IO_DIN;
            me->free_mess_ptr->sensors_array[sensorI].descriptor.data_size=1;
            *(uint8_t*)&me->free_mess_ptr->sensors_array[sensorI].data.any_data=SysInfo.flag.din1;
            sensorI++; 
				 }
        }
				
				
				if (sensorI<MAX_SENSORS_IN_MESSAGE)
        {
        if (conf.fin.mode[1]==IO_RPM)
            {
            me->free_mess_ptr->sensors_array[sensorI].id=FIN2;
            me->free_mess_ptr->sensors_array[sensorI].descriptor.data_id=IO_RPM;
            me->free_mess_ptr->sensors_array[sensorI].descriptor.data_size=2;
            *(uint16_t*)&me->free_mess_ptr->sensors_array[sensorI].data.any_data=SysInfo.counter[1].moment;
            sensorI++;
            }
				else if (conf.fin.mode[1]==IO_DIN)
				 {
					 me->free_mess_ptr->sensors_array[sensorI].id=DIN2;
            me->free_mess_ptr->sensors_array[sensorI].descriptor.data_id=IO_DIN;
            me->free_mess_ptr->sensors_array[sensorI].descriptor.data_size=1;
            *(uint8_t*)&me->free_mess_ptr->sensors_array[sensorI].data.any_data=SysInfo.flag.din2;
            sensorI++; 
				 }
        }
				
    /*	if(sensorI<MAX_SENSORS_IN_MESSAGE)
        {
            me->free_mess_ptr->sensors_array[sensorI].id=DIN1;
            me->free_mess_ptr->sensors_array[sensorI].descriptor.data_size=4;
            me->free_mess_ptr->sensors_array[sensorI].descriptor.data_id=IO_ACCUM_COUNT;
            *(uint32_t*)&me->free_mess_ptr->sensors_array[sensorI].data.any_data=conf.log.evt_counter.restart;
            sensorI++;
        }*/
				 for (uint32_t i=0;i<MAX_RS485_SENSORS_COUNT;i++)
				{
          memcpy(&nav_dbg_sens[i],&me->free_mess_ptr->sensors_array[i],sizeof(QSensorType));
				}
    me->free_mess_ptr->sensors_count=sensorI;
    DataPtrEvt *pe = Q_NEW(DataPtrEvt,DISK_POINT_SIG);
    pe->ptr=me->free_mess_ptr;
    pe->data.u32_data=point_ptr->prio;
    QACTIVE_POST(AO_Disk,&pe->super, me);
    if (me->free_mess_ptr== &points_pool[POINTS_POOL_SIZE-1])me->free_mess_ptr= &points_pool[0];
    else me->free_mess_ptr++;
    return 1;
}


/*	 void SendPointDebug(NAVFILTER * const me,GpsDataExtendedType* point_ptr)
   {
   SatGsmType sg={0};
   me->free_mess_ptr->gps.time=point_ptr->time;
   me->free_mess_ptr->gps.lat_int=(int32_t)(point_ptr->latitude*10000000.0);
   me->free_mess_ptr->gps.lon_int=(int32_t)(point_ptr->longitude*10000000.0);
   uint16_t temp=(uint16_t)point_ptr->speed;
   if (temp<255)me->free_mess_ptr->gps.speed= temp;
   else me->free_mess_ptr->gps.speed= 255;
   me->free_mess_ptr->gps.alt=(int16_t)point_ptr->altitude;
   me->free_mess_ptr->gps.angle=(int16_t)point_ptr->course;
   sg.sat_gsm.sat_used=point_ptr->sat_used;
   if (SysInfo.gsmq==99)sg.sat_gsm.gsmq=0;
   else sg.sat_gsm.gsmq=SysInfo.gsmq/2;
   me->free_mess_ptr->gps.sat_gsmq=sg.SatGsm;
   me->free_mess_ptr->gps.accel=0; 
   me->free_mess_ptr->gps.voltage=12;
//----------------------------------------------------------------------------------------------------
   uint8_t sensorI=0;
//----------------------------------------------------------------------------------------------------
   me->free_mess_ptr->sensors_count=sensorI;
   DataPtrEvt *pe = Q_NEW(DataPtrEvt,DISK_POINT_SIG);
   pe->ptr=me->free_mess_ptr;
   pe->data.u32_data=point_ptr->prio;
   QACTIVE_POST(AO_Disk,&pe->super, me);
   if (me->free_mess_ptr== &points_pool[POINTS_POOL_SIZE-1])me->free_mess_ptr= &points_pool[0];
   else me->free_mess_ptr++;
   }*/




uint32_t LSM303DLHC_TIMEOUT_UserCallback(void)
{
    return 0;
}


#ifdef DEBUG_NAV_FILTER		
void OutDebugNavFilter( char const *str)
{
    QS_BEGIN(QS_NAV_FILTER, AO_NavFilter)                                  
    QS_STR(str);        
    QS_END()
}


void OutDebugNavFilterSprintf( char const *str,uint32_t val)
{
    QS_BEGIN(QS_NAV_FILTER, AO_NavFilter)                                  
    QS_STR(str); 
    QS_U32(4, val);       
    QS_END()
}

#endif



