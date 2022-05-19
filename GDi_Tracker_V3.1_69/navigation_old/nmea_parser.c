

#include "stm32f10x.h"
#include "qp_port.h"
#include "nmea_parser.h"
#include "nav_filter.h"
#include "gps_hw.h"
#include <ctype.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "config.h"
#include "Disk.h"
#include "gps_ring_buf.h"

#define GPS_RX_BUF_SIZE             500
   
typedef struct
{
   QActive super;
	uint8_t nmea_counter; 
	uint16_t off_counter;
} NPAR;


//------------PRIVATE--------------------------------------------------------

static int motion_det_flag;
 GpsDataExtendedType *point_ptr;
 GpsDataExtendedType point1,point2;
static SatInfoType GpsSatInfo[MAXSATINVIEW];
static SatInfoType GlonassSatInfo[MAXSATINVIEW];
//char gps_buf_1[GPS_BUF_SIZE];
//char gps_buf_2[GPS_BUF_SIZE];
//char *current_gps_buf_ptr=gps_buf_1;
uint8_t gps_rx_buf[GPS_RX_BUF_SIZE]; 
static int axtoi(const char *hexStg);
static void GpsParser_hwinit (void);
static FieldEnumType NmeaParserGetField(const char *data_ptr, char *field_ptr, uint32_t field_num, int32_t max_field_len);
static int WriteMagneticVariation(const char *buf_ptr,const char** str_ptr,const unsigned int buf_size,double* magnetic_variation);
static int WriteMagneticVariationEW(const char *buf_ptr,const char** str_ptr,const unsigned int buf_size,char* magnetic_variation_EW);
static void ParseNMEASentence(NPAR * const me,const char *addressField, const char *buf,const unsigned int bufSize);
static void ProcessGPRMC(const char *buf_ptr, const unsigned int bufSize);
static void ProcessGPGGA(const char *buf_ptr, const unsigned int bufSize);
static void ProcessGNGSA(const char *buf_ptr, const unsigned int bufSize);
static void ProcessGPGSA(const char *buf_ptr, const unsigned int bufSize);
static void ProcessGNRMC(const char *buf_ptr, const unsigned int bufSize);
static void ProcessGPGSV(const char *pData);
static int ProcessGLGSV(const char *pData);
static void NmeaParser(NPAR * const me,const unsigned char ch);
static QState Gparser_initial(NPAR * const me, QEvt const * const e);
static QState Gparser_power_on_start(NPAR * const me, QEvt const * const e);
static QState Gparser_work(NPAR * const me, QEvt const * const e);
//static QState Gparser_off(NPAR * const me, QEvt const * const e);

static NPAR gparser; 
QActive * const AO_GpsParser = &gparser.super; 

void GpsParser_ctor(void)
{
   NPAR *me = &gparser;
   QActive_ctor(&me->super, Q_STATE_CAST(&Gparser_initial));
}

 QState Gparser_initial(NPAR * const me, QEvt const * const e)
{
	 QS_OBJ_DICTIONARY(&gparser);
   QS_FUN_DICTIONARY(&Gparser_initial);
   QS_FUN_DICTIONARY(&Gparser_work);
	 QS_FUN_DICTIONARY(&Gparser_power_on_start);
	// QS_FUN_DICTIONARY(&Gparser_off);
	// QS_FILTER_SM_OBJ(&gparser);
   QActive_subscribe(&me->super, TIC_100ms_SIG);
	me->nmea_counter=0;
	point_ptr=&point1;
	//memset(gps_buf_1,0,GPS_BUF_SIZE);
	//memset(gps_buf_2,0,GPS_BUF_SIZE);
	 GpsRingBuf_init(gps_rx_buf,GPS_RX_BUF_SIZE);
   return Q_TRAN(&Gparser_power_on_start);
}

QState Gparser_power_on_start(NPAR * const me, QEvt const * const e)
{
	static uint8_t reset_counter;
   switch (e->sig)
   {
      case Q_ENTRY_SIG:
			{
            GpsParser_hwinit();	
				   // GPS_NRESET_LOW();	
						GPS_POWER_ON();	
            reset_counter=0;				
			}
				return Q_HANDLED();
			case TIC_100ms_SIG:
			{
				if(++reset_counter==10)
				{
					//GPS_NRESET_HIGH();	
					return Q_TRAN(&Gparser_work);	
				}
			}
			 return Q_HANDLED();
			 case Q_EXIT_SIG: return Q_HANDLED();
   }
   return Q_SUPER(&QHsm_top);
}


int debug_index;
char debug_char;
QState Gparser_work(NPAR * const me, QEvt const * const e)
{
   switch (e->sig)
   {
      case Q_ENTRY_SIG: 
			{
				me->nmea_counter=0;
				me->off_counter=0;
			}
				return Q_HANDLED();
			case TIC_100ms_SIG:
			{
				 if(GpsRingBuf_count())
				 {
					 me->nmea_counter=0; 
				 }
				 while (GpsRingBuf_count())
                  {
										NmeaParser( me,ring_buf_pop_byte());
                  }
				if(++me->nmea_counter==50)
							{
								me->nmea_counter=0;
						   	SysInfo.nav_state  =no_nmea;
								conf.log.evt_counter.nmea_err++;
                conf.log.save=1;
								return Q_TRAN(&Gparser_power_on_start);	
							}			
			}
			 return Q_HANDLED();
     
			 case Q_EXIT_SIG: return Q_HANDLED();
   }
   return Q_SUPER(&QHsm_top);
}


/*QState Gparser_off(NPAR * const me, QEvt const * const e)
{
   switch (e->sig)
   {
      case Q_ENTRY_SIG: 
			{
				me->nmea_counter=0;
				me->off_counter=0;
				GPS_POWER_OFF();
			}
				return Q_HANDLED();
			case TIC_100ms_SIG:
			{
				if(++me->off_counter==300)
				{
				return Q_TRAN(&Gparser_power_on_start);	
				}
			}
			 return Q_HANDLED();
     
			 case Q_EXIT_SIG: return Q_HANDLED();
   }
   return Q_SUPER(&QHsm_top);
}*/



char debug_ch;
void NmeaParser(NPAR * const me,const unsigned char ch)
    {
    static enum
        {
        SearchForSOS ,
        RetrieveAddressField,
        ReceiveSentenceData,
        GetFirstChecksumCharacter,
        GetSecondChecksumCharacter,
        WaitForST
        } state=SearchForSOS;

    static unsigned int CalcChecksum;
    static char Checksum[3];
    static char NMEASequence[NMEA_SEQUENCE_MAX_LENGTH];
    static unsigned int SequenceIndex;
    static char AddressField[ADDRESS_FIELD_MAX_LENGTH];
    static unsigned int AddressFieldIndex;
				debug_ch=ch;

    switch (state)
        {
        case SearchForSOS:
            {
                if (ch == '$')
                    {
                    AddressFieldIndex = 0;
                    SequenceIndex = 0;
                    CalcChecksum = 0;
                    state = RetrieveAddressField;
                    }
                break;
            }

        case RetrieveAddressField:
            {
                if (SequenceIndex == NMEA_SEQUENCE_MAX_LENGTH - 1)state = SearchForSOS;
                else
                    {
                    NMEASequence[SequenceIndex++] = ch;
                    CalcChecksum ^= ch;
                    if (ch == ',')
                        {
                        AddressField[AddressFieldIndex] = '\0';
													if (NULL!=(strstr((const char*)AddressField,"GPRMC")))
													{
														OutDebugNmeaParser("GPRMC finded");
													}
                        state = ReceiveSentenceData;
                        }
                    else if (AddressFieldIndex == ADDRESS_FIELD_MAX_LENGTH - 1 )
											     {
											      OutDebugNmeaParser("ADDRESS_FIELD_MAX_LENGTH ERROR");
									          state = SearchForSOS;
								           }
										/*else if (!isalpha(ch))
											     {
											      OutDebugNmeaParser("ADDRESS_FIELD isalpha ERROR");
														 AddressField[AddressFieldIndex++] = ch;
														 AddressField[AddressFieldIndex] = '\0';
														 OutDebugNmeaParser(AddressField);
									          state = SearchForSOS;
								           }*/
										else if (islower(ch))
											     {
											      OutDebugNmeaParser("ADDRESS_FIELD islower ERROR");
														 AddressField[AddressFieldIndex++] = ch;
														 AddressField[AddressFieldIndex] = '\0';
														 OutDebugNmeaParser(AddressField);
									          state = SearchForSOS;
								           }
                         else AddressField[AddressFieldIndex++] = ch;
                    }
                break;
            }
        case ReceiveSentenceData:
            {
                if (SequenceIndex == NMEA_SEQUENCE_MAX_LENGTH - 1)
							    	{
											OutDebugNmeaParser("NMEA_SEQUENCE_MAX_LENGTH ERROR");
									   state = SearchForSOS;
								     }
                else
                    {
                    NMEASequence[SequenceIndex++] = ch;
                    if (ch == '*')state = GetFirstChecksumCharacter;
                    else if (ch == '\n'||ch == '\r')
                        {
                         OutDebugNmeaParser("ReceiveSentenceData::UNEXPECTED END OF MESSAGE");
                        state = SearchForSOS;
                        }
                    else
                        CalcChecksum ^= ch;
                    }
                break;
            }

        case GetFirstChecksumCharacter:
            {
                if (SequenceIndex == NMEA_SEQUENCE_MAX_LENGTH - 1 || (!isdigit(ch) && (ch < 'A' || ch > 'F')))
                     {
                      OutDebugNmeaParser("GetFirstChecksumCharacter::ERROR");
                       state = SearchForSOS;
                     }
                else
                    {
                    NMEASequence[SequenceIndex++] = ch;
                    Checksum[0] = ch;
                    state = GetSecondChecksumCharacter;
                    }
                break;
            }

        case GetSecondChecksumCharacter:
            {
                if (SequenceIndex == NMEA_SEQUENCE_MAX_LENGTH - 1 || (!isdigit(ch) && (ch < 'A' || ch > 'F')))
									   {
                       OutDebugNmeaParser("GetSecondChecksumCharacter::char ERROR");
                       state = SearchForSOS;
                     }
                else
                    {
                    NMEASequence[SequenceIndex++] = ch;
                    Checksum[1] = ch;
                    Checksum[2] = '\0';
                    unsigned int iChecksum = axtoi(Checksum);
                    if (iChecksum == CalcChecksum)state = WaitForST;
                    else 
											{
                       OutDebugNmeaParser("GetSecondChecksumCharacter::crc ERROR");
                       state = SearchForSOS;
                     }
                    }
                break;
            }

        case WaitForST:
            {
                if (SequenceIndex == NMEA_SEQUENCE_MAX_LENGTH - 1 || (ch!= '\n' && ch != '\r'))state = SearchForSOS;
                else if (ch == '\r')
                    {
                    NMEASequence[SequenceIndex++] = ch;
                    NMEASequence[SequenceIndex] = '\0';
                    ParseNMEASentence(me,AddressField, NMEASequence,SequenceIndex);
                    state = SearchForSOS;
                    }
                break;
            }
        default:  state = SearchForSOS;
            break;
        }
    }
		
			
		
void ParseNMEASentence(NPAR * const me,const char *addressField, const char *buf,const unsigned int bufSize)
{
	__disable_irq();
	if (strcmp(addressField, "GPRMC") == 0)
        {
					 OutDebugNmeaParser("ProcessGPRMC");
					 ProcessGPRMC(buf, bufSize);
        }
     else if (strcmp(addressField, "GNRMC") == 0)
        {
        ProcessGNRMC(buf, bufSize);
        }
     else if (strcmp(addressField, "GPGGA") == 0)
        {
        ProcessGPGGA(buf, bufSize);
        }
     else if (strcmp(addressField, "GPGSA") == 0)
        {
        ProcessGPGSA(buf, bufSize);
        }
     else if (strcmp(addressField, "GNGSA") == 0)
        {
        ProcessGNGSA(buf, bufSize);
        }
     else if (strcmp(addressField, "GPGSV") == 0)
        {
        ProcessGPGSV(buf);
        }
     else if (strcmp(addressField, "GLGSV") == 0)
        {
        ProcessGLGSV(buf);
        }
     else if (strcmp(addressField, "GPGLL") == 0 || strcmp(addressField, "GNGLL") == 0)
		         {		
               uint8_t MaxSig=0,i;							 
	          	 for (i=0;i<MAXSATINVIEW;i++)
                   {
                    if (MaxSig<GpsSatInfo[i].signal_quality)MaxSig=GpsSatInfo[i].signal_quality;
                   }
               for (i=0;i<MAXSATINVIEW;i++)
                   {
                    if (MaxSig<GlonassSatInfo[i].signal_quality)MaxSig=GlonassSatInfo[i].signal_quality;
                   }
					     SysInfo.NavSignalAndSat=100*MaxSig+point_ptr->sat.used; 
#ifdef DEBUG_NMEA_PARSER
								static uint8_t debug_counter=0;
								if(++debug_counter==10)
								  {
								   debug_counter=0;
								   OutDebugGpsParserSprintf("GPS SAT IN VIEW=",point_ptr->sat.view_glo+point_ptr->sat.view_gps);
	                }
#endif      
						point_ptr->move=motion_det_flag;						
						DataPtrEvt *pe = Q_NEW(DataPtrEvt,NAV_FILTER_NAV_DATA_SIG);
						pe->ptr=point_ptr;
					  QACTIVE_POST(AO_NavFilter, &pe->super, me);
						if(point_ptr==&point1)point_ptr=&point2;
						else point_ptr=&point1;
            memset((uint8_t*)point_ptr,0,sizeof(GpsDataExtendedType));
            memset((uint8_t*)&GpsSatInfo,0,sizeof(SatInfoType)*MAXSATINVIEW);
						memset((uint8_t*)&GlonassSatInfo,0,sizeof(SatInfoType)*MAXSATINVIEW);    
	         }
					 __enable_irq();
}

	GprmcFieldsEnum dbg_field;

char field_buf[MAXFIELD];
void ProcessGPRMC(const char *buf_ptr, const unsigned int bufSize)
    {
    char temp_buf[10];
		char print_buf[50];
    struct tm time;
     FieldEnumType result;
		
    const char *str_ptr = buf_ptr, *comma_ptr;

    if ((unsigned int)(str_ptr - buf_ptr) >= bufSize)return;
    if (bufSize < 6)return;
			dbg_field=gprmc_time_field;
		 result=NmeaParserGetField(buf_ptr, field_buf, gprmc_time_field, MAXFIELD);
   if(result== ok_field)
	 {
	 OutDebugNmeaParser(field_buf);
	 str_ptr=field_buf;
	 strncpy(temp_buf, str_ptr, 2);
    temp_buf[2] = '\0';
    time.tm_hour = atoi(temp_buf);
	  str_ptr += 2;
    strncpy(temp_buf, str_ptr, 2);
    temp_buf[2] = '\0';
    time.tm_min = atoi(temp_buf);
    str_ptr += 2;
    strncpy(temp_buf, str_ptr, 2);
    temp_buf[2] = '\0';
    time.tm_sec = atoi(temp_buf);
	 }
	 else  OutDebugNmeaParser("GPRMC time_field ERROR");
  //status 'A' or 'V'
	 dbg_field=gprmc_status_field;
    result=NmeaParserGetField(buf_ptr, field_buf, gprmc_status_field, MAXFIELD); 
     if(result==ok_field)
		 {
   if(*field_buf == 'A')
		{
			 OutDebugNmeaParser("GPRMC DATA VALID");
			point_ptr->status.gps_fix =1; 
		}	
		 else
		{
			 OutDebugNmeaParser("GPRMC DATA NOT VALID");
			point_ptr->status.gps_fix =0;
		}
	}
else  OutDebugNmeaParser("GPRMC status_field ERROR");	
// Latitude
	dbg_field=gprmc_lat_field;
    result=NmeaParserGetField(buf_ptr, field_buf, gprmc_lat_field, MAXFIELD); 
		if(result==ok_field)
		{
	 if ((comma_ptr = strchr(field_buf, '.')) == NULL )
	 {
		  OutDebugNmeaParser("GPRMC gprmc_lat_field comma_ptr ERROR");
	 }
	 else
	 {
		  OutDebugNmeaParser(field_buf);
		 point_ptr->latitude = atof(comma_ptr - 2) / 60.0;
    field_buf[comma_ptr - 2 - field_buf] = '\0';
    point_ptr->latitude += atof(field_buf);
		 #ifdef DEBUG_NMEA_PARSER
		 sprintf(print_buf,"GPRMC lat=%4.8f",point_ptr->latitude);
		 #endif
		 OutDebugNmeaParser(print_buf);
	 }
 }
 else  OutDebugNmeaParser("GPRMC lat_field ERROR");
 //N/S
 dbg_field=gprmc_ns_field;
    result=NmeaParserGetField(buf_ptr, field_buf, gprmc_ns_field, MAXFIELD);
 if(result==ok_field)
		 {
			 if (*field_buf == 'S')point_ptr->latitude = -point_ptr->latitude; 
		 }
  else OutDebugNmeaParser("GPRMC gprmc_ns_field ERROR");  
// Longitude
		 dbg_field=gprmc_lon_field;
   result=NmeaParserGetField(buf_ptr, field_buf, gprmc_lon_field, MAXFIELD); 
		if(result==ok_field)
		{
	 if ((comma_ptr = strchr(field_buf, '.')) == NULL )
	 {
		  OutDebugNmeaParser("GPRMC gprmc_lon_field comma_ptr ERROR");
	 }
	 else
	 {
		  OutDebugNmeaParser(field_buf);
		 point_ptr->longitude = atof(comma_ptr - 2) / 60.0;
    field_buf[comma_ptr - 2 - field_buf] = '\0';
    point_ptr->longitude += atof(field_buf);
		 #ifdef DEBUG_NMEA_PARSER
		 sprintf(print_buf,"GPRMC lon=%4.8f",point_ptr->longitude);
		 #endif
		 OutDebugNmeaParser(print_buf);
	 }
 }
 else   OutDebugNmeaParser("GPRMC lon_field ERROR");


// Ground speed
 dbg_field=gprmc_speed_field;
 result=NmeaParserGetField(buf_ptr, field_buf, gprmc_speed_field, MAXFIELD); 
 if(result==ok_field)
		 {
			 float tmp_speed=atof(field_buf);
      point_ptr->speed = 1.85*tmp_speed;
			 #ifdef DEBUG_NMEA_PARSER
		  sprintf(print_buf,"GPRMC speed=%4.2f",point_ptr->speed);
			 #endif
		  OutDebugNmeaParser(print_buf);
		 }
	else OutDebugNmeaParser("GPRMC gprmc_speed_field ERROR");
// Course over ground (degrees)
		 dbg_field=gprmc_course_field;
 result=NmeaParserGetField(buf_ptr, field_buf, gprmc_course_field, MAXFIELD); 
 if(result==ok_field)
		 {
      point_ptr->course = atof(field_buf);
			 #ifdef DEBUG_NMEA_PARSER
		  sprintf(print_buf,"GPRMC course=%4.2f",point_ptr->course);
			 #endif
		  OutDebugNmeaParser(print_buf);
		 }
 else OutDebugNmeaParser("GPRMC gprmc_course_field ERROR");
// Date
		 dbg_field=gprmc_date_field;
		 result=NmeaParserGetField(buf_ptr, field_buf, gprmc_date_field, MAXFIELD); 
		 if(result==ok_field)
		 {
			str_ptr=field_buf;
      strncpy(temp_buf, str_ptr, 2);
      temp_buf[2] = '\0';
			time.tm_mday = atoi(temp_buf);
      str_ptr += 2;
      strncpy(temp_buf, str_ptr, 2);
      temp_buf[2] = '\0';
			time.tm_mon = atoi(temp_buf)-1;
      str_ptr += 2;
      strncpy(temp_buf, str_ptr, 2);
      temp_buf[2] = '\0';
      time.tm_year = atoi(temp_buf);
			 if(time.tm_year==80)time.tm_year+=1900;
			 else time.tm_year+=2000;
		 }
   else OutDebugNmeaParser("GPRMC gprmc_date_field ERROR");
//-------------------------------------------------------------------------		 
    time.tm_year=time.tm_year-1900;
    point_ptr->time=mktime(&time);
    sprintf(print_buf,"GPRMC time=%u",point_ptr->time);
		  OutDebugNmeaParser(print_buf);		 
   /* if (point_ptr->status.data_valid==1)
        {
					OutDebugNmeaParser("GPRMC data_valid==1");
        point_ptr->time=mktime(&time);
        }
			else OutDebugNmeaParser("GPRMC data_valid==0");*/
     
    }
		
		
		void ProcessGPGSA(const char *buf_ptr, const unsigned int bufSize)
    {
    const char *str_ptr, *comma_ptr;
    char temp_buf[12]; 
    str_ptr=buf_ptr;
    //OUTDEBUG_NMEA_PARSER("ProcessGPGSA\r\n");
    if ((unsigned int)(str_ptr - buf_ptr) >= bufSize)return;
    if (bufSize < 6)return;
    strncpy(temp_buf, buf_ptr, 5);
    temp_buf[5] = '\0';
    if (strcmp(temp_buf, "GPGSA") != 0 || buf_ptr[5] != ',')return;
    //$GPGSA,A,1,,,,,,,,,,,,,,,*1E
    else str_ptr += 6;// first char of MODE1 field
//------------------MODE1-----------------------------------
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    str_ptr=comma_ptr+1;  //set str_ptr to first char MODE2 field
//------------------MODE2---------------------------------------------------
    if ((unsigned int)(str_ptr - buf_ptr) >= bufSize)return;
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    strncpy(temp_buf, str_ptr, comma_ptr - str_ptr);
    temp_buf[comma_ptr - str_ptr] = '\0';

    point_ptr->pos_mode = atoi(temp_buf);
    str_ptr = comma_ptr + 1;//set str_ptr to first char SAT USED CH1  field
//------------------SAT USED CH1-------------------------------------
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    str_ptr=comma_ptr+1;  //set str_ptr to first char SAT USED CH2 field
//------------------SAT USED CH2--------------------------------------------------
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    str_ptr=comma_ptr+1;  //set str_ptr to first char SAT USED CH3 field
//------------------SAT USED CH3--------------------------------------------
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    str_ptr=comma_ptr+1;  //set str_ptr to first char SAT USED CH4  field
//-------------------SAT USED CH4---------------------------------------------
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    str_ptr=comma_ptr+1;  //set str_ptr to first char SAT USED CH5 field
//------------------SAT USED CH5--------------------------------------------------
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    str_ptr=comma_ptr+1;  //set str_ptr to first char SAT USED CH6 field
//------------------SAT USED CH6--------------------------------------------
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    str_ptr=comma_ptr+1;  //set str_ptr to first char SAT USED CH7  field
//-------------------SAT USED CH7---------------------------------------------
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    str_ptr=comma_ptr+1;  //set str_ptr to first char SAT USED CH8 field
    //---------------SAT USED CH8--------------------------------------------------
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    str_ptr=comma_ptr+1;  //set str_ptr to first char SAT USED CH9 field
//------------------SAT USED CH9--------------------------------------------
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    str_ptr=comma_ptr+1;  //set str_ptr to first char SAT USED CH10  field
//-------------------SAT USED CH10---------------------------------------------
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    str_ptr=comma_ptr+1;  //set str_ptr to first char SAT USED CH11 field
//------------------SAT USED CH11--------------------------------------------------
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    str_ptr=comma_ptr+1;  //set str_ptr to first char SAT USED CH12 field
//------------------SAT USED CH12--------------------------------------------
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    str_ptr=comma_ptr+1;  //set str_ptr to first char PDOP  field
//---------------------PDOP----------------------------------------
    // OUTDEBUG_NMEA_PARSER("ProcessGPGSA PDOP\r\n");
    if ((unsigned int)(str_ptr - buf_ptr) >= bufSize)return;
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    strncpy(temp_buf, str_ptr, comma_ptr - str_ptr);
    temp_buf[comma_ptr - str_ptr] = '\0';
    point_ptr->pdop = atof(temp_buf);
    str_ptr = comma_ptr + 1;//set str_ptr to first char HDOP  field
//----------------------HDOP--------------------------------------
    // OUTDEBUG_NMEA_PARSER("ProcessGPGSA HDOP\r\n");
    if ((unsigned int)(str_ptr - buf_ptr) >= bufSize)return;
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    strncpy(temp_buf, str_ptr, comma_ptr - str_ptr);
    temp_buf[comma_ptr - str_ptr] = '\0';
    point_ptr->hdop = atof(temp_buf);
    str_ptr = comma_ptr + 1;//set str_ptr to first char VDOP  field
//----------------------VDOP------------------------------------
    // OUTDEBUG_NMEA_PARSER("ProcessGPGSA VDOP\r\n");
    if ((unsigned int)(str_ptr - buf_ptr) >= bufSize)return;
    if ((comma_ptr = strchr(str_ptr, '.')) == NULL)return;
    strncpy(temp_buf, str_ptr, (comma_ptr+3) - str_ptr);
    temp_buf[(comma_ptr+3) - str_ptr] = '\0';

    point_ptr->vdop = atof(temp_buf);
    }

		
		

void ProcessGNRMC(const char *buf_ptr, const unsigned int bufSize)
    {
    char temp_buf[10];
    struct tm time;
    //time_t unix_time=0;
    const char *str_ptr = buf_ptr, *comma_ptr;
    if ((unsigned int)(str_ptr - buf_ptr) >= bufSize)
		{
//			OUTDEBUG_NMEA_PARSER("\x1b[31mGNRMC RETURN 1\r\n");
			return;
		}
    if (bufSize < 6)
			{
//			OUTDEBUG_NMEA_PARSER("\x1b[31mGNRMC RETURN 2\r\n");
			return;
		}
    strncpy(temp_buf, buf_ptr, 5);
    temp_buf[5] = '\0';
    if (strcmp(temp_buf, "GNRMC") != 0 || buf_ptr[5] != ',')
			{
		//	OUTDEBUG_NMEA_PARSER("\x1b[31mGNRMC RETURN 3\r\n");
			return;
		 }
    else str_ptr += 6;

// Time
    if ((unsigned int)(str_ptr - buf_ptr) >= bufSize)
			{
		//	OUTDEBUG_NMEA_PARSER("\x1b[31mGNRMC RETURN 4\r\n");
			return;
		}
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)
			{
		//	OUTDEBUG_NMEA_PARSER("\x1b[31mGNRMC RETURN 5\r\n");
			return;
		}
    strncpy(temp_buf, str_ptr, 2);
    temp_buf[2] = '\0';
    time.tm_hour = atoi(temp_buf);
    str_ptr += 2;
    strncpy(temp_buf, str_ptr, 2);
    temp_buf[2] = '\0';
    time.tm_min = atoi(temp_buf);
    str_ptr += 2;
    strncpy(temp_buf, str_ptr, 2);
    temp_buf[2] = '\0';
    time.tm_sec = atoi(temp_buf);
    str_ptr = comma_ptr + 1;
    if ((unsigned int)(str_ptr - buf_ptr) >= bufSize)
			{
			//OUTDEBUG_NMEA_PARSER("\x1b[31mGNRMC RETURN 6\r\n");
			return;
		}
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)
			{
			//OUTDEBUG_NMEA_PARSER("\x1b[31mGNRMC RETURN 7\r\n");
			return;
		}
    if (comma_ptr == str_ptr)
			{
			//OUTDEBUG_NMEA_PARSER("\x1b[31mGNRMC RETURN 8\r\n");
			return;
		}
   // if (*str_ptr != 'A')point_ptr->status.data_valid =0;
   // else point_ptr->status.data_valid =1;    
    str_ptr = comma_ptr + 1;  
    point_ptr->status.gps_fix =  (*str_ptr == 'A');		
// Latitude
    if ((unsigned int)(str_ptr - buf_ptr) >= bufSize)
			{
//			OUTDEBUG_NMEA_PARSER("\x1b[31mGNRMC RETURN 9\r\n");
			return;
		}
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)
			{
			//OUTDEBUG_NMEA_PARSER("\x1b[31mGNRMC RETURN 10\r\n");
			return;
		}
    strncpy(temp_buf, str_ptr, comma_ptr - str_ptr);
    temp_buf[comma_ptr - str_ptr] = '\0';
    str_ptr = comma_ptr + 1;
    if ((comma_ptr = strchr(temp_buf, '.')) == NULL)
			{
		//	OUTDEBUG_NMEA_PARSER("\x1b[31mGNRMC RETURN 11\r\n");
			return;
		}
    if (comma_ptr-temp_buf < 2)
			{
//			OUTDEBUG_NMEA_PARSER("\x1b[31mGNRMC RETURN 12\r\n");
			return;
		}

    point_ptr->latitude = atof(comma_ptr - 2) / 60.0;
    temp_buf[comma_ptr - 2 - temp_buf] = '\0';
    point_ptr->latitude += atof(temp_buf);
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)
			{
//			OUTDEBUG_NMEA_PARSER("\x1b[31mGNRMC RETURN 13\r\n");
			return;
		}
    if (comma_ptr - str_ptr != 1)
			{
//			OUTDEBUG_NMEA_PARSER("\x1b[31mGNRMC RETURN 14\r\n");
			return;
		}
    if (*str_ptr == 'S')point_ptr->latitude = -point_ptr->latitude;
    else if (*str_ptr != 'N')
			{
//			OUTDEBUG_NMEA_PARSER("\x1b[31mGNRMC RETURN 15\r\n");
			return;
		}
    str_ptr = comma_ptr + 1;    
// Longitude
    if ((unsigned int)(str_ptr - buf_ptr) >= bufSize)
			{
//			OUTDEBUG_NMEA_PARSER("\x1b[31mGNRMC RETURN 16\r\n");
			return;
		}
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)
			{
			//OUTDEBUG_NMEA_PARSER("\x1b[31mGNRMC RETURN 17\r\n");
			return;
		}
    strncpy(temp_buf, str_ptr, comma_ptr - str_ptr);
    temp_buf[comma_ptr - str_ptr] = '\0';
    str_ptr = comma_ptr + 1;

    if ((comma_ptr = strchr(temp_buf, '.')) == NULL)
			{
			//OUTDEBUG_NMEA_PARSER("\x1b[31mGNRMC RETURN 18\r\n");
			return;
		}
    point_ptr->longitude = atof(comma_ptr - 2) / 60.0;
    temp_buf[comma_ptr - 2 - temp_buf] = '\0';
    point_ptr->longitude += atof(temp_buf);
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)
			{
//			OUTDEBUG_NMEA_PARSER("\x1b[31mGNRMC RETURN 19\r\n");
			return;
		}
    if (comma_ptr - str_ptr != 1)
			{
		//	OUTDEBUG_NMEA_PARSER("\x1b[31mGNRMC RETURN 20\r\n");
			return;
		}
    if (*str_ptr == 'W')point_ptr->longitude = -point_ptr->longitude;
    else if (*str_ptr != 'E')
			{
		//	OUTDEBUG_NMEA_PARSER("\x1b[31mGNRMC RETURN 21\r\n");
			return;
		}
    str_ptr = comma_ptr + 1;

// Ground speed
    if ((unsigned int)(str_ptr - buf_ptr) >= bufSize)
			{
//			OUTDEBUG_NMEA_PARSER("\x1b[31mGNRMC RETURN 22\r\n");
			return;
		}
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)
			{
		//	OUTDEBUG_NMEA_PARSER("\x1b[31mGNRMC RETURN 23\r\n");
			return;
		}
    strncpy(temp_buf, str_ptr, comma_ptr - str_ptr);
    temp_buf[comma_ptr - str_ptr] = '\0';
    point_ptr->speed = 1.85*atof(temp_buf);
    str_ptr = comma_ptr + 1;

// Course over ground (degrees)
    if ((unsigned int)(str_ptr - buf_ptr) >= bufSize)
			{
		//	OUTDEBUG_NMEA_PARSER("\x1b[31mGNRMC RETURN 24\r\n");
			return;
		}
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)
			{
		//	OUTDEBUG_NMEA_PARSER("\x1b[31mGNRMC RETURN 25\r\n");
			return;
		}
    strncpy(temp_buf, str_ptr, comma_ptr - str_ptr);
    temp_buf[comma_ptr - str_ptr] = '\0';
    point_ptr->course = atof(temp_buf);
    str_ptr = comma_ptr + 1; //first char of date
// Date
    if ((unsigned int)(str_ptr - buf_ptr) >= bufSize)
			{
		//	OUTDEBUG_NMEA_PARSER("\x1b[31mGNRMC RETURN 26\r\n");
			return;
		}
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)
			{
		//	OUTDEBUG_NMEA_PARSER("\x1b[31mGNRMC RETURN 27\r\n");
			return;
		}
    strncpy(temp_buf, str_ptr, 2);
    temp_buf[2] = '\0';

    time.tm_mday = atoi(temp_buf);
    str_ptr += 2;
    strncpy(temp_buf, str_ptr, 2);
    temp_buf[2] = '\0';
    //strcpy(new_gps_data_ptr->month_str,temp_buf);
    //sprintf(temp_print_buf,"MONTH=%s\r\n", temp_buf);
    //  OUTDEBUG_NMEA_PARSER(temp_print_buf);
    time.tm_mon = atoi(temp_buf)-1;
    str_ptr += 2;
    strncpy(temp_buf, str_ptr, 2);
    temp_buf[2] = '\0';
    time.tm_year = 2000 + atoi(temp_buf);
//new_gps_data_ptr->year = atoi(temp_buf);
    //   strcpy(new_gps_data_ptr->year_str,temp_buf);
    str_ptr = comma_ptr + 1; //set str_ptr to first char of magnetic variation field
    double temp_mvar;
    if (0==WriteMagneticVariation(buf_ptr,&str_ptr,bufSize,&temp_mvar))
        {
        // OUTDEBUG_NMEA_PARSER("GPRMC MAGNETIC VARIATION ERROR: EXIT\r\n");
//			OUTDEBUG_NMEA_PARSER("\x1b[31mGNRMC RETURN 28\r\n");
			return;
        }
    char temp;
//if(FALSE==WriteMagneticVariationEW(buf_ptr,&str_ptr,bufSize,&new_gps_data_ptr->magnetic_variation_EW))
    if (0==WriteMagneticVariationEW(buf_ptr,&str_ptr,bufSize,&temp))
        {
        //OUTDEBUG_NMEA_PARSER("GPRMC MAGNETIC VARIATION EW ERROR: EXIT\r\n");
			//OUTDEBUG_NMEA_PARSER("\x1b[31mGNRMC RETURN 29\r\n");
			return;
        }

    //if (0==WriteMode(buf_ptr,str_ptr,bufSize,&new_gps_data_ptr->mode))
    // {
    //OUTDEBUG_NMEA_PARSER("GPRMC MODE field ERROR: EXIT\r\n");
    // return;
    // }
    if (*(str_ptr+1)!='*')
        {
        //OUTDEBUG_NMEA_PARSER("GPRMS END OF MESSAGE (*) NOT FINDED-ERROR EXIT\r\n");
			//OUTDEBUG_NMEA_PARSER("\x1b[31mGNRMC RETURN 30\r\n");
			return;
        }

    time.tm_year=time.tm_year-1900;   
    if (point_ptr->status.gps_fix==1)
        {
			  //OUTDEBUG_NMEA_PARSER("GNRMC data_valid==1\r\n");
        point_ptr->time=mktime(&time);
        }
//  else OUTDEBUG_NMEA_PARSER("\x1b[31mGNRMC data_valid==0\r\n");
    }


		
		void ProcessGPGGA(const char *buf_ptr, const unsigned int bufSize)
    {
    const char *str_ptr, *comma_ptr;
    char temp_buf[10]; 
    str_ptr=buf_ptr;

    if ((unsigned int)(str_ptr - buf_ptr) >= bufSize)return;
    if (bufSize < 6)return;
    strncpy(temp_buf, buf_ptr, 5);
    temp_buf[5] = '\0';
    if (strcmp(temp_buf, "GPGGA") != 0 || buf_ptr[5] != ',')return;
    else str_ptr += 6;// first char of Time field
//------------------TIME----------------------------------------------------
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    str_ptr=comma_ptr+1;  //set str_ptr to first char LATITUDE field
//----------------LATITUDE---------------------------------------------------
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    str_ptr=comma_ptr+1;  //set str_ptr to first char N/S field
//---------------------------N/S-------------------------------------
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    str_ptr=comma_ptr+1;  //set str_ptr to char LONGITUDE field
//------------------LONGITUDE---------------------------------------------------
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    str_ptr=comma_ptr+1;  //set str_ptr to first char E/W field
//-----------------E/W---------------------------------------------
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    str_ptr=comma_ptr+1;  //set str_ptr to char "GPS QUALITY" field
//-----------------GPS QUALITY---------------------------------------------
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    str_ptr=comma_ptr+1;  //set str_ptr to char "SAT USED" field
//---------------------SAT USED----------------------------------------
    if ((unsigned int)(str_ptr - buf_ptr) >= bufSize)return;
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    strncpy(temp_buf, str_ptr, comma_ptr - str_ptr);
    temp_buf[comma_ptr - str_ptr] = '\0';
    point_ptr->sat.used = atoi(temp_buf);
    str_ptr=comma_ptr+1;  //set str_ptr to first char  HDOP field
//--------------------------------HDOP-----------------------------
    if ((unsigned int)(str_ptr - buf_ptr) >= bufSize)return;
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    // strncpy(temp_buf, str_ptr, comma_ptr - str_ptr);
    // temp_buf[comma_ptr - str_ptr] = '\0';
    // new_gps_data_ptr->hdop = atof(temp_buf);
    str_ptr = comma_ptr + 1;//set str_ptr to first char ALTITUDE field
//-----------------------------ALTITUDE---------------------------
    if ((unsigned int)(str_ptr - buf_ptr) >= bufSize)return;
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    strncpy(temp_buf, str_ptr, comma_ptr - str_ptr);
    temp_buf[comma_ptr - str_ptr] = '\0';
//   strcpy(new_gps_data_ptr->alt_str,temp_buf);
    point_ptr->altitude = atof(temp_buf);
    }
			
		
		void ProcessGNGSA(const char *buf_ptr, const unsigned int bufSize)
    {
    const char *str_ptr, *comma_ptr;
    char temp_buf[12]; 
    str_ptr=buf_ptr;
    //OUTDEBUG_NMEA_PARSER("ProcessGNGSA\r\n");
    if ((unsigned int)(str_ptr - buf_ptr) >= bufSize)return;
    if (bufSize < 6)return;
    strncpy(temp_buf, buf_ptr, 5);
    temp_buf[5] = '\0';
    if (strcmp(temp_buf, "GNGSA") != 0 || buf_ptr[5] != ',')return;
    //$GPGSA,A,1,,,,,,,,,,,,,,,*1E
    else str_ptr += 6;// first char of MODE1 field
//------------------MODE1-----------------------------------
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    str_ptr=comma_ptr+1;  //set str_ptr to first char MODE2 field
//------------------MODE2---------------------------------------------------
    if ((unsigned int)(str_ptr - buf_ptr) >= bufSize)return;
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    strncpy(temp_buf, str_ptr, comma_ptr - str_ptr);
    temp_buf[comma_ptr - str_ptr] = '\0';

    point_ptr->pos_mode = atoi(temp_buf);
    str_ptr = comma_ptr + 1;//set str_ptr to first char SAT USED CH1  field
//------------------SAT USED CH1-------------------------------------
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    str_ptr=comma_ptr+1;  //set str_ptr to first char SAT USED CH2 field
//------------------SAT USED CH2--------------------------------------------------
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    str_ptr=comma_ptr+1;  //set str_ptr to first char SAT USED CH3 field
//------------------SAT USED CH3--------------------------------------------
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    str_ptr=comma_ptr+1;  //set str_ptr to first char SAT USED CH4  field
//-------------------SAT USED CH4---------------------------------------------
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    str_ptr=comma_ptr+1;  //set str_ptr to first char SAT USED CH5 field
//------------------SAT USED CH5--------------------------------------------------
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    str_ptr=comma_ptr+1;  //set str_ptr to first char SAT USED CH6 field
//------------------SAT USED CH6--------------------------------------------
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    str_ptr=comma_ptr+1;  //set str_ptr to first char SAT USED CH7  field
//-------------------SAT USED CH7---------------------------------------------
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    str_ptr=comma_ptr+1;  //set str_ptr to first char SAT USED CH8 field
    //---------------SAT USED CH8--------------------------------------------------
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    str_ptr=comma_ptr+1;  //set str_ptr to first char SAT USED CH9 field
//------------------SAT USED CH9--------------------------------------------
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    str_ptr=comma_ptr+1;  //set str_ptr to first char SAT USED CH10  field
//-------------------SAT USED CH10---------------------------------------------
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    str_ptr=comma_ptr+1;  //set str_ptr to first char SAT USED CH11 field
//------------------SAT USED CH11--------------------------------------------------
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    str_ptr=comma_ptr+1;  //set str_ptr to first char SAT USED CH12 field
//------------------SAT USED CH12--------------------------------------------
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    str_ptr=comma_ptr+1;  //set str_ptr to first char PDOP  field
//---------------------PDOP----------------------------------------
    // OUTDEBUG_NMEA_PARSER("ProcessGNGSA PDOP\r\n");
    if ((unsigned int)(str_ptr - buf_ptr) >= bufSize)return;
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    strncpy(temp_buf, str_ptr, comma_ptr - str_ptr);
    temp_buf[comma_ptr - str_ptr] = '\0';
    point_ptr->pdop = atof(temp_buf);
    str_ptr = comma_ptr + 1;//set str_ptr to first char HDOP  field
//----------------------HDOP--------------------------------------
    // OUTDEBUG_NMEA_PARSER("ProcessGNGSA HDOP\r\n");
    if ((unsigned int)(str_ptr - buf_ptr) >= bufSize)return;
    if ((comma_ptr = strchr(str_ptr, ',')) == NULL)return;
    strncpy(temp_buf, str_ptr, comma_ptr - str_ptr);
    temp_buf[comma_ptr - str_ptr] = '\0';
    point_ptr->hdop = atof(temp_buf);
    str_ptr = comma_ptr + 1;//set str_ptr to first char VDOP  field
//----------------------VDOP------------------------------------
    // OUTDEBUG_NMEA_PARSER("ProcessGNGSA VDOP\r\n");
    if ((unsigned int)(str_ptr - buf_ptr) >= bufSize)return;
    if ((comma_ptr = strchr(str_ptr, '.')) == NULL)return;
    strncpy(temp_buf, str_ptr, (comma_ptr+3) - str_ptr);
    temp_buf[(comma_ptr+3) - str_ptr] = '\0';
    point_ptr->vdop = atof(temp_buf);
    }
		
	
		void ProcessGPGSV(const char *pData)
    {
    uint8_t TotalNumOfMsg, MsgNum;
    char field_buf[MAXFIELD];
    FieldEnumType result;
    result=NmeaParserGetField(pData, field_buf, 1, MAXFIELD);
    if (ok_field==result)// Total number of messages
        {
        TotalNumOfMsg = atoi(field_buf);
        if (TotalNumOfMsg > 4 ) return;
        }
    else return;
    result=NmeaParserGetField(pData, field_buf, 2, MAXFIELD);
    if (ok_field==result)// message number
        {
        MsgNum = atoi(field_buf);
        if (MsgNum > 4) return;
        }
    else return;
    result=NmeaParserGetField(pData, field_buf, 3, MAXFIELD);
    if (ok_field==result)// Total satellites in view
        {
        point_ptr->sat.view_gps = atoi(field_buf);
        if (point_ptr->sat.view_gps==0)return;
        }
    //--------------------PRN-----------------------------------
    for (int i = 0; i < 4; i++)
        {
        switch (NmeaParserGetField(pData, field_buf, 4 + 4*i, MAXFIELD))
            {
            case ok_field:
                {
                    GpsSatInfo[i+(MsgNum-1)*4].prn = atoi(field_buf);
                }
                break;
            case error_field:
            case end_field: return;
            default:
                {
                    GpsSatInfo[i+(MsgNum-1)*4].prn = 0; 
                }
                break;
            }
//---------------------ELEVATION----------------------------------------------------------				 
        switch (NmeaParserGetField(pData, field_buf, 5 + 4*i, MAXFIELD))
            {
            case ok_field:
                {
                    GpsSatInfo[i+(MsgNum-1)*4].elevation = atoi(field_buf);
                }
                break;
            case error_field:
            case end_field: return;
            default:
                {
                    GpsSatInfo[i+(MsgNum-1)*4].elevation = 0; 
                }
                break;
            }
//-------------------------AZIMUTH-----------------------------------------------------------				 
        switch (NmeaParserGetField(pData, field_buf, 6 + 4*i, MAXFIELD))
            {
            case ok_field:
                {
                    GpsSatInfo[i+(MsgNum-1)*4].azimuth = atoi(field_buf);
                }
                break;
            case error_field:
            case end_field: return;
            default:
                {
                    GpsSatInfo[i+(MsgNum-1)*4].azimuth = 0; 
                }
                break;
            }
//-------------------------SIGNAL QUALITY-----------------------------------------------------------				 
        switch (NmeaParserGetField(pData, field_buf, 7 + 4*i, MAXFIELD))
            {
            case ok_field:
                {
                    GpsSatInfo[i+(MsgNum-1)*4].signal_quality = atoi(field_buf);
                }
                break;
            case error_field:
            case end_field: return;
            default:
                {
                    GpsSatInfo[i+(MsgNum-1)*4].signal_quality = 0; 
                }
                break;
            }
        }
    }
			
		
		
		int ProcessGLGSV(const char *pData)
    {
  static  uint8_t TotalNumOfMsg, MsgNum;
			int all_mess_processed=0;
    char field_buf[MAXFIELD];
    FieldEnumType result;
    result=NmeaParserGetField(pData, field_buf, 1, MAXFIELD);
    if (ok_field!=result)return 0;// Total number of messages
     TotalNumOfMsg = atoi(field_buf);
    result=NmeaParserGetField(pData, field_buf, 2, MAXFIELD);
    if (ok_field!=result)return 0;// message number
    MsgNum = atoi(field_buf); 
	if(TotalNumOfMsg==MsgNum)all_mess_processed=1;	
    result=NmeaParserGetField(pData, field_buf, 3, MAXFIELD);
    if (ok_field==result)// Total satellites in view
        {
        point_ptr->sat.view_glo  = atoi(field_buf);
        if (point_ptr->sat.view_glo==0)return 0;
        }
    //--------------------PRN-----------------------------------
    for (int i = 0; i < 4; i++)
        {
        switch (NmeaParserGetField(pData, field_buf, 4 + 4*i, MAXFIELD))
            {
            case ok_field: GlonassSatInfo[i+(MsgNum-1)*4].prn = atoi(field_buf);
                break;
            case end_field: return all_mess_processed;
						case error_field: return 0;		
            default: GlonassSatInfo[i+(MsgNum-1)*4].prn = 0; 
                break;
            }
//---------------------ELEVATION----------------------------------------------------------				 
        switch (NmeaParserGetField(pData, field_buf, 5 + 4*i, MAXFIELD))
            {
            case ok_field:
                {
                    GlonassSatInfo[i+(MsgNum-1)*4].elevation = atoi(field_buf);
                }
                break;
            case end_field: return all_mess_processed;
						 case error_field: return 0;	
            default:
                {
                    GlonassSatInfo[i+(MsgNum-1)*4].elevation = 0; 
                }
                break;
            }
//-------------------------AZIMUTH-----------------------------------------------------------				 
        switch (NmeaParserGetField(pData, field_buf, 6 + 4*i, MAXFIELD))//elevation
            {
            case ok_field:
                {
                    GlonassSatInfo[i+(MsgNum-1)*4].azimuth = atoi(field_buf);
                }
                break;
            case end_field: return all_mess_processed;
						 case error_field: return 0;	
            default:
                {
                    GlonassSatInfo[i+(MsgNum-1)*4].azimuth = 0; 
                }
                break;
            }
//-------------------------SIGNAL QUALITY-----------------------------------------------------------				 
        switch (NmeaParserGetField(pData, field_buf, 7 + 4*i, MAXFIELD))//elevation
            {
            case ok_field:
                {
                    GlonassSatInfo[i+(MsgNum-1)*4].signal_quality = atoi(field_buf);
                }
                break;
            case end_field: return all_mess_processed;
						 case error_field: return 0;	
            default:
                {
                    GlonassSatInfo[i+(MsgNum-1)*4].signal_quality = 0; 
                }
                break;
            }
        }
				return all_mess_processed;
    }

		
		
	/*	void GpsParser_hwinit (void)
{
   NVIC_InitTypeDef NVIC_InitStructure;
   GPIO_InitTypeDef GPIO_InitStructure;
   USART_InitTypeDef USART_InitStructure;
   DMA_InitTypeDef DMA_InitStructure;


   GPIO_InitStructure.GPIO_Pin = GPS_POWER_PIN;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPS_POWER_GPIO_PORT, &GPIO_InitStructure);

   GPS_POWER_OFF();
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Pin = GPS_UART_RX_Pin;
    GPIO_Init(GPS_UART_GPIO , &GPIO_InitStructure);
	
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Pin = GPS_UART_TX_Pin;
    GPIO_Init(GPS_UART_GPIO , &GPIO_InitStructure);

 
   USART_InitStructure.USART_BaudRate = 9600;
   USART_InitStructure.USART_WordLength = USART_WordLength_8b;
   USART_InitStructure.USART_StopBits = USART_StopBits_1;
   USART_InitStructure.USART_Parity = USART_Parity_No;
   USART_InitStructure.USART_HardwareFlowControl= USART_HardwareFlowControl_None;
   USART_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
   USART_Init(GPS_UART, &USART_InitStructure);
   USART_Cmd(GPS_UART,ENABLE);

   DMA_DeInit(DMA2_Channel3);
   DMA_InitStructure.DMA_PeripheralBaseAddr = (unsigned int) &(GPS_UART->DR);
   DMA_InitStructure.DMA_MemoryBaseAddr = (u32)current_gps_buf_ptr;
   DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
   DMA_InitStructure.DMA_BufferSize = GPS_BUF_SIZE;
   DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
   DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
   DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
   DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
   DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
   DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
   DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
   DMA_Init(DMA2_Channel3, &DMA_InitStructure);
   USART_ClearFlag(GPS_UART, USART_FLAG_RXNE | USART_FLAG_TC | USART_FLAG_TXE);
   USART_DMACmd(GPS_UART,USART_DMAReq_Rx, ENABLE);
   DMA_Cmd(DMA2_Channel3, ENABLE);

   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
   NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel3_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = GPS_UART_IRQ_PRIO;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure); 
   DMA_ITConfig(DMA2_Channel3, DMA_IT_TC, ENABLE);
}*/

/*void  DMA2_Channel3_IRQHandler(void)
{
	static uint8_t Gps_IRQHandler;
	static DataPtrEvt nav_buf_evt = {NAV_BUF_SIG, 0U, 0U };
 QK_ISR_ENTRY(); 
   DMA2_Channel3->CCR &= CCR_ENABLE_Reset; //turn  DMA ch off
   nav_buf_evt.ptr=current_gps_buf_ptr; 
   QACTIVE_POST(AO_GpsParser, &nav_buf_evt.super,&Gps_IRQHandler);
   if (current_gps_buf_ptr==gps_buf_1)current_gps_buf_ptr=gps_buf_2;
   else current_gps_buf_ptr=gps_buf_1;
   DMA2_Channel3->CMAR =(u32)current_gps_buf_ptr; 
   DMA2_Channel3->CNDTR = GPS_BUF_SIZE;
   DMA2_Channel3->CCR |= CCR_ENABLE_Set; //turn  DMA ch  on
   DMA_ClearITPendingBit(DMA2_IT_TC3); 
 QK_ISR_EXIT(); 
}*/

void GpsParser_hwinit (void)
{
	  NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

   GPIO_InitStructure.GPIO_Pin = GPS_POWER_PIN;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPS_POWER_GPIO_PORT, &GPIO_InitStructure);

   GPS_POWER_OFF();
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Pin = GPS_UART_RX_Pin;
   GPIO_Init(GPS_UART_GPIO , &GPIO_InitStructure);
	
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_Pin = GPS_UART_TX_Pin;
   GPIO_Init(GPS_UART_GPIO , &GPIO_InitStructure);

     
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl= USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx;
    USART_Init(GPS_UART, &USART_InitStructure);
    USART_Cmd(GPS_UART,ENABLE);
		
		 NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
   NVIC_Init(&NVIC_InitStructure); 
   USART_ITConfig(GPS_UART, USART_IT_RXNE, ENABLE);  
}

	void UART4_IRQHandler (void)
{
   USART_ClearITPendingBit(GPS_UART, USART_IT_RXNE);
   uint8_t data=GPS_UART->DR;
   GpsRingBuf_push(data);  
}

int axtoi(const char *hexStg)
{
   int n = 0; // position in string
   int m = 0; // position in digit[] to shift
   int count; // loop index
   int intValue = 0; // integer value of hex string
   int digit[5]; // hold values to convert
   while (n < 4)
      {
      if (hexStg[n] == '\0')
         break;
      if (hexStg[n] > 0x29 && hexStg[n] < 0x40) //if 0 to 9
         digit[n] = hexStg[n] & 0x0f; //convert to int
      else if (hexStg[n] >= 'a' && hexStg[n] <= 'f') //if a to f
         digit[n] = (hexStg[n] & 0x0f) + 9; //convert to int
      else if (hexStg[n] >= 'A' && hexStg[n] <= 'F') //if A to F
         digit[n] = (hexStg[n] & 0x0f) + 9; //convert to int
      else
         break;
      n++;
      }
   count = n;
   m = n - 1;
   n = 0;
   while (n < count)
      {
      intValue = intValue | (digit[n] << (m << 2));
      m--; // adjust the position to set
      n++; // next digit to process
      }
   return(intValue);
}


int WriteMagneticVariation(const char *buf_ptr,const char** str_ptr,const unsigned int buf_size,double* magnetic_variation)
    {
    char* comma_ptr,temp_buf[10];
    if ((unsigned int)(*str_ptr - buf_ptr) >= buf_size)
        {
        //OUTDEBUG_NMEA_PARSER("WriteMagneticVariation ?????? ??????? ?? ??????? ??????-ERROR EXIT\r\n");
        return(0);
        }
    if ((comma_ptr = strchr(*str_ptr, ',')) == NULL)
        {
        //OUTDEBUG_NMEA_PARSER("WriteMagneticVariation NO NEXT FIELD Magnetic variation-ERROR EXIT\r\n");
        return(0);
        }
    if (**str_ptr == ',')
        {
        //OUTDEBUG_NMEA_PARSER("GPRMS Magnetic variation FIELD is empty\r\n");
        //sprintf(print_buf,"GPRMS Magnetic variation str_ptr=%s\r\n",*str_ptr);
        //OUTDEBUG_NMEA_PARSER(print_buf);
        (*str_ptr)++;
        return(1);
        }
    strncpy(temp_buf, *str_ptr, comma_ptr - *str_ptr);//copy field to temp buf
    temp_buf[comma_ptr - *str_ptr] = '\0';//terminate string
    *str_ptr = comma_ptr + 1;//set str_ptr to first char of field
    *magnetic_variation = atof(temp_buf);
    return(1);
    }
		
		
		int WriteMagneticVariationEW(const char *buf_ptr,const char** str_ptr,const unsigned int buf_size,char* magnetic_variation_EW)
    {
    if ((unsigned int)(*str_ptr - buf_ptr) >= buf_size)
        {
        // OUTDEBUG_NMEA_PARSER("GPRMS ?????? ??????? ?? ??????? ??????-ERROR EXIT\r\n");
        return(0);
        }
    if (strchr(*str_ptr, ',') == NULL)
        {
        // OUTDEBUG_NMEA_PARSER("GPRMS NO NEXT FIELD Magnetic variationEW-ERROR EXIT\r\n");
        return(0);
        }
    if (**str_ptr == ',')
        {
        //OUTDEBUG_NMEA_PARSER("GPRMS Magnetic variationEW FIELD is empty\r\n");
        //sprintf(print_buf,"GPRMS Magnetic variationEW str_ptr=%s\r\n",*str_ptr);
        //OUTDEBUG_NMEA_PARSER(print_buf);
        (*str_ptr)++;
        return(1);
        }
    (*str_ptr)++;
    if (**str_ptr == 'E')*magnetic_variation_EW=**str_ptr;
    else if (**str_ptr == 'W')*magnetic_variation_EW=**str_ptr;
    else
        {
        //OUTDEBUG_NMEA_PARSER("GPRMS Magnetic variationEW FIELD unknown value\r\n");
        }
    (*str_ptr)++;
    return(1);
    }
		
		
		
		FieldEnumType NmeaParserGetField(const char *data_ptr, char *field_ptr, uint32_t field_num, int32_t max_field_len)
    {
    if (data_ptr == NULL || field_ptr == NULL || max_field_len <= 0)//error param return 
        {
        return error_field;
        }

    // Go to the beginning of the selected field
    uint32_t i = 0;
    uint32_t field = 0;
    while (field != field_num && data_ptr[i])
        {
        if (data_ptr[i] == ',')field++;//next field
        i++;
        if (data_ptr[i] =='\0')//unexpected end of string
            {
            field_ptr=NULL;
            return end_field;
            }
        }

    if (data_ptr[i] == ',')//target field is comma
        {
        field_ptr[0] = '\0';
        return comma_field;
        }
    if (data_ptr[i] == '*')//target field is end of message
        {
        field_ptr[0] = '\0';
        return end_field;
        }
    // copy field from data_ptr to field_ptr
    int i2 = 0;
    while (data_ptr[i] != ',' && data_ptr[i] != '*' && data_ptr[i])
        {
        field_ptr[i2] = data_ptr[i];
        i2++; i++;
        // check if field is too big to fit on passed parameter. If it is,
        // crop returned field to its max length.
        if (i2 >= max_field_len)//field len error
            {
            i2 = max_field_len-1;
            return error_field;
            }
        }
    field_ptr[i2] = '\0';
    return ok_field;
    }
	
#ifdef DEBUG_NMEA_PARSER		
	void OutDebugNmeaParser( char const *str)
   {
   QS_BEGIN(QS_NAV_RECEIVER, AO_GpsParser)                                  
   QS_STR(str);   		 
   QS_END()
   }
	 
		
void OutDebugGpsParserSprintf( char const *str,uint32_t val)
   {
   QS_BEGIN(QS_NAV_RECEIVER, AO_GpsParser)                                  
   QS_STR(str); 
   QS_U32(4, val);  		 
   QS_END()
   }

#endif





