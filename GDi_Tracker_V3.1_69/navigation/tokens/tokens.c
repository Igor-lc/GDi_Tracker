
#include <assert.h>
#include <stdlib.h>
//#include <stdint.h>
#include <time.h>
#include <string.h>

double nmeap_latitude(const char *plat,const char *phem)
{
    double lat;
    int    deg;
    double min;
    int    ns;
    
    assert(plat != 0);
    assert(phem != 0);
    
    if (*plat == 0) {
        return 0.0;
    }
    if (*phem == 0) {
        return 0.0;
    }
    
    /* north lat is +, south lat is - */
    if (*phem == 'N') {
        ns = 1;
    }
    else {
        ns = -1;
    }
    
    /* latitude is degrees, minutes, fractional minutes */
    /* no validation is performed on the token. it better be good.*/
    /* if it comes back 0.0 then probably the token was bad */
    lat = atof(plat);
    
    /* extract the degree part */
    deg = (int)(lat / 100.0);
    
    /* mask out the degrees */
    min = lat - (deg * 100.0);
    
    /* compute the actual latitude in degrees.decimal-degrees */
    lat = (deg + (min / 60.0)) * ns;
    
    return lat;
}

double nmeap_longitude(const char *plon,const char *phem)
{
    double lon;
    int    deg;
    double min;
    int    ew;
    
    assert(plon != 0);
    assert(phem != 0);
    if (*plon == 0) return 0.0;
    if (*phem == 0) return 0.0;
    
    /* west long is negative, east long is positive */
    if (*phem == 'E') ew = 1;
    else ew = -1;
    /* longitude is degrees, minutes, fractional minutes */
    /* no validation is performed on the token. it better be good.*/
    /* if it comes back 0.0 then probably the token was bad */
    lon = atof(plon);
    
    /* extract the degree part */
    deg = (int)(lon / 100.0);
    
    /* mask out the degrees */
    min = lon - (deg * 100.0);
    
    /* compute the actual lonitude in degrees.decimal-degrees */
    lon = (deg + (min / 60.0)) * ew;
    return lon;
}

float nmeap_altitude(const char *palt,const char *punits)
{
    double alt;
    
    if (*palt == 0) {
        return 0.0;
    }
    
    /* convert with no error checking */
    alt = atof(palt);
    
    if (*punits == 'M') {/* already in meters, do nothing */ }
    else if (*punits == 'F') {
        /* convert to meters */
        alt = alt * 3.2808399;
    }
    
    return (float)alt;
}

unsigned int nmeap_utime(const char *stime,const char *sdate)
{
    unsigned int utime;
     struct tm time;
		 const char *str_ptr;
		 char temp_buf[10];
   
				str_ptr=stime;
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
			//----------------------------------------------
     str_ptr=sdate;
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
			time.tm_year=time.tm_year-1900;
      utime=mktime(&time);
    return utime;
}


