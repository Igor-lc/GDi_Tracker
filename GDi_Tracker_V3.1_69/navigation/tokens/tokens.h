

#ifndef _TOKENS_H
#define _TOKENS_H

extern double nmeap_latitude(const char *plat,const char *phem);
extern double nmeap_longitude(const char *plon,const char *phem);
extern float nmeap_altitude(const char *palt,const char *punits);
extern unsigned int nmeap_utime(const char *stime,const char *sdate);
#endif


