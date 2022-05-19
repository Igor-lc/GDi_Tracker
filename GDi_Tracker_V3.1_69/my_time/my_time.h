#ifndef _MY_TIME
#define _MY_TIME
#include <time.h>

#define 	YEAR0      1900
#define 	EPOCH_YR   1970
#define 	SECS_DAY   (24L * 60L * 60L)
#define 	LEAPYEAR(year)   (!((year) % 4) && (((year) % 100) || !((year) % 400)))
#define 	YEARSIZE(year)   (LEAPYEAR(year) ? 366 : 365)

extern struct tm *mygmtime(const time_t unix_time);

#endif



