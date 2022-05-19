
#include <stdint.h>
#include "my_time.h"


const uint8_t _ytab[2][12] = 
{
   { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31},
   { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}
};


//struct tm *mygmtime(register const time_t *timer)
struct tm *mygmtime(const time_t unix_time)
   {
   static struct tm br_time;
   register struct tm *timep = &br_time;
   time_t time = unix_time;
   register unsigned long dayclock, dayno;
   int year = EPOCH_YR;

   dayclock = (unsigned long)time % SECS_DAY;
   dayno = (unsigned long)time / SECS_DAY;
   timep->tm_sec = dayclock % 60;
   timep->tm_min = (dayclock % 3600) / 60;
   timep->tm_hour = dayclock / 3600;
   timep->tm_wday = (dayno + 4) % 7;       /* day 0 was a thursday */
   while (dayno >= YEARSIZE(year))
      {
      dayno -= YEARSIZE(year);
      year++;
      }
   timep->tm_year = year - YEAR0;
   timep->tm_yday = dayno;
   timep->tm_mon = 0;
   while (dayno >= _ytab[LEAPYEAR(year)][timep->tm_mon])
      {
      dayno -= _ytab[LEAPYEAR(year)][timep->tm_mon];
      timep->tm_mon++;
      }
   timep->tm_mday = dayno + 1;
   timep->tm_isdst = 0;
   return timep;
   }

