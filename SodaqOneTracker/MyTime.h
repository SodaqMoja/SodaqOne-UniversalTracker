/*
 * MyTime.h
 *
 *  Created on: 3 Apr 2016
 *      Author: Willem Eradus
 */


#ifndef WETIME_H_
#define WETIME_H_

#include <stdint.h>

#define SECSPERMIN  60L
#define MINSPERHOUR 60L
#define HOURSPERDAY 24L
#define SECSPERHOUR (SECSPERMIN * MINSPERHOUR)
#define SECSPERDAY  (SECSPERHOUR * HOURSPERDAY)
#define DAYSPERWEEK 7
#define MONSPERYEAR 12

#define YEAR_BASE   1900
#define EPOCH_YEAR  1970
#define EPOCH_WDAY  4
#define EPOCH_YEARS_SINCE_LEAP 2
#define EPOCH_YEARS_SINCE_CENTURY 70
#define EPOCH_YEARS_SINCE_LEAP_CENTURY 370

#define EPOCH_ADJUSTMENT_DAYS   719468L
/* year to which the adjustment was made */
#define ADJUSTED_EPOCH_YEAR 0
/* 1st March of year 0 is Wednesday */
#define ADJUSTED_EPOCH_WDAY 3
/* there are 97 leap years in 400-year periods. ((400 - 97) * 365 + 97 * 366) */
#define DAYS_PER_ERA        146097L
/* there are 24 leap years in 100-year periods. ((100 - 24) * 365 + 24 * 366) */
#define DAYS_PER_CENTURY    36524L
/* there is one leap year every 4 years */
#define DAYS_PER_4_YEARS    (3 * 365 + 366)
/* number of days in a non-leap year */
#define DAYS_PER_YEAR       365
/* number of days in January */
#define DAYS_IN_JANUARY     31
/* number of days in non-leap February */
#define DAYS_IN_FEBRUARY    28
/* number of years per era */
#define YEARS_PER_ERA       400

enum WeekDays {Sun,Mon,Tue,Wed,Thu,Fri,Sat};
enum Months   {Jan = 1, Feb,Mar,Apr,May,Jun,Jul,Aug,Sep,Oct,Nov,Dec};
enum WeekOfMonth {First = 1, Second, Third, Fourth, Last = 5};

struct tmx
{
  int   tm_sec;
  int   tm_min;
  int   tm_hour;
  int   tm_mday;
  int   tm_mon;
  int   tm_year;
  int   tm_wday;
  int   tm_yday;
  int   tm_isdst;
};

struct rule {
    const char *name;
    uint8_t  week;
    uint8_t  weekday;
    uint8_t  month;
    uint8_t  hour;
    uint8_t  minute;
};

class Time {

public:
    Time();
    Time(int16_t utcoffset,const char *zone,uint8_t bhour,uint8_t bminute,uint8_t bmonth,uint8_t bweek, uint8_t bweekday,const char *dstzone,uint8_t ehour,uint8_t eminute,uint8_t emonth,uint8_t eweek, uint8_t eweekday);
    //
    void localtime (const uint32_t utctime,struct tmx *t) ;
    uint32_t mktime(int32_t y, uint32_t m, uint32_t d, uint8_t hour, uint8_t minute,uint8_t seconds);
    void dstwindow(int year);
    uint32_t dstfirst ();
    uint32_t dstlast ();
    //
private:
    uint32_t get_dst_time(uint8_t dst,uint8_t rweek, uint8_t rweekday,uint8_t rmonth, uint8_t rhour, uint16_t year );
    uint8_t  get_nth_dow_month_year(unsigned n, unsigned wd, unsigned month, int year);
    uint8_t  lastdayofmonth(int32_t y, unsigned m);
    uint8_t  last_day_of_month_common_year(unsigned m);
    bool is_leap(int32_t y);
    uint8_t  weekdaydifference(unsigned x, unsigned y);
    uint8_t  weekday_from_days(int32_t z);
    uint32_t days_from_civil(int32_t y, uint32_t m, uint32_t d);

    bool dst_;              // dst enabled
    uint32_t xdst[2];       // Holds begin and end of DST in UTC epoch
    int16_t  offset;        // UTC Offset in minutes
    int16_t  year;          // Year of current dst_ compute
    struct  rule rules[2];  // Rules for DST

};

#endif /* WETIME_H_ */
