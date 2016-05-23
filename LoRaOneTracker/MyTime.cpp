/*
 * Time.cpp
 *
 *  Created on: 3 Apr 2016
 *      Author: Willem Eradus
 */

#include "MyTime.h"

Time::Time() {
    // No DST settings disable it ...
    dst_ = false;
    year = 0;
    offset = 0;
}

Time::Time(int16_t utcoffset,const char *zone,uint8_t bhour,uint8_t bminute,uint8_t bmonth,uint8_t bweek, uint8_t bweekday,
        const char *dstzone,uint8_t ehour,uint8_t eminute,uint8_t emonth,uint8_t eweek, uint8_t eweekday) {
    dst_ = true;
    // Load rules ...
    offset = utcoffset / 60;
    year = 0;
    //
    rules[0].name = zone;         // Not used
    rules[0].hour = bhour;        // 2 am
    rules[0].minute = bminute;    // Not used
    rules[0].month = bmonth;      // March
    rules[0].week = bweek;        // Last
    rules[0].weekday = bweekday;  // Sunday
    // Rule DST end
    rules[1].name = dstzone;
    rules[1].hour = ehour;
    rules[1].minute = eminute;
    rules[1].month = emonth;
    rules[1].week = eweek;
    rules[1].weekday = eweekday;
}

uint32_t Time::days_from_civil(int32_t y, uint32_t m, uint32_t d) {
    y -= m <= 2;
    const int32_t  era = (y >= 0 ? y : y-399) / 400;
    const uint32_t yoe = static_cast<uint32_t>(y - era * 400);      // [0, 399]
    const uint32_t doy = (153*(m + (m > 2 ? -3 : 9)) + 2)/5 + d-1;  // [0, 365]
    const uint32_t doe = yoe * 365 + yoe/4 - yoe/100 + doy;         // [0, 146096]
    //
    return (era * DAYS_PER_ERA + static_cast<int32_t>(doe) - EPOCH_ADJUSTMENT_DAYS);
}

uint8_t Time::weekday_from_days(int32_t z) {
    return static_cast<uint8_t>(z >= -4 ? (z+4) % 7 : (z+5) % 7 + 6);
}

uint8_t Time::weekdaydifference(unsigned x, unsigned y) {
    x -= y;
    return x <= 6 ? x : x + 7;
}

bool Time::is_leap(int32_t y) {
    return  y % 4 == 0 && (y % 100 != 0 || y % 400 == 0);
}

uint8_t Time::last_day_of_month_common_year(unsigned m) {
    const uint8_t a[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    return a[m-1];
}

uint8_t Time::lastdayofmonth(int32_t y, unsigned m)  {
    return m != 2 || !is_leap(y) ? last_day_of_month_common_year(m) : 29u;
}

uint8_t Time::get_nth_dow_month_year(unsigned n, unsigned wd, unsigned month, int year) {
    const unsigned wd_1st = weekday_from_days(days_from_civil(year, month, 1));
    uint8_t day = weekdaydifference(wd, wd_1st) + 1 + (n-1)*7;
    if (day > lastdayofmonth(year,month)) day -= 7;
    return day;
}

uint32_t Time::get_dst_time(uint8_t dst_offset,uint8_t rule_week, uint8_t rule_weekday,uint8_t rule_month, uint8_t rule_hour, uint16_t current_year ) {
    int8_t utc_hour;    // [0,23]
    int    month_day;   // [1,31]
    // Calculate beginning or end of DST
    month_day = get_nth_dow_month_year(rule_week,rule_weekday, rule_month, current_year);
    utc_hour = rule_hour - offset - dst_offset;
    if (utc_hour < 0) {
        utc_hour = 24 + utc_hour;
        month_day--;
    }
    return Time::mktime(current_year, rule_month, month_day, utc_hour, 0, 0);
}

void Time::dstwindow(int cyear) {
    if (cyear != year && dst_ == true) {
        year = cyear;
        // Calculate beginning of DST
        xdst[0] = get_dst_time(0,rules[0].week,rules[0].weekday,rules[0].month,rules[0].hour,year);
        // Calculate end  of DST
        xdst[1] = get_dst_time(1,rules[1].week,rules[1].weekday,rules[1].month,rules[1].hour,year);
    }
}

uint32_t Time::dstfirst () {
    return xdst[0];
}

uint32_t Time::dstlast () {
    return xdst[1]-1;
}

uint32_t Time::mktime(int32_t year, uint32_t month, uint32_t day, uint8_t hour, uint8_t minute,uint8_t seconds) {
     year -= month <= 2;
     const int32_t  era = (year >= 0 ? year : year-399) / 400;
     const uint32_t yoe = static_cast<uint32_t>(year - era * 400);              // [0, 399]
     const uint32_t doy = (153*(month + (month > 2 ? -3 : 9)) + 2)/5 + day-1;   // [0, 365]
     const uint32_t doe = yoe * 365 + yoe/4 - yoe/100 + doy;                    // [0, 146096]
     //
     return (era * DAYS_PER_ERA + static_cast<int32_t>(doe) - EPOCH_ADJUSTMENT_DAYS) * SECSPERDAY + hour * 3600 + minute * 60 + seconds;
     }

void Time::localtime (const uint32_t utctime,struct tmx *t) {
     //
     uint32_t lcltime = utctime + offset*3600;
     if (dst_ == true && utctime >= xdst[0] && utctime < xdst[1]) {
         t->tm_isdst = true;
         lcltime += 3600;
     }
     else
         t->tm_isdst = false;
     //
     int days    = lcltime / SECSPERDAY + EPOCH_ADJUSTMENT_DAYS;
     int rem     = lcltime % SECSPERDAY;
     int weekday = ((ADJUSTED_EPOCH_WDAY + days) % DAYSPERWEEK);
     //
     int era     = (days >= 0 ? days : days - (DAYS_PER_ERA - 1)) / DAYS_PER_ERA;
     int eraday  = days - era * DAYS_PER_ERA;    /* [0, 146096] */
     int erayear = (eraday - eraday / (DAYS_PER_4_YEARS - 1) + eraday / DAYS_PER_CENTURY - eraday / (DAYS_PER_ERA - 1)) / 365;   /* [0, 399] */
     //
     int yearday = eraday - (DAYS_PER_YEAR * erayear + erayear / 4 - erayear / 100); /* [0, 365] */
     int month   = (5 * yearday + 2) / 153;
     int day     = yearday - (153 * month + 2) / 5 + 1;  /* [1, 31] */
     month  += month < 10 ? 2 : -10;  // [0, 11]
     //
     int year    = ADJUSTED_EPOCH_YEAR + erayear + era * YEARS_PER_ERA + (month <= 1);
     t->tm_yday  = yearday >= DAYS_PER_YEAR - DAYS_IN_JANUARY - DAYS_IN_FEBRUARY ? yearday - (DAYS_PER_YEAR - DAYS_IN_JANUARY - DAYS_IN_FEBRUARY) : yearday + DAYS_IN_JANUARY + DAYS_IN_FEBRUARY + is_leap(erayear);
     t->tm_year  = year - YEAR_BASE;
     t->tm_mon   = month;
     t->tm_mday  = day;
     t->tm_wday  = weekday;
     //
     t->tm_hour = (int) (rem / SECSPERHOUR);
     rem %= SECSPERHOUR;
     t->tm_min = (int) (rem / SECSPERMIN);
     t->tm_sec = (int) (rem % SECSPERMIN);
     //
     }

