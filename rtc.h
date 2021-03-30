#ifndef RTC_H_
#define RTC_H_

#include <time.h>

#define RTC_ADDRESS 0x56
#define BIT(x) 1<<x

void rtc_battery_charge();
void rtc_set_time(int year, int month, int day, int hour, int minute, int second);
void rtc_get_time(struct tm * tmTime, time_t * current_time);

#endif //RTC_H_