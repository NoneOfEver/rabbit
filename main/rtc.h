#ifndef MAIN_RTC_H_
#define MAIN_RTC_H_
#include <ds3231.h>
#include <stdio.h>
#include <string.h>
#include <ds3231.h>

void rtc_get_time(struct tm *time);
void rtc_set_time(uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute);
void my_rtc_init();
void ds3231_task(void *pvParameters);
#endif
