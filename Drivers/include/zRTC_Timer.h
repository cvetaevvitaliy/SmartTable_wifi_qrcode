#ifndef __ZRTC_TIMER_H
#define __ZRTC_TIMER_H
 
#include "stm32f10x.h"



typedef struct 
{
   uint16_t  year;
   uint8_t  month;
   uint8_t  day;
	
   uint8_t  hour;
   uint8_t  min;
   uint8_t  sec;                  
   uint8_t  week;             
}DateTime;

void RTC_Config(void);
uint8_t DateTime_Modify(DateTime * NewTime);
uint8_t DateTime_Now(DateTime * Timer);
uint8_t DateTime_GetWeek(uint16_t year,uint8_t month,uint8_t day);
void  DateTime_RTCConfig(uint32_t RTCSeconds);
uint32_t DateTime_TStrutoSec(DateTime * NewTime);
void DateTime_SectoTstru(uint32_t timecount, DateTime * Timer);
void DateTime_GMT2BeiJingTime(DateTime * GMTDateTime);

#endif


