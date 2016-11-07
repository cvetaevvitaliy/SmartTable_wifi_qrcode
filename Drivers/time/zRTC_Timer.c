#include <stdio.h>
#include "stm32f10x.h"
#include "stm32f10x_rtc.h"
#include "stm32f10x_bkp.h"
#include "stm32f10x_pwr.h"
#include "zRTC_Timer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Sys_Init.h"


#ifndef RTC_32K
	#define RTC_32K 0
#endif
#define RTCConfigedFlagAddr BKP_DR1
//初始化rtc
#if RTC_32K 
void RTC_Config(void)
{
		u16 WaitForOscSource;
	      /* Enable PWR and BKP clocks */
      RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
      /* Allow access to BKP Domain */
      PWR_BackupAccessCmd(ENABLE); 
  
  RCC_LSEConfig(RCC_LSE_ON);  
	for(WaitForOscSource=0;WaitForOscSource<500;WaitForOscSource++);
  while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)  
  {}  
  
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);  
  
  RCC_RTCCLKCmd(ENABLE);  
  
  RTC_WaitForSynchro();  
  
  RTC_WaitForLastTask();   
   
  
  RTC_WaitForLastTask();  
  
  RTC_SetPrescaler(32767); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */  
  
  RTC_WaitForLastTask();   
	
}
#else


void RTC_Config(void)
{
	u16 i;
	 //重新配置RTC
	u16 WaitForOscSource;
	/* Enable PWR and BKP clocks */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	/* Allow access to BKP Domain */
	PWR_BackupAccessCmd(ENABLE);
	/* Reset Backup Domain */
	//BKP_DeInit();
	/* Enable LSE */
	RCC_LSEConfig( RCC_LSE_ON);
	for(WaitForOscSource=0;WaitForOscSource<5000;WaitForOscSource++);
	for(i=0;i<1000;i++)
	{
		for(WaitForOscSource=0;WaitForOscSource<50000;WaitForOscSource++);
		if(RCC_GetFlagStatus(RCC_FLAG_LSERDY) != RESET)
			break;
	}
	if(i==1000)
	{
		RCC_LSEConfig(RCC_LSE_Bypass);	// 
		for(WaitForOscSource=0;WaitForOscSource<500;WaitForOscSource++);
		RCC_RTCCLKConfig(RCC_RTCCLKSource_HSE_Div128); 
		printf("i is %d",i);
	}
	else
	{
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
	}
    /* Wait till LSE is ready */
    // while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);
     /* Select LSE as RTC Clock Source */
      /* Enable RTC Clock */
      RCC_RTCCLKCmd(ENABLE);
      /* Wait for RTC registers synchronization */
      RTC_WaitForSynchro();
      /* Wait until last write operation on RTC registers has finished */
      RTC_WaitForLastTask();
      /* Enable the RTC Second */
      //RTC_ITConfig(RTC_IT_SEC, ENABLE);
      /* Wait until last write operation on RTC registers has finished */
     // RTC_WaitForLastTask();
      /* Set RTC prescaler: set RTC period to 1sec */
		if(i==1000)
		{
			RTC_SetPrescaler(62499); //(32767); /* RTC period = RTCCLK/RTC_PR = (8M/128)/(62499+1) */
			 
		}
		else
		{
			RTC_SetPrescaler(32767); // 
			 
		}
      /* Wait until last write operation on RTC registers has finished */
      RTC_WaitForLastTask();
      //配置完成后，向后备寄存器中写特殊字符0x5A5A     
	
}
#endif

//配置rtc时间
void DateTime_RTCConfig(uint32_t RTCSeconds)
{
	DateTime NewTime;
     //我们在BKP的后备寄存器1中，存了一个特殊字符0xA5A5
    //第一次上电或后备电源掉电后，该寄存器数据丢失，
    //表明RTC数据丢失，需要重新配置

   if(BKP_ReadBackupRegister(RTCConfigedFlagAddr) != 0x5A5A)
   {
		if((RTCSeconds !=0)&&(RTCSeconds != 0xffffffff))
		{
			DateTime_SectoTstru(RTCSeconds+1, &NewTime); //有记录从记录时间里提取最新时间，保证新纪录时间比老记录大
		}
		else
		{
		  NewTime.year  = 2015;
		  NewTime.month = 1;
		  NewTime.day   = 1;  
		  NewTime.hour  = 0;
		  NewTime.min   = 0;
		  NewTime.sec   = 0;                 
		  NewTime.week  = 0;
		}
		RTC_Config();
		DateTime_Modify(&NewTime);//默认时间
		BKP_WriteBackupRegister(RTCConfigedFlagAddr, 0x5A5A);
    }
   else
   {
	   RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	   /* Allow access to BKP Domain */
	   PWR_BackupAccessCmd(ENABLE); 		 
	   // 		 if (RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET) 
	   // 		 {
	   // 		 }
	   // 		 else if (RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET)
	   // 		 {}
	   RCC_RTCCLKCmd(ENABLE);
	   RTC_WaitForSynchro();  
	   RTC_WaitForLastTask();
   }
   RCC_ClearFlag();
}
//判断是否是闰年函数

//月份   1  2  3  4  5  6  7  8  9  10 11 12

//闰年   31 29 31 30 31 30 31 31 30 31 30 31

//非闰年 31 28 31 30 31 30 31 31 30 31 30 31

//输入:年份

//输出:该年份是不是闰年.1,是.0,不是

uint8_t Is_Leap_Year(uint16_t year)
{                     
   if(year%4==0) //必须能被4整除
   { 
		  if(year%100==0) 
		  { 
				 if(year%400==0)return 1;//如果以00结尾,还要能被400整除          
				 else return 0;   
		  }else return 1;   
   }else return 0; 
}                           

//设置时钟

//把输入的时钟转换为秒钟

//以1970年1月1日为基准

//1970~2099年为合法年份

//返回值:0,成功;其他:错误代码.

//月份数据表                                                                        
const uint8_t table_week[12]={0,3,3,6,1,4,6,2,5,0,3,5}; //月修正数据表   
//平年的月份日期表
const uint8_t mon_table[12]={31,28,31,30,31,30,31,31,30,31,30,31};

uint8_t DateTime_Modify(DateTime * NewTime)
{
//	uint16_t t;
	uint32_t seccount=0;
	if(NewTime->year<2014||NewTime->year>2099)return 1;//syear范围1970-2099，此处设置范围为2014-2099        
	seccount =  DateTime_TStrutoSec(NewTime);
	 taskENTER_CRITICAL(); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
	PWR_BackupAccessCmd(ENABLE);
	RTC_WaitForLastTask();
	RTC_SetCounter(seccount);
	RTC_WaitForLastTask();
	 taskEXIT_CRITICAL(); 

	return 0;      

}

//得到当前的时间

//返回值:0,成功;其他:错误代码.

uint8_t DateTime_Now(DateTime * Timer)
{
	//static uint16 daycnt=0;
	uint32_t timecount=0;  
	  taskENTER_CRITICAL(); 
	timecount=RTC_GetCounter();
	 taskEXIT_CRITICAL(); 
	DateTime_SectoTstru(timecount,Timer);		 
//	DateTime_GMT2BeiJingTime(Timer);
	return 0;
} 


//将时间转化为秒

uint32_t DateTime_TStrutoSec(DateTime * NewTime)
{
	uint8_t month;
	uint16_t t;
	uint32_t seccount=0;
	month =  NewTime->month;
	//if(NewTime->year<2014||NewTime->year>2099)return 1;//syear范围1970-2099，此处设置范围为2014-2099        
	for(t=1970;t<NewTime->year;t++) //把所有年份的秒钟相加
	{
	  if(Is_Leap_Year(t))seccount+=31622400;//闰年的秒钟数
	  else seccount+=31536000;                    //平年的秒钟数
	}
	month-=1;
	for(t=0;t<month;t++)         //把前面月份的秒钟数相加
	{
	  seccount+=(uint32_t)mon_table[t]*86400;//月份秒钟数相加
	  if(Is_Leap_Year(NewTime->year) && (t==1))seccount+=86400;//闰年2月份增加一天的秒钟数         
	}
	seccount+=(uint32_t)(NewTime->day-1)*86400;//把前面日期的秒钟数相加 
	seccount+=(uint32_t)NewTime->hour*3600;//小时秒钟数
	seccount+=(uint32_t)NewTime->min*60;      //分钟秒钟数
	seccount+=NewTime->sec;//最后的秒钟加上去
	return(seccount);
}

///将秒转化为年月日时分秒
void DateTime_SectoTstru(uint32_t timecount, DateTime * Timer)
{
	uint32_t temp=0;
	uint16_t temp1=0;
	temp=timecount/86400;   //得到天数(秒钟数对应的)
	Timer->week= ((temp + 3) % 7) + 1;//获取星期  1970年1月1日星期4
	{ 
		//daycnt=temp;
		temp1=1970;  //从1970年开始
		while(temp>=365)
		{                       
			if(Is_Leap_Year(temp1))//是闰年
			{
				if(temp>=366)
					temp-=366;//闰年的秒钟数
				else 
				{
					temp1++;break;
				}
			}
			else temp-=365;       //平年 
			temp1++;  
		}   
		Timer->year=temp1;//得到年份
		temp1=0;
		while(temp>=28)//超过了一个月
		{
			if(Is_Leap_Year(Timer->year)&&temp1==1)//当年是不是闰年/2月份
			{
				if(temp>=29)temp-=29;//闰年的秒钟数
				else break; 
			}
			else 
			{
				if(temp>=mon_table[temp1])temp-=mon_table[temp1];//平年
				else break;
			}
			temp1++;  
		}
		Timer->month=temp1+1;//得到月份
		Timer->day=temp+1;  //得到日期 
	}
	temp=timecount%86400;     //得到秒钟数      
	Timer->hour=temp/3600;     //小时
	Timer->min=(temp%3600)/60; //分钟      
	Timer->sec=(temp%3600)%60; //秒钟   
}

void DateTime_GMT2BeiJingTime(DateTime * GMTDateTime)
{
   uint8_t monday;

   GMTDateTime->hour += 8;
   if(GMTDateTime->hour > 23)
   {
	   //获取月的天数
	   if((GMTDateTime->month==0)||(GMTDateTime->month > 12)) return;
	   monday = mon_table[GMTDateTime->month];
	   if(GMTDateTime->month == 2)//二月闰年29天
		   if(Is_Leap_Year(GMTDateTime->year))
			   monday = 29;
	   GMTDateTime->hour -= 24;
	   GMTDateTime->day++;
	   if(GMTDateTime->day > monday)
	   {
		   GMTDateTime->day = 1;
		   GMTDateTime->month ++;
		   if(GMTDateTime->month > 12)
		   {
			   GMTDateTime->year ++;
			   GMTDateTime->month = 1;
		   }
	   }
   }
}
    
