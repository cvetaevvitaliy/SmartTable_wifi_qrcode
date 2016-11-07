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
//��ʼ��rtc
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
	 //��������RTC
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
      //������ɺ���󱸼Ĵ�����д�����ַ�0x5A5A     
	
}
#endif

//����rtcʱ��
void DateTime_RTCConfig(uint32_t RTCSeconds)
{
	DateTime NewTime;
     //������BKP�ĺ󱸼Ĵ���1�У�����һ�������ַ�0xA5A5
    //��һ���ϵ��󱸵�Դ����󣬸üĴ������ݶ�ʧ��
    //����RTC���ݶ�ʧ����Ҫ��������

   if(BKP_ReadBackupRegister(RTCConfigedFlagAddr) != 0x5A5A)
   {
		if((RTCSeconds !=0)&&(RTCSeconds != 0xffffffff))
		{
			DateTime_SectoTstru(RTCSeconds+1, &NewTime); //�м�¼�Ӽ�¼ʱ������ȡ����ʱ�䣬��֤�¼�¼ʱ����ϼ�¼��
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
		DateTime_Modify(&NewTime);//Ĭ��ʱ��
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
//�ж��Ƿ������꺯��

//�·�   1  2  3  4  5  6  7  8  9  10 11 12

//����   31 29 31 30 31 30 31 31 30 31 30 31

//������ 31 28 31 30 31 30 31 31 30 31 30 31

//����:���

//���:������ǲ�������.1,��.0,����

uint8_t Is_Leap_Year(uint16_t year)
{                     
   if(year%4==0) //�����ܱ�4����
   { 
		  if(year%100==0) 
		  { 
				 if(year%400==0)return 1;//�����00��β,��Ҫ�ܱ�400����          
				 else return 0;   
		  }else return 1;   
   }else return 0; 
}                           

//����ʱ��

//�������ʱ��ת��Ϊ����

//��1970��1��1��Ϊ��׼

//1970~2099��Ϊ�Ϸ����

//����ֵ:0,�ɹ�;����:�������.

//�·����ݱ�                                                                        
const uint8_t table_week[12]={0,3,3,6,1,4,6,2,5,0,3,5}; //���������ݱ�   
//ƽ����·����ڱ�
const uint8_t mon_table[12]={31,28,31,30,31,30,31,31,30,31,30,31};

uint8_t DateTime_Modify(DateTime * NewTime)
{
//	uint16_t t;
	uint32_t seccount=0;
	if(NewTime->year<2014||NewTime->year>2099)return 1;//syear��Χ1970-2099���˴����÷�ΧΪ2014-2099        
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

//�õ���ǰ��ʱ��

//����ֵ:0,�ɹ�;����:�������.

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


//��ʱ��ת��Ϊ��

uint32_t DateTime_TStrutoSec(DateTime * NewTime)
{
	uint8_t month;
	uint16_t t;
	uint32_t seccount=0;
	month =  NewTime->month;
	//if(NewTime->year<2014||NewTime->year>2099)return 1;//syear��Χ1970-2099���˴����÷�ΧΪ2014-2099        
	for(t=1970;t<NewTime->year;t++) //��������ݵ��������
	{
	  if(Is_Leap_Year(t))seccount+=31622400;//�����������
	  else seccount+=31536000;                    //ƽ���������
	}
	month-=1;
	for(t=0;t<month;t++)         //��ǰ���·ݵ����������
	{
	  seccount+=(uint32_t)mon_table[t]*86400;//�·����������
	  if(Is_Leap_Year(NewTime->year) && (t==1))seccount+=86400;//����2�·�����һ���������         
	}
	seccount+=(uint32_t)(NewTime->day-1)*86400;//��ǰ�����ڵ���������� 
	seccount+=(uint32_t)NewTime->hour*3600;//Сʱ������
	seccount+=(uint32_t)NewTime->min*60;      //����������
	seccount+=NewTime->sec;//�������Ӽ���ȥ
	return(seccount);
}

///����ת��Ϊ������ʱ����
void DateTime_SectoTstru(uint32_t timecount, DateTime * Timer)
{
	uint32_t temp=0;
	uint16_t temp1=0;
	temp=timecount/86400;   //�õ�����(��������Ӧ��)
	Timer->week= ((temp + 3) % 7) + 1;//��ȡ����  1970��1��1������4
	{ 
		//daycnt=temp;
		temp1=1970;  //��1970�꿪ʼ
		while(temp>=365)
		{                       
			if(Is_Leap_Year(temp1))//������
			{
				if(temp>=366)
					temp-=366;//�����������
				else 
				{
					temp1++;break;
				}
			}
			else temp-=365;       //ƽ�� 
			temp1++;  
		}   
		Timer->year=temp1;//�õ����
		temp1=0;
		while(temp>=28)//������һ����
		{
			if(Is_Leap_Year(Timer->year)&&temp1==1)//�����ǲ�������/2�·�
			{
				if(temp>=29)temp-=29;//�����������
				else break; 
			}
			else 
			{
				if(temp>=mon_table[temp1])temp-=mon_table[temp1];//ƽ��
				else break;
			}
			temp1++;  
		}
		Timer->month=temp1+1;//�õ��·�
		Timer->day=temp+1;  //�õ����� 
	}
	temp=timecount%86400;     //�õ�������      
	Timer->hour=temp/3600;     //Сʱ
	Timer->min=(temp%3600)/60; //����      
	Timer->sec=(temp%3600)%60; //����   
}

void DateTime_GMT2BeiJingTime(DateTime * GMTDateTime)
{
   uint8_t monday;

   GMTDateTime->hour += 8;
   if(GMTDateTime->hour > 23)
   {
	   //��ȡ�µ�����
	   if((GMTDateTime->month==0)||(GMTDateTime->month > 12)) return;
	   monday = mon_table[GMTDateTime->month];
	   if(GMTDateTime->month == 2)//��������29��
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
    
