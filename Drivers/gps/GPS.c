/**
  ******************************************************************************
  * @file    main.c 
  * @author  zzr
  * @version V1.0.0
  * @date    2015-04-22
  * @brief  STM32 GPS数据解析获取时间日期位置速度航向
  ******************************************************************************
  * COPYRIGHT (C)  
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "GPS.h"
#include <string.h>
#include <stdlib.h>
#include "serial.h"
#include "Sys_Init.h"
#include "zRTC_Timer.h"
#include "task.h"
#include "pcf8563.h"
 

/* Private typedef -----------------------------------------------------------*/
//static struct GPS_DEV g_GpsDev;
 struct GPS_DEV g_GpsDev;
/* Private define ------------------------------------------------------------*/
#define CouSatNum 1 
#define SatReldata 2
 
int FindStrPos(const char* src,uint16_t src_len,const char * search,uint16_t search_len);
/* Private variables ---------------------------------------------------------*/
 //USART_InitTypeDef USART_InitStructure;
/* Private functions ---------------------------------------------------------*/
struct GPS_DEV *GetGpsDev(void)//结构体变量地址 返回给结构体指针（必须赋值后才可使用）
{
	return &g_GpsDev;
}

 
//获得指定位置上的参数
uint16_t  GPS_GetGPSParaStr(uint8_t  * Dat,uint8_t  * ValStr,uint16_t  Paraindex)
{  
	struct GPS_DEV *pDev = GetGpsDev();
	uint16_t  i,StartIndex,Len;
	if(Paraindex == 0) return(0);
	Len = pDev->wReadPos;
	StartIndex = 0xffff;
	for(i=0;i<Len;i++)
   {
       if(Dat[i] == ',')
	   {
	       if(Paraindex == 1) 
	           StartIndex = i + 1;
		   else if(Paraindex == 0) 
		   {
		       if(i>StartIndex)
			   {
				  strncpy((char*)ValStr,(const char*)(Dat+StartIndex),i-StartIndex);
				  ValStr[i-StartIndex] = 0;
				  return(1);
			   }
			   else
			      return(0);
		   }
		   Paraindex --;
	   }
   }
   return(0);
}

void  GPS_ClearDot(uint8_t * SDat,uint8_t * TDat)//清除字符串中的小数点
{
   while(SDat[0] != 0)
   {
      if(SDat[0] != '.')
	  {
         TDat[0] = SDat[0];
		 TDat ++;
	  }
	  SDat ++;
   }
   TDat[0] = 0;
}

uint16_t  GPS_ASC2Hex(uint8_t * Dat,uint8_t  * ValHex)
{
   uint16_t  temp = 0;

   while((Dat[0]!=0)&&((Dat[1]!=0)))
   {
      if((Dat[0] < 0x30)||(Dat[0] > 0x39)) return(0);
	  if((Dat[1] < 0x30)||(Dat[1] > 0x39)) return(0);

	  *ValHex  = (Dat[0] - 0x30) * 16; 
	  *ValHex += (Dat[1] - 0x30); 

	  Dat += 2;
	  ValHex ++;

	  temp = 1;
   }
   return(temp);
}

uint16_t  GPS_HexStr2Dec(uint8_t  * Dat,uint8_t * ValDec)
{
   uint16_t  temp = 0;

   while((Dat[0]!=0)&&((Dat[1]!=0)))
   {
      if((Dat[0] < 0x30)||(Dat[0] > 0x39)) return(0);
	  if((Dat[1] < 0x30)||(Dat[1] > 0x39)) return(0);

	  *ValDec  = (Dat[0] - 0x30) * 10; 
	  *ValDec += (Dat[1] - 0x30); 

	  Dat += 2;
	  ValDec ++;

	  temp = 1;
   }
   return(temp);
}

uint8_t  GPS_Dec2BCD(uint8_t  val)
{
   return(((val / 10) << 4) + (val % 10));
}

const uint8_t  GPSmon_table[14]={0,31,28,31,30,31,30,31,31,30,31,30,31,0};

uint8_t  GPS_Leap_Year(uint16_t  year)//是否闰年
{  
	if(year%4==0) //必须能被4整除
	{ 
		if(year%100==0) 
		{ 
			if(year%400==0)
				return 1;//如果以00结尾,还要能被400整除          
			else 
				return 0;   
		}
		else
			return 1;   
	}
	else
		return 0; 
} 

void GPS_GMT2BeiJingTime(GPSDateTime * GMTDateTime)
{
   uint8_t  monday;
   GMTDateTime->hour += 8;
   if(GMTDateTime->hour > 23)
   {
      //获取月的天数
	  if((GMTDateTime->month==0)||(GMTDateTime->month > 12)) return;
      monday = GPSmon_table[GMTDateTime->month];
	  if(GMTDateTime->month == 2)//二月闰年29天
	  if(GPS_Leap_Year(GMTDateTime->year + 2000))
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
#if 0
int GPS_DataProcessEx(struct GPS_DEV *pDev)
{

	//pDev->tGpsInfo.DateTime[0] = 0;
	uint8_t  GPSData[16];
	static  u8 ucTimeSetFlag = 0;
	u8  ucTemp = 0;
	static u16 wTimeTicks = 0;
	DateTime tTimer;
	float s_titude = 0;
	u32 wTemp = 0;
	double tempfloat = 0;
	
	
   switch(pDev->ucRecFlag)
   {
	   case  CouSatNum: //获取卫星数量
           if(GPS_GetGPSParaStr(pDev->buf,GPSData,7))
		   {
			  
			   pDev->tGpsInfo.SatCount = atoi((const char*)GPSData);
			   memset(GPSData,0,16);
		   }
		   if(GPS_GetGPSParaStr(pDev->buf,GPSData,9))   //msl altitude
		   {
			      GPS_ClearDot(GPSData,GPSData);
			    pDev->tGpsInfo.mslAltitude =atoi((const char*)GPSData);
			    memset(GPSData,0,16);
		   }
		   if(GPS_GetGPSParaStr(pDev->buf,GPSData,11))
		   {
			    GPS_ClearDot(GPSData,GPSData);
			    pDev->tGpsInfo.Altref =atoi((const char*)GPSData);
			    memset(GPSData,0,16);
		   }
           break;
	   case SatReldata: //解析出时间日期
	       if(GPS_GetGPSParaStr(pDev->buf,GPSData,1))  //时分秒
	       if(GPS_GetGPSParaStr(pDev->buf,GPSData+8,9))  //日月年
				{
					GPS_HexStr2Dec(GPSData,GPSData);
					GPS_HexStr2Dec(GPSData+8,GPSData+8);

					pDev->tGpsInfo.DateTime[0] = GPSData[10];
					pDev->tGpsInfo.DateTime[1] = GPSData[9];
					pDev->tGpsInfo.DateTime[2] = GPSData[8];       
					pDev->tGpsInfo.DateTime[3] = GPSData[0];
					pDev->tGpsInfo.DateTime[4] = GPSData[1];
					pDev->tGpsInfo.DateTime[5] = GPSData[2];
					

			   GPS_GMT2BeiJingTime((GPSDateTime*)pDev->tGpsInfo.DateTime);//GMT时间换算成北京时间
		   } 
		   //解析定位状态
			if(GPS_GetGPSParaStr(pDev->buf,GPSData,2))
		   {
	          pDev->tGpsInfo.GPSActive = (GPSData[0]=='A'); 
			      pDev->tGpsInfo.PositionUpdated	= 1;
		   }
		   else
		      pDev->tGpsInfo.GPSActive = 0;
		   //解析纬度
	     if(GPS_GetGPSParaStr(pDev->buf,GPSData,3))
		   {
		       GPS_ClearDot(GPSData,GPSData); //清除数据中的小数点
				 pDev->tGpsInfo.Latitude[0] = 0x30;
				 memcpy(&(pDev->tGpsInfo.Latitude[1]),GPSData,4);
					 wTemp = ((GPSData[4]-0x30)*10000+(GPSData[5]-0x30)*1000+(GPSData[6]-0x30)*100+(GPSData[7]-0x30)*10+(GPSData[8]-0x30));
					s_titude = wTemp*6/10000;
				 ucTemp = (u8)s_titude;
				 if(ucTemp<60)
				 {
					 pDev->tGpsInfo.Latitude[5] = (ucTemp/10)+0x30;
					 pDev->tGpsInfo.Latitude[6] = (ucTemp%10)+0x30;					 
				 }
				 else
				 {
					 pDev->tGpsInfo.Latitude[5] = 0x30;
					 pDev->tGpsInfo.Latitude[6] = 0x30;					 
					}
					//memcpy(pDev->tGpsInfo.Latitude,GPSData,6);
					//GPS_ASC2Hex(GPSData,pDev->tGpsInfo.Latitude);
		   } 
		   //解析纬度半球
	       if(GPS_GetGPSParaStr(pDev->buf,GPSData,4))
	           pDev->tGpsInfo.NanBei = GPSData[0]; 
		   //解析经度
	       if(GPS_GetGPSParaStr(pDev->buf,GPSData,5))
				{
					GPS_ClearDot(GPSData,GPSData); //清除数据中的小数点
					memcpy(pDev->tGpsInfo.Longitude,GPSData,5);
					wTemp = ((GPSData[5]-0x30)*10000+(GPSData[6]-0x30)*1000+(GPSData[7]-0x30)*100+(GPSData[8]-0x30)*10+(GPSData[9]-0x30));
					s_titude = wTemp*6/10000;
					ucTemp = (u8)s_titude;
					if(ucTemp<60)
					{
						pDev->tGpsInfo.Longitude[5] = (ucTemp/10)+0x30;
						pDev->tGpsInfo.Longitude[6] = (ucTemp%10)+0x30;					 
				 }
				 else
				 {
					 pDev->tGpsInfo.Longitude[5] = 0x30;
					 pDev->tGpsInfo.Longitude[6] = 0x30;					 
					}
					//GPS_ASC2Hex(GPSData,pDev->tGpsInfo.Longitude);
		   } 
		   //解析经度半球 
	       if(GPS_GetGPSParaStr(pDev->buf,GPSData,6))
	          pDev->tGpsInfo.DongXi = GPSData[0]; 
		   //解析速度,原始速度单位节，1节=1.852公里/小时
	       if(GPS_GetGPSParaStr(pDev->buf,GPSData,7))
		   {
			   tempfloat = atof((const char*)GPSData); 
			   tempfloat *= 1.852;
			   if(tempfloat < 255)
			       pDev->tGpsInfo.GpsSpeed = tempfloat;
			   else
				   pDev->tGpsInfo.GpsSpeed = 255; 
		   }
		   //解析航向
	       if(GPS_GetGPSParaStr(pDev->buf,GPSData,8))
		   {
			   tempfloat = atof((const char*)GPSData); 
			   tempfloat /= 2.0;
			   if(tempfloat < 179)
			       pDev->tGpsInfo.DirAngle = tempfloat;
			   else
				   pDev->tGpsInfo.DirAngle = 179; 
		   }
           break;
		default:
           break;
   } 
   if((ucTimeSetFlag == 0) &&(pDev->tGpsInfo.DateTime[1] !=0))
   {
	   //设置 本地RTC时间
	   tTimer.year 	= pDev->tGpsInfo.DateTime[0] + 2000;
	   tTimer.month = pDev->tGpsInfo.DateTime[1];
	   tTimer.day 	= pDev->tGpsInfo.DateTime[2];
	   tTimer.hour 	= pDev->tGpsInfo.DateTime[3]; 
	   tTimer.min 	= pDev->tGpsInfo.DateTime[4];
	   tTimer.sec 	= pDev->tGpsInfo.DateTime[5];
	   taskENTER_CRITICAL(); 
	   PFC_SET_TIME(&tTimer);
	   taskEXIT_CRITICAL(); 
      //DateTime_Modify(&tTimer);		  
	   ucTimeSetFlag = 1;	     
   }
   wTimeTicks++;
   if((wTimeTicks > 43200)&&(pDev->tGpsInfo.GPSActive !=0))
   {
	   wTimeTicks = 0;
	   ucTimeSetFlag = 0;	  
   }
   return 0;
}
#endif
int GPS_DataProcessEx_t(struct GPS_DEV *pDev)
{

	//pDev->tGpsInfo.DateTime[0] = 0;
	uint8_t  GPSData[16];
	static  u8 ucTimeSetFlag = 0;
	u8  ucTemp = 0;
	static u16 wTimeTicks = 0;
	DateTime tTimer;
	float s_titude = 0;
	u32 wTemp = 0;
	double tempfloat = 0;
	
   switch(pDev->ucRecFlag)
   {
	   case  CouSatNum: //获取卫星数量
           if(GPS_GetGPSParaStr(pDev->backbuf,GPSData,7))
	            pDev->tGpsInfo.SatCount = atoi((const char*)GPSData); 
		   if(GPS_GetGPSParaStr(pDev->backbuf,GPSData,9))   //msl altitude
		   {
			   GPS_ClearDot(GPSData,GPSData);
			    pDev->tGpsInfo.mslAltitude =atoi((const char*)GPSData);
			    memset(GPSData,0,16);
		   }
		   if(GPS_GetGPSParaStr(pDev->backbuf,GPSData,11))
		   {
			    GPS_ClearDot(GPSData,GPSData);
			    pDev->tGpsInfo.Altref =atoi((const char*)GPSData);
			    memset(GPSData,0,16);
		   }
           break;
	   case SatReldata: //解析出时间日期
	       if(GPS_GetGPSParaStr(pDev->backbuf,GPSData,1))  //时分秒
	       if(GPS_GetGPSParaStr(pDev->backbuf,GPSData+8,9))  //日月年
				{
					GPS_HexStr2Dec(GPSData,GPSData);
					GPS_HexStr2Dec(GPSData+8,GPSData+8);

					pDev->tGpsInfo.DateTime[0] = GPSData[10];
					pDev->tGpsInfo.DateTime[1] = GPSData[9];
					pDev->tGpsInfo.DateTime[2] = GPSData[8];       
					pDev->tGpsInfo.DateTime[3] = GPSData[0];
					pDev->tGpsInfo.DateTime[4] = GPSData[1];
					pDev->tGpsInfo.DateTime[5] = GPSData[2];
					

			   GPS_GMT2BeiJingTime((GPSDateTime*)pDev->tGpsInfo.DateTime);//GMT时间换算成北京时间
		   } 
		   //解析定位状态
			if(GPS_GetGPSParaStr(pDev->backbuf,GPSData,2))
		   {
	          pDev->tGpsInfo.GPSActive = (GPSData[0]=='A'); 
			      pDev->tGpsInfo.PositionUpdated	= 1;
		   }
		   else
		      pDev->tGpsInfo.GPSActive = 0;
		   //解析纬度
		  
	     if(GPS_GetGPSParaStr(pDev->backbuf,GPSData,3))
		   {
		       GPS_ClearDot(GPSData,GPSData); //清除数据中的小数点
				 pDev->tGpsInfo.Latitude[0] = 0x30;
				 memcpy(&(pDev->tGpsInfo.Latitude[1]),GPSData,4);
					 wTemp = ((GPSData[4]-0x30)*10000+(GPSData[5]-0x30)*1000+(GPSData[6]-0x30)*100+(GPSData[7]-0x30)*10+(GPSData[8]-0x30));
					s_titude = wTemp*6/10000;
				 ucTemp = (u8)s_titude;
				 if(ucTemp<60)
				 {
					 pDev->tGpsInfo.Latitude[5] = (ucTemp/10)+0x30;
					 pDev->tGpsInfo.Latitude[6] = (ucTemp%10)+0x30;					 
				 }
				 else
				 {
					 pDev->tGpsInfo.Latitude[5] = 0x30;
					 pDev->tGpsInfo.Latitude[6] = 0x30;					 
				}
					//memcpy(pDev->tGpsInfo.Latitude,GPSData,6);
					//GPS_ASC2Hex(GPSData,pDev->tGpsInfo.Latitude);
		   } 
		   //解析纬度半球
	       if(GPS_GetGPSParaStr(pDev->backbuf,GPSData,4))
	           pDev->tGpsInfo.NanBei = GPSData[0]; 
		   //解析经度
		        if(GPS_GetGPSParaStr(pDev->backbuf,GPSData,5))
				{					
					GPS_ClearDot(GPSData,GPSData); //清除数据中的小数点
					memcpy(pDev->tGpsInfo.Longitude,GPSData,5);
					wTemp = ((GPSData[5]-0x30)*10000+(GPSData[6]-0x30)*1000+(GPSData[7]-0x30)*100+(GPSData[8]-0x30)*10+(GPSData[9]-0x30));
					s_titude = wTemp*6/10000;
					ucTemp = (u8)s_titude;
					if(ucTemp<60)
					{
						pDev->tGpsInfo.Longitude[5] = (ucTemp/10)+0x30;
						pDev->tGpsInfo.Longitude[6] = (ucTemp%10)+0x30;					 
				 }
				 else
				 {
					 pDev->tGpsInfo.Longitude[5] = 0x30;
					 pDev->tGpsInfo.Longitude[6] = 0x30;					 
					}
					//GPS_ASC2Hex(GPSData,pDev->tGpsInfo.Longitude);
		   } 
		   //解析经度半球 
	       if(GPS_GetGPSParaStr(pDev->backbuf,GPSData,6))
	          pDev->tGpsInfo.DongXi = GPSData[0]; 
		   //解析速度,原始速度单位节，1节=1.852公里/小时
	       if(GPS_GetGPSParaStr(pDev->backbuf,GPSData,7))
		   {
			   tempfloat = atof((const char*)GPSData); 
			   tempfloat *= 1.852;
			   if(tempfloat < 255)
			       pDev->tGpsInfo.GpsSpeed = tempfloat;
			   else
				   pDev->tGpsInfo.GpsSpeed = 255; 
		   }
		   //解析航向
	       if(GPS_GetGPSParaStr(pDev->backbuf,GPSData,8))
		   {
			   tempfloat = atof((const char*)GPSData); 
			   tempfloat /= 2.0;
			   if(tempfloat < 179)
			       pDev->tGpsInfo.DirAngle = tempfloat;
			   else
				   pDev->tGpsInfo.DirAngle = 179; 
		   }
           break;
		default:
           break;
   } 
   if((ucTimeSetFlag == 0) &&(pDev->tGpsInfo.DateTime[1] !=0))
   {
	   //设置 本地RTC时间
	   tTimer.year 	= pDev->tGpsInfo.DateTime[0] + 2000;
	   tTimer.month = pDev->tGpsInfo.DateTime[1];
	   tTimer.day 	= pDev->tGpsInfo.DateTime[2];
	   tTimer.hour 	= pDev->tGpsInfo.DateTime[3]; 
	   tTimer.min 	= pDev->tGpsInfo.DateTime[4];
	   tTimer.sec 	= pDev->tGpsInfo.DateTime[5];
	   PFC_SET_TIME(&tTimer);//DateTime_Modify(&tTimer);		  
	   ucTimeSetFlag = 1;	     
   }
   wTimeTicks++;
   if((wTimeTicks > 43200)&&(pDev->tGpsInfo.GPSActive !=0))
   {
	   wTimeTicks = 0;
	   ucTimeSetFlag = 0;	  
   }
   return 0;
}		

static int Gps_RecvHandler(  unsigned char *data, u16 len)
{
//  ITStatus ErrorITStatus;
  //uart has received a character in UDR
   struct GPS_DEV *pDev = GetGpsDev();
#if 0
	u16 wReadPos = pDev->wReadPos;
// if(USART_GetITStatus(dev->base,USART_IT_RXNE))
//  {
    //接收数据
	if(pDev->ucRecFlag == 0)//
	{
		pDev->buf[wReadPos] = data[0];//one byte data 
		pDev->wReadPos++;
		if(pDev->wReadPos >= GPS_DATA_MAX_LEN)//超出buf 重新读取
		{
			pDev->wReadPos = 0;
			pDev->wReadPosOld = 0;
		}
		
		if(pDev->wReadPos >= 6) //
		{
		   if(strncmp((const char*)(pDev->buf + pDev->wReadPosOld),"$GPGGA",6)==0) //若6字节数据与后者不等 存储数据buf 指针后移一位 
		   {
				pDev->ucRecFlag =  CouSatNum;//count satellite num
				pDev->wReadPos = 0;
				(pDev->buf + pDev->wReadPosOld)[5]=0;
		   }
		   else if(strncmp((const char*)(pDev->buf + pDev->wReadPosOld),"$GPRMC",6)==0)
		   {
				pDev->ucRecFlag = SatReldata;//process sat time data
				pDev->wReadPos = 0;
			   (pDev->buf + pDev->wReadPosOld)[5]=0;
		   }
		   else
		   {
				pDev->wReadPosOld++;
				pDev->ucRecFlag = 0;
		   }
		}
	}
	else
	{
	   if(pDev->wReadPos < GPS_DATA_MAX_LEN)//
	   {	  
          pDev->buf[wReadPos] = *data;//
		  
		  if((pDev->buf[wReadPos] == 0x0D)||(pDev->buf[wReadPos] == 0x0A))
		  {
		     Serial_RecvEnable(GPS_SERCOM, 0);//close com1
			  // GPS_DataProcess();
			  GPS_DataProcessEx(pDev);
			  Serial_RecvEnable(GPS_SERCOM, 1);
			 pDev->ucRecFlag = 0;
			 pDev->wReadPos = 0;
			 
		  }
		  else	
		     pDev->wReadPos++;
	   }
	   else
	   {
	     pDev->wReadPos = 0;
			 pDev->wReadPosOld = 0;
	   }
//	}	 
 }
#endif
 //USART_ClearITPendingBit(USART1,USART_IT_RXNE);
 	return 0;
}

int GPS_Init(void)
{
	/*pin init*/
	struct GPS_DEV *pDev = GetGpsDev();
 
	// ENABLE_BLE_RX;
        ConfigGPSDMARec(); 
//	Serial_Init(GPS_SERCOM,9600);
	Serial_RegRecvHandler(GPS_SERCOM, Gps_RecvHandler);
	pDev->ucUartPort = GPS_SERCOM;
	pDev->ucRecFlag = 0;

	memset(pDev->buf, 0, GPS_DATA_MAX_LEN);
	return 0;
}

int GPS_GetWorkPara(GPS_WORKPARA *ptGpsInfo )//获取gps数据 存入sd
{
	struct GPS_DEV *pDev = GetGpsDev();
	if(ptGpsInfo != NULL)
	{
		memcpy(ptGpsInfo, &pDev->tGpsInfo, sizeof(GPS_WORKPARA));

	}
	return 0;
}

///**********************************************************************************
//Func    Name:
//Descriptions:
//Input   para:
//In&Out  Para:
//Output  para:
//Return value:
//Others  :   
//***********************************************************************************/
void ConfigGPSDMARec(void)
{
	DMA_InitTypeDef DMA_InitStructure; 
	//NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 , ENABLE); 
 
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
//	 
//	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
	 
	DMA_DeInit(GPS_DMA_CHANNEL);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART2->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)g_GpsDev.buf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = GPS_DATA_MAX_LEN;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(GPS_DMA_CHANNEL,&DMA_InitStructure);

	DMA_Cmd(GPS_DMA_CHANNEL,ENABLE);
	//USART_DMACmd(GPS_USART,USART_DMAReq_Tx,ENABLE);
	USART_DMACmd(GPS_USART,USART_DMAReq_Rx,ENABLE);
    USART_Cmd(GPS_USART, ENABLE); 
	
}
int GPS_DMARecvHandler(void)
{
	int strpos;
	u16 i,j;
	  struct GPS_DEV *pDev = GetGpsDev();
	Serial_RecvEnable(GPS_SERCOM,DISABLE);
	strpos = FindStrPos((const char*)(pDev->buf ),pDev->wLen,"$GPGGA",6);
	 if(strpos != -1) 
	 {
		 j = 0;
		 pDev->wReadPos = 0;
		 strpos += 6;  // 子字符串长度 6
		for(i = strpos;i<pDev->wLen;i++ )
		 {
			 if(pDev->buf[i] == 0x0d)
				 break;
			 pDev->backbuf[j++] = pDev->buf[i];
			 if(j > GPS_DATA_BACKBUFSIZE)
				 return -1;
		 }
		 pDev->wReadPos = j;
		pDev->ucRecFlag =  CouSatNum; 	
		 GPS_DataProcessEx_t(pDev);		 
	 }
	
	 strpos = FindStrPos((const char*)(pDev->buf ),pDev->wLen,"$GPRMC",6);
	 if( strpos!= -1) 
	 {
		 strpos += 6;   // 子字符串长度
		 j = 0;
		  pDev->wReadPos = 0;
		 for(i = strpos;i<pDev->wLen;i++ )
		 {
			 if(pDev->buf[i] == 0x0d)
				 break;
			 pDev->backbuf[j++] = pDev->buf[i];
			 if(j > GPS_DATA_BACKBUFSIZE)
				 return -1;
		 }
		  pDev->wReadPos = j;
		pDev->ucRecFlag =  SatReldata; 
		  GPS_DataProcessEx_t(pDev);
	 }
	  
	 memset(pDev->buf,0,GPS_DATA_MAX_LEN);
	 Serial_RecvEnable(GPS_SERCOM,ENABLE);

	return 0;
}

 
/**********************************************************************************
Func    Name:FindStrPos
Descriptions: 在某字符串中找到子字符串位置
Input   para: src 源字符串
In&Out  Para:
Output  para:
Return value:
Others  :   
***********************************************************************************/
  int FindStrPos(const char* src,uint16_t src_len,const char * search,uint16_t search_len)
{
	uint16_t i,j;
	if(src_len < search_len)
		return -1;
	if((src_len == 0)||(search_len == 0))
		return -1;
	
	for(i = 0; i< src_len;i++) 
	{
		if(src[i] == search[0])
		{
			for(j=0;j< search_len;j++)
			{
				if(search[j] == src[j+i])
					continue;
				else
					break;					
			}
			if(j == search_len)
			{
				return i;
			 
			}
		}
	}
	return -1;
	
}