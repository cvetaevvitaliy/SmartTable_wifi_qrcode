#ifndef __WIFI_ESP_H
#define __WIFI_ESP_H

#include "stm32f10x.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_tim.h"
#include "sys_tick_delay.h"

#define WIFI_USART  USART3
#define WIFI_USART_DMA_CHANNEL  DMA1_Channel3
#define WIFI_RECBUFSIZE 512

//WIFI ESP AT 指令
#define  AT  "AT\r\n\0"
#define  AT_RST  "AT\r\n\0"  	 //重启
#define  AT_GMR "AT\r\n\0"		//版本信息


//struct USART3_REC_STRUCT
//{
//	u8 buf[USART3_BUFSIZE];
//	u16 wReadPos;
//	u16 wWritePos;
//	u16 wTimerFlag;
//	u16 wRecDataFlag;
//	//u32 dwStatus; 	 
//};

 
 
 
 


struct WIFI_Dev 
{
	//u8 *buf;
	u8 rec_buf[WIFI_RECBUFSIZE];
	u8 databuf[WIFI_RECBUFSIZE];
	
	u16 idleflag;
	u16 wLen;
 	u16 wStatus;
	u16 wTcpStatus;
	//u16 dwCmdAck;

	int (*Init)(struct WIFI_Dev *dev); 
	int (*Open)(struct WIFI_Dev *dev);
	int (*Close)(struct WIFI_Dev *dev,uint8_t bCloseMode);
	int (*SendCmd)(struct WIFI_Dev *dev, char *pCmd,char *ack,u16 waittime);
	int (*Read)(struct WIFI_Dev *dev, u8 *pBuf, u16 *pwLen);
	int (*Write)(struct WIFI_Dev *dev, u8 *pBuf, u16 wLen);
	int (*GetConnetStatus)(struct WIFI_Dev *dev);
//	int (*GetSignal)(struct WIFI_Dev *dev,u8* dwVal);
	int (*ResetDev)(struct WIFI_Dev *dev,u8  hr_sr);
//	int (*HandleRecv)(struct WIFI_Dev *dev, u8 *pBuf, u16 wLen);
  
};
struct WIFI_Dev *GetWifiDev(void);

int Wifi_Init(void);
#endif
