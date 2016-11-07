#ifndef __GPS_H
#define __GPS_H
#include "stm32f10x.h"
#include "stm32f10x_dma.h"
////////////////////////////////////////////////////////////////////////////////
#define GPS_SERCOM  serCOM2
#define GPS_DATA_MAX_LEN 512
#define GPS_DATA_BACKBUFSIZE 128
#define GPS_USART   USART2
#define GPS_DMA_CHANNEL  DMA1_Channel6
 
typedef struct 
 {
	 uint8_t DateTime[6];	 //ʱ��
	 uint8_t Backup1[4];	 //ռλ
	 uint8_t GPSActive;	 //��λ��Ϣ�Ƿ���Ч
	 uint8_t SatCount;		 //��������
	 uint8_t Longitude[7];  //����
	 uint8_t Latitude[7];   //γ��
	 uint8_t DirAngle;		 //���з�λ
	 uint8_t GpsSpeed;	     //�����ٶ�
	 uint8_t DongXi;		 //��������
	 uint8_t NanBei;		 //��γ��γ
	 uint8_t PositionUpdated;//GPS��Ϣ���¹�
	 uint8_t temp;
	 s16 mslAltitude;
	 s16 Altref;
 }GPS_WORKPARA;

 typedef struct 
{
   uint8_t  year;
   uint8_t  month;
   uint8_t  day;       
   uint8_t  hour;
   uint8_t  min;
   uint8_t  sec;                              
}GPSDateTime;


struct GPS_DEV
{
	u8 ucUartPort;
	u8 ucRecFlag; /*0 recv head, 1 recv payload*/
	u8 buf[GPS_DATA_MAX_LEN];
        u8 backbuf[GPS_DATA_BACKBUFSIZE];
	u16 wReadPos;
	u16 wReadPosOld;
	u16 wLen;	
	GPS_WORKPARA tGpsInfo;	
};
void ConfigGPSDMARec(void);
int GPS_Init(void);
int GPS_GetWorkPara(GPS_WORKPARA *ptGpsInfo);
int GPS_DMARecvHandler(void);
struct GPS_DEV *GetGpsDev(void);
#endif
