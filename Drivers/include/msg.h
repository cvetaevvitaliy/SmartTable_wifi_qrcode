#ifndef __MSG_H
#define __MSG_H

#include "stm32f10x.h" 

#define RECV_PAYLOAD_SIZE 	64
#define RESP_PAYLOAD_SIZE 	200
#define MSG_MAXSIZE			256
#define HEAD_SIZE			8
#define MSG_HEADSIZE		8

#define MSG_ALL_HEADSIZE 	13

#define ANTIINTER_DATA  	0
#define SYNC_DATA	    	1
#define HEAD_DATA 			2
#define REL_DATA			3


typedef struct _MSG
{
	u32 wMsgType; 
	u8 Data[MSG_MAXSIZE];
}T_MSG;

typedef struct _MSG_RESP
{ 
	u8 AntiInterHead[2];
	u8 SyncHead[3];
	u8 wDeviceID[2];
	u8 ucSerial;
	
	u8 ucFuc;
	u8 ucCmd;
	u8 wLen[2];
	
	u8 ucHeadSum;
	u8 Data[RESP_PAYLOAD_SIZE];
	u8 wDataCrc[2];//
	u8 backup;
}T_MSG_RESP;

typedef struct _MSG_RECV
{
	u8 wDeviceID[2];
	u8 ucSerial;
	u8 ucFuc;
	u8 ucCmd;
	u8 wLen[2];
	u8 ucHeadSum;
	u8 Data[RECV_PAYLOAD_SIZE];
	u8 wDataCrc[2];//
}T_MSG_RECV;

typedef struct _DATA_UP
{
 	u16 USBPOWERON;  
	u16 CallSta;
	u16 BatteryVolt;
	u16 USBOutSta;
	 	
//	u8  takeup[2];
}T_DATA_UP;

//typedef	struct 
//{
//	uint16_t year;
//	uint8_t month;
//	uint8_t day;
//	uint8_t hour;
//	uint8_t min;
//	uint8_t second;
//}DateTime;


/*cmd*/
#define MCU_RESET 					0xd1
#define WIFI_SOFT_RESET				0xd2
#define WIFI_HARD_RESET				0xd3
#define SET_DEVICEID				0xd4
#define OPEN_USB_OUTPUT	 			0xd5
#define CLOSE_USB_OUTPUT			0xd6
#define CHECKOUT					0xd7
#define SET_BATTERY_UP_TIME			0xd8

/* query */
#define DEVICEID_INQUIRE 					0xa1
#define DEVICE_STATUS_INQUIRE 				0xa2
 
 
#define 	MSG_TYPE_BLANK				0x00
#define 	MSG_TYPE_RECV_WIFI_TCP		0x01
 
#endif
