#ifndef __WIFI_ESP_CONFIG_H
#define __WIFI_ESP_CONFIG_H
#include "wifi_esp.h"
#include "msg.h"

#define WIFI_SERVER    

#define WIFI_HARDWARE_RST		0X0000
#define WIFI_AT_RST				0X0001
#define WIFI_AT_TEST			0X0002
#define WIFI_GET_MAC			0X0004
#define WIFI_SMART_LINK			0X0008

#define WIFI_CHECK_AP	  		0X0010
#define TCP_CONNECT_CONFIG   	0X0020 
#define TCP_CONNECT_OK   		0X0100 

#define WIFI_RESTORE			0X0400


//wTcpStatus

#define TCP_STATUS_GET_IP		0X0002
#define TCP_STATUS_CONNECTED	0X0003
#define TCP_STATUS_OFF_TCP  	0X0004
#define TCP_STATUS_NO_WIFI		0X0005

u8 smart_connect(struct WIFI_Dev *dev);
void wifi_fsm(void);
void CycleBufPro(void);
int DealMsg(T_MSG *msg);
u8 wifi_tcp_send(struct WIFI_Dev *dev,u8 *dat,u16 wlength);

int MsgPackage(T_MSG_RESP *ptRespMsg,const u8 type );
int MsgRSEPPackage(T_MSG_RECV *ptdev,u8 type);
#endif
