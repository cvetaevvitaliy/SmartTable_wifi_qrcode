#include "wifi_esp_config.h"
#include "sys_tick_delay.h"
#include <string.h>
#include <stdio.h>
#include "bsp.h"
#include "Buttons.h"
#include "bsp_leds.h"
T_DATA_UP gDeviceStaDataUp;
u8 SSID[32];
u8 ssid_pwd[64];
char gWifi_mac[18];
  u16 heartticks = 0;
 extern u8 gLedStatus ;
 extern u32  gSysKeyState ; 
 extern int atoi(const char * s);
 extern u8 	gMcuID[12];
u8 wifi_quit_ap(struct WIFI_Dev *dev)
{
	return 0;
}
char p_ip[256];

/**
  * @brief  check the version .
  * @param  None
  * @retval None
  */
u8 wifi_get_gmr(struct WIFI_Dev *dev)
{
	//AT+CWJAP?
	if(dev->SendCmd(dev,"AT+GMR\r\n\0","0",1) == 0)
	{
		// 成功连入 ap
		//dev->wStatus = WIFI_CHECK_AP;
		//setbit(gLedStatus,BATTERY_LOW);
		return 1;
	}
	else
	{
		//dev->wStatus = TCP_CONNECT_CONFIG;
		return 0;
	}
	 
}
u8 wifi_get_ap(struct WIFI_Dev *dev)
{
	//AT+CWJAP?
	if(dev->SendCmd(dev,"AT+CWJAP?\r\n\0","No AP",3) == 0)
	{
		// 成功连入 ap
		dev->wStatus = WIFI_CHECK_AP;
		//setbit(gLedStatus,BATTERY_LOW);
		return 1;
	}
	else
	{
		dev->wStatus = TCP_CONNECT_CONFIG;
		return 0;
	}
	 
}
//AT+ CIPAPMAC?
u8 wifi_get_ap_mac(struct WIFI_Dev *dev)
{
	char *strx; 
	 
	if(dev->SendCmd(dev,"AT+CIPAPMAC?\r\n\0","+CIPAPMAC:",2) == 0)
	{
		strx = strstr((const char*)dev->databuf,"+CIPAPMAC:");
		if(strx != 0)
		{
			__nop();
			memcpy(gWifi_mac,strx+11,17);
			gWifi_mac[17] = '\0';
			dev->wStatus = WIFI_CHECK_AP;
			return 0;
		}
	}
	return 1;
}
u8 wifi_get_station_ip(struct WIFI_Dev *dev)
{
	//AT+CIFSR
	char *strx; 
	//if(dev->SendCmd(dev,"AT+CIPSTA?\r\n\0","+CIPSTA:",3) == 0)
	if(dev->SendCmd(dev,"AT+CIFSR?\r\n\0","OK",3) == 0)
	{
		strx = strstr((const char*)dev->databuf,"+CIFSR:");
		if(strx != 0)
		{
			__nop();
			strcpy(p_ip,(strx+11));
			return 0;
		}
	}
	return 1;	
}
u8 wifi_tcp_config(struct WIFI_Dev *dev)
{
	
	static u8 err_times = 0;
	//AT+CIPSTART="TCP","192.168.101.110",1000  www.lorytech.com  lorytest.lorytech.com  117.88.2.42  7903
	if(dev->SendCmd(dev,"AT+CIPSTART=\"TCP\",\"lorytest.lorytech.com\",7903\r\n\0","CONNECT",5) == 0)
	{
		// 连接成功
		dev->wStatus = TCP_CONNECT_OK;
		err_times = 0;
		return 0;
	}
	else
	{
		
		// ERROR  ALREAY CONNECT
		if(strstr((const char*)dev->databuf,"ALREAY CONNECT"))
		{
			err_times = 0;
			dev->wStatus = TCP_CONNECT_OK;
			//gLedStatus = NORMAL_WORKING;
			return 0;
		}
		else
		{
			err_times++;
			if(err_times > 0x0f)
			{
				err_times = 0;
				dev->Close(dev,1);
				dev->wStatus = WIFI_CHECK_AP;
				__nop(); 
			}
			return 1;
		}
	}
	
}
u8 wifi_get_tcp_status(struct WIFI_Dev *dev)
{
//	u8 tcp_status;
	char *strx;
	static u16 ticks = 0;
	//AT+CIPSTATUS
	ticks++;
	if((ticks>>7) & 0x0001)
	{
		if(dev->SendCmd(dev, "AT+CIPSTATUS\r\n","STATUS:",5) == 0)
		{
			strx = strstr((const char*)dev->databuf,"STATUS:");
			if(strx != 0)
			{
 
				dev->wTcpStatus = *(strx+7)-0x30; 
				switch(dev->wTcpStatus)
				{
					case TCP_STATUS_GET_IP:
						
						break;
					case TCP_STATUS_CONNECTED:
						break;
					case TCP_STATUS_OFF_TCP:
						dev->wStatus = WIFI_CHECK_AP;
						break;
					case TCP_STATUS_NO_WIFI:
						dev->wStatus = WIFI_CHECK_AP;
						break;
					default:						
						break;
				}
				return 0;
			}
			
		}
	}
	
	return 1;	
}


u8 wifi_tcp_send(struct WIFI_Dev *dev,u8 *dat,u16 wlength)
{
//	char temp[5];
	char cmd_buf[32];
//	static u8 timeout_nums = 0;
	u16  wlen_temp; 
//	uint16_t timeout = 0;
	 
	heartticks = 0;  // 心跳包计数置零
	if(wlength > 2048)
		wlen_temp = 2048;
	else
		wlen_temp = wlength;
//	sprintf(temp, "%d", wlen_temp);
	sprintf(cmd_buf,"AT+CIPSEND=%d\r\n",wlen_temp);
	if(dev->SendCmd(dev, cmd_buf,">",5) == 0)
	{
		dev->Write(dev,dat,wlen_temp);
		__nop();
	}
	else
	{
		//link is not valid
		if(strstr((const char*)dev->databuf,"link is not valid"))
		{
			dev->wStatus = WIFI_CHECK_AP;
		}
		if(strstr((const char*)dev->databuf,"ERROR"))
		{
			dev->wStatus = WIFI_CHECK_AP;
		}
		
	}
 	return 0;
	
}

/*********
0: 使用 安信可 AI-LINK技术
1: 使用 ESP-TOUCH技术
2: 使用 AIR-KISS
**********/
/**
  * @brief  Configures  智能连接 开启手机app.
  * @param  WIFI_Dev
  * @retval 0 success   other  failure
  */
u8 smart_connect(struct WIFI_Dev *dev)
{
	u16 s_timeout = 5;
	// AT+CWSMARTSTART  启用智能连接
	
		//Sys_delay_ms(1000);
		//Sys_delay_ms(1000);
 
	SysTickDelay_ms(2000);
	while(s_timeout--)
	{
		if(dev->SendCmd(dev,"AT+CWSMARTSTART=0\r\n\0","OK",2) == 0)
		{
			//启动成功
			__nop();
			break;
			
		}
	}
	if(s_timeout == 0)
		return 1;
	// 等待数据
	s_timeout = 300;
	while(s_timeout--)
	{
		//
		SysTickDelay_ms(1000);
		GreenLedToggle();RedLedOff();
		if(dev->idleflag)
		{
			__nop();
			if(strstr((const char*)dev->databuf,"SMART SUCCESS"))
				break;			
		}
	}
	if(s_timeout == 0)
	{
		dev->wStatus = WIFI_HARDWARE_RST;
		if(dev->SendCmd(dev,"AT+CWSMARTSTOP\r\n\0","OK",2) != 0)
		{
			// 
			
			return 1;
			
		}
	}
		
	return 0;
}
int wifi_tcp_rec_pro(u8 *recbuf,u16 wLen)
{
	u8 buf[128];
	u8 ucHeadSum;
	u16 i,headstart;
	u16 wMsgLen;
	T_MSG tMsg; 
    T_MSG_RECV *ptRecvMsg = (T_MSG_RECV *)&tMsg.Data[0];
	 
	if(wLen > 128)
		return -1;
	memcpy(buf,recbuf,wLen);
	


	if(wLen < HEAD_SIZE)
		return -1;
 
	for(i=0;i<wLen;i++)
	{
		if(recbuf[i] == 0xc0)
			if(recbuf[i+1] == 0xc0)
				if(recbuf[i+2] == 0xc0)
					break;
	}
	if(i == wLen)
		return -1;
	headstart = i+3;
	ucHeadSum = 0;
	for(i=headstart;i<headstart+7;i++)
		ucHeadSum += recbuf[i];
	if(ucHeadSum != recbuf[headstart+7])
		return -1;
	tMsg.wMsgType = MSG_TYPE_RECV_WIFI_TCP;
	memcpy((u8*)tMsg.Data,&recbuf[headstart],wLen-headstart);
	wMsgLen = ptRecvMsg->wLen[0] + (ptRecvMsg->wLen[1]<<8);	
    ptRecvMsg->wDataCrc[0] = tMsg.Data[wMsgLen+MSG_HEADSIZE-2];
    ptRecvMsg->wDataCrc[1] = tMsg.Data[wMsgLen+MSG_HEADSIZE-1];
	DealMsg(&tMsg);
	return 0;
	
}
void CycleBufPro(void)
{
	char *pStr;
	int wRecLen;
	u8 lenbuf[5],i; 
	struct WIFI_Dev *pDev = GetWifiDev();
	
	
	if(pDev->idleflag)
	{
		
		if(pDev->wLen > 5)
		{
			pStr = strstr((const char*)pDev->rec_buf,"+IPD,");
			if(pStr != 0)
			{
				pStr = pStr+5;
				for(i=0; i<4; i++)
				{
					lenbuf[i] = *(pStr++);
					if(*(pStr) == ':')
					{
						lenbuf[i+1] = 0;
						break;
					}
						 
				}
				wRecLen = atoi((const char*)lenbuf);
				wifi_tcp_rec_pro((u8*)(pStr+1),wRecLen);
			}			
			pStr = strstr((const char*)pDev->rec_buf,"WIFI CONNECTED");
			if(pStr != 0)
			{
				pDev->wStatus = WIFI_CHECK_AP;
			}
			pStr = strstr((const char*)pDev->rec_buf,"CLOSED");
			if(pStr != 0)
			{
				pDev->wStatus = WIFI_CHECK_AP;
			}
		}
		
		
		pDev->idleflag = 0;
	}
	
}
// wifi 状态机
void wifi_fsm(void)
{
	u16 wifi_Status;
	static u16 firstsendflag = 0;
	static u16 heartticks = 0;
	struct WIFI_Dev *pWifiDev = GetWifiDev();
	T_MSG_RESP tRespMsg; 
	wifi_Status = pWifiDev->GetConnetStatus(pWifiDev);
 /*#define WIFI_AT_RST			0X0001
#define WIFI_AT_TEST		0X0002

#define WIFI_SMART_LINK		0X0008

#define WIFI_JOIN_AP_OK  	0X0010

#define TCP_CONNECT_OK   	0X0100 ***********/
	switch(wifi_Status)
	{
		case WIFI_HARDWARE_RST:
			pWifiDev->ResetDev(pWifiDev,0);
			pWifiDev->wStatus = WIFI_AT_TEST;
			break;
		case WIFI_AT_RST:
			// at 重启  
			pWifiDev->ResetDev(pWifiDev,1);
			pWifiDev->wStatus = WIFI_AT_TEST;

			break;
		case WIFI_AT_TEST:
			pWifiDev->Open(pWifiDev);
			
			break;
		case WIFI_GET_MAC:
			wifi_get_gmr(pWifiDev);
			SysTickDelay_ms(200);
			wifi_get_ap_mac(pWifiDev);
			break;
		case WIFI_SMART_LINK:
			smart_connect(pWifiDev);
			pWifiDev->wStatus = WIFI_CHECK_AP;
			break;
		case WIFI_CHECK_AP:
			wifi_get_ap(pWifiDev);
			break;
		case TCP_CONNECT_CONFIG:			
			wifi_tcp_config( pWifiDev);
			firstsendflag = 0;
			break;
		case TCP_CONNECT_OK:
			// send data
		//	wifi_tcp_send(pWifiDev,(u8*)wifisenddat,strlen(wifisenddat));
			if((heartticks & 0x07 ) == 0)
				wifi_get_tcp_status(pWifiDev);		
			heartticks++;
			if(heartticks > 50)
			{
				heartticks = 0;
				firstsendflag = 0;
			}
			if(firstsendflag == 0)
			{
				if(gWifi_mac[2]!= ':')
					pWifiDev->wStatus = WIFI_GET_MAC;
				firstsendflag = 1;
				tRespMsg.wLen[0] =8+18+2;//
				tRespMsg.wLen[1] = 0x00;
				tRespMsg.ucCmd = DEVICEID_INQUIRE;
				memcpy((u8*)&tRespMsg.Data[0],(u8*)gMcuID,12);
				memcpy((u8*)&tRespMsg.Data[12],gWifi_mac ,18);
				MsgPackage(&tRespMsg,1);
			} 
			break;
		case WIFI_RESTORE:
			// 恢复出厂设置
			pWifiDev->ResetDev(pWifiDev,2);
			break;
		default:
			break;
	}
	
	
	
}

int DealMsg(T_MSG *msg)
{
	struct WIFI_Dev *pDev = GetWifiDev();
	
	T_MSG_RECV *ptRecvMsg;
	T_MSG_RESP tRespMsg; 
 
 

	//ParameterStruct tGetPar;
	//DateTime tTime;
	//GetSetedPar(&tGetPar);
	ptRecvMsg = (T_MSG_RECV *)&msg->Data[0]; 
	switch(ptRecvMsg->ucCmd)
	{
		case MCU_RESET:
			RestartMCU();
			break;
		case WIFI_SOFT_RESET:
			pDev->ResetDev(pDev,1);
			break;
		case WIFI_HARD_RESET:
			pDev->ResetDev(pDev,0);
			break;
		case OPEN_USB_OUTPUT:
			USB_OUT_POWER_ON;
			break;
		case CLOSE_USB_OUTPUT:
			USB_OUT_POWER_OFF; 
			break;
		case CHECKOUT:
			D1_RGB_G_ON;
			gDeviceStaDataUp.CallSta = 2;
			break;
		case DEVICEID_INQUIRE:
			tRespMsg.wLen[0] =12+18+2;//
			tRespMsg.wLen[1] = 0x00;
			tRespMsg.ucCmd = DEVICEID_INQUIRE;
			memcpy((u8*)&tRespMsg.Data[0],(u8*)gMcuID,12);
			memcpy((u8*)&tRespMsg.Data[12],gWifi_mac ,18);
			break;		
		case DEVICE_STATUS_INQUIRE:
			tRespMsg.wLen[0] =8+2;//
			tRespMsg.wLen[1] = 0x00;
			tRespMsg.ucCmd = DEVICE_STATUS_INQUIRE;
			memcpy((u8*)&tRespMsg.Data[0],(u8*)&gDeviceStaDataUp,sizeof(gDeviceStaDataUp));			
			break;
 
		default:
			break;
	}	
	if((ptRecvMsg->ucCmd == DEVICEID_INQUIRE) || (ptRecvMsg->ucCmd == DEVICE_STATUS_INQUIRE))
	{
		MsgPackage(&tRespMsg,1);
	}
	else
		MsgRSEPPackage(ptRecvMsg,1);
	
	
	return 0;
	
}

//	wlen = ptRespMsg->wLen[0] +(ptRespMsg->wLen[1]<<8);
	//						Gprs_Write((unsigned char*)&tMsg.Data,13+wlen );
int MsgRSEPPackage(T_MSG_RECV *ptdev,u8 type)//设置指令回复
{ 
	
	u16 wlen; 	 
	struct WIFI_Dev *pDev = GetWifiDev();
	T_MSG tMsg;
	T_MSG_RESP *tRespMsg = (T_MSG_RESP *)&tMsg.Data[0];
 
	
 	
	memset(tRespMsg->AntiInterHead, 0xaa, 2); 
	memset(tRespMsg->SyncHead, 0xc0, 3); 
 
	memcpy(tRespMsg->wDeviceID,ptdev->wDeviceID,8);	
	wlen = tRespMsg->wLen[0] + (tRespMsg->wLen[1]<<8);
	if(wlen > 2)
	{
		memcpy(tRespMsg->Data,ptdev->Data,wlen-2);
		tRespMsg->Data[wlen-2] = ptdev->wDataCrc[0];
		tRespMsg->Data[wlen-1] = ptdev->wDataCrc[1];
		tRespMsg->wDataCrc[0] = ptdev->wDataCrc[0];
		tRespMsg->wDataCrc[1] = ptdev->wDataCrc[1];
	}
	else
	{
		__nop();
	}
	tMsg.wMsgType = type;
	if(pDev->wStatus == TCP_CONNECT_OK)
	{
		 
		wifi_tcp_send(pDev,(unsigned char*)&tMsg.Data,13+wlen );
	 				 
	}
	
	return 0;
} 
int MsgPackage(T_MSG_RESP *ptRespMsg,const u8 type ) 
{ 
	//struct GPRS_DEV *pDev = GetGprsDev();
	static u8 ucSerNum = 0;
	u8	ucCheckSum;
	u8 CRCDATATemp[2];
	u16 i,wLen; 
	T_MSG tMsg;
	
	u8* pCheck = (u8*)&ptRespMsg->wDeviceID;	
	struct WIFI_Dev *pDev = GetWifiDev(); 	
	
	 
	//u32 dwGprsStatus = 0;
	ucSerNum++;
//	Gprs_GetConnectStatus(&dwGprsSt atus);
	memset(ptRespMsg->AntiInterHead, 0xaa, 2); 
	memset(ptRespMsg->SyncHead, 0xc0, 3); 
	ptRespMsg->wDeviceID[0] = 0;			
	ptRespMsg->wDeviceID[1] = 0; 	
	ptRespMsg->ucSerial = ucSerNum;
	ptRespMsg->ucFuc = 0;
	ucCheckSum = 0;
	for(i=0;i<7;i++)
		ucCheckSum += *pCheck++;
	ptRespMsg->ucHeadSum = ucCheckSum;
	wLen = ptRespMsg->wLen[0] + (ptRespMsg->wLen[1]<<8);
 
	if(wLen>2)
	{
		if((MSG_ALL_HEADSIZE+wLen)> MSG_MAXSIZE)
			return -1;
		CRC16(ptRespMsg->Data,wLen-2,CRCDATATemp);
		ptRespMsg->wDataCrc[0]= CRCDATATemp[0];
		ptRespMsg->wDataCrc[1]= CRCDATATemp[1];
		ptRespMsg->Data[wLen-2] = CRCDATATemp[0];
		ptRespMsg->Data[wLen-1] = CRCDATATemp[1];
		memset((u8*)&tMsg,0,sizeof(tMsg));
		memcpy(tMsg.Data,ptRespMsg->AntiInterHead,MSG_ALL_HEADSIZE+wLen);
		
	}
	else
	{
		memcpy(tMsg.Data,ptRespMsg->AntiInterHead,MSG_ALL_HEADSIZE);
		
	}
	tMsg.wMsgType = type;	
	if(pDev->wStatus == TCP_CONNECT_OK)
	{
		 
		wifi_tcp_send(pDev,(unsigned char*)&tMsg.Data,13+wLen );
	 				 
	}
	return 0;
}
