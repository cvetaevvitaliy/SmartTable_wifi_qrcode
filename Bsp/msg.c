#include "msg.h"
#include "wifi_esp.h"
#include "wifi_esp_config.h"
#include "encryption.h"
#include <stdio.h>
#include <string.h>
#include "bsp.h"
#include "usb_contrl.h"

extern T_DATA_UP gDeviceStaDataUp;
extern u8 	gMcuID[12];
extern char gWifi_mac[18];
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
			 
			UsbOutContrl(1);
			break;
		case CLOSE_USB_OUTPUT:
			 
			UsbOutContrl(0);
			break;
		case CHECKOUT:
			 
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
		MsgPackage(&tRespMsg,MSG_TYPE_WIFI);
	}
	else
		MsgRSEPPackage(ptRecvMsg,1);
	
	
	return 0;
	
}

//	wlen = ptRespMsg->wLen[0] +(ptRespMsg->wLen[1]<<8);
	//						Gprs_Write((unsigned char*)&tMsg.Data,13+wlen );
int MsgRSEPPackage(T_MSG_RECV *ptdev,u8 type)//ÉèÖÃÖ¸Áî»Ø¸´
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
