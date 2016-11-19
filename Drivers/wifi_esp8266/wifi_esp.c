 
#include <stdio.h>
#include <string.h>
#include "wifi_esp.h" 
#include "wifi_esp_config.h"

#define WIFI_RST_PORT	GPIOC
#define WIFI_RST_PIN  	GPIO_Pin_5

#define WIFI_PWR_PORT		GPIOA
#define WIFI_PWR_PIN  		GPIO_Pin_3
#define WIFI_PWR_ENABLE		GPIO_SetBits(WIFI_PWR_PORT,WIFI_PWR_PIN)
#define WIFI_PWR_DISABLE	GPIO_ResetBits(WIFI_PWR_PORT,WIFI_PWR_PIN)
struct WIFI_Dev g_WifiDev;

struct WIFI_Dev *GetWifiDev(void)
{
	return &g_WifiDev ;
}

void USART3_IRQHandler(void)
{
 	u8 ucDat;
	u16 temp;
    /* ÖÐ¶Ï½ÓÊÕ*/
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) 
    {
        ucDat = USART_ReceiveData(USART3);   
//		g_Usart3RecStr.buf[g_Usart3RecStr.wWritePos++] = ucDat;
//		if(g_Usart3RecStr.wWritePos == USART3_BUFSIZE)
//		{
//			g_Usart3RecStr.wWritePos = 0;
//			g_Usart3RecStr.wRecDataFlag = (g_Usart3RecStr.wRecDataFlag+1) &0xff;
//			
//		}
//		TIM4_Set(1); 
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);        
    }	
	if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)  
    { 
		temp = USART3->SR;  
		temp = USART3->DR; //?USART_IT_IDLE?? 
		USART_ClearITPendingBit(USART3, USART_IT_IDLE);		
		DMA_Cmd(WIFI_USART_DMA_CHANNEL,DISABLE);  
		g_WifiDev.wLen = WIFI_RECBUFSIZE - DMA_GetCurrDataCounter(WIFI_USART_DMA_CHANNEL); 
		g_WifiDev.idleflag = 1;
		memcpy(g_WifiDev.databuf,g_WifiDev.rec_buf, g_WifiDev.wLen);
		DMA_SetCurrDataCounter(WIFI_USART_DMA_CHANNEL,WIFI_RECBUFSIZE);  
		DMA_Cmd(WIFI_USART_DMA_CHANNEL,ENABLE);  
    }
	
}
#if 0
void TIM4_Set(u8 sta)
{
	if(sta)
	{       
		TIM_Cmd(TIM4, ENABLE);  //??TIMx	
		TIM_SetCounter(TIM4,0);//?????
	}else TIM_Cmd(TIM4, DISABLE);//?????4	   
}
//??????????
 
void TIM4_Init(u16 time_ms)
{	
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //????//TIM4????    
	
	//???TIM3???
	TIM_TimeBaseStructure.TIM_Period = (time_ms*10)-1; //???????????????????????????	
	TIM_TimeBaseStructure.TIM_Prescaler = 7199; //??????TIMx??????????? 72M/7200 =10kHZ 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //??????:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM??????
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //??????????TIMx???????
 
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); //?????TIM4??,??????

	 	  
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x05 ;//?????1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//????1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ????
	NVIC_Init(&NVIC_InitStructure);	//??????????VIC???
	
}
void TIM4_IRQHandler(void)
{ 	
	struct WIFI_Dev *pDev = GetWifiDev(); 
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)//?????
	{	 
		//pDev->idleflag = 1;		 
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  //??TIMx??????    
		TIM4_Set(0);			//??TIM4  
	}
	 
}
#endif
void WIFI_UART(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO , ENABLE );
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE );
	 
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB,&GPIO_InitStructure); 
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode=  GPIO_Mode_IN_FLOATING; //GPIO_Mode_IPU
	GPIO_Init(GPIOB,&GPIO_InitStructure);
		  /* Enable the   gloabal Interrupt */
	  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);	
	/********
	*²¨ÌØÂÊ 
	************/
	USART_InitStructure.USART_BaudRate            = 115200  ;    
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;
	USART_InitStructure.USART_Parity              = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(WIFI_USART, &USART_InitStructure);
	//USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
	//USART_ITConfig(WIFI_USART, USART_IT_RXNE, ENABLE);

	USART_ITConfig(WIFI_USART, USART_IT_TC, DISABLE);
	USART_ITConfig(WIFI_USART, USART_IT_RXNE, DISABLE);
	USART_ITConfig(WIFI_USART, USART_IT_IDLE, ENABLE);
	USART_ClearFlag(WIFI_USART,USART_FLAG_TC);
	//USART_Cmd(WIFI_USART, ENABLE);
	 
}
 void ConfigGPSDMARec(void)
{
	DMA_InitTypeDef DMA_InitStructure; 
	//NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3 , ENABLE); 
 
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
//	 
//	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
	 
	DMA_DeInit(WIFI_USART_DMA_CHANNEL);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART3->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)g_WifiDev.rec_buf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = WIFI_RECBUFSIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(WIFI_USART_DMA_CHANNEL,&DMA_InitStructure);

	DMA_Cmd(WIFI_USART_DMA_CHANNEL,ENABLE);
	//USART_DMACmd(GPS_USART,USART_DMAReq_Tx,ENABLE);
	USART_DMACmd(WIFI_USART,USART_DMAReq_Rx,ENABLE);
    USART_Cmd(WIFI_USART, ENABLE); 
	
}

 void WIFI_IOConfig(void)
 {
	 GPIO_InitTypeDef GPIO_InitStructure; 
	 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO , ENABLE );
	 
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_NoJTRST, ENABLE); // ÊÍ·Åpb4
//	 // wifi rst io
//	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
//	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
//	GPIO_Init(GPIOA,&GPIO_InitStructure); 
//	// wifi config key
//	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;
//	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz;
//	GPIO_InitStructure.GPIO_Mode=  GPIO_Mode_IN_FLOATING; //GPIO_Mode_IPU
//	GPIO_Init(GPIOA,&GPIO_InitStructure);
//	 
 	GPIO_InitStructure.GPIO_Pin= WIFI_PWR_PIN;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	 
	GPIO_Init(WIFI_PWR_PORT,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin= WIFI_RST_PIN;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	 
	GPIO_Init(WIFI_RST_PORT,&GPIO_InitStructure);
	  
	WIFI_PWR_ENABLE;
	 
	 
 }
 void esp8266_hard_reset(void)
 {
	 GPIO_ResetBits(WIFI_RST_PORT,WIFI_RST_PIN);
	 SysTickDelay_ms(500);
	 GPIO_SetBits(WIFI_RST_PORT,WIFI_RST_PIN);
//	 GPIO_ResetBits(WIFI_PWR_PORT,WIFI_PWR_PIN);
//	 SysTickDelay_ms(500);
//	 GPIO_SetBits(WIFI_PWR_PORT,WIFI_PWR_PIN);
	 
 }
 
// //Ïòsim900a·¢ËÍÃüÁî

 
//esp8266·¢ËÍÃüÁîºó,¼ì²â½ÓÊÕµ½µÄÓ¦´ð
//str:ÆÚ´ýµÄÓ¦´ð½á¹û
//·µ»ØÖµ:0,Ã»ÓÐµÃµ½ÆÚ´ýµÄÓ¦´ð½á¹û
//    ÆäËû,ÆÚ´ýÓ¦´ð½á¹ûµÄÎ»ÖÃ(strµÄÎ»ÖÃ)
u8* esp8266_check_cmd(struct WIFI_Dev *dev,char *str)
{
	char *strx=0; 
	 
//	if(str == 0)
//		return 0;
	if(dev->idleflag )		//½ÓÊÕµ½Ò»´ÎÊý¾ÝÁË pUsartBuf->flag ÔÚ¶¨Ê±Æ÷ÖÐÖÃÎª1
	{ 
		//½«fifoµÄÊý¾Ý ¶Á³öÀ´
		if(dev->wLen < (WIFI_RECBUFSIZE-1))
		{
			dev->databuf[dev->wLen+1] = 0;
		}
		//USART2_RX_BUF[USART2_RX_STA&0X7FFF]=0;//Ìí¼Ó½áÊø·û
		strx=strstr((const char*)dev->databuf,(const char*)str);
		dev->idleflag = 0; 
	}
	else
		return 0;
//	if(dev->wLen > 0)
//	{
//		strx=strstr((const char*)dev->buf,(const char*)str);
//		dev->wLen = 0;
//	}
//	else
//		return 0;
	return (u8*)strx;
}
////cmd:·¢ËÍµÄÃüÁî×Ö· 
////ack:ÆÚ´ýµÄÓ¦´ð½á¹û,Èç¹ûÎª¿Õ,Ôò±íÊ¾²»ÐèÒªµÈ´ýÓ¦´ð
////waittime:µÈ´ýÊ±¼ä(µ¥Î»:10ms)
////·µ»ØÖµ:0,·¢ËÍ³É¹¦(µÃµ½ÁËÆÚ´ýµÄÓ¦´ð½á¹û)
////       1,·¢ËÍÊ§°Ü
/******
bool ESP8266_Cmd ( char * cmd, char * reply1, char * reply2, u32 waittime )
{
	strEsp8266_Fram_Record .InfBit .FramLength = 0;               //???????????
	ESP8266_Usart ( "%s\r\n", cmd );
	if ( ( reply1 == 0 ) && ( reply2 == 0 ) )                      //???????
		return true;
	Delay_ms ( waittime );                 //??
	strEsp8266_Fram_Record .Data_RX_BUF [ strEsp8266_Fram_Record .InfBit .FramLength ]  = '\0';
	PC_Usart ( "%s", strEsp8266_Fram_Record .Data_RX_BUF );
	if ( ( reply1 != 0 ) && ( reply2 != 0 ) )
		return ( ( bool ) strstr ( strEsp8266_Fram_Record .Data_RX_BUF, reply1 ) ||  ( bool ) strstr ( strEsp8266_Fram_Record .Data_RX_BUF, reply2 ) ); 
	else if ( reply1 != 0 )
		return ( ( bool ) strstr ( strEsp8266_Fram_Record .Data_RX_BUF, reply1 ) );
	else
		return ( ( bool ) strstr ( strEsp8266_Fram_Record .Data_RX_BUF, reply2 ) );
}
******/
int ESP8266_Cmd(struct WIFI_Dev *dev, char *pCmd,char *ack,u16 waittime)
{
	int res = 0;
	if(dev == NULL)
		return -1;
	if(pCmd == NULL)
		return -1;
	
	dev->idleflag = 0;
	
	memset(dev->rec_buf,0,WIFI_RECBUFSIZE);
	
	while(*pCmd != '\0')
	{
		USART_SendData(WIFI_USART, *pCmd++);
		while(USART_GetFlagStatus(WIFI_USART,USART_FLAG_TXE) == RESET);
	}
	while(USART_GetFlagStatus(WIFI_USART,USART_FLAG_TC) == RESET)
	{
		__nop();
	}
	
	if(ack&&waittime)		//ÐèÒªµÈ´ýÓ¦´ð
	{
		while(--waittime)	//µÈ´ýµ¹¼ÆÊ±
		{
			SysTickDelay_ms(20);
			if(esp8266_check_cmd(dev,ack))
				break;//µÃµ½ÓÐÐ§Êý¾Ý 
			 
		}
		if(waittime==0)
			res = -1; 
	}
	return res;
}


int ESP8266_Write(struct WIFI_Dev *dev, u8 *pBuf, u16 wLen)
{
	u16 i = 0;
	for(i=0;i<wLen;i++)
	{
		USART_SendData(WIFI_USART, pBuf[i]);
		while(USART_GetFlagStatus(WIFI_USART,USART_FLAG_TXE) == RESET);
	}
	while(USART_GetFlagStatus(WIFI_USART,USART_FLAG_TC) == RESET);

	return 0;
}
int ESP8266_Init(struct WIFI_Dev *dev)
{
	WIFI_IOConfig();
	WIFI_UART();
	ConfigGPSDMARec();
	esp8266_hard_reset();
	SysTickDelay_ms(1000);
	USART_Cmd(WIFI_USART, ENABLE);
 
	dev->Open(dev);
	return 0;
	
}
 
int ESP8266_Open(struct WIFI_Dev *dev)
{
	u8 try_times = 5;
	SysTickDelay_ms(500);
	//AT+SLEEP?
	while(try_times--)
	{
		if(dev->SendCmd(dev,"AT\r\n\0","OK",2) == 0)
		{
			dev->wStatus = WIFI_GET_MAC;
			//return 0;
		}
		else
		{
			__nop();
		}
		if(dev->SendCmd(dev,"AT+SLEEP?\r\n\0","OK",2) == 0)
		{
			dev->wStatus = WIFI_GET_MAC;
			return 0;
		}
	}
	dev->wStatus = WIFI_HARDWARE_RST;
	return 1;
}
int ESP8266_Close(struct WIFI_Dev *dev,uint8_t bCloseMode)
{
	u8 try_times = 5;
	SysTickDelay_ms(500);
	while(try_times--)
	{
		if(dev->SendCmd(dev,"AT+CIPCLOSE\r\n\0","OK",2) == 0)
		{
			dev->wStatus = WIFI_GET_MAC;
			return 0;
		}
		else
		{
			__nop();
		}
	}
	return 1;	
}
int ESP8266_Read(struct WIFI_Dev *dev, u8 *pBuf, u16 *pwLen)
{
	
	return 0;
}
	 
//	int (*GetConnetStatus)(struct WIFI_Dev *dev);
int ESP8266_GetSignal(struct WIFI_Dev *dev,u8* dwVal)
{
	return 0;
}
int ESP8266_Reset(struct WIFI_Dev *dev, u8 hr_sr)
{
 
	
	switch(hr_sr)
	{
		case 0:
			// Ó²¼þÖØÆô
			esp8266_hard_reset();
			
			break;
		case 1:
			// at ÖØÆô
			dev->SendCmd(dev,"AT+RST\r\n\0","OK",2);
			SysTickDelay_ms(100);	
			break;
		case 2:
			// at »Ö¸´³ö³§ÉèÖÃ
			dev->SendCmd(dev,"AT+RESTORE\r\n\0","OK",2);
			SysTickDelay_ms(100);	
			break;
		default:
			break;
	}
	dev->wStatus =  WIFI_AT_TEST;
	return 0;
}
int ESP8266_GetStatus(struct WIFI_Dev *dev)
{
	return dev->wStatus;
}
	//int (*HandleRecv)(struct WIFI_Dev *dev, u8 *pBuf, u16 wLen);
int Wifi_Init(void)
{
	struct  WIFI_Dev *pDev = GetWifiDev();
//	Gprs_IOConfig();
// 
 
//	TIM4_Init(10);    // ÓÃÓÚ´®¿ÚÖÐ¶Ï ¶¨Ê±
//	TIM4_Set(0); 
	memset( pDev->rec_buf,0,WIFI_RECBUFSIZE); 
	

	pDev->Init 		= 			ESP8266_Init;
	pDev->Open 		= 			ESP8266_Open;
	pDev->Close 	= 			ESP8266_Close;
	pDev->SendCmd 	= 			ESP8266_Cmd;
	pDev->Read 		= 			ESP8266_Read;
	pDev->Write 	= 			ESP8266_Write;
	pDev->GetConnetStatus =		ESP8266_GetStatus;
	//pDev->GetSignal = ESP8266_GetSignal;
	pDev->ResetDev 	= 			ESP8266_Reset;
//	pDev->HandleRecv = 0;

	pDev->Init(pDev);
	
	return 0;
}


//static int Gprs_RecvHandler( unsigned char *data, u16 len)
//{
//	struct GPRS_DEV *pDev = GetGprsDev();
////	static char gprs_checkbuf[6];
////	uint8_t i;
//	u16 wReadPos = pDev->wReadPos;
////	T_MSG tMsg;
////	static u16 wDataLen = 0 ;
////	T_MSG_RECV *ptRecvMsg = (T_MSG_RECV *)&tMsg.Data[0];
//	//portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
//	
//	if(wReadPos > GPRSRECBUFDEEP)
//		pDev->wReadPos = 0;
//	if(len > GPRSRECBUFDEEP)
//	{
//		len  = 0;
//		return 0;
//	}
//	memcpy(&pDev->buf[wReadPos], data, len);
//	pDev->wReadPos ++;
//	TIM4_Set(1); 
//	return 0;	
//}
 


 
 static  int Search_SubArray(const uint8_t* src,uint16_t src_len,const uint8_t * search,uint16_t search_len)
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
				return 0;
			 
			}
		}
	}
	return -1;
	
}
