#include "stm32f10x.h"
#include "msg.h"
#include "bsp_serial.h"
#define BSP_USART  				USART1
#define BSP_USART_DMA_CHANNEL	DMA1_Channel5
#define BSP_RECBUFSIZE			64
//u8 bsp_serial_buf[BSP_RECBUFSIZE];


struct BSP_Ser 
{
	//u8 *buf;
	u8 	bsp_serial_buf[BSP_RECBUFSIZE];
	
	u16 idleflag;
	u16 wLen;
 	//u16 wStatus; 
};
struct BSP_Ser tBspSer;
void bsp_serial_dma(void);


void bsp_serial_config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_USART1|RCC_APB2Periph_AFIO , ENABLE );
	 
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode=  GPIO_Mode_IN_FLOATING; //GPIO_Mode_IPU
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	/********
	*波特率 
	************/
	USART_InitStructure.USART_BaudRate            = 9600  ;    
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;
	USART_InitStructure.USART_Parity              = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(BSP_USART, &USART_InitStructure);
	//USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
	
	USART_ITConfig(BSP_USART, USART_IT_TC, DISABLE);
	USART_ITConfig(BSP_USART, USART_IT_RXNE, DISABLE);
	USART_ITConfig(BSP_USART, USART_IT_IDLE, ENABLE);
	USART_ClearFlag(BSP_USART,USART_FLAG_TC);
			
	bsp_serial_dma();
	
	
}
void bsp_serial_dma(void)
{
	DMA_InitTypeDef DMA_InitStructure; 
	//NVIC_InitTypeDef NVIC_InitStructure;
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 , ENABLE); 
 
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
//	 
//	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
	 
	DMA_DeInit(BSP_USART_DMA_CHANNEL);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART1->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) tBspSer.bsp_serial_buf ;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = BSP_RECBUFSIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(BSP_USART_DMA_CHANNEL,&DMA_InitStructure);

	DMA_Cmd(BSP_USART_DMA_CHANNEL,ENABLE);
	//USART_DMACmd(GPS_USART,USART_DMAReq_Tx,ENABLE);
	USART_DMACmd(BSP_USART,USART_DMAReq_Rx,ENABLE);
    
	USART_Cmd(BSP_USART, ENABLE);
}


void USART1_IRQHandler(void)
{
 	u8 ucDat;
	u16 temp;
    /* 中断接收*/
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) 
    { 
		ucDat = USART_ReceiveData(USART1);   
//		g_Usart3RecStr.buf[g_Usart3RecStr.wWritePos++] = ucDat;
//		if(g_Usart3RecStr.wWritePos == USART3_BUFSIZE)
//		{
//			g_Usart3RecStr.wWritePos = 0;
//			g_Usart3RecStr.wRecDataFlag = (g_Usart3RecStr.wRecDataFlag+1) &0xff;
//			
//		}
//		TIM4_Set(1); 
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);        
    }	
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)  
    { 
		temp = USART1->SR;  
		temp = USART1->DR; //?USART_IT_IDLE?? 
		USART_ClearITPendingBit(USART1, USART_IT_IDLE);		
		DMA_Cmd(BSP_USART_DMA_CHANNEL,DISABLE);  
		tBspSer.wLen = BSP_RECBUFSIZE - DMA_GetCurrDataCounter(BSP_USART_DMA_CHANNEL); 
		tBspSer.idleflag = 1;
		DMA_SetCurrDataCounter(BSP_USART_DMA_CHANNEL,BSP_RECBUFSIZE);  
		DMA_Cmd(BSP_USART_DMA_CHANNEL,ENABLE);  
    }
	
}

void ProcessBspSer(void)
{
	T_MSG tMsg; 
    T_MSG_RECV *ptRecvMsg = (T_MSG_RECV *)&tMsg.Data[0];
	 
	if(tBspSer.idleflag )
	{
		tMsg.wMsgType = MSG_TYPE_SER;
		
		 
		
	}
		
}
