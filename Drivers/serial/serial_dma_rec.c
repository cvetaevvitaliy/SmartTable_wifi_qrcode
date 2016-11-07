#include "stm32f10x.h"

 

#include "serial.h"
#include "sys_init.h"
#include "serial_dma_rec.h"
RS485_RX_STRUCT gRS485RecData;


/**********************************************************************************
Func    Name:
Descriptions:
Input   para:
In&Out  Para:
Output  para:
Return value:
Others  :   
***********************************************************************************/
void ConfigRS485Rec(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	//NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 , ENABLE); 
 
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
//	 
//	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
	 
	DMA_DeInit(RS485_RX_DMACHANNEL);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART1->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)gRS485RecData.RecBuf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = RS485_RXBUFSIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(RS485_RX_DMACHANNEL,&DMA_InitStructure);

	DMA_Cmd(RS485_RX_DMACHANNEL,ENABLE);
	//USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
    USART_Cmd(USART1, ENABLE); 
	
}


void BSP_RS485_Config(void)
{
	Serial_Init(RS485_SERPORT,9600); 
	 
//	Serial_RegRecvHandler(RS485_SERPORT, RS485_RecvHandler);
	//Serial_RecvEnable(RS485_SERPORT,ENABLE);
	gRS485RecData.xDataRxMutex = xSemaphoreCreateMutex();
	xSemaphoreGive(gRS485RecData.xDataRxMutex);
	ConfigRS485Rec();
	
}