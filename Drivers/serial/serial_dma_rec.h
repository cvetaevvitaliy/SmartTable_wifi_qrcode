#ifndef __SERIAL_DAM_REC_H
#define __SERIAL_DAM_REC_H
#include "stm32f10x.h"
#include "stm32f10x_dma.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#define RS485_RXBUFSIZE  128
#define RS485_RX_DMACHANNEL DMA1_Channel5
#define RS485_USART   USART1
typedef struct 
{
	xSemaphoreHandle xDataRxMutex;
	u16 wRecLen;
	u8 RecBuf[RS485_RXBUFSIZE];
	
	
}RS485_RX_STRUCT;

void ConfigRS485Rec(void);
void BSP_RS485_Config(void);
#endif
