/*
    FreeRTOS V8.1.2 - Copyright (C) 2014 Real Time Engineers Ltd. 
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that has become a de facto standard.             *
     *                                                                       *
     *    Help yourself get started quickly and support the FreeRTOS         *
     *    project by purchasing a FreeRTOS tutorial book, reference          *
     *    manual, or both from: http://www.FreeRTOS.org/Documentation        *
     *                                                                       *
     *    Thank you!                                                         *
     *                                                                       *
    ***************************************************************************

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available from the following
    link: http://www.freertos.org/a00114.html

    1 tab == 4 spaces!

    ***************************************************************************
     *                                                                       *
     *    Having a problem?  Start by reading the FAQ "My application does   *
     *    not run, what could be wrong?"                                     *
     *                                                                       *
     *    http://www.FreeRTOS.org/FAQHelp.html                               *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org - Documentation, books, training, latest versions,
    license and Real Time Engineers Ltd. contact details.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.OpenRTOS.com - Real Time Engineers ltd license FreeRTOS to High
    Integrity Systems to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*
	BASIC INTERRUPT DRIVEN SERIAL PORT DRIVER FOR UART0.
*/

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "Sys_Init.h" 	
#include "zRTC_Timer.h"
#include "printf_embedded.h"
/* Library includes. */
//#include "stm32f10x_lib.h"

/* Demo application includes. */
#include "serial.h"
#include "btooth.h"
#include "serial_dma_rec.h"
#include "gps.h"
/*-----------------------------------------------------------*/

/* Misc defines. */
#define serINVALID_QUEUE				( ( QueueHandle_t ) 0 )
#define serNO_BLOCK						( ( TickType_t ) 0 )
#define serTX_BLOCK_TIME				( 40 / portTICK_PERIOD_MS )

/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

/* UART interrupt handler. */
void vUARTInterruptHandler( void );
extern RS485_RX_STRUCT gRS485RecData;
extern int RS485_DMARecvHandler(void);
/*-----------------------------------------------------------*/
/*-----------------------------------------------------------*/


UART_DEV g_tUartDev[serMaxPort];

UART_DEV *Uart_GetDev(u32 dwPort)
{
	if(dwPort == 0)
		return NULL;
	return &g_tUartDev[dwPort];
}

int Uart_Init(UART_DEV *dev, u32 dwPort)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;


  /* Set the Vector Table base location at 0x08000000 */
 //NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);

  /* 2 bit for pre-emption priority, 2 bits for subpriority */
	/* Configure the NVIC Preemption Priority Bits */  

	switch(dwPort)
	{
		case serCOM1:
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
			//USART1中断
			NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);  
			dev->base = USART1;
			//USART1 初始化=========================================================	
		    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;		        // USART1 TXD
		    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		    GPIO_Init(GPIOA, &GPIO_InitStructure); 
		    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;			    // USART1 RXD
		    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		    GPIO_Init(GPIOA, &GPIO_InitStructure);
			//485 方向控制
			GPIO_InitStructure.GPIO_Pin = UART485DIR_PIN;		        // USART1 TXD
		    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		    GPIO_Init(UART485DIR_PORT, &GPIO_InitStructure); 
			break;
		case serCOM2:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
			/* Enable the USART2 Interrupt */
			NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
				/* Enable the USART2 Interrupt */
			NVIC_Init(&NVIC_InitStructure);
			dev->base = USART2;
			//USART2_TX   PA.2
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
			GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化PA2
			
			//USART2_RX	  PA.3
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
			GPIO_Init(GPIOA, &GPIO_InitStructure);  //初始化PA3
	 
			
			break;
		case serCOM3:
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB| RCC_APB2Periph_AFIO, ENABLE);
			NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
 			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
 		 	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  			NVIC_Init(&NVIC_InitStructure); 
			dev->base = USART3;
			//USART3 初始化=========================================================	
		    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;		        // USART3 TXD
		    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		    GPIO_Init(GPIOB, &GPIO_InitStructure); 
		    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;			    // USART3 RXD
		    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		    GPIO_Init(GPIOB, &GPIO_InitStructure);	 				 
			break;
		 default:
           break;
	}
	
	//USART 初始化设置
	USART_DeInit(dev->base);  //复位串口

	USART_Init(dev->base, &dev->config); //初始化串口	

    USART_ClearFlag(dev->base,USART_FLAG_TC);
	if(dwPort ==  RS485_SERPORT) 
	{
		USART_ITConfig(dev->base, USART_IT_TC, DISABLE);
		USART_ITConfig(dev->base, USART_IT_RXNE, DISABLE);
		USART_ITConfig(dev->base, USART_IT_IDLE, ENABLE);
		USART_Cmd(dev->base, DISABLE); 
	}
	else
	{
		USART_ITConfig(dev->base, USART_IT_RXNE, ENABLE);
	}
//	USART_ITConfig( dev->base, USART_IT_TC, ENABLE );
	#if ENABLE_BLUETOOTH
	if(dwPort ==  BLE_USARTPORT) 
	{
		 
		USART_DMACmd(dev->base,USART_DMAReq_Tx,ENABLE);
		USART_Cmd(dev->base, DISABLE);   
	}
	else if(dwPort ==  RS485_SERPORT)
	{
		USART_Cmd(dev->base, DISABLE);
	}
	else
		USART_Cmd(dev->base, ENABLE);  
	#else
	if(dwPort ==  RS485_SERPORT)
	{
		USART_Cmd(dev->base, DISABLE);
	}
	else
		USART_Cmd(dev->base, ENABLE);  
	#endif
	#if ENABLE_GPS
	if(dwPort ==  GPS_SERCOM) 
	{
		USART_ITConfig(dev->base, USART_IT_TC, DISABLE);
		USART_ITConfig(dev->base, USART_IT_RXNE, DISABLE);
		USART_ITConfig(dev->base, USART_IT_IDLE, ENABLE);
		USART_Cmd(dev->base, DISABLE); 
	}
	#endif
	
	return 0;
}

int Uart_Open(UART_DEV *dev, u32 dwPort)
{

	return 0;
}
int Uart_Close(UART_DEV *dev, u32 dwPort)
{

	return 0;
}
int Uart_RecvEnable(UART_DEV *dev, u32 dwPort, u32 dwEnable)
{
	if(dwEnable)
		USART_Cmd(dev->base, ENABLE);
	else
		USART_Cmd(dev->base, DISABLE);
		
	return 0;
}


int Uart_PutChar(UART_DEV *dev, unsigned char cOutChar, TickType_t xBlockTime )
{
	if( xQueueSend( dev->SendQueueHdl, &cOutChar, xBlockTime ) == pdPASS )
	{
		//USART_ITConfig( dev->base, USART_IT_TC, ENABLE );//USART_IT_TC
		//USART_ClearFlag(dev->base, USART_FLAG_TC);
		//USART_ITConfig( dev->base, USART_IT_TC, ENABLE );
			USART_ITConfig( dev->base, USART_IT_TC, ENABLE );
		return 0;
	}
	else
	{
		return -1;
	}

}
int Uart_Write(UART_DEV *dev, const unsigned char *pcData, u16 wLen )
{
	int i = 0;

	if(dev->dwPort == RS485_SERPORT)
	{
		UART485Dir_TXD;
		__nop();
	//        wLen = wLen+1;
	}
	for(i=0; i<wLen; i++)
	{
		Uart_PutChar( dev, pcData[i], serNO_BLOCK );
	}

	return 0;
}
int Uart_RegDataProc(UART_DEV *dev, Uart_RecvHandle DataProcHandler)
{

	dev->RecvHandle = DataProcHandler;
	
	return 0;
}

int Serial_Init(u32 dwPort,u32 bandrate)
{
//	char  *pcString;
	UART_DEV *dev = Uart_GetDev(dwPort);

	dev->dwPort = dwPort;
//	Uart_Init(dev,dwPort );
	dev->Init = Uart_Init;
	dev->Open = Uart_Open;
	dev->Close = Uart_Close;
	dev->Write = Uart_Write;
	dev->RecvEn = Uart_RecvEnable;
	dev->RecvHandle = NULL;
	
	dev->SendQueueHdl =  xQueueCreate( 2304, ( unsigned portBASE_TYPE ) sizeof( signed char ) );
	dev->RecvQueueHdl =  xQueueCreate( 512, ( unsigned portBASE_TYPE ) sizeof( signed char ) );
	
	switch(dwPort)
	{
		case serCOM1:
						/*uart config*/
			dev->config.USART_BaudRate = bandrate;// 
			dev->config.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
			dev->config.USART_StopBits = USART_StopBits_1;//一个停止位
			dev->config.USART_Parity = USART_Parity_No;//无奇偶校验位
			dev->config.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
			dev->config.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
			break;
		case serCOM2:
			/*uart config*/
			dev->config.USART_BaudRate = bandrate;// 
			dev->config.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
			dev->config.USART_StopBits = USART_StopBits_1;//一个停止位
			dev->config.USART_Parity = USART_Parity_No;//无奇偶校验位
			dev->config.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
			dev->config.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

			break;
		case serCOM3:
			/*uart config*/
			dev->config.USART_BaudRate = bandrate;// 
			dev->config.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
			dev->config.USART_StopBits = USART_StopBits_1;//一个停止位
			dev->config.USART_Parity = USART_Parity_No;//无奇偶校验位
			dev->config.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
			dev->config.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

			break;
		default:
			break;
	}
	dev->Init(dev, dwPort);//串口init
	return 0;
}
int Serial_RecvEnable(u32 dwPort, u32 dwEnable)
{
	UART_DEV *dev = Uart_GetDev(dwPort);

	return dev->RecvEn(dev, dwPort, dwEnable);
	
}

int Serial_PutData(u32 dwPort, unsigned char *pcData, u16 wLen)
{
	UART_DEV *dev = Uart_GetDev(dwPort);

	dev->Write(dev, (const unsigned char* ) pcData, wLen);
	
	return 0;
}

int Serial_PutString(u32 dwPort,unsigned char *pcStr)
{
	UART_DEV *dev = Uart_GetDev(dwPort);
	unsigned char *pxNext;

	//USART2_TXDir_En;
	if(dev->dwPort == RS485_SERPORT)
	{
		UART485Dir_TXD;
		__nop();__nop();__nop();
	}
	/* Send each character in the string, one at a time. */
	pxNext = ( unsigned char *)pcStr;
	while(*pxNext)
	{
		Uart_PutChar(dev, *pxNext, serNO_BLOCK );
		pxNext++;
	}
	if(dev->dwPort == RS485_SERPORT)
	{
		Uart_PutChar(dev, 0x55, serNO_BLOCK );
	}
	return 0;
}

int Serial_RegRecvHandler(u32 dwPort, Uart_RecvHandle pRecvHdl)
{
	UART_DEV *dev = Uart_GetDev(dwPort);
	
	if(dev != NULL)
  		Uart_RegDataProc(dev, pRecvHdl);

	return 0;
}

void UART_InterruptHandler(u32 dwPort)
{
	UART_DEV *dev = Uart_GetDev(dwPort);
	struct GPS_DEV  *Gps_dev = GetGpsDev();
	USART_TypeDef *USART = dev->base;
	QueueHandle_t pRxedChars = dev->RecvQueueHdl;
	QueueHandle_t pCharsForTx = dev->SendQueueHdl;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	unsigned char uChar;
	unsigned char temp;
//    DateTime    tDTime;

	if( USART_GetITStatus( USART, USART_IT_TC ) == SET )	//USART_IT_TC//
	{
		/* The interrupt was caused by the THR becoming empty.  Are there any
		more characters to transmit? */
		if( xQueueReceiveFromISR( pCharsForTx, &uChar, &xHigherPriorityTaskWoken ) == pdTRUE )
		{
			/* A character was retrieved from the queue so can be sent to the
			THR now. */		 
			//while(USART_GetITStatus( USART, USART_IT_TC ));
		
			USART_SendData( USART, uChar );
		}
		else
		{
			USART_ITConfig( USART, USART_IT_TC, DISABLE );		
			__nop();__nop();__nop();			
			//USART_ITConfig( USART, USART_IT_TC, DISABLE );
			if(dev->dwPort == RS485_SERPORT)/*485 com rx default*/
			{
 
				UART485Dir_RXD;
			}
		}		
	}	
	if( USART_GetITStatus( USART, USART_IT_RXNE ) == SET )
	{
		uChar = USART_ReceiveData( USART );
		//xQueueSendFromISR( pRxedChars, &cChar, &xHigherPriorityTaskWoken );
		if(dev->RecvHandle)
			dev->RecvHandle(&uChar, 1);
	}
	if(USART_GetITStatus(USART, USART_IT_IDLE) != RESET)  
    {  
		if(USART == RS485_USART) 
		{
        //USART_ClearFlag(USART1,USART_IT_IDLE);  
			temp = USART1->SR;  
			temp = USART1->DR; //?USART_IT_IDLE?? 
			USART_ClearITPendingBit(USART, USART_IT_IDLE);		
			DMA_Cmd(RS485_RX_DMACHANNEL,DISABLE);  
	  
			gRS485RecData.wRecLen = RS485_RXBUFSIZE - DMA_GetCurrDataCounter(RS485_RX_DMACHANNEL);  
		  
			if(gRS485RecData.wRecLen > HEAD_SIZE)
			{
				RS485_DMARecvHandler();

			}           
			DMA_SetCurrDataCounter(RS485_RX_DMACHANNEL,RS485_RXBUFSIZE);  
          
			DMA_Cmd(RS485_RX_DMACHANNEL,ENABLE); 
		}
		if(USART ==GPS_USART )
		{
			
			temp = USART2->SR;  
			temp = USART2->DR; //?USART_IT_IDLE?? 
			USART_ClearITPendingBit(USART, USART_IT_IDLE);		
			DMA_Cmd(GPS_DMA_CHANNEL,DISABLE);  
			Gps_dev->wLen = GPS_DATA_MAX_LEN - DMA_GetCurrDataCounter(GPS_DMA_CHANNEL);  
			if(Gps_dev->wLen > 0)
			{
				GPS_DMARecvHandler(); 
			}
			Gps_dev->wLen		 =  0;
			DMA_SetCurrDataCounter(GPS_DMA_CHANNEL,GPS_DATA_MAX_LEN);  
			DMA_Cmd(GPS_DMA_CHANNEL,ENABLE);  
		} 
    }
	
    if(USART_GetITStatus(USART, USART_IT_RXNE) != RESET)
    {
        /* Clear the USART Receive interrupt */
        USART_ClearITPendingBit(USART, USART_IT_RXNE);

    }
	if( USART_GetITStatus( USART, USART_FLAG_ORE| USART_FLAG_LBD|USART_FLAG_NE|USART_FLAG_FE) == SET )
	{
		USART_ClearITPendingBit(USART, USART_FLAG_ORE| USART_FLAG_LBD|USART_FLAG_NE|USART_FLAG_FE);
	}
	if(USART_GetFlagStatus(USART,USART_FLAG_ORE| USART_FLAG_LBD|USART_FLAG_NE|USART_FLAG_FE) == SET)
	{
		USART_ClearFlag(USART,USART_FLAG_ORE| USART_FLAG_LBD|USART_FLAG_NE|USART_FLAG_FE);		
	}  	
	//USART_ClearITPendingBit(USART,USART_IT_RXNE);//w
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );

}

 
void USART1_IRQHandler( void )
{
	UART_InterruptHandler(serCOM1);

} 

void USART2_IRQHandler( void )
{
	UART_InterruptHandler(serCOM2);

}

void USART3_IRQHandler( void )
{
	UART_InterruptHandler(serCOM3);

}

