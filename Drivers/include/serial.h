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

#ifndef __SERIAL_H
#define __SERIAL_H

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "queue.h"

typedef void * xComPortHandle;


typedef enum
{ 
	serCOM0,
	serCOM1, 
	serCOM2, 
	serCOM3, 
	serCOM4, 
	serCOM5,
	serMaxPort,
} eCOMPort;

typedef enum 
{ 
	serNO_PARITY, 
	serODD_PARITY, 
	serEVEN_PARITY, 
	serMARK_PARITY, 
	serSPACE_PARITY 
} eParity;

typedef enum 
{ 
	serSTOP_1, 
	serSTOP_2 
} eStopBits;

typedef enum 
{ 
	serBITS_5, 
	serBITS_6, 
	serBITS_7, 
	serBITS_8 
} eDataBits;

typedef enum 
{ 
	ser50,		
	ser75,		
	ser110,		
	ser134,		
	ser150,    
	ser200,
	ser300,		
	ser600,		
	ser1200,	
	ser1800,	
	ser2400,   
	ser4800,
	ser9600,		
	ser19200,	
	ser38400,	
	ser57600,	
	ser115200
} eBaud;

typedef int (*Uart_RecvHandle)( unsigned char *data, u16 len);

typedef struct _UART_DEV
{
	USART_TypeDef *base;
	USART_InitTypeDef config;
	
	u32 dwPort;

	QueueHandle_t RecvQueueHdl;
	QueueHandle_t SendQueueHdl;

	int (*Init)(struct _UART_DEV *dev, u32 dwPort);
	int (*Open)(struct _UART_DEV *dev, u32 dwPort);
	int (*Close)(struct _UART_DEV *dev, u32 dwPort);
 	int (*Write)(struct _UART_DEV *dev, const unsigned char * const pcString, u16 usStringLength );	
	int (*RecvEn)(struct _UART_DEV *dev, u32 dwPort, u32 dwEn);
	Uart_RecvHandle RecvHandle;
	
}UART_DEV;

//xComPortHandle xSerialPortInitMinimal( unsigned long ulWantedBaud, unsigned portBASE_TYPE uxQueueLength );
//xComPortHandle xSerialPortInit( eCOMPort ePort, eBaud eWantedBaud, eParity eWantedParity, eDataBits eWantedDataBits, eStopBits eWantedStopBits, unsigned portBASE_TYPE uxBufferLength );
//void vSerialPutString( xComPortHandle pxPort, const signed char * const pcString, unsigned short usStringLength );
//signed portBASE_TYPE xSerialGetChar( xComPortHandle pxPort, signed char *pcRxedChar, TickType_t xBlockTime );
//signed portBASE_TYPE xSerialPutChar( xComPortHandle pxPort, signed char cOutChar, TickType_t xBlockTime );
//portBASE_TYPE xSerialWaitForSemaphore( xComPortHandle xPort );
//void vSerialClose( xComPortHandle xPort );

int Serial_PutString(u32 dwPort, unsigned char *pcStr);
int Serial_PutData(u32 dwPort, unsigned char *pcData, u16 wLen);
int Serial_RegRecvHandler(u32 dwPort, Uart_RecvHandle pRecvHdl);
int Serial_RecvEnable(u32 dwPort, u32 dwEnable);

#define USART2_TXDir_En		 //GPIO_SetBits (GPIOB, GPIO_Pin_9)
#define USART2_RXDir_En		 //GPIO_ResetBits(GPIOB, GPIO_Pin_9)
#define USART3_TXDir_En		 //GPIO_SetBits (GPIOB, GPIO_Pin_8)
#define USART3_RXDir_En		 //GPIO_ResetBits(GPIOB, GPIO_Pin_8)
#define UART485DIR_PORT   	GPIOD
#define UART485DIR_PIN		GPIO_Pin_8
#define UART485Dir_TXD 	GPIO_SetBits(UART485DIR_PORT,UART485DIR_PIN)
#define UART485Dir_RXD 	GPIO_ResetBits(UART485DIR_PORT,UART485DIR_PIN)
int Serial_Init(u32 dwPort,u32 bandrate);
int Uart_Write(UART_DEV *dev, const unsigned char *pcData, u16 wLen );

#endif

