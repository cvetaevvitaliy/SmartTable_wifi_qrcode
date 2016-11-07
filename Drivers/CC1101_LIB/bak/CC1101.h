/*
================================================================================
Copyright   : Ebyte electronic co.,LTD
Website     : http://yh-ebyte.taobao.com
              http://yiheliyong.cn.alibaba.com
Description : This module contains the low level operations for CC1101
================================================================================
*/
#ifndef __CC1101_H
#define __CC1101_H

 
#include "CC1101_REG.h"		//DO NOT modify

/*
================================================================================
------------------------------Internal IMPORT functions-------------------------
you must offer the following functions for this module
1. u8 SPI_ExchangeByte( u8 input ); //SPI Send and Receive function
2. CC_CSN_LOW( );                        //Pull down the CSN line
3. CC_CSN_HIGH( );                       //Pull up the CSN Line
================================================================================
*/
 
#define CC1101_CS_PORT			GPIOA
#define CC1101_CS_PIN       	GPIO_Pin_4  //
#define CC1101_CLK_PIN       	GPIO_Pin_5
#define CC1101_MOSI_PIN       	GPIO_Pin_7 
#define CC1101_MISO_PIN       	GPIO_Pin_6
#define CC1101_GOD_PORT			GPIOB
#define CC1101_GOD0_PIN       	GPIO_Pin_11
#define CC1101_GOD2_PIN       	GPIO_Pin_0

//#define CC1101_CS_PIN_OUT
#define CC1101_CS_L       	 GPIO_ResetBits(CC1101_CS_PORT, CC1101_CS_PIN) // Add your code here.
#define CC1101_CS_H     	 GPIO_SetBits(CC1101_CS_PORT, CC1101_CS_PIN)  // Add your code here.


#define CC1101_MOSI_H 		GPIO_SetBits(CC1101_CS_PORT, CC1101_MOSI_PIN)  
#define CC1101_MOSI_L 		GPIO_ResetBits(CC1101_CS_PORT, CC1101_MOSI_PIN)  
#define CC1101_SCLK_H 		GPIO_SetBits(CC1101_CS_PORT, CC1101_CLK_PIN)  
#define CC1101_SCLK_L 		GPIO_ResetBits(CC1101_CS_PORT, CC1101_CLK_PIN)  
#define CC1101_READMISO 	GPIO_ReadInputDataBit(CC1101_CS_PORT, CC1101_MISO_PIN) 
#define CC1101_MISO_H		GPIO_SetBits(CC1101_CS_PORT, CC1101_MISO_PIN) 
#define CC1101_MISO_L		GPIO_ResetBits(CC1101_CS_PORT, CC1101_MISO_PIN) 
#define CC1101_DRY			GPIO_ReadInputDataBit(CC1101_CS_PORT, CC1101_MISO_PIN)		//

#define CC1101_READGOD2		GPIO_ReadInputDataBit(CC1101_GOD_PORT,CC1101_GOD2_PIN )		//
#define CC1101_READGOD0		GPIO_ReadInputDataBit(CC1101_GOD_PORT,CC1101_GOD0_PIN )		//

 
 
#define CC_CSN_LOW( )   GPIO_ResetBits( CC1101_CS_PORT, CC1101_CS_PIN );while( CC1101_READMISO != 0);
#define CC_CSN_HIGH( )  GPIO_SetBits( CC1101_CS_PORT, CC1101_CS_PIN )


/*
================================================================================
-----------------------------------macro definitions----------------------------
================================================================================
*/
typedef enum { TX_MODE, RX_MODE }TRMODE;

void halRfWriteRfSettings(void) ;
/*read a byte from the specified register*/
u8 CC1101ReadReg( u8 addr );

/*Read some bytes from the rigisters continously*/
void CC1101ReadMultiReg( u8 addr, u8 *buff, u8 size );

/*Read a status register*/
u8 CC1101ReadStatus( u8 addr );

/*Set the device as TX mode or RX mode*/
void CC1101SetTRMode( TRMODE mode );

/*Write a byte to the specified register*/
void CC1101WriteReg( u8 addr, u8 value );

/*Write some bytes to the specified register*/
void CC1101WriteMultiReg( u8 addr, u8 *buff, u8 size );

/*Write a command byte to the device*/
void CC1101WriteCmd( u8 command );

/*Reset the CC1101 device*/
void CC1101Reset( void );

/*Set the CC1101 into IDLE mode*/
void CC1101SetIdle( void );

/*Flush the TX buffer of CC1101*/
void CC1101ClrTXBuff( void );

/*Flush the RX buffer of CC1101*/
void CC1101ClrRXBuff( void );

/*Send a packet*/
void CC1101SendPacket( u8 *txbuffer, u8 size );

/*Get received count of CC1101*/
u8 CC1101GetRXCnt( void );

/*Receive a packet*/
u8 CC1101RecPacket( u8 *rxBuffer, u8 *length );


void CC1101Init( void );
void CC1101_test(void);

#endif // _CC1101_H_
/*
================================================================================
------------------------------------THE END-------------------------------------
================================================================================
*/
