/*
================================================================================
Copyright   : Ebyte electronic co.,LTD
Website     : http://yh-ebyte.taobao.com
              http://yiheliyong.cn.alibaba.com
Description : This module contains the low level operations for CC1101
================================================================================
*/
#include "stm32f10x.h"
#include "CC1101.h"

#define halRfWriteReg CC1101WriteReg
#define TI_CC_SPIWriteReg CC1101WriteReg
#define CC1101_DELAY  spi_delay(2); 
 
 
  u8 Channel_num = 0x01;

static void spi_delay (__IO uint32_t nDelay)
{
	while (nDelay--);
}
 
void SPI_CC1101_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA , ENABLE);
	
	
	GPIO_InitStructure.GPIO_Pin = CC1101_CLK_PIN | CC1101_CS_PIN | CC1101_MOSI_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(CC1101_CS_PORT, &GPIO_InitStructure);
	
	
	GPIO_InitStructure.GPIO_Pin = CC1101_MISO_PIN ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING ;   //GPIO_Mode_IPU
	GPIO_Init(CC1101_CS_PORT, &GPIO_InitStructure);
	
	/* Configure I/O for PC4,PC5  */
	GPIO_InitStructure.GPIO_Pin = CC1101_GOD0_PIN| CC1101_GOD2_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(CC1101_GOD_PORT, &GPIO_InitStructure);
	/* Deselect the FLASH: Chip Select high */
	// SPI_FLASH_CS_HIGH();

 
}
void halRfWriteRfSettings(void) 
{
	// Write register settings, 433m ,250k
   /*  TI_CC_SPIWriteReg(CCxxx0_IOCFG2,   0x06); // GDO2 output pin config.
    TI_CC_SPIWriteReg(CCxxx0_IOCFG0,   0x06); // GDO0 output pin config.
    TI_CC_SPIWriteReg(CCxxx0_PKTLEN,   0xFF); // Packet length.
    TI_CC_SPIWriteReg(CCxxx0_PKTCTRL1, 0x05); // Packet automation control.
    TI_CC_SPIWriteReg(CCxxx0_PKTCTRL0, 0x05); // Packet automation control.
    TI_CC_SPIWriteReg(CCxxx0_ADDR,     0x01); // Device address.
    TI_CC_SPIWriteReg(CCxxx0_CHANNR,   0x00); // Channel number.
    TI_CC_SPIWriteReg(CCxxx0_FSCTRL1,  0x0B); // Freq synthesizer control.
    TI_CC_SPIWriteReg(CCxxx0_FSCTRL0,  0x00); // Freq synthesizer control.
    TI_CC_SPIWriteReg(CCxxx0_FREQ2,    0x10); // Freq control word, high byte
    TI_CC_SPIWriteReg(CCxxx0_FREQ1,    0xA7); // Freq control word, mid byte.
    TI_CC_SPIWriteReg(CCxxx0_FREQ0,    0x62); // Freq control word, low byte.
    TI_CC_SPIWriteReg(CCxxx0_MDMCFG4,  0x2D); // Modem configuration.
    TI_CC_SPIWriteReg(CCxxx0_MDMCFG3,  0x3B); // Modem configuration.
    TI_CC_SPIWriteReg(CCxxx0_MDMCFG2,  0x73); // Modem configuration.
    TI_CC_SPIWriteReg(CCxxx0_MDMCFG1,  0x22); // Modem configuration.
    TI_CC_SPIWriteReg(CCxxx0_MDMCFG0,  0xF8); // Modem configuration.
    TI_CC_SPIWriteReg(CCxxx0_DEVIATN,  0x00); // Modem dev (when FSK mod en)
    TI_CC_SPIWriteReg(CCxxx0_MCSM1 ,   0x3F); //MainRadio Cntrl State Machine
    TI_CC_SPIWriteReg(CCxxx0_MCSM0 ,   0x18); //MainRadio Cntrl State Machine
    TI_CC_SPIWriteReg(CCxxx0_FOCCFG,   0x1D); // Freq Offset Compens. Config
    TI_CC_SPIWriteReg(CCxxx0_BSCFG,    0x1C); //  Bit synchronization config.
    TI_CC_SPIWriteReg(CCxxx0_AGCCTRL2, 0xC7); // AGC control.
    TI_CC_SPIWriteReg(CCxxx0_AGCCTRL1, 0x00); // AGC control.
    TI_CC_SPIWriteReg(CCxxx0_AGCCTRL0, 0xB2); // AGC control.
    TI_CC_SPIWriteReg(CCxxx0_FREND1,   0xB6); // Front end RX configuration.
    TI_CC_SPIWriteReg(CCxxx0_FREND0,   0x10); // Front end RX configuration.
    TI_CC_SPIWriteReg(CCxxx0_FSCAL3,   0xEA); // Frequency synthesizer cal.
    TI_CC_SPIWriteReg(CCxxx0_FSCAL2,   0x0A); // Frequency synthesizer cal.
    TI_CC_SPIWriteReg(CCxxx0_FSCAL1,   0x00); // Frequency synthesizer cal.
    TI_CC_SPIWriteReg(CCxxx0_FSCAL0,   0x11); // Frequency synthesizer cal.
    TI_CC_SPIWriteReg(CCxxx0_FSTEST,   0x59); // Frequency synthesizer cal.
    TI_CC_SPIWriteReg(CCxxx0_TEST2,    0x88); // Various test settings.
    TI_CC_SPIWriteReg(CCxxx0_TEST1,    0x31); // Various test settings.
    TI_CC_SPIWriteReg(CCxxx0_TEST0,    0x0B); // Various test settings. */
//--------------------------------------------------------------------------	
	//
	// Rf settings for CC1101, 433M, 2.4KBPS
	//
        TI_CC_SPIWriteReg(CCxxx0_IOCFG2,0x06);  //GDO2 Output Pin Configuration
	TI_CC_SPIWriteReg(CCxxx0_IOCFG0,0x06);  //GDO0 Output Pin Configuration
	TI_CC_SPIWriteReg(CCxxx0_FIFOTHR,0x47); //RX FIFO and TX FIFO Thresholds
	TI_CC_SPIWriteReg(CCxxx0_PKTCTRL0,0x05);//Packet Automation Control
        
    TI_CC_SPIWriteReg(CCxxx0_CHANNR,  Channel_num); // Channel number.
        
	TI_CC_SPIWriteReg(CCxxx0_FSCTRL1,0x06); //Frequency Synthesizer Control
	/****433Mhz*********/
        TI_CC_SPIWriteReg(CCxxx0_FREQ2,0x10);   //Frequency Control Word, High Byte
	TI_CC_SPIWriteReg(CCxxx0_FREQ1,0xB1);   //Frequency Control Word, Middle Byte
	TI_CC_SPIWriteReg(CCxxx0_FREQ0,0x3B);   //Frequency Control Word, Low Byte
//        /***** 420 Mhz ******/
//        TI_CC_SPIWriteReg(CCxxx0_FREQ2,0x10);   //Frequency Control Word, High Byte
//	TI_CC_SPIWriteReg(CCxxx0_FREQ1,0x27);   //Frequency Control Word, Middle Byte
//	TI_CC_SPIWriteReg(CCxxx0_FREQ0,0x62);   //Frequency Control Word, Low Byte
//        
	TI_CC_SPIWriteReg(CCxxx0_MDMCFG4,0xF6); //Modem Configuration
	TI_CC_SPIWriteReg(CCxxx0_MDMCFG3,0x83); //Modem Configuration
	TI_CC_SPIWriteReg(CCxxx0_MDMCFG2,0x13); //Modem Configuration
        TI_CC_SPIWriteReg(CCxxx0_MDMCFG1,  0x23);
        TI_CC_SPIWriteReg(CCxxx0_MDMCFG0,  0x7A);
        
	TI_CC_SPIWriteReg(CCxxx0_DEVIATN,0x15); //Modem Deviation Setting
	TI_CC_SPIWriteReg(CCxxx0_MCSM0,0x18);   //Main Radio Control State Machine Configuration
	TI_CC_SPIWriteReg(CCxxx0_FOCCFG,0x16);  //Frequency Offset Compensation Configuration
	TI_CC_SPIWriteReg(CCxxx0_WORCTRL,0xFB); //Wake On Radio Control
	TI_CC_SPIWriteReg(CCxxx0_FSCAL3,0xE9);  //Frequency Synthesizer Calibration
	TI_CC_SPIWriteReg(CCxxx0_FSCAL2,0x2A);  //Frequency Synthesizer Calibration
	TI_CC_SPIWriteReg(CCxxx0_FSCAL1,0x00);  //Frequency Synthesizer Calibration
	TI_CC_SPIWriteReg(CCxxx0_FSCAL0,0x1F);  //Frequency Synthesizer Calibration
	TI_CC_SPIWriteReg(CCxxx0_TEST2,0x81);   //Various Test Settings
	TI_CC_SPIWriteReg(CCxxx0_TEST1,0x35);   //Various Test Settings
	TI_CC_SPIWriteReg(CCxxx0_TEST0,0x09);   //Various Test Settings

}

/*Write a command byte to the device*/
void CC1101WriteCmd( u8 command );
 
//------------------------------------------------/
//函数功能:通过SPI写读一字节                     /
//输入参数:value                                 /
//输出参数:B_value                               /
//-----------------------------------------------/
//u8 SPI_WR_Byte(u8 value)
u8 SPI_ExchangeByte(u8 value)
{
	u8 i,B_value=0;
	CC1101_SCLK_L;
	CC1101_DELAY;
	for(i=0;i<8;i++)
	{
		if(value&0x80)
			CC1101_MOSI_H;
		else
			CC1101_MOSI_L;
		value<<=1;
		CC1101_SCLK_H;
		CC1101_DELAY;
		B_value<<=1;
		if(CC1101_READMISO)
			B_value|=0x01;
		else
			__nop();
		CC1101_SCLK_L;
		CC1101_DELAY;

	}
	return B_value;
}
/*
 
================================================================================
Function : CC1101ReadReg( )
    read a byte from the specified register
INPUT    : addr, The address of the register
OUTPUT   : the byte read from the rigister
================================================================================
*/
u8 CC1101ReadReg( u8 addr )
{
	u8 i;
	CC_CSN_LOW;
	CC1101_DELAY;
	while (CC1101_DRY );
	SPI_ExchangeByte( addr | READ_SINGLE);
	CC1101_DELAY;
	i = SPI_ExchangeByte( 0xFF );
	CC1101_DELAY;
	CC_CSN_HIGH ;
	return i;
}
/*
================================================================================
Function : CC1101ReadMultiReg( )
    Read some bytes from the rigisters continously
INPUT    : addr, The address of the register
		   buff, The buffer stores the data
		   size, How many bytes should be read
OUTPUT   : None
================================================================================
*/
void CC1101ReadMultiReg( u8 addr, u8 *buff, u8 size )
{
	u8 i, j;
	CC_CSN_LOW;
	CC1101_DELAY;
	SPI_ExchangeByte( addr | READ_BURST);
	CC1101_DELAY;
	for( i = 0; i < size; i ++ )
	{
		for( j = 0; j < 20; j ++ );
		*( buff + i ) = SPI_ExchangeByte( 0xFF );
	}
	CC1101_DELAY;
	CC_CSN_HIGH;
}
/*
================================================================================
Function : CC1101ReadStatus( )
    Read a status register
INPUT    : addr, The address of the register
OUTPUT   : the value read from the status register
================================================================================
*/
u8 CC1101ReadStatus( u8 addr )
{
	u8 i;
	CC_CSN_LOW;
	CC1101_DELAY;
	SPI_ExchangeByte( addr | READ_BURST);
	CC1101_DELAY;
	i = SPI_ExchangeByte( 0xFF );
	CC1101_DELAY;
	CC_CSN_HIGH;
	return i;
}
/*
================================================================================
Function : CC1101SetTRMode( )
    Set the device as TX mode or RX mode
INPUT    : mode selection
OUTPUT   : None
================================================================================
*/
void CC1101SetTRMode( TRMODE mode )
{
	if( mode == TX_MODE ) 		
	{ 
		CC1101WriteCmd( CCxxx0_STX ); 
	}
	else if( mode == RX_MODE )	
	{ 
		CC1101WriteCmd( CCxxx0_SRX ); 
	} 
}
/*
================================================================================
Function : CC1101WriteReg( )
    Write a byte to the specified register
INPUT    : addr, The address of the register
		   value, the byte you want to write
OUTPUT   : None
================================================================================
*/
void CC1101WriteReg( u8 addr, u8 value )
{
	CC_CSN_LOW;
	CC1101_DELAY;
	SPI_ExchangeByte( addr );
	CC1101_DELAY;
	SPI_ExchangeByte( value );
	CC1101_DELAY;
	CC_CSN_HIGH;
}
/*
================================================================================
Function : CC1101WriteMultiReg( )
    Write some bytes to the specified register
INPUT    : addr, The address of the register
		   buff, a buffer stores the values
		   size, How many byte should be written
OUTPUT   : None
================================================================================
*/
void CC1101WriteMultiReg( u8 addr, u8 *buff, u8 size )
{
	u8 i;
	CC_CSN_LOW;
	CC1101_DELAY;
 
	SPI_ExchangeByte( addr | WRITE_BURST );
	CC1101_DELAY;
	for( i = 0; i < size; i ++ )
	{
		SPI_ExchangeByte( *( buff + i ) );	
	}
	CC1101_DELAY;
	CC_CSN_HIGH;
}
/*
================================================================================
Function : CC1101WriteCmd( )
    Write a command byte to the device
INPUT    : command, the byte you want to write
OUTPUT   : None
================================================================================
*/
void CC1101WriteCmd( u8 command )
{
	CC_CSN_LOW;
	CC1101_DELAY;
	SPI_ExchangeByte( command );
	CC1101_DELAY;
	CC_CSN_HIGH;
}
/*
================================================================================
Function : CC1101Reset( )
    Reset the CC1101 device
INPUT    : None
OUTPUT   : None
================================================================================
*/
void CC1101Reset( void )
{
 	CC_CSN_LOW;
	while(CC1101_DRY );	
	CC1101WriteCmd( CCxxx0_SRES );	//CCxxx0_SRES=0x30
	while(CC1101_DRY  );
	CC1101_CS_H;
	CC1101_DELAY;
}
 
//*****************************************************************************************
 
//*****************************************************************************************
void POWER_UP_RESET_CC1100(void)
{
	 
	CC1101_CS_H;
	spi_delay(2);
	CC_CSN_LOW;
	spi_delay(4);
	CC1101_CS_H;
	spi_delay(82);
	CC1101Reset();                   //??CC1100
}
/*
================================================================================
Function : CC1101SetIdle( )
    Set the CC1101 into IDLE mode
INPUT    : None
OUTPUT   : None
================================================================================
*/
void CC1101SetIdle( void )
{
    CC1101WriteCmd(CCxxx0_SIDLE);
}
/*
================================================================================
Function : CC1101ClrTXBuff( )
    Flush the TX buffer of CC1101
INPUT    : None
OUTPUT   : None
================================================================================
*/
void CC1101ClrTXBuff( void )
{
    CC1101WriteCmd( CCxxx0_SFTX );    //CC1101SetIdle();//  
}
/*
================================================================================
Function : CC1101ClrRXBuff( )
    Flush the RX buffer of CC1101
INPUT    : None
OUTPUT   : None
================================================================================
*/
void CC1101ClrRXBuff( void )
{
        CC1101WriteCmd( CCxxx0_SFRX );//CC1101SetIdle();// 
}
/*
================================================================================
Function : CC1101SendPacket( )
    Send a packet
INPUT    : txbuffer, The buffer stores data to be sent
           size, How many bytes should be sent
OUTPUT   : None
================================================================================
*/
void CC1101SendPacket( u8 *txbuffer, u8 size )
{
//	u8 i = 0;  
	CC1101ClrTXBuff( );
	CC1101WriteReg( CCxxx0_TXFIFO, size );
	CC1101WriteMultiReg( CCxxx0_TXFIFO, txbuffer, size );
	CC1101SetTRMode( TX_MODE );

	//    i=CC1101ReadStatus(0xF5);
	//while( ...... );
	while( CC1101_READGOD0 == 0 );
	while( CC1101_READGOD0 != 0 ); 

	CC1101SetIdle();
	CC1101ClrTXBuff( );
}
/*
================================================================================
Function : CC1101GetRXCnt( )
    Get received count of CC1101
INPUT    : None
OUTPUT   : How many bytes hae been received
================================================================================
*/
u8 CC1101GetRXCnt( void )
{
    return ( CC1101ReadStatus( CCxxx0_RXBYTES )  & BYTES_IN_RXFIFO );
}
/*
================================================================================
Function : CC1101RecPacket( )
    Receive a packet
INPUT    : rxBuffer, A buffer store the received data
           size, How many bytes should be received
OUTPUT   : 1:received, 0:no data 
================================================================================
*/
u8 CC1101RecPacket( u8 *rxBuffer, u8 *length )
{
	char status[2];
	char pktLen;
	u16 x , j = 0;

	CC1101SetTRMode( RX_MODE );
	while(CC1101_READGOD2 != 0 )
	{
		for( x = 0; x < 10; x ++ );
		if( ++j >= 20 )  return 0;
	}
	
	if ( CC1101GetRXCnt( ) != 0 )
	{
		for( x = 0; x < 200; x ++ );
		pktLen = CC1101ReadReg(CCxxx0_RXFIFO);   // Read length byte

		if (pktLen <= *length)                    // If pktLen size <= rxBuffer
		{
			CC1101ReadMultiReg(CCxxx0_RXFIFO, rxBuffer, pktLen); // Pull data
			*length = pktLen;                     // Return the actual size
			CC1101ReadMultiReg(CCxxx0_RXFIFO, (u8*)status, 2);// Read  status bytes
					
			CC1101ClrRXBuff( );
			if( status[1]&CRC_OK ) { return 1; }
			else {   return 0; }
		}                                       // Return CRC_OK bit
		else
		{
			*length = pktLen;                   // Return the large size
			CC1101SetIdle();
			CC1101ClrRXBuff( );      	        // Flush RXFIFO
			return 0;                           // Error
		}
	}
	else   {  return 0; }                       // Error
}
u8 CC1101_ID, cc1101_ver;
void CC1101Init( void )
{
	u8 PaTabel[] = {0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0};   //10dBm   
	volatile u8 i;  
	SPI_CC1101_Init();
	CC1101_DELAY;
	POWER_UP_RESET_CC1100( );
	halRfWriteRfSettings( );
	CC1101WriteMultiReg(CCxxx0_PATABLE, PaTabel, 8);
	CC1101_DELAY;CC1101_DELAY;
	CC1101_ID = CC1101ReadStatus( CCxxx0_PARTNUM );//for test, must be 0x80
	 
	cc1101_ver = CC1101ReadStatus( CCxxx0_VERSION );//for test, refer to the datasheet
	spi_delay(10);
}
//u8 rf_sendbuf[10]={0x31,0x31,0x32,0x32,0x33,0x33,0x34,0x34,0x0d,0x0a};
//u8 rxBuffer[128];
//u8 rxLength;
//void CC1101_test(void)
//{
//	u8 R_Flag;
//	CC1101Init();
//	CC1101SetTRMode( TX_MODE  ) ; //TX_MODE  RX_MODE 
//	while(1)
//	{
//		spi_delay(8000*300);
////		CC1101SendPacket(rf_sendbuf,10);
//		  R_Flag = CC1101RecPacket( rxBuffer, &rxLength );
//		if(R_Flag!=0)
//		{
//			UART1_SentData(rxBuffer,rxLength);
//		}
//		
//	}
//}

/*
================================================================================
------------------------------------THE END-------------------------------------
================================================================================
*/
