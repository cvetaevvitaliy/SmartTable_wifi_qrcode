/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : spi_flash.c
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : This file provides a set of functions needed to manage the
*                      communication between SPI peripheral and SPI M25P64 FLASH.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "spi_cc1101.h"
#include "stm32f10x_spi.h"
#include "delay.h"



/*
//INT8U PaTabel[8] = {0x04 ,0x04 ,0x04 ,0x04 ,0x04 ,0x04 ,0x04 ,0x04};  //-30dBm   功率最小
//INT8U PaTabel[8] = {0x60 ,0x60 ,0x60 ,0x60 ,0x60 ,0x60 ,0x60 ,0x60};  //0dBm
INT8U PaTabel[8] = {0xC0 ,0xC0 ,0xC0 ,0xC0 ,0xC0 ,0xC0 ,0xC0 ,0xC0};   //10dBm    功率最大
*/
INT8U PaTabel[8] = {0xC0 ,0xC0 ,0xC0 ,0xC0 ,0xC0 ,0xC0 ,0xC0 ,0xC0};
INT8U paTableLen = 4;                                         //对应功率值，默认4则为0dbm发送

INT8U txBuffer[20]={15,1,3,4//,5,6,7,8,9,4,5,5,6,2,1,3,4,5,6,7
                  //,3,4,5,6,7,8,9,4,5,5,6,2,1,3,4,5,6,7,8,9
                  ,7,8,9,4,5,5,6,2,1,3,4,5,6,7,8,6,};
INT8U rxBuffer[19];




 
/*******************************************************************************
* Function Name  : SPI_FLASH_Init
* Description    : Initializes the peripherals used by the SPI FLASH driver.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_CC1101_Init(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOB , ENABLE);
	
	/* Configure SPI2 pins: SCK, MISO and MOSI */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_SCLK | GPIO_Pin_SO | GPIO_Pin_SI;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO_CS, &GPIO_InitStructure);
	 
	/* Configure I/O for Flash Chip select */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_CS;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO_CS, &GPIO_InitStructure);
	
	/* Configure I/O for PC4,PC5  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_GD2| GPIO_Pin_GD0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	 
	GPIO_Init(GPIO_GD, &GPIO_InitStructure);
	/* Deselect the FLASH: Chip Select high */
	//SPI_CC1101_CS_HIGH();
	
	
	/* Enable SPI1 and GPIO clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	
	
	/* SPI2 configuration */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//SPI_Direction_2Lines_RxOnly;//SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	//SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI_CC1101, &SPI_InitStructure);
	
	/* Enable SPI2  */
	SPI_Cmd(SPI_CC1101, ENABLE);
}

/*******************************************************************************
* Function Name  : SPI_FLASH_ReadByte
* Description    : Reads a byte from the SPI Flash.
*                  This function must be used only if the Start_Read_Sequence
*                  function has been previously called.
* Input          : None
* Output         : None
* Return         : Byte Read from the SPI Flash.
*******************************************************************************/
u8 SPI_CC1101_ReadByte(void)
{
  return (SPI_CC1101_SendByte(Dummy_Byte));
}

/*******************************************************************************
* Function Name  : SPI_CC1101_SendByte
* Description    : Sends a byte through the SPI interface and return the byte
*                  received from the SPI bus.
* Input          : byte : byte to send.
* Output         : None
* Return         : The value of the received byte.
*******************************************************************************/
u8 SPI_CC1101_SendByte(u8 byte)
{
  /* Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(SPI_CC1101, SPI_I2S_FLAG_TXE) == RESET);

  /* Send byte through the SPI2 peripheral */
  SPI_I2S_SendData(SPI_CC1101, byte);

  /* Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(SPI_CC1101, SPI_I2S_FLAG_RXNE) == RESET);

  /* Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(SPI_CC1101);
}

/**********************************CC1101********************/

static void cc1101_delay(vu32 nCount)
{
  int i,j;
  for(j=0;j<nCount;j++)
  {
     for(i=0;i<10;i++);
  }
}


INT8U SPI_CC1101_ReadID(void)
{
	INT8U id;
	SPI_CC1101_CS_LOW();
	cc1101_delay(5);
	//	 SPI_CC1101_SendByte(0x30|0xc0);
	//	 id = SPI_CC1101_SendByte(0xff);
	//	 SPI_CC1101_SendByte(0x31|0xc0);

	SPI_CC1101_SendByte(0x31);
	id = SPI_CC1101_SendByte(0xff);
	cc1101_delay(5);
	SPI_CC1101_CS_HIGH();

	return id;
}

void CC1101_POWER_RESET(void)
{

 /* Deselect the FLASH: Chip Select high */
  SPI_CC1101_CS_HIGH();     
  GPIO_SetBits(GPIO_CS, GPIO_Pin_SCLK); //SCLK=1
  GPIO_ResetBits(GPIO_CS, GPIO_Pin_SI); //SI=0
  /* Select the FLASH: Chip Select low */
  Sys_delay_us(5000);
  SPI_CC1101_CS_HIGH();
  Sys_delay_us(1);
  SPI_CC1101_CS_LOW();
  Sys_delay_us(1);
  SPI_CC1101_CS_HIGH();
  Sys_delay_us(41);
  SPI_CC1101_CS_LOW();
  while (GPIO_ReadInputDataBit(GPIO_CS,GPIO_Pin_SO) );//waite SO =0
  SPI_CC1101_SendByte(CCxxx0_SRES);
  while (GPIO_ReadInputDataBit(GPIO_CS,GPIO_Pin_SO) );//waite SO =0 again 
  SPI_CC1101_CS_HIGH(); 
}

//*****************************************************************************************
//函数名：void halSpiWriteReg(INT8U addr, INT8U value)
//输入：地址和配置字
//输出：无
//功能描述：SPI写寄存器
//*****************************************************************************************
void halSpiWriteReg(INT8U addr, INT8U value) 
{
    SPI_CC1101_CS_LOW();
    while (GPIO_ReadInputDataBit(GPIO_CS,GPIO_Pin_SO) );
    SPI_CC1101_SendByte(addr);		//写地址
    SPI_CC1101_SendByte(value);		//写入配置
    SPI_CC1101_CS_HIGH(); 
}

//*****************************************************************************************
//函数名：void halSpiWriteBurstReg(INT8U addr, INT8U *buffer, INT8U count)
//输入：地址，写入缓冲区，写入个数
//输出：无
//功能描述：SPI连续写配置寄存器
//*****************************************************************************************
void halSpiWriteBurstReg(INT8U addr, INT8U *buffer, INT8U count) 
{
    INT8U i, temp;
	temp = addr | WRITE_BURST;
    SPI_CC1101_CS_LOW();
    while (GPIO_ReadInputDataBit(GPIO_CS,GPIO_Pin_SO) );
    SPI_CC1101_SendByte(temp);
    for (i = 0; i < count; i++)
 	{
        SPI_CC1101_SendByte(buffer[i]);
    }
    SPI_CC1101_CS_HIGH(); 
}

//*****************************************************************************************
//函数名：void halSpiStrobe(INT8U strobe)
//输入：命令
//输出：无
//功能描述：SPI写命令
//*****************************************************************************************
void halSpiStrobe(INT8U strobe) 
{
    SPI_CC1101_CS_LOW();
    while (GPIO_ReadInputDataBit(GPIO_CS,GPIO_Pin_SO) );
    SPI_CC1101_SendByte(strobe);		//写入命令
    SPI_CC1101_CS_HIGH();
}

//*****************************************************************************************
//函数名：void halRfSendPacket(INT8U *txBuffer, INT8U size)
//输入：发送的缓冲区，发送数据个数
//输出：无
//功能描述：CC1100发送一组数据
//*****************************************************************************************

void halRfSendPacket(INT8U *txBuffer, INT8U size) 
{

	//halSpiWriteReg(CCxxx0_TXFIFO, size); //写入长度
	//halSpiWriteReg(CCxxx0_TXFIFO, 0x12);//写入接受地址
    halSpiWriteBurstReg(CCxxx0_TXFIFO, txBuffer, size);	//写入要发送的数据

    halSpiStrobe(CCxxx0_STX);		//进入发送模式发送数据	

    // Wait for GDO0 to be set -> sync transmitted
    while (!GPIO_ReadInputDataBit(GPIO_GD,GPIO_Pin_GD0) );//while (!GDO0);
    // Wait for GDO0 to be cleared -> end of packet
    while (GPIO_ReadInputDataBit(GPIO_GD,GPIO_Pin_GD0) );// while (GDO0);
	halSpiStrobe(CCxxx0_SFTX);
}

//*****************************************************************************************
//函数名：void halRfSendData(INT8U txData)
//输入：发送的数据
//输出：无
//功能描述：CC1100发送一个数据
//*****************************************************************************************

void halRfSendData(INT8U txData)
{
    halSpiWriteReg(CCxxx0_TXFIFO, txData);	//写入要发送的数据

    halSpiStrobe(CCxxx0_STX);		//进入发送模式发送数据	

    // Wait for GDO0 to be set -> sync transmitted
    while (!GPIO_ReadInputDataBit(GPIO_GD,GPIO_Pin_GD0) );//while (!GDO0);
    // Wait for GDO0 to be cleared -> end of packet
    while (GPIO_ReadInputDataBit(GPIO_GD,GPIO_Pin_GD0) );// while (GDO0);
	halSpiStrobe(CCxxx0_SFTX);
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
		halSpiStrobe( CCxxx0_STX ); 
	}
	else if( mode == RX_MODE )	
	{ 
		halSpiStrobe( CCxxx0_SRX ); 
	} 
}
//*****************************************************************************************
//函数名：void halRfWriteRfSettings(RF_SETTINGS *pRfSettings)
//输入：无
//输出：无
//功能描述：配置CC1100的寄存器
//*****************************************************************************************
void halRfWriteRfSettings(void) 
{

	halSpiWriteReg(CCxxx0_FSCTRL0,  rfSettings.FSCTRL2);//自已加的
    // Write register settings
    halSpiWriteReg(CCxxx0_FSCTRL1,  rfSettings.FSCTRL1);
    halSpiWriteReg(CCxxx0_FSCTRL0,  rfSettings.FSCTRL0);
    halSpiWriteReg(CCxxx0_FREQ2,    rfSettings.FREQ2);
    halSpiWriteReg(CCxxx0_FREQ1,    rfSettings.FREQ1);
    halSpiWriteReg(CCxxx0_FREQ0,    rfSettings.FREQ0);
    halSpiWriteReg(CCxxx0_MDMCFG4,  rfSettings.MDMCFG4);
    halSpiWriteReg(CCxxx0_MDMCFG3,  rfSettings.MDMCFG3);
    halSpiWriteReg(CCxxx0_MDMCFG2,  rfSettings.MDMCFG2);
    halSpiWriteReg(CCxxx0_MDMCFG1,  rfSettings.MDMCFG1);
    halSpiWriteReg(CCxxx0_MDMCFG0,  rfSettings.MDMCFG0);
    halSpiWriteReg(CCxxx0_CHANNR,   rfSettings.CHANNR);
    halSpiWriteReg(CCxxx0_DEVIATN,  rfSettings.DEVIATN);
    halSpiWriteReg(CCxxx0_FREND1,   rfSettings.FREND1);
    halSpiWriteReg(CCxxx0_FREND0,   rfSettings.FREND0);
    halSpiWriteReg(CCxxx0_MCSM0,  	rfSettings.MCSM0);
    halSpiWriteReg(CCxxx0_FOCCFG,   rfSettings.FOCCFG);
    halSpiWriteReg(CCxxx0_BSCFG,    rfSettings.BSCFG);
    halSpiWriteReg(CCxxx0_AGCCTRL2, rfSettings.AGCCTRL2);
	halSpiWriteReg(CCxxx0_AGCCTRL1, rfSettings.AGCCTRL1);	 //
    halSpiWriteReg(CCxxx0_AGCCTRL0, rfSettings.AGCCTRL0);
    halSpiWriteReg(CCxxx0_FSCAL3,   rfSettings.FSCAL3);
	halSpiWriteReg(CCxxx0_FSCAL2,   rfSettings.FSCAL2);
	halSpiWriteReg(CCxxx0_FSCAL1,   rfSettings.FSCAL1);	//
    halSpiWriteReg(CCxxx0_FSCAL0,   rfSettings.FSCAL0);
    halSpiWriteReg(CCxxx0_FSTEST,   rfSettings.FSTEST);
    halSpiWriteReg(CCxxx0_TEST2,    rfSettings.TEST2);
    halSpiWriteReg(CCxxx0_TEST1,    rfSettings.TEST1);
    halSpiWriteReg(CCxxx0_TEST0,    rfSettings.TEST0);
    halSpiWriteReg(CCxxx0_IOCFG2,   rfSettings.IOCFG2);
    halSpiWriteReg(CCxxx0_IOCFG0,   rfSettings.IOCFG0);    
    halSpiWriteReg(CCxxx0_PKTCTRL1, rfSettings.PKTCTRL1);
    halSpiWriteReg(CCxxx0_PKTCTRL0, rfSettings.PKTCTRL0);
    halSpiWriteReg(CCxxx0_ADDR,     rfSettings.ADDR);
    halSpiWriteReg(CCxxx0_PKTLEN,   rfSettings.PKTLEN);
}
/*
================================================================================
Function : CC1101ReadStatus( )
    Read a status register
INPUT    : addr, The address of the register
OUTPUT   : the value read from the status register
================================================================================
*/
INT8U CC1101ReadStatus( INT8U addr )
{
	INT8U i;
	 SPI_CC1101_CS_LOW();
	cc1101_delay(2);
	SPI_CC1101_SendByte( addr | READ_BURST);
	cc1101_delay(2);
	i = SPI_CC1101_ReadByte();
	cc1101_delay(2);
	 SPI_CC1101_CS_HIGH( );
	return i;
}

unsigned char send_num=0;
u8 CC1101_ID,cc1101_ver;
u8 CC1101_Config(void)
{
	u8 res = 1;
	SPI_CC1101_Init();
	cc1101_delay(1000);
	CC1101_POWER_RESET();
	halRfWriteRfSettings();
	halSpiWriteBurstReg(CCxxx0_PATABLE, PaTabel, 8);
//	halRfSendPacket(TxBuf,8);	// Transmit Tx buffer data
	 cc1101_delay(6000);
	 CC1101_ID = CC1101ReadStatus(0x30);
	cc1101_ver = CC1101ReadStatus(0x31);
	if((CC1101_ID == 0)&&(cc1101_ver == 0x14))
	{
		res = 0;
	}
	return res;
}

 

 


void CC1101_test(void)
{
     
//	INT8U tf =0;	
	while(CC1101_Config() != 0);
	
	while(1)
    {
		//GPIO_ResetBits(GPIOC, GPIO_Pin_6);
		send_num++;                                   //发送数据个数累加
		halSpiStrobe(CCxxx0_SIDLE);             //进入IDLE模式
		txBuffer[0] =0x13;                              // Packet length
		txBuffer[1] = 0x12;                           // Packet address
		txBuffer[2] = send_num;                       //写入当前发送数据个数
		halRfSendPacket(txBuffer, 20);
//		
		
		
	//	 halRfSendData(0x50);
//		
 		cc1101_delay(180000);
//		GPIO_SetBits(GPIOC, GPIO_Pin_6);
//		Delay(180000);
    } 	
}

/*************************************************************/
