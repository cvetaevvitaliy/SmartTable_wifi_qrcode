/***************************************************************************//**
 *   @file   Communication.c
 *   @brief  Implementation of Communication Driver for RENESAS RX62N
 *           Processor.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************/


/******************************************************************************/
/* Include Files                                                              */

/******************************************************************************/
#include "Communication.h"
#include "stm32f10x.h"
#include "stm32f10x_spi.h"


void SPI_IO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_AFIO  , ENABLE);
	 /* Configure SPI1 pins: SCK, MISO and MOSI */ //SCLK,DIN,DOUT
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	 GPIO_Init(GPIOA, &GPIO_InitStructure);
 	 /* Configure CS for Chip select */
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;	//SPI CS
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //推挽输出
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(GPIOA, &GPIO_InitStructure);

	 
	 GPIO_ResetBits(GPIOA,GPIO_Pin_4);
	 
}
//SPIx 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
u8 SPI1_ReadWriteByte(u8 TxData)
{		
	u8 retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
	{
		retry++;
		if(retry>200)
			return 0;
	}			  
	SPI_I2S_SendData(SPI1, TxData); 
	retry=0;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)//
	{
		retry++;
		if(retry>200)
			return 0;
	}	  						    
	return SPI_I2S_ReceiveData(SPI1); //·μ??í¨1ySPIx×??ü?óê?μ?êy?Y					    
}


 /***************************************************************************//**
  * @brief Initializes the SPI communication peripheral.
  *
  * @param lsbFirst - Transfer format (0 or 1).
  * 				  Example: 0x0 - MSB first.
  * 						   0x1 - LSB first.
  * @param clockFreq - SPI clock frequency (Hz).
  * 				   Example: 1000 - SPI clock frequency is 1 kHz.
  * @param clockPol - SPI clock polarity (0 or 1).
  * 				  Example: 0x0 - idle state for SPI clock is low.
  * 						   0x1 - idle state for SPI clock is high.
  * @param clockPha - SPI clock phase (0 or 1).
  * 				  Example: 0x0 - data is latched on the leading edge of SPI
  * 								 clock and data changes on trailing edge.
  * 						   0x1 - data is latched on the trailing edge of SPI
  * 								 clock and data changes on the leading edge.
  *
  * @return 0 - Initialization failed, 1 - Initialization succeeded.
 *******************************************************************************/
 unsigned char SIM_SPI_Init(unsigned char lsbFirst,unsigned char clockPol,unsigned char clockPha)
 {
 	SPI_InitTypeDef  SPI_InitStructure;
		SPI_IO_Config(); 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1|RCC_APB2Periph_AFIO  , ENABLE);
		SPI_Cmd(SPI1, DISABLE);
	
		/* SPI1 configuration */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                //设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                //设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = (clockPol&0x01)<<1; //SPI_CPOL_High               //选择了串行时钟的稳态:时钟悬空高
	SPI_InitStructure.SPI_CPHA = clockPha&0x01;  // SPI_CPHA_2Edge;        //数据捕获于第二个时钟沿
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                //NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;                //定义波特率预分频的值:波特率预分频值为256
	if(lsbFirst)
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_LSB;        //指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	else
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;        //指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;            //CRC值计算的多项式
	SPI_Init(SPI1, &SPI_InitStructure);             //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器

	//SPI1->CR1|=1<<6; //SPI设备使能
	/* Enable SPI1  */
	SPI_Cmd(SPI1, ENABLE); //使能SPI外设

	SPI1_ReadWriteByte(0xff);//启动传输           

	return 0;
}


static void spi_delay (__IO uint32_t nDelay)
{
	while (nDelay--);
}



 


//---------------------------------
//void ReadFromAD7799(unsigned char count,unsigned char *buf)
//---------------------------------
//Function that reads from the AD7799 via the SPI port. 
//--------------------------------------------------------------------------------
 
/***************************************************************************//**
 * @brief Writes data to SPI.
 *
 * @param data - Write data buffer:
 *               - first byte is the chip select number;
 *               - from the second byte onwards are located data bytes to write.
 * @param bytesNumber - Number of bytes to write.
 *
 * @return Number of written bytes.
*******************************************************************************/
unsigned char SIM_SPI_Write(unsigned char* data,
                        unsigned char bytesNumber)
{
	// Add your code here.
	unsigned char i;
	unsigned char ValueToWrite;
	//spi_delay(2);
	for(i=0;i<bytesNumber;i++)
 	{
	 	ValueToWrite = *(data + i );
		SPI1_ReadWriteByte(ValueToWrite);
	}
	return  i;	
}

/***************************************************************************//**
 * @brief Reads data from SPI.
 *
 * @param data - As an input parameter, data represents the write buffer:
 *               - first byte is the chip select number;
 *               - from the second byte onwards are located data bytes to write.
 *               As an output parameter, data represents the read buffer:
 *               - from the first byte onwards are located the read data bytes. 
 * @param bytesNumber - Number of bytes to write.
 *
 * @return Number of written bytes.
*******************************************************************************/
unsigned char SIM_SPI_Read(unsigned char* data,
                       unsigned char bytesNumber)
{
	// Add your code here.
	unsigned	char	j = 0;
	spi_delay(1);
	for(j=bytesNumber; j>0; j--)
	{
		*(data + j - 1)= SPI1_ReadWriteByte(0xFF);
	}
	return 0;
}


 

