/***************************************************************************//**
 *   @file   Communication.c
 *   @brief  Implementation of Communication Driver for RENESAS RX62N
 *           Processor.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************/


/******************************************************************************/
/* Include Files                                                              */

/******************************************************************************/
#include "AD7799.h"
#include "ADC2.h"
#include "stdio.h"
#include "stm32f10x_spi.h"
#include "printf_embedded.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
/***************************************************************************//**
  
*******************************************************************************/
static void adc2_delay(__IO uint32_t nCount)
{

   //for(; nCount != 0; nCount--);	//5555 = 1ms
	while(nCount--);	//8000 = 1ms	
}

 void ADC2_SPI_IOConfig(void)
{
	// Add your code here.
	GPIO_InitTypeDef GPIO_InitStructure;  	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |RCC_APB2Periph_AFIO  , ENABLE);
	 /* Configure SPI1 pins: SCK, MISO and MOSI */ //SCLK,DIN,DOUT
	 GPIO_InitStructure.GPIO_Pin = ADC2_CLK_PIN | ADC2_MOSI_PIN | ADC2_MISO_PIN;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	 GPIO_Init(GPIOB, &GPIO_InitStructure);
 	 /* Configure CS for Chip select */
	 GPIO_InitStructure.GPIO_Pin = ADC2_CS_PIN;	//SPI CS
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  //推挽输出
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	 GPIO_Init(GPIOB, &GPIO_InitStructure);	 
	 
     
	ADC2_SCLK_L;
	
}
//SPIx 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
u8 SPI2_ReadWriteByte(u8 TxData)
{		
	u8 retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
	{
		retry++;
		if(retry>200)
			return 0;
	}			  
	SPI_I2S_SendData(SPI2, TxData); 
	retry=0;
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)//
	{
		retry++;
		if(retry>200)
			return 0;
	}	  						    
	return SPI_I2S_ReceiveData(SPI2); //·μ??í¨1ySPIx×??ü?óê?μ?êy?Y					    
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
 unsigned char ADC2_SPI_Init(unsigned char lsbFirst,unsigned char clockPol,unsigned char clockPha)
 {
 	SPI_InitTypeDef  SPI_InitStructure;
		ADC2_SPI_IOConfig(); 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2  , ENABLE);
		SPI_Cmd(SPI2, DISABLE);
	
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
	SPI_Init(SPI2, &SPI_InitStructure);             //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器

	//SPI1->CR1|=1<<6; //SPI设备使能
	/* Enable SPI1  */
	SPI_Cmd(SPI2, ENABLE); //使能SPI外设

	SPI2_ReadWriteByte(0xff);//启动传输           

	return 0;
}




 

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
unsigned char ADC2_SPI_Write(unsigned char* data,
                        unsigned char bytesNumber)
{
	// Add your code here.
	unsigned char i;
	unsigned char ValueToWrite;
	//spi_delay(2);
	for(i=0;i<bytesNumber;i++)
 	{
	 	ValueToWrite = *(data + i );
		SPI2_ReadWriteByte(ValueToWrite);
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
unsigned char ADC2_SPI_Read(unsigned char* data,
                       unsigned char bytesNumber)
{
	// Add your code here.
	unsigned	char	j = 0;
	adc2_delay(5);
	for(j=bytesNumber; j>0; j--)
	{
		*(data + j - 1)= SPI2_ReadWriteByte(0xFF);
	}
	return 0; 

}

 




/***************************************************************************//**
	 * @brief Initializes the AD7799 and checks if the device is present.
	 *
	 * @param None.
	 *
	 * @return status - Result of the initialization procedure.
	 *					Example: 1 - if initialization was successful (ID is 0x0B).
	 *							 0 - if initialization was unsuccessful.
	*******************************************************************************/
unsigned char ADC2_Init(void)
{ 
	unsigned char status = 0x1;
	if((ADC2_GetRegisterValue(AD7799_REG_ID, 1) & AD7799_ID_MASK) != AD7798_ID)
	{
		status = 0x0;
	}
	return(status);
 }


/***************************************************************************//**
 * @brief Sends 32 consecutive 1's on SPI in order to reset the part.
 *
 * @param None.
 *
 * @return  None.    
*******************************************************************************/
void ADC2_Reset(void)
{
	unsigned char dataToSend[4] = {0xff, 0xff, 0xff, 0xff};
	__nop();__nop();__nop();__nop();
	ADC2_SPI_Write(dataToSend,4);
	__nop();__nop();__nop();__nop();
	//AD7799_CS_H;	
}


/***************************************************************************//**
 * @brief Reads /RDY bit of status reg.
 *
 * @param None.
 *
 * @return rdy	- 0 if RDY is 1.
 *              - 1 if RDY is 0.
*******************************************************************************/
unsigned char ADC2_Ready(void)
{
    unsigned char rdy = 0;
    rdy = (ADC2_GetRegisterValue( AD7799_REG_STAT,1) & 0x80); 
		return(!rdy);
}
char ADC2_WaitBusy(void)
{
    u32 timeout = AD_LONGTIMEOUT;
    while(ADC2_Ready())
    {
        adc2_delay(100);
        timeout --;
        if(timeout == 0)
            return 1;
    }
    return 0;
}


/***************************************************************************//**
 * @brief Reads the value of the selected register
 *
 * @param regAddress - The address of the register to read.
 * @param size - The size of the register to read.
 *
 * @return data - The value of the selected register register.
*******************************************************************************/
unsigned long ADC2_GetRegisterValue(unsigned char regAddress, unsigned char size)
{
	unsigned char data[5] = {0x03, 0x00, 0x00, 0x00, 0x00};
	unsigned long receivedData = 0x00;	
	data[0] = AD7799_COMM_READ |  AD7799_COMM_ADDR(regAddress) ;
	ADC2_CS_L;  
	__nop();__nop();__nop();	 
	ADC2_SPI_Write(data,1);
    adc2_delay(200);
	ADC2_SPI_Read(data,size);
	__nop();

//	ADC2_CS_HIGH;
	if(size == 1)
	{
		receivedData += (data[0] << 0);
	}
	if(size == 2)
	{
		receivedData += (data[1] << 8);
		receivedData += (data[0] << 0);
	}
	if(size == 3)
	{
		receivedData += (data[2] << 16);
		receivedData += (data[1] << 8);
		receivedData += (data[0] << 0);
	}
    return receivedData;
}
/***************************************************************************//**
 * @brief Writes the value to the register
 *
 * @param -  regAddress - The address of the register to write to.
 * @param -  regValue - The value to write to the register.
 * @param -  size - The size of the register to write.
 *
 * @return  None.    
*******************************************************************************/
void ADC2_SetRegisterValue(unsigned char regAddress,
                             unsigned long regValue, 
                             unsigned char size)
{
	unsigned char data[5] = {0x03, 0x00, 0x00, 0x00, 0x00};	
	data[0] = AD7799_COMM_WRITE |  AD7799_COMM_ADDR(regAddress);
    if(size == 1)
    {
        data[1] = (unsigned char)regValue;
    }
    if(size == 2)
    {
			data[2] = (unsigned char)((regValue & 0x0000FF) >> 0);
			data[1] = (unsigned char)((regValue & 0x00FF00) >> 8);
    }
    if(size == 3)
    {
			data[3] = (unsigned char)((regValue & 0x0000FF) >> 0);
			data[2] = (unsigned char)((regValue & 0x00FF00) >> 8);
			data[1] = (unsigned char)((regValue & 0xFF0000) >> 16);
    }
	ADC2_CS_L;
	__nop();__nop();__nop();		
	ADC2_SPI_Write(data,size+1 );
	__nop();
//		AD7799_CS_H;

}
 

/***************************************************************************//**
 * @brief Sets the operating mode of AD7799.
 *
 * @param mode - Mode of operation.
 *
 * @return  None.    
*******************************************************************************/
void ADC2_SetMode(unsigned long mode)
{
    unsigned long command;
    command = AD7799_GetRegisterValue(AD7799_REG_MODE,2);
    command &= ~AD7799_MODE_SEL(0xFF);
   // command |= AD7799_MODE_SEL(mode);	
	 command |= AD7799_MODE_SEL(mode) | AD7799_MODE_PSW(x) ;
    AD7799_SetRegisterValue(
            AD7799_REG_MODE,
            command,
            2
    );
}
/***************************************************************************//**
 * @brief Sets the psw(power switch) of AD7799.
 *
 * @param mode - Mode of operation.
 *
 * @return  None.    
*******************************************************************************/
void ADC2_SetPSW(unsigned long mode)
{
    unsigned long command;
    command = ADC2_GetRegisterValue(AD7799_REG_MODE,2);
    if(mode == 0)
		command &=~(1<<12)  ;
	else
	{
		// command |= AD7799_MODE_SEL(mode);	
		command |=  AD7799_MODE_PSW(x) ;
	}
    ADC2_SetRegisterValue(
            AD7799_REG_MODE,
            command,
            2
    );
}
/***************************************************************************//**
 * @brief Selects the channel of AD7799.
 *
 * @param  channel - ADC channel selection.
 *
 * @return  None.    
*******************************************************************************/
void ADC2_SetChannel(unsigned long channel)
{
    unsigned long command;
    command = ADC2_GetRegisterValue(AD7799_REG_CONF,2);
    command &= ~AD7799_CONF_CHAN(0xFF);
    command |= AD7799_CONF_CHAN(channel);
    ADC2_SetRegisterValue(
            AD7799_REG_CONF,
            command,
            2
    );
}

/***************************************************************************//**
 * @brief  Sets the gain of the In-Amp.
 *
 * @param  gain - Gain.
 *
 * @return  None.    
*******************************************************************************/
void ADC2_SetGain(unsigned long gain)
{
    unsigned long command;
    command = ADC2_GetRegisterValue(AD7799_REG_CONF,2);
    command &= ~AD7799_CONF_GAIN(0xFF);
    command |= AD7799_CONF_GAIN(gain);
    ADC2_SetRegisterValue(
            AD7799_REG_CONF,
            command,
            2
    );
}
/***************************************************************************//**
 * @brief Enables or disables the reference detect function.
 *
 * @param state - State of the reference detect function.
 *               Example: 0	- Reference detect disabled.
 *                        1	- Reference detect enabled.
 *
 * @return None.    
*******************************************************************************/
void ADC2_SetReference(unsigned char state)
{
    unsigned long command = 0;
    command = ADC2_GetRegisterValue(AD7799_REG_CONF,2);
    command &= ~AD7799_CONF_REFDET(1);
    command |= AD7799_CONF_REFDET(state);
    ADC2_SetRegisterValue(AD7799_REG_CONF,
							command,
							2);
}
/***************************************************************************//**
 * @brief set unipolar or bipolar.
 *
 * @param state - State of the reference detect function.
 *               Example: 0	-  bipolar.
 *                        1	- unipolar.
 *
 * @return None.    
*******************************************************************************/
void ADC2_SetPolar(unsigned char state)
{
    unsigned long command = 0;
    command = ADC2_GetRegisterValue(AD7799_REG_CONF,2);
    command &= ~AD7799_CONF_UNIPOLAR;
    if(state)
		command |= AD7799_CONF_UNIPOLAR;
    ADC2_SetRegisterValue(AD7799_REG_CONF,
							command,
							2);
}
/***************************************************************************//**
 * @brief get unipolar or bipolar.
 *
 * @param   
 *
 * @return State of polar.
 *               Example: 0	-  bipolar.
 *                        1	- unipolar..    
*******************************************************************************/
unsigned char  ADC2_GetPolar( void)
{
    unsigned long command = 0;
    command = ADC2_GetRegisterValue(AD7799_REG_CONF,2);
    command  &=  AD7799_CONF_UNIPOLAR;
	command = command >>12;
	return command;
}
/***************************************************************************//**
 * @brief set burnout current enable.
 *
 * @param state - State of the  .
 *               Example: 0	-  disable .
 *                        1	-  burnout current enable.
 *
 * @return None.    
*******************************************************************************/
void ADC2_SetBO(unsigned char state)
{
    unsigned long command = 0;
    command = ADC2_GetRegisterValue(AD7799_REG_CONF,2);
    command &= ~AD7799_CONF_BO_EN;
    if(state)
		command |= AD7799_CONF_BO_EN;
    ADC2_SetRegisterValue(AD7799_REG_CONF,
							command,
							2);
}
/***************************************************************************//**
 * @brief  ad7798 calibrate.
 *
 * @param   .
 *
 * @return None.    
*******************************************************************************/
 


static unsigned char ADC2_Calibrate(unsigned char CHx,unsigned char Gain)
{
	uint8_t R ;
//	uint16_t reg;
	 
	R = 0;	
	ADC2_WaitBusy();
 
	ADC2_SetRegisterValue(AD7799_REG_CONF,AD7799_CONF_UNIPOLAR|AD7799_CONF_BUF|AD7799_CONF_GAIN(Gain)|AD7799_CONF_CHAN(CHx),2);
	ADC2_WaitBusy();
//	reg = ADC2_GetRegisterValue(AD7799_REG_MODE,2);
//	printf("before int_zero AD7799_REG_MODE is %x\r\n",reg); 
	ADC2_SetRegisterValue(AD7799_REG_MODE,0x800a,2);
	//等待校准完成
	adc2_delay(160000);
	ADC2_WaitBusy(); 
	//reg = AD7799_GetRegisterValue(AD7799_REG_MODE,2);
//	printf("after int_zero AD7799_REG_MODE is %x\r\n",reg); 
	//进行内部校准
	//AD7799_SetMode(AD7799_MODE_CAL_INT_FULL);	
	ADC2_SetRegisterValue(AD7799_REG_MODE,0xa00a,2);
	adc2_delay(160000);
	ADC2_WaitBusy();
  
	R = 1;
	return R;
}




 
unsigned char ADC2_Config(void)
{
	u32 i = 0;
//	u16 reg;
	
	ADC2_SPI_Init(0,1,1);
	ADC2_CS_H;
	adc2_delay(100);
	ADC2_CS_L;
	adc2_delay(100000);
	//ADC2_WaitBusy();
	ADC2_Reset();
	adc2_delay(100000);	 
	i = 0;
	while(!ADC2_Init())
	{
		i++;
		adc2_delay(100);
		if(i>1000)
		{
			ADC2_Reset();
			return 1;
		}
	}
	adc2_delay(100);
	ADC2_SetRegisterValue(AD7799_REG_CONF,AD7799_CONF_UNIPOLAR|AD7799_CONF_BUF,2);
	//AD7799_SetRegisterValue(AD7799_REG_CONF,AD7799_CONF_UNIPOLAR,2);
	ADC2_Calibrate(0,0);	
	ADC2_WaitBusy();
	//AD7799_SetMode(AD7799_MODE_CONT);
 
	ADC2_SetRegisterValue(AD7799_REG_MODE,AD7799_MODE_SEL(AD7799_MODE_CONT)|AD7799_MODE_RATE(2),2); //AD7799_MODE_PSW(x)|
	adc2_delay(100000);
	ADC2_WaitBusy();
	//**************//
//	AD7799_SetChannel(2);	
	/*****************/
	 
	adc2_delay(8000);
	return 0;
}


unsigned int ADC2_GetValue(unsigned char channel)
{
	u8 rdy;
	u16 reg_data;
	u16 try_times;
	
	ADC2_SetChannel(channel);	
	adc2_delay(10000);
	if(ADC2_WaitBusy())
		return 0;
	rdy = 0;
	while(1)
	{
		adc2_delay(50000);
		rdy = ADC2_GetRegisterValue(AD7799_REG_STAT,1);
		if((rdy&0x03) == channel)
				break;
		try_times++;
		if(try_times > 10)
		{

			printfk("adc2 channel %d rdy is %d err\r\n",channel,rdy); 
			
			__nop();
			return 0;
		}
	}
//	rdy = ADC2_GetRegisterValue(AD7799_REG_STAT,1);
//	printf("adc2 rdy is %x\r\n",rdy);
 
	reg_data = ADC2_GetRegisterValue(AD7799_REG_DATA, 2);
 //	 printf("ch is %d  sig data is %x\r\n",channel,reg_data);
	return reg_data;
}
