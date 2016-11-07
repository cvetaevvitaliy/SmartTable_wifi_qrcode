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


/***************************************************************************//**
  
 * @return 0 - Initialization failed, 1 - Initialization succeeded.
*******************************************************************************/
//unsigned char SIM_SPI_Init(unsigned char lsbFirst,
//                       unsigned long clockFreq,
//                       unsigned char clockPol,
//                       unsigned char clockPha)
 void ADC2_SPI_Init(void)
{
	// Add your code here.
	GPIO_InitTypeDef GPIO_InitStructure;				  										  
	 	 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |RCC_APB2Periph_AFIO  , ENABLE);
	//**************  #define ADC2_CLK_PIN  #define ADC2_MOSI_PIN    ADC2_MISO_PIN ********//
	GPIO_InitStructure.GPIO_Pin = ADC2_CS_PIN | ADC2_CLK_PIN| ADC2_MOSI_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =  ADC2_MISO_PIN ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //miso要用模拟输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	ADC2_SCLK_H;
	
}
static void spi_delay (__IO uint32_t nDelay)
{
	while (nDelay--);
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
	unsigned char i,j;
	unsigned char ValueToWrite;
	//spi_delay(2);
	for(i=0;i<bytesNumber;i++)
 	{
	 	ValueToWrite = *(data + i );
		for(j=0; j<8; j++)
		{
			ADC2_SCLK_L;
			spi_delay(2);
			if(0x80 == (ValueToWrite & 0x80))
			{
				ADC2_MOSI_H;	  //Send one to SDO pin
			}
			else
			{
				ADC2_MOSI_L;	  //Send zero to SDO pin
			}
			spi_delay(2);
			ADC2_SCLK_H;			 
			ValueToWrite <<= 1;	//Rotate data
		}
	}
return  0;	
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
	unsigned	char	i = 0;
	unsigned	char	j = 0;
	unsigned	char  	ReadData = 0;
 
	spi_delay(1);
	for(j=bytesNumber; j>0; j--)
	{
		for(i=0; i<8; i++)
		{
			ADC2_SCLK_L;		 
			ReadData=ReadData<<1 ;
			if(ADC2_MISO)ReadData+=1 ;
			spi_delay(2);			
			ADC2_SCLK_H;	
			spi_delay(2);			
		}
		*(data + j - 1)= ReadData;
	}	 
 return 0;
}

 



static void ad_delay(__IO uint32_t nCount)
{

   //for(; nCount != 0; nCount--);	//5555 = 1ms
	while(nCount--);	//8000 = 1ms	
}

/***************************************************************************//**
 * @brief Initializes the AD7799 and checks if the device is present.
 *
 * @param None.
 *
 * @return status - Result of the initialization procedure.
 *                  Example: 1 - if initialization was successful (ID is 0x0B).
 *                           0 - if initialization was unsuccessful.
*******************************************************************************/
unsigned char ADC2_Init(void)
{ 
	unsigned char status = 0x1;
	static unsigned char getID = 0;
	getID = ADC2_GetRegisterValue(AD7799_REG_ID, 1);
//	printf("ADC2 id is %d \r\n",getID);
	if( (getID & AD7799_ID_MASK) != AD7798_ID)
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
	ADC2_CS_L;
	__nop();__nop();__nop();__nop();
	ADC2_SPI_Write(dataToSend,4);
	__nop();__nop();__nop();__nop();
	//ADC2_CS_H;	
}

/*--------------------------------------------------------- 
Func: ADC2忙判断 
Note: 0/OK >0/ERROR,timeout 
---------------------------------------------------------*/  
char ADC2_WaitBusy(void)  
{  
	uint32_t i; 	 
	i=0;  
	while(ADC2_DRY)
	{  
		i++; 
		if(i>600000)
		{
			ADC2_Reset();
			return 1;  
		}
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
	if(ADC2_WaitBusy())
		return 0;
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
	//rdy = AD7799_GetRegisterValue( AD7799_REG_STAT,1);
// 	printf("rdy is %x\r\n",rdy);
	return(!rdy);
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
	ad_delay(160000);
	ADC2_WaitBusy(); 
	//reg = AD7799_GetRegisterValue(AD7799_REG_MODE,2);
//	printf("after int_zero AD7799_REG_MODE is %x\r\n",reg); 
	//进行内部校准
	//AD7799_SetMode(AD7799_MODE_CAL_INT_FULL);	
	ADC2_SetRegisterValue(AD7799_REG_MODE,0xa00a,2);
	ad_delay(160000);
	ADC2_WaitBusy();
  
	R = 1;
	return R;
}




 
unsigned char ADC2_Config(void)
{
	u32 i = 0;
//	u16 reg;
	
	ADC2_SPI_Init();
	ADC2_CS_H;
	ad_delay(100);
	ADC2_CS_L;
	ad_delay(100000);
	//ADC2_WaitBusy();
	ADC2_Reset();
	ad_delay(100000);	 
	i = 0;
	while(!ADC2_Init())
	{
		i++;
		ad_delay(100);
		if(i>1000)
		{
			ADC2_Reset();
			return 1;
		}
	}
	ad_delay(100);
	ADC2_SetRegisterValue(AD7799_REG_CONF,AD7799_CONF_UNIPOLAR|AD7799_CONF_BUF,2);
	//AD7799_SetRegisterValue(AD7799_REG_CONF,AD7799_CONF_UNIPOLAR,2);
	ADC2_Calibrate(0,0);	
	ADC2_WaitBusy();
	//AD7799_SetMode(AD7799_MODE_CONT);
 
	ADC2_SetRegisterValue(AD7799_REG_MODE,AD7799_MODE_SEL(AD7799_MODE_CONT)|AD7799_MODE_RATE(2),2); //AD7799_MODE_PSW(x)|
	ad_delay(100000);
	ADC2_WaitBusy();
	//**************//
//	AD7799_SetChannel(2);	
	/*****************/
	 
	ad_delay(8000);
	return 0;
}


unsigned int ADC2_GetValue(unsigned char channel)
{
	u16 reg_data;

	ADC2_SetChannel(channel);	
	ad_delay(10000);
	if(ADC2_WaitBusy())
		return 0;
	ad_delay(10000);
//	rdy = ADC2_GetRegisterValue(AD7799_REG_STAT,1);
//	printf("adc2 rdy is %x\r\n",rdy);
 
	reg_data = ADC2_GetRegisterValue(AD7799_REG_DATA, 2);
 //	 printf("ch is %d  sig data is %x\r\n",channel,reg_data);
	return reg_data;
}
