/***************************************************************************//**
 *   @file   AD7799.c
 *   @brief  Implementation of AD7799 Driver.
 *   @author Bancisor MIhai
********************************************************************************/
 
/* Include Files                                                              */
/******************************************************************************/
#include "AD7799.h"				// AD7799 definitions.
#include "Communication.h"		// Communication definitions.
#include <stdio.h>
#include "printf_embedded.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#define AD_SHORTTIMEOUT  0x1000
#define AD_LONGTIMEOUT  0x8000

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
	 *					Example: 1 - if initialization was successful (ID is 0x0B).
	 *							 0 - if initialization was unsuccessful.
	*******************************************************************************/
unsigned char AD7799_Init(void)
{ 
	unsigned char status = 0x1;
	if((AD7799_GetRegisterValue(AD7799_REG_ID, 1) & AD7799_ID_MASK) != AD7798_ID)
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
void AD7799_Reset(void)
{
	unsigned char dataToSend[4] = {0xff, 0xff, 0xff, 0xff};
	__nop();__nop();__nop();__nop();
	SIM_SPI_Write(dataToSend,4);
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
unsigned char AD7799_Ready(void)
{
    unsigned char rdy = 0;
    rdy = (AD7799_GetRegisterValue( AD7799_REG_STAT,1) & 0x80); 
	return(!rdy);
}
char AD7799_WaitBusy(void)
{
    u32 timeout = AD_LONGTIMEOUT;
    while(AD7799_Ready())
    {
				ad_delay(100);
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
unsigned long AD7799_GetRegisterValue(unsigned char regAddress, unsigned char size)
{
	unsigned char data[5] = {0x03, 0x00, 0x00, 0x00, 0x00};
	unsigned long receivedData = 0x00;	
    //unsigned int timeout = AD_SHORTTIMEOUT;
	data[0] = AD7799_COMM_READ |  AD7799_COMM_ADDR(regAddress) ;
	__nop();__nop();__nop();	 
	SIM_SPI_Write(data,1);
	ad_delay(200);
	SIM_SPI_Read(data,size);
	__nop();

//	AD7799_CS_HIGH;
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
void AD7799_SetRegisterValue(unsigned char regAddress,
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
	__nop();__nop();__nop();		
	SIM_SPI_Write(data,size+1 );
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
void AD7799_SetMode(unsigned long mode)
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
void AD7799_SetPSW(unsigned long mode)
{
    unsigned long command;
    command = AD7799_GetRegisterValue(AD7799_REG_MODE,2);
    if(mode == 0)
		command &=~(1<<12)  ;
	else
	{
		// command |= AD7799_MODE_SEL(mode);	
		command |=  AD7799_MODE_PSW(x) ;
	}
    AD7799_SetRegisterValue(
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
void AD7799_SetChannel(unsigned long channel)
{
    unsigned long command;
    command = AD7799_GetRegisterValue(AD7799_REG_CONF,2);
    command &= ~AD7799_CONF_CHAN(0xFF);
    command |= AD7799_CONF_CHAN(channel);
    AD7799_SetRegisterValue(
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
static void AD7799_SetGain(unsigned long gain)
{
    unsigned long command;
    command = AD7799_GetRegisterValue(AD7799_REG_CONF,2);
    command &= ~AD7799_CONF_GAIN(0xFF);
    command |= AD7799_CONF_GAIN(gain);
    AD7799_SetRegisterValue(
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
static void AD7799_SetReference(unsigned char state)
{
    unsigned long command = 0;
    command = AD7799_GetRegisterValue(AD7799_REG_CONF,2);
    command &= ~AD7799_CONF_REFDET(1);
    command |= AD7799_CONF_REFDET(state);
    AD7799_SetRegisterValue(AD7799_REG_CONF,
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
static void AD7799_SetPolar(unsigned char state)
{
    unsigned long command = 0;
    command = AD7799_GetRegisterValue(AD7799_REG_CONF,2);
    command &= ~AD7799_CONF_UNIPOLAR;
    if(state)
		command |= AD7799_CONF_UNIPOLAR;
    AD7799_SetRegisterValue(AD7799_REG_CONF,
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
static unsigned char  AD7799_GetPolar( void)
{
    unsigned long command = 0;
    command = AD7799_GetRegisterValue(AD7799_REG_CONF,2);
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
static void AD7799_SetBO(unsigned char state)
{
    unsigned long command = 0;
    command = AD7799_GetRegisterValue(AD7799_REG_CONF,2);
    command &= ~AD7799_CONF_BO_EN;
    if(state)
		command |= AD7799_CONF_BO_EN;
    AD7799_SetRegisterValue(AD7799_REG_CONF,
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
 
static unsigned char AD7799_Calibrate(unsigned char CHx,unsigned char Gain)
{
	uint8_t R ;
	uint16_t reg;
	 
	R = 0;	
	AD7799_WaitBusy();
//	AD7799_SetGain(Gain); 
//	AD7799_SetChannel(CHx);
	AD7799_SetRegisterValue(AD7799_REG_CONF,AD7799_CONF_UNIPOLAR|AD7799_CONF_BUF|AD7799_CONF_GAIN(Gain)|AD7799_CONF_CHAN(CHx),2);
	AD7799_WaitBusy();
	reg = AD7799_GetRegisterValue(AD7799_REG_MODE,2);
	printf("before int_zero AD7799_REG_MODE is %x\r\n",reg); 
	AD7799_SetRegisterValue(AD7799_REG_MODE,0x800a,2);
	//等待校准完成
	ad_delay(160000);
	AD7799_WaitBusy(); 
	reg = AD7799_GetRegisterValue(AD7799_REG_MODE,2);
	printf("after int_zero AD7799_REG_MODE is %x\r\n",reg); 
	//进行内部校准
	//AD7799_SetMode(AD7799_MODE_CAL_INT_FULL);	
	AD7799_SetRegisterValue(AD7799_REG_MODE,0xa00a,2);
	ad_delay(160000);
	AD7799_WaitBusy();
	reg = AD7799_GetRegisterValue(AD7799_REG_MODE,2);
	printf("after cal_full AD7799_REG_MODE is %x\r\n",reg); 
			reg = AD7799_GetRegisterValue(AD7799_REG_CONF,2);
	printf("after int_zero AD7799_REG_CONF is %x\r\n",reg);
	 
	R = 1;
	return R;
} 
unsigned char AD7799_Config(void)
{
	u32 i = 0;
	u16 reg;
	
	SIM_SPI_Init(0,1,1);
	AD7799_CS_H;
	ad_delay(100);
	AD7799_CS_L;
	ad_delay(100000);
	//AD7799_WaitBusy();
	AD7799_Reset();
	ad_delay(100000);	 
	i = 0;
    while(!AD7799_Init())
    {
        i++;
        ad_delay(100);
        if(i>1000)
        {
            AD7799_Reset();
            return 1;
        }
    }
	ad_delay(100);
	AD7799_SetRegisterValue(AD7799_REG_CONF,AD7799_CONF_UNIPOLAR|AD7799_CONF_BUF,2);//AD7799_SetRegisterValue(AD7799_REG_CONF,AD7799_CONF_UNIPOLAR,2);
	
	AD7799_Calibrate(0,0);	
	AD7799_WaitBusy();
 
	reg = AD7799_GetRegisterValue(AD7799_REG_MODE,2);
	printf("AD7799_REG_MODE is %x\r\n",reg);
	AD7799_SetRegisterValue(AD7799_REG_MODE,AD7799_MODE_SEL(AD7799_MODE_CONT)|AD7799_MODE_RATE(2),2); //AD7799_MODE_PSW(x)|
	ad_delay(100000);
	AD7799_WaitBusy();
 
//	reg = AD7799_GetRegisterValue(AD7799_REG_MODE,2);
//	printf("AD7799_REG_MODE is %x\r\n",reg);
//	reg = AD7799_GetRegisterValue(AD7799_REG_CONF,2);
//	printf("AD7799_REG_CONF is %x\r\n",reg);
	ad_delay(8000);
	return 0;
}
unsigned int AD7799_GetValue(unsigned char channel)
{
	u16 reg_data;
	 u8 rdy;
	u16 try_times = 0;
		AD7799_SetChannel(channel);	
	//AD7799_SetMode(AD7799_MODE_SINGLE);
	//AD7799_SetPolar(0);
	ad_delay(10000);
	if(AD7799_WaitBusy())
		return 0;
	rdy = 0;
	while(1)
	{
		ad_delay(50000);
		rdy = AD7799_GetRegisterValue(AD7799_REG_STAT,1);
		if((rdy&0x03) == channel)
				break;
		try_times++;
		if(try_times > 10)
		{
			__nop();			  
			printfk("adc1 channel %d rdy is %d err\r\n",channel,rdy);			 
			return 0;
		}
	}
//	printf("rdy is %x\r\n",rdy);
//	if((rdy & 0x40) != 0)
//	{
//		while(AD7799_Ready())
//		{
//			ad_delay(1000);
//			AD7799_Reset();
//		}
//		GPIO_SetBits(GPIOA, GPIO_Pin_1);
//	}
//	else
//	{
//		GPIO_ResetBits(GPIOA, GPIO_Pin_1);
//	}
	reg_data = AD7799_GetRegisterValue(AD7799_REG_DATA, 2);
 	// printf("ch is %d  sig data is %x\r\n",channel,reg_data);
	return reg_data;
}
