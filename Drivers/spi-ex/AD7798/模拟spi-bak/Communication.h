/***************************************************************************//**
 *   @file   Communication.h
 **/
/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
 
 #include "stm32f10x.h"
/******************************************************************************/
/* GPIO Definitions                                                           */
/******************************************************************************/

#define AD7799_CS_PIN       GPIO_Pin_4  // Add your code here.
#define AD7799_CS_PORT			GPIOA
//#define AD7799_CS_PIN_OUT
#define AD7799_CS_L        GPIO_ResetBits(AD7799_CS_PORT, AD7799_CS_PIN) // Add your code here.
#define AD7799_CS_H     	 GPIO_SetBits(AD7799_CS_PORT, AD7799_CS_PIN)  // Add your code here.


#define AD_MOSI_H 	GPIO_SetBits(GPIOA, GPIO_Pin_7)  
#define AD_MOSI_L 	GPIO_ResetBits(GPIOA, GPIO_Pin_7)  
#define AD_SCLK_H 	GPIO_SetBits(GPIOA, GPIO_Pin_5)  
#define AD_SCLK_L 	GPIO_ResetBits(GPIOA, GPIO_Pin_5)  
#define AD_MISO 		GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) 
#define AD_MISO_H	GPIO_SetBits(GPIOA, GPIO_Pin_6) 
#define AD_MISO_L	GPIO_ResetBits(GPIOA, GPIO_Pin_6) 
#define AD_DRY			GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6)		//µÍµçÆ½Îª¿ÕÏÐ

 #define DELAYS10 __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();


/******************************************************************************/
/* Functions Prototypes                                                       */
/******************************************************************************/
/* Initializes the SPI communication peripheral. */
//unsigned char SIM_SPI_Init(unsigned char lsbFirst,
//                       unsigned long clockFreq,
//                       unsigned char clockPol,
//                       unsigned char clockPha);
/* Writes data to SPI. */
unsigned char SIM_SPI_Write(unsigned char* data,
                        unsigned char bytesNumber);
/* Reads data from SPI. */
unsigned char SIM_SPI_Read(unsigned char* data,
                       unsigned char bytesNumber);
 void SIM_SPI_Init(void);

//void A7799_Ini(void);
//u32 ReadAd7799ConversionData(void);
//void WriteByteToAd7799(unsigned char WriteData);
