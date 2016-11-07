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

#define ADC2_CS_PIN       GPIO_Pin_12  // Add your code here.
#define ADC2_CS_PORT			GPIOB
#define ADC2_CLK_PIN       GPIO_Pin_13
#define ADC2_MOSI_PIN       GPIO_Pin_15
#define ADC2_MISO_PIN       GPIO_Pin_14

//#define ADC2_CS_PIN_OUT
#define ADC2_CS_L        GPIO_ResetBits(ADC2_CS_PORT, ADC2_CS_PIN) // Add your code here.
#define ADC2_CS_H     	 GPIO_SetBits(ADC2_CS_PORT, ADC2_CS_PIN)  // Add your code here.


#define ADC2_MOSI_H 	GPIO_SetBits(ADC2_CS_PORT, ADC2_MOSI_PIN)  
#define ADC2_MOSI_L 	GPIO_ResetBits(ADC2_CS_PORT, ADC2_MOSI_PIN)  
#define ADC2_SCLK_H 	GPIO_SetBits(ADC2_CS_PORT, ADC2_CLK_PIN)  
#define ADC2_SCLK_L 	GPIO_ResetBits(ADC2_CS_PORT, ADC2_CLK_PIN)  
#define ADC2_MISO 		GPIO_ReadInputDataBit(ADC2_CS_PORT, ADC2_MISO_PIN) 
#define ADC2_MISO_H	GPIO_SetBits(ADC2_CS_PORT, ADC2_MISO_PIN) 
#define ADC2_MISO_L	GPIO_ResetBits(ADC2_CS_PORT, ADC2_MISO_PIN) 
#define ADC2_DRY			GPIO_ReadInputDataBit(ADC2_CS_PORT, ADC2_MISO_PIN)		//µÍµçÆ½Îª¿ÕÏÐ

 #define DELAYS10 __nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();


/******************************************************************************/
/* Functions Prototypes                                                       */
/******************************************************************************/
/* Initializes the SPI communication peripheral. */
//unsigned char ADC2_SPI_Init(unsigned char lsbFirst,
//                       unsigned long clockFreq,
//                       unsigned char clockPol,
//                       unsigned char clockPha);
/* Writes data to SPI. */
unsigned char ADC2_SPI_Write(unsigned char* data,
                        unsigned char bytesNumber);
/* Reads data from SPI. */
unsigned char ADC2_SPI_Read(unsigned char* data,
                       unsigned char bytesNumber);
 void ADC2_SPI_Init(void);
unsigned long ADC2_GetRegisterValue(unsigned char regAddress, unsigned char size);
unsigned char ADC2_Config(void);
unsigned int ADC2_GetValue(unsigned char channel);
//void A7799_Ini(void);
//u32 ReadADC2ConversionData(void);
//void WriteByteToADC2(unsigned char WriteData);
