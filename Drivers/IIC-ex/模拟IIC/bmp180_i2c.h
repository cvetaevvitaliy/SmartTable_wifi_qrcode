 /**
  ******************************************************************************
  * @file   bmp180_i2c.h
  * @author  zzr
  * @version V1.0.0
  * @date    2015-04-24
  * @brief    
  ******************************************************************************
  * COPYRIGHT (C) 南京奔联软件科技有限公司 
  ******************************************************************************
  */  



#ifndef __BMP180_I2C_H
#define __BMP180_I2C_H
 

 





/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
  
/* Private variables ---------------------------------------------------------*/
 
/* Private functions ---------------------------------------------------------*/

#define Open_I2C                        	I2C1
//#define Open_I2C_CLK                    	RCC_APB1Periph_I2C1


#define I2C_SPEED               100000
#define I2C_SLAVE_ADDRESS7      0xee
					  
/* Private define ------------------------------------------------------------*/
#define AT24C01A
//#define AT24C01

#define ADDR_BMP180		0xee



/* Maximum Timeout values for flags and events waiting loops. These timeouts are
   not based on accurate values, they just guarantee that the application will 
   not remain stuck if the I2C communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */   
#define I2C_Open_FLAG_TIMEOUT         ((uint32_t)0x1000)
#define I2C_Open_LONG_TIMEOUT         ((uint32_t)(10 * I2C_Open_FLAG_TIMEOUT))

/* Private function prototypes -----------------------------------------------*/
/*
uint8_t I2C_Read(I2C_TypeDef *I2Cx,uint8_t I2C_Addr,uint8_t addr,uint8_t *buf,uint16_t num);
uint8_t I2C_Write(I2C_TypeDef *I2Cx,uint8_t I2C_Addr,uint8_t addr,uint8_t *buf,uint16_t num);
uint8_t I2C_WriteOneByte(I2C_TypeDef *I2Cx,uint8_t I2C_Addr,uint8_t addr,uint8_t value);
*/
 void I2C_GPIO_Config(void);
  //s8 I2C_routine(void) ;
	u16 BMP180_ReadTwoByte(u8 regaddr);
	u8 BMP180_ReadOneByte(u8 ReadAddr);
	void BMP180_delay_msek(u32 msek);
void BMP180_WriteOneByte(u8 WriteAddr,u8 DataToWrite);
//uint8_t I2C_Read(uint8_t I2C_Addr,uint8_t addr,uint8_t *buf,uint16_t num);

//uint8_t I2C_WriteOneByte(uint8_t DeviceAddress,uint8_t addr,uint8_t value);
//uint8_t I2C_Write(uint8_t I2C_Addr,uint8_t addr,uint8_t *buf,uint16_t num);
#endif 

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/

