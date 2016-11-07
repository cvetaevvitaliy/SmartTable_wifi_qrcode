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
void I2C_Configuration(void);
uint8_t I2C_Read(I2C_TypeDef *I2Cx,uint8_t I2C_Addr,uint8_t addr,uint8_t *buf,uint16_t num);
uint8_t I2C_Write(I2C_TypeDef *I2Cx,uint8_t I2C_Addr,uint8_t addr,uint8_t *buf,uint16_t num);
uint8_t I2C_WriteOneByte(I2C_TypeDef *I2Cx,uint8_t I2C_Addr,uint8_t addr,uint8_t value);

#endif 

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/

