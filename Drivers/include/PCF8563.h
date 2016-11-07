#ifndef __PCF8563_H
#define __PCF8563_H
#include "stm32f10x.h"
#include  "zRTC_Timer.h"
 
#define PFC8563_IIC_PORT   GPIOD
#define PFC8563_IIC_Clock   RCC_APB2Periph_GPIOD
#define PFC8563_IIC_SCL    GPIO_Pin_10
#define PFC8563_IIC_SDA    GPIO_Pin_11

#define PFC_SCL_H()   GPIO_SetBits(PFC8563_IIC_PORT, PFC8563_IIC_SCL)
#define PFC_SCL_L()	  GPIO_ResetBits(PFC8563_IIC_PORT, PFC8563_IIC_SCL)
#define PFC_SDA_H()	  GPIO_SetBits(PFC8563_IIC_PORT, PFC8563_IIC_SDA)
#define PFC_SDA_L()	  GPIO_ResetBits(PFC8563_IIC_PORT, PFC8563_IIC_SDA)
#define PFC_READ_SDA() GPIO_ReadInputDataBit(PFC8563_IIC_PORT,PFC8563_IIC_SDA)

//pcf8563 time reg 
#define REG_SECOND    0x02 //????  
#define REG_MINUTE    0x03 //????  
#define REG_HOUR   0x04 //????  
#define REG_DAY    0x05 //????  
#define REG_WEEK   0x06 //????  
#define REG_MONTH 0x07 //????  
#define REG_YEAR   0x08 //????  
#define PCF_READ_ADDR 	0xA3 //  
#define PCF_WRITE_ADDR 	0xA2 // 

 
 
void PCF8563_Init(void); // 
uint8_t PFC_SET_TIME(DateTime * TimeToSet);
s8 PFC8563_GET_TIME(DateTime * TimeToGet);
 
 



#endif



