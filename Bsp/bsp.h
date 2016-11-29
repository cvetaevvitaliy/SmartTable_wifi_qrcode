#ifndef __BSP_H
#define __BSP_H 			   
#include "stm32f10x.h"
#include "sys_tick_delay.h"
//#include "log.h"
 
#include "encryption.h"
#include "Savepartoflash.h"





#define setbit(x,y) x|=(1<<y) // 
#define clrbit(x,y) x&=~(1<<y) // 


#define BATTERYLOW_ADC   0X07FF
/*****ϵͳ״̬*****/

#define CALL_KEY_PRESS   	0x0001
#define WIFI_KEY_PRESS   	0x0002
#define NORMAL_KEY_PRESS   	0x0004
#define WIFI_CONNECT	   	0x0008
#define DATA_SEND		   	0x0010
#define SYSTEM_IDLE			0X1000


#define IWDG_ENBLE      	0
 
 
 
 
 
//#define CC1101_SURPORT 1
 
 
//#define ENBALE_GAS_POWER    	GPIO_SetBits(GPIOA,GPIO_Pin_3)
//#define DISABLE_GAS_POWER    	GPIO_ResetBits(GPIOA,GPIO_Pin_3)

 
//#define SecondTicks (uint16)RTC_GetCounter()
//#define PosBKPAddr BKP_DR2
 
 void POWERD_init(void);
 void RCC_Configuration(void);
 void PortInit_All(void);
 
 void GetSTM32ID(uint32_t *Device_Serial);
 void RestartMCU(void);
 
  
#endif
