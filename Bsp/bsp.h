#ifndef __BSP_H
#define __BSP_H 			   
#include "stm32f10x.h"
#include "sys_tick_delay.h"
//#include "log.h"
//#include "printf_embedded.h" 
#include "encryption.h"
#include "Savepartoflash.h"





#define setbit(x,y) x|=(1<<y) // 
#define clrbit(x,y) x&=~(1<<y) // 


#define BATTERYLOW_ADC   0X07FF
/*****系统状态*****/

#define CALL_KEY_PRESS   	0x0001
#define WIFI_KEY_PRESS   	0x0002
#define NORMAL_KEY_PRESS   	0x0004
#define WIFI_CONNECT	   	0x0008
#define DATA_SEND		   	0x0010
#define SYSTEM_IDLE			0X1000


#define IWDG_ENBLE      	0
 
 
 
 
 
//#define CC1101_SURPORT 1
#define LED_TOGGLE				GPIOB->ODR ^= GPIO_Pin_0
#define LED_ON					GPIO_ResetBits(GPIOB,GPIO_Pin_0)
#define LED_OFF					GPIO_SetBits(GPIOB,GPIO_Pin_0)

 #define D1_RGB_R_ON 			GPIO_ResetBits(GPIOB,GPIO_Pin_5);GPIO_SetBits(GPIOB,GPIO_Pin_6);GPIO_SetBits(GPIOB,GPIO_Pin_7)
 #define D2_RGB_R_ON			GPIO_ResetBits(GPIOB,GPIO_Pin_13);GPIO_SetBits(GPIOB,GPIO_Pin_14);GPIO_SetBits(GPIOB,GPIO_Pin_15)
 #define D3_RGB_R_ON			GPIO_ResetBits(GPIOA,GPIO_Pin_2);GPIO_SetBits(GPIOA,GPIO_Pin_3);GPIO_SetBits(GPIOA,GPIO_Pin_4)
 #define D4_RGB_R_ON			GPIO_ResetBits(GPIOA,GPIO_Pin_5);GPIO_SetBits(GPIOA,GPIO_Pin_6);GPIO_SetBits(GPIOA,GPIO_Pin_7)

 
 #define D1_RGB_G_ON			GPIO_ResetBits(GPIOB,GPIO_Pin_6);GPIO_SetBits(GPIOB,GPIO_Pin_5);GPIO_SetBits(GPIOB,GPIO_Pin_7)
 #define D2_RGB_G_ON			GPIO_ResetBits(GPIOB,GPIO_Pin_14);GPIO_SetBits(GPIOB,GPIO_Pin_13);GPIO_SetBits(GPIOB,GPIO_Pin_15)
 #define D3_RGB_G_ON			GPIO_ResetBits(GPIOA,GPIO_Pin_3);GPIO_SetBits(GPIOA,GPIO_Pin_2);GPIO_SetBits(GPIOA,GPIO_Pin_4)
 #define D4_RGB_G_ON			GPIO_ResetBits(GPIOA,GPIO_Pin_6);GPIO_SetBits(GPIOA,GPIO_Pin_5);GPIO_SetBits(GPIOA,GPIO_Pin_7)

 
 #define D1_RGB_B_ON			GPIO_ResetBits(GPIOB,GPIO_Pin_7);GPIO_SetBits(GPIOB,GPIO_Pin_6);GPIO_SetBits(GPIOB,GPIO_Pin_5)
 #define D2_RGB_B_ON			GPIO_ResetBits(GPIOB,GPIO_Pin_15);GPIO_SetBits(GPIOB,GPIO_Pin_13);GPIO_SetBits(GPIOB,GPIO_Pin_14)
 #define D3_RGB_B_ON			GPIO_ResetBits(GPIOA,GPIO_Pin_4);GPIO_SetBits(GPIOA,GPIO_Pin_2);GPIO_SetBits(GPIOA,GPIO_Pin_3)
 #define D4_RGB_B_ON		 	GPIO_ResetBits(GPIOA,GPIO_Pin_7);GPIO_SetBits(GPIOA,GPIO_Pin_5);GPIO_SetBits(GPIOA,GPIO_Pin_6)
 
 
 
 #define D1_RGB_ALL_OFF			GPIO_SetBits(GPIOB,GPIO_Pin_7);GPIO_SetBits(GPIOB,GPIO_Pin_6);GPIO_SetBits(GPIOB,GPIO_Pin_5)
 #define D2_RGB_ALL_OFF			GPIO_SetBits(GPIOB,GPIO_Pin_15);GPIO_SetBits(GPIOB,GPIO_Pin_13);GPIO_SetBits(GPIOB,GPIO_Pin_14)
 #define D3_RGB_ALL_OFF			GPIO_SetBits(GPIOA,GPIO_Pin_4);GPIO_SetBits(GPIOA,GPIO_Pin_2);GPIO_SetBits(GPIOA,GPIO_Pin_3)
 #define D4_RGB_ALL_OFF			GPIO_SetBits(GPIOA,GPIO_Pin_6);GPIO_SetBits(GPIOA,GPIO_Pin_7);GPIO_SetBits(GPIOA,GPIO_Pin_5)
 
 
 #define USB_OUT_POWER_ON   	GPIO_SetBits(GPIOA,GPIO_Pin_0)
 #define USB_OUT_POWER_OFF   	GPIO_ResetBits(GPIOA,GPIO_Pin_0)

//#define ENBALE_GAS_POWER    	GPIO_SetBits(GPIOA,GPIO_Pin_3)
//#define DISABLE_GAS_POWER    	GPIO_ResetBits(GPIOA,GPIO_Pin_3)

#define SysRunTicks 	PDInit_GetSysTick()
#define SecondTicks 	((u16)(PDInit_GetSysTick()/2000))
//#define SecondTicks (uint16)RTC_GetCounter()
//#define PosBKPAddr BKP_DR2
typedef struct _DATA_STRUCT
{
	u16 wSendCounter;
	s16 wTemperature;
	u16 wHumidity;
	u16 wAt_pressure ;
	u16 wPM25;		 
	u16 wPM10;
	u16 wGas;
	u16 wRecCounter;
	u16 wBackup0;
	u16 wBackup1;
	
}T_DATA_TRANS;
 void POWERD_init(void);
 void RCC_Configuration(void);
 void PortInit_All(void);
 uint32_t PDInit_GetSysTick(void);
 void GetSTM32ID(uint32_t *Device_Serial);
 void RestartMCU(void);
 
#define FA_OPEN_DEFAULT			(uint8_t)(FA_OPEN_EXISTING | FA_READ | FA_WRITE)	//可读写操作
#define FA_OPEN_READONLY		(uint8_t)(FA_OPEN_EXISTING | FA_READ)				//只读取，不执行写
#define FA_OPEN_ADD_DATA		(uint8_t)(FA_OPEN_ALWAYS | FA_READ | FA_WRITE)		//文件不存在则创建新文件
																					//可用f_lseek函数在文件上追加数据
#define FA_OPEN_NEW_FAIL		(uint8_t)(FA_CREATE_NEW | FA_READ | FA_WRITE)		//新建文件，如果存在则失败
#define FA_OPEN_NEW_COVER		(uint8_t)(FA_CREATE_ALWAYS | FA_READ | FA_WRITE)	//新建文件，如果存在则覆盖

#endif
