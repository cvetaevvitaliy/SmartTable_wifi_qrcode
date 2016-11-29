#ifndef __SYS_TICK_DELAY_H  
#define __SYS_TICK_DELAY_H 			   
#include "stm32f10x.h"


#define SysRunTicks 	PDInit_GetSysTick()
#define SecondTicks 	((u16)(PDInit_GetSysTick()/2000))

 
void SYS_TickDelayConfig(void); 	
void Sys_delay_init(void);
void Sys_delay_us(u32 nus);
void Sys_delay_ms(u16 nms);
u8 SysTickDelay_ms ( __IO uint32_t delay );
uint32_t PDInit_GetSysTick(void);
#endif



