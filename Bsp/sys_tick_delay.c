#include "stm32f10x.h"
#include "sys_tick_delay.h" 
#include "stm32f10x_TIM.h"
 
#define SYS_DELAY_USE  TIMER2

__IO u32 SysTicksVal32Bit;

void SYS_TickDelayConfig(void)
{
	#if (SYS_DELAY_USE == TIMER2)
	    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	  NVIC_InitTypeDef NVIC_InitStructure;
	  /* Enable the TIM2 gloabal Interrupt */
	  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);	
	
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_DeInit(TIM2);                                           //重新将Timer设置为缺省值
       
    TIM_InternalClockConfig(TIM2);                              //采用内部时钟给TIM2提供时钟源      
    TIM_TimeBaseStructure.TIM_Prescaler = 36000-1;               //预分频系数为36000-1，这样计数器时钟为72MHz/36000 = 2kHz       
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //设置时钟分割      
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //设置计数器模式为向上计数模式       
    TIM_TimeBaseStructure.TIM_Period = 0xffff;                  //设置计数溢出大小， 就产生一个更新事件
    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);              //将配置应用到TIM2中

	  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);	//清除中断标志
	  TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);	//开中断

	  TIM_Cmd(TIM2, ENABLE);
	#else
	 if( SysTick_Config( SystemCoreClock / 1000 ) )
	 {
		 //error
		 
	 }
	#endif
}
uint32_t PDInit_GetSysTick(void)
{
	 ((uint16_t*)&SysTicksVal32Bit)[0] = (uint32_t)TIM2 -> CNT;
	 return(SysTicksVal32Bit);	
}
//extern volatile uint32_t TickCounter;
u8 SysTickDelay_ms ( __IO uint32_t delay )
{
    // Wait delay ms
    uint32_t startTick = SysRunTicks;
		if(startTick == 0)
			return 0;
    while( ( SysRunTicks - startTick ) < (delay*2) );  
		return 1;
}
//	if( SysTick_Config( SystemCoreClock / 1000 ) )
//  { 
//		/* Capture error */ 
//		while (1);
//	}

//void PDInit_Timer2(void)
//{
	
//}
//该函数不能在中断函数内调用

void TIM2_IRQHandler(void)
{
	((uint16_t*)&SysTicksVal32Bit)[1] += 1;
	((uint16_t*)&SysTicksVal32Bit)[0] = (uint32_t)TIM2 -> CNT;
	
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);	//清除中断标志
}


static u8  fac_us=0;//us延时倍乘数
static u16 fac_ms=0;//ms延时倍乘数

//初始化延迟函数
//当使用ucos的时候,此函数会初始化ucos的时钟节拍
//SYSTICK的时钟固定为HCLK时钟的1/8
//SYSCLK:系统时钟
void Sys_delay_init(void)	 
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//选择外部时钟  HCLK/8
	fac_us=SystemCoreClock/8000000;	//为系统时钟的1/8   
	fac_ms=(u16)fac_us*1000;// 代表每个ms需要的systick时钟数    
}

//延时nus
//nus为要延时的us数.		    								   
void Sys_delay_us(u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*fac_us; //时间加载	  		 
	SysTick->VAL=0x00;        //清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //开始倒数	 
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	 
}
//延时nms
//注意nms的范围
//SysTick->LOAD为24位寄存器,所以,最大延时为:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK单位为Hz,nms单位为ms
//对72M条件下,nms<=1864 
void Sys_delay_ms(u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;           //清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //开始倒数  
	do
	{
 		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //关闭计数器
	SysTick->VAL =0X00;       //清空计数器	  	    
} 

