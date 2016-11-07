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

    TIM_DeInit(TIM2);                                           //���½�Timer����Ϊȱʡֵ
       
    TIM_InternalClockConfig(TIM2);                              //�����ڲ�ʱ�Ӹ�TIM2�ṩʱ��Դ      
    TIM_TimeBaseStructure.TIM_Prescaler = 36000-1;               //Ԥ��Ƶϵ��Ϊ36000-1������������ʱ��Ϊ72MHz/36000 = 2kHz       
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //����ʱ�ӷָ�      
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ü�����ģʽΪ���ϼ���ģʽ       
    TIM_TimeBaseStructure.TIM_Period = 0xffff;                  //���ü��������С�� �Ͳ���һ�������¼�
    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);              //������Ӧ�õ�TIM2��

	  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);	//����жϱ�־
	  TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);	//���ж�

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
//�ú����������жϺ����ڵ���

void TIM2_IRQHandler(void)
{
	((uint16_t*)&SysTicksVal32Bit)[1] += 1;
	((uint16_t*)&SysTicksVal32Bit)[0] = (uint32_t)TIM2 -> CNT;
	
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);	//����жϱ�־
}


static u8  fac_us=0;//us��ʱ������
static u16 fac_ms=0;//ms��ʱ������

//��ʼ���ӳٺ���
//��ʹ��ucos��ʱ��,�˺������ʼ��ucos��ʱ�ӽ���
//SYSTICK��ʱ�ӹ̶�ΪHCLKʱ�ӵ�1/8
//SYSCLK:ϵͳʱ��
void Sys_delay_init(void)	 
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//ѡ���ⲿʱ��  HCLK/8
	fac_us=SystemCoreClock/8000000;	//Ϊϵͳʱ�ӵ�1/8   
	fac_ms=(u16)fac_us*1000;// ����ÿ��ms��Ҫ��systickʱ����    
}

//��ʱnus
//nusΪҪ��ʱ��us��.		    								   
void Sys_delay_us(u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*fac_us; //ʱ�����	  		 
	SysTick->VAL=0x00;        //��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //��ʼ����	 
	do
	{
		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //�رռ�����
	SysTick->VAL =0X00;       //��ռ�����	 
}
//��ʱnms
//ע��nms�ķ�Χ
//SysTick->LOADΪ24λ�Ĵ���,����,�����ʱΪ:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK��λΪHz,nms��λΪms
//��72M������,nms<=1864 
void Sys_delay_ms(u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;//ʱ�����(SysTick->LOADΪ24bit)
	SysTick->VAL =0x00;           //��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;          //��ʼ����  
	do
	{
 		temp=SysTick->CTRL;
	}
	while(temp&0x01&&!(temp&(1<<16)));//�ȴ�ʱ�䵽��   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;       //�رռ�����
	SysTick->VAL =0X00;       //��ռ�����	  	    
} 

