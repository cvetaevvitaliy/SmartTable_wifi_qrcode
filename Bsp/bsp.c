
/************************************************
// Copyright (c) 2016,�Ͼ��i������Ƽ����޹�˾
// All rights reserved
//GASBOX 	   
//�޸�����:2015/07/03
//�汾��V1.1
//��Ȩ
//���ߣ�zzr
***********************************************/

//#include "AllStruct.h" 

#include "bsp.h"  
#include "stm32f10x_dma.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_adc.h"
#define STM32_UNIQUEID_ADDR   0x1FFFF7E8
/**
 * �������뷨
 */

//extern UART2_BufSTRUCT UART2_Buf;
//  void POWERD_init(void)  //ϵͳ���߿��� 
//  {
//   GPIO_InitTypeDef  GPIO_InitStructure;
//  	
//   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOE, ENABLE);	 //ʹ��PB,PE�˿�ʱ��
// 	
//   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;				 //power-->PB.15 �˿�����
//   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
//   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
//   GPIO_Init(GPIOB, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOB.15

//  }
 void PDInit_GPIOPortA(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/////////////////////////////////////////////////////////////////////////////////////////////////
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
 
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|\
								  GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7   ;      
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
}

 void PDInit_GPIOPortB(void)
{
//   //PB10��11����,����ΪGPIO���
  GPIO_InitTypeDef GPIO_InitStructure;
/////////////////////////////////////////////////////////////////////////////////////////////////
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
  //GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
		// led
  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0| GPIO_Pin_13|GPIO_Pin_14|\
								  GPIO_Pin_15|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_5 ; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB,&GPIO_InitStructure);
 
// ///////////////////////////////////////////////////////////////////////////////////////////////////	 
}
 void PDInit_GPIOPortE(void)
{
//	GPIO_InitTypeDef GPIO_InitStructure;
	 
}
void PDInit_UART1(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	USART_InitTypeDef USART_InitStructure;
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO|RCC_APB2Periph_USART1 , ENABLE );
	 
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode=  GPIO_Mode_IN_FLOATING; //GPIO_Mode_IPU
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	/********
	*������ 38400
	************/
	USART_InitStructure.USART_BaudRate            = 19200  ;    
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;
	USART_InitStructure.USART_Parity              = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	//USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
	USART_ClearFlag(USART1,USART_FLAG_TC);
	USART_Cmd(USART1, ENABLE);
			
}


void PDInit_UART2(void)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	USART_InitTypeDef USART_InitStructure;
	  NVIC_InitTypeDef NVIC_InitStructure;
	
 	RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART2 , ENABLE );
	 
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode=  GPIO_Mode_IN_FLOATING; //GPIO_Mode_IPU
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	
		
	  /* Enable the TIM2 gloabal Interrupt */
	  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);	
	/********
	*������ 38400
	************/
	USART_InitStructure.USART_BaudRate            = 9600  ;    
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;
	USART_InitStructure.USART_Parity              = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	//USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
	USART_ClearFlag(USART2,USART_FLAG_TC);
	
    
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART2, ENABLE);
			
}
////////////////////////////////////DMA1???///////////////////
void PDInit_UART2RXDDMA(void)
{
//	DMA_InitTypeDef DMA_InitStruct;
//	//
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
//	//
//	DMA_DeInit(DMA1_Channel6);
//	DMA_InitStruct.DMA_PeripheralBaseAddr=(uint32_t)&(USART2->DR); //USART->DR
////	DMA_InitStruct.DMA_MemoryBaseAddr= (uint32_t)UART2_Buf.Buf ;
//	DMA_InitStruct.DMA_DIR=DMA_DIR_PeripheralSRC;//DMA_DIR_PeripheralDST
//	DMA_InitStruct.DMA_BufferSize= UART2_MAXBUFDEEP;
//	DMA_InitStruct.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
//	DMA_InitStruct.DMA_MemoryInc=DMA_MemoryInc_Enable;
//	DMA_InitStruct.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;
//	DMA_InitStruct.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;
//	DMA_InitStruct.DMA_Mode=DMA_Mode_Normal;//DMA_Mode_Normal;//Circular;
//	DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh ;//DMA_Priority_High;//;//VeryHigh  Medium    w
//	DMA_InitStruct.DMA_M2M=DMA_M2M_Disable;
//	DMA_Init(DMA1_Channel6,&DMA_InitStruct);
//	//
//	DMA_ITConfig(DMA1_Channel6,DMA_IT_TC|DMA_IT_HT|DMA_IT_TE,DISABLE); //  w
//	//
//	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
//	DMA_Cmd(DMA1_Channel6,ENABLE);
  
//////////////////////////////////////////////////////////////////////////
}

 




void PortInit_All(void)
{

////////////////////////////////////        
//    PDInit_GPIOPortA();
//    PDInit_GPIOPortB();
//	
//	PDInit_UART1();
//	PDInit_UART2(); 
//	PDInit_Timer2();
	//PDInit_UART2RXDDMA();
	NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x0 );
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}
void RCC_Configuration(void)
{
	//�ο���ַ��http://blog.csdn.net/iamlvshijie/article/details/9249545
	ErrorStatus HSEStartUpStatus; 
	RCC_DeInit();     //����RCC�Ĵ�����������ΪĬ��ֵ
	RCC_HSEConfig (RCC_HSE_ON);//���ⲿ����ʱ�Ӿ���HSE��
	HSEStartUpStatus = RCC_WaitForHSEStartUp();//�ȴ��ⲿʱ�Ӿ����ȶ�����
	if(HSEStartUpStatus == SUCCESS)//SUCCESS��HSE�����ȶ��Ҿ���
	{
		//FLASH_PrefetchBufferCmd(ENABLE);          //����FLASH��Ԥȡ����
		//FLASH_SetLatency(FLASH_Latency_2);      //FLASH�ӳ���������
		RCC_HCLKConfig(RCC_SYSCLK_Div1);//����AHBʱ�Ӳ���Ƶ
		RCC_PCLK2Config(RCC_HCLK_Div1); //����APB2ʱ�Ӳ���Ƶ
		RCC_PCLK1Config(RCC_HCLK_Div2); //����APB1ʱ�Ӷ���Ƶ ��Ӧ�Ķ�ʱ����ʱ��Ϊ2��Ƶ
//	FLASH_SetLatency(FLASH_Latency_2);//����FLASH�洢����ʱ��������FLASH_Latency_2 2��ʱ����
//	FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);//ѡ��FLASHԤȡָ�����ģʽ��Ԥȡָ����ʹ��
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1,RCC_PLLMul_9);//����PLL   
		RCC_PLLCmd(ENABLE); //PLLʱ��ʹ��
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		while(RCC_GetSYSCLKSource()!=0x08);//�ȴ�������
	}
	else
	{
		__nop();
	}
	RCC_LSICmd(ENABLE);
  while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);
 
	//��������˿�ʱ��
	//ʹ��PA~PE�˿�ʱ��IN1~IN32
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);	 
//	//��������1��2��3ʱ��
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	//ʹ��USART1ʱ��
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//ʹ��USART2��USART3ʱ��
//	
	/* Configure HCLK clock as SysTick clock source. */
	SysTick_CLKSourceConfig( SysTick_CLKSource_HCLK );
	//������ʱ��
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);//
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);//
}
void bsp_initial(void)
{
	RCC_Configuration();
}


/**********************************************************
* �������� ---> ��ȡCPU 96λΨһID
* ��ڲ��� ---> none
* ������ֵ ---> 
* ����˵�� ---> none
**********************************************************/
void GetSTM32ID(uint32_t *Device_Serial)
{
	//uint32_t Device_Serial0,Device_Serial1,Device_Serial2;
	*Device_Serial++ = *(uint32_t*)(STM32_UNIQUEID_ADDR);      //12 Bytes Serial Number
	*Device_Serial++ = *(uint32_t*)(STM32_UNIQUEID_ADDR + 4);
	*Device_Serial++ = *(uint32_t*)(STM32_UNIQUEID_ADDR + 8);	
}

 void RestartMCU(void)
{
	__set_FAULTMASK(1);  //  
	NVIC_SystemReset(); // 
}

//////////////////////////////////////////////////////////////////////
////ϵͳʱ�����ʵ�ֺ��� 
//////////////////////////////////////////////////////////////////////
//uint32_t SysTicksVal32Bit;
//void TIM2_IRQHandler(void)
//{
//	((uint16_t*)&SysTicksVal32Bit)[1] += 1;
//	((uint16_t*)&SysTicksVal32Bit)[0] = (uint32_t)TIM2 -> CNT;
//	
//	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);	//����жϱ�־
//}
////�ú����������жϺ����ڵ���
//uint32_t PDInit_GetSysTick(void)
//{
//	 ((uint16_t*)&SysTicksVal32Bit)[0] = (uint32_t)TIM2 -> CNT;
//	 return(SysTicksVal32Bit);	
//}
 
