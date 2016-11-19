#include "stm32f10x.h" 
#include "sys_adc.h" 	
#include "stm32f10x_dma.h"
 
#include "stm32f10x_adc.h"  
  

#define AD_BUFSIZE 48
#define M 3 
u16 AD_Value[AD_BUFSIZE];
u16 adc_filter[M];
u16 gI_UsbOut;u16 gV_Battery;u16 gGas;
u8 tgs_flag = 0;
void sys_adc_init(void)
{
    ADC_InitTypeDef  ADC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC| RCC_APB2Periph_AFIO,ENABLE);
    GPIO_InitStructure.GPIO_Pin  =  ADC_IOUT_PIN |ADC_BATTERY_PIN | ADC_TAG_PIN ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA,&GPIO_InitStructure); //

	GPIO_InitStructure.GPIO_Pin  =  TGS_PLUS_PIN   ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(TGS_PLUS_PORT,&GPIO_InitStructure); //
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);
    ADC_DeInit(ADC1);
	//ADC_StructInit(&ADC_InitStructure);  
    ADC_InitStructure.ADC_Mode                  =   ADC_Mode_Independent;  //¶ÀÁ¢Ä£Ê½
    ADC_InitStructure.ADC_ScanConvMode          =  	ENABLE;  	// DISABLE;    //Á¬Ğø¶àÍ¨µÀÄ£Ê½
    ADC_InitStructure.ADC_ContinuousConvMode    =   ENABLE;      //Á¬Ğø×ª»»
    ADC_InitStructure.ADC_ExternalTrigConv      =   ADC_ExternalTrigConv_None; //×ª»»²»ÊÜÍâ½ç¾ö¶¨
    ADC_InitStructure.ADC_DataAlign             =   ADC_DataAlign_Right;   //ÓÒ¶ÔÆë
    ADC_InitStructure.ADC_NbrOfChannel          =   M;       //É¨ÃèÍ¨µÀÊı
    ADC_Init(ADC1,&ADC_InitStructure);
 
	ADC_RegularChannelConfig(ADC1,ADC_Channel_0, 1,ADC_SampleTime_239Cycles5); //
	ADC_RegularChannelConfig(ADC1,ADC_Channel_1, 2,ADC_SampleTime_239Cycles5); //
	ADC_RegularChannelConfig(ADC1,ADC_Channel_2, 3,ADC_SampleTime_239Cycles5); //
  
	ADC_DMACmd(ADC1,ENABLE);//
    ADC_Cmd  (ADC1,ENABLE);             //Ê¹ÄÜ»òÕßÊ§ÄÜÖ¸¶¨µÄADC
	//ADC_TempSensorVrefintCmd(ENABLE);
    //ADC_SoftwareStartConvCmd(ADC1,ENABLE);//Ê¹ÄÜ»òÕßÊ§ÄÜÖ¸¶¨µÄADCµÄÈí¼ş×ª»»Æô¶¯¹¦ÄÜ
    ADC_ResetCalibration(ADC1);//¸´Î»Ö¸¶¨µÄADC1µÄĞ£×¼¼Ä´æÆ÷
    while(ADC_GetResetCalibrationStatus(ADC1));//»ñÈ¡ADC1¸´Î»Ğ£×¼¼Ä´æÆ÷µÄ×´Ì¬,ÉèÖÃ×´Ì¬ÔòµÈ´
    ADC_StartCalibration(ADC1);//¿ªÊ¼Ö¸¶¨ADC1µÄĞ£×¼×´Ì¬
    while(ADC_GetCalibrationStatus(ADC1)); //»ñÈ¡Ö¸¶¨ADC1
}

void sys_adc_dma_config(void)
{
    DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&NVIC_InitStructure);          // Enable the DMA Interrupt 
	
    DMA_DeInit(DMA1_Channel1);//½«DMAµÄÍ¨µÀ1¼Ä´æÆ÷ÖØÉèÎªÈ±Ê¡Öµ
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR; //DMAÍâÉèADC»ùµØÖ·
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&AD_Value;//DMAÄÚ´æ»ùµØÖ·
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; //ÄÚ´æ×÷ÎªÊı¾İ´«ÊäµÄÄ¿µÄµØ
    DMA_InitStructure.DMA_BufferSize  =	AD_BUFSIZE;//DMAÍ¨µÀµÄDMA»º´æµÄ´óĞ¡
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//ÍâÉèµØÖ·¼Ä´æÆ÷²»±ä
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //ÄÚ´æµØÖ·¼Ä´æÆ÷µİÔö
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//Êı¾İ¿í¶ÈÎª16Î»
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//Êı¾İ¿í¶ÈÎª16Î»
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//¹¤×÷ÔÚ 
    DMA_InitStructure.DMA_Priority = DMA_Priority_High; //DMAÍ¨µÀ?xÓµÓĞ¸ßÓÅÏÈ¼¶
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//DMAÍ¨µÀxÃ»ÓĞÉèÖÃÎªÄÚ´æµ½ÄÚ´æ´«Êä
    DMA_Init(DMA1_Channel1,&DMA_InitStructure);//¸ù¾İDMA_InitStructÖĞÖ¸¶¨µÄ²ÎÊı³õÊ¼»¯DMAµÄÍ¨µÀ 
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);   //open interrupt  use for filter
	
    DMA_Cmd(DMA1_Channel1,ENABLE); //Æô¶¯DMAÍ¨µÀ?
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
}


//¶¨Ê±Æ÷ ¶¨Ê± É¨Ãè°´¼ü
void TimerTGSConfig(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	NVIC_InitTypeDef NVIC_InitStructure;
	/* Enable the TIM2 gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_DeInit(TIM4);                                           //ÖØĞÂ½«TimerÉèÖÃÎªÈ±Ê¡Öµ

	TIM_InternalClockConfig(TIM4);                              //²ÉÓÃÄÚ²¿Ê±ÖÓ¸øTIM2Ìá¹©Ê±ÖÓÔ´      
	TIM_TimeBaseStructure.TIM_Prescaler = psc;			//36000-1;               //Ô¤·ÖÆµÏµÊıÎª36000-1£¬ÕâÑù¼ÆÊıÆ÷Ê±ÖÓÎª72MHz/36000 = 2kHz       
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //ÉèÖÃÊ±ÖÓ·Ö¸î      
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //ÉèÖÃ¼ÆÊıÆ÷Ä£Ê½ÎªÏòÉÏ¼ÆÊıÄ£Ê½       
	TIM_TimeBaseStructure.TIM_Period = arr;             //0xffff;                  //ÉèÖÃ¼ÆÊıÒç³ö´óĞ¡£¬ ¾Í²úÉúÒ»¸ö¸üĞÂÊÂ¼ş
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);              //½«ÅäÖÃÓ¦ÓÃµ½TIM2ÖĞ
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);	//Çå³ıÖĞ¶Ï±êÖ¾
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);	//¿ªÖĞ¶Ï
	TIM_Cmd(TIM4, ENABLE);
}
 //u16 adc_filter[3];

 
//Temperature= (1.42 - ADC_Value*3.3/4096)*1000/4.35 + 25
u16 test_tgs;
void filter(void)
{
	u8 i;
	u32  sum1 = 0;
	u32  sum2 = 0;
	u32  sum3 = 0;

	for(i=0;i<(AD_BUFSIZE/3);i++)
	{
		sum1 += AD_Value[3*i+0];
		sum2 += AD_Value[3*i+1] ;
		// sum3 += AD_Value[3*i+2] ;


	}
	if(tgs_flag)
	{
		test_tgs = AD_Value[AD_BUFSIZE-1];
//		for(i=0;i<(AD_BUFSIZE/3);i++)
//		{
//			sum3 += AD_Value[3*i+2] ;
//		}  
	}
	adc_filter[0] = (u16) (sum1/(AD_BUFSIZE/3)) ;
	adc_filter[1] = (u16) (sum2/(AD_BUFSIZE/3)) ;
	adc_filter[2] = (u16) (sum3/(AD_BUFSIZE/3)) ;
	gI_UsbOut = (u16)(adc_filter[0]*66*1000/4096);  // usb Êä³öµçÁ÷ µ¥Î» mA
	gV_Battery = (u16)(adc_filter[1]*33*(75+49.9)/(4096*49.9));   // µç³ØµçÑ¹  0.1V
	gGas  = (u16)((4095/(test_tgs*1.1))*100-160)>>1;  //  (Rs/Ro)*100
	//  Temperature = (1.42 - adc_filter[1]*3.3/4096)*1000/4.35 + 25;
	//   adc_filter[2] = (u16) (sum3/(AD_BUFSIZE));

}
void bsp_adc_config(void)
{
	sys_adc_init();
	sys_adc_dma_config();
	TimerTGSConfig(50,1439);  //1ms ÖĞ¶Ï
}
u16 CurrDataCounterEnd = 0;
void DMA1_Channel1_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_IT_TC1) != RESET)
	{
		DMA_Cmd(DMA1_Channel1,DISABLE);
		CurrDataCounterEnd=DMA_GetCurrDataCounter(DMA1_Channel1);
		DMA_ClearITPendingBit(DMA1_IT_TC1);
		filter();		
		DMA_SetCurrDataCounter(DMA1_Channel1,AD_BUFSIZE);
		DMA_Cmd(DMA1_Channel1,ENABLE);
	}
}
 
void TIM4_IRQHandler(void)
{
	static u16 tgs_ticks = 0;
	
	if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET ) /*¼ì²éTIM4¸üĞÂÖĞ¶Ï·¢ÉúÓë·ñ*/
    {
		tgs_ticks++;
		if(tgs_ticks > 997)
		{
			tgs_flag = 1;
			TGS_PLUS_ENABLE;
		}
		if(tgs_ticks == 1000)
		{
			tgs_ticks = 0;
			TGS_PLUS_DISABLE;
			tgs_flag = 0;
		}
		
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update); /*Çå³ıTIMx¸üĞÂÖĞ¶Ï±êÖ¾ */
	}	 
	
}