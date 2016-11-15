#ifndef __SYS_ADC_H
#define __SYS_ADC_H 

 
 
#define ADC_IOUT_PIN		 	GPIO_Pin_0
#define ADC_BATTERY_PIN		 	GPIO_Pin_1
#define ADC_TAG_PIN		 		GPIO_Pin_2
#define TGS_PLUS_PORT			GPIOC
#define TGS_PLUS_PIN			GPIO_Pin_7

#define TGS_PLUS_ENABLE			GPIO_SetBits(TGS_PLUS_PORT,TGS_PLUS_PIN)
#define TGS_PLUS_DISABLE		GPIO_ResetBits(TGS_PLUS_PORT,TGS_PLUS_PIN)



//void sys_adc_init(void);
//void sys_adc_dma_config(void);
void bsp_adc_config(void);
 
 


#endif
