#ifndef __BSP_LEDS_H
#define __BSP_LEDS_H


#define LED1_PORT  			GPIOC
#define LED1_PIN  			GPIO_Pin_8

#define LED2_PORT  			GPIOB
#define LED2_PIN  			GPIO_Pin_15

#define LED3_PORT  			GPIOB
#define LED3_PIN  			GPIO_Pin_13

#define LED4_PORT  			GPIOB
#define LED4_PIN  			GPIO_Pin_8

#define LED5_PORT  			GPIOB
#define LED5_PIN  			GPIO_Pin_12

#define LED6_PORT  			GPIOB
#define LED6_PIN  			GPIO_Pin_14

#define LED7_PORT  			GPIOC
#define LED7_PIN  			GPIO_Pin_6

#define LED8_PORT  			GPIOC
#define LED8_PIN  			GPIO_Pin_9

#define LED9_PORT  			GPIOB
#define LED9_PIN  			GPIO_Pin_7

#define LED10_PORT  		GPIOB
#define LED10_PIN  			GPIO_Pin_6

//#define LED9_Toggle			LED9_PORT->ODR ^= LED9_PIN
void BspLed_IOConfig(void);
void RedLedOn(void);
void GreenLedOn(void);
void RedLedOff(void);
void GreenLedOff(void);
void RedLedToggle(void); 
void GreenLedToggle(void); 
void AllLedsRun(void);
#endif
