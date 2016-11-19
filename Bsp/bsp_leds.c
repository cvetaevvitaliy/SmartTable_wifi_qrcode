#include "stm32f10x.h"
#include "bsp_leds.h"
#include "wifi_esp.h" 
#include "wifi_esp_config.h"
#include "sys_tick_delay.h"
void bsp_led_config(void)
{
	
	
}
void GpioConfig(GPIO_TypeDef* GPIOx ,uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin  	= GPIO_Pin   ; 
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_Out_PP;  
 	GPIO_Init(GPIOx, &GPIO_InitStructure); 
	
}
 void BspLed_IOConfig(void)
 {
 
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB| RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|RCC_APB2Periph_AFIO,ENABLE);// 

//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);//关闭jtag，使能SWD，可以用SWD模式调试
	
	 GpioConfig( LED1_PORT,	LED1_PIN );
	 GpioConfig( LED2_PORT,	LED2_PIN );
	 GpioConfig( LED3_PORT,	LED3_PIN );
	 GpioConfig( LED4_PORT,	LED4_PIN );
	 GpioConfig( LED5_PORT,	LED5_PIN );
	 GpioConfig( LED6_PORT,	LED6_PIN );
	 GpioConfig( LED7_PORT,	LED7_PIN );
	 GpioConfig( LED8_PORT,	LED8_PIN );
	 GpioConfig( LED9_PORT,	LED9_PIN );
	 GpioConfig( LED10_PORT,LED10_PIN );
 }
 void RedLedOn(void)
 {
	 GPIO_ResetBits( LED1_PORT,	LED1_PIN );
	 GPIO_ResetBits( LED2_PORT,	LED2_PIN );
	 GPIO_ResetBits( LED3_PORT,	LED3_PIN );
	 GPIO_ResetBits( LED4_PORT,	LED4_PIN );
	 GPIO_ResetBits( LED10_PORT,	LED10_PIN );
	 
	 
 }
  void GreenLedOn(void)
 {
	 GPIO_ResetBits( LED6_PORT,	LED6_PIN );
	 GPIO_ResetBits( LED7_PORT,	LED7_PIN );
	 GPIO_ResetBits( LED8_PORT,	LED8_PIN );
	 GPIO_ResetBits( LED9_PORT,	LED9_PIN );

	 GPIO_ResetBits( LED5_PORT,	LED5_PIN ); 
 }
  void RedLedOff(void)
 {
	 GPIO_SetBits( LED1_PORT,	LED1_PIN );
	 GPIO_SetBits( LED2_PORT,	LED2_PIN );
	 GPIO_SetBits( LED3_PORT,	LED3_PIN );
	 GPIO_SetBits( LED4_PORT,	LED4_PIN );

 	 GPIO_SetBits( LED10_PORT,	LED10_PIN );

 }
  void GreenLedOff(void)
 {
	 GPIO_SetBits( LED6_PORT,	LED6_PIN );
	 GPIO_SetBits( LED7_PORT,	LED7_PIN );
	 GPIO_SetBits( LED8_PORT,	LED8_PIN );
	 GPIO_SetBits( LED9_PORT,	LED9_PIN );

	 GPIO_SetBits( LED5_PORT,	LED5_PIN );
 }
 void RedLedToggle(void)
 {
	 LED1_PORT->ODR ^= LED1_PIN;
	 LED2_PORT->ODR ^= LED2_PIN;
	 LED3_PORT->ODR ^= LED3_PIN;
	 LED4_PORT->ODR ^= LED4_PIN;
	 LED10_PORT->ODR ^= LED10_PIN;
	
 }
 void GreenLedToggle(void)
 {
	  LED5_PORT->ODR ^= LED5_PIN;
	 LED6_PORT->ODR ^= LED6_PIN;
	 LED7_PORT->ODR ^= LED7_PIN;
	 LED8_PORT->ODR ^= LED8_PIN;
	 LED9_PORT->ODR ^= LED9_PIN;
 }
void AllLedsRun(void)
{
	struct  WIFI_Dev *pWifiDev = GetWifiDev();
	static u32 LedTicks = 0;
	
	if((u32)(SysRunTicks - LedTicks ) > 1000)
	{
		LedTicks = SysRunTicks;	 
		if(pWifiDev->wStatus != TCP_CONNECT_OK)
		{
//			if(pWifiDev->wStatus == WIFI_SMART_LINK)
//			{
//				
//				GreenLedToggle();
//				RedLedOff();
//			}
//			else
//			{
				RedLedToggle();
				GreenLedOff();
//			}
		}
		else 
		{
			RedLedOff();
			GreenLedOn();
			
		}
	}
}
