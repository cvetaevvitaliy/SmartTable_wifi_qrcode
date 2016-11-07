#ifndef __USB_CONTRL_H
#define __USB_CONTRL_H
#include "stm32f10x.h"

#define USB_OUT_CONTRL_PORT  GPIOA
#define USB_OUT_CONTRL_PIN  GPIO_Pin_8
#define USB_WAKEUP_PORT		GPIOA
#define USB_WAKEUP_PIN		GPIO_Pin_11

#define USB_OUT_ENABLE  	GPIO_SetBits( USB_OUT_CONTRL_PORT,	USB_OUT_CONTRL_PIN )
#define USB_OUT_DISABLE  	GPIO_ResetBits( USB_OUT_CONTRL_PORT,	USB_OUT_CONTRL_PIN )



void UsbOutConfig(void );

#endif