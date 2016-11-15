

#include "usb_contrl.h"

void UsbOutConfig(void )
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin  	= USB_OUT_CONTRL_PIN   ; 
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_Out_PP;  
 	GPIO_Init(USB_OUT_CONTRL_PORT, &GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin  	= USB_WAKEUP_PIN   ; 
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IPU; 
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_10MHz;	
 	GPIO_Init(USB_WAKEUP_PORT, &GPIO_InitStructure); 
	
	
	//USB_OUT_ENABLE;
	USB_OUT_DISABLE;
		
}
void UsbOutContrl(u8 flag)
{
	if(flag)
	{
		// enable usb out for charge
		USB_OUT_ENABLE;
	}
	else
	{
		USB_OUT_DISABLE;
	}
}