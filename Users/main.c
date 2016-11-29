#include "stm32f10x.h"
#include "GUI.h"
#include "QDTFT_demo.h"
#include "Lcd_Driver.h"
#include "printf_em.h"
#include "QRCodeApp.h"
#include "bsp.h"
#include "sys_tick_delay.h"
#include "bsp_leds.h"
#include "wifi_esp_config.h"
#include "sys_adc.h"
#include "sht1x.h"
#include "usb_contrl.h"
#include "Buttons.h"
#include "bsp_serial.h"


//#define QRCODE_Y 80
u8 	gMcuID[12];
u16 Humi;
s16 Temp;
u8 lcd_page = 0;

void DISPLAY_RENCODE_TO_TFT(u8 *qrcode_data);
void DISPLAY_Temp(void);

#if 1
#pragma import(__use_no_semihosting)       
//标准库需要的支持函数                
struct FILE
{
 int handle;
 
 
 
};

struct FILE __stdout;
//定义_sys_exit()以避免使用半主机模式   
_sys_exit(int x)
{
 x = x;
}
//重定义fputc函数
int fputc(int ch )
{
	USART1->DR = (u8) ch;     
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕  
	return ch;
}
int putchar(int ch )
{
	USART1->DR = (u8) ch;     
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕  
	return ch;
}
#endif
int main(void)
{
	static u8 old_lcd_page = 0;
	u8 page_ticks = 0;
	u32 WifiCheckTicks = 0;
	RCC_Configuration();
	PortInit_All();
	GetSTM32ID((u32*)gMcuID);
	
	 
	SYS_TickDelayConfig();
	Sys_delay_init();
	bsp_serial_config();
	em_printf("initializing...\r\n");

	BspLed_IOConfig();
	ButtonsConfig();
	bsp_adc_config();
	sht_io_config();	
	Wifi_Init();
	
	SysTickDelay_ms(1000);
	Lcd_Init();
	SysTickDelay_ms(1000);
	DISPLAY_RENCODE_TO_TFT("www.baidu.com");
	em_printf("display qrcode \r\n");
	UsbOutConfig();
	while(1)
	{
		 
		CycleBufPro();
		if((u32)(SecondTicks - WifiCheckTicks) > 5)
		{ 
		//	em_printf(" ss %d \r\n",SecondTicks);
			WifiCheckTicks  = SecondTicks; 
			wifi_fsm();
			//DISPLAY_RENCODE_TO_TFT("1234567890122456");
			 
		}
		AllLedsRun();
		Get_SHT11(&Humi,&Temp);
		 
		SysTickDelay_ms(1000);
		if(old_lcd_page != lcd_page)
		{
			old_lcd_page = lcd_page;
			switch(lcd_page)
			{
				case 0:
					DISPLAY_RENCODE_TO_TFT("www.baidu.com");
					break;
				case 1:
					DISPLAY_Temp( );
					break;
				default:
					break;
			}
		}
		else
		{
			page_ticks++;
			if((page_ticks & 0x1F) == 0)
			{
				page_ticks = 0;
				switch(lcd_page)
				{
					case 0:
						DISPLAY_RENCODE_TO_TFT("www.baidu.com");
					break;
					case 1:
						DISPLAY_Temp( );
					break;
					default:
						break;
				}
			}
		}
		
	}
}
void DISPLAY_RENCODE_TO_TFT(u8 *qrcode_data)
{
	u8 i,j;
	u16 x,y,p;
	u8 qrencode_buff[12];			//存放LCD ID字符串
	EncodeData((char *)qrcode_data);
	//LCD_Fill(10,110,15,115,BLACK);
//	LCD_Fill(0,40,240,320,WHITE);
	sprintf((char*)qrencode_buff,"size:%d",m_nSymbleSize);//将LCD ID打印到lcd_id数组。
//	LCD_ShowString(10,40,200,16,16,qrencode_buff);		//显示LCD ID	 
  
//	OLED_Fill(10,10,100,50,1);
	Lcd_Clear(GRAY0);
	 
	if(m_nSymbleSize*2>128)	
	{
	//	LCD_ShowString(10,60,200,16,16,(u8 *)"The QR Code is too large!");//太大显示不下
		 
		return;
	}
	for(i=0;i<10;i++)
	{
		if((m_nSymbleSize*i*2)>128)	break;
	}
	p=(i-1)*2;//点大小
	//p = 3;
	x=(128-m_nSymbleSize*p)/2;
	y=0;
	sprintf((char*)qrencode_buff,"piont:%d",p);//将LCD ID打印到lcd_id数组。
	//LCD_ShowString(10,60,200,16,16,qrencode_buff);
	//OLED_ShowString(10,60,qrencode_buff,12); 
	for(i=0;i<m_nSymbleSize;i++)
	{
		for(j=0;j<m_nSymbleSize;j++)
		{
			//USART1_SendData(m_byModuleData[j][i]);
			if(m_byModuleData[i][j]==1)
			{
				LCD_Fill(x+p*i,y+p*j,x+p*(i+1)-1,y+p*(j+1)-1,BLACK);
			}

		}
			
	}
	 
}
void DISPLAY_Temp(void)
{
	u8 buff[32];
	sprintf((char*)buff,"tempture:%.1f C",(Temp*0.1));
	Lcd_Clear(GRAY0);
	Gui_DrawFont_GBK16(16,10,BLUE,GRAY0, buff);
	sprintf((char*)buff,"humidity:%.1f",(Humi*0.1));	
	Gui_DrawFont_GBK16(16,42,BLUE,GRAY0, buff);
}
///**
//  * @brief  Retargets the C library printf function to the USART.
//  * @param  None
//  * @retval None
//  */
//PUTCHAR_PROTOTYPE
//{
//  /* Place your implementation of fputc here */
//  /* e.g. write a character to the USART1 and Loop until the end of transmission */
// 
//	USART_SendData(USART1,(u8)ch);
//	//等待发送完毕
//	while(USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET) { }
//	//返回ch
//	return ch;
//}