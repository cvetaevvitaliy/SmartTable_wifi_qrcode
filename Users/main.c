#include "stm32f10x.h"
#include "oled.h"
#include "stdio.h"
#include "QRCodeApp.h"
#include "bsp.h"
#include "sys_tick_delay.h"
#include "bsp_leds.h"
#include "wifi_esp_config.h"
#include "sys_adc.h"
#include "sht1x.h"
#include "usb_contrl.h"
#include "Buttons.h"
#define QRCODE_Y 80
u8 	gMcuID[12];
void DISPLAY_RENCODE_TO_TFT(u8 *qrcode_data);
u16 Humi;
s16 Temp;

int main(void)
{
	u32 WifiCheckTicks = 0;
	RCC_Configuration();
	GetSTM32ID((u32*)gMcuID);
	UsbOutConfig();
	SYS_DelayFunConfig();
	Sys_delay_init();
	BspLed_IOConfig();
	ButtonsConfig();
	bsp_adc_config();
	sht_io_config();	
	Wifi_Init();
	
	OLED_Init();
	DISPLAY_RENCODE_TO_TFT("1234567890122456");
	UsbOutConfig();
	while(1)
	{
		CycleBufPro();
		if((u32)(SecondTicks - WifiCheckTicks) > 5)
		{ 
			WifiCheckTicks  = SecondTicks; 
			wifi_fsm();
		}
		AllLedsRun();
		Get_SHT11(&Humi,&Temp);
		SysTickDelay_ms(1000);
		//DISPLAY_RENCODE_TO_TFT("123");
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
 	OLED_ShowString(10,1,"E",12);
//	OLED_Fill(10,10,100,50,1);
	OLED_Refresh_Gram();
	//OLED_ShowString2(10,10,qrencode_buff,12);
	if(m_nSymbleSize*2>128)	
	{
	//	LCD_ShowString(10,60,200,16,16,(u8 *)"The QR Code is too large!");//太大显示不下
		OLED_ShowString(10,60,(u8 *)"The QR Code is too large!",12);
		return;
	}
	for(i=0;i<10;i++)
	{
		if((m_nSymbleSize*i*2)>240)	break;
	}
	p=3;//(i-3)*1;//点大小
	x=(240-m_nSymbleSize*p)/2 -50;
	y=1;
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
				OLED_Fill(x+p*i,y+p*j,x+p*(i+1)-1,y+p*(j+1)-1,1);
			//	LCD_Fill(x+p*i,y+p*j,x+p*(i+1)-1,y+p*(j+1)-1,BLACK);
			}

		}
			
	}
	OLED_Refresh_Gram();
}
