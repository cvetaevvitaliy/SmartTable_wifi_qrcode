//////////////////////////////////////////////////////////////////////////////////	 
 
//  文 件 名   : oled.h
//  版 本 号   : v1.0
//  作    者   : zzr
//  生成日期   : 2016-10-14
//  最近修改   : 
//  功能描述   : 0.69寸OLED 接口演示例程(STM32F103ZE系列IIC)
//              说明: 
//              ----------------------------------------------------------------
//              GND   电源地
//              VCC   接5V或3.3v电源
//              SCL   接PB13（SCL）
//              SDA   接PB15（SDA） 
//               
//              ----------------------------------------------------------------
//Copyright(C)  LoryTech
//All rights reserved
//////////////////////////////////////////////////////////////////////////////////
#ifndef __OLED_H
#define __OLED_H			  	 
//#include "sys.h"
#include "stdlib.h"	 
#include "stm32f10x.h"
#define OLED_MODE 	0
#define SIZE 				8
#define XLevelL			0x00
#define XLevelH			0x10
#define Max_Column	128
#define Max_Row			64
#define	Brightness	0xFF 
#define X_WIDTH 		128
#define Y_WIDTH 		64	    						  
//-----------------OLED IIC端口定义----------------  					   
#define OLED_SCL_PORT GPIOB
#define OLED_SCL_PIN  GPIO_Pin_6

#define OLED_SDA_PORT GPIOB
#define OLED_SDA_PIN  GPIO_Pin_7



#define OLED_SCLK_Clr() GPIO_ResetBits(OLED_SCL_PORT,OLED_SCL_PIN)//CLK
#define OLED_SCLK_Set() GPIO_SetBits(OLED_SCL_PORT,OLED_SCL_PIN)

#define OLED_SDIN_Clr() GPIO_ResetBits(OLED_SDA_PORT,OLED_SDA_PIN)//DIN
#define OLED_SDIN_Set() GPIO_SetBits(OLED_SDA_PORT,OLED_SDA_PIN)

//#define OLED_RST_Clr() GPIO_ResetBits(GPIOB,GPIO_Pin_11)//RES
//#define OLED_RST_Set() GPIO_SetBits(GPIOB,GPIO_Pin_11)
// 		     
#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据


//OLED控制用函数
void OLED_WR_Byte(unsigned dat,unsigned cmd);  
void OLED_Display_On(void);
void OLED_Display_Off(void);	   							   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_Fill(u8 x1,u8 y1,u8 x2,u8 y2,u8 dot);
//void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 Char_Size);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size);
//void OLED_ShowString(u8 x,u8 y, u8 *p,u8 Char_Size);	 
void OLED_ShowString(u8 x,u8 y,const u8 *p,u8 size);
void OLED_ShowString2(u8 x,u8 y,u8 *chr,u8 Char_Size);
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_ShowCHinese(u8 x,u8 y,u8 no);
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);
 
void fill_picture(unsigned char fill_Data);
void Picture(void);
 void OLED_Refresh_Gram(void);


#endif
