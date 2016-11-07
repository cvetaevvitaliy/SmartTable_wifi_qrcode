/**
  ******************************************************************************
  * @file    main.c 
  * @author  zzr
  * @version V1.0.0
  * @date    2015-04-22
  * @brief   Main program body
  ******************************************************************************
  * COPYRIGHT (C) 南京砳磊软件科技有限公司 
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>

#include "sht1x.h"
#include "math.h"
//#ifdef  
// #include " "
//#elif defined USE_STM3210B_EVAL 
//#endif

 

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
  
/* Private variables ---------------------------------------------------------*/
 //USART_InitTypeDef USART_InitStructure;
 enum {TEMP,HUMI};
 // float humi_val,temp_val,dew_point;
#define SHT_IIC_DELAY  sht_delay(2)
/* Private functions ---------------------------------------------------------*/
static int sht_delay(u32 nsecs)
{
	if(nsecs == 0) return 1;
	while(nsecs--)
	{
		__nop();
	}
	return 0;
	
}

//****************************************************************   
//
//SHT11 相关函数
//
//**************************************************************** 
void sht_io_config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 

	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO,ENABLE);
	/* SCL and SDA */
	GPIO_InitStructure.GPIO_Pin =  SHT_DAT_PIN ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD ;//GPIO_Mode_Out_PP; //GPIO_Mode_Out_OD
	GPIO_Init(SHT_DAT_PORT, &GPIO_InitStructure);
	 GPIO_InitStructure.GPIO_Pin =   SHT_SCK_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;//GPIO_Mode_Out_PP; //GPIO_Mode_Out_OD
	GPIO_Init(SHT_SCK_PORT, &GPIO_InitStructure);
	SHT_SDA_H();//sda=1;  起始状
	SHT_IIC_DELAY;
	SHT_SCK_H();//scl=1;
	SHT_IIC_DELAY;
	
}
static u8 SHT_SDA_OUT(void)
{
 
	GPIO_InitTypeDef  GPIO_InitStructure;
 	return 0;

	GPIO_InitStructure.GPIO_Pin =  SHT_DAT_PIN ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_Out_PP; //GPIO_Mode_Out_OD GPIO_Mode_Out_PP
	GPIO_Init(SHT_DAT_PORT, &GPIO_InitStructure);
	 
 }
static u8 SHT_SDA_IN(void)
{
	
	GPIO_InitTypeDef  GPIO_InitStructure;
	return 0;
	GPIO_InitStructure.GPIO_Pin =  SHT_DAT_PIN ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //GPIO_Mode_Out_OD  GPIO_Mode_IPU  GPIO_Mode_IN_FLOATING
	GPIO_Init(SHT_DAT_PORT, &GPIO_InitStructure);
	
}

 void s_connectionreset(void)
{
	unsigned char i;
	
	sht_io_config();
 	SHT_SDA_OUT();
 	 SHT_IIC_DELAY;
	
	SHT_IIC_DELAY;
	SHT_SDA_H();	//GPIO_SetBits(GPIOB, GPIO_Pin_7);//SDA=1;
	SHT_SCK_L();	//GPIO_ResetBits(GPIOB, GPIO_Pin_6);//SCL=0;
	SHT_IIC_DELAY;
	for(i=0;i<9;i++)                  //9 SCK cycles
	{
		SHT_SCK_H();	//GPIO_SetBits(GPIOB, GPIO_Pin_6);//SCK=1;
		SHT_IIC_DELAY;
		SHT_SCK_L();	//GPIO_ResetBits(GPIOB, GPIO_Pin_6);//SCK=0;
		SHT_IIC_DELAY;
	}
	SHT_SCK_H();		//GPIO_SetBits(GPIOB, GPIO_Pin_6);//SCK=1;
	SHT_IIC_DELAY;
	SHT_SDA_L();		//GPIO_ResetBits(GPIOB, GPIO_Pin_7);//SDA=0;
	SHT_IIC_DELAY;
	SHT_SCK_L();		// GPIO_ResetBits(GPIOB, GPIO_Pin_6);//SCK=0;
	SHT_IIC_DELAY;
	SHT_SCK_H();		//SCK=1;
	SHT_IIC_DELAY;
	SHT_SDA_H();		//SDA=1;
	SHT_IIC_DELAY;
 
	 
}
//----------------------------------------------------------------
char s_write_byte(unsigned char value)
{ 
	unsigned char i,error=0;  
  SHT_SDA_OUT();
 	 SHT_IIC_DELAY;
  for (i=0x80;i>0;i/=2)             //shift bit for masking
  {
	  if (i & value) SHT_SDA_H();          //masking value with i , write to SENSI-BUS
   	  else SHT_SDA_L();
	  SHT_IIC_DELAY;   
	  SHT_SCK_H();                          //clk for SENSI-BUS
	  SHT_IIC_DELAY;        //pulswith approx. 5 us  
	  SHT_SCK_L();
	  SHT_IIC_DELAY;
  }
  SHT_SDA_H();                           //release DATA-line
  SHT_IIC_DELAY;
  SHT_SCK_H();                            //clk #9 for ack 
  SHT_IIC_DELAY;
  error=SHT_READ_SDA();                       //check ack (DATA will be pulled down by SHT11)
  SHT_SCK_L();
  SHT_IIC_DELAY; 
 
  return error;                     //error=1 in case of no acknowledge

}

//----------------------------------------------------------------------------------
char s_read_byte(unsigned char ack)
{ 
  unsigned char i,val=0;
	
   SHT_SDA_H();
	SHT_SDA_IN();
   SHT_IIC_DELAY;                           //release DATA-line
  for (i=0x80;i>0;i/=2)             //shift bit for masking
  { 
    SHT_SCK_H();                          //clk for SENSI-BUS
    SHT_IIC_DELAY;
	if (SHT_READ_SDA()) val=(val | i);        //read bit  
    SHT_SCK_L();
	SHT_IIC_DELAY;        
  }
  if(ack)SHT_SDA_L();else SHT_SDA_H();      //in case of "ack==1" pull down DATA-Line
  SHT_IIC_DELAY;
  SHT_SCK_H();                            //clk #9 for ack
  SHT_IIC_DELAY;          //pulswith approx. 5 us 
  SHT_SCK_L();          
  SHT_IIC_DELAY;
  SHT_SDA_H();                           //release DATA-line
  SHT_IIC_DELAY;
  return val;

}
//----------------------------------------------------------------
//       _____          ________
// DATA:      |_______|
//           ___     ___
// SCK : ___|   |___|   |______
void s_transstart(void)
{  
 SHT_SDA_OUT();
   SHT_IIC_DELAY;
   SHT_SCK_H();//SCK=1;
   SHT_IIC_DELAY;
   SHT_SDA_L();//DATA=0;
   SHT_IIC_DELAY; 
   SHT_SCK_L();//SCK=0; 
   SHT_IIC_DELAY;
   SHT_SCK_H();//SCK=1;
   SHT_IIC_DELAY;
   SHT_SDA_H();//DATA=1;    
   SHT_IIC_DELAY;
   SHT_SCK_L();//SCK=0; 
   SHT_IIC_DELAY;   		     
}
//----------------------------------------------------------------
// resets the sensor by a softreset 
char s_softreset(void)
{
	unsigned char error=0;  
	u32 i;
	s_connectionreset();              //reset communication
	error+=s_write_byte(RESET);       //send RESET-command to sensor
	if(error != 0)
		return error;
	else
	{
		i= 8000*12;
		while(i--);	// wait for 11ms
		return error;                     //error=1 in case of no response form the sensor
	}
}
//----------------------------------------------------------------
// reads the status register with checksum (8-bit)
char s_read_statusreg(unsigned char *p_value, unsigned char *p_checksum)
{  unsigned char error=0;

  s_transstart();                   //transmission start
  error=s_write_byte(STATUS_REG_R); //send command to sensor
  *p_value=s_read_byte(ACK);        //read status register (8-bit)
  *p_checksum=s_read_byte(noACK);   //read checksum (8-bit)  
  return error;                     //error=1 in case of no response form the sensor
}
//----------------------------------------------------------------
// writes the status register with checksum (8-bit)
char s_write_statusreg(unsigned char *p_value)
{  unsigned char error=0;

  s_transstart();                   //transmission start
  error+=s_write_byte(STATUS_REG_W);//send command to sensor
  error+=s_write_byte(*p_value);    //send value of status register
  return error;                     //error>=1 in case of no response form the sensor
}
//----------------------------------------------------------------
// makes a measurement (humidity/temperature) with checksum
char s_measure(u16 *p_value, unsigned char *p_checksum, unsigned char mode)
{ unsigned error=0;
  u32 i;
  unsigned char Hbyte,Lbyte,crc;

  s_transstart();                   //transmission start
  switch(mode){                     //send command to sensor
    case TEMP : error+=s_write_byte(MEASURE_TEMP); break;
    case HUMI : error+=s_write_byte(MEASURE_HUMI); break;
    default     : break;  
  }
  for (i=0;i<8000*400;i++) // 14bit 320ms*(1+/- 0.3)
  {
	  if((SHT_READ_SDA())==0) 
		  break;  //wait until sensor has finished the measurement
  }
  if(SHT_READ_SDA()) 
  {
	  error+=1;                                        // or timeout (~2 sec.) is reached
	  return error;
	  
  }
  Hbyte =s_read_byte(ACK);                 //read the first byte (MSB)
  Lbyte =s_read_byte(ACK);              //read the second byte (LSB)
  crc=s_read_byte(noACK);       //read checksum
  
  *p_value=0;
  *p_value |= Hbyte<<8;                 //read the first byte (MSB)
  *p_value |= Lbyte;              //read the second byte (LSB)
  *p_checksum =crc;       //read checksum
  return error;
}

//----------------------------------------------------------------
// calculates temperature and humidity [%RH] 
// input :  humi [Ticks] (12 bit) 
//          temp [Ticks] (14 bit)
// output:  humi [%RH]
//          temp  
void calc_sth11(float *p_humidity ,float *p_temperature)
{ const float C1=-4.0;              // for 12 Bit
  const float C2=+0.0405;           // for 12 Bit
  const float C3=-0.0000028;        // for 12 Bit
  const float T1=+0.01;             // for 14 Bit @ 5V
  const float T2=+0.00008;           // for 14 Bit @ 5V 
  float rh=*p_humidity;             // rh:      Humidity [Ticks] 12 Bit 
  float t=*p_temperature;           // t:       Temperature [Ticks] 14 Bit
  float rh_lin;                     // rh_lin:  Humidity linear
  float rh_true;                    // rh_true: Temperature compensated humidity
  float t_C;                        // t_C   :  Temperature 
  t_C=t*0.01 - 40;                  //calc. temperature from ticks to 
  rh_lin=C3*rh*rh + C2*rh + C1;     //calc. humidity from ticks to [%RH]
  rh_true=(t_C-25)*(T1+T2*rh)+rh_lin;   //calc. temperature compensated humidity [%RH]
  if(rh_true>100)rh_true=100;       //cut if the value is outside of
  if(rh_true<0.1)rh_true=0.1;       //the physical possible range
  *p_temperature=t_C;               //return temperature 
  *p_humidity=rh_true;              //return humidity[%RH]
}
//----------------------------------------------------------------
// 计算露点
// input:   humidity [%RH], temperature 
// output:  dew point 
float calc_dewpoint(float h,float t)
{ 
	float logEx,dew_point;
	logEx=0.66077+7.5*t/(237.3+t)+(log10(h)-2);
	dew_point = (logEx - 0.66077)*237.3/(0.66077+7.5-logEx);
	return dew_point;
}


//*********************************************************
//获取SHT11 SENSOR DATA
//
//u16 Get_SHT11(void)
u8 Get_SHT11(u16 *Humi,s16 *Temp)
{ 
	//struct value humi_val,temp_val;
	struct SHT_value humi_val;
	struct SHT_value temp_val;
//	float dew_point;
	unsigned char error ;
//	u16 i;
    error=0;
    error+=s_measure(&humi_val.i,&humi_val.crc,HUMI);  //measure humidity
    error+=s_measure(&temp_val.i,&temp_val.crc,TEMP);  //measure temperature
	
    if(error!=0) 
	{
//		printfk("sht15 err\r\n");
		s_connectionreset();                 //in case of an error: connection reset
		return 1;
	}
    else
	{
		humi_val.f=(float)humi_val.i;                   //converts integer to float
		temp_val.f=(float)temp_val.i;                   //converts integer to float
		calc_sth11(&humi_val.f,&temp_val.f);            //calculate humidity, temperature
		//	dew_point=calc_dewpoint(humi_val.f,temp_val.f); //calculate dew point
		//  printf("SHT11: temp:%5.2fC, humi:%5.1f%%, dew point:%5.1fC \r\n",temp_val.f,humi_val.f,dew_point);
		*Humi = (u16)( humi_val.f *10);
		*Temp = (s16)( temp_val.f *10);
	}
	return 0;
} 


