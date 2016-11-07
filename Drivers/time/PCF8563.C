#include  "pcf8563.h"
#include  "zRTC_Timer.h"
#include <stdio.h>
#include <string.h>


/*	Brief : The delay routine
 *	\param : delay in ms
*/
//static void delay_msek(u32 msek)
//{
//	/*Here you can write your own delay routine*/
//	u32 i = 1;
//	while(msek--)
//	{
//		i = 8000;
//		while(i--);
//	}
//}
static void I2C_delay(void)
{
	u8 i = 2;
	while(i--)
	{
		__nop();
	}
}
u8 BCD_To_Decimal(u8 source_char)//?BCD????????
{
	u8	temp1,temp2,desit_char;
	temp1 = source_char>>4;
	temp2 = source_char&0x0f;
	desit_char = temp1*10 + temp2;
	return	desit_char;
}
	  
u8 Decimal_To_BCD(u8 source_char)// 
{
	if(source_char >99)
		return 0;
	else
		return (u8)(((source_char/10)<<4) |source_char%10 );
}


static void PFC_SDA_IN(void) //sda??? ,PA11
{
	GPIO_InitTypeDef GPIO_InitStructure;//??GPIO??????			
	// Configure I2C1 pins:SDA
	GPIO_InitStructure.GPIO_Pin =  PFC8563_IIC_SDA;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(PFC8563_IIC_PORT, &GPIO_InitStructure);
}


static void PFC_SDA_OUT(void) //sda???
{
	GPIO_InitTypeDef GPIO_InitStructure;//??GPIO??????
	GPIO_InitStructure.GPIO_Pin =  PFC8563_IIC_SDA;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(PFC8563_IIC_PORT, &GPIO_InitStructure);
}

 //产生IIC起始信号
 
static void IIC_Start(void)
{
	PFC_SDA_OUT(); // 
	PFC_SDA_H();
	PFC_SCL_H();
	I2C_delay();
	PFC_SDA_L();//START:when CLK is high,DATA change form high to low
	I2C_delay();
	PFC_SCL_L();// 
}
static void IIC_Stop(void)
{
	PFC_SDA_OUT();//sda???
	I2C_delay();
	PFC_SCL_L();
	PFC_SDA_L();//STOP:when CLK is high DATA change form low to high
	I2C_delay();
	PFC_SCL_H();
	PFC_SDA_H();//??I2C??????
	I2C_delay();
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
 
static u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	PFC_SDA_IN(); //SDA?????
	PFC_SDA_H();I2C_delay();;
	PFC_SCL_H();I2C_delay();;
	while(PFC_READ_SDA())
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	PFC_SCL_L();//????0
	return 0;
}

 

//产生ACK应答
 
static void IIC_Ack(void)
{
	PFC_SCL_L();
	PFC_SDA_OUT();
	PFC_SDA_L();
	I2C_delay();
	PFC_SCL_H();
	I2C_delay();
	PFC_SCL_L();
}
//不产生ACK应答 
static void IIC_NAck(void)
{
	PFC_SCL_L();
	PFC_SDA_OUT();
	PFC_SDA_H();
	I2C_delay();
	PFC_SCL_H();
	I2C_delay();
	PFC_SCL_L();
}

// 

//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答 
    
static void IIC_Send_Byte(u8 txd)
{
	u8 t;
	PFC_SDA_OUT();
	I2C_delay();
	PFC_SCL_L(); 
	for(t=0;t<8;t++)
	{
		if(txd&0x80)PFC_SDA_H();
		else PFC_SDA_L();			 
		txd<<=1;
		I2C_delay(); // 
		PFC_SCL_H();
		I2C_delay();
		PFC_SCL_L();
		I2C_delay();
	}
}
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK  
 
static u8 IIC_Read_Byte(u8 ack)
{
	u8 i,receive=0;
	PFC_SDA_IN();//SDA?????
	for(i=0;i<8;i++ )
	{
		PFC_SCL_L();
		I2C_delay();
		PFC_SCL_H();
		receive<<=1;
		if(PFC_READ_SDA())receive++;
		I2C_delay();
	}
	if (!ack)
	IIC_NAck();// 
	else
	IIC_Ack(); // 
	return receive;
}
 static s8 I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = 0;
	 
	IIC_Start(); 
	IIC_Send_Byte(dev_addr);
	if(IIC_Wait_Ack()) 
	{
		iError = 1;
	} 
	IIC_Send_Byte(reg_addr);
	if(IIC_Wait_Ack()) 
	{
		iError = 1;
	}
	while(cnt)
	{
		IIC_Send_Byte(*reg_data++);
		if(IIC_Wait_Ack())
		{
			iError = 1;
			break ;
		}
		cnt--;		
	}
	IIC_Stop();
	I2C_delay(); 
	
	return (s8)iError;
}

 /*	\Brief: The function is used as I2C bus read
 *	\Return : Status of the I2C read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be read
 *	\param reg_data : This data read from the sensor, which is hold in an array
 *	\param cnt : The no of byte of data to be read
 */
static s8 I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = 0;
	IIC_Start(); 
	IIC_Send_Byte(dev_addr);
	if(IIC_Wait_Ack())
	{
		iError = 1;
	} 
	IIC_Send_Byte(reg_addr);
	if(IIC_Wait_Ack())
	{
		iError = 1;
	} 
	IIC_Start();
	IIC_Send_Byte((dev_addr)|0x01);
	if(IIC_Wait_Ack()) 
	{
		iError = 1;
	} 
	while(cnt)
	{
		
		if(cnt == 1)
		{
			*reg_data++ = IIC_Read_Byte(0);
		}
		else
			*reg_data++ = IIC_Read_Byte(1);	 
		cnt--;
		
	}
	IIC_Stop();
	I2C_delay(); 
	
	return (s8)iError;
}
 
//WriteAddr  :写入数据的目的地址   
//DataToWrite:要写入的数据
static s8 PFC8563_WriteOneByte(u8 WriteAddr,u8 DataToWrite)
{
	s8 iError = 0;
           
	IIC_Start(); 
	IIC_Send_Byte(PCF_WRITE_ADDR);    //发送写命令
	if(IIC_Wait_Ack()) 
	{
		iError = 1;
	}  
	IIC_Send_Byte(WriteAddr);   //发送地址
	if(IIC_Wait_Ack()) 
	{
		iError = 1;
	}      
	IIC_Send_Byte(DataToWrite);     //发送字节  
	if(IIC_Wait_Ack()) 
	{
		iError = 1;
	}        
	IIC_Stop();//产生一个停止条件
	I2C_delay();
	return (s8)iError;	
}

static u8 PFC8563_IIC_ReadOneByte(u8 ReadAddr)
{
	u8 date;
	s8 iError = 0;
	IIC_Start();
	IIC_Send_Byte(PCF_WRITE_ADDR);  //???
	if(IIC_Wait_Ack()) 
	{
		iError = 1;
	}
	IIC_Send_Byte(ReadAddr); 
	if(IIC_Wait_Ack()) 
	{
		iError = 1;
	}
	IIC_Start();
	IIC_Send_Byte(PCF_READ_ADDR);  // 
	if(IIC_Wait_Ack()) 
	{
		iError = 1;
	}
	date=IIC_Read_Byte(0); 	
	IIC_Stop();
	return date;
}
 


 


/* 	*/
s8 PFC8563_GET_TIME(DateTime * TimeToGet)
{
	u8 readbuf[7];
	memset(readbuf,0,7);
	if(I2C_bus_read(PCF_WRITE_ADDR,REG_SECOND,readbuf,7) != 0)
		return 1;
	TimeToGet->sec   = BCD_To_Decimal(readbuf[0]&0x7f);
	TimeToGet->min   = BCD_To_Decimal(readbuf[1]&0x7f);
	TimeToGet->hour  = BCD_To_Decimal(readbuf[2]&0x3f);
	TimeToGet->day  = BCD_To_Decimal(readbuf[3]&0x3f);
	TimeToGet->week   = BCD_To_Decimal(readbuf[4]&0x07);
	TimeToGet->month = BCD_To_Decimal(readbuf[5]&0x1f);
	TimeToGet->year  = BCD_To_Decimal(readbuf[6]&0xff)+2000;
    return 0;
//	pcf_timer.sec=BCD_To_Decimal(PFC8563_IIC_ReadOneByte(0X02)&0x7f);
//	pcf_timer.min=BCD_To_Decimal(PFC8563_IIC_ReadOneByte(0X03)&0x7f);
//	pcf_timer.hour=BCD_To_Decimal(PFC8563_IIC_ReadOneByte(0X04)&0x3f); 
//	pcf_timer.day=BCD_To_Decimal(PFC8563_IIC_ReadOneByte(0X05)&0x3f); 
////	timer.week=IIC_Read_Addr(0X06)&0x07;
//	pcf_timer.month=BCD_To_Decimal(PFC8563_IIC_ReadOneByte(0X07)&0x1f);
//	pcf_timer.year=BCD_To_Decimal(PFC8563_IIC_ReadOneByte(0X08)&0xff);
//	pcf_timer.year+=2000;	 
}


/* 	*/
uint8_t PFC_SET_TIME(DateTime * TimeToSet)	//?????
{
	// sec minu hour weekday month year
	u8 writebuf[7];
	if(TimeToSet->sec >= 60)
		return 1;
	if(TimeToSet->min >= 60)
		return 2;
	if(TimeToSet->month >= 13)
		return 3;

	if( TimeToSet->year >= 2099)
		return 4;

	writebuf[0] = Decimal_To_BCD(TimeToSet->sec);
	writebuf[1] = Decimal_To_BCD(TimeToSet->min);
	writebuf[2] = Decimal_To_BCD(TimeToSet->hour);
	writebuf[3] = Decimal_To_BCD(TimeToSet->day);
	writebuf[4] = Decimal_To_BCD(TimeToSet->week);
	writebuf[5] = Decimal_To_BCD(TimeToSet->month);
	writebuf[6] = Decimal_To_BCD((u8)(TimeToSet->year%100));
	I2C_bus_write(PCF_WRITE_ADDR,REG_SECOND,writebuf,7);
//	PFC8563_WriteOneByte(0x02,second);
//	PFC8563_WriteOneByte(0x03,mint);
//	PFC8563_WriteOneByte(0x04,hour);
//	PFC8563_WriteOneByte(0x05,day);
////	IIC_Write_Addr(0x06,0x04);
//	PFC8563_WriteOneByte(0x07,month);
//	PFC8563_WriteOneByte(0x08,year);
	return 0;
}

void PCF8563_Init(void)
{
	DateTime	pcf_timer; 
	GPIO_InitTypeDef GPIO_InitStructure;//??GPIO??????
	RCC_APB2PeriphClockCmd(PFC8563_IIC_Clock,ENABLE);		
	// Configure I2C1 pins: SCL and SDA 
	GPIO_InitStructure.GPIO_Pin =  PFC8563_IIC_SDA | PFC8563_IIC_SCL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(PFC8563_IIC_PORT, &GPIO_InitStructure);
	I2C_delay();I2C_delay();
	
		// read vl_seconds  after reset , the first bit is 1;
	if(PFC8563_IIC_ReadOneByte(REG_SECOND)&0x80)
	{
		pcf_timer.year = 2016;
		pcf_timer.month = 1;
		pcf_timer.day = 5;
		pcf_timer.hour = 8;
		pcf_timer.min = 0;
		pcf_timer.sec = 0;
		pcf_timer.week = 1;
		PFC_SET_TIME(&pcf_timer);
	}

}
