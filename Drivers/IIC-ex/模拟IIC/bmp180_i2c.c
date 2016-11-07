/*********************************************************************************************************
*
* File                : 24C02.c
* Hardware Environment: 
* Build Environment   : RealView MDK-ARM  Version: 4.20
* Version             : V1.0
* By                  : 
*
*                                  (c) Copyright 2005-2011, WaveShare
*                                       http://www.waveshare.net
*                                          All Rights Reserved
*
*********************************************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "bmp180_i2c.h"
#include "bmp180.h"

//#include "usart.h"

 

#define BMP_IIC   GPIOB

#define SCLH   	GPIO_SetBits(BMP_IIC, GPIO_Pin_6)
#define SCLL	  GPIO_ResetBits(BMP_IIC, GPIO_Pin_6)
#define SDAH	  GPIO_SetBits(BMP_IIC, GPIO_Pin_7)
#define SDAL	  GPIO_ResetBits(BMP_IIC, GPIO_Pin_7)
#define SDAread GPIO_ReadInputDataBit(BMP_IIC,GPIO_Pin_7)
#define SCLread    GPIO_ReadInputDataBit(BMP_IIC,GPIO_Pin_6)
#define BMP180_WR_ADDR 	0xEE
#define BMP180_RD_ADDR  0xef
#define BMP_OSS  0


struct bmp180_t bmp180;

 void I2C_GPIO_Config(void)
  {
		GPIO_InitTypeDef  GPIO_InitStructure; 
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
	// Configure I2C1 pins: SCL and SDA 
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 |GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_Init(GPIOB, &GPIO_InitStructure);
		SCLH;
		SDAH;
		
	
  }
static void BMP_SDA_OUT(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //GPIO_Mode_Out_OD
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	 
 }
static void BMP_SDA_IN(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //GPIO_Mode_Out_OD
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
}
void bmp_iic_delay(__IO u32 time)
{
	while(time--);
}
static void I2C_delay(void)
{ 
	u8 i=50; //这里可以优化速度 ，经测试最低到5还能写入
	while(i) 
	{ 
	   i--; 
	 } 
}
static void bmp_delay_ms(u16 wTime)
{
	while(wTime--)
	{
		bmp_iic_delay(8000);
	}
}


  //产生IIC起始信号
void IIC_Start(void)
{ 
	BMP_SDA_OUT();     //sda线输出
	SDAH;
	SCLH;
	I2C_delay();
	SDAL;//START:when CLK is high,DATA change form high to low
	I2C_delay();
	SCLL;//钳住I2C总线，准备发送或接收数据
} 
//产生IIC停止信号
void IIC_Stop(void)
{
	BMP_SDA_OUT();//sda线输出
	SCLL;
	SDAL;//STOP:when CLK is high DATA change form low to high
	I2C_delay();
	SCLH;
	SDAH;//发送I2C总线结束信号
	I2C_delay();   
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{

	u8 ucErrTime=0;
	BMP_SDA_IN();      //SDA设置为输入 
//	SDAH;
	I2C_delay();
	SCLH;
	I2C_delay();
	while(SDAread)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	SCLL;//时钟输出0   
	return 0; 
} 
//产生ACK应答
void IIC_Ack(void)
{ 
	SCLL;
	BMP_SDA_OUT();
	
	SDAL;
	I2C_delay();
	SCLH;
	I2C_delay();
	SCLL; 
}
//不产生ACK应答   
void IIC_NAck(void)
{
	SCLL;
	BMP_SDA_OUT();
	
	SDAH;
	I2C_delay();
	SCLH;
	I2C_delay();
	SCLL; 
}     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答 
void IIC_Send_Byte(u8 txd)
{

	u8 t;  
	BMP_SDA_OUT();    
	SCLL;//拉低时钟开始数据传输
	for(t=0;t<8;t++)
	{
		//IIC_SDA=(txd&0x80)>>7;
		if(txd&0x80)
			SDAH;
		else
			SDAL;
		txd<<=1;  
		I2C_delay();
		SCLH;
		I2C_delay();
		SCLL;
		I2C_delay();

	}
 
}    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK  
u8 IIC_Read_Byte(unsigned char ack)
{

	unsigned char i,receive=0;
	BMP_SDA_IN();//SDA设置为输入
	for(i=0;i<8;i++ )
	{
		SCLL;
		I2C_delay();
		SCLH;
		receive<<=1;
		if(SDAread)receive++;  
		I2C_delay();		
	}
	if (ack)
		IIC_Ack(); //发送ACK 
	else
		IIC_NAck();//发送nACK 
	return receive;
}
 
u8 BMP180_ReadOneByte(u8 ReadAddr)
{
	u8 temp=0;      
	IIC_Start(); 
	IIC_Send_Byte(BMP180_WR_ADDR);   //发送写命令
	IIC_Wait_Ack();  
	IIC_Send_Byte(ReadAddr);   //发送要读的寄存器地址
	IIC_Wait_Ack();   
	IIC_Start();     
	IIC_Send_Byte(BMP180_RD_ADDR);           //进入接收模式  
	IIC_Wait_Ack();
	temp=IIC_Read_Byte(0);  
	IIC_Stop();//产生一个停止条件   
	return temp;
 
} 
//WriteAddr  :写入数据的目的地址   
//DataToWrite:要写入的数据
void BMP180_WriteOneByte(u8 WriteAddr,u8 DataToWrite)
{
           
	IIC_Start(); 
	IIC_Send_Byte(BMP180_WR_ADDR);    //发送写命令
	IIC_Wait_Ack();  
	IIC_Send_Byte(WriteAddr);   //发送地址
	IIC_Wait_Ack();      
	IIC_Send_Byte(DataToWrite);     //发送字节  
	IIC_Wait_Ack();        
	IIC_Stop();//产生一个停止条件
	I2C_delay(); 
}

u16 BMP180_ReadTwoByte(u8 regaddr)
{
	u16 temp=0;
	u8 msb,lsb;
	IIC_Start(); 
	IIC_Send_Byte(BMP180_WR_ADDR);   //发送写命令
	IIC_Wait_Ack();  
	IIC_Send_Byte(regaddr);   //发送要读的寄存器地址
	IIC_Wait_Ack();   
	IIC_Start();     
	IIC_Send_Byte(BMP180_RD_ADDR);           //进入接收模式  
	IIC_Wait_Ack();
	msb=IIC_Read_Byte(1);
	lsb = IIC_Read_Byte(0);
	temp = (msb<<8 )| lsb;
	
	IIC_Stop();//产生一个停止条件   
	return temp;
}

#define	I2C_BUFFER_LEN 8
#define I2C0 5
/*-------------------------------------------------------------------*
*	This is a sample code for read and write the data by using I2C
*	Configure the below code to your I2C driver
*	The device address is defined in the bmp180.c file
*-----------------------------------------------------------------------*/
/*	\Brief: The function is used as I2C bus write
 *	\Return : Status of the I2C write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register, will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */
s8 BMP180_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BMP180_INIT_VALUE;
//	u8 array[I2C_BUFFER_LEN];
//	u8 stringpos = BMP180_INIT_VALUE;
//	array[BMP180_INIT_VALUE] = reg_addr;
//	for (stringpos = BMP180_INIT_VALUE; stringpos < cnt; stringpos++) {
//		array[stringpos + 1] = *(reg_data + stringpos);
//	}
	/*
	* Please take the below function as your reference for
	* write the data using I2C communication
	* "IERROR = I2C_WRITE_STRING(DEV_ADDR, ARRAY, CNT+C_BMP180_ONE_U8X)"
	* add your I2C write function here
	* iError is an return value of I2C read function
	* Please select your valid return value
	* In the driver SUCCESS defined as BMP180_INIT_VALUE
    * and FAILURE defined as -C_BMP180_ONE_U8X
	* Note :
	* This is a full duplex operation,
	* The first read data is discarded, for that extra write operation
	* have to be initiated. For that cnt+C_BMP180_ONE_U8X operation done in the I2C write string function
	* For more information please refer data sheet SPI communication:
	*/	 
	IIC_Start(); 
	IIC_Send_Byte(dev_addr&0xfe);
	IIC_Wait_Ack(); 
	IIC_Send_Byte(reg_addr&0xfe);
	IIC_Wait_Ack(); 
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
s8 BMP180_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	s32 iError = BMP180_INIT_VALUE;
//	u8 array[I2C_BUFFER_LEN] = {BMP180_INIT_VALUE};
//	u8 stringpos = BMP180_INIT_VALUE;
//	array[BMP180_INIT_VALUE] = reg_addr;
//	/* Please take the below function as your reference
//	 * for read the data using I2C communication
//	 * add your I2C rad function here.
//	 * "IERROR = I2C_WRITE_READ_STRING(DEV_ADDR, ARRAY, ARRAY, C_BMP180_ONE_U8X, CNT)"
//	 */
//	for (stringpos = BMP180_INIT_VALUE; stringpos < cnt; stringpos++) {
//		*(reg_data + stringpos) = array[stringpos];
//	}
	
		IIC_Start(); 
	IIC_Send_Byte(dev_addr&0xfe);
	IIC_Wait_Ack(); 
	IIC_Send_Byte(reg_addr&0xfe);
	IIC_Wait_Ack(); 
	IIC_Start();
	IIC_Send_Byte((dev_addr&0xfe)|0x01);
	IIC_Wait_Ack(); 
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
/*	Brief : The delay routine
 *	\param : delay in ms
*/
void BMP180_delay_msek(u32 msek)
{
	/*Here you can write your own delay routine*/
	u32 i = 1;
	while(msek--)
	{
		i = 8000;
		while(i--);
	}
}
// s8 I2C_routine(void) 
//{
///*--------------------------------------------------------------------------*
// *  By using bmp180 the following structure parameter can be accessed
// *	Bus write function pointer: BMP180_WR_FUNC_PTR
// *	Bus read function pointer: BMP180_RD_FUNC_PTR
// *	Delay function pointer: delay_msec
// *	I2C address: dev_addr
// *--------------------------------------------------------------------------*/
//	bmp180.bus_write = BMP180_I2C_bus_write;
//	bmp180.bus_read = BMP180_I2C_bus_read;
//	bmp180.dev_addr = BMP180_I2C_ADDR;
//	bmp180.delay_msec = BMP180_delay_msek;

//	return BMP180_INIT_VALUE;
//}

/************** I2C buffer length ******/



//u16 BMP180_CRC_Read(u8 addr)
//{ 
//	u8 msb,lsb;
//	short data;
//	IIC_Start();
//	IIC_Send_Byte(BMP180_WR_ADDR);   //发送写命令
//	IIC_Wait_Ack();  
//	IIC_Send_Byte(addr);   //发送要读的寄存器地址
//	IIC_Wait_Ack();
//	IIC_Start();
//	IIC_Send_Byte(BMP180_RD_ADDR);
//	 
//	IIC_Wait_Ack();
//	msb=IIC_Read_Byte(1);
//	//IIC_Wait_Ack();
//	lsb=IIC_Read_Byte(0);
//	//IIC_Wait_Ack();
//	IIC_Stop();
//	I2C_delay();
//	data= msb << 8;
//	data|= lsb;
//	return data;
// 
//}
//u16 BMP085_Get_UT(void)
//{
// 
//	IIC_Start();
//	IIC_Send_Byte(BMP180_WR_ADDR);//发送写命令
//	IIC_Wait_Ack();
//	IIC_Send_Byte(0xf4);        //发送字
//	IIC_Wait_Ack();
//	IIC_Send_Byte(0x2E);
//	IIC_Stop();
//	bmp_delay_ms(5);
//	return BMP180_CRC_Read(0xf6);
// 
//}
//u32  BMP_UP_Read(void)
//{
// 
//  long temp;
//  BMP180_WriteOneByte(0xF4,0x34+(BMP_OSS<<6));
//  bmp_delay_ms(27);
//  temp=(BMP180_ReadOneByte(0xf6)<<16)|(BMP180_ReadOneByte(0xf7)<<8)|BMP180_ReadOneByte(0xf8);
//  return temp>>(8-BMP_OSS);
// 
//}
// 
//u32 BMP085_Get_UP(void)
//{
//	 
//	long pressure=0;
//	u8 msb,lsb,xlsb;
//	
//	BMP180_WriteOneByte(0xF4,0x34+(BMP_OSS<<6));
//	bmp_delay_ms(2+(3<<BMP_OSS));
//	IIC_Start();
//	IIC_Send_Byte(BMP180_RD_ADDR);
//	IIC_Wait_Ack();
//	msb=IIC_Read_Byte(1);
//	lsb=IIC_Read_Byte(1);
//	xlsb=IIC_Read_Byte(0);
//	IIC_Stop();
//	pressure=(msb<<16)|(lsb<<8)|xlsb;
//	return pressure>>(8-BMP_OSS);
// 
//}
//  
 
//  
//  //写入1字节数据		待写入数据	  待写入地址	   器件类型(24c16或SD2403)
//uint8_t I2C_WriteOneByte(uint8_t DeviceAddress,uint8_t addr,uint8_t value)
//{
//	  
//	  if(I2C_Start())
//			return 1;
//	  //I2C_SendByte(((WriteAddress & 0x0700) >>7) | DeviceAddress & 0xFFFE);//设置高起始地址+器件地址 
//	  I2C_SendByte( DeviceAddress & 0xFE);//写器件地址 
//	  if(I2C_WaitAck() !=0)
//	  {
//	  	I2C_Stop();
//			return 1;
//	  }
//	  I2C_SendByte((u8)((addr) & 0xFF));   //设置低起始地址 	 
//	  I2C_WaitAck(); 
//	  I2C_SendByte(value);		   //写数据
//	  I2C_WaitAck();   
//	  I2C_Stop(); 
//	  return 0;
//  }

//  /*******************************************************************************
//* Function Name  : I2C_Write
//* Description    : 
//* Input          : 
//* Output         : None
//* Return         : 
//* Attention		 : None
//*******************************************************************************/
//uint8_t I2C_Write(uint8_t I2C_Addr,uint8_t addr,uint8_t *buf,uint16_t num)
//{
//	uint8_t err=0;
//	
//	while(num--)
//	{
//		if(I2C_WriteOneByte( I2C_Addr,addr++,*buf++))
//		{
//			err++;
//		}
//	}
//	if(err)
//		return 1;
//	else 
//		return 0;	
//}
//  /*******************************************************************************
//  * Function Name  : I2C_Read
//  * Description    : 
//  * Input		   : 
//  * Output		   : 
//  * Return		   : 
//  * Attention	   : None
//  *******************************************************************************/
//uint8_t I2C_Read(uint8_t DeviceAddress,uint8_t addr,uint8_t *buf,uint16_t num)
//{
//	if(I2C_Start())
//	return 1;
//	I2C_SendByte((DeviceAddress & 0xFE));//
//	I2C_SendByte(addr);
//	I2C_WaitAck();  
//	I2C_Start();
//	I2C_SendByte((DeviceAddress & 0xFE)|0x01);
//	while (num)
//	{
//		if(num==1)
//		{
//			I2C_Ack(); 
//		}
//		*buf = I2C_ReceiveByte();
//		buf++;
//		/* Decrement the read bytes counter */
//		num--;
//	}  
//	I2C_NoAck();	
//	I2C_Stop();

//	return 0;
//}


  /*********    END FILE*****************************************/
	