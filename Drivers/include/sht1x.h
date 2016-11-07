//SHT11
#ifndef __SHT1X_H
#define __SHT1X_H

#include "stm32f10x.h"
#define SHT_SCK_PORT  GPIOC
#define SHT_SCK_PIN	  GPIO_Pin_12
#define SHT_DAT_PORT  GPIOD
#define SHT_DAT_PIN	  GPIO_Pin_2
  
//SHT Òý½Å¶¨Òå
#define SHT_SCK_H()   GPIO_SetBits(SHT_SCK_PORT, SHT_SCK_PIN)
#define SHT_SCK_L()	  GPIO_ResetBits(SHT_SCK_PORT, SHT_SCK_PIN)
#define SHT_SDA_H()	  GPIO_SetBits(SHT_DAT_PORT, SHT_DAT_PIN)
#define SHT_SDA_L()	  GPIO_ResetBits(SHT_DAT_PORT, SHT_DAT_PIN)
#define SHT_READ_SDA() GPIO_ReadInputDataBit(SHT_DAT_PORT,SHT_DAT_PIN)


#define noACK 0
#define ACK   1
#define SHT11_ADDR		0x00
#define STATUS_REG_W  0x06   //000    0011          0
#define STATUS_REG_R  0x07   //000    0011          1
#define MEASURE_TEMP  0x03   //000    0001          1
#define MEASURE_HUMI  0x05   //000    0010          1
#define RESET         0x1e   //000    1111          0

 struct SHT_value
{
	u16 i;
	u8  crc;
	float f;
};
void sht_io_config(void);
void s_connectionreset(void);
char s_write_byte(unsigned char value);
char s_read_byte(unsigned char ack);
void s_transstart(void);
char s_softreset(void);
char s_read_statusreg(unsigned char *p_value, unsigned char *p_checksum);
char s_write_statusreg(unsigned char *p_value);
char s_measure(u16 *p_value, unsigned char *p_checksum, unsigned char mode);
void calc_sth11(float *p_humidity ,float *p_temperature); 
float calc_dewpoint(float h,float t);
//u8 Get_SHT11(struct SHT_value *humi_val,struct SHT_value *temp_val );
u8 Get_SHT11(u16 *Humi,s16 *Temp);

#endif
