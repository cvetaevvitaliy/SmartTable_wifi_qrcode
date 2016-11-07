/***************************************************************************//**
 *   @file   Communication.c
 *   @brief  Implementation of Communication Driver for RENESAS RX62N
 *           Processor.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************/


/******************************************************************************/
/* Include Files                                                              */

/******************************************************************************/
#include "Communication.h"



/***************************************************************************//**
  
 * @return 0 - Initialization failed, 1 - Initialization succeeded.
*******************************************************************************/
//unsigned char SIM_SPI_Init(unsigned char lsbFirst,
//                       unsigned long clockFreq,
//                       unsigned char clockPol,
//                       unsigned char clockPha)
 void SIM_SPI_Init(void)
{
	// Add your code here.
	GPIO_InitTypeDef GPIO_InitStructure;				  										  
	 	 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_AFIO  , ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5| GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //misoҪ��ģ������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	AD_SCLK_H;
	
}
static void spi_delay (__IO uint32_t nDelay)
{
	while (nDelay--);
}



 


//---------------------------------
//void ReadFromAD7799(unsigned char count,unsigned char *buf)
//---------------------------------
//Function that reads from the AD7799 via the SPI port. 
//--------------------------------------------------------------------------------
 
/***************************************************************************//**
 * @brief Writes data to SPI.
 *
 * @param data - Write data buffer:
 *               - first byte is the chip select number;
 *               - from the second byte onwards are located data bytes to write.
 * @param bytesNumber - Number of bytes to write.
 *
 * @return Number of written bytes.
*******************************************************************************/
unsigned char SIM_SPI_Write(unsigned char* data,
                        unsigned char bytesNumber)
{
	// Add your code here.
	unsigned char i,j;
	unsigned char ValueToWrite;
	//spi_delay(2);
	for(i=0;i<bytesNumber;i++)
 	{
	 	ValueToWrite = *(data + i );
		for(j=0; j<8; j++)
		{
			AD_SCLK_L;
			spi_delay(2);
			if(0x80 == (ValueToWrite & 0x80))
			{
				AD_MOSI_H;	  //Send one to SDO pin
			}
			else
			{
				AD_MOSI_L;	  //Send zero to SDO pin
			}
			spi_delay(2);
			AD_SCLK_H;			 
			ValueToWrite <<= 1;	//Rotate data
		}
	}
return  0;	
}

/***************************************************************************//**
 * @brief Reads data from SPI.
 *
 * @param data - As an input parameter, data represents the write buffer:
 *               - first byte is the chip select number;
 *               - from the second byte onwards are located data bytes to write.
 *               As an output parameter, data represents the read buffer:
 *               - from the first byte onwards are located the read data bytes. 
 * @param bytesNumber - Number of bytes to write.
 *
 * @return Number of written bytes.
*******************************************************************************/
unsigned char SIM_SPI_Read(unsigned char* data,
                       unsigned char bytesNumber)
{
	// Add your code here.
	unsigned	char	i = 0;
	unsigned	char	j = 0;
	unsigned	char  	ReadData = 0;
 
	spi_delay(1);
	for(j=bytesNumber; j>0; j--)
	{
		for(i=0; i<8; i++)
		{
			AD_SCLK_L;		 
			ReadData=ReadData<<1 ;
			if(AD_MISO)ReadData+=1 ;
			spi_delay(2);			
			AD_SCLK_H;	
			spi_delay(2);			
		}
		*(data + j - 1)= ReadData;
	}	 
 return 0;
}


 
//int WaiteRDY(void)
//{
//    unsigned int iint ;
//    iint=0 ;
//    while(AD_DRY)
//    {
//        iint++;
//        if(iint>65530)
//        {
//            //reset ad7799
//             
//            A7799_Ini();
//            return -1;
//        }
//    }
//	return 0;
//}
 
//void A7799_Ini(void)
//{
//    WriteByteToAd7799(0x10);
//    //b0001 0000
//    /* Writes to Communications Register Setting Next Operation as Write to CONFIGURATION Register*/
//    //дͨѶ�Ĵ���Ϊ����һ����д���üĴ��� WriteByteToAd7799(0x10��b0001 0000    0дͨѶ0�²���д010���üĴ���0��������00����
//    WriteByteToAd7799(0x37);//����Ϊ128  B0011 0111   00����1������Դ1����  0����111��128������ 
//    WriteByteToAd7799(0x00);  //ͨ���� 0 B0011 0000   00����1��׼Ĭ��1������ 0����000ͨ��0
//    //1ͨ��
//    /*CONFIGURATION REGISTER[00,BO(0),U/B(0),0(0),G2(1),G1(1),G0(1),0,0,REF_DET(0),BUF(1),0(0),CH2(0),CH1(0),CH0(0)]*/
//    //WriteByteToAd7799(0x08);
//    //b0000 1000
//    /* Writes to Communications Register Setting Next Operation as Write to Mode Register*/
//    //WriteByteToAd7799(0x80);
//    //WriteByteToAd7799(0x0a);
//    /* Writes to Mode Register Initiating Internal Zero-Scale Calibration*/
//    //WaiteRDY();
//    /* Wait for RDY pin to go low to indicate end of calibration cycle*/
//    //WriteByteToAd7799(0x08);
//    /* Writes to Communications Register Setting Next Operation as Write to
//        Mode Register*/
//    //WriteByteToAd7799(0xa0);
//    //WriteByteToAd7799(0x0a);
//    /* Writes to Mode Register Initiating Internal Full-Scale Calibration*/
//    //WaiteRDY();
//    /* Wait for RDY pin to go low to indicate end of calibration cycle*/
//    WriteByteToAd7799(0x08);//b0000 1000
//    /* Writes to Communications Register Setting Next Operation as Write to Mode Register*/
//    WriteByteToAd7799(0x00);   //000����ģʽ0��PSW0000����
//    WriteByteToAd7799(0x09);   //0000����0011(123Hz)1010(16.7HZ65dB)
//    /* Mode Register[MD2(0),MD1(0),MD0(0),PSW(0),0(0),0(0),0(0),0(0),(0),(0),0(0),0(0),FS3(1),FS2(0),FS1(1),FS0(0)]*/
//    /*ģʽ0 Continuous-Conversion Mode.��Fadc=16.7HZ;*/

//}
//u32 ReadAd7799ConversionData(void)
//{
//    u32 ConverData ;
//    unsigned char ADSAT ;
//    unsigned char ErrNUM=0;
//    WaiteRDY();              //�ȴ�����READY
//    WriteByteToAd7799(0x40);  //0100 0000 ������һ����Ϊ����״̬�Ĵ���
//    ADSAT=ReadByteFromAd7799();   //����״̬ 8λ
//    while((ADSAT&0x40)||(!(ADSAT&0x08)))    //������߶�д�쳣
//    {
//        WriteByteToAd7799(0xff);         //��λ
//        WriteByteToAd7799(0xff);
//        WriteByteToAd7799(0xff);
//        WriteByteToAd7799(0xff);
//        A7799_Ini();                   //��ʼ��
//        WaiteRDY();                     //��״̬
//        WriteByteToAd7799(0x40);        //��һ��������״̬
//        ADSAT=ReadByteFromAd7799();     //��״̬
//        
//        ErrNUM++;
//        if(ErrNUM>5)return(0xffffff);//if(ErrNUM>5)break;              //����5�ζ�������
//    }
//    
//    WriteByteToAd7799(0x58);  //0101 1000 ������һ�����������ݼĴ�����000
//    /* Writes to Communications Register Setting Next Operation as Continuous Read From Data Register*/
//    WaiteRDY();
//    /* Wait for RDY pin to go low to indicate end of calibration cycle*/
//    if(!AD_DRY)
//    {
//        ConverData=0 ;
//        ConverData=ReadByteFromAd7799();
//        ConverData=ConverData<<8 ;
//        ConverData=ReadByteFromAd7799()+ConverData;
//        ConverData=ConverData<<8 ;
//        ConverData=ReadByteFromAd7799()+ConverData;
//    }
//    /* Read Conversion Result from AD7799's Data Register*/
//    return(ConverData);
//}


