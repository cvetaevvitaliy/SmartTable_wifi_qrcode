/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : spi_flash.h
* Author             : MCD Application Team
* Version            : V2.0.3
* Date               : 09/22/2008
* Description        : Header for spi_flash.c file.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_FLASH_H
#define __SPI_FLASH_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"



#define		INT8U		unsigned char

 typedef enum { TX_MODE, RX_MODE }TRMODE;
 
 #define GPIO_CS                  GPIOA
 #define RCC_APB2Periph_GPIO_CS   RCC_APB2Periph_GPIOA
 #define GPIO_Pin_CS              GPIO_Pin_4
 
 #define GPIO_Pin_SCLK			  GPIO_Pin_5
 #define GPIO_Pin_SO			  GPIO_Pin_6
 #define GPIO_Pin_SI			  GPIO_Pin_7
 #define GPIO_GD                 GPIOB
 #define GPIO_Pin_GD2			  GPIO_Pin_0
 #define GPIO_Pin_GD0			  GPIO_Pin_11
 #define SPI_CC1101					SPI1
//#else /* USE_STM3210E_EVAL */
// #define GPIO_CS                  GPIOB
// #define RCC_APB2Periph_GPIO_CS   RCC_APB2Periph_GPIOB
// #define GPIO_Pin_CS              GPIO_Pin_12
//#endif

 

//#define CC1101_CS_PIN_OUT
 
#define CC1101_DRY			GPIO_ReadInputDataBit(GPIO_CS, GPIO_Pin_SO)		//

#define CC1101_READGOD2		GPIO_ReadInputDataBit(GPIO_GD,GPIO_Pin_GD2 )		//
#define CC1101_READGOD0		GPIO_ReadInputDataBit(GPIO_GD,GPIO_Pin_GD0 )		//



/* Exported macro ------------------------------------------------------------*/
/* Select SPI FLASH: Chip Select pin low  */
#define SPI_CC1101_CS_LOW()       GPIO_ResetBits(GPIO_CS, GPIO_Pin_CS)
/* Deselect SPI FLASH: Chip Select pin high */
#define SPI_CC1101_CS_HIGH()      GPIO_SetBits(GPIO_CS, GPIO_Pin_CS)

/* Private define ------------------------------------------------------------*/
#define WRITE      0x82  /* Write to Memory instruction */
#define READ       0xD3  /* Read from Memory instruction */
#define RDSR       0xD7  /* Read Status Register instruction  */
#define RDID       0x9F  /* Read identification */
#define PE         0x81  /* Page Erase instruction */
#define BE1        0xC7  /* Bulk Erase instruction */
#define BE2        0x94  /* Bulk Erase instruction */
#define BE3        0x80  /* Bulk Erase instruction */
#define BE4        0x9A  /* Bulk Erase instruction */

#define BUSY_Flag  0x01 /* Ready/busy status flag */

#define Dummy_Byte 0xff

/*******************************************************************************/
#define 	WRITE_BURST     	0x40						//连续写入
#define 	READ_SINGLE     	0x80						//读
#define 	READ_BURST      	0xC0						//连续读
#define 	BYTES_IN_RXFIFO     0x7F  						//接收缓冲区的有效字节数
#define 	CRC_OK              0x80 						//CRC校验通过位标志
// CC1100 STROBE, CONTROL AND STATUS REGSITER
#define CCxxx0_IOCFG2       0x00        // GDO2 output pin configuration
#define CCxxx0_IOCFG1       0x01        // GDO1 output pin configuration
#define CCxxx0_IOCFG0       0x02        // GDO0 output pin configuration
#define CCxxx0_FIFOTHR      0x03        // RX FIFO and TX FIFO thresholds
#define CCxxx0_SYNC1        0x04        // Sync word, high INT8U
#define CCxxx0_SYNC0        0x05        // Sync word, low INT8U
#define CCxxx0_PKTLEN       0x06        // Packet length
#define CCxxx0_PKTCTRL1     0x07        // Packet automation control
#define CCxxx0_PKTCTRL0     0x08        // Packet automation control
#define CCxxx0_ADDR         0x09        // Device address
#define CCxxx0_CHANNR       0x0A        // Channel number
#define CCxxx0_FSCTRL1      0x0B        // Frequency synthesizer control
#define CCxxx0_FSCTRL0      0x0C        // Frequency synthesizer control
#define CCxxx0_FREQ2        0x0D        // Frequency control word, high INT8U
#define CCxxx0_FREQ1        0x0E        // Frequency control word, middle INT8U
#define CCxxx0_FREQ0        0x0F        // Frequency control word, low INT8U
#define CCxxx0_MDMCFG4      0x10        // Modem configuration
#define CCxxx0_MDMCFG3      0x11        // Modem configuration
#define CCxxx0_MDMCFG2      0x12        // Modem configuration
#define CCxxx0_MDMCFG1      0x13        // Modem configuration
#define CCxxx0_MDMCFG0      0x14        // Modem configuration
#define CCxxx0_DEVIATN      0x15        // Modem deviation setting
#define CCxxx0_MCSM2        0x16        // Main Radio Control State Machine configuration
#define CCxxx0_MCSM1        0x17        // Main Radio Control State Machine configuration
#define CCxxx0_MCSM0        0x18        // Main Radio Control State Machine configuration
#define CCxxx0_FOCCFG       0x19        // Frequency Offset Compensation configuration
#define CCxxx0_BSCFG        0x1A        // Bit Synchronization configuration
#define CCxxx0_AGCCTRL2     0x1B        // AGC control
#define CCxxx0_AGCCTRL1     0x1C        // AGC control
#define CCxxx0_AGCCTRL0     0x1D        // AGC control
#define CCxxx0_WOREVT1      0x1E        // High INT8U Event 0 timeout
#define CCxxx0_WOREVT0      0x1F        // Low INT8U Event 0 timeout
#define CCxxx0_WORCTRL      0x20        // Wake On Radio control
#define CCxxx0_FREND1       0x21        // Front end RX configuration
#define CCxxx0_FREND0       0x22        // Front end TX configuration
#define CCxxx0_FSCAL3       0x23        // Frequency synthesizer calibration
#define CCxxx0_FSCAL2       0x24        // Frequency synthesizer calibration
#define CCxxx0_FSCAL1       0x25        // Frequency synthesizer calibration
#define CCxxx0_FSCAL0       0x26        // Frequency synthesizer calibration
#define CCxxx0_RCCTRL1      0x27        // RC oscillator configuration
#define CCxxx0_RCCTRL0      0x28        // RC oscillator configuration
#define CCxxx0_FSTEST       0x29        // Frequency synthesizer calibration control
#define CCxxx0_PTEST        0x2A        // Production test
#define CCxxx0_AGCTEST      0x2B        // AGC test
#define CCxxx0_TEST2        0x2C        // Various test settings
#define CCxxx0_TEST1        0x2D        // Various test settings
#define CCxxx0_TEST0        0x2E        // Various test settings

// Strobe commands
#define CCxxx0_SRES         0x30        // Reset chip.
#define CCxxx0_SFSTXON      0x31        // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
                                        // If in RX/TX: Go to a wait state where only the synthesizer is
                                        // running (for quick RX / TX turnaround).
#define CCxxx0_SXOFF        0x32        // Turn off crystal oscillator.
#define CCxxx0_SCAL         0x33        // Calibrate frequency synthesizer and turn it off
                                        // (enables quick start).
#define CCxxx0_SRX          0x34        // Enable RX. Perform calibration first if coming from IDLE and
                                        // MCSM0.FS_AUTOCAL=1.
#define CCxxx0_STX          0x35        // In IDLE state: Enable TX. Perform calibration first if
                                        // MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled:
                                        // Only go to TX if channel is clear.
#define CCxxx0_SIDLE        0x36        // Exit RX / TX, turn off frequency synthesizer and exit
                                        // Wake-On-Radio mode if applicable.
#define CCxxx0_SAFC         0x37        // Perform AFC adjustment of the frequency synthesizer
#define CCxxx0_SWOR         0x38        // Start automatic RX polling sequence (Wake-on-Radio)
#define CCxxx0_SPWD         0x39        // Enter power down mode when CSn goes high.
#define CCxxx0_SFRX         0x3A        // Flush the RX FIFO buffer.
#define CCxxx0_SFTX         0x3B        // Flush the TX FIFO buffer.
#define CCxxx0_SWORRST      0x3C        // Reset real time clock.
#define CCxxx0_SNOP         0x3D        // No operation. May be used to pad strobe commands to two
                                        // INT8Us for simpler software.

#define CCxxx0_PARTNUM      0x30
#define CCxxx0_VERSION      0x31
#define CCxxx0_FREQEST      0x32
#define CCxxx0_LQI          0x33
#define CCxxx0_RSSI         0x34
#define CCxxx0_MARCSTATE    0x35
#define CCxxx0_WORTIME1     0x36
#define CCxxx0_WORTIME0     0x37
#define CCxxx0_PKTSTATUS    0x38
#define CCxxx0_VCO_VC_DAC   0x39
#define CCxxx0_TXBYTES      0x3A
#define CCxxx0_RXBYTES      0x3B

#define CCxxx0_PATABLE      0x3E
#define CCxxx0_TXFIFO       0x3F
#define CCxxx0_RXFIFO       0x3F

//RF_SETTINGS is a data structure which contains all relevant CCxxx0 registers
typedef struct S_RF_SETTINGS
{
	INT8U FSCTRL2;		//自已加的
    INT8U FSCTRL1;   // Frequency synthesizer control.
    INT8U FSCTRL0;   // Frequency synthesizer control.
    INT8U FREQ2;     // Frequency control word, high INT8U.
    INT8U FREQ1;     // Frequency control word, middle INT8U.
    INT8U FREQ0;     // Frequency control word, low INT8U.
    INT8U MDMCFG4;   // Modem configuration.
    INT8U MDMCFG3;   // Modem configuration.
    INT8U MDMCFG2;   // Modem configuration.
    INT8U MDMCFG1;   // Modem configuration.
    INT8U MDMCFG0;   // Modem configuration.
    INT8U CHANNR;    // Channel number.
    INT8U DEVIATN;   // Modem deviation setting (when FSK modulation is enabled).
    INT8U FREND1;    // Front end RX configuration.
    INT8U FREND0;    // Front end RX configuration.
    INT8U MCSM0;     // Main Radio Control State Machine configuration.
    INT8U FOCCFG;    // Frequency Offset Compensation Configuration.
    INT8U BSCFG;     // Bit synchronization Configuration.
    INT8U AGCCTRL2;  // AGC control.
	INT8U AGCCTRL1;  // AGC control.
    INT8U AGCCTRL0;  // AGC control.
    INT8U FSCAL3;    // Frequency synthesizer calibration.
    INT8U FSCAL2;    // Frequency synthesizer calibration.
	INT8U FSCAL1;    // Frequency synthesizer calibration.
    INT8U FSCAL0;    // Frequency synthesizer calibration.
    INT8U FSTEST;    // Frequency synthesizer calibration control
    INT8U TEST2;     // Various test settings.
    INT8U TEST1;     // Various test settings.
    INT8U TEST0;     // Various test settings.
    INT8U IOCFG2;    // GDO2 output pin configuration
    INT8U IOCFG0;    // GDO0 output pin configuration
    INT8U PKTCTRL1;  // Packet automation control.
    INT8U PKTCTRL0;  // Packet automation control.
    INT8U ADDR;      // Device address.
    INT8U PKTLEN;    // Packet length.
} RF_SETTINGS;

// RF output power = 0 dBm
// RX filterbandwidth = 540.000000 kHz
// Deviation = 0.000000
// Datarate = 250.000000 kbps
// Modulation = (7) MSK
// Manchester enable = (0) Manchester disabled
// RF Frequency = 433.000000 MHz
// Channel spacing = 199.951172 kHz
// Channel number = 0
// Optimization = Sensitivity
// Sync mode = (3) 30/32 sync word bits detected
// Format of RX/TX data = (0) Normal mode, use FIFOs for RX and TX
// CRC operation = (1) CRC calculation in TX and CRC check in RX enabled
// Forward Error Correction = (0) FEC disabled
// Length configuration = (1) Variable length packets, packet length configured by the first received byte after sync word.
// Packetlength = 255
// Preamble count = (2) 4 bytes
// Append status = 1
// Address check = (11) No address check
// FIFO autoflush = 0
// Device address = 0
// GDO0 signal selection = ( 6)
// GDO2 signal selection = (11) Serial Clock
const RF_SETTINGS rfSettings = {
	0x00,
	0x08, // FSCTRL1 Frequency synthesizer control.

	0x00, // FSCTRL0 Frequency synthesizer control.
	0x10, // FREQ2 Frequency control word, high byte.
	0xA7, // FREQ1 Frequency control word, middle byte.
	0x62, // FREQ0 Frequency control word, low byte.

	0x5B,  //0x2D, // MDMCFG4 Modem configuration.
	0xF8,  //0x3B, // MDMCFG3 Modem configuration.
	0x02,   //0x73, // MDMCFG2 Modem configuration.
	0x22, // MDMCFG1 Modem configuration.
	0xF8, // MDMCFG0 Modem configuration.
	0x00, // CHANNR Channel number.
	0x47, // DEVIATN Modem deviation setting (when FSK modulation is enabled).

	0xB6, // FREND1 Front end RX configuration.
	0x10, // FREND0 Front end RX configuration.
	0x18, // MCSM0 Main Radio Control State Machine configuration.
	0x1D, // FOCCFG Frequency Offset Compensation Configuration.
	0x1C, // BSCFG Bit synchronization Configuration.
	0x07, // AGCCTRL2 AGC control.
	0x00, //0x00, // AGCCTRL1 AGC control.
	0xB2, // AGCCTRL0 AGC control.
	0xEA, // FSCAL3 Frequency synthesizer calibration.

	0x2A, // FSCAL2 Frequency synthesizer calibration.

	0x00, // FSCAL1 Frequency synthesizer calibration.
	0x1F, // FSCAL0 Frequency synthesizer calibration.
	0x59, // FSTEST Frequency synthesizer calibration.
	0x88, // TEST2 Various test settings.
	0x31, // TEST1 Various test settings.
	0x0B, // TEST0 Various test settings.
	0x0B, // IOCFG2 GDO2 output pin configuration.
	0x06, // IOCFG0D GDO0 output pin configuration.
	0x05, // PKTCTRL1 Packet automation control.
	0x05, // PKTCTRL0 Packet automation control.
	0x34, // ADDR Device address.
	0xff // PKTLEN Packet length.
};

/* Exported functions ------------------------------------------------------- */
/*----- High layer function -----*/
void SPI_FLASH_Init(void);  
u32 SPI_FLASH_ReadID(void);

/*----- Low layer function -----*/
u8 SPI_CC1101_ReadByte(void);
u8 SPI_CC1101_SendByte(u8 byte);
u16 SPI_FLASH_SendHalfWord(u16 HalfWord);
void SPI_FLASH_WaitForWriteEnd(void);

u8 CC1101_Config(void);
void CC1101_test(void);
#endif /* __SPI_FLASH_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
