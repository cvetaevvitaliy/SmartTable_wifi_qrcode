#ifndef __LOG_H
#define __LOG_H
#include <stdio.h>
#include "stm32f10x.h"
#define LOG_START_ADDR 0x20017800
#define LOG_BUF_HEAD_SIZE   16
#define LOG_BUF_SIZE     (2048 - LOG_BUF_HEAD_SIZE)


#ifndef NULL
#define NULL 0
#endif
#ifndef SYSTEMLOG_SDCARDMODE_DISABLE 
	#define SYSTEMLOG_SDCARDMODE_DISABLE 0
#endif
typedef struct _LOG_MNG
{
	u32 syn_sta;
	u32 write;
	u32 read;
	u16 chk_sum;
	u16 take_nop;
}LOG_MNG;

u32 log_init(void);
int write_log_char(char c);
int show_allram_log(void);
u16 GetRAMLogLength(void);
u16 GetSDLogLength(void);
int ReadLogFromSD(unsigned char *pData,unsigned int offset,unsigned int wLen);
int WriteStrLogToSdcard( char *pcStr);
int WriteDataLogToSdCard(unsigned char *pData,unsigned int wLen);
#endif

