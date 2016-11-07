#include "stm32f10x.h"
#include <stdio.h>
#include <stdarg.h>
#include "log.h"
#include "serial.h"
#include "ff.h"
#include "diskio.h"
#include "ffconf.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "sys.h"
#include "zRTC_Timer.h"	 
#include "PCF8563.h"
#include "printf_embedded.h"


FATFS g_tLogFatFs;
FIL g_tLogFil;
FIL g_tLogFilWrite;
DIR g_tLogDir;
 
 
LOG_MNG *g_ptLog = NULL;
u8 *g_pLogBuf = NULL;
extern int fputc(int ch, FILE *f);
extern xSemaphoreHandle g_SDWriteProtectMutex;
 	 
u32 log_init(void)
{	 
    g_pLogBuf = (u8*)(LOG_START_ADDR + LOG_BUF_HEAD_SIZE);
    g_ptLog = (LOG_MNG*)(LOG_START_ADDR);
	if(g_ptLog->chk_sum != (u16)(g_ptLog->read+g_ptLog->write))
	{
		g_ptLog->read = 0;
		g_ptLog->write = 0;
		g_ptLog->syn_sta = 0;
	}

    
    return 0;
}

u32 write_log_buf( char **out, const char *format, va_list args )
{
	return 0;

}

int write_log(const char *format, ...)
{
        va_list args;

        va_start( args, format );
        
        return write_log_buf( 0, format, args );
}

int write_log_char(char c)
{
    
    if(g_ptLog->read > g_ptLog->write)
        g_ptLog->read++;
    g_pLogBuf[g_ptLog->write++] = c;   

    if(g_ptLog ->write == LOG_BUF_SIZE)
    {
        g_ptLog->write = 0;
        g_ptLog->read = 1;
		g_ptLog->syn_sta = 1;
    }
	g_ptLog->chk_sum = (u16)(g_ptLog->read + g_ptLog->write);
    return 0;
}

int show_allram_log(void)
{
    int read = g_ptLog->read;


    while(1)
    {
        if(read < g_ptLog->write )
        {
//			printf("%c", g_pLogBuf[read++]);
			//fputc(g_pLogBuf[read++],NULL);
			//Serial_PutData(serCOM1,&g_pLogBuf[read++], 1);
           if(read == g_ptLog->write)
            break;
        }
        else
        {
//			printf("%c", g_pLogBuf[read++]);
          //  fputc(g_pLogBuf[read++],NULL);
			 //Serial_PutData(serCOM1,&g_pLogBuf[read++], 1);
            if(read >= LOG_BUF_SIZE)
                read = 0; 
        }
    }
	return 0;    
}
u16 GetRAMLogLength(void)
{
	u16 wLen = 0;
	 if(g_ptLog->read < g_ptLog->write )
	 {
			wLen =  g_ptLog->write;
	 }
	 else
	{
		wLen = LOG_BUF_SIZE;
	}
	return wLen;
}

u16 GetSDLogLength(void)
{
	u16 wLen = 0;
	FIL *fp = &g_tLogFilWrite; 
	FRESULT res;
	res = f_open(fp, "SystemLog.txt" , FA_OPEN_ADD_DATA);
	if(res == FR_OK)
	{
		wLen = (u16)fp->fsize;
	}
	f_close(fp);
	return wLen;
}
int ReadLogFromSD(unsigned char *pData,unsigned int offset,unsigned int wLen)
{
	FIL *fp = &g_tLogFilWrite; 
	FRESULT res;
	UINT br; 
	#if SYSTEMLOG_SDCARDMODE_DISABLE
	return 0;
	#endif 
	if( xSemaphoreTake(g_SDWriteProtectMutex, portMAX_DELAY  ) == pdTRUE )
	{
		res = f_open(fp, "SystemLog.txt" , FA_OPEN_READONLY);
		if(res == FR_OK)
		{
			if(offset > fp->fsize)
				goto ERR;
			res = f_lseek(fp,fp->fsize-offset);	// 追加
			res = f_read(fp,pData,wLen, &br); //
			if(res != FR_OK)
			{
				goto ERR;
			}			 
			if(res != FR_OK)
			{
				goto ERR;
			}
		}
		else
			goto ERR;
		f_close(fp);	//关闭文件
		xSemaphoreGive( g_SDWriteProtectMutex );
		return FR_OK;
	}
	else
	{
		__nop();
	}
	return 0;
	ERR:
		f_close(fp);
		printfk("Read sd log data err\r\n");
		xSemaphoreGive( g_SDWriteProtectMutex );
		return -1;
	
}

 int WriteDataLogToSdCard(unsigned char *pData,unsigned int wLen)
{
	FIL *fp = &g_tLogFilWrite; 
	FRESULT res;
	UINT bw;
	DateTime tCurTime;
	char cTimeBuf[32];
	#if SYSTEMLOG_SDCARDMODE_DISABLE
	return 0;
	#endif
//	DateTime_Now(&tCurTime);
	PFC8563_GET_TIME(&tCurTime);
	sprintf(cTimeBuf, "%d-%d-%d %d:%d:%d\r\n", tCurTime.year,tCurTime.month,tCurTime.day,tCurTime.hour,tCurTime.min,tCurTime.sec);

	if( xSemaphoreTake(g_SDWriteProtectMutex, portMAX_DELAY  ) == pdTRUE )
	{
		res = f_open(fp, "SystemLog.txt" , FA_OPEN_ADD_DATA);
		if(res == FR_OK)
		{
			if(fp->fsize > 0x8000)  // 如果大于32k 删除  0x4000
			{
				f_close(fp);  // 必须先关闭文件 才能删除
				res = f_unlink("SystemLog.txt");	// 删除
				if(res == FR_OK)
				{
					// recreat a file
					res = f_open(fp, "SystemLog.txt" , FA_OPEN_ADD_DATA);
					if(res != FR_OK)
						return -1;
				}
			}	
			res = f_lseek(fp,fp->fsize);	// 追加
			res = f_write(fp,cTimeBuf,strlen(cTimeBuf), &bw); //写时间
			if(res != FR_OK)
			{
				goto ERR;
			}
			res = f_write(fp,pData,wLen, &bw);	// 写字符串
			if(res != FR_OK)
			{
				goto ERR;
			}
		}
		f_close(fp);	//关闭文件
		xSemaphoreGive( g_SDWriteProtectMutex );
		return FR_OK;
	}
	else
	{
		__nop();
	}
	ERR:
		printfk("sd save log data err\r\n");
		
		f_close(fp);
		xSemaphoreGive( g_SDWriteProtectMutex );
		return -1;
}

int WriteStrLogToSdcard( char *pcStr)
{
	
	unsigned char *pxNext;
	unsigned int wLen;

	#if SYSTEMLOG_SDCARDMODE_DISABLE
		return 0;
	#endif
	pxNext = ( unsigned char *)pcStr;
	wLen = 0;

	while(*pxNext)
	{		
		wLen++;
		pxNext++;
	}
	if(wLen > LOG_BUF_SIZE)
		wLen = LOG_BUF_SIZE;
	WriteDataLogToSdCard((u8*)pcStr,wLen);
	return 0;
	
}


 
 


