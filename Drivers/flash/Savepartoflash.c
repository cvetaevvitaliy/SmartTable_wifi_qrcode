#include "Savepartoflash.h"
#include "stm32f10x_flash.h" 
#include <string.h>
//#include "FreeRTOS.h"
//#include "queue.h"
//#include "semphr.h"
//#include "task.h"
#include "bsp.h"
//使用内部Flash ISP 需开启内部高速晶振
//////////////////////////////////////////////////////////////////////////////////////////////
// extern WORKPARA_STRUCT WorkPara
#define WORKPARAAVR     SetParameters	       //修改成对应的结构体变量
#define WORKPARASTRUCT  ParameterStruct	   //修改成对应的结构体
 ParameterStruct SetParameters ;
#ifdef USE_FREERTOS
  xSemaphoreHandle g_ParSetMutex  ;
#endif

int GetSetedPar(ParameterStruct *ptPar)
{ 
    if(ptPar == NULL)
        return -1;
	#ifdef USE_FREERTOS
    if(xSemaphoreTake(g_ParSetMutex, portMAX_DELAY ) == pdTRUE)
    {
        memcpy(ptPar, &SetParameters, sizeof(ParameterStruct));
        xSemaphoreGive(g_ParSetMutex);
    }
    else
    {
        __nop();
    }
	#else
	memcpy(ptPar, &SetParameters, sizeof(ParameterStruct));
	#endif
    return 0;     
}
void ParameterDefault(void)
{
//	uint16_t serID[6];
//	u16 dev_id;
	memset(SetParameters.GPRSSeverAddr, 0, sizeof(SetParameters.GPRSSeverAddr));
	//SetParameters.GPRSSeverAddr[0] = 0;
	//SetParameters.SensorState[0] = 0;
 
		
}

////加载参数错误时，恢复默认值
void CFG_LoadDefaultVal(void)			   //修改成配置默认值
{
// 	WORKPARAAVR.Para1 = 0x1234;
// 	WORKPARAAVR.Para2 = 0x5634;
// 	WORKPARAAVR.Para3 = 0x7834; 
	ParameterDefault();
	
}
///////////////////////////////////////////////////////////////////////////////////////////////

void CFG_SaveWorkPara( ParameterStruct *ptPar)
{
   uint16_t i,CheckSum;
   uint32_t WriteAddr;

   CheckSum = 0xAA55;
   memcpy(&SetParameters,ptPar,  sizeof(ParameterStruct));
   FLASH_Unlock();				   //解锁;
   FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
   FLASH_ErasePage(CONFIGDATAADDR);//擦除这个扇区

   WriteAddr = CONFIGDATAADDR;
   for(i=0;i<sizeof(WORKPARASTRUCT)/2;i++)
   {
		FLASH_ProgramHalfWord(WriteAddr,((uint16_t*)&WORKPARAAVR)[i]);
		CheckSum ^= ((uint16_t*)&WORKPARAAVR)[i];

		WriteAddr += 2;//地址增加2.
   } 

   FLASH_ProgramHalfWord(WriteAddr,CheckSum);

   FLASH_Lock(); //上锁
}

void CFG_LoadWorkPara(void)
{
   uint16_t i,CheckSum;
   uint32_t WriteAddr;

   WriteAddr = CONFIGDATAADDR;
   CheckSum = 0xAA55;
	#ifdef USE_FREERTOS
	g_ParSetMutex = xSemaphoreCreateMutex();
	#endif
   for(i=0;i<sizeof(WORKPARASTRUCT)/2;i++)
   {
        ((uint16_t*)&WORKPARAAVR)[i] = *(uint16_t *)WriteAddr;
		CheckSum ^= *(uint16_t *)WriteAddr; 

		WriteAddr += 2;//地址增加2.
   }   

   CheckSum ^= *(uint16_t *)WriteAddr; 

   /////////////////////////////////////////////////////////////////////////////////////
   if(CheckSum != 0)//参数校验不正确，恢复默认设置
   {
	    CFG_LoadDefaultVal();
   }
   /////////////////////////////////////////////////////////////////////////////////////
}

