#include "Savepartoflash.h"
#include "stm32f10x_flash.h" 
#include <string.h>
//#include "FreeRTOS.h"
//#include "queue.h"
//#include "semphr.h"
//#include "task.h"
#include "bsp.h"
//ʹ���ڲ�Flash ISP �迪���ڲ����پ���
//////////////////////////////////////////////////////////////////////////////////////////////
// extern WORKPARA_STRUCT WorkPara
#define WORKPARAAVR     SetParameters	       //�޸ĳɶ�Ӧ�Ľṹ�����
#define WORKPARASTRUCT  ParameterStruct	   //�޸ĳɶ�Ӧ�Ľṹ��
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

////���ز�������ʱ���ָ�Ĭ��ֵ
void CFG_LoadDefaultVal(void)			   //�޸ĳ�����Ĭ��ֵ
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
   FLASH_Unlock();				   //����;
   FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
   FLASH_ErasePage(CONFIGDATAADDR);//�����������

   WriteAddr = CONFIGDATAADDR;
   for(i=0;i<sizeof(WORKPARASTRUCT)/2;i++)
   {
		FLASH_ProgramHalfWord(WriteAddr,((uint16_t*)&WORKPARAAVR)[i]);
		CheckSum ^= ((uint16_t*)&WORKPARAAVR)[i];

		WriteAddr += 2;//��ַ����2.
   } 

   FLASH_ProgramHalfWord(WriteAddr,CheckSum);

   FLASH_Lock(); //����
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

		WriteAddr += 2;//��ַ����2.
   }   

   CheckSum ^= *(uint16_t *)WriteAddr; 

   /////////////////////////////////////////////////////////////////////////////////////
   if(CheckSum != 0)//����У�鲻��ȷ���ָ�Ĭ������
   {
	    CFG_LoadDefaultVal();
   }
   /////////////////////////////////////////////////////////////////////////////////////
}

