#include "stm32f10x.h"


#define CS_LOW()    GPIO_ResetBits(GPIOC, GPIO_Pin_10)               // Card Select
#define CS_HIGH()    GPIO_SetBits(GPIOC, GPIO_Pin_10)
#define DUMMY_CHAR 0xFF
