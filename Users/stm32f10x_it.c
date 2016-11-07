/**
  ******************************************************************************
  * @file    GPIO/IOToggle/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h" 


 
void NMI_Handler(void)
{
}
 
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}
//void HardFault_Handler_C( unsigned int *args )
//{
//    volatile unsigned int stacked_r0;
//    volatile unsigned int stacked_r1;
//    volatile unsigned int stacked_r2;
//    volatile unsigned int stacked_r3;
//    volatile unsigned int stacked_r12;
//    volatile unsigned int stacked_lr;
//    volatile unsigned int stacked_pc;
//    volatile unsigned int stacked_psr;

//    stacked_r0 = ( ( unsigned long) args[0] );
//    stacked_r1 = ( ( unsigned long) args[1] );
//    stacked_r2 = ( ( unsigned long) args[2] );
//    stacked_r3 = ( ( unsigned long) args[3] );

//    stacked_r12 = ( ( unsigned long) args[4] );
//    stacked_lr = ( ( unsigned long) args[5] );
//    stacked_pc = ( ( unsigned long) args[6] );
//    stacked_psr = ( ( unsigned long) args[7] );

//    ( void )stacked_r0;
//    ( void )stacked_r1;
//    ( void )stacked_r2;
//    ( void )stacked_r3;

//    ( void )stacked_r12;
//    ( void )stacked_lr ;
//    ( void )stacked_pc ;
//    ( void )stacked_psr;
//    
//    while( 1 );
//}

// 
//__ASM  void HardFault_Handler(void) 
//{
//    TST LR, #4
//    ITE EQ
//    MRSEQ r0, MSP
//    MRSNE r0, PSP
//    B __cpp(HardFault_Handler_C)
//}
 
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

 
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}
 
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}
 
void SVC_Handler(void)
{
}
 
void DebugMon_Handler(void)
{
}
 
void PendSV_Handler(void)
{
}
 
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
