/**************************************************************************************************
  Revised:        $Date: 2007-07-06 11:19:00 -0700 (Fri, 06 Jul 2007) $
  Revision:       $Revision: 13579 $

  Copyright 2007 Texas Instruments Incorporated.  All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights granted under
  the terms of a software license agreement between the user who downloaded the software,
  his/her employer (which must be your employer) and Texas Instruments Incorporated (the
  "License"). You may not use this Software unless you agree to abide by the terms of the
  License. The License limits your use, and you acknowledge, that the Software may not be
  modified, copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio frequency
  transceiver, which is integrated into your product. Other than for the foregoing purpose,
  you may not use, reproduce, copy, prepare derivative works of, modify, distribute,
  perform, display or sell this Software and/or its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE PROVIDED 揂S IS?
  WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY
  WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
  IN NO EVENT SHALL TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE
  THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY
  INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST
  DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY
  THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/* =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 *   BSP (Board Support Package)
 *   Top-level BSP code file.
 * =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
 */

/* ------------------------------------------------------------------------------------------------
 *                                            Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "stm32f10x.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_usart.h"
#include "bsp.h"
#include "bsp_driver_defs.h"
//#include "bsp_external/mrfi_board_defs.h"
#include "ucos_ii.h"
#include "cc1100.h"

extern void MRFI_GpioIsr(void);

/**************************************************************************************************
 * @fn          BSP_Init
 *
 * @brief       Initialize the board and drivers.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
 /*
void BSP_InitForCC1100(void)
{
	printf_0(" \n\n@@@@@@@@  register isr\n\n");
	//register recevie data irq fun
	//hwGenGpioIrqReg(__mrfi_GDO2_BIT__,MRFI_GpioIsr,0);
	//hwGenGpioIrqReg(__mrfi_GDO2_BIT__,MRFI_GpioIsr,1);
	//hwGenGpioIrqReg(__mrfi_GDO2_BIT__,MRFI_GpioIsr,0);
	hwGenGpioIrqReg(__mrfi_GDO0_BIT__,MRFI_GpioIsr,1);
	//hwGenGpioIrqReg(__mrfi_GDO0_BIT__,MRFI_GpioIsr,0);

}
*/
//#define MRFI_INTERRUPT_LINE	EXTI_Line0
#define MRFI_INTERRUPT_LINE	GPIO_Pin_0

static void (*pMRFI_GpioIsr)(void)=NULL;

void RegMrfiIsr(void (*pf)(void))
{
	OS_CPU_SR stat;
	ENTER_CRITICAL(stat);
	pMRFI_GpioIsr=pf;
	EXIT_CRITICAL(stat);
}

void ReleaseMrfiIsr(void)
{
	OS_CPU_SR stat;
	ENTER_CRITICAL(stat);
	pMRFI_GpioIsr=NULL;
	EXIT_CRITICAL(stat);
}

void BSP_InitForCC1100(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(MRFI_TR_EN_CLK, ENABLE);
	/* Configure SPI pins: SCK, MISO and MOSI */
	GPIO_InitStructure.GPIO_Pin = MRFI_TX_EN_PIN | MRFI_RX_EN_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MRFI_TR_EN_PORT, &GPIO_InitStructure);

	RegMrfiIsr(MRFI_GpioIsr);
	  	/* Configure I/O for GDO0 and GDO2 */
	//GPIO_InitStructure.GPIO_Pin = SPI_CC1100_GDO0 | SPI_CC1100_GDO2;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	//GPIO_Init(SPI_CC1100_GDO, &GPIO_InitStructure);

	
	/* Enable Button GPIO clock */
	RCC_APB2PeriphClockCmd(MRFI_GDO_PORT_CLK | RCC_APB2Periph_AFIO, ENABLE);
	
	/* Configure Button pin as input floating */
	GPIO_InitStructure.GPIO_Pin = MRFI_GDO0_PIN | MRFI_GDO2_PIN;
	//GPIO_InitStructure.GPIO_Pin = MRFI_GDO0_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(MRFI_GDO_PORT, &GPIO_InitStructure);
	
	GPIO_EXTILineConfig(MRFI_GDO_PORT_SOURCE, MRFI_GDO0_PIN_SOURCE);
	//GPIO_EXTILineConfig(MRFI_GDO_PORT_SOURCE, MRFI_GDO2_PIN_SOURCE);
	
	/* Configure Button EXTI line */
	//EXTI_InitStructure.EXTI_Line = MRFI_GDO0_LINE | MRFI_GDO2_LINE;
	EXTI_InitStructure.EXTI_Line = MRFI_GDO0_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = MRFI_TRIGGER;
	//EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	/* Enable and set Button EXTI Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = MRFI_GDO0_IRQ_CHANNEL;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_SetPriority(MRFI_GDO0_IRQ_CHANNEL, 15);
	//NVIC_InitStructure.NVIC_IRQChannel = MRFI_GDO2_IRQ_CHANNEL;
	//NVIC_Init(&NVIC_InitStructure);
}

void EXTI0_IRQHandler(void)
//void EXTI1_IRQHandler(void)
//void EXTI2_IRQHandler(void)
//void EXTI3_IRQHandler(void)
//void EXTI4_IRQHandler(void)
//void EXTI9_5_IRQHandler(void)
//void EXTI15_10_IRQHandler(void)
{
#if OS_CRITICAL_METHOD == 3
	OS_CPU_SR  cpu_sr;
#endif
	OS_ENTER_CRITICAL();  //保存全局中断标志,关总中断/* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();	  //恢复全局中断标志
	NVIC_DisableIRQ(EXTI0_IRQn);
	//if(EXTI_GetITStatus(KEY_BUTTON_EXTI_LINE) != RESET)
	if(EXTI_GetITStatus(MRFI_INTERRUPT_LINE) != RESET)
	{
		/* Toggle LED1 */
		//STM_EVAL_LEDToggle(LED1);
		if(pMRFI_GpioIsr)
			pMRFI_GpioIsr();
		/* Clear the Key Button EXTI line pending bit */
		EXTI_ClearITPendingBit(MRFI_INTERRUPT_LINE);

	}
	NVIC_EnableIRQ(EXTI0_IRQn);
	OSIntExit();  //在os_core.c文件里定义,如果有更高优先级的任务就绪了,则执行一次任务切换            
}


#ifdef WERS_EXTENDER_DEVICE
//void WERS_485_Rx_Int(unsigned char ch);
void WERSPCPridge(UINT8 ch);
void USART1_IRQHandler(void)
{
	uint16_t sr;
#if OS_CRITICAL_METHOD == 3
	OS_CPU_SR  cpu_sr;
#endif

	//if(EXTI_GetITStatus(KEY_BUTTON_EXTI_LINE) != RESET)
	//if(EXTI_GetITStatus(EXTI_Line1) != RESET)
	OS_ENTER_CRITICAL();  //保存全局中断标志,关总中断/* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();	  //恢复全局中断标志

	sr=USART1->SR;
	if(sr & USART_FLAG_RXNE)
	{
		//WERS_485_Rx_Int(USART1->DR);
		WERSPCPridge(USART1->DR);
	}

	OSIntExit();  //在os_core.c文件里定义,如果有更高优先级的任务就绪了,则执行一次任务切换            
}

#endif

/* ************************************************************************************************
 *                                   Compile Time Integrity Checks
 * ************************************************************************************************
 */
BSP_STATIC_ASSERT( sizeof(  uint8_t ) == 1 );
BSP_STATIC_ASSERT( sizeof(   int8_t ) == 1 );
BSP_STATIC_ASSERT( sizeof( uint16_t ) == 2 );
BSP_STATIC_ASSERT( sizeof(  int16_t ) == 2 );
BSP_STATIC_ASSERT( sizeof( uint32_t ) == 4 );
BSP_STATIC_ASSERT( sizeof(  int32_t ) == 4 );


/**************************************************************************************************
 */
