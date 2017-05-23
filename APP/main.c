/**
  ******************************************************************************
  * @file    SPI/M25P64_FLASH/main.c 
  * @author  MCD Application Team
  * @version V3.1.0
  * @date    06/19/2009
  * @brief   Main program body
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include <includes.h>
#include <stdlib.h>
//#include "spi_flash.h"
//#include "spi_cc1100.h"
#include "stm32_eval.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_fsmc.h"
#include "stm32f10x_rcc.h"
#include <stdio.h>
#include "uart.h"
#include "..\RF\cc1100.h"
#include "dtimer.h"
#include "..\RF\app\WERS.h"
#include "main.h"
#include "printf_level.h"
#include "flash.h"
#include "rf_wd.h"

//#include "lwip/opt.h"
//#include "lwip/def.h"
//#include "lwip/mem.h"
//#include "lwip/memp.h"
//#include "lwip/pbuf.h"
//#include "lwip/sys.h"
#include "lwip/tcpip.h"
#include "lwip/api.h"
#include "lwip/stats.h"
//#include "netif/etharp.h"
//#include "netif/loopif.h"
#include "netif/dm9kif.h"
#include "arch/sys_arch.h"

#include "sock_app.h"


/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup SPI_M25P64_FLASH
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* Private define ------------------------------------------------------------*/
#if 1
#define  FLASH_WriteAddress     (1 * 1024 * 1024)	/* 容量2MB, 测试地址1M */
#else
#define  FLASH_WriteAddress     0x700000	/* 容量 16MB */
#endif

#define  FLASH_ReadAddress      FLASH_WriteAddress
#define  FLASH_SectorToErase    FLASH_WriteAddress

#define  M25P64_FLASH_ID        0x202017	 /* ST原装开发板 */
#define  SST25VF016B_FLASH_ID	0xBF2541	 /* 安富莱STM32F103ZE-EK */
#define  BufferSize (countof(Tx_Buffer)-1)

/* Private macro -------------------------------------------------------------*/
#define countof(a) (sizeof(a) / sizeof(*(a)))

/* Private variables ---------------------------------------------------------*/
//uint8_t Tx_Buffer[] = "STM32F10x SPI Firmware Library Example: communication with an M25P64 SPI FLASH";
//uint8_t Tx_Buffer[] = "STM32F10x SPI Firmware Library Example: communication with an SST25VF1601B SPI FLASH";
//uint8_t Index, Rx_Buffer[BufferSize];
//volatile TestStatus TransferStatus1 = FAILED, TransferStatus2 = PASSED;
//__IO uint32_t FlashID = 0;

/* Private functions ---------------------------------------------------------*/
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength);
void RCC_Configuration(void);
void GPIO_Configuration(void);

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
//static OS_STK AppTaskStartStk[APP_TASK_START_STK_SIZE];
//static OS_STK AppTask1Stk[APP_TASK_KBD_STK_SIZE];
//static OS_STK WERSEDBgroundTaskStk[APP_TASK_WERSEDBground_SIZE];
OS_STK AppTaskStartStk[APP_TASK_START_STK_SIZE];
//static OS_STK AppTask1Stk[64];
//OS_STK WERSEDBgroundTaskStk[APP_TASK_WERSEDBground_STK_SIZE];
//static OS_STK UIPSendTaskStk[APP_UIP_SEND_STK_SIZE];
//static OS_STK UIPReceiveTaskStk[APP_UIP_RECEIVE_STK_SIZE];
//static OS_STK UIPSendTaskStk[APP_UIP_SEND_STK_SIZE];
//static OS_STK TimeUIPTaskStk[APP_TIME_UIP_STK_SIZE];
//OS_STK lwip_init_stk[APP_LWIP_INIT_STK_SIZE];
uint32_t g_dbg_cnt=0;

//OS_EVENT *uIP_Mutex;

static void AppTaskStart(void *p_arg);
void AppTask1(void *p_arg);
void UIPTask(void *arg);
void Tim_uip_proc(void *arg);
//void lwip_init_task(void * pParam);
void lwip_init(void);

__asm void SystemReset(void)
{
	MOV R0, #1           //; 
	MSR FAULTMASK, R0    //; 清除FAULTMASK 禁止一切中断产生
	LDR R0, =0xE000ED0C  //;
	LDR R1, =0x05FA0004  //; 
	STR R1, [R0]         //; 系统软件复位   
 
deadloop
    B deadloop        //; 死循环使程序运行不到下面的代码
}

//#define EXAMPLE_NAME	"uIP TCP Demp"
//#define EXAMPLE_DATE	"2010-02-01"

//void InitNet(void);
//void UserPro(void);
//void UIPReceiveTask(void *arg);
//void UIPSendTask(void *arg);
void ButtonInit()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 第1步：打开GPIOA GPIOC GPIOD GPIOF GPIOG的时钟
	   注意：这个地方可以一次性全打开
	*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC
			| RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOF | RCC_APB2Periph_GPIOG,
				ENABLE);

	/* 第2步：配置所有的按键GPIO为浮动输入模式(实际上CPUf复位后就是输入状态) */

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOG, &GPIO_InitStructure);	/* PG8 */
}

void FSMC_SRAM_Init(void)
{
  FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
  FSMC_NORSRAMTimingInitTypeDef  p;
  GPIO_InitTypeDef GPIO_InitStructure; 
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOG | RCC_APB2Periph_GPIOE |
                         RCC_APB2Periph_GPIOF, ENABLE);
  
/*-- GPIO Configuration ------------------------------------------------------*/
  /* SRAM Data lines configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9 |
                                GPIO_Pin_10 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 

  /* armfly : STM32F103ZE-EK使用了A21和A22做地址译码，因此PE5,PE6需要设置为总线模式
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
                                GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | 
                                GPIO_Pin_15;
 */  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 |
                                GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | 
                                GPIO_Pin_15 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_3 | GPIO_Pin_4;

  GPIO_Init(GPIOE, &GPIO_InitStructure);
  
  /* SRAM Address lines configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | 
                                GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_12 | GPIO_Pin_13 | 
                                GPIO_Pin_14 | GPIO_Pin_15;
  GPIO_Init(GPIOF, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | 
                                GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_Init(GPIOG, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13; 
  GPIO_Init(GPIOD, &GPIO_InitStructure);
   
  /* NOE and NWE configuration */  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 |GPIO_Pin_5;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  
  /* NE3 configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_12; 
  GPIO_Init(GPIOG, &GPIO_InitStructure);
  
  /* NBL0, NBL1 configuration */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; 
  GPIO_Init(GPIOE, &GPIO_InitStructure); 
  
/*-- FSMC Configuration ------------------------------------------------------*/
  p.FSMC_AddressSetupTime = 0;
  p.FSMC_AddressHoldTime = 0;
  p.FSMC_DataSetupTime = 2;
  p.FSMC_BusTurnAroundDuration = 0;
  p.FSMC_CLKDivision = 0;
  p.FSMC_DataLatency = 0;
  p.FSMC_AccessMode = FSMC_AccessMode_A;

  FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM3;
  FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
  FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;
  FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
  FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
  FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
  FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &p;
  FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &p;

  FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure); 

  /* Enable FSMC Bank1_SRAM Bank */
  FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM3, ENABLE);  
}

int main(void)
{
	INT8U  err;
    OSInit();                          /* Initialize "uC/OS-II, The Real-Time Kernel"              */

   /* Create the start task */
#if 0
	OSTaskCreateExt(AppTaskStart,
					(void *)0,
					(OS_STK *)&AppTaskStartStk[APP_TASK_START_STK_SIZE-1],
					APP_TASK_START_PRIO,
					APP_TASK_START_PRIO,
					(OS_STK *)&AppTaskStartStk[0],
					APP_TASK_START_STK_SIZE,
					(void *)0,
					OS_TASK_OPT_STK_CHK|OS_TASK_OPT_STK_CLR);
#else
	OSTaskCreate(AppTaskStart,(void *)0,(OS_STK *)&AppTaskStartStk[APP_TASK_START_STK_SIZE-1],APP_TASK_START_PRIO);
#endif
	/* Initialize Leds mounted on STM3210X-EVAL board */
#if (OS_TASK_NAME_SIZE > 13)
    OSTaskNameSet(APP_TASK_START_PRIO, "Start Task", &err);
#endif
	OSStart();				/* Start multitasking (i.e. give control to uC/OS-II)       */
}

/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval PASSED: pBuffer1 identical to pBuffer2
  *   FAILED: pBuffer1 differs from pBuffer2
  */
TestStatus Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
	while (BufferLength--)
	{
		if (*pBuffer1 != *pBuffer2)
		{
			return FAILED;
		}
		
		pBuffer1++;
		pBuffer2++;
	}
	
	return PASSED;
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	
	/* Infinite loop */
	while (1);
}
#endif

extern unsigned int g_dbg_sendto;
extern unsigned int g_dbg_q;
extern unsigned g_dbg_conn;

unsigned char test_align[]={1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7,8,9,0};
unsigned char gg_buf[32];
static  void  AppTaskStart (void *p_arg)
{
//	static unsigned char 
	volatile unsigned char *pCPUID;
	int i;
//	int i=1001,j;
	(void)p_arg;
	SystemInit();
	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);
	//FSMC_SRAM_Init();


	ButtonInit();
	DTimerInit();

	init_dbguart();
#if 0		//数据对齐情况测试
	printf("-----\n%x\n",*(unsigned int*)(test_align+1));
	printf("%x\n",*(unsigned int*)(test_align+2));
	printf("%x\n",*(unsigned short*)(test_align+1));
	printf("%x\n",*(unsigned short*)(test_align+2));
#endif
	//Flash_Write(4090,"1234567890",10);
	//Flash_Write(4090+2048,"\xff\xff\xff\xff",4);
#if 0		//外部SRAM测试
	pCPUID=(unsigned char *)0x68000000;
	sprintf((char *)pCPUID,"abcdefghijk1234567890987654321qwertyuiop\n");
	printf("%s",pCPUID);
	pCPUID += 64*1024-16;
	*((int *)pCPUID)=0x12345678;
	*((int *)(pCPUID+4))=0x78563412;
	*((int *)(pCPUID+8))=0x13245768;
	*((int *)(pCPUID+12))=0x87654321;
	printf("%08x,%08x\n",*((int *)pCPUID),*((int *)(pCPUID+4)));
	printf("%08x,%08x\n",*((int *)(pCPUID+8)),*((int *)(pCPUID+12)));
#endif

	STM_EVAL_LEDInit(LED1);
	STM_EVAL_LEDInit(LED2);
	STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED4);

	STM_EVAL_LEDOn(LED1);
	STM_EVAL_LEDOn(LED2);
	STM_EVAL_LEDOn(LED3);
	STM_EVAL_LEDOn(LED4);
	OSTimeDly(OS_TICKS_PER_SEC);
	STM_EVAL_LEDOff(LED1);
	STM_EVAL_LEDOff(LED2);
	STM_EVAL_LEDOff(LED3);
	STM_EVAL_LEDOff(LED4);
	//CC1100Init();
	#if (OS_TASK_STAT_EN > 0)
	OSStatInit();                                               /* Determine CPU capacity                                   */
	#endif

	//Flash_Write(FLASH_PAGE_SIZE*2+1,test_align,3);
	//Flash_Read(gg_buf,FLASH_PAGE_SIZE*2,32);
	//for(i=0;i<32;i++)printf("%02x ",gg_buf[i]);
	//printf("\n----:%s\n",gg_buf);

	//SystemReset();
	pCPUID=(unsigned char *)0x1ffff7e8;
	printf("**********\n");
	printf("CPUID:\n");
	for(i=0;i<11;i++)
		printf("%02X-",*pCPUID++);
	printf("%02X\n",*pCPUID);

//#ifndef WERS_EXTENDER_DEVICE
	lwip_init();
//#endif
	WERS_init();
	printf("WERS_init OK\n");
#if 1
	while(DEF_TRUE)
	{
		//SendDataToPC("12345\r\n",7);
		//printf("Chip ID = %d\r\n",halSpiReadStatus(0x31));
		STM_EVAL_LEDToggle(LED1);
	/*
		if(g_dbg_cnt>2)
		{
			DM9K_Dump();
			printf("%d,%d\n",g_dbg_sendto,g_dbg_conn);
		}
		g_dbg_cnt++;
	*/
		OSTimeDly(OS_TICKS_PER_SEC/5);
		
		/* Task body, always written as an infinite loop. */
		//OSTaskSuspend(OS_PRIO_SELF);
		//OSTimeDlyHMSM(0,0,0,200); 
	}
#endif
}

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
