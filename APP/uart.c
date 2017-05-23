#include <stdio.h>
#include "stm32_eval.h"
#include "misc.h"
#include "stm32f10x_usart.h"
#include "uart.h"
#include "main.h"

//#if DEBUG_COM == COM2
//#define DEBUG_COM_PORT	EVAL_COM2
//#endif
union _InitStruct{
	USART_InitTypeDef	USART_InitStructure;
	NVIC_InitTypeDef	NVIC_InitStructure;
}g_InitStruct;

void init_dbguart(void)
{
	g_InitStruct.USART_InitStructure.USART_BaudRate = 115200;
	g_InitStruct.USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	g_InitStruct.USART_InitStructure.USART_StopBits = USART_StopBits_1;
	g_InitStruct.USART_InitStructure.USART_Parity = USART_Parity_No;
	g_InitStruct.USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	g_InitStruct.USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	STM_EVAL_COMInit(DEBUG_COM, &g_InitStruct.USART_InitStructure);
}
#ifdef WERS_EXTENDER_DEVICE
void init_485_uart(void)
{
	g_InitStruct.USART_InitStructure.USART_BaudRate = 19200;
	//g_InitStruct.USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	g_InitStruct.USART_InitStructure.USART_WordLength = USART_WordLength_9b;
	g_InitStruct.USART_InitStructure.USART_StopBits = USART_StopBits_1;
	//g_InitStruct.USART_InitStructure.USART_Parity = USART_Parity_No;
	g_InitStruct.USART_InitStructure.USART_Parity = USART_Parity_Even;
	g_InitStruct.USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	g_InitStruct.USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	STM_EVAL_COMInit(RS485_COM, &g_InitStruct.USART_InitStructure);
	USART_ITConfig(RS485_COM_PORT, USART_IT_RXNE, ENABLE);
	
	g_InitStruct.NVIC_InitStructure.NVIC_IRQChannel = RS485_COM_IRQn;
	g_InitStruct.NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	g_InitStruct.NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	g_InitStruct.NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&g_InitStruct.NVIC_InitStructure);
}
#endif

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */
	if(ch=='\n')
	{
		USART_SendData(DEBUG_COM_PORT, '\r');
		/* Loop until the end of transmission */
		while (USART_GetFlagStatus(DEBUG_COM_PORT, USART_FLAG_TC) == RESET);
	}
	USART_SendData(DEBUG_COM_PORT, (uint8_t) ch);
	/* Loop until the end of transmission */
	while (USART_GetFlagStatus(DEBUG_COM_PORT, USART_FLAG_TC) == RESET);
	
	return ch;
}

void Dump(void *buf,uint32_t len)
{
	uint32_t i,j;
	uint8_t *p=(uint8_t *)buf;
	for(i=0;;)
	{
		for(j=0;j<16;j++,i++)
		{
			if(i==len)
			{
				printf("\n");
				return;
			}
			printf("%02x ",p[i]);
		}
		printf("\n");
	}
}

