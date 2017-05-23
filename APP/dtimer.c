#include <stdio.h>
#include "stm32f10x.h"
#include "misc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "ucos_ii.h"

#include "dtimer.h"
//#include "ucos_ii.h"

#define TIMER_NO	3
#define SOFTTIMER_NUM	4


#define _TIMER(timer_no)	TIM##timer_no
#define	TIMER(timer_no)	_TIMER(timer_no)

#define CURRENT_TIMER	TIMER(TIMER_NO)

#if TIMER_NO==1
	#undef TIMER_NO
	#define TIMER_NO	8
#endif

#if TIMER_NO == 8
#define RCC_NO	2
#define RCC_APBPeriphXClockCmd RCC_APB2PeriphClockCmd
#else
#define RCC_NO	1
#define RCC_APBPeriphXClockCmd RCC_APB1PeriphClockCmd
#endif

#define __RCC_APBPeriph_TIM(rcc_no,timer_no)	RCC_APB##rcc_no##Periph_TIM##timer_no
#define _RCC_APBPeriph_TIM(rcc_no,timer_no)	__RCC_APBPeriph_TIM(rcc_no,timer_no)
#define RCC_APBPeriph_TIM	_RCC_APBPeriph_TIM(RCC_NO,TIMER_NO)

#define __TIMER_IRQHandler(timer_no)	TIM##timer_no##_IRQHandler
#define _TIMER_IRQHandler(timer_no)		__TIMER_IRQHandler(timer_no)
#define TIMER_IRQHandler				_TIMER_IRQHandler(TIMER_NO)

#define __TIMER_IRQn(timer_no)	TIM##timer_no##_IRQn
#define _TIMER_IRQn(timer_no)	__TIMER_IRQn(timer_no)
#define TIMER_IRQn				_TIMER_IRQn(TIMER_NO)

typedef __packed struct _stimer_s{
	uint8_t valid;
	uint8_t start;
	uint16_t mode;
	uint32_t interval;
	uint32_t val;
	void (*pISR)(void);
}stimer_t,*pstimer_t;

stimer_t g_stimer[SOFTTIMER_NUM];

/************************************************************************/

#if 1
static volatile uint32_t g_msec;

void TIMER_IRQHandler(void)
{
	TIM_TypeDef *tim=CURRENT_TIMER;
	uint32_t i;
#if OS_CRITICAL_METHOD == 3
	OS_CPU_SR  cpu_sr;
#endif
	//uint32_t stat;
	OS_ENTER_CRITICAL();  //保存全局中断标志,关总中断/* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();	  //恢复全局中断标志
	
	if(tim->SR & TIM_IT_CC1)
	{
		//ENTER_CRITICAL(stat);
		__disable_irq();
		tim->SR = ~TIM_IT_CC1;
		tim->CCR1+=1000;
		g_msec++;
		//EXIT_CRITICAL(stat);
		__enable_irq();
		for(i=0;i<SOFTTIMER_NUM;i++)
		{
			if(g_stimer[i].valid && g_stimer[i].start)
			{
				if(g_stimer[i].val)g_stimer[i].val--;
				else
				{
					if(g_stimer[i].mode == DSTIME_MODE_CYCLE)g_stimer[i].val=g_stimer[i].interval;
					else g_stimer[i].start=0;
					if(g_stimer[i].pISR)g_stimer[i].pISR();
				}
			}
		}
	}
	OSIntExit();  //在os_core.c文件里定义,如果有更高优先级的任务就绪了,则执行一次任务切换
}
#endif

void DTimerInit()
{
	uint32_t i;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	for(i=0;i<SOFTTIMER_NUM;i++)
	{
		g_stimer[i].valid=0;
		g_stimer[i].pISR=NULL;
	}

	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APBPeriphXClockCmd(RCC_APBPeriph_TIM, ENABLE);

  /* Enable the TIM3 gloabal Interrupt */
	//NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannel = TIMER_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_SetPriority(TIMER_IRQn, 7);

	TIM_TimeBaseStruct.TIM_Period=65535u;
	//TIM_TimeBaseStruct.TIM_Prescaler=36;
	TIM_TimeBaseStruct.TIM_Prescaler=72;
	TIM_TimeBaseStruct.TIM_ClockDivision=0;
	TIM_TimeBaseStruct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseStruct.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(CURRENT_TIMER, &TIM_TimeBaseStruct);
	//TIM_PrescalerConfig(CURRENT_TIMER, 36, TIM_PSCReloadMode_Immediate);
	
	  /* Output Compare Timing Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 1000;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OC1Init(CURRENT_TIMER, &TIM_OCInitStructure);

	TIM_Cmd(CURRENT_TIMER, ENABLE);
	TIM_ITConfig(CURRENT_TIMER,TIM_IT_CC1,ENABLE);
	//TIM_SetCounter(CURRENT_TIMER, 0);
	//TIM_GetCounter(CURRENT_TIMER);
	//TIM_SetAutoreload(CURRENT_TIMER, 1);
}

void uDelay(uint16_t usec)
{
#if 1
	uint16_t save;
	TIM_TypeDef *tim=CURRENT_TIMER;
	save=tim->CNT;
	while((uint16_t)(tim->CNT-save)<usec);
#else
	CURRENT_TIMER->CCR2=CURRENT_TIMER->CNT+usec;
	CURRENT_TIMER->SR=~TIM_IT_CC2;
	while(!(CURRENT_TIMER->SR&TIM_IT_CC2));
	return;
#endif
}

void mDelay(uint32_t msec)
{
	uint16_t save;
	TIM_TypeDef *tim=CURRENT_TIMER;
	save=tim->CNT;
	do{
		while((uint16_t)(tim->CNT-save)<((uint16_t)1000));
		save+=1000u;
		msec--;
	}while(msec);	
}

uint32_t tmrMsTimeGet(void)
{
	return g_msec;
}

uint32_t tmrDateTimeGet(void)
{
	return 0;
}

void MsDelay(uint32_t msec)
{
	uint32_t save;
	save=g_msec;
	while(g_msec-save<msec);
}

int32_t CreateTimerEx(void (*pISR)(void),uint32_t interval,int16_t mode)
{
	uint32_t i;
	for(i=0;i<SOFTTIMER_NUM;i++)
	{
		if(g_stimer[i].valid==0)
		{
			g_stimer[i].interval=interval;
			g_stimer[i].val=interval;
			g_stimer[i].pISR=pISR;
			g_stimer[i].mode=mode;
			g_stimer[i].start=0;
			g_stimer[i].valid=1;
			return i;
		}
	}
	return -1;
}

int32_t CreateTimer(void (*pISR)(void),uint32_t interval)
{
	uint32_t ret;
	//ret = CreateTimerEx(pISR, interval ,DSTIME_MODE_CYCLE);
	ret = CreateTimerEx(pISR, interval ,DSTIME_MODE_ONCE);
	StartTimer(ret);
	return ret;
}

void StopTimer(int32_t timerID)
{
	if(timerID<SOFTTIMER_NUM)
	{
		g_stimer[timerID].start=0;	
	}
}
void StartTimer(int32_t timerID)
{
	if(timerID<SOFTTIMER_NUM)
	{
		g_stimer[timerID].start=1;	
	}
}

void ResetTimer(int timerID)
{
	if(timerID<SOFTTIMER_NUM)
	{
		g_stimer[timerID].val=g_stimer[timerID].interval;	
		g_stimer[timerID].start=1;	
	}
}

void DeleteTimer(int32_t timerID)
{
	if(timerID<SOFTTIMER_NUM)g_stimer[timerID].valid=0;
}

