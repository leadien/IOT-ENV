#include "def.h"
#include "stm32f10x_iwdg.h"
#include "dtimer.h"
#include "rf_wd.h"
#include "wers.h"

extern volatile UINT32	g_ApTaskCondition;
extern volatile UINT32	g_ApInitOk;
extern volatile UINT32	g_intr_idle;
extern void ResetReceive(void);
static void watchDogTask(void);

int32_t g_WatchDogTimerID = -1;

void WatchDogInit(void)
{
	/* watch dog */
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

  /* IWDG counter clock: 40KHz(LSI) / 64 = 0.625 KHz */
  IWDG_SetPrescaler(IWDG_Prescaler_64);

  /* Set counter reload value to 4095 */
  IWDG_SetReload(4095);		// (4095+1)*(1/625) = 6.5536s

  /* Reload IWDG counter */
  IWDG_ReloadCounter();

  /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
	
#ifndef RF_WATCHDOG_DEBUG_EN

#ifndef WERS_EXTENDER_DEVICE
	g_WatchDogTimerID = CreateTimerEx(watchDogTask, 800 ,DSTIME_MODE_CYCLE);
#else
	g_WatchDogTimerID = CreateTimerEx(watchDogTask, 500 ,DSTIME_MODE_CYCLE);
#endif
	if(g_WatchDogTimerID < 0)
	while(1);
	StartTimer(g_WatchDogTimerID);
#endif
	IWDG_Enable();
}

#ifndef WERS_EXTENDER_DEVICE
static void watchDogTask(void)
{
	if (  0 == g_ApInitOk )  // init state
	{
		if ( g_ApTaskCondition++<40 ) // < 32s
		{
		        //feed dog
  			IWDG_ReloadCounter();
		}
	}
	else if ( 1 == g_ApInitOk ) // task running state
	{
		if ( g_ApTaskCondition++<20 ) // < 16s		
		{
			 // AP task is running.
			
		    //feed dog
  			IWDG_ReloadCounter();
		}

	
		 /*  This can happen for several reasons:
		 *   1) Incoming packet has an incorrect format or is corrupted.
		 *   2) The receive FIFO overflowed.  Overflow is indicated by the high
		 *      bit of rxBytes.  This guarantees the value of rxBytes value will not
		 *      match the number of bytes in the FIFO for overflow condition.
		 *   3) Interrupts were blocked for an abnormally long time which
		 *      allowed a following packet to at least start filling the
		 *      receive FIFO.  In this case, all received and partially received
		 *      packets will be lost - the packet in the FIFO and the packet coming in.
		 *      This is the price the user pays if they implement a giant
		 *      critical section.
		 *   4) A failed transmit forced radio to IDLE state to flush the transmit FIFO.
		 *      This could cause an active receive to be cut short.*/		 
		if ( g_intr_idle++ >= 4 ) // 4*800 = 3200ms
		{
			g_intr_idle = 0;
			ResetReceive();
		}
	}

}
#else
extern volatile UINT8 gs_DeviceState;
extern volatile uint32_t g_NeedReset;

static void watchDogTask(void)
{

	if ( DEVICE_INIT == gs_DeviceState ) // init state
	{
		if ( g_ApTaskCondition++ < 40 ) // < 20s
		{
			//feed dog
			IWDG_ReloadCounter();
		}		
	}
	else 
	{	if ( 0 == g_NeedReset)
		{
			if ( g_ApTaskCondition++ < 20  ) // check task runing state    < 10s
			{
			        //feed dog
				IWDG_ReloadCounter();
			}
		}

		 /*  This can happen for several reasons:
		 *   1) Incoming packet has an incorrect format or is corrupted.
		 *   2) The receive FIFO overflowed.  Overflow is indicated by the high
		 *      bit of rxBytes.  This guarantees the value of rxBytes value will not
		 *      match the number of bytes in the FIFO for overflow condition.
		 *   3) Interrupts were blocked for an abnormally long time which
		 *      allowed a following packet to at least start filling the
		 *      receive FIFO.  In this case, all received and partially received
		 *      packets will be lost - the packet in the FIFO and the packet coming in.
		 *      This is the price the user pays if they implement a giant
		 *      critical section.
		 *   4) A failed transmit forced radio to IDLE state to flush the transmit FIFO.
		 *      This could cause an active receive to be cut short.*/		 
		if ( g_intr_idle++ >= 12 ) // 12*500 = 6s
		{
			g_intr_idle = 0;
			ResetReceive();
		}
	}

	
	//printf_1("dogTask\n");
	//sio_printf(">>>>>>>dogTask\n");
}
#endif
