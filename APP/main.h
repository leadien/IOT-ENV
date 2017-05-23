#ifndef __MAIN_H__
#define __MAIN_H__

#define  APP_TASK_START_PRIO			2

//#define  OS_PROBE_TASK_PRIO			11
//#define  OS_PROBE_TASK_ID				11

//#define APP_TASK_WERSEDBground_PRIO	5
//#define APP_UIP_RECEIVE_PRIO			2
//#define APP_TIME_UIP_PRIO				4
//#define APP_UIP_SEND_PRIO				11
//#define APP_LWIP_INIT_PRIO			3
//#define APP_UIP_RECEIVE_PRIO			7
//#define APP_UIP_SEND_PRIO				6
//#define APP_TIME_UIP_PRIO				8
#define APP_TASK_WERSEDBground_PRIO		28

#if 0
#define APP_TASK_WERSEDBground_STK_SIZE	256
#define APP_UIP_RECEIVE_STK_SIZE		256
#define APP_UIP_SEND_STK_SIZE			256
#define APP_TIME_UIP_STK_SIZE			256
#define APP_LWIP_INIT_STK_SIZE			256
#else
#define APP_TASK_WERSEDBground_STK_SIZE	96
//#define APP_LWIP_INIT_STK_SIZE			128
#endif

#ifndef WERS_EXTENDER_DEVICE
#define DEBUG_COM		COM1
#define DEBUG_COM_PORT	EVAL_COM1
#define RS485_COM		COM2
#define RS485_COM_PORT	EVAL_COM2
#define RS485_COM_IRQn	USART2_IRQn
#else
#define DEBUG_COM		COM2
#define DEBUG_COM_PORT	EVAL_COM2
#define RS485_COM		COM1
#define RS485_COM_PORT	EVAL_COM1
#define RS485_COM_IRQn	USART1_IRQn
#endif

#endif

