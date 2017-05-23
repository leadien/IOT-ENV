/**************************************************************************************************
  Filename:       WERS.h
  Revised:        $Date: 2008-06-19  $
  Revision:       $Revision: 1 $
  Author:         $Author: cbwang $

  Description:    This file supports the WERS appliction layer API.

**************************************************************************************************/
#ifndef __WERS_H__
#define __WERS_H__
#include <stdlib.h>
//#include "spca_general.h"
#include "ucos_ii.h"

#define osQuePost	OSQPost
#define osQuePend	OSQPend
#define osQueAccept	OSQAccept
#define osQueCreate	OSQCreate
#define osSemCreate	OSSemCreate
#define osSemPend	OSSemPend
#define osSemAccept	OSSemAccept
#define osSemPost	OSSemPost
#define osTaskCreate OSTaskCreate
#define osTimeDly	OSTimeDly
#define osTaskSuspend	OSTaskSuspend

//#define rf_alloc(x)	memAlloc(x)
//#define rf_free(p)	memFree(p)
#if 0
#if 0
#define rf_alloc(x)	malloc(x)
#define rf_free(p)	free(p)
#else
void * rf_alloc(size_t size);
void rf_free(void *p);
#endif
#else
#include "rf_mem.h"
#define rf_alloc(x)	rf_mem_malloc(x)
#define rf_free(p)	rf_mem_free(p)
#endif

#if 0
#define AP_LED_RADIO	LED2		/*radio is working*/
#define AP_LED_RUN		LED3	/*is run*/
#define AP_LED_DATA		LED4		/*data process*/
#else
#define AP_LED_RADIO	LED4		/*radio is working*/
#define AP_LED_RUN		LED3	/*is run*/
#define AP_LED_DATA		LED2		/*data process*/
#endif

#define SPMP_TurnLED(led,val)	STM_EVAL_LEDSet(led,val)



#define COM_CMD_APGET		0xF2
#define COM_CMD_APSET		0xF3
#define COM_CMD_AP_RESET	0xFE

/*---- WERS Response Code ------*/
// NWK error code
#define WERS_RES_OK					0x00
#define WERS_TIMEOUT				0x01
#define WERS_BAD_PARAM				0x02
#define WERS_NOMEM					0x03
#define WERS_NO_FRAME				0x04
#define WERS_NO_LINK				0x05
#define WERS_NO_JOIN				0x06
#define WERS_NO_SLEEP				0x07
#define WERS_NO_CHANNEL				0x08

#define WERS_INVAILD_DATA				0x11
#define WERS_INCOMLIANCE_SYTEMID		0x17

#define WERS_ROUTTABLE_COUNT			4

/*Payload Count*/
#define WERS_ParamCount				3


#define WERS_COMMAND_BLOCK			1
#define WERS_DATA_BLOCK				2
#define WERS_RESPONSE_BLOCK			3
#define WERS_EVENT_BLOCK			4
#define WERS_ACK_BLOCK				5	
#define WERS_CONTROL_BLOCK			5	//for wifi

#define WERS_DATA_IN					0x80
#define WERS_DATA_OUT					0x40

#define WERS_DATA_JOIN				1
#define WERS_DATA_QUIT				2
#define WERS_DATA_REFRESH_JOIN_NO	3
#define WERS_DATA_GET_JOIN_NO		4
#define WERS_DATA_ENUM_JOIN_START	5
#define WERS_DATA_ENUM_JOIN			6
#define WERS_DATA_CONNECT			7
#define WERS_DATA_DISCONNECT		8
#define WERS_DATA_ERROR				9
#define WERS_DATA_CONTAINER_ACK		10		//用于PC和AP通信时的流控
#define WERS_DATA_CONTAINER_BASE		16
#define WERS_DATA_TCP_CONTAINER			16
#define WERS_DATA_UDP_CONTAINER			17


#define WERS_DATA_BLOCK_HEADER_LEN	sizeof(WERSContainerStruct)
#define WERS_CONTAINER_SIZE			sizeof(WERSContainerStruct)

enum{									//for wifi
  WERS_CONTROL_PING = 160,
  WERS_CONTROL_PINGRESPONSE = 161,
};

#define WERS_SYSTEM_ID				0x1
#define WERS_SEND_RELAY_NUMBER		20

#define WERS_LISTEN_RELAY_NUMBER	10//20

#define WERS_OPEN_RTS_CTS_DATA_LEN	64

#if (!defined(WERS_AP_DEVICE) && !defined(WERS_EXTENDER_DEVICE))
#define WERS_NWK_TIME_OUT_TIME_LIMIT	10000 // 10s
#define WERS_ACK_TIME_OUT_TIME_LIMIT	1000 // 1s
#else
#define WERS_NWK_TIME_OUT_TIME_LIMIT	5000 // 5s
#endif

#define IS_TIME_OUT		CheckTimeOut(WERS_NWK_TIME_OUT_TIME_LIMIT)

enum E_MENU_NWK_STATE
{
	RESEND_REQUEST = 0x10,	
	DEVICE_INIT = 0xA0,
	DEVICE_JOIN,
	DEVICE_LINK,
	DEVICE_CONNECTED,
	DEVICE_CONNECTTING,
	DEVICE_DISCONNECTED,
	DEVICE_USB_IN,
	DEVICE_USB_OUT,
	DEVICE_RESET,
};

enum E_MENU_NWK_RESPONSE_CODE
{
	RESEND_COMMAND_PACKAGE = 0xB0,	
};


typedef UINT32 (*BroadCastCallBackProcess)(UINT8 *pData,UINT16 dataLen);


typedef __packed struct WERS_header{

	/*
	This field encodes as an unsigned integer the number
	of bytes in this container. WERS sysem uses this field
	to determine the size of the container*/
	UINT16 ContainerLength;

	/*
	0 undefined
	1 Command Block
	2 Data Block
	3 Response Block
	4 Event Block  */
	UINT8   ContainerType;

	/*
	This field contains the WERS OperationCode,
	ResponseCode, or EventCode. The Data Block will
	use the OperationCode from the Command Block*/
	UINT8   Code;		

	/*hotel&host id*/
	UINT16 SystemID;

	/*link id*/
	UINT8   Prot;	
	
	/*transmit id*/
	UINT8  TransID;

	/* data tag,support rout */
	UINT8 RouteMap[WERS_ROUTTABLE_COUNT];

	UINT32 GUID;	
	/*
	The contents of this field depend on the operation and
	phase of the WERS operation.*/
	//UINT16 Parameter[WERS_ParamCount];
	
}WERSContainerStruct;

//#define WERS_TYPE(t)		((t)&0x7F)
#define WERS_TYPE(t)		((t)&0x3F)
#define WERS_SET_CP(t)	(t)|=0x80
#define WERS_CLS_CP(t)	(t)&=~0x80
#define WERS_IS_CP(t)	((t)&0x80)

#define DATA_COMPRESS	0

typedef struct
{
	UINT8  addr[4];
} WERSAddr_t;
//void WERSAPTask(void *arg);
void WERS_init(void);


#endif  /* __WERS_H__ */

