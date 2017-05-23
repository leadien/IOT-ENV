/**************************************************************************************************
  Filename:       WERS.c
  Revised:        $Date: 2008-06-19  $
  Revision:       $Revision: 1 $
  Author:         $Author: cbwang $

  Description:    This file supports the WERS appliction layer API.

**************************************************************************************************/
//#define DEBUG_LEVEL 4

/******************************************************************************
 * INCLUDES
 */
#include "stm32_eval.h"
#include "def.h"
#include "bsp.h"
#include "WERS.h"
#include "mrfi.h"
#include "nwk_types.h"
#include "nwk_api.h"
#include "nwk_frame.h"
#include "nwk.h"
#include "nwk_app.h"
#include "mrfi.h"
#include "nwk_globals.h"
#include "nwk_freq.h"
#include "dtimer.h"
#include "uart.h"
//#include "spca_general.h"
//#include "mem_api.h"
#include <string.h>
#include <stdlib.h>
//#include "fs_api.h"
//#include "_uart.h"
//#include "bsp_external/mrfi_board_defs.h"
//#include "list.h"
//#include "rf_mem.h"
//#include "irq_number.h"
//#include "hw_sdram.h"
#include "lwip\sockets.h"
#include "ucos_ii.h"
#include "flash.h"
#include "main.h"
#include "rf_wd.h"


//#define WERS_DEBUG
#ifndef WERS_EXTENDER_DEVICE
#include "sock_app.h"
#define WERS_USING_NETWORK
#endif

#define WERS_NEWVER_TRANSACTION_SUPPORT
//#define WERS_NEWVER_TRANSACTION_DBGON
/******************************************************************************
 * MACROS
 */
//#define rf_printf_0	printf_0
#define rf_printf_0	printf_1
#define rf_printf_1	printf_1
#if 1
//#define rf_hwUsWait(Usdelay)	hwWait(1, Usdelay)
//#define rf_hwMsWait(delay)		hwWait(0, delay)
#define rf_hwUsWait(usec)	uDelay(usec)
//#define rf_hwMsWait(msed)	MsDelay(msed)
#define rf_hwMsWait(msed)	mDelay(msed)
#else

#define rf_hwUsWait(Usdelay)	\
{ 	   							\
	/* 72Mhz */					\
	UINT32 re = Usdelay;		\
	UINT32 cnt = 72*re; 		\
	while(cnt--); 				\
}

#define rf_hwMsWait(delay)		\
{ 	   							\
	/* 72Mhz */					\
	UINT32 re = delay;			\
	UINT32 cnt = 72000*re; 		\
	while(cnt--); 				\
}
#endif

#define Set_485_BackOffTimeOut(x)		g_485BackOffBegintime = x;
#define Clear_485_BackOffTimeOut()		g_485BackOffBegintime = 0;

#define SetBackOffTimeOut(x)		g_BackOffBegintime = x;
#define ClearBackOffTimeOut()	g_BackOffBegintime = 0;
#define SetTimeOut(x)	sBegintime = x;
#define ClearTimeOut()	sBegintime = 0;
#define Change485CurrentState(x)	g_485CurrentStates = x;


#define M_HiByteOfWord(x)	(UINT8)((x) >> 8)
#define M_LoByteOfWord(x)	(UINT8)(x)

#define M_Byte0OfDWord(x)	(UINT8)(x & 0x000000FF)
#define M_Byte1OfDWord(x)	(UINT8)(((x) >> 8) & 0x000000FF)
#define M_Byte2OfDWord(x)	(UINT8)(((x) >> 16) & 0x000000FF)
#define M_Byte3OfDWord(x)	(UINT8)(((x) >> 24) & 0x000000FF)


#define WERSGenerateGUID()	WERSMsTimeGet()

#define  Calculate_RF_BackOff_Time(datasize,exms)   ( (((datasize + (datasize/MAX_APP_PAYLOAD*14 ))*8)*1000 /RF_DATA_RATE )+ exms );

/******************************************************************************
 * CONSTANTS AND DEFINES
 */
#define WERS_485_AP_HOST
//#define WERS_485_PC_HOST

/* Version: V01.0.1 */
#define MAIN_VERSION	0x10
#define SUB_VERSION		0x02


#define RF_DATA_RATE	76000   //76Kbps
#define DEVICE_PATH		"A:\\CONFIG\\device.dat"

#define BROADCAST_INTER			3000
#define CONNECTED_INTER		3000

#define EX_PING_TIME_INTER		20000	//20s
//#define EX_PING_TIME_INTER		5000	//5s		//Modified by shichu

#if 0
#define OS_PRIO_WERS_RF_RECEIVER_TASK		5
#define OS_PRIO_WERS_485_SEND_TASK			6
#define OS_PRIO_WERS_485_BRIDGE_TASK		6

#define OS_PRIO_WERS_485_RECEIVE_TASK		7
#define OS_PRIO_WERS_485_POLL_TASK			8
#define OS_PRIO_WERS_485_SEND_CHECK_TASK	9
#else

#if 1

#define OS_PRIO_WERS_RF_RECEIVER_TASK		5
#define OS_PRIO_WERS_RF_BRIDGE_TASK			6
#define OS_PRIO_WERS_485_BRIDGE_TASK		6
#define OS_PRIO_WERS_485_SEND_TASK			6

#define OS_PRIO_WERS_485_RECEIVE_TASK		7
#define OS_PRIO_WERS_485_POLL_TASK			8
#define OS_PRIO_WERS_485_SEND_CHECK_TASK	9

#else

#define OS_PRIO_WERS_RF_BRIDGE_TASK			17
#define OS_PRIO_WERS_RF_RECEIVER_TASK		18
#define OS_PRIO_WERS_485_RECEIVE_TASK		20
//#define OS_PRIO_WERS_RF_RECEIVER_TASK		20
#define OS_PRIO_WERS_485_SEND_TASK			21
#define OS_PRIO_WERS_485_BRIDGE_TASK		22

//#define OS_PRIO_WERS_485_RECEIVE_TASK		23
#define OS_PRIO_WERS_485_POLL_TASK			24
#define OS_PRIO_WERS_485_SEND_CHECK_TASK	25

#endif

#endif
#define TRY_COUNT_FOR_485_SEND	20
#define BACK_OFF_TIME_FOR_485_COMUNICATION				3000	//ms

#if 0
#define MAC_QUE_SIZE			64
#define WERS_QUE_SIZE			100//512
#define WERS_485_OUT_QUE_SIZE			128
#define WERS_485_IN_QUE_SIZE			128
#define WERS_485_LISTEN_QUE_SIZE			32
#define WERS_RTS_QUE_SIZE		256
#else
#define MAC_QUE_SIZE			32
#define WERS_QUE_SIZE			32
#define WERS_485_OUT_QUE_SIZE			32
#define WERS_485_IN_QUE_SIZE			32
#define WERS_485_LISTEN_QUE_SIZE			32
#define WERS_RTS_QUE_SIZE		32
#endif

#define WERS_MSG_TIMER_0		0x10
#define WERS_MSG_TIMER_1		0x11

#define WERS_TO_PC_FRAME_SIZE		64
#define WERS_TRANSACTION_IN_FRAME_Q_SIZE 32
#define WERS_IN_FRAME_Q_SIZE		64
#define WERS_FREE					0
#define WERS_HOLD					1
#define WERS_RX_QUE_SIZE			32/*no use 2048*/
#define WERS_RX_QUE1_SIZE			2048
#define WERS_RX_QUE2_SIZE			2048

#ifdef WERS_485_AP_HOST
#define NUM_GUID_FOR_PC_EMENU 		64
#endif

#define WERS_TRANSCATION_QUE_SIZE  200

#ifndef WERS_NEWVER_TRANSACTION_SUPPORT
#define WERS_TRANSACTION_GUID_QUEQ_SIZE 64
#else
#define WERS_TRANSACTION_GUID_QUEQ_SIZE 16
#endif

#define STACKSIZE_WERS_TRASMIT 		(256)
#define STACKSIZE_485_RECEVIE_TASK	(256)
#define STACKSIZE_485_SEND_TASK		(256)
#define STACKSIZE_485_BRIDGE_TASK	(256)
#define STACKSIZE_485_SEND_CHECK	(256)
#define STACKSIZE_485_POLL_TASK		(256)
//#define STACKSIZE_RF_RECEIVER		(128)
//#define STACKSIZE_RF_RECEIVER		(192)
#define STACKSIZE_RF_RECEIVER		(256)

#ifdef WERS_EXTENDER_DEVICE
#define STACKSIZE_RF_BRDIGE_TASK	(256)
#endif

//Multiplicative Increase Line Decrease
#define COUNTER_MAX		100
#define COUNTER_MIN		2
#define A_VALUE			2
#define B_VALUE			6

#define L_485_RTS_PLAYLOAD_SIZE	10

#define FRAME_TYPE_485_DATA					0x0
#define FRAME_TYPE_485_COMMAND_RTS			0x1
#define FRAME_TYPE_485_COMMAND_CTS			0x2
#define FRAME_TYPE_485_COMMAND_DATA_ACK		0x3
#define FRAME_TYPE_485_COMMAND_BTS			0x4
#define FRAME_TYPE_485_EVENT				0x5

#define FRAME_TYPE_485_REQUST				0x6
#define FRAME_TYPE_485_NO_DATA				0x7

#define ACTION_RECEIVE_FROM_PC	0x01
#define ACTION_SEND_TO_PC		0x02

#define MAX_TIME_OUT_TO_DROP_PACKATE	(60000)


/*       ****   frame information objects
 * info kept on each frame object
 */
#define   WERS_FI_AVAILABLE         0   /* entry available for use */
#define   WERS_FI_INUSE_UNTIL_DEL   1   /* in use. will be explicitly reclaimed */
#define   WERS_FI_INUSE_TRANSACTION  4   /* being retrieved. do not delete in Rx ISR thread. */

#define WERS_TRANSCATION_MAX_GET_COUNT 	4

#define TRANSCATION_TIME_OUT	(60000)	// 60s

/******************************************************************************
 * TYPEDEFS
 */
enum E_WERS_485_STATUS
{
	E_485_SENDING = 0xA,
	E_485_RECEIVE, 
	E_485_IDLE,
	E_485_REQUEST,
};

enum E_WERS_RF_NETWORK_STATUS
{
	RF_OTHER_NODE_SENDING = 0x8,
	RF_SENDING, 
	RF_NETWORK_IDLE,
};

enum E_WERS_DEVICE_RF_STATUS
{
	DEVICE_RF_SENDING = 0x2,
	DEVICE_RF_RECEIVE, 
};

struct DEV_ADDR{
	uint8_t hotel;
	uint8_t host;
	uint16_t dev;
};

typedef __packed struct WERSMsg
{
	UINT8	code;
	UINT8	datasize;
	UINT8 	data[64];
}WERSMsg_t;


typedef __packed struct WERSRTS
{
	UINT8	cmd;
	UINT8	no_use;
	UINT8	linkID;
	UINT8	opCode;
	UINT32 	GUID;
	UINT32 	dataSize;
	UINT32	srcAddress;
	UINT32	dstAddress;	
}WERSRTSPlayload;

typedef __packed struct WERSData_Ack
{
	UINT8	cmd;
	UINT32	dstAddress;	
	UINT32	GUID;
}WERSDataAck;


typedef __packed struct WERSFramInfo
{
 	volatile UINT8   fi_usage;
	volatile SINT8   getcount;
       volatile UINT16 orderStamp;
	volatile UINT32 packetSize;
	volatile UINT8 *pPacketData;
}WERSFramInfo_t;

typedef __packed struct WERSRFNotify_Info
{
	UINT8	cmd;
	UINT8	playloadSize;
	UINT8	playload[128];
}WERSRFNotifInfo;

#ifndef WERS_NEWVER_TRANSACTION_SUPPORT
typedef __packed struct WERSTranscationGUID
{
	UINT8 linkID;
	UINT8 operationCode;
	UINT8 fi_usage;
	//UINT8 transID;
	UINT32 GUID;
}WERSTranscationGUID_t;

typedef __packed struct WERSTransaction
{
	UINT8 number;
	WERSTransactionGUID_t transGUIDQue[WERS_TRANSACTION_GUID_QUEQ_SIZE];
}WERSTransaction_t;
#else

#define WERS_TRANSACTION_TIMEOUT	1000
typedef __packed struct WERSTransactionGUID
{
	struct WERSTransactionGUID *next;
	UINT32 GUID;
	UINT32 TimeStamp;
	UINT8 linkID;
	UINT8 operationCode;
}WERSTransactionGUID_t;

typedef __packed struct WERSTransaction
{
	UINT32 number;
	//UINT8 reserved[3];
	WERSTransactionGUID_t *Unused,*Using,*UsingTail;
	WERSTransactionGUID_t transGUIDQue[WERS_TRANSACTION_GUID_QUEQ_SIZE];	
}WERSTransaction_t;

#endif

typedef __packed struct WERS485_header{

	/*
	This field encodes as an unsigned integer the number
	of bytes in this container. WERS sysem uses this field
	to determine the size of the container*/
	
	UINT16 ContainerLength;

	/*CRC filed*/
	UINT16 DataCRC;
	
	/*
	0 Data Block
	1 RTS
	2 CTS
	3 ACK
	*/
	UINT8  ContainerType;

	//UINT8  HeaderCRC;
}WERS485FrameHeader;


typedef __packed struct WERS485_Data_ack_header{
	/*
	This field encodes as an unsigned integer the number
	of bytes in this container. WERS sysem uses this field
	to determine the size of the container*/
	
	UINT16 ContainerLength;

	/*CRC filed*/
	UINT16 DataCRC;
	
	/*
	0 Data Block
	1 RTS
	2 CTS
	3 ACK
	*/
	UINT8  ContainerType;
	
	UINT8  HeaderCRC;
	
	UINT32 GUID;
}WERS485DataAck;


typedef __packed struct WERS485_RTS{

	/* link ID*/	
	UINT8 linkID;
	
	/*Opode*/
	UINT8 code;	
	
	UINT32 GUID;
	
	UINT32 dataSize;	
}WERS485RTSPlayload;



typedef __packed struct WERS485_Signal_Header{
	UINT32	Cmd;
	UINT32	Datasize;
}WERS485SigHeader;

typedef struct WERS485_RAW_s{
	uint8_t *pbuf;
	uint32_t DataCRC;
}WERS485_RAW_t;

/******************************************************************************
 * LOCAL VARIABLES
 */
#ifndef WERS_USING_NETWORK
//static void* WERSMacQueBuf[MAC_QUE_SIZE];
static void* WERSQueBuf[WERS_QUE_SIZE];
static void* WERSQue = NULL;
#endif
#ifdef WERS_EXTENDER_DEVICE
static void* TransactionQue[WERS_TRANSCATION_QUE_SIZE];
#endif

#ifndef WERS_DEBUG
#ifndef WERS_USING_NETWORK
//static void* g_485OutQueBuf[WERS_485_OUT_QUE_SIZE];
static void* g_485InQueBuf[WERS_485_IN_QUE_SIZE];

#ifndef WERS_EXTENDER_DEVICE
#if (defined WERS_485_PC_HOST )
static void* g_485ListenQueBuf[WERS_485_LISTEN_QUE_SIZE];
#endif
#endif

static void* g_485OutQue = NULL;
#ifdef WERS_EXTENDER_DEVICE
static UINT8 WERSTime0flag = 0;
static void*g_pTranscationQue = NULL;
static UINT8  gs_linkID = 0;
#endif
static void* g_485InQue = NULL;
#if (defined WERS_485_PC_HOST )
static void* g_485ListenQue = NULL;
#endif
//static void* g_pMacQue = NULL;
#endif	/*WERS_USING_NETWORK*/

#endif	//WERS_DEBUG
static void* RTSQueBuf[WERS_RTS_QUE_SIZE];
static void* g_RTSQue = NULL;


//static volatile uint8_t sPeerFrameSem;
static volatile uint32_t sPeerFrameSem = 0;
static volatile uint8_t sJoinSem;
static volatile uint32_t receivepackeNumber = 0;

// reserve space for the maximum possible peer Link IDs
#if NUM_CONNECTIONS > 0
static linkID_t sLID[NUM_CONNECTIONS];
#else
static linkID_t sLID[1];
#endif
static uint8_t  sNumCurrentPeers;
//static uint8_t gsTransID = 0;
static UINT32 sBegintime = 0;
static UINT32 sBackOffCounter = 0;
//static UINT32 recivepackagecount = 0;
static volatile UINT32  g_485CurrentStates = E_485_IDLE;
static volatile UINT32  g_NetworkCurrentStates = RF_NETWORK_IDLE;
static volatile UINT32  g_DeviceStatus = DEVICE_RF_RECEIVE;
static UINT32 timemsStart = 0;
static UINT32 datemsStart = 0;





/******************************************************************************
 * LOCAL FUNCTIONS
 */
static void WERSExWorking(void);
static void processMessage(linkID_t lid, uint8_t *msg, uint8_t len);
static BOOL CheckTimeOut(UINT32 timelimit);
static uint8_t sCB(linkID_t lid);
static BOOL BackOffTimeOut(void);
#ifndef WERS_USING_NETWORK
static BOOL F_485_BackOffTimeOut(void);
static UINT32 WERS_processReSendRequest(UINT8 code,void *pVal,UINT32 Valsize);
#endif
static UINT32 WERS_RTS_CTS_Notify(UINT8 code,void *pVal,UINT32 Valsize);
static BOOL BackOffTimeOutEx(UINT32 Endtimelimit);
static void WERSRFBridgeTask(void *arg);

/******************************************************************************
 * GLOBAL VARIABLES
 */ 
extern uint8_t g_MyNodeType;
extern volatile uint32_t g_intr_idle;
extern int8_t g_mrfi_rssi;
extern uint16_t sBackoffHelper;
extern UINT32  g_CRCfailedPacketNumber;
extern UINT32  g_MismatchLengthNumber;
extern UINT32 g_ReceiveNotEmptyPacketNumber;

//extern UINT32 g_WatchDogTimerID;
//int32_t g_WatchDogTimerID;
int g_timerID;
void SystemReset(void);


#ifndef WERS_USING_NETWORK
WERSTransaction_t g_st485GetTransaction;
#endif
WERSTransaction_t g_stWERSGetTransaction;
WERSFramInfo_t sWERSInFrameQ[WERS_IN_FRAME_Q_SIZE];
//#ifdef WERS_EXTENDER_DEVICE
volatile UINT8 gs_DeviceState =  DEVICE_INIT;
//#endif
WERSFramInfo_t sWERSTransactionInQ[WERS_TRANSACTION_IN_FRAME_Q_SIZE];
UINT32 gRFReceiverStack[STACKSIZE_RF_RECEIVER];
#ifndef WERS_DEBUG
#ifndef WERS_USING_NETWORK
UINT32 g485SendTaskStack[STACKSIZE_485_SEND_TASK];
UINT32 g485BridgeTaskStack[STACKSIZE_485_BRIDGE_TASK];
#ifdef WERS_EXTENDER_DEVICE
UINT32 gRFBridgeTaskStack[STACKSIZE_RF_BRDIGE_TASK];
volatile uint32_t g_NeedReset  = 0;
#endif
UINT32 g485RecevieTaskStack[STACKSIZE_485_RECEVIE_TASK];
UINT32 g485SendCheckStack[STACKSIZE_485_SEND_CHECK];
UINT32 g485PollTaskStack[STACKSIZE_485_POLL_TASK];
#endif	/*WERS_USING_NETWORK*/
#endif	/*WERS_DEBUG*/
OS_STK WERSEDBgroundTaskStk[APP_TASK_WERSEDBground_STK_SIZE];

#ifdef ENABLE_485_DMA_TRANSMIT
void *g_p485SendCheckSem = NULL;
#endif

//void *g_pSemReceive = NULL;
void *g_pSemCTS = NULL;
//void *g_pRFSem = NULL;
#ifndef WERS_DEBUG
//void *g_p485SemReceive = NULL;
void *g_p485PollSem = NULL;
#endif

#if (defined WERS_485_AP_HOST)	
void *g_pSem485Bridge = NULL;
void *g_p485DataAckSem = NULL;
void *g_pDataRecevieOkSem = NULL;
void *g_pDataComingSem = NULL;


volatile uint32_t g_485AckGUID = 0;
#ifndef WERS_USING_NETWORK
static UINT32 sGUIDMaps[NUM_GUID_FOR_PC_EMENU];
#endif
static UINT32  sNumGUIDforPCEmenu = 0;
#endif

UINT16 g_WERSSystemId = 1;
volatile uint32_t g_NotifyCode = 0;
volatile uint32_t g_DataAckGUID = 0;
volatile uint32_t g_485NotifyCode = 0;

volatile uint32_t g_BackOffBegintime = 0;

volatile uint32_t g_485BackOffBegintime = 0;

volatile uint32_t g_RFBackOfftime = 0;

volatile uint32_t g_getCTS = 0;

volatile uint32_t	g_ApInitOk  = 0;
volatile uint32_t	g_ApTaskCondition  = 0;


/* array of function pointers to handle NWK application frames */
UINT32 (* const callBackfunc[])(UINT8 code,void *pVal,UINT32 Valsize) = {
//#ifndef WERS_EXTENDER_DEVICE
#ifndef WERS_USING_NETWORK
	WERS_processReSendRequest,
#else
	NULL, 
#endif
	WERS_RTS_CTS_Notify,
};


// crc8
const unsigned char crc8_ta[] = {
	0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
	157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
	35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
	190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
	70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
	219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
	101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
	248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
	140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
	17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
	175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
	50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
	202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
	87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
	233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
	116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
};



/******************************************************************************
 * GLOBAL FUNCTIONS
 */
void WERSSetSystemID(UINT16 sysID);
UINT16 WERSGetSystemID(void);
void WERSSetDeviceAddress(WERSAddr_t *paddr);
UINT32 SMPL_SendEx(UINT8 lid, UINT8 *msg, UINT32 len);
void WERSTranscationProcess(void);
UINT32 WERSGetResponseForPC(UINT8 lid,UINT16 systemID,UINT8 **pdata, UINT8 *pdataSize);
UINT32 WERSSendDataAck(UINT8 linkID,UINT32 GUID,UINT32 trycount);

#ifdef WERS_NEWVER_TRANSACTION_SUPPORT
void InitTransactionGUID(WERSTransaction_t *pWERSTransaction);
UINT32 PendTransactionToQue(WERSTransaction_t *pWERSTransaction, UINT8 linkID, UINT8 operationCode, UINT32 GUID);
#endif

UINT32 AddNewTransactionToQue(WERSTransaction_t *pHistoryQue,UINT8 linkID, UINT8 operationCode, UINT32 GUID);
UINT32 SearchTransactionGUID(WERSTransaction_t *pHistoryQue,UINT8 linkID, UINT8 operationCode, UINT32 GUID);

UINT32 WERSSendResponseForAP(WERSContainerStruct *pHeader,UINT8 *pdata, UINT16 datalen);
UINT32 WERS_InsertTransactionToResponseQueue(UINT8 *pData,UINT32 datasize,UINT8 linkID, UINT8 operationCode, UINT8 transID);
UINT32 WERS_InsertTransactionToCmdQueue(UINT8 *pData,UINT32 datasize,UINT8 linkID, UINT8 operationCode, UINT8 transID);
UINT32 WERSGetDataFromPC(UINT8 **pData,UINT16 *dataLen);
UINT32 WERSSendRTS(UINT8 linkID,UINT32 datalen,UINT8 opCode, UINT32 GUID,UINT32 *pOutCode);
UINT32 RF_SendRTS(UINT8 linkID,UINT8 opCode, UINT32 GUID,UINT32 datasize);
WERSFramInfo_t* WERS_GetReceivedTranscationFromQ(void);
UINT32 SendDataToPC(UINT8 *pData, UINT32 len);
void WERS_QInit(void);
void WERS_QadjustOrder(uint16_t stamp);
void WERS_QadjustOrder(uint16_t stamp);
void WERS_RemoveTranscationFromQByGUID(UINT32 GUID);
void WERS_SaveReceivedTranscationInQ(UINT8 *pData,UINT32 ByteNumber,SINT8 getcount);
WERSFramInfo_t *WERS_QfindSlot(void);
WERSFramInfo_t *WERS_QfindOldest(void);
WERSFramInfo_t *WERS_QfindByGUID(UINT32 GUID);
WERSFramInfo_t * WERS_GetOldestTranscation(void);
unsigned char cal_crc8(const unsigned char * ptr, int len);
unsigned short cal_crc(const unsigned char * ptr, int len);
void WERSAPRestartNotifyPC(void);
UINT32 WERSGenerate_RF_GUID(void);
UINT32 WERSGenerate_485_GUID(void);
UINT32 WERSMsTimeSet(UINT32 timevalue);
UINT32 WERSMsTimeGet(void);
UINT32 WERSResponseCTS(WERSRTSPlayload *pRTSPlayload);
void WERS_RemoveTranscationFromQ(WERSFramInfo_t *pTranscation);
void WERS_ClearTranscationQueu(void);
void WERSPCDataProcess(void *pData,UINT16 dataLen);
#ifndef WERS_DEBUG
#ifndef WERS_USING_NETWORK
void WERS_RemoveTimeOutGUIDFromMap(UINT32 timeOut);
void WERS485SendCheckTask(void);
void WERS485SendTask(void *arg);
void WERS485RecevieTask(void *arg);
UINT32 WERS485Send(UINT8 DataType,UINT8 *pdata, UINT16 datalen);
void WERS485BridgeTask(void *arg);
void WERS485Poll(void *arg);
extern UINT32 hwWait(UINT32 tmrUnit,UINT32 waitCnt);
//void DMA_485_Data_Coming_Int_Handler(void);
#else	/*WERS_USING_NETWORK*/
void WERS_TCPRecevieTask(void *arg);
#endif
#endif



extern void nwk_ResetSet( void (*pReset)(void) );
extern void MRFI_RxOn(void);
extern void ResetReceive(void);
//extern void SPMP_TurnLED(UINT8 led, UINT8 flag);
extern uint8_t MRFI_RandomByte(void);
#ifndef WERS_DEBUG

#ifdef WERS_EXTENDER_DEVICE
#define SPMP_Config485ToSend()
#define SPMP_Config485ToReceive()
#else
extern void SPMP_Config485ToSend(void);
extern void SPMP_Config485ToReceive(void);
extern UINT32 SPMP_Get485LanStatus(void);
#endif

#endif
extern UINT32 WERS_UartDrv_Tx(UINT8 *buf, UINT32 num_to_send, UINT32 *numSend);


/******************************************************************************
 * @fn          BackOffTimeOut
 *
 * @brief
 *
 * input parameters
 *
 * output parameters
 *
 * @return
 */
static BOOL BackOffTimeOut()
{
	UINT32 status;
	
	ENTER_CRITICAL(status);
	if(tmrMsTimeGet() > g_BackOffBegintime)
	{	
		ClearBackOffTimeOut();
		EXIT_CRITICAL(status);
		
		return TRUE;
	}
	else
	{
		EXIT_CRITICAL(status);
#ifdef WERS_EXTENDER_DEVICE
		rf_hwMsWait(3);
#endif
	}

	return FALSE;
}

static BOOL BackOffTimeOutEx(UINT32 Endtimelimit )
{
	UINT32 status;
	UINT32 currentMsTime;
	currentMsTime = tmrMsTimeGet();
	
	ENTER_CRITICAL(status);
	if( currentMsTime > g_BackOffBegintime || currentMsTime > Endtimelimit )
	{	
		ClearBackOffTimeOut();
		EXIT_CRITICAL(status);
		
		return TRUE;
	}
	else
	{
		EXIT_CRITICAL(status);
		
		rf_hwMsWait(3);
	}

	return FALSE;
}

#ifndef WERS_USING_NETWORK
static BOOL F_485_BackOffTimeOut()
{
	UINT32 status;
	
	ENTER_CRITICAL(status);	
	if( tmrMsTimeGet() > g_485BackOffBegintime)
	{	
		Clear_485_BackOffTimeOut();
		EXIT_CRITICAL(status);
		
		return TRUE;		
	}
	else
	{
		EXIT_CRITICAL(status);
		
		rf_hwMsWait(3);
	}
	
	return FALSE;
}
#endif

static BOOL CheckTimeOut(UINT32 timelimit)
{
	if(sBegintime == 0)
	{
		sBegintime = tmrMsTimeGet() + timelimit;
	}
	else if(tmrMsTimeGet() > sBegintime)
	{	
		ClearTimeOut();
		//sBegintime = 0;
		return TRUE;
	}
	else
	{
		//hwWait(1, 5); // wait 200 us
		#if 0
		UINT32 i = 0;
		do{
			i++;
		}while(i < 6000);
		#else
		
		rf_hwMsWait(5);
		#endif
	}
	
	return FALSE;
}

static BOOL CheckTimeOutEx(UINT32 timelimit,UINT32 waitms)
{
	if(sBegintime == 0)
	{
		sBegintime = tmrMsTimeGet() + timelimit;
	}
	else if(tmrMsTimeGet() > sBegintime)
	{	
		ClearTimeOut();
		//sBegintime = 0;
		return TRUE;
	}
	else
	{
		UINT32 i;
		for( i = 0; i < waitms; i++)
		{
			//for watch dog, can't remove
			g_ApTaskCondition = 0;
			rf_hwMsWait(1);
		}
	}
	
	return FALSE;
}



UINT32 WERSMsTimeSet(UINT32 timevalue)
{	
	UINT32 err = RF_SUCCESS;
	datemsStart = timevalue;
	timemsStart = tmrMsTimeGet();
	return err;
}


UINT32 WERSMsTimeGet(void)
{
	UINT32 interval;
	interval = (tmrMsTimeGet() - timemsStart);
	return (datemsStart + interval);
}


void WERSSetSystemID(UINT16 sysID)
{
	g_WERSSystemId = sysID;
}
UINT16 WERSGetSystemID(void)
{
	return g_WERSSystemId;
}

void WERSGetDeviceAddress(WERSAddr_t *paddr)
{
	if( paddr != NULL )
	{
		SMPL_Ioctl(IOCTL_OBJ_ADDR, IOCTL_ACT_GET, paddr);
	}
}

void WERSSetDeviceAddress(WERSAddr_t *paddr)
{
	if( paddr != NULL )
	{
		SMPL_Ioctl(IOCTL_OBJ_ADDR, IOCTL_ACT_SET, paddr);
	}
}

/**************************************************************************************************
 * @fn          WERS_RandomDelay
 *
 * @brief       -
 *
 *    tmrUnit : 0 : 1ms                                                   *
 *              1 : 100us                                                 *
 *		   2:  1us                                                     *
 *
 * @return      none
 **************************************************************************************************
 */
void WERS_RandomDelay(UINT32 tmrUnit)
{
  uint8_t backoffs;
//  uint8_t i;

  /* calculate random value for backoffs - 1 to 16 */
  backoffs = (MRFI_RandomByte() & 0x0F) + 1;

  /* delay for randomly computed number of backoff periods */
  //for (i=0; i<backoffs; i++)
  {
    	//Mrfi_DelayUsec( MRFI_BACKOFF_PERIOD_USECS );
    	if ( tmrUnit == 1 )
    	{
		rf_hwUsWait(3*backoffs*100);
    	}
	else if ( tmrUnit == 2 )
	{
		rf_hwUsWait(3*backoffs);
	}
	else
	{
    	rf_hwMsWait(3*backoffs);
	}
	//hwWait(tmrUnit, 3*backoffs);
  }
}

static UINT32 WERS_RTS_CTS_Notify(UINT8 code,void *pVal,UINT32 Valsize)
{
	UINT32 ret = 0;
	//bspIState_t intState = 0;
	
	switch(code)
	{
		case MGMT_CMD_OTHER_NODE_RTS:
			//rf_printf_0("\n OTHER_RTS  !\n");
			SetBackOffTimeOut(tmrMsTimeGet() + 15); // backoff 10 ms
			break;
		case MGMT_CMD_OTHER_NODE_CTS:
		{
			UINT32 datalen = *((UINT32*)pVal);			
			UINT32 delaytime = Calculate_RF_BackOff_Time(datalen,200);

			//rf_printf_0("\n OTHER_NODE_CTS ...( %d ) ..   : %d,  %d, %d\n",code,datalen,g_BackOffBegintime,delaytime);
			delaytime = (delaytime < 1500 ) ? delaytime : 1500;
			//rf_printf_0("\n OTHER_CTS  %d!\n",delaytime);
			
			SetBackOffTimeOut(tmrMsTimeGet() + delaytime);
			break;
		}
		case MGMT_CMD_DS:
		{
			UINT32 datalen = *((UINT32*)pVal);	
			UINT32 delaytime =  Calculate_RF_BackOff_Time(datalen,200);

			//rf_printf_0(" ===MGMT_CMD_DS (datasize: %d   delaytime:%d ms)\n",datalen,delaytime);
			
			delaytime = (delaytime < 1500 ) ? delaytime : 1500;
			
			SetBackOffTimeOut(tmrMsTimeGet() + delaytime); //max backoff 1500ms	
			break;
		}		
		case MGMT_CMD_RTS:
		{
			rf_printf_1(" ===MGMT_CMD_RTS %d \n",Valsize);
			if ( pVal && Valsize )
			{	
				WERSRTSPlayload *pRTSPlayload= rf_alloc(sizeof(WERSRTSPlayload));
				if ( pRTSPlayload )
				{
					memcpy(pRTSPlayload,pVal,sizeof(WERSRTSPlayload));
					if(osQuePost(g_RTSQue, pRTSPlayload) == OS_Q_FULL)
						rf_free(pRTSPlayload);
				}
			}
			break;
		}		
		case MGMT_CMD_CTS:	
			rf_printf_0(" ===MGMT_CMD_CTS \n");
			g_NotifyCode = MGMT_CMD_CTS;
			//osQuePost(g_pMacQue, (void*)MGMT_CMD_CTS);
			break;
		case MGMT_CMD_OTHER_DATA_ACK:
			rf_printf_0("Other Data Ack!\n");
			ClearBackOffTimeOut();
			break;			
		case MGMT_CMD_DATA_ACK:
			rf_printf_0(" ===MGMT_CMD_Data_Ack! \n");
			if ( pVal && Valsize )
			{
				WERSDataAck *pDataAck = (WERSDataAck*)pVal;
				g_DataAckGUID = pDataAck->GUID;
			}
			else
			{
				g_DataAckGUID = 0;
			}
						
			ClearBackOffTimeOut();
			g_NotifyCode = MGMT_CMD_DATA_ACK;
			//osQuePost(g_pMacQue, (void*)MGMT_CMD_DATA_ACK);
			break;
		default:
			break;
	}
	
	return ret;
}

UINT32 MILD_min() 
{
	UINT32 tempv = sBackOffCounter *A_VALUE;
	tempv -= sBackOffCounter/2;
	sBackOffCounter = (tempv > COUNTER_MAX)?COUNTER_MAX:tempv;
	return sBackOffCounter;
}

UINT32 MILD_max()
{
	sBackOffCounter = (sBackOffCounter >= B_VALUE)?COUNTER_MIN:(sBackOffCounter  - B_VALUE);
	return sBackOffCounter;
}

UINT32 RF_SendRTS(UINT8 linkID,UINT8 opCode, UINT32 GUID,UINT32 datasize)
{
	ioctlRawSend_t    send;
	smplStatus_t ret = SMPL_BAD_PARAM;
	WERSRTSPlayload sRTS;
	uint8_t  radioState =0;

	
	connInfo_t * pCInfo = nwk_getConnInfo(linkID);
	if ( pCInfo )
	{
		sRTS.cmd = MGMT_CMD_RTS;
		sRTS.no_use = 0;
		sRTS.linkID = linkID;
		sRTS.opCode= opCode;
		sRTS.GUID = GUID;
		sRTS.dataSize = datasize;
		
		// pack dst address
		memcpy((UINT8*)&(sRTS.dstAddress),(UINT8*)pCInfo->peerAddr,sizeof(addr_t));

		// pack src address
		memcpy((UINT8*)&(sRTS.srcAddress),(UINT8*)nwk_getMyAddress(),sizeof(addr_t));

		send.addr = (addr_t *)nwk_getBCastAddress();
		send.msg  = (uint8_t*)&sRTS;
		send.len  = sizeof(sRTS);
		send.port = SMPL_PORT_MGMT;


		ret = SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_WRITE, &send);

		radioState = MRFI_GetRadioState();
		NWK_CHECK_FOR_SETRX();
	}

	return ret;
}

UINT32 WERSResponseCTS(WERSRTSPlayload *pRTSPlayload)
{
	uint8_t  msg[MGMT_CTS_FRAME_SIZE];
	ioctlRawSend_t send;  
	UINT32 ret = SMPL_SUCCESS;

	if (pRTSPlayload)
	{
		msg[M_CTS_CMD_OS] = MGMT_CMD_CTS;

		//pack dst address
		memcpy( (msg+M_CTS_DST_ADDRESS_OS),((const void *)(&pRTSPlayload->srcAddress)),sizeof(addr_t));

		//pack data len
		memcpy((msg + M_CTS_DATA_LEN_OS),((const void *)(&pRTSPlayload->dataSize)),sizeof(UINT32));

		send.addr = (addr_t *)nwk_getBCastAddress();
		send.msg  = msg;
		send.len  = sizeof(msg);
		send.port = SMPL_PORT_MGMT;

		ret = SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_WRITE, &send);

		rf_printf_1("send CTS!!\n");
	}

	return ret;
}

void RestNotifyCode()
{
	bspIState_t intState;
	BSP_ENTER_CRITICAL_SECTION(intState);
	g_NotifyCode = 0;
	BSP_EXIT_CRITICAL_SECTION(intState);
}

#ifndef WERS_NEWVER_TRANSACTION_SUPPORT
UINT32 SearchTransactionGUID(WERSTranscation_t *pHistoryQue,UINT8 linkID, UINT8 operationCode, UINT32 GUID)
{
	WERSTranscationGUID_t *pTransGUID = NULL;
	UINT8 *pTransGUIDNumber;

	rf_printf_1("==Serarch: linkID:%d  opcode: %d  GUID: %d\n",linkID,operationCode,GUID);

	if ( NULL != pHistoryQue )
	{
		//pTransGUID = g_stWERSGetTranscation.transGUIDQue;
		//pTransGUIDNumber = &g_stWERSGetTranscation.number;
		pTransGUID = pHistoryQue->transGUIDQue;	
		pTransGUIDNumber = &pHistoryQue->number;
		
		if ( pTransGUID  )
		{
			UINT32 i;
			for ( i = 0; i < *pTransGUIDNumber; i++)
			{
				if ( pTransGUID->linkID == linkID
				&& pTransGUID->operationCode == operationCode
				&& pTransGUID->GUID == GUID )
				{
					return (UINT32)pTransGUID;
				}
				pTransGUID++;
			}
		}
	}
	
	return NULL;
}

UINT32 RemoveTransactionGUIDBylinkID(WERSTranscation_t *pHistoryQue,UINT8 linkID)
{
	WERSTranscationGUID_t *pTransGUID = NULL;
	UINT8 *pTransGUIDNumber;

	if ( NULL != pHistoryQue )
	{
		//pTransGUID = g_stWERSGetTranscation.transGUIDQue;	
		//pTransGUIDNumber = &g_stWERSGetTranscation.number;

		pTransGUID = pHistoryQue->transGUIDQue;
		pTransGUIDNumber = &pHistoryQue->number;
		
		if ( pTransGUID  )
		{
			UINT32 i,j;
			UINT8 number = *pTransGUIDNumber;
			
			for ( i = 0; i < number; i++)
			{
				if ( pTransGUID->linkID == linkID )
				{
					//memset(pTransGUID,0,sizeof(WERSTranscationGUID_t));
					pTransGUID->fi_usage = WERS_FREE;
				}
				pTransGUID++;
			}


			//pTransGUID = g_stWERSGetTranscation.transGUIDQue;
			pTransGUID = pHistoryQue->transGUIDQue;
			for ( i = 0; i < number; i++)
			{
				if ( WERS_FREE == pTransGUID[i].fi_usage )
				{
					for ( j= i+1; j < number; j++)
					{
						if ( WERS_HOLD == pTransGUID[j].fi_usage )
						{
							pTransGUID[i].fi_usage = pTransGUID[j].fi_usage;
							pTransGUID[i].linkID = pTransGUID[j].linkID;
							pTransGUID[i].operationCode = pTransGUID[j].operationCode;
							pTransGUID[i].GUID = pTransGUID[j].GUID;
							
							pTransGUID[j].fi_usage = WERS_FREE;
							
							*pTransGUIDNumber = *pTransGUIDNumber -1;
							break;
						}
					}
				}
			}
		}
	}
	return NULL;
}

UINT32 AddNewTransactionToQue(WERSTranscation_t *pHistoryQue,UINT8 linkID, UINT8 operationCode, UINT32 GUID)
{
	WERSTranscationGUID_t *pTransGUID = NULL;
	UINT8 *pTransGUIDNumber;
	UINT32 ret = SMPL_BAD_PARAM;
	UINT32 status;
	

	if ( NULL != pHistoryQue )
	{
		//pTransGUID = g_stWERSGetTranscation.transGUIDQue;	
		//pTransGUIDNumber = &g_stWERSGetTranscation.number;
		pTransGUID = pHistoryQue->transGUIDQue;	
		pTransGUIDNumber = &pHistoryQue->number;
		
		if ( pTransGUID  )
		{
			ENTER_CRITICAL(status);

			if( *pTransGUIDNumber  >= WERS_TRANSACTION_GUID_QUEQ_SIZE )
			{
				*pTransGUIDNumber = *pTransGUIDNumber -1;
				memcpy(pTransGUID,&pTransGUID[1],sizeof(WERSTranscationGUID_t) * (*pTransGUIDNumber));
			}

			// add new transcation GUID
			pTransGUID +=*pTransGUIDNumber;
			
			pTransGUID->linkID = linkID;
			pTransGUID->operationCode = operationCode;
			pTransGUID->GUID = GUID;
			pTransGUID->fi_usage = WERS_HOLD;
			
			*pTransGUIDNumber = *pTransGUIDNumber + 1;
			EXIT_CRITICAL(status);	

			ret = SMPL_SUCCESS;
		}
	}
	rf_printf_0("===AddNewTranscationToQue====(linkID:%d, Op:%d, trID:%d)\n",linkID,operationCode,GUID);
	return ret;
}
#else	//WERS_NEWVER_TRANSACTION_SUPPORT

void InitTransactionGUID(WERSTransaction_t *pWERSTransaction)
{
	int i;
	WERSTransactionGUID_t *pTransGUID;
	memset(pWERSTransaction,0,sizeof(WERSTransaction_t));
	pTransGUID = pWERSTransaction->transGUIDQue;
	pWERSTransaction->Unused =  pTransGUID;
	for(i = 0; i < WERS_TRANSACTION_GUID_QUEQ_SIZE-1; i++)
	{
		pTransGUID[i].next = &pTransGUID[i+1];
		//pTransGUID ++;
	}
	pTransGUID[i].next = NULL;
	//pTransGUID->next = NULL;
}

UINT32 SearchTransactionGUID(WERSTransaction_t *pWERSTransaction, UINT8 linkID, UINT8 operationCode, UINT32 GUID)
{
	WERSTransactionGUID_t *pTransGUID;
//	UINT32 ret = SMPL_BAD_PARAM;
	bspIState_t intState;

	BSP_ENTER_CRITICAL_SECTION(intState);
	for(pTransGUID = pWERSTransaction->Using; pTransGUID; pTransGUID = pTransGUID->next)
	{
		if ( pTransGUID->linkID == linkID 
			&& pTransGUID->operationCode == operationCode 
			&& pTransGUID->GUID == GUID )
		{
			BSP_EXIT_CRITICAL_SECTION(intState);
			return (UINT32)pTransGUID;
		}
	}

	BSP_EXIT_CRITICAL_SECTION(intState);
	//return NULL;
	return 0;
}


UINT32 AddNewTransactionToQue(WERSTransaction_t *pWERSTransaction, UINT8 linkID, UINT8 operationCode, UINT32 GUID)
{
	//static UINT32 dbgGUID = 0;
	WERSTransactionGUID_t *pTransGUID;
	UINT32 ret = SMPL_BAD_PARAM;
	bspIState_t intState;

	BSP_ENTER_CRITICAL_SECTION(intState);
	pTransGUID = pWERSTransaction->Unused;
	if(pTransGUID)
	{
		pWERSTransaction->Unused = pTransGUID->next;
		pTransGUID->linkID = linkID;
		pTransGUID->operationCode = operationCode;
		pTransGUID->GUID = GUID;
		pTransGUID->TimeStamp = tmrMsTimeGet();
		pTransGUID->next = NULL;
		if(pWERSTransaction->UsingTail == NULL)
		{
			pWERSTransaction->UsingTail = pTransGUID;
			pWERSTransaction->Using = pTransGUID;
		}
		else
		{
			pWERSTransaction->UsingTail->next = pTransGUID;
			pWERSTransaction->UsingTail = pTransGUID;
		}
		if(pWERSTransaction->number < WERS_TRANSACTION_GUID_QUEQ_SIZE)pWERSTransaction->number ++;
		ret = SMPL_SUCCESS;
	}
	else
	{
		pTransGUID = pWERSTransaction->Using;
		if(pTransGUID)
		{
			pWERSTransaction->Using = pTransGUID->next;

			pTransGUID->linkID = linkID;
			pTransGUID->operationCode = operationCode;
			pTransGUID->GUID = GUID;
			pTransGUID->TimeStamp = tmrMsTimeGet();
			pTransGUID->next = NULL;
			pWERSTransaction->UsingTail->next = pTransGUID;
			pWERSTransaction->UsingTail = pTransGUID;
		}

		printf_0("Transaction Full\n");
	}
	BSP_EXIT_CRITICAL_SECTION(intState);
	return ret;

}

UINT32 PendTransactionToQue(WERSTransaction_t *pWERSTransaction, UINT8 linkID, UINT8 operationCode, UINT32 GUID)
{
	WERSTransactionGUID_t *pTransGUID,*pPreTrans;
	UINT32 status,CurrentMs;
	bspIState_t intState;

	BSP_ENTER_CRITICAL_SECTION(intState);
	CurrentMs = tmrMsTimeGet();
	if(pPreTrans = pWERSTransaction->Using)
	{
		do 
		{
			if(CurrentMs - pPreTrans->TimeStamp > WERS_TRANSACTION_TIMEOUT)
				status = 2;
			else if ( pPreTrans->linkID == linkID 
				&& pPreTrans->operationCode == operationCode 
				&& pPreTrans->GUID == GUID )
				status = 1;
			else
				status = 0;
			if(status)
			{
				pWERSTransaction->Using = pPreTrans->next;
				if(pWERSTransaction->Using == NULL)
				{
					pWERSTransaction->UsingTail = NULL;
				}
				pPreTrans->next = pWERSTransaction->Unused;
				pWERSTransaction->Unused = pPreTrans;

				if(pWERSTransaction->number)pWERSTransaction->number --;
				if(status == 1)
				{
					BSP_EXIT_CRITICAL_SECTION(intState);
					return (UINT32)pPreTrans;
				}
				pPreTrans = pWERSTransaction->Using;
			}
			else
				break;
		} while (pPreTrans);

		if(!pPreTrans)
		{
			BSP_EXIT_CRITICAL_SECTION(intState);
			return 0;
		}
#ifdef WERS_NEWVER_TRANSACTION_DBGON
		{
			int i;
			WERSTransactionGUID_t *ptmp;

			printf_0("CurrentMs %d\n",CurrentMs);
			printf_0("GUID:0x%08x\n",GUID);

			i = 0;
			for (ptmp = pWERSTransaction->Using; ptmp; ptmp=ptmp->next)
			{
				printf_0("GUID:0x%08x opCode:%02x TimeStamp:%d\n",ptmp->GUID,ptmp->operationCode,ptmp->TimeStamp);
				i++;
			}
			printf_0("Using %d\n",i);

			i = 0;
			for (ptmp = pWERSTransaction->Unused; ptmp; ptmp=ptmp->next)
			{
				i++;
			}
			printf_0("Unused %d\n",i);
		}

		printf_0("PendTransactionToQue %d\n",pWERSTransaction->number);
#endif
		for(pTransGUID = pPreTrans->next; pTransGUID;)
		{
			if(CurrentMs - pTransGUID->TimeStamp > WERS_TRANSACTION_TIMEOUT)
				status = 2;
			else if ( pTransGUID->linkID == linkID 
				&& pTransGUID->operationCode == operationCode 
				&& pTransGUID->GUID == GUID )
				status = 1;
			else
				status = 0;
			if(status)
			{
				if((pPreTrans->next = pTransGUID->next) == NULL)
					pWERSTransaction->UsingTail = pPreTrans;

				pTransGUID->next = pWERSTransaction->Unused;
				pWERSTransaction->Unused = pTransGUID;
				if(pWERSTransaction->number)pWERSTransaction->number --;
				if(status == 1)
				{
					BSP_EXIT_CRITICAL_SECTION(intState);
					return (UINT32)pTransGUID;
				}
				pTransGUID = pPreTrans->next;
			}
			else
			{
				pPreTrans = pTransGUID;
				pTransGUID = pTransGUID->next;
			}
		}
	}
	BSP_EXIT_CRITICAL_SECTION(intState);
	return 0;
}
#endif		//WERS_NEWVER_TRANSACTION_SUPPORT


UINT32 WERSSendRTS(UINT8 linkID,UINT32 datalen,UINT8 opCode, UINT32 GUID,UINT32 *pOutCode)
{
	ioctlRawSend_t    send;
	smplStatus_t ret = SMPL_TIMEOUT;
	uint8_t msg[MGMT_RTS_FRAME_SIZE];
	//SINT32 trycount = 10;
	//UINT32 err;
	//UINT32 notifmsg;
		
	connInfo_t * pCInfo = nwk_getConnInfo(linkID);
	if ( pCInfo )
	{
		msg[0] = MGMT_CMD_RTS;
		// pack dst address
		memcpy((UINT8*)(msg+M_RTS_DST_ADDRESS_OS),(UINT8*)pCInfo->peerAddr,sizeof(addr_t));

		// pack data size
		memcpy((UINT8*)(msg+M_RTS_DATA_LEN_OS),(UINT8*)&datalen,sizeof(datalen));

		msg[M_RTS_LINK_ID_OS] = linkID;
		msg[M_RTS_OPERATON_CODE_OS] = opCode;
		memcpy((msg+M_RTS_GUID_OS),&GUID,sizeof(GUID));

		send.addr = (addr_t *)nwk_getBCastAddress();
		send.msg  = (uint8_t*)msg;
		send.len  = sizeof(msg);
		send.port = SMPL_PORT_MGMT;

		//nwk_disableRTSProcess();
		ClearTimeOut();		
		do{	
			if ( BackOffTimeOut() ) // wait other send data
			{
				// start to compete RTS
				ret = SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_WRITE, &send);
				if ( SMPL_SUCCESS == ret )
				{
					//osSemPend(g_pSemCTS, 10, &err); // wait 100ms
					//notifmsg = osQuePend(g_pMacQue, 10, &err); // wait 100ms
					UINT32 i;
					for( i = 0; i < 100; i++)
					{
						if ( 0 != g_NotifyCode  )
						{
							break;
						}
						rf_hwMsWait(1);
					}
											
					//if ( OS_TIMEOUT == err ) // no received CTS 
					if ( i >= 100 ) // nothing
					{					
						rf_printf_0("==no received CTS \n");
						#if 0
						  uint8_t backoffs;
						  uint8_t i;

						  /* calculate value for backoffs -1 - 50*/
						  backoffs = MILD_min();
						  for (i=0; i<backoffs; i++)
						  {
							    //rf_hwUsWait( MRFI_BACKOFF_PERIOD_USECS );
							    rf_hwUsWait( sBackoffHelper );
						  }
						 #endif
						 rf_hwMsWait(MILD_min());
						 //rf_hwMsWait(MILD_max());
					}
					else
					{
						if ( MGMT_CMD_CTS == g_NotifyCode ) // get CTS
						{
							ClearTimeOut();
							MILD_max();
							//MILD_min();
							rf_printf_0("...get CTS, start to send data \n");
							// get CTS, start to send data
							RestNotifyCode();
							*pOutCode = MGMT_CMD_CTS;
							//break;
							return SMPL_SUCCESS;
						}
						else if ( MGMT_CMD_DATA_ACK == g_NotifyCode )
						{
							// get Data ack
							RestNotifyCode();
							*pOutCode = MGMT_CMD_DATA_ACK;
							//break;
							return SMPL_SUCCESS;
						}
					}
				}
				else
				{
					// run Backoff code
					rf_printf_0("run Backoff code \n");
					#if 1
					rf_hwMsWait(MILD_min());
					//rf_hwMsWait(MILD_max());
					#else
					uint8_t backoffs;
					uint8_t i;

					/* calculate value for backoffs -1 - 50*/
					backoffs = MILD_min();					
					for (i=0; i<backoffs; i++)
					{
					    //rf_hwUsWait( MRFI_BACKOFF_PERIOD_USECS );
					    rf_hwUsWait( sBackoffHelper );
					}
					#endif
				}
			}
		}while( !CheckTimeOutEx(10000,0) );
		//nwk_enableRTSProcess();
	}
	
	return SMPL_TIMEOUT;
}	

UINT32 RFMacSemPend(UINT32 timeout,UINT32 *err)
{
	UINT32 i;
	UINT32 ms = 0;

	if ( OS_NO_WAIT == timeout || 0 == timeout )
	{
		if ( 0 != g_NotifyCode  )
		{
			*err = OS_NO_ERR;
			return g_NotifyCode;
		}
	}
	else
	{
		ms = timeout;
		for( i = 0; i < ms; i++ )
		{
			if ( 0 != g_NotifyCode  )
			{
				*err = OS_NO_ERR;
				return g_NotifyCode;
			}

			//for watch dog, can't remove
			g_ApTaskCondition = 0;

			rf_hwMsWait(1);
		}
	}

	*err = OS_TIMEOUT;
	return 0;
}

UINT32 WERS_Transmit(UINT8 lid, UINT8 *pData, UINT32 datasize)
{
	smplStatus_t ret = SMPL_TIMEOUT;
	WERSContainerStruct *pContainer =  (WERSContainerStruct*)pData;
	connInfo_t * pCInfo;
	bspIState_t intState;
	UINT32 err;
//	SINT32 SendDatatryCount = 10;
	SINT32 gotCTSandSendDataNumber = 0;
	SINT32 gotDataAckNumber = 0;
	UINT32 waitTimeOut;
	UINT32 backoffTime;
	SINT32 RTSSendFailNumber = 0;
//	UINT32 notifycode = 0;
	UINT32 ackGUID = 0;
	SINT32 sendRTSNumber = 0;

	if ( NULL == pContainer || datasize < sizeof(WERSContainerStruct) )
	{
		rf_printf_0("=====bad parameter!! %s %d\n",__FUNCTION__,__LINE__);
		return SMPL_BAD_PARAM;
	}

	pCInfo = nwk_getConnInfo(lid);
	if ( NULL == pCInfo )
	{
		rf_printf_0("=====invaild linkID!! %s %d\n",__FUNCTION__,__LINE__);
		return SMPL_BAD_PARAM;
	}


	
	BSP_ENTER_CRITICAL_SECTION(intState);
	g_NotifyCode = 0;
	BSP_EXIT_CRITICAL_SECTION(intState);


	while( 1 )
	{
		// do backoff
		{
			UINT32 Timelimit = tmrMsTimeGet() + 2000;
			do{
				//for watch dog, can't remove
				g_ApTaskCondition = 0;
			}while(!BackOffTimeOutEx(Timelimit));
		}

		
		if ( SMPL_SUCCESS == RF_SendRTS(lid,pContainer->Code,pContainer->GUID,datasize) )
		{
			sendRTSNumber++;
			if ( sendRTSNumber >=  10 )
			{
				// recevier no longer in line. break.
				break;
			}

			//wait time out for CTS 
			waitTimeOut = 300; // 200ms
			RTSSendFailNumber--;
			rf_printf_0("RTS count:%d \n",sendRTSNumber);
		}
		else
		{	
			rf_printf_0("run Backoff code \n");
			
			RTSSendFailNumber++;
			if ( RTSSendFailNumber > 15 )
			{
				//cancel .....
				rf_printf_0("break 00 \n");
				break;
			}

			//  Backoff 
			waitTimeOut = OS_NO_WAIT;
		}


		//for watch dog, can't remove
		g_ApTaskCondition = 0;
		
_LoopLable_1:
		//notifycode = RFMacSemPend(waitTimeOut,&err);
		RFMacSemPend(waitTimeOut,&err);
		if ( OS_TIMEOUT == err )
		{
			////////////////////////////////////////////
			backoffTime = MILD_min();	
			rf_hwMsWait(backoffTime);
			////////////////////////////////////////////
		}
		else
		{
			sendRTSNumber = 0;
			BSP_ENTER_CRITICAL_SECTION(intState);
			if ( MGMT_CMD_CTS == g_NotifyCode )
			{
				g_NotifyCode = 0;
				BSP_EXIT_CRITICAL_SECTION(intState);

				rf_printf_0("CTS 1 \n");

				
				if ( gotCTSandSendDataNumber >= 5 )
				{
					break;
				}

				#if 0
				// start to send
				SendDatatryCount = 5;
				do
				{
					//for watch dog, can't remove
					g_ApTaskCondition = 0;
					
					ret = SMPL_SendEx(lid,pData,datasize);
					if( ret == SMPL_SUCCESS )
					{
						break;
					}
					else
					{
						WERS_RandomDelay(1); // random n*100us
					}
				}while(SMPL_SUCCESS != ret &&  SendDatatryCount-- );
				#else
				MILD_max();

				if ( SMPL_SUCCESS ==  SMPL_SendEx(lid,pData,datasize) )
				{
					// to notify recevier recevie data.
					if ( SMPL_SUCCESS == RF_SendRTS(lid,pContainer->Code,pContainer->GUID,datasize) )
					{
						gotCTSandSendDataNumber++;
						waitTimeOut = 500;
						rf_printf_0("send data ok 1 \n");
						goto _LoopLable_1;
					}
				}
				#endif
			}
			else if ( MGMT_CMD_DATA_ACK == g_NotifyCode )
			{
				g_NotifyCode = 0;
				ackGUID = g_DataAckGUID;
				g_DataAckGUID = 0;
				BSP_EXIT_CRITICAL_SECTION(intState);

				ClearBackOffTimeOut();
								
				rf_printf_0("ACK! \n");
				
				if (( (gotCTSandSendDataNumber > 0 )
					&& (ackGUID == pContainer->GUID) )
					|| (gotDataAckNumber++ >= 10) )
				{
					// send data ok!, break while
					ret = SMPL_SUCCESS;
					break;
				}
			}
			else
			{
				BSP_EXIT_CRITICAL_SECTION(intState);
				rf_printf_0("other!");
			}
		}
	}
	BSP_ENTER_CRITICAL_SECTION(intState);
	g_NotifyCode = 0;
	BSP_EXIT_CRITICAL_SECTION(intState);

	return ret;
}

UINT32 WERSSendResponseForAP(WERSContainerStruct *pHeader,UINT8 *pdata, UINT16 datalen)
{
	UINT32 ret = SMPL_SUCCESS;
	WERSContainerStruct *pContainer;
	uint16_t buffersize = sizeof(WERSContainerStruct) + datalen;
	uint8_t *pOutData;
	UINT8 lid;
	
	if ( NULL == pHeader )
	{
		rf_printf_0("=====bad param!! %s %d\n",__FUNCTION__,__LINE__);
		return SMPL_BAD_PARAM;
	}
	
	pOutData	 = (uint8_t *)rf_alloc(buffersize);
	if( pOutData == NULL  )
	{
		rf_printf_0("=====not enough memory!! %s %d\n",__FUNCTION__,__LINE__);
		return SMPL_NOMEM;
	}

	// has data
	if( pdata != NULL && datalen > 0 )
	{
		memcpy(pOutData+sizeof(WERSContainerStruct),pdata,datalen);
	}
	
	pContainer = (WERSContainerStruct*)pOutData;
	memcpy(pContainer,pHeader,sizeof(WERSContainerStruct));	
	pContainer->ContainerType = WERS_RESPONSE_BLOCK;
	pContainer->GUID = WERSGenerate_RF_GUID();
	pContainer->ContainerLength = buffersize;
	lid = pHeader->Prot;
#if 0
	SINT32 trycount = 10;

	// send to nwk
	do
	{
		// send response to ED
		ret = SMPL_SendEx(lid,pOutData,buffersize);
		if( ret == SMPL_SUCCESS )
		{
			break;
		}
		else
		{
			WERS_RandomDelay(0);
			trycount--;
		}
	}while( trycount > 0 );
#else
	//nwk_disableRTSProcess();
	ret = WERS_Transmit(lid,pOutData,buffersize);
	//nwk_enableRTSProcess();
#endif


	#if 0
	WERS_InsertTranscationToResponseQueue(pOutData, pContainer->ContainerLength,lid,pContainer->Code,pContainer->TransID);
	#else
	if( pOutData != NULL )
	{
		rf_free(pOutData);
	}
	#endif
	
	return ret;
}
#if 0
/******************************************************************************
 * @fn          WERSSendData
 *
 * @brief       Send a message to a peer application.
 *
 * input parameters
 * @param   lid     - Link ID (port) from application
 * @param   msg     - pointer to message from app to be sent
 * @param   len     - length of enclosed message
 *
 * output parameters
 *
 * @return   Status of operation. Either succeeds or it fails because of a timeout.
 *           A timeout can occur if the channel cannot be obtained. The frame
 *           buffer is discarded and the Send call must be redone by the app.
 */
UINT32 WERSSendData(UINT8 lid, UINT8 *msg, UINT32 len)
{
	UINT32 ret = FAIL;
	SINT32 trycount = 30;
	
	if ( NULL != msg && len > 0)
	{	
		do
		{
			ret = SMPL_SendEx(lid,msg,len);
			if( ret == SMPL_SUCCESS )
			{
				break;
			}
			else
			{
				WERS_RandomDelay(0);
			}
			trycount--;
		}while(ret != SMPL_SUCCESS &&  trycount > 0 );
	}
	return ret;
}
#endif
UINT32 WERSSendDataAck(UINT8 linkID,UINT32 GUID,UINT32 trycount)
{
	ioctlRawSend_t    send;
	smplStatus_t ret = SMPL_TIMEOUT;
	uint8_t msg[MGMT_DATA_ACK_FRAME_SIZE];
	int32_t resend = trycount;
	

	connInfo_t * pCInfo = nwk_getConnInfo(linkID);
	if ( pCInfo )
	{
		msg[M_DATA_ACK_CMD_OS] = MGMT_CMD_DATA_ACK;

		// pack dst address
		memcpy((UINT8*)(msg+M_DATA_ACK_DST_ADDRESS_OS),(UINT8*)pCInfo->peerAddr,sizeof(addr_t));

		// pack GUID
		memcpy((UINT8*)(msg+M_DATA_ACK_GUID_OS),(UINT8*)&GUID,sizeof(GUID));

		send.addr = (addr_t *)nwk_getBCastAddress();
		send.msg  = (uint8_t*)msg;
		send.len  = sizeof(msg);
		send.port = SMPL_PORT_MGMT;

		do
		{
			ret = SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_WRITE, &send);
			if ( SMPL_SUCCESS == ret )
			{
				break;
			}
			else
			{
				if ( trycount > 0 )
				{
					//osTimeDly(5);
					rf_hwMsWait(2);
				}
			}
			resend--;
		}while(resend);
		
		rf_printf_0("send data ack %d!\n",GUID);
	}	

	return ret;
}

UINT32 WERSSendACK(UINT8 lid,UINT16 systemID, UINT8 Code, UINT32 transID)
{
	smplStatus_t ret = SMPL_SUCCESS;
	WERSContainerStruct Container;

	Container.Code = Code;
	Container.ContainerType = WERS_ACK_BLOCK;
	Container.SystemID = systemID;
	Container.Prot = lid;
	Container.TransID = transID;
	Container.ContainerLength = sizeof(WERSContainerStruct);

	// send to nwk
	do{
		ret = SMPL_Send(lid,(uint8_t*)&Container,sizeof(WERSContainerStruct));
		if(ret == SMPL_SUCCESS )
		{
			break;
		}
		else
		{
			WERS_RandomDelay(0);
		}
	}while( ret != SMPL_SUCCESS );

	return ret;
}


UINT32 WERSGetACKResponse(UINT8 lid,UINT16 systemID, UINT8 Code, UINT32 transID,UINT32 timeOut)
{	
	uint8_t len;
	//uint8_t *pDataInBuffer;
	//uint8_t *pDataOffset;
	//int32_t datalen;
	uint8_t msg[MAX_APP_PAYLOAD];
	smplStatus_t ret;
	//uint8_t wait = 0;
	WERSContainerStruct *pContainer;
	
	rf_printf_0("==========WERSGetACKResponse=============\n");


	ClearTimeOut();
	do
	{
		ret = SMPL_Receive(lid, msg, &len);
		if ( SMPL_SUCCESS ==  ret)
		{
			//rf_printf_0("receive ok!\n");
			ClearTimeOut();
			break;
		}
		else{
			if(CheckTimeOut(timeOut))
			{
				rf_printf_0("==receive SMPL_TIMEOUT: %d\n",ret);
				return SMPL_TIMEOUT;
			}
		}
	}while(1);

	pContainer = (WERSContainerStruct*)msg;


	// get remaining data completed
	
	//rf_printf_0("\n  %d...ACK from linkID(%d )   %d .........\n",len,lid, pContainer->ContainerLength);
	#if 0 //debug
	int i;
	for( i = 0; i < pContainer->ContainerLength; i++)
	{
		rf_printf_0("%02x  ",msg[i]);
	}
	rf_printf_0("\n......................................\n");
	#endif


	/* error check */
	if (WERS_TYPE(pContainer->ContainerType) != WERS_ACK_BLOCK
		|| pContainer->ContainerLength != WERS_DATA_BLOCK_HEADER_LEN 
		|| pContainer->TransID != transID
		|| pContainer->SystemID != systemID
		)
	{
			rf_printf_0("data type error! %s %d\n",__FUNCTION__,__LINE__);
			return SMPL_BAD_PARAM;
	}
	
	
	if(pContainer->Code != Code)
		return pContainer->Code+100;


	return SMPL_SUCCESS;
}

/******************************************************************************
 * @fn          SMPL_SendEx
 *
 * @brief       Send a message to a peer application.
 *
 * input parameters
 * @param   lid     - Link ID (port) from application
 * @param   msg     - pointer to message from app to be sent
 * @param   len     - length of enclosed message
 *
 * output parameters
 *
 * @return   Status of operation. Either succeeds or it fails because of a timeout.
 *           A timeout can occur if the channel cannot be obtained. The frame
 *           buffer is discarded and the Send call must be redone by the app.
 */
UINT32 SMPL_SendEx(UINT8 lid, UINT8 *msg, UINT32 len)
//smplStatus_t SMPL_SendEx(linkID_t lid, uint8_t *msg, uint32_t len)
{
	uint8_t *data = msg;
	int32_t left = (int32_t)len;
	smplStatus_t ret = SMPL_SUCCESS;
	int32_t trycount = WERS_SEND_RELAY_NUMBER;
	//UINT32 err;
	
	rf_printf_0("\n=======SMPL_SendEx====== len: %d \n",len);
	while( left>0 )
	{
		trycount = WERS_SEND_RELAY_NUMBER;
		
		do{
			//for watch dog, can't remove
			g_ApTaskCondition = 0;
			
			ret = SMPL_Send(lid,data, (left > MAX_APP_PAYLOAD)?MAX_APP_PAYLOAD:left);
			if( ret == SMPL_SUCCESS )
				break;
			
			WERS_RandomDelay(1);
			
			trycount--;
			rf_printf_1(" send one packte   ret: %d\n",ret);
			
		}while( ret != SMPL_SUCCESS && trycount > 0 );
		
		
		if( ret != SMPL_SUCCESS )
		{
			rf_printf_0(" send one packte error!! :%d\n",ret);
			
			return ret;
		}	
		
		data+=MAX_APP_PAYLOAD;
		left-=MAX_APP_PAYLOAD;
	}

	rf_printf_1(" send data completed!! \n");
	return ret;
}

UINT32 WERSGetResponseForPC(UINT8 lid,UINT16 systemID,UINT8 **pdata, UINT8 *pdataSize)
//smplStatus_t WERSGetResponse(linkID_t lid,uint16_t systemID,uint8_t **pdata, uint16_t *pdataSize)
{	
	uint8_t len;
	uint8_t *pDataInBuffer;
	uint8_t *pDataOffset;
	int32_t datalen;
	uint8_t msg[MAX_APP_PAYLOAD];
	smplStatus_t ret;
	WERSContainerStruct *pContainer;

	rf_printf_0("==========getPlayload=============\n");

	if( pdata == NULL || pdataSize == NULL )
	{
		return SMPL_BAD_PARAM;
	}

	do
	{
		ret = SMPL_Receive(lid, msg, &len);
		//rf_printf_0("==receive ret: %d\n",ret);
		if ( SMPL_SUCCESS ==  ret)
		{
			ClearTimeOut();
			rf_printf_0("==get ok!=\n");
			break;
		}
		else{
			if(IS_TIME_OUT)
			{
				*pdataSize = 0;
				pdata = NULL;
				rf_printf_0("==receive SMPL_TIMEOUT: %d\n",ret);
				return SMPL_TIMEOUT;
			}
		}
	}while(1);


	pContainer = (WERSContainerStruct*)msg;


	/* error check */	
	if (WERS_TYPE(pContainer->ContainerType) != WERS_RESPONSE_BLOCK
		|| pContainer->ContainerLength < WERS_DATA_BLOCK_HEADER_LEN 
		|| pContainer->SystemID != systemID )
	{
		rf_printf_0("data type error! %s %d\n",__FUNCTION__,__LINE__);
		*pdataSize = 0;
		pdata = NULL;
		return SMPL_BAD_PARAM;
	}



	*pdataSize = datalen = pContainer->ContainerLength;
	rf_printf_0("==========getPlayload len: %d\n",datalen);
	
	*pdata = pDataOffset = pDataInBuffer = (uint8_t *)rf_alloc(datalen);
	if(pDataInBuffer == NULL )
	{
		rf_printf_0("=====not enough memory!! %s %d\n",__FUNCTION__,__LINE__);
		*pdataSize = 0;
		pdata = NULL;
		return  SMPL_NOMEM;							
	}
	memcpy(pDataOffset,msg,len);
	pDataOffset += len;
	
	// get remaining data
	if( datalen > len ) 
	{
		do
		{
			if ( SMPL_SUCCESS == SMPL_Receive(lid, pDataOffset, &len) )
			{
				ClearTimeOut();
				pDataOffset+=len;
				datalen-=len;
			}
			else
			{
				if(IS_TIME_OUT)
				{
					/* time out process */
					if(pDataInBuffer != NULL )
					{
						rf_free(pDataInBuffer);
					}

					*pdataSize = 0;
					pdata = NULL;
					return SMPL_TIMEOUT;
				}
			}
		}while(datalen > 0);

	}

	// get remaining data completed
	#if 0 //debug
	rf_printf_0("\n.........WERS response data from linkID(%d) .........\n",lid);
	int i;
	for( i = 0; i < *pdataSize; i++)
	{
		rf_printf_0("%02x  ",pDataInBuffer[i]);
	}
	rf_printf_0("\n......................................\n");
	#endif

	return SMPL_SUCCESS;
}



UINT32 WERSBoadcast(UINT8 CmdCode)
{
	WERSContainerStruct WERS;
	ioctlRawSend_t    send;
	int32_t trycount = WERS_SEND_RELAY_NUMBER;
	UINT8 ret = SMPL_SUCCESS;

	WERS.Code = CmdCode;
	WERS.ContainerLength =sizeof(WERSContainerStruct);
	WERS.ContainerType = WERS_COMMAND_BLOCK;
	WERS.Prot = SMPL_PORT_USER_BCAST;

	send.addr = (addr_t *)nwk_getBCastAddress();
	send.msg  = (uint8_t*)&WERS;
	send.len  = WERS.ContainerLength;
	send.port = SMPL_PORT_USER_BCAST;

	do{	
		rf_hwMsWait( 1);
		ret = SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_WRITE, &send);
		
		trycount--;
		rf_printf_1(" send one packte   ret: %d\n",ret);
		
	}while( ret != SMPL_SUCCESS && trycount > 0 );

	return ret;
}

static UINT16 WERSReadDevice(struct DEV_ADDR *pAddr)
{
//	INT32 fd = -1;
//	UINT32 bytes;
//	UINT16 ret;
	static const struct DEV_ADDR Default_Addr= {100,100,2};


	if(NULL == pAddr)
		return RF_FAIL;

#if 0
	ret =  fsOpen(DEVICE_PATH, FS_O_RDONLY, &fd);
	if(ret != RF_SUCCESS )
	{
		rf_printf_0("open file error:%x\n", ret);
		return ret;
	}

	ret = fsRead(fd, (UINT8 *)pAddr, sizeof(struct DEV_ADDR), &bytes);
	fsClose(fd);
	if(bytes < sizeof(struct DEV_ADDR))
		ret = RF_FAIL;
	if(ret != RF_SUCCESS )
	{
		rf_printf_0("read file error:%x\n", ret);
		return ret;
	}
#else
	Flash_Read(pAddr,FLASH_AP_DEVICE_CFG,sizeof(struct DEV_ADDR));
	if(pAddr->hotel == 0xff || pAddr->hotel == 0x00)
	{
		#if 0
		pAddr->hotel=201;
		pAddr->host=30;
		pAddr->dev=2;
		#else
		memcpy(pAddr, &Default_Addr, sizeof(struct DEV_ADDR));
		#endif
		Flash_Write(FLASH_AP_DEVICE_CFG,pAddr,sizeof(struct DEV_ADDR));
	}
	
#endif
	pAddr->dev &= ((1<<12)-1);
	pAddr->dev |= ( (UINT16)g_MyNodeType<<12);
	return RF_SUCCESS;
}

static UINT16 WERSWriteDevice(struct DEV_ADDR *pAddr)
{
//	UINT32 fd = -1;
//	UINT32 bytes;
//	UINT16 ret;

	if(NULL == pAddr)
		return RF_FAIL;

	if(Flash_Write(FLASH_AP_DEVICE_CFG,pAddr,sizeof(struct DEV_ADDR))<0)return RF_FAIL;

#if 0
	ret =  fsOpen(DEVICE_PATH, FS_O_CREATE|FS_O_WRONLY, &fd);
	if(ret != RF_SUCCESS )
	{
		rf_printf_0("open file error:%x\n", ret);
		return RF_FAIL;
	}

	ret = fsWrite(fd, (UINT8 *)pAddr, sizeof(struct DEV_ADDR), &bytes);
	fsClose(fd);
	if(ret != RF_SUCCESS )
	{
		rf_printf_0("write file error:%x\n", ret);
	}
	return ret;
#endif
	return RF_SUCCESS;
}

#ifdef WERS_EXTENDER_DEVICE
UINT32 SendDataToPC(UINT8 *pData, UINT32 len)
{
	#if 0
	if( pData != NULL && len > 0)
	{
		UINT32 sendnumber;

		return WERS_UartDrv_Tx(pData,len,&sendnumber);
	}
	else
	{
		return SMPL_BAD_PARAM;
	}
	
	#else
	
	int i;

	if( pData == NULL )
	{
		return SMPL_BAD_PARAM;
	}

	

	SPMP_Config485ToSend();

	
	for(i = 0; i < len; i++)
	{
		//_uartPutChar(1,pData[i]);
		USART_SendData(RS485_COM_PORT,pData[i]);
		while (USART_GetFlagStatus(RS485_COM_PORT, USART_FLAG_TC) == RESET);
	}

	SPMP_Config485ToReceive();

	return SMPL_SUCCESS;
	#endif
}

UINT32 WERS485Send(UINT8 DataType,UINT8 *pdata, UINT16 datalen)
{
	WERS485FrameHeader	 *p485FrameHeader;
	UINT32 buffersize = sizeof(WERS485FrameHeader) + datalen;

	UINT8 *pOutBuffer;

	pOutBuffer = (uint8_t *)rf_alloc(buffersize);
	if( pOutBuffer == NULL  )
	{
		rf_printf_0("=====not enough memory!! %s %d\n",__FUNCTION__,__LINE__);
		return SMPL_NOMEM;
	}

	if( pdata != NULL && datalen > 0 )
	{
		memcpy(pOutBuffer+sizeof(WERS485FrameHeader),pdata,datalen);
	}

	p485FrameHeader = (WERS485FrameHeader *)pOutBuffer;

	p485FrameHeader->ContainerLength = buffersize;
	p485FrameHeader->DataCRC = cal_crc((const UINT8 *)pdata,datalen);
	p485FrameHeader->ContainerType = DataType;
	//p485FrameHeader->HeaderCRC = cal_crc8((const UINT8 *)p485FrameHeader, (sizeof(WERS485FrameHeader) -1) );


	//send
	#ifdef ENABLE_485_DMA_TRANSMIT
	WERS_UartDrv_Tx(pOutBuffer,buffersize,&buffersize);
	#else
	SendDataToPC(pOutBuffer, buffersize);
	#endif

	// destory memory
	if (pOutBuffer)
	{
		rf_free((void*)pOutBuffer);
		pOutBuffer = NULL;
	}

	return SMPL_SUCCESS;
}
#endif

UINT16 WERSAPCmd(WERSContainerStruct *pWERS)
{
	UINT8 data[64];
	WERSContainerStruct *pResponse = (WERSContainerStruct *)data;
	struct DEV_ADDR stDevice;
	UINT16 ret;
	UINT8 *pVersion;

	ioctlRawSend_t    send;
//	smplStatus_t ret = SMPL_SUCCESS;
	int32_t trycount = WERS_SEND_RELAY_NUMBER;
	UINT8 msg[10];


	if(NULL == pWERS)
		return RF_FAIL;
	switch(pWERS->Code)
	{
	case COM_CMD_APGET:/*get ap info*/
	{

		pResponse->ContainerLength = sizeof(WERSContainerStruct) + sizeof(UINT16);
		if(WERS_EVENT_BLOCK == WERS_TYPE(pWERS->ContainerType))
			pResponse->ContainerType = WERS_EVENT_BLOCK;
		else
			pResponse->ContainerType = WERS_RESPONSE_BLOCK;
		pResponse->SystemID = g_WERSSystemId;
		pResponse->Code = COM_CMD_APGET;

		WERSGetDeviceAddress( (WERSAddr_t*)&stDevice );

		stDevice.dev &= 0x0FFF;
		//pResponse->Prot = RF_HOST_AP;
		pResponse->Prot = g_MyNodeType;
#ifdef WERS_EXTENDER_DEVICE
		if(DEVICE_CONNECTED == gs_DeviceState)
			pResponse->TransID = (UINT8)g_mrfi_rssi;
		else
			pResponse->TransID = 0;
#else
		pResponse->TransID = (UINT8)g_mrfi_rssi;
#endif

		memcpy(pResponse->RouteMap, &stDevice, sizeof(stDevice));
		//pack version
		pVersion = (UINT8*)(pResponse + 1);
		pVersion[0] = MAIN_VERSION;
		pVersion[1] = SUB_VERSION;
		break;
	}
	case COM_CMD_APSET:/*set ap info*/
		memcpy(&stDevice, pWERS->RouteMap, sizeof(stDevice));
		stDevice.dev &= 0x0FFF;
		stDevice.dev |= (UINT16)(pWERS->Prot) << 12;
		//stDevice.dev |= ((UINT16)RF_HOST_AP) << 12;
		ret = WERSWriteDevice(&stDevice);

		pResponse->ContainerLength = sizeof(WERSContainerStruct);
		pResponse->ContainerType = WERS_RESPONSE_BLOCK;
		pResponse->SystemID = g_WERSSystemId;
		pResponse->Code = COM_CMD_APSET;
		//pResponse->Prot = RF_HOST_AP;
		pResponse->Prot = g_MyNodeType;
		pResponse->TransID = (RF_SUCCESS == ret) ? 0 : 1;
		break;
	case COM_CMD_AP_RESET:
	{
		//tmrPeriodFuncDel(TIMER0, g_WatchDogTimerID);
		//DeleteTimer(g_WatchDogTimerID);
		SystemReset();
		return RF_SUCCESS;	
	}
	case 0xF5:  // for debug
		
		send.addr = (addr_t *)nwk_getBCastAddress();
		send.msg  = msg;
		send.len  = sizeof(msg);
		send.port = SMPL_PORT_USER_BCAST;


		do{
			rf_hwMsWait( 1);
			ret = SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_WRITE, &send);
			
			trycount--;
			rf_printf_1(" send one packte   ret: %d\n",ret);
			
		}while( ret != SMPL_SUCCESS && trycount > 0 );

		return RF_SUCCESS;					
	case 0xF6:
		{
			UINT8 data[256] ={0};
			UINT32 datalen = 0;
			connInfo_t  *pCInfo = NULL;
			UINT32 i;
			UINT8 *pBuffer = NULL;

			data[0] = sJoinSem;
			data[1] = sPeerFrameSem;
			data[2] = sNumCurrentPeers;

			pBuffer = (UINT8*)&data[3];
			for( i = 0; i < sNumCurrentPeers; i ++)
			{
				pBuffer[0] = sLID[i];
				pBuffer++;
				
				pCInfo = nwk_getConnInfo(sLID[i]);
				if ( pCInfo )
				{
					memcpy(pBuffer,pCInfo->peerAddr,NET_ADDR_SIZE);
					pBuffer+=NET_ADDR_SIZE;
				}
			}

			datalen = pBuffer - data;

			SendDataToPC((UINT8*)data, datalen);
		}
		return RF_SUCCESS;
	default:
		return RF_FAIL;
	}
#ifndef WERS_EXTENDER_DEVICE
	SendDataToPC((UINT8*)pResponse,pResponse->ContainerLength);
#else
	WERS485Send(FRAME_TYPE_485_DATA,(UINT8*)pResponse,pResponse->ContainerLength);
#endif
	return RF_SUCCESS;
}

UINT32 WERS_ReSend_Reply(UINT8 linkID,UINT8 code)
{
	ioctlRawSend_t    send;
	smplStatus_t ret = SMPL_BAD_PARAM;
	uint8_t msg[2];
	SINT32 trycount = 10;

	connInfo_t * pCInfo = nwk_getConnInfo(linkID);
	if ( pCInfo )
	{
		msg[0] = MGMT_CMD_RESEND_REQUEST_REPLY;
		msg[1] = code;
		send.addr = (addr_t *)pCInfo->peerAddr;
		send.msg  = (uint8_t*)msg;
		send.len  = sizeof(msg);
		send.port = SMPL_PORT_MGMT;
		do{
			ret = SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_WRITE, &send);
			if ( SMPL_SUCCESS == ret )
			{
				break;
			}
		}while( --trycount );
	}
	return ret;
}


//#ifndef WERS_EXTENDER_DEVICE
#ifndef WERS_USING_NETWORK
static UINT32 WERS_processReSendRequest(UINT8 code,void *pVal,UINT32 Valsize)
{
	WERSMsg_t *pMsg = NULL;
	UINT32 ret;
	
	if ( pVal && Valsize < 64 )
	{
		pMsg = rf_alloc(sizeof(WERSMsg_t));
		if ( pMsg )
		{
			pMsg->code = RESEND_REQUEST;
			pMsg->datasize = Valsize;
			memcpy(pMsg->data,pVal,Valsize);

			//rf_printf_0("post resend msg\n");
			ret = osQuePost( WERSQue, (void*)pMsg);
			if ( OS_Q_FULL == ret )
			{
				rf_printf_0("\n			post QS Que Full!!\n");
				rf_free(pMsg);
				pMsg = NULL;
			}
		}
	}
	return 0;
}

#endif

static void sReset(void)
{
	bspIState_t intState;

	sNumCurrentPeers = 0;
	sPeerFrameSem = 0;
	BSP_ENTER_CRITICAL_SECTION(intState);
	sJoinSem=0;
	BSP_EXIT_CRITICAL_SECTION(intState);
	memset(sLID, 0, sizeof(linkID_t) * NUM_CONNECTIONS);
}

// Runs in ISR context. Reading the frame should be done in the
// application thread not in the ISR thread.
static uint8_t sCB(linkID_t lid)
{	
	if (lid)
	{
		sPeerFrameSem++;
	}
	else
	{
		sJoinSem++;
	}

	// leave frame to be read by application.
	return 0;
}

void WERSPCPridge(UINT8 ch)
{
	static UINT32 ls_time = 0;
	static int ls_pos = 0;
	static UINT8 ls_data[32+sizeof(WERS485FrameHeader)];
	static UINT16 blockSize;
	static WERS485FrameHeader *pWERS485 = (WERS485FrameHeader*)ls_data;
	static WERSContainerStruct *pWERS = (WERSContainerStruct*)(ls_data + sizeof(WERS485FrameHeader));
	static UINT8 status=0;
	UINT32 tm;

	tm = tmrMsTimeGet();
	//if(ls_time + 1000 < tm)
	if(ls_time + 500 < tm)
	{
		status =0;
		ls_pos = 0;
		blockSize = 0;
	}
	ls_time = tm;

	switch(status)
	{
	case 0:
		ls_data[ls_pos++] = ch;
		if(ls_pos == 2)
		{
			blockSize = pWERS485->ContainerLength;
			pWERS485->ContainerLength = blockSize & (~0x8000);
			if(blockSize & 0x8000)
				blockSize = pWERS485->ContainerLength + 1;
			else
				blockSize = pWERS485->ContainerLength;
			if( blockSize >= sizeof(ls_data) ||
				pWERS485->ContainerLength < sizeof(WERS485FrameHeader) + sizeof(WERSContainerStruct) )
			{
				if(blockSize <= 2)
					ls_pos = 0;
				else
					status = 2;
			}
			else
				status ++;
		}
		break;
	case 1:
		ls_data[ls_pos++] = ch;
		if(blockSize <= ls_pos)
		{
			if(pWERS->ContainerLength >= sizeof(WERSContainerStruct) )
			if(WERS_COMMAND_BLOCK == WERS_TYPE(pWERS->ContainerType) || WERS_EVENT_BLOCK == WERS_TYPE(pWERS->ContainerType))
			if(COM_CMD_APGET == pWERS->Code || COM_CMD_APSET == pWERS->Code)
			{
				WERSAPCmd(pWERS);
			}
			status = 0;
			ls_pos = 0;
		}
		break;
	case 2:
		ls_pos ++;
		if(blockSize <= ls_pos)
		{
			status = 0;
			ls_pos = 0;
		}
		break;
	}
}


void WERSRFReceiverTask(void *arg)
{
	bspIState_t intState;
	INT8U osQuePendRet;
	WERSRTSPlayload *pRTS;

	while(1)
	{
		pRTS = (WERSRTSPlayload*)osQuePend(g_RTSQue, OS_WAIT_FOREVER, &osQuePendRet);
		SPMP_TurnLED(AP_LED_DATA, 2);
	
		if ( OS_NO_ERR == osQuePendRet && NULL != pRTS )
		{			
			if (sPeerFrameSem)
			{
				uint8_t     msg[MAX_APP_PAYLOAD], len;//, i;
				smplStatus_t ret;
				linkID_t linkID;
				linkID = pRTS->linkID;

				ret= SMPL_Receive(linkID, msg, &len);
				if(ret == SMPL_SUCCESS)	
				{
					#if 0
					int i;
					rf_printf_0("\n\n >>>>receive data from linkID %d     data show in following:\n",sLID[i]);
					for( i = 0; i < len; i++)
					{
						rf_printf_0("%02x  ",msg[i]);
					}
					rf_printf_0("\n\n\n>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
					#endif

					receivepackeNumber = 1;

					processMessage(linkID, msg, len);
					

					BSP_ENTER_CRITICAL_SECTION(intState);
					sPeerFrameSem-=receivepackeNumber;
					BSP_EXIT_CRITICAL_SECTION(intState);
				}			
			}
#ifndef WERS_NEWVER_TRANSACTION_SUPPORT
			if ( SearchTransactionGUID(&g_stWERSGetTransaction,pRTS->linkID, pRTS->opCode, pRTS->GUID) )
#else
			if ( PendTransactionToQue(&g_stWERSGetTransaction,pRTS->linkID, pRTS->opCode, pRTS->GUID) )

#endif
			{
				// recevied data ,relpy data ack to sender
				
				WERSSendDataAck(pRTS->linkID,pRTS->GUID,3);
				
				//rf_printf_0("..relpy data ack to sender\n");
			}
			else
			{
				if ( BackOffTimeOut() )
				{
					// relpy CTS to sender;
					WERSResponseCTS(pRTS);
				}
			}
			rf_free(pRTS);
			pRTS = NULL;
		}
	}
}


#if (defined WERS_485_PC_HOST )
void WERS485ListenTask(void)
{
	INT8U err;
	WERSFramInfo_t *pTranscation;
	
	while(1)
	{
		pTranscation = osQuePend(g_485ListenQue, OS_WAIT_FOREVER, &err);
		if ( OS_NO_ERR == err && NULL != pTranscation )
		{
			//SPMP_TurnLED(AP_LED_RUN, 2);

			if ( pTranscation->pPacketData && pTranscation->packetSize > 0 )		
			{
				#ifndef WERS_USING_NETWORK
				WERS485Send(FRAME_TYPE_485_DATA, (UINT8*)pTranscation->pPacketData,  pTranscation->packetSize);
				#else
				SendDataToPC((UINT8*)pTranscation->pPacketData,  pTranscation->packetSize);
				#endif
				pTranscation->fi_usage = WERS_FI_INUSE_UNTIL_DEL;
			}
			else
			{
				// erro, drop it.
				WERS_RemoveTranscationFromQ(pTranscation);
			}
		}
	}
}
#endif

#ifndef WERS_USING_NETWORK
void WERSAPRestartNotifyPC(void)
{
	UINT8 *pVersion;
	struct DEV_ADDR stDevice;
	UINT32 eventsize = sizeof(WERSContainerStruct) + sizeof(UINT16);

	UINT8 *pEventdata = 	rf_alloc(eventsize);
	if ( pEventdata )
	{
		WERSContainerStruct *pContianer = (WERSContainerStruct *)pEventdata;
		pContianer->ContainerLength = eventsize;
		pContianer->ContainerType = WERS_EVENT_BLOCK;
		pContianer->SystemID = g_WERSSystemId;
		pContianer->Code = COM_CMD_APGET;
		pContianer->Prot = RF_HOST_AP;
		pContianer->TransID = 0;

		pContianer->GUID = WERSMsTimeGet();//WERSGenerate_485_GUID();

		WERSGetDeviceAddress( (WERSAddr_t*)&stDevice );
		stDevice.dev &= 0x0FFF;
		
		memcpy(pContianer->RouteMap, &stDevice, sizeof(stDevice));
		pVersion = (UINT8*)(pContianer + 1);
		pVersion[0] = MAIN_VERSION;
		pVersion[1] = SUB_VERSION;

		#ifdef WERS_485_AP_HOST
		#ifndef WERS_DEBUG
		#ifndef WERS_USING_NETWORK
		WERS485Send(FRAME_TYPE_485_EVENT, (UINT8*)pEventdata, eventsize);
		#endif
		#endif
		#elif (defined WERS_485_PC_HOST)
		WERS_SaveReceivedTranscationInQ((UINT8 *)pEventdata, eventsize,1);
		#endif
	}
}
#endif	/*WERS_USING_NETWORK*/

#ifndef WERS_EXTENDER_DEVICE
void WERSAPTask(void *arg)
{
	bspIState_t intState;
	struct DEV_ADDR stDevice;
//	UINT32 osQuePendRet = OS_NO_ERR;
	UINT16 ret = RF_FAIL;
//	UINT32 counter = 0;

	UINT8 blink = 0;
#if 0
	struct DEV_ADDR Addr;
	Addr.hotel=201;
	Addr.host=30;
	Addr.dev=2;
	WERSWriteDevice(&Addr);
#endif
	g_timerID=CreateTimer(ResetReceive, 3200);
	//smplStatus_t nwkret;
	//SINT32 trycount = WERS_LISTEN_RELAY_NUMBER;
//	UINT8 *pMsg;

	rf_printf_0("\n====WERSAPTask========\n");

	//g_pRFSem = osSemCreate("RFSem", 0);
	//g_p485SemReceive = osSemCreate("485Sem", 0);	
	//g_pSemReceive = osSemCreate("Receive_Sem", 0);
#if (defined WERS_485_AP_HOST)	
	//g_pSem485Bridge = osSemCreate("Sem485Bridge", 0);
	//g_p485DataAckSem = osSemCreate("Sem485DataAck", 0);
	//g_pDataRecevieOkSem = osSemCreate("485DataRecevieOkSem", 0);
	//g_pDataComingSem = osSemCreate("485DataComingSem", 0);
	//g_p485PollSem = osSemCreate("485PollSem", 0);

	g_pDataRecevieOkSem = osSemCreate(0);
	g_pDataComingSem = osSemCreate(0);
#ifndef WERS_DEBUG
	g_pSem485Bridge = osSemCreate(0);
	g_p485DataAckSem = osSemCreate(0);
	g_p485PollSem = osSemCreate(0);
#endif
#endif


#ifdef ENABLE_485_DMA_TRANSMIT
	g_p485SendCheckSem = osSemCreate("485SendCheckSem", 0);
#endif

	//g_485OutQue =  osQueCreate("485OutQue", (void *)g_485OutQueBuf, WERS_485_OUT_QUE_SIZE);
#if (defined WERS_485_PC_HOST )
	//g_485ListenQue =  osQueCreate("485ListenQue", (void *)g_485ListenQueBuf, WERS_485_LISTEN_QUE_SIZE);
	g_485ListenQue =  osQueCreate((void *)g_485ListenQueBuf, WERS_485_LISTEN_QUE_SIZE);
#endif
	//g_485InQue =  osQueCreate("485InQue", (void *)g_485InQueBuf, WERS_485_IN_QUE_SIZE);
	//WERSQue  = osQueCreate("WERSQue", (void *)WERSQueBuf, WERS_QUE_SIZE);
	//g_RTSQue  = osQueCreate("WERSRTSQue", (void *)RTSQueBuf, WERS_RTS_QUE_SIZE);
	//g_pMacQue = osQueCreate("WERSMacQue", (void *)WERSMacQueBuf, MAC_QUE_SIZE);
#ifndef WERS_DEBUG
#ifndef WERS_USING_NETWORK
	g_485InQue =  osQueCreate((void *)g_485InQueBuf, WERS_485_IN_QUE_SIZE);
	g_pMacQue = osQueCreate((void *)WERSMacQueBuf, MAC_QUE_SIZE);
	WERSQue  = osQueCreate((void *)WERSQueBuf, WERS_QUE_SIZE);
#endif
#endif
	g_RTSQue  = osQueCreate((void *)RTSQueBuf, WERS_RTS_QUE_SIZE);
	
	osTaskCreate(
		//(UINT8*)"RFReceiver",
		WERSRFReceiverTask,
		(void *)0,
		(void *)(&gRFReceiverStack[STACKSIZE_RF_RECEIVER - 1]),
		OS_PRIO_WERS_RF_RECEIVER_TASK);


#ifndef WERS_DEBUG

#if (defined WERS_485_PC_HOST)
		osTaskCreate(
		//(UINT8*)"485ListenTask",
		WERS485ListenTask,
		(void *)0,
		(void *)(&g485SendTaskStack[STACKSIZE_485_SEND_TASK - 1]),
		OS_PRIO_WERS_485_SEND_TASK);


#elif (defined WERS_485_AP_HOST)
#if 0	//delete by shichu
	osTaskCreate(
		//(UINT8*)"485BridgeTask",
		WERS485BridgeTask,
		(void *)0,
		(void *)(&g485BridgeTaskStack[STACKSIZE_485_BRIDGE_TASK - 1]),
		OS_PRIO_WERS_485_BRIDGE_TASK);

	osTaskCreate(
		//(UINT8*)"485PollTask",
		WERS485Poll,
		(void *)0,
		(void *)(&g485PollTaskStack[STACKSIZE_485_POLL_TASK - 1]),
		OS_PRIO_WERS_485_POLL_TASK);
#endif

#endif
#ifndef WERS_USING_NETWORK
	osTaskCreate(
		//(UINT8*)"485RecevieTask",
		WERS485RecevieTask,
		(void *)0,
		(void *)(&g485RecevieTaskStack[STACKSIZE_485_RECEVIE_TASK- 1]),
		OS_PRIO_WERS_485_RECEIVE_TASK);
#endif	/*WERS_USING_NETWORK*/
#endif

#ifdef ENABLE_485_DMA_TRANSMIT
	osTaskCreate(
		//(UINT8*)"485SendCheck",
		WERS485SendCheckTask,
		(void *)0,
		(void *)(&g485SendCheckStack[STACKSIZE_485_SEND_CHECK - 1]),
		OS_PRIO_WERS_485_SEND_CHECK_TASK);
#endif


	sBackOffCounter = MRFI_RandomByte();
#ifndef WERS_NEWVER_TRANSACTION_SUPPORT
	memset(&g_stWERSGetTransaction,0,sizeof(g_stWERSGetTransaction));
#else
	InitTransactionGUID(&g_stWERSGetTransaction);
#endif

	//memset(&g_st485GetTransaction,0,sizeof(g_st485GetTransaction));
	memset(sWERSInFrameQ,0,WERS_IN_FRAME_Q_SIZE*sizeof(WERSFramInfo_t));
	//memset(sWERSTranactionInQ,0,WERS_TRANSACTION_IN_FRAME_Q_SIZE*sizeof(WERSFramInfo_t));
	WERS_QInit();

	//g_pMacQue  = osQueCreate("MACQue", (void *)WERSMacQueBuf, MAC_QUE_SIZE);

	ret = WERSReadDevice(&stDevice);
	if(RF_SUCCESS != ret)
	{
		stDevice.hotel = 1;
		stDevice.host = 1;
		stDevice.dev = 999;
	}
	#if 0 // for debug
	stDevice.hotel = 202;
	stDevice.host = 40;
	stDevice.dev = 365;
	#endif

	g_WERSSystemId = stDevice.host;
	g_WERSSystemId <<= 8;
	g_WERSSystemId |= stDevice.hotel;
	//sio_printf("WERSReadDevice  ok  !! hotel:%x host:%x pda:%04x\n",stDevice.hotel, stDevice.host, stDevice.dev);
	stDevice.dev &= 0x0FFF;
	stDevice.dev |= ((UINT16)RF_HOST_AP)<<12;
	WERSSetDeviceAddress((WERSAddr_t*)&stDevice);

	
	printf_0("===init AP===\nsystemID: %d host: %d hotel: %d\n\n",g_WERSSystemId,stDevice.host,stDevice.hotel);

	SMPL_Init(sCB);
	nwk_ResetSet(sReset);

	SPMP_TurnLED(AP_LED_RADIO, 1);
	SPMP_TurnLED(AP_LED_RUN, 1);
	SPMP_TurnLED(AP_LED_DATA, 1);

#ifndef WERS_USING_NETWORK
	WERSAPRestartNotifyPC();
#endif
	//added by cbwang
	g_ApInitOk = 1;


	//tmrPeriodFuncSet(TIMER0, WERSTimerISR, 40000);

	//initQueue(&g_WERSCmdQ);
	//initQueue(&g_WERSResponseQ);
	//memset(&g_WERSTranscations,0,sizeof(g_WERSTranscations));
	#ifndef WERS_DEBUG
	#if 0 // debug
	//SPMP_Config485ToSend();
	#else
	//SPMP_Config485ToReceive();
	#endif
	#endif

#ifdef RF_WATCK_DOG_FUNCTION_EN
	WatchDogInit();
	printf("WatchDogInit OK\n");
#endif
	// notify child node to reset.
	while(1)
	{
		g_ApTaskCondition = 0;
		if(SMPL_Ioctl(IOCTL_OBJ_MANAGE, IOCTL_ACT_CMD_RESET, NULL) == SMPL_SUCCESS)break;
		WERS_RandomDelay(0);
	}

	// main work loop
	while (1)
	{
	#ifndef WERS_DEBUG
	#if 0
		if ( SPMP_Get485LanStatus() )
		{
			SPMP_TurnLED(AP_LED_DATA, 1);
		}
		else
		{
			SPMP_TurnLED(AP_LED_DATA, 0);
		}
	#endif
	#endif
		
		// clear ap task condition flag, break watchdog to reset.
		g_ApTaskCondition = 0;


		if ( blink++ > 4 )
		{
			SPMP_TurnLED(AP_LED_RUN, 2);
			blink = 0;
		}

	#if 1
		// Wait for the Join semaphore to be set by the receipt of a Join frame from a
		// device that supports an End Device.
		//
		// An external method could be used as well. A button press could be connected
		// to an ISR and the ISR could set a semaphore that is checked by a function
		// call here, or a command shell running in support of a serial connection
		// could set a semaphore that is checked by a function call.
		//rf_printf_0("sem:%d peer:%d\n", sJoinSem, sNumCurrentPeers);
		if (sJoinSem && (sNumCurrentPeers < NUM_CONNECTIONS))
		{
			if (SMPL_SUCCESS == SMPL_LinkListen(&sLID[sNumCurrentPeers]))
			{
				BSP_ENTER_CRITICAL_SECTION(intState);
				sNumCurrentPeers++;
				sJoinSem--;
				BSP_EXIT_CRITICAL_SECTION(intState);

				rf_printf_0("\n.............SMPL_LinkListen OK!  LinkID: %d\n",sLID[sNumCurrentPeers]);

			}
		}

		//WERSTranscationProcess();

		//WERSPCPridge();
		//rf_hwMsWait(50);
		//#else
		//osTimeDly(10);

		//rf_hwMsWait(50);

		ResetTimer(g_timerID);
		osTimeDly(50/(1000/OS_TICKS_PER_SEC));
	#endif
		//printf_0("\n----RF RUNING----\n");
	}


}
#else
void WERSTimerISR(void)
{
	if( NULL != WERSQue )
	{
		if( 0 == WERSTime0flag )
		{
			WERSTime0flag = 1;
			osQuePost( WERSQue, (void*) (WERS_MSG_TIMER_0) );
		}
	}
	else
	{
		rf_printf_0(" ===WERSQue is NULL \n");
	}
}
void WERSExTask(void *arg)
{      
	int32_t timeID = -1;
	//int32_t timeID1 = -1;
	UINT32 rcount = 0;
//	UINT32 counter = 0;
	INT8U ret;
	UINT32 Msg;
	UINT32 blink = 0;
	//int32_t trycount = WERS_SEND_RELAY_NUMBER;
	//smplStatus_t linkret = WERS_NO_LINK;
	smplStatus_t linkret = SMPL_NO_LINK;
	struct DEV_ADDR stDevice;
	bspIState_t intState;
	int i;
	
	rf_printf_0(" ===WERSDiscover=====\n\n");
	SPMP_Config485ToSend();
	
	//g_pSemReceive = osSemCreate("Receive_Sem", 0);
	//g_485InQue =  osQueCreate("485InQue", (void *)g_485InQueBuf, WERS_485_IN_QUE_SIZE);
	//g_pTranscationQue  = osQueCreate("TranscationQue", (void *)TranscationQue, WERS_TRANSCATION_QUE_SIZE);	
	//g_RTSQue  = osQueCreate("WERSRTSQue", (void *)RTSQueBuf, WERS_RTS_QUE_SIZE);	
	g_485InQue =  osQueCreate((void *)g_485InQueBuf, WERS_485_IN_QUE_SIZE);
	g_pTranscationQue  = osQueCreate((void *)TransactionQue, WERS_TRANSCATION_QUE_SIZE);	
	g_RTSQue  = osQueCreate((void *)RTSQueBuf, WERS_RTS_QUE_SIZE);	
	
	osTaskCreate(
		//(UINT8*)"Receiver",
		WERSRFReceiverTask,
		(void *)0,
		(void *)(&gRFReceiverStack[STACKSIZE_RF_RECEIVER - 1]),
		OS_PRIO_WERS_RF_RECEIVER_TASK);
	
	osTaskCreate(
		//(UINT8*)"RFBridgeTask",
		WERSRFBridgeTask,
		(void *)0,
		(void *)(&gRFBridgeTaskStack[STACKSIZE_RF_BRDIGE_TASK- 1]),
		OS_PRIO_WERS_RF_BRIDGE_TASK);
	
	osTaskCreate(
		//(UINT8*)"485RecevieTask",
		WERS485RecevieTask,
		(void *)0,
		(void *)(&g485RecevieTaskStack[STACKSIZE_485_RECEVIE_TASK- 1]),
		OS_PRIO_WERS_485_RECEIVE_TASK);
	
	init_485_uart();
	ret = WERSReadDevice(&stDevice);
	if(SUCCESS != ret)
	{
		stDevice.hotel = 1;
		stDevice.host = 1;
		stDevice.dev = 999;
		stDevice.dev |= ((UINT16)RF_AP_TYPE_1)<<12;
	}
#if 0 //for debug
#if 0
	stDevice.hotel = 252;
	stDevice.host = 1;
	stDevice.dev = 866;
	stDevice.dev |= ((UINT16)RF_AP_TYPE_1)<<12;
#else
	stDevice.hotel = 201;
	stDevice.host = 30;
	stDevice.dev = 866;
	stDevice.dev |= ((UINT16)RF_AP_TYPE_1)<<12;
#endif
#endif

	g_WERSSystemId = stDevice.host;
	g_WERSSystemId <<= 8;
	g_WERSSystemId |= stDevice.hotel;
	// setting RF network node type
	g_MyNodeType = stDevice.dev >> 12;
	if(0 == g_MyNodeType || g_MyNodeType > RF_AP_TYPE_1 )
	{
		g_MyNodeType = RF_AP_TYPE_1;
	}

	printf_0("WERSReadDevice  ok  !! hotel:%x host:%x pda:%04x   g_MyNodeType: %d\n",stDevice.hotel, stDevice.host, stDevice.dev,g_MyNodeType);
	stDevice.dev &= 0x0FFF;
	stDevice.dev |= ((UINT16)g_MyNodeType)<<12;
	WERSSetDeviceAddress((WERSAddr_t*)&stDevice);

	printf_0("===init EX==111111===\nsystemID:%d    host: %d   hotel: %d\n\n",g_WERSSystemId,stDevice.host,stDevice.hotel);
	rf_printf_0(" ===WERSDiscover==111111===systemID:%d    host: %d   hotel: %d    deviceID: %04x \n\n",g_WERSSystemId,stDevice.host,stDevice.hotel,stDevice.dev);

	SPMP_TurnLED(AP_LED_RADIO, 0);
	SPMP_TurnLED(AP_LED_RUN, 0);
	SPMP_TurnLED(AP_LED_DATA, 0);

	nwk_ResetSet(sReset);

	// notify watchdog to change state
	gs_DeviceState = DEVICE_DISCONNECTED;
	g_ApTaskCondition = 0;
#ifdef RF_WATCK_DOG_FUNCTION_EN
	WatchDogInit();
	printf("WatchDogInit OK\n");
#endif

	rf_printf_0("==state 2===   g_ApTaskCondition: %d\n",g_ApTaskCondition);
	// reseted, notify child server in network to reset
	if ( RF_AP_TYPE_4 < g_MyNodeType )
	{
		rf_printf_0("==start SMPL_Init==\n");
		SMPL_Init(sCB);
		rf_printf_0("==End SMPL_Init==\n");
		do
		{
			g_ApTaskCondition = 0;
			rf_printf_0("==send reset....\n");
			ret = SMPL_Ioctl(IOCTL_OBJ_MANAGE, IOCTL_ACT_CMD_RESET, NULL);
			WERS_RandomDelay(0);
			//}while(SUCCESS != ret );
		}while(SMPL_SUCCESS != ret );
		
		rf_printf_0(" send reset signal, notify child server to reset!!\n");
	}
	
	rf_printf_0("==send reset end....\n");
	SPMP_Config485ToReceive();
	//SPMP_Config485ToSend();
	//WERSQue  = osQueCreate("WERSQue", (void *)WERSQueBuf, WERS_QUE_SIZE);	
	WERSQue  = osQueCreate((void *)WERSQueBuf, WERS_QUE_SIZE);	
	osQuePost( WERSQue, (void*) (DEVICE_JOIN) );

	g_NeedReset = 0;

	
	while (1)
	{
		//Msg = (UINT32)osQuePend(WERSQue, OS_NO_WAIT, &ret);
		Msg = (UINT32)osQueAccept(WERSQue,&ret);
		switch( Msg )
		{
		case WERS_MSG_TIMER_0:
			rf_printf_0(" Time_0\n");
			if ( DEVICE_CONNECTED == gs_DeviceState )
			{
				if ( !BackOffTimeOut() )
				{
					WERSTime0flag = 0;
					break;
				}
				rf_printf_0(" Ping!\n");
				if ( SMPL_SUCCESS != SMPL_Ping(gs_linkID) )
				{
					rcount++;
					rf_printf_0("\n==SMPL_Ping  faild  \n");
					if ( rcount >= 6 ) // 120s
					{
						////////////////////////////
						g_NeedReset = 1;
						////////////////////////////
						WERSTime0flag = 0;
						//osQuePost( WERSQue, (void*) (DEVICE_DISCONNECTED) );
					}
					else
					{
						WERSTime0flag = 0;
					}
				}
				else
				{
					rf_printf_0(" Ping Ok!\n");
					rcount = 0;
					WERSTime0flag = 0;
				}
				
			}
			else if ( DEVICE_DISCONNECTED== gs_DeviceState )
			{
				osQuePost( WERSQue, (void*)(DEVICE_DISCONNECTED) );
			}
			break;
#if 0
		case WERS_MSG_TIMER_1:
#if 0
			if(rcount)
				gs_DeviceSignlValue = 0;
			else
			{
				gs_DeviceSignlValue = (g_mrfi_rssi + 128) > 0 ? (g_mrfi_rssi + 128) : 0;
			}
#endif
			BroadcastProcess();
			break;
#endif
		case DEVICE_RESET:
			{
				//bspIState_t intState;

				//rf_printf_01("\n\n==sReset===\n");

				// notify child server to reset
				do
				{
					g_ApTaskCondition = 0;
					
					ret = SMPL_Ioctl(IOCTL_OBJ_MANAGE, IOCTL_ACT_CMD_RESET, NULL);
					WERS_RandomDelay(0);
				}while(SUCCESS != ret );

				sNumCurrentPeers = 0;
				sPeerFrameSem = 0;
				BSP_ENTER_CRITICAL_SECTION(intState);
				sJoinSem=0;
				BSP_EXIT_CRITICAL_SECTION(intState);
				memset(sLID, 0, sizeof(linkID_t) * NUM_CONNECTIONS);

				SMPL_Reset(sCB);

				osQuePost( WERSQue, (void*) (DEVICE_DISCONNECTED) );
			}

			break;
		case DEVICE_DISCONNECTED:
			rf_printf_0("\n==DEVICE_DISCONNECTED   \n");
			gs_DeviceState = DEVICE_DISCONNECTED;

			if( timeID >= 0 )
			{
				//tmrPeriodFuncDel(TIMER0,timeID);
				DeleteTimer(timeID);
				timeID = -1;
			}
			SMPL_LinkClose(gs_linkID);
			osQuePost( WERSQue, (void*) (DEVICE_JOIN) );
			break;

		case DEVICE_CONNECTED:
			rf_printf_0("\n==DEVICE_CONNECTED   \n");
			gs_DeviceState = DEVICE_CONNECTED;
			WERSTime0flag = 0;
			rcount = 0;
			//timeID = (int32_t)tmrPeriodFuncSet(TIMER0, WERSTimerISR, EX_PING_TIME_INTER);
			timeID = CreateTimerEx(WERSTimerISR,EX_PING_TIME_INTER,DSTIME_MODE_CYCLE);
			StartTimer(timeID);	
			break;

		case DEVICE_JOIN:	
			gs_DeviceState = DEVICE_JOIN;
			/*init and link WERS*/
			
			rf_printf_0("\n====WERS ED Init=====\n");
			
			/* Keep trying to join until successful. Toggle LEDs to indicate that
			* joining has not occurred.
			*/
			do
			{
				while (WERS_RES_OK != SMPL_Init(sCB))
				{
					g_ApTaskCondition = 0;
					osTimeDly(50);
					rf_printf_0("\n====SMPL_Init=====\n");
					//rf_hwMsWait( 500);
				};
				rf_printf_0("\n======join ok!!! =====\n\n\n");
				rf_printf_0("\n======start Link =====\n");
				/* Keep trying to link...*/
				for(i = 0; i < 10; i++)
				{
					g_ApTaskCondition = 0;
					linkret =  SMPL_Link(&gs_linkID);
					if (WERS_RES_OK != linkret)
					{
						osTimeDly(50);
						//rf_hwMsWait( 500);
					}
					else
					{
						break;
					}
				}
				//if( WERS_RES_OK != linkret  )
				//{
				//	rf_hwMsWait(1000);
				//}
			}while( WERS_RES_OK != linkret );
			/*end*/

			//rf_printf_0("			>>>>>>>>>>>>init linkid %d \n\n",gs_linkID);
			printf_0("			>>>>>>>>>>>>init linkid %d \n\n",gs_linkID);
			BSP_ENTER_CRITICAL_SECTION(intState);
			sNumCurrentPeers = 0;
			sLID[sNumCurrentPeers] = gs_linkID;
			sNumCurrentPeers++;
			BSP_EXIT_CRITICAL_SECTION(intState);

			osQuePost( WERSQue, (void*) (DEVICE_CONNECTED) );
			break;

		default:
			if ( DEVICE_CONNECTED == gs_DeviceState )
			{
				if ( blink++ > 4 )
				{
					SPMP_TurnLED(AP_LED_RUN, 2);
					blink = 0;	
				}

				// clear ap task condition flag, break watchdog to reset.
				g_ApTaskCondition = 0;
				
				WERSExWorking();
			}
			break;
		}

		// clear ap task condition flag, break watchdog to reset.
		g_ApTaskCondition = 0;
		
		//rf_hwMsWait(50);
		osTimeDly(5);
		//rf_hwMsWait(50);
	}
}


static void WERSExWorking(void)
{
	bspIState_t intState;
//	UINT32 osQuePendRet = OS_NO_ERR;
//	UINT8 *pMsg;
	
	// Wait for the Join semaphore to be set by the receipt of a Join frame from a
	// device that supports an End Device.
	//
	// An external method could be used as well. A button press could be connected
	// to an ISR and the ISR could set a semaphore that is checked by a function
	// call here, or a command shell running in support of a serial connection
	// could set a semaphore that is checked by a function call.
	//rf_printf_0("sem:%d peer:%d\n", sJoinSem, sNumCurrentPeers);
	if (sJoinSem && (sNumCurrentPeers < NUM_CONNECTIONS))
	{
		if (SMPL_SUCCESS == SMPL_LinkListen(&sLID[sNumCurrentPeers]))
		{
			//rf_printf_0("......LinkListen ok\n");
			rf_printf_0("@@@@==SMPL_LinkListen  link id: %d\n",sLID[sNumCurrentPeers]);
			BSP_ENTER_CRITICAL_SECTION(intState);
			sNumCurrentPeers++;
			sJoinSem--;
			BSP_EXIT_CRITICAL_SECTION(intState);
		}
	}
}

static void WERSRFBridgeTask(void *arg)
{
//	bspIState_t intState;
	INT8U osQuePendRet = OS_NO_ERR;
	UINT8 *pMsg;

	while(1)
	{
		// transcatino poll and process
		pMsg = (UINT8*)osQuePend(g_pTranscationQue, OS_WAIT_FOREVER, &osQuePendRet);
		if ( OS_NO_ERR == osQuePendRet && NULL != pMsg )
		{
			WERSContainerStruct *pContainer = (WERSContainerStruct*)(pMsg);

			SPMP_TurnLED(AP_LED_DATA, 2);

			//check timestamp. 
			if ( WERSMsTimeGet() > (pContainer->GUID + TRANSCATION_TIME_OUT) )
			{
				// hold long time,drop it, invaild.
			}
			else
			{
				pContainer->GUID = WERSGenerateGUID();

				rf_printf_0("\n===get que msg (linkID:%d, GUID:%d )\n",pContainer->Prot,pContainer->GUID);

				if ( pContainer->Prot == gs_linkID )
				{
					// from the top AP, foward it to leaf node
					rf_printf_0("===from the top AP\n");
					
					if ( g_MyNodeType <= WERS_ROUTTABLE_COUNT )
					{
						//g_MyNodeType
						UINT8 toLinkID = pContainer->RouteMap[g_MyNodeType-1];

						rf_printf_0("===####  foward it to leaf node=====link id:%d    toLinkID: %d \n",pContainer->Prot,toLinkID);

						WERS_Transmit( toLinkID, pMsg, (UINT32)pContainer->ContainerLength);
					}
				}
				else
				{
					// from the leaf node, foward it to top AP
					rf_printf_0("===&&&&  foward data to top AP =====link id:%d\n",gs_linkID);
#if 0 //debug
					if ( pContainer->Code ==  12 )
					{
						UINT8 data[1024];
						data[0] = 0;
						UINT8* pPlayLoadData = (UINT8*)(pContainer + 1);
						UINT32 responselen = READ16(pPlayLoadData +6);
						if ( SMPL_SUCCESS == WERSSendResponseForAP(pContainer->Prot, pContainer->SystemID,pContainer->TransID,pContainer->Code, data, responselen+1) )
						{
							rf_printf_0(">send ok!!%d \n\n\n",responselen);
						}
						else
						{
							rf_printf_0(">>response fail!! %d \n\n\n",responselen);
						}
					}
#else
					WERS_Transmit( gs_linkID, pMsg, (UINT32)pContainer->ContainerLength);
#endif
				}
			}
			if ( NULL != pMsg )
			{
				rf_free(pMsg);
				pMsg = NULL;
			}
		}
	}
}
#endif

void WERSPCDataProcess(void *pData,UINT16 dataLen)
{	
	uint8_t *pResponseData;
	uint16_t responseDataLen;
	WERSContainerStruct *pContainer;
	linkID_t lid;

	rf_printf_0("====WERSPCDataProcess \n");

	//g_TCPPingCount=0;

	if (  NULL != pData && dataLen > 0)
	{
		pResponseData = pData;
		responseDataLen = dataLen;
		pContainer =  (WERSContainerStruct*)pResponseData;

		/* data error check */
		if( pContainer->ContainerLength >= WERS_CONTAINER_SIZE 
			//&&WERS_TYPE(pContainer->ContainerType) == WERS_RESPONSE_BLOCK
			&& pContainer->ContainerLength == responseDataLen )
		{
			if(WERS_COMMAND_BLOCK == WERS_TYPE(pContainer->ContainerType) )
			{
				if( RF_SUCCESS == WERSAPCmd(pContainer) )
				{
					return;
				}
			}
			else if ( WERS_EVENT_BLOCK == WERS_TYPE(pContainer->ContainerType) )
			{
				ioctlRawSend_t    send;
				smplStatus_t ret = SMPL_SUCCESS;
				int32_t trycount = WERS_SEND_RELAY_NUMBER;
				connInfo_t  *pCInfo = NULL;

				uint8_t  radioState = MRFI_GetRadioState();
				lid = pContainer->Prot;
				pCInfo = nwk_getConnInfo(lid);
				if ( pCInfo )
				{
					send.addr = (addr_t *)pCInfo->peerAddr;
					//send.addr =(addr_t *)nwk_getBCastAddress();
					send.msg  = pResponseData;
					send.len  = responseDataLen;
					send.port = SMPL_PORT_CALL;


					do{
						ret = SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_WRITE, &send);
						if( ret == SMPL_SUCCESS )
						{
							NWK_CHECK_FOR_RESTORE_IDLE();
							break;
						}
						else
						{
							WERS_RandomDelay(0);
						}
						trycount--;
						rf_printf_1(" send one packte   ret: %d\n",ret);
						
					}while( trycount > 0 );
				}
			}
#ifdef WERS_USING_NETWORK
			else if( WERS_CONTROL_BLOCK== WERS_TYPE(pContainer->ContainerType) )
			{
				//if(WERS_CONTROL_PINGRESPONSE== (pContainer->Code))
				//	g_TCPPingCount=0;
				return;
			}
#endif
			else
			{
				//SPMP_TurnLED(AP_LED_DATA, 2);
				pContainer->GUID = WERSGenerate_RF_GUID();
				lid = pContainer->Prot;
#ifdef RUN_IN_INTER_MEM
				//OSTimeDly(1);
				//MsDelay(10);
//				rf_hwMsWait(5);
#endif
				WERS_Transmit(lid, pResponseData, responseDataLen);
			
			}
			rf_printf_0("==========transmit data ok \n");
		}
		else
		{
			rf_printf_0("==========not a response data from PC: \n");
		}
	}
}

#ifndef WERS_DEBUG
#ifndef WERS_USING_NETWORK
void WERS485RecevieTask(void *arg)
{
	INT8U osQuePendRet = OS_NO_ERR;
	UINT8 *pMsg;
	//UINT32 err;
	//bspIState_t intState = 0;
	//SINT32 tryCount = 10;
#if 0// debug
	while(1)
	{
		pMsg = (UINT8*)osQuePend(WERSQue, OS_WAIT_FOREVER, &osQuePendRet);
		if ( OS_NO_ERR == osQuePendRet && NULL != pMsg )
		{		
			WERSContainerStruct *pContainer = (WERSContainerStruct*)pMsg;

			//check timestamp. 
			if ( WERSMsTimeGet() > (pContainer->GUID + TRANSCATION_TIME_OUT) )
			{
				// hold long time,drop it, invaild.
				rf_printf_0("hold long time, invaild!\n");
			}
			else
			{
				pContainer->GUID =  WERSMsTimeGet();
				
				rf_printf_0("\nGet que msg (linkID:%d, TransID:%d,GUID:%d )\n",pContainer->Prot,pContainer->TransID ,pContainer->GUID);
				if ( pContainer->Code ==  12 )
				{
					UINT8 data[1024];
					data[0] = 0;
					UINT8* pPlayLoadData = (UINT8*)(pMsg+ sizeof(WERSContainerStruct));
					UINT32 responselen = READ16(pPlayLoadData +6);	

					//simulation pc process time.
					rf_hwMsWait(200); 
					//////////////////////////////////

					
					if ( SMPL_SUCCESS == WERSSendResponseForAP(pContainer,data, responselen+1) )
					{
						rf_printf_0(">send ok!!%d \n\n\n",responselen);
					}
					else
					{
						rf_printf_0(">>response fail!! %d \n\n\n",responselen);
					}
				}
			}
			rf_free(pMsg);
			pMsg = NULL;
		}
	}
#else

#if 0
	while(1)
	{
		pMsg = (UINT8*)osQuePend(g_485InQue, OS_WAIT_FOREVER, &osQuePendRet);
		if ( OS_NO_ERR == osQuePendRet && NULL != pMsg )
		{
			WERS485SigHeader *pSig = (WERS485SigHeader *)pMsg;

			// paser action
			if ( ACTION_RECEIVE_FROM_PC == pSig->Cmd )
			{
				WERS485FrameHeader *p485FrameHeader = (WERS485FrameHeader*)(pMsg + sizeof(WERS485SigHeader));
				
				switch( p485FrameHeader->ContainerType )
				{
					case FRAME_TYPE_485_DATA:
					{
						UINT8 *pPlayload = (UINT8 *)(pMsg + sizeof(WERS485SigHeader) + sizeof(WERS485FrameHeader));
						WERSContainerStruct *pContainer = (WERSContainerStruct *)pPlayload;

						//check timestamp. 
						if ( WERSMsTimeGet() > (pContainer->GUID + TRANSCATION_TIME_OUT) )
						{
							// hold long time,drop it, invaild.
							//rf_printf_0("hold long time, invaild!\n");
						}
						else
						{
							WERSPCDataProcess(pPlayload,pContainer->ContainerLength);
						}
						
						break;
					}
					default:
						break;
				}
			}
			rf_free(pMsg);
			pMsg = NULL;
		}
	}
#else
	while(1)
	{
		pMsg = (UINT8*)osQuePend(g_485InQue, OS_WAIT_FOREVER, &osQuePendRet);
		if ( OS_NO_ERR == osQuePendRet && NULL != pMsg )
		{
			WERS485FrameHeader *p485FrameHeader = (WERS485FrameHeader*)(pMsg);
			
			switch( p485FrameHeader->ContainerType )
			{
				case FRAME_TYPE_485_DATA:
				{
					UINT8 *pPlayload = (UINT8 *)(p485FrameHeader + 1);
					WERSContainerStruct *pContainer = (WERSContainerStruct *)pPlayload;

					//check timestamp.
					if ( WERSMsTimeGet() > (pContainer->GUID + TRANSCATION_TIME_OUT) )
					{
						// hold long time,drop it, invaild.
						//rf_printf_0("hold long time, invaild!\n");
					}
					else
					{
						// process 
						WERSPCDataProcess(pPlayload,pContainer->ContainerLength);
					}
					break;
				}
				default:
					break;
			}
			
			rf_free(pMsg);
			pMsg = NULL;
		}
	}

#endif
#endif	
}

void WERS485SendResponsePoll(void)
{
	WERS485FrameHeader	frame;

	frame.ContainerLength = sizeof(WERS485FrameHeader);
	frame.DataCRC =  0;
	frame.ContainerType = FRAME_TYPE_485_REQUST;
	//frame.HeaderCRC = cal_crc8((const UINT8 *)&frame,(sizeof(WERS485FrameHeader) -1 ));
	//send
	SendDataToPC((UINT8*)&frame, sizeof(WERS485FrameHeader));
}

UINT32 WERS485SendDataAck(UINT32 GUID)
{
	WERS485DataAck dataack;

	dataack.ContainerLength = sizeof(WERS485DataAck);
	dataack.ContainerType = FRAME_TYPE_485_COMMAND_DATA_ACK;
	dataack.DataCRC =  cal_crc((const UINT8 *)&GUID,sizeof(GUID));
	dataack.HeaderCRC = cal_crc8((const UINT8 *)&dataack,(sizeof(WERS485FrameHeader) -1 ));

	dataack.GUID = GUID;

	//send
	SendDataToPC((UINT8*)&dataack, sizeof(WERS485DataAck));

	return SMPL_SUCCESS;	
}

#ifdef WERS_485_AP_HOST
#if 0
BOOL Check485DataComing(UINT32 timeout,UINT32 *err)
{
	UINT32 i;
	UINT32 ms = 0;
	if ( OS_NO_WAIT == timeout || 0 == timeout )
	{
		//if ( !_uartRxFifoIsEmpty(WERS_PC_CONNECT_PORT) )
		if ( hwUartRXDataCount(WERS_PC_CONNECT_PORT) > 0  || (!(_uartRxFifoIsEmpty(WERS_PC_CONNECT_PORT))) )
		{
			*err = OS_NO_ERR;
			return TRUE;
		}
	}
	else
	{
		ms = timeout;
		for( i = 0; i < ms; i++ )
		{
			//if ( !_uartRxFifoIsEmpty(WERS_PC_CONNECT_PORT) )
			if ( ( hwUartRXDataCount(WERS_PC_CONNECT_PORT) > 0 ) || (!(_uartRxFifoIsEmpty(WERS_PC_CONNECT_PORT))) )
			{
				*err = OS_NO_ERR;
				return TRUE;
			}
			//for watch dog, can't remove
			g_ApTaskCondition = 0;
			//hwWait(0, 1);
			//MsDelay(1);
			mDelay(1);
			//rf_hwMsWait(1);
		}
	}
	*err = OS_TIMEOUT;
	return FALSE;
}
#endif

void WERS_SaveGUIDToMap(UINT32 GUID)
{
	bspIState_t intState;
	BSP_ENTER_CRITICAL_SECTION(intState);

	if ( sNumGUIDforPCEmenu >= NUM_GUID_FOR_PC_EMENU )
	{
		sNumGUIDforPCEmenu--;
		memcpy(sGUIDMaps,&sGUIDMaps[1],sizeof(UINT32)*sNumGUIDforPCEmenu);
	}
	sGUIDMaps[sNumGUIDforPCEmenu] = GUID;
	sNumGUIDforPCEmenu++;
	
	BSP_EXIT_CRITICAL_SECTION(intState);
}

void WERS_RemoveGUIDFromMap(UINT32 GUID)
{
	bspIState_t intState;
	UINT32 i,j;

	for( i = 0 ; i < sNumGUIDforPCEmenu; i++)
	{
		BSP_ENTER_CRITICAL_SECTION(intState);
		
		if ( GUID == sGUIDMaps[i] )
		{
			sNumGUIDforPCEmenu--;
			
			for( j = i; j < sNumGUIDforPCEmenu;j++)
			{
				sGUIDMaps[j] = sGUIDMaps[j+1];
			}
			
			BSP_EXIT_CRITICAL_SECTION(intState);
			
			break;
		}
		else
		{
			BSP_EXIT_CRITICAL_SECTION(intState);
		}
	}
}


void WERS_RemoveTimeOutGUIDFromMap(UINT32 timeOut)
{
	bspIState_t intState;
	UINT32 i,invalidNumber = 0;

	BSP_ENTER_CRITICAL_SECTION(intState);
	
	for( i = 0 ; i < sNumGUIDforPCEmenu; i++)
	{
		if ( WERSMsTimeGet() > (sGUIDMaps[i] + timeOut) )
		{
			invalidNumber++;
		}
	}
	sNumGUIDforPCEmenu -=invalidNumber;
	
	BSP_EXIT_CRITICAL_SECTION(intState);
}

void WERS485Poll(void *arg)
{
	INT8U err;

//	irqIsrReg(IRQ_NUM_DRAM, (void*)DMA_485_Data_Coming_Int_Handler);

	while(1)
	{
		osSemPend(g_p485PollSem, OS_WAIT_FOREVER, &err);

		// check need poll
		WERS_RemoveTimeOutGUIDFromMap(MAX_TIME_OUT_TO_DROP_PACKATE);
		
		if ( sNumGUIDforPCEmenu )
		{
			// clear data receice ok sem.
			do{
				//osSemPend(g_pDataRecevieOkSem, OS_NO_WAIT, &err);
				osSemAccept(g_pDataRecevieOkSem);
			}while( OS_NO_ERR == err );

			WERS485SendResponsePoll();

			//enable Uart Rx DMA interupt.
			//hwUartRXMoreThanIntEn(WERS_PC_CONNECT_PORT,1);
			// wait 50ms for check data comming	
			osSemPend(g_pDataComingSem, 5, &err);
			//if ( Check485DataComing(200,&err) )
			if ( OS_NO_ERR == err )
			{
				//SPMP_TurnLED(AP_LED_DATA, 2);
				//max wait 1500ms to 485 receive data .
				osSemPend(g_pDataRecevieOkSem, 160, &err);
			}
			else
			{
				// no data comming

				//resend poll after wait 200ms
				//osTimeDly(10);
			}
			osSemPost(g_p485PollSem);
		}		
	}
}


void WERS485BridgeTask(void *arg)
{	
	INT8U err;
	WERSFramInfo_t *pTranscation;
	bspIState_t intState = 0;
	UINT32 GUID = 0;
	WERSContainerStruct *pContainer = NULL;
//	UINT32 osQuePendRet = OS_NO_ERR;
//	UINT8 *pMsg;

//	irqIsrReg(IRQ_NUM_DRAM, (void*)DMA_485_Data_Coming_Int_Handler);

	while(1)
	{
		osSemPend(g_pSem485Bridge, OS_WAIT_FOREVER, &err);

		g_ApTaskCondition = 0;
		
		SPMP_TurnLED(AP_LED_RUN, 2);
		//SPMP_TurnLED(AP_LED_DATA, 2);
		
		if ( (pTranscation = WERS_GetOldestTranscation() ) != NULL )
		{
			if ( pTranscation->pPacketData && pTranscation->packetSize > 0 )
			{
				#ifndef WERS_USING_NETWORK
				WERS485Send(FRAME_TYPE_485_DATA, (UINT8*)pTranscation->pPacketData,  pTranscation->packetSize);

				pTranscation->fi_usage = WERS_FI_INUSE_UNTIL_DEL;
				
				//wait 200ms for data ack
				//osSemPend(g_p485DataAckSem, 20, &err);
				osSemPend(g_p485DataAckSem, 20, &err);
				if ( OS_NO_ERR == err )
				{
					BSP_ENTER_CRITICAL_SECTION(intState);
					GUID = g_485AckGUID;
					g_485AckGUID =0;
					BSP_EXIT_CRITICAL_SECTION(intState);


					pContainer = (WERSContainerStruct*)pTranscation->pPacketData;
					if ( GUID == pContainer->GUID )
					{
						if ( WERS_COMMAND_BLOCK == WERS_TYPE(pContainer->ContainerType) )
						{
							WERS_SaveGUIDToMap(GUID);
							osSemPost(g_p485PollSem);
						}
						
						WERS_RemoveTranscationFromQ(pTranscation);
					}
					else
					{
						WERS_RemoveTranscationFromQByGUID(GUID);
						osSemPost(g_pSem485Bridge);
					}
				}
				else
				{
					//time out, next to resend
					osSemPost(g_pSem485Bridge);
				}
				#else
				SendDataToPC((UINT8*)pTranscation->pPacketData,  pTranscation->packetSize);
				#endif
			}
			else
			{
				// erro,  free it.
				WERS_RemoveTranscationFromQ(pTranscation);

				// to check next.
				osSemPost(g_pSem485Bridge);
			}
		}
		else
		{
			// que is empt.
		}

		#if 0
		// check need poll
		WERS_RemoveTimeOutGUIDFromMap(MAX_TIME_OUT_TO_DROP_PACKATE);
		
		if ( sNumGUIDforPCEmenu )
		{
			// clear data receice ok sem.
			do{
				//osSemPend(g_pDataRecevieOkSem, OS_NO_WAIT, &err);
				osSemAccept(g_pDataRecevieOkSem);
			}while( OS_NO_ERR == err );

		
			WERS485SendResponsePoll();

			//enable Uart Rx DMA interupt.
			hwUartRXMoreThanIntEn(WERS_PC_CONNECT_PORT,1);
			// wait 50ms for check data comming	
			osSemPend(g_pDataComingSem, 5, &err);
			//if ( Check485DataComing(200,&err) )
			if ( OS_NO_ERR == err )
			{
				//SPMP_TurnLED(AP_LED_DATA, 2);
				//max wait 1500ms to 485 receive data .
				osSemPend(g_pDataRecevieOkSem, 160, &err);
			}
			else
			{
				// no data comming

				//resend poll after wait 200ms
				//osTimeDly(10);
			}

			osSemPost(g_pSem485Bridge);
		}		
		#endif
	}
}
#endif
#if 0
void DMA_485_Data_Coming_Int_Handler(void)
{
	if( hwUartRXMoreThanIntSts(WERS_PC_CONNECT_PORT) == 0x11)
	{		
		//SPMP_TurnLED(AP_LED_DATA, 2);
		osSemPost(g_pDataComingSem);		
		hwUartRXMoreThanIntEn(WERS_PC_CONNECT_PORT,0);		
	}
}
#endif


UINT32 PC485SemPend(UINT32 timeout,UINT32 *err)
{
	UINT32 i;
	UINT32 ms = 0;

	if ( OS_NO_WAIT == timeout || 0 == timeout )
	{
		if ( 0 != g_485NotifyCode  )
		{
			*err = OS_NO_ERR;
			return g_485NotifyCode;
		}
	}
	else
	{
		ms = timeout;
		for( i = 0; i < ms; i++ )
		{
			if ( 0 != g_485NotifyCode  )
			{
				*err = OS_NO_ERR;
				return g_485NotifyCode;
			}
			rf_hwMsWait(1);
		}
	}
	
	*err = OS_TIMEOUT;
	return 0;
}

void WERS485SendTask(void *arg)
{
	INT8U osQuePendRet = OS_NO_ERR;
	UINT8 *pMsg;
	UINT32 err;
	bspIState_t intState = 0;
	SINT32 tryCount = TRY_COUNT_FOR_485_SEND;
	UINT32 wait485SemTimeout = 0;
	BOOL getCTSandSendData = FALSE;

	while(1)
	{
		pMsg = (UINT8*)osQuePend(g_485OutQue, OS_WAIT_FOREVER, &osQuePendRet);
		if ( OS_NO_ERR == osQuePendRet && NULL != pMsg )
		{
			WERS485SigHeader *pSig = (WERS485SigHeader *)pMsg;

			// paser action
			if ( ACTION_SEND_TO_PC == pSig->Cmd )
			{
				UINT8 *pPlayload = (UINT8 *)(pMsg + sizeof(WERS485SigHeader));
				WERSContainerStruct *pContainer = (WERSContainerStruct *)pPlayload;

				// pack RTS playload data
				WERS485RTSPlayload RTSPlayLoad;

				pContainer->GUID = tmrDateTimeGet() + tmrMsTimeGet();
				
				RTSPlayLoad.linkID = pContainer->Prot;
				RTSPlayLoad.code   = pContainer->Code;
				RTSPlayLoad.GUID = pContainer->GUID;
				RTSPlayLoad.dataSize = pContainer->ContainerLength + sizeof(WERS485FrameHeader);
				//memcpy(&RTSPlayLoad.GUID,  &pContainer->GUID, sizeof( pContainer->GUID));
				//memcpy(&RTSPlayLoad.dataSize,  &pContainer->ContainerLength, sizeof( pContainer->ContainerLength));

				
				if ( E_485_RECEIVE == g_485CurrentStates )
				{
					while(!F_485_BackOffTimeOut());
				}
				BSP_ENTER_CRITICAL_SECTION(intState);
				Change485CurrentState(E_485_REQUEST);
				g_485NotifyCode = 0;
				BSP_EXIT_CRITICAL_SECTION(intState);



				tryCount = TRY_COUNT_FOR_485_SEND;
				getCTSandSendData = FALSE;


				
				while(1)
				{
					WERS485Send(FRAME_TYPE_485_COMMAND_RTS,(UINT8*)&RTSPlayLoad,sizeof(RTSPlayLoad));
					tryCount--;
					 
					wait485SemTimeout = 500;// 150 ms
_LoopLable_1:
					// wait receiver signal
					PC485SemPend(wait485SemTimeout,&err);
					if ( OS_TIMEOUT == err )
					{
						if ( tryCount <= 0 )
						{
							//cancel .....
							break;
						}

						WERS_RandomDelay(0);
						////////////////////////////////////////////
						if ( 0 == g_485NotifyCode )
						{
							// no notify code coming during in backoff time, continue send RTS PC.
						}
						else
						{
							// got notify code during in backoff time ,goto check.
							wait485SemTimeout  = OS_NO_WAIT;
							goto _LoopLable_1;
						}
					}
					else
					{
						BSP_ENTER_CRITICAL_SECTION(intState);
						if ( FRAME_TYPE_485_COMMAND_CTS == g_485NotifyCode )
						{
							g_485NotifyCode = 0;
							BSP_EXIT_CRITICAL_SECTION(intState);

							// start to send
							WERS485Send(FRAME_TYPE_485_DATA,pPlayload,pContainer->ContainerLength);

							getCTSandSendData = TRUE;
							wait485SemTimeout = 600; // 400ms
							
							goto _LoopLable_1;
						}
						else if ( FRAME_TYPE_485_COMMAND_DATA_ACK == g_485NotifyCode )
						{
							g_485NotifyCode = 0;
							BSP_EXIT_CRITICAL_SECTION(intState);

							if ( getCTSandSendData )
							{
								// send data ok!, break
								break;
							}
							else
							{
								// phase error!  continue, can't break!
							}
						}
						else  if ( FRAME_TYPE_485_COMMAND_BTS == g_485NotifyCode 
								|| FRAME_TYPE_485_COMMAND_RTS == g_485NotifyCode )
						{
							g_485NotifyCode = 0;
							Change485CurrentState(E_485_IDLE);
							BSP_EXIT_CRITICAL_SECTION(intState);

							//backoff, suspend current task.
							osTaskSuspend(OS_PRIO_WERS_485_SEND_TASK);

							Change485CurrentState(E_485_REQUEST);
							//tryCount = TRY_COUNT_FOR_485_SEND;
						}
						else
						{
							BSP_EXIT_CRITICAL_SECTION(intState);
						}
					}
				}
				
				BSP_ENTER_CRITICAL_SECTION(intState);
				g_485NotifyCode = 0;
				Change485CurrentState(E_485_IDLE);
				BSP_EXIT_CRITICAL_SECTION(intState);
				
			}
			
			rf_free(pMsg);
			pMsg = NULL;
		}
	}
}
#else		/*WERS_USING_NETWORK*/

void SendApInfo(void)
{
	UINT8 data[32];
	WERSContainerStruct *pResponse = (WERSContainerStruct *)data;
	struct DEV_ADDR stDevice;
	UINT8 *pVersion;

	pResponse->ContainerLength = sizeof(WERSContainerStruct) + sizeof(UINT16);
	pResponse->ContainerType = WERS_RESPONSE_BLOCK;
	pResponse->SystemID = g_WERSSystemId;
	pResponse->Code = COM_CMD_APGET;

	//WERSGetDeviceAddress( (WERSAddr_t*)&stDevice );
	WERSReadDevice( &stDevice );

	stDevice.dev &= 0x0FFF;
	//pResponse->Prot = RF_HOST_AP;
	pResponse->Prot = g_MyNodeType;
#ifdef WERS_EXTENDER_DEVICE
	if(DEVICE_CONNECTED == gs_DeviceState)
		pResponse->TransID = (UINT8)g_mrfi_rssi;
	else
		pResponse->TransID = 0;
#else
	pResponse->TransID = (UINT8)g_mrfi_rssi;
#endif

	memcpy(pResponse->RouteMap, &stDevice, sizeof(stDevice));
	//pack version
	pVersion = (UINT8*)(pResponse + 1);
	pVersion[0] = MAIN_VERSION;
	pVersion[1] = SUB_VERSION;
	SendDataToPC((UINT8*)pResponse,pResponse->ContainerLength);
}

#define SUPPORT_DATA_CON_ACK

#ifdef SUPPORT_DATA_CON_ACK

#define WERS_DATA_CON_ACK_MAX	8

#define WERS_DATA_CON_ACK_BEGIN		0
#define WERS_DATA_CON_ACK_ADD		1
#define WERS_DATA_CON_ACK_FLUSH		2
#define WERS_DATA_CON_ACK_INIT		3
void WERSDataContainerAck(UINT32 GUID, UINT32 ackcmd)
{
	static char ackbuf[sizeof(WERSContainerStruct)+WERS_DATA_CON_ACK_MAX*sizeof(UINT32)];
	static WERSContainerStruct * const pWERSSend = (WERSContainerStruct *)ackbuf;
	static UINT32 * pGUID;
	static UINT32 ack_count;
//	UINT32 currentGUID;
	
	if(ackcmd == WERS_DATA_CON_ACK_BEGIN)
	{
		pGUID = (UINT32 *)(ackbuf + sizeof(WERSContainerStruct));
		ack_count = 0;
		pWERSSend->ContainerLength = sizeof(WERSContainerStruct);
	}
	else if(ackcmd == WERS_DATA_CON_ACK_ADD)
	{
		if(GUID)
		{
			*pGUID++ = GUID;
			ack_count ++;
			pWERSSend->ContainerLength += sizeof(UINT32);
			if(ack_count >= 8)
			{
				SendDataToPC((UINT8 *)pWERSSend, pWERSSend->ContainerLength);
				pGUID = (UINT32 *)(ackbuf + sizeof(WERSContainerStruct));
				ack_count = 0;
				pWERSSend->ContainerLength = sizeof(WERSContainerStruct);
			}
		}
	}
	else if(ackcmd == WERS_DATA_CON_ACK_FLUSH)
	{
		if(pWERSSend->ContainerLength > sizeof(WERSContainerStruct))
		{
			SendDataToPC((UINT8*)pWERSSend, pWERSSend->ContainerLength);
			pGUID = (UINT32 *)(ackbuf + sizeof(WERSContainerStruct));
			ack_count = 0;
			pWERSSend->ContainerLength = sizeof(WERSContainerStruct);
		}
	}
	else if(ackcmd == WERS_DATA_CON_ACK_INIT)
	{
		memset(pWERSSend, 0, sizeof(WERSContainerStruct));
		pWERSSend->ContainerType = WERS_DATA_BLOCK;
		pWERSSend->Code = WERS_DATA_CONTAINER_ACK;
	}
}

#else

#define WERSDataContainerAck(...)

#endif

int32_t net_led;
void ledoff(void)
{
	SPMP_TurnLED(AP_LED_DATA, 1);
}
#include "lwip\sockets.h"
int sock_reset(void);
#define WERS_SOCK_REC_BUF	4096
//#define WERS_SOCK_REC_BUF	6144
//#define WERS_SOCK_REC_BUF	(8192 + 512)
void WERS_TCPRecevieTask(void *arg)
{
	int sockfd=(int)arg;
	int len;
	static char receivebuf[WERS_SOCK_REC_BUF];
	int start = 0, data_len = 0;
	char *pProccess;
#ifdef SUPPORT_DATA_CON_ACK
	UINT32 GUID;
#endif
	//int count = 0;

	WERSDataContainerAck(0, WERS_DATA_CON_ACK_INIT);
	net_led=CreateTimerEx(ledoff, 50 ,DSTIME_MODE_ONCE);
	SendApInfo();
	while(1)
	{
		len=lwip_recv(sockfd, receivebuf+start, WERS_SOCK_REC_BUF-start, 0);
		if(len>0)
		{
			//count++;
			SPMP_TurnLED(AP_LED_DATA, 2);
			ResetTimer(net_led);	
			if(start==0 || start==1)
			{
				if(len + start > 1)data_len=*((unsigned short*)receivebuf);
			}
			if(data_len<=WERS_SOCK_REC_BUF)
			{
				sock_ping_stop();
				start += len;
				if(start >= data_len)
				{
					//Dump(receivebuf,len);
					//Dump(receivebuf,data_len);
					//printf("C:%d\n",count);
					//count=0;
					pProccess = receivebuf;
					WERSDataContainerAck(0, WERS_DATA_CON_ACK_BEGIN);
					do{
#ifdef SUPPORT_DATA_CON_ACK
						if(WERS_TYPE(((WERSContainerStruct*)pProccess)->ContainerType) == WERS_DATA_BLOCK)
							GUID = ((WERSContainerStruct*)pProccess)->GUID;
						else
							GUID = 0;
#endif
						WERSPCDataProcess(pProccess,data_len);
						WERSDataContainerAck(GUID, WERS_DATA_CON_ACK_ADD);
						start -= data_len;
						pProccess += data_len;
						if(start > 1)data_len = *((unsigned short*)pProccess);
						else break;
					}while(data_len && data_len <= start);
					WERSDataContainerAck(0, WERS_DATA_CON_ACK_FLUSH);
					if(start)
					{
						memcpy(receivebuf,pProccess,start);
					}
				}
				sock_ping_reset();
			}
			else
			{
				start = 0;
				sockfd=sock_reset();
			}
			//SPMP_TurnLED(AP_LED_DATA, 1);
		}
		else if(len==0)
		{
			start = 0;
			sockfd=sock_reset();
			SendApInfo();
			//OSTimeDly(OS_TICKS_PER_SEC / 2);
		}
	}
}

void WERS_TCPSendPing(void)
{
	static const WERSContainerStruct WERS_PING_PACK={
		sizeof(WERSContainerStruct),
		WERS_CONTROL_BLOCK,
		WERS_CONTROL_PING,
		0,0,0,
		{0,0,0,0},
		0,
	};
	SendDataToPC((uint8_t *)(&WERS_PING_PACK),sizeof(WERSContainerStruct));
}
#endif		/*WERS_USING_NETWORK*/
#endif

unsigned char cal_crc8(const unsigned char * ptr, int len)
{
	unsigned char crc = 0;

	while (len-- > 0)
		crc = crc8_ta[crc ^ *ptr++];

	return crc;
}

#if 0
unsigned short cal_crc(const unsigned char * ptr, int len)
{
	unsigned char crc[2];
	unsigned char i = 0x00;

	crc[0] = 0;
	crc[1] = 0;
	while (len-- > 0)
	{
		crc[i] = crc8_ta[crc[i] ^ *ptr++];
		i ^= 0x01;
	}

	return ( ((unsigned short)crc[1]<<8) | crc[0] );
}
#else
unsigned short cal_crc(const unsigned char *ptr, int len)
{ 
	unsigned short crc;
	unsigned char da;
	/* CRC 余式表 */
	static const unsigned short crc_ta[16]={0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef}; 

	crc=0;
	while(len-- > 0)
	{
		da = ((unsigned char)(crc>>8))>>4;	/* 暂存CRC的高4位 */
		crc <<= 4;							/* CRC右移4位,相当于取CRC的低12位 */
		crc ^= crc_ta[da^(*ptr>>4)];		/* CRC的高4位和本字节的前半字节相加后查表计算CRC,然后加上上一次CRC的余数 */
		da = ((unsigned char)(crc>>8))>>4;	/* 暂存CRC的高4位 */
		crc <<= 4;							/* CRC右移4位,相当于取CRC的低12位 */
		crc ^= crc_ta[da^(*ptr&0x0F)];		/* CRC的高4位和本字节的后半字节相加后查表计算CRC,然后再加上上一次CRC的余数 */
		ptr++;
	}
	return crc;
}
#endif

void WERS_InsertPacketToQEx(UINT8 *pData,UINT32 ByteNumber,UINT32 orderOffset)
{
	UINT32 i,sr;
	UINT8* pPacket;
	UINT8 newOder = 0;
	WERSFramInfo_t *pIn = sWERSInFrameQ;
	WERSFramInfo_t *pSave = NULL;

	// prameter check
	if ( NULL == pData || 0 == ByteNumber )
	{
		return;
	}

	for( i = 0; i < WERS_IN_FRAME_Q_SIZE; i++,pIn++)
	{
		ENTER_CRITICAL(sr);
		if( pIn->fi_usage == WERS_FREE )
		{
			pSave = pIn;
			pSave->fi_usage = WERS_HOLD;
			EXIT_CRITICAL(sr);
			break;
		}
		else
		{
			EXIT_CRITICAL(sr);
		}
	}

	
	if ( pSave )//found
	{
		pPacket = (UINT8*)rf_alloc(ByteNumber);
		if( pPacket != NULL )
		{
			pPacket = (UINT8*)( (UINT32)pPacket |0x3C000000);
			newOder = orderOffset;
			pSave->orderStamp = newOder;
			pSave->packetSize = ByteNumber;
			memcpy(pPacket,pData,ByteNumber);
			pSave->pPacketData = pPacket;
		}
		else
		{
			pSave->fi_usage = WERS_FREE;
		}
	}
}


#if (defined WERS_485_PC_HOST)
void WERS_InsertPacketToQ(UINT8 *pData,UINT32 ByteNumber)
{
//	bspIState_t intState = 0;
//	UINT8 *pNextPacakge = NULL;
//	UINT32 leftbytes = 0;
	UINT32 playloadSize = 0;
	UINT32 frameSize = 0;
	WERS485FrameHeader *p485FrameHeader = NULL;

	// prameter check
	if ( NULL != pData || ByteNumber )
	{
		if ( ByteNumber < sizeof(WERS485FrameHeader) )
		{
			//eror, drop it
			// notify no longer to wait.
			return;
		}
		else
		{
			p485FrameHeader = (WERS485FrameHeader*)pData;

			// fix a bug
			if( p485FrameHeader->ContainerLength & 0x8000 )
			{
				ByteNumber--;
				p485FrameHeader->ContainerLength &= 0x7FFF;
			}

			if ( ByteNumber < p485FrameHeader->ContainerLength )
			{
				//eror, drop it
				// notify no longer to wait.
				return;
			}
			else if ( ByteNumber == p485FrameHeader->ContainerLength )
			{
				//leftbytes = 0;
				//pNextPacakge = NULL;
				frameSize = p485FrameHeader->ContainerLength;
				playloadSize = (ByteNumber - sizeof(WERS485FrameHeader));
			}
			else
			{
				frameSize = p485FrameHeader->ContainerLength;
				playloadSize = (p485FrameHeader->ContainerLength - sizeof(WERS485FrameHeader));
				
				//leftbytes = ByteNumber - p485FrameHeader->ContainerLength;
				//pNextPacakge = pData + playloadSize;
			}


			{
			//crc check
			UINT8 *pPlayloadData = (pData + sizeof(WERS485FrameHeader));
			
			if ( p485FrameHeader->DataCRC != cal_crc((const UINT8*)pPlayloadData,playloadSize) )
			{
				// data error. dorp it
				// notify no longer to wait.
				return;
			}
			
			// step 1.  Data CRC check. to be do
			//parser frame type
			switch( p485FrameHeader->ContainerType )
			{
#ifndef WERS_EXTENDER_DEVICE
				case FRAME_TYPE_485_REQUST:
				{
					WERSFramInfo_t *pTranscation;
					if ( (pTranscation = WERS_GetOldestTranscation() ) )
					{
						if ( g_485ListenQue )
						{
							osQuePost(g_485ListenQue, pTranscation);
						}
					}
					break;
				}
				case FRAME_TYPE_485_COMMAND_DATA_ACK:
					{
						WERS485DataAck *pDataAck = (WERS485DataAck *)pData;

						//SPMP_TurnLED(AP_LED_RUN, 2);
						//WERS485Send(FRAME_TYPE_485_DATA,&pDataAck->GUID,4);
						WERS_RemoveTranscationFromQByGUID(pDataAck->GUID);
					}
					break;
#endif
				case FRAME_TYPE_485_DATA:
				{
					UINT8* pDatabuffer;
					
					WERSContainerStruct *pContainer = (WERSContainerStruct *)(pData + sizeof(WERS485FrameHeader));

					// playload erro check
					if (  playloadSize < sizeof(WERSContainerStruct) || playloadSize != pContainer->ContainerLength )
					{
						// notify no longer to wait.
					}
					else
					{
#ifndef WERS_EXTENDER_DEVICE
						//SPMP_TurnLED(AP_LED_RUN, 2);
						if ( WERS_RESPONSE_BLOCK == WERS_TYPE(pContainer->ContainerType) )
						{
							WERS_RemoveTranscationFromQByGUID(pContainer->GUID);
							#ifndef WERS_DEBUG
							WERS485SendDataAck(pContainer->GUID);
							#endif
						}
#endif
						if ( g_485InQue )
						{
							pDatabuffer = (UINT8*)rf_alloc(frameSize);
							if( pDatabuffer )
							{
#ifndef WERS_EXTENDER_DEVICE
								pContainer->GUID = WERSMsTimeGet();
#endif
								memcpy(pDatabuffer,pData,frameSize);

								osQuePost(g_485InQue, pDatabuffer);
							}
						}
					}
					break;
				}	
				default:
					break;
			}
		}

		}
#ifndef WERS_EXTENDER_DEVICE

		if ( leftbytes && pNextPacakge)
		{
			WERS_InsertPacketToQ(pNextPacakge,leftbytes);
		}
#endif
	}
}

#elif (defined WERS_485_AP_HOST)
#ifndef WERS_USING_NETWORK
//void WERS_InsertPacketToQ(UINT8 *pData, UINT32 ByteNumber)
void WERS_InsertPacketToQ(UINT8 *pData, UINT16 ByteNumber, UINT16 dataCRC)
{
	UINT8 *pNextPacakge = NULL;
	UINT32 leftbytes = 0;
	UINT32 playloadSize = 0;
	UINT32 frameSize = 0;
	BOOL timeOut = FALSE;
	WERS485FrameHeader *p485FrameHeader = NULL;

	// prameter check
	if ( NULL != pData || ByteNumber )
	{
		if ( ByteNumber < sizeof(WERS485FrameHeader) )
		{
			//eror, drop it
			// notify no longer to wait.
			osSemPost(g_pDataRecevieOkSem);
			return;
		}
		else
		{
			p485FrameHeader = (WERS485FrameHeader*)pData;

			// fix a bug
			if( p485FrameHeader->ContainerLength & 0x8000 )
			{
				ByteNumber--;
				p485FrameHeader->ContainerLength &= 0x7FFF;
			}

			if ( ByteNumber < p485FrameHeader->ContainerLength )
			{
				//eror, drop it
				// notify no longer to wait.
				osSemPost(g_pDataRecevieOkSem);
				return;
			}
			else if ( ByteNumber == p485FrameHeader->ContainerLength )
			{
				leftbytes = 0;
				pNextPacakge = NULL;
				frameSize = p485FrameHeader->ContainerLength;
				playloadSize = (ByteNumber - sizeof(WERS485FrameHeader));
			}
			else 
			{
				frameSize = p485FrameHeader->ContainerLength;
				playloadSize = (p485FrameHeader->ContainerLength - sizeof(WERS485FrameHeader));
				
				leftbytes = ByteNumber - p485FrameHeader->ContainerLength;
				pNextPacakge = pData + playloadSize;
			}


			{
			//crc check
			UINT8 *pPlayloadData = (pData + sizeof(WERS485FrameHeader));
			
			if ( p485FrameHeader->DataCRC != cal_crc((const UINT8*)pPlayloadData,playloadSize) )
			//if ( p485FrameHeader->DataCRC != dataCRC )
			{
				// data error. dorp it
				// notify no longer to wait.
				osSemPost(g_pDataRecevieOkSem);
				return;
			}
			
			// step 1.  Data CRC check. to be do
			//parser frame type
			switch( p485FrameHeader->ContainerType )
			{
				case FRAME_TYPE_485_COMMAND_DATA_ACK:		
				{
					WERS485DataAck *pDataAck = (WERS485DataAck *)pData;
					g_485AckGUID = pDataAck->GUID;
					osSemPost(g_p485DataAckSem);
					break;
				}
				case FRAME_TYPE_485_DATA:
				{
					UINT8* pDatabuffer;

					WERSContainerStruct *pContainer = (WERSContainerStruct *)(pData + sizeof(WERS485FrameHeader));

					// playload erro check
					if (  playloadSize < sizeof(WERSContainerStruct) || playloadSize != pContainer->ContainerLength )
					{
						// notify no longer to wait.
						osSemPost(g_pDataRecevieOkSem);
					}
					else
					{
						//SPMP_TurnLED(AP_LED_RUN, 2);	
						timeOut = FALSE;
						osSemPost(g_pDataRecevieOkSem);

						if ( WERS_RESPONSE_BLOCK == WERS_TYPE(pContainer->ContainerType) )
						{
							#ifndef WERS_DEBUG
							WERS485SendDataAck(pContainer->GUID);
							WERS_RemoveGUIDFromMap(pContainer->GUID);
							#endif

							if ( WERSMsTimeGet() > (pContainer->GUID + MAX_TIME_OUT_TO_DROP_PACKATE) )
							{
								timeOut = TRUE;
							}
						}

						
						if ( FALSE == timeOut )
						{	
							pDatabuffer = (UINT8*)rf_alloc(frameSize);
							if( pDatabuffer )
							{
								pContainer->GUID = WERSMsTimeGet();

								memcpy(pDatabuffer,pData,frameSize);
								#ifndef WERS_DEBUG
								osQuePost(g_485InQue, pDatabuffer);
								#endif
							}
						}
					}
					break;
				}	
				default:
					break;
			}
		}

		}

		//if ( leftbytes && pNextPacakge)
		//{
		//	WERS_InsertPacketToQ(pNextPacakge,leftbytes);
		//}
	}
}
#endif	/*WERS_USING_NETWORK*/
#endif

WERSFramInfo_t* WERS_ReceiveFromPC(void)
{
	UINT32 i;//,sr;
	UINT8 oldOder = 0xff;
	WERSFramInfo_t *pIn = sWERSInFrameQ;
	WERSFramInfo_t *pOut = NULL;		

	for( i = 0; i < WERS_IN_FRAME_Q_SIZE; i++)
	{			
		if( pIn->fi_usage == WERS_HOLD)
		{
			if(  pIn->orderStamp < oldOder  )
			{
				oldOder = pIn->orderStamp;
				pOut = pIn;
			}
		}
		
		pIn++;
	}
#ifdef WERS_EXTENDER_DEVICE
	if(pOut)
		pOut->fi_usage = WERS_FREE;
#endif
	if ( pOut != NULL && pOut->pPacketData == NULL )
	{
		pOut = NULL;
	}
	return pOut;
}

void WERS_RemoveTranscationFromQ(WERSFramInfo_t *pTranscation)
{
	if ( pTranscation )	
	{
		if ( WERS_FI_AVAILABLE != pTranscation->fi_usage)
		{
			WERS_QadjustOrder(pTranscation->orderStamp);

			// free it	
			pTranscation->getcount = 0;
			pTranscation->packetSize = 0;
			pTranscation->orderStamp = 0;
			if ( pTranscation->pPacketData )
			{
				rf_free((void*)pTranscation->pPacketData);
				pTranscation->pPacketData = NULL;
			}

			// don't change follow order.   consider ISR switch. must be in last,
			pTranscation->fi_usage = WERS_FI_AVAILABLE;	
		}
	}
}

void WERS_RemoveTranscationFromQByGUID(UINT32 GUID)
{
	WERSFramInfo_t * pMatch;
//	bspIState_t  intState;

	if ( (pMatch = WERS_QfindByGUID(GUID)) != NULL )
	{
		#if 1
		WERS_RemoveTranscationFromQ(pMatch);
		#else
		WERS_QadjustOrder(pMatch->orderStamp);

		// free it	
		pMatch->getcount = 0;
		pMatch->packetSize = 0;
		pMatch->orderStamp = 0;
		if ( pMatch->pPacketData )
		{
			rf_free((void*)pMatch->pPacketData);
			pMatch->pPacketData = NULL;
		}

		// don't change follow order.   consider ISR switch. must be in last,
		pMatch->fi_usage = WERS_FI_AVAILABLE;
		#endif
	}
}

void WERS_ClearTranscationQueu(void)
{
	WERSFramInfo_t *pOldest;

	while((pOldest = WERS_QfindOldest()) != NULL )
	{
		WERS_RemoveTranscationFromQ(pOldest);
	}
}


WERSFramInfo_t * WERS_GetOldestTranscation(void)
{
#if 1
	WERSFramInfo_t *pOldest;

	do
	{		
		if ( (pOldest = WERS_QfindOldest()) != NULL )
		{
			if ( pOldest->pPacketData && pOldest->packetSize )
			{
				UINT32 currentTime  =WERSMsTimeGet();
				WERSContainerStruct *pContianer = (WERSContainerStruct *)pOldest->pPacketData;

				
				pOldest->getcount--;
				
				if ( currentTime > (pContianer->GUID + MAX_TIME_OUT_TO_DROP_PACKATE)
					|| pOldest->getcount  < 0 )
				{
					// invaild , drop it.
					WERS_RemoveTranscationFromQ(pOldest);
				}
				else
				{
					break;
				}
			}
			else
			{
				// invaild , drop it.
				// free it	
				WERS_RemoveTranscationFromQ(pOldest);
			}
		}	
		else
		{
			break;
		}		
	}	while(1);


	return pOldest;
#else
	WERSFramInfo_t *pOldest;
	
	if ( (pOldest = WERS_QfindOldest()) )
	{
		// get,  send to pc

		WERS485Send(FRAME_TYPE_485_DATA,(UINT8*)pOldest->pPacketData,pOldest->packetSize);
		
		//  restore state */
		pOldest->fi_usage = WERS_FI_INUSE_UNTIL_DEL;
	}
#endif
}

void WERS_QInit(void)
{
	memset(sWERSTransactionInQ,0,sizeof(sWERSTransactionInQ));
}


WERSFramInfo_t *WERS_QfindSlot()
{
	  WERSFramInfo_t *pFI, *oldest= 0, *newFI = 0;
	  //uint8_t        i, num, newOrder = 0, orderTest;
	  uint16_t        i, num, newOrder = 0, orderTest;


	    pFI  = sWERSTransactionInQ;
	    num  = WERS_TRANSACTION_IN_FRAME_Q_SIZE;


	  orderTest = num + 1;

	  for (i=0; i<num; ++i, ++pFI)
	  {
	    /* if frame is available it's a candidate. */
	    if (pFI->fi_usage != WERS_FI_AVAILABLE)
	    {	     
	        // keep track of the newest one so we can add a frame.
	        if (pFI->orderStamp > newOrder)
	        {
	          // need to know the max number in case there's already a free spot
	          newOrder = pFI->orderStamp;
	        }

	        /* make sure nwk_retrieveFrame() is not processing this frame */
	        if (WERS_FI_INUSE_TRANSACTION == pFI->fi_usage)
	        {
	          continue;
	        }
	        /* is this frame older than any we've seen? */
	        if (orderTest > pFI->orderStamp)
	        {
	          /* yes. */
	          oldest    = pFI;
	          orderTest = pFI->orderStamp;
	        }
	      
	    }
	    else
	    {
	      newFI = pFI;
	    }
	  }

	  /* did we find anything? */
	  if (!newFI)
	  {
	    /* queue was full. cast-out happens here...unless... */
	    if ((1 == i) || !oldest)
	    {
	      /* This can happen if the queue is only of size 1 and the
	      * frame is in transition when the Rx interrupt occurs. it
	      * can also happen if all (2) entries are in transition.
	       */
	      //return (frameInfo_t *)0;
	      return NULL;
	    }
	    newFI = oldest;
	    WERS_QadjustOrder(newFI->orderStamp);
	    newFI->orderStamp = i;
	  }
	  else
	  {
	    /* mark the available slot. */
	    newFI->orderStamp = ++newOrder;
	  }

	  return newFI;
}


void WERS_QadjustOrder(uint16_t stamp)
{
	WERSFramInfo_t *pFI;
	uint16_t      i, num;
	bspIState_t  intState;

	pFI  = sWERSTransactionInQ;
	num  = WERS_TRANSACTION_IN_FRAME_Q_SIZE;


	BSP_ENTER_CRITICAL_SECTION(intState);

	for (i=0; i<num; ++i, ++pFI)
	{
		if ((pFI->fi_usage != WERS_FI_AVAILABLE) && (pFI->orderStamp > stamp))
		{
		  pFI->orderStamp--;
		}
	}

	BSP_EXIT_CRITICAL_SECTION(intState);

	return;
}


WERSFramInfo_t *WERS_QfindOldest()
{
	uint16_t		i, oldest, num;
	uint8_t      uType;
	bspIState_t  intState;
	WERSFramInfo_t *fPtr = 0, *wPtr;


	wPtr   = sWERSTransactionInQ;
	num    = WERS_TRANSACTION_IN_FRAME_Q_SIZE;
	oldest = WERS_TRANSACTION_IN_FRAME_Q_SIZE+1;


	uType = WERS_FI_INUSE_UNTIL_DEL;

	for (i=0; i<num; ++i, ++wPtr)
	{

		BSP_ENTER_CRITICAL_SECTION(intState);   /* protect the frame states */

		/* only check entries in use and waiting for this port */
		if (uType == wPtr->fi_usage)
		{
			  wPtr->fi_usage = WERS_FI_INUSE_TRANSACTION;

			  BSP_EXIT_CRITICAL_SECTION(intState);  /* release hold */

			  if (wPtr->orderStamp < oldest)
			  {
				if (fPtr)
				{
					/* restore previous oldest one */
					fPtr->fi_usage = uType;
				}

				oldest = wPtr->orderStamp;
				fPtr   = wPtr;

				continue;
			  }
			  else
			  {
				/* not oldest. restore state */
				wPtr->fi_usage = uType;
			  }

		}
		else
		{
		  BSP_EXIT_CRITICAL_SECTION(intState);
		}
	}

	return fPtr;
}


WERSFramInfo_t *WERS_QfindByGUID(UINT32 GUID)
{
	//uint16_t		oldest;
	uint16_t		i, num;
	uint8_t      uType;
	bspIState_t  intState;
	WERSFramInfo_t *fPtr = 0, *wPtr;
	WERSContainerStruct *pCmdContainer = NULL;

	wPtr   = sWERSTransactionInQ;
	num    = WERS_TRANSACTION_IN_FRAME_Q_SIZE;
	//oldest = WERS_TRANSACTION_IN_FRAME_Q_SIZE+1;


	uType = WERS_FI_INUSE_UNTIL_DEL;

	for (i=0; i<num; ++i, ++wPtr)
	{

		BSP_ENTER_CRITICAL_SECTION(intState);   /* protect the frame states */

		/* only check entries in use and waiting for this port */
		if (uType == wPtr->fi_usage)
		{
			wPtr->fi_usage = WERS_FI_INUSE_TRANSACTION;

			BSP_EXIT_CRITICAL_SECTION(intState);  /* release hold */


			pCmdContainer = (WERSContainerStruct *)wPtr->pPacketData;
			if ( pCmdContainer && pCmdContainer->GUID == GUID )
			{
				//find
				fPtr   = wPtr;
				break;
			}
			else
			{
				/* not a match. restore state */
				wPtr->fi_usage = uType;
			}

		}
		else
		{
		  BSP_EXIT_CRITICAL_SECTION(intState);
		}
	}

	return fPtr;
}



void WERS_SaveReceivedTranscationInQ(UINT8 *pData,UINT32 ByteNumber,SINT8 getcount)
{
	WERSFramInfo_t *fInfoPtr = NULL;

//	bspIState_t  intState;
	
	// prameter check
	if ( NULL == pData || 0 == ByteNumber )
	{
		return;
	}
	
	if ( (fInfoPtr=WERS_QfindSlot())!=NULL )
	{
		//got
		fInfoPtr->pPacketData = pData;
		fInfoPtr->packetSize = ByteNumber;
		fInfoPtr->getcount = getcount;
		// don't change follow order.   consider ISR switch. must be in last,
		fInfoPtr->fi_usage = WERS_FI_INUSE_UNTIL_DEL;
	}
	else
	{
		// que full, delete the oldest.
		if ( (fInfoPtr = WERS_QfindOldest())!=NULL )
		{
			WERS_QadjustOrder(fInfoPtr->orderStamp);

			if (fInfoPtr->pPacketData)
			{
				rf_free((void*)fInfoPtr->pPacketData);
			}
			fInfoPtr->orderStamp = WERS_TRANSACTION_IN_FRAME_Q_SIZE;
			fInfoPtr->pPacketData = pData;
			fInfoPtr->packetSize = ByteNumber;
			fInfoPtr->getcount = getcount;

			// don't change follow order.   consider ISR switch. must be in last,
			fInfoPtr->fi_usage = WERS_FI_INUSE_UNTIL_DEL;
		}
	}
	
}

UINT32
WERSGetDataFromPC(
	UINT8 **pData,
	UINT16 *dataLen
)
{
	UINT16 *pPayloadLen;
	UINT32 PackeSize,offset;
	UINT32 len;
	SINT32 leftDataLen;
	UINT8 *inbuffer;
	UINT8 *pPlayload;

	WERSFramInfo_t *fptr;

	//rf_printf_0("==========WERSGetPCData=============\n");

	fptr = WERS_ReceiveFromPC();
	if ( fptr == NULL )
	{
		/* time out */
		*pData = NULL;
		*dataLen = 0;
		//SPMP_TurnLED(AP_LED_DATA, 1);
		return SMPL_TIMEOUT;
	}
	//SPMP_TurnLED(AP_LED_DATA, 1);
	//sio_output("begin get data from pc:%d\n", tmrMsTimeGet());
	offset = 0;
	leftDataLen = 0;

	pPlayload = (UINT8* )fptr->pPacketData;
	PackeSize = fptr->packetSize;

	/* get playload length */
	pPayloadLen = (UINT16*)pPlayload;
	*dataLen = len = (*pPayloadLen) & 0x7fff;
	if( (*pPayloadLen) & 0x8000 )
		len += 1;
	leftDataLen = len;
	(*pPayloadLen) &= 0x7fff;

	//data check

	if( len <= 0 || PackeSize < 2 )
	{
		/* time out */
		*pData = NULL;
		*dataLen = 0;

		//free memory
		if( fptr->pPacketData!= NULL )
		{
			rf_free((void*)fptr->pPacketData);
			fptr->pPacketData = NULL;
		}

		fptr->fi_usage = WERS_FREE;
		
		return SMPL_BAD_PARAM;
	}

	if( PackeSize > len )
	{
		//SPMP_TurnLED(AP_LED_RUN, 2);
		// split data
		WERS_InsertPacketToQEx(pPlayload + len,(PackeSize - len), fptr->orderStamp);

		PackeSize = len;
	}
	

	/* ready buffer for revice indata */
	*pData = inbuffer = (UINT8*)rf_alloc(len);
	if( inbuffer == NULL )
	{
		rf_printf_0("\n No memory!! %s %s %d\n",__FILE__,__FUNCTION__,__LINE__);
		*pData = NULL;
		*dataLen = 0;

		//free memory
		if( fptr->pPacketData!= NULL )
		{
			rf_free((void*)fptr->pPacketData);
			fptr->pPacketData = NULL;
		}		

		fptr->fi_usage = WERS_FREE;

		return SMPL_NOMEM;
	}

	memcpy(inbuffer,pPlayload,PackeSize);
	offset +=PackeSize;
	leftDataLen -= PackeSize;

	//free memory
	if( fptr->pPacketData!= NULL )
	{
		rf_free((void*)fptr->pPacketData);
		fptr->pPacketData = NULL;
	}
	fptr->fi_usage = WERS_FREE;

	ClearTimeOut();	

	/* get data */
	while(leftDataLen)
	{
		do
		{
			fptr = WERS_ReceiveFromPC();
			if ( fptr )
			{
				ClearTimeOut();
				break;
			}
			else
			{
				if(IS_TIME_OUT)
				{
					/* time out */
					*pData = NULL;
					*dataLen = 0;
					//SPMP_TurnLED(AP_LED_DATA, 1);
					return SMPL_TIMEOUT;
				}
			}
		}while(1);
		/* get low byte of the length*/
		pPlayload  = (UINT8* )fptr->pPacketData;

		/* get packet length */
		PackeSize = fptr->packetSize;

		//data check
		if (  PackeSize <= 0 )
		{
			/* time out */
			*pData = NULL;
			*dataLen = 0;

			//free memory
			if ( fptr->pPacketData != NULL )
			{
				rf_free((void*)fptr->pPacketData);
				fptr->pPacketData = NULL;
			}

			fptr->fi_usage = WERS_FREE;

			return SMPL_BAD_PARAM;
		}

		if(PackeSize > leftDataLen  )
		{
			//SPMP_TurnLED(AP_LED_RUN, 2);
			// split data
			WERS_InsertPacketToQEx(pPlayload + leftDataLen,(PackeSize - leftDataLen), fptr->orderStamp);

			PackeSize = leftDataLen;
		}

		memcpy(inbuffer + offset ,pPlayload,PackeSize);
		offset +=PackeSize;
		leftDataLen -= PackeSize;

		//free memory
		if ( fptr->pPacketData != NULL )
		{
			rf_free((void*)fptr->pPacketData);
			fptr->pPacketData = NULL;
		}
		fptr->fi_usage = WERS_FREE;

	}
	return SMPL_SUCCESS;
}

UINT32 WERSGenerate_RF_GUID(void)
{
	return  WERSMsTimeGet();
}

UINT32 WERSGenerate_485_GUID(void)
{
#if 0
	static UINT8 sequence  = 0;
	

	UINT32 GUID = WERSMsTimeGet() + (sequence++);
	
	return GUID;
#else
	return WERSMsTimeGet();
#endif
}

static void processMessage(linkID_t lid, uint8_t *msg, uint8_t len)
{
	//int16_t datalen;
	int32_t datalen;
	uint8_t *pDataInBufferOut;
	uint8_t *pDataOffset;
	uint8_t size;
	smplStatus_t ret;
//	WERS485SigHeader *pSigHeader;
//	static UINT8 sequence  = 0;
		
	// do something useful	
	if (len && msg != NULL )
	{
		WERSContainerStruct *pContainer = (WERSContainerStruct*)msg;

		//if(WERS_TYPE(pContainer->ContainerType) == WERS_COMMAND_BLOCK && pContainer->ContainerLength >= WERS_CONTAINER_SIZE )
		// recive the packet from device and forward it to pc
		if( pContainer->ContainerLength >= WERS_CONTAINER_SIZE )
		{
#ifdef WERS_EXTENDER_DEVICE
			if ( lid == gs_linkID )
			{
				// from the top AP, foward it to leaf node
			}
			else
			{
				// from the leaf node, foward it to top AP
				if ( g_MyNodeType <= WERS_ROUTTABLE_COUNT )
				{
					pContainer->RouteMap[g_MyNodeType-1] = lid;
				}
			}
#endif
			pContainer->Prot = lid;

			datalen = pContainer->ContainerLength;
			rf_printf_0("=====getPlayload len: %d  %d\n",datalen,len);

			#if 0// for debug//Motified by shichu
			pDataInBufferOut =  (uint8_t *)rf_alloc(datalen + 64); //must more alloc 64 bytes, for fix buffer overflow may be caused  by frame error!
			if(pDataInBufferOut == NULL )
			{
				rf_printf_0("=====not enough memory!! %s %d\n",__FUNCTION__,__LINE__);
				ClearBackOffTimeOut();
				return;
			}
			pDataOffset = pDataInBufferOut;
			#else
			pDataOffset = pDataInBufferOut =  (uint8_t *)rf_alloc(datalen + 64); //must more alloc 64 bytes, for fix buffer overflow may be caused  by frame error!
			//if(pDataInBufferOut == NULL )
			//{
				//rf_printf_0("=====not enough memory!! %s %d\n",__FUNCTION__,__LINE__);
				//ClearBackOffTimeOut();//add by shichu
				//return;
			//}
			#endif
			
			if(pDataInBufferOut == NULL)
			{
				pDataOffset=msg;
			}
			else
			{
				memcpy(pDataOffset,msg,len);
				pDataOffset += len;
			}

			// get remaining data
			if( datalen > len ) 
			{
				datalen -=len;
				ClearTimeOut();
				do
				{
					//for watch dog, can't remove
					g_ApTaskCondition = 0;
					
					ret = SMPL_Receive(lid, pDataOffset, &size);
					rf_printf_1("==========receive ret: %d \n",ret);
					if ( SMPL_SUCCESS == ret )
					{
						rf_printf_0("==========receive ok size: %d \n",size);
						if(pDataInBufferOut)pDataOffset+=size;
						datalen-=size;

						ClearTimeOut();

						receivepackeNumber++;
					}
					else
					{
						if (CheckTimeOut(300))
						{
							/* time out process */
							if(pDataInBufferOut != NULL )
							{
								rf_free(pDataInBufferOut);
							}
							//else
							//{
							//	printf_0("=====not enough memory!! %s %d\n",__FUNCTION__,__LINE__);
							//}
							printf_4("==========time out: \n");
							//printf_0("==========time out: \n");
							ClearTimeOut();
							ClearBackOffTimeOut();
							return;
						}
					}
				}while(datalen > 0);
				datalen ++;
			}
			if(pDataInBufferOut == NULL)
			{
				//printf_0("=====not enough memory!! %s %d\n",__FUNCTION__,__LINE__);
				return;
			}
			// get remaining data completed
			if ( SearchTransactionGUID(&g_stWERSGetTransaction,lid,pContainer->Code,pContainer->GUID) )
			{
				// find the same.drop it.
				rf_free(pDataInBufferOut);
				rf_printf_0("find the same.drop it\n");
				//printf_0("find the same.drop it\n");
			}
			else
			{
				// get remaining data completed
				AddNewTransactionToQue(&g_stWERSGetTransaction,lid,pContainer->Code,pContainer->GUID);

				rf_printf_0("add Que: %d, %d, %d\n",lid,pContainer->Code,pContainer->GUID);

				// add timestamp.
				pContainer = (WERSContainerStruct*)pDataInBufferOut;
#ifndef WERS_EXTENDER_DEVICE
#ifndef WERS_USING_NETWORK		//add by shichu
				 /*  generate GUID for 485 communication */
				pContainer->GUID = WERSMsTimeGet();	
				WERS_SaveReceivedTranscationInQ(pDataInBufferOut,pContainer->ContainerLength,WERS_TRANSCATION_MAX_GET_COUNT);

				#ifdef WERS_485_AP_HOST
				osSemPost(g_pSem485Bridge);
				#endif
#else
				pContainer->GUID = WERSMsTimeGet();//WERSGenerate_485_GUID();	
				SendDataToPC(pDataInBufferOut, pContainer->ContainerLength);
				rf_free(pDataInBufferOut);
#endif
#endif
				#if 1	
				#if 1 //debug
				// add timestamp.
				pContainer->GUID = WERSMsTimeGet();
#ifndef WERS_EXTENDER_DEVICE				
//				osQuePost(WERSQue,pDataInBufferOut);
#else
				//pContainer->GUID = WERSMsTimeGet();
				osQuePost(g_pTranscationQue,pDataInBufferOut);
#endif
				#else
				pSigHeader->Cmd = ACTION_SEND_TO_PC;
				pSigHeader->Datasize = pContainer->ContainerLength;
				osQuePost(g_485OutQue,pDataInBufferOut);
				#endif
				#endif

			}
			ClearBackOffTimeOut();
		}
		else
		{
			rf_printf_0("==len error!!\n");
		}
	}
}

smplStatus_t  GetPlayload(linkID_t lid,uint8_t **msg,uint32_t *len)
{
	uint8_t *pDataInBuffer;
	uint8_t *pDataOffset;
	int32_t datalen;
	uint8_t size;
	
	WERSContainerStruct *pContainer = (WERSContainerStruct*)msg;

	
	rf_printf_0("==========getPlayload=============\n");

	if (msg == NULL || len == NULL )
	{
		/* error parameter */
		return SMPL_BAD_PARAM;
	}

	*len = datalen = pContainer->ContainerLength;
	rf_printf_0("==========getPlayload len: %d\n",datalen);
	if(datalen <= 0)
	{
		return SMPL_BAD_PARAM;
	}
	
	
	pDataOffset = pDataInBuffer = rf_alloc(datalen);
	if(pDataInBuffer == NULL )
	{
		rf_printf_0("=====not enough memory!! %s %d\n",__FUNCTION__,__LINE__);
		return SMPL_NOMEM;
	}
	memcpy(pDataOffset,*msg,*len);
	pDataOffset += *len;

	// get remaining data
	if( pContainer->ContainerLength > *len ) 
	{
		do
		{
			if ( SMPL_SUCCESS == SMPL_Receive(lid, pDataOffset, &size) )
			{
				pDataOffset+=size;
				datalen-=size;
			}
			else
			{
				if(IS_TIME_OUT)
				{
					/* time out process */
					if(pDataInBuffer != NULL )
					{
						rf_free(pDataInBuffer);
					}

					*msg = NULL;
					*len = 0;
					
					return SMPL_TIMEOUT;
				}
			}
		}while(datalen > 0);

	}

	*msg = pDataInBuffer;
	
	return SMPL_SUCCESS;
	
}



void CC1100_RF_Test(void)
{
	
  bspIState_t intState;
	#ifdef I_WANT_TO_CHANGE_DEFAULT_ROM_DEVICE_ADDRESS_PSEUDO_CODE
	{
		addr_t lAddr;

		createRandomAddress(&lAddr);
		SMPL_Ioctl(IOCTL_OBJ_ADDR, IOCTL_ACT_SET, &lAddr);
	}	
	#endif // I_WANT_TO_CHANGE_DEFAULT_ROM_DEVICE_ADDRESS_PSEUDO_CODE

	/* Keep trying to join until successful. Toggle LEDs to indicate that
	* joining has not occurred.
	*/

	rf_printf_0("\n\nCC1100 start step 1==================\n");

 #if SPMP3050AP
  /* Keep trying to join until successful. Toggle LEDs to indicate that
   * joining has not occurred.
   */

 	while (SMPL_SUCCESS != SMPL_Init(sCB))
	{
		NWK_DELAY(250);
	};
	

	linkID_t sLinkID1;
	uint8_t msge[] = "1112223333333333333333333333333333333333366666666666666666666666666667777777777777777777777777788888888888888888889999999999999999999993222222222222222222222222222222233333333333333333333333444aaaaabbbbbbbbbbcccccccccceeeeeee";


	rf_printf_0("===start====SMPL_Link \n");

	// Keep trying to link...
	  while (SMPL_SUCCESS != SMPL_Link(&sLinkID1))
	  {
	    	NWK_DELAY(500);
	  }

	rf_printf_0("===start====SMPL_Send   $$$$$$$$$  sLinkID1:%d \n\n",sLinkID1);

	smplStatus_t ret = SMPL_SendEx(sLinkID1, msge, sizeof(msge));

	rf_printf_0(">>>>>>>>SMPL_Send  ret: %d\n",ret);
	
	if(ret == SMPL_SUCCESS)
	{
		rf_printf_0("\n\n=========send data ok!\n");
	}
  
	#else
	  SMPL_Init(sCB);

  rf_printf_0("\n====main work loop start=============d\n");

	  // main work loop
  while (1)
  {
  //PacketReceive();
    // Wait for the Join semaphore to be set by the receipt of a Join frame from a
    // device that supports an End Device.
    //
    // An external method could be used as well. A button press could be connected
    // to an ISR and the ISR could set a semaphore that is checked by a function
    // call here, or a command shell running in support of a serial connection
    // could set a semaphore that is checked by a function call.
    if (sJoinSem && (sNumCurrentPeers < NUM_CONNECTIONS))
    {
    	rf_printf_0("\n\asdfsdafksdkjfjsdj*********============\n");

      // listen for a new connection
      while (1)
      {
        if (SMPL_SUCCESS == SMPL_LinkListen(&sLID[sNumCurrentPeers]))
        {
        rf_printf_0("\n>>>>>>>>>>>>>>SMPL_LinkListen ok  sLID[sNumCurrentPeers]: %d\n",sLID[sNumCurrentPeers]);
          break;
        }
        // Implement fail-to-link policy here. otherwise, listen again.
      }

      sNumCurrentPeers++;

      BSP_ENTER_CRITICAL_SECTION(intState);
      sJoinSem--;
      BSP_EXIT_CRITICAL_SECTION(intState);
    }
	//rf_printf_0("\n\n\n\n...............sPeerFrameSem: %d........sNumCurrentPeers:%d.........\n",sPeerFrameSem,sNumCurrentPeers);


    // Have we received a frame on one of the ED connections?
    // No critical section -- it doesn't really matter much if we miss a poll
    //if (sPeerFrameSem)
    {
      uint8_t     msg[MAX_APP_PAYLOAD], len, i;
		int j;
	 smplStatus_t ret; 

      // process all frames waiting
      for (i=0; i<sNumCurrentPeers; ++i)
      {
        ret= SMPL_Receive(sLID[i], msg, &len);
		
	if(ret == SMPL_SUCCESS)	
        {

		rf_printf_0("\n\n\n\n.....|||||||||.....smpl receive ok.........#########...\n");
		//int i;
		for( j = 0; j < len; j++)
			{
			rf_printf_0("%02x  ",msg[j]);
			}
		rf_printf_0("\n\n\n>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");

		 // processMessage(sLID[i], msg, len);

          BSP_ENTER_CRITICAL_SECTION(intState);
          sPeerFrameSem--;
          BSP_EXIT_CRITICAL_SECTION(intState);

          // If a message is to be sent back it can be done here. The peer must
          // then either explicitly poll the frame queue with SMPL_Receive() calls
          // or implement the SMPL_Receive() call as a result of a signal from
          // the Rx callbak as is done here.
          // SMPL_Send(sLID[i], returnMsg, sizeof(returnMsg));
        }
      }
    }
  }
	#endif
	rf_printf_0("\n\nCC1100 start step 2==================\n");
}

#if 0
void * rf_alloc(size_t size)
{
	void *pRet;
#if OS_CRITICAL_METHOD == 3
	OS_CPU_SR  cpu_sr;
#endif
	
	OS_ENTER_CRITICAL();
	pRet = malloc(size);
	OS_EXIT_CRITICAL();
	return pRet;
}

void rf_free(void *p)
{
	void *pRet;
#if OS_CRITICAL_METHOD == 3
	OS_CPU_SR  cpu_sr;
#endif
	
	OS_ENTER_CRITICAL();
	free(p);
	OS_EXIT_CRITICAL();
}
#endif

void WERS_init(void)
{
	rf_mem_init();
#ifndef WERS_EXTENDER_DEVICE
	OSTaskCreate(WERSAPTask,(void *)0,(OS_STK *)&WERSEDBgroundTaskStk[APP_TASK_WERSEDBground_STK_SIZE-1],APP_TASK_WERSEDBground_PRIO);
#else 
	OSTaskCreate(WERSExTask,(void *)0,(OS_STK *)&WERSEDBgroundTaskStk[APP_TASK_WERSEDBground_STK_SIZE-1],APP_TASK_WERSEDBground_PRIO);
#endif
}

