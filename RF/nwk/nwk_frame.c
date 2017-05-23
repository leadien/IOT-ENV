
/**************************************************************************************************
Filename:       nwk_frame.c
Revised:        $Date: 2008-03-20 12:01:59 -0700 (Thu, 20 Mar 2008) $
Revision:       $Revision: 16617 $
Author          $Author: lfriedman $

Description:    This file supports the SimpliciTI frame handling functions.

Copyright 2004-2007 Texas Instruments Incorporated. All rights reserved.

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

YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS?
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

//#define DEBUG_LEVEL 4
/******************************************************************************
* INCLUDES
*/

#include <string.h>
#include "bsp.h"
#include "mrfi.h"
#include "nwk_types.h"
#include "nwk_frame.h"
#include "nwk.h"
#include "nwk_app.h"
#include "nwk_QMgmt.h"
#include "nwk_globals.h"
#include "nwk_mgmt.h"

/******************************************************************************
* MACROS
*/

/******************************************************************************
* CONSTANTS AND DEFINES
*/

/******************************************************************************
* TYPEDEFS
*/

/******************************************************************************
* LOCAL VARIABLES
*/

#if SIZE_INFRAME_Q > 0
/* array of function pointers to handle NWK application frames */
static  fhStatus_t (* const func[])(mrfiPacket_t *) = { nwk_processPing,
nwk_processLink,
nwk_processJoin,
nwk_processSecurity,
nwk_processFreq,
nwk_processMgmt
};
static char *func_names[]={"nwk_processPing",
"nwk_processLink",
"nwk_processJoin",
"nwk_processSecurity",
"nwk_processFreq",
"nwk_processMgmt"};
#endif  /* SIZE_INFRAME_Q > 0 */

/* changed by cbwang@20080603*/
#if 0
__no_init static uint8_t sTRACTID; // __no_init to get random value at power-on
#else
static uint8_t sTRACTID; // __no_init to get random value at power-on
#endif
/*end*/
extern volatile uint32_t g_BackOffBegintime;

static addr_t const *sMyAddr;

static uint8_t  sMyRxType, sMyTxType;

#ifndef RX_POLLS
static uint8_t  (*spCallback)(linkID_t);
#endif

/******************************************************************************
* LOCAL FUNCTIONS
*/
/*added by cbwang*/
#ifdef RANGE_EXTENDER
extern void NotifyRxFrameComein(frameInfo_t *fiPtr);
extern void NotifyRelay(void);
#endif
/*end*/

#if SIZE_INFRAME_Q > 0
/* local helper functions for Rx devices */
static void  dispatchFrame(frameInfo_t *);
#if defined(RX_NEVER)
static void handleTxOnlyPort(frameInfo_t *);
#endif  // RX_NEVER
#ifndef END_DEVICE
// only APs and REs repeat frames
static void  replayFrame(frameInfo_t *);
#ifdef ACCESS_POINT
/* only Access Points need to worry about duplicate S&F frames */
uint8_t  isDupSandFFrame(mrfiPacket_t *);
#endif /* ACCESS_POINT */
#endif  /* !END_DEVICE */
#endif  /* SIZE_INFRAME_Q > 0 */

/******************************************************************************
* GLOBAL VARIABLES
*/

extern int8_t g_mrfi_rssi;
extern int8_t g_mrfi_LQI;

/******************************************************************************
* GLOBAL FUNCTIONS
*/
#if SIZE_INFRAME_Q > 0
/******************************************************************************
* @fn          MRFI_RxCompleteISR
*
* @brief       Here on Rx interrupt from radio. Process received frame from the
*              radio Rx FIFO.
*
* input parameters
*
* output parameters
*
* @return      void
*/
void MRFI_RxCompleteISR()
{
	frameInfo_t  *fInfoPtr;

	/* room for more? */
	if ( NULL != (fInfoPtr=nwk_QfindSlot(INQ)))
	{
		MRFI_Receive(&fInfoPtr->mrfiPkt);

		dispatchFrame(fInfoPtr);
	}
	else
	{
		printf_0("\n\nno room for receice new package!!\n");
	}

	return;
}

/******************************************************************************
* @fn          nwk_retrieveFrame
*
* @brief       Retrieve frame from Rx frame queue. Invoked by application-level
*              code either app through SMPL_Receive() or IOCTL through raw Rx. This
*              should run in a user thread, not an ISR thread.
*
* input parameters
* @param    port    - port on which to get a frame
*
* output parameters
* @param    msg     - pointer to where app payload should be copied. Buffer
*                     allocated should be == MAX_APP_PAYLOAD.
*
* @param    len      - pointer to where payload length should be stored. Caller
*                      can check for non-zero when polling the port. initialized
*                      to 0 even if no frame is retrieved.
* @param    srcAddr  - if non-NULL, a pointer to where to copy the source address
*                      of the retrieved message.
* @param    hopCount - if non-NULL, a pointer to where to copy the hop count
of the retrieved message.
*
* @return    SMPL_SUCCESS of there is a frame on the port otherwise SMPL_NO_FRAME.
*
*/
smplStatus_t nwk_retrieveFrame(rcvContext_t *rcv, uint8_t *msg, uint8_t *len, addr_t *srcAddr, uint8_t *hopCount)
{
	frameInfo_t *fPtr = 0;

	/* look for a frame on requested port. */
	*len = 0;
	fPtr = nwk_QfindOldest(INQ, rcv, USAGE_NORMAL);
	if (fPtr)
	{
		connInfo_t  *pCInfo = 0;

		if (RCV_APP_LID == rcv->type)
		{
			pCInfo = nwk_getConnInfo(rcv->t.lid);
			if (!pCInfo)
			{
				return SMPL_BAD_PARAM;
			}
		}

		/* it's on the requested port. */
		*len = MRFI_GET_PAYLOAD_LEN(&fPtr->mrfiPkt) - F_APP_PAYLOAD_OS;
		memcpy(msg, MRFI_P_PAYLOAD(&fPtr->mrfiPkt)+F_APP_PAYLOAD_OS, *len);
		/* save signal info */
		if (pCInfo)
		{
			/* Save Rx metrics... */
			pCInfo->sigInfo.rssi = fPtr->mrfiPkt.rxMetrics[MRFI_RX_METRICS_RSSI_OFS];
			pCInfo->sigInfo.lqi  = fPtr->mrfiPkt.rxMetrics[MRFI_RX_METRICS_CRC_LQI_OFS];
		}
		if (srcAddr)
		{
			/* copy source address if requested */
			memcpy(srcAddr, MRFI_P_SRC_ADDR(&fPtr->mrfiPkt), NET_ADDR_SIZE);
		}
		if (hopCount)
		{
			/* copy hop count if requested */
			*hopCount = GET_FROM_FRAME(MRFI_P_PAYLOAD(&fPtr->mrfiPkt), F_HOP_COUNT);
		}
		/* input frame no longer needed. free it. */
		nwk_QadjustOrder(INQ, fPtr->orderStamp);

		fPtr->fi_usage = FI_AVAILABLE;
		return SMPL_SUCCESS;
	}

	return SMPL_NO_FRAME;
}

/******************************************************************************
* @fn          nwk_retrieveFrameNotFree
*
* @brief       Retrieve frame from Rx frame queue. Invoked by application-level
*              code either app through SMPL_Receive() or IOCTL through raw Rx. This
*              should run in a user thread, not an ISR thread.
*
* input parameters
* @param    port    - port on which to get a frame
*
* output parameters
* @param    msg     - pointer to where app payload should be copied. Buffer
*                     allocated should be == MAX_APP_PAYLOAD.
*
* @param    len      - pointer to where payload length should be stored. Caller
*                      can check for non-zero when polling the port. initialized
*                      to 0 even if no frame is retrieved.
* @param    srcAddr  - if non-NULL, a pointer to where to copy the source address
*                      of the retrieved message.
* @param    hopCount - if non-NULL, a pointer to where to copy the hop count
of the retrieved message.
*
* @return    SMPL_SUCCESS of there is a frame on the port otherwise SMPL_NO_FRAME.
*
*/
smplStatus_t nwk_retrieveFrameNoFree(rcvContext_t *rcv, uint8_t *msg, uint8_t *len)
{
	frameInfo_t *fPtr = 0;

	/* look for a frame on requested port. */
	*len = 0;
	fPtr = nwk_QfindOldest(INQ, rcv, USAGE_NORMAL);
	if (fPtr)
	{
		connInfo_t  *pCInfo = 0;

		if (RCV_APP_LID == rcv->type)
		{
			pCInfo = nwk_getConnInfo(rcv->t.lid);
			if (!pCInfo)
			{
				return SMPL_BAD_PARAM;
			}
		}

		/* it's on the requested port. */
		*len = MRFI_GET_PAYLOAD_LEN(&fPtr->mrfiPkt) - F_APP_PAYLOAD_OS;
		memcpy(msg, MRFI_P_PAYLOAD(&fPtr->mrfiPkt)+F_APP_PAYLOAD_OS, *len);

		return SMPL_SUCCESS;
	}

	return SMPL_NO_FRAME;
}


#ifndef RANGE_EXTENDER //added by cbwang
extern UINT32 tmrMsTimeGet(void);

typedef struct {
	uint8_t tackID;
	uint8_t  addr[NET_ADDR_SIZE];
	uint32_t stamp;
}packetInfo_t;

#define STORE_PRE_PACKET_NUM 32
typedef struct {
	uint8_t count;
	uint8_t index;
	packetInfo_t packetinfo[STORE_PRE_PACKET_NUM];
}InPackeQ_t;

static InPackeQ_t ComInPackeQ;

void AddPacketInComeQ(frameInfo_t *fiPtr)
{	
	uint8_t pos = ComInPackeQ.index;

	ComInPackeQ.packetinfo[pos].tackID = GET_FROM_FRAME(MRFI_P_PAYLOAD(&fiPtr->mrfiPkt), F_TRACTID_OS);
	memcpy(ComInPackeQ.packetinfo[pos].addr,MRFI_P_SRC_ADDR(&fiPtr->mrfiPkt),NET_ADDR_SIZE);	
	ComInPackeQ.packetinfo[pos].stamp = tmrMsTimeGet();
	pos++;

	if(ComInPackeQ.count < STORE_PRE_PACKET_NUM )
	{
		ComInPackeQ.count++;
	}

	ComInPackeQ.index = (pos >= STORE_PRE_PACKET_NUM)? 0: ComInPackeQ.index + 1;
}

uint32_t FindSamePacket(frameInfo_t *fiPtr)
{
	int i;
	uint8_t tackID = GET_FROM_FRAME(MRFI_P_PAYLOAD(&fiPtr->mrfiPkt), F_TRACTID_OS);
	uint8_t  addr[NET_ADDR_SIZE];
	uint32_t stamp = tmrMsTimeGet();

	memcpy(addr, MRFI_P_SRC_ADDR(&fiPtr->mrfiPkt), NET_ADDR_SIZE );

	for( i = 0; i < ComInPackeQ.count; i++)
	{
		if( (ComInPackeQ.packetinfo[i].tackID ==  tackID) && !memcmp(addr, ComInPackeQ.packetinfo[i].addr, NET_ADDR_SIZE) )
		{
			if( (ComInPackeQ.packetinfo[i].stamp + 2000) < stamp)
			{
				memset(ComInPackeQ.packetinfo[i].addr, 0, NET_ADDR_SIZE);
				return 0;
			}
			else
				return 1;
		}
	}

	return 0;
}
#endif

/******************************************************************************
* @fn          dispatchFrame
*
* @brief       Received frame looks OK so far. Dispatch to either NWK app by
*              invoking the handler or the user's app by simply leaving the
*              frame in the queue and letting the app poll the port.
*
* input parameters
* @param   fiPtr    - frameInfo_t pointer to received frame
*
* output parameters
*
* @return   void
*/
static void dispatchFrame(frameInfo_t *fiPtr)
{
	uint8_t     i          = GET_FROM_FRAME(MRFI_P_PAYLOAD(&fiPtr->mrfiPkt), F_PORT_OS);
	uint8_t     nwkAppSize = sizeof(func)/sizeof(func[0]);
	fhStatus_t  rc;
	linkID_t    lid;

#ifndef END_DEVICE
	uint8_t isForMe;
#endif
	printf_4("\n====dispatchFrame======\n");

	/* be sure it's not an echo... */
	if (!memcmp(MRFI_P_SRC_ADDR(&fiPtr->mrfiPkt), nwk_getMyAddress(), NET_ADDR_SIZE))
	{
		fiPtr->fi_usage = FI_AVAILABLE;
		return;
	}

#if 0
	/* added by cbwang */
#ifndef RANGE_EXTENDER

	if ( FindSamePacket(fiPtr) )
	{
		printf_0("== finde same packet! discard!\n");

		// ignore rogue messages
		fiPtr->fi_usage = FI_AVAILABLE;
		return;
	}

	AddPacketInComeQ(fiPtr);

#endif
	/*end*/
#endif
	//added by cbwang
	{
		uint8_t srcDeviceType = GET_FROM_FRAME(MRFI_P_PAYLOAD(&fiPtr->mrfiPkt), F_TX_DEVICE);
		if ( F_TX_DEVICE_AP == srcDeviceType )
		{
			g_mrfi_rssi = fiPtr->mrfiPkt.rxMetrics[MRFI_RX_METRICS_RSSI_OFS];
			g_mrfi_LQI = fiPtr->mrfiPkt.rxMetrics[MRFI_RX_METRICS_CRC_LQI_OFS];
			printf_4("\n==(Port: %d   RSSI: %d   LQI: %d )\n",i,g_mrfi_rssi,g_mrfi_LQI);

		}
		else
		{
			printf_4("\n=from ED=(Port: %d)\n",i);
		}
	}
	//end

	// if it's a network application port dispatch to service routine. if that
	// routine returns non-zero keep the frame and we're done.
	if (i && (i <= nwkAppSize))
	{
		rc = func[i-1](&fiPtr->mrfiPkt);

		printf_1("\n=====fun: %s,rc: %d\n",func_names[i-1],rc);

		if (FHS_KEEP == rc)
		{
			printf_1("\n=FHS_KEEP\n");
			fiPtr->fi_usage = FI_INUSE_UNTIL_DEL;
		}
#ifndef END_DEVICE
		else if (FHS_REPLAY == rc)
		{
#if 0    
			// an AP or an RE could be relaying a NWK application frame...
			/* improved by cbwang  */
#ifdef RANGE_EXTENDER
			if( i == 6 )   // nwk manage packet
			{
				NotifyRelay();
			}
			else
			{
				/*
				fiPtr->fi_usage = FI_INUSE_UNTIL_DEL;
				NotifyRxFrameComein(fiPtr);
				*/
				replayFrame(fiPtr);
			}
#else	  
			if( i != 6 )   // nwk manage packet
			{
				replayFrame(fiPtr);
			}
#endif
			/* end */	  
#else
			fiPtr->fi_usage = FI_AVAILABLE;
#endif  
		}
#endif
		else  /* rc == FHS_RELEASE (default...) */
		{
			fiPtr->fi_usage = FI_AVAILABLE;
		}
		return;
	}
#if defined(RX_NEVER)
	// handle frames sent to Tx-only port
	else if (SMPL_PORT_TX_ONLY == i)
	{
		handleTxOnlyPort(fiPtr);
		return;
	}
#endif  // RX_NEVER
	/* sanity check */
	else if ((i != SMPL_PORT_USER_BCAST) && (i != SMPL_PORT_CALL) && ((i < PORT_BASE_NUMBER) || (i > SMPL_PORT_USER_MAX)))
	{
		/* bogus port. drop frame */
		fiPtr->fi_usage = FI_AVAILABLE;
		return;
	}

	/* At this point we know the target is a user app. If this is an end device
	* and we got this far save the frame and we're done. If we're an AP there
	* are 3 cases: it's for us, it's for s store-and-forward client, or we need
	* to replay the frame. If we're and RE and the frame didn't come from an RE
	* and it's not for us, replay the frame.
	*/

#ifdef END_DEVICE
	/* If we're s polling end device we only accept application frames from
	* the AP. This prevents duplicate reception if we happen to be on when
	* a linked peer sends.
	*/
#ifdef RX_POLLS
	if (F_TX_DEVICE_ED != GET_FROM_FRAME(MRFI_P_PAYLOAD(&fiPtr->mrfiPkt), F_TX_DEVICE))
	{
		// it's destined for a user app.
		fiPtr->fi_usage = (nwk_isConnectionValid(&fiPtr->mrfiPkt, &lid)) ? FI_INUSE_UNTIL_DEL : FI_AVAILABLE;
	}
	else
	{
		fiPtr->fi_usage = FI_AVAILABLE;
	}
#else
	/* it's destined for a user app. */
	if (nwk_isConnectionValid(&fiPtr->mrfiPkt, &lid))
	{
		fiPtr->fi_usage = FI_INUSE_UNTIL_DEL;
		if (spCallback && spCallback(lid))
		{
			// spCallback return unzero
			fiPtr->fi_usage = FI_AVAILABLE;
			return;
		}
	}
	else
	{
		fiPtr->fi_usage = FI_AVAILABLE;
	}
#endif  /* RX_POLLS */

#else   /* END_DEVICE */

	isForMe = !memcmp(sMyAddr, MRFI_P_DST_ADDR(&fiPtr->mrfiPkt), NET_ADDR_SIZE);
	if (!isForMe )
	{
		printf_0("not for me!\n");
	}

	if (isForMe)
	{
		//printf_0(" for me ok!!\n");
		if (nwk_isConnectionValid(&fiPtr->mrfiPkt, &lid))
		{
			//printf_0(" Connection valid !!\n");
			fiPtr->fi_usage = FI_INUSE_UNTIL_DEL;
			if (spCallback && spCallback(lid))
			{
				// spCallback return unzero
				fiPtr->fi_usage = FI_AVAILABLE;
				return;
			}
		}
		else
		{
			printf_0("invalid connection!\n");
			fiPtr->fi_usage = FI_AVAILABLE;
		}
	}
#if defined( ACCESS_POINT )
	/* Check to see if we need to save this for a S and F client. Otherwise,
	* if it's not for us, get rid of it.
	*/
	else if (nwk_isSandFClient(MRFI_P_DST_ADDR(&fiPtr->mrfiPkt)))
	{
		// don't bother if it is a duplicate frame. it just takes up space.
		if (!isDupSandFFrame(&fiPtr->mrfiPkt))
		{
			// reset the hop count. we know the max distance from here is MAX_HOPS_FROM_AP
			PUT_INTO_FRAME(MRFI_P_PAYLOAD(&fiPtr->mrfiPkt), F_HOP_COUNT, MAX_HOPS_FROM_AP);
			fiPtr->fi_usage = FI_INUSE_UNTIL_FWD;
		}
		else
		{
			fiPtr->fi_usage = FI_AVAILABLE;
		}
	}
	else if (GET_FROM_FRAME(MRFI_P_PAYLOAD(&fiPtr->mrfiPkt), F_TX_DEVICE) == F_TX_DEVICE_AP)
	{
		/* I'm an AP and this frame came from an AP. Don't replay. */
		fiPtr->fi_usage = FI_AVAILABLE;
	}
#elif defined( RANGE_EXTENDER )
	else if (GET_FROM_FRAME(MRFI_P_PAYLOAD(&fiPtr->mrfiPkt), F_TX_DEVICE) == F_TX_DEVICE_RE)
	{
		/* I'm an RE and this frame came from an RE. Don't replay. */
		fiPtr->fi_usage = FI_AVAILABLE;
	}
#endif
	else
	{
		/* It's not for me and I'm either an AP or I'm an RE and the frame
		* didn't come from an RE. Replay the frame.
		*/
#if 0
		/* improved by cbwang  */
#ifdef RANGE_EXTENDER
		fiPtr->fi_usage = FI_INUSE_UNTIL_DEL;
		NotifyRxFrameComein(fiPtr);
#else
		replayFrame(fiPtr);
#endif
#else
		fiPtr->fi_usage = FI_AVAILABLE;
#endif
		/* end */	
	}
#endif  /* !END_DEVICE */
	return;
}
#endif   /* SIZE_INFRAME_Q > 0 */

/******************************************************************************
* @fn          nwk_frameInit
*
* @brief       Initialize network context.
*
* input parameters
*
* output parameters
*
* @return    void
*/

void nwk_frameInit(uint8_t (*pF)(linkID_t))
{

	/****** fill static values for the DEVICEINFO byte that will go in each frame ******/
	// Rx type when frame originates from this device. set in nwk_buildFrame()
	// Tx type when frame sent from this device. set in nwk_sendframe()
#ifndef END_DEVICE
	sMyRxType = F_RX_TYPE_ALWAYS_ON;
#ifdef ACCESS_POINT
	sMyTxType = F_TX_DEVICE_AP;
#else
	sMyTxType = F_TX_DEVICE_RE;
#endif
#else
	sMyTxType = F_TX_DEVICE_ED;
#ifdef RX_LISTENS
	sMyRxType = F_RX_TYPE_LISTENS;
#endif
#ifdef RX_POLLS
	sMyRxType = F_RX_TYPE_POLLS;
#endif
#ifdef RX_NEVER
	sMyRxType = F_RX_TYPE_NEVER;
#endif
#ifdef RX_ALWAYS
	sMyRxType = F_RX_TYPE_ALWAYS_ON;
#endif
#endif
	/****** DONE fill static values for the DEVICEINFO byte that will go in each frame ******/

#ifndef RX_POLLS
	spCallback = pF;
#else
	(void) pF;
#endif

	sMyAddr = nwk_getMyAddress();

#ifndef RANGE_EXTENDER
	ComInPackeQ.index = 0;
	ComInPackeQ.count = 0;
#endif

	return;
}

/******************************************************************************
* @fn          nwk_buildFrame
*
* @brief       Builds an output frame for the port and message enclosed.
*              This routine prepends the frame header and populates the
*              frame in the output queue.
*
* input parameters
* @param   port    - port from application
* @param   msg     - pointer to message from app to be sent
* @param   len     - length of enclosed message
* @param   hops    - number of hops allowed. this is less than MAX_HOPS
*                    whenever the frame is being sent to the AP. this is to
*                    help mitigate the (short) broadcast storms
*
* output parameters
*
* @return   pointer to frameInfo_t structure created. NULL if there is
*           no room in output queue.
*/
frameInfo_t *nwk_buildFrame(uint8_t port, uint8_t *msg, uint8_t len, uint8_t hops)
{
	frameInfo_t  *fInfoPtr;

	if (!(fInfoPtr=nwk_QfindSlot(OUTQ)))
	{
		return (frameInfo_t *)0;
	}

	MRFI_SET_PAYLOAD_LEN(&fInfoPtr->mrfiPkt, len+F_APP_PAYLOAD_OS);

	// TODO: implement encryption. it is of for now.
	PUT_INTO_FRAME(MRFI_P_PAYLOAD(&fInfoPtr->mrfiPkt), F_ENCRYPT_OS, 0);
	PUT_INTO_FRAME(MRFI_P_PAYLOAD(&fInfoPtr->mrfiPkt), F_PORT_OS, port);
	PUT_INTO_FRAME(MRFI_P_PAYLOAD(&fInfoPtr->mrfiPkt), F_TRACTID_OS, sTRACTID);
	sTRACTID++;
	PUT_INTO_FRAME(MRFI_P_PAYLOAD(&fInfoPtr->mrfiPkt), F_RX_TYPE, sMyRxType);
	PUT_INTO_FRAME(MRFI_P_PAYLOAD(&fInfoPtr->mrfiPkt), F_HOP_COUNT, hops);

	memcpy(MRFI_P_PAYLOAD(&fInfoPtr->mrfiPkt)+F_APP_PAYLOAD_OS, msg, len);
	memcpy(MRFI_P_SRC_ADDR(&fInfoPtr->mrfiPkt), sMyAddr, NET_ADDR_SIZE);

	return fInfoPtr;
}

/******************************************************************************
* @fn          nwk_sendFrame
*
* @brief       Send a frame by copying it to the radio Tx FIFO.
*
* input parameters
* @param   pFrameInfo   - pointer to frame to be sent
* @param   txOption     - do CCA or force frame out.
*
* output parameters
*
* @return    Status of operation. Returns SMPL_TIMEOUT if Tx never occurs.
*            Tx FIFO flushed in this case.
*/
smplStatus_t nwk_sendFrame(frameInfo_t *pFrameInfo, uint8_t txOption)
{
	smplStatus_t rc;

#if 0
	//added for RTS/CTS
	if( g_BackOffBegintime > tmrMsTimeGet() )
	{
		g_BackOffBegintime = 0;
		//wait other node to send data
		printf_0("....wait other send!\n");
		return SMPL_SUCCESS;
	}

	//end
#endif

	/* set the type of device sending the frame in the header */
	PUT_INTO_FRAME(MRFI_P_PAYLOAD(&pFrameInfo->mrfiPkt), F_TX_DEVICE, sMyTxType);

	if (MRFI_TX_RESULT_SUCCESS == MRFI_Transmit(&pFrameInfo->mrfiPkt, txOption))
	{
		rc = SMPL_SUCCESS;
	}
	else
	{
		/* Tx failed -- probably CCA. free up frame buffer. We do not have NWK
		* level retries. Let application do it.
		*/
		rc = SMPL_TIMEOUT;
	}

	/* TX is done. free up the frame buffer */
	pFrameInfo->fi_usage = FI_AVAILABLE;

	return rc;
}


/******************************************************************************
* @fn          nwk_getMyRxType
*
* @brief       Get my Rx type. Used to help populate the hops count in the
*              frame header to try and limit the broadcast storm. Info is
*              exchanged when linking.
*
* input parameters
*
* output parameters
*
* @return      The address LSB.
*/
uint8_t nwk_getMyRxType(void)
{
	return sMyRxType;
}

#if defined(RX_NEVER)
/******************************************************************************
* @fn          handleTxOnlyPort
*
* @brief       Handle a frame on the Tx-only port. A frame coming on this port
*              needs to be mapped to the port assigned when the linking was
*              done. The Tx-only device could not receive the remote port to
*              send to so it uses this default tport. But the local device has
*              actually assigned a port so this function finds that assignment
*              and maps the frame to that port so that it is received.
*
* input parameters
*
* output parameters
*
* @return      The address LSB.
*/
static void handleTxOnlyPort(frameInfo_t *fiPtr)
{
	connInfo_t *pCInfo;

	// get information from the connection info array (if it's there)
	if (pCInfo=nwk_findAddressMatch(fiPtr->mrfiPkt))
	{
		// put the expected port number into the frame and let the app retrieve
		// it by that number.
		PUT_INTO_FRAME(MRFI_P_PAYLOAD(&fiPtr->mrfiPkt), F_PORT_OS, pCInfo->lidRx);
		return;
	}
	else
	{
		fiPtr->fi_usage = FI_AVAILABLE;
	}

	return;
}
#endif  // RX_NEVER

#ifndef END_DEVICE
/******************************************************************************
* @fn          replayFrame
*
* @brief       Deal with hop count on a Range Extender or Access Point replay.
*              Queue entry usage always left as available when done.
*
* input parameters
* @param   pFrameInfo   - pointer to frame information structure
*
* output parameters
*
* @return      void
*/
static void  replayFrame(frameInfo_t *pFrameInfo)
{
	uint8_t  hops = GET_FROM_FRAME(MRFI_P_PAYLOAD(&pFrameInfo->mrfiPkt), F_HOP_COUNT);

	// if hops are zero, drop frame. othewise send it.
	if (hops--)
	{
		PUT_INTO_FRAME(MRFI_P_PAYLOAD(&pFrameInfo->mrfiPkt),F_HOP_COUNT,hops);
		// don't care if the Tx fails because of TO. either someone else
		// will retransmit or the application itself will recover.
		NWK_DELAY(1);  // extra delay to allow ED response to get through
		nwk_sendFrame(pFrameInfo, MRFI_TX_TYPE_CCA);
	}
	else
	{
		pFrameInfo->fi_usage = FI_AVAILABLE;
	}
	return;
}

#ifndef END_DEVICE
/*added by cbwang*/
/******************************************************************************
* @fn          nwk_sendFrameEx
*
* @brief       Send a frame by copying it to the radio Tx FIFO.
*
* input parameters
* @param   pFrameInfo   - pointer to frame to be sent
* @param   txOption     - do CCA or force frame out.
*
* output parameters
*
* @return    Status of operation. Returns SMPL_TIMEOUT if Tx never occurs.
*            Tx FIFO flushed in this case.
*/
smplStatus_t nwk_sendFrameEx(frameInfo_t *pFrameInfo, uint8_t txOption)
{
	smplStatus_t rc;

	// set the type of device sending the frame in the header
	PUT_INTO_FRAME(MRFI_P_PAYLOAD(&pFrameInfo->mrfiPkt), F_TX_DEVICE, sMyTxType);

	if (MRFI_TX_RESULT_SUCCESS == MRFI_Transmit(&pFrameInfo->mrfiPkt, txOption))
	{
		pFrameInfo->fi_usage = FI_AVAILABLE;
		rc = SMPL_SUCCESS;
	}
	else
	{
		// Tx failed -- probably CCA. free up frame buffer. we do not have NWK
		// level retries. let application do it.
		rc = SMPL_TIMEOUT;
	}

	// TX is done. free up the frame buffer
	//pFrameInfo->fi_usage = FI_AVAILABLE;

	return rc;
}

/******************************************************************************
* @fn          replayFrame
*
* @brief       Deal with hop count on a Range Extender or Access Point replay.
*              Queue entry usage always left as available when done.
*
* input parameters
* @param   pFrameInfo   - pointer to frame information structure
*
* output parameters
*
* @return      void
*/
static smplStatus_t  replayFrameEx(frameInfo_t *pFrameInfo)
{
	smplStatus_t ret = SMPL_SUCCESS;
	uint8_t  hops = GET_FROM_FRAME(MRFI_P_PAYLOAD(&pFrameInfo->mrfiPkt), F_HOP_COUNT);

	/* if hops are zero, drop frame. othewise send it. */
	if (hops--)
	{
		PUT_INTO_FRAME(MRFI_P_PAYLOAD(&pFrameInfo->mrfiPkt),F_HOP_COUNT,hops);
		/* Don't care if the Tx fails because of TO. Either someone else
		* will retransmit or the application itself will recover.
		*/
		NWK_DELAY(1);  // extra delay to allow ED response to get through
		ret = nwk_sendFrameEx(pFrameInfo, MRFI_TX_TYPE_CCA);
	}
	else
	{
		pFrameInfo->fi_usage = FI_AVAILABLE;
	}
	return ret;
}
/******************************************************************************
* @fn          ExreplayFrame
*
* @brief       Deal with hop count on a Range Extender or Access Point replay.
*              Queue entry usage always left as available when done.
*
* input parameters
* @param   pFrameInfo   - pointer to frame information structure
*
* output parameters
*
* @return      void
*/
smplStatus_t  nwk_replayFrame(frameInfo_t *pFrameInfo)
{
	return replayFrameEx(pFrameInfo);
}
/*end*/
#endif

#ifdef ACCESS_POINT
/******************************************************************************
* @fn          nwk_getSandFFrame
*
* @brief       Get any frame waiting for the client on the port supplied in
*              the frame payload.
*              TODO: support returning NWK application frames always. the
*              port requested in the call should be an user application port.
*              NWK app ports will never be in the called frame.
*              TODO: deal with broadcast NWK frames from AP.
*
* input parameters
* @param   frame   - pointer to frame in question
*
* output parameters
*
* @return      pointer to frame if there is one, otherwise 0.
*/
frameInfo_t *nwk_getSandFFrame(mrfiPacket_t *frame, uint8_t osPort)
{
	uint8_t        i, port = *(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+osPort);
	frameInfo_t *fiPtr;
	rcvContext_t rcv;

	rcv.type  = RCV_RAW_POLL_FRAME;
	rcv.t.pkt = frame;
	/* check the input queue for messages sent by others. */
	if ( NULL != (fiPtr=nwk_QfindOldest(INQ, &rcv, USAGE_FWD)))
	{
		return fiPtr;
	}

	/* Check the output queue to see if we ourselves need to send anything.
	* TODO: use the cast-out scheme for output queue so this routine finds
	* the oldest in either queue.
	*/
	fiPtr = nwk_getQ(OUTQ);
	for (i=0; i<SIZE_OUTFRAME_Q; ++i, fiPtr++)
	{
		if (FI_INUSE_UNTIL_FWD == fiPtr->fi_usage)
		{
			if (!memcmp(MRFI_P_DST_ADDR(&fiPtr->mrfiPkt), MRFI_P_SRC_ADDR(frame), NET_ADDR_SIZE))
			{
				if (GET_FROM_FRAME(MRFI_P_PAYLOAD(&fiPtr->mrfiPkt), F_PORT_OS) == port)
				{
					return fiPtr;
				}
			}
		}
	}
	return 0;
}

/******************************************************************************
* @fn          nwk_SendEmptyPollRspFrame
*
* @brief       There are no frames waiting for the requester on the specified
*              port. Send a frame back to that port with no payload.
*
* input parameters
* @param   frame   - pointer to frame in question
*
* output parameters
*
* @return      void
*/
void nwk_SendEmptyPollRspFrame(mrfiPacket_t *frame)
{
	mrfiPacket_t dFrame;
	uint8_t      port = *(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+M_POLL_PORT_OS);
	uint8_t   encrypt = GET_FROM_FRAME(MRFI_P_PAYLOAD(frame), F_ENCRYPT_OS);

	/* set the type of device sending the frame in the header. we know it's an AP */
	PUT_INTO_FRAME(MRFI_P_PAYLOAD(&dFrame), F_TX_DEVICE, F_TX_DEVICE_AP);
	/* set the listen type of device sending the frame in the header. we know it's
	* an AP is is probably always on...but use the static variable anyway.
	*/
	PUT_INTO_FRAME(MRFI_P_PAYLOAD(&dFrame), F_RX_TYPE, sMyRxType);
	/* destination address from received frame (polling device) */
	memcpy(MRFI_P_DST_ADDR(&dFrame), MRFI_P_SRC_ADDR(frame), NET_ADDR_SIZE);
	/* source address */
	memcpy(MRFI_P_SRC_ADDR(&dFrame), MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+M_POLL_ADDR_OS, NET_ADDR_SIZE);
	/* port is the port requested */
	PUT_INTO_FRAME(MRFI_P_PAYLOAD(&dFrame), F_PORT_OS, port);
	// encryption state -- taken from original frame
	// TODO: when encryption is implemented this dummy frame must conform...
	PUT_INTO_FRAME(MRFI_P_PAYLOAD(&dFrame), F_ENCRYPT_OS, encrypt);
	/* frame length... */
	MRFI_SET_PAYLOAD_LEN(&dFrame,F_APP_PAYLOAD_OS);
	/* transaction ID... */
	PUT_INTO_FRAME(MRFI_P_PAYLOAD(&dFrame), F_TRACTID_OS, sTRACTID);
	sTRACTID++;
	/* hop count... */
	PUT_INTO_FRAME(MRFI_P_PAYLOAD(&dFrame), F_HOP_COUNT, MAX_HOPS_FROM_AP);

	MRFI_Transmit(&dFrame, MRFI_TX_TYPE_FORCED);

	return;
}

/******************************************************************************
* @fn          isDupSandFFrame
*
* @brief       Have we already stored this frame on behalf of a client?
*
* input parameters
* @param   frame   - pointer to frame in question
*
* output parameters
*
* @return      Returns 1 if the frame is a duplicate, otherwise 0.
*/
uint8_t  isDupSandFFrame(mrfiPacket_t *frame)
{
	//uint8_t      i, plLen = MRFI_GET_PAYLOAD_LEN(frame);
	uint16_t  i;
	uint8_t	plLen = MRFI_GET_PAYLOAD_LEN(frame);
	frameInfo_t *fiPtr;

	/* check the input queue for duplicate S&F frame. */
	fiPtr = nwk_getQ(INQ);
	for (i=0; i<SIZE_INFRAME_Q; ++i, fiPtr++)
	{
		if (FI_INUSE_UNTIL_FWD == fiPtr->fi_usage)
		{
			/* compare everything except the DEVICE INFO byte. */
			if (MRFI_GET_PAYLOAD_LEN(&fiPtr->mrfiPkt) == plLen                                   &&
				!memcmp(MRFI_P_DST_ADDR(&fiPtr->mrfiPkt), MRFI_P_DST_ADDR(frame), NET_ADDR_SIZE) &&
				!memcmp(MRFI_P_SRC_ADDR(&fiPtr->mrfiPkt), MRFI_P_SRC_ADDR(frame), NET_ADDR_SIZE) &&
				!memcmp(MRFI_P_PAYLOAD(&fiPtr->mrfiPkt), MRFI_P_PAYLOAD(frame), 1)               &&
				!memcmp(MRFI_P_PAYLOAD(&fiPtr->mrfiPkt)+F_TRACTID_OS, MRFI_P_PAYLOAD(frame)+F_TRACTID_OS, plLen-F_TRACTID_OS)
				)
			{
				return 1;
			}
		}
	}
	return 0;
}
#endif  /* ACCESS_POINT */

#endif  /* !END_DEVICE */
