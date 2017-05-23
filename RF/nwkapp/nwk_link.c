/**************************************************************************************************
  Filename:       nwk_link.c
  Revised:        $Date: 2008-04-17 11:26:53 -0700 (Thu, 17 Apr 2008) $
  Revision:       $Revision: 16868 $
  Author:         $Author: lfriedman $

  Description:    This file supports the SimpliciTI Link network application.

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
#include "def.h"
#include "bsp.h"
#include "mrfi.h"
#include "nwk_types.h"
#include "nwk_api.h"
#include "nwk_frame.h"
#include "nwk.h"
#include "nwk_link.h"
#include "nwk_globals.h"
#include "common/mrfi_f1f2.h"
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
static uint32_t          sLinkToken = 0, sDefaultLinkToken;
static volatile uint8_t  sListenActive = 0;
#if 0 //orgin
static volatile linkID_t sServiceLinkID;
#else
#if NUM_CONNECTIONS > 0
volatile linkID_t sServiceLinkID[NUM_CONNECTIONS];
#endif
static volatile uint8_t  sNumLinkers = 0;
#endif
static volatile uint8_t  sTid = 0;
/******************************************************************************
 * LOCAL FUNCTIONS
 */

static void smpl_send_link_reply(mrfiPacket_t *);

extern UINT32 WERSMsTimeSet(UINT32 timevalue);
extern UINT32 WERSMsTimeGet(void);

/******************************************************************************
 * GLOBAL VARIABLES
 */
extern uint8_t g_MyNodeType;
extern int8_t g_mrfi_rssi;
extern int8_t g_mrfi_LQI;

/******************************************************************************
 * GLOBAL FUNCTIONS
 */
/******************************************************************************
 * @fn          nwk_linkInit
 *
 * @brief       Initialize link app. Set link token to the default.
 *
 * input parameters
 *
 * output parameters
 *
 * @return   void
 */
void nwk_linkInit(void)
{
  sDefaultLinkToken = DEFAULT_LINK_TOKEN;

  if (!sLinkToken)
  {
    /* if the link token has not been set externally by the time we get here
     * (such as by the ioctl token-setting interface) assign the default
     */
    sLinkToken = DEFAULT_LINK_TOKEN;
  }

  /* set a non-zero TID. */
  while (!(sTid = MRFI_RandomByte()))  ;

#if NUM_CONNECTIONS > 0
  memset((void *)&sServiceLinkID, 0x0, sizeof(sServiceLinkID));
#endif

  return;
}

/******************************************************************************
 * @fn          nwk_setLinkToken
 *
 * @brief       Sets the link token received in a Join reply.
 *
 * input parameters
 * @param   token   - Link token to be used on this network to link to any peer.
 *
 * output parameters
 *
 * @return   void
 */
void nwk_setLinkToken(uint32_t token)
{
  /* only set if the supplied token is non-zero. */
  if (token)
  {
    sLinkToken = token;
  }

  return;
}


/******************************************************************************
 * @fn          nwk_link
 *
 * @brief       Called from the application level to  accomplish the link
 *
 * input parameters
 *
 * output parameters
 * @param   lid     - pointer to Link ID (port) assigned for this link
 *
 * @return   Status of the operation.
 */
smplStatus_t nwk_link(linkID_t *lid)
{
	uint32_t timestamp;
  uint8_t      msg[LINK_FRAME_SIZE];
  connInfo_t  *pCInfo = nwk_getNextConnection();

  if (pCInfo)
  {
    addr_t              addr;
    union
    {
      ioctlRawSend_t    send;
      ioctlRawReceive_t recv;
    } ioctl_info;

    if (!nwk_allocateLocalRxPort(LINK_SEND, pCInfo))
    {
      nwk_freeConnection(pCInfo);
      return SMPL_NOMEM;
    }

    //memcpy(addr.addr, nwk_getBCastAddress(), NET_ADDR_SIZE);
    memcpy(addr.addr, nwk_getServerAddress(), NET_ADDR_SIZE);
    
    ioctl_info.send.addr = &addr;
    ioctl_info.send.msg  = msg;
    ioctl_info.send.len  = sizeof(msg);
    ioctl_info.send.port = SMPL_PORT_LINK;

    // copy link token in
    memcpy(msg+L_LINK_TOKEN_OS, &sLinkToken, sizeof(sLinkToken));
    /* set port to which the remote device should send */
    msg[L_RMT_PORT_OS] = pCInfo->portRx;

    /* set the transaction ID. this allows target to figure out duplicates */
    msg[LB_TID_OS] = sTid;

    /* set my Rx type */
    msg[L_MY_RXTYPE_OS] = nwk_getMyRxType();
//added by cbwang
	msg[L_MY_NODETYPE_OS] = g_MyNodeType;
//end
    // set app info byte
    msg[LB_APP_INFO_OS] = sizeof(msg) - 1;

    SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_WRITE, &ioctl_info.send);

    // the connection strcutre is OK as-is for Tx-only devices. the default
    // values fro Tx-only were popuated when the connection structure was
    // obtained.
#ifndef RX_NEVER
    {
      uint8_t spin       = NWK_RX_RETRY_COUNT;
      uint8_t radioState = MRFI_GetRadioState();

      NWK_CHECK_FOR_SETRX();

      ioctl_info.recv.port = SMPL_PORT_LINK;
      ioctl_info.recv.msg  = msg;
      ioctl_info.recv.addr = (addr_t *)pCInfo->peerAddr;

      do
      {
      	NWK_DELAY(WAIT_FOR_REPLY);
	NWK_DELAY(WAIT_FOR_REPLY);
        if (SMPL_SUCCESS == SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_READ, &ioctl_info.recv))
        {
        //added by cbwang
         /* save AP address */
        nwk_setAPAddress((addr_t *)pCInfo->peerAddr);
        //end
          break;
        }
        if (!spin)
        {
          // invalidate connection object
          nwk_freeConnection(pCInfo);
          return SMPL_NO_LINK;
        }
        
        --spin;
      } while (1);

      pCInfo->connState = CONNSTATE_CONNECTED;
      pCInfo->portTx    = msg[LR_RMT_PORT_OS];    /* link reply returns remote port */
      *lid              = pCInfo->thisLinkID;     /* return our local port number */
	//added by cbwang
	pCInfo->peerNodeType = msg[LR_MY_NODETYPE_OS];

	// sync time tick
#ifdef WERS_EXTENDER_DEVICE
	memcpy(&timestamp,msg+LR_TIME_STAMP_OS,sizeof(timestamp));

	//sio_printf("\n=======get time tick: %d\n",timestamp);
	WERSMsTimeSet(timestamp);	
#endif
	
      /* Set hop count. If it's a polling device set the count to the
       * distance to the AP. Otherwise, set it to the max less the remaining
       * which will be the path taken for this frame. It will be no worse
       * then tha max and probably will be better.
       */
      if (F_RX_TYPE_POLLS == msg[LR_MY_RXTYPE_OS])
      {
        pCInfo->hops2target = MAX_HOPS;
      }
      else
      {
        /* Can't really use this trick because the device could move. If the
         * devices are all static this may work unless the initial reception
         * was marginal.
         */
  //      pCInfo->hops2target = MAX_HOPS - ioctl_info.recv.hopCount;
        pCInfo->hops2target = MAX_HOPS;
      }

      NWK_CHECK_FOR_RESTORE_IDLE();
    }
#endif  // !RX_NEVER

    /* guard against duplicates... */
    ++sTid;
    if (!sTid)
    {
      sTid = 1;
    }
#ifdef WERS_EXTENDER_DEVICE
    nwk_setListenContext(LINK_LISTEN_ON);
#endif
    return SMPL_SUCCESS;
  }

  return SMPL_NOMEM;
}

/*added by cbwang */
/******************************************************************************
 * @fn          nwk_linkClose
 *
 * @brief       close link
 *
 * input parameters
 * @param   linkID   -link id
 *
 * output parameters
 *
 * @return   void
 */
void nwk_linkClose(uint8_t linkID)
{
	nwk_freeConnectionEx(linkID);
}
/* end*/

/******************************************************************************
 * @fn          smpl_send_link_reply
 *
 * @brief       Send the link reply to the device trying to link
 *
 * input parameters
 * @param   frame   - frame received from linker
 *
 * output parameters
 *
 * @return   void
 */
static void smpl_send_link_reply(mrfiPacket_t *frame)
{
#if NUM_CONNECTIONS > 0
  frameInfo_t *pOutFrame;
  connInfo_t  *pCInfo;
  uint8_t      remotePort;
  UINT32     msTime;
  uint8_t      msg[LINK_REPLY_FRAME_SIZE];
  uint8_t      isTxOnly = (F_RX_TYPE_NEVER == GET_FROM_FRAME(MRFI_P_PAYLOAD(frame), F_RX_TYPE));
	uint8_t SRCNodeType;
  // see if token is correct
  if (memcmp(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+L_LINK_TOKEN_OS, &sLinkToken, sizeof(sLinkToken)))
  {
    // if it is a Tx-only device the default Link token is valid
    if (isTxOnly)
    {
      // it's Tx-only. check default Link token
      if (memcmp(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+L_LINK_TOKEN_OS, &sDefaultLinkToken, sizeof(sDefaultLinkToken)))
      {
        // this failed too. we're done.
        return;
      }
    }
    else
    {
      // nope. not Tx-only. match failed. we're done.
      //TODO: maybe return a NAK error packet??
      return;
    }
  }

  if (!MRFI_CheckSameSystemID(MRFI_P_SRC_ADDR(frame)))
  {
  	printf_0(" not the same system data,  drop it !! ");
  	return;
  }

    SRCNodeType = *(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+ L_MY_NODETYPE_OS);

	printf_1("g_MyNodeType: %d     SRCNodeType:%d\n",g_MyNodeType, SRCNodeType);

	switch(g_MyNodeType)
	{
		case RF_HOST_AP: //replay olny AP_TYPE_1 and ED jion packet
			if ( RF_AP_TYPE_1 != SRCNodeType && RF_ED != SRCNodeType )
			{
				return;
			}			
			break;			
		case RF_AP_TYPE_1: //replay olny AP_TYPE_2 and ED jion packet
			if ( RF_AP_TYPE_2 != SRCNodeType && RF_ED != SRCNodeType )
			{
				return;
			}
			break;
		case RF_AP_TYPE_2: //replay olny AP_TYPE_3 and ED jion packet
			if ( RF_AP_TYPE_3 != SRCNodeType && RF_ED != SRCNodeType )
			{
				return;
			}
			break;
		case RF_AP_TYPE_3:	// replay olny ED
			if ( RF_ED != SRCNodeType )
			{
				return;
			}			
			break;
		default:
			break;
	}


  /* if we get here either the token matched or it's Tx-only and the default matched.*/

  /* is this a duplicate request? */
  remotePort = *(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+L_RMT_PORT_OS);
  if ( NULL != (pCInfo=nwk_isLinkDuplicate(MRFI_P_SRC_ADDR(frame), remotePort)))
  {
    /* resend reply */
    msg[LB_APP_INFO_OS] = (sizeof(msg) - 1) | NWK_APP_REPLY_BIT;

    /* sender's TID */
    msg[LB_TID_OS] = *(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+LB_TID_OS);

    /* Send reply with the local port number so the remote device knows where to
     * send packets.
     */
    msg[LR_RMT_PORT_OS] = pCInfo->portRx;

    /* put my Rx type in there. used to know how to set hops when sending back. */
    msg[LR_MY_RXTYPE_OS] = nwk_getMyRxType();

//added by cbwang
	msg[LR_MY_NODETYPE_OS] = g_MyNodeType;

	/* time stamp*/
	msTime = WERSMsTimeGet();
	memcpy(msg+LR_TIME_STAMP_OS,&msTime,sizeof(msTime));
//end
    if (NULL != (pOutFrame = nwk_buildFrame(SMPL_PORT_LINK, msg, sizeof(msg), MAX_HOPS-(GET_FROM_FRAME(MRFI_P_PAYLOAD(frame),F_HOP_COUNT)))))
    {
      /* destination address is the source adddress of the received frame. */
      memcpy(MRFI_P_DST_ADDR(&pOutFrame->mrfiPkt), MRFI_P_SRC_ADDR(frame), NET_ADDR_SIZE);
      nwk_sendFrame(pOutFrame, MRFI_TX_TYPE_FORCED);
    }
    return;
  }
#if 0
  if (!sListenActive)
  {
    /* We've checked for duplicate and resent reply. In that case we weren't listening
     * so just go back`.
     */
    return;
  }
#endif  

  /* room to link? */
#if defined(AP_IS_DATA_HUB)
  pCInfo = nwk_findAlreadyJoined(frame);

  if (!pCInfo)
#endif
  {
    pCInfo = nwk_getNextConnection();
  }

  if (pCInfo)
  {
    /* yes there's room and it's not a dup. address. */
    memcpy(&pCInfo->peerAddr, MRFI_P_SRC_ADDR(frame), NET_ADDR_SIZE);

#ifndef RX_NEVER
    if (!nwk_allocateLocalRxPort(LINK_REPLY, pCInfo))
    {
      nwk_freeConnection(pCInfo);
      return;
    }
#endif   // RX_NEVER

    /* The local Rx port is the one returned in the connection structure. The
     * caller is waiting on this to be set. The code here is running in an ISR
     * thread so the caller will see this change after RETI.
     */
#if 0 //orgin
    sServiceLinkID = pCInfo->thisLinkID;
#else
    if (NUM_CONNECTIONS == sNumLinkers)
    {
      /* Something is wrong -- no room to stack Link request */
      nwk_freeConnection(pCInfo);
      /* we're done with the packet */      
      return;  
    }
    sServiceLinkID[sNumLinkers++] = pCInfo->thisLinkID;
	printf_0(" this linkID: %d\n",pCInfo->thisLinkID,sNumLinkers);

#endif

    /* save the remote Tx port */
    pCInfo->portTx = remotePort;

    /* connection is valid... */
    pCInfo->connState = CONNSTATE_CONNECTED;

    // don't bother with the rest if it's a Tx-only device
    if (!isTxOnly)
    {
    /* Set hop count. If it's a polling device set the count to the
     * distance to the AP. otherwise, set it to the max less the remaining
     * which will be the path taken for this frame. It will be no worse
     * then tha max and probably will be better.
     */
      if (F_RX_TYPE_POLLS == *(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+L_MY_RXTYPE_OS))
      {
      /* It polls. so. we'll be sending to the AP which will store the
       * frame. The AP is only MAX_HOPS_FROM_AP hops away from us.
       */
        pCInfo->hops2target = MAX_HOPS_FROM_AP;
      }
      else
      {
      /* Can't really use this trick because the device could move. If the
       * devices are all static this may work unless the initial reception
       * was marginal.
       */
#ifdef DEVICE_DOES_NOT_MOVE
        pCInfo->hops2target = MAX_HOPS - GET_FROM_FRAME(MRFI_P_PAYLOAD(frame), F_HOP_COUNT);
#else
        pCInfo->hops2target = MAX_HOPS;
#endif
      }
    /* Send reply with the local port number so the remote device knows where to
     * send packets.
     */
      msg[LR_RMT_PORT_OS]  = pCInfo->portRx;

    /* put my Rx type in there. used to know how to set hops when sending back. */
      msg[LR_MY_RXTYPE_OS] = nwk_getMyRxType();
//added by cbwang
	msg[LR_MY_NODETYPE_OS] = g_MyNodeType;

	/* time stamp*/	
	msTime = WERSMsTimeGet();
	memcpy(msg+LR_TIME_STAMP_OS,&msTime,sizeof(msTime));
//end

      msg[LB_APP_INFO_OS]  = (sizeof(msg) - 1) | NWK_APP_REPLY_BIT;

    /* sender's TID */
      msg[LB_TID_OS]       = *(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+LB_TID_OS);

      if ( NULL != (pOutFrame = nwk_buildFrame(SMPL_PORT_LINK, msg, sizeof(msg), MAX_HOPS-(GET_FROM_FRAME(MRFI_P_PAYLOAD(frame),F_HOP_COUNT)))))
      {
      /* destination address is the source adddress of the received frame. */
        memcpy(MRFI_P_DST_ADDR(&pOutFrame->mrfiPkt), MRFI_P_SRC_ADDR(frame), NET_ADDR_SIZE);

        if (SMPL_SUCCESS != nwk_sendFrame(pOutFrame, MRFI_TX_TYPE_FORCED))
        {
          /* better release the connection structure */
          nwk_freeConnection(pCInfo);
        }
      }
      else
      {
        /* better release the connection structure */
        nwk_freeConnection(pCInfo);
      }
    }
  }

  /* we're done with the packet */
  return;
#endif //NUM_CONNECTIONS 
}

/******************************************************************************
 * @fn          nwk_processLink
 *
 * @brief       Process Link frame. Just save the frame for the Link app if it
 *              a reply. If it isn't a reply, send the reply in this thread.
 *
 * input parameters
 * @param   frame   - pointer to frame to be processed
 *
 * output parameters
 *
 * @return   Non-zero if frame should be kept otherwise 0 to delete frame.
 */
 #if 0 //orign version
fhStatus_t nwk_processLink(mrfiPacket_t *frame)
{
  fhStatus_t   rc;
  uint8_t      replyType;

  /* If we sent this then this is the reply. Validate the
   * packet for reception by client app. If we didn't send
   * it then we are the target. send the reply.
   */
  if (SMPL_MY_REPLY == (replyType=nwk_isValidReply(frame, sTid, LB_APP_INFO_OS, LB_TID_OS)))
  {
    /* It's a match and it's a reply. Validate the received packet by
     * returning a 1 so it can be received by the client app.
     */
    rc = FHS_KEEP;
  }
#if !defined( END_DEVICE )
  else if (SMPL_A_REPLY == replyType)
  {
    /* no match. if i'm not an ED this is a reply that should be passed on. */
    rc = FHS_REPLAY;
  }
#endif  /* !END_DEVICE */
  else
  {
    /* no, we didn't send it. send reply assuming it's a link
     * intended for us. we need to be actively iistening
     */
    smpl_send_link_reply(frame);  // this will handle duplicates

    rc = FHS_RELEASE;
#if !defined( END_DEVICE )
    if (!sListenActive)
    {
      // i'm not listening. if i'm not an ED the frame needs to be passed on.
      rc = FHS_REPLAY;
    }
#endif  // !END_DEVICE
  }

  (void) replyType;  // keep compiler happy when ED built...

  return rc;
}
 #else
fhStatus_t nwk_processLink(mrfiPacket_t *frame)
{
  fhStatus_t   rc;
  uint8_t      replyType;

  // if we sent this then this is the reply. validate the
  // packet for reception by client app. If we didn't send
  // it then we are the target. send the reply.
  if (SMPL_MY_REPLY == (replyType=nwk_isValidReply(frame, sTid, LB_APP_INFO_OS, LB_TID_OS)))
  {
    // it's a match and it's a reply. validate the received packet by
    // returning a 1 so it can be received by the client app.
    rc = FHS_KEEP;
  }
#if !defined( END_DEVICE )
  else if (SMPL_A_REPLY == replyType)
  {
    // no match. if i'm not an ED this is a reply that should be passed on.
    rc = FHS_REPLAY;
  }
#endif  // !END_DEVICE
  else
  {
#if 0 // orgin  
  #if defined(RANGE_EXTENDER)
 	sio_printf("EX---\n");
   	rc = FHS_REPLAY;
  #else
    // no, we didn't send it. send reply assuming it's a link
    // intended for us. we need to be actively iistening
    smpl_send_link_reply(frame);  // this will handle duplicates

    rc = FHS_RELEASE;
#if !defined( END_DEVICE )
    if (!sListenActive)
    {
      // i'm not listening. if i'm not an ED the frame needs to be passed on.
      rc = FHS_REPLAY;
    }
#endif  // !END_DEVICE
#endif

#else
  #if defined(RANGE_EXTENDER)
 	printf_0("EX---\n");
   	rc = FHS_REPLAY;
  #else
    // no, we didn't send it. send reply assuming it's a link
    // intended for us. we need to be actively iistening
    #if !defined( END_DEVICE )
    smpl_send_link_reply(frame);  // this will handle duplicates
    #endif	
   
    rc = FHS_RELEASE;
#if !defined( END_DEVICE )
    if (!sListenActive)
    {
      // i'm not listening. if i'm not an ED the frame needs to be passed on.
      rc = FHS_REPLAY;
    }
#endif  // !END_DEVICE
#endif


#endif
  }

  (void) replyType;  /* keep compiler happy when ED built... */

  return rc;
}

#endif

/******************************************************************************
 * @fn          nwk_getLocalLinkID
 *
 * @brief       This routine checks to see if a service port has been assigned
 *              as a result of a link reply frame being received. It is the means
 *              by which the user thread knows that the waiting is over for the
 *              link listen. the value is set in an interrupt thread.
 *
 * input parameters
 *
 * output parameters
 *
 * @return   Local port assigned when the link reply was received.
 */
#if 0 // orgin
linkID_t nwk_getLocalLinkID(void)
{
  linkID_t lid = 0;

  if (sServiceLinkID)
  {
    lid            = sServiceLinkID;
    sServiceLinkID = 0;
    nwk_setListenContext(LINK_LISTEN_OFF);
  }

  return lid;
}
#else
linkID_t nwk_getLocalLinkID(void)
{
  linkID_t    lid = 0;
#if NUM_CONNECTIONS > 0
  uint8_t     i;
  bspIState_t intState;


  BSP_ENTER_CRITICAL_SECTION(intState);
  if (sNumLinkers)
  {
    sNumLinkers--;
    BSP_EXIT_CRITICAL_SECTION(intState);

    //nwk_setListenContext(LINK_LISTEN_OFF);
    lid = sServiceLinkID[0];
    /* If more than one Link frame has been processed without an intervening
     * Listen assume that there will be another Link Listen call that will
     * poll for completion which has already occurred. Age any existing entries.
     * This code was added to deal with the possibility of mulitple EDs being
     * activated simultaneously in the AP-as-data-hub example. This opens a
     * window of opportunity for a "typical" scenario to get hosed. But for
     * a "typical" scenario to get hosed a number of improbable events have to
     * occur. These are deemed far less likely than the multiple-ED-activation
     * scenario in the AP-as-dat-hub case.
     */
    for (i=0; i<sNumLinkers; ++i)
    {
      sServiceLinkID[i] = sServiceLinkID[i+1];
    }
  }
  else
  {
    BSP_EXIT_CRITICAL_SECTION(intState);
  }
#endif  /* NUM_CONNECTIONS */

  return lid;
}

#endif
/******************************************************************************
 * @fn          nwk_setListenContext
 *
 * @brief       Sets the context when a LinkListen is executed. This prevents
 *              processing other link frames from being confused with the real
 *              one. Without this semaphore other broadcast link messages
 *              could wait int the input queue and accidently be processed if
 *              a listen is done later.
 *
 * input parameters
 *
 * @param   context - listen on or off
 *
 * output parameters
 *
 * @return   void
 */
void nwk_setListenContext(uint8_t context)
{
  sListenActive = (context == LINK_LISTEN_ON) ? 1 : 0;
}
