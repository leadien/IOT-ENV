
/**************************************************************************************************
  Filename:       nwk_join.c
  Revised:        $Date: 2008-03-18 17:12:36 -0700 (Tue, 18 Mar 2008) $
  Revision:       $Revision: 16605 $
  Author:         $Author: lfriedman $

  Description:    This file supports the SimpliciTI Join network application.

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
#include "nwk_api.h"
#include "nwk_frame.h"
#include "nwk.h"
#include "nwk_link.h"
#include "nwk_join.h"
#include "nwk_globals.h"
#include "nwk_freq.h"
#include "radios/common/mrfi_f1f2.h"
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

static          uint32_t sJoinToken = 0;
static          uint8_t (*spCallback)(linkID_t) = NULL;
static volatile uint8_t  sTid = 0;

#ifdef ACCESS_POINT
static uint32_t sLinkToken;
static addr_t   sSandFClients[NUM_STORE_AND_FWD_CLIENTS];
static uint8_t  sCurNumSandFClients;
static uint8_t  sJoinOK = 0;
#endif

/******************************************************************************
 * LOCAL FUNCTIONS
 */
#ifdef ACCESS_POINT
static void smpl_send_join_reply(mrfiPacket_t *frame);
static void generateLinkToken(void);

#ifndef WERS_EXTENDER_DEVICE
 uint8_t g_MyNodeType = RF_HOST_AP;
#else  /*  WERS_EXTENDER_DEVICE */
uint8_t g_MyNodeType = RF_AP_TYPE_1;
#endif  /*  WERS_EXTENDER_DEVICE */

#endif  /*  ACCESS_POINT */

/******************************************************************************
 * GLOBAL FUNCTIONS
 */

/******************************************************************************
 * @fn          nwk_joinInit
 *
 * @brief       Initialize Join application.
 *
 * input parameters
 *
 * output parameters
 *
 * @return   void
 */
void nwk_joinInit(uint8_t (*pf)(linkID_t))
{
  sJoinToken = DEFAULT_JOIN_TOKEN;
  spCallback = pf;
  (void) spCallback;  /* keep compiler happy if we don't use this */

  sTid = MRFI_RandomByte() ;

#ifdef ACCESS_POINT
  generateLinkToken();
  nwk_setLinkToken(sLinkToken);
  sJoinOK = 1;
#endif

  return;
}

/******************************************************************************
 * @fn          generateLinkToken
 *
 * @brief       Generate the link token to be used for the network controlled
 *              by this Access Point.
 *
 * input parameters
 *
 * output parameters
 *
 * @return   void
 */
#ifdef ACCESS_POINT
static void generateLinkToken(void)
{
  sLinkToken = 0xDEADBEEF;

  return;
}

/******************************************************************************
 * @fn          smpl_send_join_reply
 *
 * @brief       Send the Join reply. Include the Link token. If the device is
 *              a polling sleeper put it into the list of store-and-forward
 *              clients.
 *
 * input parameters
 * @param   frame     - join frame for which a reply is needed...maybe
 *
 * output parameters
 *
 * @return   void
 */
 extern uint16_t g_WERSSystemId;
static void smpl_send_join_reply(mrfiPacket_t *frame)
{
  frameInfo_t *pOutFrame;
   uint8_t      msg[JOIN_REPLY_FRAME_SIZE];
   uint8_t SRCNodeType;
  // see if join token is correct
  if (memcmp(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+J_JOIN_TOKEN_OS, &sJoinToken, sizeof(sJoinToken)))
  {
    //TODO: maybe return a NAK error packet??
    printf_0("line:%d\n", __LINE__);
    return;
  }
  
//added by cbwang
  if (!MRFI_CheckSameSystemID(MRFI_P_SRC_ADDR(frame)))
  {
    printf_1(" not the same system data,  drop it !! ");
  	return;
  }
//end
  SRCNodeType = *(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+ J_NODETYPE_OS);

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

//end

  printf_4("===send_join_reply  222\n");

  /* send reply with tid, the link token, and the encryption context */
  memcpy(msg+JR_LINK_TOKEN_OS, (void const *)&sLinkToken, sizeof(sLinkToken));
  msg[JR_CRYPTKEY_SIZE_OS] = SEC_CRYPT_KEY_SIZE;
  msg[JB_APP_INFO_OS]      = (sizeof(msg) - 1) | NWK_APP_REPLY_BIT;
  /* sender's tid... */
  msg[JB_TID_OS]           = *(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+JB_TID_OS);

  if ( NULL != (pOutFrame = nwk_buildFrame(SMPL_PORT_JOIN, msg, sizeof(msg), MAX_HOPS_FROM_AP)))
  {
    printf_4("== 333\n");

    /* destination address is the source adddress of the received frame. */
    memcpy(MRFI_P_DST_ADDR(&pOutFrame->mrfiPkt), MRFI_P_SRC_ADDR(frame), NET_ADDR_SIZE);

#ifdef AP_IS_DATA_HUB
    /* if source device supports ED objects save source address to detect duplicate joins */
    if (*(MRFI_P_PAYLOAD(frame)+F_APP_PAYLOAD_OS+J_NUMCONN_OS))
    {
    #if 0
    	uint8_t rc = nwk_saveJoinedDevice(frame);
    	if( 0 == rc )
		return;
    	if(1==rc && spCallback)
		spCallback(0);
    #else
      if (nwk_saveJoinedDevice(frame) && spCallback)
      {
        spCallback(0);
      }
    #endif
    }
#endif
  }
  else
  {
    /* oops -- no room left for Tx frame. Don't send reply. */
    return;
  }
  printf_4("== 444\n");

  /* If this device polls we need to provide store-and-forward support */
  if (GET_FROM_FRAME(MRFI_P_PAYLOAD(frame),F_RX_TYPE) == F_RX_TYPE_POLLS)
  {
    // save me!
    if (sCurNumSandFClients < NUM_STORE_AND_FWD_CLIENTS)
    {
      uint8_t i;
      printf_4("== 555\n");

      // make sure it's not a duplicate
      for (i=0; i<sCurNumSandFClients; ++i)
      {
        if (!memcmp(sSandFClients[i].addr, MRFI_P_SRC_ADDR(frame), NET_ADDR_SIZE))
        {
          break;
        }
      }
      if (i == sCurNumSandFClients)
      {
        // save it -- it's not a duplicate
        memcpy(sSandFClients[sCurNumSandFClients].addr, MRFI_P_SRC_ADDR(frame), NET_ADDR_SIZE);
        sCurNumSandFClients++;
      }
    }
    else
    {
      // oops -- no room left. Don't send reply.
      return;
    }
  }
  printf_4("== 666\n");

  nwk_sendFrame(pOutFrame, MRFI_TX_TYPE_CCA);

  return;
}

/******************************************************************************
 * @fn          nwk_join
 *
 * @brief       Stub Join function for Access Points.
 *
 * input parameters
 *
 * output parameters
 *
 * @return   Always returns SMPL_SUCCESS.
 */
smplStatus_t nwk_join(void)
{
  printf_0(" \n>>>>>>>>> ap path\n");
  return SMPL_SUCCESS;
}

/******************************************************************************
 * @fn          nwk_isSandFClient
 *
 * @brief       Helper function to see if the destination of a frame we have is
 *              one of AP's store-and-forward clients.
 *
 * input parameters
 * @param   fPtr     - pointer to address in frame in question
 *
 * output parameters
 *
 * @return   Returns 1 if the destination is a store-and-forward client, else 0.
 */
uint8_t nwk_isSandFClient(uint8_t *pAddr)
{
  uint8_t i, end=sizeof(sSandFClients)/sizeof(sSandFClients[0]);

  for (i=0; i<end; ++i)
  {
    if (!memcmp(&sSandFClients[i], pAddr, NET_ADDR_SIZE))
    {
      return 1;
    }
  }

  return 0;
}

/******************************************************************************
 * @fn          nwk_setJoinContext
 *
 * @brief       Helper function to set Join context for Access Point. This will
 *              allow arbitration bewteen potentially nearby Access Points when
 *              a new device is joining.
 *
 * input parameters
 * @param   which   - Join context is either off or on
 *
 * output parameters
 *
 * @return   void
 */
void nwk_setJoinContext(uint8_t which)
{
  sJoinOK = (JOIN_CONTEXT_ON == which) ? 1 : 0;

  return;
}

#else  /* ACCESS_POINT */

/******************************************************************************
 * @fn          nwk_join
 *
 * @brief       Join functioanlity for non-AP devices. Send the Join token
 *              and wait for the reply.
 *
 * input parameters
 *
 * output parameters
 *
 * @return   Status of operation.
 */
smplStatus_t nwk_join(void)
{
  uint8_t  spin, done=0;
  uint8_t  msg[JOIN_FRAME_SIZE];
  uint32_t linkToken;
  addr_t   apAddr;
  uint8_t  radioState = MRFI_GetRadioState();
  union
  {
    ioctlRawSend_t    send;
    ioctlRawReceive_t recv;
  } ioctl_info;
  
#if defined( FREQUENCY_AGILITY )
  uint8_t  i, numChan;
  freqEntry_t channels[NWK_FREQ_TBL_SIZE];

  if (!(numChan=nwk_scanForChannels(channels)))
  {
    return SMPL_NO_CHANNEL;
  }

  for (i=0; i<numChan && !done; ++i)
  {
    nwk_setChannel(&channels[i]);
#else
  {
#endif

    //ioctl_info.send.addr = (addr_t *)nwk_getBCastAddress();
    ioctl_info.send.addr = (addr_t *)nwk_getServerAddress();
    
    ioctl_info.send.msg  = msg;
    ioctl_info.send.len  = sizeof(msg);
    ioctl_info.send.port = SMPL_PORT_JOIN;

    // copy join token in
    memcpy(msg+J_JOIN_TOKEN_OS, &sJoinToken, sizeof(sJoinToken));
    /* set app info byte */
    msg[JB_APP_INFO_OS] = sizeof(msg) - 1;
    msg[JB_TID_OS]      = sTid;
    /* Set number of connections supported. Used only by AP if it is
     * a data hub.
     */
    msg[J_NUMCONN_OS] = NUM_CONNECTIONS;
//added by cbwang
	msg[J_NODETYPE_OS] = g_MyNodeType;
//end

    SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_WRITE, &ioctl_info.send);

    NWK_CHECK_FOR_SETRX();

    ioctl_info.recv.port = SMPL_PORT_JOIN;
    ioctl_info.recv.msg  = msg;
    ioctl_info.recv.addr = &apAddr;    /* save AP address from reply */

    spin = NWK_RX_RETRY_COUNT;
    while (spin)
    {
	NWK_DELAY(WAIT_FOR_REPLY);
	NWK_DELAY(WAIT_FOR_REPLY);
	NWK_DELAY(WAIT_FOR_REPLY);
      if (SMPL_SUCCESS == SMPL_Ioctl(IOCTL_OBJ_RAW_IO, IOCTL_ACT_READ, &ioctl_info.recv))
      {
        /* join reply returns link token */
        memcpy(&linkToken, msg+JR_LINK_TOKEN_OS, sizeof(uint32_t));
        nwk_setLinkToken(linkToken);
        /* save AP address */
        nwk_setAPAddress(&apAddr);
        done = 1;
        sTid++;   /* guard against duplicates */
        break;
      }

      spin--;

    }

    NWK_CHECK_FOR_RESTORE_IDLE();

    /* TODO: process encryption stuff */
  }

  return done ? SMPL_SUCCESS : SMPL_NO_JOIN;

}

#endif /* ACCESS_POINT */

/******************************************************************************
 * @fn          nwk_processJoin
 *
 * @brief       Processes a Join frame. If this is a reply let it go to the
 *              application. Otherwise generate and send the reply.
 *
 * input parameters
 * @param   frame     - Pointer to Join frame
 *
 * output parameters
 *
 * @return   Returns 1 if the frame should be saved in the Rx queue, else 0.
 */
 #if 0 //orign version
fhStatus_t nwk_processJoin(mrfiPacket_t *frame)
{
  fhStatus_t rc = FHS_RELEASE;
  uint8_t    replyType;

  /* Make sure this is a reply and see if we sent this. Validate the
   * packet for reception by client app.
   */
  if (SMPL_MY_REPLY == (replyType=nwk_isValidReply(frame, sTid, JB_APP_INFO_OS, JB_TID_OS)))
  {
    /* It's a match and it's a reply. Validate the received packet by
     * returning a 1 so it can be received by the client app.
     */
    rc = FHS_KEEP;
  }
#if !defined( END_DEVICE )
  else if (SMPL_A_REPLY == replyType)
  {
    /* No match. If I'm not an ED this is a reply that should be passed on. */
    rc = FHS_REPLAY;
  }
#if defined( ACCESS_POINT )
  else
  {
    /* Send reply if we're an Access Point otherwise ignore the frame. */
    if ((SMPL_NOT_REPLY == replyType) && sJoinOK)
    {
      smpl_send_join_reply(frame);
    }
  }
#endif  // ACCESS_POINT
#endif  // !END_DEVICE

  (void) replyType;  // keep compiler happy

  return rc;
}
#else
fhStatus_t nwk_processJoin(mrfiPacket_t *frame)
{
  fhStatus_t rc = FHS_RELEASE;
  uint8_t    replyType;

  // make sure this is a reply and see if we sent this. validate the
  // packet for reception by client app.
  if (SMPL_MY_REPLY == (replyType=nwk_isValidReply(frame, sTid, JB_APP_INFO_OS, JB_TID_OS)))
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
  else
  {
  #if defined( ACCESS_POINT )

    // send reply if we're an Access Point otherwise ignore the frame.
    if ((SMPL_NOT_REPLY == replyType) && sJoinOK)
    {
      smpl_send_join_reply(frame);
    }
  #elif defined(RANGE_EXTENDER)
    if (SMPL_NOT_REPLY == replyType)
    {
   	rc = FHS_REPLAY;
    }
  #endif// ACCESS_POINT
  }
#endif  // !END_DEVICE

  (void) replyType;  /* keep compiler happy */

  return rc;
}

#endif

